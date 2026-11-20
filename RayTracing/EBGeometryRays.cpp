// SPDX-FileCopyrightText: 2026 Robert Marskar <robert.marskar@sintef.no>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The EBGeometry side: reading the scene (EBGeometry's own reader, shared by both libraries) and a
// TriMeshSDF traced with firstHit().
//
// This is the only translation unit that includes EBGeometry, and it defines
// EBGEOMETRY_HOST_TUNED_DEFAULTS, as the documentation requires of every translation unit of a build
// that uses the macro (ConfigurationOptions.rst, "Branching factor and SIMD width"). With it,
// TriangleSoA::DefaultWidth<float>() is TriangleSoA::HostWidth<float>() (8 with AVX2). The portable
// defaults (K = 4, W = 4, what every build without the macro gets) are then spelled out literally.

#define EBGEOMETRY_HOST_TUNED_DEFAULTS 1

#include <cmath>
#include <optional>
#include <stdexcept>

#include <EBGeometry.hpp>

#include "Common.hpp"

namespace {

using T    = float;
using Vec3 = EBGeometry::Vec3T<T>;

constexpr std::size_t K         = 4; // BVH::DefaultBranchingRatio<float>(), with or without the macro
constexpr std::size_t PortableW = 4; // TriangleSoA::DefaultWidth<float>() without the macro
constexpr std::size_t TunedW    = EBGeometry::TriangleSoA::DefaultWidth<T>();

static_assert(EBGeometry::BVH::DefaultBranchingRatio<T>() == K);
static_assert(TunedW == EBGeometry::TriangleSoA::HostWidth<T>());

// The triangles loadScene() read, as EBGeometry's own type.
std::vector<EBGeometry::Triangle<T>> g_readerTriangles;

// readIntoTriMeshSDF's defaults: binned SAH, at most four groups of W triangles per leaf.
constexpr std::size_t DefaultMaxLeafGroups = 4;

template <std::size_t W>
class EBGeometryAccel final : public Accel
{
public:
  using SDF = EBGeometry::TriMeshSDF<T, K, W>;

  EBGeometryAccel(const Scene& a_scene, const std::size_t a_maxLeafGroups) : m_maxLeafGroups(a_maxLeafGroups)
  {
    // The reader's triangles (with the normals it computed), numbered by their index in the shared list,
    // which is what tinybvh reports as its prim (untimed setup, like tinybvh's float4 copy).
    m_triangles = g_readerTriangles;

    for (std::size_t i = 0; i < m_triangles.size(); i++) {
      m_triangles[i].setFaceId(static_cast<uint32_t>(i));
    }

    if (m_triangles.size() != a_scene.size()) {
      throw std::runtime_error("EBGeometryAccel: scene and reader triangles differ");
    }
  }

  std::string
  name() const override
  {
    return "EBGeometry TriMeshSDF K=" + std::to_string(K) + " W=" + std::to_string(W) +
           (W == PortableW ? " (defaults)" : " (host-tuned)") +
           (m_maxLeafGroups == DefaultMaxLeafGroups ? "" : ", leaf <= " + std::to_string(m_maxLeafGroups) + "W");
  }

  bool
  threadedBuild() const override
  {
    return false;
  }

  void
  build(bool /*a_threaded*/) override
  {
    m_sdf.reset();
    m_pool.reset();
    m_pool = std::make_unique<EBGeometry::Pool>(EBGeometry::hostMemoryResource());
    m_sdf.emplace(m_triangles, *m_pool, EBGeometry::BVH::Construction::SAH, m_maxLeafGroups);
  }

  void
  trace(const Ray3* a_rays, Hit* a_hits, std::size_t a_n) const override
  {
    const SDF& sdf = *m_sdf;

    for (std::size_t i = 0; i < a_n; i++) {
      const Ray3&                    r = a_rays[i];
      const EBGeometry::RayT<T>      ray{Vec3(r.o[0], r.o[1], r.o[2]), Vec3(r.d[0], r.d[1], r.d[2])};
      const EBGeometry::HitT<T>      hit =
        EBGeometry::firstHit(sdf, ray, T(0), EBGeometry::Math::Limits<T>::infinity());

      if (hit.hit()) {
        a_hits[i] = Hit{hit.m_t, hit.m_primitive};
      }
      else {
        a_hits[i] = Hit{INFINITY, UINT32_MAX};
      }
    }
  }

private:
  std::size_t                          m_maxLeafGroups;
  std::vector<EBGeometry::Triangle<T>> m_triangles;
  std::unique_ptr<EBGeometry::Pool>    m_pool;
  std::optional<SDF>                   m_sdf;
};

} // namespace

Scene
loadScene(const std::string& a_file, const bool a_warnOnDefect)
{
  const auto onDefect = a_warnOnDefect ? EBGeometry::Parser::OnDefect::Warn : EBGeometry::Parser::OnDefect::Throw;
  g_readerTriangles = EBGeometry::Parser::readIntoTriangles<T>(a_file, onDefect);
  const auto& tris  = g_readerTriangles;

  Scene s;
  s.name = a_file;
  s.xyz.reserve(9 * tris.size());
  s.faceId.reserve(tris.size());

  for (int k = 0; k < 3; k++) {
    s.lo[k] = +INFINITY;
    s.hi[k] = -INFINITY;
  }

  for (const auto& tri : tris) {
    const auto& v = tri.getVertexPositions();

    for (int j = 0; j < 3; j++) {
      for (int k = 0; k < 3; k++) {
        s.xyz.push_back(v[j][k]);
        s.lo[k] = std::min(s.lo[k], v[j][k]);
        s.hi[k] = std::max(s.hi[k], v[j][k]);
      }
    }

    s.faceId.push_back(tri.getFaceId());
  }

  return s;
}

std::unique_ptr<Accel>
makeEBGeometry(const Scene& a_scene, const int a_width, const int a_maxLeafGroups)
{
  const std::size_t groups = a_maxLeafGroups > 0 ? std::size_t(a_maxLeafGroups) : DefaultMaxLeafGroups;

  if (a_width == int(PortableW)) {
    return std::make_unique<EBGeometryAccel<PortableW>>(a_scene, groups);
  }

  return std::make_unique<EBGeometryAccel<TunedW>>(a_scene, groups);
}
