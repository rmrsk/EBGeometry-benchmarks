// SPDX-FileCopyrightText: 2026 Robert Marskar <robert.marskar@sintef.no>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The tinybvh side (https://github.com/jbikker/tinybvh): its plain binary BVH (the layout every other
// layout is produced from) and its wide CPU layouts BVH4_CPU and BVH8_CPU, each built with its own
// default builder through Build() and traced with Intersect(). The only translation unit that
// includes tinybvh, and the one that emits its implementation.

#define TINYBVH_IMPLEMENTATION
#include "tiny_bvh.h"

#include <cmath>
#include <stdexcept>

#include "Common.hpp"

namespace {

template <class Layout>
class TinyBVHAccel final : public Accel
{
public:
  TinyBVHAccel(const Scene& a_scene, std::string a_name) : m_name(std::move(a_name))
  {
    // tinybvh's input: three float4 vertices per triangle (w unused), referenced by the BVH, not copied.
    // Untimed setup, like EBGeometry's Triangle list.
    m_vertices.resize(3 * a_scene.size());

    for (std::size_t i = 0; i < 3 * a_scene.size(); i++) {
      const float* p = &a_scene.xyz[3 * i];
      m_vertices[i]  = tinybvh::bvhvec4(p[0], p[1], p[2], 0.0f);
    }
  }

  std::string
  name() const override
  {
    return "tinybvh " + m_name;
  }

  bool
  threadedBuild() const override
  {
    return true;
  }

  void
  build(const bool a_threaded) override
  {
    m_bvh = std::make_unique<Layout>();
    m_bvh->settings.enableThreading = a_threaded;
    m_bvh->Build(m_vertices.data(), static_cast<uint32_t>(m_vertices.size() / 3));
  }

  void
  trace(const Ray3* a_rays, Hit* a_hits, std::size_t a_n) const override
  {
    for (std::size_t i = 0; i < a_n; i++) {
      const Ray3&  r = a_rays[i];
      tinybvh::Ray ray(tinybvh::bvhvec3(r.o[0], r.o[1], r.o[2]), tinybvh::bvhvec3(r.d[0], r.d[1], r.d[2]));

      m_bvh->Intersect(ray);

      if (ray.hit.t < BVH_FAR) {
        a_hits[i] = Hit{ray.hit.t, ray.hit.prim};
      }
      else {
        a_hits[i] = Hit{INFINITY, UINT32_MAX};
      }
    }
  }

private:
  std::string                   m_name;
  std::vector<tinybvh::bvhvec4> m_vertices;
  std::unique_ptr<Layout>       m_bvh;
};

} // namespace

std::unique_ptr<Accel>
makeTinyBVH(const Scene& a_scene, const std::string& a_layout)
{
  if (a_layout == "BVH") {
    return std::make_unique<TinyBVHAccel<tinybvh::BVH>>(a_scene, a_layout);
  }
  if (a_layout == "BVH4_CPU") {
    return std::make_unique<TinyBVHAccel<tinybvh::BVH4_CPU>>(a_scene, a_layout);
  }
  if (a_layout == "BVH8_CPU") {
    return std::make_unique<TinyBVHAccel<tinybvh::BVH8_CPU>>(a_scene, a_layout);
  }

  throw std::runtime_error("makeTinyBVH: unknown layout " + a_layout);
}

std::string
tinyBVHVersion()
{
  return std::to_string(TINY_BVH_VERSION_MAJOR) + "." + std::to_string(TINY_BVH_VERSION_MINOR) + "." +
         std::to_string(TINY_BVH_VERSION_SUB);
}
