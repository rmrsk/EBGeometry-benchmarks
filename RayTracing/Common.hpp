// SPDX-FileCopyrightText: 2026 Robert Marskar <robert.marskar@sintef.no>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// The plain types shared by the three translation units of this benchmark: the triangle list both
// libraries build from, a ray, a hit, and the interface each library's structure is driven through.
// Nothing here includes EBGeometry or tinybvh, so the two libraries are compiled in separate
// translation units (EBGeometryRays.cpp, TinyBVHRays.cpp) and never see each other's headers.

#ifndef RAYTRACING_COMMON_HPP
#define RAYTRACING_COMMON_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// The scene: one triangle soup, read once by EBGeometry's reader, in file order.
struct Scene
{
  std::string           name;
  std::size_t           fileFaces = 0; // faces in the file (polygons count as their fan triangles)
  std::vector<float>    xyz;           // 9 floats per triangle: v0, v1, v2
  std::vector<uint32_t> faceId;        // the file face each triangle was cut from
  float                 lo[3]{};       // bounding box
  float                 hi[3]{};

  std::size_t
  size() const
  {
    return faceId.size();
  }
};

// A ray: origin and unit direction, so that t is a distance. Every query is over [0, +inf).
struct Ray3
{
  float o[3];
  float d[3];
};

// A closest hit: t = +inf and prim = UINT32_MAX for a miss. prim is the index in Scene's list.
struct Hit
{
  float    t;
  uint32_t prim;
};

// One library's acceleration structure, as the benchmark drives it.
class Accel
{
public:
  virtual ~Accel() = default;

  // The configuration's name, as printed in the tables.
  virtual std::string
  name() const = 0;

  // Whether build() can use more than one thread (tinybvh's default builds can, EBGeometry's cannot).
  virtual bool
  threadedBuild() const = 0;

  // Build the structure from the scene's triangles (anything left from a previous build is released
  // first). With a_threaded false, on the calling thread only.
  virtual void
  build(bool a_threaded) = 0;

  // The closest hit of each of a_n rays, on the calling thread.
  virtual void
  trace(const Ray3* a_rays, Hit* a_hits, std::size_t a_n) const = 0;
};

// EBGeometryRays.cpp
Scene
loadScene(const std::string& a_file, bool a_warnOnDefect);

// a_width: 4 for TriMeshSDF<float, 4, 4> (the portable defaults), 0 for the host-tuned width.
// a_maxLeafGroups: groups of W triangles per leaf at most; 0 for readIntoTriMeshSDF's default (4).
std::unique_ptr<Accel>
makeEBGeometry(const Scene& a_scene, int a_width, int a_maxLeafGroups = 0);

// TinyBVHRays.cpp: a_layout is "BVH", "BVH4_CPU" or "BVH8_CPU".
std::unique_ptr<Accel>
makeTinyBVH(const Scene& a_scene, const std::string& a_layout);

std::string
tinyBVHVersion();

#endif
