// SPDX-FileCopyrightText: 2026 Robert Marskar <robert.marskar@sintef.no>
//
// SPDX-License-Identifier: GPL-3.0-or-later

// Benchmark: EBGeometry's TriMeshSDF vs tinybvh (https://github.com/jbikker/tinybvh) on closest-hit
// rays against a triangle mesh -- the task both are built for.
//
// The scene is read once (untimed) by EBGeometry's reader into one triangle list; each library is
// handed the same triangles, in the same order, in its own input format (untimed). Then, per
// configuration:
//   - the BVH build is timed (tinybvh: on one thread and with its default threaded build;
//     EBGeometry's builder is single-threaded);
//   - closest-hit queries over [0, inf) are timed for two ray sets, coherent camera rays (one pinhole
//     frame) and incoherent random rays (origins in the scene's box, uniform directions, fixed seed),
//     on one thread and on all hardware threads (std::thread, static round-robin chunks, the same
//     harness for both libraries).
// Every configuration's answers are cross-checked ray by ray against tinybvh's plain BVH, and a
// sample of rays against a double-precision brute-force loop over every triangle; each mismatch is
// classified (see classify()).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <sched.h>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "Common.hpp"

namespace {

// ---------------------------------------------------------------------------------------------
// Options
// ---------------------------------------------------------------------------------------------

struct Options
{
  std::string scene   = "../Submodules/common-3d-test-models/data/armadillo.obj"; // the CI default
  bool        warn    = false;      // load with OnDefect::Warn (meshes that are not closed manifolds)
  std::string camera  = "outside";  // outside: orbit view of the whole scene; inside: interior view
  int         width   = 1920;
  int         height  = 1080;
  std::size_t random  = 0;          // incoherent rays; 0 = as many as camera rays
  int         reps    = 5;          // timed repetitions after one warm-up
  std::size_t sample  = 10000;      // brute-force rays per ray kind
  int         threads = 0;          // 0 = std::thread::hardware_concurrency()
  int         pinCpu  = -1;         // pin single-thread runs to this CPU (-1: no pinning)
  std::string configs = "eb4,eb8,BVH,BVH4_CPU,BVH8_CPU";
};

void
usage()
{
  std::printf("usage: RayTracing.ex [--scene file] [--warn] [--camera outside|inside] [--width W] [--height H]\n"
              "                  [--random N] [--reps R] [--sample S] [--threads N] [--pin-cpu C]\n"
              "                  [--configs eb4,eb8,BVH,BVH4_CPU,BVH8_CPU] (eb4:G, eb8:G: leaves of at most G groups)\n");
}

Options
parse(int argc, char** argv)
{
  Options o;

  for (int i = 1; i < argc; i++) {
    const std::string a    = argv[i];
    auto              next = [&]() -> std::string {
      if (i + 1 >= argc) {
        usage();
        std::exit(1);
      }
      return argv[++i];
    };

    if (a == "--scene") o.scene = next();
    else if (a == "--warn") o.warn = true;
    else if (a == "--camera") o.camera = next();
    else if (a == "--width") o.width = std::stoi(next());
    else if (a == "--height") o.height = std::stoi(next());
    else if (a == "--random") o.random = std::stoul(next());
    else if (a == "--reps") o.reps = std::stoi(next());
    else if (a == "--sample") o.sample = std::stoul(next());
    else if (a == "--threads") o.threads = std::stoi(next());
    else if (a == "--pin-cpu") o.pinCpu = std::stoi(next());
    else if (a == "--configs") o.configs = next();
    else {
      usage();
      std::exit(a == "--help" ? 0 : 1);
    }
  }

  if (o.threads <= 0) {
    o.threads = int(std::thread::hardware_concurrency());
  }

  return o;
}

// ---------------------------------------------------------------------------------------------
// Machine
// ---------------------------------------------------------------------------------------------

std::string
cpuModel()
{
  std::ifstream f("/proc/cpuinfo");
  std::string   line;

  while (std::getline(f, line)) {
    if (line.rfind("model name", 0) == 0) {
      return line.substr(line.find(':') + 2);
    }
  }

  return "unknown";
}

// Pin the calling thread (and the threads it spawns later) to one CPU, or release it (a_cpu < 0).
void
pin(const int a_cpu)
{
  cpu_set_t set;
  CPU_ZERO(&set);

  if (a_cpu >= 0) {
    CPU_SET(a_cpu, &set);
  }
  else {
    for (int c = 0; c < CPU_SETSIZE; c++) {
      CPU_SET(c, &set);
    }
  }

  sched_setaffinity(0, sizeof(set), &set);
}

// ---------------------------------------------------------------------------------------------
// Rays
// ---------------------------------------------------------------------------------------------

// splitmix64: a fixed, platform-independent random sequence (unlike the standard distributions).
struct Rng
{
  uint64_t s;

  uint64_t
  next()
  {
    uint64_t z = (s += 0x9e3779b97f4a7c15ull);
    z          = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
    z          = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
    return z ^ (z >> 31);
  }

  // Uniform in [0, 1).
  double
  uniform()
  {
    return double(next() >> 11) * 0x1.0p-53;
  }
};

struct V3
{
  double x, y, z;
};

V3
operator+(V3 a, V3 b)
{
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}
V3
operator-(V3 a, V3 b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}
V3
operator*(double s, V3 a)
{
  return {s * a.x, s * a.y, s * a.z};
}
double
dot(V3 a, V3 b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
V3
cross(V3 a, V3 b)
{
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
V3
normalize(V3 a)
{
  return (1.0 / std::sqrt(dot(a, a))) * a;
}

Ray3
makeRay(V3 o, V3 d)
{
  d = normalize(d);
  return Ray3{{float(o.x), float(o.y), float(o.z)}, {float(d.x), float(d.y), float(d.z)}};
}

// One pinhole frame, row by row. outside: the scene seen from the front, a little from above and the
// side (y up), at 0.7 of the distance at which its bounding sphere would fill the frame's height. inside: from a quarter of the way
// along the longest horizontal axis, at a quarter of the height, looking along that axis (an interior
// view, for a building such as Sponza).
std::vector<Ray3>
cameraRays(const Scene& s, const Options& o, std::string& a_description)
{
  const V3 lo{s.lo[0], s.lo[1], s.lo[2]};
  const V3 hi{s.hi[0], s.hi[1], s.hi[2]};
  const V3 c   = 0.5 * (lo + hi);
  const V3 ext = hi - lo;

  V3     eye, target;
  double fovY;

  if (o.camera == "inside") {
    const bool alongX = ext.x >= ext.z;
    const V3   axis   = alongX ? V3{1, 0, 0} : V3{0, 0, 1};
    const double len  = alongX ? ext.x : ext.z;
    eye               = V3{c.x, lo.y + 0.25 * ext.y, c.z} - (0.35 * len) * axis;
    target            = V3{c.x, lo.y + 0.35 * ext.y, c.z} + (0.35 * len) * axis;
    fovY              = 60.0;
  }
  else {
    const double r = 0.5 * std::sqrt(dot(ext, ext));
    fovY           = 40.0;
    const double d = 0.7 * r / std::tan(0.5 * fovY * M_PI / 180.0);
    eye            = c + d * normalize(V3{0.35, 0.25, 1.0});
    target         = c;
  }

  const V3     fwd    = normalize(target - eye);
  const V3     right  = normalize(cross(fwd, V3{0, 1, 0}));
  const V3     up     = cross(right, fwd);
  const double tanY   = std::tan(0.5 * fovY * M_PI / 180.0);
  const double aspect = double(o.width) / double(o.height);

  std::vector<Ray3> rays;
  rays.reserve(std::size_t(o.width) * std::size_t(o.height));

  for (int j = 0; j < o.height; j++) {
    for (int i = 0; i < o.width; i++) {
      const double px = (2.0 * (i + 0.5) / o.width - 1.0) * tanY * aspect;
      const double py = (1.0 - 2.0 * (j + 0.5) / o.height) * tanY;
      rays.push_back(makeRay(eye, fwd + px * right + py * up));
    }
  }

  char buf[256];
  std::snprintf(buf,
                sizeof(buf),
                "%dx%d pinhole, %s, eye (%.4g, %.4g, %.4g), target (%.4g, %.4g, %.4g), vertical fov %.0f deg",
                o.width,
                o.height,
                o.camera.c_str(),
                eye.x,
                eye.y,
                eye.z,
                target.x,
                target.y,
                target.z,
                fovY);
  a_description = buf;

  return rays;
}

// Origins uniform in the scene's box, directions uniform on the sphere.
std::vector<Ray3>
randomRays(const Scene& s, const std::size_t n, const uint64_t seed)
{
  Rng               rng{seed};
  std::vector<Ray3> rays(n);

  for (auto& r : rays) {
    const V3 o{s.lo[0] + rng.uniform() * (s.hi[0] - s.lo[0]),
               s.lo[1] + rng.uniform() * (s.hi[1] - s.lo[1]),
               s.lo[2] + rng.uniform() * (s.hi[2] - s.lo[2])};

    const double z   = 2.0 * rng.uniform() - 1.0;
    const double phi = 2.0 * M_PI * rng.uniform();
    const double rho = std::sqrt(std::max(0.0, 1.0 - z * z));

    r = makeRay(o, V3{rho * std::cos(phi), rho * std::sin(phi), z});
  }

  return rays;
}

// ---------------------------------------------------------------------------------------------
// Threading harness: the same for every library
// ---------------------------------------------------------------------------------------------

constexpr std::size_t ChunkRays = 1024;

// Rays in chunks of ChunkRays; chunk c goes to thread c % a_threads (static, round-robin, so that
// the threads share the frame's empty and busy regions alike). One thread runs on the caller.
template <class F>
void
parallelChunks(const std::size_t a_n, const int a_threads, F&& a_f)
{
  const std::size_t chunks = (a_n + ChunkRays - 1) / ChunkRays;

  auto work = [&](const int a_tid) {
    for (std::size_t c = std::size_t(a_tid); c < chunks; c += std::size_t(a_threads)) {
      const std::size_t b = c * ChunkRays;
      a_f(b, std::min(a_n, b + ChunkRays) - b);
    }
  };

  if (a_threads == 1) {
    work(0);
    return;
  }

  std::vector<std::thread> pool;
  pool.reserve(std::size_t(a_threads));

  for (int t = 0; t < a_threads; t++) {
    pool.emplace_back(work, t);
  }

  for (auto& t : pool) {
    t.join();
  }
}

double
now()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct Stats
{
  double median, min, max;
};

// a_values are per-run values (seconds or rates); returns median, min, max.
Stats
stats(std::vector<double> a_values)
{
  std::sort(a_values.begin(), a_values.end());
  const std::size_t n = a_values.size();
  const double      m = (n % 2) ? a_values[n / 2] : 0.5 * (a_values[n / 2 - 1] + a_values[n / 2]);
  return Stats{m, a_values.front(), a_values.back()};
}

// Mrays/s of tracing a_rays, a warm-up run and then a_reps timed runs.
Stats
timeTrace(const Accel& a_accel, const std::vector<Ray3>& a_rays, std::vector<Hit>& a_hits, const int a_threads,
          const int a_reps)
{
  a_hits.resize(a_rays.size());
  std::vector<double> rates;

  for (int r = -1; r < a_reps; r++) {
    const double t0 = now();
    parallelChunks(a_rays.size(), a_threads, [&](std::size_t b, std::size_t n) {
      a_accel.trace(&a_rays[b], &a_hits[b], n);
    });
    const double t1 = now();

    if (r >= 0) {
      rates.push_back(1.0e-6 * double(a_rays.size()) / (t1 - t0));
    }
  }

  return stats(rates);
}

// Build time in ms: a warm-up build, then a_reps timed builds.
Stats
timeBuild(Accel& a_accel, const bool a_threaded, const int a_reps)
{
  std::vector<double> ms;

  for (int r = -1; r < a_reps; r++) {
    const double t0 = now();
    a_accel.build(a_threaded);
    const double t1 = now();

    if (r >= 0) {
      ms.push_back(1.0e3 * (t1 - t0));
    }
  }

  return stats(ms);
}

struct Checksum
{
  std::size_t hits = 0;
  double      sumT = 0.0;
  uint64_t    sumP = 0;
};

Checksum
checksum(const std::vector<Hit>& a_hits)
{
  Checksum c;

  for (const auto& h : a_hits) {
    if (h.prim != UINT32_MAX) {
      c.hits++;
      c.sumT += double(h.t);
      c.sumP += h.prim;
    }
  }

  return c;
}

// ---------------------------------------------------------------------------------------------
// Independent reference and cross-checks
// ---------------------------------------------------------------------------------------------

V3
vertex(const Scene& s, uint32_t tri, int k)
{
  const float* p = &s.xyz[9 * std::size_t(tri) + 3 * std::size_t(k)];
  return V3{p[0], p[1], p[2]};
}

// The ray's crossing of triangle a_tri's plane, in double: its t, and the signed in-plane distance of
// that point from the triangle's boundary (positive inside, negative outside). det == 0: no crossing.
struct PlaneHit
{
  bool   ok;
  double t;
  double edgeDistance;
  bool   inWidened; // inside EBGeometry's widened triangle test (TriangleRay::crossing()), in float epsilon
  double thickness; // the triangle's smallest altitude
};

PlaneHit
planeHit(const Scene& s, const Ray3& r, const uint32_t a_tri)
{
  const V3 o{r.o[0], r.o[1], r.o[2]};
  const V3 d{r.d[0], r.d[1], r.d[2]};
  const V3 v0 = vertex(s, a_tri, 0), v1 = vertex(s, a_tri, 1), v2 = vertex(s, a_tri, 2);
  const V3 e1 = v1 - v0, e2 = v2 - v0;
  const V3 p   = cross(d, e2);
  const double det = dot(e1, p);

  if (det == 0.0) {
    return PlaneHit{false, 0, 0, false, 0};
  }

  const double inv = 1.0 / det;
  const V3     sv  = o - v0;
  const double u   = dot(sv, p) * inv;
  const V3     q   = cross(sv, e1);
  const double v   = dot(d, q) * inv;
  const double t   = dot(e2, q) * inv;
  const double w   = 1.0 - u - v;

  // Barycentric coordinate times the altitude onto the opposite edge is the distance from that edge.
  const double area2 = std::sqrt(dot(cross(e1, e2), cross(e1, e2)));
  const V3     e12   = v2 - v1;
  const double hu    = area2 / std::sqrt(dot(e2, e2));   // u weighs v1: its opposite edge is v0-v2
  const double hv    = area2 / std::sqrt(dot(e1, e1));   // v weighs v2: its opposite edge is v0-v1
  const double hw    = area2 / std::sqrt(dot(e12, e12)); // w weighs v0: its opposite edge is v1-v2

  // EBGeometry's widening, as TriangleRay::crossing() computes it: each barycentric coordinate times a
  // lower bound on twice the area (the largest component of e1 x e2) may be as low as -slack.
  const V3     n     = cross(e1, e2);
  const double a2lo  = std::max({std::abs(n.x), std::abs(n.y), std::abs(n.z)});
  const double ulps  = 4.0 * double(std::numeric_limits<float>::epsilon());
  const double delta = ulps * (std::abs(o.x) + std::abs(o.y) + std::abs(o.z) + std::abs(v0.x) + std::abs(v0.y) +
                               std::abs(v0.z));
  const double slack = ulps * a2lo + delta * (std::abs(e1.x) + std::abs(e1.y) + std::abs(e1.z) + std::abs(e2.x) +
                                              std::abs(e2.y) + std::abs(e2.z));
  const bool   inW   = u * a2lo + slack >= 0 && v * a2lo + slack >= 0 && w * a2lo + slack >= 0;

  return PlaneHit{true, t, std::min({u * hu, v * hv, w * hw}), inW, std::min({hu, hv, hw})};
}

// Brute force: every triangle, double-precision Moller-Trumbore, no tolerance, t >= 0.
Hit
bruteForce(const Scene& s, const Ray3& r)
{
  const V3 o{r.o[0], r.o[1], r.o[2]};
  const V3 d{r.d[0], r.d[1], r.d[2]};
  double   bestT = INFINITY;
  uint32_t best  = UINT32_MAX;

  for (uint32_t i = 0; i < uint32_t(s.size()); i++) {
    const V3     v0 = vertex(s, i, 0);
    const V3     e1 = vertex(s, i, 1) - v0;
    const V3     e2 = vertex(s, i, 2) - v0;
    const V3     p  = cross(d, e2);
    const double det = dot(e1, p);

    if (det == 0.0) {
      continue;
    }

    const double inv = 1.0 / det;
    const V3     sv  = o - v0;
    const double u   = dot(sv, p) * inv;
    const V3     q   = cross(sv, e1);
    const double v   = dot(d, q) * inv;
    const double t   = dot(e2, q) * inv;

    if (u >= 0.0 && v >= 0.0 && u + v <= 1.0 && t >= 0.0 && t < bestT) {
      bestT = t;
      best  = i;
    }
  }

  return best == UINT32_MAX ? Hit{INFINITY, UINT32_MAX} : Hit{float(bestT), best};
}

// The classes of disagreement between two answers A and B for one ray (B is the reference).
enum class Kind
{
  Agree,       // both miss, or both hit within the tolerance and name the same triangle
  Tie,         // both hit within the tolerance, different triangles (an edge or vertex they share, or
               // coincident triangles): not a mismatch
  Rounding,    // both hit the same triangle, at distances that differ by more than the tolerance (float
               // rounding of t, at a grazing angle): explained, not a mismatch
  OnSurface,   // the ray starts on a triangle (t within the tolerance of 0): one counts the crossing at
               // t = 0, the other does not (its t rounds below 0)
  Widened,     // the nearer (or only) hit is outside its triangle by more than the graze distance, but
               // inside EBGeometry's widened test, whose widening grows with the triangle's longest edges
               // (sliver triangles)
  Sliver,      // the nearer (or only) hit's triangle is thinner than the graze distance (nearly degenerate):
               // float rounding decides whether it is crossed
  GrazeExtra,  // one hits, the other misses, and the hit point is within the graze distance of the hit
               // triangle's boundary
  GrazeCrack,  // both hit beyond the tolerance, and the farther one passed within the graze distance
               // of the nearer triangle's boundary (through an edge or vertex)
  Other        // anything else: a real disagreement
};

const char*
kindName(Kind k)
{
  switch (k) {
  case Kind::Agree: {
    return "agree";
  }
  case Kind::Tie: {
    return "tie (same t, other triangle)";
  }
  case Kind::Rounding: {
    return "same triangle, t rounded differently (grazing angle)";
  }
  case Kind::OnSurface: {
    return "ray starts on a triangle (t = 0)";
  }
  case Kind::Widened: {
    return "hit outside a sliver triangle, within EBGeometry's widened test";
  }
  case Kind::Sliver: {
    return "nearly degenerate triangle (thinner than the graze distance)";
  }
  case Kind::GrazeExtra: {
    return "hit/miss at a triangle's edge (grazing)";
  }
  case Kind::GrazeCrack: {
    return "farther hit through the nearer triangle's edge/vertex";
  }
  default: {
    return "OTHER (unexplained)";
  }
  }
}

struct Tolerances
{
  double t;     // |tA - tB| within this is the same crossing
  double graze; // within this distance of a triangle's boundary is a grazing ray
};

Kind
classify(const Scene& s, const Ray3& r, const Hit& a, const Hit& b, const Tolerances& tol)
{
  const bool ha = a.prim != UINT32_MAX;
  const bool hb = b.prim != UINT32_MAX;

  if (!ha && !hb) {
    return Kind::Agree;
  }

  if (ha && hb && std::abs(double(a.t) - double(b.t)) <= tol.t) {
    return a.prim == b.prim ? Kind::Agree : Kind::Tie;
  }

  if (ha && hb && a.prim == b.prim) {
    return Kind::Rounding;
  }

  const Hit&     first = (ha != hb) ? (ha ? a : b) : (a.t < b.t ? a : b);
  const PlaneHit h     = planeHit(s, r, first.prim);

  if (h.ok && std::abs(h.t) <= tol.t) {
    return Kind::OnSurface;
  }

  if (h.ok && std::abs(h.edgeDistance) <= tol.graze) {
    return (ha != hb) ? Kind::GrazeExtra : Kind::GrazeCrack;
  }

  if (h.ok && h.edgeDistance < 0 && h.inWidened) {
    return Kind::Widened;
  }

  if (h.ok && h.thickness <= tol.graze) {
    return Kind::Sliver;
  }

  return Kind::Other;
}

// Print an unexplained mismatch: both answers, the brute-force answer, and where each answer's triangle
// is crossed (t and the distance from its boundary, negative outside).
void
explain(const Scene& s, const Ray3& r, const std::size_t a_index, const Hit& a, const Hit& b)
{
  const Hit bf = bruteForce(s, r);
  std::printf("             ray %zu: o (%.9g, %.9g, %.9g) d (%.9g, %.9g, %.9g)\n", a_index, r.o[0], r.o[1], r.o[2],
              r.d[0], r.d[1], r.d[2]);
  const Hit*  hs[3]    = {&a, &b, &bf};
  const char* names[3] = {"this", "reference", "brute force"};
  for (int k = 0; k < 3; k++) {
    if (hs[k]->prim == UINT32_MAX) {
      std::printf("               %-12s miss\n", names[k]);
    }
    else {
      const PlaneHit h = planeHit(s, r, hs[k]->prim);
      std::printf("               %-12s t %.9g prim %u (double: t %.9g, edge distance %.3g%s)\n", names[k],
                  hs[k]->t, hs[k]->prim, h.t, h.edgeDistance, h.inWidened ? ", inside the widened test" : "");
      for (int j = 0; j < 3; j++) {
        const V3 v = vertex(s, hs[k]->prim, j);
        std::printf("                 v%d (%.9g, %.9g, %.9g)\n", j, v.x, v.y, v.z);
      }
    }
  }
}

using Tally = std::map<Kind, std::size_t>;

// Unexplained mismatches over the whole run: the program exits with an error if there are any.
std::size_t g_unexplained = 0;

std::string
tallyString(const Tally& t, std::size_t n)
{
  std::ostringstream os;
  std::size_t        mismatches = 0;

  for (const auto& [k, c] : t) {
    if (k != Kind::Agree && k != Kind::Tie && k != Kind::Rounding) {
      mismatches += c;
    }
  }

  os << mismatches << "/" << n << " mismatches";

  if (t.count(Kind::Other)) {
    g_unexplained += t.at(Kind::Other);
  }

  for (const auto& [k, c] : t) {
    if (k != Kind::Agree) {
      os << "; " << kindName(k) << ": " << c;
    }
  }

  return os.str();
}

std::size_t
countFileTriangles(const std::string& a_file)
{
  std::ifstream f(a_file);
  std::string   line;
  std::size_t   n = 0;

  if (a_file.size() > 4 && a_file.substr(a_file.size() - 4) == ".ply") {
    while (std::getline(f, line) && line != "end_header") {
      if (line.rfind("element face", 0) == 0) {
        return std::stoul(line.substr(13));
      }
    }
    return 0;
  }

  while (std::getline(f, line)) {
    if (line.rfind("f ", 0) == 0) {
      std::istringstream is(line.substr(2));
      std::string        tok;
      int                k = 0;
      while (is >> tok) {
        k++;
      }
      n += std::size_t(std::max(0, k - 2));
    }
  }

  return n;
}

std::vector<std::string>
split(const std::string& a_s)
{
  std::vector<std::string> out;
  std::stringstream        ss(a_s);
  std::string              item;

  while (std::getline(ss, item, ',')) {
    out.push_back(item);
  }

  return out;
}

} // namespace

int
main(int argc, char** argv)
{
  const Options opt = parse(argc, argv);

  std::printf("RayTracing benchmark: EBGeometry TriMeshSDF vs tinybvh %s, closest-hit rays, float\n",
              tinyBVHVersion().c_str());
  std::printf("  CPU: %s, %u hardware threads; all-thread runs use %d threads; single-thread runs %s\n",
              cpuModel().c_str(),
              std::thread::hardware_concurrency(),
              opt.threads,
              opt.pinCpu >= 0 ? ("pinned to CPU " + std::to_string(opt.pinCpu)).c_str() : "not pinned");
#if defined(__VERSION__)
  std::printf("  Compiler: %s\n", __VERSION__);
#endif

  // Read the scene (untimed): one triangle list for both libraries.
  Scene scene;
  {
    const double t0 = now();
    try {
      scene = loadScene(opt.scene, opt.warn);
    }
    catch (const std::exception& e) {
      std::printf("  EBGeometry could not read %s: %s\n", opt.scene.c_str(), e.what());
      return 1;
    }
    std::printf("  Scene: %s: %zu triangles in the file, %zu after EBGeometry's reader (read in %.1f s)\n",
                opt.scene.c_str(),
                countFileTriangles(opt.scene),
                scene.size(),
                now() - t0);
  }

  const double diag = std::sqrt(double(scene.hi[0] - scene.lo[0]) * double(scene.hi[0] - scene.lo[0]) +
                                double(scene.hi[1] - scene.lo[1]) * double(scene.hi[1] - scene.lo[1]) +
                                double(scene.hi[2] - scene.lo[2]) * double(scene.hi[2] - scene.lo[2]));
  std::printf("  Box: (%.5g, %.5g, %.5g) to (%.5g, %.5g, %.5g), diagonal %.5g\n",
              scene.lo[0],
              scene.lo[1],
              scene.lo[2],
              scene.hi[0],
              scene.hi[1],
              scene.hi[2],
              diag);

  std::string             camDescription;
  const std::vector<Ray3> camera = cameraRays(scene, opt, camDescription);
  const std::vector<Ray3> random = randomRays(scene, opt.random ? opt.random : camera.size(), 20261120ull);
  std::printf("  Coherent rays:   %zu, %s\n", camera.size(), camDescription.c_str());
  std::printf("  Incoherent rays: %zu, origins uniform in the box, directions uniform, seed 20261120\n",
              random.size());
  std::printf("  Timing: one warm-up, then %d repetitions; median [min, max]\n\n", opt.reps);

  const Tolerances tol{1.0e-5 * diag, 1.0e-5 * diag};

  struct RaySet
  {
    const char*              name;
    const std::vector<Ray3>* rays;
  };
  const RaySet sets[2] = {{"coherent", &camera}, {"incoherent", &random}};

  // Brute-force reference on an evenly spread sample of each ray set (all threads, untimed).
  std::vector<std::size_t> sampleIdx[2];
  std::vector<Hit>         brute[2];

  for (int k = 0; k < 2; k++) {
    const std::size_t n = std::min(opt.sample, sets[k].rays->size());

    for (std::size_t i = 0; i < n; i++) {
      sampleIdx[k].push_back(i * sets[k].rays->size() / n);
    }

    brute[k].resize(n);
    const double t0 = now();
    parallelChunks(n, opt.threads, [&](std::size_t b, std::size_t m) {
      for (std::size_t i = b; i < b + m; i++) {
        brute[k][i] = bruteForce(scene, (*sets[k].rays)[sampleIdx[k][i]]);
      }
    });
    std::printf("  Brute force (double, every triangle): %zu %s rays in %.1f s\n", n, sets[k].name, now() - t0);
  }

  std::printf("  Tolerances: same crossing if |dt| <= %.3g (1e-5 of the diagonal); grazing if within %.3g of "
              "a triangle's boundary\n\n",
              tol.t,
              tol.graze);

  // The configurations.
  std::vector<std::unique_ptr<Accel>> accels;

  for (const auto& c : split(opt.configs)) {
    // eb4, eb8: the defaults; eb4:G, eb8:G: at most G groups of W triangles per leaf (not a default).
    if (c.rfind("eb4", 0) == 0 || c.rfind("eb8", 0) == 0) {
      const int groups = c.size() > 4 ? std::stoi(c.substr(4)) : 0;
      accels.push_back(makeEBGeometry(scene, c[2] == '4' ? 4 : 0, groups));
    }
    else accels.push_back(makeTinyBVH(scene, c));
  }

  // The plain tinybvh BVH's answers are the per-ray reference; computed first if it is listed.
  std::vector<Hit> reference[2];

  struct Row
  {
    std::string name;
    Stats       build1, buildN;
    bool        threadedBuild;
    Stats       rate[2][2]; // [set][1 thread, all threads]
  };
  std::vector<Row> rows;

  std::vector<std::size_t> order(accels.size());
  for (std::size_t i = 0; i < order.size(); i++) {
    order[i] = i;
  }
  std::stable_partition(order.begin(), order.end(), [&](std::size_t i) { return accels[i]->name() == "tinybvh BVH"; });

  for (const std::size_t ai : order) {
    Accel& a = *accels[ai];
    Row    row;
    row.name          = a.name();
    row.threadedBuild = a.threadedBuild();

    std::printf("== %s\n", row.name.c_str());

    // Builds: on one (pinned) thread, then, where the library can, its default threaded build. The
    // structure traced below is the one the last build left: the default build.
    pin(opt.pinCpu);
    row.build1 = timeBuild(a, false, opt.reps);
    pin(-1);
    std::printf("  build, 1 thread:     %9.1f ms [%.1f, %.1f]\n", row.build1.median, row.build1.min, row.build1.max);

    if (row.threadedBuild) {
      row.buildN = timeBuild(a, true, opt.reps);
      std::printf("  build, threaded:     %9.1f ms [%.1f, %.1f]\n", row.buildN.median, row.buildN.min, row.buildN.max);
    }

    for (int k = 0; k < 2; k++) {
      std::vector<Hit> hits;

      pin(opt.pinCpu);
      row.rate[k][0] = timeTrace(a, *sets[k].rays, hits, 1, opt.reps);
      pin(-1);
      const Checksum c1 = checksum(hits);

      row.rate[k][1] = timeTrace(a, *sets[k].rays, hits, opt.threads, opt.reps);
      const Checksum cN = checksum(hits);

      std::printf("  %-10s 1 thread: %8.2f Mrays/s [%.2f, %.2f]   %d threads: %8.2f Mrays/s [%.2f, %.2f]\n",
                  sets[k].name,
                  row.rate[k][0].median,
                  row.rate[k][0].min,
                  row.rate[k][0].max,
                  opt.threads,
                  row.rate[k][1].median,
                  row.rate[k][1].min,
                  row.rate[k][1].max);
      std::printf("             checksum: %zu hits, sum t %.9g, sum prim %llu%s\n",
                  cN.hits,
                  cN.sumT,
                  (unsigned long long)cN.sumP,
                  (c1.hits == cN.hits && c1.sumT == cN.sumT && c1.sumP == cN.sumP) ? "" : "  (1-thread run DIFFERS)");

      // Cross-checks.
      if (reference[k].empty() && row.name == "tinybvh BVH") {
        reference[k] = hits;
      }
      else if (!reference[k].empty()) {
        Tally t;
        for (std::size_t i = 0; i < hits.size(); i++) {
          const Kind kind = classify(scene, (*sets[k].rays)[i], hits[i], reference[k][i], tol);
          t[kind]++;
          if (kind == Kind::Other && t[kind] <= 3) {
            explain(scene, (*sets[k].rays)[i], i, hits[i], reference[k][i]);
          }
        }
        std::printf("             vs tinybvh BVH, every ray: %s\n", tallyString(t, hits.size()).c_str());
      }

      Tally tb;
      for (std::size_t i = 0; i < sampleIdx[k].size(); i++) {
        tb[classify(scene, (*sets[k].rays)[sampleIdx[k][i]], hits[sampleIdx[k][i]], brute[k][i], tol)]++;
      }
      std::printf("             vs brute force, sample:    %s\n", tallyString(tb, sampleIdx[k].size()).c_str());
    }

    rows.push_back(row);
    std::printf("\n");
  }

  // The summary table.
  std::printf("Summary (%s, %zu triangles; median of %d runs)\n", opt.scene.c_str(), scene.size(), opt.reps);
  std::printf("%-44s %10s %10s | %12s %12s | %12s %12s\n",
              "",
              "build ms",
              "build ms",
              "coherent",
              "coherent",
              "incoherent",
              "incoherent");
  std::printf("%-44s %10s %10s | %12s %12s | %12s %12s\n", "Configuration", "1 thread", "threaded", "1 thr Mr/s",
              (std::to_string(opt.threads) + " thr Mr/s").c_str(), "1 thr Mr/s",
              (std::to_string(opt.threads) + " thr Mr/s").c_str());

  for (const auto& r : rows) {
    char buildN[32];
    if (r.threadedBuild) {
      std::snprintf(buildN, sizeof(buildN), "%10.1f", r.buildN.median);
    }
    else {
      std::snprintf(buildN, sizeof(buildN), "%10s", "-");
    }
    std::printf("%-44s %10.1f %s | %12.2f %12.2f | %12.2f %12.2f\n",
                r.name.c_str(),
                r.build1.median,
                buildN,
                r.rate[0][0].median,
                r.rate[0][1].median,
                r.rate[1][0].median,
                r.rate[1][1].median);
  }

  if (g_unexplained > 0) {
    std::printf("\nFAILED: %zu unexplained mismatches (see above)\n", g_unexplained);
    return 2;
  }

  return 0;
}
