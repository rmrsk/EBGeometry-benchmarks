RayTracing
==========

Closest-hit rays against a triangle mesh: EBGeometry's `TriMeshSDF` traced with `firstHit()` next to
[tinybvh](https://github.com/jbikker/tinybvh), a header-only BVH library for ray tracing triangles. The
scene is read once by EBGeometry's reader into one triangle list; each library then builds its own
structure over the same triangles, in the same order, and answers the same rays.

What is measured
----------------

* **Build** time from the triangle list, in ms. Each library's input format (EBGeometry's `Triangle`
  list, tinybvh's three `float4` per triangle) is prepared untimed. EBGeometry's builder is
  single-threaded; tinybvh's default builder is threaded, so tinybvh is timed both on one thread
  (`settings.enableThreading = false`) and with its default threaded build. The traced structure is the
  one its default build makes.
* **Closest-hit rays** over `[0, inf)`, in Mrays/s, for two ray sets: coherent camera rays (one
  1920x1080 pinhole frame of the scene) and incoherent random rays (as many, origins uniform in the
  scene's box, directions uniform, fixed seed). On one thread and on all hardware threads, with the same
  harness for both libraries (`std::thread`, static round-robin chunks of 1024 rays).
* A warm-up run, then several repetitions; the median and the [min, max] are printed, with checksums.

Configurations:

| Configuration | What it is |
|---|---|
| EBGeometry `TriMeshSDF<float, 4, 4>` | the portable defaults (`readIntoTriMeshSDF`'s K and W) |
| EBGeometry `TriMeshSDF<float, 4, HostWidth>` | the host-tuned triangle width (8 with AVX2), with `EBGEOMETRY_HOST_TUNED_DEFAULTS` defined for EBGeometry's translation unit, the only one that includes it |
| tinybvh `BVH` | its plain binary BVH, the layout every other is made from, default (Bonsai) builder |
| tinybvh `BVH4_CPU` | its 4-wide CPU layout (AVX2), which its README calls the fastest on CPU |
| tinybvh `BVH8_CPU` | its 8-wide CPU layout (AVX2) |

EBGeometry builds with its default for `readIntoTriMeshSDF`: binned SAH, at most four groups of W
triangles per leaf.

Cross-checks
------------

Every configuration's answer is compared, ray by ray, with tinybvh's plain `BVH`, and on an evenly spread
sample of 10,000 rays of each set with a double-precision brute-force loop over every triangle. Two hits
are the same crossing if their distances agree within 1e-5 of the scene's diagonal. Each disagreement is
classified:

* *tie*: same distance, another triangle (an edge or vertex that triangles share, or coincident
  triangles). Not a mismatch.
* *hit/miss at a triangle's edge*: one hits and the other misses, and the hit point lies within 1e-5 of
  the diagonal of the hit triangle's boundary (a grazing ray).
* *farther hit through the nearer triangle's edge*: both hit, at different distances, and the farther
  one passed within that distance of the nearer triangle's boundary (a ray through a crack between
  triangles).
* *hit outside a sliver triangle, within EBGeometry's widened test*: the nearer (or only) hit lies outside
  its triangle by more than that distance, but inside EBGeometry's widened triangle test, whose widening
  is scaled by the triangle's longest edges (so it reaches far outside a long, thin triangle).
* *same triangle, t rounded differently*: both hit the same triangle, at distances differing by more than
  the tolerance (float rounding of t at a grazing angle).
* *nearly degenerate triangle*: the hit triangle is thinner than the tolerance; float rounding decides.
* *ray starts on a triangle*: a random origin lies on a triangle (t = 0), which one side counts and the
  other rounds below 0.
* *other*: anything else. The program prints the first few with their triangles and exits with an
  error if there are any.

Ties and same-triangle rounding are not counted as mismatches; the other classes are, and are
explained.

What each library's query computes
----------------------------------

Both return the nearest crossing of any triangle, from either side (no back-face culling), and neither
depends on whether the ray starts inside or outside the mesh: `TriMeshSDF`'s ray form walks its BVH and
tests triangles, and the sign of the distance plays no part. The differences:

* EBGeometry widens its triangle test by the rounding error of the crossing point, so a ray through an
  edge or vertex shared by two triangles crosses at least one of them (no cracks), and a ray passing
  just outside a triangle may count as crossing it. tinybvh's default Moller-Trumbore test is not widened
  (its `WATERTIGHT_TRITEST` is off by default).
* Where several triangles are crossed within a few ulps (a shared edge or vertex), EBGeometry reports
  the lowest face id; tinybvh the first it found.
* EBGeometry's `HitT` also carries the hit point, the triangle's normal and whether the ray enters;
  tinybvh's hit record carries t, the barycentrics u and v, and the primitive.

Building and running
--------------------

Requires the `tinybvh` submodule, and the mesh submodule for the default input:

```bash
git submodule update --init ../Submodules/tinybvh ../Submodules/common-3d-test-models
make && ./RayTracing.ex
```

`EBGEOMETRY_HOME` and `TINYBVH_HOME` default to the submodules; override them
(`make EBGEOMETRY_HOME=...`) to build against other checkouts. EBGeometry's translation unit is C++17,
tinybvh's needs C++17 for its threaded builds, and all three files are compiled with the same flags
(EBGeometry requires one set of SIMD flags per program).

Without arguments, the program traces the armadillo of `common-3d-test-models` (99,976 triangles); CI
runs it that way. The results below are for two standard ray-tracing scenes, which are not in the
repository. Download each into a directory of its own, outside the repository:

| Scene | Source | sha256 of the download | Triangles |
|---|---|---|---|
| Stanford dragon, `dragon_recon/dragon_vrip.ply` | [Stanford 3D Scanning Repository](http://graphics.stanford.edu/pub/3Dscanrep/dragon/dragon_recon.tar.gz) | `74ac1d90989c9b1732edee82d57e9ce71452144cf4355f108d8c9c616d28d02f` | 871,414 in the file; EBGeometry's reader removes 108 zero-area faces, leaving 871,306 |
| Crytek Sponza, `sponza.obj` | [Morgan McGuire's Computer Graphics Archive](https://casual-effects.com/g3d/data10/common/model/crytek_sponza/sponza.zip) | `da005cbee0be2df2abc8513f3ceb61bcb6f69aac112babcd9c00169a27c2770c` | 262,267 (after fan triangulation) |

```bash
mkdir -p ~/scenes/dragon ~/scenes/sponza
curl -L -o ~/scenes/dragon/dragon_recon.tar.gz http://graphics.stanford.edu/pub/3Dscanrep/dragon/dragon_recon.tar.gz
curl -L -o ~/scenes/sponza/sponza.zip https://casual-effects.com/g3d/data10/common/model/crytek_sponza/sponza.zip
sha256sum ~/scenes/dragon/dragon_recon.tar.gz ~/scenes/sponza/sponza.zip
tar -xzf ~/scenes/dragon/dragon_recon.tar.gz -C ~/scenes/dragon
unzip -q ~/scenes/sponza/sponza.zip sponza.obj -d ~/scenes/sponza

./RayTracing.ex --scene ~/scenes/dragon/dragon_recon/dragon_vrip.ply --pin-cpu 2
./RayTracing.ex --scene ~/scenes/sponza/sponza.obj --warn --camera inside --pin-cpu 2
```

The dragon has holes, which EBGeometry's reader allows; it loads as it is. Sponza is not a closed,
consistently oriented surface, so EBGeometry's reader throws on it unless given `--warn`
(`Parser::OnDefect::Warn`), with which it loads (with warnings) and traces. `--camera inside` puts the
camera in the atrium, looking along it; the default (`outside`) views the whole object.

`--pin-cpu` pins the single-thread runs to one CPU (on a hybrid CPU, pick a performance core). Run
`./RayTracing.ex --help` for the other options (frame size, ray counts, repetitions, threads, and
`--configs`, which also takes `eb4:G`/`eb8:G` for EBGeometry leaves of at most G groups of W triangles).

Representative result (one machine, illustrative -- numbers vary widely across machines, compilers,
and ISAs; re-run rather than quote)
------------------------------------

Intel Core i9-12900K (8 P-cores with 2 threads each, 8 E-cores; 24 threads), g++ 13.3.0
`-std=c++17 -O3 -march=native` (AVX2), EBGeometry b4fb4c6, tinybvh 1.9.2 (58439ef), single-thread runs
pinned to a P-core, the machine otherwise idle; 2,073,600 rays per set (the dragon frame from outside,
29% of its rays hit; Sponza from inside, all hit), median of 5 runs after a warm-up (the spread was
within 2% almost everywhere). Build in ms, rays in Mrays/s.

Stanford dragon (871,306 triangles after the reader removed 108 zero-area faces):

```
                                               build ms   build ms |     coherent     coherent |   incoherent   incoherent
Configuration                                  1 thread   threaded |   1 thr Mr/s  24 thr Mr/s |   1 thr Mr/s  24 thr Mr/s
tinybvh BVH                                        78.0       54.0 |         9.89       124.49 |         1.89        29.56
EBGeometry TriMeshSDF K=4 W=4 (defaults)          682.6          - |         8.97       106.05 |         2.23        30.66
EBGeometry TriMeshSDF K=4 W=8 (host-tuned)        636.6          - |         9.41       103.80 |         2.30        29.05
tinybvh BVH4_CPU                                  130.5       45.2 |        20.58       254.57 |         4.38        62.86
tinybvh BVH8_CPU                                  142.6       48.6 |        20.77       252.23 |         5.15        77.51
```

Crytek Sponza (262,267 triangles), interior view:

```
                                               build ms   build ms |     coherent     coherent |   incoherent   incoherent
Configuration                                  1 thread   threaded |   1 thr Mr/s  24 thr Mr/s |   1 thr Mr/s  24 thr Mr/s
tinybvh BVH                                        20.6        4.7 |         4.70        52.87 |         2.83        38.69
EBGeometry TriMeshSDF K=4 W=4 (defaults)          192.1          - |         1.85        21.18 |         2.25        29.20
EBGeometry TriMeshSDF K=4 W=8 (host-tuned)        171.1          - |         1.92        20.19 |         2.40        28.30
tinybvh BVH4_CPU                                   36.0       14.1 |         8.34       109.49 |         5.66        79.27
tinybvh BVH8_CPU                                   39.7       15.5 |         8.49        97.10 |         7.69       101.19
```

Cross-checks of that run: against the brute force, tinybvh had 0 mismatches and EBGeometry at most 2 in
10,000 (explained: a crack, a sliver). Against tinybvh's `BVH`, ray by ray, EBGeometry had 28 to 206
mismatches in 2,073,600 rays and tinybvh's wide layouts 1 to 8, all in the explained classes; ties
numbered 2,000 to 16,000 per set.

Observations:

* On the dragon, EBGeometry is within about 10% of tinybvh's plain binary `BVH` on coherent rays and about
  20% faster on incoherent ones; tinybvh's wide CPU layouts are about 2.2x faster than EBGeometry. On
  Sponza's interior view, EBGeometry is 2.5x slower than the plain `BVH` on coherent rays, and 3x to 4.5x
  slower than the wide layouts.
* EBGeometry's host-tuned triangle width (8) changes its ray times by at most 5%.
* EBGeometry's build is single-threaded and 4.5x to 9x slower than tinybvh's on one thread.
* Smaller EBGeometry leaves (`eb4:1`) were not faster in a small test, so leaf size does not explain the gap.
  From the code (not profiled): tinybvh's wide layouts use octant-specialized traversal and precomputed
  triangle leaves, while EBGeometry's traversal keeps only child slots on its stack and recomputes the
  parent's child-box entries on every pop, and tests each leaf group's lanes in a scalar loop with its
  tie rule.
