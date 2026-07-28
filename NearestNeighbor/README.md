NearestNeighbor
================

For every point in a 500,000-point cloud (double precision), find its nearest *other* point.
Compares EBGeometry's `PointCloudBVH` and `PointCloudHashGrid` against
[picoflann](https://github.com/rmsalinas/picoflann),
[nanoflann](https://github.com/jlblancoc/nanoflann), and
[kd3](https://github.com/KaruroChori/kd3) (a header-only SoA/SIMD KD-tree; needs C++23). The same
comparison is run over two point distributions, since spatial structures behave very differently
depending on how the points fill space:

* **Uniform in the unit cube** — points fill a 3D volume evenly (the balanced, easy case).
* **On the unit-sphere surface** — points lie on a 2D manifold: locally dense, globally hollow. This
  is the harder case for a uniform grid, whose bounding box is then mostly empty interior cells.

Building
--------

Requires the `picoflann`, `nanoflann`, and `kd3` submodules:

```bash
git submodule update --init ../Submodules/picoflann ../Submodules/nanoflann ../Submodules/kd3
make && ./NearestNeighbor.ex
```

`EBGEOMETRY_HOME` defaults to `../Submodules/EBGeometry`; override it (`make EBGEOMETRY_HOME=...`)
to build against a different EBGeometry checkout.

Every result is cross-checked against a brute-force baseline so a wrong answer shows up as a
mismatch — correctness is the part that transfers across machines even when the timings do not.

Representative result (one machine, illustrative — numbers vary widely across machines, compilers,
and ISAs; re-run rather than quote)
------------------------------------

```
                        uniform cube            sphere surface
Method               Build(ms)  Query(us/pt)  Build(ms)  Query(us/pt)
PointCloudBVH           ~118        0.62         ~105        0.48
PointCloudHashGrid       ~12        1.50          ~11        1.60
picoflann               ~104        0.75         ~104        0.48
nanoflann               ~225        0.41         ~230        0.36
kd3 (double)             ~83        0.57          ~82        0.45
kd3 (float)              ~75        0.51          ~74        0.42
```

- The KD-tree queries are iterated in **Hilbert order** so their node cache is as warm as
  EBGeometry's leaf-order batch (querying in natural order is ~2x slower, which would not be a
  like-for-like comparison). That order costs the KD-trees a one-time spatial sort (~0.35 us/pt here);
  the EBGeometry structures reuse the ordering their build already produced.
- **`PointCloudHashGrid` trades query speed for build speed**: an O(N) uniform grid builds ~8x faster
  than the BVH but scans neighbor cells per query, so it queries ~2x slower. It is the weakest on the
  sphere surface (query ~1.6 us/pt) — the hollow distribution leaves its grid mostly empty while the
  occupied surface cells are denser than the ~1-point-per-cell target.
- The tree/BVH methods, by contrast, get *faster* on the sphere surface than in the cube (the local
  neighborhood is effectively lower-dimensional, so pruning is tighter). nanoflann has the fastest
  raw per-query traversal throughout; `PointCloudBVH` is competitive end-to-end because it gets its
  query order for free and builds faster. Which structure to pick depends on the build/query balance
  and the point distribution — that is the point of running both cases.
- **kd3's headline "~2.2x query throughput vs nanoflann" is a single-threaded `float` result** — in
  kd3's own benchmark both query loops are plain serial loops, so the speedup is its SoA/SIMD
  per-query kernel, *not* multithreading. Here kd3 is run in **double**, which halves that SIMD width;
  that is the main reason it doesn't reach 2x in this table, and it's why these numbers are a lower
  bound on what kd3 can do. It is compiled **without `-fopenmp`**, so its build is single-threaded
  like the others — a *fairer* build comparison than kd3's own docs, whose fast build number is
  OpenMP-parallel (its author notes it is still faster serially). In this fair single-threaded mode
  kd3 posts the fastest build and a competitive query. (kd3 needs C++23, so the benchmark is built
  with `-std=c++23`.)
- **`kd3 (float)` is additionally shown as kd3's native best case** — its SoA/SIMD fast path in its
  intended precision (a different-precision reference, not apples-to-apples with the double field,
  exactly like TriangleMeshDistance's double column in the MeshSDF benchmark). It runs ~7–11% faster
  than `kd3 (double)` here; the gain is modest on this particular machine because its AVX-512 clocks
  down, damping the float SIMD-width advantage — elsewhere the float/double gap (and kd3's lead) is
  larger. This is precisely why the tables are labelled machine-dependent snapshots.
