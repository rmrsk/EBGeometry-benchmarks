MeshSDF
=======

Closest-point queries against a triangle mesh (armadillo, ~100k triangles) — the task
EBGeometry's `TriMeshSDF`, [fcpw](https://github.com/rohan-sawhney/fcpw), and
[TriangleMeshDistance](https://github.com/InteractiveComputerGraphics/TriangleMeshDistance) are all
built for. The mesh is parsed once; each library then builds its own structure over the same
triangles and answers the same queries.

Building
--------

Requires the mesh submodule, `fcpw` (Enoki is vendored inside fcpw; Eigen is a nested submodule),
and `TriangleMeshDistance`:

```bash
git submodule update --init ../Submodules/fcpw ../Submodules/common-3d-test-models ../Submodules/TriangleMeshDistance
git -C ../Submodules/fcpw submodule update --init deps/eigen   # Eigen only (skip GPU slang-rhi)
make && ./MeshSDF.ex
```

`EBGEOMETRY_HOME` defaults to `../Submodules/EBGeometry`; override it (`make EBGEOMETRY_HOME=...`)
to build against a different EBGeometry checkout. fcpw is built with its **Enoki CPU
vectorization** (`FCPW_USE_ENOKI`, vectorized MBVH), the same SIMD fast path `TriMeshSDF` uses.
`FCPW_SIMD_WIDTH` in the `GNUmakefile` should match your ISA (4=SSE, 8=AVX2, 16=AVX-512).

Every result is cross-checked against the other two libraries so a wrong answer shows up as a
mismatch — correctness is the part that transfers across machines even when the timings do not.

Representative result (one machine, 100k queries — illustrative, numbers vary widely across
machines, compilers, and ISAs; re-run rather than quote)
-----------------------------------------------------------

```
Method                  Build(ms)   Query(us/query)
TriMeshSDF                  ~63          2.6
fcpw (Enoki)                ~46          3.5
TriangleMeshDistance       ~115         10.5
```

- fcpw's `findClosestPoint` returns the *unsigned* closest point.
- **TriangleMeshDistance** is a header-only, **double-precision, scalar** (non-SIMD) signed-distance
  library — so its query runs in double and is not a same-precision comparison; it is included as a
  widely-used point of reference.
- `TriMeshSDF` and TriangleMeshDistance compute the *signed* distance (their purpose); the comparison
  is on unsigned closest-surface distance, and the cross-check confirms all three agree.
