EBGeometry-benchmarks
======================

Reproducible micro-benchmarks that put [EBGeometry](https://github.com/rmrsk/EBGeometry)'s geometry
queries next to other open-source libraries on tasks they have in common, so a reader can see how
the tradeoffs play out and re-run the measurements on their own hardware. The point is the
methodology and the cross-checks, not a scoreboard — numbers vary widely across machines, compilers,
and ISAs, so the tables in each folder's `README.md` are illustrative single-machine snapshots
rather than definitive results.

This is a standalone companion repository, not part of the main EBGeometry repository: the
comparison libraries below have their own build systems, release cadences, and licenses, so they
are kept out of EBGeometry's own history and CI. Every benchmark result is cross-checked against a
brute-force / independent baseline, so a wrong answer shows up as a mismatch — correctness is the
part that transfers across machines even when the timings do not.

* [`NearestNeighbor/`](NearestNeighbor/README.md) — all-nearest-neighbor on a point cloud:
  EBGeometry's `PointCloudBVH`/`PointCloudHashGrid` vs
  [nanoflann](https://github.com/jlblancoc/nanoflann),
  [picoflann](https://github.com/rmsalinas/picoflann), and
  [kd3](https://github.com/KaruroChori/kd3).
* [`MeshSDF/`](MeshSDF/README.md) — closest-point on a triangle mesh: EBGeometry's `TriMeshSDF` vs
  [fcpw](https://github.com/rohan-sawhney/fcpw) and
  [TriangleMeshDistance](https://github.com/InteractiveComputerGraphics/TriangleMeshDistance).

Cloning
-------

```bash
git clone --recurse-submodules git@github.com:rmrsk/EBGeometry-benchmarks.git
```

All dependencies — EBGeometry itself and every comparison library — are pinned as git submodules
under `Submodules/`. If cloned without `--recurse-submodules`, run
`git submodule update --init --recursive` afterward. `Submodules/EBGeometry` tracks the latest
commit on EBGeometry's `main` branch (see "Keeping EBGeometry up to date" below); every other
submodule is pinned to a fixed commit and only bumped by hand when a comparison library needs
updating.

Building
--------

Each benchmark folder is self-contained: see its own `README.md` for what it measures and its own
`GNUmakefile` for how to build it (`cd <Folder> && make && ./<Name>.ex`).

Keeping EBGeometry up to date
------------------------------

Unlike the comparison libraries, `Submodules/EBGeometry` is not pinned by hand: a scheduled
workflow (`.github/workflows/bump-ebgeometry.yml`) periodically checks EBGeometry's `main` branch
and, if it has moved, builds and runs both benchmarks against that candidate commit first. Only if
that succeeds does it commit the new submodule pointer and push — a broken EBGeometry commit is
never pinned, it just fails the workflow run (visible in the Actions tab). So a fresh clone is
never more than one polling interval behind EBGeometry's `main`, and `main` here never ends up
pinned to an EBGeometry commit that doesn't build or crashes. See that workflow file for the exact
schedule and for how to move to tracking tagged releases instead, once EBGeometry starts cutting
them.

CI
--

`.github/workflows/CI.yml` fetches every submodule, compiles each benchmark, and runs it (unlike
EBGeometry's own CI, which only compiles these as an external smoke test). A run failure here means
a benchmark stopped building or crashed against the current EBGeometry commit — not a performance
regression; read the job log for the printed cross-check output if you want the numbers.

License
-------

GPL-3.0-or-later, matching EBGeometry; see `LICENSE`. The comparison libraries under `Submodules/`
carry their own licenses.
