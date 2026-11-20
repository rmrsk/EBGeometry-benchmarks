# Benchmark: EBGeometry TriMeshSDF vs tinybvh (closest-hit rays on a triangle mesh).
# Requires the tinybvh submodule, and the mesh submodule for the default input (see README.md):
#   git submodule update --init ../Submodules/tinybvh ../Submodules/common-3d-test-models
# EBGEOMETRY_HOME defaults to the EBGeometry submodule, TINYBVH_HOME to the tinybvh one.
# EBGeometry's translation unit is C++17; tinybvh's needs C++17 for its threaded builds.
# Every file is compiled with the same flags (EBGeometry requires one set of SIMD flags per program).

EBGEOMETRY_HOME ?= ../Submodules/EBGeometry
TINYBVH_HOME    ?= ../Submodules/tinybvh
CXX             ?= g++
CXXFLAGS        ?= -std=c++17 -O3 -march=native
LDFLAGS         ?= -pthread

TARGET := RayTracing.ex
OBJS   := main.o EBGeometryRays.o TinyBVHRays.o

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) $(OBJS) -o $@ $(LDFLAGS)

main.o: main.cpp Common.hpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

EBGeometryRays.o: EBGeometryRays.cpp Common.hpp
	$(CXX) $(CXXFLAGS) -I$(EBGEOMETRY_HOME) -c $< -o $@

TinyBVHRays.o: TinyBVHRays.cpp Common.hpp
	$(CXX) $(CXXFLAGS) -I$(TINYBVH_HOME) -c $< -o $@

.PHONY: run clean
run: $(TARGET)
	./$(TARGET)

clean:
	$(RM) $(TARGET) $(OBJS)
