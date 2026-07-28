# Benchmark: PointCloudBVH vs picoflann vs nanoflann vs kd3 (all-nearest-neighbor).
# Requires the picoflann, nanoflann, kd3 submodules:
#   git submodule update --init ../Submodules/picoflann ../Submodules/nanoflann ../Submodules/kd3
# kd3 needs C++23 (std::expected/std::span), so the whole benchmark is built with -std=c++23; it is
# compiled WITHOUT -fopenmp so kd3 (like the others) runs single-threaded, for a fair comparison.
# EBGEOMETRY_HOME defaults to the EBGeometry submodule.

EBGEOMETRY_HOME ?= ../Submodules/EBGeometry
CXX             ?= g++
CXXFLAGS        ?= -std=c++23 -O3 -march=native

INCLUDES := -I$(EBGEOMETRY_HOME) \
            -I../Submodules/picoflann \
            -I../Submodules/nanoflann/include \
            -I../Submodules/kd3/include

TARGET := NearestNeighbor.ex

$(TARGET): main.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) $< -o $@

.PHONY: run clean
run: $(TARGET)
	./$(TARGET)

clean:
	$(RM) $(TARGET)
