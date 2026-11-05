#pragma once
#include "common.hpp"

namespace navgen {
using Tri = std::array<V3, 3>;
std::vector<V4> axis_planes(const std::vector<V3>& points, const std::vector<V3>& axes);
std::vector<V3> edge_axes(const std::vector<V3>& edges, double epsilon);
std::vector<V4> bevel_planes(const std::vector<V3>& points, const std::vector<std::array<int64_t, 2>>& edges);
std::pair<std::vector<V4>, V4> triangle_kernel(const Tri& points);
}
