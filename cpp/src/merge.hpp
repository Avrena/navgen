#pragma once
#include "collision.hpp"
#include <functional>
#include <optional>

namespace navgen {
std::vector<Box> boxes(const Json& nav);
std::pair<V3,double> native_portal(const Box& a,const Box& b,int direction);
std::optional<Box> union_box(const Box& a,const Box& b,double tolerance=.5,double max_size=800.);
bool fits(const Box& box,const std::vector<V3>& points,double tolerance);
std::array<bool,3> edge_class(const Box& a,const Box& b,int direction);
using WaterProbe=std::function<std::optional<bool>(const V3&)>;
struct MergeEnvironment { WaterProbe water; std::vector<int64_t> locked_ids; Json environment; };
Json merge_nav(Json& nav,const WaterProbe& water={},const std::vector<int64_t>& locked_ids={},double tolerance=.5,double max_size=125.,Scene* scene=nullptr,double slope=.7);
std::optional<double> ground_height(const Scene& scene,const V3& point);
uint32_t world_contents(const BSP& bsp,const V3& point);
MergeEnvironment merge_environment(const BSP& bsp,Collision& collision,const Json& nav);
}
