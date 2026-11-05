#pragma once
#include "sampling.hpp"
#include <optional>
namespace navgen {
inline constexpr std::int64_t CROUCH=1,JUMP=2,STAIRS=4096,NO_MERGE=8192,OBSTACLE_TOP=16384;
using AreaCorners = std::array<V3,4>;
using PortalPair = std::array<V3,2>;
struct PortalProbe { int code; V3 p,q; };
inline fs::path areas_dump_directory;
struct AreasResult { Json nav,cleanup,link_audit,timings; };
double height_at(const Box& box,double x,double y);
AreaCorners corner_points(const Box& box);
std::array<V3,2> surface_normals(const Box& box);
std::optional<PortalPair> portal(const Box& box,const Box& other,int direction);
PortalProbe portal_probe(Scene& scene,const Box& box,const Box& other,int direction,double slope_limit=.7,bool alternatives=false);
bool jump_down(Scene& scene,const V3& start,const V3& end);
int stair_line(Scene& scene,const V3& start,const V3& end,double slope_limit=.7);
AreasResult build_areas(Collision& collision,SamplingArrays& arrays,const NodeIndex& index,std::uint64_t bsp_size,double slope_limit=.7,int threads=1);
}
