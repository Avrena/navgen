#pragma once
#include "collision.hpp"
#include <array>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace navgen {
using NodeId = std::int64_t;
using NodeCorners = std::array<NodeId,4>;
using CornerFlags = std::array<std::uint8_t,4>;
inline constexpr std::array<V3,4> DIRECTIONS{{{0.,-1.,0.},{1.,0.,0.},{0.,1.,0.},{-1.,0.,0.}}};
inline constexpr std::array<V3,4> CORNERS{{{-1.,-1.,0.},{1.,-1.,0.},{1.,1.,0.},{-1.,1.,0.}}};
inline constexpr V3 CUBE_MIN{{-.5,-.5,-.5}}, CUBE_MAX{{.5,.5,.5}};
struct XYKey { std::int64_t x,y; bool operator==(const XYKey&) const = default; };
struct XYHash { std::size_t operator()(const XYKey& key) const noexcept; };
using NodeIndex = std::unordered_map<XYKey,NodeId,XYHash>;
struct SamplingArrays {
    std::vector<V3> pos,normal;
    std::vector<NodeCorners> links;
    std::vector<NodeId> parent,nextxy,owner;
    std::vector<std::uint8_t> visited,disp;
    std::vector<CornerFlags> crouch,blocked;
    std::vector<V4> ground,obsh,obsa,obsb;
    void resize(std::size_t size);
    std::size_t size() const { return pos.size(); }
};
struct SamplingResult { SamplingArrays arrays; NodeIndex index; Json speculation; };
struct AdjacentResult { bool good; V3 point; Trace result; };
struct MoveResult { bool good; V3 point; Trace result; double height,obs_start,obs_end; };
struct RectangleResult { std::vector<Box> boxes; std::vector<NodeCorners> corners; std::vector<std::int64_t> attrs; std::vector<std::uint8_t> valid; };
SamplingArrays allocate(std::size_t capacity);
V3 endpoint(const V3& start,const V3& end,const Trace& result);
std::pair<Trace,V3> floor_trace(Scene& scene,const V3& start,double limit=400.);
AdjacentResult adjacent(Scene& scene,const V3& origin,const V3& target,double slope_limit=.7);
MoveResult move_node(Scene& scene,const V3& origin,const V3& target,double slope_limit=.7);
bool overlapped(Scene& scene,const V3& point);
bool embedded_displacement(Scene& scene,const V3& point);
void crouch_node(Scene& scene,const V3& point,CornerFlags& crouch,CornerFlags& blocked,V4& ground);
void crouch_nodes(Scene& scene,SamplingArrays& arrays,int threads=1);
NodeId find_node(const NodeIndex& index,const SamplingArrays& arrays,const V3& point);
SamplingResult sample(Collision& collision,const std::vector<V3>& seeds,const std::array<V3,2>& bounds,double slope_limit=.7,int threads=1);
std::vector<std::uint8_t> closed_cells(const SamplingArrays& arrays);
bool test_rectangle(Scene& scene,const SamplingArrays& arrays,const std::vector<std::uint8_t>& valid,NodeId node,int width,int height);
RectangleResult rectangles(Scene& scene,SamplingArrays& arrays,int threads=1);
bool can_traverse(Scene& scene,const V3& start,const V3& end,double slope_limit=.7);
}
