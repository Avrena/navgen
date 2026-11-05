#pragma once
#include "bsp.hpp"
#include <memory>
#include <optional>

namespace navgen {
inline constexpr double EPS=.03125;
inline constexpr V3 THIN_MIN={-.45,-.45,0.};
inline constexpr V3 THIN_MAX={.45,.45,55.};
using Trace = std::array<double,10>;
struct TraceCounts {
    // Parallel kernels have distinct worker_index slots and join before totals/reset.
    struct alignas(64) Worker { uint64_t value=0; };
    std::vector<Worker> workers=std::vector<Worker>(static_cast<size_t>(logical_cores()));
    void increment() { ++workers[worker_index].value; }
    uint64_t total() const { uint64_t result=0;for(const auto& worker:workers)result+=worker.value;return result; }
    void reset() { for(auto& worker:workers)worker.value=0; }
};
struct Scene {
    std::vector<V4> planes;
    std::vector<int64_t> pflags, first, count;
    std::vector<V3> lo,hi;
    std::vector<std::array<int64_t,6>> meta;
    std::vector<V4> front;
    std::vector<V3> blo,bhi;
    std::vector<int> escape,order;
    std::vector<std::array<int,2>> leaf;
    std::shared_ptr<TraceCounts> trace_count=std::make_shared<TraceCounts>();
    uint64_t traces() const { return trace_count->total(); }
};
struct Clip { double fraction; bool startsolid,allsolid; int64_t plane; };
Clip clip(const std::vector<V4>& planes,const V3& start,const V3& end,const V3& mins,const V3& maxs,int64_t first,int64_t count);
Trace trace(const Scene& scene,const V3& start,const V3& end,const V3& mins,const V3& maxs,int64_t mask=BRUSH_MASK,bool walk=true,bool brute=false);
std::vector<Trace> batch_trace(const Scene& scene,const std::vector<V3>& starts,const std::vector<V3>& ends,const std::vector<V3>& mins,const std::vector<V3>& maxs,int64_t mask=BRUSH_MASK,bool walk=true,bool brute=false);
int blocked_vertical_span(const Scene& scene,const V3& start,const V3& end,const V3& mins,const V3& maxs,int hit,int limit);
std::pair<std::vector<V4>,V4> triangle_planes(const Tri& points);
void build_bvh(Scene& scene);
struct SceneBuilder {
    Scene scene;
    void add(const std::vector<V4>& planes,const V3& lo,const V3& hi,int64_t contents=1,bool walkable=false,bool monster=false,bool transparent=false,bool push=false,int kind=0,const std::optional<std::vector<int64_t>>& flags=std::nullopt,const std::optional<V4>& front=std::nullopt);
    Scene pack();
};
std::pair<std::optional<bool>,bool> entity_rules(const Entity& entity);
struct Collision {
    Scene scene;
    Json counts=Json::object(),unresolved=Json::array(),content_audit=Json::object(),sampling_speculation=Json::object();
    Trace trace(const V3& start,const V3& end,const V3& mins=THIN_MIN,const V3& maxs=THIN_MAX,int64_t mask=BRUSH_MASK,bool walk=true) const;
    static Collision load(const BSP& bsp,const std::filesystem::path& game,const std::string& missing_phy="skip",bool progress=true);
};
}
