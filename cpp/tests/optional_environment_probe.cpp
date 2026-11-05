#include "discovery.hpp"
#include "geometry.hpp"
#include "merge.hpp"
#include <cfenv>

using namespace navgen;
namespace {
Scene make_scene(const Json& value) {
    SceneBuilder builder;
    for(const auto& item:value.value("boxes",Json::array())) {
        auto lo=item.at("lo").get<V3>(),hi=item.at("hi").get<V3>();
        builder.add(convex_planes(box_vertices(lo,hi)),lo,hi,item.value("contents",1LL),item.value("walkable",false),item.value("monster",false),item.value("transparent",false),item.value("push",false),item.value("kind",0));
    }
    for(const auto& item:value.value("objects",Json::array())) {
        std::optional<std::vector<int64_t>> flags;
        if(item.contains("flags")) flags=item["flags"].get<std::vector<int64_t>>();
        std::optional<V4> front;
        if(item.contains("front")) front=item["front"].get<V4>();
        builder.add(item.at("planes").get<std::vector<V4>>(),item.at("lo").get<V3>(),item.at("hi").get<V3>(),item.value("contents",1LL),item.value("walkable",false),item.value("monster",false),item.value("transparent",false),item.value("push",false),item.value("kind",0),flags,front);
    }
    return builder.pack();
}
Json specs_json(const std::vector<LadderSpec>& specs) {
    Json list=Json::array();
    for(const auto& spec:specs) {
        Json item={{"class",spec.class_name},{"hammerid",spec.hammerid},{"bottom",spec.bottom},{"top",spec.top},{"width",spec.width},{"axis",spec.axis<0?Json(nullptr):Json(spec.axis)}};
        if(spec.axis>=0) {item["lo"]=spec.lo;item["hi"]=spec.hi;}
        list.push_back(item);
    }
    return list;
}
}
int main() {
    std::fesetround(FE_TONEAREST);
    try {
        Json request;std::cin>>request;
        const auto op=request.at("op").get<std::string>();
        Collision collision;collision.scene=make_scene(request.value("scene",Json::object()));
        Json result;
        if(op=="movement") {
            result=Json::array();
            for(const auto& move:request.at("moves")) {
                const auto start=move.at("start").get<V3>(),end=move.at("end").get<V3>();
                auto floor=centre_floor(collision.scene,start,request.value("slope",.7));
                result.push_back({{"clear",clear_move(collision.scene,start,end,move.value("height",55.))},
                    {"move",hull_move(collision.scene,start,end,move.value("jump",64.),move.value("drop",400.))},
                    {"floor",Json::array({floor.good,std::isnan(floor.z)?Json(nullptr):Json(floor.z),floor.normal})}});
            }
        } else {
            BSP bsp(request.at("path").get<std::string>());
            if(op=="sources") {
                auto sources=entity_sources(bsp,request.at("seeds").get<std::vector<V3>>());
                result={{"sources",sources.sources},{"specs",specs_json(sources.ladders.specs)},{"skipped",sources.ladders.skipped}};
            } else if(op=="discovery") {
                auto discovered=discover_seeds(bsp,collision,request.at("seeds").get<std::vector<V3>>(),request.value("slope",.7),request.value("max_seeds",size_t{4096}),request.value("max_probes",size_t{200000}),false);
                result={{"effective",discovered.effective},{"details",discovered.details},{"specs",specs_json(discovered.ladder_specs)}};
            } else if(op=="environment") {
                auto nav=request.at("nav");auto environment=merge_environment(bsp,collision,nav);
                Json water=Json::array();
                for(const auto& value:request.value("points",Json::array())) {
                    auto point=value.get<V3>();auto wet=environment.water(point);
                    water.push_back({{"water",wet?Json(*wet):Json(nullptr)},{"contents",world_contents(bsp,point)},{"leaf_area",leaf_area(bsp,point)}});
                }
                auto stats=merge_nav(nav,environment.water,environment.locked_ids,.5,125.,&collision.scene,.7);stats.erase("seconds");
                result={{"environment",environment.environment},{"locked_ids",environment.locked_ids},{"water",water},{"nav",nav},{"stats",stats},{"traces",collision.scene.traces()}};
            } else throw Error("Unknown optional probe operation");
        }
        std::cout<<result.dump()<<'\n';return 0;
    } catch(const std::exception& error) {
        std::cerr<<error.what()<<'\n';return 1;
    }
}
