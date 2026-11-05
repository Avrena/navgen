#pragma once
#include "collision.hpp"

namespace navgen {
struct LadderSpec {
    std::string class_name;
    Json hammerid=nullptr;
    V3 bottom{},top{},lo{},hi{};
    double width=25.;
    int axis=-1;
};
struct LadderSources { std::vector<LadderSpec> specs; Json skipped=Json::array(); };
struct EntitySources { std::vector<std::pair<V3,std::string>> sources; LadderSources ladders; };
struct DiscoveryResult { std::vector<V3> effective; Json details; std::vector<LadderSpec> ladder_specs; };
LadderSources ladder_specs(const BSP& bsp);
EntitySources entity_sources(const BSP& bsp,const std::vector<V3>& base);
int leaf_area(const BSP& bsp,const V3& point);
DiscoveryResult discover_seeds(const BSP& bsp,Collision& collision,const std::vector<V3>& base,double slope=.7,std::size_t max_seeds=4096,std::size_t max_probes=200000,bool progress=true);
}
