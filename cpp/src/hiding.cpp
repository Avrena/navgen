#include "hiding.hpp"
#include <algorithm>
#include <cmath>
#include <numbers>

namespace navgen {
namespace {
constexpr uint8_t IN_COVER=1,EXPOSED=8;
constexpr int64_t JUMP=2,DONT_HIDE=0x200;
F3 float_point(const Json& p) { return {p[0].get<float>(),p[1].get<float>(),p[2].get<float>()}; }
bool has_id(const Json& links,int64_t id) {
    return std::any_of(links.begin(),links.end(),[id](const Json& value){return value.get<int64_t>()==id;});
}
}

F3 spot_position(const Json& area,int corner) {
    const auto nw=float_point(area.at("nw")),se=float_point(area.at("se"));
    const std::array<F3,4> corners{{nw,{se[0],nw[1],area.at("ne_z").get<float>()},se,{nw[0],se[1],area.at("sw_z").get<float>()}}};
    constexpr std::array<std::array<int,2>,4> signs{{{1,1},{-1,1},{-1,-1},{1,-1}}};
    const auto origin=corners.at(static_cast<size_t>(corner));
    const auto half=(se-nw)*.5f;
    const std::array<std::array<float,2>,6> offsets{{{12.5f,12.5f},{12.5f,half[1]},{half[0],12.5f},{half[0],half[1]},{1.f,1.f},{0.f,0.f}}};
    for(const auto& offset:offsets) {
        const F3 pos{origin[0]+offset[0]*signs[corner][0],origin[1]+offset[1]*signs[corner][1],origin[2]+0.f};
        if(nw[0]<=pos[0] && pos[0]<=se[0] && nw[1]<=pos[1] && pos[1]<=se[1]) return pos;
    }
    return origin;
}

std::vector<F3> area_spots(const Json& area,const std::map<int64_t,const Json*>& by_id) {
    if(area.at("attr").get<int64_t>()&(JUMP|DONT_HIDE)) return {};
    std::array<int,4> counts{};
    constexpr std::array<std::array<int,2>,4> edges{{{0,1},{1,2},{3,2},{0,3}}};
    for(int d=0;d<4;++d) {
        const int axis=d%2==0 ? 0 : 1;
        float lo=static_cast<float>(999999.9),hi=static_cast<float>(-999999.9);
        for(const auto& target:area.at("links")[d]) {
            const auto& other=*by_id.at(target.get<int64_t>());
            if((other.at("attr").get<int64_t>()&JUMP) || !has_id(other.at("links")[(d+2)%4],area.at("id").get<int64_t>())) continue;
            lo=std::min(lo,other.at("nw")[axis].get<float>());
            hi=std::max(hi,other.at("se")[axis].get<float>());
        }
        if(static_cast<float>(lo-area.at("nw")[axis].get<float>())>=20) ++counts[edges[d][0]];
        if(static_cast<float>(area.at("se")[axis].get<float>()-hi)>=20) ++counts[edges[d][1]];
    }
    std::vector<F3> spots;
    for(int corner=0;corner<4;++corner) {
        if(counts[corner]!=2) continue;
        const auto pos=spot_position(area,corner);
        bool close=false;
        if(corner) for(const auto& other:spots) {
            const auto delta=other-pos;
            float distance=-0.f;
            for(int j=0;j<3;++j) distance+=delta[j]*delta[j];
            if(distance<900) { close=true; break; }
        }
        if(!close) spots.push_back(pos);
    }
    return spots;
}

std::vector<uint8_t> cover_flags(const Scene& scene,const std::vector<F3>& positions,int threads) {
    static_cast<void>(threads);
    std::vector<uint8_t> flags(positions.size());
    constexpr V3 zero{};
    const float increment=static_cast<float>(std::numbers::pi/8);
    parallel_for(positions.size(),[&](size_t i) {
        V3 start{positions[i][0],positions[i][1],static_cast<float>(positions[i][2]+35.5f)},end=start;
        end[2]=static_cast<float>(start[2]+20.f);
        if(trace(scene,start,end,zero,zero,BRUSH_MASK,false)[0]!=1) { flags[i]=IN_COVER; return; }
        int count=0;
        float angle=0.f;
        while(angle<2*std::numbers::pi) {
            end[0]=static_cast<float>(static_cast<float>(start[0])+100.f*static_cast<float>(std::cos(static_cast<double>(angle))));
            end[1]=static_cast<float>(static_cast<float>(start[1])+100.f*static_cast<float>(std::sin(static_cast<double>(angle))));
            end[2]=static_cast<float>(start[2]+35.5f);
            if(trace(scene,start,end,zero,zero,BRUSH_MASK,false)[0]!=1) ++count;
            angle=static_cast<float>(angle+increment);
        }
        flags[i]=count>=8 ? IN_COVER : EXPOSED;
    });
    return flags;
}

Json compute_hiding_spots(Json& nav,Collision& collision,int threads) {
    std::map<int64_t,const Json*> by_id;
    for(const auto& area:nav.at("areas")) by_id.emplace(area.at("id").get<int64_t>(),&area);
    std::vector<std::vector<F3>> per_area;
    std::vector<F3> positions;
    for(const auto& area:nav.at("areas")) {
        per_area.push_back(area_spots(area,by_id));
        positions.insert(positions.end(),per_area.back().begin(),per_area.back().end());
    }
    const auto before=collision.scene.traces();
    const auto flags=cover_flags(collision.scene,positions,threads);
    size_t next_id=0,area_index=0,covered=0,exposed=0,with_spots=0;
    std::map<size_t,size_t> count_distribution;
    for(auto& area:nav["areas"]) {
        const auto& spots=per_area[area_index++];
        area["hiding"]=Json::array();
        for(const auto& pos:spots) {
            area["hiding"].push_back(Json::array({next_id,V3{pos[0],pos[1],pos[2]},flags[next_id]}));
            covered+=flags[next_id]==IN_COVER; exposed+=flags[next_id]==EXPOSED; ++next_id;
        }
        with_spots+=!spots.empty(); ++count_distribution[spots.size()];
    }
    nav["analyzed"]=1;
    Json distribution=Json::object();
    for(const auto& [count,n]:count_distribution) distribution[std::to_string(count)]=n;
    return {{"total",next_id},{"in_cover",covered},{"exposed",exposed},{"areas_with_spots",with_spots},
            {"spots_per_area",distribution},{"traces",collision.scene.traces()-before},{"analysis_mode","GMod nav_quicksave 2"}};
}
}
