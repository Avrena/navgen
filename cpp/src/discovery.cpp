#include "discovery.hpp"
#include "geometry.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <map>
#include <set>
#include <stdexcept>

namespace navgen {
namespace {
std::string get(const Entity& e,const std::string& key,const std::string& fallback="") {
    const auto it=e.find(key); return it==e.end() ? fallback : it->second;
}
Json source(const LadderSpec& spec) { return {{"class",spec.class_name},{"hammerid",spec.hammerid}}; }
bool finite(const V3& p) { return std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]); }
std::pair<V3,V3> transformed_bounds(const BSP& bsp,const Entity& e,const V3& origin) {
    auto model=python_integer(get(e,"model").substr(1)).clamped();
    if(model<0)model+=static_cast<int64_t>(bsp.models.size());
    if(model<0||static_cast<uint64_t>(model)>=bsp.models.size())throw Error("list index out of range","IndexError");
    const auto& record=bsp.models.at(static_cast<size_t>(model));
    auto points=box_vertices({record[0],record[1],record[2]},{record[3],record[4],record[5]});
    const auto r=rotation(entity_angles(e));
    for(auto& p:points) p={dot3(p,r[0])+origin[0],dot3(p,r[1])+origin[1],dot3(p,r[2])+origin[2]};
    auto lo=points.front(),hi=lo;
    for(const auto& p:points) for(int j=0;j<3;++j) { lo[j]=std::min(lo[j],p[j]); hi[j]=std::max(hi[j],p[j]); }
    return {lo,hi};
}
uint32_t u32(const Bytes& b,size_t offset) {
    if(offset+4>b.size()) throw std::runtime_error("BSP node or leaf outside lump");
    return static_cast<uint32_t>(b[offset])|(static_cast<uint32_t>(b[offset+1])<<8)|(static_cast<uint32_t>(b[offset+2])<<16)|(static_cast<uint32_t>(b[offset+3])<<24);
}
void increment(Json& counts,const std::string& key) { counts[key]=counts.value(key,int64_t{0})+1; }
}

LadderSources ladder_specs(const BSP& bsp) {
    LadderSources result;
    for(const auto& entity:bsp.entities) {
        Entity e;
        for(const auto& [key,value]:entity) {
            auto lower=key;
            std::transform(lower.begin(),lower.end(),lower.begin(),[](unsigned char c){return static_cast<char>(std::tolower(c));});
            e[lower]=value;
        }
        const auto cls=get(e,"classname");
        if(cls!="info_ladder" && cls!="func_useableladder") continue;
        LadderSpec spec; spec.class_name=cls;
        if(e.find("hammerid")!=e.end()) spec.hammerid=e.at("hammerid");
        auto rejected=source(spec);
        if(python_integer(get(e,"startdisabled","0")).nonzero() || python_integer(get(e,"spawnflags","0")).nonzero()) {
            rejected["reason"]="disabled or non-default ladder flags"; result.skipped.push_back(rejected); continue;
        }
        if(!get(e,"parentname").empty()) {
            rejected["reason"]="parented ladder requires runtime transform"; result.skipped.push_back(rejected); continue;
        }
        try {
            if(cls=="info_ladder") {
                for(int j=0;j<3;++j) {
                    const std::string axis(1,"xyz"[j]);
                    spec.lo[j]=python_float(e.at("mins."+axis)); spec.hi[j]=python_float(e.at("maxs."+axis));
                }
                if(!finite(spec.lo) || !finite(spec.hi)) throw std::invalid_argument("invalid bounds");
                for(int j=0;j<3;++j) if(spec.hi[j]<=spec.lo[j]) throw std::invalid_argument("invalid bounds");
                spec.axis=spec.hi[0]-spec.lo[0]<=spec.hi[1]-spec.lo[1] ? 0 : 1;
                spec.bottom=(spec.lo+spec.hi)*.5; spec.top=spec.bottom;
                spec.bottom[2]=spec.lo[2]; spec.top[2]=spec.hi[2];
                spec.width=spec.hi[1-spec.axis]-spec.lo[1-spec.axis];
            } else {
                spec.bottom=vector(e.at("point0")); spec.top=vector(e.at("point1"));
                if(spec.bottom[2]>spec.top[2]) std::swap(spec.bottom,spec.top);
                if(!finite(spec.bottom) || !finite(spec.top) || spec.top[2]<=spec.bottom[2]) throw std::invalid_argument("invalid endpoints");
            }
            result.specs.push_back(spec);
        } catch(const std::out_of_range&) {
            rejected["reason"]="invalid ladder geometry"; result.skipped.push_back(rejected);
        } catch(const std::invalid_argument&) {
            rejected["reason"]="invalid ladder geometry"; result.skipped.push_back(rejected);
        } catch(const Error& error) {
            if(error.kind!="ValueError") throw;
            rejected["reason"]="invalid ladder geometry"; result.skipped.push_back(rejected);
        }
    }
    return result;
}

EntitySources entity_sources(const BSP& bsp,const std::vector<V3>& base) {
    EntitySources result;
    for(const auto& p:base) result.sources.emplace_back(p,"base");
    for(const auto& e:bsp.entities) {
        const auto cls=get(e,"classname"); const auto origin=vector(get(e,"origin"));
        if(cls.starts_with("info_player_") || cls=="info_ladder_dismount") result.sources.emplace_back(origin,cls);
        else if(cls=="func_door" || cls=="func_door_rotating" || cls=="prop_door_rotating" || cls=="func_button" || cls=="func_rot_button" || cls=="momentary_rot_button") {
            V3 lo{},hi{};
            if(get(e,"model").starts_with('*')) std::tie(lo,hi)=transformed_bounds(bsp,e,origin);
            else { lo=origin-V3{24.,24.,0.}; hi=origin+V3{24.,24.,64.}; }
            const auto center=(lo+hi)*.5;
            for(int axis:{0,1}) for(int sign:{-1,1}) {
                auto p=center; p[axis]=sign<0 ? lo[axis]-24 : hi[axis]+24;
                result.sources.emplace_back(p,cls);
            }
        }
    }
    result.ladders=ladder_specs(bsp);
    for(const auto& spec:result.ladders.specs) for(const auto& name:{std::string("bottom"),std::string("top")})
        for(const V3 delta:std::array<V3,4>{{{32.,0.,36.},{-32.,0.,36.},{0.,32.,36.},{0.,-32.,36.}}})
            result.sources.emplace_back((name=="bottom" ? spec.bottom : spec.top)+delta,spec.class_name+"_"+name);
    return result;
}

int leaf_area(const BSP& bsp,const V3& point) {
    int64_t node=static_cast<int64_t>(bsp.models.at(0)[9]);
    while(node>=0) {
        const size_t offset=static_cast<size_t>(node)*32;
        const int32_t plane=static_cast<int32_t>(u32(bsp.nodes,offset));
        const auto& p=bsp.planes.at(static_cast<size_t>(plane));
        node=static_cast<int32_t>(u32(bsp.nodes,offset+(dot3({p[0],p[1],p[2]},point)>p[3] ? 4 : 8)));
    }
    const size_t offset=static_cast<size_t>(-1-node)*bsp.leaf_stride+6;
    if(offset+2>bsp.leaves.size()) throw std::runtime_error("BSP leaf area outside lump");
    return (static_cast<unsigned>(bsp.leaves[offset])|(static_cast<unsigned>(bsp.leaves[offset+1])<<8))&511;
}

DiscoveryResult discover_seeds(const BSP& bsp,Collision& collision,const std::vector<V3>& base,double slope,size_t max_seeds,size_t max_probes,bool progress) {
    auto sources=entity_sources(bsp,base);
    auto& scene=collision.scene;
    std::vector<V3> accepted;
    Json records=Json::array(),counts=Json::object();
    using Key=std::array<int64_t,3>;
    std::set<Key> probes;
    std::map<Key,std::vector<V3>> buckets;
    const auto& model=bsp.models.at(0);
    const V3 lo{model[0],model[1],model[2]},hi{model[3],model[4],model[5]};
    auto key_for=[](const V3& p,double xy,double z) {
        return Key{static_cast<int64_t>(std::floor(p[0]/xy)),static_cast<int64_t>(std::floor(p[1]/xy)),static_cast<int64_t>(std::floor(p[2]/z))};
    };
    auto nearby=[&](const V3& p,double xy,double z) {
        const auto key=key_for(p,xy,z);
        for(int dx:{-1,0,1}) for(int dy:{-1,0,1}) for(int dz:{-1,0,1}) {
            const auto it=buckets.find({key[0]+dx,key[1]+dy,key[2]+dz});
            if(it!=buckets.end()) for(const auto& other:it->second)
                if(std::abs(other[2]-p[2])<=z && norm2(std::array<double,2>{other[0]-p[0],other[1]-p[1]})<=xy) return true;
        }
        return false;
    };
    for(const auto& [p,source_name]:sources.sources) {
        if(accepted.size()>=max_seeds || counts.value("probes",size_t{0})>=max_probes) continue;
        bool outside=false; for(int j=0;j<3;++j) outside|=p[j]<lo[j] || p[j]>hi[j];
        if(outside) { increment(counts,"outside_bounds"); continue; }
        const auto key=key_for(p,32.,16.);
        if(!probes.insert(key).second) continue;
        increment(counts,"probes");
        std::optional<V3> floor;
        for(double rise:{2.,0.}) {
            auto start=p,end=p; start[2]+=rise; end[2]-=3000.;
            const auto hit=trace(scene,start,end,MINS,DUCK);
            if(hit[4] || hit[0]>=1 || hit[3]<slope || (static_cast<int64_t>(hit[8])&134)) continue;
            auto point=start+(end-start)*hit[0]; point[2]+=.01;
            const auto body=trace(scene,point,point,MINS,DUCK);
            if(body[4] || body[5]) continue;
            floor=point; break;
        }
        if(!floor) { increment(counts,"rejected_floor"); continue; }
        if(nearby(*floor,24.,18.)) { increment(counts,"duplicate_floor"); continue; }
        buckets[key_for(*floor,24.,18.)].push_back(*floor);
        accepted.push_back(*floor);
        records.push_back({{"point",*floor},{"source",source_name},{"leaf_area",leaf_area(bsp,*floor)}});
    }
    auto effective=base; effective.insert(effective.end(),accepted.begin(),accepted.end());
    std::stable_sort(effective.begin(),effective.end());
    effective.erase(std::unique(effective.begin(),effective.end()),effective.end());
    Json sky=Json::array(),source_counts=Json::object(),bands=Json::object();
    for(const auto& e:bsp.entities) if(get(e,"classname")=="sky_camera") {
        const auto p=vector(get(e,"origin")); sky.push_back({{"origin",p},{"leaf_area",leaf_area(bsp,p)}});
    }
    for(const auto& r:records) {
        increment(source_counts,r.at("source").get<std::string>());
        const auto band=static_cast<int64_t>(std::nearbyint(r.at("point")[2].get<double>()/128.))*128;
        increment(bands,std::to_string(band));
    }
    Json details={{"counts",counts},{"accepted",accepted.size()},{"total_seeds",effective.size()},
        {"max_seeds",max_seeds},{"max_probes",max_probes},{"nearby_floors",false},
        {"limit_reached",accepted.size()>=max_seeds || counts.value("probes",size_t{0})>=max_probes},
        {"sources",source_counts},{"height_bands",bands},{"accepted_floors",records},{"sky_cameras",sky},
        {"ladder_sources",sources.ladders.specs.size()},{"skipped_ladder_sources",sources.ladders.skipped}};
    if(progress) std::cout<<"expanded seeds: "<<accepted.size()<<" floors from "<<counts.value("probes",size_t{0})<<" probes; limit reached="<<(details["limit_reached"].get<bool>() ? "True" : "False")<<'\n';
    return {std::move(effective),std::move(details),std::move(sources.ladders.specs)};
}
}
