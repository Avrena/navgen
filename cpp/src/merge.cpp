#include "merge.hpp"
#include "areas.hpp"
#include "pyset.hpp"
#include <algorithm>
#include <chrono>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <tuple>

namespace navgen {
namespace {
using Id=int64_t;
using Incoming=std::pair<Id,int>;
using Directed=std::tuple<Id,Id,int>;
constexpr int64_t BLOCKED_FLAGS=8192|256|INT64_C(0x80000000);
V3 center(const Box& b) { return {(b[0]+b[2])/2,(b[1]+b[3])/2,(b[4]+b[6])/2}; }
void increment(Json& counts,const char* key) { counts[key]=counts.value(key,int64_t{0})+1; }
std::string get(const Entity& e,const std::string& key,const std::string& fallback="") {
    const auto it=e.find(key); return it==e.end() ? fallback : it->second;
}
uint32_t u32(const Bytes& data,size_t offset) {
    if(offset+4>data.size()) throw Error("BSP node or leaf outside lump");
    return static_cast<uint32_t>(data[offset])|(static_cast<uint32_t>(data[offset+1])<<8)|(static_cast<uint32_t>(data[offset+2])<<16)|(static_cast<uint32_t>(data[offset+3])<<24);
}
PySet<Id> all_outgoing(const std::array<PySet<Id>,4>& outgoing) {
    // set.union(*outgoing) makes one copy, followed by in-place updates.
    auto result=outgoing[0].copy();
    for(int d=1;d<4;++d) result.update(outgoing[d]);
    return result;
}
std::vector<V3> corners_vector(const Box& b) {
    const auto c=corner_points(b); return {c.begin(),c.end()};
}
bool truthy(const Json& value) {
    if(value.is_null()) return false;
    if(value.is_boolean()) return value.get<bool>();
    if(value.is_number()) return value.get<double>()!=0.;
    return !value.empty();
}
bool witness(Scene& scene,const Box& a,const Box& b,int d,double slope) {
    const auto status=portal_probe(scene,a,b,d,slope,true).code;
    return status==1 || status==2 || status==3;
}
}

std::vector<Box> boxes(const Json& nav) {
    std::vector<Box> result; result.reserve(nav.at("areas").size());
    for(const auto& a:nav.at("areas")) result.push_back({a.at("nw")[0].get<double>(),a.at("nw")[1].get<double>(),a.at("se")[0].get<double>(),a.at("se")[1].get<double>(),
        a.at("nw")[2].get<double>(),a.at("ne_z").get<double>(),a.at("se")[2].get<double>(),a.at("sw_z").get<double>()});
    return result;
}

std::pair<V3,double> native_portal(const Box& a,const Box& b,int d) {
    const int axis=d==0 || d==2 ? 1 : 0,tangent=1-axis;
    const double lo=std::min(a[tangent+2],std::max(a[tangent],std::max(a[tangent],b[tangent])));
    const double hi=std::min(a[tangent+2],std::max(a[tangent],std::min(a[tangent+2],b[tangent+2])));
    V3 p{}; p[axis]=d==0 || d==3 ? a[axis] : a[axis+2]; p[tangent]=(lo+hi)*.5;
    p[2]=height_at(a,p[0],p[1]);
    return {p,(hi-lo)*.5};
}

std::optional<Box> union_box(const Box& first,const Box& second,double tolerance,double max_size) {
    const Box* a=&first; const Box* b=&second;
    Box result{};
    if((*a)[1]==(*b)[1] && (*a)[3]==(*b)[3] && ((*a)[2]==(*b)[0] || (*b)[2]==(*a)[0])) {
        if((*b)[2]==(*a)[0]) std::swap(a,b);
        if(std::max(std::abs((*a)[5]-(*b)[4]),std::abs((*a)[6]-(*b)[7]))>std::min(.5,tolerance)) return std::nullopt;
        result={(*a)[0],(*a)[1],(*b)[2],(*a)[3],(*a)[4],(*b)[5],(*b)[6],(*a)[7]};
    } else if((*a)[0]==(*b)[0] && (*a)[2]==(*b)[2] && ((*a)[3]==(*b)[1] || (*b)[3]==(*a)[1])) {
        if((*b)[3]==(*a)[1]) std::swap(a,b);
        if(std::max(std::abs((*a)[7]-(*b)[4]),std::abs((*a)[6]-(*b)[5]))>std::min(.5,tolerance)) return std::nullopt;
        result={(*a)[0],(*a)[1],(*a)[2],(*b)[3],(*a)[4],(*a)[5],(*b)[6],(*b)[7]};
    } else return std::nullopt;
    const double width=result[2]-result[0],height=result[3]-result[1];
    if(std::max(width,height)>max_size || width*height>300000) return std::nullopt;
    const auto na=surface_normals(*a),nb=surface_normals(*b);
    double minimum=std::numeric_limits<double>::infinity();
    for(const auto& x:na) for(const auto& y:nb) minimum=std::min(minimum,dot3(x,y));
    if(minimum<.99) return std::nullopt;
    return result;
}

bool fits(const Box& box,const std::vector<V3>& points,double tolerance) {
    double maximum=-std::numeric_limits<double>::infinity();
    for(const auto& point:points) {
        const double u=(point[0]-box[0])/(box[2]-box[0]),v=(point[1]-box[1])/(box[3]-box[1]);
        const double z=(box[4]+u*(box[5]-box[4]))*(1-v)+(box[7]+u*(box[6]-box[7]))*v;
        maximum=std::max(maximum,std::abs(z-point[2]));
    }
    return maximum<=tolerance+1e-8;
}

std::array<bool,3> edge_class(const Box& a,const Box& b,int d) {
    const auto p=native_portal(a,b,d).first,q=native_portal(b,a,(d+2)%4).first;
    const float dz=static_cast<float>(q[2]-p[2]);
    return {dz>=18.,dz>=58.,dz<-200.};
}

Json merge_nav(Json& nav,const WaterProbe& water,const std::vector<int64_t>& locked_ids,double tolerance,double max_size,Scene* scene,double slope) {
    if(!std::isfinite(tolerance) || tolerance<0) throw Error("Merge tolerance must be finite and nonnegative");
    if(!std::isfinite(max_size) || max_size<=0) throw Error("Merge size must be finite and positive");
    const double start=now();
    const auto source_area_count=nav.at("areas").size();
    std::map<Id,Json> areas;
    std::map<Id,Box> bounds;
    std::map<Id,std::vector<V3>> points;
    std::map<Id,std::array<PySet<Id>,4>> outgoing;
    std::map<Id,PySet<Incoming>> incoming;
    std::map<Id,std::vector<Id>> groups;
    std::map<Id,Id> mapping;
    std::map<Id,std::optional<bool>> wet;
    std::vector<Id> initial_order;
    const auto initial_bounds=boxes(nav);
    size_t area_index=0;
    for(const auto& area:nav.at("areas")) {
        const auto i=area.at("id").get<Id>();
        initial_order.push_back(i); areas[i]=area; bounds[i]=initial_bounds[area_index++]; points[i]=corners_vector(bounds.at(i));
        auto& links=outgoing[i];
        for(int d=0;d<4;++d) for(const auto& target:area.at("links")[d]) links[d].insert(target.get<Id>());
        incoming.emplace(i,PySet<Incoming>{}); groups[i]={i}; mapping[i]=i;
    }
    for(const auto i:initial_order) for(int d=0;d<4;++d) for(const auto j:outgoing.at(i)[d]) incoming.at(j).insert({i,d});
    PySet<Id> locked(locked_ids);
    Json rejected=Json::object();
    size_t merged=0;
    for(const auto i:initial_order) {
        const auto& a=areas.at(i);
        bool ladder_attached=false; for(const auto& list:a.at("ladders")) ladder_attached|=truthy(list);
        if((a.at("attr").get<int64_t>()&BLOCKED_FLAGS) || ladder_attached || truthy(a.value("blocked",Json(false))) || truthy(a.value("damaging",Json(false)))) locked.insert(i);
        if(water) wet[i]=water(center(bounds.at(i)));
        else {
            const auto underwater=a.value("underwater",Json(false));
            wet[i]=underwater.is_null() ? std::nullopt : std::optional<bool>(truthy(underwater));
        }
        if(!wet.at(i).has_value()) locked.insert(i);
    }
    std::vector<Id> sorted_ids=initial_order; std::sort(sorted_ids.begin(),sorted_ids.end());
    std::deque<Id> queue(sorted_ids.begin(),sorted_ids.end());
    PySet<Id> queued(sorted_ids);
    while(!queue.empty()) {
        const auto i=queue.front(); queue.pop_front(); queued.erase(i);
        if(!areas.contains(i) || locked.contains(i)) continue;
        auto& a=areas.at(i);
        auto candidates=all_outgoing(outgoing.at(i)).values(); std::sort(candidates.begin(),candidates.end());
        for(const auto j:candidates) {
            if(j==i || !areas.contains(j) || locked.contains(j)) continue;
            const auto& b=areas.at(j);
            if(a.at("attr")!=b.at("attr") || a.at("place")!=b.at("place") || wet.at(i)!=wet.at(j)) continue;
            bool reciprocal=false; for(const auto& list:outgoing.at(j)) reciprocal|=list.contains(i);
            if(!reciprocal || a.at("light")!=b.at("light")) continue;
            const auto maybe_box=union_box(bounds.at(i),bounds.at(j),tolerance,max_size);
            if(!maybe_box) continue;
            const auto box=*maybe_box,ai=bounds.at(i),bj=bounds.at(j);
            const int direction=ai[2]==bj[0] ? 1 : (bj[2]==ai[0] ? 3 : (ai[3]==bj[1] ? 2 : 0));
            if(!outgoing.at(i)[direction].contains(j) || !outgoing.at(j)[(direction+2)%4].contains(i)) continue;
            if(scene && (!witness(*scene,ai,bj,direction,slope) || !witness(*scene,bj,ai,(direction+2)%4,slope))) {
                increment(rejected,"shared_edge_probe"); continue;
            }
            auto old=points.at(i); old.insert(old.end(),points.at(j).begin(),points.at(j).end());
            if(!fits(box,old,tolerance)) { increment(rejected,"original_corner_error"); continue; }
            const PySet<Id> removed{i,j};
            bool compatible=true;
            for(const auto oldid:{i,j}) {
                for(int d=0;d<4;++d) {
                    for(const auto k:outgoing.at(oldid)[d]-removed)
                        if(edge_class(bounds.at(oldid),bounds.at(k),d)!=edge_class(box,bounds.at(k),d)) { compatible=false; break; }
                    if(!compatible) break;
                }
                if(!compatible) break;
                for(const auto [k,d]:incoming.at(oldid))
                    if(k!=i && k!=j && edge_class(bounds.at(k),bounds.at(oldid),d)!=edge_class(bounds.at(k),box,d)) { compatible=false; break; }
                if(!compatible) break;
            }
            if(!compatible) { increment(rejected,"native_height_class"); continue; }
            if(scene) {
                for(const auto oldid:{i,j}) {
                    for(int d=0;d<4;++d) {
                        for(const auto k:outgoing.at(oldid)[d]-removed)
                            if(!witness(*scene,box,bounds.at(k),d,slope)) { compatible=false; break; }
                        if(!compatible) break;
                    }
                    if(!compatible) break;
                    for(const auto [k,d]:incoming.at(oldid))
                        if(k!=i && k!=j && !witness(*scene,bounds.at(k),box,d,slope)) { compatible=false; break; }
                    if(!compatible) break;
                }
                if(!compatible) { increment(rejected,"frontier_probe"); continue; }
            }
            if(water && water(center(box))!=wet.at(i)) { increment(rejected,"merged_water"); continue; }
            std::array<PySet<Id>,4> newout;
            for(int d=0;d<4;++d) newout[d]=(outgoing.at(i)[d]|outgoing.at(j)[d])-removed;
            PySet<Incoming> newin;
            for(const auto [k,d]:incoming.at(i)|incoming.at(j)) if(k!=i && k!=j) newin.insert({k,d});
            for(const auto oldid:{i,j}) {
                for(int d=0;d<4;++d) for(const auto k:outgoing.at(oldid)[d]) incoming.at(k).erase({oldid,d});
                for(const auto [k,d]:incoming.at(oldid).values()) outgoing.at(k)[d].erase(oldid);
            }
            outgoing[i]=std::move(newout); incoming[i]=std::move(newin);
            for(int d=0;d<4;++d) for(const auto k:outgoing.at(i)[d]) incoming.at(k).insert({i,d});
            for(const auto [k,d]:incoming.at(i)) outgoing.at(k)[d].insert(i);
            bounds[i]=box;
            auto minz=old.front()[2],maxz=minz;
            for(const auto& p:old) { minz=std::min(minz,p[2]); maxz=std::max(maxz,p[2]); }
            points[i]=maxz-minz==0 ? corners_vector(box) : std::move(old);
            for(const auto k:groups.at(j)) mapping[k]=i;
            groups.at(i).insert(groups.at(i).end(),groups.at(j).begin(),groups.at(j).end());
            groups.erase(j); areas.erase(j); outgoing.erase(j); incoming.erase(j); bounds.erase(j); points.erase(j); wet.erase(j);
            a["nw"]=V3{box[0],box[1],box[4]}; a["se"]=V3{box[2],box[3],box[6]}; a["ne_z"]=box[5]; a["sw_z"]=box[7];
            a["hiding"]=Json::array(); a["encounters"]=Json::array(); a["visible"]=Json::array(); a["inherit_vis"]=0;
            ++merged;
            auto affected=PySet<Id>{i}|all_outgoing(outgoing.at(i));
            PySet<Id> incoming_ids; for(const auto [k,d]:incoming.at(i)) { static_cast<void>(d); incoming_ids.insert(k); }
            affected=affected|incoming_ids;
            auto affected_ids=affected.values(); std::sort(affected_ids.begin(),affected_ids.end());
            for(const auto k:affected_ids) if(!queued.contains(k)) { queue.push_back(k); queued.insert(k); }
            break;
        }
    }
    PySet<Directed> expected,actual;
    for(const auto& a:nav.at("areas")) for(int d=0;d<4;++d) for(const auto& target:a.at("links")[d]) {
        const auto i=mapping.at(a.at("id").get<Id>()),j=mapping.at(target.get<Id>());
        if(i!=j) expected.insert({i,j,d});
    }
    for(const auto& [i,links]:outgoing) for(int d=0;d<4;++d) for(const auto j:links[d]) actual.insert({i,j,d});
    if(!(expected==actual)) throw Error("Merge changed the directed graph contraction");
    size_t changed=0;
    for(const auto [i,j,d]:actual) { static_cast<void>(d); changed+=groups.at(i).size()>1 || groups.at(j).size()>1; }
    Json link_validation={{"directed_contraction_verified",true},{"changed_links",changed},
        {"changed_links_geometrically_verified",scene ? changed : 0},{"inherited_unchanged_links",actual.size()-changed},
        {"policy","Changed frontier links require SDK movement/drop/portal witnesses; unchanged SDK exceptions are retained."}};
    nav["areas"]=Json::array();
    for(auto& [i,a]:areas) {
        Json links=Json::array();
        for(const auto& ls:outgoing.at(i)) { auto ids=ls.values(); std::sort(ids.begin(),ids.end()); links.push_back(ids); }
        a["links"]=std::move(links); nav["areas"].push_back(std::move(a));
    }
    Json mapping_json=Json::object(),groups_json=Json::object();
    for(const auto& [k,v]:mapping) mapping_json[std::to_string(k)]=v;
    for(const auto& [k,v]:groups) groups_json[std::to_string(k)]=v;
    return {{"source_areas",source_area_count},{"merged_areas",areas.size()},{"merges",merged},{"locked",locked.size()},
        {"tolerance",tolerance},{"max_size",max_size},{"rejected",rejected},{"seconds",now()-start},
        {"mapping",mapping_json},{"groups",groups_json},{"link_validation",link_validation}};
}

std::optional<double> ground_height(const Scene& scene,const V3& point) {
    auto start=point,end=point; start[2]+=35.501; end[2]-=10000.;
    constexpr V3 zero{};
    while(end[2]-point[2]<100.) {
        const auto hit=trace(scene,start,end,zero,zero,NPC_MASK,true);
        const double z=start[2]+(end[2]-start[2])*hit[0];
        if(!hit[4] && (hit[0]==1 || start[2]-z>=35.5)) return z;
        end[2]=hit[4] ? start[2] : z; start[2]=end[2]+35.501;
    }
    return std::nullopt;
}

uint32_t world_contents(const BSP& bsp,const V3& point) {
    int64_t node=static_cast<int64_t>(bsp.models.at(0)[9]);
    while(node>=0) {
        const size_t offset=static_cast<size_t>(node)*32;
        const int32_t plane=static_cast<int32_t>(u32(bsp.nodes,offset));
        const auto& p=bsp.planes.at(static_cast<size_t>(plane));
        node=static_cast<int32_t>(u32(bsp.nodes,offset+(dot3(V3{p[0],p[1],p[2]},point)>p[3] ? 4 : 8)));
    }
    return u32(bsp.leaves,static_cast<size_t>(-1-node)*bsp.leaf_stride);
}

MergeEnvironment merge_environment(const BSP& bsp,Collision& collision,const Json& nav) {
    auto cache=std::make_shared<std::map<V3,std::optional<bool>>>();
    WaterProbe water=[cache,&bsp,&collision](const V3& c)->std::optional<bool> {
        auto it=cache->find(c); if(it!=cache->end()) return it->second;
        const auto z=ground_height(collision.scene,c);
        std::optional<bool> result;
        if(z) { auto p=c; p[2]=*z+1.; result=bool(world_contents(bsp,p)&0x4030); }
        cache->emplace(c,result); return result;
    };
    PySet<Id> locked;
    Json records=Json::array();
    const auto bounds=boxes(nav);
    for(const auto& entity:bsp.entities) {
        const auto cls=get(entity,"classname");
        if(cls!="trigger_hurt" && cls!="func_nav_blocker" && cls!="func_nav_avoid" && cls!="func_nav_cost" && cls!="func_water" && cls!="func_water_analog") continue;
        const auto model=get(entity,"model");
        if(!model.starts_with('*')) continue;
        auto index=python_integer(model.substr(1)).clamped();
        if(index<0)index+=static_cast<int64_t>(bsp.models.size());
        if(index<0||static_cast<uint64_t>(index)>=bsp.models.size())throw Error("list index out of range","IndexError");
        const auto& record=bsp.models.at(static_cast<size_t>(index));
        auto points=box_vertices({record[0],record[1],record[2]},{record[3],record[4],record[5]});
        const auto rotation_matrix=rotation(entity_angles(entity));
        const auto origin=vector(get(entity,"origin"));
        for(auto& p:points) p={dot3(p,rotation_matrix[0])+origin[0],dot3(p,rotation_matrix[1])+origin[1],dot3(p,rotation_matrix[2])+origin[2]};
        auto lo=points.front(),hi=lo;
        for(const auto& p:points) for(int j=0;j<3;++j) { lo[j]=std::min(lo[j],p[j]); hi[j]=std::max(hi[j],p[j]); }
        std::vector<Id> ids;
        for(size_t i=0;i<bounds.size();++i) {
            const auto& b=bounds[i];
            const double minz=*std::min_element(b.begin()+4,b.end()),maxz=*std::max_element(b.begin()+4,b.end());
            if(b[0]<=hi[0] && b[2]>=lo[0] && b[1]<=hi[1] && b[3]>=lo[1] && minz<=hi[2] && maxz+71>=lo[2]) {
                const auto id=nav.at("areas")[i].at("id").get<Id>(); ids.push_back(id); locked.insert(id);
            }
        }
        records.push_back({{"class",cls},{"hammerid",entity.find("hammerid")!=entity.end() ? Json(entity.at("hammerid")) : Json(nullptr)},
            {"bounds",Json::array({lo,hi})},{"locked_area_ids",ids}});
    }
    Json environment={{"static_volume_sources",records},{"runtime_state_verified",false},
        {"water_policy","SDK ground probe and world BSP leaf contents; runtime water entity volumes are locked"}};
    return {std::move(water),locked.values(),std::move(environment)};
}
}
