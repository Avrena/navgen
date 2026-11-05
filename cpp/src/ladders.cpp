#include "ladders.hpp"
#include "areas.hpp"
#include "geometry.hpp"
#include "merge.hpp"
#include <algorithm>
#include <cmath>
#include <optional>
#include <tuple>

namespace navgen {
int ladder_direction(const V3& vector) {
    if(std::abs(vector[0])>std::abs(vector[1])) return vector[0]>0 ? 1 : 3;
    return vector[1]>0 ? 2 : 0;
}

int top_slot(int facing,const V3& delta) {
    const int d=ladder_direction(delta);
    return d==facing ? 3 : (d==(facing+2)%4 ? 0 : (d==(facing+3)%4 ? 1 : 2));
}

std::vector<ClimbLine> climb_lines(const LadderSpec& spec) {
    std::vector<ClimbLine> result;
    for(int facing=0;facing<4;++facing) {
        const auto& normal=DIRECTIONS[facing];
        if(spec.axis<0) {
            for(double offset:{0.,16.1,32.1}) result.push_back({facing,spec.bottom+normal*offset,spec.top+normal*offset});
        } else {
            const int axis=spec.axis,tangent=1-axis;
            if(std::abs(normal[axis])!=1) continue;
            const double face=normal[axis]>0 ? spec.hi[axis] : spec.lo[axis];
            const double low=spec.lo[tangent]+std::min(16.,spec.width/4),high=spec.hi[tangent]-std::min(16.,spec.width/4);
            std::vector<double> coordinates;
            for(double coordinate:{(low+high)*.5,low,high}) {
                if(std::find(coordinates.begin(),coordinates.end(),coordinate)!=coordinates.end()) continue;
                coordinates.push_back(coordinate);
                auto b=spec.bottom,t=spec.top;
                b[axis]=t[axis]=face+normal[axis]*16.1;
                b[tangent]=t[tangent]=coordinate;
                result.push_back({facing,b,t});
            }
        }
    }
    return result;
}

Json build_ladders(Json& nav,Collision& collision,const std::vector<LadderSpec>& specs,bool support_convention,double slope) {
    const auto bounds=boxes(nav);
    auto& scene=collision.scene;
    Json records=Json::array(),rejected=Json::array();
    struct Attachment { double distance; size_t index; V3 point; };
    auto attachments=[&](const V3& endpoint,bool upper,double mid) {
        std::vector<Attachment> candidates;
        for(size_t i=0;i<bounds.size();++i) {
            const auto& b=bounds[i];
            // np.clip applies the lower bound and then the upper bound.
            const double x=std::min(std::max(endpoint[0],b[0]+.01),b[2]-.01);
            const double y=std::min(std::max(endpoint[1],b[1]+.01),b[3]-.01);
            if(std::abs(x-endpoint[0])>80 || std::abs(y-endpoint[1])>80) continue;
            V3 p{x,y,height_at(b,x,y)};
            if(support_convention) {
                auto supported=support_origin(scene,p,slope);
                if(!supported.good) continue;
                p=supported.point;
            }
            if(std::abs(p[2]-endpoint[2])>64 || (p[2]>mid)!=upper) continue;
            candidates.push_back({norm3(p-endpoint),i,p});
        }
        std::stable_sort(candidates.begin(),candidates.end(),[](const Attachment& a,const Attachment& b){return std::tie(a.distance,a.index)<std::tie(b.distance,b.index);});
        if(candidates.size()>32) candidates.resize(32);
        return candidates;
    };
    struct Option { double cost; int facing; size_t bottom_index,top_index; int slot; V3 start,end,bottom_witness,top_witness; };
    for(const auto& spec:specs) {
        Json source={{"class",spec.class_name},{"hammerid",spec.hammerid}};
        const auto& bottom=spec.bottom; const auto& top=spec.top;
        const double mid=(top[2]+bottom[2])*.5;
        std::optional<Option> selected;
        size_t bottom_count=0,top_count=0,clear_count=0;
        for(const auto& line:climb_lines(spec)) {
            const auto facing=line.facing; const auto& b=line.bottom; const auto& t=line.top;
            const auto bottoms=attachments(b,false,mid),tops=attachments(t,true,mid);
            bottom_count=std::max(bottom_count,bottoms.size()); top_count=std::max(top_count,tops.size());
            for(const auto& bd:bottoms) {
                auto start=b; start[2]=std::max(bottom[2],bd.point[2])+.01;
                if(!hull_move(scene,bd.point,start) || !hull_move(scene,start,bd.point)) continue;
                for(const auto& td:tops) {
                    if(td.index==bd.index) continue;
                    V3 delta{td.point[0]-t[0],td.point[1]-t[1],0.};
                    if(norm2(delta)<.01) delta={(bounds[td.index][0]+bounds[td.index][2])*.5-t[0],(bounds[td.index][1]+bounds[td.index][3])*.5-t[1],0.};
                    const int slot=top_slot(facing,delta);
                    if(slot==3) continue;
                    auto end=t; end[2]=std::max(top[2],td.point[2])+.01;
                    if(end[2]<=start[2]+18 || !clear_move(scene,start,end)) continue;
                    ++clear_count;
                    if(!hull_move(scene,end,td.point) || !hull_move(scene,td.point,end)) continue;
                    Option option{bd.distance+td.distance+norm3(start-b)+norm3(end-t),facing,bd.index,td.index,slot,start,end,bd.point,td.point};
                    if(!selected || std::tie(option.cost,option.facing,option.bottom_index,option.top_index)<std::tie(selected->cost,selected->facing,selected->bottom_index,selected->top_index)) selected=option;
                }
            }
        }
        if(!selected) {
            source["reason"]="no clear climb with reciprocal supported top and bottom attachments";
            source["bottom_candidates"]=bottom_count; source["top_candidates"]=top_count; source["clear_climb_trials"]=clear_count;
            rejected.push_back(source); continue;
        }
        const auto& o=*selected;
        const auto ident=nav.at("ladders").size()+1;
        const auto& normal=DIRECTIONS[o.facing];
        const auto face_bottom=o.start-normal*16.1,face_top=o.end-normal*16.1;
        std::array<int64_t,5> area_ids{};
        area_ids[o.slot]=nav["areas"][o.top_index]["id"].get<int64_t>();
        area_ids[4]=nav["areas"][o.bottom_index]["id"].get<int64_t>();
        nav["ladders"].push_back({{"id",ident},{"width",std::min(spec.width,32.)},{"top",face_top},{"bottom",face_bottom},
                                  {"length",norm3(face_top-face_bottom)},{"dir",o.facing},{"areas",area_ids}});
        nav["areas"][o.bottom_index]["ladders"][0].push_back(ident);
        nav["areas"][o.top_index]["ladders"][1].push_back(ident);
        for(auto index:{o.bottom_index,o.top_index}) nav["areas"][index]["attr"]=nav["areas"][index]["attr"].get<int64_t>()|8192;
        source["id"]=ident; source["bottom_area"]=area_ids[4]; source["top_area"]=area_ids[o.slot]; source["top_slot"]=o.slot;
        source["climb_bottom"]=o.start; source["climb_top"]=o.end; source["bottom_witness"]=o.bottom_witness; source["top_witness"]=o.top_witness;
        records.push_back(source);
    }
    return {{"sources",specs.size()},{"written",records.size()},{"records",records},{"skipped",rejected},{"native_traversal_verified",false}};
}
}
