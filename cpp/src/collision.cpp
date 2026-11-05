#include "collision.hpp"
#include <functional>
#include <iomanip>
#include <sstream>

namespace navgen {
Clip clip(const std::vector<V4>& planes,const V3& start,const V3& end,const V3& mins,const V3& maxs,int64_t first,int64_t count){
    double enter=-1e30,leave=1e30;int64_t hitplane=-1;bool startout=false,endout=false;
    for(int64_t k=first;k<first+count;k++){
        const auto& p=planes[static_cast<size_t>(k)];double d1=-p[3],d2=-p[3];
        for(int axis=0;axis<3;axis++){double n=p[axis],offset=n>=0?mins[axis]:maxs[axis];d1+=n*(start[axis]+offset);d2+=n*(end[axis]+offset);}
        if(d1>0)startout=true;if(d2>0)endout=true;
        if(d1>0&&d2>=d1)return {1.,false,false,-1};if(d1<=0&&d2<=0)continue;
        if(d1>d2){double f=(d1-EPS)/(d1-d2);if(f>enter){enter=f;hitplane=k;}}
        else leave=std::min(leave,(d1+EPS)/(d1-d2));
    }
    if(!startout)return {endout?1.:0.,true,!endout,-1};
    if(enter<=leave+1e-12&&enter>-1&&enter<1)return {std::max(0.,enter),false,false,hitplane};
    return {1.,false,false,-1};
}
Trace trace(const Scene& s,const V3& start,const V3& end,const V3& mins,const V3& maxs,int64_t mask,bool walk,bool brute){
    s.trace_count->increment();
    Trace result={1.,0.,0.,0.,0.,0.,0.,0.,0.,-1.};V3 qlo,qhi;
    for(int a=0;a<3;a++){qlo[a]=(std::min(start[a],end[a])+mins[a])-EPS;qhi[a]=(std::max(start[a],end[a])+maxs[a])+EPS;}
    size_t node=0,limit=brute?s.order.size():s.blo.size();
    while(node<limit){
        int begin,finish;
        if(brute){begin=static_cast<int>(node);finish=begin+1;node++;}
        else{
            if(s.blo[node][0]>qhi[0]||s.blo[node][1]>qhi[1]||s.blo[node][2]>qhi[2]||s.bhi[node][0]<qlo[0]||s.bhi[node][1]<qlo[1]||s.bhi[node][2]<qlo[2]){node=static_cast<size_t>(s.escape[node]);continue;}
            begin=s.leaf[node][0];finish=s.leaf[node][1];node++;if(begin<0)continue;
        }
        for(int ii=begin;ii<finish;ii++){
            int bid=s.order[static_cast<size_t>(ii)];const auto& meta=s.meta[bid];
            if(!(meta[0]&mask))continue;if(walk&&meta[1])continue;if(meta[2]&&!(mask&0x2000000))continue;if(meta[3]&&!(mask&2))continue;if(meta[4]&&!(mask&0x4000))continue;
            if(s.lo[bid][0]>qhi[0]||s.lo[bid][1]>qhi[1]||s.lo[bid][2]>qhi[2]||s.hi[bid][0]<qlo[0]||s.hi[bid][1]<qlo[1]||s.hi[bid][2]<qlo[2])continue;
            bool isdisp=meta[5]==2;
            if(isdisp){double motion=0.,back=-s.front[bid][3];for(int a=0;a<3;a++){double n=s.front[bid][a];motion+=n*(end[a]-start[a]);back+=n*(start[a]+(n>=0?maxs[a]:mins[a]));}if(motion>1e-9||back<-EPS)continue;}
            auto r=clip(s.planes,start,end,mins,maxs,s.first[bid],s.count[bid]);result[4]=result[4]||r.startsolid;result[5]=result[5]||r.allsolid;
            if(r.fraction<result[0]){
                result[0]=r.fraction;result[6]=static_cast<double>(meta[0]);result[7]=isdisp;result[9]=bid;
                if(r.plane>=0){for(int a=0;a<3;a++)result[a+1]=s.planes[static_cast<size_t>(r.plane)][a];result[8]=static_cast<double>(s.pflags[static_cast<size_t>(r.plane)]);}
                else if(isdisp)for(int a=0;a<3;a++)result[a+1]=s.front[bid][a];
            }
            if(result[0]==0.&&result[4]&&result[5])return result;
        }
    }
    return result;
}
std::vector<Trace> batch_trace(const Scene& s,const std::vector<V3>& starts,const std::vector<V3>& ends,const std::vector<V3>& mins,const std::vector<V3>& maxs,int64_t mask,bool walk,bool brute){
    std::vector<Trace> out(starts.size());parallel_for(starts.size(),[&](size_t i){out[i]=trace(s,starts[i],ends[i],mins[i],maxs[i],mask,walk,brute);});return out;
}
int blocked_vertical_span(const Scene& s,const V3& start,const V3& end,const V3& mins,const V3& maxs,int hit,int limit){
    if(hit<0||s.meta[hit][5]==2)return 0;double enter=0.,leave=1.;
    for(int64_t k=s.first[hit];k<s.first[hit]+s.count[hit];k++){
        const auto& p=s.planes[static_cast<size_t>(k)];double a=-p[3],b=a;
        for(int axis=0;axis<3;axis++){double n=p[axis],offset=n>=0?mins[axis]:maxs[axis];a+=n*(start[axis]+offset);b+=n*(end[axis]+offset);}
        if(a>0&&b>=a)return 0;if(a<=0&&b<=0)continue;if(a>b)enter=std::max(enter,a/(a-b));else leave=std::min(leave,a/(a-b));
    }
    if(leave-enter<1e-8)return 0;V3 witness=start+(end-start)*((enter+leave)*.5);
    double span=std::min(static_cast<double>(limit),s.hi[hit][2]-(std::min(start[2],end[2])+mins[2]-EPS)-1e-7);
    for(int64_t k=s.first[hit];k<s.first[hit]+s.count[hit];k++){
        const auto& p=s.planes[static_cast<size_t>(k)];double distance=-p[3];for(int axis=0;axis<3;axis++){double n=p[axis],offset=n>=0?mins[axis]:maxs[axis];distance+=n*(witness[axis]+offset);}
        if(distance>=-EPS-1e-7)return 0;if(p[2]>0)span=std::min(span,(-distance-EPS-1e-7)/p[2]);
    }
    int result=std::max(0,static_cast<int>(std::floor(span)));if(result==0)return 0;if(std::min(start[2]+result,end[2]+result)+mins[2]-EPS>s.hi[hit][2])return 0;witness[2]+=result;
    for(int64_t k=s.first[hit];k<s.first[hit]+s.count[hit];k++){
        const auto& p=s.planes[static_cast<size_t>(k)];double distance=-p[3];for(int axis=0;axis<3;axis++){double n=p[axis],offset=n>=0?mins[axis]:maxs[axis];distance+=n*(witness[axis]+offset);}if(distance>=-EPS-1e-7)return 0;
    }
    return result;
}
std::pair<std::vector<V4>,V4> triangle_planes(const Tri& p){return triangle_kernel(p);}
void build_bvh(Scene& s){
    s.blo.clear();s.bhi.clear();s.escape.clear();s.leaf.clear();s.order.clear();std::vector<V3> centers(s.lo.size());for(size_t i=0;i<centers.size();i++)centers[i]=(s.lo[i]+s.hi[i])*.5;
    std::function<void(std::vector<int>)> build=[&](std::vector<int> ids){
        const int index=static_cast<int>(s.blo.size());V3 lo=s.lo[ids.front()],hi=s.hi[ids.front()];
        for(int id:ids)for(int a=0;a<3;a++){lo[a]=std::min(lo[a],s.lo[id][a]);hi[a]=std::max(hi[a],s.hi[id][a]);}
        s.blo.push_back(lo);s.bhi.push_back(hi);s.escape.push_back(0);s.leaf.push_back({-1,-1});
        if(ids.size()<=8){int begin=static_cast<int>(s.order.size());s.order.insert(s.order.end(),ids.begin(),ids.end());s.leaf[index]={begin,static_cast<int>(s.order.size())};}
        else{V3 minc=centers[ids.front()],maxc=minc;for(int id:ids)for(int a=0;a<3;a++){minc[a]=std::min(minc[a],centers[id][a]);maxc[a]=std::max(maxc[a],centers[id][a]);}int axis=0;for(int a=1;a<3;a++)if(maxc[a]-minc[a]>maxc[axis]-minc[axis])axis=a;
            std::stable_sort(ids.begin(),ids.end(),[&](int a,int b){return centers[a][axis]<centers[b][axis];});size_t mid=ids.size()/2;build(std::vector<int>(ids.begin(),ids.begin()+static_cast<ptrdiff_t>(mid)));build(std::vector<int>(ids.begin()+static_cast<ptrdiff_t>(mid),ids.end()));}
        s.escape[index]=static_cast<int>(s.blo.size());
    };
    if(!s.lo.empty()){std::vector<int> ids(s.lo.size());std::iota(ids.begin(),ids.end(),0);build(std::move(ids));}
}
void SceneBuilder::add(const std::vector<V4>& p,const V3& lo,const V3& hi,int64_t contents,bool walkable,bool monster,bool transparent,bool push,int kind,const std::optional<std::vector<int64_t>>& flags,const std::optional<V4>& front){
    if(p.empty())return;for(const auto& row:p)for(double v:row)if(!std::isfinite(v))throw Error("Non-finite collision plane");
    scene.first.push_back(static_cast<int64_t>(scene.planes.size()));scene.count.push_back(static_cast<int64_t>(p.size()));scene.planes.insert(scene.planes.end(),p.begin(),p.end());
    if(flags)scene.pflags.insert(scene.pflags.end(),flags->begin(),flags->end());else scene.pflags.insert(scene.pflags.end(),p.size(),0);
    scene.lo.push_back(lo);scene.hi.push_back(hi);scene.meta.push_back({contents,walkable,monster,transparent,push,kind});scene.front.push_back(front.value_or(V4{}));
}
Scene SceneBuilder::pack(){build_bvh(scene);return std::move(scene);}
namespace {
std::string get(const Entity& e,const std::string& k,const std::string& fallback=""){auto it=e.find(k);return it==e.end()?fallback:it->second;}
std::string lowercase(std::string s){return unicode_lower(s);}
std::pair<V3,V3> bounds(const std::vector<V3>& points){V3 lo=points.front(),hi=lo;for(const auto& p:points)for(int a=0;a<3;a++){lo[a]=std::min(lo[a],p[a]);hi[a]=std::max(hi[a],p[a]);}return {lo,hi};}
V3 rotated(const V3& p,const M3& rot){return {dot3(p,rot[0]),dot3(p,rot[1]),dot3(p,rot[2])};}
float little_float(const Bytes& bytes,size_t off){if(off+4>bytes.size())throw Error("unpack_from requires a buffer of at least "+std::to_string(off+4)+" bytes","error");uint32_t v=uint32_t(bytes[off])|(uint32_t(bytes[off+1])<<8)|(uint32_t(bytes[off+2])<<16)|(uint32_t(bytes[off+3])<<24);return std::bit_cast<float>(v);}
std::string seconds(double begin){std::ostringstream out;out<<std::fixed<<std::setprecision(2)<<now()-begin;return out.str();}
}
std::pair<std::optional<bool>,bool> entity_rules(const Entity& input){
    Entity e;for(const auto& [k,v]:input)e[lowercase(k)]=v;std::string cls=get(e,"classname");uint64_t flags=python_integer(get(e,"spawnflags","0")).low_bits;
    static const std::set<std::string> nonsolid={"func_illusionary","func_viscluster","func_detail","func_occluder","func_smokevolume","func_dustmotes","func_dustcloud","func_precipitation","func_precipitation_blocker","func_vehicleclip","func_clip_vphysics","func_traincontrols"};
    if(cls.starts_with("trigger_")||cls.starts_with("func_areaportal")||nonsolid.count(cls))return {false,false};
    if(cls=="func_brush"||cls=="func_reflective_glass"||cls=="func_monitor"){auto solidity=python_integer(get(e,"solidity","0"));bool active=!solidity.equals(1)&&(solidity.equals(2)||!python_integer(get(e,"startdisabled","0")).nonzero());return {active,cls=="func_brush"&&solidity.equals(0)&&active};}
    if(cls=="func_lod")return {!python_integer(get(e,"solid","0")).nonzero(),false};
    if(cls=="func_breakable"||cls=="func_breakable_surf")return {true,cls=="func_breakable_surf"||(python_float(get(e,"health","0"))!=0&&!(flags&1))};
    static const std::map<std::string,std::pair<int64_t,bool>> classes={
        {"func_wall",{0,false}},{"func_wall_toggle",{1,false}},{"func_conveyor",{2,false}},{"func_rotating",{64,false}},{"func_movelinear",{8,false}},{"momentary_door",{8,false}},{"func_button",{0,false}},{"func_rot_button",{1,false}},{"momentary_rot_button",{1,false}},{"func_door",{8,true}},{"func_door_rotating",{8,true}},{"func_water",{0,false}},{"func_water_analog",{0,false}},{"func_plat",{0,false}},{"func_platrot",{0,false}},{"func_train",{8,false}},{"func_tracktrain",{8,false}},{"func_trackchange",{0,false}},{"func_trackautochange",{0,false}},{"func_physbox",{0,true}},{"func_pushable",{0,true}},{"func_playerinfected_clip",{0,true}}};
    auto it=classes.find(cls);if(it!=classes.end())return {!(flags&it->second.first),it->second.second};return {std::nullopt,false};
}
Trace Collision::trace(const V3& start,const V3& end,const V3& mins,const V3& maxs,int64_t mask,bool walk)const{return navgen::trace(scene,start,end,mins,maxs,mask,walk);}
Collision Collision::load(const BSP& bsp,const std::filesystem::path& game,const std::string& missing_phy,bool verbose){
    double begin=now();SceneBuilder builder;std::map<std::string,int64_t> counts,unknown;Json unresolved=Json::array();auto c=[&](const std::string& key){auto it=counts.find(key);return it==counts.end()?int64_t(0):it->second;};
    for(int bid:bsp.model_brushes(0)){auto brush=bsp.brush(bid);builder.add(brush.planes,brush.lo,brush.hi,bsp.brushes[bid].contents,false,false,false,false,0,brush.flags);counts["world_brushes"]++;}
    for(const auto& e:bsp.entities){auto model=get(e,"model");if(!model.starts_with('*'))continue;auto [active,walkable]=entity_rules(e);if(!active)unknown[get(e,"classname")]++;if(!active.value_or(false))continue;
        auto org=vector(get(e,"origin")),angles=entity_angles(e);auto rot=rotation(angles);
        auto model_index=python_integer(model.substr(1)).clamped();int index=static_cast<int>(std::clamp<int64_t>(model_index,std::numeric_limits<int>::min(),std::numeric_limits<int>::max()));
        for(int bid:bsp.model_brushes(index)){auto b=bsp.brush(bid);std::optional<std::vector<int64_t>> flags=b.flags;
            if(std::max({std::abs(angles[0]),std::abs(angles[1]),std::abs(angles[2])})>1e-8){auto points=brush_vertices(b.planes,b.sides);for(auto& p:points)p=rotated(p,rot)+org;b.planes=convex_planes(points);std::tie(b.lo,b.hi)=bounds(points);flags=std::nullopt;}
            else{for(auto& p:b.planes){V3 n={p[0],p[1],p[2]};p[3]+=(b.planes.size()==1?dot3(n,org):gemv3(n,org));}b.lo+=org;b.hi+=org;}
            builder.add(b.planes,b.lo,b.hi,bsp.brushes[bid].contents,walkable,false,python_integer(get(e,"rendermode","0")).nonzero(),true,0,flags);counts["entity_brushes"]++;
        }
    }
    if(verbose)progress("collision brushes: "+std::to_string(c("world_brushes"))+" world, "+std::to_string(c("entity_brushes"))+" entity ("+seconds(begin)+"s)");
    Content content(bsp,game);int64_t nunknown=0;for(const auto& [name,n]:unknown){nunknown+=n;unresolved.push_back("unknown brush class skipped ("+std::to_string(n)+" entities): "+name);}counts["unknown_brush_entities"]=nunknown;for(const auto& error:content.archive_errors)unresolved.push_back(error);
    auto props=bsp.static_props();struct Model {std::optional<std::pair<V3,V3>> bounds;std::optional<std::vector<std::vector<V3>>> pieces;};std::map<std::string,Model> models;for(const auto& p:props)models.emplace(p.name,Model{});
    for(auto& [name,model]:models){auto mdl=content.read(name),phy=content.read(name.substr(0,name.size()-4)+".phy");if(!mdl){unresolved.push_back("missing MDL: "+name);counts["missing_models"]++;continue;}
        if(mdl->size()<128||std::string(mdl->begin(),mdl->begin()+4)!="IDST")throw Error("Invalid MDL: "+name);V3 lo,hi;for(int a=0;a<3;a++){lo[a]=little_float(*mdl,104+a*4);hi[a]=little_float(*mdl,116+a*4);}model.bounds=std::pair{lo,hi};
        if(phy){try{model.pieces=parse_phy(*phy);}catch(const std::exception& e){throw Error("PHY "+name+": "+e.what());}counts["phy_models"]++;counts["phy_convex_pieces"]+=static_cast<int64_t>(model.pieces->size());}else counts["models_without_phy"]++;
    }
    if(verbose)progress("collision models: "+std::to_string(models.size())+", PHY parsed "+std::to_string(c("phy_models"))+" ("+seconds(begin)+"s)");
    std::map<std::string,int64_t> missing_instances;struct Shape {std::vector<V4> planes;V3 lo,hi;};using Key=std::tuple<std::string,double,double,double,double>;std::map<Key,std::vector<Shape>> oriented;
    for(const auto& prop:props){counts["static_props"]++;if(prop.solid==0)continue;auto& model=models.at(prop.name);if(!model.bounds)continue;const auto& [boundlo,boundhi]=*model.bounds;
        if(prop.solid==2){auto points=box_vertices(boundlo*prop.scale+prop.origin,boundhi*prop.scale+prop.origin);auto [lo,hi]=bounds(points);builder.add(convex_planes(points),lo,hi,1,false,false,false,false,1);counts["bbox_props"]++;continue;}
        if(prop.solid!=6){unresolved.push_back("unsupported static prop solid "+std::to_string(prop.solid)+": "+prop.name);continue;}
        std::vector<std::vector<V3>> fallback;const auto* pieces=model.pieces?&*model.pieces:nullptr;if(!pieces){missing_instances[prop.name]++;if(missing_phy=="skip")continue;fallback.push_back(box_vertices(boundlo,boundhi));pieces=&fallback;}
        Key key={prop.name,prop.scale,prop.angles[0],prop.angles[1],prop.angles[2]};auto it=oriented.find(key);if(it==oriented.end()){auto rot=rotation(prop.angles);std::vector<Shape> shapes;for(const auto& piece:*pieces){auto points=piece;for(auto& p:points)p=rotated(p*prop.scale,rot);auto [lo,hi]=bounds(points);shapes.push_back({convex_planes(points),lo,hi});}it=oriented.emplace(key,std::move(shapes)).first;}
        for(const auto& shape:it->second){auto planes=shape.planes;for(auto& p:planes){V3 n={p[0],p[1],p[2]};p[3]+=(planes.size()==1?dot3(n,prop.origin):gemv3(n,prop.origin));}builder.add(planes,shape.lo+prop.origin,shape.hi+prop.origin,1,false,false,false,false,1);counts["prop_convex_instances"]++;}
    }
    counts["unique_prop_orientations"]=static_cast<int64_t>(oriented.size());int64_t missing_mdl=0,excluded=0,missing_total=0;for(const auto& p:props)if(p.solid&&!models[p.name].bounds)missing_mdl++;for(const auto& e:bsp.entities)if(lowercase(get(e,"model")).ends_with(".mdl"))excluded++;for(const auto& [name,n]:missing_instances)missing_total+=n;
    counts["missing_mdl_solid_instances"]=missing_mdl;counts["excluded_nonbrush_model_entities"]=excluded;counts["missing_phy_solid_instances"]=missing_total;counts["missing_phy_skipped_instances"]=missing_phy=="skip"?missing_total:0;counts["missing_phy_bbox_instances"]=missing_phy=="bbox"?missing_total:0;
    for(const auto& [name,n]:missing_instances)unresolved.push_back("missing PHY ("+std::to_string(n)+" solid instances, "+missing_phy+"): "+name);
    if(verbose)progress("collision static props: "+std::to_string(c("prop_convex_instances"))+" convex instances, missing PHY "+std::to_string(missing_total)+" ("+missing_phy+"; "+seconds(begin)+"s)");
    auto terrain=bsp.displacements();std::vector<std::pair<std::vector<V4>,V4>> triangles(terrain.size());parallel_for(terrain.size(),[&](size_t i){triangles[i]=triangle_kernel(terrain[i].points);});
    for(size_t i=0;i<terrain.size();i++){const auto& t=terrain[i];const auto& [planes,front]=triangles[i];if(planes.empty())continue;auto [lo,hi]=bounds(std::vector<V3>(t.points.begin(),t.points.end()));builder.add(planes,lo,hi,t.contents,false,false,false,false,2,std::nullopt,front);counts["displacement_triangles"]++;}
    if(verbose)progress("collision terrain: "+std::to_string(c("displacement_triangles"))+" triangles; building BVH");
    auto scene=builder.pack();counts["collision_objects"]=static_cast<int64_t>(scene.first.size());counts["collision_planes"]=static_cast<int64_t>(scene.planes.size());counts["content_conflicts"]=static_cast<int64_t>(content.conflicts.size());Json jout=counts;jout["build_seconds"]=now()-begin;
    Json audit={{"selected_reads",content.used},{"conflicts",content.conflicts},{"archive_errors",content.archive_errors},{"gma_archives",content.gma_archives},{"unknown_brush_classes",unknown}};
    Collision out;out.scene=std::move(scene);out.counts=std::move(jout);out.unresolved=std::move(unresolved);out.content_audit=std::move(audit);return out;
}
}
