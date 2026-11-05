#include "areas.hpp"
#include "pyset.hpp"
#include <deque>
#include <memory>

namespace navgen {
double height_at(const Box& b,double x,double y){
    const double u=std::min(1.,std::max(0.,(x-b[0])/(b[2]-b[0]))),v=std::min(1.,std::max(0.,(y-b[1])/(b[3]-b[1])));
    const auto north=b[4]+u*(b[5]-b[4]),south=b[7]+u*(b[6]-b[7]);return north+v*(south-north);
}
AreaCorners corner_points(const Box& b){return {{{b[0],b[1],b[4]},{b[2],b[1],b[5]},{b[2],b[3],b[6]},{b[0],b[3],b[7]}}};}
std::array<V3,2> surface_normals(const Box& b){
    const auto c=corner_points(b);auto a=cross(c[1]-c[0],c[3]-c[0]),d=cross(c[3]-c[2],c[1]-c[2]);return {a/std::max(norm3(a),1e-9),d/std::max(norm3(d),1e-9)};
}
std::optional<PortalPair> portal(const Box& b,const Box& other,int d){
    const int axis=(d==0||d==2)?1:0,tangent=1-axis;const auto lo=std::max(b[tangent],other[tangent]),hi=std::min(b[tangent+2],other[tangent+2]);
    if(hi-lo<=1e-5)return std::nullopt;V3 p{},q{};p[tangent]=q[tangent]=(lo+hi)*.5;
    p[axis]=(d==0||d==3)?b[axis]:b[axis+2];q[axis]=(d==0||d==3)?other[axis+2]:other[axis];
    p[2]=height_at(b,p[0],p[1]);q[2]=height_at(other,q[0],q[1]);return PortalPair{p,q};
}
PortalProbe portal_probe(Scene& scene,const Box& box,const Box& other,int d,double slope_limit,bool alternatives){
    const auto pair=portal(box,other,d);if(!pair)return {0,{},{}};auto p=(*pair)[0],q=(*pair)[1];p-=DIRECTIONS[d]*.05;q+=DIRECTIONS[d]*.05;p[2]+=.05;q[2]+=.05;
    if(can_traverse(scene,p,q,slope_limit))return {1,p,q};if(!alternatives)return {4,p,q};if(jump_down(scene,p,q))return {2,p,q};
    const int tangent=(d==0||d==2)?0:1;const auto lo=std::max(box[tangent],other[tangent]),hi=std::min(box[tangent+2],other[tangent+2]),inset=std::min(.5,(hi-lo)/4);
    std::vector<double> coordinates{lo+inset,hi-inset};for(double coordinate=std::ceil(lo/25)*25;coordinate<hi;coordinate+=25)coordinates.push_back(coordinate);
    for(const auto coordinate:coordinates){if(!(lo+1e-5<coordinate&&coordinate<hi-1e-5))continue;auto a=p,b=q;a[tangent]=b[tangent]=coordinate;
        a[2]=height_at(box,a[0],a[1])+.05;b[2]=height_at(other,b[0],b[1])+.05;
        if(can_traverse(scene,a,b,slope_limit)||jump_down(scene,a,b))return {3,a,b};}
    return {4,p,q};
}
bool jump_down(Scene& scene,const V3& start,const V3& end){
    const auto dz=start[2]-end[2];if(dz<=64||dz>=400)return false;
    for(int height=1;height<=200;++height){auto up=start;up[2]+=height;auto r=trace(scene,start,up,THIN_MIN,THIN_MAX,BRUSH_MASK,false);
        if(r[0]<=0||r[4])continue;auto a=endpoint(start,up,r);a[2]-=.5;auto b=end;b[2]=a[2];r=trace(scene,a,b,THIN_MIN,THIN_MAX,BRUSH_MASK,false);
        if(r[0]!=1||r[4])continue;auto finish=end;finish[2]+=2;r=trace(scene,b,finish,THIN_MIN,THIN_MAX,BRUSH_MASK,false);
        return r[0]>0&&!r[4]&&endpoint(b,finish,r)[2]<=finish[2]+18;
    }return false;
}
int stair_line(Scene& scene,const V3& start,const V3& end,double slope_limit){
    if(std::abs(start[2]-end[2])<=18)return 0;const auto delta=start-end;const double length=std::sqrt(delta[0]*delta[0]+delta[1]*delta[1]);if(length<1)return 0;
    V3 mins{-2.5,-2.5,0.},maxs{2.5,2.5,1.};const auto axis=std::abs(start[0]-end[0])>std::abs(start[1]-end[1])?0:1;mins[axis]=-8;maxs[axis]=8;
    double previous=0;bool seen=false;const auto steps=static_cast<int>(std::ceil(length/5));
    for(int i=0;i<=steps;++i){auto point=start+(end-start)*std::min(1.,i*5/length),a=point,b=point;a[2]+=36;b[2]-=36;const auto r=trace(scene,a,b,mins,maxs,NPC_MASK,false);
        if(r[4]||r[7]||r[0]>=1||r[3]<.97)return -1;const auto z=endpoint(a,b,r)[2];
        if(i==0){if(std::abs(z-start[2])>18)return -1;}else{const auto dz=std::abs(z-previous);if(dz>18.01)return -1;if(dz>5*std::tan(std::acos(slope_limit)))seen=true;}previous=z;
    }if(std::abs(previous-end[2])>18)return -1;return seen?1:0;
}
namespace {
using Id=NodeId;using Incoming=std::array<Id,2>;using LinkKey=std::tuple<Id,Id,int>;
struct Area {
    Id id;Box box;std::int64_t attr;NodeCorners nodes;std::vector<Id> cells;bool disp;
    std::array<PySet<Id>,4> links;PySet<Incoming> incoming;
    double z(double x,double y)const{return height_at(box,x,y);}
};
using AreaPtr=std::shared_ptr<Area>;using AreaMap=std::map<Id,AreaPtr>;
struct Spatial {
    const AreaMap& areas;double size;std::unordered_map<XYKey,std::vector<Id>,XYHash> bins;
    Spatial(const AreaMap& source,double bin_size=128):areas(source),size(bin_size){for(const auto& [id,a]:areas)add(*a);}
    void add(const Area& a){const auto& b=a.box;for(auto x=static_cast<Id>(std::floor(b[0]/size));x<=static_cast<Id>(std::floor(b[2]/size));++x)
        for(auto y=static_cast<Id>(std::floor(b[1]/size));y<=static_cast<Id>(std::floor(b[3]/size));++y)bins[{x,y}].push_back(a.id);}
    AreaPtr below(const V3& point,double limit=400,double above=.001)const{
        const auto [x,y,z]=point;AreaPtr best;double bestz=-std::numeric_limits<double>::infinity();const auto bucket=bins.find({static_cast<Id>(std::floor(x/size)),static_cast<Id>(std::floor(y/size))});
        if(bucket==bins.end())return {};for(const auto id:bucket->second){const auto& a=areas.at(id);const auto& b=a->box;
            if(b[0]-.001<=x&&x<=b[2]+.001&&b[1]-.001<=y&&y<=b[3]+.001){const auto az=a->z(x,y);if(z-limit<=az&&az<=z+above&&az>bestz){best=a;bestz=az;}}}return best;
    }
    bool contains(const Area& a,const V3& point)const{
        const auto [x,y,z]=point;const auto& box=a.box;if(!(box[0]<=x&&x<=box[2]&&box[1]<=y&&y<=box[3]))return false;const auto az=a.z(x,y);if(az-18>z)return false;
        const auto bucket=bins.find({static_cast<Id>(std::floor(x/size)),static_cast<Id>(std::floor(y/size))});if(bucket==bins.end())return true;
        for(auto id:bucket->second){const auto& other=*areas.at(id);const auto& b=other.box;if(id!=a.id&&b[0]<=x&&x<=b[2]&&b[1]<=y&&y<=b[3]){const auto oz=other.z(x,y);if(az<oz&&oz<=z)return false;}}return true;
    }
};
struct Edge {Id other;int direction;std::string kind;};
struct AreaGraph {
    Collision& collision;Scene& scene;SamplingArrays& arrays;const NodeIndex& index;double slope_limit;int threads;
    AreaMap areas;Id next_id;Json counts=Json::object(),link_audit=Json::object();std::map<LinkKey,std::string> link_kinds;std::unique_ptr<Spatial> fence_spatial;
    AreaGraph(Collision& c,SamplingArrays& a,const NodeIndex& ix,const RectangleResult& rectangles,double slope,int thread_count):collision(c),scene(c.scene),arrays(a),index(ix),slope_limit(slope),threads(thread_count),next_id(rectangles.boxes.size()){
        std::vector<std::vector<Id>> groups(rectangles.boxes.size());for(std::size_t i=0;i<a.size();++i)if(a.owner[i]>=0)groups[a.owner[i]].push_back(i);
        for(std::size_t i=0;i<groups.size();++i)if(!groups[i].empty()){
            bool disp=false;for(auto cell:groups[i])disp=disp||a.disp[cell];areas[i]=std::make_shared<Area>(Area{static_cast<Id>(i),rectangles.boxes[i],rectangles.attrs[i],rectangles.corners[i],std::move(groups[i]),disp,{},{}});
        }
    }
    std::vector<AreaPtr> snapshot()const{std::vector<AreaPtr> result;result.reserve(areas.size());for(const auto& [id,a]:areas)result.push_back(a);return result;}
    std::deque<Id> queue_ids()const{std::deque<Id> result;for(const auto& [id,a]:areas)result.push_back(id);return result;}
    void connect(Id a,Id b,int d,const std::string& kind=""){
        if(a==b||!areas.contains(a)||!areas.contains(b))return;areas.at(a)->links[d].insert(b);areas.at(b)->incoming.insert({a,d});
        const LinkKey key{a,b,d};if(!kind.empty())link_kinds[key]=kind;else if(!link_kinds.contains(key))link_kinds[key]="ordinary_step";
    }
    void disconnect(Id a,Id b,int d){if(areas.contains(a))areas.at(a)->links[d].erase(b);if(areas.contains(b))areas.at(b)->incoming.erase({a,d});link_kinds.erase({a,b,d});}
    void remove(Id i){const auto a=areas.at(i);for(int d=0;d<4;++d)for(const auto b:a->links[d].values())disconnect(i,b,d);for(const auto incoming:a->incoming.values())disconnect(incoming[0],i,static_cast<int>(incoming[1]));for(auto cell:a->cells)arrays.owner[cell]=-1;areas.erase(i);}
    AreaPtr insert(const Box& box,std::int64_t attr,const std::vector<Id>& cells={},const std::optional<NodeCorners>& nodes=std::nullopt,bool disp=false){
        const auto id=next_id++;NodeCorners corners;if(nodes)corners=*nodes;else{const auto points=corner_points(box);for(int k=0;k<4;++k)corners[k]=find_node(index,arrays,points[k]);}
        auto a=std::make_shared<Area>(Area{id,box,attr,corners,cells,disp,{},{}});areas[id]=a;for(auto cell:cells)arrays.owner[cell]=id;return a;
    }
    void initial_links();void mark_jumps();void merge();std::optional<std::array<AreaPtr,2>> split(Id i,int axis,double coordinate);
    void split_overhangs();void square();void mark_stairs();void stitch_jumps();void fence_tops();bool create_fence(const AreaPtr& a,const AreaPtr& b,int d,bool multi);
    void remove_overlapping_fences();void corner_helpers();void sdk_connection_fixes();std::optional<std::array<Id,2>> reciprocal_node_pair(const Area& a,const Area& b,int d)const;
    void fix_connections();Json nav(std::uint64_t bsp_size)const;
};
double min_z(const Box& b){return *std::min_element(b.begin()+4,b.end());}
double max_z(const Box& b){return *std::max_element(b.begin()+4,b.end());}
double mean_z(const Box& b){return ((b[4]+b[5])+b[6]+b[7])/4.;}
bool no_nodes(const Area& a){return *std::min_element(a.nodes.begin(),a.nodes.end())<0;}
template<class F> void area_parallel(std::size_t count,int threads,F work){
    const auto workers=std::min<std::size_t>(std::max(threads,1),count);if(workers<=1){for(std::size_t i=0;i<count;++i)work(i);return;}
    std::vector<std::thread> pool;pool.reserve(workers-1);std::exception_ptr error;std::mutex lock;
    auto run=[&](std::size_t worker){
        worker_index=worker;
        try{for(auto i=count*worker/workers;i<count*(worker+1)/workers;++i)work(i);}
        catch(...){std::lock_guard guard(lock);if(!error)error=std::current_exception();}
    };
    try{for(std::size_t t=1;t<workers;++t)pool.emplace_back(run,t);}
    catch(...){for(auto& thread:pool)thread.join();throw;}
    run(0);for(auto& thread:pool)thread.join();if(error)std::rethrow_exception(error);
}
void AreaGraph::initial_links(){
    const auto& links=arrays.links;const auto& owner=arrays.owner;for(int d=0;d<4;++d){std::set<std::pair<Id,Id>> edges;
        for(std::size_t i=0;i<arrays.size();++i){auto other=links[i][d];if(owner[i]>=0&&other>=0&&owner[other]>=0&&owner[other]!=owner[i]&&links[other][(d+2)%4]==static_cast<Id>(i))edges.insert({owner[i],owner[other]});}
        for(auto [a,b]:edges)connect(a,b,d);
    }
    Spatial spatial(areas);std::array<int,2> parts{};const auto& pos=arrays.pos;const auto& blocked=arrays.blocked;
    auto probe=[&](const AreaPtr& a,Id node,int d){const auto p=pos[node];auto start=p+DIRECTIONS[d]*12.5;start[2]+=35.5;
        for(double distance:{25.,50.,75.,100.}){auto q=start+DIRECTIONS[d]*distance;const auto tr=trace(scene,start,q,THIN_MIN,THIN_MAX,BRUSH_MASK,false);if(tr[0]<1)break;
            const auto b=spatial.below(q,400,5.);if(!b)continue;if(b->id!=a->id){auto dest=q;dest[2]=b->z(q[0],q[1]);if(jump_down(scene,p,dest)){const auto existed=a->links[d].contains(b->id);connect(a->id,b->id,d,"drop");return existed?0:1;}}break;
        }return 0;};
    static constexpr int positions[4][3]={{0,1,1},{1,2,2},{3,2,1},{0,3,2}};
    for(const auto& a:snapshot())for(int d=0;d<4;++d){const auto c0=positions[d][0],c1=positions[d][1],along=positions[d][2];
        for(int part=0;part<((d==1||d==2)?2:1);++part){auto node=a->nodes[c0],end=a->nodes[c1];if(part==0&&(d==1||d==2)){node=node>=0?links[node][(d+2)%4]:-1;end=end>=0?links[end][(d+2)%4]:-1;}
            std::set<Id> seen;while(node>=0&&node!=end&&!seen.contains(node)){seen.insert(node);const auto adj=links[node][d];const bool direct=adj>=0&&owner[adj]>=0&&links[adj][(d+2)%4]==node;
                if(part==0){if(!direct)parts[0]+=probe(a,node,d);}else if(owner[node]<0&&std::none_of(blocked[node].begin(),blocked[node].end(),[](auto x){return x!=0;})&&!(adj>=0&&std::any_of(blocked[adj].begin(),blocked[adj].end(),[](auto x){return x!=0;}))){if(adj<0||owner[adj]<0)parts[1]+=probe(a,node,d);}
                node=links[node][along];
            }
        }
    }
    counts["drop_connections_found"]=parts[0]+parts[1];counts["drop_part1"]=parts[0];counts["drop_part2"]=parts[1];
}
void AreaGraph::mark_jumps(){
    std::int64_t count=0;for(const auto& [id,a]:areas){const auto normals=surface_normals(a->box);const auto nz=std::min(normals[0][2],normals[1][2]);bool jump=nz<slope_limit;
        if(!jump&&nz<slope_limit+.1){V3 p{(a->box[0]+a->box[2])*.5,(a->box[1]+a->box[3])*.5,0.};p[2]=a->z(p[0],p[1])+35.5;auto end=p;end[2]-=9999.9;
            const auto r=trace(scene,p,end,V3{},V3{},BRUSH_MASK,false);jump=r[0]<1&&std::abs(r[3]-nz)>.1;}
        if(jump){a->attr|=JUMP|NO_MERGE;++count;}}
    counts["jump_areas_marked"]=count;
}
void AreaGraph::merge(){
    auto queue=queue_ids();std::int64_t merged=0;while(!queue.empty()){const auto i=queue.front();queue.pop_front();if(!areas.contains(i))continue;auto a=areas.at(i);if(a->attr&NO_MERGE)continue;bool joined=false;
        for(int d:{0,2,3,1}){auto targets=a->links[d].values();std::sort(targets.begin(),targets.end());for(auto j:targets){if(!areas.contains(j))continue;auto b=areas.at(j);
            if(a->attr!=b->attr||(b->attr&NO_MERGE)||!b->links[(d+2)%4].contains(i))continue;const int axis=(d==0||d==2)?1:0,tangent=1-axis;
            if(std::abs(a->box[tangent]-b->box[tangent])>.01||std::abs(a->box[tangent+2]-b->box[tangent+2])>.01)continue;
            const auto pair=portal(a->box,b->box,d);if(!pair||norm2((*pair)[0]-(*pair)[1])>.01)continue;
            const auto low=std::min(a->box[axis],b->box[axis]),high=std::max(a->box[axis+2],b->box[axis+2]);if(high-low>1250.01)continue;
            const auto n=surface_normals(a->box),m=surface_normals(b->box);const auto tolerance=a->disp||b->disp?.7:.99;
            if(std::min({dot3(n[0],m[0]),dot3(n[0],m[1]),dot3(n[1],m[0]),dot3(n[1],m[1])})<tolerance)continue;
            auto newbox=a->box;newbox[axis]=low;newbox[axis+2]=high;const auto points=corner_points(newbox);
            for(int k=0;k<4;++k){const auto& p=points[k];const auto source=a->box[0]-.01<=p[0]&&p[0]<=a->box[2]+.01&&a->box[1]-.01<=p[1]&&p[1]<=a->box[3]+.01?a:b;newbox[4+k]=source->z(p[0],p[1]);}
            bool bad=false;for(const auto& old:{a,b})for(const auto& p:corner_points(old->box))if(std::abs(height_at(newbox,p[0],p[1])-p[2])>5)bad=true;if(bad)continue;
            std::vector<Edge> outgoing,incoming;for(int d2=0;d2<4;++d2)for(auto k:b->links[d2])if(k!=i)outgoing.push_back({k,d2,link_kinds.at({j,k,d2})});
            for(auto edge:b->incoming)if(edge[0]!=i)incoming.push_back({edge[0],static_cast<int>(edge[1]),link_kinds.at({edge[0],j,static_cast<int>(edge[1])})});
            auto cells=a->cells;cells.insert(cells.end(),b->cells.begin(),b->cells.end());remove(j);a->box=newbox;a->cells=std::move(cells);a->disp=a->disp||b->disp;
            const auto corners=corner_points(newbox);for(int k=0;k<4;++k)a->nodes[k]=find_node(index,arrays,corners[k]);for(auto cell:a->cells)arrays.owner[cell]=i;
            for(const auto& edge:outgoing)connect(i,edge.other,edge.direction,edge.kind);for(const auto& edge:incoming)connect(edge.other,i,edge.direction,edge.kind);
            queue.push_back(i);for(auto incoming_edge:a->incoming)queue.push_back(incoming_edge[0]);++merged;joined=true;break;
        }if(joined)break;}
    }counts["merges"]=merged;
}
std::optional<std::array<AreaPtr,2>> AreaGraph::split(Id i,int axis,double coordinate){
    const auto a=areas.at(i);const auto box=a->box;if(!(box[axis]+.1<coordinate&&coordinate<box[axis+2]-.1))return std::nullopt;
    std::vector<Edge> out,inc;for(int d=0;d<4;++d)for(auto b:a->links[d])out.push_back({b,d,link_kinds.at({i,b,d})});
    for(const auto& item:a->incoming){const auto b=item[0];const auto d=static_cast<int>(item[1]);inc.push_back({b,d,link_kinds.at({b,i,d})});}
    std::array<Box,2> boxes{box,box};boxes[0][axis+2]=coordinate;boxes[1][axis]=coordinate;
    for(auto& b:boxes){const auto points=corner_points(b);for(int k=0;k<4;++k)b[4+k]=a->z(points[k][0],points[k][1]);}
    std::array<std::vector<Id>,2> groups;for(auto cell:a->cells)groups[arrays.pos[cell][axis]<coordinate?0:1].push_back(cell);remove(i);
    std::array<AreaPtr,2> parts{insert(boxes[0],a->attr,groups[0],std::nullopt,a->disp),insert(boxes[1],a->attr,groups[1],std::nullopt,a->disp)};
    for(const auto& part:parts){
        for(const auto& link:out){const auto j=link.other;const int d=link.direction;if(!areas.contains(j))continue;const int dim=(d==0||d==2)?1:0,edge=(d==0||d==3)?dim:dim+2;
            if(std::abs(part->box[edge]-box[edge])<.01&&portal(part->box,areas.at(j)->box,d))connect(part->id,j,d,link.kind);}
        for(const auto& link:inc){const auto j=link.other;const int d=link.direction;
            if(std::none_of(out.begin(),out.end(),[&](const Edge& e){return e.direction==(d+2)%4;}))continue;if(!areas.contains(j))continue;
            const int reverse=(d+2)%4,dim=(d==0||d==2)?1:0,edge=(reverse==0||reverse==3)?dim:dim+2;
            if(std::abs(part->box[edge]-box[edge])<.01&&portal(areas.at(j)->box,part->box,d))connect(j,part->id,d,link.kind);
        }
    }
    const int d=axis==0?1:2;connect(parts[0]->id,parts[1]->id,d);connect(parts[1]->id,parts[0]->id,(d+2)%4);return parts;
}
void AreaGraph::split_overhangs(){
    std::int64_t count=0;auto queue=queue_ids();while(!queue.empty()){const auto i=queue.front();queue.pop_front();if(!areas.contains(i))continue;const auto a=areas.at(i);bool done=false;
        for(int d=0;d<4;++d){for(auto j:a->links[d].values()){if(!areas.contains(j))continue;const auto b=areas.at(j);
            if(std::min(a->box[2],b->box[2])-std::max(a->box[0],b->box[0])<=.1||std::min(a->box[3],b->box[3])-std::max(a->box[1],b->box[1])<=.1)continue;
            const bool a_lower=mean_z(a->box)<mean_z(b->box);const auto lower=a_lower?a:b,upper=a_lower?b:a;if(min_z(upper->box)<=max_z(lower->box)+55)continue;
            const int axis=(d==0||d==2)?1:0,down=upper==a?d:(d+2)%4,up=(down+2)%4;double edge,length;
            if(upper->box[axis+2]<lower->box[axis+2]){edge=upper->box[axis+2];length=edge-lower->box[axis];}else{edge=upper->box[axis];length=lower->box[axis+2]-edge;}
            if(length<25){if(length<7.5||lower->box[axis+2]-lower->box[axis]<=50)continue;edge+=(25-length)*((down==0||down==3)?-1:1);}
            if(!(lower->box[axis]+.1<edge&&edge<lower->box[axis+2]-.1))continue;
            const bool from_above=upper->links[down].contains(lower->id),from_below=lower->links[up].contains(upper->id);
            const auto get_kind=[&](Id x,Id y,int direction){const auto it=link_kinds.find({x,y,direction});return it==link_kinds.end()?std::string{}:it->second;};
            const auto above_kind=get_kind(upper->id,lower->id,down),below_kind=get_kind(lower->id,upper->id,up);
            disconnect(upper->id,lower->id,down);disconnect(lower->id,upper->id,up);const auto parts=split(lower->id,axis,edge);
            if(parts){const auto exposed=(*parts)[(down==0||down==3)?0:1];if(from_above)connect(upper->id,exposed->id,down,above_kind);if(from_below)connect(exposed->id,upper->id,up,below_kind);
                ++count;for(const auto& p:*parts)queue.push_back(p->id);queue.push_back(upper->id);done=true;}
            if(done)break;
        }if(done)break;}
    }counts["overhang_splits"]=count;
}
void AreaGraph::square(){
    auto queue=queue_ids();std::int64_t count=0;while(!queue.empty()){auto i=queue.front();queue.pop_front();if(!areas.contains(i))continue;const auto a=areas.at(i);
        const std::array<double,2> sizes{a->box[2]-a->box[0],a->box[3]-a->box[1]};if(std::max(sizes[0],sizes[1])<=3.01*std::min(sizes[0],sizes[1]))continue;
        const int axis=sizes[0]>=sizes[1]?0:1;const auto cut=std::floor((a->box[axis]+a->box[axis+2])/50+.5)*25;const auto parts=split(i,axis,cut);
        if(parts){++count;for(const auto& p:*parts)queue.push_back(p->id);}
    }counts["square_splits"]=count;
}
void AreaGraph::mark_stairs(){
    std::int64_t count=0;std::vector<AreaPtr> candidates;std::vector<V3> starts,ends;
    for(const auto& [id,a]:areas){const auto& b=a->box;if(std::max(b[2]-b[0],b[3]-b[1])<=25||max_z(b)-min_z(b)<=18)continue;const auto normals=surface_normals(b);if(dot3(normals[0],normals[1])<.95)continue;
        const auto x0=b[0]+5,y0=b[1]+5,x1=b[2]-5,y1=b[3]-5,xm=(x0+x1)*.5,ym=(y0+y1)*.5;
        const std::array<std::array<double,4>,6> pairs{{{x0,y0,x1,y0},{x0,y1,x1,y1},{x0,y0,x0,y1},{x1,y0,x1,y1},{xm,y0,xm,y1},{x1,ym,x0,ym}}};candidates.push_back(a);
        for(auto pair:pairs){starts.push_back({pair[0],pair[1],a->z(pair[0],pair[1])});ends.push_back({pair[2],pair[3],a->z(pair[2],pair[3])});}
    }
    std::vector<int> flags(starts.size());area_parallel(starts.size(),threads,[&](std::size_t i){flags[i]=stair_line(scene,starts[i],ends[i],slope_limit);});
    for(std::size_t i=0;i<candidates.size();++i){auto first=flags.begin()+i*6,last=first+6;if(*std::min_element(first,last)>=0&&*std::max_element(first,last)>0){candidates[i]->attr=STAIRS;++count;}}
    counts["stair_areas"]=count;
}
void AreaGraph::stitch_jumps(){
    std::int64_t removed=0,added=0;std::vector<Id> jumps;for(const auto& [id,a]:areas)if(a->attr&JUMP)jumps.push_back(id);
    for(auto i:jumps){const auto a=areas.at(i);for(const auto& entry:a->incoming.values()){const auto source=entry[0];const auto d=static_cast<int>(entry[1]);if(!areas.contains(source)||(areas.at(source)->attr&JUMP))continue;
        for(auto dest:a->links[d].values()){if(!areas.contains(dest)||(areas.at(dest)->attr&JUMP))continue;const auto pair=portal(areas.at(source)->box,areas.at(dest)->box,d);
            if(!pair||norm2((*pair)[0]-(*pair)[1])>=75)continue;if((areas.at(source)->attr&STAIRS)&&(*pair)[1][2]-(*pair)[0][2]>18)continue;connect(source,dest,d,"stitched_jump");++added;}
    }}for(auto i:jumps){remove(i);++removed;}counts["jump_areas_removed"]=removed;counts["jump_links_added"]=added;
}
void AreaGraph::fence_tops(){
    std::int64_t raised=0,created=0,removed=0;const auto& pos=arrays.pos;const auto& obsh=arrays.obsh;const auto& obsa=arrays.obsa;
    for(const auto& a:snapshot()){
        if(std::max(std::abs(a->box[2]-a->box[0]-25),std::abs(a->box[3]-a->box[1]-25))>.01||no_nodes(*a))continue;
        bool have_best=false,stair_neighbor=false;int direction=0;std::array<double,2> best_top{};double best_start=0;
        for(int d=0;d<4;++d){const std::array<Id,2> ids{a->nodes[(d+3)%4],a->nodes[(d+2)%4]};bool stair_here=false;for(auto j:a->links[d])stair_here=stair_here||bool(areas.at(j)->attr&STAIRS);stair_neighbor=stair_neighbor||stair_here;
            const int number=(obsh[ids[0]][d]>18?1:0)+(obsh[ids[1]][d]>18?1:0);if(number<(stair_here?1:2))continue;
            const std::array<double,2> top{pos[ids[0]][2]+obsh[ids[0]][d],pos[ids[1]][2]+obsh[ids[1]][d]};
            if(!have_best||std::max(top[0],top[1])>std::max(best_top[0],best_top[1])){have_best=true;direction=d;best_top=top;best_start=std::min(obsa[ids[0]][d],obsa[ids[1]][d]);}
        }
        if(!have_best)continue;if(stair_neighbor){remove(a->id);++removed;continue;}
        const auto d=direction;auto top=best_top;double start=std::max(0.,best_start-5),end=std::min(25.,start+10);start=end-10;
        const auto axis=(d==0||d==2)?1:0;auto box=a->box;
        if(d==1||d==2){box[axis+2]=box[axis]+end;box[axis]+=start;}else{box[axis]=box[axis+2]-end;box[axis+2]-=start;}
        if(d==1){box[0]=a->box[0];box[2]=a->box[0]+end-start;}if(d==1||d==2||d==3)std::swap(top[0],top[1]);
        box[4]=top[0];box[5]=top[1];box[6]=top[1];box[7]=top[0];a->box=box;a->attr=(a->attr|NO_MERGE|OBSTACLE_TOP)&~JUMP;a->nodes={-1,-1,-1,-1};
        for(auto orth:{(d+1)%4,(d+3)%4}){for(auto j:a->links[orth].values())disconnect(a->id,j,orth);for(const auto& incoming:a->incoming.values())if((incoming[1]+2)%4==orth)disconnect(incoming[0],a->id,static_cast<int>(incoming[1]));}
        ++raised;
    }
    fence_spatial=std::make_unique<Spatial>(areas);for(const auto& a:snapshot()){
        if(a->attr&(JUMP|OBSTACLE_TOP))break;if(no_nodes(*a))continue;
        for(int d=0;d<4;++d)for(auto j:a->links[d].values()){const auto b=areas.at(j);if(b->attr&(JUMP|OBSTACLE_TOP))continue;if(create_fence(a,b,d,false)||create_fence(a,b,d,true))++created;}
    }
    fence_spatial.reset();counts["obstacle_areas_raised"]=raised;counts["obstacle_areas_created"]=created;counts["obstacle_areas_removed"]=removed;
}
bool AreaGraph::create_fence(const AreaPtr& a,const AreaPtr& b,int d,bool multi){
    const auto& pos=arrays.pos;const auto& links=arrays.links;const auto& obsh=arrays.obsh;const auto& obsa=arrays.obsa;const auto& obsb=arrays.obsb;
    const int axis=(d==0||d==2)?1:0,tangent=1-axis;const auto lo=std::max(a->box[tangent],b->box[tangent]),hi=std::min(a->box[tangent+2],b->box[tangent+2]);if(hi<=lo)return false;
    std::array<double,2> center{};center[tangent]=(lo+hi)*.5;center[axis]=(d==0||d==3)?a->box[axis]:a->box[axis+2];
    std::unique_ptr<Spatial> local_spatial;auto spatial=fence_spatial.get();if(multi&&!spatial){local_spatial=std::make_unique<Spatial>(areas);spatial=local_spatial.get();}
    auto node=a->nodes[d];const auto last_node=a->nodes[(d+1)%4];const int edge=(d+1)%4;std::vector<double> heights,levels,starts,ends;std::set<Id> seen;
    while(node>=0&&!seen.contains(node)){seen.insert(node);const auto p=pos[node];const auto dx=p[0]-center[0],dy=p[1]-center[1],radius=(hi-lo)*.5+1;
        if(dx*dx+dy*dy<radius*radius){auto height=obsh[node][d],start=obsa[node][d],end=obsb[node][d];if(multi){height=0.;const auto other=links[node][d];
            if(!spatial->contains(*b,p)&&other>=0&&pos[other][2]-p[2]>18){const auto q=pos[other];const bool inside=spatial->contains(*b,q)&&std::abs(q[2]-b->z(q[0],q[1]))<2;const auto further=links[other][d];
                if(!inside&&further>=0){const auto r=pos[further];if(spatial->contains(*b,r)&&std::abs(p[2]-r[2])<=18){height=q[2]-p[2];start=20.;end=30.;}}
            }
        }heights.push_back(height);levels.push_back(p[2]);starts.push_back(start);ends.push_back(end);}
        if(node==last_node)break;node=links[node][edge];
    }
    if(heights.empty()||*std::min_element(heights.begin(),heights.end())<=18)return false;auto first=heights.front(),last=heights.back();const auto highest=*std::max_element(heights.begin(),heights.end());if(highest>std::max(first,last))first=last=highest;
    std::array<double,2> top{levels.front()+first,levels.back()+last};if(d==2||d==3)std::swap(top[0],top[1]);auto start=*std::min_element(starts.begin(),starts.end()),end=*std::max_element(ends.begin(),ends.end());const auto limit=multi?50.:25.;
    if(end-start<10){start=std::max(0.,(start+end)*.5-5);end=std::min(limit,start+10);start=end-10;}
    const auto edge_coord=(d==0||d==3)?a->box[axis]:a->box[axis+2];const int sign=(d==0||d==3)?-1:1;std::array<double,2> bounds{edge_coord+sign*start,edge_coord+sign*end};std::sort(bounds.begin(),bounds.end());
    Box box{};box[tangent]=lo;box[tangent+2]=hi;box[axis]=bounds[0];box[axis+2]=bounds[1];box[4]=top[0];box[5]=top[1];box[6]=top[1];box[7]=top[0];
    const auto fence=insert(box,a->attr|NO_MERGE|OBSTACLE_TOP,{},NodeCorners{-1,-1,-1,-1});const int back=(d+2)%4;if(spatial)spatial->add(*fence);
    disconnect(a->id,b->id,d);connect(a->id,fence->id,d,"fence");connect(fence->id,a->id,back,"fence");connect(fence->id,b->id,d,"fence");
    if(b->links[back].contains(a->id)){disconnect(b->id,a->id,back);connect(b->id,fence->id,back,"fence");}return true;
}
void AreaGraph::remove_overlapping_fences(){
    std::vector<AreaPtr> fences;std::set<Id> removed;for(const auto& [id,a]:areas)if(a->attr&OBSTACLE_TOP)fences.push_back(a);
    for(std::size_t k=0;k<fences.size();++k){const auto& a=fences[k];const auto normals=surface_normals(a->box);if(std::min(normals[0][2],normals[1][2])<slope_limit)removed.insert(a->id);
        for(std::size_t j=k+1;j<fences.size();++j){const auto& b=fences[j];if(std::min(std::min(a->box[2],b->box[2])-std::max(a->box[0],b->box[0]),std::min(a->box[3],b->box[3])-std::max(a->box[1],b->box[1]))<=0)continue;
            if(a->box[0]<=b->box[0]&&a->box[1]<=b->box[1]&&a->box[2]>=b->box[2]&&a->box[3]>=b->box[3]&&a->box[4]<=b->box[4]&&a->box[6]>=b->box[6])removed.insert(b->id);
            else if(b->box[0]<=a->box[0]&&b->box[1]<=a->box[1]&&b->box[2]>=a->box[2]&&b->box[3]>=a->box[3]&&b->box[4]<=a->box[4]&&b->box[6]>=a->box[6])removed.insert(a->id);
            else removed.insert((a->box[2]-a->box[0])*(a->box[3]-a->box[1])>(b->box[2]-b->box[0])*(b->box[3]-b->box[1])?b->id:a->id);
        }
    }for(auto i:removed)remove(i);counts["overlapping_or_steep_obstacle_areas_removed"]=removed.size();
}
void AreaGraph::corner_helpers(){
    std::map<V3,std::vector<std::pair<Id,int>>> by_corner;Spatial spatial(areas);std::int64_t created=0;std::set<Box> seen;
    auto key3=[](V3 p){for(auto& v:p)v=round_decimal(v,1e4);return p;};
    for(const auto& [id,a]:areas){const auto points=corner_points(a->box);for(int c=0;c<4;++c)by_corner[key3(points[c])].push_back({id,c});}
    for(const auto& a:snapshot()){const auto points=corner_points(a->box);for(int c=0;c<4;++c){const auto point=points[c];const int right=c,left=(c+3)%4;
        bool has_incoming=false;for(const auto& incoming:a->incoming){const auto direction=(incoming[1]+2)%4;if(direction==right||direction==left){has_incoming=true;break;}}
        if(!a->links[right].empty()||!a->links[left].empty()||has_incoming)continue;
        for(const auto [other,other_corner]:by_corner[key3(point)]){if(other==a->id||other_corner!=(c+2)%4)continue;const auto b=areas.at(other);
            const std::array<std::array<int,2>,2> directions{{{left,(left+3)%4},{right,(right+1)%4}}};
            for(const auto& pair:directions){const auto d_other=pair[0],d_ours=pair[1];auto po=point+DIRECTIONS[d_other]*12.5;po[2]=b->z(po[0],po[1]);auto pu=point+DIRECTIONS[d_ours]*12.5;pu[2]=a->z(pu[0],pu[1]);auto far=point+(DIRECTIONS[d_other]+DIRECTIONS[d_ours])*12.5;
                auto first=adjacent(scene,point,po,slope_limit);if(!first.good||std::abs(first.point[2]-point[2])>18)continue;
                auto second=adjacent(scene,po,far,slope_limit);if(!second.good||std::abs(second.point[2]-po[2])>18)continue;
                auto third=adjacent(scene,pu,far,slope_limit);if(!third.good||std::abs(third.point[2]-pu[2])>18||std::abs(third.point[2]-second.point[2])>1)continue;far=third.point;
                if(spatial.below(far+V3{0.,0.,.01},18))continue;const AreaCorners vertices{point,po,pu,far};Box box{};box[0]=box[2]=point[0];box[1]=box[3]=point[1];
                for(const auto& p:vertices){box[0]=std::min(box[0],p[0]);box[1]=std::min(box[1],p[1]);box[2]=std::max(box[2],p[0]);box[3]=std::max(box[3],p[1]);}
                const auto corners=corner_points(box);for(int k=0;k<4;++k){std::size_t closest=0;double distance=std::numeric_limits<double>::infinity();for(std::size_t j=0;j<vertices.size();++j){const auto dx=vertices[j][0]-corners[k][0],dy=vertices[j][1]-corners[k][1],length=std::sqrt(dx*dx+dy*dy);if(length<distance){distance=length;closest=j;}}box[4+k]=vertices[closest][2];}
                auto key=box;for(auto& v:key)v=round_decimal(v,1e4);if(seen.contains(key))continue;seen.insert(key);const auto helper=insert(box,a->attr,{},NodeCorners{-1,-1,-1,-1});
                connect(a->id,helper->id,d_other,"corner_helper");connect(helper->id,a->id,(d_other+2)%4,"corner_helper");connect(b->id,helper->id,d_ours,"corner_helper");connect(helper->id,b->id,(d_ours+2)%4,"corner_helper");++created;
            }
        }
    }}counts["corner_helpers"]=created;
}
void AreaGraph::sdk_connection_fixes(){
    std::int64_t stairs_removed=0,redundant=0;const auto& ground=arrays.ground;const auto& positions=arrays.pos;
    for(const auto& a:snapshot()){if(!(a->attr&STAIRS)||no_nodes(*a))continue;const auto corners=corner_points(a->box);for(int d=0;d<4;++d){const std::array<int,2> edge{d,(d+1)%4};if(std::abs(corners[edge[0]][2]-corners[edge[1]][2])<18)continue;
        for(auto j:a->links[d].values()){const auto b=areas.at(j);if(no_nodes(*b))continue;const auto pair=portal(a->box,b->box,d);if(!pair)continue;
            auto nearest=[&](const Area& ar,const V3& p){std::size_t result=0;double distance=std::numeric_limits<double>::infinity();for(std::size_t k=0;k<4;++k){const auto norm=axis_norm3(positions[ar.nodes[k]]-p);if(norm<distance){distance=norm;result=k;}}return ar.nodes[result];};
            const auto ni=nearest(*a,(*pair)[0]),nj=nearest(*b,(*pair)[1]);const std::array<int,2> other_edge{(d+2)%4,(d+3)%4};
            if(std::max(ground[ni][edge[0]],ground[ni][edge[1]])>18||positions[nj][2]+std::max(ground[nj][other_edge[0]],ground[nj][other_edge[1]])>positions[ni][2]+18){disconnect(a->id,j,d);++stairs_removed;}
        }
    }}
    for(const auto& a:snapshot())for(int d=0;d<4;++d){const auto targets=a->links[d].copy();PySet<Id> removed;for(auto b:targets)removed.update(targets.intersection(areas.at(b)->links[d]));for(auto b:removed){disconnect(a->id,b,d);++redundant;}}
    counts["stair_side_links_removed"]=stairs_removed;counts["redundant_links_removed"]=redundant;
}
std::optional<std::array<Id,2>> AreaGraph::reciprocal_node_pair(const Area& a,const Area& b,int d)const{
    const int axis=(d==0||d==2)?1:0,tangent=1-axis;const auto edge=(d==0||d==3)?a.box[axis]:a.box[axis+2],other=(d==0||d==3)?b.box[axis+2]:b.box[axis];
    const auto lo=std::max(a.box[tangent],b.box[tangent]),hi=std::min(a.box[tangent+2],b.box[tangent+2]);if(std::abs(edge-other)>.01||hi-lo<=1e-5)return std::nullopt;
    const int sign=(d==0||d==3)?-1:1;for(const auto source:a.cells){const auto target=arrays.links[source][d];if(arrays.owner[source]!=a.id||target<0)continue;if(arrays.owner[target]!=b.id||arrays.links[target][(d+2)%4]!=source)continue;
        const auto& p=arrays.pos[source];const auto& q=arrays.pos[target];
        if((p[axis]-edge)*sign<=.01&&(q[axis]-edge)*sign>=-.01&&p[tangent]>=lo-.01&&p[tangent]<hi-.01&&std::abs(p[tangent]-q[tangent])<.01)return std::array<Id,2>{source,target};
    }return std::nullopt;
}
void AreaGraph::fix_connections(){
    std::int64_t removed=0;Json by_kind=Json::object(),examples=Json::array(),evidence=Json::array();std::vector<LinkKey> edges;
    for(const auto& [id,a]:areas)for(int d=0;d<4;++d)for(const auto j:a->links[d])edges.push_back({id,j,d});std::vector<PortalProbe> probes(edges.size());
    auto probe=[&](std::size_t i){const auto [a,b,d]=edges[i];probes[i]=portal_probe(scene,areas.at(a)->box,areas.at(b)->box,d,slope_limit,true);};
    if(edges.size()<64){for(std::size_t i=0;i<edges.size();++i)probe(i);}else area_parallel(edges.size(),threads,probe);
    static constexpr const char* statuses[]={"no_portal","midpoint","drop_probe","portal_sample","unverified"};std::size_t cursor=0;
    for(const auto& a:snapshot())for(int d=0;d<4;++d)for(auto j:a->links[d].values()){
        const auto b=areas.at(j);const auto kind=(a->attr|b->attr)&OBSTACLE_TOP?std::string("fence"):link_kinds.at({a->id,j,d});
        if(!by_kind.contains(kind))by_kind[kind]={{"tested",0},{"midpoint_failed",0},{"recovered",0},{"retained_unverified",0},{"removed_no_portal",0},{"node_backed",0},{"removed_no_node_evidence",0}};
        auto& c=by_kind[kind];auto increase=[&](const char* key){c[key]=c[key].get<std::int64_t>()+1;};increase("tested");const auto& result=probes[cursor++];std::string status=statuses[result.code];if(status=="midpoint")continue;increase("midpoint_failed");
        if(status=="no_portal"){disconnect(a->id,j,d);++removed;increase("removed_no_portal");}
        else if(status=="unverified"){
            const auto witness=kind=="ordinary_step"?reciprocal_node_pair(*a,*b,d):std::nullopt;
            if(kind=="ordinary_step")evidence.push_back({{"source_generation_id",a->id},{"target_generation_id",j},{"direction",d},{"node_pair",witness?Json(*witness):Json(nullptr)},{"positions",witness?Json(PortalPair{arrays.pos[(*witness)[0]],arrays.pos[(*witness)[1]]}):Json(nullptr)}});
            if(kind=="ordinary_step"&&!witness){disconnect(a->id,j,d);++removed;increase("removed_no_node_evidence");status="no_node_evidence";}
            else{increase("retained_unverified");if(witness)increase("node_backed");}
        }else increase("recovered");
        std::size_t count=0;for(const auto& example:examples)if(example.at("kind")==kind&&example.at("status")==status)++count;
        if(count<4)examples.push_back({{"kind",kind},{"status",status},{"source_generation_id",a->id},{"target_generation_id",j},{"direction",d},{"source_box",a->box},{"target_box",b->box},{"probe",result.code==0?Json(nullptr):Json(PortalPair{result.p,result.q})}});
    }
    counts["invalid_links_removed"]=removed;
    link_audit={{"policy","Remove no-portal links and unverified ordinary steps without reciprocal sampled nodes across their shared edge; retain other unverified links"},{"by_kind",by_kind},{"examples",examples},{"ordinary_node_evidence",evidence}};
}
Json AreaGraph::nav(std::uint64_t bsp_size)const{
    std::map<Id,Id> ids;Id next=1;for(const auto& [old,a]:areas)ids[old]=next++;Json records=Json::array();
    for(const auto& [old,a]:areas){const auto& b=a->box;Json links=Json::array();for(const auto& ls:a->links){auto targets=ls.values();std::sort(targets.begin(),targets.end());Json direction=Json::array();for(auto j:targets)direction.push_back(ids.at(j));links.push_back(std::move(direction));}
        records.push_back({{"id",ids.at(old)},{"attr",a->attr},{"nw",V3{b[0],b[1],b[4]}},{"se",V3{b[2],b[3],b[6]}},{"ne_z",b[5]},{"sw_z",b[7]},{"links",links},{"hiding",Json::array()},{"encounters",Json::array()},{"place",0},{"ladders",Json::array({Json::array(),Json::array()})},{"occupy",std::array<double,2>{0.,0.}},{"light",V4{1.,1.,1.,1.}},{"visible",Json::array()},{"inherit_vis",0}});
    }
    return {{"magic",0xFEEDFACE},{"version",16},{"subversion",0},{"bsp_size",bsp_size},{"analyzed",0},{"places",Json::array()},{"has_unnamed",1},{"areas",records},{"ladders",Json::array()},{"trailing",Json::array()}};
}
}
AreasResult build_areas(Collision& collision,SamplingArrays& arrays,const NodeIndex& index,std::uint64_t bsp_size,double slope_limit,int threads){
    AreasResult out;out.timings=Json::object();auto started=now();const auto result=rectangles(collision.scene,arrays,threads);out.timings["rectangles"]=now()-started;
    std::size_t valid=0;for(auto value:result.valid)valid+=value;progress("rectangles: "+std::to_string(result.boxes.size())+" areas, "+std::to_string(valid)+" closed cells");
    AreaGraph graph(collision,arrays,index,result,slope_limit,threads);
    if(!areas_dump_directory.empty()){fs::create_directories(areas_dump_directory);write_json(areas_dump_directory/"areas-rectangles.json",graph.nav(bsp_size));dump_array(areas_dump_directory/"rectangle-boxes.npy",result.boxes,"<f8");dump_array(areas_dump_directory/"rectangle-corners.npy",result.corners,"<i8");dump_array(areas_dump_directory/"rectangle-attrs.npy",result.attrs,"<i8");dump_array(areas_dump_directory/"rectangle-valid.npy",result.valid,"|b1");}
    using Method=void(AreaGraph::*)();const std::array<std::pair<const char*,Method>,12> methods{{
        {"connections",&AreaGraph::initial_links},{"jump_marking",&AreaGraph::mark_jumps},{"merge",&AreaGraph::merge},{"overhang_splits",&AreaGraph::split_overhangs},{"square_up",&AreaGraph::square},{"stairs",&AreaGraph::mark_stairs},{"jump_stitching",&AreaGraph::stitch_jumps},{"fence_tops",&AreaGraph::fence_tops},{"fence_overlap_cleanup",&AreaGraph::remove_overlapping_fences},{"corner_helpers",&AreaGraph::corner_helpers},{"sdk_connection_fixes",&AreaGraph::sdk_connection_fixes},{"connection_validation",&AreaGraph::fix_connections}}};
    for(const auto& [name,method]:methods){started=now();(graph.*method)();out.timings[name]=now()-started;std::size_t links=0;for(const auto& [id,a]:graph.areas)for(const auto& ls:a->links)links+=ls.size();
        progress(std::string(name)+": "+std::to_string(graph.areas.size())+" areas, "+std::to_string(links)+" links");
        if(!areas_dump_directory.empty())write_json(areas_dump_directory/(std::string("areas-")+name+".json"),graph.nav(bsp_size));
    }
    out.nav=graph.nav(bsp_size);out.cleanup=graph.counts;out.link_audit=graph.link_audit;return out;
}
}
