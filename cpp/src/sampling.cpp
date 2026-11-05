#include "sampling.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <thread>

namespace navgen {
namespace {
template<class F> void sampling_parallel(std::size_t count,int threads,F work) {
    const auto workers=std::min<std::size_t>(std::max(threads,1),count);
    if(workers<=1){for(std::size_t i=0;i<count;++i)work(i);return;}
    std::vector<std::thread> pool;pool.reserve(workers-1);std::exception_ptr error;std::mutex lock;
    auto run=[&](std::size_t worker){
        worker_index=worker;
        try{const auto begin=count*worker/workers,end=count*(worker+1)/workers;for(auto i=begin;i<end;++i)work(i);}
        catch(...){std::lock_guard guard(lock);if(!error)error=std::current_exception();}
    };
    try{for(std::size_t t=1;t<workers;++t)pool.emplace_back(run,t);}
    catch(...){for(auto& thread:pool)thread.join();throw;}
    run(0);for(auto& thread:pool)thread.join();if(error)std::rethrow_exception(error);
}
XYKey node_key(const V3& p){return {static_cast<std::int64_t>(std::nearbyint(p[0]/25)),static_cast<std::int64_t>(std::nearbyint(p[1]/25))};}
double sum_square(const V3& p){return (p[0]*p[0]+p[1]*p[1])+p[2]*p[2];}
void add_node(NodeIndex& index,SamplingArrays& a,const V3& point,const V3& normal,NodeId source,NodeId count,bool on_disp){
    a.pos[count]=point;a.normal[count]=normal;a.parent[count]=source;a.disp[count]=static_cast<std::uint8_t>(on_disp);
    const auto key=node_key(point);const auto old=index.find(key);a.nextxy[count]=old==index.end()?-1:old->second;index[key]=count;
}
bool obstacles_ok(const SamplingArrays& a,NodeId node,int width,int height,int x,int y){
    return (width==1&&height==1)||!((x>0&&a.obsh[node][3]>18)||(y>0&&a.obsh[node][0]>18)||
        (x<width-1&&a.obsh[node][1]>18)||(y<height-1&&a.obsh[node][2]>18));
}
bool rectangle_unowned(const SamplingArrays& a,NodeId node,int width,int height){
    NodeId row=node;for(int y=0;y<height;++y){auto cur=row;for(int x=0;x<width;++x){if(a.owner[cur]>=0)return false;cur=a.links[cur][1];}row=a.links[row][2];}return true;
}
}
std::size_t XYHash::operator()(const XYKey& key) const noexcept {
    auto x=static_cast<std::uint64_t>(key.x),y=static_cast<std::uint64_t>(key.y);
    x^=x>>30;x*=UINT64_C(0xbf58476d1ce4e5b9);x^=x>>27;x*=UINT64_C(0x94d049bb133111eb);x^=x>>31;
    y^=y>>30;y*=UINT64_C(0xbf58476d1ce4e5b9);y^=y>>27;y*=UINT64_C(0x94d049bb133111eb);y^=y>>31;
    return static_cast<std::size_t>(x^(y+UINT64_C(0x9e3779b97f4a7c15)+(x<<6)+(x>>2)));
}
void SamplingArrays::resize(std::size_t size){
    pos.resize(size);normal.resize(size);links.resize(size,{-1,-1,-1,-1});parent.resize(size,-1);visited.resize(size);
    nextxy.resize(size,-1);crouch.resize(size);blocked.resize(size);ground.resize(size);obsh.resize(size);obsa.resize(size);obsb.resize(size);disp.resize(size);owner.resize(size,-1);
}
SamplingArrays allocate(std::size_t capacity){SamplingArrays result;result.resize(capacity);return result;}
V3 endpoint(const V3& start,const V3& end,const Trace& r){return start+(end-start)*r[0];}
std::pair<Trace,V3> floor_trace(Scene& scene,const V3& start,double limit){auto end=start;end[2]-=limit;const auto r=trace(scene,start,end,THIN_MIN,THIN_MAX);return {r,endpoint(start,end,r)};}
AdjacentResult adjacent(Scene& scene,const V3& origin,const V3& target,double slope_limit){
    auto start=origin,end=target,point=target;Trace r{};
    for(int depth=0;depth<64;++depth){
        r=trace(scene,start,end,THIN_MIN,THIN_MAX);point=endpoint(start,end,r);
        if(r[4])return {false,point,r};
        if(std::abs(point[0]-end[0])<1e-5&&std::abs(point[1]-end[1])<1e-5){
            auto floor=floor_trace(scene,point);r=floor.first;point=floor.second;return {!(r[4]||r[0]>=1||r[3]<slope_limit),point,r};
        }
        auto difference=start-point;
        if(depth&&difference[0]*difference[0]+difference[1]*difference[1]<1)return {false,point,r};
        auto floor=floor_trace(scene,point);r=floor.first;point=floor.second;
        if(r[4]||r[0]>=1||r[3]<slope_limit)return {false,point,r};
        auto up=point;up[2]+=18;r=trace(scene,point,up,THIN_MIN,THIN_MAX);start=endpoint(point,up,r);end=target;end[2]=start[2];
    }
    return {false,point,r};
}
MoveResult move_node(Scene& scene,const V3& origin,const V3& target,double slope_limit){
    auto moved=adjacent(scene,origin,target,slope_limit);bool good=moved.good;auto point=moved.point;auto r=moved.result;
    double height=0,obs_start=0,obs_end=25,last_fraction=1.;
    if(!good){
        int h=18;
        while(h<=200){
            auto start=origin,end=target;start[2]+=h;end[2]+=h;const auto tr=trace(scene,start,end,THIN_MIN,THIN_MAX);
            if(!tr[4]&&tr[0]==1){
                auto floor=floor_trace(scene,end);r=floor.first;point=floor.second;
                if(r[4]||r[0]>=1||r[3]<slope_limit)break;
                auto up=origin;up[2]+=h;const auto ceiling=trace(scene,origin,up,THIN_MIN,THIN_MAX);if(ceiling[0]<1)break;
                good=true;height=static_cast<double>(h);break;
            }
            const auto skip=blocked_vertical_span(scene,start,end,THIN_MIN,THIN_MAX,static_cast<std::int64_t>(tr[9]),200-h);
            last_fraction=skip==0?tr[0]:-1.;h+=static_cast<int>(skip)+1;
        }
    }
    if(height<18||point[2]-origin[2]>height-2){height=0;obs_start=0;obs_end=25;}
    else {
        for(int h=static_cast<int>(height)-1;h>17;--h){
            auto start=origin,end=target;start[2]+=h;end[2]+=h;const auto reverse=trace(scene,end,start,THIN_MIN,THIN_MAX);
            if(!reverse[4]&&reverse[0]<1){
                double fraction=last_fraction;if(h!=static_cast<int>(height)-1||fraction<0)fraction=trace(scene,start,end,THIN_MIN,THIN_MAX)[0];
                const auto a=start+(end-start)*fraction,b=endpoint(end,start,reverse);
                obs_start=std::sqrt(sum_square(a-start));obs_end=std::sqrt(sum_square(b-start));if(obs_end==0)obs_end=25;break;
            }
        }
    }
    return {good,point,r,height,obs_start,obs_end};
}
bool overlapped(Scene& scene,const V3& point){
    int total=0;for(int c=0;c<4;++c){auto start=point;start[2]+=35.5;auto end=start+CORNERS[c]*25;auto r=trace(scene,start,end,CUBE_MIN,CUBE_MAX);
        if(r[4]||r[5]||r[0]<.1){++total;continue;}start=endpoint(start,end,r);end[2]-=71;r=trace(scene,start,end,CUBE_MIN,CUBE_MAX);
        if(r[4]||r[5]||r[0]==1||r[3]<.7)++total;}return total==4;
}
bool embedded_displacement(Scene& scene,const V3& point){
    auto up=point;up[2]+=10000;auto r=trace(scene,point,up,THIN_MIN,THIN_MAX);if(r[0]<=0)return false;
    auto start=endpoint(point,up,r);r=trace(scene,start,point,THIN_MIN,THIN_MAX);return r[0]<1&&endpoint(start,point,r)[2]>point[2]+18;
}
void crouch_node(Scene& scene,const V3& point,CornerFlags& crouch,CornerFlags& blocked,V4& ground){
    for(int c=0;c<4;++c){
        ground[c]=0;auto up=point;up[2]+=64;auto tr=trace(scene,point,up,THIN_MIN,THIN_MAX);const auto ceiling=64*tr[0];
        V3 mins{},maxs{};for(int axis=0;axis<3;++axis){mins[axis]=std::min(CORNERS[c][axis]*16,0.);maxs[axis]=std::max(CORNERS[c][axis]*16,0.);}
        bool found=false;for(int height=0;height<=static_cast<int>(ceiling);++height){
            auto start=point;start[2]+=height;maxs[2]=55;tr=trace(scene,start,start,mins,maxs);
            if(!tr[4]){ground[c]=height;maxs[2]=71;tr=trace(scene,start,start,mins,maxs);if(tr[4])crouch[c]=true;found=true;break;}
        }
        if(!found){ground[c]=64;blocked[c]=true;crouch[c]=true;}
    }
}
void crouch_nodes(Scene& scene,SamplingArrays& arrays,int threads){sampling_parallel(arrays.size(),threads,[&](std::size_t i){crouch_node(scene,arrays.pos[i],arrays.crouch[i],arrays.blocked[i],arrays.ground[i]);});}
NodeId find_node(const NodeIndex& index,const SamplingArrays& arrays,const V3& point){
    const auto found=index.find(node_key(point));auto cur=found==index.end()?-1:found->second;
    while(cur>=0){if(std::abs(arrays.pos[cur][2]-point[2])<11.25)return cur;cur=arrays.nextxy[cur];}return -1;
}
SamplingResult sample(Collision& collision,const std::vector<V3>& seeds,const std::array<V3,2>& bounds,double slope_limit,int threads){
    SamplingResult out;auto& a=out.arrays;auto& index=out.index;a.resize(262144);index.reserve(262144);
    NodeId count=0,current=-1;std::size_t seedindex=0;
    while(true){
        if(static_cast<std::size_t>(count+2)>=a.size())a.resize(a.size()*2);
        if(current<0){
            while(seedindex<seeds.size()){
                auto point=seeds[seedindex++];point[0]=std::floor(point[0]/25+.5)*25;point[1]=std::floor(point[1]/25+.5)*25;
                auto start=point,end=point;start[2]+=35.9;end[2]-=400;const auto r=trace(collision.scene,start,end,THIN_MIN,THIN_MAX);
                if(r[5]||r[0]>=1)continue;point=endpoint(start,end,r);if(find_node(index,a,point)>=0)continue;
                add_node(index,a,point,{r[1],r[2],r[3]},-1,count,r[7]!=0);current=count++;break;
            }
            if(current<0)break;
        }
        int direction=-1;for(int d=0;d<4;++d)if(!(a.visited[current]&(1<<d))){direction=d;break;}
        if(direction<0){current=a.parent[current];continue;}
        const auto d=direction;a.visited[current]|=static_cast<std::uint8_t>(1<<d);const auto start=a.pos[current],target=start+DIRECTIONS[d]*25;
        if(target[0]<bounds[0][0]||target[1]<bounds[0][1]||target[0]>bounds[1][0]||target[1]>bounds[1][1])continue;
        const auto move=move_node(collision.scene,start,target,slope_limit);const auto& point=move.point;const auto& r=move.result;
        if(!move.good||(static_cast<std::int64_t>(r[8])&6))continue;
        if(overlapped(collision.scene,point)||embedded_displacement(collision.scene,point))continue;
        auto other=find_node(index,a,point);const auto fresh=other<0;
        if(fresh){other=count;add_node(index,a,point,{r[1],r[2],r[3]},current,count,r[7]!=0);++count;}
        a.links[current][d]=other;a.obsh[current][d]=move.height;a.obsa[current][d]=move.obs_start;a.obsb[current][d]=move.obs_end;
        const auto delta=start[2]-point[2];if(std::abs(delta)<50){const auto back=(d+2)%4;a.links[other][back]=current;a.visited[other]|=static_cast<std::uint8_t>(1<<back);
            a.obsh[other][back]=move.height>0?std::max(0.,move.height+delta):0.;a.obsa[other][back]=25-move.obs_end;a.obsb[other][back]=25-move.obs_start;}
        if(fresh)current=other;
    }
    a.resize(count);crouch_nodes(collision.scene,a,threads);
    out.speculation={{"enabled",false},{"lookup_attempts",0},{"cache_hits",0},{"computed_jobs",0},{"lookup_hit_rate",0.},{"consumed_per_computed_job",0.},{"crouch_nodes_tested",count}};
    collision.sampling_speculation=out.speculation;
    std::cout<<"sampling: "<<count<<" nodes, seeds "<<seedindex<<'/'<<seeds.size()<<'\n';return out;
}
std::vector<std::uint8_t> closed_cells(const SamplingArrays& a){
    std::vector<std::uint8_t> valid(a.size());for(std::size_t i=0;i<a.size();++i){const auto e=a.links[i][1],s=a.links[i][2];
        if(e<0||s<0||a.links[e][3]!=static_cast<NodeId>(i)||a.links[s][0]!=static_cast<NodeId>(i))continue;
        const auto se=a.links[e][2];if(se<0||a.links[s][1]!=se||a.links[se][0]!=e||a.links[se][3]!=s)continue;valid[i]=true;}return valid;
}
bool test_rectangle(Scene& scene,const SamplingArrays& a,const std::vector<std::uint8_t>& valid,NodeId node,int width,int height){
    // Each mask selects the same corner bytes as the reference's four-bit loop.
    // bit_cast keeps the packed checks independent of the host byte order.
    static constexpr std::array<std::uint32_t,4> corner_masks{{
        std::bit_cast<std::uint32_t>(CornerFlags{0,0,255,0}),
        std::bit_cast<std::uint32_t>(CornerFlags{0,0,255,255}),
        std::bit_cast<std::uint32_t>(CornerFlags{0,255,255,0}),
        std::bit_cast<std::uint32_t>(CornerFlags{255,255,255,255})}};
    const bool expected=a.crouch[node][2];const auto n=a.normal[node];const double plane=dot3(n,a.pos[node]);auto row=node;
    for(int y=0;y<height;++y){auto cur=row;for(int x=0;x<width;++x){
        if(cur<0||!valid[cur]||a.owner[cur]>=0)return false;
        const auto mask=corner_masks[(y!=0)*2+(x!=0)];
        if(std::bit_cast<std::uint32_t>(a.blocked[cur])&mask)return false;
        const bool local=(std::bit_cast<std::uint32_t>(a.crouch[cur])&mask)!=0;
        if(local!=expected||!obstacles_ok(a,cur,width,height,x,y))return false;
        cur=a.links[cur][1];if(cur<0)return false;if((width>1||height>1)&&std::abs(dot3(a.pos[cur],n)-plane)>5)return false;
    }if(!obstacles_ok(a,cur,width,height,width,y))return false;row=a.links[row][2];if(row<0)return false;
        if((width>1||height>1)&&std::abs(dot3(a.pos[row],n)-plane)>5)return false;
    }
    auto cur=row;for(int x=0;x<width;++x){if(!obstacles_ok(a,cur,width,height,x,height))return false;cur=a.links[cur][1];if(cur<0)return false;
        if((width>1||height>1)&&std::abs(dot3(a.pos[cur],n)-plane)>5)return false;}
    if(!obstacles_ok(a,cur,width,height,width,height))return false;
    if(expected){const V3 mins{},maxs{25.,25.,55.};row=node;for(int y=0;y<height;++y){cur=row;for(int x=0;x<width;++x){auto end=a.pos[cur];end[2]+=64;
        const auto r=trace(scene,a.pos[cur],end,mins,maxs);if(r[5])return false;cur=a.links[cur][1];}row=a.links[row][2];}}
    return true;
}
RectangleResult rectangles(Scene& scene,SamplingArrays& a,int threads){
    RectangleResult out;out.valid=closed_cells(a);const auto n=a.size();int width=50,height=50;
    while(width>0&&height>0){
        std::vector<std::uint8_t> candidates;if(threads>1){candidates.resize(n);sampling_parallel(n,threads,[&](std::size_t i){if(a.owner[i]<0&&out.valid[i])candidates[i]=test_rectangle(scene,a,out.valid,i,width,height);});}
        for(NodeId i=static_cast<NodeId>(n)-1;i>=0;--i){
            if(a.owner[i]>=0||!out.valid[i])continue;
            if(!candidates.empty()){if(!candidates[i]||!rectangle_unowned(a,i,width,height))continue;}
            else if(!test_rectangle(scene,a,out.valid,i,width,height))continue;
            auto row=i;NodeId ne=-1;const auto area=static_cast<NodeId>(out.boxes.size());
            for(int y=0;y<height;++y){auto cur=row;for(int x=0;x<width;++x){a.owner[cur]=area;cur=a.links[cur][1];}if(y==0)ne=cur;row=a.links[row][2];}
            const auto sw=row;auto se=row;for(int x=0;x<width;++x)se=a.links[se][1];
            out.corners.push_back({i,ne,se,sw});out.boxes.push_back({a.pos[i][0],a.pos[i][1],a.pos[se][0],a.pos[se][1],a.pos[i][2],a.pos[ne][2],a.pos[se][2],a.pos[sw][2]});
            std::int64_t attr=a.crouch[i][2]?1:0;
            if(a.obsh[i][1]>18||a.obsh[i][2]>18||a.obsh[ne][2]>18||a.obsh[ne][3]>18||a.obsh[se][0]>18||a.obsh[se][3]>18||a.obsh[sw][0]>18||a.obsh[sw][1]>18)attr|=8192;
            out.attrs.push_back(attr);
        }
        if(width>=height)--width;else --height;
    }
    return out;
}
bool can_traverse(Scene& scene,const V3& start,const V3& end,double slope_limit){
    const auto delta=start-end;if(std::sqrt(delta[0]*delta[0]+delta[1]*delta[1])<1e-4&&std::abs(delta[2])<18)return true;
    auto target=end;target[2]=start[2];const auto result=move_node(scene,start,target,slope_limit);
    return result.good&&std::abs(result.point[2]-end[2])<=18&&start[2]-end[2]<400&&end[2]-start[2]<=200;
}
}
