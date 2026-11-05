// Include these modules so their internal graph passes and worker pools can be checked.
// The static core library supplies the remaining modules without duplicate objects.
#include "../src/areas.cpp"
#include "../src/sampling.cpp"
#include <iostream>
#include <system_error>

namespace {
using namespace navgen;
void require(bool value,const char* description){if(!value)throw std::runtime_error(description);}
Collision world(const std::vector<std::pair<V3,V3>>& boxes={}){
    SceneBuilder builder;for(const auto& [lo,hi]:boxes){const auto points=box_vertices(lo,hi);builder.add(convex_planes(points),lo,hi);}Collision c;c.scene=builder.pack();return c;
}
struct Fixture {
    Collision collision=world();SamplingArrays arrays;NodeIndex index;RectangleResult rectangles;AreaGraph graph;
    explicit Fixture(std::size_t count=4):arrays(allocate(count)),graph(collision,arrays,index,rectangles,.7,1){}
    AreaPtr area(const std::array<double,4>& bounds,double z=0,std::int64_t attr=0,NodeCorners nodes={-1,-1,-1,-1}){
        return graph.insert(Box{bounds[0],bounds[1],bounds[2],bounds[3],z,z,z,z},attr,{},nodes);
    }
};
void raised_geometry(){
    const std::array<Box,4> expected{{{0,8,25,18,40,45,45,40},{0,0,10,25,45,40,40,45},{0,7,25,17,45,40,40,45},{8,0,18,25,45,40,40,45}}};
    for(int d=0;d<4;++d){Fixture f;auto a=f.area({0,0,25,25},0,0,{0,1,2,3});const auto points=corner_points(a->box);for(int k=0;k<4;++k)f.arrays.pos[k]=points[k];
        const int first=(d+3)%4,last=(d+2)%4;f.arrays.obsh[first][d]=40;f.arrays.obsh[last][d]=45;f.arrays.obsa[first][d]=12;f.arrays.obsa[last][d]=15;f.arrays.obsb[first][d]=21;f.arrays.obsb[last][d]=24;
        f.graph.fence_tops();require(a->box==expected[d],"SDK raised fence geometry");require(a->attr==(NO_MERGE|OBSTACLE_TOP),"SDK raised fence attributes");require(a->nodes==NodeCorners{-1,-1,-1,-1},"raised fence drops nodes");
    }
}
void split_incoming(){
    for(bool outgoing:{false,true}){Fixture f;const auto high=f.area({0,0,25,100},100),low=f.area({25,0,75,100});f.graph.connect(high->id,low->id,1,"drop");if(outgoing){auto neighbour=f.area({0,0,25,100});f.graph.connect(low->id,neighbour->id,3);}
        auto pieces=f.graph.split(low->id,1,50);require(bool(pieces),"split produces both children");PySet<Id> targets;if(outgoing)for(const auto& a:*pieces)targets.insert(a->id);require(high->links[1]==targets,"incoming split follows SDK outgoing-side gate");
    }
}
void fence_creation(){
    Fixture f(2);f.arrays.pos={{25,0,0},{25,25,0}};f.arrays.links[0][2]=1;f.arrays.obsh[0][1]=40;f.arrays.obsh[1][1]=45;for(int n=0;n<2;++n){f.arrays.obsa[n][1]=10;f.arrays.obsb[n][1]=20;}
    auto a=f.area({0,0,25,25},0,0,{-1,0,1,-1}),b=f.area({50,0,75,25});f.graph.connect(a->id,b->id,1);f.graph.connect(b->id,a->id,3);
    require(f.graph.create_fence(a,b,1,false),"fence creation accepted");const auto fence=f.graph.areas.rbegin()->second;require(fence->box==Box{35,0,45,25,40,45,45,40},"east fence heights follow SDK assignment");
    require(!a->links[1].contains(b->id)&&a->links[1].contains(fence->id)&&b->links[3].contains(fence->id),"fence replaces directed portal");require(fence->links[3].contains(a->id)&&fence->links[1].contains(b->id),"fence reciprocal connections");
}
void multinode_contains(){
    Fixture f(6);f.arrays.pos={{25,0,0},{25,25,0},{50,0,40},{50,25,45},{75,0,5},{75,25,5}};f.arrays.links[0][2]=1;
    for(auto pair:std::array<std::array<int,2>,4>{{{0,2},{1,3},{2,4},{3,5}}})f.arrays.links[pair[0]][1]=pair[1];
    auto a=f.area({0,0,25,25},0,0,{-1,0,1,-1}),b=f.area({75,0,100,25});f.graph.connect(a->id,b->id,1);require(f.graph.create_fence(a,b,1,true),"multinode fence uses containment rather than height snapping");
    Fixture floors;auto lower=floors.area({0,0,100,100});floors.area({0,0,100,100},10);Spatial spatial(floors.graph.areas);
    require(spatial.contains(*lower,{25,25,5})&&!spatial.contains(*lower,{25,25,15}),"intervening floor containment");require(spatial.contains(*lower,{25,25,-18})&&!spatial.contains(*lower,{25,25,-18.01}),"step-height containment boundary");
}
void overhang_and_jumps(){
    Fixture f;auto lower=f.area({0,0,100,100}),upper=f.area({0,0,50,100},100);f.graph.connect(upper->id,lower->id,1);f.graph.connect(lower->id,upper->id,3);f.graph.split_overhangs();
    require(!f.graph.areas.contains(lower->id),"overhang split removes lower parent");AreaPtr exposed,shadow;for(const auto& [id,a]:f.graph.areas){if(a->box[0]==50)exposed=a;else if(id!=upper->id)shadow=a;}
    require(exposed&&shadow&&upper->links[1].contains(exposed->id)&&exposed->links[3].contains(upper->id)&&!upper->links[1].contains(shadow->id),"overhang reconnects exposed child");
    Fixture g;auto a=g.area({0,0,25,25}),jump=g.area({25,0,50,25},0,JUMP),b=g.area({50,0,75,25});g.graph.connect(a->id,jump->id,1);g.graph.connect(jump->id,b->id,1);g.graph.stitch_jumps();
    require(!g.graph.areas.contains(jump->id)&&a->links[1].contains(b->id)&&b->incoming.contains({a->id,1}),"jump stitching preserves direction");
}
void overlap_cleanup(){
    Fixture f;auto low=f.area({0,0,25,25},40,OBSTACLE_TOP),high=f.area({0,0,25,25},60,OBSTACLE_TOP);f.graph.remove_overlapping_fences();require(!f.graph.areas.contains(low->id)&&f.graph.areas.contains(high->id),"overlap compares corner heights");
}
void sampling_and_low_room(){
    auto c=world({{{0,0,-20},{200,200,0}},{{0,0,60},{200,200,80}}});const std::array<V3,2> bounds{{{0,0,0},{200,200,80}}};
    auto first=sample(c,{{100.,100.,1.}},bounds,.7,1);const auto traces=c.scene.traces();const auto first_arrays=first.arrays;auto built=build_areas(c,first.arrays,first.index,1,.7,1);
    double cells=0;for(const auto& a:built.nav.at("areas")){require((a.at("attr").get<int64_t>()&CROUCH)!=0,"low room requires crouch");cells+=(a.at("se")[0].get<double>()-a.at("nw")[0].get<double>())*(a.at("se")[1].get<double>()-a.at("nw")[1].get<double>())/625.;}require(cells==64.,"low-room complete cell coverage");
    auto parallel_world=world({{{0,0,-20},{200,200,0}},{{0,0,60},{200,200,80}}});auto second=sample(parallel_world,{{100.,100.,1.}},bounds,.7,2);
    require(first_arrays.pos==second.arrays.pos&&first_arrays.normal==second.arrays.normal&&first_arrays.links==second.arrays.links&&first_arrays.parent==second.arrays.parent&&first_arrays.visited==second.arrays.visited&&first_arrays.nextxy==second.arrays.nextxy&&first_arrays.crouch==second.arrays.crouch&&first_arrays.blocked==second.arrays.blocked&&first_arrays.ground==second.arrays.ground&&first_arrays.obsh==second.arrays.obsh&&first_arrays.obsa==second.arrays.obsa&&first_arrays.obsb==second.arrays.obsb&&first_arrays.disp==second.arrays.disp,"sampling thread determinism");
    require(traces==parallel_world.scene.traces(),"sampling trace counts match at every thread count");auto parallel_built=build_areas(parallel_world,second.arrays,second.index,1,.7,2);require(built.nav==parallel_built.nav&&built.cleanup==parallel_built.cleanup&&built.link_audit==parallel_built.link_audit,"rectangle/graph thread determinism");
}
void thread_creation_failure(const std::string& helper){
    // The fault-injection runner makes the second pthread_create return EAGAIN.
    // Worker 1 must finish its slice before the construction error reaches us.
    require(logical_cores()>=3,"thread failure test needs at least three logical cores");
    bool caught=false;
    if(helper=="sampling"){
        auto collision=world();auto arrays=allocate(9);
        try{crouch_nodes(collision.scene,arrays,3);}catch(const std::system_error&){caught=true;}
        require(caught,"sampling worker construction error propagated");
        require(collision.scene.traces()==36,"sampling joins the first worker before propagating");
    }else if(helper=="areas"||helper=="generic"){
        std::array<int,9> completed{};
        configure_threads(3);
        auto work=[&](std::size_t i){completed[i]=1;};
        try{if(helper=="areas")area_parallel(completed.size(),3,work);else parallel_for(completed.size(),work);}catch(const std::system_error&){caught=true;}
        require(caught,"worker construction error propagated");
        require(completed==std::array<int,9>{0,0,0,1,1,1,0,0,0},"first worker joins before construction error propagates");
    }else throw std::runtime_error("unknown thread failure helper");
    std::cout<<"thread construction failure joined workers\n";
}
void work_exception_propagation(){
    if(logical_cores()<2)return;
    const auto saved_threads=runtime_threads;configure_threads(2);
    for(int helper=0;helper<3;++helper)for(std::size_t failing_worker:{0u,1u}){
        std::array<int,6> completed{};bool caught=false;
        auto work=[&](std::size_t i){if(i==failing_worker*3)throw std::runtime_error("worker body failure");completed[i]=1;};
        try{if(helper==0)sampling_parallel(completed.size(),2,work);else if(helper==1)area_parallel(completed.size(),2,work);else parallel_for(completed.size(),work);}
        catch(const std::runtime_error& error){caught=std::string(error.what())=="worker body failure";}
        require(caught,"worker body exception reaches caller");
        for(std::size_t i=0;i<completed.size();++i)require(completed[i]==(i/3==failing_worker?0:1),"other workers finish before body exception propagates");
    }
    configure_threads(saved_threads);
}
}
int main(int argc,char** argv){try{if(argc==3&&std::string(argv[1])=="--thread-failure"){thread_creation_failure(argv[2]);return 0;}raised_geometry();split_incoming();fence_creation();multinode_contains();overhang_and_jumps();overlap_cleanup();work_exception_propagation();sampling_and_low_room();std::cout<<"sampling and area tests passed\n";return 0;}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
