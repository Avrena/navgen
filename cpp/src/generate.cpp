#include "generate.hpp"
#include "bsp.hpp"
#include "collision.hpp"
#include "sampling.hpp"
#include "areas.hpp"
#include "hiding.hpp"
#include "discovery.hpp"
#include "ladders.hpp"
#include "merge.hpp"
#include "navio.hpp"
#include "build_id.hpp"
#include <cerrno>
#ifdef _WIN32
#define NOMINMAX
#include <windows.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace navgen {
namespace {
bool same_resolved_path(const fs::path& a,const fs::path& b){
    auto left=fs::weakly_canonical(a),right=fs::weakly_canonical(b);
#ifdef _WIN32
    auto left_text=left.generic_u8string(),right_text=right.generic_u8string();
    return unicode_lower(std::string(left_text.begin(),left_text.end()))==unicode_lower(std::string(right_text.begin(),right_text.end()));
#else
    return left==right;
#endif
}
void atomic_write(const fs::path& target,std::span<const std::uint8_t> bytes){
    auto parent=target.parent_path();if(parent.empty())parent=".";fs::create_directories(parent);
    fs::path temp;bool created=false;
    auto nonce=static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
    for(unsigned attempt=0;attempt<100;++attempt){
        auto name=target.filename();name+=path_from_utf8("."+std::to_string(nonce)+"."+std::to_string(attempt)+".tmp");temp=parent/name;
#ifdef _WIN32
        HANDLE f=CreateFileW(temp.c_str(),GENERIC_WRITE,0,nullptr,CREATE_NEW,FILE_ATTRIBUTE_NORMAL,nullptr);
        if(f==INVALID_HANDLE_VALUE){if(GetLastError()==ERROR_FILE_EXISTS)continue;throw Error("Cannot create output temporary file","OSError");}
        created=true;std::size_t offset=0;bool good=true;
        while(offset<bytes.size()){auto n=static_cast<DWORD>(std::min<std::size_t>(bytes.size()-offset,1u<<30));DWORD written=0;if(!WriteFile(f,bytes.data()+offset,n,&written,nullptr)||written==0){good=false;break;}offset+=written;}
        if(!CloseHandle(f))good=false;
        if(!good){fs::remove(temp);throw Error("Cannot write output temporary file","OSError");}
#else
        int f=::open(temp.c_str(),O_WRONLY|O_CREAT|O_EXCL,0600);
        if(f<0){if(errno==EEXIST)continue;throw Error("Cannot create output temporary file","OSError");}
        created=true;std::size_t offset=0;bool good=true;
        while(offset<bytes.size()){auto n=::write(f,bytes.data()+offset,bytes.size()-offset);if(n<0&&errno==EINTR)continue;if(n<=0){good=false;break;}offset+=static_cast<std::size_t>(n);}
        if(::close(f)<0)good=false;
        if(!good){fs::remove(temp);throw Error("Cannot write output temporary file","OSError");}
#endif
        break;
    }
    if(!created)throw Error("Cannot allocate output temporary file","FileExistsError");
    try{
#ifdef _WIN32
        if(!MoveFileExW(temp.c_str(),target.c_str(),MOVEFILE_REPLACE_EXISTING))throw Error("Cannot replace NAV output","OSError");
#else
        fs::rename(temp,target);
#endif
    }catch(...){std::error_code ignored;fs::remove(temp,ignored);throw;}
}
void dump_scene(const fs::path& dir,const Scene& s){
    for(const auto& [name,values]:std::array<std::pair<const char*,const std::vector<V4>*>,2>{{{"planes",&s.planes},{"front",&s.front}}})dump_array(dir/(std::string("collision-")+name+".npy"),*values,"<f8");
    for(const auto& [name,values]:std::array<std::pair<const char*,const std::vector<V3>*>,4>{{{"lo",&s.lo},{"hi",&s.hi},{"blo",&s.blo},{"bhi",&s.bhi}}})dump_array(dir/(std::string("collision-")+name+".npy"),*values,"<f8");
    dump_array(dir/"collision-pflags.npy",s.pflags,"<i8");dump_array(dir/"collision-first.npy",s.first,"<i8");dump_array(dir/"collision-count.npy",s.count,"<i8");
    dump_array(dir/"collision-meta.npy",s.meta,"<i8");dump_array(dir/"collision-escape.npy",s.escape,"<i4");dump_array(dir/"collision-leaf.npy",s.leaf,"<i4");dump_array(dir/"collision-order.npy",s.order,"<i4");
}
void dump_sampling(const fs::path& dir,const SamplingArrays& a){
    dump_array(dir/"sampling-pos.npy",a.pos,"<f8");dump_array(dir/"sampling-normal.npy",a.normal,"<f8");dump_array(dir/"sampling-links.npy",a.links,"<i8");
    dump_array(dir/"sampling-parent.npy",a.parent,"<i8");dump_array(dir/"sampling-visited.npy",a.visited,"|u1");dump_array(dir/"sampling-nextxy.npy",a.nextxy,"<i8");
    dump_array(dir/"sampling-crouch.npy",a.crouch,"|b1");dump_array(dir/"sampling-blocked.npy",a.blocked,"|b1");dump_array(dir/"sampling-ground.npy",a.ground,"<f8");
    dump_array(dir/"sampling-obsh.npy",a.obsh,"<f8");dump_array(dir/"sampling-obsa.npy",a.obsa,"<f8");dump_array(dir/"sampling-obsb.npy",a.obsb,"<f8");
    dump_array(dir/"sampling-disp.npy",a.disp,"|b1");dump_array(dir/"sampling-owner.npy",a.owner,"<i8");
}
}
Json generate(const Options& o){
    const auto threads=o.threads?o.threads:logical_cores();configure_threads(threads);
    if(!std::isfinite(o.slope)||o.slope<=0||o.slope>1)throw Error("Slope limit must be in (0, 1]");
    auto total=now(),start=total;Json timings=Json::object();BSP bsp(o.map);
    auto candidate=o.seeds.value_or(fs::path(o.map).replace_extension(".navseeds.txt"));std::optional<fs::path> seedfile;
    if(o.seeds||fs::exists(candidate)){seedfile=candidate;progress("seeds: using "+path_utf8(candidate)+(o.seeds?" (explicit override)":" (automatic sidecar)"));}
    else progress("seeds: no sidecar found at "+path_utf8(candidate)+"; using BSP seeds");
    auto seeds=bsp.seeds(seedfile);Json seed_input=nullptr;
    if(seedfile)seed_input={{"path",path_utf8(fs::weakly_canonical(*seedfile))},{"sha256",sha256(read_file(*seedfile))},{"source",o.seeds?"explicit":"sidecar"}};
    timings["input"]=now()-start;progress("input: "+path_utf8(bsp.path.filename())+", "+std::to_string(bsp.data.size())+" bytes, "+std::to_string(seeds.size())+" seeds");
    auto dump=o.dump_stages;if(!dump.empty()){fs::create_directories(dump);dump_array(dump/"seeds.npy",seeds,"<f8");}
    start=now();auto collision=Collision::load(bsp,o.game,o.missing_phy);timings["collision"]=now()-start;
    if(!dump.empty()){dump_scene(dump,collision.scene);write_json(dump/"collision.json",{{"counts",collision.counts},{"unresolved",collision.unresolved},{"content_audit",collision.content_audit}});}
    auto effective=seeds;Json discovery=nullptr;std::vector<LadderSpec> sources;
    if(o.seed_discovery=="entities"){start=now();auto result=discover_seeds(bsp,collision,seeds,o.slope);effective=std::move(result.effective);discovery=std::move(result.details);sources=std::move(result.ladder_specs);timings["seed_discovery"]=now()-start;}
    if(!dump.empty())dump_array(dump/"effective-seeds.npy",effective,"<f8");
    const auto& model=bsp.models.at(0);std::array<V3,2> bounds{{{model[0],model[1],model[2]},{model[3],model[4],model[5]}}};
    start=now();auto sampled=sample(collision,effective,bounds,o.slope,threads);timings["sampling"]=now()-start;
    if(!dump.empty())dump_sampling(dump,sampled.arrays);
    areas_dump_directory=dump;
    auto areas=build_areas(collision,sampled.arrays,sampled.index,bsp.data.size(),o.slope,threads);timings.update(areas.timings);
    auto nav=std::move(areas.nav);Json link_audit=std::move(areas.link_audit),ladder_info=nullptr,merge_info=nullptr;
    if(!dump.empty()){write_json(dump/"sdk-nav.json",nav);write_file(dump/"sdk.nav",write_nav(nav));write_json(dump/"cleanup.json",areas.cleanup);write_json(dump/"link-audit.json",link_audit);}
    if(o.ladders){start=now();if(o.seed_discovery=="none")sources=ladder_specs(bsp).specs;ladder_info=build_ladders(nav,collision,sources,true,o.slope);timings["ladders"]=now()-start;}
    if(o.profile=="merged"){
        start=now();auto environment=merge_environment(bsp,collision,nav);
        merge_info=merge_nav(nav,environment.water,environment.locked_ids,.5,125.,&collision.scene,o.slope);merge_info["environment"]=environment.environment;
        timings["post_merge"]=now()-start;link_audit={{"sdk_before_merge",link_audit},{"merge",merge_info["link_validation"]}};
        progress("merged: "+std::to_string(merge_info["source_areas"].get<std::size_t>())+" -> "+std::to_string(nav["areas"].size())+" areas");
        if(!dump.empty())write_json(dump/"merged-nav.json",nav);
    }
    start=now();auto hiding=compute_hiding_spots(nav,collision,threads);timings["hiding_spots"]=now()-start;
    progress("hiding spots: "+std::to_string(hiding.at("total").get<std::size_t>())+" ("+std::to_string(hiding.at("in_cover").get<std::size_t>())+" cover, "+std::to_string(hiding.at("exposed").get<std::size_t>())+" exposed)");
    start=now();validate_nav(nav);auto raw=write_nav(nav);if(write_nav(read_nav(raw))!=raw)throw Error("Generated NAV round-trip failed");
    auto target=o.out.empty()?fs::path(bsp.path).replace_extension(".nav"):o.out;
    if(same_resolved_path(target,bsp.path)||(seedfile&&same_resolved_path(target,*seedfile)))throw Error("Output must not replace an input BSP or seed file");
    atomic_write(target,raw);timings["write_validate"]=now()-start;timings["total"]=now()-total;
    std::size_t links=0;for(const auto& a:nav["areas"])for(const auto& ls:a["links"])links+=ls.size();
    Json result={{"map",path_utf8(bsp.path)},{"map_sha256",sha256(bsp.data)},{"out",path_utf8(target)},{"nav_sha256",sha256(raw)},
        {"bsp_size",bsp.data.size()},{"nav_bytes",raw.size()},{"areas",nav["areas"].size()},{"links",links},{"nodes",sampled.arrays.size()},{"seeds",seeds.size()},
        {"traces",collision.scene.traces()},{"profile",o.profile},{"merge",merge_info},{"ladders",ladder_info},
        {"seed_discovery",{{"mode",o.seed_discovery},{"effective_seeds",effective},{"details",discovery}}},
        {"threads",threads},{"build_identifier",NAVGEN_BUILD_ID},{"sampling_speculation",collision.sampling_speculation},
        {"missing_phy_policy",o.missing_phy},{"slope_limit",o.slope},{"collision",collision.counts},{"unresolved",collision.unresolved},{"content_audit",collision.content_audit},
        {"cleanup",areas.cleanup},{"link_audit",link_audit},{"timings_seconds",timings},{"analyzed",nav["analyzed"]},{"hiding_spots",hiding},{"seed_file",seed_input},{"native_acceptance",false}};
    for(const auto& item:collision.unresolved)progress(item.get<std::string>());
    if(!dump.empty()){write_json(dump/"nav.json",nav);write_json(dump/"result.json",result);}
    progress("completed: "+std::to_string(nav["areas"].size())+" areas, "+std::to_string(links)+" directed links, "+std::to_string(timings["total"].get<double>())+"s, "+path_utf8(target));
    return result;
}
}
