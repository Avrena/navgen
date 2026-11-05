#include "areas.hpp"
#include "discovery.hpp"
#include "geometry.hpp"
#include "gma.hpp"
#include "hiding.hpp"
#include "ladders.hpp"
#include "merge.hpp"
#include "navio.hpp"
#include "planes.hpp"
#include "pyset.hpp"
#include <cfenv>

using namespace navgen;
namespace {
Scene scene_from(const Json& value){
    SceneBuilder builder;
    for(const auto& item:value.value("boxes",Json::array())){
        auto lo=item.at("lo").get<V3>(),hi=item.at("hi").get<V3>();
        builder.add(convex_planes(box_vertices(lo,hi)),lo,hi,item.value("contents",1LL),item.value("walkable",false),item.value("monster",false),item.value("transparent",false),item.value("push",false),item.value("kind",0));
    }
    for(const auto& item:value.value("objects",Json::array())){
        std::optional<std::vector<int64_t>> flags;if(item.contains("flags"))flags=item["flags"].get<std::vector<int64_t>>();std::optional<V4> front;if(item.contains("front"))front=item["front"].get<V4>();
        builder.add(item.at("planes").get<std::vector<V4>>(),item.at("lo").get<V3>(),item.at("hi").get<V3>(),item.value("contents",1LL),item.value("walkable",false),item.value("monster",false),item.value("transparent",false),item.value("push",false),item.value("kind",0),flags,front);
    }
    return builder.pack();
}
Json sampled_json(const SamplingArrays& a){return {{"pos",a.pos},{"normal",a.normal},{"links",a.links},{"parent",a.parent},{"visited",a.visited},{"nextxy",a.nextxy},{"crouch",a.crouch},{"blocked",a.blocked},{"ground",a.ground},{"obsh",a.obsh},{"obsa",a.obsa},{"obsb",a.obsb},{"disp",a.disp},{"owner",a.owner}};}
Json scene_json(const Scene& s){return {{"planes",s.planes},{"pflags",s.pflags},{"first",s.first},{"count",s.count},{"lo",s.lo},{"hi",s.hi},{"meta",s.meta},{"front",s.front},{"blo",s.blo},{"bhi",s.bhi},{"escape",s.escape},{"order",s.order},{"leaf",s.leaf}};}
Json bits(const std::vector<V4>& values){Json result=Json::array();for(const auto& row:values){Json r=Json::array();for(auto v:row){std::ostringstream text;text<<std::hex<<std::setw(16)<<std::setfill('0')<<std::bit_cast<uint64_t>(v);r.push_back(text.str());}result.push_back(r);}return result;}
}
int main(int argc,char** argv){
    std::fesetround(FE_TONEAREST);
    try{
        if(argc==4&&std::string(argv[1])=="nav-roundtrip"){auto data=read_file(argv[2]);auto nav=read_nav(data);auto output=write_nav(nav);write_file(argv[3],output);std::cout<<Json({{"identical",output==data},{"sha256",sha256(output)},{"areas",nav["areas"].size()}}).dump()<<'\n';return output==data?0:1;}
        Json request;std::cin>>request;const auto op=request.at("op").get<std::string>();configure_threads(request.value("threads",1));Json result;
        // Progress output is redirected so stdout remains a single JSON response.
        auto original=std::cout.rdbuf(std::cerr.rdbuf());
        if(op=="python_numbers"){
            result={{"floats",Json::array()},{"integers",Json::array()}};
            for(const auto& value:request.value("floats",Json::array())){
                try{const auto number=python_float(value.get<std::string>());std::ostringstream encoded;encoded<<std::hex<<std::setw(16)<<std::setfill('0')<<std::bit_cast<uint64_t>(number);result["floats"].push_back({{"bits",encoded.str()}});}
                catch(const Error& error){result["floats"].push_back({{"error",error.kind}});}
            }
            for(const auto& value:request.value("integers",Json::array())){
                try{const auto number=python_integer(value.get<std::string>());Json equal=Json::array();for(int64_t candidate:{-2,-1,0,1,2})equal.push_back(number.equals(candidate));result["integers"].push_back({{"low_bits",number.low_bits},{"nonzero",number.nonzero()},{"clamped",number.clamped()},{"equals",equal}});}
                catch(const Error& error){result["integers"].push_back({{"error",error.kind}});}
            }
        }
        else if(op=="convex"){auto p=convex_planes(request.at("points").get<std::vector<V3>>());result={{"planes",p},{"bits",bits(p)}};}
        else if(op=="triangle"){auto [p,f]=triangle_kernel(request.at("points").get<Tri>());result={{"planes",p},{"front",f},{"bits",bits(p)}};}
        else if(op=="bevel"){auto p=bevel_planes(request.at("points").get<std::vector<V3>>(),request.at("edges").get<std::vector<std::array<int64_t,2>>>());result={{"planes",p},{"bits",bits(p)}};}
        else if(op=="rotation"){result=rotation(request.at("angles").get<V3>());}
        else if(op=="vector"){result=vector(request.at("value").get<std::string>(),request.value("default",V3{}));}
        else if(op=="entity_angles"||op=="entity_rules"){
            Entity e;for(const auto& item:request.at("items"))e[item.at(0).get<std::string>()]=item.at(1).get<std::string>();
            if(op=="entity_angles")result=entity_angles(e);else{auto [active,walkable]=entity_rules(e);result=Json::array({active?Json(*active):Json(nullptr),walkable});}
        }
        else if(op=="phy"){result=parse_phy(read_file(path_from_utf8(request.at("path").get<std::string>())));}
        else if(op=="brush_vertices"){
            auto planes=request.at("planes").get<std::vector<V4>>();std::vector<BrushSide> sides(planes.size());auto bevel=request.value("bevel",std::vector<int>(planes.size()));
            for(size_t i=0;i<sides.size();++i)sides[i].bevel=static_cast<uint8_t>(bevel.at(i));
            result=brush_vertices(planes,sides);
        }else if(op=="collision"){
            BSP bsp(path_from_utf8(request.at("path").get<std::string>()));auto c=Collision::load(bsp,path_from_utf8(request.at("game").get<std::string>()),request.value("missing_phy",std::string("skip")),false);c.counts.erase("build_seconds");
            result={{"scene",scene_json(c.scene)},{"counts",c.counts},{"unresolved",c.unresolved},{"content_audit",c.content_audit}};
        }
        else if(op=="pyset"){
            PySet<std::array<int64_t,2>> set;for(const auto& action:request.at("actions")){
                auto kind=action.at("kind").get<std::string>();PySet<std::array<int64_t,2>> other;for(const auto& key:action.value("values",Json::array()))other.insert(key.get<std::array<int64_t,2>>());
                if(kind=="add")for(const auto& key:action.at("values"))set.insert(key.get<std::array<int64_t,2>>());
                else if(kind=="discard")for(const auto& key:action.at("values"))set.erase(key.get<std::array<int64_t,2>>());
                else if(kind=="copy")set=set.copy();else if(kind=="update")set.update(other);else if(kind=="difference")set=set-other;else if(kind=="intersection")set=set&other;else if(kind=="union")set=set|other;else throw Error("Unknown set action");
            }result=set.values();
        }else if(op=="navio"){auto nav=request.at("nav");auto bytes=write_nav(nav);result={{"sha256",sha256(bytes)},{"nav",read_nav(bytes)}};}
        else if(op=="bsp"){
            BSP bsp(path_from_utf8(request.at("path").get<std::string>()));Json tris=Json::array(),props=Json::array();for(const auto& d:bsp.displacements())tris.push_back({{"points",d.points},{"contents",d.contents},{"tags",d.tags}});for(const auto& p:bsp.static_props())props.push_back({{"name",p.name},{"solid",p.solid},{"origin",p.origin},{"angles",p.angles},{"scale",p.scale}});
            std::optional<fs::path> seedfile;if(request.contains("seeds"))seedfile=path_from_utf8(request.at("seeds").get<std::string>());
            result={{"version",bsp.version},{"entities",bsp.entities},{"models",bsp.models},{"seeds",bsp.seeds(seedfile)},{"displacements",tris},{"props",props}};
        }else if(op=="content"){
            BSP bsp(path_from_utf8(request.at("path").get<std::string>()));Content content(bsp,path_from_utf8(request.at("game").get<std::string>()));Json reads=Json::object();for(const auto& key:request.at("names")){auto name=key.get<std::string>();auto data=content.read(name);reads[name]=data?Json(sha256(*data)):Json(nullptr);}result={{"reads",reads},{"used",content.used},{"conflicts",content.conflicts},{"archive_errors",content.archive_errors},{"gma_archives",content.gma_archives}};
        }else if(op=="gma"){
            GMA gma(path_from_utf8(request.at("path").get<std::string>()));Json reads=Json::object();for(const auto& [name,entry]:gma.entries)reads[name]=sha256(gma.read(name));result=reads;
        }else if(op=="union_box"){auto box=union_box(request.at("a").get<Box>(),request.at("b").get<Box>(),request.value("tolerance",.5),request.value("max_size",800.));result=box?Json(*box):Json(nullptr);}
        else if(op=="native_portal"){auto [p,h]=native_portal(request.at("a").get<Box>(),request.at("b").get<Box>(),request.at("direction").get<int>());result=Json::array({p,h});}
        else if(op=="ladder_specs"){BSP bsp(path_from_utf8(request.at("path").get<std::string>()));auto specs=ladder_specs(bsp);Json list=Json::array();for(const auto& s:specs.specs)list.push_back({{"class",s.class_name},{"hammerid",s.hammerid},{"bottom",s.bottom},{"top",s.top},{"width",s.width},{"axis",s.axis<0?Json(nullptr):Json(s.axis)}});result={{"specs",list},{"skipped",specs.skipped}};}
        else{
            Collision collision;collision.scene=scene_from(request.value("scene",Json::object()));auto& scene=collision.scene;
            if(op=="trace"){
                result=Json::array();for(const auto& r:request.at("rays"))result.push_back(trace(scene,r.at("start").get<V3>(),r.at("end").get<V3>(),r.value("mins",V3{0,0,0}),r.value("maxs",V3{0,0,0}),r.value("mask",BRUSH_MASK),r.value("walk",true),r.value("brute",false)));
            }else if(op=="sample"||op=="areas"){
                auto sampled=sample(collision,request.at("seeds").get<std::vector<V3>>(),request.at("bounds").get<std::array<V3,2>>(),request.value("slope",.7),runtime_threads);
                if(op=="sample")result=sampled_json(sampled.arrays);
                else{auto areas=build_areas(collision,sampled.arrays,sampled.index,request.value("bsp_size",0ULL),request.value("slope",.7),runtime_threads);result={{"nav",areas.nav},{"cleanup",areas.cleanup},{"link_audit",areas.link_audit}};}
            }else if(op=="hiding"){auto nav=request.at("nav");auto stats=compute_hiding_spots(nav,collision,runtime_threads);result={{"nav",nav},{"stats",stats}};}
            else if(op=="merge"){auto nav=request.at("nav");auto stats=merge_nav(nav,{},request.value("locked_ids",std::vector<int64_t>{}),request.value("tolerance",.5),request.value("max_size",125.),request.contains("scene")?&scene:nullptr,request.value("slope",.7));stats.erase("seconds");result={{"nav",nav},{"stats",stats}};}
            else if(op=="ladders"){
                std::vector<LadderSpec> specs;for(const auto& s:request.at("specs")){LadderSpec spec;spec.class_name=s.value("class",std::string("info_ladder"));spec.hammerid=s.value("hammerid",Json(nullptr));spec.bottom=s.at("bottom").get<V3>();spec.top=s.at("top").get<V3>();spec.width=s.value("width",25.);spec.axis=s.contains("axis")&&!s["axis"].is_null()?s["axis"].get<int>():-1;spec.lo=s.value("lo",V3{});spec.hi=s.value("hi",V3{});specs.push_back(spec);}auto nav=request.at("nav");auto stats=build_ladders(nav,collision,specs,request.value("support_convention",true),request.value("slope",.55));result={{"nav",nav},{"stats",stats}};
            }else if(op=="geometry"){
                auto point=request.at("point").get<V3>();auto [good,supported]=support_origin(scene,point,request.value("slope",.55));result={{"supported",good},{"origin",supported}};
            }else throw Error("Unknown probe operation: "+op);
        }
        std::cout.rdbuf(original);std::cout<<result.dump()<<'\n';return 0;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
