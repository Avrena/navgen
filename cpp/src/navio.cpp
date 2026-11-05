#include "navio.hpp"

namespace navgen {
namespace {
struct Reader{
    std::span<const std::uint8_t> data;std::size_t cursor=0;
    template<class T>T get(){
        if(sizeof(T)>data.size()-cursor)throw Error("Truncated NAV record","error");
        std::uint64_t u=0;for(std::size_t i=0;i<sizeof(T);++i)u|=std::uint64_t(data[cursor++])<<(8*i);
        if constexpr(std::is_same_v<T,float>)return std::bit_cast<float>(static_cast<std::uint32_t>(u));
        else return static_cast<T>(u);
    }
    Json ids(std::uint32_t count){
        if(count>(data.size()-cursor)/4)throw Error("Truncated NAV connection list","error");
        Json out=Json::array();for(std::uint32_t i=0;i<count;++i)out.push_back(get<std::uint32_t>());return out;
    }
    Json floats(std::size_t count){Json out=Json::array();for(std::size_t i=0;i<count;++i)out.push_back(get<float>());return out;}
    Json bytes(std::size_t count){if(count>data.size()-cursor)throw Error("Truncated NAV string","error");Json out=Json::array();for(std::size_t i=0;i<count;++i)out.push_back(data[cursor++]);return out;}
};
struct Writer{
    Bytes data;
    template<class T>void put(T value){
        std::uint64_t u;
        if constexpr(std::is_same_v<T,float>)u=std::bit_cast<std::uint32_t>(value);else u=static_cast<std::uint64_t>(value);
        for(std::size_t i=0;i<sizeof(T);++i)data.push_back(static_cast<std::uint8_t>(u>>(8*i)));
    }
    template<class T>void integer(const Json& value){
        std::uint64_t number=0;
        if(value.is_boolean())number=value.get<bool>()?1:0;
        else if(value.is_number_unsigned())number=value.get<std::uint64_t>();
        else if(value.is_number_integer()){auto signed_value=value.get<std::int64_t>();if(signed_value<0)throw Error("argument out of range","error");number=static_cast<std::uint64_t>(signed_value);}
        else throw Error("required argument is not an integer","error");
        if(number>std::numeric_limits<T>::max())throw Error("argument out of range","error");put<T>(static_cast<T>(number));
    }
    template<class T>void count(std::size_t size){if(size>std::numeric_limits<T>::max())throw Error("argument out of range","error");put<T>(static_cast<T>(size));}
    void u32(const Json& value){integer<std::uint32_t>(value);}
    void u16(const Json& value){integer<std::uint16_t>(value);}
    void u8(const Json& value){integer<std::uint8_t>(value);}
    void f32(const Json& value){auto number=value.is_boolean()?(value.get<bool>()?1.:0.):value.get<double>();auto converted=static_cast<float>(number);if(std::isfinite(number)&&!std::isfinite(converted))throw Error("float too large to pack with f format","OverflowError");put<float>(converted);}
    void floats(const Json& values){for(const auto& value:values)f32(value);}
    void ids(const Json& values){count<std::uint32_t>(values.size());for(const auto& value:values)u32(value);}
    void bytes(const Json& values){for(const auto& value:values)u8(value);}
};
bool finite(const Json& values){for(const auto& v:values)if(!std::isfinite(v.get<double>()))return false;return true;}
}
Json read_nav(std::span<const std::uint8_t> bytes){
    Reader r{bytes};Json nav;nav["magic"]=r.get<std::uint32_t>();nav["version"]=r.get<std::uint32_t>();
    if(nav["magic"]!=0xFEEDFACEu||nav["version"]!=16)throw Error("Unsupported NAV magic/version","AssertionError");
    nav["subversion"]=r.get<std::uint32_t>();nav["bsp_size"]=r.get<std::uint32_t>();nav["analyzed"]=r.get<std::uint8_t>();
    nav["places"]=Json::array();auto places=r.get<std::uint16_t>();
    for(unsigned i=0;i<places;++i)nav["places"].push_back(r.bytes(r.get<std::uint16_t>()));
    nav["has_unnamed"]=r.get<std::uint8_t>();nav["areas"]=Json::array();auto count=r.get<std::uint32_t>();
    for(std::uint32_t i=0;i<count;++i){
        Json a;a["id"]=r.get<std::uint32_t>();a["attr"]=r.get<std::uint32_t>();a["nw"]=r.floats(3);a["se"]=r.floats(3);
        a["ne_z"]=r.get<float>();a["sw_z"]=r.get<float>();a["links"]=Json::array();
        for(int d=0;d<4;++d)a["links"].push_back(r.ids(r.get<std::uint32_t>()));
        a["hiding"]=Json::array();auto spots=r.get<std::uint8_t>();
        for(unsigned j=0;j<spots;++j){auto id=r.get<std::uint32_t>();auto pos=r.floats(3);auto flags=r.get<std::uint8_t>();a["hiding"].push_back(Json::array({id,pos,flags}));}
        a["encounters"]=Json::array();auto encounters=r.get<std::uint32_t>();
        for(std::uint32_t j=0;j<encounters;++j){
            Json head=Json::array();head.push_back(r.get<std::uint32_t>());head.push_back(r.get<std::uint8_t>());head.push_back(r.get<std::uint32_t>());head.push_back(r.get<std::uint8_t>());
            Json list=Json::array();auto n=r.get<std::uint8_t>();for(unsigned k=0;k<n;++k){auto id=r.get<std::uint32_t>();auto t=r.get<std::uint8_t>();list.push_back(Json::array({id,t}));}
            a["encounters"].push_back(Json::array({head,list}));
        }
        a["place"]=r.get<std::uint16_t>();a["ladders"]=Json::array();for(int d=0;d<2;++d)a["ladders"].push_back(r.ids(r.get<std::uint32_t>()));
        a["occupy"]=r.floats(2);a["light"]=r.floats(4);a["visible"]=Json::array();auto visible=r.get<std::uint32_t>();
        for(std::uint32_t j=0;j<visible;++j){auto id=r.get<std::uint32_t>();auto flags=r.get<std::uint8_t>();a["visible"].push_back(Json::array({id,flags}));}
        a["inherit_vis"]=r.get<std::uint32_t>();nav["areas"].push_back(std::move(a));
    }
    nav["ladders"]=Json::array();auto ladders=r.get<std::uint32_t>();
    for(std::uint32_t i=0;i<ladders;++i){Json l;l["id"]=r.get<std::uint32_t>();l["width"]=r.get<float>();l["top"]=r.floats(3);l["bottom"]=r.floats(3);l["length"]=r.get<float>();l["dir"]=r.get<std::uint32_t>();l["areas"]=r.ids(5);nav["ladders"].push_back(std::move(l));}
    nav["trailing"]=r.bytes(bytes.size()-r.cursor);return nav;
}
Bytes write_nav(const Json& nav){
    Writer w;for(auto key:{"magic","version","subversion","bsp_size"})w.u32(nav.at(key));w.u8(nav.at("analyzed"));
    w.count<std::uint16_t>(nav.at("places").size());
    for(const auto& s:nav.at("places")){w.count<std::uint16_t>(s.size());w.bytes(s);}
    w.u8(nav.at("has_unnamed"));w.count<std::uint32_t>(nav.at("areas").size());
    for(const auto& a:nav.at("areas")){
        w.u32(a.at("id"));w.u32(a.at("attr"));w.floats(a.at("nw"));w.floats(a.at("se"));w.f32(a.at("ne_z"));w.f32(a.at("sw_z"));
        for(const auto& links:a.at("links"))w.ids(links);
        w.count<std::uint8_t>(a.at("hiding").size());
        for(const auto& s:a.at("hiding")){w.u32(s[0]);w.floats(s[1]);w.u8(s[2]);}
        w.count<std::uint32_t>(a.at("encounters").size());
        for(const auto& e:a.at("encounters")){
            w.u32(e[0][0]);w.u8(e[0][1]);w.u32(e[0][2]);w.u8(e[0][3]);w.count<std::uint8_t>(e[1].size());
            for(const auto& s:e[1]){w.u32(s[0]);w.u8(s[1]);}
        }
        w.u16(a.at("place"));for(const auto& links:a.at("ladders"))w.ids(links);w.floats(a.at("occupy"));w.floats(a.at("light"));
        w.count<std::uint32_t>(a.at("visible").size());for(const auto& v:a.at("visible")){w.u32(v[0]);w.u8(v[1]);}w.u32(a.at("inherit_vis"));
    }
    w.count<std::uint32_t>(nav.at("ladders").size());
    for(const auto& l:nav.at("ladders")){w.u32(l.at("id"));w.f32(l.at("width"));w.floats(l.at("top"));w.floats(l.at("bottom"));w.f32(l.at("length"));w.u32(l.at("dir"));for(const auto& id:l.at("areas"))w.u32(id);}
    w.bytes(nav.at("trailing"));return std::move(w.data);
}
void validate_nav(const Json& nav){
    const auto& areas=nav.at("areas");std::set<std::uint32_t> ids,spots;std::map<std::uint32_t,const Json*> byid;
    for(const auto& a:areas){auto id=a.at("id").get<std::uint32_t>();ids.insert(id);byid[id]=&a;}
    if(ids.empty()||ids.size()!=areas.size())throw Error("Empty mesh or duplicate area IDs");
    for(const auto& a:areas){
        auto id=a.at("id").get<std::uint32_t>();
        if(!finite(a.at("nw"))||!finite(a.at("se"))||!std::isfinite(a.at("ne_z").get<double>())||!std::isfinite(a.at("sw_z").get<double>())||a.at("se")[0]<=a.at("nw")[0]||a.at("se")[1]<=a.at("nw")[1])throw Error("Invalid area geometry: "+std::to_string(id));
        for(const auto& links:a.at("links")){std::set<std::uint32_t> seen;for(const auto& v:links){auto other=v.get<std::uint32_t>();if(!seen.insert(other).second||other==id||!ids.contains(other))throw Error("Invalid connection from area "+std::to_string(id));}}
        for(const auto& s:a.at("hiding")){if(!spots.insert(s[0].get<std::uint32_t>()).second||!finite(s[1])||(s[2]!=1&&s[2]!=8))throw Error("Invalid hiding spot in area "+std::to_string(id));}
    }
    std::map<std::uint32_t,const Json*> ladders;
    for(const auto& l:nav.at("ladders"))if(!ladders.emplace(l.at("id").get<std::uint32_t>(),&l).second)throw Error("Duplicate ladder IDs");
    auto contains=[](const Json& array,std::uint32_t id){return std::find(array.begin(),array.end(),Json(id))!=array.end();};
    for(const auto& [id,p]:ladders){const auto& l=*p;const auto& refs=l.at("areas");bool bad=refs.size()!=5;
        if(!finite(l.at("top"))||!finite(l.at("bottom"))||!std::isfinite(l.at("width").get<double>())||!std::isfinite(l.at("length").get<double>())||l.at("width")<=0||l.at("length")<=0||l.at("top")[2]<=l.at("bottom")[2]||l.at("dir")<0||l.at("dir")>3)bad=true;
        if(!bad){bad=refs[4]==0||(refs[0]==0&&refs[1]==0&&refs[2]==0);for(const auto& r:refs)if(r!=0&&!ids.contains(r.get<std::uint32_t>()))bad=true;}
        if(bad)throw Error("Invalid ladder geometry or area references: "+std::to_string(id));
        if(!contains(byid.at(refs[4].get<std::uint32_t>())->at("ladders")[0],id))throw Error("Nonreciprocal ladder attachment: "+std::to_string(id));
        for(int i=0;i<4;++i)if(refs[i]!=0&&!contains(byid.at(refs[i].get<std::uint32_t>())->at("ladders")[1],id))throw Error("Nonreciprocal ladder attachment: "+std::to_string(id));
    }
    for(const auto& a:areas)for(int d=0;d<2;++d){std::set<std::uint32_t> seen;for(const auto& v:a.at("ladders")[d]){auto id=v.get<std::uint32_t>();if(!seen.insert(id).second||!ladders.contains(id))throw Error("Invalid area ladder reference");const auto& refs=ladders.at(id)->at("areas");bool found=false;for(int i=d==0?4:0;i<(d==0?5:4);++i)if(refs[i]==a.at("id"))found=true;if(!found)throw Error("Area refers to an unattached ladder");}}
}
}
