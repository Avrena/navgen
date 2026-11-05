#include "bsp.hpp"
#include "pyset.hpp"
#include "zlib.h"
extern "C" {
#include "LzmaDec.h"
#include "qhull_ra.h"
}
#include <cstdio>
#include <cstdlib>
#include <regex>
#include <unordered_set>

namespace navgen {
namespace {
template<class T>T integer(std::span<const uint8_t> d,size_t at){if(at>d.size()||sizeof(T)>d.size()-at)throw Error("Truncated binary structure");using U=std::make_unsigned_t<T>;U v=0;for(size_t k=0;k<sizeof(T);k++)v|=U(d[at+k])<<(8*k);return std::bit_cast<T>(v);}
float f32(std::span<const uint8_t> d,size_t at){return std::bit_cast<float>(integer<uint32_t>(d,at));}
V3 vec3(std::span<const uint8_t> d,size_t at){return {f32(d,at),f32(d,at+4),f32(d,at+8)};}
Bytes sliced(std::span<const uint8_t> d,size_t begin,size_t size){if(begin>d.size()||size>d.size()-begin)throw Error("Truncated binary payload");return Bytes(d.begin()+static_cast<ptrdiff_t>(begin),d.begin()+static_cast<ptrdiff_t>(begin+size));}
std::string cstring(std::span<const uint8_t> d,size_t& at){size_t begin=at;while(at<d.size()&&d[at])at++;if(at==d.size())throw Error("Truncated binary string");std::string out(reinterpret_cast<const char*>(d.data()+begin),at-begin);at++;return out;}
std::string lower_slashes(std::string s){s=unicode_lower(s);std::replace(s.begin(),s.end(),'\\','/');return s;}
std::string evalue(const Entity& e,const std::string& key,const std::string& fallback=""){auto it=e.find(key);return it==e.end()?fallback:it->second;}
std::vector<std::string> words(std::string_view text){auto points=unicode_codepoints(text);std::vector<std::string> out;size_t at=0;while(at<points.size()){while(at<points.size()&&unicode_whitespace(points[at]))++at;size_t begin=at;while(at<points.size()&&!unicode_whitespace(points[at]))++at;if(at>begin)out.push_back(unicode_utf8(std::span(points).subspan(begin,at-begin)));}return out;}
std::vector<std::string> lines(std::string_view text){auto points=unicode_codepoints(text);std::vector<std::string> out;size_t begin=0,at=0;while(at<points.size()){if(unicode_linebreak(points[at])){out.push_back(unicode_utf8(std::span(points).subspan(begin,at-begin)));if(points[at]==13&&at+1<points.size()&&points[at+1]==10)++at;begin=++at;}else ++at;}if(begin<points.size())out.push_back(unicode_utf8(std::span(points).subspan(begin)));return out;}
bool path_less(const fs::path& a,const fs::path& b){
    std::vector<std::string> left,right;for(const auto& part:a)left.push_back(path_utf8(part));for(const auto& part:b)right.push_back(path_utf8(part));
#ifdef _WIN32
    for(auto& part:left)part=unicode_lower(part);for(auto& part:right)part=unicode_lower(part);
#endif
    return left<right;
}
std::string glob_name(const fs::path& path){auto name=path_utf8(path.filename());
#ifdef _WIN32
    name=unicode_lower(name);
#endif
    return name;
}
void* lz_alloc(ISzAllocPtr,size_t n){return std::malloc(n);}void lz_free(ISzAllocPtr,void* p){std::free(p);}
Bytes raw_lzma(std::span<const uint8_t> packed,std::span<const uint8_t> props,size_t actual){
    Bytes out(actual);SizeT dest=out.size(),source=packed.size();ISzAlloc alloc={lz_alloc,lz_free};ELzmaStatus status;
    SRes r=LzmaDecode(out.data(),&dest,packed.data(),&source,props.data(),static_cast<unsigned>(props.size()),LZMA_FINISH_END,&status,&alloc);
    if(r!=SZ_OK||dest!=actual)throw Error("Invalid compressed BSP lump length");return out;
}
std::vector<fs::path> sorted_paths(const fs::path& directory,const std::function<bool(const fs::directory_entry&)>& accept){std::vector<fs::path> out;if(!fs::exists(directory))return out;for(const auto& e:fs::directory_iterator(directory))if(accept(e))out.push_back(e.path());std::stable_sort(out.begin(),out.end(),path_less);return out;}
}
V3 vector(const std::string& value,V3 fallback){if(value.empty())return fallback;auto parts=words(value);V3 out{};for(size_t i=0;i<std::min<size_t>(3,parts.size());i++)out[i]=python_float(parts[i]);return out;}
V3 entity_angles(const Entity& e){if(!evalue(e,"angles").empty())return vector(evalue(e,"angles"));double yaw=python_float(evalue(e,"angle","0"));if(yaw==-1)return {-90.,0.,0.};if(yaw==-2)return {90.,0.,0.};return {0.,yaw,0.};}
M3 rotation(const V3& angles){constexpr double factor=3.141592653589793238462643383279502884/180.;double p=angles[0]*factor,y=angles[1]*factor,r=angles[2]*factor;double cp=std::cos(p),sp=std::sin(p),cy=std::cos(y),sy=std::sin(y),cr=std::cos(r),sr=std::sin(r);return {V3{cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr},V3{sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr},V3{-sp,cp*sr,cp*cr}};}
std::vector<V3> box_vertices(const V3& lo,const V3& hi){std::vector<V3> out;out.reserve(8);for(int x=0;x<2;x++)for(int y=0;y<2;y++)for(int z=0;z<2;z++)out.push_back({x?hi[0]:lo[0],y?hi[1]:lo[1],z?hi[2]:lo[2]});return out;}
BSP::BSP(const fs::path& p):path(p),data(read_file(p)){
    if(data.size()<1036||std::string(data.begin(),data.begin()+4)!="VBSP")throw Error("Not a Source BSP");version=integer<uint32_t>(data,4);if(version!=20)throw Error("Expected BSP v20, got "+std::to_string(version));for(int i=0;i<64;i++)for(int j=0;j<4;j++)lumps[i][j]=integer<uint32_t>(data,8+16*i+4*j);
    auto ent=lump(0);std::string entity_text=latin1_utf8(std::string_view(reinterpret_cast<const char*>(ent.data()),ent.size()));std::regex block(R"(\{([^{}]*)\})"),kv("\"([^\"\\n]*)\"\\s+\"([^\"\\n]*)\"");
    for(auto i=std::sregex_iterator(entity_text.begin(),entity_text.end(),block);i!=std::sregex_iterator();++i){Entity e;auto text=(*i)[1].str();for(auto j=std::sregex_iterator(text.begin(),text.end(),kv);j!=std::sregex_iterator();++j)e[(*j)[1].str()]=(*j)[2].str();entities.push_back(std::move(e));}
    auto pl=lump(1);if(pl.size()%20)throw Error("Invalid BSP planes length");for(size_t at=0;at<pl.size();at+=20)planes.push_back({f32(pl,at),f32(pl,at+4),f32(pl,at+8),f32(pl,at+12)});
    auto br=lump(18);if(br.size()%12)throw Error("Invalid BSP brushes length");for(size_t at=0;at<br.size();at+=12)brushes.push_back({integer<int32_t>(br,at),integer<int32_t>(br,at+4),integer<uint32_t>(br,at+8)});
    auto si=lump(19);if(si.size()%8)throw Error("Invalid BSP brush sides length");for(size_t at=0;at<si.size();at+=8)sides.push_back({integer<uint16_t>(si,at),integer<int16_t>(si,at+2),integer<int16_t>(si,at+4),si[at+6],si[at+7]});
    auto mo=lump(14);if(mo.size()%48)throw Error("Invalid BSP models length");for(size_t at=0;at<mo.size();at+=48){std::array<double,12> m;for(int a=0;a<9;a++)m[a]=f32(mo,at+a*4);for(int a=9;a<12;a++)m[a]=integer<int32_t>(mo,at+a*4);models.push_back(m);}
    nodes=lump(5);leaves=lump(10);leaf_stride=lumps[10][2]==1?32:56;auto lb=lump(17);for(size_t at=0;at<lb.size();at+=2)leafbrushes.push_back(integer<uint16_t>(lb,at));texinfo=lump(6);faces=lump(7);if(faces.empty())faces=lump(58);
    auto vs=lump(3);if(vs.size()%12)throw Error("Invalid BSP vertices length");for(size_t at=0;at<vs.size();at+=12)vertices.push_back(vec3(vs,at));auto ed=lump(12);for(size_t at=0;at<ed.size();at+=4)edges.push_back({integer<uint16_t>(ed,at),integer<uint16_t>(ed,at+2)});auto se=lump(13);for(size_t at=0;at<se.size();at+=4)surfedges.push_back(integer<int32_t>(se,at));
}
Bytes BSP::lump(int i)const{auto [off,size,ver,cc]=lumps.at(i);if(uint64_t(off)+size>data.size())throw Error("Truncated BSP lump "+std::to_string(i));auto out=sliced(data,off,size);if(out.size()>=4&&std::string(out.begin(),out.begin()+4)=="LZMA"){auto actual=integer<uint32_t>(out,4),packed=integer<uint32_t>(out,8);if(out.size()<17||packed>out.size()-17)throw Error("Truncated compressed BSP lump");return raw_lzma(std::span(out).subspan(17,packed),std::span(out).subspan(12,5),actual);}return out;}
std::vector<int> BSP::model_brushes(int model)const{int64_t index=model;if(index<0)index+=static_cast<int64_t>(models.size());std::vector<int> todo={static_cast<int>(models.at(static_cast<size_t>(index))[9])};std::set<int> visited,found;while(!todo.empty()){int node=todo.back();todo.pop_back();if(!visited.insert(node).second)continue;if(node>=0){todo.push_back(integer<int32_t>(nodes,size_t(node)*32+4));todo.push_back(integer<int32_t>(nodes,size_t(node)*32+8));}else{size_t at=size_t(-1-int64_t(node))*leaf_stride+24;auto first=integer<uint16_t>(leaves,at),count=integer<uint16_t>(leaves,at+2);for(size_t j=first;j<std::min(size_t(first)+count,leafbrushes.size());j++)found.insert(leafbrushes[j]);}}return {found.begin(),found.end()};}
BrushData BSP::brush(int index)const{
    const auto& b=brushes.at(index);BrushData out;double inf=std::numeric_limits<double>::infinity();out.lo={-inf,-inf,-inf};out.hi={inf,inf,inf};
    for(int64_t i=b.first;i<int64_t(b.first)+b.count;i++){const auto& s=sides.at(static_cast<size_t>(i));out.sides.push_back(s);out.planes.push_back(planes.at(s.plane));out.flags.push_back(s.tex>=0?integer<uint32_t>(texinfo,size_t(s.tex)*72+64):0);}
    for(int axis=0;axis<3;axis++){double plus=inf,minus=inf;for(const auto& p:out.planes){bool axial=true;for(int a=0;a<3;a++)if(a!=axis&&!(std::abs(p[a])<1e-7))axial=false;if(axial&&p[axis]>.99999)plus=std::min(plus,p[3]);if(axial&&p[axis]<-.99999)minus=std::min(minus,p[3]);}out.hi[axis]=plus;out.lo[axis]=-minus;}
    for(int a=0;a<3;a++)if(!std::isfinite(out.lo[a])||!std::isfinite(out.hi[a]))throw Error("Brush "+std::to_string(index)+" has no axial bounds");return out;
}
std::pair<std::vector<V3>,V3> BSP::face_vertices(int index)const{size_t at=size_t(index)*56;auto pn=integer<uint16_t>(faces,at);uint8_t side=faces.at(at+2);auto first=integer<int32_t>(faces,at+4);auto count=integer<int16_t>(faces,at+8);std::vector<V3> points;for(int64_t i=first;i<int64_t(first)+count;i++){int se=surfedges.at(static_cast<size_t>(i));const auto& ed=edges.at(static_cast<size_t>(se<0?-int64_t(se):se));points.push_back(vertices.at(ed[se<0]));}const auto& plane=planes.at(pn);V3 normal={plane[0],plane[1],plane[2]};normal*=side?-1.:1.;return {points,normal};}
std::vector<Displacement> BSP::displacements()const{
    auto info=lump(26),verts=lump(33),tags=lump(48);std::vector<Displacement> out;
    for(size_t off=0;off<info.size();off+=176){auto start=vec3(info,off);int vi=integer<int32_t>(info,off+12),ti=integer<int32_t>(info,off+16),power=integer<int32_t>(info,off+20);int contents=integer<int32_t>(info,off+32);auto face=integer<uint16_t>(info,off+36);auto [corners,normal]=face_vertices(face);if(corners.size()!=4||power<2||power>4)throw Error("Invalid displacement base face/power");
        size_t nearest=0;double nearest_dist=axis_norm3(corners[0]-start);for(size_t i=1;i<4;i++){double d=axis_norm3(corners[i]-start);if(d<nearest_dist){nearest=i;nearest_dist=d;}}std::rotate(corners.begin(),corners.begin()+static_cast<ptrdiff_t>(nearest),corners.end());
        int n=(1<<power)+1;std::vector<V3> points(size_t(n)*n);double step=1./(n-1);
        for(int yy=0;yy<n;yy++)for(int xx=0;xx<n;xx++){double x=xx*step,y=yy*step;size_t i=size_t(yy)*n+xx;for(int a=0;a<3;a++){
            double p=(((1.-x)*(1.-y))*corners[0][a]+(x*(1.-y))*corners[3][a])+(x*y)*corners[2][a];p+=((1.-x)*y)*corners[1][a];float displacement=f32(verts,(size_t(vi)+i)*20+a*4)*f32(verts,(size_t(vi)+i)*20+12);points[i][a]=p+double(displacement);
        }}
        int k=ti;for(int yy=0;yy<n-1;yy++)for(int xx=0;xx<n-1;xx++){int a=yy*n+xx;std::array<std::array<int,3>,2> pairs=(xx+yy)&1?std::array{std::array{a,a+n,a+1},std::array{a+1,a+n,a+n+1}}:std::array{std::array{a,a+n,a+n+1},std::array{a,a+n+1,a+1}};for(const auto& ids:pairs){auto tag=integer<uint16_t>(tags,size_t(k++)*2);if(!(tag&32))out.push_back({Tri{points[ids[0]],points[ids[2]],points[ids[1]]},contents,tag});}}
    }return out;
}
std::vector<StaticProp> BSP::static_props()const{
    auto gl=lump(35);std::vector<StaticProp> out;if(gl.empty())return out;int nlumps=integer<int32_t>(gl,0);
    for(int i=0;i<nlumps;i++){size_t at=4+size_t(i)*16;auto gid=integer<uint32_t>(gl,at),off=integer<uint32_t>(gl,at+8),size=integer<uint32_t>(gl,at+12);auto flags=integer<uint16_t>(gl,at+4),ver=integer<uint16_t>(gl,at+6);if(gid!=0x73707270)continue;if(flags&1)throw Error("Compressed static-prop game lump unsupported");auto d=sliced(data,off,size);int count=integer<int32_t>(d,0);std::vector<std::string> names;
        for(int k=0;k<count;k++){size_t pos=4+size_t(k)*128;auto raw=sliced(d,pos,128);auto end=std::find(raw.begin(),raw.end(),0);names.push_back(lower_slashes(latin1_utf8(std::string(raw.begin(),end))));}
        size_t o=4+size_t(count)*128;int leaves_count=integer<int32_t>(d,o);o+=4+size_t(leaves_count)*2;count=integer<int32_t>(d,o);o+=4;if(count==0)return out;if(o>d.size()||count<0)throw Error("Unsupported static-prop record size");size_t stride=(d.size()-o)/size_t(count);if((d.size()-o)%size_t(count)||stride<56)throw Error("Unsupported static-prop record size");
        for(int k=0;k<count;k++){size_t pos=o+size_t(k)*stride;auto org=vec3(d,pos),angles=vec3(d,pos+12);auto name=names.at(integer<uint16_t>(d,pos+24));int solid=d.at(pos+30);double scale=ver==11&&stride>=84?f32(d,pos+80):1.;out.push_back({name,solid,org,angles,scale});}
    }return out;
}
std::vector<V3> BSP::seeds(const std::optional<fs::path>& file)const{
    std::set<std::string> targets;for(const auto& e:entities)if(evalue(e,"classname")=="trigger_teleport")targets.insert(evalue(e,"target"));std::vector<V3> out;
    for(const auto& e:entities){auto cls=evalue(e,"classname"),targetname=evalue(e,"targetname");if(cls=="info_player_start"||(!targetname.empty()&&targets.count(targetname)))out.push_back(vector(evalue(e,"origin")));}
    if(file){auto bytes=read_file(*file);size_t at=bytes.size()>=3&&bytes[0]==0xef&&bytes[1]==0xbb&&bytes[2]==0xbf?3:0;std::string text(bytes.begin()+static_cast<ptrdiff_t>(at),bytes.end());int lineno=0;for(auto line:lines(text)){lineno++;auto comment=line.find('#');if(comment!=std::string::npos)line.resize(comment);auto parts=words(line);if(parts.empty())continue;auto p=vector(line);if(parts.size()!=3||!std::all_of(p.begin(),p.end(),[](double x){return std::isfinite(x);}))throw Error("Invalid seed at line "+std::to_string(lineno));out.push_back(p);}}
    if(out.empty())throw Error("No info_player_start, teleport destination or supplied seed");std::sort(out.begin(),out.end());out.erase(std::unique(out.begin(),out.end()),out.end());return out;
}
namespace {
class ZipArchive {
public:
    struct Entry {uint16_t method,flags;uint32_t crc;uint64_t packed,size,offset;};
    Bytes bytes;std::map<std::string,Entry> entries;
    explicit ZipArchive(Bytes data):bytes(std::move(data)){
        if(bytes.empty())return;size_t end=std::string::npos;
        if(bytes.size()>=22)for(size_t at=bytes.size()-22;;--at){if(integer<uint32_t>(bytes,at)==0x06054b50){end=at;break;}if(at==0||bytes.size()-at>65557)break;}
        if(end==std::string::npos)throw Error("File is not a zip file","BadZipFile");uint64_t count=integer<uint16_t>(bytes,end+10),central=integer<uint32_t>(bytes,end+16);
        if(count==0xffff||central==0xffffffffu){if(end<20||integer<uint32_t>(bytes,end-20)!=0x07064b50)throw Error("Invalid ZIP64 locator","BadZipFile");auto zoff=integer<uint64_t>(bytes,end-12);if(integer<uint32_t>(bytes,size_t(zoff))!=0x06064b50)throw Error("Invalid ZIP64 directory","BadZipFile");count=integer<uint64_t>(bytes,size_t(zoff)+32);central=integer<uint64_t>(bytes,size_t(zoff)+48);}
        size_t at=static_cast<size_t>(central);
        for(uint64_t i=0;i<count;i++){
            if(integer<uint32_t>(bytes,at)!=0x02014b50)throw Error("Bad magic number for central directory","BadZipFile");Entry e;e.flags=integer<uint16_t>(bytes,at+8);e.method=integer<uint16_t>(bytes,at+10);e.crc=integer<uint32_t>(bytes,at+16);e.packed=integer<uint32_t>(bytes,at+20);e.size=integer<uint32_t>(bytes,at+24);e.offset=integer<uint32_t>(bytes,at+42);
            size_t nl=integer<uint16_t>(bytes,at+28),el=integer<uint16_t>(bytes,at+30),cl=integer<uint16_t>(bytes,at+32);auto raw=sliced(bytes,at+46,nl);std::string name(raw.begin(),raw.end());if(!(e.flags&0x800))name=cp437_utf8(name);size_t extra=at+46+nl,limit=extra+el;
            while(extra+4<=limit){auto kind=integer<uint16_t>(bytes,extra),length=integer<uint16_t>(bytes,extra+2);size_t field=extra+4;if(kind==1){if(e.size==0xffffffffu){e.size=integer<uint64_t>(bytes,field);field+=8;}if(e.packed==0xffffffffu){e.packed=integer<uint64_t>(bytes,field);field+=8;}if(e.offset==0xffffffffu)e.offset=integer<uint64_t>(bytes,field);}extra+=4+length;}
            entries[lower_slashes(name)]=e;at+=46+nl+el+cl;
        }
    }
    Bytes read(const std::string& name)const{
        const auto& e=entries.at(name);if(e.flags&1)throw Error("File is encrypted, password required","RuntimeError");size_t at=static_cast<size_t>(e.offset);if(integer<uint32_t>(bytes,at)!=0x04034b50)throw Error("Bad magic number for file header","BadZipFile");size_t pos=at+30+integer<uint16_t>(bytes,at+26)+integer<uint16_t>(bytes,at+28);auto packed=sliced(bytes,pos,static_cast<size_t>(e.packed));Bytes out;
        if(e.method==0)out=std::move(packed);
        else if(e.method==8){out.resize(std::max<size_t>(static_cast<size_t>(e.size),1));z_stream z{};if(inflateInit2(&z,-15)!=Z_OK)throw Error("Cannot initialise deflate decoder");z.next_in=packed.data();z.avail_in=static_cast<uInt>(packed.size());z.next_out=out.data();z.avail_out=static_cast<uInt>(out.size());int code=inflate(&z,Z_FINISH);size_t total=z.total_out;inflateEnd(&z);if(code!=Z_STREAM_END||total!=e.size)throw Error("Invalid deflated ZIP entry","BadZipFile");out.resize(static_cast<size_t>(e.size));}
        else if(e.method==14){auto propsize=integer<uint16_t>(packed,2);if(packed.size()<size_t(4)+propsize)throw Error("Invalid ZIP LZMA header","BadZipFile");out=raw_lzma(std::span(packed).subspan(4+propsize),std::span(packed).subspan(4,propsize),static_cast<size_t>(e.size));}
        else throw Error("That compression method is not supported","NotImplementedError");
        if(out.size()!=e.size)throw Error("Bad uncompressed size for file '"+name+"'","BadZipFile");uLong crc=crc32(0L,Z_NULL,0);size_t off=0;while(off<out.size()){auto n=std::min(out.size()-off,size_t(std::numeric_limits<uInt>::max()));crc=crc32(crc,out.data()+off,static_cast<uInt>(n));off+=n;}if(crc!=e.crc)throw Error("Bad CRC-32 for file '"+name+"'","BadZipFile");return out;
    }
};
class VPKArchive {
public:
    struct Entry {Bytes preload;uint16_t archive;uint64_t offset;uint32_t length;};fs::path path;std::map<std::string,Entry> entries;
    explicit VPKArchive(const fs::path& p):path(p){
        auto d=read_file(p);if(integer<uint32_t>(d,0)!=0x55aa1234)throw Error("File is not VPK (invalid magic)");auto ver=integer<uint32_t>(d,4),tree=integer<uint32_t>(d,8);size_t header=ver==1?12:ver==2?28:0;if(!header)throw Error("Invalid header, or unsupported version");size_t at=header;
        if(ver==2){if(integer<uint32_t>(d,20)!=48)throw Error("Self hashes section size mismatch","AssertionError");uint64_t hashes=uint64_t(header)+tree+integer<uint32_t>(d,12)+integer<uint32_t>(d,16);if(hashes>d.size()||d.size()-hashes<48)throw Error("unpack requires a buffer of 48 bytes","error");}
        for(;;){if(at>size_t(tree)+header)throw Error("Error parsing index (out of bounds)");auto ext=cstring(d,at);if(ext.empty())break;for(;;){auto dir=cstring(d,at);if(dir.empty())break;dir=dir==" "?"":dir+"/";for(;;){auto name=cstring(d,at);if(name.empty())break;auto preload=integer<uint16_t>(d,at+4),archive=integer<uint16_t>(d,at+6);uint64_t offset=integer<uint32_t>(d,at+8);auto length=integer<uint32_t>(d,at+12);if(integer<uint16_t>(d,at+16)!=0xffff)throw Error("Error while parsing index");at+=18;if(archive==0x7fff)offset+=header+tree;auto bytes=sliced(d,at,preload);at+=preload;entries[unicode_lower(dir+name+"."+ext)]={std::move(bytes),archive,offset,length};}}}
    }
    Bytes read(const std::string& name)const{const auto& e=entries.at(name);auto target=path;if(e.archive!=0x7fff){std::string s=path_utf8(target);size_t at;while((at=s.find("english"))!=std::string::npos)s.erase(at,7);std::ostringstream replacement;replacement<<std::setw(3)<<std::setfill('0')<<e.archive<<'.';at=0;while((at=s.find("dir.",at))!=std::string::npos){s.replace(at,4,replacement.str());at+=replacement.str().size();}target=path_from_utf8(s);}
        std::ifstream f(target,std::ios::binary);if(!f)throw Error("Cannot read "+path_utf8(target),"FileNotFoundError");f.seekg(static_cast<std::streamoff>(e.offset));Bytes out=e.preload;size_t start=out.size();out.resize(start+e.length);if(e.length){f.read(reinterpret_cast<char*>(out.data()+start),e.length);out.resize(start+static_cast<size_t>(f.gcount()));}return out;}
};
}
struct Content::Impl {struct Source {std::string kind,label;fs::path path;std::shared_ptr<GMA> gma;std::shared_ptr<VPKArchive> vpk;};std::shared_ptr<ZipArchive> pak;std::vector<Source> sources;};
Content::Content(const BSP& bsp,const fs::path& p):impl(std::make_shared<Impl>()),game(p){
    auto pak=bsp.lump(40);if(!pak.empty())impl->pak=std::make_shared<ZipArchive>(std::move(pak));auto addons=game/"addons";
    for(const auto& path:sorted_paths(addons,[](const fs::directory_entry& e){return e.is_directory();}))impl->sources.push_back({"folder",python_path_text(path),path,{},{}});
    std::optional<fs::path> steamapps;for(auto parent=game.parent_path();!parent.empty();){if(lower_slashes(path_utf8(parent.filename()))=="steamapps"){steamapps=parent;break;}auto next=parent.parent_path();if(next==parent)break;parent=next;}
    auto archives=sorted_paths(addons,[](const fs::directory_entry& e){return glob_name(e.path()).ends_with(".gma");});if(steamapps){auto workshop=*steamapps/"workshop"/"content"/"4000";for(const auto& dir:sorted_paths(workshop,[](const fs::directory_entry& e){return e.is_directory();})){auto more=sorted_paths(dir,[](const fs::directory_entry& e){return glob_name(e.path()).ends_with(".gma");});archives.insert(archives.end(),more.begin(),more.end());}}std::stable_sort(archives.begin(),archives.end(),path_less);
    for(const auto& path:archives){try{auto archive=std::make_shared<GMA>(path);impl->sources.push_back({"gma",python_path_text(path),path,archive,{}});gma_archives++;}catch(const std::exception& e){archive_errors.push_back(e.what());}}
    auto engine=game.parent_path()/"sourceengine";std::vector<fs::path> paths={game/"garrysmod_dir.vpk"};std::set<std::string> known={"hl2_misc_dir.vpk","content_hl2_dir.vpk","content_cstrike_dir.vpk"};for(const auto& n:{"hl2_misc_dir.vpk","content_hl2_dir.vpk","content_cstrike_dir.vpk"})paths.push_back(engine/n);
    for(const auto& path:sorted_paths(engine,[](const fs::directory_entry& e){return glob_name(e.path()).ends_with("_dir.vpk");}))if(!known.count(path_utf8(path.filename())))paths.push_back(path);
    for(const auto& path:paths)if(fs::is_regular_file(path))impl->sources.push_back({"vpk",python_path_text(path),path,{},std::make_shared<VPKArchive>(path)});
    for(const auto& path:{game,engine,game/"download"})impl->sources.push_back({"folder",python_path_text(path),path,{},{}});
}
std::optional<Bytes> Content::read(std::string name){
    name=content_path(std::move(name));std::optional<Bytes> selected;std::string selected_source,selected_hash;Json alternatives=Json::array();
    if(impl->pak&&impl->pak->entries.count(name)){selected=impl->pak->read(name);selected_source="BSP pakfile";}
    for(const auto& source:impl->sources){std::optional<Bytes> data;
        if(source.kind=="folder"){auto p=source.path/path_from_utf8(name);if(fs::is_regular_file(p))data=read_file(p);}
        else if(source.kind=="gma"&&source.gma->entries.count(name))data=source.gma->read(name);
        else if(source.kind=="vpk"&&source.vpk->entries.count(name))data=source.vpk->read(name);
        if(!data)continue;if(!selected){selected=std::move(data);selected_source=source.label;}
        else if(*data!=*selected){if(selected_hash.empty())selected_hash=sha256(*selected);alternatives.push_back({{"source",source.label},{"sha256",sha256(*data)},{"bytes",data->size()}});}
    }
    if(selected){used[selected_source]++;if(!alternatives.empty())conflicts.push_back({{"path",name},{"selected",selected_source},{"sha256",selected_hash},{"bytes",selected->size()},{"different_sources",alternatives}});return selected;}
    missing.insert(name);return std::nullopt;
}
std::vector<std::vector<V3>> parse_phy(const Bytes& data){
    int hsize=integer<int32_t>(data,0),nsolids=integer<int32_t>(data,8);size_t o=static_cast<size_t>(hsize);std::vector<std::vector<V3>> pieces;
    for(int s=0;s<nsolids;s++){int size=integer<int32_t>(data,o);size_t start=o+4;if(size<=0||start>data.size()||size_t(size)>data.size()-start)throw Error("Truncated PHY solid");size_t surf=start;
        if(std::string(data.begin()+static_cast<ptrdiff_t>(start),data.begin()+static_cast<ptrdiff_t>(start+4))=="VPHY"){auto kind=integer<int16_t>(data,start+6);if(kind!=0)throw Error("Unsupported PHY surface type "+std::to_string(kind));surf=start+28;}
        int tree=integer<int32_t>(data,surf+32);size_t lo=surf+48,end=surf+static_cast<size_t>(tree);
        while(lo<end){int point_off=integer<int32_t>(data,lo);auto flags=integer<uint32_t>(data,lo+8);auto ntri=integer<int16_t>(data,lo+12);if(ntri<=0||point_off<=0)throw Error("Invalid PHY ledge");end=std::min(end,lo+static_cast<size_t>(point_off));if((flags&3)==0){std::set<uint32_t> ids;for(int t=0;t<ntri;t++)for(int e=0;e<3;e++)ids.insert(integer<uint32_t>(data,lo+20+size_t(t)*16+e*4)&0xffff);std::vector<V3> points;for(auto i:ids){auto p=vec3(data,lo+size_t(point_off)+16*size_t(i));points.push_back({p[0]/.0254,p[2]/.0254,(p[1]*-1.)/.0254});}if(points.empty())throw Error("Invalid PHY convex points");for(const auto& p:points)for(double x:p)if(!std::isfinite(x))throw Error("Invalid PHY convex points");pieces.push_back(std::move(points));}lo+=16+size_t(ntri)*16;}
        o=start+static_cast<size_t>(size);
    }if(pieces.empty())throw Error("PHY contains no convex pieces");return pieces;
}
namespace {
int matrix_rank(const std::vector<V3>& points){
    // One-sided Jacobi SVD avoids squaring the condition number of flat ledges.
    std::vector<std::array<double,3>> a;for(const auto& p:points)a.push_back({p[0],p[1],p[2]});
    for(int sweep=0;sweep<64;sweep++){bool changed=false;for(int p=0;p<2;p++)for(int q=p+1;q<3;q++){
        double aa=0,bb=0,ab=0;for(const auto& row:a){aa+=row[p]*row[p];bb+=row[q]*row[q];ab+=row[p]*row[q];}
        if(aa==0||bb==0||std::abs(ab)<=std::numeric_limits<double>::epsilon()*std::sqrt(aa*bb)*4)continue;
        double tau=(bb-aa)/(2*ab);double t=std::copysign(1.,tau)/(std::abs(tau)+std::sqrt(1+tau*tau));double c=1/std::sqrt(1+t*t),s=c*t;
        for(auto& row:a){auto x=row[p],y=row[q];row[p]=c*x-s*y;row[q]=s*x+c*y;}changed=true;
    }if(!changed)break;}
    int rank=0;for(int j=0;j<3;j++){double n=0;for(const auto& row:a)n+=row[j]*row[j];if(std::sqrt(n)>1e-7)rank++;}return rank;
}
M3 flat_basis(const std::vector<V3>& centered,int rank){
    const M3 eye={V3{1.,0.,0.},V3{0.,1.,0.},V3{0.,0.,1.}};if(rank==0)return eye;V3 best{};double score=-1.;
    auto consider=[&](const V3& candidate){double length2=dot3(candidate,candidate);if(length2>score){best=candidate;score=length2;}};
    if(rank==2){for(size_t i=0;i<centered.size();i++)for(size_t j=i+1;j<centered.size();j++)consider(cross(centered[i],centered[j]));}else for(const auto& row:centered)consider(row);
    int largest=0;for(int j=1;j<3;j++)if(std::abs(best[j])>std::abs(best[largest]))largest=j;if(best[largest]<0)best=-best;best/=std::sqrt(dot3(best,best));
    int smallest=0;for(int j=1;j<3;j++)if(std::abs(best[j])<std::abs(best[smallest]))smallest=j;V3 first=cross(eye[smallest],best);first/=std::sqrt(dot3(first,first));V3 second=cross(best,first);
    return rank==2?M3{first,second,best}:M3{best,first,second};
}
struct Hull {
    qhT qh{};FILE* messages;std::vector<double> points;bool initialised=false;
    Hull(const std::vector<V3>& source,int dimension,const M3* basis=nullptr):messages(std::tmpfile()){
        if(source.empty())throw Error("No points given");points.reserve(source.size()*size_t(dimension));for(const auto& p:source){if(dimension==3)points.insert(points.end(),p.begin(),p.end());else{points.push_back(dot3(p,(*basis)[0]));points.push_back(dot3(p,(*basis)[1]));}}
        FILE* errors=messages?messages:stderr;qh_zero(&qh,errors);initialised=true;char options[]="qhull i Qt";int code=qh_new_qhull(&qh,dimension,static_cast<int>(source.size()),points.data(),False,options,nullptr,errors);
        if(code){std::string text="Qhull failed";if(messages){std::fflush(messages);std::rewind(messages);std::array<char,4096> b{};text.clear();while(size_t n=std::fread(b.data(),1,b.size(),messages))text.append(b.data(),n);}cleanup();throw Error(text,"QhullError");}qh_triangulate(&qh);
    }
    void cleanup(){if(initialised){int curlong=0,totlong=0;qh_freeqhull(&qh,qh_ALL);qh_memfreeshort(&qh,&curlong,&totlong);initialised=false;}if(messages){std::fclose(messages);messages=nullptr;}}
    ~Hull(){cleanup();}
    Hull(const Hull&)=delete;Hull& operator=(const Hull&)=delete;
    std::vector<int> vertices2(){std::vector<int> out;qh.visit_id++;qh.vertex_visit++;facetT* facet=qh.facet_list;facetT* start=facet;while(facet){if(facet->visitid==qh.visit_id)throw Error("Qhull internal error: loop in facet list","QhullError");vertexT* a;vertexT* b;facetT* next;
        if(facet->toporient){a=static_cast<vertexT*>(facet->vertices->e[0].p);b=static_cast<vertexT*>(facet->vertices->e[1].p);next=static_cast<facetT*>(facet->neighbors->e[0].p);}else{b=static_cast<vertexT*>(facet->vertices->e[0].p);a=static_cast<vertexT*>(facet->vertices->e[1].p);next=static_cast<facetT*>(facet->neighbors->e[1].p);}
        if(a->visitid!=qh.vertex_visit){a->visitid=qh.vertex_visit;out.push_back(qh_pointid(&qh,a->point));}if(b->visitid!=qh.vertex_visit){b->visitid=qh.vertex_visit;out.push_back(qh_pointid(&qh,b->point));}facet->visitid=qh.visit_id;facet=next;if(facet==start)break;
    }return out;}
};
bool solve3(std::array<V3,3> a,V3 b,V3& result,double& determinant){
    determinant=1.;for(int k=0;k<3;k++){int pivot=k;for(int i=k+1;i<3;i++)if(std::abs(a[i][k])>std::abs(a[pivot][k]))pivot=i;if(pivot!=k){std::swap(a[k],a[pivot]);std::swap(b[k],b[pivot]);determinant=-determinant;}double diag=a[k][k];determinant*=diag;if(diag==0){determinant=0;return false;}
        for(int i=k+1;i<3;i++){double factor=a[i][k]/diag;a[i][k]=factor;for(int j=k+1;j<3;j++)a[i][j]-=factor*a[k][j];b[i]-=factor*b[k];}}
    for(int i=2;i>=0;i--){double v=b[i];for(int j=i+1;j<3;j++)v-=a[i][j]*result[j];result[i]=v/a[i][i];}return true;
}
}
std::vector<V4> convex_planes(const std::vector<V3>& points){
    if(points.empty())throw Error("No points given");std::vector<V3> centered;centered.reserve(points.size());for(const auto& p:points)centered.push_back(p-points[0]);int rank=matrix_rank(centered);
    std::vector<V3> eye={V3{1.,0.,0.},V3{0.,1.,0.},V3{0.,0.,1.}};
    if(rank<3){auto basis=flat_basis(centered,rank);std::vector<V3> axes=eye;for(int i=rank;i<3;i++)axes.push_back(basis[i]);std::vector<V3> edgevecs;
        if(rank==2){Hull hull(centered,2,&basis);auto ids=hull.vertices2();for(size_t i=0;i<ids.size();i++)edgevecs.push_back(points[ids[(i+1)%ids.size()]]-points[ids[i]]);}
        else if(rank==1)edgevecs.push_back(points.back()-points.front());auto extra=edge_axes(edgevecs,1e-9);axes.insert(axes.end(),extra.begin(),extra.end());return axis_planes(points,axes);
    }
    Hull hull(points,3);std::vector<V4> planes;PySet<std::array<int64_t,2>> edges;
    for(facetT* facet=hull.qh.facet_list;facet&&facet->next;facet=facet->next){if(!facet->simplicial&&(qh_setsize(&hull.qh,facet->vertices)!=3||qh_setsize(&hull.qh,facet->neighbors)!=3))throw Error("non-simplical facet encountered","QhullError");std::array<int64_t,3> ids;for(int j=0;j<3;j++){auto vertex=static_cast<vertexT*>(facet->vertices->e[j].p);ids[j]=qh_pointid(&hull.qh,vertex->point);}planes.push_back({facet->normal[0],facet->normal[1],facet->normal[2],-facet->offset});for(const auto& pair:{std::array<int,2>{0,1},std::array<int,2>{1,2},std::array<int,2>{2,0}}){int64_t a=ids[pair[0]],b=ids[pair[1]];if(a>b)std::swap(a,b);edges.insert({a,b});}}
    auto axial=axis_planes(points,eye),bevel=bevel_planes(points,edges.values());planes.insert(planes.end(),axial.begin(),axial.end());planes.insert(planes.end(),bevel.begin(),bevel.end());std::set<V4> unique;std::vector<V4> out;
    for(const auto& p:planes){V4 rounded;for(int j=0;j<4;j++)rounded[j]=round_decimal(p[j],1e6);if(unique.insert(rounded).second)out.push_back(p);}return out;
}
std::vector<V3> brush_vertices(const std::vector<V4>& planes,const std::vector<BrushSide>& sides){
    std::vector<V4> p;for(size_t i=0;i<planes.size();i++)if(sides[i].bevel==0)p.push_back(planes[i]);std::vector<V3> points;
    for(size_t i=0;i<p.size();i++)for(size_t j=i+1;j<p.size();j++)for(size_t k=j+1;k<p.size();k++){
        M3 a={V3{p[i][0],p[i][1],p[i][2]},V3{p[j][0],p[j][1],p[j][2]},V3{p[k][0],p[k][1],p[k][2]}};V3 b={p[i][3],p[j][3],p[k][3]},point{};double determinant;if(!solve3(a,b,point,determinant)||std::abs(determinant)<1e-9)continue;bool inside=true;for(const auto& row:p){V3 n={row[0],row[1],row[2]};if((p.size()==1?dot3(n,point):gemv3(n,point))>row[3]+1e-4){inside=false;break;}}if(inside)points.push_back(point);
    }
    if(points.size()<4)throw Error("Cannot reconstruct rotated brush");for(auto& pnt:points)for(double& x:pnt)x=round_decimal(x,1e6);std::sort(points.begin(),points.end());points.erase(std::unique(points.begin(),points.end()),points.end());return points;
}
}
