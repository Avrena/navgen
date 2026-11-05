#include "gma.hpp"
#include "zlib.h"
#include <algorithm>
#include <fstream>
#include <limits>
#include <stdexcept>

namespace navgen {
namespace {
using Py_UCS4=uint32_t;
struct _PyUnicode_TypeRecord {const int upper,lower,title;const unsigned char decimal,digit;const unsigned short flags;};
#include "../third_party/python/unicodetype_db.h"
const _PyUnicode_TypeRecord& unicode_type(uint32_t point){int index=point<0x110000?index2[(index1[point>>SHIFT]<<SHIFT)+(point&((1<<SHIFT)-1))]:0;return _PyUnicode_TypeRecords[index];}
#undef SHIFT
void append_utf8(std::string& out,uint32_t p){
    if(p<0x80)out.push_back(static_cast<char>(p));
    else if(p<0x800){out.push_back(static_cast<char>(0xc0|(p>>6)));out.push_back(static_cast<char>(0x80|(p&63)));}
    else if(p<0x10000){out.push_back(static_cast<char>(0xe0|(p>>12)));out.push_back(static_cast<char>(0x80|((p>>6)&63)));out.push_back(static_cast<char>(0x80|(p&63)));}
    else{out.push_back(static_cast<char>(0xf0|(p>>18)));out.push_back(static_cast<char>(0x80|((p>>12)&63)));out.push_back(static_cast<char>(0x80|((p>>6)&63)));out.push_back(static_cast<char>(0x80|(p&63)));}
}
[[noreturn]] void utf8_error(std::string_view text,size_t start,size_t end,const char* reason){
    std::ostringstream message;message<<"'utf-8' codec can't decode ";
    if(end==start+1)message<<"byte 0x"<<std::hex<<std::setw(2)<<std::setfill('0')<<static_cast<unsigned>(static_cast<uint8_t>(text[start]))<<std::dec<<" in position "<<start;
    else message<<"bytes in position "<<start<<'-'<<end-1;
    message<<": "<<reason;throw Error(message.str(),"UnicodeDecodeError");
}
uint64_t read_int(std::istream& f,size_t n,const std::filesystem::path& p){uint64_t v=0;for(size_t i=0;i<n;i++){int c=f.get();if(c==EOF)throw Error("Truncated GMA table: "+python_path_text(p));v|=uint64_t(uint8_t(c))<<(i*8);}return v;}
std::string cstr(std::istream& f,const std::filesystem::path& p){std::string s;while(s.size()<1048576){int c=f.get();if(c==0)return s;if(c==EOF)throw Error("Truncated GMA string: "+python_path_text(p));s.push_back(static_cast<char>(c));}throw Error("Oversized GMA string: "+python_path_text(p));}
}
std::vector<uint32_t> unicode_codepoints(std::string_view text){
    std::vector<uint32_t> out;out.reserve(text.size());size_t i=0;
    while(i<text.size()){
        size_t start=i;uint8_t c=static_cast<uint8_t>(text[i++]);if(c<128){out.push_back(c);continue;}int n;uint32_t cp;
        if(c>=0xc2&&c<=0xdf){n=1;cp=c&31;}else if(c>=0xe0&&c<=0xef){n=2;cp=c&15;}else if(c>=0xf0&&c<=0xf4){n=3;cp=c&7;}else utf8_error(text,start,i,"invalid start byte");
        for(int k=0;k<n;k++){
            if(i==text.size())utf8_error(text,start,i,"unexpected end of data");uint8_t d=static_cast<uint8_t>(text[i]);
            if((d&0xc0)!=0x80||(k==0&&((c==0xe0&&d<0xa0)||(c==0xed&&d>=0xa0)||(c==0xf0&&d<0x90)||(c==0xf4&&d>=0x90))))utf8_error(text,start,i,"invalid continuation byte");
            ++i;cp=(cp<<6)|(d&63);
        }out.push_back(cp);
    }return out;
}
std::string unicode_utf8(std::span<const uint32_t> points){std::string out;for(auto p:points)append_utf8(out,p);return out;}
int unicode_decimal(uint32_t point){const auto& type=unicode_type(point);return type.flags&2?type.decimal:-1;}
bool unicode_whitespace(uint32_t point){return _PyUnicode_IsWhitespace(point)!=0;}
bool unicode_linebreak(uint32_t point){return _PyUnicode_IsLinebreak(point)!=0;}
std::string unicode_lower(std::string_view text){
    auto points=unicode_codepoints(text);std::string out;out.reserve(text.size());
    for(size_t i=0;i<points.size();i++){
        auto p=points[i];const auto& type=unicode_type(p);
        if(p==0x3a3){
            size_t before=i;while(before&&unicode_type(points[before-1]).flags&0x1000)--before;
            bool final=before&&(unicode_type(points[before-1]).flags&0x2000);
            if(final){size_t after=i+1;while(after<points.size()&&(unicode_type(points[after]).flags&0x1000))++after;final=after==points.size()||!(unicode_type(points[after]).flags&0x2000);}
            append_utf8(out,final?0x3c2:0x3c3);
        }else if(type.flags&0x4000){int at=type.lower&0xffff,n=type.lower>>24;for(int k=0;k<n;k++)append_utf8(out,_PyUnicode_ExtendedCase[at+k]);}
        else append_utf8(out,p+type.lower);
    }return out;
}
std::string latin1_utf8(std::string_view text){std::string out;out.reserve(text.size());for(unsigned char c:text)append_utf8(out,c);return out;}
std::string cp437_utf8(std::string_view text){
    static constexpr uint32_t high[128]={
        0x00c7,0x00fc,0x00e9,0x00e2,0x00e4,0x00e0,0x00e5,0x00e7,
        0x00ea,0x00eb,0x00e8,0x00ef,0x00ee,0x00ec,0x00c4,0x00c5,
        0x00c9,0x00e6,0x00c6,0x00f4,0x00f6,0x00f2,0x00fb,0x00f9,
        0x00ff,0x00d6,0x00dc,0x00a2,0x00a3,0x00a5,0x20a7,0x0192,
        0x00e1,0x00ed,0x00f3,0x00fa,0x00f1,0x00d1,0x00aa,0x00ba,
        0x00bf,0x2310,0x00ac,0x00bd,0x00bc,0x00a1,0x00ab,0x00bb,
        0x2591,0x2592,0x2593,0x2502,0x2524,0x2561,0x2562,0x2556,
        0x2555,0x2563,0x2551,0x2557,0x255d,0x255c,0x255b,0x2510,
        0x2514,0x2534,0x252c,0x251c,0x2500,0x253c,0x255e,0x255f,
        0x255a,0x2554,0x2569,0x2566,0x2560,0x2550,0x256c,0x2567,
        0x2568,0x2564,0x2565,0x2559,0x2558,0x2552,0x2553,0x256b,
        0x256a,0x2518,0x250c,0x2588,0x2584,0x258c,0x2590,0x2580,
        0x03b1,0x00df,0x0393,0x03c0,0x03a3,0x03c3,0x00b5,0x03c4,
        0x03a6,0x0398,0x03a9,0x03b4,0x221e,0x03c6,0x03b5,0x2229,
        0x2261,0x00b1,0x2265,0x2264,0x2320,0x2321,0x00f7,0x2248,
        0x00b0,0x2219,0x00b7,0x221a,0x207f,0x00b2,0x25a0,0x00a0,
    };std::string out;out.reserve(text.size());for(unsigned char c:text)append_utf8(out,c<128?c:high[c-128]);return out;
}
std::string content_path(std::string name){
    name=unicode_lower(name);std::replace(name.begin(),name.end(),'\\','/');bool bad=name.empty()||name.front()=='/'||name.find(':')!=std::string::npos;
    size_t pos=0;while(pos<name.size()){auto end=name.find('/',pos);if(end==std::string::npos)end=name.size();if(name.substr(pos,end-pos)=="..")bad=true;pos=end+1;}
    if(bad)throw Error("Invalid content path: '"+name+"'");return name;
}
GMA::GMA(const std::filesystem::path& p):path(p){
    auto size=std::filesystem::file_size(path);std::ifstream f(path,std::ios::binary);if(!f)throw Error("Cannot open GMA: "+python_path_text(path));
    char signature[4]{};f.read(signature,4);if(f.gcount()!=4||std::string(signature,4)!="GMAD")throw Error("Unsupported GMA signature: "+python_path_text(path));
    auto version=read_int(f,1,path);if(version<1||version>3)throw Error("Unsupported GMA version "+std::to_string(version)+": "+python_path_text(path));
    read_int(f,8,path);read_int(f,8,path);if(version>1)while(!cstr(f,path).empty()){}
    cstr(f,path);cstr(f,path);cstr(f,path);read_int(f,4,path);uint64_t offset=0;
    while(read_int(f,4,path)){
        std::string raw=cstr(f,path);
        auto name=content_path(std::move(raw));auto length=read_int(f,8,path);auto crc=static_cast<uint32_t>(read_int(f,4,path));
        if(length>uint64_t(std::numeric_limits<int64_t>::max())||length>size||offset>size-length||entries.count(name))throw Error("Invalid GMA entry "+name+": "+python_path_text(path));
        entries.emplace(name,Entry{offset,length,crc});offset+=length;
    }
    data_offset=static_cast<uint64_t>(f.tellg());if(data_offset>size||offset>size-data_offset)throw Error("Truncated GMA payload: "+python_path_text(path));
}
Bytes GMA::read(const std::string& name)const{
    const auto& e=entries.at(content_path(name));std::ifstream f(path,std::ios::binary);if(!f)throw Error("Cannot open GMA: "+python_path_text(path));
    f.seekg(static_cast<std::streamoff>(data_offset+e.offset));Bytes out(static_cast<size_t>(e.length));f.read(reinterpret_cast<char*>(out.data()),static_cast<std::streamsize>(out.size()));
    uLong crc=crc32(0L,Z_NULL,0);size_t at=0;while(at<out.size()){auto n=std::min(out.size()-at,size_t(std::numeric_limits<uInt>::max()));crc=crc32(crc,out.data()+at,static_cast<uInt>(n));at+=n;}
    if(static_cast<uint64_t>(f.gcount())!=e.length||crc!=e.crc)throw Error("GMA payload/CRC mismatch for "+name+": "+python_path_text(path));return out;
}
}
