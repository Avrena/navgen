#include "common.hpp"
#include "gma.hpp"
#include <cstdlib>

namespace navgen {
namespace {
bool decimal_digit(char c) {return c>='0'&&c<='9';}
std::string_view trimmed_number(std::string_view value) {
    const auto begin=value.find_first_not_of(" \t\n\r\v\f");
    if(begin==std::string_view::npos)return {};
    return value.substr(begin,value.find_last_not_of(" \t\n\r\v\f")-begin+1);
}
[[noreturn]] void invalid_float(const std::string& value) {
    throw Error("could not convert string to float: '"+value+"'");
}
[[noreturn]] void invalid_integer(const std::string& value) {
    throw Error("invalid literal for int() with base 10: '"+value+"'");
}
}
std::optional<std::string> python_number_text(const std::string& value) {
    std::string result;result.reserve(value.size());
    try {
        for(const auto point:unicode_codepoints(value)) {
            if(point<128)result+=static_cast<char>(point);
            else if(unicode_whitespace(point))result+=' ';
            else {
                const auto decimal=unicode_decimal(point);
                if(decimal<0)return std::nullopt;
                result+=static_cast<char>('0'+decimal);
            }
        }
    } catch(const Error& error) {
        if(error.kind!="UnicodeDecodeError")throw;
        return std::nullopt;
    }
    return result;
}
ParsedInteger python_integer(const std::string& value) {
    const auto normalised=python_number_text(value);
    if(!normalised)invalid_integer(value);
    const auto text=trimmed_number(*normalised);std::size_t at=0,digits=0;ParsedInteger result;
    if(at<text.size()&&(text[at]=='+'||text[at]=='-'))result.negative=text[at++]=='-';
    if(at==text.size()||!decimal_digit(text[at]))invalid_integer(value);
    constexpr auto maximum=std::numeric_limits<std::uint64_t>::max();
    for(;at<text.size();++at) {
        const auto c=text[at];
        if(c=='_'&&at+1<text.size()&&decimal_digit(text[at-1])&&decimal_digit(text[at+1]))continue;
        if(!decimal_digit(c))invalid_integer(value);
        const auto digit=static_cast<unsigned>(c-'0');++digits;
        result.low_bits=result.low_bits*10+digit;
        if(!result.overflow) {
            if(result.magnitude>(maximum-digit)/10) {result.magnitude=maximum;result.overflow=true;}
            else result.magnitude=result.magnitude*10+digit;
        }
    }
    // CPython 3.10.11's default security limit for decimal integer conversion.
    if(digits>4300)throw Error("Exceeds the limit (4300 digits) for integer string conversion: value has "+std::to_string(digits)+" digits");
    if(result.negative)result.low_bits=std::uint64_t(0)-result.low_bits;
    return result;
}
bool ParsedInteger::equals(std::int64_t value) const {
    if(overflow)return false;
    if(!magnitude)return value==0;
    if(negative!=(value<0))return false;
    const auto other=value<0?std::uint64_t(0)-static_cast<std::uint64_t>(value):static_cast<std::uint64_t>(value);
    return magnitude==other;
}
std::int64_t ParsedInteger::clamped() const {
    const auto limit=static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max())+(negative?1:0);
    if(overflow||magnitude>=limit)return negative?std::numeric_limits<std::int64_t>::min():std::numeric_limits<std::int64_t>::max();
    return negative?-static_cast<std::int64_t>(magnitude):static_cast<std::int64_t>(magnitude);
}
double python_float(const std::string& value) {
    const auto normalised=python_number_text(value);
    if(!normalised)invalid_float(value);
    const auto text=trimmed_number(*normalised);std::size_t at=0;std::string clean;
    if(at<text.size()&&(text[at]=='+'||text[at]=='-'))clean+=text[at++];
    std::string special(text.substr(at));
    for(auto& c:special)if(c>='A'&&c<='Z')c=static_cast<char>(c-'A'+'a');
    if(special=="inf"||special=="infinity"||special=="nan") {
        clean+=special;return std::strtod(clean.c_str(),nullptr);
    }
    auto digits=[&]() {
        const auto begin=at;
        while(at<text.size()) {
            if(decimal_digit(text[at]))clean+=text[at++];
            else if(text[at]=='_'&&at>begin&&at+1<text.size()&&decimal_digit(text[at-1])&&decimal_digit(text[at+1]))++at;
            else break;
        }
        return at!=begin;
    };
    const bool before=digits();bool after=false;
    if(at<text.size()&&text[at]=='.') {clean+=text[at++];after=digits();}
    if(!before&&!after)invalid_float(value);
    if(at<text.size()&&(text[at]=='e'||text[at]=='E')) {
        clean+=text[at++];
        if(at<text.size()&&(text[at]=='+'||text[at]=='-'))clean+=text[at++];
        if(!digits())invalid_float(value);
    }
    if(at!=text.size())invalid_float(value);
    // The runtime uses the C locale. strtod retains subnormals and rounds
    // overflow to infinity; finite/range validation belongs to the caller.
    return std::strtod(clean.c_str(),nullptr);
}
std::string sha256(std::span<const std::uint8_t> data){
    static constexpr std::array<std::uint32_t,64> k{
        0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
        0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
        0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
        0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
        0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
        0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
        0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
        0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2};
    std::array<std::uint32_t,8> h{0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
    auto block=[&](const std::uint8_t* p){
        std::array<std::uint32_t,64> w{};
        for(int i=0;i<16;++i)w[i]=(std::uint32_t(p[4*i])<<24)|(std::uint32_t(p[4*i+1])<<16)|(std::uint32_t(p[4*i+2])<<8)|p[4*i+3];
        for(int i=16;i<64;++i){auto a=w[i-15],b=w[i-2];w[i]=w[i-16]+(std::rotr(a,7)^std::rotr(a,18)^(a>>3))+w[i-7]+(std::rotr(b,17)^std::rotr(b,19)^(b>>10));}
        auto a=h[0],b=h[1],c=h[2],d=h[3],e=h[4],f=h[5],g=h[6],hh=h[7];
        for(int i=0;i<64;++i){auto t1=hh+(std::rotr(e,6)^std::rotr(e,11)^std::rotr(e,25))+((e&f)^(~e&g))+k[i]+w[i];auto t2=(std::rotr(a,2)^std::rotr(a,13)^std::rotr(a,22))+((a&b)^(a&c)^(b&c));hh=g;g=f;f=e;e=d+t1;d=c;c=b;b=a;a=t1+t2;}
        h[0]+=a;h[1]+=b;h[2]+=c;h[3]+=d;h[4]+=e;h[5]+=f;h[6]+=g;h[7]+=hh;
    };
    std::size_t offset=0;for(;offset+64<=data.size();offset+=64)block(data.data()+offset);
    std::array<std::uint8_t,128> tail{};auto left=data.size()-offset;
    if(left)std::memcpy(tail.data(),data.data()+offset,left);tail[left]=0x80;
    auto size=left<56?64:128;auto bits=static_cast<std::uint64_t>(data.size())*8;
    for(int i=0;i<8;++i)tail[size-1-i]=static_cast<std::uint8_t>(bits>>(i*8));
    block(tail.data());if(size==128)block(tail.data()+64);
    std::ostringstream out;out<<std::hex<<std::setfill('0');for(auto v:h)out<<std::setw(8)<<v;return out.str();
}
void write_json(const fs::path& path,const Json& value){
    auto text=value.dump(2)+"\n";write_file(path,{reinterpret_cast<const std::uint8_t*>(text.data()),text.size()});
}
void write_npy(const fs::path& path,const void* data,std::size_t bytes,const std::string& dtype,std::span<const std::size_t> shape){
    if constexpr(std::endian::native!=std::endian::little)throw Error("Stage dumps require a little-endian host");
    std::string header="{'descr': '"+dtype+"', 'fortran_order': False, 'shape': (";
    for(std::size_t i=0;i<shape.size();++i){header+=std::to_string(shape[i]);if(i+1<shape.size()||shape.size()==1)header+=", ";}
    header+="), }";header+=std::string((64-(10+header.size()+1)%64)%64,' ');header+='\n';
    if(header.size()>65535)throw Error("NPY header exceeds v1 limit");
    Bytes prefix{0x93,'N','U','M','P','Y',1,0,static_cast<std::uint8_t>(header.size()),static_cast<std::uint8_t>(header.size()>>8)};
    std::ofstream f(path,std::ios::binary);if(!f)throw Error("Cannot write stage dump","OSError");
    f.write(reinterpret_cast<const char*>(prefix.data()),prefix.size());f.write(header.data(),header.size());
    f.write(static_cast<const char*>(data),bytes);if(!f)throw Error("Cannot finish stage dump","OSError");
}
}
