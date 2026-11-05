#pragma once
#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <mutex>
#include <numeric>
#include <optional>
#include <set>
#include <span>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>
#include <nlohmann/json.hpp>

namespace navgen {
using Json = nlohmann::json;
using V3 = std::array<double,3>;
using V4 = std::array<double,4>;
using Box = std::array<double,8>;
using Bytes = std::vector<std::uint8_t>;
namespace fs = std::filesystem;
inline fs::path path_from_utf8(std::string_view text){return fs::path(std::u8string(text.begin(),text.end()));}
inline std::string path_utf8(const fs::path& path){auto text=path.u8string();return std::string(text.begin(),text.end());}

struct Error : std::runtime_error {
    std::string kind;
    explicit Error(std::string message, std::string type="ValueError")
        : std::runtime_error(std::move(message)), kind(std::move(type)) {}
};

// Python's integer consumers here need exact small comparisons and bit masks,
// rather than a general-purpose arbitrary-precision arithmetic implementation.
struct ParsedInteger {
    std::uint64_t low_bits=0,magnitude=0;
    bool negative=false,overflow=false;
    bool nonzero() const {return overflow||magnitude!=0;}
    bool equals(std::int64_t value) const;
    std::int64_t clamped() const;
};
std::optional<std::string> python_number_text(const std::string& value);
ParsedInteger python_integer(const std::string& value);
double python_float(const std::string& value);

template<class T,std::size_t N> inline std::array<T,N> operator+(const std::array<T,N>& a,const std::array<T,N>& b) {
    std::array<T,N> r; for(std::size_t i=0;i<N;++i)r[i]=a[i]+b[i]; return r;
}
template<class T,std::size_t N> inline std::array<T,N> operator-(const std::array<T,N>& a,const std::array<T,N>& b) {
    std::array<T,N> r; for(std::size_t i=0;i<N;++i)r[i]=a[i]-b[i]; return r;
}
template<class T,std::size_t N> inline std::array<T,N> operator-(const std::array<T,N>& a) {
    std::array<T,N> r; for(std::size_t i=0;i<N;++i)r[i]=-a[i]; return r;
}
template<class T,std::size_t N,class S> inline std::array<T,N> operator*(const std::array<T,N>& a,S b) {
    std::array<T,N> r; for(std::size_t i=0;i<N;++i)r[i]=a[i]*b; return r;
}
template<class T,std::size_t N,class S> inline std::array<T,N> operator/(const std::array<T,N>& a,S b) {
    std::array<T,N> r; for(std::size_t i=0;i<N;++i)r[i]=a[i]/b; return r;
}
template<class T,std::size_t N,class S> inline std::array<T,N> operator*(S b,const std::array<T,N>& a) {return a*b;}
template<class T,std::size_t N> inline std::array<T,N>& operator+=(std::array<T,N>& a,const std::array<T,N>& b){a=a+b;return a;}
template<class T,std::size_t N> inline std::array<T,N>& operator-=(std::array<T,N>& a,const std::array<T,N>& b){a=a-b;return a;}
template<class T,std::size_t N,class S> inline std::array<T,N>& operator*=(std::array<T,N>& a,S b){a=a*b;return a;}
template<class T,std::size_t N,class S> inline std::array<T,N>& operator/=(std::array<T,N>& a,S b){a=a/b;return a;}

// Reference OpenBLAS SkylakeX kernels. Ordinary arithmetic must not contract.
// Their zero-initialised accumulation also makes an all-negative-zero sum +0.
template<class T> inline T dot3(const std::array<T,3>& a,const std::array<T,3>& b) {
    return std::fma(a[2],b[2],std::fma(a[1],b[1],a[0]*b[0]))+T(0);
}
template<class T> inline T gemv3(const std::array<T,3>& a,const std::array<T,3>& b) {
    return std::fma(a[2],b[2],std::fma(a[0],b[0],a[1]*b[1]))+T(0);
}
template<class T> inline T dot2(const std::array<T,2>& a,const std::array<T,2>& b) {
    return std::fma(a[1],b[1],a[0]*b[0])+T(0);
}
template<class T> inline T norm3(const std::array<T,3>& a) { return std::sqrt(dot3(a,a)); }
template<class T> inline T norm2(const std::array<T,2>& a) { return std::sqrt(dot2(a,a)); }
template<class T> inline T norm2(const std::array<T,3>& a) { return std::sqrt(std::fma(a[1],a[1],a[0]*a[0])); }
template<class T> inline T axis_norm3(const std::array<T,3>& a) { return std::sqrt((a[0]*a[0]+a[1]*a[1])+a[2]*a[2]); }
template<class T> inline std::array<T,3> cross(const std::array<T,3>& a,const std::array<T,3>& b) {
    return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]};
}
inline double round_even(double x) {return std::nearbyint(x);}
inline double round_decimal(double x,double scale) {return std::nearbyint(x*scale)/scale;}
inline std::int64_t floor_div(std::int64_t a,std::int64_t b) {
    auto q=a/b,r=a%b; return q-((r!=0)&&((r<0)!=(b<0)));
}
inline std::int64_t floor_mod(std::int64_t a,std::int64_t b) {return a-floor_div(a,b)*b;}
inline int runtime_threads=1;
inline thread_local std::size_t worker_index=0;
inline int logical_cores(){return static_cast<int>(std::max(1u,std::thread::hardware_concurrency()));}
inline void configure_threads(int n){if(n<1||n>logical_cores())throw Error("Threads must be an integer from 1 to "+std::to_string(logical_cores()));runtime_threads=n;}
template<class F> void parallel_for(std::size_t count,F fn) {
    const auto n=std::min<std::size_t>(runtime_threads,count);
    if(n<=1){for(std::size_t i=0;i<count;++i)fn(i);return;}
    std::vector<std::thread> workers;std::exception_ptr error;std::mutex lock;
    auto run=[&](std::size_t t){
        worker_index=t;
        try{for(std::size_t i=count*t/n;i<count*(t+1)/n;++i)fn(i);}
        catch(...){std::lock_guard guard(lock);if(!error)error=std::current_exception();}
    };
    workers.reserve(n-1);
    try{for(std::size_t t=1;t<n;++t)workers.emplace_back(run,t);}
    catch(...){for(auto& thread:workers)thread.join();throw;}
    run(0);for(auto& thread:workers)thread.join();if(error)std::rethrow_exception(error);
}
inline double now(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
inline void progress(const std::string& line){std::cout<<line<<std::endl;}
inline Bytes read_file(const fs::path& path){
    std::ifstream f(path,std::ios::binary);if(!f)throw Error("Cannot read "+path_utf8(path),fs::exists(path)?"OSError":"FileNotFoundError");
    f.seekg(0,std::ios::end);auto size=f.tellg();if(size<0)throw Error("Cannot determine file size","OSError");
    Bytes data(static_cast<std::size_t>(size));f.seekg(0);if(!data.empty())f.read(reinterpret_cast<char*>(data.data()),size);
    if(!f)throw Error("Truncated read: "+path_utf8(path),"OSError");return data;
}
inline void write_file(const fs::path& path,std::span<const std::uint8_t> data){
    std::ofstream f(path,std::ios::binary|std::ios::trunc);if(!f)throw Error("Cannot write "+path_utf8(path),"OSError");
    f.write(reinterpret_cast<const char*>(data.data()),static_cast<std::streamsize>(data.size()));f.close();
    if(!f)throw Error("Cannot finish writing "+path_utf8(path),"OSError");
}
std::string sha256(std::span<const std::uint8_t> data);
inline std::string sha256(const std::string& data){return sha256(std::span(reinterpret_cast<const std::uint8_t*>(data.data()),data.size()));}
void write_json(const fs::path& path,const Json& value);
void write_npy(const fs::path& path,const void* data,std::size_t bytes,const std::string& dtype,std::span<const std::size_t> shape);
template<class T,std::size_t N> void dump_array(const fs::path& path,const std::vector<std::array<T,N>>& values,const std::string& dtype){
    std::array<std::size_t,2> shape{values.size(),N};write_npy(path,values.data(),values.size()*sizeof(T)*N,dtype,shape);
}
template<class T> void dump_array(const fs::path& path,const std::vector<T>& values,const std::string& dtype){
    std::array<std::size_t,1> shape{values.size()};write_npy(path,values.data(),values.size()*sizeof(T),dtype,shape);
}
}
