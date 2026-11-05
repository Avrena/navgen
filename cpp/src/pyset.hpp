#pragma once
#include "common.hpp"

namespace navgen {
// CPython 3.10's 64-bit numeric/tuple hashes and set-table semantics.
// Algorithm source: CPython v3.10.11, Objects/{set,tuple}object.c;
// the PSF licence is retained in third_party/python/LICENSE.
template<class T> struct PyHash {
    std::int64_t operator()(T value) const {
        constexpr std::uint64_t modulus=(UINT64_C(1)<<61)-1;
        const auto v=static_cast<std::int64_t>(value);bool negative=v<0;
        auto mag=negative?std::uint64_t(0)-static_cast<std::uint64_t>(v):static_cast<std::uint64_t>(v);
        auto result=static_cast<std::int64_t>(mag%modulus);if(negative)result=-result;
        return result==-1?-2:result;
    }
};
inline std::int64_t tuple_hash(std::span<const std::int64_t> lanes){
    constexpr auto p1=UINT64_C(11400714785074694791),p2=UINT64_C(14029467366897019727),p5=UINT64_C(2870177450012600261);
    std::uint64_t acc=p5;for(auto lane:lanes){acc+=static_cast<std::uint64_t>(lane)*p2;acc=std::rotl(acc,31);acc*=p1;}
    acc+=lanes.size()^(p5^UINT64_C(3527539));return acc==UINT64_MAX?1546275796:std::bit_cast<std::int64_t>(acc);
}
template<class T,std::size_t N>struct PyHash<std::array<T,N>>{
    std::int64_t operator()(const std::array<T,N>& v)const{std::array<std::int64_t,N> h;for(std::size_t i=0;i<N;++i)h[i]=PyHash<T>{}(v[i]);return tuple_hash(h);}
};
template<class A,class B>struct PyHash<std::pair<A,B>>{
    std::int64_t operator()(const std::pair<A,B>& v)const{std::array<std::int64_t,2> h{PyHash<A>{}(v.first),PyHash<B>{}(v.second)};return tuple_hash(h);}
};
template<class... T>struct PyHash<std::tuple<T...>>{
    std::int64_t operator()(const std::tuple<T...>& v)const{return std::apply([](const auto&... a){std::array<std::int64_t,sizeof...(T)> h{PyHash<T>{}(a)...};return tuple_hash(h);},v);}
};
template<class T,class Hash=PyHash<T>> class PySet {
    struct Entry{T key{};std::int64_t hash=0;std::uint8_t state=0;}; // empty, live, dummy
    std::vector<Entry> table_=std::vector<Entry>(8);
    std::size_t used_=0,fill_=0;
    void clean_insert(const T& key,std::int64_t h){
        const auto mask=table_.size()-1;std::uint64_t perturb=static_cast<std::uint64_t>(h);std::size_t i=perturb&mask;
        for(;;){auto probes=i+9<=mask?9:0;for(int j=0;j<=probes;++j)if(!table_[i+j].state){table_[i+j]={key,h,1};return;}
            perturb>>=5;i=(i*5+1+perturb)&mask;}
    }
    void resize(std::size_t minused){
        std::size_t size=8;while(size<=minused)size*=2;
        if(size==8&&table_.size()==8&&fill_==used_)return;
        auto old=std::move(table_);table_=std::vector<Entry>(size);fill_=used_;
        for(const auto& e:old)if(e.state==1)clean_insert(e.key,e.hash);
    }
    std::size_t lookup(const T& key,std::int64_t h)const{
        const auto mask=table_.size()-1;std::uint64_t perturb=static_cast<std::uint64_t>(h);std::size_t i=perturb&mask;
        for(;;){auto probes=i+9<=mask?9:0;for(int j=0;j<=probes;++j){auto& e=table_[i+j];if(e.state==0||(e.state==1&&e.hash==h&&e.key==key))return i+j;}
            perturb>>=5;i=(i*5+1+perturb)&mask;}
    }
public:
    PySet()=default;
    PySet(std::initializer_list<T> values){for(const auto& x:values)insert(x);}
    template<class It>PySet(It first,It last){for(;first!=last;++first)insert(*first);}
    explicit PySet(const std::vector<T>& values):PySet(values.begin(),values.end()){}
    bool insert(const T& key){
        auto h=Hash{}(key);const auto mask=table_.size()-1;std::uint64_t perturb=static_cast<std::uint64_t>(h);std::size_t i=perturb&mask,free=table_.size();
        for(;;){auto probes=i+9<=mask?9:0;
            for(int j=0;j<=probes;++j){auto& e=table_[i+j];
                if(e.state==0){auto slot=free<table_.size()?free:i+j;table_[slot]={key,h,1};++used_;
                    if(free==table_.size()){++fill_;if(fill_*5>=mask*3)resize(used_>50000?used_*2:used_*4);}return true;}
                if(e.state==1&&e.hash==h&&e.key==key)return false;
                if(e.state==2)free=i+j;
            }perturb>>=5;i=(i*5+1+perturb)&mask;
        }
    }
    bool add(const T& key){return insert(key);}
    bool erase(const T& key){auto i=lookup(key,Hash{}(key));if(table_[i].state!=1)return false;table_[i].state=2;table_[i].hash=-1;--used_;return true;}
    bool discard(const T& key){return erase(key);}
    bool contains(const T& key)const{return table_[lookup(key,Hash{}(key))].state==1;}
    std::size_t count(const T& key)const{return contains(key)?1:0;}
    std::size_t size()const{return used_;}
    bool empty()const{return used_==0;}
    void clear(){table_=std::vector<Entry>(8);used_=fill_=0;}
    std::vector<T> values()const{std::vector<T> out;out.reserve(used_);for(const auto& e:table_)if(e.state==1)out.push_back(e.key);return out;}
    struct iterator{
        const PySet* owner;std::size_t slot;
        void skip(){while(slot<owner->table_.size()&&owner->table_[slot].state!=1)++slot;}
        const T& operator*()const{return owner->table_[slot].key;}
        iterator& operator++(){++slot;skip();return *this;}
        bool operator!=(const iterator& b)const{return slot!=b.slot;}
        bool operator==(const iterator& b)const{return slot==b.slot;}
    };
    iterator begin()const{iterator i{this,0};i.skip();return i;}
    iterator end()const{return {this,table_.size()};}
    void update(const PySet& other){
        if(this==&other||other.empty())return;
        if((fill_+other.used_)*5>=(table_.size()-1)*3)resize((used_+other.used_)*2);
        if(fill_==0&&table_.size()==other.table_.size()&&other.fill_==other.used_){table_=other.table_;fill_=other.fill_;used_=other.used_;return;}
        if(fill_==0){used_=fill_=other.used_;for(const auto& e:other.table_)if(e.state==1)clean_insert(e.key,e.hash);return;}
        for(const auto& e:other.table_)if(e.state==1)insert(e.key);
    }
    PySet copy()const{PySet result;result.update(*this);return result;}
    PySet intersection(const PySet& other)const{
        if(this==&other)return copy();const PySet* scan=&other;const PySet* check=this;
        if(other.size()>size())std::swap(scan,check);PySet result;
        for(const auto& key:*scan)if(check->contains(key))result.insert(key);return result;
    }
    void difference_update(const PySet& other){
        if(this==&other){clear();return;}
        if((other.size()>>3)>size()){auto overlap=intersection(other);for(const auto& key:overlap)erase(key);}
        else for(const auto& key:other)erase(key);
        if(fill_-used_>(table_.size()-1)/4)resize(used_>50000?used_*2:used_*4);
    }
    PySet difference(const PySet& other)const{
        if((size()>>2)>other.size()){auto result=copy();result.difference_update(other);return result;}
        PySet result;for(const auto& key:*this)if(!other.contains(key))result.insert(key);return result;
    }
    PySet union_with(const PySet& other)const{auto result=copy();if(this!=&other)result.update(other);return result;}
    PySet operator|(const PySet& other)const{return union_with(other);}
    PySet operator-(const PySet& other)const{return difference(other);}
    PySet operator&(const PySet& other)const{return intersection(other);}
    PySet& operator|=(const PySet& other){update(other);return *this;}
    bool operator==(const PySet& other)const{if(size()!=other.size())return false;for(const auto& key:*this)if(!other.contains(key))return false;return true;}
};
}
