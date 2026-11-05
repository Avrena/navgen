#include "geometry.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace navgen {
bool clear_move(const Scene& scene,const V3& start,const V3& end,double height) {
    const auto r=trace(scene,start,end,MINS,{16.,16.,height});
    return r[0]==1. && !r[4] && !r[5];
}

int hull_move(const Scene& scene,const V3& start,const V3& end,double jump,double drop) {
    const double dz=end[2]-start[2];
    if(dz>jump+.01 || dz<=-drop) return 0;
    auto a=start,b=end;
    a[2]+=.01; b[2]+=.01;
    if(clear_move(scene,a,b)) return std::abs(dz)<=18.01 ? 1 : (dz>0 ? 3 : 4);
    auto high=a; high[2]+=jump;
    const auto ceiling=trace(scene,a,high,MINS,DUCK);
    if(ceiling[4]) return 0;
    const double available=std::max(0.,jump*ceiling[0]-.05);
    const double first=std::max(18.,dz+.05);
    if(first>available) return 0;
    for(int trial=0;trial<static_cast<int>(std::ceil(available-first))+2;++trial) {
        const double h=trial==0 ? available : first+trial-1;
        if(h>available) continue;
        auto up=a,across=b; up[2]+=h; across[2]=up[2];
        if(clear_move(scene,up,across) && clear_move(scene,across,b))
            return h<=18.01 && std::abs(dz)<=18.01 ? 2 : (dz<-18 ? 4 : 3);
    }
    return 0;
}

CentreFloor centre_floor(const Scene& scene,const V3& point,double slope) {
    constexpr V3 zero{};
    for(double offset:{18.,36.,72.,2.,.05}) {
        auto a=point,b=point; a[2]+=offset; b[2]-=400.;
        const auto r=trace(scene,a,b,zero,zero);
        if(r[4] || r[0]>=1) continue;
        return {r[3]>=slope && !(static_cast<int64_t>(r[8])&6),
                a[2]+(b[2]-a[2])*r[0]-.03125/std::max(r[3],.01),{r[1],r[2],r[3]}};
    }
    return {false,std::numeric_limits<double>::quiet_NaN(),{}};
}

SupportedOrigin support_origin(const Scene& scene,const V3& point,double slope) {
    const auto floor=centre_floor(scene,point,slope);
    if(!floor.good) return {false,point};
    const double rise=16.*(std::abs(floor.normal[0])+std::abs(floor.normal[1]))/std::max(floor.normal[2],.01);
    for(double offset:{.1,rise+.1,2.1,4.1,8.1,12.1,18.1}) {
        auto high=point; high[2]=floor.z+offset;
        auto low=high; low[2]=floor.z-2.;
        auto r=trace(scene,high,low,MINS,DUCK);
        if(r[4] || r[0]>=1 || r[3]<slope || (static_cast<int64_t>(r[8])&6)) continue;
        auto p=high+(low-high)*r[0]; p[2]+=.01;
        r=trace(scene,p,p,MINS,DUCK);
        if(!(r[4] || r[5])) return {true,p};
    }
    return {false,point};
}
}
