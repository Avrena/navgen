#include "planes.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace navgen {
namespace {
const std::array<V3,3> axes3={V3{1.,0.,0.},V3{0.,1.,0.},V3{0.,0.,1.}};
}
std::vector<V4> axis_planes(const std::vector<V3>& points,const std::vector<V3>& axes) {
    std::vector<V4> out;out.reserve(axes.size()*2);
    for(const auto& axis:axes) for(double sign:{1.,-1.}) {
        V3 n={sign*axis[0],sign*axis[1],sign*axis[2]};
        double d=-std::numeric_limits<double>::infinity();
        for(const auto& point:points)d=std::max(d,gemv3(point,n));
        out.push_back({n[0],n[1],n[2],d});
    }
    return out;
}
std::vector<V3> edge_axes(const std::vector<V3>& edges,double epsilon) {
    std::vector<V3> out;out.reserve(edges.size()*3);
    for(const auto& edge:edges)for(const auto& axis:axes3) {
        V3 n=cross(edge,axis);double length=std::sqrt(dot3(n,n));
        if(length>epsilon)out.push_back({n[0]/length,n[1]/length,n[2]/length});
    }
    return out;
}
std::vector<V4> bevel_planes(const std::vector<V3>& points,const std::vector<std::array<int64_t,2>>& edges) {
    std::vector<V4> out;out.reserve(edges.size()*3);
    for(const auto& ids:edges) {
        const auto& a=points.at(static_cast<size_t>(ids[0]));const auto& b=points.at(static_cast<size_t>(ids[1]));
        V3 edge={b[0]-a[0],b[1]-a[1],b[2]-a[2]};
        for(const auto& axis:axes3) {
            V3 n=cross(edge,axis);double length=std::sqrt(dot3(n,n));
            if(length<1e-8)continue;
            for(auto& v:n)v/=length;
            double d=dot3(n,a),dmin=std::numeric_limits<double>::infinity(),dmax=-dmin;
            for(const auto& p:points){double x=gemv3(p,n)-d;dmin=std::min(dmin,x);dmax=std::max(dmax,x);}
            if(dmax<1e-5)out.push_back({n[0],n[1],n[2],d});
            else if(dmin>-1e-5)out.push_back({-n[0],-n[1],-n[2],-d});
        }
    }
    return out;
}
std::pair<std::vector<V4>,V4> triangle_kernel(const Tri& points) {
    std::vector<V3> edges(3);
    for(int i=0;i<3;i++)for(int a=0;a<3;a++)edges[i][a]=points[(i+1)%3][a]-points[i][a];
    V3 neg={-edges[2][0],-edges[2][1],-edges[2][2]};V3 normal=cross(edges[0],neg);
    double length=std::sqrt(dot3(normal,normal));if(length<1e-8)return {{},{0.,0.,0.,0.}};
    for(auto& v:normal)v/=length;
    auto extra=edge_axes(edges,1e-8);std::vector<V3> axes={normal,axes3[0],axes3[1],axes3[2]};
    axes.insert(axes.end(),extra.begin(),extra.end());
    V4 front={normal[0],normal[1],normal[2],dot3(points[0],normal)};
    return {axis_planes(std::vector<V3>(points.begin(),points.end()),axes),front};
}
}
