#pragma once
#include "collision.hpp"

namespace navgen {
inline constexpr V3 MINS{-16.,-16.,0.};
inline constexpr V3 DUCK{16.,16.,55.};
inline constexpr V3 STAND{16.,16.,71.};
bool clear_move(const Scene& scene,const V3& start,const V3& end,double height=55.);
int hull_move(const Scene& scene,const V3& start,const V3& end,double jump=64.,double drop=400.);
struct CentreFloor { bool good; double z; V3 normal; };
CentreFloor centre_floor(const Scene& scene,const V3& point,double slope);
struct SupportedOrigin { bool good; V3 point; };
SupportedOrigin support_origin(const Scene& scene,const V3& point,double slope=.55);
}
