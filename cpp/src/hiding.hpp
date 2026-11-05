#pragma once
#include "collision.hpp"
#include <map>

namespace navgen {
using F3=std::array<float,3>;
F3 spot_position(const Json& area,int corner);
std::vector<F3> area_spots(const Json& area,const std::map<int64_t,const Json*>& by_id);
std::vector<uint8_t> cover_flags(const Scene& scene,const std::vector<F3>& positions,int threads=1);
Json compute_hiding_spots(Json& nav,Collision& collision,int threads=1);
}
