#pragma once
#include "discovery.hpp"

namespace navgen {
int ladder_direction(const V3& vector);
int top_slot(int facing,const V3& delta);
struct ClimbLine { int facing; V3 bottom,top; };
std::vector<ClimbLine> climb_lines(const LadderSpec& spec);
Json build_ladders(Json& nav,Collision& collision,const std::vector<LadderSpec>& specs,bool support_convention=false,double slope=.55);
}
