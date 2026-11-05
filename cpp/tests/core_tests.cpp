#include "common.hpp"
#include "navio.hpp"
#include "pyset.hpp"
#include <cfenv>
using namespace navgen;
static void require(bool good,const char* text){if(!good)throw Error(text);}
int main(){try{
    std::fesetround(FE_TONEAREST);
    require(sha256(std::string{})=="e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855","SHA256 empty");
    require(sha256(std::string("abc"))=="ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad","SHA256 abc");
    require(floor_div(-1,25)==-1&&floor_mod(-1,25)==24,"Python negative integer division");
    require(round_even(2.5)==2&&round_even(3.5)==4&&round_even(-2.5)==-2,"ties to even");
    require(!std::signbit(dot3(V3{-0.,-0.,-0.},V3{1.,1.,1.})),"BLAS dot positive zero");
    require(!std::signbit(gemv3(V3{-0.,-0.,-0.},V3{1.,1.,1.})),"BLAS GEMV positive zero");
    require(PyHash<std::array<int64_t,2>>{}({1,2})==-3550055125485641917LL,"CPython tuple hash");
    PySet<int> s;for(int i=0;i<500;++i)s.insert(i);for(int i=0;i<500;i+=2)s.erase(i);
    require(s.size()==250&&s.contains(499)&&!s.contains(498),"set tombstones");
    auto copied=s.copy();require(copied==s,"Python set copy");require((s-PySet<int>{1,3,5}).size()==247,"set difference");
    PySet<std::array<int,2>> edge_set{{-84,-18},{-71,-47},{-11,-28}};
    require((edge_set|edge_set).values()==std::vector<std::array<int,2>>{{-11,-28},{-71,-47},{-84,-18}},"CPython self-union order");
    auto p=read_nav(write_nav(Json{{"magic",0xFEEDFACEu},{"version",16},{"subversion",0},{"bsp_size",123},{"analyzed",0},{"places",Json::array({Json::array({65,0})})},{"has_unnamed",1},{"areas",Json::array()},{"ladders",Json::array()},{"trailing",Json::array({0,1,255})}}));
    require(p["places"][0][0]==65&&p["trailing"][2]==255,"NAV opaque bytes round trip");
    std::cout<<"native core checks passed\n";return 0;
}catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}}
