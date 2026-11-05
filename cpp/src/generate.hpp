#pragma once
#include "common.hpp"
namespace navgen {
struct Options {
    fs::path map,game,out,dump_stages;
    std::optional<fs::path> seeds;
    double slope=.7;
    int threads=0;
    std::string profile="sdk",seed_discovery="none",missing_phy="skip";
    bool ladders=false;
};
Json generate(const Options& options);
}
