#include "bsp.hpp"
#include <cstdlib>
#include <iterator>
#include <regex>
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace navgen {
std::vector<fs::path> steam_libraries(const fs::path& root){
    std::vector<fs::path> out{root};
    std::ifstream file(root/"steamapps"/"libraryfolders.vdf",std::ios::binary);
    if(!file)return out;
    const std::string text((std::istreambuf_iterator<char>(file)),std::istreambuf_iterator<char>());
    static const std::regex pattern(R"re("path"\s+"((?:[^"\\]|\\.)*)")re",std::regex::icase);
    for(std::sregex_iterator it(text.begin(),text.end(),pattern),end;it!=end;++it){
        const std::string raw=(*it)[1];std::string value;
        for(std::size_t i=0;i<raw.size();++i){if(raw[i]=='\\'&&i+1<raw.size())++i;value+=raw[i];}
        out.push_back(path_from_utf8(value));
    }
    return out;
}

fs::path find_game(){
    std::vector<fs::path> roots;fs::path fallback;
#ifdef _WIN32
    std::wstring value(1024,L'\0');DWORD size=DWORD(value.size()*sizeof(wchar_t));
    if(RegGetValueW(HKEY_CURRENT_USER,L"Software\\Valve\\Steam",L"SteamPath",RRF_RT_REG_SZ,nullptr,value.data(),&size)==ERROR_SUCCESS){
        value.resize(size/sizeof(wchar_t));while(!value.empty()&&value.back()==L'\0')value.pop_back();roots.emplace_back(value);
    }
    roots.emplace_back(L"C:\\Program Files (x86)\\Steam");fallback=roots.back();
#else
    const char* home=std::getenv("HOME");const fs::path base=home?fs::path(home):fs::path();
    roots={base/".local/share/Steam",base/".steam/steam"};fallback=roots.front();
#endif
    for(const auto& root:roots)for(const auto& library:steam_libraries(root)){
        auto game=library/"steamapps"/"common"/"GarrysMod"/"garrysmod";std::error_code error;
        if(fs::is_directory(game,error))return game;
    }
    return fallback/"steamapps"/"common"/"GarrysMod"/"garrysmod";
}
}
