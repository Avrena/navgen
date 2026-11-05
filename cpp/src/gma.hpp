#pragma once
#include "common.hpp"
#include <filesystem>
#include <map>
#include <span>
#include <string_view>

namespace navgen {
std::vector<uint32_t> unicode_codepoints(std::string_view text);
std::string unicode_utf8(std::span<const uint32_t> points);
int unicode_decimal(uint32_t point);
bool unicode_whitespace(uint32_t point);
bool unicode_linebreak(uint32_t point);
std::string unicode_lower(std::string_view text);
std::string latin1_utf8(std::string_view text);
std::string cp437_utf8(std::string_view text);
std::string content_path(std::string name);
class GMA {
public:
    struct Entry { uint64_t offset, length; uint32_t crc; };
    std::filesystem::path path;
    std::map<std::string, Entry> entries;
    uint64_t data_offset = 0;
    explicit GMA(const std::filesystem::path& path);
    Bytes read(const std::string& name) const;
};
}
