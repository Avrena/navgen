#pragma once
#include "common.hpp"
namespace navgen {
Json read_nav(std::span<const std::uint8_t> bytes);
Bytes write_nav(const Json& nav);
void validate_nav(const Json& nav);
}
