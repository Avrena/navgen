#pragma once
#include "common.hpp"
#include "gma.hpp"
#include "planes.hpp"
#include <filesystem>
#include <map>
#include <memory>
#include <optional>

namespace navgen {
inline constexpr int64_t BRUSH_MASK = 0x2400B;
inline constexpr int64_t NPC_MASK = BRUSH_MASK | 0x2000000;
// A Steam root followed by the library folders listed in its libraryfolders.vdf.
std::vector<std::filesystem::path> steam_libraries(const std::filesystem::path& root);
// The first garrysmod directory in the Steam libraries, else the default Steam location.
std::filesystem::path find_game();
using Entity = nlohmann::ordered_map<std::string, std::string>;
using M3 = std::array<V3, 3>;
V3 vector(const std::string& value, V3 fallback={0.,0.,0.});
V3 entity_angles(const Entity& entity);
M3 rotation(const V3& angles);
std::vector<V3> box_vertices(const V3& lo, const V3& hi);
struct BrushRecord { int32_t first, count; uint32_t contents; };
struct BrushSide { uint16_t plane; int16_t tex, disp; uint8_t bevel, thin; };
struct BrushData {
    std::vector<V4> planes;
    std::vector<int64_t> flags;
    V3 lo, hi;
    std::vector<BrushSide> sides;
};
struct Displacement { Tri points; int64_t contents; uint16_t tags; };
struct StaticProp { std::string name; int solid; V3 origin, angles; double scale; };
class BSP {
public:
    std::filesystem::path path;
    Bytes data;
    uint32_t version;
    std::array<std::array<uint32_t,4>,64> lumps;
    std::vector<Entity> entities;
    std::vector<V4> planes;
    std::vector<BrushRecord> brushes;
    std::vector<BrushSide> sides;
    std::vector<std::array<double,12>> models;
    Bytes nodes, leaves, texinfo, faces;
    size_t leaf_stride;
    std::vector<uint16_t> leafbrushes;
    std::vector<V3> vertices;
    std::vector<std::array<uint16_t,2>> edges;
    std::vector<int32_t> surfedges;
    explicit BSP(const std::filesystem::path& path);
    Bytes lump(int index) const;
    std::vector<int> model_brushes(int model) const;
    BrushData brush(int index) const;
    std::pair<std::vector<V3>,V3> face_vertices(int index) const;
    std::vector<Displacement> displacements() const;
    std::vector<StaticProp> static_props() const;
    std::vector<V3> seeds(const std::optional<std::filesystem::path>& file=std::nullopt) const;
};
class Content {
    struct Impl;
    std::shared_ptr<Impl> impl;
public:
    std::filesystem::path game;
    std::map<std::string,int64_t> used;
    std::set<std::string> missing;
    Json conflicts=Json::array(), archive_errors=Json::array();
    int64_t gma_archives=0;
    Content(const BSP& bsp, const std::filesystem::path& game);
    std::optional<Bytes> read(std::string name);
};
std::vector<std::vector<V3>> parse_phy(const Bytes& data);
std::vector<V4> convex_planes(const std::vector<V3>& points);
std::vector<V3> brush_vertices(const std::vector<V4>& planes, const std::vector<BrushSide>& sides);
}
