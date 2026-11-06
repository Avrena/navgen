#include "bsp.hpp"

namespace {
using namespace navgen;

void require(bool value, const char* message) {
    if (!value) throw std::runtime_error(message);
}
void u16(Bytes& data, size_t at, uint16_t value) {
    for (int i=0; i<2; ++i) data.at(at+i)=static_cast<uint8_t>(value>>(8*i));
}
void u32(Bytes& data, size_t at, uint32_t value) {
    for (int i=0; i<4; ++i) data.at(at+i)=static_cast<uint8_t>(value>>(8*i));
}
void f32(Bytes& data, size_t at, float value) { u32(data, at, std::bit_cast<uint32_t>(value)); }
void v3(Bytes& data, size_t at, const V3& value) {
    for (int i=0; i<3; ++i) f32(data, at+4*i, static_cast<float>(value[i]));
}
void directory_entry(Bytes& data, int number, uint32_t offset, uint32_t size,
                     uint32_t version=0, uint32_t fourcc=0, bool l4d2=false) {
    const std::array<uint32_t,4> fields=l4d2 ? std::array{version,offset,size,fourcc}
                                           : std::array{offset,size,version,fourcc};
    for (int i=0; i<4; ++i) u32(data, 8+16*number+4*i, fields[i]);
}
Bytes bsp_header(uint32_t version) {
    Bytes data(1036);
    data[0]='V'; data[1]='B'; data[2]='S'; data[3]='P'; u32(data,4,version);
    return data;
}
const std::string entities="{\n\"classname\" \"worldspawn\"\n}\n"
    "{\n\"classname\" \"info_player_start\"\n\"origin\" \"32 64 16\"\n}\n";
Bytes entity_bytes() {
    Bytes data(entities.begin(),entities.end()); data.push_back(0); return data;
}
Bytes compressed_entities(bool end_marker) {
    // Source's 17-byte LZMA header and a raw LZMA1 stream of entity_bytes().
    // Kept fixed so this CTest needs no Python or encoder dependency.
    constexpr std::string_view with_end_marker=
        "4c5a4d41560000004e0000005d00001000003d8280452210f174dbd4fb9bd48080a9a8"
        "39f63212b9b099241da89ed04d173498bb8db3a462679e086a7e5dc9dc5f5810a3f6f"
        "de0809facf47afece571cb0e3868d0e15d83a6db3bffcb44c00";
    // liblzma 5.8.1 LZMA1EXT, ext_flags=0: a complete stream with no marker.
    constexpr std::string_view without_end_marker=
        "4c5a4d4156000000490000005d00001000003d8280452210f174dbd4fb9bd48080a9a8"
        "39f63212b9b099241da89ed04d173498bb8db3a462679e086a7e5dc9dc5f5810a3f6f"
        "de0809facf47afece571cb0e3868d0e15d2000000";
    const auto hex=end_marker ? with_end_marker : without_end_marker;
    Bytes data;
    auto nibble=[](char c) { return c<='9' ? c-'0' : c-'a'+10; };
    for (size_t i=0; i<hex.size(); i+=2)
        data.push_back(static_cast<uint8_t>((nibble(hex[i])<<4)|nibble(hex[i+1])));
    return data;
}
Bytes geometry_bsp(uint32_t version, bool l4d2, bool compressed=false, bool with_entities=true, bool end_marker=true) {
    Bytes data=bsp_header(version);
    auto add=[&](int number, const Bytes& payload, uint32_t lump_version=0, uint32_t fourcc=0) {
        data.resize((data.size()+3)&~size_t(3));
        directory_entry(data,number,static_cast<uint32_t>(data.size()),
                        static_cast<uint32_t>(payload.size()),lump_version,fourcc,l4d2);
        data.insert(data.end(),payload.begin(),payload.end());
    };
    if (with_entities) add(0,compressed ? compressed_entities(end_marker) : entity_bytes(),0,
                           compressed ? static_cast<uint32_t>(entity_bytes().size()) : 0);
    const std::array<V4,6> planes{{{1,0,0,128},{-1,0,0,0},{0,1,0,128},
                                  {0,-1,0,0},{0,0,1,0},{0,0,-1,16}}};
    Bytes pl(120);
    for (size_t i=0; i<planes.size(); ++i) {
        for (int a=0; a<4; ++a) f32(pl,i*20+a*4,static_cast<float>(planes[i][a]));
        u32(pl,i*20+16,static_cast<uint32_t>(i/2));
    }
    add(1,pl);
    const std::array<V3,4> corners{{{0,0,0},{0,128,0},{128,128,0},{128,0,0}}};
    Bytes vertices(48);
    for (size_t i=0; i<corners.size(); ++i) v3(vertices,12*i,corners[i]);
    add(3,vertices);
    Bytes texinfo(72); u32(texinfo,64,0x20); add(6,texinfo);
    Bytes face(56); u16(face,0,4); face[3]=1; u16(face,8,4); add(7,face);
    // Refer to the second leaf so a wrong 56-byte stride cannot pass.
    Bytes leaves(64); u16(leaves,32+26,1); add(10,leaves,1);
    Bytes edges(16),surfedges(16);
    for (int i=0; i<4; ++i) { u16(edges,4*i,i); u16(edges,4*i+2,(i+1)%4); u32(surfedges,4*i,i); }
    add(12,edges); add(13,surfedges);
    Bytes model(48); v3(model,0,{0,0,-16}); v3(model,12,{128,128,4});
    u32(model,36,0xfffffffeu); u32(model,44,1); add(14,model);
    add(17,Bytes(2));
    Bytes brush(12); u32(brush,4,6); u32(brush,8,1); add(18,brush);
    Bytes sides(48);
    for (int i=0; i<6; ++i) { u16(sides,8*i,i); u16(sides,8*i+4,0xffff); }
    add(19,sides);
    Bytes disp(176); u32(disp,20,2); u32(disp,32,1); add(26,disp);
    Bytes dispverts(25*20);
    for (int i=0; i<25; ++i) { f32(dispverts,i*20+8,1); f32(dispverts,i*20+12,4); }
    add(33,dispverts);
    Bytes tags(64);
    for (int i=0; i<32; ++i) u16(tags,2*i,i==1 ? 32 : 1);
    add(48,tags);

    // L4D2 sprp v9 is 72 bytes, including disableX360 and its padding.
    // Two props make a wrong stride visible; game-lump offsets are absolute.
    // Dictionary: 4+128; leaves: 4+2*2; count: 4 = 144 bytes.
    Bytes props(144+2*72); u32(props,0,1);
    const std::string name="Models\\Props\\Test.mdl";
    std::copy(name.begin(),name.end(),props.begin()+4);
    u32(props,132,2); u16(props,138,1); u32(props,140,2);
    for (int i=0; i<2; ++i) {
        const size_t at=144+size_t(i)*72;
        v3(props,at,i==0 ? V3{8,16,32} : V3{80,96,128});
        v3(props,at+12,i==0 ? V3{0,90,0} : V3{0,0,45});
        u16(props,at+28,1); props[at+30]=i==0 ? 6 : 2;
        f32(props,at+56,1); props[at+60]=1; props[at+61]=3; props[at+62]=2; props[at+63]=4;
        props[at+64]=255; props[at+65]=128; props[at+66]=64; props[at+67]=255; props[at+68]=1;
    }
    const auto game_offset=static_cast<uint32_t>((data.size()+3)&~size_t(3));
    Bytes game(20); u32(game,0,1); u32(game,4,0x73707270); u16(game,10,9);
    u32(game,12,game_offset+20); u32(game,16,static_cast<uint32_t>(props.size()));
    game.insert(game.end(),props.begin(),props.end()); add(35,game);
    return data;
}
struct TemporaryFile {
    fs::path directory,path;
    TemporaryFile() {
        auto stamp=std::chrono::steady_clock::now().time_since_epoch().count();
        for (unsigned suffix=0;;++suffix) {
            directory=fs::temp_directory_path()/("navgen-bsp-"+std::to_string(stamp)+"-"+std::to_string(suffix));
            if (fs::create_directory(directory)) break;
        }
        path=directory/"fixture.bsp";
    }
    ~TemporaryFile() { std::error_code ignored; fs::remove_all(directory,ignored); }
    BSP read(const Bytes& data) { write_file(path,data); return BSP(path); }
};
void rejected(TemporaryFile& file, const Bytes& data, std::string_view reason) {
    try { file.read(data); }
    catch (const Error& e) {
        require(e.kind=="ValueError","invalid BSP should raise ValueError");
        require(std::string_view(e.what()).find(reason)!=std::string_view::npos,"unexpected BSP rejection reason");
        return;
    }
    throw std::runtime_error("invalid BSP was accepted");
}
void geometry_checks(TemporaryFile& file, uint32_t version, bool l4d2) {
    auto bsp=file.read(geometry_bsp(version,l4d2));
    require(bsp.version==version,"BSP version retained");
    require(bsp.leaf_stride==32 && bsp.lumps[10][2]==1,"normalised leaf lump version");
    require(bsp.model_brushes(0)==std::vector<int>{0},"second leaf's brush reference");
    require(bsp.seeds()==std::vector<V3>{{32,64,16}},"entity seed");
    auto brush=bsp.brush(0);
    require(brush.planes.size()==6 && brush.sides.size()==6,"brush plane/side count");
    require(brush.flags==std::vector<int64_t>(6,0x20),"brush texture flags");
    require(brush.lo==V3{0,0,-16} && brush.hi==V3{128,128,0},"floor bounds");
    auto [points,normal]=bsp.face_vertices(0);
    require(points==std::vector<V3>{{0,0,0},{0,128,0},{128,128,0},{128,0,0}},"face vertices");
    require(normal==V3{0,0,1},"face normal");
    auto triangles=bsp.displacements();
    require(triangles.size()==31,"removed displacement triangle");
    require(triangles[0].points==Tri{{{0,0,4},{32,32,4},{0,32,4}}},"displacement triangle coordinates");
    require(triangles[0].contents==1 && triangles[0].tags==1,"displacement contents and tags");
    auto props=bsp.static_props();
    require(props.size()==2,"v9 prop count");
    require(props[0].name=="models/props/test.mdl" && props[1].name==props[0].name,"prop model names");
    require(props[0].solid==6 && props[1].solid==2,"v9 prop solidity");
    require(props[0].origin==V3{8,16,32} && props[1].origin==V3{80,96,128},"v9 prop origins");
    require(props[0].angles==V3{0,90,0} && props[1].angles==V3{0,0,45},"v9 prop angles");
    require(props[0].scale==1 && props[1].scale==1,"v9 props remain unscaled");
    for (bool end_marker:{false,true}) {
        auto compressed=file.read(geometry_bsp(version,l4d2,true,true,end_marker));
        require(compressed.lump(0)==entity_bytes(),"Source LZMA entities");
        require(compressed.lumps[0][3]==entity_bytes().size(),"uncompressed-size field preserved");
        require(compressed.seeds()==bsp.seeds() && compressed.model_brushes(0)==bsp.model_brushes(0),"compressed BSP semantics");
    }
    auto no_entities=file.read(geometry_bsp(version,l4d2,false,false));
    require(no_entities.entities.empty() && no_entities.leaf_stride==32,"layout detection with empty entity lump");
    require(no_entities.model_brushes(0)==std::vector<int>{0},"non-entity directory evidence");
    auto hdr=geometry_bsp(version,l4d2);
    std::copy_n(hdr.begin()+8+16*7,16,hdr.begin()+8+16*58);
    std::fill_n(hdr.begin()+8+16*7,16,0);
    auto hdr_bsp=file.read(hdr);
    require(hdr_bsp.face_vertices(0)==bsp.face_vertices(0),"HDR faces when LDR faces are absent");
    require(hdr_bsp.displacements().size()==31,"HDR displacement geometry");
}
void directory_checks(TemporaryFile& file, uint32_t version, bool l4d2) {
    require(file.read(bsp_header(version)).entities.empty(),"empty directory");
    auto empty=bsp_header(version);
    directory_entry(empty,10,1036,0,1,0,l4d2);
    auto bsp=file.read(empty);
    require(bsp.leaf_stride==32 && bsp.lump(10).empty(),"empty versioned lump at EOF");
    const auto source=geometry_bsp(version,l4d2);
    const auto size_at_eof=static_cast<uint32_t>(source.size());
    for (auto [offset,size]:std::array<std::pair<uint32_t,uint32_t>,4>{{{100,1},{size_at_eof,1},{size_at_eof+1,0},{0xfffffffcu,16}}}) {
        auto invalid=source;
        directory_entry(invalid,63,offset,size,0,0,l4d2);
        rejected(file,invalid,"BSP lump directory");
    }
}
void compressed_length_checks(TemporaryFile& file, uint32_t version, bool l4d2) {
    auto replace_entities=[&](const Bytes& payload) {
        auto data=geometry_bsp(version,l4d2);
        directory_entry(data,0,static_cast<uint32_t>(data.size()),static_cast<uint32_t>(payload.size()),0,0,l4d2);
        data.insert(data.end(),payload.begin(),payload.end());
        return data;
    };
    // Even an empty raw LZMA stream requires the five-byte range state.
    for (uint32_t packed:{0u,1u,4u,5u}) {
        Bytes payload(17+packed);
        payload[0]='L'; payload[1]='Z'; payload[2]='M'; payload[3]='A';
        u32(payload,8,packed); payload[12]=0x5d; u32(payload,13,1u<<20);
        auto data=replace_entities(payload);
        if (packed<5) rejected(file,data,"compressed BSP lump");
        else {
            auto bsp=file.read(data);
            require(bsp.entities.empty() && bsp.model_brushes(0)==std::vector<int>{0},"empty no-marker LZMA stream");
        }
    }
    for (auto [actual,packed]:std::array<std::pair<uint32_t,uint32_t>,3>{{{85,73},{87,73},{86,74}}}) {
        auto payload=compressed_entities(false); u32(payload,4,actual); u32(payload,8,packed);
        rejected(file,replace_entities(payload),"compressed BSP lump");
    }
}
}
int main() {
    try {
        TemporaryFile file;
        for (auto [version,l4d2]:std::array<std::pair<uint32_t,bool>,3>{{{20,false},{21,false},{21,true}}}) {
            geometry_checks(file,version,l4d2); directory_checks(file,version,l4d2);
            compressed_length_checks(file,version,l4d2);
        }
        auto ambiguous=bsp_header(21); ambiguous.resize(2072);
        u32(ambiguous,8+16*63,1036); u32(ambiguous,8+16*63+4,1036); u32(ambiguous,8+16*63+8,16);
        rejected(file,ambiguous,"Ambiguous BSP v21");
        auto short_header=bsp_header(21); short_header.pop_back();
        rejected(file,short_header,"Not a Source BSP");
        rejected(file,Bytes{'V','B','S','P'},"Not a Source BSP");
        auto bad_magic=bsp_header(21); bad_magic[0]='X'; rejected(file,bad_magic,"Not a Source BSP");
        for (uint32_t version:{19u,22u}) rejected(file,bsp_header(version),std::to_string(version));
        rejected(file,geometry_bsp(20,true),"BSP lump directory");
        std::cout<<"native BSP v20/v21 checks passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr<<e.what()<<'\n'; return 1;
    }
}
