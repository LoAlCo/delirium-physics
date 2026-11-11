// rdr2nmx: the .nmx container, shared by the ASI and the offline tests. Format 1, unchanged: a 4 KB header, then
// records of a 40-byte head {tag, len, seq, agent, step, qpc, tid, crc32(body)} and a body of objects
// {u32 kind, u64 addr, u32 vtable rva, u32 size, bytes}. Mode B adds record tags 7..13 (below); a reader that
// doesn't know a tag skips the record by its length, so old readers still read old and new files.
#pragma once
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace nmxf {

constexpr uint32_t FORMAT = 1, HEADER = 4096;

// Record tags. 1..6 are Mode A's (rdr2nmx.cpp); 7..13 are written by the Mode B hooks (nmxhook.h).
enum Tag : uint32_t {
    RIG = 1, GLOBALS = 2, STEP = 3, SCENARIO = 4, END = 5, NOTE = 6,
    HOOKINFO = 7,   // at file open with hooks on: hooked RVAs, gate, window, the .data constants 0x2842988 compares with
    HSTEP = 8,      // one NM step of the shot agent (0x2a5ad28 exit): step counter, dt / ctx+0x47b0..b8, legs' joints, bal
    HMANIFOLD = 9,  // a manifold of the shot agent with the world, at 0x2842988's entry (phase 0) and exit (phase 1)
    HREMOVE = 10,   // 0x2844140 called from inside that 0x2842988: which contact goes, from which call site
    HSPARE = 11,    // 0x285b248 called from inside that 0x2842988: the slid contact, spared or not
    HOOKSTAT = 12,  // at file close: counts, ring overflows
    HTARGET = 13,   // the agent the hooks follow, once it is known (how it was chosen, its collider window)
};

// Object kinds. 1..19 are Mode A's; 20.. are Mode B's. Mode B also uses K_JOINT (2), K_RAGE_JOINT (14), K_TEXT (17),
// K_COLLIDER (12) with their Mode A meaning.
enum Kind : uint32_t {
    K_JOINT = 2, K_COLLIDER = 12, K_RAGE_JOINT = 14, K_TEXT = 17,
    K_HSTEP = 20,      // struct HStep
    K_HMAN = 21,       // struct HMan
    K_MAN_RAW = 22,    // the manifold's first 0x100 bytes
    K_CONTACT = 23,    // one contact's first 0x100 bytes (addr = the contact)
    K_HREMOVE = 24,    // struct HRemove
    K_HSPARE = 25,     // struct HSpare
    K_HCONST = 26,     // a .data constant (addr = runtime address, bytes = its value)
    K_HSTAT = 27,      // struct HStat
    K_COLLIDER_MAP = 28, // *(collider+0x270), 0x100 bytes
};

#pragma pack(push, 1)
struct RecHead { uint32_t tag, len; uint64_t seq; uint32_t agent, step; uint64_t qpc; uint32_t tid, crc; };
struct ObjHead { uint32_t kind; uint64_t addr; uint32_t vt, size; };
#pragma pack(pop)
static_assert(sizeof(RecHead) == 40 && sizeof(ObjHead) == 20, "layout");

inline uint32_t crc32(const uint8_t* p, size_t n) {
    uint32_t c = 0xffffffff;
    for (size_t i = 0; i < n; i++) { c ^= p[i]; for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xedb88320 & (0 - (c & 1))); }
    return ~c;
}

inline void write_record(FILE* f, uint64_t& seq, uint32_t tag, uint32_t agent, uint32_t step, uint64_t qpc, uint32_t tid,
                         const uint8_t* body, size_t n) {
    RecHead h{tag, (uint32_t)n, seq++, agent, step, qpc, tid, crc32(body, n)};
    fwrite(&h, sizeof h, 1, f);
    if (n) fwrite(body, 1, n, f);
}

inline void write_header(FILE* f, uint64_t base, uint32_t image_size, uint32_t timestamp, char mode, const std::string& text) {
    std::vector<uint8_t> h(HEADER);
    memcpy(h.data(), "RDR2NMX", 8);
    uint32_t v[2] = {FORMAT, HEADER}; memcpy(&h[8], v, 8);
    memcpy(&h[16], &base, 8);
    uint32_t img[2] = {image_size, timestamp}; memcpy(&h[24], img, 8);
    h[32] = (uint8_t)mode;
    memcpy(&h[64], text.data(), (std::min)(text.size(), (size_t)4000));
    fwrite(h.data(), 1, h.size(), f);
}

// Appends one object to a body.
inline void put_obj(std::vector<uint8_t>& b, uint32_t kind, uint64_t addr, uint32_t vt, const void* bytes, uint32_t size) {
    ObjHead o{kind, addr, vt, size};
    b.insert(b.end(), (const uint8_t*)&o, (const uint8_t*)&o + sizeof o);
    if (size) b.insert(b.end(), (const uint8_t*)bytes, (const uint8_t*)bytes + size);
}

} // namespace nmxf
