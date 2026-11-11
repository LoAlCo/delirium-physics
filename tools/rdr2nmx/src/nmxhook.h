// rdr2nmx Mode B: four inline hooks that record, per NM step, the shot agent's world contacts, 0x2842988's choice
// of the contact that goes, and the legs' muscle state. Shared by the ASI (rdr2nmx.cpp) and the offline test
// (test/hooktest.cpp), which runs the same code on stubs and on fake game memory.
//
// Hooks (RVAs in the build studied; every one's first 64 bytes are in the hook gate):
//   0x2a5ad28  the per-agent NM step (mgr, idx, dt): once per physics step per agent. Exit = the step's muscle
//              state as handed to Rage; ctx+0x47b0..b8 line the step up with Mode A's per-frame records.
//   0x2842988  the contact refresh of one manifold at the head of a pair's update (rdr2_helpers "Rage's collide"):
//              entry = the manifold as the last solve left it (impulses final), exit = after ageing and the removal.
//   0x2844140  remove contact idx from a manifold: recorded only when called from inside a recorded 0x2842988 on
//              the same thread and manifold (call site 0x2842d88: the margin re-query; 0x2842eee: the final one).
//   0x285b248  the slid contact's spare test: recorded only from inside a recorded 0x2842988.
// Each is a 5-byte rel32 jmp to a relay near the module, written as one aligned 8-byte store with every other
// thread suspended and none inside the bytes replaced. Trampolines hold the stolen instructions (decoded here;
// RIP-relative or branch instructions refuse the hook) and a jump back. Hooks never allocate, lock or do file I/O:
// they copy (SEH-guarded) into a fixed lock-free multi-producer ring that the script thread drains each frame.
#pragma once
#include <windows.h>
#include <tlhelp32.h>
#include <intrin.h>
#include <atomic>
#include <cstdint>
#include <cstring>
#include "nmxfile.h"

#pragma intrinsic(_ReturnAddress)

namespace nmxh {

// ---- what is hooked ----
constexpr uint32_t RVA_STEP = 0x2a5ad28, RVA_REFRESH = 0x2842988, RVA_REMOVE = 0x2844140, RVA_SPARE = 0x285b248;
constexpr uint32_t RET_REMOVE_MARGIN = 0x2842d88;  // 0x2842d83 call: in the loop, a contact whose margin moved
constexpr uint32_t RET_REMOVE_FINAL = 0x2842eee;   // 0x2842ee9 call: the one removal at the end (slid or far)
constexpr uint32_t RET_SPARE = 0x2842de3;          // 0x2842dde call

// The capture window, NM time (ctx+0x47b8) from the shot agent's first step after the shot.
constexpr float WINDOW_S = 0.40f;
// An agent counts as the shot one when it is new (ctx+0x47b8 under this at its step's entry) after arming.
constexpr float NEW_AGENT_S = 0.05f;

// ---- game layout (study notes; [C] unless marked) ----
constexpr uint32_t NM_INST_VT = 0x3662e50;   // NM physics instance class (rdr2_core2 6.1)
constexpr uint32_t NM_INST_CB = 0x2a0f05c;   // its vtable +0xf8, the contact callback
constexpr uint32_t INST_AGENT = 0x180;       // agent index on the instance, -1 none (0x2a0eff4)
constexpr uint32_t MGR_AGENTS = 0x1a0, CTX_INSERTED = 0x4803, CTX_DT = 0x47b0;   // +0x47b0 dt, +0x47b4, +0x47b8 time
constexpr uint32_t CTX_JOINTS = 0x3130, JOINT_RAGE_HINGE = 0xf8, JOINT_RAGE_3DOF = 0x100;
constexpr uint32_t VT_HINGE = 0x3663d98, VT_3DOF = 0x3663ec0;
constexpr int LEG_JOINTS[6] = {0, 3, 4, 1, 5, 6};   // hip, knee, ankle left; hip, knee, ankle right (rdr2_core2 1.1)
constexpr uint32_t MAN_CONTACTS = 0x70, MAN_COUNT = 0xfe, MAN_FLAGS = 0xff, MAN_COMP0 = 0xf4, MAN_COMP1 = 0xf6;
constexpr uint32_t MAN_COPY = 0x100, CONTACT_COPY = 0x100;
constexpr int MAX_CONTACTS = 8;   // the code keeps at most 6 (0x28236e4)

// .data constants 0x2842988 compares with, written into HOOKINFO so the reader has the build's values.
struct Const { uint32_t rva, size; const char* what; };
constexpr Const CONSTS[] = {
    {0x364e030, 16, "slid squared over which the most slid contact goes (0x2842988)"},
    {0x33b8850, 16, "far bound before the fixed side's min (0x2842988)"},
    {0x3c9f23c, 4, "margin change that drops a contact in the loop (0x2842988)"},
    {0x330f494, 4, "margin sentinel: none (0x2842988)"},
    {0x364e010, 16, "normal dot for the manifold's normal reset (0x2842988)"},
    {0x3c9f242, 4, "0x285b248's gate byte"},
};

// ---- guarded reads (the only SEH in Mode B) ----
inline bool safe_copy(void* dst, uintptr_t src, size_t n) {
    __try { memcpy(dst, reinterpret_cast<const void*>(src), n); return true; }
    __except (EXCEPTION_EXECUTE_HANDLER) { return false; }
}
template <typename T> inline T rd(uintptr_t at, T fallback = T{}) { T v; return (at && safe_copy(&v, at, sizeof v)) ? v : fallback; }

// ---- the x64 length decoder (prologue subset) ----
// Returns the instruction's length, or -1 for anything it doesn't know. rip: a RIP-relative operand; branch: a
// jump, call or return (either makes the caller refuse to relocate the instruction).
inline int insn_len(const uint8_t* p, bool& rip, bool& branch) {
    rip = branch = false;
    int i = 0;
    bool opsz = false;
    while (p[i] == 0x66 || p[i] == 0xf2 || p[i] == 0xf3) { opsz |= p[i] == 0x66; if (++i > 3) return -1; }
    bool rexw = false;
    if ((p[i] & 0xf0) == 0x40) { rexw = (p[i] & 8) != 0; i++; }
    uint8_t op = p[i++];
    auto modrm = [&](int imm) -> int {
        uint8_t m = p[i++];
        int mod = m >> 6, rm = m & 7;
        if (mod != 3 && rm == 4) { uint8_t sib = p[i++]; if (mod == 0 && (sib & 7) == 5) i += 4; }
        if (mod == 0 && rm == 5) { rip = true; i += 4; }
        else if (mod == 1) i += 1;
        else if (mod == 2) i += 4;
        return i + imm;
    };
    if (op == 0x0f) {
        uint8_t op2 = p[i++];
        if ((op2 & 0xf0) == 0x80) { branch = true; return -1; }
        switch (op2) {
        case 0x10: case 0x11: case 0x28: case 0x29: case 0x1f: case 0x57: case 0xb6: case 0xb7: case 0xbe: case 0xbf:
            return modrm(0);
        default: return -1;
        }
    }
    if (op >= 0x50 && op <= 0x5f) return i;                  // push / pop
    if (op >= 0xb8 && op <= 0xbf) return i + (rexw ? 8 : opsz ? 2 : 4);
    if ((op >= 0x70 && op <= 0x7f) || op == 0xe8 || op == 0xe9 || op == 0xeb || op == 0xc2 || op == 0xc3 || op == 0xcc ||
        op == 0xe3 || (op >= 0xe0 && op <= 0xe2)) { branch = true; return -1; }
    switch (op) {
    case 0x01: case 0x03: case 0x09: case 0x0b: case 0x21: case 0x23: case 0x29: case 0x2b: case 0x31: case 0x33:
    case 0x39: case 0x3b: case 0x63: case 0x84: case 0x85: case 0x88: case 0x89: case 0x8a: case 0x8b: case 0x8d:
        return modrm(0);
    case 0x80: case 0x83: case 0xc0: case 0xc1: case 0xc6: return modrm(1);
    case 0x81: case 0xc7: case 0x69: return modrm(opsz ? 2 : 4);
    case 0x90: return i;
    case 0xff: { int r = (p[i] >> 3) & 7; if (r >= 2 && r <= 5) { branch = true; return -1; } return modrm(0); }
    default: return -1;
    }
}

// ---- the ring: fixed size, multi-producer, one consumer ----
// Entry at an 8-aligned position: {u32 len, u32 commit, u32 tag, u32 agent, u32 step, u32 tid, u64 qpc,
// u32 body_len, u32 0, body}, len rounded to 8. A producer reserves by CAS on head (refused, and counted, when it
// wouldn't fit before tail), copies (wrapping), then stores commit last. The consumer stops at the first entry
// not yet committed.
constexpr uint32_t ENTRY_HEAD = 40;
struct Ring {
    uint8_t* buf = nullptr;
    uint64_t cap = 0;   // power of two
    std::atomic<uint64_t> head{0}, tail{0};
    std::atomic<uint64_t> dropped{0}, dropped_bytes{0}, entries{0};
};
inline Ring g_ring;

inline uint32_t commit_word(uint64_t pos) { return (uint32_t)(pos >> 3) | 0x80000000u; }

struct W { uint64_t pos, at, end; };

inline void ring_store(uint64_t at, const void* src, size_t n) {
    uint64_t off = at & (g_ring.cap - 1), first = (std::min)((uint64_t)n, g_ring.cap - off);
    memcpy(g_ring.buf + off, src, (size_t)first);
    if (n > first) memcpy(g_ring.buf, (const uint8_t*)src + first, (size_t)(n - first));
}
// Guarded copy from game memory straight into the ring; zeros on a fault.
inline bool ring_load(uint64_t at, uintptr_t src, size_t n) {
    uint64_t off = at & (g_ring.cap - 1), first = (std::min)((uint64_t)n, g_ring.cap - off);
    bool ok = true;
    if (!safe_copy(g_ring.buf + off, src, (size_t)first)) { memset(g_ring.buf + off, 0, (size_t)first); ok = false; }
    if (n > first && !safe_copy(g_ring.buf, src + (uintptr_t)first, (size_t)(n - first))) { memset(g_ring.buf, 0, (size_t)(n - first)); ok = false; }
    return ok;
}

inline bool ring_begin(W& w, uint32_t tag, uint32_t agent, uint32_t step, size_t body) {
    if (!g_ring.buf) return false;
    uint64_t n = (ENTRY_HEAD + body + 7) & ~7ull;
    if (n > g_ring.cap / 2) { g_ring.dropped++; g_ring.dropped_bytes += n; return false; }
    uint64_t h = g_ring.head.load(std::memory_order_relaxed);
    for (;;) {
        uint64_t t = g_ring.tail.load(std::memory_order_acquire);
        if (h + n - t > g_ring.cap) { g_ring.dropped++; g_ring.dropped_bytes += n; return false; }
        if (g_ring.head.compare_exchange_weak(h, h + n, std::memory_order_acq_rel, std::memory_order_relaxed)) break;
    }
    w.pos = h; w.end = h + n; w.at = h + ENTRY_HEAD;
    LARGE_INTEGER q; QueryPerformanceCounter(&q);
    uint32_t hd[4] = {tag, agent, step, GetCurrentThreadId()};
    uint64_t qpc = (uint64_t)q.QuadPart;
    uint32_t bl[2] = {(uint32_t)body, 0};
    ring_store(h + 8, hd, 16);
    ring_store(h + 24, &qpc, 8);
    ring_store(h + 32, bl, 8);
    return true;
}
inline void w_obj(W& w, uint32_t kind, uint64_t addr, uint32_t vt, uint32_t size) {
    nmxf::ObjHead o{kind, addr, vt, size};
    ring_store(w.at, &o, sizeof o); w.at += sizeof o;
}
inline void w_put(W& w, const void* p, size_t n) { ring_store(w.at, p, n); w.at += n; }
inline void w_load(W& w, uintptr_t src, size_t n) { ring_load(w.at, src, n); w.at += n; }
inline void ring_commit(W& w) {
    uint64_t pad = w.end - w.at;
    if (pad) { uint8_t z[8] = {}; ring_store(w.at, z, (size_t)pad); }
    uint32_t len = (uint32_t)(w.end - w.pos);
    uint8_t* hdr = g_ring.buf + (w.pos & (g_ring.cap - 1));
    *(volatile uint32_t*)hdr = len;
    InterlockedExchange((volatile LONG*)(hdr + 4), (LONG)commit_word(w.pos));   // release: everything above first
    g_ring.entries++;
}

struct Entry { uint32_t tag, agent, step, tid; uint64_t qpc; };
// Consumer: calls f(entry, body, size) for each committed entry in order. Script thread only.
template <class F> size_t ring_drain(std::vector<uint8_t>& scratch, F f) {
    if (!g_ring.buf) return 0;
    size_t n = 0;
    uint64_t t = g_ring.tail.load(std::memory_order_relaxed), h = g_ring.head.load(std::memory_order_acquire);
    while (t < h) {
        uint8_t* hdr = g_ring.buf + (t & (g_ring.cap - 1));
        if (*(volatile uint32_t*)(hdr + 4) != commit_word(t)) break;   // a producer is still writing it
        _ReadWriteBarrier();
        uint32_t len = *(volatile uint32_t*)hdr;
        if (len < ENTRY_HEAD || len > g_ring.cap / 2 || (len & 7)) break;   // can't happen; stop rather than read garbage
        scratch.resize(len);
        uint64_t off = t & (g_ring.cap - 1), first = (std::min)((uint64_t)len, g_ring.cap - off);
        memcpy(scratch.data(), g_ring.buf + off, (size_t)first);
        if (len > first) memcpy(scratch.data() + first, g_ring.buf, (size_t)(len - first));
        Entry e;
        uint32_t body = 0;
        memcpy(&e.tag, &scratch[8], 4); memcpy(&e.agent, &scratch[12], 4); memcpy(&e.step, &scratch[16], 4);
        memcpy(&e.tid, &scratch[20], 4); memcpy(&e.qpc, &scratch[24], 8); memcpy(&body, &scratch[32], 4);
        if (body > len - ENTRY_HEAD) body = len - ENTRY_HEAD;
        f(e, scratch.data() + ENTRY_HEAD, (size_t)body);
        *(volatile uint32_t*)(hdr + 4) = 0;
        *(volatile uint32_t*)hdr = 0;
        t += len;
        g_ring.tail.store(t, std::memory_order_release);
        n++;
    }
    return n;
}

inline bool ring_init(size_t cap) {
    if (g_ring.buf) return true;
    void* p = VirtualAlloc(nullptr, cap, MEM_RESERVE | MEM_COMMIT, PAGE_READWRITE);
    if (!p) return false;
    g_ring.cap = cap;
    g_ring.buf = (uint8_t*)p;   // never freed: a hook could still be writing when the DLL goes
    return true;
}

// ---- record structs (nmxfile kinds K_HSTEP...) ----
#pragma pack(push, 1)
struct HStep {
    uint32_t step;       // 1.. for the shot agent, from arming
    int32_t agent;
    float dt_arg;        // the driver's dt (xmm2)
    float pre[3];        // ctx+0x47b0, +0x47b4, +0x47b8 at entry
    float post[3];       // the same at exit
    float t_rel;         // pre[2] - the first step's pre[2]
    uint32_t flags;      // bit 0 balancer active, bit 1 first step of the window
    uint64_t ctx, bal;
    float bal_3c0[4];    // swing target
    uint8_t bal_550[16]; // +0x550 contact bits and what follows
    uint8_t bal_440[0x30]; // +0x440..+0x46f: +0x44c steps, +0x45c stepping foot, +0x46b support flags
    uint64_t qpc_entry;
};
struct HMan {
    uint32_t phase;      // 0 entry, 1 exit
    uint32_t call;       // links the entry, exit, removals and the spare of one 0x2842988 call
    int32_t side;        // which side is the shot agent's instance (0 / 1)
    int32_t agent;
    uint64_t inst[2];
    uint16_t comp[2];    // manifold +0xf4, +0xf6
    uint8_t count, flags;
    uint8_t do_remove, reset, p9, pad;
    int32_t threshold;   // age threshold (4 from 0x2842fb8)
    uint32_t step;       // the step counter when this ran
    uint32_t ret_rva;    // 0x2842988's caller
};
struct HRemove {
    uint32_t call; int32_t idx; uint32_t ret_rva; uint32_t count_before;
    uint64_t manifold, contact; uint32_t step, pad;
};
struct HSpare {
    uint32_t call; int32_t idx; uint32_t result; uint32_t step;
    uint64_t manifold, contact; float centre[4];
};
struct HStat {
    uint64_t entries, dropped, dropped_bytes, lost_no_file;
    uint32_t steps, manifolds, removes, spares, window_closed, target, target_how, pad;
};
#pragma pack(pop)

// ---- state ----
inline uintptr_t g_base;
inline std::atomic<int> g_armed{0};        // the script thread armed a capture at a shot
inline std::atomic<int> g_open{0};         // the hooks write
inline std::atomic<int> g_target{-1};      // the shot agent's index, -1 not known yet
inline std::atomic<int> g_target_how{0};   // 1 new agent seen by the step hook, 2 nearest agent by the script thread
inline std::atomic<uint32_t> g_step{0};
inline std::atomic<uint32_t> g_t0{0};      // float bits; valid when g_t0_set
inline std::atomic<int> g_t0_set{0};
inline std::atomic<int> g_closed{0};       // the window ran out (by NM time)
inline std::atomic<uint32_t> g_calls{0};
inline std::atomic<int> g_inflight{0};
inline std::atomic<uint32_t> g_n_step{0}, g_n_man{0}, g_n_rem{0}, g_n_spare{0};

// Diagnostics: raw calls while the window is open, whatever the target, and the first few refresh calls' arguments.
inline std::atomic<uint32_t> g_raw_ref{0}, g_raw_ref_nm{0}, g_raw_rem{0}, g_raw_spare{0};
struct RefSample { uint32_t ret, vt[4]; int ag[4]; int raw180[2]; };
constexpr int N_SAMPLES = 12;
inline RefSample g_samples[N_SAMPLES];
inline std::atomic<uint32_t> g_n_samples{0};

// The physics split 0x7037ec: 2 slices when the frame step (0x59b393c) is at least 1/60 (.rdata 0x338e2d0) and
// byte +0x14 of [[0x5978610] + 0x10] is set; the slice step goes to 0x59b390c, the count to 0x39b783c.
constexpr uint32_t RVA_FRAME_STEP = 0x59b393c, RVA_SLICE_STEP = 0x59b390c, RVA_SLICE_COUNT = 0x39b783c, RVA_SIM = 0x5978610;
struct SliceSample { float dt, frame, slice; int32_t count; uint8_t flag; uint32_t sub_vt; uint64_t qpc; };
constexpr int N_SLICE = 48;
inline SliceSample g_slice[N_SLICE];
inline std::atomic<uint32_t> g_n_slice{0};

struct InFlight { InFlight() { g_inflight++; } ~InFlight() { g_inflight--; } };

struct RefreshCtx { uintptr_t manifold; uint32_t call; int32_t agent; int active; };
inline thread_local RefreshCtx t_refresh;

using FnStep = void (*)(uintptr_t mgr, int idx, float dt);
// rax is passed back: the call at 0x2842f9d is a tail of its function (0x2842fa2 returns straight after)
using FnRefresh = uint64_t (*)(uintptr_t man, int thr, uint8_t do_remove, uint8_t reset, uintptr_t m0, uintptr_t m1,
                           uintptr_t inst0, uintptr_t inst1, uint8_t p9);
using FnRemove = uint64_t (*)(uintptr_t man, int idx);
using FnSpare = uint64_t (*)(uintptr_t man, int idx, uintptr_t inst0, uintptr_t inst1, uintptr_t centre);
inline FnStep o_step;
inline FnRefresh o_refresh;
inline FnRemove o_remove;
inline FnSpare o_spare;

inline uint32_t rva_of(uintptr_t a) { return (a > g_base && a - g_base < 0x8000000) ? (uint32_t)(a - g_base) : 0; }

// The agent index of an NM instance, -1 for any other instance.
inline int nm_agent(uintptr_t inst) {
    if (!inst) return -1;
    uintptr_t vt = rd<uintptr_t>(inst);
    if (vt != g_base + NM_INST_VT) {
        if (!rva_of(vt) || rd<uintptr_t>(vt + 0xf8) != g_base + NM_INST_CB) return -1;
    }
    int a = rd<int>(inst + INST_AGENT, -1);
    return (a >= 0 && a < 32) ? a : -1;
}

// ---- script-thread controls ----
inline void arm() {
    g_open = 0;
    g_target = -1; g_target_how = 0; g_step = 0; g_t0_set = 0; g_closed = 0;
    g_n_step = 0; g_n_man = 0; g_n_rem = 0; g_n_spare = 0;
    g_raw_ref = 0; g_raw_ref_nm = 0; g_raw_rem = 0; g_raw_spare = 0; g_n_samples = 0; g_n_slice = 0;
    g_armed = 1;
    g_open = 1;
}
inline void disarm() { g_open = 0; g_armed = 0; }
inline bool select_target(int idx, int how) {
    int e = -1;
    if (g_target.compare_exchange_strong(e, idx)) { g_target_how = how; return true; }
    return false;
}

// ---- records ----
inline void rec_step(const HStep& s, uintptr_t ctx) {
    uintptr_t jp[6], rj[6]; uint32_t jvt[6], jsz[6], rsz[6];
    size_t body = sizeof(nmxf::ObjHead) + sizeof s;
    for (int k = 0; k < 6; k++) {
        jp[k] = rd<uintptr_t>(ctx + CTX_JOINTS + 8 * LEG_JOINTS[k]);
        jvt[k] = rva_of(rd<uintptr_t>(jp[k]));
        bool hinge = jvt[k] == VT_HINGE;
        jsz[k] = jp[k] ? (hinge ? 0x150 : 0x1b0) : 0;
        rj[k] = jp[k] ? rd<uintptr_t>(jp[k] + (hinge ? JOINT_RAGE_HINGE : JOINT_RAGE_3DOF)) : 0;
        rsz[k] = rj[k] ? (hinge ? 0x2c0 : 0x540) : 0;
        body += 2 * sizeof(nmxf::ObjHead) + jsz[k] + rsz[k];
    }
    W w;
    if (!ring_begin(w, nmxf::HSTEP, (uint32_t)s.agent, s.step, body)) return;
    w_obj(w, nmxf::K_HSTEP, ctx, 0, sizeof s); w_put(w, &s, sizeof s);
    for (int k = 0; k < 6; k++) {
        w_obj(w, nmxf::K_JOINT, jp[k], jvt[k], jsz[k]); w_load(w, jp[k], jsz[k]);
        w_obj(w, nmxf::K_RAGE_JOINT, rj[k], rva_of(rd<uintptr_t>(rj[k])), rsz[k]); w_load(w, rj[k], rsz[k]);
    }
    ring_commit(w);
    g_n_step++;
}

inline void rec_manifold(uint32_t phase, uint32_t call, uintptr_t m, int side, int agent, uintptr_t i0, uintptr_t i1,
                         int thr, uint8_t do_remove, uint8_t reset, uint8_t p9, uint32_t ret_rva) {
    HMan h{};
    h.phase = phase; h.call = call; h.side = side; h.agent = agent; h.inst[0] = i0; h.inst[1] = i1;
    h.comp[0] = rd<uint16_t>(m + MAN_COMP0); h.comp[1] = rd<uint16_t>(m + MAN_COMP1);
    h.count = rd<uint8_t>(m + MAN_COUNT); h.flags = rd<uint8_t>(m + MAN_FLAGS);
    h.do_remove = do_remove; h.reset = reset; h.p9 = p9; h.threshold = thr; h.step = g_step.load(); h.ret_rva = ret_rva;
    int n = h.count > MAX_CONTACTS ? MAX_CONTACTS : h.count;
    uintptr_t c[MAX_CONTACTS];
    size_t body = 2 * sizeof(nmxf::ObjHead) + sizeof h + MAN_COPY;
    for (int k = 0; k < n; k++) {
        c[k] = rd<uintptr_t>(m + MAN_CONTACTS + 8 * k);
        body += sizeof(nmxf::ObjHead) + (c[k] ? CONTACT_COPY : 0);
    }
    W w;
    if (!ring_begin(w, nmxf::HMANIFOLD, (uint32_t)agent, h.step, body)) return;
    // aux: the first object's vtable RVA (the class, for the reader to group by)
    w_obj(w, nmxf::K_HMAN, m, i0 ? rva_of(rd<uintptr_t>(i0)) : 0, sizeof h); w_put(w, &h, sizeof h);
    w_obj(w, nmxf::K_MAN_RAW, m, 0, MAN_COPY); w_load(w, m, MAN_COPY);
    for (int k = 0; k < n; k++) { w_obj(w, nmxf::K_CONTACT, c[k], (uint32_t)k, c[k] ? CONTACT_COPY : 0); if (c[k]) w_load(w, c[k], CONTACT_COPY); }
    ring_commit(w);
    g_n_man++;
}

// ---- detours ----
inline void det_step(uintptr_t mgr, int idx, float dt) {
    InFlight in;
    if (!g_armed.load(std::memory_order_acquire) || idx < 0 || idx > 31) { o_step(mgr, idx, dt); return; }
    uintptr_t ctx = rd<uintptr_t>(mgr + MGR_AGENTS + 8 * idx);
    if (!ctx || !rd<uint8_t>(ctx + CTX_INSERTED)) { o_step(mgr, idx, dt); return; }
    HStep s{};
    float pre[3] = {};
    safe_copy(pre, ctx + CTX_DT, 12);
    if (g_target.load() == -1 && pre[2] < NEW_AGENT_S) select_target(idx, 1);
    if (g_target.load() != idx) { o_step(mgr, idx, dt); return; }
    if (!g_t0_set.load()) { uint32_t b; memcpy(&b, &pre[2], 4); g_t0 = b; g_t0_set = 1; s.flags |= 2; }
    uint32_t b = g_t0.load(); float t0; memcpy(&t0, &b, 4);
    if (pre[2] < t0) { memcpy(&b, &pre[2], 4); g_t0 = b; t0 = pre[2]; }   // re-inserted: start again
    s.t_rel = pre[2] - t0;
    if (s.t_rel > WINDOW_S) { g_open = 0; g_armed = 0; g_closed = 1; o_step(mgr, idx, dt); return; }
    {   // the frame's physics split (0x7037ec): frame step, slice step and count, and the flag it reads
        uint32_t k = g_n_slice.load();
        if (k < N_SLICE && g_n_slice.compare_exchange_strong(k, k + 1)) {
            SliceSample& z = g_slice[k];
            z.dt = dt; z.frame = rd<float>(g_base + RVA_FRAME_STEP); z.slice = rd<float>(g_base + RVA_SLICE_STEP);
            z.count = rd<int32_t>(g_base + RVA_SLICE_COUNT, -1);
            uintptr_t sim = rd<uintptr_t>(g_base + RVA_SIM), sub = sim ? rd<uintptr_t>(sim + 0x10) : 0;
            z.flag = sub ? rd<uint8_t>(sub + 0x14, 0xff) : 0xfe; z.sub_vt = sub ? rva_of(rd<uintptr_t>(sub)) : 0;
            LARGE_INTEGER q; QueryPerformanceCounter(&q); z.qpc = (uint64_t)q.QuadPart;
        }
    }
    s.step = g_step.fetch_add(1) + 1;
    s.agent = idx; s.dt_arg = dt; memcpy(s.pre, pre, 12); s.ctx = ctx;
    LARGE_INTEGER q; QueryPerformanceCounter(&q); s.qpc_entry = (uint64_t)q.QuadPart;
    o_step(mgr, idx, dt);
    safe_copy(s.post, ctx + CTX_DT, 12);
    // the balancer, behaviour id 1: row mgr+0x2a0+0x8+0x278*idx, slot +0x8+8*id; active byte +0x1c
    uintptr_t bal = rd<uintptr_t>(mgr + 0x2a0 + 0x8 + 0x278 * idx + 0x8 + 8 * 1);
    s.bal = bal;
    if (bal) {
        if (rd<uint8_t>(bal + 0x1c)) s.flags |= 1;
        safe_copy(s.bal_3c0, bal + 0x3c0, 16);
        safe_copy(s.bal_550, bal + 0x550, 16);
        safe_copy(s.bal_440, bal + 0x440, 0x30);
    }
    rec_step(s, ctx);
}

inline uint64_t det_refresh(uintptr_t m, int thr, uint8_t do_remove, uint8_t reset, uintptr_t m0, uintptr_t m1,
                            uintptr_t i0, uintptr_t i1, uint8_t p9) {
    InFlight in;
    if (!g_open.load(std::memory_order_acquire)) return o_refresh(m, thr, do_remove, reset, m0, m1, i0, i1, p9);
    int tgt = g_target.load();
    int a0 = nm_agent(i0), a1 = nm_agent(i1);
    g_raw_ref++;
    if (a0 >= 0 || a1 >= 0) g_raw_ref_nm++;
    {
        uint32_t k = g_n_samples.load();
        // the first calls, then ones that touch an NM instance on any of the four object arguments
        bool nm_any = a0 >= 0 || a1 >= 0 || nm_agent(m0) >= 0 || nm_agent(m1) >= 0;
        if (k < N_SAMPLES && (k < N_SAMPLES / 2 || nm_any) && g_n_samples.compare_exchange_strong(k, k + 1)) {
            RefSample& s = g_samples[k];
            uintptr_t o[4] = {m0, m1, i0, i1};
            s.ret = rva_of((uintptr_t)_ReturnAddress());
            for (int j = 0; j < 4; j++) { s.vt[j] = o[j] ? rva_of(rd<uintptr_t>(o[j])) : 0xffffffff; s.ag[j] = nm_agent(o[j]); }
            s.raw180[0] = i0 ? rd<int>(i0 + INST_AGENT, -99) : -98;
            s.raw180[1] = i1 ? rd<int>(i1 + INST_AGENT, -99) : -98;
        }
    }
    auto match = [&](int a) { return a >= 0 && (tgt == -1 || a == tgt); };
    int side = match(a0) ? 0 : match(a1) ? 1 : -1;
    // The first capture (2026-11-09) showed the pair's objects are never the NM instance (vtables 0x33cbbc8, 0x33cc0e8,
    // 0x33cbdb8), so every refresh in the window is recorded, side -1 when unmatched; the reader picks the shot
    // ped's manifolds by where their contacts are.
    if (t_refresh.active) return o_refresh(m, thr, do_remove, reset, m0, m1, i0, i1, p9);
    int agent = side < 0 ? (tgt >= 0 ? tgt : 0) : side ? a1 : a0;
    uint32_t call = ++g_calls;
    uint32_t ret = rva_of((uintptr_t)_ReturnAddress());
    rec_manifold(0, call, m, side, agent, i0, i1, thr, do_remove, reset, p9, ret);
    t_refresh = {m, call, agent, 1};
    uint64_t r = o_refresh(m, thr, do_remove, reset, m0, m1, i0, i1, p9);
    t_refresh.active = 0;
    rec_manifold(1, call, m, side, agent, i0, i1, thr, do_remove, reset, p9, ret);
    return r;
}

inline uint64_t det_remove(uintptr_t m, int idx) {
    InFlight in;
    if (g_open.load(std::memory_order_relaxed)) g_raw_rem++;
    if (t_refresh.active && t_refresh.manifold == m && g_open.load(std::memory_order_acquire)) {
        HRemove r{};
        r.call = t_refresh.call; r.idx = idx; r.ret_rva = rva_of((uintptr_t)_ReturnAddress());
        r.count_before = rd<uint8_t>(m + MAN_COUNT); r.manifold = m; r.step = g_step.load();
        r.contact = (idx >= 0 && idx < MAX_CONTACTS) ? rd<uintptr_t>(m + MAN_CONTACTS + 8 * idx) : 0;
        W w;
        uint32_t csz = r.contact ? CONTACT_COPY : 0;
        if (ring_begin(w, nmxf::HREMOVE, (uint32_t)t_refresh.agent, r.step, 2 * sizeof(nmxf::ObjHead) + sizeof r + csz)) {
            w_obj(w, nmxf::K_HREMOVE, m, 0, sizeof r); w_put(w, &r, sizeof r);
            w_obj(w, nmxf::K_CONTACT, r.contact, (uint32_t)idx, csz); if (csz) w_load(w, r.contact, csz);
            ring_commit(w);
            g_n_rem++;
        }
    }
    return o_remove(m, idx);
}

inline uint64_t det_spare(uintptr_t m, int idx, uintptr_t i0, uintptr_t i1, uintptr_t centre) {
    InFlight in;
    if (g_open.load(std::memory_order_relaxed)) g_raw_spare++;
    bool rec = t_refresh.active && t_refresh.manifold == m && g_open.load(std::memory_order_acquire);
    HSpare s{};
    if (rec) {
        s.call = t_refresh.call; s.idx = idx; s.manifold = m; s.step = g_step.load();
        s.contact = (idx >= 0 && idx < MAX_CONTACTS) ? rd<uintptr_t>(m + MAN_CONTACTS + 8 * idx) : 0;
        safe_copy(s.centre, centre, 16);
    }
    uint64_t r = o_spare(m, idx, i0, i1, centre);
    if (rec) {
        s.result = (uint8_t)r;
        W w;
        uint32_t csz = s.contact ? CONTACT_COPY : 0;
        if (ring_begin(w, nmxf::HSPARE, (uint32_t)t_refresh.agent, s.step, 2 * sizeof(nmxf::ObjHead) + sizeof s + csz)) {
            w_obj(w, nmxf::K_HSPARE, m, 0, sizeof s); w_put(w, &s, sizeof s);
            w_obj(w, nmxf::K_CONTACT, s.contact, (uint32_t)idx, csz); if (csz) w_load(w, s.contact, csz);
            ring_commit(w);
            g_n_spare++;
        }
    }
    return r;
}

// ---- installing ----
struct Hook {
    const char* name;
    uint32_t rva;
    int expect;          // the stolen length the build studied gives (checked against the decoder)
    void* detour;
    void** orig;
    uint8_t saved[8];
    uint8_t patch[8];
    uint8_t* tramp;
    int stolen;
    bool on;
};
inline Hook g_hooks[] = {
    {"step 0x2a5ad28", RVA_STEP, 7, (void*)&det_step, (void**)&o_step},
    {"refresh 0x2842988", RVA_REFRESH, 7, (void*)&det_refresh, (void**)&o_refresh},
    {"remove 0x2844140", RVA_REMOVE, 5, (void*)&det_remove, (void**)&o_remove},
    {"spare 0x285b248", RVA_SPARE, 7, (void*)&det_spare, (void**)&o_spare},
};
inline uint8_t* g_block;   // relays + trampolines, within +-2 GB of the module; never freed

using Log = void (*)(const char* fmt, ...);

// Free memory below the module (or, failing that, past its image), so every relay is in rel32 reach of every hook.
inline uint8_t* alloc_near(uintptr_t base, size_t image, size_t size) {
    SYSTEM_INFO si; GetSystemInfo(&si);
    uintptr_t gran = si.dwAllocationGranularity;
    for (uintptr_t d = gran; d < 0x60000000; d += gran) {
        if (d < base) {
            void* p = VirtualAlloc((void*)((base - d) & ~(gran - 1)), size, MEM_RESERVE | MEM_COMMIT, PAGE_EXECUTE_READWRITE);
            if (p) return (uint8_t*)p;
        }
        void* p = VirtualAlloc((void*)((base + image + d) & ~(gran - 1)), size, MEM_RESERVE | MEM_COMMIT, PAGE_EXECUTE_READWRITE);
        if (p) return (uint8_t*)p;
    }
    return nullptr;
}

// Decodes the instructions at code until at least `need` bytes; -1 (and why) on anything it can't relocate.
inline int steal_len(const uint8_t* code, int need, const char** why) {
    int n = 0;
    while (n < need) {
        bool rip, branch;
        int l = insn_len(code + n, rip, branch);
        if (rip) { *why = "a RIP-relative instruction in the bytes to move"; return -1; }
        if (branch) { *why = "a branch in the bytes to move"; return -1; }
        if (l <= 0) { *why = "an instruction the decoder doesn't know"; return -1; }
        n += l;
    }
    return n;
}

struct Suspended { DWORD n = 0, missed = 0; bool ok = false; HANDLE h[2048]; };

// Suspends every other thread of the process. ok = false when the snapshot failed or there were more threads than
// room (then nothing may be patched). missed: threads that couldn't be opened or suspended.
inline void suspend_others(Suspended& s) {
    s.n = 0; s.missed = 0; s.ok = false;
    DWORD self = GetCurrentThreadId(), pid = GetCurrentProcessId();
    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPTHREAD, 0);
    if (snap == INVALID_HANDLE_VALUE) return;
    THREADENTRY32 te; te.dwSize = sizeof te;
    bool room = true;
    for (BOOL ok = Thread32First(snap, &te); ok; ok = Thread32Next(snap, &te)) {
        if (te.th32OwnerProcessID != pid || te.th32ThreadID == self) continue;
        if (s.n == 2048) { room = false; break; }
        HANDLE t = OpenThread(THREAD_SUSPEND_RESUME | THREAD_GET_CONTEXT, FALSE, te.th32ThreadID);
        if (!t) { s.missed++; continue; }
        if (SuspendThread(t) == (DWORD)-1) { CloseHandle(t); s.missed++; continue; }
        s.h[s.n++] = t;
    }
    CloseHandle(snap);
    s.ok = room;
}
inline void resume_others(Suspended& s) {
    for (DWORD i = 0; i < s.n; i++) { ResumeThread(s.h[i]); CloseHandle(s.h[i]); }
    s.n = 0;
}
// Whether any suspended thread stands inside the bytes a patch replaces (after the first).
inline bool someone_inside(Suspended& s) {
    for (DWORD i = 0; i < s.n; i++) {
        CONTEXT c; c.ContextFlags = CONTEXT_CONTROL;
        if (!GetThreadContext(s.h[i], &c)) continue;
        for (Hook& h : g_hooks) {
            uintptr_t a = g_base + h.rva;
            if (c.Rip > a && c.Rip < a + 8) return true;
        }
    }
    return false;
}

inline bool write8(uintptr_t at, const uint8_t* bytes) {
    DWORD old;
    if (!VirtualProtect((void*)at, 8, PAGE_EXECUTE_READWRITE, &old)) return false;
    int64_t v; memcpy(&v, bytes, 8);
    InterlockedExchange64((volatile LONG64*)at, v);
    VirtualProtect((void*)at, 8, old, &old);
    FlushInstructionCache(GetCurrentProcess(), (void*)at, 8);
    return true;
}

inline bool installed() { for (Hook& h : g_hooks) if (h.on) return true; return false; }

// All four or none. The prologues must already have passed the hook gate (their SHA-256).
inline bool install(Log log, size_t image_size) {
    if (installed()) return true;
    if (!ring_init(8u << 20)) { log("hooks: no ring memory"); return false; }
    for (Hook& h : g_hooks) {
        uintptr_t a = g_base + h.rva;
        if (a & 7) { log("hooks: %s not 8-byte aligned: no hooks", h.name); return false; }
        const char* why = "";
        h.stolen = steal_len((const uint8_t*)a, 5, &why);
        if (h.stolen < 0) { log("hooks: %s: %s: no hooks", h.name, why); return false; }
        if (h.stolen != h.expect) { log("hooks: %s: decoded %d bytes to move, the study has %d: no hooks", h.name, h.stolen, h.expect); return false; }
    }
    if (!g_block) g_block = alloc_near(g_base, image_size, 0x1000);
    if (!g_block) { log("hooks: no memory within 2 GB of the module: no hooks"); return false; }
    uint8_t* p = g_block;
    for (Hook& h : g_hooks) {
        uintptr_t a = g_base + h.rva;
        // relay: jmp [rip+0]; dq detour
        uint8_t* relay = p;
        relay[0] = 0xff; relay[1] = 0x25; memset(relay + 2, 0, 4); uint64_t d = (uint64_t)h.detour; memcpy(relay + 6, &d, 8);
        p += 16;
        // trampoline: the stolen instructions, then jmp [rip+0]; dq a + stolen
        h.tramp = p;
        memcpy(p, (const void*)a, h.stolen);
        uint8_t* j = p + h.stolen;
        j[0] = 0xff; j[1] = 0x25; memset(j + 2, 0, 4); uint64_t back = a + h.stolen; memcpy(j + 6, &back, 8);
        p += (h.stolen + 14 + 15) & ~15;
        int64_t rel = (int64_t)(uintptr_t)relay - (int64_t)(a + 5);
        if (rel < INT32_MIN || rel > INT32_MAX) { log("hooks: %s: relay out of rel32 reach: no hooks", h.name); return false; }
        memcpy(h.saved, (const void*)a, 8);
        memcpy(h.patch, h.saved, 8);
        h.patch[0] = 0xe9; int32_t r32 = (int32_t)rel; memcpy(h.patch + 1, &r32, 4);
        *h.orig = h.tramp;
    }
    FlushInstructionCache(GetCurrentProcess(), g_block, 0x1000);
    DWORD old; VirtualProtect(g_block, 0x1000, PAGE_EXECUTE_READ, &old);
    // patch with every other thread stopped outside the bytes being replaced
    static Suspended s;   // 16 KB; install runs on one thread
    for (int tries = 0; tries < 50; tries++) {
        suspend_others(s);
        if (!s.ok) { resume_others(s); log("hooks: couldn't stop the other threads: no hooks"); return false; }
        if (someone_inside(s)) { resume_others(s); Sleep(1); continue; }
        // nothing in here may take a lock a suspended thread could hold: no logging, no allocation
        DWORD missed = s.missed;
        bool ok = true;
        for (Hook& h : g_hooks) {
            if (!write8(g_base + h.rva, h.patch)) { ok = false; break; }
            h.on = true;
        }
        if (!ok) for (Hook& h : g_hooks) if (h.on) { write8(g_base + h.rva, h.saved); h.on = false; }
        resume_others(s);
        if (!ok) { log("hooks: VirtualProtect refused: no hooks"); return false; }
        if (missed) log("hooks: %lu threads couldn't be suspended (patched anyway: an aligned 8-byte store)", missed);
        for (Hook& h : g_hooks) log("hooks: %s on (%d bytes moved, trampoline %p)", h.name, h.stolen, h.tramp);
        return true;
    }
    log("hooks: a thread stayed inside a prologue for 50 tries: no hooks");
    return false;
}

// Puts the original bytes back (one aligned 8-byte store each: a thread sees either the jmp or the original), then
// waits for calls still in the detours. Trampolines and the ring stay allocated for threads still in them.
inline void uninstall(Log log) {
    disarm();
    bool any = false;
    for (Hook& h : g_hooks) if (h.on) { write8(g_base + h.rva, h.saved); h.on = false; any = true; }
    for (int i = 0; i < 200 && g_inflight.load() > 0; i++) Sleep(5);
    if (any && log) log("hooks: off (%d calls still inside)", g_inflight.load());
}

} // namespace nmxh
