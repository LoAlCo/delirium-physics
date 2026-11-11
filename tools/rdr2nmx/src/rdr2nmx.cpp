// rdr2nmx: exports Red Dead Redemption 2's NaturalMotion (Euphoria) state to files, for offline study.
// Mode A: no code patches. From the ScriptHook script thread, once per game frame, it reads the NM manager and every
// inserted agent (character context, joints, parts, behaviours, limb request queues) and writes raw blobs plus a
// few decoded fields. Read-only except the test-scenario natives (spawn a ped, shoot or push it, delete it).
// Mode B (nmxhook.h): during a shot scenario, four inline hooks record per NM step the shot agent's world contact
// manifolds, 0x2842988's choice of the contact that goes, and the legs' muscle state, for NM_WINDOW after the shot.
// Story mode only; refuses a game build other than the one studied (Mode A and the hooks have separate gates: a
// hook gate failure leaves Mode A working and installs no hook).
//
// Keys: F9 run the selected scenario (the first one, "capture", runs the leg and chest shots 3 times each),
//       Ctrl+F9 next scenario, F10 abort, F11 free logging on/off, F12 write RIG + GLOBALS for every agent now.
// Output: <game dir>\rdr2nmx\<time>_<name>.nmx and rdr2nmx.log. Disable: delete/rename rdr2nmx.asi, or create
// <game dir>\rdr2nmx\DISABLE (also takes the hooks out); <game dir>\rdr2nmx\NOHOOKS keeps Mode A and takes the hooks out.
#include <windows.h>
#include <bcrypt.h>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <string>
#include <vector>
#include "..\sdk\inc\types.h"
#include "..\sdk\inc\nativeCaller.h"
#include "..\sdk\inc\natives.h"
#include "..\sdk\inc\main.h"
#include "nmxfile.h"
#include "nmxhook.h"

#pragma comment(lib, "bcrypt.lib")
#pragma comment(lib, "user32.lib")

namespace {

using nmxf::FORMAT;
constexpr uintptr_t MGR_GLOBAL = 0x59afec8;     // NM manager pointer
constexpr size_t CTX_SIZE = 0x48e0;

// The build check: 64 bytes at each RVA must hash to these.
struct Gate { uint32_t rva; const char* sha; };
const Gate GATES[] = {
    {0x2a5ad28, "f8d8951d98c855aad54076f785504558269193d3b85e2b0a03a1807e58bb826a"},
    {0x2a4653c, "697e10749351b0c7d0848afd2dd5e3a36483aab00c6d94b984e311ad86222d89"},
    {0x2a3886c, "0c993244f95e7a3b7ea1bbeadef6f78b0fc2a908e0f5f927276ef28ab3affb22"},
    {0x2a0eff4, "e923488766a0a91f8d0a562c05c6aee5254cf85fbac551874604e51f48d4938a"},
    {0x2a0f804, "5637c8156a369e71ee99688bc43528ebd083fcb5069128a9dc0c1acc67c2fa00"},
    {0x2a3cb48, "a9f9bba0e2fde1ac203d08f9f12061790c3b4882bde907eefbaa29c83a9580ca"},
    {0x2a585a4, "1e2efcf4f882639cbfc3fd5b07829ec70234a88e2ef7d393a46e93c7c882d975"},
    {0x2a175e4, "7a2017ddc08bb58358c9c3c07bba0133059ba55103a6aa0757e1ce5975f61da1"},
    {0x2aabbe0, "b8050b9d0bd969b053a6de743b19f91cd88394caee4643bdf31e61947e19852a"},
    {0x2aabb48, "87bbc4d09359fbb3ebe0ada931d82423f1bd7fe9c352355db681ae0cde5bf751"},
    {0x2a3cd4c, "28546933a327808b362de7362522f2dfb57506e1c39dd513e779aa0a2a30065c"},
};
// The hook gate: every hooked function's first 64 bytes (the bytes moved to the trampolines and what follows).
const Gate HOOK_GATES[] = {
    {nmxh::RVA_STEP, "f8d8951d98c855aad54076f785504558269193d3b85e2b0a03a1807e58bb826a"},
    {nmxh::RVA_REFRESH, "b3733605470d65dce960006d3d7ddeff171fbaf402bd29e1a0976b7d482f98fd"},
    {nmxh::RVA_REMOVE, "095014d5b71daedb63c4358fb2a7222761d764976cf32cb93d33bf9aa4a2c249"},
    {nmxh::RVA_SPARE, "625c614595679323f96260b95066e35f45ea81ac28764cf82ada9603034cb290"},
};

using namespace nmxf;   // record tags and object kinds
// Mode A's object kinds (Mode B's are in nmxfile.h).
enum KindA : uint32_t {
    K_CTX = 1, K_PART = 3, K_RAGE_PART = 4, K_BEHAVIOUR = 5, K_LIMB = 6, K_QUEUE_DATA = 7,
    K_RIG_RECORD = 8, K_RIG_HINGES = 9, K_RIG_3DOF = 10, K_ART = 11, K_COLLIDER_TYPE = 13,
    K_WINDOW = 15, K_MGR = 16, K_BODY = 18,
};

HMODULE g_self;
uintptr_t g_base;
std::string g_dir;
FILE* g_log;
bool g_gate_ok, g_hook_gate_ok;

void say(const char* fmt, ...) {
    if (!g_log) return;
    SYSTEMTIME t; GetLocalTime(&t);
    fprintf(g_log, "%02d:%02d:%02d.%03d ", t.wHour, t.wMinute, t.wSecond, t.wMilliseconds);
    va_list a; va_start(a, fmt); vfprintf(g_log, fmt, a); va_end(a);
    fputc('\n', g_log); fflush(g_log);
}

// A guarded copy: a bad pointer costs the copy, not the game.
bool copy(void* dst, uintptr_t src, size_t n) { return nmxh::safe_copy(dst, src, n); }

template <typename T> bool get(uintptr_t at, T& out) { return copy(&out, at, sizeof(T)); }
uintptr_t ptr(uintptr_t at) { uintptr_t p = 0; return get(at, p) ? p : 0; }

std::string sha256(const uint8_t* p, size_t n) {
    BCRYPT_ALG_HANDLE alg; BCRYPT_HASH_HANDLE h; uint8_t out[32]; std::string s;
    if (BCryptOpenAlgorithmProvider(&alg, BCRYPT_SHA256_ALGORITHM, nullptr, 0)) return s;
    if (!BCryptCreateHash(alg, &h, nullptr, 0, nullptr, 0, 0)) {
        BCryptHashData(h, const_cast<uint8_t*>(p), (ULONG)n, 0);
        BCryptFinishHash(h, out, 32, 0);
        BCryptDestroyHash(h);
        char b[3]; for (uint8_t x : out) { sprintf_s(b, "%02x", x); s += b; }
    }
    BCryptCloseAlgorithmProvider(alg, 0);
    return s;
}

uint32_t image_size() {
    auto nt = (IMAGE_NT_HEADERS*)(g_base + ((IMAGE_DOS_HEADER*)g_base)->e_lfanew);
    return nt->OptionalHeader.SizeOfImage;
}

// ---- the output file ----
struct Out {
    FILE* f = nullptr;
    uint64_t seq = 0;
    std::vector<uint8_t> buf;
    DWORD flushed = 0;
    void obj(uint32_t k, uintptr_t addr, uint32_t size, const void* bytes) {
        uintptr_t vt = 0; get(addr, vt);
        uint32_t vt_rva = (vt > g_base && vt - g_base < 0x8000000) ? (uint32_t)(vt - g_base) : 0;
        put_obj(buf, k, addr, vt_rva, bytes, size);
    }
    // Copies `size` bytes at `addr` as one object (kept even when the copy fails: then all zero, size 0).
    bool raw(uint32_t k, uintptr_t addr, uint32_t size) {
        if (!addr) return false;
        std::vector<uint8_t> b(size);
        if (!copy(b.data(), addr, size)) { obj(k, addr, 0, b.data()); return false; }
        obj(k, addr, size, b.data());
        return true;
    }
    void text(const std::string& s) { put_obj(buf, K_TEXT, 0, 0, s.data(), (uint32_t)s.size()); }
    void record(uint32_t tag, uint32_t agent, uint32_t step) {
        if (!f) { buf.clear(); return; }
        LARGE_INTEGER q; QueryPerformanceCounter(&q);
        write_record(f, seq, tag, agent, step, (uint64_t)q.QuadPart, GetCurrentThreadId(), buf.data(), buf.size());
        buf.clear();
        if (GetTickCount() - flushed > 500) { fflush(f); flushed = GetTickCount(); }
    }
    // A record the hooks built: its own time and thread.
    void record_raw(uint32_t tag, uint32_t agent, uint32_t step, uint64_t qpc, uint32_t tid, const uint8_t* body, size_t n) {
        if (f) write_record(f, seq, tag, agent, step, qpc, tid, body, n);
    }
    bool open(const char* name, const std::string& note) {
        close();
        SYSTEMTIME t; GetLocalTime(&t);
        char path[MAX_PATH];
        sprintf_s(path, "%s\\%04d%02d%02d-%02d%02d%02d_%s.nmx", g_dir.c_str(), t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute, t.wSecond, name);
        if (fopen_s(&f, path, "wb") || !f) { f = nullptr; say("cannot open %s", path); return false; }
        auto nt = (IMAGE_NT_HEADERS*)(g_base + ((IMAGE_DOS_HEADER*)g_base)->e_lfanew);
        // mode 'B' when the hooks are in (the file may then hold records 7..13)
        write_header(f, g_base, nt->OptionalHeader.SizeOfImage, nt->FileHeader.TimeDateStamp, nmxh::installed() ? 'B' : 'A',
                     std::string(name) + "\n" + note);
        fflush(f);
        seq = 0;
        say("writing %s", path);
        return true;
    }
    void close();
} g_out;

// ---- the build gates ----
bool check(const Gate* gates, size_t n, const char* what) {
    bool ok = true;
    for (size_t i = 0; i < n; i++) {
        uint8_t b[64];
        std::string got = copy(b, g_base + gates[i].rva, 64) ? sha256(b, 64) : std::string("unreadable");
        bool same = got == gates[i].sha;
        if (!same) say("%s: rva 0x%x differs (%s)", what, gates[i].rva, got.c_str());
        ok &= same;
    }
    return ok;
}

// ---- Mode B: the hooks ----
uint64_t g_lost;   // hook records drained while no file was open
DWORD g_armed_at;  // tick of the shot that armed the hooks, 0 none
bool g_target_logged;
constexpr DWORD ARM_TIMEOUT_MS = 3000;   // the window is NM time (nmxh::WINDOW_S); this only bounds a capture that never sees its agent

bool disabled() { return GetFileAttributesA((g_dir + "\\DISABLE").c_str()) != INVALID_FILE_ATTRIBUTES; }

// <game>\rdr2nmx\NOHOOKS: Mode A only (no code patched), the scenarios still run.
bool nohooks() { return GetFileAttributesA((g_dir + "\\NOHOOKS").c_str()) != INVALID_FILE_ATTRIBUTES; }

bool hooks_on() {
    if (nmxh::installed()) return true;
    if (!g_hook_gate_ok || disabled()) return false;
    if (nohooks()) { say("hooks: NOHOOKS file: Mode A only"); return false; }
    nmxh::g_base = g_base;
    return nmxh::install(say, image_size());
}

void hooks_off(const char* why) {
    if (!nmxh::installed()) return;
    say("hooks: taking them out (%s)", why);
    nmxh::uninstall(say);
}

void drain_hooks() {
    static std::vector<uint8_t> scratch;
    nmxh::ring_drain(scratch, [](const nmxh::Entry& e, const uint8_t* body, size_t n) {
        if (g_out.f) g_out.record_raw(e.tag, e.agent, e.step, e.qpc, e.tid, body, n);
        else g_lost++;
    });
}

void hook_info() {
    std::string s = "Mode B hooks:";
    for (auto& h : nmxh::g_hooks) { char b[96]; sprintf_s(b, " %s (%d bytes moved);", h.name, h.stolen); s += b; }
    char b[256];
    sprintf_s(b, " window %.2f s of NM time from the shot agent's first step; removal call sites margin 0x%x final 0x%x; spare 0x%x",
              nmxh::WINDOW_S, nmxh::RET_REMOVE_MARGIN, nmxh::RET_REMOVE_FINAL, nmxh::RET_SPARE);
    s += b;
    g_out.text(s);
    for (auto& c : nmxh::CONSTS) {
        uint8_t v[16] = {};
        copy(v, g_base + c.rva, c.size);
        put_obj(g_out.buf, K_HCONST, g_base + c.rva, c.rva, v, c.size);
        g_out.text(c.what);
    }
    g_out.record(HOOKINFO, 0xffffffff, 0);
}

void Out::close() {
    if (!f) return;
    if (nmxh::installed()) {
        drain_hooks();
        nmxh::HStat st{};
        st.entries = nmxh::g_ring.entries; st.dropped = nmxh::g_ring.dropped; st.dropped_bytes = nmxh::g_ring.dropped_bytes;
        st.lost_no_file = g_lost; st.steps = nmxh::g_n_step; st.manifolds = nmxh::g_n_man; st.removes = nmxh::g_n_rem;
        st.spares = nmxh::g_n_spare; st.window_closed = nmxh::g_closed; st.target = (uint32_t)nmxh::g_target.load();
        st.target_how = (uint32_t)nmxh::g_target_how.load();
        put_obj(buf, K_HSTAT, 0, 0, &st, sizeof st);
        record(HOOKSTAT, st.target, st.steps);
        say("hooks: %u steps, %u manifold records, %u removals, %u spares recorded; ring %llu entries, %llu dropped",
            st.steps, st.manifolds, st.removes, st.spares, st.entries, st.dropped);
        say("hooks raw while open: refresh %u (%u with an NM instance as i0/i1), remove %u, spare %u; target %d",
            nmxh::g_raw_ref.load(), nmxh::g_raw_ref_nm.load(), nmxh::g_raw_rem.load(), nmxh::g_raw_spare.load(),
            nmxh::g_target.load());
        {
            std::string ln;
            for (uint32_t k = 0; k < nmxh::g_n_slice.load() && k < (uint32_t)nmxh::N_SLICE; k++) {
                const auto& z = nmxh::g_slice[k];
                char b[96];
                snprintf(b, sizeof b, " %.4f/%.4f/%.4f n%d f%d", z.dt, z.frame, z.slice, z.count, (int)z.flag);
                ln += b;
                if (k % 6 == 5) { say("  split (NM dt/frame/slice, count, flag):%s", ln.c_str()); ln.clear(); }
            }
            if (!ln.empty()) say("  split (NM dt/frame/slice, count, flag):%s", ln.c_str());
            if (nmxh::g_n_slice.load()) say("  split: flag object vtable %x", nmxh::g_slice[0].sub_vt);
        }
        for (uint32_t k = 0; k < nmxh::g_n_samples.load() && k < (uint32_t)nmxh::N_SAMPLES; k++) {
            const auto& s = nmxh::g_samples[k];
            say("  refresh %u: ret %x | m0 vt %x ag %d | m1 vt %x ag %d | i0 vt %x ag %d +180 %d | i1 vt %x ag %d +180 %d",
                k, s.ret, s.vt[0], s.ag[0], s.vt[1], s.ag[1], s.vt[2], s.ag[2], s.raw180[0], s.vt[3], s.ag[3], s.raw180[1]);
        }
    }
    record(END, 0, 0);
    fflush(f); fclose(f); f = nullptr;
}

// ---- reading NM ----
uintptr_t manager() { return ptr(g_base + MGR_GLOBAL); }

struct Agent { uint32_t idx; uintptr_t ctx; };

std::vector<Agent> agents(uintptr_t mgr) {
    std::vector<Agent> out;
    for (uint32_t i = 0; i < 32; i++) {
        uintptr_t ctx = ptr(mgr + 0x1a0 + 8 * i);
        uint8_t inserted = 0;
        if (ctx && get(ctx + 0x4803, inserted) && inserted) out.push_back({i, ctx});
    }
    return out;
}

// Behaviour object sizes by id where known (study notes); others get a generous window.
uint32_t behaviour_size(uint32_t id) {
    switch (id) {
    case 1: return 0xbf0;  // balancer
    case 5: return 0x7a0;  // shot
    case 12: return 0x2c0; // catchFall
    case 14: return 0xb8;  // bodyRollUp
    default: return 0x900;
    }
}

int joint_count(uintptr_t ctx) { int n = 0; get(ctx + 0x2f44, n); return (n > 0 && n < 64) ? n : 0; }

void rig(const Agent& a, uintptr_t mgr) {
    uintptr_t ctx = a.ctx;
    g_out.raw(K_CTX, ctx, CTX_SIZE);
    g_out.raw(K_MGR, mgr, 0x400);
    int asset = -1; get(ctx + 0x351c, asset);
    uintptr_t recs = ptr(mgr + 0x8);
    if (recs && asset >= 0 && asset < 64) {
        uintptr_t rec = recs + 0x60 * asset;
        g_out.raw(K_RIG_RECORD, rec, 0x60);
        int hinges = 0, threes = 0, n = joint_count(ctx);
        for (int j = 0; j < n; j++) {
            uintptr_t jp = ptr(ctx + 0x3130 + 8 * j), vt = ptr(jp);
            if (vt == g_base + 0x3663d98) hinges++; else if (vt == g_base + 0x3663ec0) threes++;
        }
        g_out.raw(K_RIG_HINGES, ptr(rec + 0x40), 0x20 * (hinges ? hinges : 1));
        g_out.raw(K_RIG_3DOF, ptr(rec + 0x48), 0x44 * (threes ? threes : 1));
    }
    uintptr_t art = ptr(ctx + 0x3110);
    g_out.raw(K_ART, art, 0x200);
    uintptr_t col = ptr(ctx + 0x3128);
    g_out.raw(K_COLLIDER, col, 0x260);
    g_out.raw(K_COLLIDER_TYPE, ptr(col + 0x10), 0x200);
    int n = joint_count(ctx);
    for (int j = 0; j < n; j++) {
        uintptr_t jp = ptr(ctx + 0x3130 + 8 * j), vt = ptr(jp);
        bool hinge = vt == g_base + 0x3663d98;
        g_out.raw(K_JOINT, jp, hinge ? 0x150 : 0x1b0);
        g_out.raw(K_RAGE_JOINT, ptr(jp + (hinge ? 0xf8 : 0x100)), hinge ? 0x2c0 : 0x540);
    }
    g_out.record(RIG, a.idx, 0);
}

void step(const Agent& a, uintptr_t mgr, uint32_t frame) {
    uintptr_t ctx = a.ctx;
    g_out.raw(K_CTX, ctx, CTX_SIZE);
    int n = joint_count(ctx);
    for (int j = 0; j < n; j++) {
        uintptr_t jp = ptr(ctx + 0x3130 + 8 * j);
        g_out.raw(K_JOINT, jp, ptr(jp) == g_base + 0x3663d98 ? 0x150 : 0x1b0);
    }
    for (int i = 0; i <= n; i++) g_out.raw(K_PART, ptr(ctx + 0x2f48 + 8 * i), 0x1e0);
    uintptr_t art = ptr(ctx + 0x3110);
    uint8_t links = 0; get(art + 0x88, links);
    uintptr_t rparts = ptr(art + 0xb0);
    if (rparts && links && links < 64) g_out.raw(K_RAGE_PART, rparts, 0x140 * links);
    // behaviours of this agent: row R = mgr+0x2a0+0x8+0x278*idx; R+0x8+8*id
    uintptr_t row = mgr + 0x2a0 + 0x8 + 0x278 * a.idx;
    for (uint32_t id = 0; id < 38; id++) {
        uintptr_t b = ptr(row + 0x8 + 8 * id);
        uint8_t active = 0;
        if (b && get(b + 0x1c, active) && active) g_out.raw(K_BEHAVIOUR, b, behaviour_size(id));
    }
    // the body and its limbs, with each queued request's data
    uintptr_t body = ptr(ctx + 0x2de8);
    g_out.raw(K_BODY, body, 0x100);
    for (int l = 0; l < 5; l++) {
        uintptr_t limb = ptr(body + 0x8 + 8 * l);
        if (!g_out.raw(K_LIMB, limb, 0x5e0)) continue;
        uint32_t count = 0; get(limb + 0x388, count);
        for (uint32_t q = 0; q < count && q < 18; q++) {
            uintptr_t e = limb + 0x28 + 0x28 * q, data = ptr(e + 0x18);
            uint32_t size = 0; get(e + 0x20, size);
            g_out.raw(K_QUEUE_DATA, data, (std::min)(size ? size : 0x100u, 0x400u));
        }
    }
    g_out.record(STEP, a.idx, frame);
}

void globals() {
    const uint32_t W[][2] = {{0x3cb0000, 0x20000}, {0x5978000, 0x1000}, {0x59ae000, 0x6000}};
    for (auto& w : W) g_out.raw(K_WINDOW, g_base + w[0], w[1]);
    g_out.record(GLOBALS, 0, 0);
}

// ---- scenarios ----
bool story_only() { return !NETWORK::NETWORK_IS_SESSION_STARTED() && !NETWORK::NETWORK_IS_GAME_IN_PROGRESS(); }

struct Scenario { const char* name; const char* note; };
// A 1-damage bullet into 1000 health only plays an animated hit reaction (session 1): NM runs when the hit is real.
constexpr int HEALTH = 400, DAMAGE = 60;
const Scenario SCENARIOS[] = {
    {"capture", "shot_leg and real_shot three times each, alternating, each to its own file, with the hooks (Mode B)"},
    {"falls", "a townsman dropped from 4, 8, 15 and 30 m, twice each, 6 m ahead of the player, 8 s logged each (Mode A)"},
    {"pushes", "3 bumps (a townsman sprints through another) then 3 fist fights (one fights a passive one), 5 m ahead (Mode A)"},
    {"real_shot", "a clone of the player 4 m ahead, 3 s to settle, one bullet (60 of 400 health) at the chest from 3 m in front, 6 s logged"},
    {"shot_back", "the same, the bullet from behind"},
    {"shot_leg", "the same, the bullet at the left thigh from the front"},
    {"shot_ragdoll", "the same chest shot, the clone ragdolled a frame before it (if the plain shots stay animated)"},
    {"push", "a clone, 3 s to settle, ragdolled and pushed at the chest, 5 s logged"},
    {"stand", "a clone standing, 5 s logged (the balancer at rest, if NM runs)"},
};
constexpr int CAPTURE_RUNS = 3;
constexpr DWORD CAPTURE_GAP_MS = 1000;   // between runs, so the last ped is gone
int g_pick = 0;   // capture
volatile bool g_run, g_abort, g_free, g_dump;

// Agents whose rig is already in the open file: NM inserts an agent only when the ped ragdolls, so the rig is
// written the first frame each one appears.
uint32_t g_rigged;

void steps(uint32_t frame) {
    uintptr_t mgr = manager();
    if (!mgr) return;
    for (auto& a : agents(mgr)) {
        if (!(g_rigged >> a.idx & 1)) { rig(a, mgr); g_rigged |= 1u << a.idx; say("agent %u inserted: rig written", a.idx); }
        step(a, mgr, frame);
    }
}

// Once per frame while a capture is armed: drain, fall back to the agent nearest the ped when the step hook
// hasn't seen a new one, write HTARGET once the agent is known, end at the timeout.
void hook_frame(Ped ped) {
    if (!nmxh::installed()) return;
    if (disabled() || nohooks()) { hooks_off(disabled() ? "DISABLE file" : "NOHOOKS file"); return; }
    drain_hooks();
    if (!g_armed_at) return;
    uintptr_t mgr = manager();
    int tgt = nmxh::g_target.load();
    Vector3 p = ped ? ENTITY::GET_ENTITY_COORDS(ped, TRUE, FALSE) : Vector3{};
    if (tgt == -1 && mgr && GetTickCount() - g_armed_at > 50) {
        // the step hook takes a newly inserted agent at once; this is for an agent that was already in
        int best = -1; float bd = 1e9f;
        for (auto& a : agents(mgr)) {
            float c[3] = {};
            if (!copy(c, a.ctx + 0xf40, 12)) continue;
            float d = sqrtf((c[0] - p.x) * (c[0] - p.x) + (c[1] - p.y) * (c[1] - p.y) + (c[2] - p.z) * (c[2] - p.z));
            if (d < bd) { bd = d; best = (int)a.idx; }
        }
        if (best >= 0 && bd < 2.0f && nmxh::select_target(best, 2)) say("hooks: agent %d chosen as the nearest to the ped (%.2f m)", best, bd);
        tgt = nmxh::g_target.load();
    }
    if (tgt >= 0 && !g_target_logged && mgr) {
        g_target_logged = true;
        uintptr_t ctx = ptr(mgr + 0x1a0 + 8 * tgt);
        float c[3] = {};
        copy(c, ctx + 0xf40, 12);
        float d = sqrtf((c[0] - p.x) * (c[0] - p.x) + (c[1] - p.y) * (c[1] - p.y) + (c[2] - p.z) * (c[2] - p.z));
        char b[200];
        sprintf_s(b, "agent %d, %s; its COM (ctx+0xf40) %.2f m from the ped's coordinates", tgt,
                  nmxh::g_target_how == 1 ? "new after the shot (step hook)" : "nearest to the ped (script thread)", d);
        say("hooks: %s", b);
        if (d > 2.0f) say("hooks: WARNING: the agent followed may not be the shot ped");
        g_out.text(b);
        uintptr_t col = ptr(ctx + 0x3128);
        g_out.raw(K_COLLIDER, col, 0x300);
        g_out.raw(K_COLLIDER_MAP, ptr(col + 0x270), 0x100);
        g_out.record(HTARGET, (uint32_t)tgt, 0);
    }
    if (nmxh::g_closed.load()) {
        say("hooks: window closed after %u steps", nmxh::g_step.load());
        g_armed_at = 0;
    } else if (GetTickCount() - g_armed_at > ARM_TIMEOUT_MS) {
        nmxh::disarm();
        say("hooks: no window end by NM time after %u ms (%u steps recorded, agent %d): disarmed", ARM_TIMEOUT_MS,
            nmxh::g_step.load(), nmxh::g_target.load());
        g_armed_at = 0;
    }
}

void log_frames(int ms, Ped ped, uint32_t& frame) {
    DWORD end = GetTickCount() + ms;
    while (GetTickCount() < end && !g_abort) {
        steps(frame);
        hook_frame(ped);
        if (ped) {
            Vector3 p = ENTITY::GET_ENTITY_COORDS(ped, TRUE, FALSE);
            int t = GAMEPLAY::GET_GAME_TIMER();
            float v[4] = {p.x, p.y, p.z, (float)t};
            g_out.obj(K_TEXT, 0, sizeof v, v);
            g_out.record(SCENARIO, 0xffffffff, frame);
        }
        frame++;
        WAIT(0);
    }
}

// `what` is the scenario's kind (real_shot, shot_leg, ...); `file` names the output.
void run_scenario(const char* what_c, const char* file, const char* note) {
    if (!story_only()) { say("not in story mode: no scenario"); return; }
    std::string what = what_c;
    bool shot = what.rfind("shot", 0) == 0 || what == "real_shot";
    if (shot && !hooks_on()) say("%s: hooks not in: Mode A only", file);
    Ped player = PLAYER::PLAYER_PED_ID();
    Vector3 at = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(player, 0.0f, 4.0f, 0.0f);
    float heading = ENTITY::GET_ENTITY_HEADING(player);
    // An ordinary townsman, not a clone of the player: in session 1 the player's bullets went through his clone into
    // the branch behind (a clone is treated like the player). The clone is only the fallback.
    Ped ped = 0;
    for (const char* m : {"a_m_m_valfarmer_01", "a_m_m_rancher_01", "a_m_m_valtownfolk_01"}) {
        Hash h = GAMEPLAY::GET_HASH_KEY(const_cast<char*>(m));
        if (!STREAMING::IS_MODEL_VALID(h)) continue;
        STREAMING::REQUEST_MODEL(h, FALSE);
        for (int i = 0; i < 200 && !STREAMING::HAS_MODEL_LOADED(h); i++) WAIT(10);
        if (!STREAMING::HAS_MODEL_LOADED(h)) continue;
        ped = PED::CREATE_PED(h, at.x, at.y, at.z, heading + 180.0f, FALSE, FALSE, FALSE, FALSE);
        STREAMING::SET_MODEL_AS_NO_LONGER_NEEDED(h);
        if (ped) { PED::SET_PED_VISIBLE(ped, TRUE); say("%s: spawned %s", file, m); break; }   // 0x283978A1: random outfit
    }
    if (!ped) { ped = PED::CLONE_PED(player, heading + 180.0f, FALSE, TRUE); say("%s: no townsman model, using a clone", file); }
    if (!ped) { say("%s: spawn failed", file); return; }
    ENTITY::SET_ENTITY_AS_MISSION_ENTITY(ped, TRUE, TRUE);
    ENTITY::SET_ENTITY_COORDS(ped, at.x, at.y, at.z, FALSE, FALSE, FALSE, FALSE);
    ENTITY::SET_ENTITY_HEADING(ped, heading + 180.0f);
    PED::SET_BLOCKING_OF_NON_TEMPORARY_EVENTS(ped, TRUE);
    PED::SET_PED_CAN_RAGDOLL(ped, TRUE);
    PED::SET_PED_MAX_HEALTH(ped, HEALTH);
    ENTITY::SET_ENTITY_HEALTH(ped, HEALTH, 0);
    AI::TASK_STAND_STILL(ped, 20000);
    if (!g_out.open(file, note)) { PED::DELETE_PED(&ped); return; }
    g_rigged = 0;
    globals();
    if (nmxh::installed()) hook_info();
    uint32_t frame = 0;
    log_frames(3000, ped, frame);
    if (shot) {
        bool back = what == "shot_back", leg = what == "shot_leg";
        if (what == "shot_ragdoll") { PED::SET_PED_TO_RAGDOLL(ped, 4000, 4000, 0, FALSE, FALSE, FALSE); WAIT(0); }
        // chest 0.45 m above the root, the left thigh 0.35 m below it and 0.12 m to the side
        Vector3 c = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(ped, leg ? -0.12f : 0.0f, 0.0f, leg ? -0.35f : 0.45f);
        // fired from 2 m away along a level line through the target, so it cannot pass over or beside him
        Vector3 from = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(ped, leg ? -0.12f : 0.0f, back ? -2.0f : 2.0f, leg ? -0.35f : 0.45f);
        Hash gun = GAMEPLAY::GET_HASH_KEY(const_cast<char*>("WEAPON_REVOLVER_CATTLEMAN"));
        if (nmxh::installed()) {
            drain_hooks();
            nmxh::arm();
            g_armed_at = GetTickCount(); if (!g_armed_at) g_armed_at = 1;
            g_target_logged = false;
        }
        GAMEPLAY::SHOOT_SINGLE_BULLET_BETWEEN_COORDS(from.x, from.y, from.z, c.x, c.y, c.z, DAMAGE, TRUE, gun, player, TRUE, FALSE, -1.0f, TRUE);
        say("%s: bullet fired (%d damage)%s", file, DAMAGE, nmxh::installed() ? ", hooks armed" : "");
        log_frames(6000, ped, frame);
        if (g_armed_at) { nmxh::disarm(); g_armed_at = 0; say("%s: hooks disarmed at the end of the run", file); }
    } else if (what == "push") {
        PED::SET_PED_TO_RAGDOLL(ped, 4000, 4000, 0, FALSE, FALSE, FALSE);
        // a shove: -60 threw him metres back in session 1
        ENTITY::APPLY_FORCE_TO_ENTITY(ped, 1, 0.0f, -12.0f, 0.0f, 0.0f, 0.0f, 0.4f, 0, TRUE, TRUE, TRUE, FALSE, TRUE);
        log_frames(5000, ped, frame);
    } else {
        log_frames(5000, ped, frame);
    }
    nmxh::disarm();
    g_out.close();
    PED::DELETE_PED(&ped);
    say("%s: done%s", file, g_abort ? " (aborted)" : "");
}

// The capture: the leg shot (stance foot) and the chest shot (swing foot), CAPTURE_RUNS each, alternating.
void run_capture() {
    if (!story_only()) { say("not in story mode: no capture"); return; }
    if (!hooks_on()) say("capture: the hooks are not in (see above): Mode A records only");
    DWORD t0 = GetTickCount();
    say("capture: %d leg and %d chest shots, about %d s", CAPTURE_RUNS, CAPTURE_RUNS, CAPTURE_RUNS * 2 * 11);
    for (int i = 1; i <= CAPTURE_RUNS && !g_abort; i++) {
        for (int k = 0; k < 2 && !g_abort; k++) {
            if (disabled()) { say("capture: DISABLE file: stopped"); hooks_off("DISABLE file"); return; }
            const char* what = k == 0 ? "shot_leg" : "real_shot";
            char file[64]; sprintf_s(file, "capture_%s_%d", k == 0 ? "leg" : "chest", i);
            char note[256];
            sprintf_s(note, "capture run %d of %d: %s (%s)", i, CAPTURE_RUNS, what,
                      k == 0 ? "the bullet at the left thigh from the front" : "the bullet at the chest from the front");
            run_scenario(what, file, note);
            DWORD gap = GetTickCount() + CAPTURE_GAP_MS;
            while (GetTickCount() < gap && !g_abort) { hook_frame(0); WAIT(0); }
        }
    }
    say("capture: done%s in %.1f s", g_abort ? " (aborted)" : "", (GetTickCount() - t0) / 1000.0);
}

// One drop: the townsman stands 6 m ahead for 1 s (model in, standing), then is put `h` m above the ground there, still,
// and left to the game: its own fall and NM tasks (CTaskNMHighFall) run as they would. 2000 health so a 30 m landing
// isn't a death.
void run_drop(float h, const char* file, const char* note) {
    Ped player = PLAYER::PLAYER_PED_ID();
    Vector3 at = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(player, 0.0f, 6.0f, 0.0f);
    float heading = ENTITY::GET_ENTITY_HEADING(player);
    float ground = at.z;
    float g = 0.0f;
    if (GAMEPLAY::GET_GROUND_Z_FOR_3D_COORD(at.x, at.y, at.z + 3.0f, &g, FALSE) && fabsf(g - at.z) < 3.0f) ground = g;
    Ped ped = 0;
    for (const char* m : {"a_m_m_valfarmer_01", "a_m_m_rancher_01", "a_m_m_valtownfolk_01"}) {
        Hash hh = GAMEPLAY::GET_HASH_KEY(const_cast<char*>(m));
        if (!STREAMING::IS_MODEL_VALID(hh)) continue;
        STREAMING::REQUEST_MODEL(hh, FALSE);
        for (int i = 0; i < 200 && !STREAMING::HAS_MODEL_LOADED(hh); i++) WAIT(10);
        if (!STREAMING::HAS_MODEL_LOADED(hh)) continue;
        ped = PED::CREATE_PED(hh, at.x, at.y, ground, heading + 180.0f, FALSE, FALSE, FALSE, FALSE);
        STREAMING::SET_MODEL_AS_NO_LONGER_NEEDED(hh);
        if (ped) { PED::SET_PED_VISIBLE(ped, TRUE); say("%s: spawned %s", file, m); break; }
    }
    if (!ped) { say("%s: spawn failed", file); return; }
    ENTITY::SET_ENTITY_AS_MISSION_ENTITY(ped, TRUE, TRUE);
    PED::SET_BLOCKING_OF_NON_TEMPORARY_EVENTS(ped, TRUE);
    PED::SET_PED_CAN_RAGDOLL(ped, TRUE);
    PED::SET_PED_MAX_HEALTH(ped, 2000);
    ENTITY::SET_ENTITY_HEALTH(ped, 2000, 0);
    if (!g_out.open(file, note)) { PED::DELETE_PED(&ped); return; }
    g_rigged = 0;
    globals();
    uint32_t frame = 0;
    log_frames(1000, ped, frame);
    ENTITY::SET_ENTITY_COORDS(ped, at.x, at.y, ground + h, FALSE, FALSE, FALSE, FALSE);
    ENTITY::SET_ENTITY_HEADING(ped, heading + 180.0f);
    ENTITY::SET_ENTITY_VELOCITY(ped, 0.0f, 0.0f, 0.0f);
    say("%s: up %.0f m (ground %.2f)", file, h, ground);
    log_frames(8000, ped, frame);
    say("%s: health after the landing %d of 2000", file, ENTITY::GET_ENTITY_HEALTH(ped));
    g_out.close();
    PED::DELETE_PED(&ped);
    say("%s: done%s", file, g_abort ? " (aborted)" : "");
}

// A townsman at `at`, facing `heading`, standing, not reacting to events (so he neither flees nor fights back).
Ped spawn_townsman(Vector3 at, float heading, const char* file, int health) {
    Ped ped = 0;
    for (const char* m : {"a_m_m_valfarmer_01", "a_m_m_rancher_01", "a_m_m_valtownfolk_01"}) {
        Hash hh = GAMEPLAY::GET_HASH_KEY(const_cast<char*>(m));
        if (!STREAMING::IS_MODEL_VALID(hh)) continue;
        STREAMING::REQUEST_MODEL(hh, FALSE);
        for (int i = 0; i < 200 && !STREAMING::HAS_MODEL_LOADED(hh); i++) WAIT(10);
        if (!STREAMING::HAS_MODEL_LOADED(hh)) continue;
        ped = PED::CREATE_PED(hh, at.x, at.y, at.z, heading, FALSE, FALSE, FALSE, FALSE);
        STREAMING::SET_MODEL_AS_NO_LONGER_NEEDED(hh);
        if (ped) { PED::SET_PED_VISIBLE(ped, TRUE); say("%s: spawned %s", file, m); break; }
    }
    if (!ped) { say("%s: spawn failed", file); return 0; }
    ENTITY::SET_ENTITY_AS_MISSION_ENTITY(ped, TRUE, TRUE);
    ENTITY::SET_ENTITY_COORDS(ped, at.x, at.y, at.z, FALSE, FALSE, FALSE, FALSE);
    ENTITY::SET_ENTITY_HEADING(ped, heading);
    PED::SET_PED_CAN_RAGDOLL(ped, TRUE);
    PED::SET_PED_MAX_HEALTH(ped, health);
    ENTITY::SET_ENTITY_HEALTH(ped, health, 0);
    return ped;
}

// One push or melee run: a target 5 m ahead of the player, facing him; `bump`: a second townsman sprints straight through
// the target from 6 m to his side; else the second one, 1.3 m in front of him, unarmed, is told to fight him. The game's
// own tasks (CTaskNMBalance / CTaskNMMelee / CTaskNMFlinch) do the rest.
void run_pair(bool bump, const char* file, const char* note) {
    Ped player = PLAYER::PLAYER_PED_ID();
    Vector3 at = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(player, 0.0f, 5.0f, 0.0f);
    float heading = ENTITY::GET_ENTITY_HEADING(player) + 180.0f;
    Ped target = spawn_townsman(at, heading, file, HEALTH);
    if (!target) return;
    PED::SET_BLOCKING_OF_NON_TEMPORARY_EVENTS(target, TRUE);
    AI::TASK_STAND_STILL(target, 20000);
    // the target's own frame: +y is where he faces, +x his right
    Vector3 from = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(target, bump ? -6.0f : 0.0f, bump ? 0.0f : 1.3f, 0.0f);
    Ped other = spawn_townsman(from, bump ? heading - 90.0f : heading + 180.0f, file, HEALTH);
    if (!other) { PED::DELETE_PED(&target); return; }
    if (!g_out.open(file, note)) { PED::DELETE_PED(&target); PED::DELETE_PED(&other); return; }
    g_rigged = 0;
    globals();
    uint32_t frame = 0;
    log_frames(1500, target, frame);
    if (bump) {
        Vector3 to = ENTITY::GET_OFFSET_FROM_ENTITY_IN_WORLD_COORDS(target, 8.0f, 0.0f, 0.0f);
        PED::SET_BLOCKING_OF_NON_TEMPORARY_EVENTS(other, TRUE);
        AI::TASK_GO_STRAIGHT_TO_COORD(other, to.x, to.y, to.z, 3.0f, 8000, heading - 90.0f, 0.0f, 0);
        say("%s: the runner goes", file);
        log_frames(7000, target, frame);
    } else {
        WEAPON::REMOVE_ALL_PED_WEAPONS(other, TRUE, TRUE);
        AI::TASK_COMBAT_PED(other, target, 0, 16);
        say("%s: the fight starts", file);
        log_frames(11000, target, frame);
    }
    say("%s: target health after %d of %d", file, ENTITY::GET_ENTITY_HEALTH(target), HEALTH);
    g_out.close();
    PED::DELETE_PED(&target);
    PED::DELETE_PED(&other);
    say("%s: done%s", file, g_abort ? " (aborted)" : "");
}

void run_pushes() {
    if (!story_only()) { say("not in story mode: no pushes"); return; }
    DWORD t0 = GetTickCount();
    say("pushes: 3 bumps then 3 fist fights, about 70 s; stand somewhere open with 10 m clear ahead");
    for (int k = 0; k < 2 && !g_abort; k++) {
        for (int i = 1; i <= 3 && !g_abort; i++) {
            if (disabled()) { say("pushes: DISABLE file: stopped"); return; }
            char file[64]; sprintf_s(file, "%s_%d", k == 0 ? "bump" : "melee", i);
            char note[200];
            sprintf_s(note, k == 0 ? "a townsman sprints through a standing one from his side (run %d of 3)"
                                   : "an unarmed townsman fights a standing one that doesn't fight back (run %d of 3)", i);
            run_pair(k == 0, file, note);
            DWORD gap = GetTickCount() + CAPTURE_GAP_MS;
            while (GetTickCount() < gap && !g_abort) WAIT(0);
        }
    }
    say("pushes: done%s in %.1f s", g_abort ? " (aborted)" : "", (GetTickCount() - t0) / 1000.0);
}

void run_falls() {
    if (!story_only()) { say("not in story mode: no falls"); return; }
    DWORD t0 = GetTickCount();
    const float H[] = {4.0f, 8.0f, 15.0f, 30.0f};
    say("falls: 4, 8, 15 and 30 m, twice each, about 90 s; stand somewhere open with 6 m clear ahead");
    for (float h : H) {
        for (int i = 1; i <= 2 && !g_abort; i++) {
            if (disabled()) { say("falls: DISABLE file: stopped"); return; }
            char file[64]; sprintf_s(file, "fall_%dm_%d", (int)h, i);
            char note[160]; sprintf_s(note, "a townsman dropped from %.0f m, still, facing the player (run %d of 2)", h, i);
            run_drop(h, file, note);
            DWORD gap = GetTickCount() + CAPTURE_GAP_MS;
            while (GetTickCount() < gap && !g_abort) WAIT(0);
        }
    }
    say("falls: done%s in %.1f s", g_abort ? " (aborted)" : "", (GetTickCount() - t0) / 1000.0);
}

void on_key(DWORD key, WORD, BYTE, BOOL, BOOL, BOOL, BOOL up) {
    if (up) return;
    bool ctrl = GetAsyncKeyState(VK_CONTROL) & 0x8000;
    if (key == VK_F9 && ctrl) { g_pick = (g_pick + 1) % (int)(sizeof SCENARIOS / sizeof *SCENARIOS); say("next scenario: %s", SCENARIOS[g_pick].name); }
    else if (key == VK_F9) g_run = true;
    else if (key == VK_F10) g_abort = true;
    else if (key == VK_F11) { g_free = !g_free; say("free logging %s", g_free ? "on" : "off"); }
    else if (key == VK_F12) g_dump = true;
}

void main_loop() {
    g_gate_ok = check(GATES, sizeof GATES / sizeof *GATES, "build check");
    say(g_gate_ok ? "build check: ok (the build that was studied)" : "build check FAILED: exporting nothing");
    if (!g_gate_ok) { while (true) WAIT(1000); }
    g_hook_gate_ok = check(HOOK_GATES, sizeof HOOK_GATES / sizeof *HOOK_GATES, "hook gate");
    say(g_hook_gate_ok ? "hook gate: ok (hooks go in at the first shot scenario)" : "hook gate FAILED: no hooks, Mode A only");
    say("ready: F9 runs %s (Ctrl+F9 next), F10 abort, F11 free logging, F12 rig + globals now", SCENARIOS[g_pick].name);
    uint32_t frame = 0;
    while (true) {
        if (disabled()) { hooks_off("DISABLE file"); WAIT(1000); continue; }
        if (!story_only()) { if (g_out.f) g_out.close(); hooks_off("not story mode"); WAIT(500); continue; }
        if (g_run) {
            g_run = false; g_abort = false;
            const Scenario& s = SCENARIOS[g_pick];
            if (!strcmp(s.name, "capture")) run_capture();
            else if (!strcmp(s.name, "falls")) run_falls();
            else if (!strcmp(s.name, "pushes")) run_pushes();
            else run_scenario(s.name, s.name, s.note);
        }
        if (g_dump) {
            g_dump = false;
            if (g_out.open("snapshot", "rig and globals of every agent")) {
                uintptr_t mgr = manager();
                globals();
                size_t n = 0;
                if (mgr) for (auto& a : agents(mgr)) { rig(a, mgr); n++; }
                if (!n) say("snapshot: no ped is ragdolling, so only the globals (NM has no agent until one ragdolls)");
                g_out.close();
            }
        }
        if (g_free) {
            // the file opens with the first ragdoll, not when F11 is pressed
            uintptr_t mgr = manager();
            if (!g_out.f && mgr && !agents(mgr).empty() && g_out.open("free", "free logging (F11)")) { g_rigged = 0; frame = 0; }
            if (g_out.f) steps(frame);
            frame++;
        } else if (g_out.f) {
            g_out.close();
        }
        if (nmxh::installed()) drain_hooks();   // nothing is armed here; keeps the ring empty
        WAIT(0);
    }
}

} // namespace

BOOL APIENTRY DllMain(HMODULE module, DWORD reason, LPVOID reserved) {
    if (reason == DLL_PROCESS_ATTACH) {
        g_self = module;
        g_base = (uintptr_t)GetModuleHandleA(nullptr);
        char exe[MAX_PATH]; GetModuleFileNameA(nullptr, exe, MAX_PATH);
        g_dir = exe; g_dir = g_dir.substr(0, g_dir.find_last_of("\\/")) + "\\rdr2nmx";
        CreateDirectoryA(g_dir.c_str(), nullptr);
        fopen_s(&g_log, (g_dir + "\\rdr2nmx.log").c_str(), "a");
        say("rdr2nmx loaded (format %u; Mode A read-only, Mode B hooks only during shot scenarios)", FORMAT);
        scriptRegister(module, main_loop);
        keyboardHandlerRegister(on_key);
    } else if (reason == DLL_PROCESS_DETACH) {
        // FreeLibrary (reserved null): the game goes on, so the hooks come out. Process exit: the other threads are
        // gone and the code with them; nothing to restore.
        if (!reserved) nmxh::uninstall(say);
        g_out.close();
        scriptUnregister(module);
        keyboardHandlerUnregister(on_key);
        if (g_log) fclose(g_log);
    }
    return TRUE;
}
