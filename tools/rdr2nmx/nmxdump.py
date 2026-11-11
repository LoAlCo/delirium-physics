"""Summarises an rdr2nmx .nmx file: records by tag, agents, and per STEP the active behaviours (by vtable RVA).
Mode B files (header mode 'B', record tags 7..13) also get the hooks' per-NM-step records decoded: the shot agent's
world contact manifolds, 0x2842988's removals and why, and the legs' joints.
usage: python -I nmxdump.py [--steps] [--calls] file.nmx...   (--steps / --calls print every step / every refresh call)"""
import struct, sys, zlib, collections

TAGS = {1: "RIG", 2: "GLOBALS", 3: "STEP", 4: "SCENARIO", 5: "END", 6: "NOTE",
        7: "HOOKINFO", 8: "HSTEP", 9: "HMANIFOLD", 10: "HREMOVE", 11: "HSPARE", 12: "HOOKSTAT", 13: "HTARGET"}
# behaviour vtables, by the study note that defines each (unnamed ones are not in the notes yet)
VT = {0x3665810: "shot", 0x3665770: "reachForWound", 0x3666c40: "(bodybalance note)", 0x3666d80: "(writhe note, id 27)",
      0x3666bf0: "balancerCollisionsReaction", 0x3665af0: "staggerFall", 0x3666460: "(headlook note, id 0x1d)",
      0x3666ba0: "armsWindmillAdaptive", 0x3666080: "balancer", 0x3666d30: "bodyFoetal", 0x3666560: "pedalLegs", 0x3666c90: "catchFall", 0x36657c0: "bodyRollUp", 0x3665a40: "(rollstairs note, id 21)",
      0x3665a90: "smartFall", 0x36665b0: "injuredOnGround", 0x3665b40: "spineTwist"}

# Mode B object kinds (src/nmxfile.h)
K_JOINT, K_COLLIDER, K_RAGE_JOINT, K_TEXT = 2, 12, 14, 17
K_HSTEP, K_HMAN, K_MAN_RAW, K_CONTACT, K_HREMOVE, K_HSPARE, K_HCONST, K_HSTAT, K_COLLIDER_MAP = range(20, 29)
VT_HINGE = 0x3663d98
RET_MARGIN, RET_FINAL = 0x2842d88, 0x2842eee   # 0x2844140's two call sites in 0x2842988 (return addresses)
LEG_JOINTS = ["hip_l", "knee_l", "ankle_l", "hip_r", "knee_r", "ankle_r"]   # ctx joints 0, 3, 4, 1, 5, 6
HSTEP = struct.Struct("<Iif3f3ffIQQ4f16s48sQ")
HMAN = struct.Struct("<IIii2Q2H6BiII")
HREMOVE = struct.Struct("<IiIIQQII")
HSPARE = struct.Struct("<IiIIQQ4f")
HSTAT = struct.Struct("<4Q8I")


def records(path, quiet=False):
    d = open(path, "rb").read()
    assert d[:7] == b"RDR2NMX", "not an nmx file"
    hdr = d[64:4096].split(b"\0")[0].decode(errors="replace")
    off = 4096
    while off + 40 <= len(d):
        tag, ln, seq, agent, step, qpc, tid, crc = struct.unpack_from("<IIQIIQII", d, off)
        body = d[off + 40: off + 40 + ln]
        off += 40 + ln
        objs, o = [], 0
        while o + 20 <= len(body):
            kind, addr, vt, size = struct.unpack_from("<IQII", body, o)
            objs.append((kind, addr, vt, body[o + 20: o + 20 + size]))
            o += 20 + size
        yield dict(tag=tag, seq=seq, agent=agent, step=step, qpc=qpc, tid=tid, crc_ok=(zlib.crc32(body) & 0xffffffff) == crc, objs=objs)
    if not quiet:
        print("header:", hdr.replace("\n", " | "))


# ---- Mode B ----
def f3(b, o): return struct.unpack_from("<3f", b, o)
def f1(b, o): return struct.unpack_from("<f", b, o)[0]


def contact(b, addr=0, slot=0):
    """A contact's first 0x100 bytes [C: 0x28428f4, 0x2842988, 0x2822930 unless marked]."""
    if len(b) < 0xe6:
        return None
    A, B, n = f3(b, 0x50), f3(b, 0x60), f3(b, 0x70)
    d = f1(b, 0xc)
    s = [B[k] - (A[k] + n[k] * d) for k in range(3)]
    return dict(addr=addr, slot=slot, depth=d,                     # +0xc: (B - A).n
                local_a=f3(b, 0x10), local_b=f3(b, 0x20),          # in each side's part frame
                friction=f1(b, 0x1c), elasticity=f1(b, 0x2c),      # 0x2a7452c's manifold +0x1c / +0x2c
                world_a=A, world_b=B, normal=n,                    # +0x50, +0x60 from the step's matrices; +0x70
                margin=f1(b, 0x9c),                                # -1: none (.data 0x330f494)
                impulse=struct.unpack_from("<4f", b, 0xa0),        # [P] kept when 0x2822930 replaces a contact
                elem=struct.unpack_from("<2H", b, 0xe0),           # element on each side (part, triangle)
                age=struct.unpack_from("<H", b, 0xe4)[0],
                slid2=sum(x * x for x in s))                       # 0x2842988's slide: |B - (A + n d)|^2


def joint(vt, b):
    if not b:
        return None
    g = lambda o: f1(b, o)
    if vt == VT_HINGE:
        return dict(hinge=True, desired=(g(0x11c),), actual=(g(0x114),), stiffness=g(0x120), strength=g(0x124), damping=g(0x128),
                    oppose_gravity=g(0x12c), flags=struct.unpack_from("<I", b, 0xe4)[0])
    return dict(hinge=False, desired=f3(b, 0x144), actual=f3(b, 0x12c), stiffness=g(0x154), strength=g(0x158), damping=g(0x15c),
                oppose_gravity=g(0x150), flags=struct.unpack_from("<I", b, 0xe4)[0])


def hooks(path):
    """Decodes the Mode B records of one file: dict(info, consts, steps, calls, target, stat, slid_thr, far_bound)."""
    out = dict(info=[], consts={}, steps=[], calls=collections.OrderedDict(), target=None, stat=None)
    new_call = lambda k: out["calls"].setdefault(k, dict(call=k, removes=[], spare=None))
    for r in records(path, quiet=True):
        t = r["tag"]
        if t == 7:
            for k, a, vt, b in r["objs"]:
                if k == K_TEXT: out["info"].append(b.decode(errors="replace"))
                if k == K_HCONST: out["consts"][vt] = b
        elif t == 8:
            s, joints, rage = None, [], []
            for k, a, vt, b in r["objs"]:
                if k == K_HSTEP and len(b) >= HSTEP.size:
                    v = HSTEP.unpack_from(b)
                    s = dict(step=v[0], agent=v[1], dt_arg=v[2], pre=v[3:6], post=v[6:9], t_rel=v[9], flags=v[10], ctx=v[11], bal=v[12],
                             swing_target=v[13:16], contact_bits=struct.unpack_from("<I", v[17])[0],
                             bal_steps=struct.unpack_from("<i", v[18], 0xc)[0], stepping_foot=struct.unpack_from("<i", v[18], 0x1c)[0],
                             support_flags=v[18][0x2b], qpc_entry=v[19], qpc=r["qpc"], tid=r["tid"])
                elif k == K_JOINT:
                    joints.append(joint(vt, b))
                elif k == K_RAGE_JOINT:
                    rage.append(b)
            if s:
                s["joints"] = dict(zip(LEG_JOINTS, joints))
                s["rage_joints"] = dict(zip(LEG_JOINTS, rage))   # raw: the Rage side of the muscle (not decoded)
                out["steps"].append(s)
        elif t in (9, 10, 11):
            for k, a, vt, b in r["objs"]:
                if k == K_HMAN and len(b) >= HMAN.size:
                    v = HMAN.unpack_from(b)
                    h = dict(phase=v[0], call=v[1], side=v[2], agent=v[3], inst=v[4:6], comp=v[6:8], count=v[8], flags=v[9],
                             do_remove=v[10], reset=v[11], p9=v[12], threshold=v[14], step=v[15], ret=v[16], qpc=r["qpc"], tid=r["tid"],
                             manifold=a)
                    h["contacts"] = [contact(cb, ca, cvt) for ck, ca, cvt, cb in r["objs"] if ck == K_CONTACT]
                    new_call(h["call"])["entry" if h["phase"] == 0 else "exit"] = h
                elif k == K_HREMOVE and len(b) >= HREMOVE.size:
                    v = HREMOVE.unpack_from(b)
                    cb = next((x for x in r["objs"] if x[0] == K_CONTACT), None)
                    new_call(v[0])["removes"].append(dict(call=v[0], idx=v[1], ret=v[2], count_before=v[3], manifold=v[4], contact_addr=v[5],
                                                          step=v[6], contact=contact(cb[3], cb[1], cb[2]) if cb else None))
                elif k == K_HSPARE and len(b) >= HSPARE.size:
                    v = HSPARE.unpack_from(b)
                    new_call(v[0])["spare"] = dict(call=v[0], idx=v[1], result=v[2], step=v[3], manifold=v[4], contact_addr=v[5], centre=v[6:10])
        elif t == 12:
            for k, a, vt, b in r["objs"]:
                if k == K_HSTAT and len(b) >= HSTAT.size:
                    v = HSTAT.unpack_from(b)
                    out["stat"] = dict(entries=v[0], dropped=v[1], dropped_bytes=v[2], lost_no_file=v[3], steps=v[4], manifolds=v[5],
                                       removes=v[6], spares=v[7], window_closed=v[8], target=v[9], target_how=v[10])
        elif t == 13:
            out["target"] = dict(agent=r["agent"], text=[b.decode(errors="replace") for k, a, vt, b in r["objs"] if k == K_TEXT])
    out["slid_thr"] = f1(out["consts"][0x364e030], 0) if 0x364e030 in out["consts"] else 0.0004
    out["far_bound"] = f1(out["consts"][0x33b8850], 0) if 0x33b8850 in out["consts"] else 0.04
    t_of_step = {s["step"]: s["t_rel"] for s in out["steps"]}
    for c in out["calls"].values():
        decide(c, out["slid_thr"])
        e = c.get("entry") or c.get("exit")
        c["t"] = t_of_step.get(e["step"]) if e else None
    return out


def decide(c, slid_thr):
    """0x2842988's choice for one call, from what the hooks saw [C]: the contacts after ageing (the
    exit snapshot plus each removed contact as it was at its removal), the removal's call site, and the spare test.
    The far bound for a fixed side, min(.04, the bound's median half extent), isn't recorded: a removal by distance
    shows the depth was past it, a far contact kept shows it wasn't."""
    e, x = c.get("entry"), c.get("exit")
    thr = (e or x or {}).get("threshold", 4)
    after = {k["addr"]: k for k in (x["contacts"] if x else []) if k}
    for rm in c["removes"]:
        if rm["contact"]:
            after[rm["contact_addr"]] = rm["contact"]
    elig = [k for k in after.values() if k["age"] > thr]
    slid = max(elig, key=lambda k: k["slid2"], default=None)
    far = min((k for k in elig if k["depth"] < 0), key=lambda k: k["depth"], default=None)
    c["most_slid"] = (slid["addr"], slid["slid2"]) if slid else None
    c["most_far"] = (far["addr"], far["depth"]) if far else None
    sp = c["spare"]
    for rm in c["removes"]:
        if rm["ret"] == RET_MARGIN:
            rm["why"] = "margin"   # in the loop: its margin +0x9c moved past .data 0x3c9f23c on a re-query
        elif rm["ret"] == RET_FINAL:
            rm["why"] = "slid" if (sp and (sp["result"] & 0xff) == 0 and sp["idx"] == rm["idx"]) else "far"
        else:
            rm["why"] = "call site 0x%x" % rm["ret"]
    c["n_before"] = e["count"] if e else None
    c["n_after"] = x["count"] if x else None


def print_hooks(h, all_steps=False, all_calls=False):
    st = h["stat"]
    print("hooks:", " / ".join(h["info"][:1]))
    if h["target"]: print("target:", h["target"]["agent"], *h["target"]["text"])
    if st: print("hook stat:", st)
    print("constants: slid squared over %.6g, far bound at most %.4g" % (h["slid_thr"], h["far_bound"]))
    steps = h["steps"]
    if steps:
        print("NM steps recorded: %d, t_rel %.4f .. %.4f, ctx+0x47b0 %s" % (len(steps), steps[0]["t_rel"], steps[-1]["t_rel"],
                                                                         sorted({round(s["post"][0], 5) for s in steps})))
    for s in (steps if all_steps else steps[:3] + steps[-2:]):
        j = s["joints"]
        leg = " ".join("%s %s s%.0f d%.2f" % (n, "/".join("%.3f" % v for v in j[n]["desired"]), j[n]["strength"], j[n]["damping"])
                       for n in LEG_JOINTS if j.get(n))
        print("  step %3d t %.4f nm %.4f->%.4f swing (%.3f %.3f %.3f) +0x550 %#x steps %d foot %d | %s" % (
            s["step"], s["t_rel"], s["pre"][2], s["post"][2], *s["swing_target"], s["contact_bits"], s["bal_steps"], s["stepping_foot"], leg))
    calls = list(h["calls"].values())
    comp = lambda c: (c.get("entry") or c.get("exit") or {"comp": ("?",), "side": 0})
    print("0x2842988 calls on the agent's manifolds: %d, by the agent's component: %s" % (
        len(calls), dict(collections.Counter(comp(c)["comp"][comp(c)["side"]] for c in calls))))
    print("removals by reason:", dict(collections.Counter(rm["why"] for c in calls for rm in c["removes"])))
    for c in calls:
        if not all_calls and not c["removes"] and not c["spare"]:
            continue
        e = comp(c)
        line = "  t %s step %s call %d comp %s n %s->%s" % ("%.4f" % c["t"] if c["t"] is not None else "?", e.get("step", "?"), c["call"],
                                                         e["comp"][e["side"]], c["n_before"], c["n_after"])
        if c["most_slid"]: line += " | most slid %.2e%s" % (c["most_slid"][1], " > thr" if c["most_slid"][1] > h["slid_thr"] else "")
        if c["most_far"]: line += " | most far %.4f" % c["most_far"][1]
        if c["spare"]: line += " | spare(slot %d) %s" % (c["spare"]["idx"], "spared" if c["spare"]["result"] & 0xff else "not spared")
        for rm in c["removes"]:
            k = rm["contact"]
            line += " | GOES slot %d by %s" % (rm["idx"], rm["why"])
            if k: line += " (age %d depth %.4f slid2 %.2e elem %d/%d)" % (k["age"], k["depth"], k["slid2"], *k["elem"])
        print(line)
    for c in calls:   # one snapshot in full
        e = c.get("entry")
        if e and any(e["contacts"]):
            print("  e.g. call %d entry (the last solve's state), comp %s side %d, %d contacts:" % (c["call"], e["comp"], e["side"], e["count"]))
            for k in e["contacts"]:
                if k:
                    print("    slot %d age %d depth %.4f A (%.3f %.3f %.3f) B (%.3f %.3f %.3f) n (%.2f %.2f %.2f) impulse[P] %s mu %.2f elem %d/%d" % (
                        k["slot"], k["age"], k["depth"], *k["world_a"], *k["world_b"], *k["normal"], " ".join("%.3f" % v for v in k["impulse"]),
                        k["friction"], *k["elem"]))
            break


def main(path, all_steps=False, all_calls=False):
    tags = collections.Counter()
    agents = collections.Counter()
    beh = collections.Counter()
    first = {}
    bad = 0
    pos = []
    for r in records(path):
        tags[TAGS.get(r["tag"], r["tag"])] += 1
        bad += not r["crc_ok"]
        if r["tag"] == 3:
            agents[r["agent"]] += 1
            for k, a, vt, b in r["objs"]:
                if k == 5:
                    name = VT.get(vt, hex(vt))
                    beh[name] += 1
                    first.setdefault(name, r["step"])
        if r["tag"] == 4:
            for k, a, vt, b in r["objs"]:
                if k == 17 and len(b) == 16:
                    pos.append((r["step"],) + struct.unpack("<ffff", b))
    print("records:", dict(tags), "crc bad:", bad)
    print("agent steps:", dict(agents))
    print("behaviours (vtable rva: steps, first step):", {k: (v, first[k]) for k, v in beh.items()})
    if pos:
        z0 = pos[0][3]
        print("ped frames:", len(pos), "z start %.3f min %.3f end %.3f" % (z0, min(p[3] for p in pos), pos[-1][3]))
    if any(tags.get(TAGS[t]) for t in range(7, 14)):
        print_hooks(hooks(path), all_steps, all_calls)


if __name__ == "__main__":
    for p in [a for a in sys.argv[1:] if not a.startswith("--")]:
        print("==", p)
        main(p, "--steps" in sys.argv, "--calls" in sys.argv)
