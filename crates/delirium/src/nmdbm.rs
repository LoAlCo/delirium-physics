use crate::fneg;
use crate::Fma;
use crate::Vd;
use crate::Vx;
use crate::prelude::*;

use crate::collision::Collision;
use crate::nmrs::{Character, ANKLE, HIP, KNEE, SPINE, WRIST};
use crate::ragdoll::{Build, Ragdoll};
use crate::rigid::qrot;

#[derive(Clone, Copy, Debug, Default)]
pub struct M34 {
    pub a: Vec3,
    pub b: Vec3,
    pub c: Vec3,
    pub d: Vec3,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Part {
    pub m: M34,
    pub v: Vec3,
    pub w: Vec3,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Eff3 {
    pub m: M34,
    pub pos: Vec3,
    pub damping: f32,
    pub strength: f32,
    pub half_twist: f32,
    pub half_lean1: f32,
    pub half_lean2: f32,
    pub mid_lean1: f32,
    pub mid_lean2: f32,
    pub mid_twist: f32,
    pub desired: [f32; 3],
    pub actual: [f32; 3],
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Eff1 {
    pub m: M34,
    pub child: M34,
    pub pos: Vec3,
    pub damping: f32,
    pub strength: f32,
    pub half: f32,
    pub mid: f32,
    pub desired: f32,
    pub actual: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Leg {
    pub hip: Eff3,
    pub thigh: Part,
    pub knee: Eff1,
    pub shin: Part,
    pub ankle: Eff3,
    pub foot: Part,
    pub limb: M34,
}

pub fn rage_link(links: &[Quat], k: usize) -> Quat {
    let q = link(links, k).normalize();
    if k == 0 { q * Quat::from_rotation_z(-std::f32::consts::FRAC_PI_2) } else { q }
}

#[allow(non_snake_case)]
#[inline]
fn HOMER_PARTS() -> usize { delirium_data::kv!(111 int) }

pub fn link(links: &[Quat], k: usize) -> Quat {
    links.get(k).copied().unwrap_or(Quat::IDENTITY)
}

pub fn asset_links(nm: usize) -> Vec<Quat> {
    logged_links().filter(|_| nm == crate::nmrs::HOMER).map_or(vec![], <[Quat]>::to_vec)
}

pub fn logged_links() -> Option<&'static [Quat]> {
    static LINKS: std::sync::OnceLock<Vec<Quat>> = std::sync::OnceLock::new();
    let t = LINKS.get_or_init(|| {
        std::env::var("NM_LINKS").ok().and_then(|p| std::fs::read_to_string(p).ok()).map_or(vec![], |t| {
            t.lines().filter_map(|l| {
                let v: Vec<f32> = l.split_whitespace().filter_map(|x| x.parse().ok()).collect();
                (v.len() == delirium_data::kv!(1 int)).then(|| Quat::from_xyzw(v[0], v[1], v[2], v[delirium_data::kv!(0 int)]))
            }).collect()
        })
    });
    (!t.is_empty()).then_some(t.as_slice())
}

fn step_clamp(b: &Body, c: Config) -> f32 {
    if b.ch.mode == 2 { c.step_clamp * delirium_data::kv!(74) } else { c.step_clamp }
}

#[derive(Clone, Copy, Debug)]
pub struct Block {
    pub com: Vec3,
    pub vel: Vec3,
    pub angvel: Vec3,
    pub m: M34,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Snap {
    pub com: Vec3,
    pub vel: Vec3,
    pub hvel: Vec3,
    pub angvel: Vec3,
    pub m: M34,
    pub legs: [Leg; 2],
    pub spine: Eff3,
    pub pelvis: Part,
    pub flags: u8,
    pub height: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Params {
    pub up: Vec3,
    pub gup: Vec3,
    pub target: [Vec3; 2],
    pub normal: [Vec3; 2],
    pub gvel: [Vec3; 2],
    pub dt: f32,
    pub pelvis_height: f32,
    pub hip_dist: f32,
    pub leg_length: f32,
    pub foot_width: f32,
    pub p148: f32,
    pub p152: f32,
    pub step_offset: f32,
    pub abort: f32,
    pub lead: f32,
    pub p168: f32,
    pub p172: f32,
    pub lean_bias: f32,
    pub step_height: f32,
    pub leg_stiffness: f32,
    pub step_clamp: f32,
    pub taper: bool,
    pub low_abort: bool,
    pub contact: [bool; 2],
    /// RDR2 core's balancer additions (nmdbm_rdr2.rs); None: GTA IV's
    pub rdr2: Option<crate::nmdbm_rdr2::Rdr2>,
    /// RDR2 core: the up the legs' pelvis frame uses, tilted by the hip lean (zero: the balancer's up)
    pub hip_up: Vec3,
}

impl Params {
    /// The leg scale the reach is measured in: his standing pelvis height over the feet, and with the RDR2 core
    /// half the ankle's height above the sole on top (RDR2's bal+0x210, measured to the soles).
    pub fn leg_scale(&self) -> f32 {
        self.pelvis_height + if self.rdr2.is_some() { 0.5 * crate::nmdbm_rdr2::FOOT_HEIGHT } else { 0.0 }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Feet {
    pub pos: [Vec3; 2],
    pub centre: Vec3,
    pub start: Vec3,
    pub prev_target: Vec3,
    pub prev_sole: Vec3,
    pub height: [f32; 2],
    pub countdown: i32,
    pub dist: [f32; 2],
    pub stepping: u32,
    pub flags: u8,
    /// RDR2 (bal+0x438): s the swing target has been kept near the pelvis this step
    pub near_time: f32,
    /// RDR2 (bal+0x46b & 0x20): the COM over the feet, before a re-step clears the placement's flag
    pub inside_raw: bool,
}

impl Default for Feet {
    fn default() -> Feet {
        Feet { pos: [Vec3::ZERO; 2], centre: Vec3::ZERO, start: Vec3::ZERO, prev_target: Vec3::ZERO, prev_sole: Vec3::ZERO,
               height: [0.0; 2], countdown: delirium_data::kv!(41 int), dist: [0.0; 2], stepping: 0, flags: 0, near_time: 0.0, inside_raw: false }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Pelvis {
    pub off: Vec3,
    pub vel: Vec3,
    pub centre: Vec3,
    pub acc: f32,
    pub p52: f32,
    pub p56: f32,
    pub yaw: [f32; 2],
    pub p68: f32,
    pub out: [f32; 2],
    pub planted: [bool; 2],
    pub fixed_vel: bool,
}

/// The lower foot's height along up.
fn up_low(up: Vec3, l: Vec3, r: Vec3) -> f32 {
    up.vd(l).min(up.vd(r))
}

fn horizontal(v: Vec3, up: Vec3) -> Vec3 {
    v - up * up.vd(v)
}

pub(crate) fn sdot(a: Vec3, b: Vec3, o: &[u8; 3]) -> f32 {
    let [p, q, r] = o.map(|c| (c - b'x') as usize);
    a[r].fma(b[r], a[q].fma(b[q], a[p] * b[p]))
}

pub(crate) fn slen(v: Vec3, o: &[u8; 3]) -> f32 {
    sdot(v, v, o).sqrt()
}

pub(crate) fn shz(v: Vec3, up: Vec3, o: &[u8; 3]) -> Vec3 {
    up.mul_add(Vec3::splat(fneg(sdot(v, up, o))), v)
}

fn sat(v: Vec3, up: Vec3, h: f32, o: &[u8; 3]) -> Vec3 {
    up.mul_add(Vec3::splat(h - sdot(v, up, o)), v)
}

fn rotvec_quat(r: Vec3) -> Quat {
    let n2 = sdot(r, r, b"yzx");
    if n2 <= 0.0 {
        return Quat::IDENTITY;
    }
    let len = n2.sqrt();
    let y = crate::estimate::vrefp(len);
    let axis = r * (-y).mul_add(len, 1.0).mul_add(y, y);
    let h = len * 0.5;
    let (c, sn) = ((h as f64).cos() as f32, (h as f64).sin() as f32);
    Quat::from_xyzw(axis.x * sn, axis.y * sn, axis.z * sn, c)
}

fn quat_rows(q: Quat) -> [Vec3; 3] {
    let k = std::f32::consts::SQRT_2;
    let (x, y, z, w) = (q.x * k, q.y * k, q.z * k, q.w * k);
    let (xz, xy, wz, wy, yz, wx) = (z * x, y * x, w * z, w * y, z * y, w * x);
    let (zz, yy, xx) = (z * z, y * y, x * x);
    [Vec3::new(1.0 - (zz + yy), xy + wz, xz - wy), Vec3::new(xy - wz, 1.0 - (zz + xx), wx + yz), Vec3::new(wy + xz, yz - wx, 1.0 - (yy + xx))]
}

pub(crate) fn vnorm(v: Vec3) -> Vec3 {
    let n = v.vd(v);
    let y = crate::estimate::vrsqrtefp(n);
    let y = if y == f32::INFINITY { 0.0 } else { y };
    v * (-n).mul_add(y * y, 1.0).mul_add(0.5 * y, y)
}

pub(crate) fn atan2(y: f32, x: f32) -> f32 {
    (y as f64).atan2(x as f64) as f32
}

pub(crate) fn atan2f(y: f32, x: f32) -> f32 {
    use std::f32::consts::{FRAC_PI_2, PI};
    let (e, at) = (delirium_data::kv!(70), |v: f32| (v as f64).atan() as f32);
    if x > e {
        if y.abs() > e { at(y / x) } else { 0.0 }
    } else if x < -e {
        if y > e { PI - at(-(y / x)) } else if y < -e { at(y / x) - PI } else { PI }
    } else if y > e {
        FRAC_PI_2
    } else if y < -e {
        -FRAC_PI_2
    } else {
        0.0
    }
}

pub(crate) fn acos(x: f32) -> f32 {
    if x <= -1.0 { std::f32::consts::PI } else if x >= 1.0 { 0.0 } else { (x as f64).acos() as f32 }
}

fn separate(a: &mut Vec3, b: &mut Vec3, min: f32) {
    let d = *b - *a;
    let n = slen(d, b"xyz");
    if n >= min {
        return;
    }
    let dir = vnorm(d);
    *a = dir.mul_add(Vec3::splat((n - min) * 0.5), *a);
    *b = dir.mul_add(Vec3::splat((min - n) * 0.5), *b);
}

fn clamp_length(v: Vec3, lo: f32, hi: f32) -> Vec3 {
    let n2 = sdot(v, v, b"yxz");
    let k = if n2 > hi * hi {
        hi
    } else if n2 < lo * lo {
        lo
    } else {
        return v;
    };
    if n2 <= 0.0 {
        return Vec3::ZERO;
    }
    let (n, mut y) = (n2 as f64, crate::estimate::frsqrte(n2 as f64));
    for _ in 0..2 {
        y = fneg(y * y).mul_add(n, 1.0).mul_add(y * 0.5, y);
    }
    v * (y as f32 * k)
}

/// `Snap::flags`: a foot's contact as of its last tick, left and right (RDR2's +0x550 bits 1 and 3).
#[allow(non_snake_case)]
fn LEFT_LAST() -> u8 { let k: u8 = delirium_data::kv!(1 int); k << k }
#[allow(non_snake_case)]
fn RIGHT_LAST() -> u8 { let k: u8 = delirium_data::kv!(1 int); k << 2 }

fn support(pc: Vec3, l: &Part, r: &Part, w: f32, len: f32, up: Vec3) -> f32 {
    support_at(pc, &l.m, &r.m, w, len, up).0
}

fn support_at(pc: Vec3, l: &M34, r: &M34, w: f32, len: f32, up: Vec3) -> (f32, Vec3) {
    let (sw, sl) = (Vec3::splat(w), Vec3::splat(len));
    let c0 = l.c.mul_add(-sl, l.a.mul_add(-sw, l.d));
    let c1 = r.c.mul_add(-sl, r.a.mul_add(sw, r.d));
    let c2 = r.c.mul_add(sl, r.a.mul_add(sw, r.d));
    let c3 = l.c.mul_add(sl, l.a.mul_add(-sw, l.d));
    let n = |e: Vec3| vnorm(e.vx(up));
    let (n0, n1, n2, n3) = (n(c1 - c0), n(c2 - c1), n(c3 - c2), n(c0 - c3));
    let (d3, d1) = (pc - c3, pc - c1);
    let a = sdot(n2, d3, b"zyx");
    let b = sdot(n3, d3, b"zyx");
    let c = sdot(n1, d1, b"zyx");
    let e = sdot(n0, d1, b"zyx");
    let dist = a.min(b).min(c).min(e);
    let p = up.mul_add(Vec3::splat(sdot(up, (l.d + r.d) * 0.5 - pc, b"zyx")), pc);
    let edge = if b < 0.0 && b < e && b < a { Some((c3, c0)) }
        else if c < 0.0 && c < e && c < a { Some((c2, c1)) }
        else if e < 0.0 { Some((c0, c1)) }
        else if a < 0.0 { Some((c3, c2)) }
        else { None };
    match edge.filter(|_| dist < 0.0) {
        Some((u, v)) => {
            let t = (sdot(p - u, v - u, b"zyx") / sdot(v - u, v - u, b"zyx")).clamp(0.0, 1.0);
            (dist, (v - u).mul_add(Vec3::splat(t), u))
        }
        None => (dist, p),
    }
}

fn reach(t: Vec3, up: Vec3, pelvis: &M34, f1: f32, f2: f32) -> Vec3 {
    let uc = sdot(up, pelvis.c, b"yzx");
    let fwd = vnorm(shz(pelvis.c, up, b"yzx"));
    let side = fwd.vx(up);
    let centre = side.mul_add(Vec3::splat(sdot(up, pelvis.b, b"zyx") * delirium_data::kv!(116)), fwd.mul_add(Vec3::splat(uc * delirium_data::kv!(116)), pelvis.d));
    let centre = up.mul_add(Vec3::splat(sdot(t, up, b"yxz") - sdot(centre, up, b"zyx")), centre);
    let rel = t - centre;
    let (x, y) = (sdot(rel, fwd, b"yzx") * (1.0 / (f1 * delirium_data::kv!(12))), sdot(rel, side, b"zyx") * (1.0 / (f2 * delirium_data::kv!(33))));
    let r = y.fma(y, x * x).sqrt();
    let s = (1.0 / (r + delirium_data::kv!(50))).min(1.0);
    let (x, y) = (sdot(rel, fwd, b"yzx"), sdot(rel, side, b"zyx"));
    side.mul_add(Vec3::splat(s * y), fwd.mul_add(Vec3::splat(s * x), centre))
}

fn hip_leans(hip: &Eff3, t: Vec3, mid1: f32, mid2: f32, side: f32) -> (f32, f32) {
    let d = t - hip.m.d;
    let v = vnorm(Vec3::new(hip.m.a.vd(d), hip.m.b.vd(d), hip.m.c.vd(d)));
    let th = acos(v.z);
    let k = 1.0 / (v.y.fma(v.y, v.x * v.x).sqrt() + delirium_data::kv!(50));
    ((v.x * k).fma(th, mid1), (v.y * k * th).fma(side, mid2))
}

fn hip_room(hip: &Eff3, l1: f32, l2: f32, l3: f32, l4: f32) -> f32 {
    let (a1, a2) = (hip.actual[0], hip.actual[1]);
    let (s1, s2) = (hip.half_lean1 * delirium_data::kv!(12), hip.half_lean2 * delirium_data::kv!(12));
    let (m1, m2) = (hip.mid_lean1, hip.mid_lean2);
    let (i1, i2) = (1.0 / s1, 1.0 / s2);
    let (c1, c2) = ((a1 + l3).fma(0.5, -m1) * i1, (a2 + l4).fma(0.5, -m2) * i2);
    let d1 = (l1 - m1).fma(i1, -((l3 - m1) * i1));
    let d2 = (l2 - m2).fma(i2, -((l4 - m2) * i2));
    let num = d2.fma(c2, d1 * c1);
    let den = num - (d2 + c2).fma(d2, (d1 + c1) * d1);
    let t = if den.abs() > delirium_data::kv!(134) { 0.0 } else { num / den };
    let (p1, p2) = (t.fma(d1, c1), t.fma(d2, c2));
    let k = 1.0 / (d2.fma(d2, d1 * d1).sqrt() + delirium_data::kv!(50));
    let (u1, u2) = (k * d1, k * d2);
    let off = p1.fma(u2, -(p2 * u1)).abs();
    let (e1, e2) = if off >= 1.0 {
        (a1, a2)
    } else {
        let h = (-off).fma(off, 1.0).sqrt();
        (h.fma(u1, p1).fma(s1, m1), h.fma(u2, p2).fma(s2, m2))
    };
    let (g1, g2) = (l1 - l3, l2 - l4);
    let k = 1.0 / (g2.fma(g2, g1 * g1).sqrt() + delirium_data::kv!(50));
    (k * g2).fma(e2 - a2, k * g1 * (e1 - a1))
}

fn decide(s: &Snap, f: &mut Feet, r: Option<&crate::nmdbm_rdr2::Rdr2>) {
    let mut give = None;
    // with the COM over his feet neither foot counts as farther, unless (RDR2) he is turning on the spot
    let inside = f.flags & delirium_data::kv!(1 int) != 0;
    let free = !inside || r.is_some_and(|r| r.turning);
    // RDR2's stepIfInSupport off (or standing still): no step at all while the COM is over his feet
    if let Some(r) = r {
        if f.stepping == 0 && inside && !r.turning && (!r.p.step_if_in_support || r.speed <= 0.0) {
            return;
        }
    }
    // RDR2, moving sideways or staggering (0x2ab32bc): after a step the other foot leads this decision; before the first
    // there's no last foot and no lead
    let mut d = f.dist;
    if let Some(i) = r.filter(|r| r.sideways || r.stagger).and_then(|r| r.last_foot) {
        d[1 - i] += crate::nmdbm_rdr2::Rdr2::alternate();
    }
    let a = d[0] > d[1] && free;
    let b = d[1] > d[0] && free;
    if let Some(r) = r {
        if f.stepping != 0 && r.redecide() {
            f.stepping = 0;
        }
        // RDR2's running anti-crossing (0x2ab32bc): the foot in the air gives way to the other when the planted one is too
        // far behind him along his way and the swing one ahead; the thresholds by whether he moves forward (bal+0x2f0, +0x2f4)
        // Measured from the pelvis, horizontally; moving back, along from the stepping foot's side to the other's if that
        // has a part along his way. bal+0x338 turns it off.
        if r.running && !r.falling && f.stepping != 0 {
            let up = r.up_now;
            let forward = s.vel.vd(horizontal(-s.m.c, up)) > 0.0;
            let mut d = s.vel;
            if !forward {
                let e = horizontal(s.legs[0].foot.m.d - s.legs[1].foot.m.d, up) * if f.stepping == 2 { -1.0 } else { 1.0 };
                if s.vel.dot(e) > 0.0 {
                    d = e;
                }
            }
            let d = d.normalize_or_zero();
            let (back, ahead) = if forward { (r.p.run_back_fwd, r.p.run_ahead_fwd) } else { (r.p.run_back, r.p.run_ahead) };
            let (fl, fr) = (d.dot(horizontal(s.legs[0].foot.m.d - s.pelvis.m.d, up)), d.dot(horizontal(s.legs[1].foot.m.d - s.pelvis.m.d, up)));
            if (f.stepping == 2 && fl < -back && fr > ahead) || (f.stepping == 1 && fr < -back && fl > ahead) {
                crate::extras_core_rdr2::trace_once("running anti-crossing", || format!("{fl:.2} {fr:.2} past {back} {ahead}"));
                give = Some(2 - f.stepping as usize);
            }
        }
        // bal+0x334: the legs met, and the step gives way to the other foot, unless that foot is itself waiting
        if r.give_way && f.stepping != 0 {
            let other = 2 - f.stepping as usize;
            if r.wait[other] <= 0.0 {
                crate::extras_core_rdr2::trace_once("legs met", || format!("foot {} gives way", f.stepping));
                give = Some(other);
            }
        }
        if give.is_some() {
            f.stepping = 0;
        }
        if f.stepping == 0 && give.is_none() && std::env::var("NMX_WAIT").ok().and_then(|v| v.parse::<f32>().ok()).is_some_and(|w| r.time < w) { return; } // NMXHOOK
        // "don't change step": early in the balance (dontStepTime), or fallType 1 once failing (0x2aba1cc)
        if give.is_none() && f.stepping == 0 && (!r.may_step() || r.force_balance || (r.failing.is_some() && r.p.fall_type == 1) || r.time < r.p.dont_step_time) {
            return;
        }
    }
    if f.stepping != 0 {
        return;
    }
    let (lc, rc) = (s.flags & delirium_data::kv!(251 int) != 0, s.flags & delirium_data::kv!(7 int) != 0);
    let mut left = if rc { a || !lc } else { !lc && b };
    let mut right = if lc { b || !rc } else { !rc && a };
    if r.is_some_and(|r| r.p.always_step_with_farthest) {
        (left, right) = (a, b);
    }
    // RDR2's forced foot: it takes the next step
    if let Some((i, _)) = r.and_then(|r| r.forced.filter(|&(_, at)| r.steps < at + 1)) {
        (left, right) = (i == 0, i == 1);
    }
    if let Some(i) = give {
        (left, right) = (i == 0, i == 1);
    }
    if left || right {
        if f.countdown >= 0 {
            f.countdown -= 1;
        }
        if f.countdown <= 0 {
            (left, right) = (false, false);
        }
    }
    let (alone, far) = if left {
        f.stepping = 1;
        (rc && !lc, f.dist[0] < f.dist[1])
    } else if right {
        f.stepping = 2;
        (lc && !rc, f.dist[1] < f.dist[0])
    } else {
        return;
    };
    f.flags = (f.flags & !delirium_data::kv!(107 int)) | (alone as u8) << delirium_data::kv!(1 int) | (far as u8) << delirium_data::kv!(0 int);
    f.start = f.pos[f.stepping as usize - 1];
    f.near_time = 0.0;
}

pub fn foot_placement(p: &Params, s: &Snap, f: &mut Feet) {
    let up = p.up;
    f.height = [sdot(p.target[0], up, b"zyx"), sdot(p.target[1], up, b"zyx")];
    f.pos[0] = sat(s.legs[0].foot.m.d, up, f.height[0], b"zyx");
    f.pos[1] = sat(s.legs[1].foot.m.d, up, f.height[1], b"zyx");
    if s.flags & delirium_data::kv!(251 int) == 0 || s.flags & delirium_data::kv!(7 int) == 0 {
        let [l, r] = &mut f.pos;
        separate(l, r, p.rdr2.map_or(p.foot_width, |r| p.foot_width + r.p.extra_feet_apart));
    } else if let Some(r) = p.rdr2 {
        // RDR2: both feet down further apart than legsApartMax draw in toward the predicted COM
        let [lf, rf] = &mut f.pos;
        crate::nmdbm_rdr2::Rdr2::pull_in(lf, rf, r.p.legs_apart_max + p.foot_width + r.p.extra_feet_apart,
                                         s.vel.mul_add(Vec3::splat(p.lead), s.com), up);
    }
    f.centre = p.rdr2.map_or_else(|| (f.pos[0] + f.pos[1]) * 0.5, |r| r.support(f.pos[0], f.pos[1]));
    f.dist = [0.0; 2];
    let pc = sat(s.hvel.mul_add(Vec3::splat(p.lead), s.com), up, sdot(f.centre, up, b"zyx"), b"yxz");
    // RDR2 core: braceForImpact's car box clips the prediction, else teeter's exclusion plane keeps it on its side
    let pc = match p.rdr2 {
        Some(r) if r.car.is_some() => r.clip_car(s.com, pc),
        Some(r) if r.plane.is_some() => r.exclude(pc),
        _ => pc,
    };
    let inside = match p.rdr2 {
        Some(r) => support(pc, &s.legs[0].foot, &s.legs[1].foot, delirium_data::kv!(68), delirium_data::kv!(9), up) > p.p172 - r.p.step_decision_threshold,
        None => support(pc, &s.legs[0].foot, &s.legs[1].foot, delirium_data::kv!(68), delirium_data::kv!(9), up) > p.p172,
    };
    if std::env::var_os("NMX_SUP").is_some() { eprintln!("SUP pc {pc:?} val {} up {up:?} L a {:?} c {:?} d {:?} R a {:?} c {:?} d {:?} k {} {}", support(pc, &s.legs[0].foot, &s.legs[1].foot, delirium_data::kv!(68), delirium_data::kv!(9), up), s.legs[0].foot.m.a, s.legs[0].foot.m.c, s.legs[0].foot.m.d, s.legs[1].foot.m.a, s.legs[1].foot.m.c, s.legs[1].foot.m.d, delirium_data::kv!(68) as f32, delirium_data::kv!(9) as f32); }
    f.flags = (f.flags & !delirium_data::kv!(1 int)) | (inside as u8) << 2;
    f.inside_raw = inside;
    let (lpos, rpos) = (s.legs[0].foot.m.d, s.legs[1].foot.m.d);
    match p.rdr2 {
        Some(r) => {
            let apart = slen(shz(rpos - lpos, up, b"yzx"), b"zyx");
            if f.flags & delirium_data::kv!(1 int) != 0 && (apart > p.foot_width + r.legs_apart() || apart < p.foot_width - r.p.legs_together_restep) {
                f.flags &= !delirium_data::kv!(1 int);
            }
        }
        None => if f.flags & delirium_data::kv!(1 int) != 0 && slen(shz(rpos - lpos, up, b"yzx"), b"zyx") > p.foot_width + delirium_data::kv!(60) {
            f.flags &= !delirium_data::kv!(1 int);
        },
    }
    let (a, c, so) = (s.m.a, s.m.c, p.step_offset);
    let [sa, sb] = p.rdr2.map_or([so; 2], |r| r.widths(p.foot_width, slen(s.hvel, b"zyx"), f.stepping));
    let dir_b = vnorm(shz(pc - rpos, up, b"yxz"));
    let target_a = (c * sdot(c, dir_b, b"zyx") * sdot(a, dir_b, b"zyx")).mul_add(Vec3::splat(-sa), pc) - a * sa;
    let dir_a = vnorm(shz(pc - lpos, up, b"yxz"));
    let target_b = a.mul_add(Vec3::splat(sb), (c * sdot(c, dir_a, b"zyx") * sdot(a, dir_a, b"zyx")).mul_add(Vec3::splat(sb), pc));
    // RDR2's avoidLeg (0x2ab127c): each target stays on its side of the other foot, by the corridor's half width, half a
    // foot ahead of it; the two lines crossing push the targets apart by avoidFeedback (the running case's extra isn't here)
    let (target_a, target_b) = match p.rdr2.filter(|r| r.p.avoid_leg) {
        Some(r) => {
            let n = horizontal(s.legs[1].hip.m.d - s.legs[0].hip.m.d, up).normalize_or_zero();
            let w = if s.vel.dot(horizontal(-s.m.c, up)) > 0.0 { r.p.avoid_foot_width_fwd } else { r.p.avoid_foot_width };
            let ahead = |i: usize| horizontal(-s.legs[i].foot.m.c, up).normalize_or_zero() * 0.15;
            let (cl, cr) = (rpos + ahead(1) - n * w, lpos + ahead(0) + n * w);
            let (mut ta, mut tb) = (target_a, target_b);
            if (ta - cl).dot(n) > 0.0 || (tb - cr).dot(n) < 0.0 {
                crate::extras_core_rdr2::trace_once("avoidLeg", || format!("{:.3} {:.3}, half width {w}", (ta - cl).dot(n), (tb - cr).dot(n)));
            }
            ta -= n * (ta - cl).dot(n).max(0.0);
            tb -= n * (tb - cr).dot(n).min(0.0);
            let e = (cl - cr).dot(n);
            if e < 0.0 {
                ta += n * (r.p.avoid_feedback * e);
                tb -= n * (r.p.avoid_feedback * e);
            }
            (ta, tb)
        }
        None => (target_a, target_b),
    };
    f.dist = [slen(shz(target_a - lpos, up, b"yxz"), b"zyx"), slen(shz(target_b - rpos, up, b"yzx"), b"zyx")];
    // RDR2 (0x2ab127c): while the foot that gave way waits, no decision and no landing
    let waiting = |f: &Feet| p.rdr2.is_some_and(|r| (f.stepping == 1 && r.wait[1] > 0.0) || (f.stepping == 2 && r.wait[0] > 0.0));
    if !waiting(f) {
        decide(s, f, p.rdr2.as_ref());
    }
    if f.stepping == 0 {
        f.flags = (f.flags & delirium_data::kv!(252 int)) | (s.flags & delirium_data::kv!(251 int)) | (s.flags & delirium_data::kv!(7 int)) << 1;
        return;
    }
    let two = f.stepping == 2;
    let (i, side, mut t, contact) = if two {
        (1, -1.0, target_b, s.flags & delirium_data::kv!(7 int) != 0)
    } else {
        (0, 1.0, target_a, s.flags & delirium_data::kv!(251 int) != 0)
    };
    let leg = &s.legs[i];
    let stance = f.pos[1 - i];
    // RDR2's "other foot farther" needs a margin (bal+0x314): 0.2 leg lengths, 0.01 turning
    let margin = p.rdr2.map_or(0.0, |r| if r.turning && !r.turn_margin { 0.01 } else { 0.2 });
    if if two { f.dist[1] < f.dist[0] - margin } else { f.dist[0] < f.dist[1] - margin } {
        f.flags |= delirium_data::kv!(23 int);
    }
    if !contact {
        f.flags |= delirium_data::kv!(54 int);
    }
    f.centre = s.hvel.mul_add(Vec3::splat(delirium_data::kv!(9)), s.com);
    if let Some(r) = p.rdr2 {
        t = keep_near_pelvis(t, &s.pelvis.m, up, side, r.running, p.dt, &mut f.near_time);
    }
    t = reach(t, up, &s.pelvis.m, p.step_clamp * leg.hip.half_lean1, p.step_clamp * leg.hip.half_lean2);
    let fp = leg.foot.m.d;
    let dir_t = vnorm(shz(t - f.start, up, b"yxz"));
    let ahead = sdot(t, dir_t, b"zyx") - sdot(fp, dir_t, b"zyx");
    let closing = sdot(s.vel, dir_t, b"zyx") - sdot(leg.foot.v, dir_t, b"zyx");
    let left_frac = slen(shz(t - fp, up, b"zyx"), b"zyx") / (slen(shz(f.start - fp, up, b"zyx"), b"zyx") + delirium_data::kv!(50));
    let sole = if p.rdr2.is_some() { fp - leg.foot.m.c * 0.099 } else { fp - leg.foot.m.c * delirium_data::kv!(60) * 0.5 };
    let passed = slen(shz(sole - f.prev_target, up, b"zyx"), b"yxz") > slen(shz(f.prev_sole - t, up, b"zyx"), b"zyx");
    let mut speed = (slen(shz(f.prev_sole - f.prev_target, up, b"zyx"), b"yxz") - slen(shz(sole - t, up, b"zyx"), b"yxz")) / p.dt;
    f.prev_target = t;
    f.prev_sole = sole;
    let t1 = up.mul_add(Vec3::splat(sdot(f.pos[i], up, b"zyx") - sdot(t, up, b"yxz")), t);
    let (r80, r84) = hip_leans(&leg.hip, t1, leg.hip.mid_lean1, leg.hip.mid_lean2, side);
    let (r88, r92) = hip_leans(&leg.hip, f.start, leg.hip.mid_lean1, leg.hip.mid_lean2, side);
    // RDR2 (0x2ab127c): the foot's own speed counts 0.03 s, not 0.1 s, and with the stance foot off the ground the
    // target hardly moves from the ground (0.01), unless running or staggering
    let closed = closing.min(0.0).fma(p.rdr2.map_or(delirium_data::kv!(78), |_| 0.03), ahead);
    let stance_down = if f.flags & delirium_data::kv!(251 int) != 0 { s.flags & delirium_data::kv!(251 int) != 0 } else { s.flags & delirium_data::kv!(7 int) != 0 };
    let share = match p.rdr2 {
        Some(r) if !stance_down && !r.running && !r.stagger => 0.01,
        _ => 0.5,
    };
    let lift = (closed * share).clamp(-p.step_height, p.step_height);
    let (n, foot) = if two { (p.normal[1], &s.legs[1].foot) } else { (p.normal[0], &s.legs[0].foot) };
    let c2 = foot.w.vx(foot.m.c).mul_add(Vec3::splat(delirium_data::kv!(112)), foot.m.c);
    let tilt = delirium_data::kv!(12) / (sdot(n, up, b"zyx") + 0.5);
    let toe = sdot(n, c2, b"zyx");
    let room = hip_room(&leg.hip, r80, r84, r88, r92);
    let mut h = match p.rdr2 {
        // RDR2: the foot's pitch only while the foot is still short of the target, and no limit from the hip's room
        Some(_) => if closed > 0.04 { toe.abs().min(1.0).fma(delirium_data::kv!(78), lift) } else { lift },
        None => toe.abs().min(1.0).fma(delirium_data::kv!(78), lift).min(room),
    };
    let mut g = p.target[i];
    // RDR2 clears the step's lift while the COM is over his feet, except on a turn step
    if f.flags & delirium_data::kv!(54 int) == 0 && p.rdr2.is_none_or(|r| r.turning || !inside) {
        h += p.rdr2.map_or(delirium_data::kv!(112), |r| if r.turning { r.turn_lift } else { r.lift });
        g = p.gup.mul_add(Vec3::splat(sdot(p.gup, f.start - g, b"zyx").max(0.0)), g);
    }
    let h = g.x.fma(up.x, g.z.fma(up.z, tilt.fma(h, g.y * up.y)));
    if toe < p.rdr2.map_or(0.0, |_| -0.4) || passed {
        speed = -1.0;
    }
    let t2 = sat(t1, up, h, b"yxz");
    let t3 = reach(t2 + clamp_length(shz(t2 - f.pos[i], up, b"zyx"), 0.0, delirium_data::kv!(78)), up, &s.pelvis.m,
                   p.step_clamp * leg.hip.half_lean1, p.step_clamp * leg.hip.half_lean2);
    f.pos[i] = t3;
    let mid = (t3 + stance) * 0.5;
    f.centre = up.mul_add(Vec3::splat(sdot(mid, up, b"yzx") - sdot(f.centre, up, b"yzx")), f.centre);
    f.flags = (f.flags & delirium_data::kv!(252 int)) | if two { delirium_data::kv!(251 int) } else { delirium_data::kv!(253 int) };
    let other = if two { s.flags & delirium_data::kv!(251 int) != 0 } else { s.flags & delirium_data::kv!(7 int) != 0 };
    let landed = contact && speed < 0.0 && (left_frac < 0.5 || (f.flags & delirium_data::kv!(23 int) != 0 && f.flags & delirium_data::kv!(54 int) != 0));
    // RDR2: the other foot off the ground this tick and the last, and not while running
    let lone = match p.rdr2 {
        Some(r) => {
            let last: u8 = if two { LEFT_LAST() } else { RIGHT_LAST() };
            contact && !other && s.flags & last == 0 && !r.running
        }
        None => contact && !other,
    };
    if std::env::var_os("NMX_DBG").is_some() { eprintln!("PLACE dist {:?} vel {:?} com {:?} Lf {:?} Rf {:?} lead {} stepping {} flags {:b} contact {} other {} speed {:.3} left {:.3} toe {:.3} passed {} landed {} lone {} running {:?}", f.dist, s.hvel, s.com, s.legs[0].foot.m.d, s.legs[1].foot.m.d, p.lead, f.stepping, s.flags, contact, other, speed, left_frac, toe, passed, landed, lone, p.rdr2.map(|r| r.running)); }
    if !waiting(f) && (landed || lone) {
        f.flags |= if two { delirium_data::kv!(253 int) } else { delirium_data::kv!(251 int) };
        f.stepping = 0;
        decide(s, f, p.rdr2.as_ref());
    }
}

/// RDR2 (0x2ab127c): the swing target's horizontal reach from under the pelvis (p = target - pelvis + pelvis row a, about
/// horizontal when the leg is straight), 0.544 m out on the foot's own side once it has been there, for the rest of the
/// step, and 0.32 m when it has crossed to the other side.
fn keep_near_pelvis(t: Vec3, pelvis: &M34, up: Vec3, side: f32, running: bool, dt: f32, time: &mut f32) -> Vec3 {
    let p = t - pelvis.d + pelvis.a;
    let (len, dir) = (slen(p, b"zyx"), vnorm(shz(p, up, b"yxz")));
    let d = sdot(dir, vnorm(shz(-pelvis.b, up, b"yxz")), b"zyx") * side;
    let (mut t, mut v) = (t, dir);
    if !running && (d < -0.7 || *time >= 1.0 / 15.0) {
        *time += dt;
        v = dir * len;
        let c = clamp_length(v, 0.0, 0.544);
        t -= v - c;
        v = c;
    }
    if d > 0.7 {
        let w = v * len;
        t -= w - clamp_length(w, 0.0, 0.32);
    }
    t
}

fn yaw(k: f32, pelvis: &M34, w: Vec3, toward: bool, predict: bool, angvel: Vec3) -> f32 {
    let w1 = if predict { qrot(rotvec_quat(angvel * -delirium_data::kv!(60)), w, false) } else { w };
    let a = atan2(fneg(sdot(pelvis.c, w1, b"zyx")), fneg(sdot(pelvis.b, w1, b"zyx")) * k);
    let half = std::f32::consts::FRAC_PI_2;
    let mut a = if toward {
        if a >= 0.0 { a - half } else { a + half }
    } else {
        let mut a = a - half;
        if a < -std::f32::consts::PI {
            a += std::f32::consts::TAU;
        }
        if !(-delirium_data::kv!(254)..=delirium_data::kv!(254)).contains(&a) { 0.0 } else { a }
    };
    if predict {
        a = slen(w, b"yxz").min(1.0) * a * delirium_data::kv!(255);
    }
    a
}

pub fn pelvis_control(p: &Params, s: &Snap, pv: &mut Pelvis) {
    let up = p.up;
    let [pl, pr] = pv.planted;
    // RDR2 (0x2ab3bb4): staggering doesn't count as both planted, and the pitch target stays where it is (bal+0x337)
    let staggering = p.rdr2.is_some_and(|r| r.stagger);
    let both = pl && pr && !staggering && !p.rdr2.is_some_and(|r| r.kneeling);
    pv.off = shz(s.pelvis.m.d - s.com, up, b"zyx");
    let w = if pv.fixed_vel { pv.vel } else { s.hvel };
    // RDR2: slower than useComDirTurnVelThresh the twist takes the way he faces, not the velocity
    let w = match p.rdr2 {
        Some(r) if !pv.fixed_vel && s.vel.length() < r.p.use_com_dir_turn_vel_thresh => {
            crate::extras_core_rdr2::trace_once("useComDirTurnVelThresh", || format!("{:.2} m/s under {}", s.vel.length(), r.p.use_com_dir_turn_vel_thresh));
            horizontal(-s.m.c, up).normalize_or_zero()
        }
        _ => w,
    };
    // RDR2 (0x2ab3bb4): moving sideways the leg's twist doesn't turn him toward his way
    let toward = !pv.fixed_vel && !p.rdr2.is_some_and(|r| r.sideways);
    let m = &s.pelvis.m;
    let blend = |y: f32, cur: f32| ((y.clamp(-delirium_data::kv!(119), delirium_data::kv!(119)) - cur) * (slen(w, b"yxz") * 2.0).min(1.0)).fma(delirium_data::kv!(60), cur);
    // RDR2, running (staggering too): the twist goes by the feet's contact, not the planted flags. One foot down follows as
    // walking; both down relax; neither follows at half rate, each leg to minus its own heading error
    let running = p.rdr2.is_some_and(|r| r.running);
    let (pl, pr) = if running { (s.flags & delirium_data::kv!(251 int) != 0, s.flags & delirium_data::kv!(7 int) != 0) } else { (pl, pr) };
    if running && !pl && !pr {
        let limit = delirium_data::kv!(119);
        let k = (slen(w, b"yxz") * 2.0).min(1.0);
        for (i, side) in [(0, 1.0), (1, -1.0)] {
            pv.yaw[i] += k * (-yaw(side, m, w, toward, false, s.angvel).clamp(-limit, limit) - pv.yaw[i]) * 0.5;
        }
    } else if pl && !pr {
        pv.yaw[0] = blend(yaw(1.0, m, w, toward, false, s.angvel), pv.yaw[0]);
        pv.yaw[1] = pv.yaw[0] * 0.5;
    } else if pr && !pl {
        pv.yaw[1] = blend(yaw(-1.0, m, w, toward, false, s.angvel), pv.yaw[1]);
        pv.yaw[0] = pv.yaw[1] * 0.5;
    } else if pv.fixed_vel && !running {
        let most = if p.rdr2.is_some_and(|r| r.turning) { crate::nmdbm_rdr2::TURN_TWIST } else { delirium_data::kv!(119) };
        pv.yaw[0] = (yaw(1.0, m, w, toward, true, s.angvel) + (pv.yaw[0] - pv.yaw[1]) * 0.5).clamp(-most, most);
        pv.yaw[1] = -pv.yaw[0];
    } else {
        let step = p.dt * 0.5;
        // RDR2: the legs' twist rest offset (a crouch start's turn-out) on top of the base
        let rest = p.rdr2.map_or(p.p148, |r| p.p148 + r.twist_rest);
        let goal = [p.p152 - rest, -p.p152 - rest];
        for k in 0..2 {
            let y = pv.yaw[k];
            pv.yaw[k] = if y > goal[k] { (y - step).max(goal[k]) } else { (y + step).min(goal[k]) };
        }
    }
    if std::env::var_os("NMX_TW").is_some() { eprintln!("TW {:.3} yaw {:.3} {:.3} w {:.3} {:.3} toward {} pl {} pr {} fixed {} run {} y1 {:.3} y2 {:.3} pelvis b {:.3} {:.3} c {:.3} {:.3} a {:.3} {:.3}", p.rdr2.map_or(0.0, |r| r.time), pv.yaw[0], pv.yaw[1], w.x, w.y, toward, pl, pr, pv.fixed_vel, running, yaw(1.0, m, w, toward, false, s.angvel), yaw(-1.0, m, w, toward, false, s.angvel), m.b.x, m.b.y, m.c.x, m.c.y, m.a.x, m.a.y); } // NMXHOOK
    let hz = shz(s.m.c, up, b"zyx");
    let hx = shz(-s.m.a, up, b"xyz");
    let d = s.com - pv.centre;
    let a = (-sdot(d, hz, b"yxz")).clamp(-delirium_data::kv!(226), delirium_data::kv!(9));
    let b = -sdot(d, hx, b"zyx");
    let c = -sdot(s.hvel, hz, b"zyx");
    let e = -sdot(s.hvel, hx, b"yzx");
    let f4 = p.lead * delirium_data::kv!(256);
    if both {
        pv.p52 = p.dt.fma(a, pv.p52).clamp(p.lean_bias - delirium_data::kv!(78), p.lean_bias + delirium_data::kv!(78));
        pv.p56 = p.dt.fma(b, pv.p56).clamp(-delirium_data::kv!(78), delirium_data::kv!(78));
    }
    // RDR2's ankle balance takes the roll integrator out
    let roll_i = if p.rdr2.is_some_and(|r| r.p.ankle_balance) { 0.0 } else { pv.p56 };
    pv.out[0] = b.fma(delirium_data::kv!(257), (f4 * 0.5).fma(e, roll_i * delirium_data::kv!(258))) * delirium_data::kv!(259);
    if both {
        pv.out[1] = a.fma(delirium_data::kv!(256), c.fma(f4, pv.p52 * delirium_data::kv!(260))) * -delirium_data::kv!(261);
        // RDR2: on both feet, once he has stepped, moving backward lowers the pitch target too
        if let Some(r) = p.rdr2.filter(|r| r.steps > 0) {
            pv.acc -= if r.p.ankle_balance { (-0.4 * c).clamp(-0.03, 0.2) + 0.03 } else { (-0.7 * c).clamp(0.0, 0.5) };
        }
    } else if staggering {
        pv.out[1] = 0.0;
    } else {
        pv.out[1] = 0.0;
        pv.acc -= match p.rdr2 {
            // RDR2: (5 - hipError) x 0.052 or 0.04, which at 0 are these
            Some(r) if r.p.ankle_balance => ((5.0 - r.hip_error().clamp(0.0, 4.0)) * -0.052 * c).clamp(0.0, 0.2),
            Some(r) => ((5.0 - r.hip_error().clamp(0.0, 4.0)) * -0.04 * c).clamp(0.0, 0.5),
            _ => (-delirium_data::kv!(60) * c).clamp(0.0, 0.5),
        };
    }
    let g = -sdot(p.normal[1], s.legs[1].foot.m.c, b"yzx") + -sdot(p.normal[0], s.legs[0].foot.m.c, b"yzx");
    if g > delirium_data::kv!(68) {
        pv.out[1] = ((g - delirium_data::kv!(68)) * delirium_data::kv!(5)).min(1.0).fma(-pv.out[1], pv.out[1]);
    }
}

pub(crate) fn asin(x: f32) -> f32 {
    if x <= -1.0 { -std::f32::consts::FRAC_PI_2 } else if x >= 1.0 { std::f32::consts::FRAC_PI_2 } else { (x as f64).asin() as f32 }
}

pub(crate) fn arc(a: Vec3, b: Vec3) -> Quat {
    let (na, mut nb) = (vnorm(a), vnorm(b));
    let d = na.y.fma(nb.y, na.z.fma(nb.z, na.x * nb.x));
    if (d - 1.0).abs() < delirium_data::kv!(50) {
        return Quat::IDENTITY;
    }
    let c = d + 1.0;
    if c.abs() < delirium_data::kv!(50) {
        let big_z = if a.x < a.y { a.y < a.z } else { a.x < a.z };
        nb = if big_z { Vec3::Y } else { Vec3::Z };
    }
    let x = na.y.fma(nb.z, -(na.z * nb.y));
    let y = na.z.fma(nb.x, -(na.x * nb.z));
    let z = na.x.fma(nb.y, -(na.y * nb.x));
    let n2 = c.fma(c, x.fma(x, z.fma(z, y * y)));
    let r = if n2 <= 0.0 {
        0.0
    } else {
        let (n, mut y) = (n2 as f64, crate::estimate::frsqrte(n2 as f64));
        for _ in 0..2 {
            y = fneg(y * y).mul_add(n, 1.0).mul_add(y * 0.5, y);
        }
        y as f32
    };
    Quat::from_xyzw(x * r, y * r, z * r, c * r)
}

fn qmul(a: Quat, b: Quat) -> Quat {
    Quat::from_xyzw(
        (-b.y).fma(a.z, b.z.fma(a.y, a.x.fma(b.w, b.x * a.w))),
        (-b.z).fma(a.x, b.y.fma(a.w, a.y.fma(b.w, a.z * b.x))),
        (-a.y).fma(b.x, b.y.fma(a.x, a.z.fma(b.w, b.z * a.w))),
        (-b.z).fma(a.z, (-b.y).fma(a.y, b.w.fma(a.w, -(a.x * b.x)))),
    )
}

fn local(m: &M34, v: Vec3) -> Vec3 {
    Vec3::new(m.a.vd(v), m.b.vd(v), m.c.vd(v))
}

fn world(m: &M34, v: Vec3) -> Vec3 {
    let col = |k: usize| Vec3::new(m.a[k], m.b[k], m.c[k]);
    Vec3::new(col(0).vd(v), col(1).vd(v), col(2).vd(v))
}

struct LegIk {
    /// RDR2's self avoidance: what it asks, the spine's top, the pelvis, the top's axis negated
    avoid: Option<(crate::extras_core_rdr2::SelfAvoid, Vec3, Vec3, Vec3)>,
    target: Vec3,
    lead: f32,
    yaw: f32,
    side: f32,
    bend: f32,
    w: f32,
    reach: f32,
    lift: f32,
    swing: bool,
    free: bool,
    /// RDR2's req+0x64: the target's greatest distance from the hip (0 off)
    max_reach: f32,
}

fn lead(c: &LegIk, hip: &Eff3, pelvis: &Part, vel: Vec3) -> Vec3 {
    let r = c.target - hip.m.d;
    let k = hip.damping / (hip.strength + delirium_data::kv!(78)) * c.lead;
    let cr = r.vx(pelvis.w) * k;
    let turned = r + cr;
    let mut out = (vnorm(turned).mul_add(Vec3::splat(slen(r, b"zyx") - slen(turned, b"zyx")), cr) - (pelvis.v - vel) * k) + c.target;
    let n = vnorm(r);
    let along = sdot(n, out - hip.m.d, b"zyx");
    if along < delirium_data::kv!(78) {
        out = n.mul_add(Vec3::splat(delirium_data::kv!(78) - along), out);
    }
    out
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn limb_ik(b: &Body, ch: &mut Character, j3: usize, j1: usize, root: usize, end: usize, target: Vec3, yaw: f32,
                      lead: f32, side: f32, bend: f32, vel: Vec3, swing: Option<(f32, f32)>) {
    limb_ik_w(b, ch, j3, j1, root, end, target, yaw, lead, side, bend, vel, swing, 1.0, false);
}

#[allow(clippy::too_many_arguments)]
pub(crate) fn limb_ik_w(b: &Body, ch: &mut Character, j3: usize, j1: usize, root: usize, end: usize, target: Vec3, yaw: f32,
                        lead: f32, side: f32, bend: f32, vel: Vec3, swing: Option<(f32, f32)>, w: f32, free: bool) {
    let (mut hip, mut knee) = (b.eff3(j3), b.eff1(j1));
    (hip.desired, hip.strength, hip.damping) = (ch.eff[j3].desired, ch.eff[j3].strength, ch.eff[j3].damping);
    (knee.desired, knee.strength, knee.damping) = (ch.eff[j1].desired[0], ch.eff[j1].strength, ch.eff[j1].damping);
    let (reach, lift) = swing.unwrap_or((0.0, 0.0));
    let avoid = if ch.rdr2 && ch.core.cfg.self_avoid.on && crate::nmrs::SHOULDER().contains(&j3) {
        let top = b.part(crate::nmrs::SPINE().last().map_or(0, |j| j + 1));
        Some((ch.core.cfg.self_avoid, top.m.d, b.part(0).m.d, -top.m.b))
    } else {
        None
    };
    let mut c = LegIk { target, lead, yaw, side, bend, w, reach, lift, swing: swing.is_some(), free, avoid, max_reach: 0.0 };
    leg_ik(&mut c, &b.limb(j3, j1), &mut knee, &mut hip, &b.part(end), &b.part(root), vel);
    ch.eff[j3].desired = hip.desired;
    ch.eff[j1].desired[0] = knee.desired;
}

fn leg_ik(c: &mut LegIk, limb: &M34, knee: &mut Eff1, hip: &mut Eff3, foot: &Part, pelvis: &Part, vel: Vec3) {
    let mut f25 = 0.0;
    let mut x = if c.lead > 0.0 { lead(c, hip, pelvis, vel) } else { c.target };
    // RDR2's arms with configureSelfAvoidance (flag 8, 0x2a4b520 in 0x2a3f0c0)
    if let Some((r, a, b, n)) = c.avoid {
        let from = if r.drag { c.target } else { x };
        let (t, twist) = crate::extras_core_rdr2::self_avoid_target(&r, a, b, n, from, foot.m.d, hip.m.d, knee.child.d);
        if (t - from).length_squared() > 1e-8 {
            crate::extras_core_rdr2::trace_first("self avoidance moved an arm's target");
        }
        x = t;
        if let Some(y) = twist {
            c.yaw = y;
        }
    }
    if c.swing {
        let d = x - foot.m.d;
        let m = slen(d, b"zxy").min(hip.damping / hip.strength * c.reach) + delirium_data::kv!(72);
        let f0 = (1.0 - delirium_data::kv!(78) / m).max(0.0);
        let f12 = if c.lift > f0 { (c.lift + f0) * 0.5 } else { c.lift };
        x = (vnorm(d) * m).mul_add(Vec3::splat(1.0 - f12), foot.m.d);
        f25 = 1.0 / (delirium_data::kv!(262) - f12);
        c.lead *= 1.0 - f12;
    }
    if c.max_reach > 0.0 {
        let d = x - hip.m.d;
        if d.length() > c.max_reach {
            x = hip.m.d + d.normalize_or_zero() * c.max_reach;
            crate::extras_core_rdr2::trace_once("balancer: lift multiplier's reach", || format!("{} m from the hip", c.max_reach));
        }
    }
    let kv = world(limb, local(&knee.child, foot.m.d - knee.child.d));
    let (ld, lk) = (slen(limb.d, b"yxz"), slen(kv, b"zyx"));
    let off = asin(sdot(limb.c, kv.vx(limb.d), b"zyx") / (lk * ld));
    let mut h = local(&hip.m, x - hip.m.d);
    let hl = slen(h, b"zyx");
    if hl > lk + ld {
        h *= (lk + ld) / hl;
    }
    let arg = (-hl).fma(hl, lk.fma(lk, ld * ld)) / (lk * ld * 2.0);
    let mut bend = (std::f32::consts::PI - acos(arg)).fma(c.bend, off);
    let v = qrot(rotvec_quat(limb.c * bend), kv, false) + limb.d;
    let ys = c.yaw * c.side;
    let vn = vnorm(v);
    let [tw, mut l1, mut l2] = crate::nmrs::angles(qmul(arc(v, h), rotvec_quat(vn * ys)));
    if tw.abs() > hip.half_twist {
        let excess = tw - tw.clamp(-hip.half_twist, hip.half_twist);
        let a = crate::nmrs::angles(qmul(arc(v, h), rotvec_quat(vn * c.yaw.fma(c.side, -excess))));
        (l1, l2) = (a[1], a[2]);
    }
    let (s1, s2) = (l1 * l1, l2 * l2);
    let e = s2 / (hip.half_lean2 * hip.half_lean2) + s1 / (hip.half_lean1 * hip.half_lean1);
    if e > 1.0 && !c.free {
        bend += ((1.0 - 1.0 / e.sqrt()) * (s1 + s2).sqrt() * c.bend).clamp(-0.5, 0.5);
    }
    let (w, u) = (c.w, 1.0 - c.w);
    knee.desired = knee.desired.fma(u, w * bend);
    let want = [hip.mid_lean1 + l1, c.side.fma(l2, hip.mid_lean2), c.side.fma(tw, hip.mid_twist)];
    for k in 0..delirium_data::kv!(0 int) {
        hip.desired[k] = want[k].fma(w, hip.desired[k] * u);
    }
    if c.swing {
        let u = 1.0 - f25;
        for k in 0..delirium_data::kv!(0 int) {
            hip.desired[k] = hip.actual[k].fma(u, f25 * hip.desired[k]);
        }
        knee.desired = knee.actual.fma(u, knee.desired * f25);
    }
}

pub(crate) fn predict_knee(hip: &Eff3, knee: &Eff1, side: f32) -> Vec3 {
    let len = slen(hip.m.d - knee.m.d, b"yzx");
    let q = crate::nmrs::rotation([(hip.desired[2] - hip.mid_twist) * side, hip.desired[0] - hip.mid_lean1,
                                   (hip.desired[1] - hip.mid_lean2) * side]);
    let v = qrot(q, Vec3::new(0.0, 0.0, len), false);
    let m = &hip.m;
    crate::vm4(m.a, m.b, m.c, m.d, v)
}

pub fn balance_solve(p: &Params, f: &Feet, pv: &Pelvis, s: &mut Snap) {
    let up = p.up;
    let (lp, rp) = (f.flags & delirium_data::kv!(251 int) != 0, f.flags & delirium_data::kv!(253 int) != 0);
    // RDR2 (0x2aaeed8): running, or with airborneStep off, a planted foot off the ground on the side he moves to swings
    let lost = match p.rdr2 {
        Some(rd) if rd.running || !rd.p.airborne_step => {
            let along = s.vel.dot(s.legs[1].foot.m.d - s.legs[0].foot.m.d);
            [lp && s.flags & delirium_data::kv!(251 int) == 0 && along < 0.0, rp && s.flags & delirium_data::kv!(7 int) == 0 && along > 0.0]
        }
        _ => [false; 2],
    };
    if lost[0] || lost[1] {
        crate::extras_core_rdr2::trace_once("airborne step", || format!("{lost:?}, running {}", p.rdr2.is_some_and(|r| r.running)));
    }
    let (lp, rp) = (lp && !lost[0], rp && !lost[1]);
    let r = p.pelvis_height;
    let drop = |x: Vec3| {
        let e = ((slen(shz(x - f.centre, up, b"zxy"), b"zyx") - p.hip_dist) * delirium_data::kv!(80)).powi(2).min(r * r * delirium_data::kv!(57));
        r - (r * r - e).sqrt()
    };
    let low = (sdot(f.pos[0], up, b"yxz") - drop(f.pos[0])).min(sdot(f.pos[1], up, b"zxy") - drop(f.pos[1]));
    let mut t = f.pos;
    if let Some(rd) = p.rdr2 {
        // RDR2: the foot that gave way goes down
        for i in 0..2 {
            if f.stepping == (1 - i) as u32 + 1 && rd.wait[i] > 0.0 {
                t[i] -= Vec3::Z * crate::nmdbm_rdr2::GIVE_WAY.1;
            }
        }
        let apart = (f.pos[1] - f.pos[0]).length();
        for (i, planted) in [lp, rp].into_iter().enumerate() {
            if planted {
                t[i] += up * (rd.limp_bend(i, apart) * r);
            }
        }
    }
    if lp {
        t[0] = up.mul_add(Vec3::splat(pv.out[0] * p.hip_dist), t[0]);
    }
    if rp {
        t[1] = up.mul_add(Vec3::splat(-(pv.out[0] * p.hip_dist)), t[1]);
    }
    // RDR2: the pelvis's height over the support centre, or on a slope over a point between the feet
    let mut base = f.centre;
    let rdr2_height = p.rdr2.map(|rd| {
        let l = p.leg_scale() * p.leg_length;
        let h = crate::nmdbm_rdr2::Rdr2::pelvis_height(l, (t[1] - t[0]).length());
        if !rd.on_slope {
            return h;
        }
        let (hi, lo) = if Vec3::Z.dot(t[0]) > Vec3::Z.dot(t[1]) { (0, 1) } else { (1, 0) };
        if f.stepping == 0 {
            t[hi] += rd.slope_foot(up, p.normal[hi], t[hi] - t[lo]);
        }
        let (b, h) = rd.slope_pelvis(up, l, h, t[hi], t[lo], f.stepping != 0, s.vel);
        base = b.unwrap_or(base);
        h
    });
    let lean = pv.out[0].clamp(-delirium_data::kv!(60), delirium_data::kv!(60));
    s.spine.desired[1] = lean;
    s.height = match p.rdr2 {
        // RDR2: the reach shortened by the feet's spread
        Some(_) => rdr2_height.unwrap_or_default(),
        None => r * p.leg_length - (sdot(f.centre, up, b"yzx") - low) + if lp && rp { 0.0 } else { delirium_data::kv!(131) },
    };
    // the running case's target for a foot that swings so: toward the COM ahead by the knee's bend (0x2aaeed8)
    if p.rdr2.is_some_and(|r| r.running) {
        for i in 0..2 {
            if lost[i] {
                let k = s.legs[i].knee.actual;
                let tau = if k > -0.7 { k * -0.542857 } else { 0.38 };
                let to = s.com + s.vel * (tau * tau);
                t[i] += horizontal(to - t[i], up);
            }
        }
    }
    let mut target = t;
    let mut normal = p.normal;
    if lp || rp {
        // RDR2: the hip lean tilts the pelvis frame the legs reach for
        let hup = if p.hip_up != Vec3::ZERO { p.hip_up } else { up };
        let side = vnorm(s.pelvis.m.c.vx(hup));
        let fwd = hup.vx(side);
        let cs = |a: f32| ((a as f64).cos() as f32, (a as f64).sin() as f32);
        let ((cy, sy), (cz, sz)) = (cs(-pv.acc), cs(-pv.out[0]));
        let e = [Vec3::new(cz * cy, sz * cy, -sy), Vec3::new(-sz, cz, 0.0), Vec3::new(cz * sy, sz * sy, cy)];
        // RDR2: dampPelvis off, no turn against the pelvis's spin
        let q = if p.rdr2.is_some_and(|r| !r.p.damp_pelvis) { Quat::IDENTITY } else { rotvec_quat(s.pelvis.w * -delirium_data::kv!(78)) };
        let qm = quat_rows(q);
        let pd: [Vec3; 3] = e.map(|e| {
            let a = fwd.mul_add(Vec3::splat(e.z), side.mul_add(Vec3::splat(e.y), hup * e.x));
            qm[2].mul_add(Vec3::splat(a.z), qm[1].mul_add(Vec3::splat(a.y), qm[0] * a.x))
        });
        let pdd = up.mul_add(Vec3::splat(pv.p68), base) + pv.off;
        let pdm = M34 { a: pd[0], b: pd[1], c: pd[2], d: pdd };
        let pel = s.pelvis.m;
        let row = |k: usize| pel.c.mul_add(Vec3::splat(pd[2][k]), pel.b.mul_add(Vec3::splat(pd[1][k]), pel.a * pd[0][k]));
        let cm = [row(0), row(1), row(2)];
        let cd = pel.c.mul_add(Vec3::splat(-pd[2].vd(pdd)), pel.b.mul_add(Vec3::splat(-pd[1].vd(pdd)), pel.a.mul_add(Vec3::splat(-pd[0].vd(pdd)), pel.d)));
        // RDR2's hipError (rdr2_balancer2.md 9): the stance legs reach as if for a pelvis frame turned on by the arc from
        // the pelvis's first axis to the desired one, scaled by hipError at 60 Hz (none at 30), at most hipErrorMax
        let stance = p.rdr2.filter(|r| r.hip_error() > 0.01 && r.p.hip_error_max > 1e-4).map_or((cm, cd), |r| {
            let (arc, angle) = (bevy_math::Quat::from_rotation_arc(pel.a.normalize_or_zero(), pd[0].normalize_or_zero()), 1.0);
            let (axis, th) = arc.to_axis_angle();
            let scaled = (th * r.hip_error() * (2.0 - 60.0 * p.dt.clamp(1.0 / 60.0, 1.0 / 30.0)) * angle).min(r.p.hip_error_max);
            crate::extras_core_rdr2::trace_once("hipError", || format!("{th:.3} rad to {scaled:.3}, max {}", r.p.hip_error_max));
            let turn = bevy_math::Quat::from_axis_angle(axis, scaled);
            let pe = pd.map(|e| turn * e);
            let row = |k: usize| pel.c.mul_add(Vec3::splat(pe[2][k]), pel.b.mul_add(Vec3::splat(pe[1][k]), pel.a * pe[0][k]));
            ([row(0), row(1), row(2)], pel.c.mul_add(Vec3::splat(-pe[2].vd(pdd)), pel.b.mul_add(Vec3::splat(-pe[1].vd(pdd)), pel.a.mul_add(Vec3::splat(-pe[0].vd(pdd)), pel.d))))
        });
        for (i, planted) in [lp, rp].into_iter().enumerate() {
            if planted {
                let hip = &s.legs[i].hip;
                let t1 = reach(t[i], up, &pdm, p.step_clamp * hip.half_lean1, p.step_clamp * hip.half_lean2);
                target[i] = crate::vm4(stance.0[0], stance.0[1], stance.0[2], stance.1, t1);
                let col = |k: usize| Vec3::new(cm[0][k], cm[1][k], cm[2][k]);
                normal[i] = Vec3::new(col(0).vd(p.normal[i]), col(1).vd(p.normal[i]), col(2).vd(p.normal[i]));
            }
        }
    }
    let vel = s.vel;
    let pelvis = s.pelvis;
    let lfoot_v = s.legs[0].foot.v;
    for (i, planted) in [lp, rp].into_iter().enumerate() {
        let leg = &mut s.legs[i];
        let yaw = match p.rdr2.filter(|r| r.turning) {
            Some(r) => pv.yaw[i].clamp(r.leg_twist.0, r.leg_twist.1) * (1.0 - crate::nmdbm_rdr2::TURN_ANKLE[!planted as usize]),
            None => pv.yaw[i],
        };
        let mut c = LegIk { avoid: None, target: target[i], lead: 0.0, yaw, side: if i == 0 { 1.0 } else { -1.0 }, bend: -1.0,
                            w: 1.0, reach: 0.0, lift: 0.0, swing: !planted, free: false, max_reach: 0.0 };
        // RDR2 (the solve, 0x2ab0863): a lift multiplier over 0 caps the reach by the leg's length, under Leff less 0.05 of the probe's length
        if let Some(r) = p.rdr2.filter(|r| r.lift_mult[i] > 0.0) {
            let l = p.leg_scale();
            c.max_reach = (r.lift_mult[i] * l).min(l * p.leg_length - 0.05).max(0.1);
        }
        if !planted {
            // RDR2 (rdr2_balancer2.md §7.2) lifts each swing foot by its own velocity; GTA IV reads the left foot's for both
            let (fv, gv) = if p.rdr2.is_some() { (leg.foot.v, p.gvel[i]) } else { (lfoot_v, p.gvel[0]) };
            let ahead = target[i] - (fv - gv).mul_add(Vec3::splat(delirium_data::kv!(68)), leg.foot.m.d);
            (c.lead, c.reach, c.lift) = (p.p168, p.rdr2.map_or(delirium_data::kv!(64), |r| r.p.max_foot_speed), sdot(up, ahead, b"yxz").fma(delirium_data::kv!(84), delirium_data::kv!(78)).clamp(delirium_data::kv!(9), delirium_data::kv!(119)));
        }
        let limb = leg.limb;
        leg_ik(&mut c, &limb, &mut leg.knee, &mut leg.hip, &leg.foot, &pelvis, vel);
    }
    // RDR2's ankle balance: each planted ankle's pitch with its foot aimed at the tilted normal
    let aim = p.rdr2.filter(|r| r.p.ankle_balance && !r.stagger).map(|_| {
        let ahead = s.com + s.vel * 0.2;
        let d = [0, 1].map(|i| horizontal(s.legs[i].foot.m.d - ahead, up).length());
        (crate::nmdbm_rdr2::Rdr2::ankle_shares(d), crate::nmdbm_rdr2::Rdr2::ankle_aim(Vec3::Z, up, s.com, horizontal(s.vel, up), f.centre))
    });
    let mut bal = [0.0f32; 2];
    // RDR2's ankle balance twist: the twist laying the foot on the ground / on the tilted normal
    let mut twist_flat = [None; 2];
    // RDR2 (0x2aae0b4, rdr2_balancer3.md §3.2 and §9): off the ankle balance, the stance ankle aims from the knee
    // predicted from the hip's desired angles to the foot target while the velocity has a part along -s.m.c (the COM
    // frame's third row, snapshot +0x4d0, whose negative is forward: moving forward); otherwise from the actual knee
    // to the actual foot. In stagger mode a swing foot moving against its own third axis (velocity relative to the
    // ground, both horizontal and unit: dot < -0.3) has its shin term add the damping part instead of taking it off.
    let (ahead, balancing) = match p.rdr2 {
        Some(_) => (s.vel.vd(horizontal(s.m.c, up)) < 0.0, aim.is_some() && s.vel.length() < 1.0),
        None => (false, false),
    };
    let contact = [s.flags & delirium_data::kv!(251 int) != 0, s.flags & delirium_data::kv!(7 int) != 0];
    let last = [s.flags & LEFT_LAST() != 0, s.flags & RIGHT_LAST() != 0];
    for (i, planted) in [lp, rp].into_iter().enumerate() {
        let side = if i == 0 { 1.0 } else { -1.0 };
        let leg = &mut s.legs[i];
        let a = if let Some(rd) = p.rdr2.filter(|r| !planted && r.flat_swing(i)) {
            crate::extras_core_rdr2::trace_once("flat swing ankle", || format!("down {:?} sideways {} running {}", rd.down_swing, rd.sideways, rd.running));
            // RDR2 (0x2aae0b4): a flat swing foot aims from the actual knee and foot; moving sideways 0.9 more
            let a = asin(sdot(leg.knee.m.c, vnorm(leg.knee.m.d - leg.foot.m.d).vx(normal[i]), b"xyz"));
            if rd.sideways { a + 0.9 } else { a }
        } else if !planted {
            let sh = &leg.shin;
            let back = p.rdr2.is_some_and(|r| r.stagger)
                && horizontal(leg.foot.v, up).normalize_or_zero().vd(horizontal(leg.foot.m.c, up).normalize_or_zero()) < -0.3;
            let a = if back {
                sdot(sh.w, sh.m.a, b"yzx").fma(leg.ankle.damping / leg.ankle.strength.max(1e-10), sdot(sh.m.c, normal[i], b"yzx"))
            } else {
                (-sdot(sh.w, sh.m.a, b"yzx")).fma(leg.ankle.damping / leg.ankle.strength, sdot(sh.m.c, normal[i], b"yzx"))
            };
            match p.rdr2 {
                // RDR2 (0x2aae0b4): the blend with the foot's side only while the foot's horizontal direction, relative to
                // the ground, is at most 0.4 along s.m.c, otherwise the shin term alone; -0.2 only in that blend, moving
                // forward and not staggering; and no acos(leg length) (the recorded swing pitch fits this to 0.01 rad).
                // That ground velocity is bal+0x530, the character's floor (ctx+0x4790, 0 here; nothing under him moves in
                // Delirium), not the leg's moving contact point p.gvel, which leaps with each new step target
                Some(r) => {
                    let along = horizontal(leg.foot.v, up).normalize_or_zero().vd(s.m.c);
                    if along <= 0.4 {
                        let a = sdot(p.normal[i], vnorm(leg.foot.m.a.vx(up)), b"zyx").fma(0.8, a * 0.2);
                        if !r.stagger && along < 0.0 { a - 0.2 } else { a }
                    } else {
                        a
                    }
                }
                None => {
                    let mut a = sdot(p.normal[i], vnorm(leg.foot.m.a.vx(up)), b"zyx").fma(delirium_data::kv!(20), a * delirium_data::kv!(60));
                    if sdot(leg.foot.v, s.m.c, b"yzx") < 0.0 {
                        a -= delirium_data::kv!(60);
                    }
                    a + acos(p.leg_length.clamp(delirium_data::kv!(72), delirium_data::kv!(263)))
                }
            }
        } else {
            // RDR2 (rdr2_balancer3.md 9): flatterStaticFeet, standing: the actual knee and foot on the renormalised
            // normal; off it, moving forward, the pitch is negated above 5 m/s or moving sideways (its caller's flag
            // and the stagger legs' states are left out)
            let flat = p.rdr2.is_some_and(|r| r.p.flatter_static_feet && !r.stagger && s.vel.length() < 1.0);
            // RDR2 (0x2aae0b4), staggering: a foot down by this tick and the last aims at the normal tilted away from his way,
            // unless he's going forward at 1 m/s or more; its pitch is negated, going forward, above 5 m/s, sideways, on a step
            // up, or above 1 m/s with a leg pushing off. A foot not so down aims from the actual knee and foot.
            if let Some(r) = p.rdr2.filter(|r| r.stagger) {
                let g = s.vel.vd(horizontal(s.m.c, up));
                let v = horizontal(s.vel, up);
                let speed = v.length();
                let mut c = speed;
                let mut n = normal[i];
                let down = contact[i] && last[i];
                if down && (speed < 1.0 || g > 0.0) {
                    let k;
                    (c, k) = if r.sideways { (speed.clamp(0.0, 1.0), 3.0) } else if r.push_off { (speed.clamp(0.0, 3.0), 3.0) } else { (speed.clamp(0.7, 1.0), 1.0) };
                    n = (n - v.normalize_or_zero() * (0.8 * c * k)).normalize_or_zero();
                }
                let forward = down && g < 0.0;
                let (kn, ft) = if forward { (predict_knee(&leg.hip, &leg.knee, side), target[i]) } else { (leg.knee.m.d, leg.foot.m.d) };
                let a = asin(sdot(leg.knee.m.c, vnorm(kn - ft).vx(n), b"xyz"));
                let step_up = if !lp { up.vd(p.target[1]) + 0.1 < up.vd(p.target[0]) } else if !rp { up.vd(p.target[0]) + 0.1 < up.vd(p.target[1]) } else { false };
                leg.ankle.desired[0] = if forward && (speed > 5.0 || r.sideways || step_up || (c > 1.0 && r.push_off)) { -a } else { a };
                continue;
            }
            let flip = !flat && !balancing && ahead && p.rdr2.is_some_and(|r| s.vel.length() > 5.0 || r.sideways);
            let (k, foot) = match p.rdr2 {
                Some(_) if flat || balancing || !ahead => (leg.knee.m.d, leg.foot.m.d),
                Some(_) => (predict_knee(&leg.hip, &leg.knee, side), target[i]),
                None => (predict_knee(&leg.hip, &leg.knee, side), target[i]),
            };
            // RDR2 (0x2aaeed8): the ankle balance lays the foot on the contact normal itself, not on the pelvis frame's turn of it
            // (with the single support pitch target that turn is 0.3 rad; recorded stance pitch flat - 0.12)
            let contact_n = if balancing { p.normal[i] } else { normal[i] };
            if let Some((_, tilt)) = aim {
                // RDR2 (0x2aaeed8): tilted only for a planted foot with the COM over the feet and no step on; otherwise
                // the balancing pitch is the flat one
                let n = if f.inside_raw && f.stepping == 0 { bevy_math::Quat::from_rotation_arc(Vec3::Z, tilt) * contact_n } else { contact_n };
                bal[i] = asin(sdot(leg.knee.m.c, vnorm(k - foot).vx(n), b"xyz"));
                let d = vnorm(k - foot);
                let tw = |n: Vec3| asin(vnorm(n.vx(leg.knee.m.c)).vd(d.vx(n)));
                twist_flat[i] = Some((tw(contact_n), tw(n)));
            }
            let n = if flat { contact_n.normalize_or_zero() } else { contact_n };
            let a = asin(sdot(leg.knee.m.c, vnorm(k - foot).vx(n), b"xyz"));
            // RDR2 (0x2aae0b4): the knee-to-foot aim is for the ankle balance, flatterStaticFeet and staggering; otherwise
            // (not a running foot in the air, which aims from the actual knee) the stance ankle's pitch is 0
            match p.rdr2 {
                Some(r) if !r.stagger && !flat && !balancing && !(r.running && !contact[i]) => 0.0,
                _ => if flip { -a } else { a },
            }
        };
        leg.ankle.desired[0] = a;
    }
    let over = |h: &Eff3| {
        let (l1, l2) = ((h.desired[0] - h.mid_lean1) / h.half_lean1, (h.desired[1] - h.mid_lean2) / h.half_lean2);
        let e = l2.fma(l2, l1 * l1).sqrt();
        (e - 1.0).max(0.0) / (e + delirium_data::kv!(50)) * (h.desired[1] - h.mid_lean2)
    };
    let (ol, or) = (over(&s.legs[0].hip), over(&s.legs[1].hip));
    // RDR2's ankle balance: the balancing pitch fades as the foot tilts off flat
    let w = [0, 1].map(|i| p.rdr2.map(|r| {
        let off = if r.p.ankle_balance { r.p.ankle_offset } else { 0.0 };
        r.ankle_weight(off + s.legs[i].ankle.actual[0] - s.legs[i].ankle.desired[0])
    }));
    // RDR2 balances with the ankles only below 1 m/s and not staggering; else the planted ankles lie flat
    let slow = s.vel.length() < 1.0 && !p.rdr2.is_some_and(|r| r.stagger);
    let rd = p.rdr2.filter(|r| r.p.ankle_balance && !r.p.ankle_keep_pelvis);
    let stance_lean = if p.rdr2.is_some_and(|r| r.stagger) { 0.0 } else { lean };
    if std::env::var_os("NMX_DBG").is_some() { eprintln!("ANK flat {:.3} {:.3} bal {:.3} {:.3} w {:?} aim {:?} act {:.3} {:.3} inside {} stepping {} lp {} rp {} slow {} lean {:.3} out1 {:.3} com {:?} centre {:?} lf {:?} rf {:?}", s.legs[0].ankle.desired[0], s.legs[1].ankle.desired[0], bal[0], bal[1], w, aim, s.legs[0].ankle.actual[0], s.legs[1].ankle.actual[0], f.inside_raw, f.stepping, lp, rp, slow, lean, pv.out[1], s.com, f.centre, s.legs[0].foot.m.d, s.legs[1].foot.m.d); }
    if let Some(r) = rd {
        for (i, sgn) in [(0, 1.0), (1, -1.0)] {
            // only this path asks a twist of the ankle (0x2aaeed8; recorded 0 otherwise)
            s.legs[i].ankle.desired[2] = 0.0;
            if let (true, Some((flat, tilted))) = (slow, twist_flat[i]) {
                let w = r.ankle_twist_weight(s.legs[i].ankle.actual[2] - flat);
                s.legs[i].ankle.desired[2] = sgn * (flat + r.p.ankle_twist_mult * aim.map_or(0.0, |a| a.0[i]) * w * (tilted - flat));
            }
        }
    }
    let ankle = |i: usize, flat: f32, w: f32| match (rd, aim) {
        (Some(r), Some((share, _))) => flat + r.p.ankle_mult * share[i] * w * (bal[i] - flat) + r.p.ankle_offset,
        _ => flat,
    };
    if lp {
        s.legs[0].ankle.desired[0] = match w[0] {
            // RDR2's lean clamp (0.2) goes on every planted ankle, off the staggering
            Some(w) if rd.is_some() => stance_lean + if slow { ankle(0, s.legs[0].ankle.desired[0], w) } else { s.legs[0].ankle.desired[0] },
            Some(w) => (pv.out[1] + lean).mul_add(w, s.legs[0].ankle.desired[0]),
            None => pv.out[1] + s.legs[0].ankle.desired[0] + lean,
        };
    }
    let twist = |i: usize, planted: bool| match p.rdr2.filter(|r| r.turning) {
        Some(r) => pv.yaw[i].clamp(r.leg_twist.0, r.leg_twist.1) * crate::nmdbm_rdr2::TURN_ANKLE[!planted as usize],
        None => pv.yaw[i] * 0.5,
    };
    s.legs[0].ankle.desired[1] = twist(0, lp);
    if rp {
        s.legs[1].ankle.desired[0] = match w[1] {
            Some(w) if rd.is_some() => (if slow { ankle(1, s.legs[1].ankle.desired[0], w) } else { s.legs[1].ankle.desired[0] }) - stance_lean,
            Some(w) => s.legs[1].ankle.desired[0] + (pv.out[1] - lean) * w,
            None => s.legs[1].ankle.desired[0] + (pv.out[1] - lean),
        };
    }
    s.legs[1].ankle.desired[1] = twist(1, rp);
    // RDR2 (0x2aaeed8): while the foot that gave way waits, the stepping leg's hip and, unless staggering forward, its ankle
    // are held up
    if let Some(r) = p.rdr2 {
        let (_, _, hip, ankle) = crate::nmdbm_rdr2::GIVE_WAY;
        let forward = horizontal(s.vel, up).normalize_or_zero().dot(horizontal(up.cross(-s.pelvis.m.b), up).normalize_or_zero()) > 0.2;
        for i in 0..2 {
            if f.stepping == i as u32 + 1 && r.wait[1 - i] > 0.0 {
                s.legs[i].hip.desired[0] -= r.wait[1 - i] * hip;
                if !(r.stagger && forward) {
                    s.legs[i].ankle.desired[0] += r.wait[1 - i] * ankle;
                }
            }
        }
    }
    let (a, b) = (ol.max(-or), or.max(-ol));
    if a > 0.0 && b <= 0.0 {
        s.legs[1].hip.desired[1] += a;
        if lp {
            s.legs[0].hip.desired[1] -= a;
        }
        s.spine.desired[1] -= a;
    }
    if b > 0.0 && a <= 0.0 {
        s.legs[0].hip.desired[1] += b;
        if rp {
            s.legs[1].hip.desired[1] -= b;
        }
        s.spine.desired[1] -= b;
    }
}

fn m34(q: Quat, d: Vec3) -> M34 {
    M34 { a: q * Vec3::X, b: q * Vec3::Y, c: q * Vec3::Z, d }
}

pub fn own_rage(doll: &Ragdoll) -> Option<(Vec<crate::rigid::Link>, Vec3, Vec<M34>)> {
    let frames: Vec<Transform> = doll.frames().collect();
    if frames.len() != HOMER_PARTS() {
        return None;
    }
    let links = (0..frames.len()).map(|k| {
        let (v, w) = doll.motion(k);
        crate::rigid::Link { r: doll.link_rows(k), d: doll.link_d(k).0, m: doll.part_mass(k), v, w }
    }).collect();
    Some((links, doll.link_d(0).1, doll.parts()))
}

pub struct Body<'a> {
    pub ch: &'a Character,
    pub build: &'a Build,
    pub doll: &'a Ragdoll,
    pub frames: &'a [Transform],
}

impl Body<'_> {
    fn rigid(&self, j: usize) -> Option<(&crate::rigid::JointConst, &crate::rigid::Link, &crate::rigid::Link, Vec3)> {
        let (jd, (l, place, _)) = (&self.ch.joints[j], self.doll.rigid.as_ref()?);
        Some((jd.rigid.as_ref()?, l.get(jd.parent)?, l.get(jd.child)?, *place))
    }

    pub fn read(&self, j: usize) -> [f32; 3] {
        let jd = &self.ch.joints[j];
        let tw = self.doll.prep_twist.get(jd.child).copied().flatten();
        let Some((c, p, l, _)) = self.rigid(j) else { return self.ch.angle_at(j, self.frames, tw) };
        if !jd.three {
            return [crate::rigid::hinge(c, p, l), 0.0, 0.0];
        }
        let (l1, l2) = crate::rigid::leans(c, p, l);
        let tw = tw.unwrap_or_else(|| crate::rigid::twist(c, p, l));
        let act = |k: usize, raw: f32| {
            let v = (jd.limits[k].1 + jd.limits[k].0).fma(0.5, raw);
            if jd.flip[k] { fneg(v) } else { v }
        };
        [act(0, l1), act(1, l2), act(2, tw)]
    }

    pub fn joint_pos(&self, j: usize) -> Vec3 {
        if let Some((c, _, l, place)) = self.rigid(j) {
            return crate::rigid::pos(c, l, place);
        }
        let c = self.ch.joints[j].child;
        self.frames[c].translation + self.frames[c].rotation * self.build.parts[c].anchor
    }

    pub fn part(&self, k: usize) -> Part {
        let (v, w) = self.doll.motion(k);
        if let Some(m) = self.doll.rigid.as_ref().filter(|_| k == 0).and_then(|(_, _, p)| p.get(k)) {
            return Part { m: *m, v, w };
        }
        if let Some((l, place)) = self.doll.rigid.as_ref().and_then(|(l, p, _)| Some((l.get(k)?, *p))) {
            let col = |i: usize| Vec3::new(l.r[0][i], l.r[1][i], l.r[2][i]);
            return Part { m: M34 { a: col(0), b: col(1), c: col(2), d: l.d + place }, v, w };
        }
        let link = if self.build.parts.len() == HOMER_PARTS() { link(&self.build.links, k) } else { Quat::IDENTITY };
        Part { m: m34(self.frames[k].rotation * link, self.frames[k].translation), v, w }
    }

    pub fn pmat(&self, k: usize) -> Part {
        match self.doll.rigid.as_ref().and_then(|(_, _, p)| p.get(k)) {
            Some(m) => Part { m: *m, ..self.part(k) },
            None => self.part(k),
        }
    }

    pub(crate) fn eff3(&self, j: usize) -> Eff3 {
        let (jd, e) = (&self.ch.joints[j], &self.ch.eff[j]);
        let mid = |k: usize| (jd.limits[k].0 + jd.limits[k].1) * 0.5;
        let half = |k: usize| (jd.limits[k].1 - jd.limits[k].0) * 0.5;
        let flip = |k: usize, x: f32| if jd.flip[k] { -x } else { x };
        let pos = self.joint_pos(j);
        let m = self.rigid(j).map_or_else(|| m34(self.frames[jd.parent].rotation * jd.rage_parent, pos), |(c, p, l, place)| crate::rigid::frames(c, p, l, place).0);
        Eff3 { m, pos, damping: e.damping, strength: e.strength, half_twist: half(2), half_lean1: half(0),
               half_lean2: half(1), mid_lean1: flip(0, mid(0)), mid_lean2: flip(1, mid(1)), mid_twist: flip(2, mid(2)),
               desired: e.desired, actual: self.ch.actual(j, self.frames) }
    }

    pub(crate) fn eff1(&self, j: usize) -> Eff1 {
        let (jd, e) = (&self.ch.joints[j], &self.ch.eff[j]);
        let pos = self.joint_pos(j);
        let (m, child) = self.rigid(j).map_or_else(|| (m34(self.frames[jd.parent].rotation * jd.in_parent, pos), m34(self.frames[jd.child].rotation * jd.in_child, pos)),
                                                  |(c, p, l, place)| crate::rigid::frames(c, p, l, place));
        Eff1 { m, child, pos, damping: e.damping, strength: e.strength, half: (jd.limits[0].1 - jd.limits[0].0) * 0.5,
               mid: (jd.limits[0].0 + jd.limits[0].1) * 0.5, desired: e.desired[0], actual: self.ch.actual(j, self.frames)[0] }
    }

    pub(crate) fn limb(&self, hip: usize, knee: usize) -> M34 {
        let i = (*crate::nmrs::LIMBS()).iter().position(|&l| l == (hip, knee));
        if let Some(m) = i.and_then(|i| self.ch.limb[i]) {
            return m;
        }
        if let (Some(k), Some((c, p, l, place))) = (self.rigid(knee), self.rigid(hip)) {
            let (k, h) = (crate::rigid::frames(k.0, k.1, k.2, k.3).0, crate::rigid::frames(c, p, l, place).1);
            let t = |v: Vec3| Vec3::new(v.vd(h.a), v.vd(h.b), v.vd(h.c));
            return M34 { a: t(k.a), b: t(k.b), c: t(k.c), d: t(k.d - h.d) };
        }
        let (h, k) = (&self.ch.joints[hip], &self.ch.joints[knee]);
        let fc = h.rage_child.inverse();
        let p = &self.build.parts;
        m34(fc * k.in_parent, fc * (p[k.child].anchor_parent - p[h.child].anchor))
    }

    pub fn at(&self, k: usize) -> Vec3 {
        match self.doll.rigid.as_ref().and_then(|(l, place, _)| Some(l.get(k)?.d + *place)) {
            Some(d) => d,
            None => self.part(k).m.d,
        }
    }

    fn zero_place(&self, k: usize) -> Option<Vec3> {
        match &self.ch.refs {
            Some(r) => r.get(k).map(|m| m[delirium_data::kv!(0 int)]),
            None => crate::nmrs::zero_places(self.build).get(k).copied(),
        }
    }

    fn foot(&self, i: usize) -> Vec3 {
        self.part(self.ch.joints[(*ANKLE())[i]].child).m.d
    }

    pub fn block(&self) -> Block {
        let refs = crate::nmrs::PARTS[self.build.nm].get().filter(|r| r.len() == self.build.parts.len() && self.build.parts.len() == HOMER_PARTS());
        if let (Some((l, place, _)), Some(r)) = (self.doll.rigid.as_ref(), refs) {
            let refs: Vec<[Vec3; 3]> = match &self.ch.refs {
                Some(g) => g.iter().map(|m| [m[0], m[1], m[2]]).collect(),
                None => r.iter().enumerate().map(|(k, m)| {
                    let m = m.map(Vec3::from);
                    if k == 0 { [m[2], m[1], -m[0]] } else { [m[0], m[1], m[2]] }
                }).collect(),
            };
            let (m, vel, angvel) = crate::rigid::block(l, &refs, *place, true, self.ch.attached.map(|(_, p, m)| (p, m)));
            return Block { com: m.d, vel, angvel, m };
        }
        let (com, vel) = self.doll.com();
        let refs = crate::nmrs::PARTS[self.build.nm].get().filter(|r| r.len() == self.build.parts.len() && self.build.parts.len() == HOMER_PARTS());
        let rows = |m: &[[f32; 3]; 4]| Mat3::from_cols(Vec3::from(m[0]), Vec3::from(m[1]), Vec3::from(m[2])).transpose();
        let (mut sum, mut w, mut mass) = (Mat3::ZERO, Vec3::ZERO, 0.0);
        for (k, p) in self.build.parts.iter().enumerate() {
            let m = self.doll.part_mass(k);
            let turn = match refs {
                Some(r) => {
                    let mut re = rows(&r[k]);
                    if k == 0 {
                        re = Mat3::from_cols(Vec3::new(0.0, 0.0, -1.0), Vec3::Y, Vec3::X) * re;
                    }
                    Mat3::from_quat(self.frames[k].rotation * rage_link(&self.build.links, k)) * re.inverse()
                }
                None => Mat3::from_quat(self.frames[k].rotation * p.bind.inverse()) * Mat3::from_rotation_z(std::f32::consts::PI),
            };
            sum += turn * m;
            w += self.doll.motion(k).1 * m;
            mass += m;
        }
        let t = sum.transpose();
        let o = Mat3::from_cols(t.x_axis.normalize(), t.y_axis.normalize(), t.z_axis.normalize());
        let c = -o.row(1).normalize();
        let a = o.row(2).vx(c).normalize();
        Block { com, vel, angvel: w / mass.max(delirium_data::kv!(50)), m: M34 { a, b: c.vx(a), c, d: com } }
    }

    pub fn snap(&self, up: Vec3) -> Snap {
        let Block { com, vel, angvel, m } = self.ch.block.unwrap_or_else(|| self.block());
        let mut s = Snap { com, vel, hvel: shz(vel, up, b"yzx"), angvel, m, ..default() };
        for i in 0..2 {
            let leg = &mut s.legs[i];
            leg.hip = self.eff3((*HIP())[i]);
            leg.thigh = self.part(self.ch.joints[(*HIP())[i]].child);
            leg.knee = self.eff1((*KNEE())[i]);
            leg.shin = self.part(self.ch.joints[(*KNEE())[i]].child);
            leg.ankle = self.eff3((*ANKLE())[i]);
            leg.foot = self.part(self.ch.joints[(*ANKLE())[i]].child);
            leg.limb = self.limb((*HIP())[i], (*KNEE())[i]);
        }
        s.spine = self.eff3((*SPINE())[0]);
        s.pelvis = self.part(self.ch.joints[(*HIP())[0]].parent);
        let t = |i: usize| self.doll.touch[self.ch.joints[(*ANKLE())[i]].child];
        let (l, r) = (t(0), t(1));
        s.flags = (l & delirium_data::kv!(7 int)) << 2 | (l & delirium_data::kv!(1 int)) << delirium_data::kv!(1 int) | (r & delirium_data::kv!(7 int)) | (r & delirium_data::kv!(1 int)) << 2;
        s
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub step_height: f32,
    pub leg_stiffness: f32,
    pub abort: f32,
    pub step_clamp: f32,
    pub max_steps: i32,
    pub taper: bool,
    pub lean_bias: f32,
    pub hip_yaw: (f32, f32),
    pub legs: (f32, f32),
}

#[derive(Clone, Copy, Debug)]
pub struct Dbm {
    pub p: Params,
    pub f: Feet,
    pub pv: Pelvis,
    pub acc: f32,
    pub leg_offset: f32,
    pub knee_scale: f32,
    /// RDR2 core: the leg stiffness per leg, left and right, when asked (bal+0x658, +0x65c)
    pub leg_each: Option<[f32; 2]>,
    pub lean: Lean,
    lean_vec: Vec3,
    as_force: bool,
    pub push: Vec3,
    ratio: f32,
    pub event: Option<f32>,
    pub success: bool,
    pub held: bool,
    pub last: (Vec3, Vec3),
    pub down: bool,
    /// RDR2 core: the hip lean asked this step (hipsLean*)
    pub hip_lean: Vec3,
}

#[derive(Clone, Copy, Debug, Default)]
pub enum LeanMode {
    #[default]
    None,
    Dir(Vec3),
    Pos(Vec3),
    Object(Vec3, i32),
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Lean {
    pub mode: LeanMode,
    pub amount: f32,
    pub as_force: bool,
}

impl Lean {
    pub fn to_position(pos: Vec3, amount: f32, as_force: bool) -> Lean {
        Lean { mode: LeanMode::Pos(pos), amount, as_force }
    }

    pub fn towards_object(instance: i32, offset: Vec3, amount: f32, as_force: bool) -> Lean {
        Lean { mode: LeanMode::Object(offset, instance), amount, as_force }
    }
}

#[allow(non_snake_case)]
#[inline]
pub fn CHAR_LEGS() -> (f32, f32) { (delirium_data::kv!(264), delirium_data::kv!(265)) }
#[allow(non_snake_case)]
#[inline]
pub fn CHAR_YAW() -> (f32, f32) { (delirium_data::kv!(266), delirium_data::kv!(267)) }
#[allow(non_snake_case)]
#[inline]
pub fn MARGE_LEGS() -> (f32, f32) { (delirium_data::kv!(268), delirium_data::kv!(269)) }
#[allow(non_snake_case)]
#[inline]
pub fn MARGE_YAW() -> (f32, f32) { (-delirium_data::kv!(270), -delirium_data::kv!(271)) }
/// RDR2's own stance numbers for its male ped (its NM context's ctx+0x34d0..+0x34e0: leg straightness and minimum
/// foot separation, the legs' splay and turn, the head's turn), as the game set them from the townsman's standing pose
/// in the reference recordings (other peds on the same body: 0.982-0.987, 0.286-0.293, 0.178). Not computed from the
/// body: RDR2 sets them per ped from its pose, as GTA IV's are Homer's (CHAR_LEGS, CHAR_YAW, CHAR_HEAD).
pub const RDR2_MALE_STANCE: ((f32, f32), (f32, f32), f32) = ((0.9849, 0.2868), (0.1779, -0.0154), 0.0904);
#[allow(non_snake_case)]
#[inline]
pub fn CHAR_HEAD() -> f32 { delirium_data::kv!(272) }
#[allow(non_snake_case)]
#[inline]
pub fn MARGE_HEAD() -> f32 { delirium_data::kv!(273) }

pub(crate) fn stance(up: Vec3, pelvis: &M34, l: &M34, r: &M34, head: &M34, zero: [Vec3; 3]) -> ((f32, f32), (f32, f32), f32) {
    let d = r.d - l.d;
    let width = slen(d - up * sdot(up, d, b"zyx"), b"zyx") * delirium_data::kv!(18);
    let height = |p: Vec3, l: Vec3, r: Vec3| sdot(up, p - (l + r) * 0.5, b"zyx");
    let leg = (height(pelvis.d, l.d, r.d) / height(zero[0], zero[1], zero[2])).min(1.0);
    let splay = asin(sdot(up, r.c.vx(l.c), b"zyx")) * delirium_data::kv!(17);
    let feet = vnorm(l.c + r.c);
    let turn = asin(sdot(up, feet.vx(pelvis.c), b"zyx")) * 0.5;
    ((leg, width), (splay, turn), asin(sdot(up, feet.vx(head.c), b"zyx")))
}

pub(crate) fn zero_stance(ch: &Character, build: &Build, frames: &[Transform]) -> Option<((f32, f32), (f32, f32), f32)> {
    zero_stance_rows(ch, build, &|k: usize| {
        let link = if build.parts.len() == HOMER_PARTS() { link(&build.links, k) } else { Quat::IDENTITY };
        m34(frames[k].rotation * link, frames[k].translation)
    })
}

pub(crate) fn zero_stance_rows(ch: &Character, build: &Build, row: &dyn Fn(usize) -> M34) -> Option<((f32, f32), (f32, f32), f32)> {
    let (pelvis, l, r) = (ch.joints[(*HIP())[0]].parent, ch.joints[(*ANKLE())[0]].child, ch.joints[(*ANKLE())[1]].child);
    let places = crate::nmrs::zero_places(build);
    let zero = |k: usize| ch.refs.as_ref().map_or(places.get(k).copied(), |r| r.get(k).map(|m| m[delirium_data::kv!(0 int)]));
    Some(stance(ch.up, &row(pelvis), &row(l), &row(r), &row(delirium_data::kv!(55 int)), [zero(pelvis)?, zero(l)?, zero(r)?]))
}

impl Dbm {
    pub fn new(b: &Body, c: Config) -> Dbm {
        let up = Vec3::Z;
        let (l, r) = (b.foot(0), b.foot(1));
        let pelvis = b.at(b.ch.joints[(*HIP())[0]].parent);
        let (zero, foot) = (|k: usize| b.zero_place(k), (*ANKLE()).map(|j| b.ch.joints[j].child));
        let cu = b.ch.up;
        let p = Params {
            up, gup: up, dt: 1.0 / delirium_data::kv!(103),
            pelvis_height: match zero(0).zip(zero(foot[0])).zip(zero(foot[1])) {
                Some(((p, l), r)) => sdot(up, p - (l + r) * 0.5, b"zyx"),
                None => up.vd(pelvis - (l + r) * 0.5),
            },
            hip_dist: slen((b.eff3((*HIP())[0]).m.d - pelvis).vx(cu), b"zyx"),
            leg_length: c.legs.0, foot_width: c.legs.1, p148: c.hip_yaw.0, p152: c.hip_yaw.1,
            step_offset: c.legs.1.fma(delirium_data::kv!(24), delirium_data::kv!(130)), abort: c.abort, lead: delirium_data::kv!(60), p168: 1.0, p172: 0.0, lean_bias: c.lean_bias,
            step_height: c.step_height, leg_stiffness: c.leg_stiffness, step_clamp: step_clamp(b, c), taper: c.taper,
            low_abort: c.taper, ..default()
        };
        let f = Feet { pos: [l, r], centre: (l + r) * 0.5, height: [up.vd(l), up.vd(r)], countdown: c.max_steps, flags: delirium_data::kv!(2 int), ..default() };
        let (fl, fr) = (b.pmat(b.ch.joints[(*ANKLE())[0]].child).m, b.pmat(b.ch.joints[(*ANKLE())[1]].child).m);
        let pm = b.pmat(b.ch.joints[(*HIP())[0]].parent).m;
        let splay = asin(sdot(cu, fr.c.vx(fl.c), b"zyx")) * delirium_data::kv!(17);
        let turn = asin(sdot(cu, vnorm(fl.c + fr.c).vx(pm.c), b"zyx")) * 0.5;
        if std::env::var("NM_YAWINIT").is_ok() {
            for k in 0..b.ch.joints.len().min(delirium_data::kv!(111 int)) { let m = b.part(k).m; let t = up.vd((fl.c + fr.c).normalize_or_zero().vx(m.c)).clamp(-1.0, 1.0).asin() * 0.5; eprintln!("YAWINIT part {k} turn {t:.4} -> [{:.4}, {:.4}]", t - splay, -t - splay); }
            eprintln!("YAWINIT splay {splay:.4} turn {turn:.4}");
        }
        let pv = Pelvis { centre: f.centre, p52: p.lean_bias, p68: delirium_data::kv!(26), yaw: [turn - splay, -turn - splay], ..default() };
        let mut d = Dbm { p, f, pv, acc: 0.0, leg_offset: 0.0, knee_scale: 1.0, leg_each: None, lean: Lean::default(), lean_vec: Vec3::ZERO, as_force: false, push: Vec3::ZERO, ratio: 0.0, event: None, success: false, held: false, last: (Vec3::Z, Vec3::ZERO), hip_lean: Vec3::ZERO,
            down: (0..2).any(|i| b.doll.touch[b.ch.joints[(*ANKLE())[i]].child] & delirium_data::kv!(7 int) != 0) };
        d.held = d.holding(&b.snap(up), l, r, None);
        d
    }

    /// Switches this balancer to the RDR2 core's (nmdbm_rdr2.rs). Its own step limit replaces GTA IV's countdown.
    pub fn rdr2(&mut self, r: crate::nmdbm_rdr2::Rdr2) {
        // the pitch integrator around ankleEquilibrium, not GTA IV's standing lean bias
        (self.p.lean_bias, self.pv.p52) = (r.p.ankle_equilibrium, r.p.ankle_equilibrium);
        self.p.rdr2 = Some(r);
        self.f.countdown = i32::MAX;
    }

    pub fn restart_over(&mut self, old: &Dbm) {
        (self.pv.off, self.pv.out, self.lean_vec, self.as_force) = (old.pv.off, old.pv.out, old.lean_vec, old.as_force);
    }

    fn tilt(&mut self, b: &Body) {
        let z = Vec3::Z;
        (self.p.up, self.push) = (z, Vec3::ZERO);
        let now = (0..2).any(|i| b.doll.touch[b.ch.joints[(*ANKLE())[i]].child] & delirium_data::kv!(7 int) != 0);
        if !std::mem::replace(&mut self.down, now) {
            return;
        }
        let u = vnorm(z + shz(self.lean_vec, z, b"yzx"));
        if !self.as_force {
            self.p.up = u;
        }
    }

    fn lean_force(&self, mass: f32) -> Vec3 {
        if !self.down || !self.as_force {
            return Vec3::ZERO;
        }
        let z = Vec3::Z;
        let u = vnorm(z + shz(self.lean_vec, z, b"yzx"));
        let g = -z * crate::ragdoll::GRAVITY() * mass;
        horizontal(g - u * sdot(u, g, b"zyx"), z)
    }

    fn lean_vector(&mut self, com: Vec3) -> Vec3 {
        let l = &mut self.lean;
        if !matches!(l.mode, LeanMode::None) {
            self.as_force = l.as_force;
        }
        let to = match l.mode {
            LeanMode::None => return Vec3::ZERO,
            LeanMode::Dir(d) => return d * l.amount,
            LeanMode::Pos(p) | LeanMode::Object(p, _) => p - com,
        };
        if sdot(to, to, b"yxz") > delirium_data::kv!(78) {
            vnorm(to) * l.amount
        } else {
            l.mode = LeanMode::None;
            to
        }
    }

    fn holding(&mut self, s: &Snap, l: Vec3, r: Vec3, ankles: Option<(Vec3, Vec3)>) -> bool {
        let up = self.p.up;
        let h = up.vd(s.com) - up.vd(l).min(up.vd(r));
        let off = horizontal(s.com - l, up).length().min(horizontal(s.com - r, up).length())
            .min(horizontal(s.com - (l + r) * 0.5, up).length());
        (self.event, self.success) = (None, false);
        if let Some(r) = self.p.rdr2.as_mut() {
            r.fail_ratio.0 = off / h.max(0.001);
        }
        if off > self.p.rdr2.and_then(|r| r.abort).unwrap_or(self.p.abort) * h || (h < 0.5 && self.p.low_abort) || self.f.countdown == -1 {
            if let Some(r) = self.p.rdr2 {
                crate::extras_core_rdr2::trace(|| format!("balancer: fail test: {off:.2} off over {h:.2} high at {:.2} s", r.time));
            }
            return false;
        }
        if let (Some(r), Some((al, ar))) = (self.p.rdr2, ankles) {
            // RDR2: the feet give out at giveupHeight whatever the knees do, and the ankles' own support test
            let ha = up.vd(s.com) - up.vd(al).min(up.vd(ar));
            let offa = horizontal(s.com - al, up).length().min(horizontal(s.com - ar, up).length())
                .min(horizontal(s.com - (al + ar) * 0.5, up).length());
            if let Some(q) = self.p.rdr2.as_mut() {
                q.fail_ratio.1 = offa / ha.max(0.001);
            }
            if h < r.giveup_height() || crate::nmdbm_rdr2::ANKLE_ABORT * ha < offa {
                crate::extras_core_rdr2::trace(|| format!("balancer: fail test: height {h:.2} under {:.2}, or ankles {ha:.2}, {offa:.2}, at {:.2} s", r.giveup_height(), r.time));
                return false;
            }
            // RDR2's stagger mode: the COM too low over the higher foot (staggerFailHeight x leg length, the leg length
            // taken as his standing pelvis height)
            // staggerFailHeight: the larger of the COM's heights over the two contacts (0x2abda8c)
            let hi = up.vd(s.com) - up.vd(self.p.target[0]).min(up.vd(self.p.target[1]));
            if r.stagger && hi < r.p.stagger_fail_height * self.p.pelvis_height {
                crate::extras_core_rdr2::trace(|| format!("balancer: fail test: stagger height {hi:.2} at {:.2} s", r.time));
                return false;
            }
        }
        let (lin, rot) = self.p.rdr2.map_or((delirium_data::kv!(17), delirium_data::kv!(17)), |r| (r.p.stable_lin, r.p.stable_rot));
        self.success = slen(s.vel, b"yxz") < lin && slen(s.angvel, b"yxz") < rot
            && !self.p.rdr2.is_some_and(|r| r.kneeling);
        let x = off / (h * delirium_data::kv!(142));
        self.ratio = if x > self.ratio { x } else { (x - self.ratio) * delirium_data::kv!(17) + self.ratio };
        if self.ratio < 1.0 && h > delirium_data::kv!(74) && self.f.countdown > 2 {
            self.event = Some(self.ratio);
        }
        true
    }

    fn probe(&mut self, b: &Body, world: &Collision) {
        let up = self.p.up;
        for i in 0..2 {
            let start = sat(self.f.pos[i], up, sdot(up, b.joint_pos((*KNEE())[i]), b"zyx"), b"zyx");
            let old = self.p.target[i];
            let hit = world.segment(start, start - up);
            self.p.contact[i] = hit.is_some();
            (self.p.target[i], self.p.normal[i]) = match hit {
                // RDR2: the contact half the ankle's height above the hit
                Some((at, n)) => (at + up * if self.p.rdr2.is_some() { 0.5 * crate::nmdbm_rdr2::FOOT_HEIGHT } else { delirium_data::kv!(131) }, n),
                None => (start - up * (self.p.leg_length * self.p.leg_scale()), up),
            };
            let y = crate::estimate::vrefp(self.p.dt);
            self.p.gvel[i] = (self.p.target[i] - old) * (-y).mul_add(self.p.dt, 1.0).mul_add(y, y);
        }
    }

    pub fn tick(&mut self, b: &Body, world: &Collision, c: Config, dt: f32) -> Option<Snap> {
        self.p.dt = dt;
        self.p.step_clamp = step_clamp(b, c);
        self.p.abort = c.abort;
        self.p.step_height = c.step_height;
        self.tilt(b);
        let mut s = b.snap(self.p.up);
        self.last = (s.m.b, s.angvel);
        // RDR2 (dbm stage 1, under bal+0x468, the first tick after a reset): the legs' twist states start from the hips'
        // actual twists off their zero (joint+0x19c, the middle of the twist limits)
        if self.p.rdr2.as_mut().is_some_and(|r| std::mem::take(&mut r.first_tick)) {
            for i in 0..2 {
                self.pv.yaw[i] = s.legs[i].hip.actual[2] - s.legs[i].hip.mid_twist;
            }
        }
        let ankles = self.p.rdr2.map(|_| (b.joint_pos((*ANKLE())[0]), b.joint_pos((*ANKLE())[1])));
        let holds = self.holding(&s, b.foot(0), b.foot(1), ankles);
        // RDR2: fall to knees' fail now; failMustCollide: a failing test waits for a part off the legs to touch something
        if self.p.rdr2.as_mut().is_some_and(|r| std::mem::take(&mut r.force_fail)) {
            return None;
        }
        // RDR2: ignoreFailure, the fail tests do nothing
        if !holds && !self.p.rdr2.is_some_and(|r| r.p.ignore_failure || ((r.must_collide || r.p.fail_must_collide) && !(0..b.doll.hit_out.len()).any(|k| !(1..=6).contains(&k) && b.doll.hit_out[k].is_some()))) {
            return None;
        }
        if self.p.rdr2.is_some() {
            let gup = self.p.gup;
            // the backward lean starts the timed fail: the spine's top against the pelvis
            let top = b.part(10).m;
            if let Some(r) = self.p.rdr2.as_mut() {
                r.back_lean(gup, &s.pelvis.m, &top);
            }
            let feet = [0, 1].map(|i| {
                let p = b.part(b.ch.joints[(*ANKLE())[i]].child);
                (p.m.d, p.v)
            });
            if let Some(r) = self.p.rdr2.as_mut() {
                r.measure(gup, s.com, s.vel, feet);
            }
        }
        self.pv.centre = self.f.centre;
        self.pv.planted = [self.f.flags & delirium_data::kv!(251 int) != 0, self.f.flags & delirium_data::kv!(253 int) != 0];
        self.p.leg_stiffness = c.leg_stiffness;
        self.p.leg_length = match self.p.rdr2 {
            // RDR2: the crouch request's knee bend instead of the extras' offset; standUp straightens him while moving
            Some(r) => {
                let up = if r.p.stand_up { (2.0 * slen(s.hvel, b"zyx")).min(1.0) * (1.0 - c.legs.0) } else { 0.0 };
                // running ignores the bend unless fall to knees (owner 12) asks it
                if r.running && r.owned.knee.owner != 12 { c.legs.0 } else { c.legs.0 + r.owned.knee.value + up }
            }
            None => c.legs.0 + self.leg_offset,
        };
        self.pv.acc = self.acc;
        self.lean_vec = self.lean_vector(s.com);
        self.p.hip_up = Vec3::ZERO;
        if self.p.rdr2.is_some() && self.down && self.hip_lean != Vec3::ZERO {
            let (u, l) = (self.p.up, self.hip_lean);
            self.p.hip_up = vnorm(u + l - u * l.dot(u));
        }
        let leg_scale = self.p.leg_scale();
        if let Some(r) = self.p.rdr2.as_mut() {
            r.straight = (c.legs.0, leg_scale);
            let low = up_low(self.p.up, b.foot(0), b.foot(1));
            let hip = (s.legs[0].hip.actual[0] + s.legs[1].hip.actual[0]) * 0.5;
            let (gup, legs) = (self.p.gup, &s.legs);
            r.crouch_start(self.p.up.vd(s.com) - low, leg_scale, c.legs.0, hip, || {
                use crate::nmdbm_rdr2::Rdr2;
                (Rdr2::legs_lying(gup, [0, 1].map(|i| (legs[i].knee.m.d, legs[i].ankle.m.d))),
                 Rdr2::turn_out(s.pelvis.m.a, s.pelvis.m.b, [0, 1].map(|i| (legs[i].thigh.m.b, legs[i].thigh.m.c))))
            });
        }
        if let Some(r) = self.p.rdr2 {
            let (lean, more) = r.lean_against(horizontal(s.hvel, self.p.up));
            self.lean_vec += lean;
            if let Some(t) = r.lead {
                self.p.lead = t + more + r.lead_add;
            } else if r.own_prediction() || r.lead_add != 0.0 {
                self.p.lead = r.prediction + more + r.lead_add;
            }
            self.p.step_clamp += r.clamp_add;
        }
        self.push = self.lean_force(b.doll.mass());
        self.probe(b, world);
        if let Ok(path) = std::env::var("NM_DBMDUMP") {
            use std::io::Write;
            let mut f = std::fs::OpenOptions::new().create(true).append(true).open(path).unwrap();
            writeln!(f, "{:?}|{:?}|{:?}|{:?}", self.p, s, self.f, self.pv).unwrap();
        }
        let was = self.f.stepping;
        if let Some(r) = self.p.rdr2.as_mut() {
            let up = self.p.up;
            r.up_now = up;
            let i = match was { 1 => 1, 2 => 0, _ => (up.vd(self.p.target[1]) > up.vd(self.p.target[0])) as usize };
            r.slope(self.p.normal[i].z);
            r.speed = s.hvel.length();
            let facing = horizontal(-s.pelvis.m.c, up).normalize_or_zero().dot(horizontal(self.pv.vel, up).normalize_or_zero());
            r.turn(self.p.dt, self.pv.fixed_vel && !r.heading_only, facing);
            // bal+0x339 is staggering or fast; only fast takes the run values (0x2aba1cc)
            r.run_fast = r.speed >= r.p.run_vel;
            r.running = r.stagger || r.run_fast;
            if r.run_fast {
                (self.p.leg_stiffness, self.p.step_height) = (r.p.leg_stiffness_run, r.p.step_height_run);
            }
            // bal+0x334 from the legs' meeting (knees ctx+0x34b0, thighs +0x34b1), the timer of the stepping foot loaded with it.
            // Staggering it also needs staggerFall's bal+0xbe3 (?b49ec3c6: on by default and no set read sends it).
            r.wait = r.wait.map(|t| t - self.p.dt);
            let met = |a: usize, c: usize| b.doll.self_met.contains(&(a.min(c), a.max(c)));
            let knees = met(b.ch.joints[(*KNEE())[0]].child, b.ch.joints[(*KNEE())[1]].child);
            let thighs = met(b.ch.joints[(*HIP())[0]].child, b.ch.joints[(*HIP())[1]].child);
            let along = horizontal(s.vel, up).normalize_or_zero().dot(horizontal(up.cross(-s.pelvis.m.b), up).normalize_or_zero()).abs();
            r.give_way = if !r.stagger || along <= crate::nmdbm_rdr2::GIVE_WAY_ALONG { knees || thighs } else { knees };
            if (r.running && (!r.stagger || s.vel.length() >= crate::nmdbm_rdr2::GIVE_WAY_SPEED)) || r.step_age <= crate::nmdbm_rdr2::GIVE_WAY.0 {
                r.give_way = false;
            }
            if r.give_way && matches!(self.f.stepping, 1 | 2) {
                r.wait[self.f.stepping as usize - 1] = crate::nmdbm_rdr2::GIVE_WAY.0;
            }
            self.p.taper = if r.run_fast { r.p.taper_knee_strength_run } else { c.taper } && !r.on_slope && r.taper;
        }
        if let (Some(r), i @ 1..=2) = (self.p.rdr2, was) {
            let (i, up) = (i as usize - 1, self.p.up);
            self.p.step_height += r.step_up(up.vd(self.p.target[i]), up.vd(self.p.target[1 - i]));
        }
        if let Some(r) = self.p.rdr2.as_mut().filter(|r| r.cancel_step) {
            r.cancel_step = false;
            self.f.stepping = 0;
        }
        // RDR2 (0x2aba1cc): moving along the pelvis's side axis (minus its second row) is read before the placement, stepping or not
        if let Some(r) = self.p.rdr2.as_mut() {
            let v = horizontal(s.hvel, self.p.up);
            r.sideways(v.length_squared(), v.normalize_or_zero().dot(horizontal(-s.pelvis.m.b, self.p.up).normalize_or_zero()));
        }
        foot_placement(&self.p, &s, &mut self.f);
        if let Some(r) = self.p.rdr2.as_mut() {
            // a step landing ends the turn step for the rest of the tick
            if was != 0 && self.f.stepping == 0 {
                (r.landed, r.turning) = (true, false);
            }
            if let i @ 1..=2 = self.f.stepping {
                r.last_foot = Some(i as usize - 1);
            }
            let v = horizontal(s.hvel, self.p.up);
            // RDR2's sideways force (stage 3, 0x2abc4ec): on one foot, the heading error to his way (the way he faces below
            // useComDirTurnVelThresh) times sidewaysCheatForce, along the stance foot to the spine's top; on the left
            // foot only moving backward, on the right only not. Both or neither: 1 N up.
            if r.sideways && !r.turning && r.p.sideways_cheat_force >= 0.01 {
                crate::extras_core_rdr2::trace_once("sideways force", || format!("{} on {:?}", r.p.sideways_cheat_force, self.pv.planted));
                let up = self.p.up;
                let dir = if s.vel.length() < r.p.use_com_dir_turn_vel_thresh { horizontal(-s.m.c, up) } else { s.vel };
                let back = sdot(v.normalize_or_zero(), horizontal(-s.m.c, up).normalize_or_zero(), b"zyx") < 0.0;
                let top = b.part(10).m.d;
                match self.pv.planted {
                    [true, false] if back => self.push += (top - s.legs[0].foot.m.d).normalize_or_zero() * (yaw(1.0, &s.pelvis.m, dir, false, false, s.angvel) * r.p.sideways_cheat_force),
                    [false, true] if !back => self.push += (top - s.legs[1].foot.m.d).normalize_or_zero() * (-yaw(-1.0, &s.pelvis.m, dir, false, false, s.angvel) * r.p.sideways_cheat_force),
                    [true, false] | [false, true] => {}
                    _ => self.push += up,
                }
            }
            r.down_swing = [0, 1].map(|i| r.p.flatten_down_swing && self.f.stepping == i as u32 + 1 && s.legs[i].foot.v.dot(self.p.up) < 0.0);
            // RDR2's +0x44c counts every step, one starting on the last one's landing tick too (rdr2_balancer.md §5.4)
            r.update(dt, s.angvel.length_squared(), was != self.f.stepping && self.f.stepping != 0, self.f.stepping != 0);
        }
        pelvis_control(&self.p, &s, &mut self.pv);
        balance_solve(&self.p, &self.f, &self.pv, &mut s);
        self.phase3(&mut s);
        Some(s)
    }

    pub(crate) fn phase3(&mut self, s: &mut Snap) {
        self.pv.p68 = s.height;
        let c = if self.p.rdr2.is_some() { crate::nmdbm_rdr2::Rdr2::swing_ankle_clamp() } else { delirium_data::kv!(274) };
        for leg in &mut s.legs {
            let x = leg.ankle.desired[0];
            leg.ankle.desired[0] = if x >= -c { x.min(c) } else { -c };
        }
    }

    pub fn activate(&self, ch: &mut Character) {
        if !self.held {
            return;
        }
        self.stiffen(ch);
        for i in [(*HIP())[0], (*KNEE())[0], (*ANKLE())[0], (*HIP())[1], (*KNEE())[1], (*ANKLE())[1], (*SPINE())[0]] {
            ch.eff[i].oppose_gravity = if (*ANKLE()).contains(&i) || (*KNEE()).contains(&i) { 2.0 } else { 1.0 };
        }
    }

    pub fn stiffen(&self, ch: &mut Character) {
        let k = self.p.leg_stiffness;
        for i in [(*HIP())[0], (*KNEE())[0], (*ANKLE())[0], (*HIP())[1], (*KNEE())[1], (*ANKLE())[1], (*SPINE())[0]] {
            (ch.eff[i].strength, ch.eff[i].damping) = (k * k, 2.0 * k);
        }
        for i in 0..2 {
            ch.eff[(*KNEE())[i]].muscle_stiffness = 2.0;
            ch.eff[(*HIP())[i]].muscle_stiffness = delirium_data::kv!(12);
        }
    }

    pub fn apply(&self, s: &Snap, ch: &mut Character) {
        // RDR2 (the tail of 0x2aba1cc): the ankles' limit half-ranges this tick. On a slope, moving back or slow, lean1 at least
        // 1; starting a crouch 1.4 [P: without the depth test]
        if let Some(r) = self.p.rdr2.as_ref() {
            let up = self.p.up;
            let (hv, hc) = (s.vel - up * up.vd(s.vel), s.m.c - up * up.vd(s.m.c));
            let raise = if r.on_slope && (hv.vd(hc) > 0.0 || s.vel.length() < r.p.legs_apart_restep.max(0.7)) { 1.0 } else { 0.0 };
            // 0x2ab617c: the crouch's depth, from the pelvis over the feet
            let height = (0..2).map(|i| up.vd(s.pelvis.m.d - s.legs[i].foot.m.d)).fold(f32::MIN, f32::max);
            let raise = if r.crouching.is_some() && crate::nmdbm_rdr2::Rdr2::crouch_depth(height, self.p.leg_scale(), self.p.leg_length) > 0.0 { 1.4f32.max(raise) } else { raise };
            if raise > 0.0 {
                for i in 0..2 {
                    ch.lim_raise((*ANKLE())[i], [raise, 0.0, 0.0]);
                }
                crate::extras_core_rdr2::trace_once("balancer: the ankles' limits raised", || format!("lean1 {raise}, crouching {}", r.crouching.is_some()));
            }
            // turn mode (+0x319): lean2 at least 0.8
            if r.turning {
                for i in 0..2 {
                    ch.lim_raise((*ANKLE())[i], [0.0, 0.8, 0.0]);
                }
                crate::extras_core_rdr2::trace_first("balancer: turn mode's ankle lean2 raised");
            }
        }
        for i in 0..2 {
            let leg = &s.legs[i];
            ch.eff[(*HIP())[i]].desired = leg.hip.desired;
            ch.eff[(*KNEE())[i]].desired[0] = leg.knee.desired;
            ch.eff[(*ANKLE())[i]].desired = leg.ankle.desired;
        }
        ch.eff[(*SPINE())[0]].desired = s.spine.desired;
        let foot = |i: usize| ch.joints[(*ANKLE())[i]].child as i32;
        match self.f.stepping {
            0 => ch.support = [-2; 2],
            1 => ch.support = [-1, foot(1)],
            2 => ch.support = [foot(0), -1],
            _ => {}
        }
        let soft = self.p.dt.fma(delirium_data::kv!(103), -1.0).fma(delirium_data::kv!(84), 1.0).clamp(1.0, delirium_data::kv!(64));
        let k = self.p.leg_stiffness * self.knee_scale;
        // RDR2's leg stiffness setter (every step): hips, knees and ankles at k (damping ratio 1), hip muscle 1.5, knee
        // muscle 2, and spine joint 0 at the stiffer leg's (the knees are then set again below)
        let each = self.leg_each.filter(|_| self.p.rdr2.is_some()).unwrap_or([self.p.leg_stiffness; 2]);
        if self.p.rdr2.is_some() {
            for i in 0..2 {
                ch.eff[(*HIP())[i]].set_stiffness(each[i], 1.0, Some(1.5));
                ch.eff[(*KNEE())[i]].set_stiffness(each[i], 1.0, Some(2.0));
                ch.eff[(*ANKLE())[i]].set_stiffness(each[i], 1.0, None);
            }
            ch.eff[(*SPINE())[0]].set_stiffness(each[0].max(each[1]), 1.0, None);
        }
        // RDR2's setter (0x2ab9df8), every step: hips and spine 0 opposeGravityLegs, knees twice it
        if let Some(r) = self.p.rdr2 {
            for i in 0..2 {
                ch.eff[(*HIP())[i]].oppose_gravity = r.p.oppose_gravity_legs;
                ch.eff[(*KNEE())[i]].oppose_gravity = 2.0 * r.p.oppose_gravity_legs;
            }
            ch.eff[(*SPINE())[0]].oppose_gravity = r.p.oppose_gravity_legs;
        }
        for i in 0..2 {
            let touching = s.flags & [delirium_data::kv!(251 int), delirium_data::kv!(7 int)][i] != 0;
            ch.eff[(*ANKLE())[i]].muscle_stiffness = if touching { delirium_data::kv!(31) } else { soft };
            let planted = self.f.flags & [delirium_data::kv!(251 int), delirium_data::kv!(253 int)][i] != 0;
            let k = if self.leg_each.is_some() && self.p.rdr2.is_some() { each[i] * self.knee_scale } else { k };
            let knee = &mut ch.eff[(*KNEE())[i]];
            if planted {
                let f = if self.p.taper { (-(-s.legs[i].knee.actual - delirium_data::kv!(12))).fma(delirium_data::kv!(275), 1.0).clamp(delirium_data::kv!(60), 1.0) } else { 1.0 };
                (knee.strength, knee.muscle_stiffness) = ((f * k).powi(2), delirium_data::kv!(12));
            } else {
                (knee.strength, knee.muscle_stiffness) = (k * k, 2.0);
                // RDR2: the swing hip's damping 2 x left/rightLegSwingDamping x k
                if let Some(r) = self.p.rdr2 {
                    crate::extras_core_rdr2::trace_once("swing damping and gravity", || format!("{} {}, gravity {} {}", r.p.swing_damping_l, r.p.swing_damping_r, r.p.oppose_gravity_legs, r.p.oppose_gravity_ankles));
                    ch.eff[(*HIP())[i]].damping = 2.0 * [r.p.swing_damping_l, r.p.swing_damping_r][i] * each[i];
                }
            }
            ch.eff[(*ANKLE())[i]].oppose_gravity = sdot(self.p.normal[i], s.legs[i].foot.m.b, b"yzx").powi(2) * 2.0;
            // RDR2 (0x2abc4ec): 2 x opposeGravityAnkles x that square, the square 1 for the stepping foot, and none for a foot
            // flattenDownSwing has flagged
            if let Some(r) = self.p.rdr2 {
                let stepping = self.f.stepping == i as u32 + 1;
                ch.eff[(*ANKLE())[i]].oppose_gravity = if r.down_swing[i] { 0.0 } else if stepping { 2.0 * r.p.oppose_gravity_ankles } else { 2.0 * r.p.oppose_gravity_ankles * sdot(self.p.normal[i], s.legs[i].foot.m.b, b"yzx").powi(2) };
            }
            // RDR2 core: a hip pitch a behaviour asked the balancer for
            if let Some(r) = self.p.rdr2.filter(|r| r.owned.hip.value != 0.0) {
                ch.eff[(*HIP())[i]].desired[0] += r.owned.hip.value;
            }
            // RDR2's minKneeAngle: a swing knee too straight is held back
            if let (Some(r), false) = (self.p.rdr2, planted) {
                let k = &s.legs[i].knee;
                ch.eff[(*KNEE())[i]].desired[0] = r.knee_limit(ch.eff[(*KNEE())[i]].desired[0], k.actual, 0.0, self.p.dt);
            }
            // RDR2 (0x2abc4ec): running with the planted foot off the ground this tick and the last, the stepping leg's hip and
            // knee relax (the foot's per-part value ctx+0x35cc has no reader found: not here)
            if self.p.rdr2.is_some_and(|r| r.running) && self.f.stepping == i as u32 + 1 {
                let bits: u8 = if self.f.flags & delirium_data::kv!(251 int) != 0 {
                    s.flags & (delirium_data::kv!(251 int) | LEFT_LAST())
                } else if self.f.flags & delirium_data::kv!(253 int) != 0 {
                    s.flags & (delirium_data::kv!(7 int) | RIGHT_LAST())
                } else {
                    1
                };
                if bits == 0 {
                    let (sk, d, m) = crate::nmdbm_rdr2::RUN_SWING;
                    ch.eff[(*HIP())[i]].set_stiffness(sk, d, Some(m));
                    ch.eff[(*KNEE())[i]].set_stiffness(sk, d, Some(m));
                }
            }
        }
    }

    /// RDR2's timed fail: the legs drain, one hip first, until a fail test drops him. Runs after every behaviour has
    /// set its stiffness, just before the muscles are driven.
    pub fn drain(&self, ch: &mut Character) {
        // (staggering, staggerFall's leg stiffness took these as caps: its floors stand, RDR2's 0x2a9e8a4)
        if let Some((leg, hip)) = self.p.rdr2.filter(|r| !r.stagger).and_then(|r| r.drain(self.p.leg_stiffness)) {
            for i in 0..2 {
                for j in [(*KNEE())[i], (*ANKLE())[i]] {
                    ch.eff[j].strength = leg * leg;
                }
                ch.eff[(*HIP())[i]].strength = hip[i] * hip[i];
            }
        }
        if let Some(g) = self.p.rdr2.and_then(|r| r.gravity_share()) {
            for i in 0..2 {
                ch.eff[(*HIP())[i]].oppose_gravity = g;
                ch.eff[(*KNEE())[i]].oppose_gravity = 2.0 * g;
                ch.eff[(*ANKLE())[i]].oppose_gravity = 2.0 * g;
            }
        }
    }
}

/// RDR2 core (rdr2_core2.md 1.1): the left hand, right hand, left foot and right foot connected for the gravity
/// opposition this step. A foot is its contact this step, whatever the balancer's stance (unless animPose set it);
/// a hand is what was last set, or its contact when animPose asked for that.
pub fn connected(b: &Body) -> [bool; 4] {
    let (ch, k) = (b.ch, &b.ch.core);
    let touch = |part: usize| b.doll.touch[part] & delirium_data::kv!(7 int) != 0;
    let hand = |i: usize| if k.hands_auto[i] { touch(ch.joints[(*WRIST())[i]].child) } else { k.hands[i] };
    let foot = |i: usize| k.feet[i].unwrap_or_else(|| touch(ch.joints[(*ANKLE())[i]].child));
    [hand(0), hand(1), foot(0), foot(1)]
}

pub fn oppose_gravity(b: &Body, attached: Option<(usize, Vec3, f32)>, out: &mut [Vec3]) {
    #[allow(non_snake_case)]
    #[inline]
    fn AHEAD() -> f32 { delirium_data::kv!(78) }
    #[allow(non_snake_case)]
    #[inline]
    fn SCALE() -> f32 { delirium_data::kv!(60) }
    let (ch, n) = (b.ch, b.build.parts.len());
    let up = ch.up;
    // RDR2 core: the connected limbs from contact every step (GTA IV: the balancer's stance feet)
    let conn = ch.rdr2.then(|| connected(b));
    let down: Vec<usize> = match conn {
        Some(c) => (0..2).filter(|&i| c[2 + i]).map(|i| ch.joints[(*ANKLE())[i]].child).collect(),
        None => (0..2).filter_map(|i| match ch.support[i] {
            -2 => Some(ch.joints[(*ANKLE())[i]].child).filter(|&f| b.doll.touch[f] & delirium_data::kv!(7 int) != 0),
            k => usize::try_from(k).ok(),
        }).collect(),
    };
    // RDR2 core, ZMP mode: no foot connected, no gravity opposition (hands or not)
    if ch.rdr2 && ch.core.zmp && down.is_empty() {
        return;
    }
    let lo = down.iter().map(|f| f - delirium_data::kv!(0 int)).min().unwrap_or(delirium_data::kv!(276 int)).min(delirium_data::kv!(276 int)) as i32;
    let hi = down.iter().map(|f| f - 1).max().map_or(-delirium_data::kv!(276 int), |h| h as i32).max(-delirium_data::kv!(276 int));
    let chain = |i: usize| (lo..=hi).contains(&(i as i32));
    let pair = hi > lo + delirium_data::kv!(0 int);
    // RDR2 core: a connected hand locks its arm's joints, and either hand the spine's; every connected limb's chain
    // carries what is above it, the torques shared between them
    let hands = conn.map_or([false; 2], |c| [c[0], c[1]]);
    let held: Vec<bool> = (0..ch.joints.len()).map(|i| {
        (0..2).any(|h| hands[h] && crate::nmshot::limb(h).e.contains(&i)) || (hands.contains(&true) && (*SPINE()).contains(&i))
    }).collect();
    let locked = |i: usize| chain(i) || held[i];
    let limbs = conn.map_or(1, |c| c.iter().filter(|&&x| x).count().max(1)) as f32;
    let place = b.doll.rigid.as_ref().map_or(Vec3::ZERO, |r| r.0[0].d + r.1 - b.doll.centre(0));
    let com = b.doll.com().0 + place;
    let feet: Vec<M34> = down.iter().map(|&f| b.part(f).m).collect();
    let mut g = (Vec3::ZERO - up) * crate::ragdoll::GRAVITY();
    if ch.rdr2 && !ch.core.zmp {
        // RDR2 core, ZMP off: the whole gravity, whatever the feet
    } else if let (Some(l), Some(r)) = (feet.first(), feet.last()) {
        let sp = support_at(com, l, r, delirium_data::kv!(68), delirium_data::kv!(9), up).1;
        let d = vnorm(sp - com);
        g = d * sdot(g, d, b"yxz");
    } else {
        g = Vec3::ZERO;
    }
    let rel = |k: usize| b.doll.rigid.as_ref().map_or(b.doll.centre(k), |r| r.0[k].d);
    let vw = |k: usize| b.doll.rigid.as_ref().map_or(b.doll.motion(k), |r| (r.0[k].v, r.0[k].w));
    let mut m: Vec<(f32, Vec3, Vec3)> = (0..n).map(|k| {
        let mk = b.doll.part_mass(k);
        (mk, rel(k) * mk, vw(k).0 * mk)
    }).collect();
    if let Some((k, pos, mass)) = attached.filter(|a| a.0 < n) {
        let pos = pos - b.doll.rigid.as_ref().map_or(Vec3::ZERO, |r| r.1);
        (m[k].0, m[k].1) = (mass + m[k].0, pos.mul_add(Vec3::splat(mass), m[k].1));
    }
    let add = |m: &mut Vec<(f32, Vec3, Vec3)>, to: usize, from: usize| {
        let f = m[from];
        m[to] = (m[to].0 + f.0, m[to].1 + f.1, m[to].2 + f.2);
    };
    for i in (0..ch.joints.len()).rev().filter(|&i| !locked(i)) {
        add(&mut m, ch.joints[i].parent, ch.joints[i].child);
    }
    for i in (0..ch.joints.len()).filter(|&i| locked(i)) {
        add(&mut m, ch.joints[i].child, ch.joints[i].parent);
    }
    let ahead = |s: &(f32, Vec3, Vec3)| s.2.mul_add(Vec3::splat(AHEAD()), s.1) * crate::gjk::recip(s.0);
    let at = |i: usize, k: usize| {
        let p = b.rigid(i).map_or_else(|| b.joint_pos(i), |(c, _, l, _)| crate::rigid::pos(c, l, Vec3::ZERO));
        let (v, w) = vw(k);
        let r = rel(k);
        let vo = b.doll.rage_vo.as_ref().and_then(|o| o.get(k).copied())
            .unwrap_or_else(|| fneg(r.zxy().mul_add(w.yzx(), fneg(r.yzx().mul_add(w.zxy(), v)))));
        let vp = fneg(p.yzx().mul_add(w.zxy(), fneg(p.zxy().mul_add(w.yzx(), vo))));
        vp.mul_add(Vec3::splat(AHEAD()), p)
    };
    for (i, (j, e)) in ch.joints.iter().zip(&ch.eff).enumerate() {
        if e.oppose_gravity == 0.0 {
            continue;
        }
        let (lever, force) = if chain(i) && pair {
            let o = if (i as i32) < lo + delirium_data::kv!(0 int) { i + delirium_data::kv!(0 int) } else { i - delirium_data::kv!(0 int) };
            let (ka, kb) = (j.parent, ch.joints[o].parent);
            let half = (m[kb].0 + m[ka].0) * 0.5;
            let mb = m[kb].2.mul_add(Vec3::splat(AHEAD()), m[kb].1);
            let ma = m[ka].2.mul_add(Vec3::splat(AHEAD()), m[ka].1);
            let c = (ma - mb).mul_add(Vec3::splat(0.5), mb) * crate::gjk::recip(half);
            let (pa, pb) = (at(i, ka), at(o, kb));
            let d = pb - pa;
            let dn = vnorm(d);
            let s = sdot(c - pa, dn, b"yxz") / sdot(d, dn, b"yxz");
            let f = g * half;
            (d.mul_add(Vec3::splat(s), pa) - c, f - dn * sdot(f, dn, b"zyx"))
        } else {
            let k = if locked(i) { j.parent } else { j.child };
            (at(i, k) - ahead(&m[k]), g * m[k].0)
        };
        let t = fneg(force.yzx().mul_add(lever.zxy(), -(force.zxy() * lever.yzx())));
        let t = if locked(i) { Vec3::ZERO - t } else { t };
        // RDR2 core: a locked joint's share is 1/n of the connected limbs (a stance pair's force is twice the mean's)
        let t = match (ch.rdr2, locked(i)) {
            (true, true) => t * ((if chain(i) && pair { 2.0 } else { 1.0 }) / limbs),
            _ => t,
        };
        let max = e.oppose_gravity * e.strength * SCALE();
        let t = if j.three {
            let mut t = clamp_length(t, 0.0, max);
            if (*ANKLE()).contains(&i) {
                let f = b.part(j.child).m;
                let x = sdot(t, f.a, b"zyx");
                if x > max * delirium_data::kv!(17) {
                    t = fneg(f.a.mul_add(Vec3::splat(x - max * delirium_data::kv!(17)), -t));
                }
                let z = sdot(t, f.c, b"zyx");
                let h = max * 0.5;
                let s = if z > h { z - h } else if z < max * -0.5 { z + h } else { 0.0 };
                t = fneg(f.c.mul_add(Vec3::splat(s), -t));
            }
            // RDR2 core: its applier drops (not clamps) a torque over 1e4
            if ch.rdr2 && t.length_squared() > 1e8 {
                continue;
            }
            t
        } else {
            let axis = b.rigid(i).map_or_else(|| b.frames[j.parent].rotation * j.in_parent * Vec3::Z, |(c, _, l, _)| crate::rigid::hinge_axis(c, l));
            let h = sdot(axis, t, b"zyx").clamp(-max, max);
            // RDR2 core: its hinge applier drops a torque over 5000
            if ch.rdr2 && h.abs() > 5000.0 {
                continue;
            }
            axis * h
        };
        out[j.child] += t;
        out[j.parent] += Vec3::ZERO - t;
    }
}

pub static NMX_RELATCH: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
pub fn foot_yaw(b: &Body, yaw: &mut [f32; 2], dt: f32) -> Vec<Vec3> {
    let relatch = NMX_RELATCH.swap(false, std::sync::atomic::Ordering::Relaxed);
    let mut out = vec![Vec3::ZERO; b.build.parts.len()];
    let up = b.ch.up;
    for i in 0..2 {
        // RDR2 core: ctx+0x47fd
        if b.ch.rdr2 && !b.ch.core.foot_hold {
            continue;
        }
        let k = b.ch.joints[(*ANKLE())[i]].child;
        let t = b.doll.touch[k];
        if t & delirium_data::kv!(7 int) == 0 {
            continue;
        }
        let m = b.part(k).m;
        let upright = sdot(up, m.b, b"zyx");
        let upright = if -upright >= 0.0 { 0.0 } else { upright };
        let x = Vec3::X;
        let u = fneg(x.yzx().mul_add(up.zxy(), -(x.zxy() * up.yzx())));
        let now = atan2((m.c.y + m.c.z).mul_add(0.0, m.c.x), sdot(m.c, u, b"zyx"));
        if relatch || t & delirium_data::kv!(1 int) == 0 || (b.ch.rdr2 && b.ch.core.foot_unlatch) {
            yaw[i] = now;
        }
        let mut d = now - yaw[i];
        if d > std::f32::consts::PI {
            d -= std::f32::consts::TAU;
        }
        if d < -std::f32::consts::PI {
            d += std::f32::consts::TAU;
        }
        let w = sdot(b.doll.motion(k).1, up, b"zyx");
        if b.ch.rdr2 {
            // RDR2 core (rdr2_core3.md 3.4): its gains are footSlipStiffness and its 2s once the balancer has run (60 and 10
            // before), over clamp(60 dt, 1, 60); its applier drops a torque over 1e4. Its foot-grip scale (the shot's
            // shockSpin footGripMult) is 1 here: nothing in Delirium sets it.
            let (kp, kd) = b.ch.core.foot_slip;
            let t = up * (d.mul_add(kp, -(w * kd)) * upright) / (60.0 * dt).clamp(1.0, 60.0);
            if t.length_squared() <= 1e8 {
                out[k] = t;
            }
            continue;
        }
        out[k] = (up * (d.mul_add(delirium_data::kv!(103), -(w * delirium_data::kv!(31))) * upright)) * crate::gjk::recip(dt * delirium_data::kv!(103));
    }
    out
}

pub fn stabilise(b: &Body, k: f32, dt: f32) -> Vec<Vec3> {
    let mut out = vec![Vec3::ZERO; b.build.parts.len()];
    let up = b.ch.up;
    let feet = [0, 1].map(|i| b.ch.joints[(*ANKLE())[i]].child);
    if !feet.iter().any(|&f| b.doll.touch[f] & delirium_data::kv!(95 int) == delirium_data::kv!(7 int)) {
        return out;
    }
    let (com, vel) = b.doll.com();
    let (cf, vf) = (|k: usize| b.doll.rigid.as_ref().map_or(b.doll.centre(k), |r| r.0[k].d + r.1), |k: usize| b.doll.rigid.as_ref().map_or(b.doll.motion(k).0, |r| r.0[k].v));
    let mid = (cf(feet[0]) + cf(feet[1])) * 0.5;
    let fv = (vf(feet[0]) + vf(feet[1])) * 0.5;
    let d = com - mid;
    let x = d.vd(d);
    let e = crate::estimate::vrsqrtefp(x);
    let e = if e.is_infinite() { 0.0 } else { e };
    let c = sdot(d * (-x).mul_add(e * e, 1.0).mul_add(0.5 * e, e), up, b"zyx");
    let c = if c <= 0.0 { 0.0 } else { c };
    let c = c * c;
    let s = c * c * k;
    let s = s * s;
    let f = (vel.mul_add(Vec3::splat(delirium_data::kv!(78)), com) - mid).mul_add(Vec3::splat(s), (vel - fv) * (s * delirium_data::kv!(277 f32)));
    let f = (f - up * sdot(f, up, b"zyx")) * (dt * delirium_data::kv!(103));
    out[feet[0]] += f * 0.5;
    out[feet[1]] += f * 0.5;
    out[b.ch.joints[(*HIP())[0]].parent] -= f;
    out
}

/// RDR2 core: turnTowardsBullets (0x2a5d774), each tick of StayUpright's applier while the balancer holds: `heading` is the
/// last bullet record's impulse (+0x140); B = -heading and A = -s.m.c, both flat (world z off) and unit; R = B turned about
/// +z by at most 10 dt, the way A would have to turn to reach B; 20 (L x R) N m on the pelvis and the two top spine parts,
/// L = minus each part's third row.
pub fn turn_towards_bullets(b: &Body, heading: Vec3, dt: f32, torque: &mut [Vec3]) {
    let flat = |v: Vec3| v.with_z(0.0).normalize_or_zero();
    let (aim, to) = (flat(-b.snap(b.ch.up).m.c), flat(-heading));
    if aim == Vec3::ZERO || to == Vec3::ZERO {
        return;
    }
    let angle = aim.dot(to).clamp(-1.0, 1.0).acos().min(10.0 * dt);
    let turned = Quat::from_rotation_z(if aim.cross(to).z >= 0.0 { angle } else { -angle }) * to;
    let spine = *SPINE();
    for k in [b.ch.joints[(*HIP())[0]].parent, b.ch.joints[spine[2]].child, b.ch.joints[spine[3]].child] {
        torque[k] += (-b.part(k).m.c).cross(turned) * 20.0;
    }
}

/// RDR2 core: StayUpright's applier (0x2a58f10), while the balancer is active: its force (0x2a1e6c0), and with
/// `righting` (the balancer balancing, no staggerFall) its righting torque (0x2a598c8). Part positions are their
/// centres here, so the feet's force points carry no moment; the trunk's points aren't in the note (ours: their
/// centres). Left out: the kneel state's foot points, the moving floor (nothing here has one), the scale ctx+0x2e1c (1
/// in every recording).
pub fn stay_upright(b: &Body, r: &crate::nmdbm_rdr2::StayUpright, dt: f32, righting: bool, torque: &mut [Vec3], at: &mut Vec<(usize, Vec3, Vec3)>) {
    use crate::nmdbm_rdr2::StayUpright as S;
    let up = b.ch.up;
    let feet = [0, 1].map(|i| b.ch.joints[(*ANKLE())[i]].child);
    let contact = feet.map(|f| b.doll.touch[f] & delirium_data::kv!(95 int) == delirium_data::kv!(7 int));
    let (com, vel) = b.doll.com();
    let speed = vel.length();
    let (cf, vf) = (|k: usize| b.doll.rigid.as_ref().map_or(b.doll.centre(k), |r| r.0[k].d + r.1), |k: usize| b.doll.rigid.as_ref().map_or(b.doll.motion(k).0, |r| r.0[k].v));
    let pts = feet.map(cf);
    let mid = (pts[0] + pts[1]) * 0.5;
    let d = com - mid;
    let lean = d.normalize_or_zero();
    let pelvis = b.ch.joints[(*HIP())[0]].parent;
    if r.use_forces && contact.contains(&true) {
        let f = if r.velocity_based { S::window(speed, r.force_threshold_vel, r.force_saturation_vel) } else { 1.0 };
        let k = f * r.force_strength;
        if k > 0.0 {
            let c = 1.0 - (1.0 - up.dot(lean).max(0.0)) * r.force_lean_reduction;
            let c = c * c;
            let s = c * c * k;
            let s = s * s;
            let damp = if r.force_damping < 0.0 { 0.2 * s } else { s * r.force_damping };
            let proj = com - up * up.dot(d);
            let i = (proj + vel * 0.1 - mid) * s + (vel - (vf(feet[0]) + vf(feet[1])) * 0.5) * damp;
            let i = (i - up * up.dot(i)) * (60.0 * dt);
            // a foot out of contact counts as 2.25 m behind
            let along = (vel - up * up.dot(vel)).normalize_or_zero();
            let behind = [0, 1].map(|n| if contact[n] { (com - pts[n]).dot(along) } else { 2.25 });
            let i = if behind[0] > r.support_position && behind[1] > r.support_position { i * r.no_support_force_mult } else { i };
            // one foot down: which foot takes forceInAirShare isn't in the note (ours: that one); 0.5 here either way
            let share = match contact {
                [true, false] => [r.force_in_air_share, 1.0 - r.force_in_air_share],
                [false, true] => [1.0 - r.force_in_air_share, r.force_in_air_share],
                _ => [0.5, 0.5],
            };
            for n in 0..2 {
                at.push((feet[n], pts[n], i * (share[n] * r.force_feet_mult)));
            }
            let top = b.ch.joints[(*SPINE())[3]].child;
            at.push((pelvis, cf(pelvis), i * -(1.0 - r.force_spine3_share)));
            at.push((top, cf(top), i * -r.force_spine3_share));
        }
    }
    if !(r.use_torques && righting) || (r.torque_only_in_air && contact.contains(&true)) {
        return;
    }
    let f = if r.velocity_based { S::window(speed, r.torque_threshold_vel, r.torque_saturation_vel) } else { 1.0 };
    let (ks, ds) = (r.torque_strength * f, r.torque_damping * f);
    if ks == Vec3::ZERO && ds == Vec3::ZERO {
        return;
    }
    let s = b.snap(up);
    let tilt = up.dot(lean).clamp(-1.0, 1.0).acos();
    let g = if tilt > r.lean_min { 1.0 - (tilt.min(r.lean_max) - r.lean_min) / (r.lean_max - r.lean_min) * r.torque_lean_reduction } else { 1.0 };
    let q = if r.lean_relative { Quat::from_rotation_arc(up, if lean == Vec3::ZERO { up } else { lean }) } else { Quat::IDENTITY };
    let (x, y) = (q * s.m.a, q * s.m.b);
    let c = if contact.contains(&true) { 1.0 } else { 0.99 };
    // e = x x u, h = u x e as the code has them (0x2a59b8b..; the note had the crosses swapped). The frame (h, u, e) then
    // turns about h by -lean (0x319ed8): e and the yaw axis lean, h doesn't
    let e0 = x.cross(up);
    let h = up.cross(e0);
    let (sl, cl) = (-r.lean).sin_cos();
    let (yaw_up, e) = (up * cl + e0 * sl, e0 * cl - up * sl);
    let w = s.angvel;
    let asin = |v: f32| v.clamp(-1.0, 1.0).asin();
    let scale = g / (30.0 * dt).clamp(1.0, 30.0);
    let mut axes = vec![(h, (-166.667 * asin(e.dot(y)) * c * ks.x - 4.0 * c * ds.x * w.dot(h)) * scale),
                        (e, (166.667 * asin(h.dot(y)) * c * ks.y - 4.0 * c * ds.y * w.dot(e)) * scale)];
    let along = vel - up * up.dot(vel);
    if along.length() > 2.0 {
        axes.push((yaw_up, (-166.667 * asin(along.normalize().dot(h)) * c * ks.z - 4.0 * c * ds.z * w.dot(yaw_up)) * scale));
    }
    let n = b.build.parts.len();
    for (axis, t) in axes {
        if r.torque_parts {
            // over every part by its inertia about the axis (ours: of their sum, so the whole torque is applied)
            let a = axis.normalize_or_zero();
            let sum: f32 = (0..n).map(|k| b.doll.inertia(k, a)).sum();
            for k in 0..n {
                torque[k] += axis * (t * b.doll.inertia(k, a) / sum);
            }
        } else {
            torque[pelvis] += axis * t;
        }
    }
}
