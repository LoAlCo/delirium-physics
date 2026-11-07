use crate::fneg;
use crate::Fma;

pub static LOOSE: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);

fn loose(j: usize, l: [(f32, f32); 3]) -> [(f32, f32); 3] {
    let want = match j { 0 | 3 => [0.0, 0.0, delirium_data::kv!(11 f32)], 2 | 5 => [delirium_data::kv!(79), delirium_data::kv!(38), 0.0], _ => return l };
    if !LOOSE.load(std::sync::atomic::Ordering::Relaxed) { return l }
    std::array::from_fn(|k| {
        let (mid, half) = ((l[k].0 + l[k].1) * 0.5, ((l[k].1 - l[k].0) * 0.5).max(want[k].to_radians()));
        (mid - half, mid + half)
    })
}
use crate::prelude::*;
use delirium_data::nm_asset;

use crate::ragdoll::{Build, Muscle, Rigid};

pub const HOMER: usize = 0;
pub const MARGE: usize = 1;
/// Red Dead Redemption 2's own bodies (Fred, Wilma), built from the player's own RDR2 files when asked for: the same
/// tables as Homer's and Marge's, in the same layout.
pub const RDR2_FRED: usize = 2;
pub const RDR2_WILMA: usize = 3;
pub fn asset_of(ped_type: &str) -> usize {
    if ped_type.eq_ignore_ascii_case("CIVFEMALE") { MARGE } else { HOMER }
}
pub static ASSET: [std::sync::OnceLock<Vec<nm_asset::Joint>>; 4] = [const { std::sync::OnceLock::new() }; 4];
pub static SELF: [std::sync::OnceLock<Vec<(usize, usize)>>; 4] = [const { std::sync::OnceLock::new() }; 4];
pub static PARTS: [std::sync::OnceLock<Vec<[[f32; 3]; 4]>>; 4] = [const { std::sync::OnceLock::new() }; 4];
pub static MASSES: [std::sync::OnceLock<Vec<f32>>; 4] = [const { std::sync::OnceLock::new() }; 4];

pub fn load(nm: usize, d: &[u8]) {
    set(nm, nm_asset::read(d), nm_asset::parts(d), nm_asset::masses(d), nm_asset::self_collision(d));
}

pub fn set(nm: usize, joints: Vec<nm_asset::Joint>, parts: Vec<[[f32; 3]; 4]>, masses: Vec<f32>, pairs: Vec<(usize, usize)>) {
    assert!(delirium_data::kv::open(), "the numbers aren't open: they're read from your own game's program first (kv::unlock)");
    let _ = ASSET[nm].set(joints);
    let _ = PARTS[nm].set(parts);
    let _ = MASSES[nm].set(masses);
    let _ = SELF[nm].set(pairs);
}

pub fn cg(nm: usize) -> Vec3 {
    let Some(p) = PARTS[nm].get().and_then(|p| p.first()) else { return Vec3::ZERO };
    let r = quarter_rows([p[0], p[1], p[2]]);
    let t = p[delirium_data::kv!(0 int)];
    Vec3::from_array(std::array::from_fn(|j| t[0] * r[0][j] + t[1] * r[1][j] + t[2] * r[2][j]))
}

fn quarter_rows(m: [[f32; 3]; 3]) -> [[f32; 3]; 3] {
    let h = std::f32::consts::FRAC_PI_2 as f64;
    let (c, s) = (h.cos() as f32, h.sin() as f32);
    let z = [[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]];
    m.map(|m| std::array::from_fn(|j| z[2][j].mul_add(m[2], z[1][j].mul_add(m[1], z[0][j] * m[0]))))
}

fn bodies(parts: &[[[f32; 3]; 4]]) -> Vec<([Vec3; 3], Vec3)> {
    let off = Vec3::from(parts[0][delirium_data::kv!(0 int)]);
    parts.iter().enumerate().map(|(i, p)| {
        let mut m = [0.0f32; 16];
        for k in 0..delirium_data::kv!(1 int) {
            m[delirium_data::kv!(1 int) * k..delirium_data::kv!(1 int) * k + delirium_data::kv!(0 int)].copy_from_slice(&p[k]);
        }
        delirium_data::matfix::matrix_fix(&mut m);
        let r: [[f32; 3]; 3] = std::array::from_fn(|k| [m[delirium_data::kv!(1 int) * k], m[delirium_data::kv!(1 int) * k + 1], m[delirium_data::kv!(1 int) * k + 2]]);
        let r = if i == 0 { quarter_rows(r) } else { r };
        (r.map(Vec3::from), Vec3::from(p[delirium_data::kv!(0 int)]) - off)
    }).collect()
}

pub(crate) fn ours(v: [f32; 3]) -> Vec3 {
    Vec3::new(-v[0], -v[1], v[2])
}

#[allow(non_snake_case)]
#[inline]
pub fn HIP() -> &'static [usize; 2] { static V: std::sync::OnceLock<[usize; 2]> = std::sync::OnceLock::new(); let f = || [0, delirium_data::kv!(0 int)]; if delirium_data::kv::open() { V.get_or_init(f) } else { Box::leak(Box::new(f())) } }
#[allow(non_snake_case)]
#[inline]
pub fn KNEE() -> &'static [usize; 2] { static V: std::sync::OnceLock<[usize; 2]> = std::sync::OnceLock::new(); let f = || [1, delirium_data::kv!(1 int)]; if delirium_data::kv::open() { V.get_or_init(f) } else { Box::leak(Box::new(f())) } }
#[allow(non_snake_case)]
#[inline]
pub fn ANKLE() -> &'static [usize; 2] { static V: std::sync::OnceLock<[usize; 2]> = std::sync::OnceLock::new(); let f = || [2, delirium_data::kv!(2 int)]; if delirium_data::kv::open() { V.get_or_init(f) } else { Box::leak(Box::new(f())) } }
#[allow(non_snake_case)]
#[inline]
pub fn SPINE() -> &'static [usize; 4] { static V: std::sync::OnceLock<[usize; 4]> = std::sync::OnceLock::new(); let f = || [delirium_data::kv!(6 int), delirium_data::kv!(22 int), delirium_data::kv!(23 int), delirium_data::kv!(104 int)]; if delirium_data::kv::open() { V.get_or_init(f) } else { Box::leak(Box::new(f())) } }
#[allow(non_snake_case)]
#[inline]
pub fn NECK() -> &'static [usize; 2] { static V: std::sync::OnceLock<[usize; 2]> = std::sync::OnceLock::new(); let f = || [delirium_data::kv!(81 int), delirium_data::kv!(113 int)]; if delirium_data::kv::open() { V.get_or_init(f) } else { Box::leak(Box::new(f())) } }
#[allow(non_snake_case)]
#[inline]
pub fn CLAVICLE() -> &'static [usize; 2] { static V: std::sync::OnceLock<[usize; 2]> = std::sync::OnceLock::new(); let f = || [delirium_data::kv!(55 int), delirium_data::kv!(54 int)]; if delirium_data::kv::open() { V.get_or_init(f) } else { Box::leak(Box::new(f())) } }
#[allow(non_snake_case)]
#[inline]
pub fn SHOULDER() -> &'static [usize; 2] { static V: std::sync::OnceLock<[usize; 2]> = std::sync::OnceLock::new(); let f = || [delirium_data::kv!(114 int), delirium_data::kv!(110 int)]; if delirium_data::kv::open() { V.get_or_init(f) } else { Box::leak(Box::new(f())) } }
#[allow(non_snake_case)]
#[inline]
pub fn ELBOW() -> &'static [usize; 2] { static V: std::sync::OnceLock<[usize; 2]> = std::sync::OnceLock::new(); let f = || [delirium_data::kv!(56 int), delirium_data::kv!(211 int)]; if delirium_data::kv::open() { V.get_or_init(f) } else { Box::leak(Box::new(f())) } }
#[allow(non_snake_case)]
#[inline]
pub fn WRIST() -> &'static [usize; 2] { static V: std::sync::OnceLock<[usize; 2]> = std::sync::OnceLock::new(); let f = || [delirium_data::kv!(198 int), delirium_data::kv!(199 int)]; if delirium_data::kv::open() { V.get_or_init(f) } else { Box::leak(Box::new(f())) } }
#[allow(non_snake_case)]
#[inline]
pub fn LIMBS() -> &'static [(usize, usize); 4] { static V: std::sync::OnceLock<[(usize, usize); 4]> = std::sync::OnceLock::new(); let f = || [(delirium_data::kv!(114 int), delirium_data::kv!(56 int)), (delirium_data::kv!(110 int), delirium_data::kv!(211 int)), (0, 1), (delirium_data::kv!(0 int), delirium_data::kv!(1 int))]; if delirium_data::kv::open() { V.get_or_init(f) } else { Box::leak(Box::new(f())) } }

#[derive(Clone, Debug)]
pub struct JointDef {
    pub parent: usize,
    pub child: usize,
    pub three: bool,
    pub in_parent: Quat,
    pub in_child: Quat,
    pub rage_parent: Quat,
    pub rage_child: Quat,
    pub limits: [(f32, f32); 3],
    pub flip: [bool; 3],
    pub muscle_stiffness: f32,
    pub damping: f32,
    pub strength: f32,
    pub max_torque: [f32; 2],
    pub rigid: Option<crate::rigid::JointConst>,
}

pub fn joints(build: &Build, asset: &[nm_asset::Joint]) -> Option<Vec<JointDef>> {
    let parts = PARTS[build.nm].get()?;
    if asset.len() != delirium_data::kv!(127 int) || build.parts.len() != delirium_data::kv!(111 int) || parts.len() != delirium_data::kv!(111 int) {
        return None;
    }
    let zero = zero_pose(build);
    let (body, off) = (bodies(parts), Vec3::from(parts[0][delirium_data::kv!(0 int)]));
    asset.iter().enumerate().map(|(i, j)| {
        // RDR2's rig, when chosen: its joint values on this body
        let j = &crate::rig::patch(build.nm, i, j);
        let child = i + 1;
        let parent = build.parts[child].parent?;
        let z = ours(j.twist_axis).normalize_or(Vec3::Z);
        let x0 = if j.three { ours(j.lean_axis) } else { z.cross(Vec3::Z) };
        let x = (x0 - z * x0.dot(z)).normalize_or(z.any_orthonormal_vector());
        let frame = Quat::from_mat3(&Mat3::from_cols(x, z.cross(x), z)).normalize();
        let (in_parent, in_child) = ((zero[parent].inverse() * frame).normalize(), (zero[child].inverse() * frame).normalize());
        let (rage_parent, rage_child) = rage_frames(j.three, &j.limits, in_parent, in_child);
        let (a, b) = (&body[parent], &body[child]);
        let rigid = rage_const(j.three, j.centred, Vec3::from(j.pos) - off, Vec3::from(j.twist_axis), Vec3::from(j.lean_axis), &j.limits,
                              (&a.0, a.1), (&b.0, b.1));
        Some(JointDef {
            parent, child, three: j.three, in_parent, in_child, rage_parent, rage_child,
            limits: j.limits, flip: j.flip,
            muscle_stiffness: j.muscle_stiffness, damping: j.damping, strength: j.strength, max_torque: j.max_torque,
            rigid: Some(rigid),
        })
    }).collect()
}

pub fn asset_layout(build: &mut Build, frames: &mut [Transform]) {
    let (Some(parts), Some(asset)) = (PARTS[build.nm].get(), ASSET[build.nm].get()) else { return };
    if parts.len() != delirium_data::kv!(111 int) || asset.len() != delirium_data::kv!(127 int) || build.parts.len() != delirium_data::kv!(111 int) || frames.len() != delirium_data::kv!(111 int) {
        return;
    }
    let zero = zero_pose(build);
    let centre = |i: usize| ours(parts[i][delirium_data::kv!(0 int)]);
    for (i, j) in asset.iter().enumerate() {
        let child = i + 1;
        let Some(parent) = build.parts[child].parent else { continue };
        let pivot = ours(j.pos);
        build.parts[child].anchor = zero[child].inverse() * (pivot - centre(child));
        build.parts[child].anchor_parent = zero[parent].inverse() * (pivot - centre(parent));
    }
    for c in 0..frames.len() {
        if let Some(p) = build.parts[c].parent {
            let pivot = frames[p].translation + frames[p].rotation * build.parts[c].anchor_parent;
            frames[c].translation = pivot - frames[c].rotation * build.parts[c].anchor;
        }
    }
}

pub fn derive_links(build: &Build, stand: &[Quat]) -> Option<Vec<Quat>> {
    let parts = PARTS[build.nm].get()?;
    if build.parts.len() != delirium_data::kv!(111 int) || parts.len() != delirium_data::kv!(111 int) || stand.len() != delirium_data::kv!(111 int) {
        return None;
    }
    Some(parts.iter().zip(stand).map(|(m, &s)| {
        let nm = asset_frame(m, false);
        (quarter(nm.inverse() * s).inverse() * nm.inverse() * asset_frame(m, true)).normalize()
    }).collect())
}

pub(crate) fn rage_turns(limits: &[(f32, f32); 3]) -> ([f32; 4], [f32; 4]) {
    let mid = |k: usize| (limits[k].0 + limits[k].1) * 0.5;
    let tan = |v: f32| (v as f64).tan() as f32;
    let l = tan_quat(0.0, tan(mid(0) * delirium_data::kv!(17)), tan(mid(1) * delirium_data::kv!(17)));
    let half = (mid(2) * 0.5) as f64;
    let (s, c) = (half.sin() as f32, half.cos() as f32);
    ([l.y, l.x, l.z, l.w], [0.0 * s, 0.0 * s, s, c])
}

pub(crate) fn rage_rows(limits: &[(f32, f32); 3], parent: [Vec3; 3], child: [Vec3; 3]) -> ([Vec3; 3], [Vec3; 3]) {
    let (a, b) = rage_turns(limits);
    (turn_rows(parent, a), turn_rows(child, b))
}

pub(crate) fn joint_frames(p: Vec3, b: Vec3, c: Vec3, a: (&[Vec3; 3], Vec3), bb: (&[Vec3; 3], Vec3)) -> crate::rigid::JointConst {
    frames(p, c, cross(b, c), b, a, bb)
}

fn cross(b: Vec3, c: Vec3) -> Vec3 {
    Vec3::new((-c.y).fma(b.z, c.z * b.y), (-c.z).fma(b.x, c.x * b.z), (-c.x).fma(b.y, c.y * b.x))
}

fn frames(p: Vec3, c: Vec3, x: Vec3, b: Vec3, (ra, ta): (&[Vec3; 3], Vec3), (rb, tb): (&[Vec3; 3], Vec3)) -> crate::rigid::JointConst {
    let w = [Vec3::new(c.x, x.x, b.x), Vec3::new(c.y, x.y, b.y), Vec3::new(c.z, x.z, b.z)];
    use crate::Vd;
    let side = |r: &[Vec3; 3], t: Vec3| -> ([Vec3; 3], Vec3) {
        let col = |k: usize| Vec3::new(r[0][k], r[1][k], r[2][k]);
        let d = p - t;
        ([0, 1, 2].map(|i| crate::rigid::row_mul(col(i), &w)), Vec3::new(d.vd(col(0)), d.vd(col(1)), d.vd(col(2))))
    };
    let ((pm, pa), (cm, ca)) = (side(ra, ta), side(rb, tb));
    crate::rigid::JointConst { pm, pa, ca, cm }
}

pub(crate) fn joint_limits(f: [f32; 3]) -> ([f32; 6], [bool; 2]) {
    let g = |x: f32| { let v = x - delirium_data::kv!(520 f32); if -v >= 0.0 { 0.0 } else { v } };
    let l = [g(f[0]), g(f[0]), g(f[1]), g(f[1]), g(f[2]), g(f[2])];
    (l, [f[0] != f[1], l[delirium_data::kv!(2 int)] != f32::MAX])
}

fn across(a: Vec3) -> (Vec3, Vec3) {
    let near = !(a.x > 0.5) && !(a.x < -0.5) && !(a.y > 0.5) && !(a.y < -0.5);
    let u = crate::nmdbm::vnorm(if near { Vec3::new(0.0, a.z, -a.y) } else { Vec3::new(a.y, -a.x, 0.0) });
    (u, crate::nmdbm::vnorm(cross(a, u)))
}

#[derive(Debug)]
pub struct PhJoint {
    pub three: bool,
    pub effector: bool,
    pub jc: crate::rigid::JointConst,
    pub limits: Option<([f32; 6], [bool; 2])>,
}

pub fn ph_joint(s: &mut nm_asset::Stream, off: Vec3, a: (&[Vec3; 3], Vec3), b: (&[Vec3; 3], Vec3)) -> Option<PhJoint> {
    let three = match s.byte().ok()? { b'3' => true, b'1' => false, _ => return None };
    let eps = delirium_data::kv!(521 f32);
    let mut v = || s.vec3().ok().map(Vec3::from);
    let p = v()? - off;
    let ax = v()?;
    let len2 = |v: Vec3| v.z.fma(v.z, v.x.fma(v.x, v.y * v.y));
    let lean = if three {
        let c = v()?;
        let d = ax - c;
        if len2(ax) < eps || len2(c) < eps || d.x.fma(d.x, d.z.fma(d.z, d.y * d.y)) < eps {
            return None;
        }
        c
    } else {
        if len2(ax) < eps {
            return None;
        }
        Vec3::ZERO
    };
    let (centred, effector) = (s.flag().ok()? != 0, s.flag().ok()? != 0);
    let n = if three { delirium_data::kv!(324 int) } else { delirium_data::kv!(7 int) };
    let blob = s.string(Some(n)).ok()?;
    if blob.len() != n {
        return None;
    }
    let f = |k: usize| f32::from_bits(u32::from_be_bytes([blob[k], blob[k + 1], blob[k + 2], blob[k + delirium_data::kv!(0 int)]]));
    let l = if three { [(f(delirium_data::kv!(55 int)), f(delirium_data::kv!(54 int))), (f(delirium_data::kv!(127 int)), f(delirium_data::kv!(107 int))), (f(delirium_data::kv!(85 int)), f(delirium_data::kv!(7 int)))] } else { [(0.0, 0.0); 3] };
    let jc = rage_const(three, centred, p, ax, lean, &l, a, b);
    if !centred {
        return Some(PhJoint { three, effector, jc, limits: None });
    }
    if !three {
        let c = delirium_data::kv!(520 f32);
        let (lo, hi) = (c + f(delirium_data::kv!(23 int)), f(delirium_data::kv!(55 int)) - c);
        let (lo, hi) = if lo <= hi { (lo, hi) } else { let m = (f(delirium_data::kv!(23 int)) + f(delirium_data::kv!(55 int))) * 0.5; (m, m) };
        return Some(PhJoint { three, effector, jc, limits: Some(([lo, hi, lo, hi, 0.0, 0.0], [false; 2])) });
    }
    let h = l.map(|(lo, hi)| (hi - lo) * 0.5);
    let (mut lim, _) = joint_limits(h);
    for k in 0..delirium_data::kv!(0 int) {
        lim[2 * k + 1] = joint_limits([f(delirium_data::kv!(94 int) + delirium_data::kv!(1 int) * k) * h[k]; 3]).0[0];
    }
    Some(PhJoint { three, effector, jc, limits: Some((lim, [true, lim[delirium_data::kv!(2 int)] != f32::MAX])) })
}

fn rage_const(three: bool, centred: bool, p: Vec3, ax: Vec3, lean: Vec3, l: &[(f32, f32); 3], a: (&[Vec3; 3], Vec3),
              b: (&[Vec3; 3], Vec3)) -> crate::rigid::JointConst {
    if !(centred && three) {
        let (u, w) = across(ax);
        return frames(p, u, w, ax, a, b);
    }
    let mut jc = joint_frames(p, ax, lean, a, b);
    let (qa, qb) = rage_turns(l);
    (jc.pm, jc.cm) = (turn_rows(jc.pm, qa), turn_rows(jc.cm, qb));
    jc
}

fn rage_frames(three: bool, limits: &[(f32, f32); 3], in_parent: Quat, in_child: Quat) -> (Quat, Quat) {
    if !three {
        return (in_parent, in_child);
    }
    let rows = |q: Quat| { let m = Mat3::from_quat(q); [m.row(0), m.row(1), m.row(2)] };
    let quat = |r: [Vec3; 3]| Quat::from_mat3(&Mat3::from_cols(r[0], r[1], r[2]).transpose()).normalize();
    let (p, c) = rage_rows(limits, rows(in_parent), rows(in_child));
    (quat(p), quat(c))
}

/// The turns RAGE gives a joint's frames in its parent and child to centre them in its limits: its frames there are
/// (frame in parent) * .0 and (frame in child) * .1.
pub fn rage_turn(three: bool, limits: &[(f32, f32); 3]) -> (Quat, Quat) {
    rage_frames(three, limits, Quat::IDENTITY, Quat::IDENTITY)
}

/// Each part's turn with every joint at zero (its rig's own zero pose).
pub fn zero_turns(build: &Build) -> Vec<Quat> {
    zero_pose(build)
}

pub(crate) fn zero_pose(build: &Build) -> Vec<Quat> {
    let Some(parts) = PARTS[build.nm].get().filter(|p| p.len() == build.parts.len()) else { return build.parts.iter().map(|p| p.bind).collect() };
    parts.iter().enumerate().map(|(k, m)| (asset_frame(m, true) * crate::nmdbm::link(&build.links, k).inverse()).normalize()).collect()
}

pub(crate) fn zero_places(build: &Build) -> Vec<Vec3> {
    let Some(parts) = PARTS[build.nm].get() else { return vec![] };
    let mut places: Vec<Vec3> = parts.iter().map(|m| Vec3::from(m[delirium_data::kv!(0 int)])).collect();
    if places.len() != build.parts.len() || build.parts.iter().all(|p| p.size == 1.0) {
        return places;
    }
    let turn = zero_pose(build);
    let mut moved = vec![Vec3::ZERO; places.len()];
    for (c, p) in build.parts.iter().enumerate() {
        if let Some(q) = p.parent {
            let grown = |s: f32| 1.0 - 1.0 / s;
            moved[c] = moved[q] + turn[q] * p.anchor_parent * grown(build.parts[q].size) - turn[c] * p.anchor * grown(p.size);
        }
        places[c] += ours(moved[c].into());
    }
    places
}

fn asset_frame(m: &[[f32; 3]; 4], rigid: bool) -> Quat {
    let r = Mat3::from_cols(Vec3::from(m[0]), Vec3::from(m[1]), Vec3::from(m[2]));
    let r = if rigid { r.transpose() } else { r };
    Quat::from_mat3(&Mat3::from_cols(ours(r.x_axis.into()), ours(r.y_axis.into()), ours(r.z_axis.into()))).normalize()
}

fn quarter(q: Quat) -> Quat {
    let m = Mat3::from_quat(q);
    let s = |v: Vec3| { let a = v.abs(); if a.x >= a.y && a.x >= a.z { Vec3::X * v.x.signum() } else if a.y >= a.z { Vec3::Y * v.y.signum() } else { Vec3::Z * v.z.signum() } };
    Quat::from_mat3(&Mat3::from_cols(s(m.x_axis), s(m.y_axis), s(m.z_axis)))
}

pub fn angles(q: Quat) -> [f32; 3] {
    let (x, y, z, w) = (q.x, q.y, q.z, q.w);
    if (x.fma(x, y * y) + z * z).sqrt() <= 0.0 {
        return [0.0; 3];
    }
    let wz = w.fma(w, z * z);
    let (tq, a, b) = if wz > delirium_data::kv!(522) {
        let s = wz.sqrt();
        let tq = if w >= 0.0 { z / (s + w) } else { z / (w - s) };
        let k = 1.0 / ((s + 1.0) * s);
        (tq, y.fma(z, fneg(w * x)) * k, w.fma(y, x * z) * k)
    } else {
        let r = 1.0 / (1.0 - wz).sqrt();
        (0.0, fneg(r * x), r * y)
    };
    let t = b.fma(b, a * a).sqrt();
    let atan = |v: f32| (v as f64).atan() as f32;
    let k = 1.0 / t * (atan(t) * delirium_data::kv!(84));
    [atan(tq) * delirium_data::kv!(84), k * b, k * a]
}

fn mat_quat(r: &[[f32; 3]; 3]) -> Quat {
    let tr = r[0][0] + r[1][1] + r[2][2];
    if tr > 0.0 {
        let s = (tr + 1.0).sqrt();
        let f = 0.5 / s;
        return Quat::from_xyzw((r[1][2] - r[2][1]) * f, (r[2][0] - r[0][2]) * f, (r[0][1] - r[1][0]) * f, s * 0.5);
    }
    let i = if r[1][1] > r[0][0] { 1 } else { 0 };
    let i = if r[2][2] > r[i][i] { 2 } else { i };
    let (j, k) = ((i + 1) % delirium_data::kv!(0 int), (i + 2) % delirium_data::kv!(0 int));
    let s = (r[i][i] - r[j][j] - r[k][k] + 1.0).sqrt();
    let mut v = [0.0; 4];
    v[i] = s * 0.5;
    if s != 0.0 {
        let f = 0.5 / s;
        v[delirium_data::kv!(0 int)] = (r[j][k] - r[k][j]) * f;
        v[j] = (r[i][j] + r[j][i]) * f;
        v[k] = (r[i][k] + r[k][i]) * f;
    }
    Quat::from_xyzw(v[0], v[1], v[2], v[delirium_data::kv!(0 int)])
}

pub fn incoming_rows(links: &[Quat], frames: &[Transform]) -> Vec<[Vec3; 3]> {
    frames.iter().enumerate().map(|(k, f)| {
        let m = Mat3::from_quat(f.rotation * crate::nmdbm::link(links, k));
        [m.x_axis, m.y_axis, m.z_axis]
    }).collect()
}

pub fn rotation([tw, l1, l2]: [f32; 3]) -> Quat {
    if l2.fma(l2, tw.fma(tw, l1 * l1)).sqrt() <= 0.0 {
        return Quat::IDENTITY;
    }
    let sw = l2.fma(l2, l1 * l1).sqrt();
    let tan = |v: f32| (v as f64).tan() as f32;
    let f = 1.0 / (sw + delirium_data::kv!(50)) * tan(sw * delirium_data::kv!(17));
    let (a, b, tq) = (f * l2, f * l1, tan(tw * delirium_data::kv!(17)));
    let (c, e) = (2.0 / tq.fma(tq, 1.0), 2.0 / (b.fma(b, a * a) + 1.0));
    let (c1, e1) = (c - 1.0, e - 1.0);
    Quat::from_xyzw((c * b).fma(tq, -(c1 * a)) * e, (c * a).fma(tq, c1 * b) * e, e1 * c * tq, e1 * c1)
}

pub fn quat_rows(q: [f32; 4]) -> [Vec3; 3] {
    let c = delirium_data::kv!(523 f32);
    let (x, y, z, w) = (q[0] * c, q[1] * c, q[2] * c, q[delirium_data::kv!(0 int)] * c);
    let (zx, yx, wz, wy, zy, wx) = (z * x, y * x, w * z, w * y, z * y, w * x);
    let (zz, yy, xx) = (z * z, y * y, x * x);
    [Vec3::new(1.0 - (zz + yy), yx + wz, zx - wy), Vec3::new(yx - wz, 1.0 - (zz + xx), wx + zy), Vec3::new(wy + zx, zy - wx, 1.0 - (yy + xx))]
}

pub fn turn_rows(m: [Vec3; 3], q: [f32; 4]) -> [Vec3; 3] {
    let [r0, r1, r2] = quat_rows(q);
    m.map(|r| Vec3::new(r.z.mul_add(r2.x, r.y.mul_add(r1.x, r0.x * r.x)), r.z.mul_add(r2.y, r.y.mul_add(r1.y, r0.y * r.x)), r.z.mul_add(r2.z, r.y.mul_add(r1.z, r0.z * r.x))))
}

pub fn tan_quat(tq: f32, a: f32, b: f32) -> Quat {
    let (c, e) = (2.0 / tq.fma(tq, 1.0), 2.0 / (a.fma(a, b * b) + 1.0));
    let (c1, e1) = (c - 1.0, e - 1.0);
    Quat::from_xyzw((tq * c).fma(b, -(c1 * a)) * e, (tq * c).fma(a, c1 * b) * e, e1 * tq * c, e1 * c1)
}

#[allow(non_snake_case)]
#[inline]
fn MIN_DAMPING() -> f32 { delirium_data::kv!(64) }

pub fn effector_angles(raw: [f32; 3], limits: &[(f32, f32); 3], flip: [bool; 3]) -> [f32; 3] {
    std::array::from_fn(|k| {
        let mid = (limits[k].0 + limits[k].1) * 0.5;
        if flip[k] { -raw[k] - mid } else { raw[k] + mid }
    })
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct SoftLimit {
    pub stiffness: f32,
    pub damping: f32,
    pub angle: f32,
    pub dir: f32,
    pub velocity_scaled: bool,
    pub enabled: bool,
}

impl SoftLimit {
    pub fn set_limit(&mut self, angle: f32, dir: f32, velocity_scaled: bool) {
        (self.angle, self.dir, self.velocity_scaled) = (angle, dir.signum(), velocity_scaled);
    }

    pub fn set_limit_angle(&mut self, angle: f32) {
        self.angle = angle;
    }

    pub fn limit(&self) -> f32 {
        self.angle.abs()
    }

    pub fn set_enabled(&mut self, on: bool) {
        self.enabled = on;
    }

    pub fn is_enabled(&self) -> bool {
        self.enabled
    }

    pub fn accel(&self, at: f32, v: f32) -> f32 {
        let err = at - self.angle;
        if !self.enabled || err * self.dir <= 0.0 {
            return 0.0;
        }
        let scale = if self.velocity_scaled { 1.0 + (v * self.dir).max(0.0) } else { 1.0 };
        -(self.stiffness * err + 2.0 * self.damping * self.stiffness.max(0.0).sqrt() * v) * scale
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Effector {
    pub desired: [f32; 3],
    pub zero: [f32; 3],
    pub strength: f32,
    pub damping: f32,
    pub muscle_stiffness: f32,
    pub oppose_gravity: f32,
    pub one: bool,
    pub soft: [SoftLimit; 3],
    pub injury: f32,
    /// Added to `desired` where the joint is driven, not stored in it: extra behaviours lay bends over whatever
    /// else sets the target (cleared before they tick, so it never builds up). Zero unless one uses it.
    pub offset: [f32; 3],
}

impl Effector {
    pub(crate) fn new(j: &JointDef, now: [f32; 3]) -> Effector {
        Effector { desired: [0.0; 3], zero: now, strength: j.strength, damping: j.damping, muscle_stiffness: j.muscle_stiffness, oppose_gravity: 0.0, one: !j.three, soft: Default::default(), injury: 0.0, offset: [0.0; 3] }
    }

    /// The target the joint is driven to: `desired`, plus `offset` when there is one (exactly `desired` otherwise).
    pub fn aim(&self, k: usize) -> f32 {
        if self.offset[k] == 0.0 { self.desired[k] } else { self.desired[k] + self.offset[k] }
    }

    pub fn set_stiffness(&mut self, s: f32, d: f32, ms: Option<f32>) {
        self.strength = s * s;
        self.damping = 2.0 * s * d;
        if let Some(ms) = ms {
            self.muscle_stiffness = ms;
        }
    }

    pub fn set_relaxation(&mut self, j: &JointDef, x: f32) {
        self.strength = j.strength * x * x;
        self.damping = j.damping * x;
    }

    pub fn set_damping_relaxation(&mut self, j: &JointDef, x: f32) {
        self.damping = j.damping * x;
    }

    pub fn reset(&mut self, j: &JointDef) {
        (self.muscle_stiffness, self.strength, self.damping, self.oppose_gravity) = (j.muscle_stiffness, j.strength, j.damping, 0.0);
    }

    pub fn blend_to_zero(&mut self, k: f32) {
        let k = k.clamp(0.0, 1.0);
        for i in 0..delirium_data::kv!(0 int) {
            self.desired[i] = if self.one { (1.0 - k).fma(self.desired[i], self.zero[i] * k) } else { self.zero[i].fma(k, self.desired[i] * (1.0 - k)) };
        }
    }
}

#[derive(Clone)]
pub struct Character {
    pub joints: Vec<JointDef>,
    pub eff: Vec<Effector>,
    pub rng: (u32, u32),
    /// RDR2 core's joint drive (mod rdr2); false: GTA IV's
    pub rdr2: bool,
    pub busy: [bool; 2],
    /// RDR2 core: ctx+0x3630: the pose sources by number, as joint frames (0x2a33e1c); empty when that source is off
    pub pose_src: [Vec<[Vec3; 3]>; 4],
    /// RDR2 core: sit on bodyBalance: noisy elbows (+0x150, cleared by its tick), the joints out of its mask, its target
    pub bb_noisy: bool,
    /// RDR2: a reaction ramps its strength (shot+0x48 under 0.99): bodyBalance's "hit"
    pub bb_hit: bool,
    pub bb_off: Vec<usize>,
    pub bb_target: Option<Vec3>,
    pub stabilise: Option<(f32, f32)>,
    pub foot_yaw: [f32; 2],
    pub attached: Option<(usize, Vec3, f32)>,
    pub agent: u32,
    pub inst_id: i32,
    pub mode: u32,
    pub block: Option<crate::nmdbm::Block>,
    pub read: Vec<[f32; 3]>,
    pub limb: [Option<crate::nmdbm::M34>; 4],
    pub refs: Option<Vec<[Vec3; 4]>>,
    pub support: [i32; 2],
    pub stance_yaw: (f32, f32),
    pub up: Vec3,
    /// RDR2 core: the arms (0 left, 1 right) reachForWound asked catchFall and bodyRollUp to leave alone for their
    /// next tick (RDR2's exclusion masks); cleared once they have ticked
    pub exclude_arms: [bool; 2],
    /// RDR2 core: balancerCollisionsReaction while it runs, for the behaviours that read it: its state, whether the
    /// impact has happened, and the leg stiffness reduction it publishes for staggerFall (RDR2's bal+0x568)
    pub bcr: Option<(u8, bool, f32, f32)>,
    /// RDR2 core: what meleeReact reads of balancerCollisionsReaction: it knows a wall or has hit one (+0x278, +0x27b),
    /// and the time to it (0 once it hit: +0x1e4, +0x224)
    pub bcr_probe: Option<(bool, f32)>,
    /// RDR2 core: arms catchFall leaves alone this tick (meleeReact's +0xfc, catchFall's +0x6c)
    pub cf_exclude: [bool; 2],
    /// RDR2 core: fallOverWall's state (+0x1a0) once it has run, for balancerCollisionsReaction
    pub fow_state: Option<i32>,
    /// RDR2 core: what its core helpers keep on the character (read only with the core on)
    pub core: CoreState,
}

/// RDR2 core: per-character values RDR2's core helpers read (rdr2_core2.md sections 1 and 4).
#[derive(Clone, Copy, Debug)]
pub struct CoreState {
    /// the two hands' weapons (ctx+0x2c70, stride 0x40; matrices ctx+0x2d00, +0x2d40)
    pub weapon: [WeaponRec; 2],
    /// shot +0x284, ?3bfc7305: the source of the default arms' zero-pose blend (-1: the joints' own)
    pub arm_source: i32,
    /// ctx+0x34e8: which hands have a weapon registered, bit 16 left, bit 19 right
    pub weapon_bits: u32,
    /// the shot's side lever for the balancer (bal+0x2fc)
    pub side_lever: Option<f32>,
    /// balancerCollisionsReaction's state (+0x268), -1 when it isn't running
    pub bcr_state: i32,
    /// ctx+0x1014, +0x1016
    pub constraint_arms: bool,
    pub constraint_legs: bool,
    /// the COM's velocity last step, for ctx+0x100c
    pub com_was: Option<Vec3>,
    /// ctx+0x2ca4, +0x2ce4: each arm's weapon state: 0 none, 1 pistol, 2 rifle, 3, 4 not usable (0x2a51944, 0x2a4e830)
    pub arm_state: [i32; 2],
    /// a fireWeapon message waiting for pointGun's tick (0x2a2f994 runs only with pointGun on)
    pub fire: Option<FireRec>,
    pub fire_seen: bool,
    /// OverrideElasticity's record, ctx+0x10d0..+0x10e6 (0x2a40a5c, 0x2a5c018)
    pub elastic: ElasticRec,
    /// ctx+0x47e0, +0x47e1: the foot's bound is short and its contacts have no friction (0x2a483f8)
    pub foot_inverted: [bool; 2],
    /// ctx+0x480c: the arms' pose this frame comes from a weapon pose (the core's default arms stay off)
    pub arms_held: bool,
    /// ctx+0xfe0: the floor's acceleration, its horizontal change low-passed. Nothing under him moves in the room, so only the test hook RDR2_FLOOR_ACC fills it
    pub floor_acc: Vec3,
    /// RDR2's joint scale fields this step (k, damping, muscle): lowered by whoever asks, over any setStiffness
    pub scale: [[f32; 3]; 32],
    /// the foot yaw hold's gains (RDR2's ctx+0x1034 / +0x1038): 60 and 10 until the balancer writes
    /// footSlipStiffness and its damping
    pub foot_slip: (f32, f32),
    /// useZMPGravityCompensation (ctx+0x2b54): gravity opposition aimed at the support point, and none with no foot
    /// connected; off (keyframe, animPose), the full gravity even airborne
    pub zmp: bool,
    /// the hands connected for the gravity opposition (ctx+0x2b4c / +0x2b4d): kept until set again
    pub hands: [bool; 2],
    /// the shot's fling2 owns this arm over the stagger's (shot+0x783, +0x78f/+0x790, +0x2a3)
    pub fling_stagger: [bool; 2],
    /// the shot's brace switch (+0x288)
    pub shot_brace: bool,
    /// hands the next gravity pass takes from contact (ctx+0x2b50 / +0x2b51), cleared by it
    pub hands_auto: [bool; 2],
    /// feet animPose set this step (their auto bytes cleared); None: from contact, as every step
    pub feet: [Option<bool>; 2],
    /// StayUpright's slow and fast sets (ctx+0x29a0, ctx+0x2a70)
    pub stay: [crate::nmdbm_rdr2::StayUpright; 2],
    /// the last bullet record's impulse (ctx+0x1150 + idx 0x160, +0x140), for turnTowardsBullets
    pub bullet: Option<bevy_math::Vec3>,
    /// a new bullet record this step (part, world point, impulse) and last step's, seen by every extra once
    pub bullet_new: Option<(usize, bevy_math::Vec3, bevy_math::Vec3)>,
    pub bullet_seen: Option<(usize, bevy_math::Vec3, bevy_math::Vec3)>,
    /// the fall-to-knees record (ctx+0x2e20)
    pub ftk: crate::nmftk_rdr2::Ftk,
    /// what RDR2's core messages hold (extras_core_rdr2.rs)
    pub cfg: crate::extras_core_rdr2::CoreCfg,
    /// the room's props that have touched him lately: id (-1 none), kg, m^3
    pub props: [(i32, f32, f32); 8],
    /// a prop's contact since BCR last looked: part (-1 none), point, normal, id
    pub prop_hit: [(i32, bevy_math::Vec3, bevy_math::Vec3, i32); 4],
    /// the foot yaw hold runs (ctx+0x47fd), and its latches are let go this step (ctx+0x4800, +0x4801)
    pub foot_hold: bool,
    pub foot_unlatch: bool,
    /// what fall-to-knees sets in catchFall (+0xcb kneel, +0xcc, +0xcd arm poses) and rollDownStairs (+0xf0..+0xf7)
    pub cf_knee: [bool; 3],
    pub rds_knee: Option<RdsKnee>,
    /// the joints' runtime limits (`JointLim`)
    pub lim: [JointLim; 32],
}

/// RDR2 core: one joint's runtime limits. A 3-DOF joint's are half-ranges (lean1, lean2, twist); a hinge's its ends (low,
/// high, unused). `base` is +0x110.. (hinge +0x104..): the rig's, or what configureConstraints committed. `work` is +0x11c..
/// (hinge +0x10c..): what this tick asks; the drive puts it back to `base` when it has run (slot +0x100). `live` is what the
/// Rage joint has: slot +0x108 takes it to `work` each step, at once when that widens it, at a rate when it narrows.
#[derive(Clone, Copy, Debug, Default)]
pub struct JointLim {
    /// the rig's own
    pub base0: [f32; 3],
    pub base: [f32; 3],
    pub work: [f32; 3],
    pub live: [f32; 3],
    pub set: bool,
}

/// Fall-to-knees' overlay on rollDownStairs (its +0xf4, +0xf0, +0xf5, +0xf6, +0xf7).
#[derive(Clone, Copy, Debug)]
pub struct RdsKnee {
    pub on: bool,
    pub lean: f32,
    pub stiff: bool,
    pub poses: [bool; 2],
}

impl CoreState {
    /// RDR2's scale(onlyLower): this step's scale fields of a joint, only ever lowered.
    pub fn lower_scale(&mut self, j: usize, k: f32, d: f32, m: f32) {
        if let Some(s) = self.scale.get_mut(j) {
            *s = [s[0].min(k), s[1].min(d), s[2].min(m)];
        }
    }
}

/// The minimum weapon the code runs on. The game's is a body with its own attachment constraint; here a mass on the hand, weighing by the constraint's strength
/// [G for the mass and for strength as the share of its weight the hand feels]. `saved` +0x24 (-1: none), `soft` +0x38, `to_hand` the gunToHand matrix.
#[derive(Clone, Copy, Debug)]
pub struct WeaponRec { pub on: bool, pub to_hand: [Vec3; 3], pub mass: f32, pub strength: f32, pub saved: f32, pub soft: bool }

impl Default for WeaponRec {
    fn default() -> WeaponRec {
        WeaponRec { on: false, to_hand: [Vec3::X, Vec3::Y, Vec3::Z], mass: 0.0, strength: 1.0, saved: -1.0, soft: false }
    }
}

impl WeaponRec {
    /// 0x2a47288: the strength saved, and .05.
    pub fn soften(&mut self) {
        if self.on && !self.soft {
            (self.saved, self.strength, self.soft) = (self.strength, 0.05, true);
        }
    }
    /// 0x2a4b400: softened, the saved strength times `k`, at least .05.
    pub fn scale(&mut self, k: f32) {
        if self.on && self.soft {
            self.strength = (k * self.saved).max(0.05);
        }
    }
    /// 0x2a47208: back: 0 stays off, under .05 as it is.
    pub fn restore(&mut self) {
        if self.on && self.saved >= 0.0 && (self.saved == 0.0 || self.saved >= 0.05) {
            self.strength = self.saved;
        }
        (self.saved, self.soft) = (-1.0, false);
    }
}

/// fireWeapon's parameters (0x2a2f994), scaled and with its direction normalised.
#[derive(Clone, Copy, Debug)]
pub struct FireRec { pub hand: usize, pub strength: f32, pub at_clavicle: bool, pub inhibit: f32, pub direction: Vec3, pub split: f32, pub other: f32 }

/// ctx+0x10d0..: OverrideElasticity's: the world contacts' elasticity (ctx+0x10e0 x +0x10d0 x the part's), while he moves slowly.
#[derive(Clone, Copy, Debug)]
pub struct ElasticRec { pub on: bool, pub elasticity: f32, pub speed: f32, pub threshold: f32, pub decay: f32, pub scale: f32, pub triggered: bool, pub active: bool }

impl Default for ElasticRec {
    fn default() -> ElasticRec {
        ElasticRec { on: false, elasticity: 0.0, speed: 0.0, threshold: 80.0, decay: 0.5, scale: 1.0, triggered: false, active: false }
    }
}

impl CoreState {
    /// 0x2a51944: a weapon mode on the arms: a hand holds a state only with its weapon bit. Mode 2 is the rifle's, on the right.
    pub fn weapon_mode(&mut self, mode: i32) {
        let (l, r) = (self.weapon_bits & 0x10000 != 0, self.weapon_bits & 0x80000 != 0);
        let one = |b: bool| if b { 1 } else { 4 };
        self.arm_state = [if l { 0 } else { 4 }, if r { 0 } else { 4 }];
        match mode {
            0 | 5 => self.arm_state[1] = one(r),
            1 => self.arm_state = [one(l), one(r)],
            2 => self.arm_state[1] = if r { 2 } else { 4 },
            3 => self.arm_state = [one(l), one(r)],
            4 => self.arm_state[0] = one(l),
            _ => {}
        }
    }

    /// 0x2a4e830: one arm's state, if that hand's weapon is there.
    pub fn set_arm_state(&mut self, hand: usize, state: i32) {
        let bit = if hand == 0 { 0x10000 } else { 0x80000 };
        self.arm_state[hand] = if self.weapon_bits & bit != 0 { state } else { 4 };
    }
}

impl Default for CoreState {
    fn default() -> CoreState {
        CoreState { weapon: [WeaponRec::default(); 2], arm_source: -1, side_lever: None, bcr_state: -1, constraint_arms: false, constraint_legs: false, fire_seen: false, com_was: None, weapon_bits: 0, arm_state: [0; 2], fire: None, elastic: ElasticRec::default(), foot_inverted: [false; 2], arms_held: false, floor_acc: Vec3::ZERO, props: [(-1, 0.0, 0.0); 8], prop_hit: [(-1, bevy_math::Vec3::ZERO, bevy_math::Vec3::ZERO, -1); 4], lim: [JointLim::default(); 32], scale: [[1.0; 3]; 32], foot_slip: (60.0, 10.0), zmp: true, bullet_new: None, bullet_seen: None, hands: [false; 2], fling_stagger: [false; 2], shot_brace: true, hands_auto: [false; 2], feet: [None; 2], stay: Default::default(), bullet: None, ftk: Default::default(), cfg: Default::default(), foot_hold: true, foot_unlatch: false, cf_knee: [false; 3], rds_knee: None }
    }
}

/// RDR2 core's joint drive (experimental): what Red Dead Redemption 2's muscle drive changes, from a study of how it
/// behaves. Same muscle law; 3-DOF targets are clamped per axis to the limit centre +- 3 half-ranges instead of the
/// elliptical swing clamp; softness stops at 0.9999; the raised torque stops at 27000 (hinge) / 35000 (3-DOF); an
/// injury weakens as RDR2's looseness, strength (1-x)^2 (at least 0.1) and damping (1-x).
pub mod rdr2 {
    pub const AXIS_CLAMP: f32 = 3.0;
    pub const SOFTNESS_MAX: f32 = 0.9999;
    pub const HINGE_CAP: f32 = 27000.0;
    pub const THREE_CAP: f32 = 35000.0;
    pub const LOOSE_MIN: f32 = 0.1;
}

fn targets_rdr2(j: &JointDef, e: &Effector, wide: [f32; 3]) -> [f32; 3] {
    std::array::from_fn(|k| {
        let (mid, half) = ((j.limits[k].0 + j.limits[k].1) * 0.5, (j.limits[k].1 - j.limits[k].0) * 0.5 + wide[k]);
        let d = if j.flip[k] { -e.aim(k) } else { e.aim(k) };
        d.clamp(mid - rdr2::AXIS_CLAMP * half, mid + rdr2::AXIS_CLAMP * half) - mid
    })
}

fn targets(j: &JointDef, e: &Effector) -> [f32; 3] {
    let mid = |k: usize| (j.limits[k].0 + j.limits[k].1) * 0.5;
    let half = |k: usize| (j.limits[k].1 - j.limits[k].0) * 0.5;
    let k = delirium_data::kv!(25) / (e.strength + delirium_data::kv!(250)) + 1.0;
    let flip = |n: usize, x: f32| if j.flip[n] { -x } else { x };
    let (mut d1, mut d2, d3) = (flip(0, e.aim(0)), flip(1, e.aim(1)), flip(2, e.aim(2)));
    let (u, v) = ((d1 - mid(0)) / half(0), (d2 - mid(1)) / half(1));
    let r2 = v.fma(v, u * u);
    if r2 > k {
        let s = k / r2.sqrt();
        (d1, d2) = ((s * u).fma(half(0), mid(0)), (s * v).fma(half(1), mid(1)));
    }
    let tw = d3.clamp(mid(2) - k * half(2), mid(2) + k * half(2));
    [d1 - mid(0), d2 - mid(1), tw - mid(2)]
}

impl Character {
    pub fn new(joints: Vec<JointDef>, frames: &[Transform]) -> Character {
        let mut c = Character { pose_src: Default::default(), eff: vec![], joints, rng: (0, 0), rdr2: false, busy: [false; 2], bb_noisy: false, bb_hit: false, bb_off: vec![], bb_target: None, stabilise: None, foot_yaw: [0.0; 2], attached: None, agent: 0, inst_id: 0, mode: 1, block: None, read: vec![], limb: [None; 4], refs: None, support: [-2; 2], up: Vec3::Z, stance_yaw: (crate::nmdbm::CHAR_YAW().1, crate::nmdbm::CHAR_HEAD()), exclude_arms: [false; 2], bcr: None, bcr_probe: None, cf_exclude: [false; 2], fow_state: None, core: CoreState::default() };
        c.seed(0);
        c.eff = (0..c.joints.len()).map(|i| Effector::new(&c.joints[i], c.angle(i, frames))).collect();
        static ONCE: std::sync::Once = std::sync::Once::new();
        ONCE.call_once(|| {
            for (i, (j, e)) in c.joints.iter().zip(&c.eff).enumerate() {
                info!("nm: joint {i} angles {:.2?} flip {:?} limits {:.2?}", e.desired, j.flip, &j.limits[..if j.three { delirium_data::kv!(0 int) } else { 1 }]);
            }
        });
        c
    }

    pub fn seed(&mut self, s: u32) {
        let s = (s << delirium_data::kv!(0 int)) & delirium_data::kv!(524 int) & delirium_data::kv!(525 int);
        self.rng = (s + (s == 0) as u32, s.rotate_left(delirium_data::kv!(54 int)) ^ s);
    }

    pub fn random(&mut self) -> f32 {
        (self.random_bits() & delirium_data::kv!(190 int)) as f32 / delirium_data::kv!(526)
    }

    pub fn random_bits(&mut self) -> u32 {
        let n = self.rng.0 as u64 * delirium_data::kv!(192 int) + self.rng.1 as u64;
        self.rng = (n as u32, (n >> delirium_data::kv!(7 int)) as u32);
        self.rng.0 & delirium_data::kv!(527 int)
    }

    pub fn actual(&self, i: usize, frames: &[Transform]) -> [f32; 3] {
        self.read.get(i).copied().unwrap_or_else(|| self.angle(i, frames))
    }

    pub fn angle(&self, i: usize, frames: &[Transform]) -> [f32; 3] {
        self.angle_at(i, frames, None)
    }

    pub fn angle_at(&self, i: usize, frames: &[Transform], twist: Option<f32>) -> [f32; 3] {
        let j = &self.joints[i];
        let rel = (frames[j.parent].rotation * j.rage_parent).inverse() * (frames[j.child].rotation * j.rage_child);
        if j.three {
            let [tw, l1, l2] = angles(rel);
            effector_angles([l1, l2, twist.unwrap_or(tw)], &j.limits, j.flip)
        } else {
            let r = if rel.w < 0.0 { -rel } else { rel };
            [2.0 * r.z.atan2(r.w), 0.0, 0.0]
        }
    }

    /// A joint's rotation as its effector's (lean1, lean2, twist): RDR2's 0x2a4acb0 and 0x2a376b4.
    pub fn joint_triple(&self, i: usize, q: Quat) -> [f32; 3] {
        let j = &self.joints[i];
        let [tw, l1, l2] = angles(q);
        effector_angles([l1, l2, tw], &j.limits, j.flip)
    }

    pub fn incoming_quat(&self, i: usize, m: &[[Vec3; 3]]) -> Quat {
        let j = &self.joints[i];
        let k = j.rigid.expect("joint constants");
        let t = |r: &[Vec3; 3]| [0, 1, 2].map(|c| crate::rigid::col(r, c));
        let mut tp = t(&m[j.parent]);
        if j.parent == 0 {
            let a = std::f32::consts::FRAC_PI_2 as f64;
            let (c, s) = (a.cos() as f32, a.sin() as f32);
            tp = crate::rigid::rows_mul(&tp, &[Vec3::new(c, s, 0.0), Vec3::new(-s, c, 0.0), Vec3::Z]);
        }
        let p = t(&crate::rigid::rows_mul(&tp, &k.pm));
        let c = t(&crate::rigid::rows_mul(&t(&m[j.child]), &k.cm));
        let r = [0, 1, 2].map(|a| [0, 1, 2].map(|b| c[a].x.mul_add(p[b].x, c[a].y.mul_add(p[b].y, c[a].z * p[b].z))));
        mat_quat(&r)
    }

    pub fn incoming_angle(&self, i: usize, m: &[[Vec3; 3]]) -> [f32; 3] {
        let j = &self.joints[i];
        let [tw, l1, l2] = angles(self.incoming_quat(i, m));
        if j.three { effector_angles([l1, l2, tw], &j.limits, j.flip) } else { [tw, 0.0, 0.0] }
    }

    pub fn pose(&self, build: &Build, root: Transform, angles: &[[f32; 3]]) -> Vec<Transform> {
        let mut out = vec![root; build.parts.len()];
        for (c, p) in build.parts.iter().enumerate() {
            let (Some(par), Some(i)) = (p.parent, self.joints.iter().position(|j| j.child == c)) else { continue };
            let (j, a) = (&self.joints[i], angles[i]);
            let rel = if j.three {
                let mid = |k: usize| (j.limits[k].0 + j.limits[k].1) * 0.5;
                let flip = |n: usize, x: f32| if j.flip[n] { -x } else { x };
                rotation([flip(2, a[2]) - mid(2), flip(0, a[0]) - mid(0), flip(1, a[1]) - mid(1)])
            } else {
                rotation([a[0], 0.0, 0.0])
            };
            let q = (out[par].rotation * j.rage_parent * rel * j.rage_child.inverse()).normalize();
            let pivot = out[par].translation + out[par].rotation * build.parts[c].anchor_parent;
            out[c] = Transform { translation: pivot - q * build.parts[c].anchor, rotation: q, ..root };
        }
        out
    }

    pub fn mask(mask: &str) -> Vec<usize> {
        let b = mask.as_bytes();
        let (p, s) = (b.first().copied().unwrap_or(b'f'), b.get(1).copied().unwrap_or(b'b'));
        let has = |set: &[u8]| set.contains(&s);
        let mut out = vec![];
        match p {
            b'f' if s == b'b' => out.extend(0..delirium_data::kv!(127 int)),
            b'u' => {
                if s == b'w' { out.extend((*WRIST())); }
                if s == b'c' { out.extend((*CLAVICLE())); }
                if has(b"abtl") { out.extend([(*CLAVICLE())[0], (*ELBOW())[0], (*SHOULDER())[0], (*WRIST())[0]]); }
                if has(b"abtr") { out.extend([(*CLAVICLE())[1], (*ELBOW())[1], (*SHOULDER())[1], (*WRIST())[1]]); }
                if has(b"sbtk") { out.extend((*SPINE())); }
                if has(b"nbk") { out.extend((*NECK())); }
            }
            b'l' => {
                if has(b"lb") { out.extend([(*KNEE())[0], (*HIP())[0], (*ANKLE())[0]]); }
                if has(b"rb") { out.extend([(*KNEE())[1], (*HIP())[1], (*ANKLE())[1]]); }
            }
            _ => {}
        }
        out
    }

    pub fn lock(&self, mask: &str) -> Option<Vec<Effector>> {
        (mask != "fb").then(|| self.eff.clone())
    }

    pub fn unlock(&mut self, mask: &str, keep: Option<Vec<Effector>>) {
        let open = Self::mask(mask);
        for (i, e) in keep.into_iter().flatten().enumerate() {
            if !open.contains(&i) {
                self.eff[i] = e;
            }
        }
    }

    pub fn unlocked(flags: u8) -> u8 {
        flags & !delirium_data::kv!(7 int)
    }

    pub fn limits(&self, j: usize) -> [f32; 6] {
        let jd = &self.joints[j];
        let l = self.lim_rage(j);
        let f = |k: usize| if jd.flip[k] { (-l[k].1, -l[k].0) } else { l[k] };
        let ((a, b), (c, d), (e, g)) = (f(0), f(1), f(2));
        [a, b, c, d, e, g]
    }

    /// RDR2 core: the joint's limits as the rig has them, plus what the runtime table has them wider by.
    fn lim_rage(&self, j: usize) -> [(f32, f32); 3] {
        let l = loose(j, self.joints[j].limits);
        let t = &self.core.lim[j];
        if !self.rdr2 || !t.set {
            return l;
        }
        if self.joints[j].three {
            std::array::from_fn(|k| (l[k].0 - (t.live[k] - t.base0[k]), l[k].1 + (t.live[k] - t.base0[k])))
        } else {
            [(l[0].0 + (t.live[0] - t.base0[0]), l[0].1 + (t.live[1] - t.base0[1])), l[1], l[2]]
        }
    }

    /// RDR2 core: how much wider than the rig each half-range is now (3-DOF only).
    fn lim_wider(&self, j: usize) -> [f32; 3] {
        let t = &self.core.lim[j];
        if t.set { [t.live[0] - t.base0[0], t.live[1] - t.base0[1], t.live[2] - t.base0[2]] } else { [0.0; 3] }
    }

    /// RDR2 core: fill the runtime table from the rig (0x2a21770, 0x2a217ec with the rig's own limits).
    pub fn lim_init(&mut self) {
        for j in 0..self.joints.len().min(32) {
            if !self.core.lim[j].set {
                let (l, three) = (loose(j, self.joints[j].limits), self.joints[j].three);
                let v = if three { std::array::from_fn(|k| (l[k].1 - l[k].0) * 0.5) } else { [l[0].0, l[0].1, 0.0] };
                self.core.lim[j] = JointLim { base0: v, base: v, work: v, live: v, set: true };
            }
        }
    }

    /// 0x2a4eaa0, 0x2a4e990, 0x2a4e9a4: this tick's half-ranges at least these (3-DOF).
    pub fn lim_raise(&mut self, j: usize, f: [f32; 3]) {
        self.lim_init();
        let t = &mut self.core.lim[j];
        for k in 0..3 {
            t.work[k] = t.work[k].max(f[k]);
        }
    }

    /// 0x2a4ea78: this tick's hinge ends at least these wide.
    pub fn lim_raise_hinge(&mut self, j: usize, lo: f32, hi: f32) {
        self.lim_init();
        let t = &mut self.core.lim[j];
        (t.work[0], t.work[1]) = (t.work[0].min(lo), t.work[1].max(hi));
    }

    /// A behaviour's floor: on a 3-DOF joint the half-ranges; on a hinge its larger end at least the first (the knee's
    /// minimum, 0x2a4ea78).
    pub fn lim_floor(&mut self, j: usize, f: [f32; 3]) {
        if j >= self.joints.len() {
            return;
        }
        if self.joints[j].three {
            self.lim_raise(j, f);
        } else {
            let l = loose(j, self.joints[j].limits)[0];
            if l.0.abs() > l.1.abs() { self.lim_raise_hinge(j, -f[0], l.1) } else { self.lim_raise_hinge(j, l.0, f[0]) }
        }
    }

    /// Slot +0xf8: wide open (a 3-DOF half-range 3.1316, a hinge's ends +-3.0416).
    pub fn lim_open(&mut self, j: usize) {
        if self.joints[j].three {
            self.lim_raise(j, [3.131593; 3]);
        } else {
            self.lim_raise_hinge(j, -3.041593, 3.041593);
        }
    }

    /// Slot +0x100: what this tick asked is dropped, back to the base.
    pub fn lim_to_base(&mut self, j: usize) {
        self.lim_init();
        let t = &mut self.core.lim[j];
        t.work = t.base;
    }

    /// Slot +0x110, not at once: the limits take in these angles (the joint's desired or actual, effector space) plus a
    /// margin. A 3-DOF joint's leans go by the ellipse: past its edge by a factor r, at a share t of the way round from
    /// lean1 to lean2, each half-range grows by r on its own side of the middle and by a share of r - 1 on the other's.
    pub fn lim_around(&mut self, j: usize, a: [f32; 3], margin: f32) {
        self.lim_init();
        if self.joints[j].three {
            let l = self.limits(j);
            let x: [f32; 3] = std::array::from_fn(|k| (a[k] - (l[2 * k] + l[2 * k + 1]) * 0.5).abs() + margin);
            let live = self.core.lim[j].live;
            let (mut h1, mut h2, h3) = (live[0], live[1], live[2].max(x[2]));
            let u = x[0] / h1;
            let r = (x[1] / h2).hypot(u);
            if r > 1.0 {
                let t = (u / r).acos() * 0.63662;
                let over = r - 1.0;
                h1 = if t <= 0.5 { h1 + 2.0 * over * h1 * t } else { r * h1 };
                h2 = if t >= 0.5 { h2 + 2.0 * over * h2 * (1.0 - t) } else { r * h2 };
            }
            self.lim_raise(j, [h1, h2, h3]);
        } else {
            let x = a[0].clamp(-std::f32::consts::PI, std::f32::consts::PI);
            self.lim_raise_hinge(j, x - margin, x + margin);
        }
    }

    /// Slot +0xf0 with 0 (configureConstraints): the base takes in what this tick asks, for good.
    pub fn lim_commit(&mut self, j: usize) {
        self.lim_init();
        let (b0, three) = (self.core.lim[j].base0, self.joints[j].three);
        let t = &mut self.core.lim[j];
        if three {
            for k in 0..3 {
                t.base[k] = t.work[k].max(b0[k]);
            }
        } else {
            (t.base[0], t.base[1]) = (t.work[0].min(b0[0]), t.work[1].max(b0[1]));
        }
    }

    /// Slot +0x108, once a step: the Rage joint's limits go to what this tick asked, at once when that widens them, at
    /// `rate(j)` per second when it narrows. A hinge changes only if the new ends make sense (low under high, inside a turn).
    pub fn lim_step(&mut self, dt: f32, rate: impl Fn(usize) -> f32) {
        self.lim_init();
        const MOST: f32 = 3.140593;
        for j in 0..self.joints.len().min(32) {
            let (three, r) = (self.joints[j].three, rate(j));
            let t = &mut self.core.lim[j];
            let to = |now: f32, want: f32, snap: bool| if snap || want == now { want } else if want < now { (now - r * dt).max(want) } else { (now + r * dt).min(want) };
            if t.live != t.work && t.work != t.base {
                crate::extras_core_rdr2::trace(|| format!("limit table: joint {j} from {:.3?} to {:.3?}, rig {:.3?}", t.live, t.work, t.base0));
            }
            if three {
                for k in 0..3 {
                    t.live[k] = to(t.live[k], t.work[k], t.live[k] < t.work[k]).clamp(0.0, MOST);
                }
            } else {
                let (lo, hi) = (to(t.live[0], t.work[0], t.work[0] < t.live[0]), to(t.live[1], t.work[1], t.work[1] > t.live[1]));
                if lo <= hi && lo.abs() <= std::f32::consts::PI && hi.abs() <= std::f32::consts::PI {
                    (t.live[0], t.live[1]) = (lo, hi);
                }
            }
        }
    }

    /// The knee and foot tweak at the head of the core's step (0x2a45344, ctx+0x1068): with a part but the lower legs and
    /// hands down on a level surface the feet's friction is 1, and a knee bent past 1.5 lets its foot go (ctx+0x1064). Without,
    /// with the leg down, one knee (his standing choice ctx+0x1069, or the bent one) may fold to ctx+0x105c. Returns the
    /// joint whose limit narrows at ctx+0x1060.
    pub fn knee_tweak(&mut self, hit: &[Option<Vec3>], touch: &[bool], actual: [f32; 2]) -> usize {
        let first = self.agent & 1 == 1;
        let knees = *KNEE();
        let slow = knees[if first { 0 } else { 1 }];
        let f = self.core.cfg.fall;
        if f.knee_tweak == 0.0 {
            return slow;
        }
        const OUT: u32 = 0xd80f0;
        let up = self.up;
        let rp = crate::extras_ysk_rdr2::rp;
        // RDR2's part numbers, ours under them
        let down = (0..21).filter(|r| OUT >> r & 1 == 0).any(|r| hit.get(rp(r)).copied().flatten().is_some_and(|n| n.dot(up) > 0.77));
        let (bent0, bent1) = (actual[0] < -1.5, actual[1] < -1.5);
        let bent = bent0 || bent1;
        let mut pick = first;
        if bent && ((!bent0 && first) || (!bent1 && !first)) {
            pick = !first;
        }
        if down {
            let cfg = &mut self.core.cfg;
            (cfg.fric[rp(5)], cfg.fric[rp(7)]) = (1.0, 1.0);
            if bent {
                cfg.fric[rp(if pick { 5 } else { 7 })] = f.foot_friction;
                cfg.fric_used = true;
                crate::extras_core_rdr2::trace_first("knee tweak: a bent knee's foot lets go");
            }
        } else {
            let mask: u32 = if pick { 0x32 } else { 0xc4 };
            if (0..21).filter(|r| mask >> r & 1 != 0).any(|r| touch.get(rp(r)).copied().unwrap_or(false)) {
                self.lim_raise_hinge(knees[if pick { 0 } else { 1 }], -f.knee_limit, 0.0);
                crate::extras_core_rdr2::trace_first("knee tweak: the knee may fold");
            }
        }
        slow
    }

    /// The end of the drive (slot +0x100 at the end of 0x2a45c20, 0x2a45fd4): every joint's asked limits back to its base.
    pub fn lim_settle(&mut self) {
        for t in self.core.lim.iter_mut().filter(|t| t.set) {
            t.work = t.base;
        }
    }

    pub fn set_relative(&mut self, j: usize, k: usize, x: f32) {
        let l = self.limits(j);
        let (lo, hi) = (l[2 * k], l[2 * k + 1]);
        self.eff[j].desired[k] = ((hi - lo) * 0.5).fma(x, (hi + lo) * 0.5);
    }

    pub fn set_stiffness(&mut self, s: f32, d: f32, mask: &str, ms: Option<f32>) {
        for i in Self::mask(mask) {
            self.eff[i].set_stiffness(s, d, ms);
        }
    }

    pub fn muscles(&mut self, root: Muscle) -> Vec<Muscle> {
        let mut m = vec![root; self.joints.len() + 1];
        for (i, (j, e)) in self.joints.iter().zip(self.eff.iter_mut()).enumerate() {
            let mid = |k: usize| (j.limits[k].0 + j.limits[k].1) * 0.5;
            let half = |k: usize| (j.limits[k].1 - j.limits[k].0) * 0.5;
            let rel = if j.three {
                let [l1, l2, tw] = targets(j, e);
                rotation([tw, l1, l2])
            } else {
                let d = e.aim(0).clamp((-half(0)).fma(delirium_data::kv!(486), mid(0)), half(0).fma(delirium_data::kv!(486), mid(0)));
                e.desired[0] = if e.offset[0] == 0.0 { d } else { d - e.offset[0] };
                rotation([d, 0.0, 0.0])
            };
            let s = e.strength.max(0.0).sqrt();
            e.damping = e.damping.max(MIN_DAMPING());
            m[i + 1] = Muscle { target: j.rage_parent * rel * j.rage_child.inverse(), stiffness: s,
                                damping: if s > 0.0 { e.damping / (2.0 * s) } else { 0.0 } };
        }
        m
    }

    pub fn rigid(&self, dt: f32, k: f32, most: f32) -> Vec<Rigid> {
        let mut out = vec![Rigid::default(); self.joints.len() + 1];
        for (i, (j, e)) in self.joints.iter().zip(&self.eff).enumerate() {
            let well = 1.0 - e.injury;
            let (strength, damping) = if self.rdr2 {
                ((e.strength * k * well * well).max(if e.injury > 0.0 { rdr2::LOOSE_MIN } else { 0.0 }), (e.damping * k * well).max(MIN_DAMPING()))
            } else {
                (e.strength * well * k, (e.damping * well * k).max(MIN_DAMPING()))
            };
            let x = damping.fma(delirium_data::kv!(65), strength) * e.muscle_stiffness * dt * 0.5;
            let blend = (delirium_data::kv!(35) * dt).max(x / (x + 1.0)).clamp(0.0, 1.0);
            let blend = if self.rdr2 { blend.min(rdr2::SOFTNESS_MAX) } else { blend };
            let target = match (j.three, self.rdr2) {
                (true, true) => targets_rdr2(j, e, self.lim_wider(i)),
                (true, false) => targets(j, e),
                _ => [e.aim(0), 0.0, 0.0],
            };
            let soft = std::array::from_fn(|k| {
                let (l, f) = (e.soft[k], j.three && j.flip[k]);
                let mid = if j.three { (j.limits[k].0 + j.limits[k].1) * 0.5 } else { 0.0 };
                SoftLimit { angle: if f { -l.angle } else { l.angle } - mid, dir: if f { -l.dir } else { l.dir }, ..l }
            });
            out[i + 1] = Rigid { frame: j.rage_parent, child: j.rage_child, limits: self.lim_rage(i), max: j.max_torque.map(|m| m * k * most), blend, hinge: !j.three, desired: e.aim(0), jc: j.rigid,
                                target, gain: [strength, damping], soft,
                                cap: self.rdr2.then_some(if j.three { rdr2::THREE_CAP } else { rdr2::HINGE_CAP }) };
        }
        out
    }
}

