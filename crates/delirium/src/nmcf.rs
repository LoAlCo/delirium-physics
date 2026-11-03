use crate::prelude::*;
use crate::Vd;
use crate::Vx;

use crate::collision::Collision;
use crate::nmhl::HeadLook;
use crate::nmrs::{Character, ANKLE, HIP, KNEE, NECK, SPINE};
use crate::nmshot::{limb, Ctx, Limb};
use crate::nmst::SpineTwist;
use crate::ragdoll::Obstacle;
use crate::nmdbm::{sdot, slen};
use crate::rigid::nrm;
use crate::Fma;

pub(crate) fn cast(world: &Collision, obstacles: &[Obstacle], from: Vec3, dir: Vec3, len: f32) -> Option<(f32, Vec3, Vec3)> {
    let mut hit = world.cast(from, dir, len).map(|(t, n)| (t, n, Vec3::ZERO));
    for ob in obstacles {
        let r = ob.rot;
        if let Some((t, n)) = ray_box(r.inverse() * (from - ob.centre), r.inverse() * dir, ob.half).filter(|h| h.0 <= len && hit.is_none_or(|b| h.0 < b.0)) {
            hit = Some((t, r * n, ob.vel_at(from + dir * t)));
        }
    }
    hit
}

fn ray_box(o: Vec3, d: Vec3, half: Vec3) -> Option<(f32, Vec3)> {
    let (mut near, mut far, mut n) = (0.0f32, f32::MAX, Vec3::ZERO);
    for k in 0..delirium_data::kv!(0 int) {
        if d[k].abs() < delirium_data::kv!(244) {
            if o[k].abs() > half[k] {
                return None;
            }
            continue;
        }
        let (a, b) = ((-half[k] - o[k]) / d[k], (half[k] - o[k]) / d[k]);
        let (t0, t1) = (a.min(b), a.max(b));
        if t0 > near {
            near = t0;
            n = Vec3::ZERO;
            n[k] = -d[k].signum();
        }
        far = far.min(t1);
        if near > far {
            return None;
        }
    }
    (near > 0.0).then_some((near, n))
}

#[allow(non_snake_case)]
#[inline]
fn SPINE3() -> usize { delirium_data::kv!(81 int) }
#[allow(non_snake_case)]
#[inline]
fn NECK_PART() -> usize { delirium_data::kv!(113 int) }
#[allow(non_snake_case)]
#[inline]
fn HEAD() -> usize { delirium_data::kv!(55 int) }
#[allow(non_snake_case)]
#[inline]
pub(crate) fn REACH() -> f32 { delirium_data::kv!(119) }
#[allow(non_snake_case)]
#[inline]
pub(crate) fn PROBE() -> f32 { delirium_data::kv!(245 f32) }
pub(crate) const FALL_G: f32 = 1.0;
pub(crate) const SOFTEN: f32 = 0.5;

#[derive(Clone, Copy, Debug)]
pub struct Params {
    pub legs_stiffness: f32,
    pub torso_stiffness: f32,
    pub arms_stiffness: f32,
    pub forward_max_arm_offset: f32,
    pub backwards_min_arm_offset: f32,
    pub z_axis_spin_reduction: f32,
    pub use_head_look: bool,
}

impl Default for Params {
    fn default() -> Params {
        Params { legs_stiffness: delirium_data::kv!(35), torso_stiffness: delirium_data::kv!(137), arms_stiffness: delirium_data::kv!(30), forward_max_arm_offset: delirium_data::kv!(140),
                 backwards_min_arm_offset: -delirium_data::kv!(17), z_axis_spin_reduction: 0.0, use_head_look: true }
    }
}

#[derive(Clone, Copy, Debug)]
struct Arm {
    vel: Vec3,
    k: f32,
    clav_lean1: f32,
    clav_lean2: f32,
    elbow: f32,
    left: bool,
}

#[derive(Clone, Debug)]
pub struct CatchFall {
    pub p: Params,
    dir: Vec3,
    forward: f32,
    k: f32,
    upright: f32,
    leg: [f32; 2],
    lean2: f32,
    arm: [Arm; 2],
    touched: bool,
    pub active: bool,
    /// RDR2's catchFall, with the RDR2 core
    rd: Option<Box<Rdr2Cf>>,
}

impl CatchFall {
    pub fn start(p: Params, c: &mut Ctx, head: &mut HeadLook, st: &mut SpineTwist) -> CatchFall {
        if c.ch.rdr2 {
            return CatchFall::start_rdr2(p, c, head, st);
        }
        let ch = &mut *c.ch;
        for (e, j) in ch.eff.iter_mut().zip(&ch.joints) {
            e.reset(j);
            e.desired = [0.0; 3];
        }
        let (k, h) = (p.torso_stiffness, 0.5);
        ch.set_stiffness(k, h, "us", None);
        if p.use_head_look {
            ch.set_stiffness(k, h, "un", None);
        }
        ch.set_stiffness(p.arms_stiffness - 1.0, 1.0, "ua", None);
        ch.set_stiffness(p.legs_stiffness, 0.5, "lb", None);
        for i in 0..2 {
            ch.eff[limb(i).e[delirium_data::kv!(0 int)]].strength = delirium_data::kv!(31);
        }
        for e in ch.eff.iter_mut() {
            e.oppose_gravity = 1.0;
        }
        let leg = [0.5 + 0.5 * ch.random(), 0.5 + 0.5 * ch.random()];
        let lean2 = ch.random().fma(delirium_data::kv!(60), -delirium_data::kv!(78));
        for i in 0..2 {
            ch.eff[(*ANKLE())[i]].desired[0] = -leg[i];
            ch.eff[(*HIP())[i]].desired[0] = leg[i];
            ch.eff[(*KNEE())[i]].desired[0] = leg[i] * -2.0;
        }
        let arm = [Arm::init(ch, true), Arm::init(ch, false)];
        *st = SpineTwist::default();
        (st.stiffness, st.arms) = (p.torso_stiffness, true);
        st.activate(ch);
        if p.use_head_look {
            *head = HeadLook { stiffness: p.torso_stiffness, ..HeadLook::default() };
            head.activate(ch, c.b);
        }
        CatchFall { p, dir: Vec3::ZERO, forward: 0.0, k: 1.0, upright: 0.0, leg, lean2, arm, touched: false, active: true, rd: None }
    }

    /// RDR2's: something of his other than the lower legs and feet has touched the ground (false for GTA IV's).
    pub fn on_ground(&self) -> bool {
        self.rd.as_ref().is_some_and(|r| r.on_ground)
    }

    pub fn deactivate(&mut self, head: &mut HeadLook, st: &mut SpineTwist) {
        if self.active {
            self.active = false;
            if self.p.use_head_look {
                head.deactivate();
            }
            st.deactivate();
        }
    }

    pub fn tick(&mut self, c: &mut Ctx, world: &Collision, obstacles: &[Obstacle], head: &mut HeadLook, st: &mut SpineTwist, dt: f32) {
        if !self.active {
            return;
        }
        if self.rd.is_some() {
            return self.tick_rdr2(c, world, obstacles, head, st, dt);
        }
        let p = self.p;
        if p.z_axis_spin_reduction > 0.0 {
            spin_reduction(c, p.z_axis_spin_reduction);
        }
        let (fl, fr) = (self.arm[0].k.max(delirium_data::kv!(24)), self.arm[1].k.max(delirium_data::kv!(24)));
        let ke = c.b.doll.kinetic();
        self.k = (((2.0 * ke).sqrt().fma(2.0, -self.k) * dt).fma(delirium_data::kv!(64), self.k)).max(delirium_data::kv!(60)).min(1.0);
        let k = self.k;
        let weak = fl.min(fr);
        if weak == delirium_data::kv!(24) {
            c.out.send(delirium_data::kv!(54 int), "catchFall", 0.0);
        }
        if weak < delirium_data::kv!(28) && k < 1.0 {
            c.out.send(delirium_data::kv!(55 int), "catchFall", 0.0);
        }
        let ch = &mut *c.ch;
        ch.set_stiffness(p.torso_stiffness * k, 0.5, "us", None);
        if p.use_head_look {
            ch.set_stiffness(p.torso_stiffness * k, 0.5, "un", None);
        }
        ch.set_stiffness(fl * p.arms_stiffness, 1.0, "ul", None);
        ch.set_stiffness(fr * p.arms_stiffness, 1.0, "ur", None);
        ch.set_stiffness(p.legs_stiffness * k, 0.5, "lb", None);
        for (i, f) in [fl, fr].into_iter().enumerate() {
            ch.eff[limb(i).e[delirium_data::kv!(0 int)]].set_stiffness((p.arms_stiffness + delirium_data::kv!(195)) * f * 0.5, 1.0, None);
            ch.eff[(*ANKLE())[i]].set_stiffness(delirium_data::kv!(31), 1.0, None);
        }
        let under = (self.arm[0].vel + self.arm[1].vel) * 0.5;
        let s3 = c.b.part(SPINE3());
        let (d, up) = (s3.v - under, c.up);
        let du = sdot(up, d, b"yzx");
        self.dir = nrm(up * ((-FALL_G).fma(delirium_data::kv!(246), du) - du) + d);
        let m = c.m;
        self.forward = -sdot(m.c, self.dir, b"yzx");
        let back = -sdot(m.c, c.vel - under, b"yzx");
        self.upright = sdot(m.b, up, b"zyx").max(0.0).min(1.0);
        let on_back = sdot(m.c, up, b"zyx").max(0.0).min(1.0);
        let ch = &mut *c.ch;
        for i in 0..2 {
            ch.eff[(*ANKLE())[i]].desired[0] = -self.leg[i];
        }
        let curl = (back * -2.0).max(0.0).min(1.0).fma(self.upright, -(self.forward * 0.5)).max(0.0).min(1.0);
        for j in (*SPINE()) {
            ch.eff[j].desired[0] = curl * k * delirium_data::kv!(60);
            ch.eff[j].desired[1] = self.lean2;
        }
        if !st.twisting {
            let decay = 1.0 / dt.fma(delirium_data::kv!(21), 1.0);
            for j in (*SPINE()) {
                ch.eff[j].desired[2] *= decay;
            }
        }
        for i in 0..2 {
            ch.eff[(*HIP())[i]].blend_to_zero(1.0);
        }
        for i in 0..2 {
            ch.eff[(*HIP())[i]].desired[0] = (self.leg[i] + curl) * k;
        }
        let bend = (on_back + 1.0).fma(k, -on_back);
        for i in 0..2 {
            ch.eff[(*KNEE())[i]].desired[0] = self.leg[i] * bend * -2.0;
        }
        let look = c.b.part(HEAD()).m.d + self.dir;
        if p.use_head_look {
            (head.pos, head.vel) = (look, s3.v);
            head.stiffness = (p.torso_stiffness + 2.0) * if fl >= fr { fl } else { fr };
        }
        st.pos = look;
        for i in 0..2 {
            let mut a = self.arm[i];
            a.tick(self, c, world, obstacles, st.twist, i, dt);
            self.arm[i] = a;
        }
        let arms: Vec<usize> = (0..2).flat_map(|i| limb(i).p).collect();
        if c.b.doll.collided(&arms) {
            self.touched = true;
        }
        if self.touched {
            let n = c.b.pmat(NECK_PART()).m;
            let v = -self.dir;
            for j in (*NECK()) {
                c.ch.eff[j].desired[0] = -sdot(n.c, v, b"xyz");
                c.ch.eff[j].desired[1] = -sdot(n.a, v, b"xyz");
            }
        }
    }
}

pub(crate) fn spin_reduction(c: &mut Ctx, k: f32) {
    let t = spin_torque(c.up, c.angvel, c.b.doll.mass(), k);
    for k in [0, delirium_data::kv!(22 int), delirium_data::kv!(23 int), delirium_data::kv!(104 int), delirium_data::kv!(81 int)] {
        c.out.torque(k, t);
    }
}

fn spin_torque(up: Vec3, w: Vec3, mass: f32, k: f32) -> Vec3 {
    let along = up.x.mul_add(w.x, w.y.mul_add(up.y, w.z * up.z));
    crate::nmdbm::vnorm(up) * (mass * k.clamp(0.0, 1.0) * along * -delirium_data::kv!(60))
}

impl Arm {
    fn init_none(left: bool) -> Arm {
        Arm { vel: Vec3::ZERO, k: 1.0, clav_lean1: 0.0, clav_lean2: 0.0, elbow: 0.0, left }
    }

    fn init(ch: &mut Character, left: bool) -> Arm {
        let mut r = || (ch.random_bits() & delirium_data::kv!(190 int)) as f32;
        let clav_lean1 = r().fma(delirium_data::kv!(247 f32), -delirium_data::kv!(116));
        let clav_lean2 = r() * delirium_data::kv!(248 f32);
        let elbow = r().fma(delirium_data::kv!(249 f32), delirium_data::kv!(74));
        Arm { vel: Vec3::ZERO, k: 1.0, clav_lean1, clav_lean2, elbow, left }
    }

    fn tick(&mut self, cf: &CatchFall, c: &mut Ctx, world: &Collision, obstacles: &[Obstacle], twist: f32, i: usize, dt: f32) {
        let l: Limb = limb(i);
        let m = c.m;
        let mut s = m.b.vx(c.up);
        if sdot(s, s, b"zxy") > delirium_data::kv!(250) {
            s = nrm(s);
        }
        let out = if cf.forward > 0.0 {
            nrm(m.b + m.b.vx(s)) * cf.upright.max(0.0).min(1.0).fma(delirium_data::kv!(119), delirium_data::kv!(116))
        } else {
            m.b
        };
        let p = cf.p;
        let from = out.mul_add(Vec3::splat(cf.forward.max(p.backwards_min_arm_offset).min(p.forward_max_arm_offset)), c.b.joint_pos(l.e[1]));
        let to = cf.dir.mul_add(Vec3::splat(PROBE()), from);
        let hand = c.b.part(l.p[delirium_data::kv!(0 int)]).m.d;
        let dir = (to - from).normalize_or_zero();
        #[allow(unused_mut)]
        let mut hit = cast(world, obstacles, from, dir, PROBE()).map(|(t, n, v)| (from + dir * t, n, v));
        let (at, normal) = match hit {
            Some((a, n, _)) => (a, n),
            None => (to, c.up),
        };
        self.vel = hit.map_or(Vec3::ZERO, |h| h.2);
        let touching = c.b.doll.collided(&l.p);
        self.k = if touching { cf.k.max((-SOFTEN).fma(dt, self.k)) } else { dt.fma(2.0, self.k).min((delirium_data::kv!(5) * cf.k).min(1.0)) };
        let shoulder = c.b.joint_pos(l.e[1]);
        let aim = if touching { (at + hand) * 0.5 } else { at };
        let mut d = aim - shoulder;
        let len = slen(d, b"zyx");
        if len > REACH() {
            d *= REACH() / len;
        } else if len < delirium_data::kv!(116) {
            d *= delirium_data::kv!(116) / (len + delirium_data::kv!(72));
        }
        let target = shoulder + d;
        let side = if self.left { 1.0 } else { -1.0 };
        let yaw = 1.0 - cf.forward;
        let (t, lead, vel) = match (touching, hit.is_some()) {
            (true, true) => (target, 0.0, Vec3::ZERO),
            (true, false) => (hand, 0.0, Vec3::ZERO),
            (false, _) => (target, 0.5, c.b.part(l.p[0]).v),
        };
        crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[delirium_data::kv!(0 int)], t, yaw, lead, side, 1.0, vel, Some((delirium_data::kv!(15), delirium_data::kv!(116))));
        let ch = &mut *c.ch;
        if cf.forward < -delirium_data::kv!(24) {
            let e = &mut ch.eff[l.e[2]].desired[0];
            *e = e.min(self.elbow);
        }
        let clav = l.e[0];
        ch.eff[clav].desired[2] = 0.0;
        if cf.forward >= 0.0 && touching {
            ch.eff[clav].desired[1] = 0.0;
            ch.eff[(*SPINE())[delirium_data::kv!(0 int)]].desired[0] = -1.0;
            ch.eff[(*SPINE())[2]].desired[0] = -1.0;
            ch.eff[clav].desired[2] = ch.actual(clav, c.b.frames)[2];
        } else {
            let saved = ch.eff[clav].desired[0];
            clav_follow(ch, clav, l.e[1]);
            if twist.abs() < 2.0 {
                ch.eff[clav].desired[0] = saved;
            } else {
                ch.eff[clav].desired = [self.clav_lean1, self.clav_lean2, delirium_data::kv!(24)];
            }
        }
        if hit.is_some() {
            crate::nmshot::Shot::wrist(c, l, target, normal, side);
        } else {
            c.ch.eff[l.e[delirium_data::kv!(0 int)]].desired = [delirium_data::kv!(24), -0.5, 1.0];
        }
    }
}

pub(crate) fn clav_follow(ch: &mut Character, clav: usize, shoulder: usize) {
    let (cl, sl) = (ch.limits(clav), ch.limits(shoulder));
    let [l1, l2, _] = ch.eff[shoulder].desired;
    ch.eff[clav].desired[0] = l1 * if l1 >= 0.0 { cl[1] / sl[1] } else { cl[0] / sl[0] };
    ch.eff[clav].desired[1] = l2 * if l2 >= 0.0 { cl[delirium_data::kv!(0 int)] / sl[delirium_data::kv!(0 int)] } else { cl[2] / sl[2] };
}


// RDR2's catchFall (with the RDR2 core), from a study of RDR2's own catchFall: GTA IV's catchFall on RDR2's terms.
// Its hands-and-knees and roll modes, the slope block and the friction scales stay off here, as RDR2's defaults leave
// them (nothing in its behaviours turns them on); the both-arms-down event, the teeter edge clamp and the big-spin
// stiffness aren't here.

/// RDR2: k may only fall after this long on the ground (s), and moves at this rate toward 2 sqrt(2E).
const R_K_TIME: f32 = 10.0;
const R_K_RATE: f32 = 2.0;
/// RDR2: the spine's lean1 gain with the fall's backwardness.
const R_SPINE_LEAN: f32 = 0.5;
/// RDR2: the probe reaches 1.5 arm lengths plus the speed (at most 10 m/s's worth).
const R_PROBE_ARMS: f32 = 1.5;

#[derive(Clone, Copy, Debug, Default)]
struct RArm {
    k: f32,
    clav_lean1: f32,
    clav_lean2: f32,
    elbow: f32,
    elbow_on: bool,
    target: Vec3,
    normal: Vec3,
    swing: (f32, f32),
}

#[derive(Clone, Debug)]
pub(crate) struct Rdr2Cf {
    /// time since the body first touched the ground
    time: f32,
    on_ground: bool,
    knee_amount: [f32; 2],
    hip_gain: [f32; 2],
    spine_lean2: f32,
    knee: [f32; 2],
    /// the arm's length
    len: f32,
    reach_scale: f32,
    arm: [RArm; 2],
    forward: f32,
    upright: f32,
    dir: Vec3,
    roll: f32,
}

impl Rdr2Cf {
    fn draw_legs(&mut self, ch: &mut Character) {
        self.knee_amount = [ch.random() * 1.8, ch.random() * 1.8];
        self.hip_gain = [ch.random() * 0.31 - 0.3, ch.random() * 0.31 - 0.3];
    }

    fn arm_init(ch: &mut Character, len: f32) -> RArm {
        let (clav_lean1, clav_lean2, elbow) = (ch.random() * 0.4 - 0.4, ch.random() * 0.5, 0.7 + ch.random() * 0.6);
        let _ = len;
        RArm { k: 1.0, clav_lean1, clav_lean2, elbow, elbow_on: false, target: Vec3::ZERO, normal: Vec3::Z, swing: (0.0, 0.0) }
    }
}

impl CatchFall {
    fn start_rdr2(p: Params, c: &mut Ctx, head: &mut HeadLook, st: &mut SpineTwist) -> CatchFall {
        let ch = &mut *c.ch;
        for (e, j) in ch.eff.iter_mut().zip(&ch.joints) {
            e.reset(j);
        }
        ch.set_stiffness(p.torso_stiffness, 0.5, "us", None);
        if p.use_head_look {
            ch.set_stiffness(p.torso_stiffness, 0.5, "un", None);
        }
        ch.set_stiffness(p.arms_stiffness - 1.0, 1.0, "ua", None);
        ch.set_stiffness(p.legs_stiffness, 0.5, "lb", None);
        for i in 0..2 {
            ch.eff[limb(i).e[3]].strength = 10.0;
        }
        for e in ch.eff.iter_mut() {
            e.oppose_gravity = 1.0;
        }
        let mut r = Rdr2Cf { time: 0.0, on_ground: false, knee_amount: [0.0; 2], hip_gain: [0.0; 2], spine_lean2: 0.0, knee: [0.0; 2],
                             len: 0.0, reach_scale: 1.0, arm: [RArm::default(); 2], forward: 0.0, upright: 0.0, dir: Vec3::ZERO, roll: 0.0 };
        r.draw_legs(ch);
        r.spine_lean2 = ch.random() * 0.2 - 0.1;
        for i in 0..2 {
            r.knee[i] = ch.actual((*KNEE())[i], c.b.frames)[0];
            ch.eff[(*KNEE())[i]].desired[0] = r.knee[i];
            ch.eff[(*ANKLE())[i]].desired[0] = 0.0;
            ch.eff[(*HIP())[i]].desired[0] = -0.7 * r.knee[i];
        }
        // the arm's length: shoulder to elbow to wrist
        let l = limb(0);
        r.len = (c.b.joint_pos(l.e[2]) - c.b.joint_pos(l.e[1])).length() + (c.b.joint_pos(l.e[3]) - c.b.joint_pos(l.e[2])).length();
        r.arm = [Rdr2Cf::arm_init(ch, r.len), Rdr2Cf::arm_init(ch, r.len)];
        *st = SpineTwist::default();
        (st.stiffness, st.arms) = (p.torso_stiffness, true);
        st.activate(ch);
        if p.use_head_look {
            *head = HeadLook { stiffness: p.torso_stiffness, ..HeadLook::default() };
            head.activate(ch, c.b);
        }
        // each hand's reach start swings round a circle
        let a = ch.random() * 0.4777 * r.len;
        for i in 0..2 {
            r.arm[i].swing = (a + ch.random() * (0.79624 * r.len - a), ch.random() * 3.1 - 1.0);
        }
        CatchFall { p, dir: Vec3::ZERO, forward: 0.0, k: 1.0, upright: 0.0, leg: [0.0; 2], lean2: r.spine_lean2,
                    arm: [Arm::init_none(true), Arm::init_none(false)], touched: false, active: true, rd: Some(Box::new(r)) }
    }

    fn tick_rdr2(&mut self, c: &mut Ctx, world: &Collision, obstacles: &[Obstacle], head: &mut HeadLook, st: &mut SpineTwist, dt: f32) {
        let p = self.p;
        let Some(mut r) = self.rd.take() else { return };
        let up = c.up;
        // on the ground: any part but the lower legs and feet touching
        let legs: Vec<usize> = (0..2).flat_map(|i| [(*KNEE())[i] + 1, (*ANKLE())[i] + 1]).collect();
        let others: Vec<usize> = (0..c.b.frames.len()).filter(|k| !legs.contains(k)).collect();
        r.on_ground |= c.b.doll.collided(&others);
        if r.on_ground {
            r.time += dt;
        }
        // the ground's normal: the arms' probes' mean once down
        let ground = if r.on_ground { nrm(r.arm[0].normal + r.arm[1].normal) } else { up };
        if p.z_axis_spin_reduction > 0.0 {
            spin_reduction(c, p.z_axis_spin_reduction);
        }
        let (kl, kr) = (r.arm[0].k.max(0.0), r.arm[1].k.max(0.0));
        let goal = (2.0 * c.b.doll.kinetic()).sqrt() * 2.0;
        let mut step = (goal - self.k) * R_K_RATE * dt;
        if r.time > R_K_TIME {
            step = -step.abs();
        }
        self.k = (self.k + step).clamp(0.0, 1.0);
        let k = self.k;
        let weak = kl.min(kr);
        if weak < 0.30001 {
            c.out.send(delirium_data::kv!(54 int), "catchFall", 0.0);
        }
        if weak < 0.9 && k < 1.0 {
            c.out.send(delirium_data::kv!(55 int), "catchFall", 0.0);
        }
        let ch = &mut *c.ch;
        let torso = p.torso_stiffness * k;
        ch.set_stiffness(torso, 0.5, "us", Some(0.5));
        ch.set_stiffness((p.arms_stiffness * kl).max(3.0).min(20.0), 1.0, "ul", Some(0.5));
        ch.set_stiffness((p.arms_stiffness * kr).max(3.0).min(20.0), 1.0, "ur", Some(0.5));
        // (one side's lerp is scaled by k twice in RDR2)
        ch.set_stiffness((p.legs_stiffness * 0.25).max(p.legs_stiffness * k * k).max(4.0), 0.5, "ll", None);
        ch.set_stiffness((p.legs_stiffness * 0.25).max(p.legs_stiffness * k).max(4.0), 0.5, "lr", None);
        for (i, ka) in [(0, kl), (1, kr)] {
            ch.eff[limb(i).e[3]].set_stiffness((6.5 + 0.5 * p.arms_stiffness) * ka.max(0.6), 1.0, Some(10.0));
            ch.eff[(*ANKLE())[i]].set_stiffness(10.0, 1.0, None);
        }
        let m = c.m;
        r.dir = nrm(c.vel - up * (0.5 * 9.8));
        r.forward = -m.c.dot(r.dir);
        let s3 = c.b.pmat(SPINE3());
        let across = s3.m.a.vx(up);
        r.roll = (-s3.m.b.dot(across)).atan2(s3.m.c.dot(across));
        r.upright = m.b.dot(up).clamp(0.0, 1.0);
        (self.dir, self.forward, self.upright) = (r.dir, r.forward, r.upright);
        let f = r.forward;
        let ch = &mut *c.ch;
        for i in 0..2 {
            ch.eff[(*ANKLE())[i]].desired[0] = 0.0;
        }
        for j in (*SPINE()) {
            ch.eff[j].desired[0] = R_SPINE_LEAN * (-f).clamp(0.0, 1.0);
            ch.eff[j].desired[1] = r.spine_lean2;
        }
        let arms: Vec<usize> = (0..2).flat_map(|i| limb(i).p).collect();
        if f > -0.2 && c.b.doll.collided(&arms) {
            let t = (2.5 * f.min(0.2) + 0.5).clamp(0.0, 1.0);
            let now = c.ch.actual((*SPINE())[3], c.b.frames)[0];
            for j in [(*SPINE())[3], (*SPINE())[2]] {
                c.ch.eff[j].desired[0] = now + (-0.4 - now) * t;
            }
        }
        let ch = &mut *c.ch;
        if !st.twisting {
            let decay = 1.0 / (12.0f32).mul_add(dt, 1.0);
            for j in (*SPINE()) {
                ch.eff[j].desired[2] *= decay;
            }
        }
        for i in 0..2 {
            let h = &mut ch.eff[(*HIP())[i]];
            h.blend_to_zero(1.0);
            h.desired[0] = -0.7 * r.knee[i];
            h.desired[1] = m.c.dot(ground).abs() * r.hip_gain[i];
        }
        let d = m.c.dot(ground);
        let most = 1.0 - 0.7 * d.clamp(0.0, 1.0);
        for i in 0..2 {
            // rate limited at 10^4 rad/s: at once
            r.knee[i] = -r.knee_amount[i].min(most);
            ch.eff[(*KNEE())[i]].desired[0] = r.knee[i];
        }
        for i in 0..2 {
            let a = r.arm_rdr2(self, c, world, obstacles, st.twist, i);
            r.arm[i] = a;
        }
        let look = c.b.part(HEAD()).m.d + r.dir;
        let chest = c.b.part(SPINE3());
        if p.use_head_look {
            (head.pos, head.vel, head.stiffness) = (look, chest.v, 2.0 + torso);
        } else if r.on_ground {
            let n = c.b.pmat(NECK_PART()).m;
            let away = -r.dir;
            for j in (*NECK()) {
                c.ch.eff[j].desired[0] = -n.c.dot(away);
                c.ch.eff[j].desired[1] = -n.a.dot(away).abs() * n.a.dot(away);
            }
        }
        st.pos = look;
        self.rd = Some(r);
    }
}

impl Rdr2Cf {
    /// One arm: its reach start, the probe along the fall, the hand's target and IK, the clavicle and the hand's normal.
    fn arm_rdr2(&mut self, cf: &CatchFall, c: &mut Ctx, world: &Collision, obstacles: &[Obstacle], twist: f32, i: usize) -> RArm {
        let p = cf.p;
        let mut a = self.arm[i];
        let l = limb(i);
        let (m, up, f) = (c.m, c.up, self.forward);
        let side = if i == 0 { 1.0 } else { -1.0 };
        let fb = (0.5 - f).max(0.0);
        let shoulder = c.b.joint_pos(l.e[1]);
        // the reach start
        let mut s = m.b.vx(up);
        if s.length_squared() > 1e-10 {
            s = nrm(s);
        }
        let out = if f > 0.0 { nrm(m.b + m.b.vx(s)) * (0.6 * self.upright + 0.4) } else { m.b };
        let mut start = shoulder + out * f.clamp(p.backwards_min_arm_offset, p.forward_max_arm_offset) + up * 0.4;
        let s3 = c.b.pmat(SPINE3()).m;
        let ch3 = nrm(s3.c - up * s3.c.dot(up));
        let g = 1.0 - ((s3.a.dot(up).abs() - 0.8) / 0.1).clamp(0.0, 1.0);
        let th = if i == 0 { self.roll } else if self.roll < 0.0 { self.roll + std::f32::consts::PI } else { std::f32::consts::PI - self.roll };
        let h = if !(-0.8..=0.6).contains(&th) {
            0.0
        } else if th < -0.7 {
            0.3 + 0.3 * (-0.7 - th) / 0.1
        } else if th > 0.5 {
            0.3 * (1.0 - 10.0 * (th - 0.5))
        } else {
            0.3
        };
        let flat = c.b.pmat(l.p[1]).m.b * -0.2 - ch3 * (g * h);
        start += Vec3::new(flat.x, flat.y, 0.0);
        // the swing circle
        let e = 1.0 - 5.0 * ((-(a.target - shoulder).z).clamp(0.3, 0.5) - 0.3);
        if e > 0.8 {
            self.reach_scale = 0.96;
        }
        let phi = 0.5 - 0.5 * f;
        let (x, y) = if i == 0 {
            (a.swing.0 * (a.swing.1 - phi).cos(), -a.swing.0 * (a.swing.1 - phi).sin())
        } else {
            (a.swing.0 * (a.swing.1 - phi).cos(), a.swing.0 * (a.swing.1 - phi).sin())
        };
        let swing = (s3.a * x - s3.b * y) * e;
        start += Vec3::new(swing.x, swing.y, 0.0);
        // the probe
        let chest_v = c.b.part(SPINE3()).v;
        let len = R_PROBE_ARMS * self.len + chest_v.length().min(10.0);
        let hit = cast(world, obstacles, start, self.dir, len);
        let (mut target, normal, vel) = match hit {
            Some((t, n, v)) => (start + self.dir * t, n, v),
            None => (start + self.dir * len, up, Vec3::ZERO),
        };
        a.normal = normal;
        a.k = cf.k;
        let reach = self.len * self.reach_scale * (0.7 + 0.3 * (1.0 - (self.upright - 0.5).clamp(0.0, 1.0)));
        // onto the sphere the arm reaches
        let rr = (target - shoulder).length().min(reach);
        let seg = target - start;
        let t = if seg.length_squared() > 0.0 { ((shoulder - start).dot(seg) / seg.length_squared()).clamp(0.0, 1.0) } else { 0.0 };
        let q = start + seg * t;
        let dq = (q - shoulder).length();
        if dq < rr {
            target = q + nrm(seg) * (rr * rr - dq * dq).sqrt();
        }
        // out of the body's plane
        let chest = c.b.pmat(SPINE3());
        let spine2 = c.b.pmat((*SPINE())[2] + 1);
        let plane = chest.m.d - chest.m.c * 0.1;
        let n = chest.m.b;
        let depth = n.dot(target - plane);
        let sh = nrm(shoulder - chest.m.d);
        let lam = sh.dot(target - shoulder).clamp(0.3 * self.len, 0.8 * self.len);
        let pushed = depth > 0.0;
        if pushed {
            target -= n * (depth * (1.0 - (lam - 0.3 * self.len) / (0.5 * self.len)));
        }
        let mut u = target - shoulder;
        let ul = u.length();
        if ul > reach {
            u *= reach / ul;
        } else if ul < 0.4 && u.dot(normal) < 0.0 && !pushed {
            u *= 0.4 / (ul + 1e-4);
        }
        if u.length() > reach {
            u *= reach / u.length();
        }
        target = shoulder + u;
        a.target = target;
        let wb = ((1.5 * (nrm(spine2.m.d - shoulder).dot(u) / (spine2.m.d - shoulder).length() - 1.0)).clamp(0.0, 1.0) + fb).clamp(0.0, 1.5);
        crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[3], target, fb, 0.0, side, 1.0, vel, Some((delirium_data::kv!(15), delirium_data::kv!(116))));
        a.elbow_on = f < -0.3 || (a.elbow_on && f < -0.2);
        if a.elbow_on {
            let e = &mut c.ch.eff[l.e[2]].desired[0];
            *e = e.min(a.elbow);
        }
        let touching = c.b.doll.collided(&l.p);
        let clav = l.e[0];
        if f < 0.0 || !touching {
            c.ch.eff[clav].desired = if twist.abs() >= 2.0 { [a.clav_lean1, a.clav_lean2, 0.3] } else { [0.0, 0.0, c.ch.eff[clav].desired[2]] };
        } else {
            c.ch.eff[clav].desired = [0.0, 0.0, c.ch.actual(clav, c.b.frames)[2]];
        }
        if a.k < 0.3 && f < 0.0 {
            c.ch.eff[clav].desired = [0.0; 3];
        }
        // the hand's normal: from the forearm's axes toward the hit's as the reach opens
        let fa = c.b.pmat(l.p[2]).m;
        let af = fa.a * side;
        let hn = if af.dot(normal) >= -0.9 { nrm(-fa.c + (af + fa.c) * (wb - 0.5).clamp(0.0, 1.0)) } else { af };
        let hn = if hit.is_some() && (c.b.part(l.p[3]).m.d - target).length() < 0.1 { normal } else { hn };
        crate::nmshot::Shot::wrist(c, l, target, hn, side);
        a
    }
}
