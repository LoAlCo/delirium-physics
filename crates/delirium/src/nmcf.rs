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

pub(crate) fn ray_box(o: Vec3, d: Vec3, half: Vec3) -> Option<(f32, Vec3)> {
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

/// RDR2's IK clavicle mode 3 (request +0xa4, 0x2a3fdd0): the clavicle's leans are 0 and what the shoulder asks past its lean1
/// limit goes to the clavicle's twist, negated (no overflow leaves the twist to lower requests).
pub(crate) fn clav_overflow(ch: &mut Character, clav: usize, shoulder: usize) {
    let sl = ch.limits(shoulder);
    let l1 = ch.eff[shoulder].desired[0];
    let over = if l1 - sl[1] > 0.0 { l1 - sl[1] } else if l1 - sl[0] < 0.0 { l1 - sl[0] } else { 0.0 };
    if over != 0.0 {
        ch.eff[clav].desired[2] = -over;
    }
    (ch.eff[clav].desired[0], ch.eff[clav].desired[1]) = (0.0, 0.0);
}

pub(crate) fn clav_follow(ch: &mut Character, clav: usize, shoulder: usize) {
    let (cl, sl) = (ch.limits(clav), ch.limits(shoulder));
    let [l1, l2, _] = ch.eff[shoulder].desired;
    ch.eff[clav].desired[0] = l1 * if l1 >= 0.0 { cl[1] / sl[1] } else { cl[0] / sl[0] };
    ch.eff[clav].desired[1] = l2 * if l2 >= 0.0 { cl[delirium_data::kv!(0 int)] / sl[delirium_data::kv!(0 int)] } else { cl[2] / sl[2] };
}


// RDR2's catchFall (with the RDR2 core), from a study of RDR2's own catchFall: GTA IV's catchFall on RDR2's terms.
// The balancer isn't kept beside it here, so the mode's hand-over of the legs to the balancer (+0x2b0, +0x467) has no use, and
// its hip requests go nowhere unless one runs;
// the both-arms-down event, the teeter edge clamp and the big-spin
// stiffness aren't here.

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
    /// hands and knees' push off the hand: +0x5f, and its timer +0x48 (init -0.1)
    push: bool,
    push_t: f32,
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
    /// the roll delay and friction delay timers (+0x294, +0x298), each hand's friction ramp (+0x2a8, +0x2ac), rolling started
    roll_wait: f32,
    friction_wait: f32,
    hand_friction: [f32; 2],
    rolling: bool,
    /// the hands-and-knees mode (+0x2bb), its hands-down timer (+0x29c, off at 0 or under) and lost-contact grace (+0x2a0)
    hk: bool,
    hands_t: f32,
    grace: f32,
    /// +0x2b8: keeps the head off the ground, from the first touch or smartFall's forceHeadAvoid
    head_away: bool,
}

impl Rdr2Cf {
    fn draw_legs(&mut self, ch: &mut Character) {
        self.knee_amount = [ch.random() * 1.8, ch.random() * 1.8];
        self.hip_gain = [ch.random() * 0.31 - 0.3, ch.random() * 0.31 - 0.3];
    }

    fn arm_init(ch: &mut Character, len: f32) -> RArm {
        let (clav_lean1, clav_lean2, elbow) = (ch.random() * 0.4 - 0.4, ch.random() * 0.5, 0.7 + ch.random() * 0.6);
        let _ = len;
        RArm { k: 1.0, clav_lean1, clav_lean2, elbow, elbow_on: false, target: Vec3::ZERO, normal: Vec3::Z, swing: (0.0, 0.0), push: false, push_t: -0.1 }
    }
}

impl CatchFall {
    fn start_rdr2(p: Params, c: &mut Ctx, head: &mut HeadLook, st: &mut SpineTwist) -> CatchFall {
        let ch = &mut *c.ch;
        let fall = ch.core.cfg.fall;
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
                             len: 0.0, reach_scale: 1.0, arm: [RArm::default(); 2], forward: 0.0, upright: 0.0, dir: Vec3::ZERO, roll: 0.0,
                             roll_wait: fall.inhibit_rolling, friction_wait: fall.change_friction_time, hand_friction: [fall.arm_friction; 2], rolling: false,
                             hk: false, hands_t: -0.1, grace: 2.0, head_away: std::mem::take(&mut ch.core.cfg.fall.force_head) };
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
        // the wrists' limits this tick (0x2a4eaa0 at the head of its tick)
        for i in 0..2 {
            c.ch.lim_raise(crate::nmrs::WRIST()[i], [1.1, 2.0, 1.1]);
        }
        // on the ground: any part but the lower legs and feet touching
        let legs: Vec<usize> = (0..2).flat_map(|i| [(*KNEE())[i] + 1, (*ANKLE())[i] + 1]).collect();
        let others: Vec<usize> = (0..c.b.frames.len()).filter(|k| !legs.contains(k)).collect();
        r.on_ground |= c.b.doll.collided(&others);
        if r.on_ground {
            r.time += dt;
            if r.roll_wait >= 0.0 {
                r.roll_wait -= dt;
            }
            if r.friction_wait > 0.0 {
                r.friction_wait -= dt;
            }
        }
        let fall = c.ch.core.cfg.fall;
        r.head_away |= r.on_ground;
        // hands and knees (5.1): asked, health over the threshold (0: smartFall's own 0.5 isn't kept), inside the window
        let hk_was = r.hk;
        let stagger_short = fall.hk_window < 0.5 && c.extras_on.contains(&"staggerFall");
        r.hk = fall.hk && c.ch.core.cfg.health > 0.0 && fall.hk_window > r.time;
        if r.hk {
            crate::extras_core_rdr2::trace_first("catchFall: hands and knees");
            // 0x2af72f4 step 4: no arm part (0xfd800) touching with the balancer active, or a BCR in 8, 9: the effective mask is the upper body's: the legs
            // are the balancer's. (bal+0x467 goes with it: the writes are all there is; no read in the balancer's code at +0x467 or, off bal+0xe0, +0x387.)
            let rp = crate::extras_ysk_rdr2::rp;
            let arm_touch = (11..=19).any(|l| c.b.doll.hit_out.get(rp(l)).is_some_and(|h| h.is_some()));
            let bcr = c.ch.core.bcr_state == 8 || c.ch.core.bcr_state == 9;
            if c.upright && (!arm_touch || bcr) {
                c.out.cf_upper = true;
                crate::extras_core_rdr2::trace_first("catchFall: hands and knees, the legs stay the balancer's");
            }
        }
        // the window ends this tick: the arms start over
        if r.time < fall.hk_window && fall.hk_window < r.time + dt {
            let keep = [r.arm[0].swing, r.arm[1].swing];
            r.arm = [Rdr2Cf::arm_init(c.ch, r.len), Rdr2Cf::arm_init(c.ch, r.len)];
            (r.arm[0].swing, r.arm[1].swing) = (keep[0], keep[1]);
        }
        if r.hk {
            if r.hands_t > 0.0 {
                r.hands_t += dt;
            }
            // the balancer's hip pitch, owner 15 (our sign is RDR2's negated)
            if r.hands_t > 0.1 && r.hands_t < 0.5 {
                c.out.dbm_crouch.push(crate::nmdbm_rdr2::Crouch::Hip(15, 0.5, 1e5));
            }
            if stagger_short {
                c.out.dbm_crouch.push(crate::nmdbm_rdr2::Crouch::Hip(15, -0.5, 1e5));
            }
        }
        // leaving the mode: the legs' draws over
        if hk_was && !r.hk {
            r.draw_legs(c.ch);
        }
        // setFallingReaction's roll: moving fast enough once the delay is over, rollDownStairs takes him (with resistRolling
        // only with a trunk or arm part down); the stop on slowing isn't here
        let parts: Vec<usize> = (0..c.b.frames.len()).filter(|k| !(1..=6).contains(k)).collect();
        // the slope (item 9): stopOnSlopes's time over, its size off the mean of the arms' normals; stopManual a floor under it
        let ground_n = if r.on_ground { nrm(r.arm[0].normal + r.arm[1].normal) } else { up };
        let (slope_side, slope) = if r.time > fall.stop_on_slopes {
            (nrm(ground_n.vx(up)).dot(c.m.b), 10.0 * (1.0 - up.dot(ground_n)).clamp(0.0, 0.1))
        } else {
            (1.0, 0.0)
        };
        let slope = slope.max(fall.stop_manual);
        if !r.rolling && fall.call_rds != 0.0 && r.roll_wait < 0.0 && c.vel.length() > 2.0 * slope + fall.com_vel_rds
            && (fall.resist_rolling == 0.0 || c.b.doll.collided(&parts))
        {
            r.rolling = true;
            let mut rds = crate::nmrds::RollDownStairs::rdr2_defaults();
            if let Some([stiffness, force, tlv, decay]) = fall.rds_smart {
                (rds.stiffness, rds.forcemag) = (stiffness, force);
                (rds.rdr2.limit_spin_reduction, rds.rdr2.target_linear_velocity, rds.rdr2.target_linear_velocity_decay_time) = (true, tlv, decay);
                crate::extras_core_rdr2::trace(|| format!("rollDownStairs from smartFall's fields: force {force}, target speed {tlv} decaying over {decay}"));
            }
            c.out.rds = Some((rds, None));
            crate::extras_core_rdr2::trace(|| format!("catchFall: rolling at {:.2} m/s over {}", c.vel.length(), fall.com_vel_rds));
        }
        // the ground's normal: the arms' probes' mean once down
        let ground = if r.on_ground { nrm(r.arm[0].normal + r.arm[1].normal) } else { up };
        if p.z_axis_spin_reduction > 0.0 {
            spin_reduction(c, p.z_axis_spin_reduction);
        }
        let (kl, kr) = (r.arm[0].k.max(0.0), r.arm[1].k.max(0.0));
        let goal = (2.0 * c.b.doll.kinetic()).sqrt() * 2.0;
        let mut step = (goal - self.k) * fall.k_rate * dt;
        if r.time > fall.k_time {
            step = -step.abs();
        }
        self.k = (self.k + step).clamp(0.0, 1.0);
        if r.hk && r.time < fall.k_time {
            self.k = 1.0;
        }
        let k = self.k;
        let weak = kl.min(kr);
        if weak < 0.30001 {
            c.out.send(delirium_data::kv!(54 int), "catchFall", 0.0);
        }
        if weak < 0.9 && k < fall.success {
            c.out.send(delirium_data::kv!(55 int), "catchFall", 0.0);
        }
        // the friction: when its delay is over the body takes groundFriction, and the clamp frictionMin to frictionMax
        // comes on; each hand and upper arm starts at armFriction and ramps while it touches (rdr2_catchfall.md 5.10)
        if r.friction_wait <= 0.0 {
            let cfg = &mut c.ch.core.cfg;
            for (k, x) in cfg.fric.iter_mut().enumerate() {
                if !matches!(k, 3 | 6 | 11 | 12 | 16 | 20) {
                    *x = fall.ground_friction;
                }
            }
            (cfg.fric_used, cfg.fric_clamp, cfg.fric_min, cfg.fric_max) = (true, true, fall.friction_min, fall.friction_max);
        }
        for i in 0..2 {
            let (hand, upper) = (limb(i).p[limb(i).p.len() - 1], limb(i).p[1]);
            let touching = c.b.doll.collided(&limb(i).p);
            let fh = if r.hk && r.time < fall.hand_window { 1.1 } else { r.hand_friction[i] };
            if fh <= 1.0 && fh < fall.ground_friction {
                c.out.part_friction.push((hand, fh));
                c.out.part_friction.push((upper, fh));
            }
            if touching && r.hand_friction[i] <= 1.0 {
                r.hand_friction[i] += 2.0 * dt;
            } else if !touching && r.hand_friction[i] >= fall.arm_friction {
                r.hand_friction[i] -= 0.666 * dt;
            }
        }
        // the arms in the mode: muscle 1; hands down a while, stiffer; just then the legs' draws for kneeling (5.3 item 7)
        let (mut m_a, mut x_a) = (0.5, 0.0);
        if r.hk {
            m_a = 1.0;
            if r.hands_t > 0.3 || stagger_short {
                if r.forward > -0.3 {
                    x_a = 2.0;
                }
                if r.hands_t < 0.3 + 1.1 * dt && !stagger_short {
                    r.knee_amount = [1.1 + c.ch.random() * 0.6, 1.1 + c.ch.random() * 0.6];
                    r.hip_gain = [-0.3 - c.ch.random() * 0.11, -0.3 - c.ch.random() * 0.11];
                    crate::extras_core_rdr2::trace_first("catchFall: hands down, the knees drawn in");
                }
            }
        }
        let ch = &mut *c.ch;
        // the stiffness floors minSpine, minArm, minLegStiffness under k (rdr2_catchfall.md 5.4)
        let (legs, torso_k, arms) = (p.legs_stiffness.max(fall.min_leg), p.torso_stiffness.max(fall.min_spine), (p.arms_stiffness + x_a).max(fall.min_arm));
        let torso = (torso_k - fall.min_spine) * k + fall.min_spine;
        ch.set_stiffness(torso, 0.5, "us", Some(0.5));
        ch.set_stiffness(((arms - fall.min_arm) * kl + fall.min_arm).max(3.0).min(20.0), 1.0, "ul", Some(m_a));
        ch.set_stiffness(((arms - fall.min_arm) * kr + fall.min_arm).max(3.0).min(20.0), 1.0, "ur", Some(m_a));
        // (one side's lerp is scaled by k twice in RDR2)
        let leg = (legs - fall.min_leg) * k + fall.min_leg;
        ch.set_stiffness((p.legs_stiffness * 0.25).max(leg * k).max(4.0), 0.5, "ll", None);
        ch.set_stiffness((p.legs_stiffness * 0.25).max(leg).max(4.0), 0.5, "lr", None);
        if r.hk {
            for i in 0..2 {
                // 12 with the other hand pushing off (the mask's 0x2 reads helper 1's flag, 0x4 helper 0's)
                ch.eff[(*HIP())[i]].set_stiffness(if r.arm[1 - i].push { 12.0 } else { 8.0 }, 0.5, Some(0.5));
            }
        }
        // 5.8: +0xc0: an arm in state 2 holds the rifle: the shoulder at 0, the elbow at 2; the core's default arms stay off (ctx+0x480c)
        for i in 0..2 {
            if fall.rifle_pose && ch.core.arm_state[i] == 2 {
                let l = limb(i);
                ch.eff[l.e[1]].desired = [0.0; 3];
                ch.eff[l.e[2]].desired[0] = 2.0;
                ch.core.arms_held = true;
                crate::extras_core_rdr2::trace_first("catchFall: riflePose, the arm in state 2");
            }
        }
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
            ch.eff[j].desired[0] = if r.hk && r.knee_amount[0] > 0.6 { fall.spine_lean } else { fall.spine_lean * (-f).clamp(0.0, 1.0) };
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
        if !st.twisting && !r.hk {
            let decay = 1.0 / (12.0f32).mul_add(dt, 1.0);
            for j in (*SPINE()) {
                ch.eff[j].desired[2] *= decay;
            }
        }
        // the curl (extraSit, its default 1: no message here sends it)
        let curl = if r.hk && !stagger_short {
            (k * ((2.0 * c.vel.dot(m.c)).clamp(0.0, 1.0) * r.upright - 0.5 * f)).clamp(0.0, 1.0)
        } else {
            0.0
        };
        for i in 0..2 {
            let h = &mut ch.eff[(*HIP())[i]];
            h.blend_to_zero(1.0);
            h.desired[0] = curl - 0.7 * r.knee[i];
            h.desired[1] = m.c.dot(ground).abs() * r.hip_gain[i];
        }
        let d = m.c.dot(ground);
        // the slope block (item 7): on a slope, or with stopManual, slower than 2 m/s or resisting, the hips close and open
        if (r.time > fall.stop_on_slopes || fall.stop_manual > 0.0) && (c.vel.length() < 2.0 || fall.resist_rolling != 0.0) && slope_side.abs() * slope > 0.0 && r.on_ground {
            let x = slope_side.abs() * slope * -d.abs();
            for i in 0..2 {
                let h = &mut ch.eff[(*HIP())[i]];
                h.desired[1] = h.desired[1] * (1.0 + x) + 0.7 * x;
            }
            crate::extras_core_rdr2::trace_first("catchFall slope block");
        }
        if r.hk {
            for i in 0..2 {
                ch.eff[(*HIP())[i]].desired[2] = 0.0;
            }
        }
        let most = if r.hk { 2.0 - 1.7 * (-d).clamp(0.0, 1.0) } else { 1.0 - 0.7 * d.clamp(0.0, 1.0) };
        for i in 0..2 {
            // rate limited at blendkneerate (by default 10^4 rad/s: at once)
            let want = -r.knee_amount[i].min(most);
            let step = fall.knee_rate * dt;
            r.knee[i] = if want.abs() < 1e4 && step < 1e3 { r.knee[i] + (want - r.knee[i]).clamp(-step, step) } else { want };
            ch.eff[(*KNEE())[i]].desired[0] = r.knee[i];
        }
        for i in 0..2 {
            let a = r.arm_rdr2(self, c, world, obstacles, st.twist, i);
            r.arm[i] = a;
        }
        // fall to knees' poses (+0xcb..+0xcd)
        let knee = c.ch.core.cf_knee;
        if knee[0] {
            for &j in HIP().iter() {
                c.ch.eff[j].strength = 81.0;
            }
        }
        let poses = if knee[1] {
            Some([[0.0, 0.9, 0.7], [0.0, 0.7, 0.7]])
        } else if knee[2] {
            let k = r.arm[0].k.min(r.arm[1].k);
            (r.arm[0].k, r.arm[1].k) = (k, k);
            Some([[0.0, -0.5, -0.7], [0.0, -0.9, -0.7]])
        } else {
            None
        };
        if let Some(a) = poses {
            for i in 0..2 {
                c.ch.eff[crate::nmrs::SHOULDER()[i]].desired = a[i];
                c.ch.eff[crate::nmrs::ELBOW()[i]].desired[0] = [0.9, 0.6][i];
            }
        }
        let mut look = c.b.part(HEAD()).m.d + r.dir;
        let chest = c.b.part(SPINE3());
        let away = r.head_away && (fall.hk_head || !r.hk);
        if p.use_head_look {
            // in the mode he looks ahead, level, off the second spine part's axes
            if r.hk {
                let s2 = c.b.pmat((*SPINE())[2] + 1).m;
                let u = if up.dot(-s2.c) >= 0.0 { -s2.c - s2.a } else { -s2.c + s2.a };
                look = c.b.part(HEAD()).m.d + (u - up * up.dot(u));
            }
            // at catchFall's priority; once down, headLook leans the head away from the ground instead
            head.look(crate::nmhl::PRIO_CATCH_FALL, look, chest.v, true);
            (head.stiffness, head.keep_head_away) = (2.0 + torso, away);
        } else if away {
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
        // the swing circle (with ?7dad7aaf on)
        let circle = c.ch.core.cfg.fall.swing_circle != 0.0;
        let e = if circle { 1.0 - 5.0 * ((-(a.target - shoulder).z).clamp(0.3, 0.5) - 0.3) } else { 0.0 };
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
        let mut reach = self.len * self.reach_scale * (0.7 + 0.3 * (1.0 - (self.upright - 0.5).clamp(0.0, 1.0)));
        // hands and knees: the hands' timers (6.4), the hand held under him when falling back (6.6), a touching arm reaches less (6.7)
        if self.hk {
            let on = c.b.doll.collided(&l.p);
            if !on {
                if self.grace < 1.0 {
                    self.grace -= c.dt;
                    if self.grace < 0.0 {
                        self.hands_t = -1.0;
                    }
                }
            } else if hit.is_some() {
                if self.hands_t <= 0.0 {
                    (self.hands_t, self.grace) = (1e-5, 0.1);
                }
                if self.grace > 1.0 {
                    self.grace = 0.1;
                }
            }
            if f < -0.3 {
                target = c.b.part(l.p[3]).m.d - normal * 0.7;
                c.out.dbm_crouch.push(crate::nmdbm_rdr2::Crouch::Hip(15, -(3.5 * f + 0.55).clamp(-2.0, 2.0), 1e5));
            }
            if on {
                reach *= (c.b.part(l.p[3]).v.length().clamp(1.0, 1.7) - 1.0) * 0.1 + 0.93;
            }
        }
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
        } else if ul < (if self.hk { 1.5 } else { 0.4 }) && u.dot(normal) < 0.0 && !pushed {
            u *= (if self.hk { 1.5 } else { 0.4 }) / (ul + 1e-4);
        }
        if u.length() > reach {
            u *= reach / u.length();
        }
        // Push-off (item 11), hands and knees only: a hand turned flat on what it touches shoves for 0.0835 s, one hand at a time
        if self.hk {
            let hand = c.b.pmat(l.p[3]).m;
            if (hand.a * side).dot(normal) < 0.0 && hand.b.dot(normal) > 0.7 && !self.arm[1 - i].push && a.push_t < -0.3 {
                (a.push, a.push_t) = (true, 0.0835);
                crate::extras_core_rdr2::trace_first("catchFall: a hand pushes off");
            }
            if a.push {
                u += normal * 0.3;
                if u.length() > reach {
                    u *= reach / u.length();
                }
            }
            if a.push_t < 0.0 {
                a.push = false;
            }
            if a.push_t >= 0.0 || (a.push_t >= -0.3 && c.b.doll.collided(&l.p)) {
                a.push_t -= c.dt;
            }
        } else {
            (a.push, a.push_t) = (false, -0.1);
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
            if twist.abs() >= 2.0 {
                c.ch.eff[clav].desired = [a.clav_lean1, a.clav_lean2, 0.3];
            } else {
                clav_overflow(c.ch, clav, l.e[1]);
            }
        } else {
            c.ch.eff[clav].desired = [0.0, 0.0, c.ch.actual(clav, c.b.frames)[2]];
        }
        if (a.k < 0.3 || c.ch.core.cfg.fall.anti_prop_clav != 0.0) && f < 0.0 {
            c.ch.eff[clav].desired = [0.0; 3];
            if c.ch.core.cfg.fall.anti_prop_clav != 0.0 {
                crate::extras_core_rdr2::trace_first("catchFall antiPropClav");
            }
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
