use crate::fneg;
use crate::prelude::*;
use crate::Vd;
use crate::Vx;
use crate::Fma;

use crate::nmdbm::{sdot, shz, slen, vnorm, Body, M34};
use crate::nmrs::{Character, ANKLE, CLAVICLE, ELBOW, HIP, KNEE, SHOULDER, SPINE, WRIST};

#[allow(non_snake_case)]
#[inline]
fn SP() -> &'static [usize; 7] { static V: std::sync::OnceLock<[usize; 7]> = std::sync::OnceLock::new(); let f = || [0, delirium_data::kv!(22 int), delirium_data::kv!(23 int), delirium_data::kv!(104 int), delirium_data::kv!(81 int), delirium_data::kv!(113 int), delirium_data::kv!(55 int)]; if delirium_data::kv::open() { V.get_or_init(f) } else { Box::leak(Box::new(f())) } }
#[allow(non_snake_case)]
#[inline]
fn SE() -> &'static [usize; 6] { static V: std::sync::OnceLock<[usize; 6]> = std::sync::OnceLock::new(); let f = || [delirium_data::kv!(6 int), delirium_data::kv!(22 int), delirium_data::kv!(23 int), delirium_data::kv!(104 int), delirium_data::kv!(81 int), delirium_data::kv!(113 int)]; if delirium_data::kv::open() { V.get_or_init(f) } else { Box::leak(Box::new(f())) } }
#[allow(non_snake_case)]
#[inline]
fn HEAD() -> usize { delirium_data::kv!(55 int) }

#[derive(Clone, Copy)]
pub(crate) struct Limb {
    pub e: [usize; 4],
    pub p: [usize; 4],
}

/// The chest part (RDR2's part 10), whose frame armsWindmill's circles are fixed to.
pub(crate) fn chest_part() -> usize {
    SP()[4]
}

/// RDR2's spine limb parts in order: pelvis, the four spine parts, neck, head.
pub(crate) fn spine_parts() -> [usize; 7] {
    *SP()
}

/// RDR2's spine limb joints in order: the four spine joints, neck, head.
pub(crate) fn spine_joints() -> [usize; 6] {
    *SE()
}

pub(crate) fn limb(i: usize) -> Limb {
    if i < 2 {
        let c = (*CLAVICLE())[i];
        Limb { e: [c, (*SHOULDER())[i], (*ELBOW())[i], (*WRIST())[i]], p: [c + 1, (*SHOULDER())[i] + 1, (*ELBOW())[i] + 1, (*WRIST())[i] + 1] }
    } else {
        let k = i - 2;
        Limb { e: [usize::MAX, (*HIP())[k], (*KNEE())[k], (*ANKLE())[k]], p: [0, (*HIP())[k] + 1, (*KNEE())[k] + 1, (*ANKLE())[k] + 1] }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Params {
    pub normal: Vec3,
    pub head_look: Vec3,
    pub hit_point: Vec3,
    pub bullet_vel: Vec3,
    pub spine_pain_twist_multiplier: f32,
    pub fling_width: f32,
    pub time_before_collapse_wound_leg: f32,
    pub grab_hold_time: f32,
    pub spine_pain_time: f32,
    pub body_stiffness: f32,
    pub looseness_amount: f32,
    pub spine_pain_multiplier: f32,
    pub time_before_reach_for_wound: f32,
    pub p84: f32,
    pub head_look_at_wound_min_timer: f32,
    pub head_look_at_wound_max_timer: f32,
    pub head_look_at_head_pos_max_timer: f32,
    pub head_look_at_head_pos_min_timer: f32,
    pub body_part: usize,
    pub max_steps: i32,
    pub shock_spin_min: f32,
    pub shock_spin_max: f32,
    pub shock_spin_lift_force_mult: f32,
    pub shock_spin_decay_mult: f32,
    pub shock_spin_scale_per_component: f32,
    pub chicken_arms: bool,
    pub always_add_shock_spin: bool,
    pub randomize_shock_spin_direction: bool,
    pub add_shock_spin: bool,
    pub use_head_look: bool,
    pub crouching: bool,
    pub fling: bool,
    pub melee: bool,
    pub no_reach: [bool; 2],
    pub stable_hands_and_neck: bool,
    pub use_catch_fall_on_fall: bool,
    pub reach_for_wound: bool,
    /// RDR2's further parameters (read with the RDR2 core)
    pub rdr2: ShotRdr2,
}

impl Default for Params {
    fn default() -> Params {
        Params {
            normal: Vec3::new(0.0, 0.0, -1.0), head_look: Vec3::ZERO, hit_point: Vec3::ZERO, bullet_vel: Vec3::new(0.0, 0.0, delirium_data::kv!(25)),
            spine_pain_twist_multiplier: 0.5, fling_width: 0.5, time_before_collapse_wound_leg: delirium_data::kv!(24), grab_hold_time: 2.0,
            spine_pain_time: delirium_data::kv!(12), body_stiffness: delirium_data::kv!(117), looseness_amount: 1.0, spine_pain_multiplier: 1.0,
            time_before_reach_for_wound: delirium_data::kv!(60), p84: 1.0, head_look_at_wound_min_timer: delirium_data::kv!(17), head_look_at_wound_max_timer: delirium_data::kv!(20),
            head_look_at_head_pos_max_timer: delirium_data::kv!(66), head_look_at_head_pos_min_timer: delirium_data::kv!(119), body_part: 0, max_steps: delirium_data::kv!(197 int),
            shock_spin_min: delirium_data::kv!(79), shock_spin_max: delirium_data::kv!(539), shock_spin_lift_force_mult: 0.0, shock_spin_decay_mult: delirium_data::kv!(84),
            shock_spin_scale_per_component: 0.5, chicken_arms: false, always_add_shock_spin: false,
            randomize_shock_spin_direction: false, add_shock_spin: false, use_head_look: false, crouching: false, fling: false,
            melee: false, no_reach: [false; 2], stable_hands_and_neck: false, use_catch_fall_on_fall: false, reach_for_wound: true,
            rdr2: ShotRdr2::default(),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Feedback {
    pub slot: u8,
    pub name: &'static str,
    pub value: f32,
}

#[derive(Default, Debug)]
pub struct Out {
    pub dbm: Option<bool>,
    pub dbm_steps: Option<i32>,
    pub dbm_leg_offset: Option<f32>,
    pub dbm_knee_scale: Option<f32>,
    pub dbm_acc: Option<f32>,
    pub dbm_turn: Option<Vec3>,
    /// RDR2 core: the injured leg (0 left, 1 right) and how badly, for the balancer's limp
    pub dbm_limp: Option<(usize, f32)>,
    /// RDR2 core: the balancer's crouch request this step, (owner id, knee bend as leg straightness, hip pitch in
    /// rad), as RDR2's behaviours ask it rather than bend the legs themselves
    pub dbm_owned: Vec<(u32, f32, f32)>,
    /// RDR2 core: requests on one channel of the crouch (RDR2's separate hip pitch and knee bend owners)
    pub dbm_crouch: Vec<crate::nmdbm_rdr2::Crouch>,
    /// RDR2 core: no new steps this step (force balance), and the turn request's facing threshold (cosine)
    pub dbm_force_balance: bool,
    /// RDR2 core: force this foot (0 left, 1 right) to step next, dropping the step in progress (Some(None): no more)
    pub dbm_force_step: Option<Option<usize>>,
    pub dbm_turn_facing: Option<f32>,
    /// RDR2 core: staggerFall's stagger mode for the balancer this step, and its prediction time
    pub dbm_stagger: bool,
    pub dbm_lead: Option<f32>,
    /// RDR2 core: teeter's addition to the balancer's prediction time this step (bal+0x6bc with +0x6c0 on)
    pub dbm_lead_add: f32,
    /// RDR2 core: the balancer's taperKneeStrength (bal+0x654) from now on, as teeter and highFall set it
    pub dbm_taper: Option<bool>,
    /// RDR2 core: teeter's exclusion plane this step (point, horizontal normal, side sign, offset): the balancer's
    /// predicted COM is kept on the plane `offset` from the edge
    pub dbm_plane: Option<(Vec3, Vec3, f32, f32)>,
    /// RDR2 core: a hip lean asked of the balancer (hipsLean*)
    pub hip_lean: Option<Vec3>,
    /// A lean asked for by an extra (RDR2's lean family, onFire's stagger): the balancer leans that way from the next
    /// step; None unless one does.
    pub lean: Option<(crate::nmdbm::LeanMode, f32, bool)>,
    pub hold_stiffness: bool,
    pub feedback: Vec<Feedback>,
    pub head_look_start: Option<(f32, f32)>,
    pub head_look_target: Option<Vec3>,
    pub head_look_activate: bool,
    pub head_look_stop: bool,
    pub catch_fall: Option<[f32; 3]>,
    pub roll_up: Option<f32>,
    pub fall_stop: bool,
    /// RDR2 core: the fall's sub-behaviours that GTA IV's hasn't stop with it (rollDownStairs, bodyWrithe, smartFall)
    pub fall_stop_rdr2: bool,
    /// RDR2 core, fallingReaction 2: rollDownStairs started with these if it isn't running, and its stiffness now
    pub rds: Option<(crate::nmrds::RollDownStairs, Option<f32>)>,
    /// RDR2 core, fallingReaction 3: smartFall started (from its hard landing) if it isn't running
    pub smart_fall: bool,
    /// RDR2 core, fallingReaction 4: bodyWrithe started with these if it isn't running
    pub writhe: Option<crate::nm::BodyWrithe>,
    pub spine_twist: Option<(Vec3, f32)>,
    pub spine_twist_stop: bool,
    pub torque: Vec<Vec3>,
    pub force: Vec<Vec3>,
    pub seq: Vec<(usize, Vec3, bool)>,
    pub impulse: Vec<(usize, Vec3, Vec3)>,
    /// RDR2 core: the default arms' releaseWound, telling reachForWound to let that hand go this step
    pub rfw_release: [bool; 2],
    /// RDR2 core: an extra (onFire) stops bodyBalance, then starts it with these (RDR2's children)
    pub bb_stop: bool,
    pub bb_start: Option<crate::nmbb::Params>,
    /// RDR2 core: the balancer's leg stiffness from now on (onFire's bal+0x658/+0x65c)
    pub dbm_leg_stiffness: Option<f32>,
    /// RDR2 core: pedalLegs' one-tick reverse flags (left, right) for its next tick, as RDR2's yanked writes them
    pub pedal_reverse: Option<[bool; 2]>,
}

impl Out {
    pub(crate) fn send(&mut self, slot: u8, name: &'static str, value: f32) {
        self.feedback.push(Feedback { slot, name, value });
    }

    pub(crate) fn torque(&mut self, k: usize, t: Vec3) {
        if self.torque.len() <= k {
            self.torque.resize(k + 1, Vec3::ZERO);
        }
        self.torque[k] += t;
        self.seq.push((k, t, false));
    }

    pub(crate) fn force(&mut self, k: usize, f: Vec3) {
        if self.force.len() <= k {
            self.force.resize(k + 1, Vec3::ZERO);
        }
        self.force[k] += f;
        self.seq.push((k, f, true));
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct Reach {
    pos: Vec3,
    normal: Vec3,
    arm: usize,
    part: usize,
    off: f32,
    sign: f32,
}

/// RDR2's hit record (0x2a40ddc), which the new-bullet router fills once for the shot and reachForWound alike: a bullet
/// from behind, unless in the neck or head, is moved to where it comes out of him, by a ray back along it from 0.4 m
/// past the hit. The part and point there are the wound; the impulse keeps the entry point.
pub(crate) fn hit_record(c: &Ctx, point: Vec3, part: i32, bullet: Vec3) -> (Vec3, i32) {
    if bullet == Vec3::ZERO || point == Vec3::ZERO || bullet.dot(c.m.c) > 0.0 || matches!(part, 11 | 12) {
        return (point, part);
    }
    let far = point + bullet.normalize_or_zero() * 0.4;
    c.b.doll.probe(c.b.build, far, point).map_or((point, part), |(k, x, _)| (x, k as i32))
}

#[derive(Clone, Debug)]
pub struct Shot {
    pub p: Params,
    new_hit: bool,
    no_balance: bool,
    since_hit: f32,
    reach_t: [f32; 2],
    time: f32,
    s150: f32,
    s154: f32,
    s158: f32,
    s15c: f32,
    s160: f32,
    s164: f32,
    s168: f32,
    s16c: f32,
    s170: f32,
    t174: f32,
    head_t: f32,
    fall: [f32; 2],
    spin: f32,
    spin_blend: f32,
    t190: f32,
    pub t194: f32,
    behind: f32,
    side: f32,
    reach_part: [Option<usize>; 2],
    loose: f32,
    wound: Vec3,
    hit_local: Vec3,
    normal_local: Vec3,
    reach_pos: [Vec3; 2],
    reach_normal: [Vec3; 2],
    chicken_t: f32,
    react_t: f32,
    react_elbow: f32,
    react_phase: f32,
    reach: [Reach; 2],
    cur_reach: usize,
    cur_arm: usize,
    m228: f32,
    m248: f32,
    m24c: f32,
    m250: f32,
    m254: f32,
    f258: f32,
    f25c: f32,
    f260: f32,
    f264: f32,
    f268: f32,
    f26c: u32,
    f270: f32,
    f274: f32,
    f278: [bool; 2],
    flags: u8,
    grab_front: bool,
    sub: [bool; 15],
    rd: ShotRdr2State,
    da: DefaultArms,
}

const LOOSE_L: usize = 0;
const LOOSE_R: usize = 1;
const COLLAPSE_L: usize = 2;
#[allow(non_snake_case)]
#[inline]
fn COLLAPSE_R() -> usize { delirium_data::kv!(0 int) }
#[allow(non_snake_case)]
#[inline]
fn REACT() -> usize { delirium_data::kv!(1 int) }
#[allow(non_snake_case)]
#[inline]
fn HIT() -> usize { delirium_data::kv!(2 int) }
#[allow(non_snake_case)]
#[inline]
fn REACH_L() -> usize { delirium_data::kv!(6 int) }
#[allow(non_snake_case)]
#[inline]
fn REACH_R() -> usize { delirium_data::kv!(22 int) }
#[allow(non_snake_case)]
#[inline]
fn STIFF() -> usize { delirium_data::kv!(23 int) }
#[allow(non_snake_case)]
#[inline]
fn LOOK() -> usize { delirium_data::kv!(104 int) }
const FALL: usize = 10;
#[allow(non_snake_case)]
#[inline]
fn CHICKEN() -> usize { delirium_data::kv!(113 int) }
#[allow(non_snake_case)]
#[inline]
fn FLING() -> usize { delirium_data::kv!(55 int) }
#[allow(non_snake_case)]
#[inline]
fn MELEE() -> usize { delirium_data::kv!(114 int) }
#[allow(non_snake_case)]
#[inline]
fn CROUCH() -> usize { delirium_data::kv!(56 int) }

pub struct Ctx<'a, 'b> {
    pub ch: &'a mut Character,
    pub b: &'a Body<'b>,
    pub out: &'a mut Out,
    pub dbm_failed: bool,
    pub(crate) m: M34,
    pub(crate) com: Vec3,
    pub(crate) vel: Vec3,
    pub(crate) angvel: Vec3,
    pub(crate) up: Vec3,
    pub cf: Option<(&'a mut Option<crate::nmcf::CatchFall>, &'a mut crate::nmhl::HeadLook, &'a mut crate::nmst::SpineTwist)>,
    /// RDR2 core's limb requests: the shot's modules each make their own (limbreq.rs)
    pub reqs: Option<&'a mut crate::limbreq::Requests>,
    /// the shot's priority for those requests (below catchFall's once it has taken over)
    pub shot_slot: u32,
    /// the balancer is running and balancing him (RDR2's bal+0x48 = 0); only the RDR2 core's extras read it
    pub upright: bool,
    /// RDR2 core: what the balancer is doing, for the extras that read it (staggerFall); None without a balancer
    pub dbm: Option<DbmView>,
    /// RDR2 core: the shot behaviour is running (staggerFall leaves the fall to it)
    pub shot_on: bool,
    /// RDR2 core: what the shot's default arms read of reachForWound (None when it isn't running)
    pub rfw: Option<RfwView>,
    /// RDR2 core: bodyBalance is running (the default arm pose leaves the arms to it)
    pub bb_on: bool,
    /// the extras running this step, by message name (RDR2's behaviours read whether staggerFall runs)
    pub extras_on: Vec<&'static str>,
    /// RDR2 core: the incoming animation's part frames (empty: none), which animPose drives the limbs to
    pub incoming: &'a [Transform],
    /// RDR2 core: the shot's strength scale for the behaviours that read it (controlledFall); 1 without a shot
    pub shot_scale: f32,
}

/// What the shot's default arms read of RDR2's reachForWound, per hand: in a reach with the hand placed, how long
/// since the reach started (its clock less timeBeforeReachForWound), and the time since that hand's reach last ended.
#[derive(Clone, Copy, Debug)]
pub struct RfwView {
    pub reached: [bool; 2],
    pub on: [f32; 2],
    pub since_stop: [f32; 2],
}

/// What an extra may read of the balancer this step (RDR2's bal+0x45c, +0x550 and its fail ramp).
#[derive(Clone, Copy, Debug, Default)]
pub struct DbmView {
    /// the stepping foot: 0 none, 1 left, 2 right
    pub stepping: u32,
    /// each foot fully planted
    pub planted: [bool; 2],
    /// the timed fail has started (RDR2's fail ramp)
    pub failing: bool,
    /// the crouch request's hip pitch now (ours: RDR2's bal+0x578 negated)
    pub hip_pitch: f32,
    /// the steps it has taken (RDR2's step counter)
    pub steps: u32,
    /// his standing pelvis height, the leg length the balancer's crouch measures use
    pub leg: f32,
}

impl<'a, 'b> Ctx<'a, 'b> {
    pub fn new(ch: &'a mut Character, b: &'a Body<'b>, out: &'a mut Out, dbm_failed: bool) -> Ctx<'a, 'b> {
        let up = ch.up;
        let s = b.snap(up);
        Ctx { ch, b, out, dbm_failed, m: s.m, com: s.com, vel: s.vel, angvel: s.angvel, up, cf: None, reqs: None, shot_slot: crate::limbreq::SHOT, upright: !dbm_failed,
              dbm: None, shot_on: false, rfw: None, bb_on: false, extras_on: vec![], incoming: &[], shot_scale: 1.0 }
    }

    /// With the RDR2 core's limb requests: what the shot just wrote is this module's request.
    /// Its reaching for the wound is RDR2's separate reachForWound behaviour, so ranks as that.
    fn module(&mut self, tag: u32) {
        let reach = tag == REACH_L() as u32 || tag == REACH_R() as u32;
        let slot = if reach && self.shot_slot == crate::limbreq::SHOT { crate::limbreq::slot("reachForWound") } else { self.shot_slot };
        if let Some(r) = self.reqs.as_deref_mut() {
            r.mark(&self.ch.eff, slot);
        }
    }

    /// With the RDR2 core's limb requests: what was just written is the request of the behaviour in `slot`.
    fn mark(&mut self, slot: u32) {
        if let Some(r) = self.reqs.as_deref_mut() {
            r.mark(&self.ch.eff, slot);
        }
    }

    fn rand(&mut self, a: f32, b: f32) -> f32 {
        a + (b - a) * self.ch.random()
    }

    fn actual(&self, j: usize) -> [f32; 3] {
        self.ch.actual(j, self.b.frames)
    }

    fn limits(&self, j: usize) -> [f32; 6] {
        self.ch.limits(j)
    }

    fn og(&mut self, j: usize, x: f32) {
        self.ch.eff[j].oppose_gravity = x;
    }

    fn spine_all(&mut self, l1: f32, l2: f32, tw: f32) {
        for j in (*SPINE()) {
            self.ch.eff[j].desired = [l1 * delirium_data::kv!(17), l2 * delirium_data::kv!(17), tw * delirium_data::kv!(17)];
        }
    }

    pub(crate) fn spine_lean(&mut self, l1: f32, l2: f32) {
        for (k, j) in (*SPINE()).into_iter().enumerate() {
            let lo = k < 2;
            self.ch.eff[j].desired[0] = if lo { 2.0 * l1 / delirium_data::kv!(35) } else { l1 / delirium_data::kv!(35) };
            self.ch.eff[j].desired[1] = if lo { 0.5 * l2 } else { delirium_data::kv!(17) * l2 };
        }
    }

    pub(crate) fn relax(&mut self, x: f32, mask: &str) {
        for j in Character::mask(mask) {
            self.ch.eff[j].blend_to_zero(x);
        }
    }
}

fn world(m: &M34, v: Vec3) -> Vec3 {
    crate::vm4(m.a, m.b, m.c, m.d, v)
}

fn rot(m: &M34, v: Vec3) -> Vec3 {
    crate::vm4(m.a, m.b, m.c, Vec3::ZERO, v)
}

fn local(m: &M34, v: Vec3) -> Vec3 {
    let d = v - m.d;
    Vec3::new(m.a.vd(d), m.b.vd(d), m.c.vd(d))
}

fn horizontal(v: Vec3, up: Vec3) -> Vec3 {
    v - up * up.vd(v)
}

fn deadzone(x: f32, d: f32) -> f32 {
    if x > d { x - d } else if x < -d { x + d } else { 0.0 }
}

pub(crate) const INIT_SPIN: [f32; 2] = [0.0, -1.0];

type Check = fn(&Shot, &Ctx) -> bool;
type Act = fn(&mut Shot, &mut Ctx);
type Tick = fn(&mut Shot, &mut Ctx, f32);

impl Shot {
    pub fn activate(p: Params, c: &mut Ctx, dt: f32) -> Shot {
        let bs = p.body_stiffness;
        let mut s = Shot {
            p, new_hit: false, no_balance: false, since_hit: 0.0, reach_t: [0.0; 2], time: 0.0,
            s150: delirium_data::kv!(117), s154: bs, s158: 1.0, s15c: bs * delirium_data::kv!(540), s160: 1.0, s164: bs / delirium_data::kv!(117) * delirium_data::kv!(25), s168: delirium_data::kv!(27) * dt,
            s16c: bs * delirium_data::kv!(541), s170: delirium_data::kv!(27) * dt, t174: if p.fling { delirium_data::kv!(201) } else { delirium_data::kv!(542) }, head_t: p.head_look_at_wound_max_timer,
            fall: [1.0; 2], spin: INIT_SPIN[0], spin_blend: INIT_SPIN[1], t190: 0.0, t194: 0.0, behind: 0.0, side: 0.0, reach_part: [None; 2],
            loose: delirium_data::kv!(51), wound: Vec3::ZERO, hit_local: Vec3::ZERO, normal_local: Vec3::ZERO, reach_pos: [Vec3::ZERO; 2],
            reach_normal: [Vec3::ZERO; 2], chicken_t: 0.0, react_t: 0.0, react_elbow: 0.0, react_phase: 0.0,
            reach: [Reach::default(); 2], cur_reach: 0, cur_arm: 0, m228: 0.0, m248: 0.0, m24c: 0.0, m250: 0.0, m254: 0.0,
            f258: 0.0, f25c: 0.0, f260: 0.0, f264: 0.0, f268: 0.0, f26c: 0, f270: 0.0, f274: 0.0, f278: [false; 2],
            flags: delirium_data::kv!(251 int), grab_front: false, sub: [false; 15], rd: ShotRdr2State::default(),
            da: DefaultArms::default(),
        };
        // RDR2 resets only the joints off the legs: the legs keep the balancer's stiffness
        let legs: Vec<usize> = if c.ch.rdr2 { Character::mask("lb") } else { vec![] };
        for j in 0..c.ch.eff.len() {
            if !legs.contains(&j) {
                let jd = c.ch.joints[j].clone();
                c.ch.eff[j].reset(&jd);
                c.ch.eff[j].desired = [0.0; 3];
            }
            c.ch.eff[j].oppose_gravity = 1.0;
        }
        if !s.no_balance {
            c.out.dbm = Some(true);
            if !p.chicken_arms {
                c.out.dbm_leg_offset = Some(-delirium_data::kv!(131));
            }
            if !c.ch.rdr2 {
                c.out.dbm_steps = Some(p.max_steps + 1);
            }
            if p.crouching {
                c.out.dbm_knee_scale = Some(0.5);
            }
        }
        if p.use_catch_fall_on_fall {
            if s.flags & 1 == 0 {
                c.out.hold_stiffness = true;
            }
        }
        if c.ch.busy == [true; 2] {
            c.relax(1.0, "ua");
            c.ch.set_stiffness(delirium_data::kv!(21), 1.0, "ua", None);
        }
        s.hit(p);
        s
    }

    pub fn hit(&mut self, p: Params) {
        self.p = p;
        self.new_hit = true;
    }

    pub fn subs(&self) -> [bool; 15] {
        self.sub
    }

    pub fn failed(&self) -> bool {
        self.flags & 1 != 0
    }

    /// RDR2 core: the shot's strength scale as other behaviours read it (0x2a8c4b8): the weakness ramp once its core
    /// strength module runs, else 1. RDR2 also multiplies in looseness4Fall while falling and looseness4Stagger
    /// while staggerFall runs; Delirium's shot has neither parameter, so those factors are 1 here.
    pub(crate) fn rdr2_scale(&self) -> f32 {
        if self.rd.started { self.rd.k } else { 1.0 }
    }

    fn crouch_check(&self, _: &Ctx) -> bool {
        self.p.crouching && !self.failed()
    }

    fn look_check(&self, _: &Ctx) -> bool {
        (self.grab_front && self.since_hit > delirium_data::kv!(543)) || self.p.use_head_look
    }

    fn look_done(&self, _: &Ctx) -> bool {
        !self.p.use_head_look && (!self.grab_front || self.new_hit)
    }

    #[allow(clippy::too_many_arguments)]
    fn run(&mut self, c: &mut Ctx, k: usize, enter_check: Option<Check>, enter: Option<Act>, tick: Option<Tick>,
           exit_check: Option<Check>, exit: Option<Act>, dt: f32) {
        if self.sub[k] && exit_check.is_some_and(|f| f(self, c)) {
            self.sub[k] = false;
            if let Some(f) = exit {
                f(self, c);
            }
        }
        if !self.sub[k] && enter_check.is_some_and(|f| f(self, c)) {
            self.sub[k] = true;
            if let Some(f) = enter {
                f(self, c);
            }
        }
        if self.sub[k] {
            if let Some(f) = tick {
                f(self, c, dt);
            }
        }
    }

    pub fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if c.ch.rdr2 {
            return self.tick_rdr2(c, dt);
        }
        let p = self.p;
        self.t190 = (self.t190 + dt).min(self.t194);
        self.fall[0] = 1.0 - self.t190 / (self.t194 + delirium_data::kv!(4));
        self.fall[1] = if self.t190 <= delirium_data::kv!(78) * self.t194 { 1.0 } else { 0.0 };
        (self.s170, self.s168) = (delirium_data::kv!(27) * dt, delirium_data::kv!(27) * dt);
        self.time += dt;
        self.since_hit += dt;
        self.reach_t[0] += dt;
        self.reach_t[1] += dt;
        if self.t190 == self.t194 && self.t194 != 0.0 {
            c.out.send(delirium_data::kv!(55 int), "shot", 0.0);
            self.flags |= 1;
        }
        if c.dbm_failed {
            self.flags |= 1;
        }
        if p.spine_pain_time > 0.0 && !p.melee {
            let k = (p.spine_pain_time - self.since_hit) / p.spine_pain_time;
            if k > 0.0 {
                let mut lean = p.spine_pain_multiplier * k;
                if self.behind > 0.0 {
                    lean *= delirium_data::kv!(18);
                }
                if [(*HIP())[0] + 1, (*KNEE())[0] + 1, (*HIP())[1] + 1, (*KNEE())[1] + 1].contains(&p.body_part) {
                    lean *= delirium_data::kv!(231);
                }
                let tw = self.side * p.spine_pain_twist_multiplier * k * -delirium_data::kv!(31);
                c.spine_all(lean, tw, tw);
                for j in (*SPINE()) {
                    c.ch.eff[j].blend_to_zero(1.0 - lean);
                }
            }
        }
        c.module(100);
        if self.failed() || self.no_balance {
            c.out.dbm = Some(false);
        }
        self.run(c, HIT(), Some(|s, _| s.new_hit), Some(Shot::hit_enter), Some(Shot::hit_tick), Some(|s, _| !s.new_hit), None, dt);
        self.wound = world(&c.b.part(p.body_part).m, self.hit_local);
        c.module(HIT() as u32);
        self.shock_spin(c, dt);
        c.module(101);
        if self.head_t < 0.0 && p.use_head_look {
            self.head_t += dt;
            c.out.head_look_target = Some(p.head_look);
            if self.head_t > 0.0 {
                self.head_t = c.rand(p.head_look_at_wound_min_timer, p.head_look_at_wound_max_timer);
            }
        } else {
            self.head_t -= dt;
            c.out.head_look_target = Some(self.wound);
            if self.head_t < 0.0 {
                self.head_t = -c.rand(p.head_look_at_head_pos_max_timer, p.head_look_at_head_pos_min_timer);
                c.out.send(delirium_data::kv!(127 int), "shot", 0.0);
            }
        }
        self.run(c, CROUCH(), Some(Shot::crouch_check), None, Some(Shot::crouch_tick), Some(|s, _| s.failed()), None, dt);
        c.module(CROUCH() as u32);
        self.run(c, STIFF(), Some(|s, _| s.new_hit && !s.failed()), Some(Shot::stiff_enter), Some(Shot::stiff_tick),
                 Some(|s, _| s.new_hit || s.failed()), Some(Shot::stiff_exit), dt);
        c.module(STIFF() as u32);
        self.run(c, REACT(), Some(|_, c| c.ch.busy != [true; 2]), Some(Shot::react_enter), Some(Shot::react_tick),
                 Some(|s, _| s.new_hit || s.failed()), None, dt);
        c.module(REACT() as u32);
        self.cur_arm = 0;
        self.run(c, LOOSE_L, Some(|s, _| s.time > 1.0 && s.flags & delirium_data::kv!(54 int) != 0 && !s.p.melee), None, Some(Shot::loose_tick),
                 Some(|s, _| s.failed()), None, dt);
        c.module(LOOSE_L as u32);
        self.cur_arm = 1;
        self.run(c, LOOSE_R, Some(|s, _| s.time > 1.0 && s.flags & delirium_data::kv!(23 int) != 0 && !s.p.melee), None, Some(Shot::loose_tick),
                 Some(|s, _| s.failed()), None, dt);
        c.module(LOOSE_R as u32);
        self.run(c, FALL, Some(|s, _| s.failed()), Some(Shot::fall_enter), Some(Shot::fall_tick), None, None, dt);
        c.module(FALL as u32);
        self.run(c, CHICKEN(), Some(|s, _| s.p.chicken_arms && s.new_hit), Some(Shot::chicken_enter), Some(Shot::chicken_tick),
                 Some(|s, _| s.since_hit > s.t174 || s.new_hit), None, dt);
        c.module(CHICKEN() as u32);
        self.run(c, FLING(), Some(|s, _| s.p.fling && s.new_hit), Some(Shot::fling_enter), Some(Shot::fling_tick),
                 Some(|s, _| s.t174 < s.since_hit || s.new_hit), Some(Shot::fling_exit), dt);
        c.module(FLING() as u32);
        self.cur_reach = 0;
        self.run(c, REACH_L(), Some(|s, c| s.reach_check(c, 0)), Some(|s, _| s.reach_enter(0)), Some(Shot::reach_tick),
                 Some(|s, c| s.reach_done(c, 0)), Some(Shot::reach_exit), dt);
        c.module(REACH_L() as u32);
        self.cur_reach = 1;
        self.run(c, REACH_R(), Some(|s, c| s.reach_check(c, 1)), Some(|s, _| s.reach_enter(1)), Some(Shot::reach_tick),
                 Some(|s, c| s.reach_done(c, 1)), Some(Shot::reach_exit), dt);
        c.module(REACH_R() as u32);
        self.run(c, COLLAPSE_L, Some(|s, _| s.flags & delirium_data::kv!(1 int) != 0 && !s.p.melee && s.since_hit > s.p.time_before_collapse_wound_leg),
                 None, Some(|_, c, _| Shot::collapse(c, "ll")), None, None, dt);
        c.module(COLLAPSE_L as u32);
        self.run(c, COLLAPSE_R(), Some(|s, _| s.flags & 2 != 0 && !s.p.melee && s.since_hit > s.p.time_before_collapse_wound_leg),
                 None, Some(|_, c, _| Shot::collapse(c, "lr")), None, None, dt);
        c.module(COLLAPSE_R() as u32);
        self.run(c, LOOK(), Some(Shot::look_check), Some(Shot::look_enter), None,
                 Some(Shot::look_done), Some(|_, c| c.out.head_look_stop = true), dt);
        c.module(LOOK() as u32);
        self.run(c, MELEE(), Some(|s, _| s.p.melee && s.new_hit), Some(Shot::melee_enter), Some(Shot::melee_tick),
                 Some(|s, _| s.failed() || s.new_hit), Some(|_, c| c.out.spine_twist_stop = true), dt);
        c.module(MELEE() as u32);
        self.new_hit = false;
    }

    pub fn deactivate(&mut self, c: &mut Ctx) {
        if self.flags & delirium_data::kv!(251 int) == 0 {
            return;
        }
        let exits: [(usize, Act); 7] = [
            (REACH_L(), Shot::reach_exit), (REACH_R(), Shot::reach_exit), (STIFF(), Shot::stiff_exit),
            (LOOK(), |_, c| c.out.head_look_stop = true), (FALL, Shot::fall_exit), (FLING(), Shot::fling_exit),
            (MELEE(), |_, c| c.out.spine_twist_stop = true),
        ];
        for (k, f) in exits {
            if self.sub[k] {
                self.sub[k] = false;
                f(self, c);
            }
        }
        self.flags &= delirium_data::kv!(467 int);
    }

    fn shock_spin(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.p;
        let there = (*SP()).contains(&p.body_part) || [(*CLAVICLE())[0] + 1, (*SHOULDER())[0] + 1, (*CLAVICLE())[1] + 1, (*SHOULDER())[1] + 1].contains(&p.body_part);
        if !(p.add_shock_spin && (p.always_add_shock_spin || there)) {
            return;
        }
        if self.new_hit && self.spin_blend < 0.0 {
            if p.randomize_shock_spin_direction {
                self.spin = c.rand(p.shock_spin_min, p.shock_spin_max);
                if (c.ch.random() * delirium_data::kv!(526)) as u32 & 1 != 0 {
                    self.spin = -self.spin;
                }
            } else {
                let up = c.up;
                let dot = |v: Vec3| up.x.mul_add(v.x, up.z.mul_add(v.z, up.y * v.y));
                let v = p.hit_point - c.com;
                let mut d = up.mul_add(Vec3::splat(fneg(dot(v))), v);
                if p.body_part == HEAD() && p.melee {
                    d = crate::nmdbm::vnorm(d);
                }
                let r = c.rand(p.shock_spin_min, p.shock_spin_max);
                let x = crate::nmdbm::vnorm(p.bullet_vel).vx(d);
                self.spin = fneg(r * dot(x));
            }
            self.spin_blend = 1.0;
        }
        if self.spin_blend >= 0.0 {
            let axis = crate::nmdbm::vnorm(c.b.joint_pos((*SPINE())[delirium_data::kv!(0 int)]) - c.b.joint_pos((*SPINE())[0])) * (self.spin * self.spin_blend);
            let k = p.shock_spin_scale_per_component;
            c.out.torque((*SP())[delirium_data::kv!(0 int)], axis);
            c.out.torque((*SP())[2], axis * k);
            c.out.torque((*SP())[delirium_data::kv!(1 int)], axis * k);
            c.out.torque((*SP())[1], axis * k * k);
            c.out.torque(p.body_part, axis * k * k);
            if p.shock_spin_lift_force_mult > 0.0 {
                let f = (axis * k * k).abs() * p.shock_spin_lift_force_mult;
                c.out.force((*SP())[delirium_data::kv!(0 int)], f);
                c.out.force((*SP())[delirium_data::kv!(1 int)], f * k);
                c.out.force((*SP())[1], f * k * k);
                c.out.force(p.body_part, f * k * k);
            }
            self.spin_blend = (p.shock_spin_decay_mult * dt).mul_add(-1.0 - self.spin_blend, self.spin_blend);
        }
    }

    fn hit_enter(&mut self, c: &mut Ctx) {
        self.since_hit = 0.0;
        self.behind = fneg(sdot(self.p.bullet_vel, c.m.c, b"yzx"));
        let (mut at, mut normal) = (self.p.hit_point, self.p.normal);
        if self.behind >= 0.0 && !(c.ch.rdr2 && matches!(self.p.body_part, 11 | 12)) {
            let far = at + self.p.bullet_vel.normalize_or_zero() * if c.ch.rdr2 { 0.4 } else { delirium_data::kv!(116) };
            if let Some((k, x, n)) = c.b.doll.probe(c.b.build, far, at) {
                (self.p.body_part, at, normal) = (k, x, n);
            }
        }
        let p = self.p;
        let m = c.b.part(p.body_part).m;
        self.hit_local = local(&m, at);
        self.normal_local = Vec3::new(m.a.vd(normal), m.b.vd(normal), m.c.vd(normal));
        let from = if (*SP()).contains(&p.body_part) { c.b.at(p.body_part) } else { c.com };
        self.side = sdot(at - from, c.m.a, b"zyx").clamp(-2.0, 2.0);
        if c.ch.rdr2 {
            self.intake_rdr2(c, at);
        }
    }

    fn hit_tick(&mut self, c: &mut Ctx, _dt: f32) {
        let p = self.p;
        let part = p.body_part;
        self.grab_front = p.reach_for_wound && self.behind < 0.0 && part != (*SP())[delirium_data::kv!(6 int)] && part != (*SP())[delirium_data::kv!(2 int)];
        let (ll, rl, la, ra) = (limb(2), limb(delirium_data::kv!(0 int)), limb(0), limb(1));
        if ll.p[1..].contains(&part) {
            self.flags |= delirium_data::kv!(1 int);
        }
        if rl.p[1..].contains(&part) {
            self.flags |= 2;
        }
        if !c.ch.rdr2 {
            c.out.dbm_steps = Some(p.max_steps + 1);
        }
        let lhit = la.p.contains(&part);
        let rhit = ra.p.contains(&part);
        let thigh = (part == ll.p[1] || part == rl.p[1]) && self.reach_part == [None; 2];
        if lhit && part != la.p[0] && self.reach_part[0].is_none() {
            self.flags |= delirium_data::kv!(54 int);
        }
        if rhit && part != ra.p[0] && self.reach_part[1].is_none() {
            self.flags |= delirium_data::kv!(23 int);
        }
        let mut r7 = self.side > -delirium_data::kv!(68) && !lhit;
        let mut r8 = self.side <= delirium_data::kv!(68) && !rhit;
        if c.ch.busy[0] && !lhit {
            r8 = true;
        }
        if c.ch.busy[1] && !rhit {
            r7 = true;
        }
        if p.no_reach[0] {
            r7 = false;
        }
        if p.no_reach[1] {
            r8 = false;
        }
        if (rhit || thigh || r7) && self.flags & delirium_data::kv!(54 int) == 0 {
            self.reach_part[0] = Some(part);
            (self.reach_pos[0], self.reach_normal[0], self.reach_t[0]) = (self.hit_local, self.normal_local, 0.0);
            self.flags |= delirium_data::kv!(253 int);
        }
        if (lhit || thigh || r8) && self.flags & delirium_data::kv!(23 int) == 0 {
            self.reach_part[1] = Some(part);
            (self.reach_pos[1], self.reach_normal[1], self.reach_t[1]) = (self.hit_local, self.normal_local, -delirium_data::kv!(78));
            self.flags |= delirium_data::kv!(7 int);
        }
    }

    fn crouch_tick(&mut self, c: &mut Ctx, _dt: f32) {
        if slen(c.vel, b"zyx") < delirium_data::kv!(68) && self.time > delirium_data::kv!(116) {
            c.out.send(delirium_data::kv!(55 int), "shot", 0.0);
        }
    }

    fn stiff_enter(&mut self, c: &mut Ctx) {
        c.ch.set_stiffness(delirium_data::kv!(35), 0.5, "lb", None);
        c.ch.set_stiffness(self.s154 * 0.5, 0.5, "us", Some(delirium_data::kv!(51)));
        c.ch.set_stiffness(self.s15c * 0.5, 0.5, "ua", Some(delirium_data::kv!(51)));
        c.ch.set_stiffness(self.s16c * 0.5, delirium_data::kv!(24), "uw", Some(delirium_data::kv!(51)));
        c.ch.set_stiffness(self.s164 * 0.5, delirium_data::kv!(24), "un", Some(delirium_data::kv!(51)));
    }

    fn stiff_tick(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.p;
        let og = (self.fall[0] + 1.0) * 0.5;
        let both = c.ch.busy == [true; 2];
        for i in 0..2 {
            let l = limb(i);
            for j in &l.e[..delirium_data::kv!(0 int)] {
                c.og(*j, if both { 1.0 } else { og });
            }
            c.og(l.e[delirium_data::kv!(0 int)], og);
        }
        for j in (*SPINE()) {
            c.og(j, self.fall[0] + 1.0);
        }
        for j in [(*SE())[delirium_data::kv!(1 int)], (*SE())[delirium_data::kv!(2 int)]] {
            c.og(j, 1.0);
        }
        for i in 0..2 {
            c.og((*HIP())[i], og);
            c.og((*KNEE())[i], og);
            c.og((*ANKLE())[i], self.fall[1] * 2.0);
        }
        c.ch.set_stiffness(self.fall[1].fma(delirium_data::kv!(32), delirium_data::kv!(64)), (self.fall[1] + 1.0) * 0.5, "lb", None);
        for j in (*ANKLE()) {
            c.ch.eff[j].muscle_stiffness = delirium_data::kv!(31);
            c.ch.eff[j].set_stiffness(self.fall[1].fma(delirium_data::kv!(137), delirium_data::kv!(5)), 1.0, None);
        }
        let f27 = delirium_data::kv!(31) / (delirium_data::kv!(103) * dt) + delirium_data::kv!(31);
        let f29 = p.looseness_amount.fma(self.loose, 1.0 - p.looseness_amount);
        c.ch.set_stiffness(self.s154 * og, self.s158 * f29, "us", Some(f29));
        c.ch.set_stiffness(self.s15c * og, self.s160 * f29, "ua", Some(f29 * 0.5));
        c.ch.set_stiffness(self.s154 * og, self.s158 * f29, "uc", Some(f29));
        c.ch.set_stiffness((self.s16c * og).min(f27), self.s170, "uw", Some(f29 * 0.5));
        c.ch.set_stiffness((self.s164 * og).min(f27), self.s168 * f29, "un", Some(f29 * 0.5));
        if p.stable_hands_and_neck {
            c.ch.set_stiffness(p.body_stiffness + 1.0, 1.0, "uw", None);
            c.ch.set_stiffness(p.body_stiffness + 1.0, 1.0, "un", None);
        }
        if p.reach_for_wound && self.reach_t[0] > p.grab_hold_time && self.reach_t[1] > p.grab_hold_time {
            c.ch.set_stiffness(p.body_stiffness + 1.0, delirium_data::kv!(12), "un", Some(0.5));
            for j in [(*SE())[delirium_data::kv!(1 int)], (*SE())[delirium_data::kv!(2 int)]] {
                c.ch.eff[j].desired = [0.0; 3];
            }
        }
        self.loose = dt.fma(delirium_data::kv!(33), self.loose).min(1.0);
        if self.fall[0] < delirium_data::kv!(24) {
            for j in (*KNEE()) {
                c.ch.eff[j].muscle_stiffness = self.fall[0] * 2.0 + 0.5;
            }
        }
        if both {
            c.ch.set_stiffness(delirium_data::kv!(21), 1.0, "ua", None);
        }
    }

    fn stiff_exit(&mut self, _c: &mut Ctx) {
        if !(self.new_hit && self.loose < 1.0) {
            self.loose = 0.0;
        }
    }

    fn react_enter(&mut self, c: &mut Ctx) {
        self.react_t = 0.0;
        self.react_elbow = ((c.ch.random_bits() & delirium_data::kv!(190 int)) as f32).fma(delirium_data::kv!(544 f32), delirium_data::kv!(116));
        self.react_phase = c.ch.random() - 0.5;
        if self.behind > 0.0 {
            self.react_phase = c.ch.random() * delirium_data::kv!(84) - 2.0;
        }
    }

    fn react_tick(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.p;
        self.react_t += dt;
        let v = c.b.part((*SP())[delirium_data::kv!(1 int)]).v;
        let ratio = (self.t190 + delirium_data::kv!(72)) / (self.t194 + delirium_data::kv!(72));
        let f10 = ((1.0 - ratio) + 1.0) * 0.5;
        let h = c.up * -sdot(c.m.b, c.up, b"yzx") + c.m.b;
        let f13 = (1.0 - slen(h, b"yzx")) * p.p84 * f10;
        let (va, vc) = (sdot(v, c.m.a, b"zyx"), sdot(v, c.m.c, b"zyx"));
        let f28 = deadzone(va - vc, delirium_data::kv!(68));
        let f27 = deadzone(fneg(vc) - va, delirium_data::kv!(68));
        let mut f25 = (f28.abs() * f13 * 0.5).min(1.0);
        let mut f23 = (f27.abs() * f13 * 0.5).min(1.0);
        let mut ph = self.react_t.fma(delirium_data::kv!(64), self.react_phase);
        ph += if self.behind < 0.0 { delirium_data::kv!(545) } else { -delirium_data::kv!(546 f32) };
        let f24 = -((ph as f64).sin() as f32) * delirium_data::kv!(80);
        let s = -delirium_data::kv!(80) * ((ph as f64).cos() as f32);
        let f22 = if f28 < 0.0 { -s } else { s };
        let f12 = if f27 < 0.0 { -s } else { s };
        if p.melee {
            f25 *= delirium_data::kv!(74);
            f23 *= delirium_data::kv!(20);
        }
        for (i, (f, k)) in [(f22, f25), (f12, f23)].into_iter().enumerate() {
            let l = limb(i);
            let tw = if i == 0 { f24.fma(delirium_data::kv!(119), -delirium_data::kv!(119)) * k } else { f24.fma(0.5, -delirium_data::kv!(74)) * k };
            c.ch.eff[l.e[0]].desired[0] = f * k * 0.5;
            c.ch.eff[l.e[0]].desired[1] = f24.fma(k, -delirium_data::kv!(179)) * delirium_data::kv!(116);
            c.ch.eff[l.e[1]].desired = [f * k * delirium_data::kv!(136), (f24 - 2.0).fma(k, delirium_data::kv!(136)) * delirium_data::kv!(119), tw];
            c.ch.eff[l.e[2]].desired[0] = ((f + 2.0) * k).fma(0.5, self.react_elbow);
            for &j in &l.e[..delirium_data::kv!(0 int)] {
                c.og(j, 0.0);
            }
            if k < 0.5 {
                let w = 2.0 * k;
                for j in [l.e[1], l.e[0], l.e[2]] {
                    let a = c.actual(j);
                    let e = &mut c.ch.eff[j];
                    for n in 0..delirium_data::kv!(0 int) {
                        e.desired[n] = (1.0 - w).fma(a[n], e.desired[n] * w);
                    }
                }
            }
            let wr = &mut c.ch.eff[l.e[delirium_data::kv!(0 int)]];
            wr.desired = [wr.desired[0] * 0.5, wr.desired[1] * 0.5, 0.0];
        }
    }

    fn loose_tick(&mut self, c: &mut Ctx, _dt: f32) {
        let l = limb(self.cur_arm);
        let lim = c.limits(l.e[0]);
        c.ch.eff[l.e[0]].desired[1] = lim[2] * 0.5;
        c.ch.eff[l.e[1]].desired = [1.0, delirium_data::kv!(74), 0.5];
        c.ch.eff[l.e[2]].desired[0] = 1.0;
    }

    fn fall_enter(&mut self, c: &mut Ctx) {
        for j in 0..c.ch.eff.len() {
            c.ch.eff[j].reset(&c.ch.joints[j]);
        }
        c.ch.set_stiffness(delirium_data::kv!(64), delirium_data::kv!(18), "lb", None);
        c.ch.set_stiffness(delirium_data::kv!(137), delirium_data::kv!(18), "ub", None);
        for j in 0..c.ch.eff.len() {
            c.ch.eff[j].desired = c.actual(j);
        }
        (self.rd.fall_t, self.rd.fall_cf, self.rd.fall_writhe) = (-10.0, false, false);
    }

    fn fall_exit(&mut self, c: &mut Ctx) {
        c.out.fall_stop = true;
        c.out.fall_stop_rdr2 = c.ch.rdr2;
    }

    fn fall_tick(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.p;
        let k = p.body_stiffness / self.s150;
        if self.new_hit {
            for j in 0..c.ch.eff.len() {
                c.ch.eff[j].reset(&c.ch.joints[j]);
            }
            c.ch.set_stiffness(k * delirium_data::kv!(64), delirium_data::kv!(18), "lb", None);
            c.ch.set_stiffness(k * delirium_data::kv!(137), delirium_data::kv!(18), "ub", None);
        }
        for j in (*SE()) {
            c.og(j, 2.0);
        }
        for j in (*HIP()) {
            c.og(j, 2.0);
        }
        // RDR2: fallingReaction 0 rolls up, 1 catches the fall, 2 rolls down (rollDownStairs), 3 is smartFall from its
        // hard landing, 4 catches the fall then writhes
        let reaction = p.rdr2.falling_reaction.filter(|_| c.ch.rdr2);
        match reaction {
            Some(2) => return self.fall_rds_rdr2(c, dt),
            Some(3) => {
                c.out.smart_fall = true;
                return;
            }
            Some(4) if self.rd.fall_writhe => return self.fall_writhe_rdr2(c),
            _ => {}
        }
        let catch = match reaction {
            Some(r) => r == 1 || r == 4,
            None => p.use_catch_fall_on_fall,
        };
        if !catch {
            c.out.roll_up = Some(k * delirium_data::kv!(137));
        } else {
            let bs = p.body_stiffness;
            let (mut legs, mut torso, mut arms) = (bs * delirium_data::kv!(24), bs * delirium_data::kv!(74), bs * delirium_data::kv!(28));
            if c.ch.rdr2 {
                // RDR2: from the relax ramps (with melee, shares of bodyStiffness)
                let (u, w) = (self.fall[0], self.fall[1]);
                (legs, torso, arms) = if p.melee { (0.3 * bs, 0.7 * bs, 0.9 * bs) } else { ((w + 1.0) * 4.125, (u + 1.0) * 5.0, (u + 1.0) * 6.5) };
            } else if !p.melee {
                let a = (self.fall[0] + 1.0) * 0.5;
                arms *= a;
                torso *= a;
                legs *= (self.fall[1] + 1.0) * 0.5;
            }
            c.out.catch_fall = Some([legs, torso, arms]);
            if let Some((cf, head, st)) = c.cf.take() {
                if !cf.as_ref().is_some_and(|f| f.active) {
                    let p = crate::nmcf::Params { torso_stiffness: torso, legs_stiffness: legs, arms_stiffness: arms, ..Default::default() };
                    *cf = Some(crate::nmcf::CatchFall::start(p, c, head, st));
                }
                // fallingReaction 4: once catchFall has him on the ground, it hands over to bodyWrithe
                if reaction == Some(4) && cf.as_ref().is_some_and(|f| f.active && f.on_ground()) {
                    self.rd.fall_writhe = true;
                }
                c.cf = Some((cf, head, st));
            }
            self.rd.fall_cf = true;
            if self.rd.fall_writhe {
                c.out.catch_fall = None;
                self.fall_writhe_rdr2(c);
            }
        }
    }

    /// RDR2's fallingReaction 2: rollDownStairs with the shot's values, softer (6) 0.4 s after the upper back first
    /// lands.
    fn fall_rds_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        let mut r = crate::nmrds::RollDownStairs::rdr2_defaults();
        (r.damping, r.stiffness, r.z_axis_spin_reduction, r.forcemag, r.asymmetrical_legs) = (1.0, 8.0, 0.0, 0.55, 0.2);
        (r.use_arm_to_slow_down, r.arm_reach_amount, r.arm_reach_length) = (-0.9, 1.4, 0.4);
        r.rdr2 = crate::nmrds::Rdr2Params { muscle_stiffness: 1.0, foetal_legs: false, limit_spin_reduction: true, target_linear_velocity: 1.0,
                                            target_linear_velocity_decay_time: 0.5, ..Default::default() };
        // (RDR2's contact mask 0x102600: we read it as the upper spine, neck and head [G])
        let upper = [delirium_data::kv!(104 int), delirium_data::kv!(81 int), delirium_data::kv!(113 int), delirium_data::kv!(55 int)];
        if self.rd.fall_t < -1.0 && c.b.doll.collided(&upper) {
            self.rd.fall_t = 0.0;
        }
        let soft = (self.rd.fall_t > 0.4).then_some(6.0);
        if self.rd.fall_t >= 0.0 {
            self.rd.fall_t += dt;
        }
        c.out.rds = Some((r, soft));
    }

    /// RDR2's fallingReaction 4 once on the ground: catchFall stopped, bodyWrithe slow and mostly over the held pose
    /// (its own stiffness off), the upper body at bodyStiffness.
    fn fall_writhe_rdr2(&mut self, c: &mut Ctx) {
        if self.rd.fall_cf {
            self.rd.fall_cf = false;
            c.out.fall_stop = true;
        }
        let rdr2 = crate::nmbw::Rdr2Params { apply_stiffness: false, blend_arms: 0.1, blend_back: 0.4, blend_legs: 0.4, ..Default::default() };
        c.out.writhe = Some(crate::nm::BodyWrithe { arm_period: 3.0, back_period: 5.0, leg_period: 4.0, arm_amplitude: 1.0, back_amplitude: 2.0,
                                                    leg_amplitude: 0.5, arm_stiffness: 13.0, back_stiffness: 13.0, leg_stiffness: 11.0,
                                                    arm_damping: 0.3, back_damping: 0.3, leg_damping: 0.3, roll_over: false, rdr2,
                                                    ..Default::default() });
        c.ch.set_stiffness(self.p.body_stiffness, 0.5, "ub", Some(0.5));
    }

    fn chicken_enter(&mut self, _c: &mut Ctx) {
        let k = (1.0 / self.s150) * self.p.body_stiffness;
        self.chicken_t = 0.0;
        (self.s158, self.s160) = (1.0, 1.0);
        (self.s154, self.s15c) = (k * delirium_data::kv!(126), k * delirium_data::kv!(126));
        (self.s16c, self.s164) = (k * delirium_data::kv!(25), k * delirium_data::kv!(25));
    }

    fn chicken_tick(&mut self, c: &mut Ctx, dt: f32) {
        let (l, r) = (limb(0), limb(1));
        let e = &mut c.ch.eff;
        e[l.e[0]].desired = [-delirium_data::kv!(116), -delirium_data::kv!(9), 0.0];
        e[l.e[1]].desired = [0.0, delirium_data::kv!(547), -delirium_data::kv!(140)];
        e[l.e[2]].desired[0] = 1.0;
        e[l.e[delirium_data::kv!(0 int)]].desired = [-delirium_data::kv!(116), delirium_data::kv!(9), delirium_data::kv!(116)];
        e[r.e[0]].desired = [-0.5, -delirium_data::kv!(9), 0.0];
        e[r.e[1]].desired = [delirium_data::kv!(68), delirium_data::kv!(547), -delirium_data::kv!(116)];
        e[r.e[2]].desired[0] = delirium_data::kv!(12);
        e[r.e[delirium_data::kv!(0 int)]].desired = [-delirium_data::kv!(17), delirium_data::kv!(17), delirium_data::kv!(116)];
        for j in &(*SE())[..delirium_data::kv!(0 int)] {
            e[*j].desired[0] = delirium_data::kv!(68);
        }
        e[(*SE())[delirium_data::kv!(0 int)]].desired[0] = -delirium_data::kv!(24);
        e[(*SE())[delirium_data::kv!(2 int)]].desired[0] = -delirium_data::kv!(24);
        e[(*SE())[delirium_data::kv!(2 int)]].desired[1] = -delirium_data::kv!(60);
        e[(*SE())[delirium_data::kv!(1 int)]].desired[0] = -delirium_data::kv!(24);
        self.chicken_t += dt;
        let t = self.chicken_t;
        let k = 1.0 - (t / self.t174).min(1.0);
        let cos = |x: f32| (x as f64).cos() as f32;
        e[l.e[1]].desired[2] = cos(delirium_data::kv!(195) * t).fma(k, e[l.e[1]].desired[2]);
        e[r.e[1]].desired[2] = cos(t.fma(delirium_data::kv!(31), 0.5)).fma(k, e[r.e[1]].desired[2]);
        self.s15c = ((2.0 * t).min(1.0) + 1.0) * delirium_data::kv!(32);
        for j in (*ELBOW()) {
            e[j].set_stiffness(self.s15c, 1.0, None);
        }
    }

    fn fling_enter(&mut self, c: &mut Ctx) {
        let p = self.p;
        (self.f258, self.f264, self.f25c, self.f260) = (0.0, 0.0, 0.0, 0.0);
        self.f278 = [true; 2];
        self.f26c = 0;
        if self.failed() {
            c.ch.set_stiffness(delirium_data::kv!(30), 0.5, "uk", None);
            self.f278 = [false; 2];
            self.f264 = delirium_data::kv!(5);
        } else {
            let part = p.body_part;
            let (ll, rl, la, ra) = (limb(2), limb(delirium_data::kv!(0 int)), limb(0), limb(1));
            self.f26c = if part == ll.p[delirium_data::kv!(0 int)] || part == ll.p[2] {
                1
            } else if part == rl.p[2] || part == rl.p[delirium_data::kv!(0 int)] {
                2
            } else if [la.p[delirium_data::kv!(0 int)], ra.p[delirium_data::kv!(0 int)], la.p[2], ra.p[2]].contains(&part) {
                delirium_data::kv!(0 int)
            } else {
                (self.f264, self.f25c, self.f260) = (1.0, 1.0, 1.0);
                delirium_data::kv!(1 int)
            };
            if self.f26c <= 2 {
                self.no_balance = true;
            }
        }
        let v = p.bullet_vel.normalize_or_zero();
        let m = c.b.part((*SP())[delirium_data::kv!(0 int)]).m;
        let k = (1.0 / self.s150) * p.body_stiffness;
        self.f268 = self.t174 * delirium_data::kv!(548 f32);
        (self.s164, self.s16c) = (k * delirium_data::kv!(31), k * delirium_data::kv!(31));
        self.s154 = k * delirium_data::kv!(21);
        self.s15c = k * delirium_data::kv!(126);
        self.f270 = sdot(v, m.b, b"zyx");
        self.f274 = sdot(v, m.c, b"zyx");
        for i in 0..2 {
            let l = limb(i);
            for j in [l.e[1], l.e[0], l.e[2]] {
                c.og(j, 1.0);
            }
        }
    }

    fn fling_tick(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.p;
        self.f258 += dt;
        if self.failed() {
            c.ch.set_stiffness(delirium_data::kv!(139), 0.5, "uk", Some(0.5));
            c.ch.set_stiffness(delirium_data::kv!(21), 0.5, "ua", Some(0.5));
            c.spine_lean(self.f274 * delirium_data::kv!(32), -self.f270);
            return;
        }
        let a = vnorm(shz(c.m.a, c.up, b"zyx"));
        let d = vnorm(shz(self.wound - c.com, c.up, b"zyx"));
        let f27 = (-(a.x * d.x)).fma(delirium_data::kv!(17), (a.z * d.z).fma(-delirium_data::kv!(17), -((a.y * d.y) * delirium_data::kv!(17))));
        let cos = |x: f32| (x as f64).cos() as f32;
        let sin = |x: f32| (x as f64).sin() as f32;
        c.ch.set_stiffness(delirium_data::kv!(126), 0.5, "ua", Some(delirium_data::kv!(17)));
        c.ch.set_stiffness(delirium_data::kv!(21), 0.5, "uw", None);
        let mut f30 = 0.0;
        let pi = std::f32::consts::PI;
        if self.f278[0] {
            let mut x = (self.f25c.abs() * ((self.f258 / self.f268).fma(pi, fneg(f27)) + delirium_data::kv!(74))).min(delirium_data::kv!(64));
            if x == delirium_data::kv!(64) {
                self.f278[0] = false;
            }
            x = x.clamp(0.0, delirium_data::kv!(549));
            f30 = x;
            let l = limb(0);
            let e = &mut c.ch.eff;
            e[l.e[2]].desired[0] = self.f25c.abs() * (delirium_data::kv!(84) - x).min(delirium_data::kv!(550)) * delirium_data::kv!(128);
            e[l.e[0]].desired[0] = sin(x).fma(delirium_data::kv!(119), p.fling_width.fma(delirium_data::kv!(551), -delirium_data::kv!(60)));
            e[l.e[0]].desired[1] = cos(x) - 0.5;
            e[l.e[1]].desired[0] = sin(x) * 0.5;
            e[l.e[1]].desired[1] = (delirium_data::kv!(12) - p.fling_width) * cos(x);
            e[l.e[1]].desired[2] = sin(x).fma(-delirium_data::kv!(33), -1.0);
        } else {
            c.ch.set_stiffness(delirium_data::kv!(137), 0.5, "ul", None);
        }
        if self.f278[1] {
            let mut x = (self.f260.abs() * ((self.f258 / self.f268).fma(pi, f27) + delirium_data::kv!(74))).min(delirium_data::kv!(552));
            if x == delirium_data::kv!(552) {
                self.f278[1] = false;
            }
            x = x.clamp(0.0, delirium_data::kv!(553));
            f30 = x;
            let r = limb(1);
            let e = &mut c.ch.eff;
            e[r.e[2]].desired[0] = self.f260.abs() * (delirium_data::kv!(84) - x).min(delirium_data::kv!(550)) * delirium_data::kv!(33);
            e[r.e[0]].desired[0] = sin(x).fma(delirium_data::kv!(119), p.fling_width.fma(delirium_data::kv!(10), -delirium_data::kv!(17)));
            e[r.e[0]].desired[1] = cos(x) - 0.5;
            e[r.e[1]].desired[0] = sin(x) * 0.5;
            e[r.e[1]].desired[1] = (delirium_data::kv!(514) - p.fling_width) * cos(x);
            e[r.e[1]].desired[2] = sin(x).fma(-delirium_data::kv!(33), -1.0);
        } else {
            c.ch.set_stiffness(delirium_data::kv!(137), 0.5, "ur", None);
        }
        c.ch.eff[(*SE())[delirium_data::kv!(1 int)]].desired[2] = sin(f30) * -delirium_data::kv!(24);
        c.spine_lean(self.f274 * self.f264 * -delirium_data::kv!(24), self.f270 * delirium_data::kv!(9));
        for j in [(*SE())[delirium_data::kv!(1 int)], (*SE())[delirium_data::kv!(2 int)]] {
            c.ch.eff[j].desired[0] = -(self.f274.abs() * self.f264);
            c.ch.eff[j].desired[1] = self.f270;
        }
        for (n, i) in [(1, 0), (2, 1)] {
            if self.f26c == n && self.f278[i] {
                let l = limb(2 + i);
                let t = c.up.mul_add(Vec3::splat(delirium_data::kv!(119)), c.b.part(l.p[delirium_data::kv!(0 int)]).m.d);
                let side = [1.0, -1.0][i];
                crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[delirium_data::kv!(0 int)], t, 0.0, 1.0, side, -1.0, Vec3::ZERO, None);
            }
        }
        if self.f258 > delirium_data::kv!(60) && (self.f26c == 1 || self.f26c == 2) {
            self.no_balance = false;
        }
    }

    fn fling_exit(&mut self, c: &mut Ctx) {
        for j in [(*SE())[delirium_data::kv!(2 int)], (*SE())[delirium_data::kv!(1 int)]] {
            c.ch.eff[j].desired = [0.0; 3];
        }
        c.spine_lean(0.0, 0.0);
        (self.s154, self.s15c) = (delirium_data::kv!(21), delirium_data::kv!(21));
        (self.s164, self.s16c) = (delirium_data::kv!(554), delirium_data::kv!(554));
    }

    fn reach_check(&self, c: &Ctx, i: usize) -> bool {
        let p = &self.p;
        c.ch.busy != [true; 2] && p.reach_for_wound && self.since_hit > p.time_before_reach_for_wound
            && self.flags & [delirium_data::kv!(253 int), delirium_data::kv!(7 int)][i] != 0 && self.reach_t[i] < p.grab_hold_time && !c.ch.busy[i]
    }

    fn reach_enter(&mut self, i: usize) {
        self.reach[i] = Reach { pos: self.reach_pos[i], normal: self.reach_normal[i], arm: i, part: self.reach_part[i].unwrap_or(0),
                                off: [delirium_data::kv!(60), -delirium_data::kv!(60)][i], sign: 1.0 };
        self.flags &= ![delirium_data::kv!(253 int), delirium_data::kv!(7 int)][i];
    }

    fn reach_done(&self, c: &Ctx, i: usize) -> bool {
        self.flags & [delirium_data::kv!(253 int), delirium_data::kv!(7 int)][i] != 0 || self.reach_t[i] > self.p.grab_hold_time || c.ch.busy[i] || !self.p.reach_for_wound
    }

    fn reach_exit(&mut self, _c: &mut Ctx) {
        let g = self.p.grab_hold_time;
        if self.reach_t[0] > g && self.reach_t[1] > g {
            self.grab_front = false;
        }
    }

    fn reach_tick(&mut self, c: &mut Ctx, dt: f32) {
        let i = self.cur_reach;
        let r = self.reach[i];
        let l = limb(r.arm);
        let lim = c.limits(l.e[0]);
        if [(*SP())[0], (*SP())[1], (*SP())[2]].contains(&r.part) {
            c.ch.eff[l.e[0]].desired[0] = lim[1];
            c.ch.eff[l.e[0]].desired[1] = lim[delirium_data::kv!(0 int)] * 0.5;
        } else {
            c.ch.eff[l.e[0]].desired[0] = lim[1] * 0.5;
            c.ch.eff[l.e[0]].desired[1] = lim[2] * delirium_data::kv!(119);
        }
        for j in [l.e[1], l.e[0], l.e[2]] {
            c.og(j, 1.0);
        }
        let m = c.b.part(r.part).m;
        let t = world(&m, r.pos);
        let n = rot(&m, r.normal) * r.sign;
        let v = vnorm(n - c.m.c);
        let mut dh = c.b.at(l.p[delirium_data::kv!(0 int)]) - t;
        let s = sdot(dh, v, b"zyx");
        dh -= v * if -s >= 0.0 { 0.0 } else { s };
        let len = slen(dh, b"zyx");
        let t2 = v.mul_add(Vec3::splat(len.fma(0.5, -delirium_data::kv!(68)).min(delirium_data::kv!(17))), t);
        let pelvis = c.b.at((*SP())[0]);
        let (a, b) = (t - pelvis, c.b.at((*SP())[delirium_data::kv!(1 int)]) - pelvis);
        let along = (sdot(a, b, b"zyx") / sdot(b, b, b"zyx")).clamp(0.0, delirium_data::kv!(20));
        let mut yaw = (-along).fma(delirium_data::kv!(33), 1.0);
        if yaw < 0.0 {
            yaw *= 1.0 - (len * delirium_data::kv!(33)).min(1.0);
        }
        if [limb(2).p[1], limb(delirium_data::kv!(0 int)).p[1], limb(2).p[2], limb(delirium_data::kv!(0 int)).p[2]].contains(&r.part) {
            yaw = 1.0;
        }
        let side = if i == 0 { 1.0 } else { -1.0 };
        crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[delirium_data::kv!(0 int)], t2, yaw + delirium_data::kv!(78) * side, 0.0, side, 1.0, Vec3::ZERO, None);
        let ret = Self::wrist(c, l, t, n, side);
        if ret < 0.0 && r.part != HEAD() {
            self.reach[i].sign = -self.reach[i].sign;
        }
        let strength = if self.t190 >= self.t194 && self.t194 != 0.0 { 0.5 } else { delirium_data::kv!(84) };
        self.pull(c, l, t, self.reach[i].sign, strength, delirium_data::kv!(78), r.part, dt);
        c.ch.eff[l.e[0]].blend_to_zero(0.5);
    }

    pub(crate) fn wrist(c: &mut Ctx, l: Limb, t: Vec3, n: Vec3, side: f32) -> f32 {
        let (mut sh, el, mut w) = (c.b.eff3(l.e[1]), c.b.eff1(l.e[2]), c.b.eff3(l.e[delirium_data::kv!(0 int)]));
        sh.desired = c.ch.eff[l.e[1]].desired;
        let e = crate::nmdbm::predict_knee(&sh, &el, side);
        use crate::rigid::nrm;
        let fore = t - e;
        let c1 = fore.vx(sh.m.d - e);
        let (pa, pb, pc);
        if slen(c1, b"zyx") > delirium_data::kv!(68) {
            let (d, cn) = (nrm(fore), nrm(c1));
            (pa, pb, pc) = (cn.vx(d), cn, d);
        } else {
            (pa, pb, pc) = (w.m.a, w.m.c.vx(w.m.a), w.m.c);
        }
        let b2 = pb * side;
        let (na, nb, npc) = (sdot(n, pa, b"yzx"), -sdot(n, b2, b"yxz"), sdot(n, pc, b"yzx"));
        let len = nb.fma(nb, na * na).sqrt() + delirium_data::kv!(50);
        let (inv, max) = (1.0 / len, len * delirium_data::kv!(555));
        let tw = ((inv * na) as f64).atan2((inv * nb) as f64) as f32;
        let tw = tw.clamp(-max, max);
        let (s, co) = ((tw as f64).sin() as f32, (tw as f64).cos() as f32);
        let pp = b2.mul_add(Vec3::splat(-co), pa.mul_add(Vec3::splat(s), Vec3::ZERO));
        let phi = (npc as f64).atan2(sdot(n, pp, b"zyx") as f64) as f32;
        w.desired = [-(phi * s), phi * co, tw];
        c.ch.eff[l.e[delirium_data::kv!(0 int)]].desired = w.desired;
        -sdot(n, b2, b"zyx")
    }

    #[allow(clippy::too_many_arguments)]
    fn pull(&self, c: &mut Ctx, l: Limb, t: Vec3, sign: f32, strength: f32, dist: f32, part: usize, dt: f32) {
        let hand = c.b.part(l.p[delirium_data::kv!(0 int)]);
        let at = hand.m.d - hand.m.a * (sign * delirium_data::kv!(118));
        let d = slen(t - at, b"xzy");
        let wrist = c.b.joint_pos(l.e[delirium_data::kv!(0 int)]);
        if d >= delirium_data::kv!(5) * dist {
            return;
        }
        let k = ((delirium_data::kv!(5) * dist - d) / (2.0 * dist)).clamp(0.0, 1.0) * strength;
        let wound = c.b.part(part);
        let wv = wound.v;
        let mut imp = (wv - hand.v).mul_add(Vec3::splat(k * delirium_data::kv!(60)), (t - at) * (k * k));
        if dt < 1.0 / delirium_data::kv!(103) {
            imp *= dt * delirium_data::kv!(103);
        }
        c.out.impulse.push((l.p[delirium_data::kv!(0 int)], wrist, imp));
        c.out.impulse.push((part, t, -imp));
    }

    fn collapse(c: &mut Ctx, mask: &str) {
        c.ch.set_stiffness(delirium_data::kv!(64), 1.0, mask, None);
        c.out.dbm_knee_scale = Some(delirium_data::kv!(556));
        c.out.dbm_acc = Some(-0.5);
    }

    fn look_enter(&mut self, c: &mut Ctx) {
        c.out.head_look_start = Some((self.p.body_stiffness + 1.0, delirium_data::kv!(12)));
    }

    fn melee_enter(&mut self, c: &mut Ctx) {
        let (a, b) = c.ch.stance_yaw;
        (self.m228, self.m250, self.m254) = (0.0, b, delirium_data::kv!(116));
        (self.m248, self.m24c) = (a * 2.0, a * 2.0);
        c.spine_all(0.0, 0.0, 0.0);
    }

    fn melee_tick(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.p;
        let target = if self.m254 > 0.0 {
            self.m254 -= dt;
            let m = c.b.part((*SP())[delirium_data::kv!(6 int)]).m;
            m.c.mul_add(Vec3::splat(-delirium_data::kv!(64)), m.d)
        } else {
            p.head_look
        };
        c.out.head_look_target = Some(target);
        c.out.head_look_activate = true;
        let to = vnorm(shz(vnorm(p.head_look - c.com), c.up, b"zyx"));
        let turn = |a: f32| {
            let sc = |a: f32| if a == 0.0 { (0.0, 1.0) } else { ((a as f64).sin() as f32, (a as f64).cos() as f32) };
            let ((sx, cx), (sy, cy), (sz, cz)) = (sc(c.up.x * a), sc(c.up.y * a), sc(c.up.z * a));
            let (czsy, szsy) = (cz * sy, sz * sy);
            let m = [Vec3::new(cz * cy, sz * cy, -sy),
                     Vec3::new(czsy.fma(sx, -(sz * cx)), szsy.fma(sx, cz * cx), cy * sx),
                     Vec3::new(czsy.fma(cx, sz * sx), szsy.fma(cx, -(cz * sx)), cy * cx)];
            Vec3::new(to.vd(Vec3::new(m[0].x, m[1].x, m[2].x)), to.vd(Vec3::new(m[0].y, m[1].y, m[2].y)), to.vd(Vec3::new(m[0].z, m[1].z, m[2].z)))
        };
        c.out.dbm_turn = Some(turn(self.m248 - self.m250));
        (c.out.dbm_leg_offset, c.out.dbm_acc) = (Some(0.0), Some(0.0));
        c.out.spine_twist = Some((turn(self.m24c - self.m250) + c.com, p.body_stiffness));
        self.m228 = ((-self.m228).mul_add(delirium_data::kv!(78), slen(c.vel, b"yxz")) * (dt * delirium_data::kv!(557))).min(1.0);
        if self.m228 < delirium_data::kv!(78) {
            self.m228 = delirium_data::kv!(134);
        }
        c.spine_lean(0.0, 0.0);
        c.relax((-self.m228).fma(0.5, 1.0), "us");
        let x = (((self.since_hit.clamp(0.0, delirium_data::kv!(246)) * delirium_data::kv!(558)) as f64).sin()) as f32;
        if self.since_hit < delirium_data::kv!(74) {
            c.relax(x.clamp(0.0, 1.0), "ua");
        } else {
            c.relax((-self.m228).fma(delirium_data::kv!(8), 1.0), "ua");
        }
    }
}


// ---- RDR2's shot core (with the RDR2 core), from a study of RDR2's shot (rdr2_shot_core.md) ----

/// RDR2's shot parameters beyond GTA IV's (its defaults). Spine pain and exaggeration are off unless asked for.
#[derive(Clone, Copy, Debug)]
pub struct ShotRdr2 {
    pub exag_duration: f32,
    pub exag_mag: f32,
    pub exag_twist_mag: f32,
    pub exag_undo_rate: f32,
    pub cpain_start: f32,
    pub cpain_rate: f32,
    pub cpain_duration: f32,
    pub cpain_mag: f32,
    pub cpain_twist_mag: f32,
    pub cpain_undo_rate: f32,
    pub arm_stiffness: f32,
    pub spine_damping: f32,
    pub initial_neck_stiffness: f32,
    pub initial_neck_damping: f32,
    pub neck_stiffness: f32,
    pub neck_damping: f32,
    pub k_mult_on_loose: f32,
    pub k_mult_4_legs: f32,
    pub initial_weakness_ramp: f32,
    pub initial_weakness_zero: f32,
    pub initial_neck_ramp: f32,
    pub initial_neck_duration: f32,
    pub spine_blend_zero: f32,
    pub min_arms_looseness: f32,
    pub min_legs_looseness: f32,
    /// 0 bodyRollUp, 1 catchFall, 2 rollDownStairs, 3 smartFall, 4 catchFall then bodyWrithe (None: GTA IV's
    /// useCatchFallOnFall picks 1 or 0)
    pub falling_reaction: Option<i32>,
    /// RDR2's shotConfigureArms for the default arms: enableDefaultArms, brace, useArmsWindmill, alwaysReachTime,
    /// AWSpeedMult, AWRadiusMult, AWStiffnessAdd, releaseWound (0 never, 1 only with brace, 2 always)
    pub default_arms: bool,
    pub brace: bool,
    pub use_arms_windmill: bool,
    pub always_reach_time: f32,
    pub aw_speed_mult: f32,
    pub aw_radius_mult: f32,
    pub aw_stiffness_add: f32,
    pub release_wound: i32,
}

impl Default for ShotRdr2 {
    fn default() -> ShotRdr2 {
        ShotRdr2 { exag_duration: 0.0, exag_mag: 1.0, exag_twist_mag: 0.5, exag_undo_rate: 2.5, cpain_start: 0.3, cpain_rate: 2.0,
                   cpain_duration: 0.0, cpain_mag: 1.0, cpain_twist_mag: 0.5, cpain_undo_rate: 0.85, arm_stiffness: 10.0,
                   spine_damping: 1.0, initial_neck_stiffness: 14.0, initial_neck_damping: 1.0, neck_stiffness: 14.0, neck_damping: 1.0,
                   k_mult_on_loose: 0.0, k_mult_4_legs: 0.3, initial_weakness_ramp: 0.4, initial_weakness_zero: 0.0,
                   initial_neck_ramp: 0.4, initial_neck_duration: 0.0, spine_blend_zero: 0.6, min_arms_looseness: 0.1,
                   min_legs_looseness: 0.1, falling_reaction: None, default_arms: true, brace: true, use_arms_windmill: true,
                   always_reach_time: 0.3, aw_speed_mult: 1.0, aw_radius_mult: 1.0, aw_stiffness_add: 4.0, release_wound: 1 }
    }
}

/// RDR2's shot core state: the weakness and neck ramps (restarted by every bullet), the lower relax timer, the
/// bullet's lean and twist, the three spine ramps {target, current, rate, diff} and when they last wrote.
#[derive(Clone, Copy, Debug, Default)]
struct ShotRdr2State {
    started: bool,
    k: f32,
    kn: f32,
    tk: f32,
    tn: f32,
    t_low: f32,
    lean: [f32; 3],
    ramps: [[f32; 4]; 3],
    wrote: f32,
    /// the fall's rollDownStairs timer (from the upper back's first landing), catchFall started, writhe taken over
    fall_t: f32,
    fall_cf: bool,
    fall_writhe: bool,
}

/// RDR2's relax: shotRelax's lower-body period with its default (ours: our relax message gives only the upper one).
const RDR2_RELAX_LOWER: f32 = 0.4;

/// RDR2's strength block (the shot's, shared with meleeReact): the weakness k, the base stiffnesses, the neck's
/// stiffness and damping, the upper and lower strengths U and W, looseness and its multipliers, and whether the
/// balancer is running (its leg stiffness then scaled up as the legs loosen).
#[derive(Clone, Copy, Debug)]
pub(crate) struct Strength {
    pub k: f32,
    pub body_stiffness: f32,
    pub arm_stiffness: f32,
    pub spine_damping: f32,
    pub neck: (f32, f32),
    pub upper: f32,
    pub lower: f32,
    pub looseness: f32,
    pub k_mult_on_loose: f32,
    pub k_mult_4_legs: f32,
    pub min_legs_looseness: f32,
    pub min_arms_looseness: f32,
    pub stable_hands_and_neck: bool,
    pub balancing: bool,
}

/// RDR2's biped strength applier: the trunk, legs, arms, neck and wrists from a strength block, looseness-blended
/// (L = looseness k + 1 - looseness); opposeGravity from U.
pub(crate) fn apply_strength(c: &mut Ctx, s: &Strength, dt: f32) {
    let (k, u, w) = (s.k, s.upper, s.lower);
    let (ns, nd) = s.neck;
    let h = (u + 1.0) * 0.5;
    let l = s.looseness * k + (1.0 - s.looseness);
    let m = ((1.0 - l) * s.k_mult_on_loose + 1.0) * h;
    let lc = l.clamp(0.05, 1.0);
    for i in 0..2 {
        for j in limb(i).e {
            c.og(j, h);
        }
    }
    for &j in SPINE().iter() {
        c.og(j, u + 1.0);
    }
    for &j in crate::nmrs::NECK().iter() {
        c.og(j, 1.0);
    }
    c.ch.set_stiffness(m * s.body_stiffness, lc * s.spine_damping, "us", Some(lc));
    let lg = s.min_legs_looseness.max(lc).min(2.0);
    for i in 0..2 {
        c.ch.eff[(*KNEE())[i]].muscle_stiffness = 1.5 * lg;
        c.ch.eff[(*HIP())[i]].muscle_stiffness = 1.5 * lg;
    }
    if s.balancing {
        // the balancer's leg stiffness, scaled up as the legs loosen (kMult4Legs)
        let ml = ((1.0 - lg) * s.k_mult_4_legs + 1.0) * (w + 1.0) * 0.5;
        for i in 0..2 {
            let knee = &mut c.ch.eff[(*KNEE())[i]];
            let base = knee.strength.sqrt();
            let sigma = (ml * base).powi(2);
            knee.strength = sigma;
            // the hip takes the same multiplier on the balancer's stiffness, whatever the knee's taper
            let hip = &mut c.ch.eff[(*HIP())[i]];
            hip.strength = (ml * hip.strength.sqrt()).powi(2);
            if w < 1.0 {
                let st = (base - 5.0) * w + 5.0;
                let damp = (w + 1.0) * 0.5;
                c.ch.eff[(*HIP())[i]].set_stiffness(st.min(sigma.sqrt()), damp, None);
                c.ch.eff[(*KNEE())[i]].set_stiffness(st.min(sigma.sqrt()), damp, None);
                c.ch.eff[(*ANKLE())[i]].set_stiffness((base - 3.0) * w + 3.0, 1.0, None);
            }
        }
    }
    let la = s.min_arms_looseness.max(lc).min(2.0);
    let (mut o, mut d) = ([0.0f32; 2], [0.0f32; 2]);
    if c.ch.busy[0] {
        (o, d) = ([0.5, -0.5], [-0.2, 0.2]);
    }
    if c.ch.busy[1] {
        (o, d) = ([-0.5, 0.5], [0.2, -0.2]);
    }
    c.ch.set_stiffness((o[0] + s.arm_stiffness) * m, (d[0] + 1.0) * lc, "ul", Some(la));
    c.ch.set_stiffness((o[1] + s.arm_stiffness) * m, (d[1] + 1.0) * lc, "ur", Some(la));
    if s.stable_hands_and_neck {
        c.ch.set_stiffness(ns, nd, "un", Some(1.5));
    } else {
        c.ch.set_stiffness((ns * m).min(1.0 / (6.0 * dt) + 10.0), nd * lc, "un", Some(1.5 * lc));
    }
    c.ch.set_stiffness(13.0, 1.0, "uw", Some(1.0));
    if u < 0.3 {
        for &j in KNEE().iter() {
            c.ch.eff[j].muscle_stiffness = 2.0 * u + 0.5;
        }
    }
}

impl Shot {
    /// RDR2's bullet intake for the core: the bullet's lean and twist over the body, the weakness and neck ramps
    /// restarted (alwaysResetLooseness), the exaggeration set going.
    fn intake_rdr2(&mut self, c: &mut Ctx, at: Vec3) {
        let (p, up) = (self.p, c.up);
        let r = p.rdr2;
        let flat = |v: Vec3| horizontal(v, up).normalize_or_zero();
        let d = flat(p.bullet_vel);
        let part = if (*SP()).contains(&p.body_part) { p.body_part } else { (*SP())[4] };
        let pm = c.b.part(part).m;
        let leg = (c.b.joint_pos((*HIP())[0]) - c.b.joint_pos((*ANKLE())[0])).length().max(0.1);
        let lever = pm.a.vd((at - pm.d).vx(p.bullet_vel.normalize_or_zero())) / leg;
        let mut lean = [-flat(c.m.c).vd(d), -flat(c.m.a).vd(d), lever];
        if p.body_part <= 8 {
            (lean[0], lean[1]) = (-lean[0], -lean[1]);
        }
        let s = &mut self.rd;
        (s.lean, s.started, s.k, s.kn, s.tk, s.tn, s.wrote) = (lean, true, 0.01, 0.01, 0.0, 0.0, 0.0);
        if r.exag_duration > f32::EPSILON {
            let cur = [lean[0] * r.exag_mag, lean[1] * r.exag_mag, lean[2] * r.exag_twist_mag * 10.0];
            for (ch, x) in s.ramps.iter_mut().zip(cur) {
                *ch = [0.0, x, r.exag_undo_rate, -x];
            }
        }
    }

    /// RDR2's exaggeration and spine pain: three rate-limited ramps written to the four spine joints (a quarter
    /// each). Returns whether they wrote this tick.
    fn spine_rdr2(&mut self, c: &mut Ctx, dt: f32) -> bool {
        let (p, t) = (self.p, self.since_hit);
        let r = p.rdr2;
        let step = |ch: &mut [f32; 4]| {
            ch[3] = ch[0] - ch[1];
            ch[1] += ch[3].clamp(-ch[2] * dt, ch[2] * dt);
        };
        let mut stepped = false;
        if t < r.cpain_start || r.cpain_duration <= f32::EPSILON {
            if r.exag_duration > f32::EPSILON && t > r.exag_duration {
                for ch in self.rd.ramps.iter_mut().filter(|ch| ch[3].abs() >= 0.01) {
                    step(ch);
                    stepped = true;
                }
            }
        } else {
            if t <= r.cpain_start + dt {
                // from the spine as it is: four times the joints' mean, i.e. their sum
                let now: Vec<[f32; 3]> = SPINE().iter().map(|&j| c.actual(j)).collect();
                for k in 0..3 {
                    self.rd.ramps[k][1] = now.iter().map(|a| a[k]).sum();
                }
                let mut mag = r.cpain_mag;
                if self.behind > 0.0 {
                    mag *= 0.75;
                }
                if [1, 2, 4, 6].contains(&p.body_part) {
                    mag *= 1.25;
                }
                let tw = self.side * -10.0 * r.cpain_twist_mag;
                for (ch, x) in self.rd.ramps.iter_mut().zip([mag, tw, tw]) {
                    (ch[0], ch[2]) = (x, r.cpain_rate);
                }
            }
            let end = r.cpain_start + r.cpain_duration;
            if t >= end && t <= end + dt {
                for ch in self.rd.ramps.iter_mut() {
                    (ch[0], ch[2]) = (0.0, r.cpain_undo_rate);
                }
            }
            for ch in self.rd.ramps.iter_mut() {
                step(ch);
            }
            stepped = true;
        }
        if stepped && !p.melee {
            let v = self.rd.ramps.map(|ch| (0.25 * ch[1]).clamp(-9.9, 9.9));
            for &j in SPINE().iter() {
                c.ch.eff[j].desired = v;
            }
            self.rd.wrote = t;
        }
        stepped
    }

    /// RDR2's strength: the bullet weakness k (0.01 back to 1 over initialWeaknessRampDuration after each bullet)
    /// loosens the trunk, arms, neck and legs through looseness-blended stiffness, damping and muscle; the legs work
    /// with the balancer's own leg stiffness; opposeGravity from the upper relax U.
    fn strength_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.p;
        let r = p.rdr2;
        let (u, w) = (self.fall[0], self.fall[1]);
        let s = &mut self.rd;
        if r.initial_weakness_zero <= s.tk && r.initial_weakness_ramp <= 1e-6 {
            s.k = 1.0;
        }
        if r.initial_neck_duration <= s.tn && r.initial_neck_ramp <= 1e-6 {
            s.kn = 1.0;
        }
        let (k, kn) = (s.k, s.kn);
        if !self.failed() {
            let ns = r.initial_neck_stiffness + (r.neck_stiffness - r.initial_neck_stiffness) * kn;
            let nd = r.initial_neck_damping + (r.neck_damping - r.initial_neck_damping) * kn;
            apply_strength(c, &Strength { k, body_stiffness: p.body_stiffness, arm_stiffness: r.arm_stiffness, spine_damping: r.spine_damping,
                                          neck: (ns, nd), upper: u, lower: w, looseness: p.looseness_amount, k_mult_on_loose: r.k_mult_on_loose,
                                          k_mult_4_legs: r.k_mult_4_legs, min_legs_looseness: r.min_legs_looseness,
                                          min_arms_looseness: r.min_arms_looseness, stable_hands_and_neck: p.stable_hands_and_neck,
                                          balancing: !self.no_balance && !c.dbm_failed }, dt);
        }
        let s = &mut self.rd;
        if r.initial_weakness_zero <= s.tk {
            s.k = if r.initial_weakness_ramp <= 1e-6 { 1.0 } else { (s.k + dt / r.initial_weakness_ramp).min(1.0) };
        }
        if r.initial_neck_duration <= s.tn {
            s.kn = if r.initial_neck_ramp <= 1e-6 { 1.0 } else { (s.kn + dt / r.initial_neck_ramp).min(1.0) };
        }
        (s.tk, s.tn) = (s.tk + dt, s.tn + dt);
    }

    /// RDR2's shot tick: relax U (upper) and W (lower), the falling decision, the spine ramps, the strength
    /// sub-module from the first bullet, the fall (fallingReaction) and the arm shapes, then the spine blend.
    fn tick_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.p;
        self.t190 = (self.t190 + dt).min(self.t194);
        self.fall[0] = 1.0 - self.t190 / (self.t194 + 1e-6);
        let low = if self.t194 > 0.0 { RDR2_RELAX_LOWER } else { 0.0 };
        self.rd.t_low = (self.rd.t_low + dt).min(low);
        self.fall[1] = 1.0 - self.rd.t_low / (low + 1e-6);
        self.time += dt;
        self.since_hit += dt;
        if self.t190 == self.t194 && self.t194 != 0.0 && !self.failed() {
            c.out.send(delirium_data::kv!(55 int), "shot", 0.0);
            self.flags |= 1;
        }
        if c.dbm_failed {
            self.flags |= 1;
        }
        if self.failed() || self.no_balance {
            c.out.dbm = Some(false);
        }
        self.run(c, HIT(), Some(|s, _| s.new_hit), Some(Shot::hit_enter), Some(Shot::hit_tick), Some(|s, _| !s.new_hit), None, dt);
        self.wound = world(&c.b.part(p.body_part).m, self.hit_local);
        c.module(HIT() as u32);
        let stepped = self.spine_rdr2(c, dt);
        c.module(100);
        self.shock_spin(c, dt);
        c.module(101);
        if self.rd.started {
            self.strength_rdr2(c, dt);
        }
        self.run(c, REACT(), Some(|s, _| s.p.rdr2.default_arms && !s.failed()), Some(Shot::arms_enter), Some(Shot::arms_tick),
                 Some(|s, _| s.new_hit || s.failed() || !s.p.rdr2.default_arms), Some(Shot::arms_exit), dt);
        c.module(REACT() as u32);
        self.run(c, FALL, Some(|s, _| s.failed()), Some(Shot::fall_enter), Some(Shot::fall_tick), None, None, dt);
        c.module(FALL as u32);
        self.run(c, CHICKEN(), Some(|s, _| s.p.chicken_arms && s.new_hit), Some(Shot::chicken_enter), Some(Shot::chicken_tick),
                 Some(|s, _| s.since_hit > s.t174 || s.new_hit), None, dt);
        c.module(CHICKEN() as u32);
        // fling: its start needs T < +0x5a0, a field only its own enter fills, so with fling = 1 it doesn't start; recorded
        // +0x133 = 1, +0x5a0 = 0 and +0x782 = 0 on every tick of both shots
        c.module(FLING() as u32);
        self.run(c, LOOK(), Some(|s, _| s.p.use_head_look), Some(Shot::look_enter), None,
                 Some(|s, _| !s.p.use_head_look), Some(|_, c| c.out.head_look_stop = true), dt);
        c.module(LOOK() as u32);
        self.run(c, MELEE(), Some(|s, _| s.p.melee && s.new_hit), Some(Shot::melee_enter), Some(Shot::melee_tick),
                 Some(|s, _| s.failed() || s.new_hit), Some(|_, c| c.out.spine_twist_stop = true), dt);
        c.module(MELEE() as u32);
        if !stepped {
        }
        c.module(102);
        self.new_hit = false;
    }
}

// ---- RDR2's default arms (configureArms runner 2, with the RDR2 core), from rdr2_shot_defaultarms.md ----

/// The default arms' state: the brace phase clocks, the windmill timers and which arms windmill, releaseWound's
/// hysteresis timers and released hands, the hold blend's clavicle, shoulder and elbow states per arm, and the
/// armsWindmill it runs.
#[derive(Clone, Copy, Debug, Default)]
struct DefaultArms {
    t: [f32; 2],
    wm_t: [f32; 2],
    wm: [bool; 2],
    release_t: [f32; 2],
    release: [bool; 2],
    hold: [[f32; 7]; 2],
    aw: Option<crate::nmwm::Windmill>,
}

/// One armsWindmill circle as the shot configures it: centre in the chest's frame, the frame's three turns (rad),
/// the radii in leg lengths, the speed in turns a second.
struct Circle {
    centre: Vec3,
    turn: Vec3,
    radii: [f32; 2],
    speed: f32,
}

/// The shot's armsWindmill: the left circle, and the right one's own radii and speed (mirrorMode 1 builds its frame
/// and centre from the left one).
const AW_CIRCLES: [Circle; 2] = [
    Circle { centre: Vec3::new(0.0, 0.5, -0.3), turn: Vec3::new(0.0, 0.2, 0.2), radii: [0.15, 0.2], speed: -0.5 },
    Circle { centre: Vec3::new(0.0, 0.5, -0.1), turn: Vec3::new(0.0, -0.2, -0.2), radii: [0.3, 0.1], speed: -0.3 },
];
/// The shot's armsWindmill shoulder and elbow stiffness (both damping 1), phaseOffset in degrees, elbow floor.
const AW_STIFFNESS: f32 = 9.0;
const AW_PHASE: f32 = 60.0;
const AW_ELBOW_MIN: f32 = 0.5;

impl Shot {
    fn arms_enter(&mut self, c: &mut Ctx) {
        self.da.t = [c.ch.random(), c.ch.random()];
    }

    fn arms_exit(&mut self, _c: &mut Ctx) {
        self.da.aw = None;
        self.da.release = [false; 2];
    }

    /// RDR2's default arms: an arm not reaching for the wound (or let go of it) windmills while he is thrown back
    /// fast, else braces (a reach towards the ground the way he moves) or, with brace off, holds the guard pose; for
    /// 0.4 s after a reach ends the arm moves to that pose at 1 rad/s. releaseWound lets a reaching hand go when he
    /// moves fast and not backwards.
    fn arms_tick(&mut self, c: &mut Ctx, dt: f32) {
        let r = self.p.rdr2;
        let u = self.fall[0];
        let up = c.up;
        let rfw = c.rfw;
        let reach = rfw.map_or([false; 2], |v| v.reached);
        for t in self.da.t.iter_mut() {
            *t += dt;
        }
        // the chest's velocity, turned by the trunk's pitching and lessened when he leans back
        let mid = c.b.part(SP()[2]).m;
        let top = c.b.part(SP()[4]);
        let mut vc = top.v * 0.6;
        let w = 0.25 * c.angvel.dot(mid.b).clamp(0.0, 4.0);
        vc -= mid.c * (vc.length() * w);
        vc += mid.a * (0.5 * vc.length() * w);
        if up.dot(mid.c) > 0.5 {
            vc -= mid.c * vc.length();
        }
        let spd = vc.length();
        let back = vc.normalize_or_zero().dot(mid.c);
        let side = c.vel.normalize_or_zero().dot(top.m.a);
        let draw = [c.ch.random(), c.ch.random()];
        let tl = draw.map(|x| 0.5 + 0.5 * x);
        let wm_on = r.use_arms_windmill && back > 0.3 && spd > 0.6;
        if wm_on {
            for t in self.da.wm_t.iter_mut() {
                *t += dt;
            }
            let live = self.da.aw.map_or([true; 2], |a| a.live);
            self.da.wm = [!reach[0] && self.da.wm_t[0] < tl[0] && side < 0.7 && live[0],
                          !reach[1] && self.da.wm_t[1] < tl[1] && side > -0.7 && live[1]];
        } else {
            self.da.aw = None;
            self.da.wm = [false; 2];
            for i in 0..2 {
                if self.da.wm_t[i] > tl[i] {
                    self.da.wm_t[i] += dt;
                    if self.da.wm_t[i] > 0.8 + 0.5 * draw[i] + 0.9 * c.ch.random() {
                        self.da.wm_t[i] = 0.0;
                    }
                }
            }
        }
        // releaseWound
        if ((r.release_wound == 1 && r.brace) || r.release_wound == 2) && back < 0.2 && spd > 0.8 {
            let legs = [limb(2), limb(3)];
            let injured = legs.iter().any(|l| l.p[1..].contains(&self.p.body_part));
            let on = rfw.map_or([0.0; 2], |v| v.on);
            let mut want = [reach[0] && on[0] > r.always_reach_time, reach[1] && on[1] > r.always_reach_time];
            let f = c.b.part(SP()[0]).m.b.dot(c.vel.normalize_or_zero());
            if f > 0.3 {
                want[0] &= !injured || c.ch.random() > 0.9;
            }
            if f < -0.3 {
                want[1] &= !injured || c.ch.random() > 0.9;
            }
            for i in 0..2 {
                self.da.release_t[i] -= dt;
                if self.da.release_t[i] < 0.0 && want[i] != self.da.release[i] {
                    self.da.release[i] = want[i];
                    self.da.release_t[i] = if want[i] { 0.4 } else { 0.6 };
                }
            }
        }
        c.out.rfw_release = self.da.release;
        let window = rfw.map_or([false; 2], |v| v.since_stop.map(|t| (0.0..0.4).contains(&t)));
        for i in 0..2 {
            if c.ch.busy[i] {
                continue;
            }
            let free = !reach[i] || self.da.release[i];
            let mut posed = None;
            if r.brace && !self.da.wm[i] && free {
                posed = Some(self.brace_arm(c, i, u, window[i]));
            }
            if !self.da.wm[i] && posed.is_none() && free {
                posed = Some(self.guard_arm(c, i, u, window[i]));
            }
            let l = limb(i);
            let hold = &mut self.da.hold[i];
            if posed == Some(true) {
                for (k, j) in [(0, l.e[0]), (3, l.e[1])] {
                    let tgt = c.ch.eff[j].desired;
                    let d = [tgt[0] - hold[k], tgt[1] - hold[k + 1], tgt[2] - hold[k + 2]];
                    let m = d.iter().fold(0.0f32, |a, x| a.max(x.abs()));
                    let f = if m >= 1.2e-7 { (dt / m).min(1.0) } else { 1.0 };
                    for n in 0..3 {
                        hold[k + n] += f * d[n];
                    }
                    c.ch.eff[j].desired = [hold[k], hold[k + 1], hold[k + 2]];
                }
                let tgt = c.ch.eff[l.e[2]].desired[0];
                hold[6] += (tgt - hold[6]).clamp(-dt, dt);
                c.ch.eff[l.e[2]].desired[0] = hold[6];
            } else if window[i] {
                let (cl, sh, el) = (c.actual(l.e[0]), c.actual(l.e[1]), c.actual(l.e[2]));
                *hold = [cl[0], cl[1], cl[2], sh[0], sh[1], sh[2], el[0]];
            }
        }
        c.module(REACT() as u32);
        if wm_on {
            self.windmill(c, r, u, spd, dt);
            c.mark(0x58);
        }
    }

    /// RDR2's brace arm: the hand thrown towards the ground the way the chest moves (out to the side, ahead or
    /// behind as the trunk and thigh frames say), 0.81..1.03 arm lengths from the shoulder, the wrist bent back near
    /// the ground; when he is nearly still the arm relaxes to the zero pose. Returns `window` when it relaxed.
    fn brace_arm(&mut self, c: &mut Ctx, i: usize, u: f32, window: bool) -> bool {
        let sg = if i == 0 { -1.0 } else { 1.0 };
        let t = self.da.t[i];
        let rho = if i == 0 { 0.812 + 0.223 * c.ch.random() } else { 0.809 + 0.216 * c.ch.random() };
        let (l, leg) = (limb(i), limb(2 + i));
        let up = c.up;
        let sp = c.b.part(SP()[2]).m;
        let (nr1, nr2) = (horizontal(-sp.b, up), horizontal(-sp.c, up));
        let feet = c.b.part(limb(2).p[3]).m.d.dot(up) + c.b.part(limb(3).p[3]).m.d.dot(up);
        let h = (c.b.part(l.p[3]).m.d.dot(up) - 0.5 * feet).max(0.0);
        let vh = horizontal(c.b.part(SP()[4]).v, up);
        let lm = c.b.part(leg.p[1]).m;
        let (r0, lr2) = (horizontal(lm.a, up), horizontal(-lm.c, up));
        let mut a = r0.dot(vh) + 0.1 * i as f32;
        if a.abs() < 0.4 {
            a = 0.4 * sg;
        }
        let b = lr2.dot(vh) + 0.1;
        let cc = if b >= 0.0 { b } else { -0.5 * b };
        if a * sg < 0.0 {
            a *= 0.8 * (3.0 * t).sin();
        }
        let dir = if b >= 0.0 { nr1 * a + nr2 * cc - up } else { r0 * (0.5 * a) + lr2 * cc - up };
        let sh = c.b.joint_pos(l.e[1]);
        let el = c.b.joint_pos(l.e[2]);
        let len = (el - sh).length() + (c.b.joint_pos(l.e[3]) - el).length();
        let target = sh + dir.normalize_or_zero() * (rho * len).min(dir.length());
        let side = if i == 0 { 1.0 } else { -1.0 };
        crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[3], target, 0.2, 0.0, side, 1.0, Vec3::ZERO, None);
        let e = &mut c.ch.eff[l.e[2]].desired[0];
        *e = e.max(0.3);
        if b < 0.0 {
            *e += 0.5 * (5.0 * t).sin();
        }
        c.ch.eff[l.e[0]].desired = [0.0; 3];
        c.ch.eff[l.e[3]].desired = [0.0, if h < 0.7 { -2.5 * (0.7 - h) } else { 0.0 }, 0.2];
        let v = c.vel.length();
        let mut ret = false;
        if v < 0.5 {
            for j in [l.e[0], l.e[1], l.e[2]] {
                c.ch.eff[j].blend_to_zero(1.0 - 2.0 * v);
            }
            c.ch.eff[l.e[3]].blend_to_zero(1.0);
            ret = window;
        }
        let s = (11.0 * u).max(1.0);
        let d = if ret { 1.0 } else { 0.7 };
        for j in l.e {
            c.ch.eff[j].set_stiffness(s, d, None);
        }
        c.ch.eff[l.e[2]].set_stiffness(0.75 * s, 0.75 * d, None);
        c.ch.eff[l.e[3]].set_stiffness((s - 1.0).max(7.0), 1.75, None);
        ret
    }

    /// RDR2's default arm pose (brace off): a guard driven by how fast the chest moves against the pelvis, relaxing
    /// to the zero pose with the shoulder a little out and the elbow bent when it doesn't. Returns `window` when it
    /// posed the arm (bodyBalance running leaves the angles to it).
    fn guard_arm(&mut self, c: &mut Ctx, i: usize, u: f32, window: bool) -> bool {
        let sg = if i == 0 { -1.0 } else { 1.0 };
        let l = limb(i);
        let (top, pel) = (c.b.part(SP()[4]), c.b.part(SP()[0]));
        let dv = top.v - pel.v;
        let d = (top.m.d - pel.m.d) + dv * 0.6;
        let f = (dv.length() - 0.15).clamp(0.0, 1.0);
        for j in [l.e[0], l.e[1], l.e[3]] {
            c.ch.eff[j].set_stiffness((2.0 * f + 8.0) * u, 1.0, None);
        }
        c.ch.eff[l.e[2]].set_stiffness((2.0 * f + 4.0) * u, 1.0, None);
        c.ch.eff[l.e[3]].set_stiffness(10.0, 1.0, None);
        let m = c.b.eff3(l.e[1]).m;
        let x = 4.0 * m.a.dot(d);
        let y = (4.0 * 0.707 * sg * (m.b - m.c).dot(d)).clamp(-8.0, 8.0);
        let mut q = -x.sin();
        if q < 0.0 {
            q *= 0.7;
        }
        let rr = x.cos() + y - 0.5 * f;
        c.og(l.e[0], f);
        c.og(l.e[1], f);
        if c.bb_on {
            return false;
        }
        let g = 1.0 - f;
        let (ls, lc) = (c.limits(l.e[1]), c.limits(l.e[0]));
        c.ch.eff[l.e[1]].desired = [(q + 0.5 * (ls[0] + ls[1])) * f, (rr + 0.5 * (ls[2] + ls[3])) * f + 0.2 * g, 0.0];
        c.ch.eff[l.e[0]].desired = [0.0, (rr + 0.5 * (lc[2] + lc[3])) * f, 0.0];
        c.ch.eff[l.e[2]].desired[0] = f + 0.7 * g;
        for j in [l.e[0], l.e[1], l.e[3]] {
            c.ch.eff[j].blend_to_zero(g);
        }
        c.ch.eff[l.e[3]].blend_to_zero(1.0);
        window
    }

    /// RDR2's armsWindmill as the shot runs it (nmwm.rs): started with each arm's phase from where its hand is, the
    /// circles' radii, speeds, stiffness and phase gap grown with how fast he goes back; each arm circles its hand on
    /// an ellipse in the chest's frame (the right one mirrored from the left), the two kept a phase apart, and stops
    /// for good once its hand touches something.
    fn windmill(&mut self, c: &mut Ctx, r: ShotRdr2, u: f32, spd: f32, dt: f32) {
        let leg = (c.b.joint_pos((*HIP())[0]) - c.b.joint_pos((*ANKLE())[0])).length().max(0.1);
        let k = &AW_CIRCLES;
        let p = crate::nmwm::Params {
            normal: [k[0].turn, k[1].turn], centre: [k[0].centre, k[1].centre],
            radius: [[k[0].radii[0] * leg, k[0].radii[1] * leg], [k[1].radii[0] * leg, k[1].radii[1] * leg]],
            speed: [k[0].speed, k[1].speed], shoulder: (AW_STIFFNESS, 1.0), elbow: (AW_STIFFNESS, 1.0), phase_offset: AW_PHASE,
            elbow_min: [AW_ELBOW_MIN; 2], use_arm: self.da.wm, ..Default::default()
        };
        let mut aw = self.da.aw.unwrap_or_else(|| crate::nmwm::Windmill::start(c, &p));
        let e = spd - 0.5;
        let rad = 0.3 * (e * r.aw_radius_mult).clamp(0.0, 1.5);
        aw.delta = [[rad * leg; 2], [rad; 2]];
        let sign = [k[0].speed, k[1].speed].map(|k| if k > 0.0 { 1.0 } else { -1.0 });
        aw.speed_add = [sign[0] * (e * r.aw_speed_mult).clamp(0.0, 1.5), sign[1] * (e * r.aw_speed_mult).clamp(0.0, 1.7)];
        aw.ks_add = 2.2222 * rad * r.aw_stiffness_add * u - (1.0 - u) * AW_STIFFNESS;
        aw.ke_add = (u - 1.0) * AW_STIFFNESS;
        aw.phase_add = 30.0 * c.m.a.dot(c.up);
        aw.tick(c, &p, dt);
        self.da.aw = Some(aw);
    }
}
