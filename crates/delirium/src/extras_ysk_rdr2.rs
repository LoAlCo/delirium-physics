//! RDR2's yanked, swim and keyframe for the RDR2 core, from the clean-room study notes rdr2_yanked.md, rdr2_swim.md
//! and rdr2_keyframe.md (none of RDR2's code). yanked runs in place of the yanked and rdr2Lasso extras' own
//! behaviour while the core is on; keyframe in place of keyframe's and softKeyframe's; swim is a message of its own.
//! Without the core nothing here runs for yanked, rdr2Lasso, keyframe or softKeyframe.
//!
//! RDR2's masks are over its part order, bit = part (0 pelvis, 1/2 thighs, 3 spine0, 4/5 left shin and foot, 6/7
//! right, 8..10 spine1..3, 11/12 clavicles, 13 neck, 14..16 left arm, 17..19 right arm, 20 head); `rp` maps them to
//! ours. RDR2's stiffness scale (ctx+0x2c64) is 1 here.
use bevy_math::{Quat, Vec3};
use bevy_transform::components::Transform;

use crate::extras::{Extra, P};
use crate::msg::{Error, Kind, Message, Param};
use crate::nmdbm::LeanMode;
use crate::nmdbm_rdr2::Crouch;
use crate::nmfl::noise01;
use crate::nmrs::{ANKLE, CLAVICLE, ELBOW, HIP, KNEE, NECK, SHOULDER, SPINE, WRIST};
use crate::nmshot::{limb, Ctx};

pub static MESSAGES: [Message; 1] = [
    Message { name: "swim", params: &[
        Param { name: "swimOffset", kind: Kind::Vec3, used: true, doc: "the reach point's offset in the chest's frame (0 0 -0.2)" },
        Param { name: "IKtwist", kind: Kind::Float, used: true, doc: "the arm IK's twist (0)" },
        Param { name: "extendDist", kind: Kind::Float, used: true, doc: "a stroke's reach, m (0.63)" },
        Param { name: "extendSpeed", kind: Kind::Float, used: true, doc: "the hand's speed along it, m/s (4)" },
        Param { name: "extendCloseEnough", kind: Kind::Float, used: true, doc: "how near the reach line the hand must be, m (0.1)" },
        Param { name: "extendEnough", kind: Kind::Float, used: true, doc: "how short of full reach counts as reached, m (0.1)" },
        Param { name: "armPhaseTimeout", kind: Kind::Float, used: true, doc: "a stroke phase's longest, s (1)" },
        Param { name: "armStiffness", kind: Kind::Float, used: true, doc: "(12)" },
        Param { name: "armDamping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "legStiffness", kind: Kind::Float, used: true, doc: "(12)" },
        Param { name: "legDamping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "spineStiffness", kind: Kind::Float, used: true, doc: "spine, neck and head (12)" },
        Param { name: "spineDamping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "surfacePoint", kind: Kind::Vec3, used: true, doc: "ours, Delirium's water (RDR2 reads the core's water plane): a point on the surface (zero: no water, he only balances)" },
        Param { name: "surfaceNormal", kind: Kind::Vec3, used: true, doc: "ours: the water's up (0 0 1)" },
        Param { name: "swimDirection", kind: Kind::Vec3, used: true, doc: "ours: where his spine aims, world (RDR2's game may set it; its reset value 0 0 1; the arms' stroke always takes 0 0 1)" },
    ] },
];

pub fn build(name: &str, p: &mut P) -> Option<Result<Box<dyn Extra>, Error>> {
    match name {
        "swim" => Some(Swim::new(p).map(|e| Box::new(e) as _)),
        _ => None,
    }
}

// ---- shared ----

/// With RDR2_TRACE set in the environment, these ports say what they did (proof they ran; no effect on the physics).
pub(crate) fn trace() -> bool {
    static T: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    *T.get_or_init(|| std::env::var_os("RDR2_TRACE").is_some())
}

/// A trace line, to stderr.
pub(crate) fn note(s: std::fmt::Arguments) {
    if trace() {
        eprintln!("{s}");
    }
}

/// RDR2's part i, ours.
pub(crate) fn rp(i: usize) -> usize {
    match i {
        0 => 0,
        1 | 2 => HIP()[i - 1] + 1,
        3 => SPINE()[0] + 1,
        4 => KNEE()[0] + 1,
        5 => ANKLE()[0] + 1,
        6 => KNEE()[1] + 1,
        7 => ANKLE()[1] + 1,
        8..=10 => SPINE()[i - 7] + 1,
        11 | 12 => CLAVICLE()[i - 11] + 1,
        13 => NECK()[0] + 1,
        14 => SHOULDER()[0] + 1,
        15 => ELBOW()[0] + 1,
        16 => WRIST()[0] + 1,
        17 => SHOULDER()[1] + 1,
        18 => ELBOW()[1] + 1,
        19 => WRIST()[1] + 1,
        _ => NECK()[1] + 1,
    }
}

/// An RDR2 part mask, our parts.
pub(crate) fn parts(bits: u32) -> Vec<usize> {
    (0..21).filter(|i| bits >> i & 1 != 0).map(rp).collect()
}

/// An RDR2 part mask, our joints (each part's joint to its parent; the pelvis has none).
pub(crate) fn joints(bits: u32) -> Vec<usize> {
    parts(bits).into_iter().filter(|&k| k > 0).map(|k| k - 1).collect()
}

/// RDR2's spine limb parts from +0x3f0: pelvis, spine0..spine3, neck, head ([P] layout, as rdr2_swim.md reads +0x410
/// as the chest).
fn spine_part(k: usize) -> usize {
    [0, 3, 8, 9, 10, 13, 20].map(rp)[k]
}

fn v3s(v: Vec3) -> String {
    format!("{},{},{}", v.x, v.y, v.z)
}

fn pos(c: &Ctx, k: usize) -> Vec3 {
    c.b.part(k).m.d
}

fn flat(v: Vec3, up: Vec3) -> Vec3 {
    v - up * v.dot(up)
}

/// The character's generator, U in [0, 1).
fn u(c: &mut Ctx) -> f32 {
    c.ch.random()
}

fn ur(c: &mut Ctx, a: f32, b: f32) -> f32 {
    a + (b - a) * c.ch.random()
}

/// RDR2's weighted pick (0x2a4b474): index i with chance w_i / sum.
fn pick(c: &mut Ctx, w: &[f32]) -> usize {
    let sum: f32 = w.iter().map(|x| x.max(0.0)).sum();
    if sum <= 0.0 {
        return 0;
    }
    let mut r = u(c) * sum;
    for (i, x) in w.iter().enumerate() {
        r -= x.max(0.0);
        if r < 0.0 {
            return i;
        }
    }
    w.len() - 1
}

fn set_k(c: &mut Ctx, js: &[usize], s: f32, d: f32, muscle: Option<f32>) {
    for &j in js {
        if j < c.ch.eff.len() {
            c.ch.eff[j].set_stiffness(s, d, muscle);
        }
    }
}

fn arm_ik(c: &mut Ctx, i: usize, to: Vec3, twist: f32, vel: Vec3) {
    let l = limb(i);
    crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[3], to, twist, 0.0, 1.0 - 2.0 * i as f32, 1.0, vel, None);
}

/// The arm IK with RDR2's dragReduction (the target led by his motion against `vel`, 0x2a37370).
fn arm_ik_drag(c: &mut Ctx, i: usize, to: Vec3, twist: f32, drag: f32, vel: Vec3) {
    let l = limb(i);
    crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[3], to, twist, drag, 1.0 - 2.0 * i as f32, 1.0, vel, None);
}

fn arm_len(c: &Ctx, i: usize) -> f32 {
    let l = limb(i);
    let (s, e, w) = (c.b.joint_pos(l.e[1]), c.b.joint_pos(l.e[2]), c.b.joint_pos(l.e[3]));
    (e - s).length() + (w - e).length() + (pos(c, l.p[3]) - w).length()
}

/// RDR2's scale setter (0x2a50ff4): a per-joint scale of strength (as a stiffness scale, so squared), damping and
/// muscle stiffness for this step; the last write to a joint counts, an onlyLower write only where it lowers. The
/// scales are laid over what is in the effectors when the extra's tick ends (as meleeReact's port does: a joint whose
/// values nobody rewrote is scaled from its unscaled values, not compounded).
#[derive(Default)]
struct Scale {
    want: Vec<Option<[f32; 3]>>,
    done: Vec<Option<([f32; 3], [f32; 3])>>,
}

impl Scale {
    fn ask(&mut self, js: &[usize], s: [f32; 3], only_lower: bool) {
        for &j in js {
            if self.want.len() <= j {
                self.want.resize(j + 1, None);
            }
            self.want[j] = Some(match self.want[j] {
                Some(w) if only_lower => [0, 1, 2].map(|k| w[k].min(s[k])),
                _ => s,
            });
        }
    }

    fn apply(&mut self, c: &mut Ctx) {
        self.done.resize(c.ch.eff.len(), None);
        for (j, e) in c.ch.eff.iter_mut().enumerate() {
            let now = [e.strength, e.damping, e.muscle_stiffness];
            let base = match self.done[j] {
                Some((base, wrote)) if wrote == now => base,
                _ => now,
            };
            let Some(s) = self.want.get(j).copied().flatten() else {
                if self.done[j].is_some_and(|(_, w)| w == now) {
                    (e.strength, e.damping, e.muscle_stiffness) = (base[0], base[1], base[2]);
                }
                self.done[j] = None;
                continue;
            };
            let to = [base[0] * s[0] * s[0], base[1] * s[1], base[2] * s[2]];
            (e.strength, e.damping, e.muscle_stiffness) = (to[0], to[1], to[2]);
            self.done[j] = Some((base, to));
        }
        self.want.clear();
    }
}

/// Sub-behaviours started by message: started once, stopped once.
#[derive(Default)]
struct Kids {
    on: Vec<&'static str>,
    out: Vec<String>,
}

impl Kids {
    fn start(&mut self, name: &'static str, line: String) {
        if !self.on.contains(&name) {
            self.on.push(name);
            self.out.push(line);
        }
    }

    fn stop(&mut self, name: &'static str) {
        if let Some(i) = self.on.iter().position(|n| *n == name) {
            self.on.remove(i);
            self.out.push(format!("{name} start=false"));
        }
    }
}

/// headLook through messages: started with a target, the target sent again when it moves.
#[derive(Default)]
struct Look {
    at: Option<Vec3>,
}

impl Look {
    fn to(&mut self, out: &mut Vec<String>, at: Vec3, stiffness: f32) {
        match self.at {
            None => out.push(format!("headLook start=true alwaysLook=true pos={} stiffness={stiffness} damping=1", v3s(at))),
            Some(was) if was.distance(at) > 0.05 => out.push(format!("headLook pos={}", v3s(at))),
            _ => return,
        }
        self.at = Some(at);
    }

    fn stop(&mut self, out: &mut Vec<String>) {
        if self.at.take().is_some() {
            out.push("headLook start=false".into());
        }
    }
}

#[allow(non_snake_case)]
#[inline]
fn SUCCESS() -> u8 { delirium_data::kv!(55 int) }
#[allow(non_snake_case)]
#[inline]
fn FAILURE() -> u8 { delirium_data::kv!(54 int) }

// ---- yanked ----

/// RDR2's yanked parameters (slot 0x88 table; RDR2's defaults).
#[derive(Clone, Copy, Debug)]
pub struct YankedParams {
    arm_start: (f32, f32), spine_start: (f32, f32), time_start: f32, ramp_from: f32, arm: (f32, f32), spine: (f32, f32),
    steps_till: i32, per_step: f32, time_end: f32, ramp_to: f32, legs: (f32, f32), loose: [f32; 3],
    hip_pitch: (f32, f32), lean_hips: (f32, f32), lean: (f32, f32), restart_up: f32, bend: f32, turn: (f32, f32),
    head_pos: Vec3, use_head: bool, head_prob: Vec3, leg_amp: f32, grab_spine_freq: f32, spine_freq: f32, spine_amp: f32,
    side: (f32, f32), roll: [f32; 4], moving: f32, roll_type: i32, ground: [(f32, f32); 3], neck_k: f32,
    pub lassoed: i32, pub grab_part: usize, grab_pos: [Vec3; 2], claw_amp: Vec3, claw_freq: Vec3,
    hands_off: Vec3, feet_prob: Vec3, spine_prob: Vec3, state_time: (f32, f32), die: f32, fsim: (f32, f32), bump: (f32, f32),
    stirrup: bool, animal: bool, stable_neck: bool, stunned: (f32, f32), bolas: bool, bolas_forces: bool,
    bolas_time: f32, bolas_dir: Vec3, pub rope: Vec3, pub rope_part: usize,
}

/// Uncracked RDR2 parameters, kept at RDR2's defaults: the arm weights on his feet (+0x220) and down (+0x240), the
/// chance the head looks at a grab (+0x210), the spine amplitudes +0x124/+0x128/+0x12c/+0x130/+0x134, the limp noise
/// rate (+0x27c) and the struggle (+0x280).
const ARM_FEET: [f32; 2] = [0.2, 0.7];
const ARM_DOWN: [f32; 3] = [0.55, 0.0, 0.1];
const HEAD_GRAB: f32 = 0.333;
const SPINE_A: [f32; 5] = [0.4, 0.4, 1.0, 4.0, 0.4];
const LIMP_RATE: f32 = 0.0;
const STRUGGLE: f32 = 0.0;

impl YankedParams {
    /// RDR2's names and defaults. `point` and `part` (the room's stand-in rope of Delirium's own yanked) are the rope's
    /// anchor and roped part; rdr2Lasso's ropePos/ropePart likewise.
    pub fn new(p: &mut P) -> Result<YankedParams, Error> {
        let rope = if p.v("ropePos", Vec3::ZERO)? != Vec3::ZERO { p.v("ropePos", Vec3::ZERO)? } else { p.v("point", Vec3::ZERO)? };
        let part = p.i("part", 10)?;
        let rope_part = p.i("ropePart", part)?.clamp(0, 20) as usize;
        // taken, not done: the raised spine lean limits, the palms' normals (the wrists aren't turned) and the carts
        let _ = (p.f("spinelean1limit", 0.5)?, p.f("spinelean2limit", 0.3)?, p.f("torqueexaggeratejumplimit", 0.2)?);
        let _ = (p.v("grabnormall", Vec3::NEG_Z)?, p.v("grabnormalr", Vec3::NEG_Z)?, p.i("DraggedByCart", 0)?);
        // and the foot and ground friction (no per-part friction here)
        let _ = (p.f("footFriction", 0.3)?, p.f("groundFriction", 1.0)?, p.f("ropeendfriction", 0.5)?, p.f("otherendfriction", 2.0)?);
        Ok(YankedParams {
            arm_start: (p.f("armStiffnessStart", 3.0)?, p.f("armDampingStart", 0.1)?),
            spine_start: (p.f("spineStiffnessStart", 3.0)?, p.f("spineDampingStart", 0.1)?),
            time_start: p.f("timeAtStartValues", 0.4)?, ramp_from: p.f("rampTimeFromStartValues", 0.1)?,
            arm: (p.f("armStiffness", 13.0)?, p.f("armDamping", 0.5)?), spine: (p.f("spineStiffness", 12.0)?, p.f("spineDamping", 0.5)?),
            steps_till: p.i("stepsTillStartEnd", 0)?, per_step: p.f("perStepReduction", 1.0)?,
            time_end: p.f("timeStartEnd", 100.0)?, ramp_to: p.f("rampTimeToEndValues", 0.0)?,
            legs: (p.f("lowerBodyStiffness", 16.0)?, p.f("lowerBodyStiffnessEnd", 15.0)?),
            loose: [p.f("loosetimetorso", 0.3)?, p.f("loosetimelegs", 0.1)?, p.f("loosestrength", 0.1)?],
            hip_pitch: (p.f("hipPitchForward", 0.3)?, p.f("hipPitchBack", 0.5)?),
            lean_hips: (p.f("leanhipsmin", -0.4)?, p.f("leanhipsmax", 0.2)?), lean: (p.f("leanmin", -0.1)?, p.f("leanmax", -0.3)?),
            restart_up: p.f("restartUpness", 1.1)?, bend: p.f("spineBend", 1.0)?,
            turn: (p.f("turnThresholdMin", 0.5)?, p.f("turnThresholdMax", 1.5)?),
            head_pos: p.v("headLookPos", Vec3::ZERO)?, use_head: p.b("useHeadLook", true)?,
            head_prob: p.v("headlookprob", Vec3::splat(0.3))?, leg_amp: p.f("legAmplitude", 0.4)?,
            grab_spine_freq: p.f("grabspinefreq", 0.5)?, spine_freq: p.f("spinefreq", 2.5)?, spine_amp: p.f("spineAmplitude", 1.0)?,
            side: (p.f("spsidefreq", 2.0)?, p.f("spsideamp", 1.0)?),
            roll: [p.f("rollHelp", 0.0)?, p.f("rollhelpback", 0.0)?, p.f("rollhelpmoving", 0.0)?, p.f("rollhelpbackmoving", 0.0)?],
            moving: p.f("movingthresh", 1.0)?, roll_type: p.i("rollType", 0)?,
            ground: [(p.f("groundLegStiffness", 11.0)?, p.f("groundLegDamping", 0.5)?), (p.f("groundArmStiffness", 11.0)?, p.f("groundArmDamping", 0.5)?),
                     (p.f("groundSpineStiffness", 11.0)?, p.f("groundSpineDamping", 0.5)?)],
            neck_k: p.f("groundneckstiffness", 11.0)?,
            lassoed: p.i("lassoedtype", 1)?, grab_part: p.i("grabPart", 10)?.clamp(0, 20) as usize,
            grab_pos: [p.v("grabposl", Vec3::new(0.07, 0.05, -0.051))?, p.v("grabposr", Vec3::new(0.09, -0.05, -0.051))?],
            claw_amp: p.v("grabpointamp", Vec3::ZERO)?, claw_freq: p.v("grabpointfreq", Vec3::new(4.0, 5.0, 3.0))?,
            hands_off: p.v("grabhandsoffset", Vec3::ZERO)?, feet_prob: p.v("feetprob", Vec3::new(0.5, 0.5, 0.0))?,
            spine_prob: p.v("spineprob", Vec3::new(0.6, 0.4, 0.05))?,
            state_time: (p.f("minstatetime", 1.0)?, p.f("maxstatetime", 1.2)?), die: p.f("timetodie", 15.0)?,
            fsim: (p.f("frictionsim", 70.0)?, p.f("frictionsimairborne", 70.0)?), bump: (p.f("bumpsim", 400.0)?, p.f("bumpsimprob", 0.08)?),
            stirrup: p.b("draggedbystirrup", false)?, animal: p.b("draggedbyanimal", false)?,
            stable_neck: p.b("stableneck", false)?, stunned: (p.f("stunnedtime", 0.0)?, p.f("stunnedblendout", 0.0)?),
            bolas: p.b("bolas", false)?, bolas_forces: p.b("bolasForces", false)?, bolas_time: p.f("bolasTime", 0.35)?,
            bolas_dir: p.v("bolasDir", Vec3::X)?, rope, rope_part,
        })
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct Sub {
    s: u8,
    t: f32,
    first: bool,
}

/// RDR2's yanked (rdr2_yanked.md), the reaction to a rope: on his feet the balancer leans and turns him about the
/// rope, his upper body goes loose at the jerk and the legs soften step by step; once the balancer fails, the ground
/// mode's noise-driven writhe, kicks, clawing and rope grabs, the sliding friction and bumps, until timetodie.
pub struct YankedRdr2 {
    pub p: YankedParams,
    started: bool,
    t: f32,
    clock: f32,
    yank: f32,
    since_struggle: f32,
    touched_t: f32,
    ground_t: f32,
    land: f32,
    phase: f32,
    phase2: f32,
    seed: f32,
    ground: bool,
    seen: bool,
    back: bool,
    turn_side: bool,
    turn_thresh: f32,
    turn_timer: f32,
    turn_hold: f32,
    turn_go: bool,
    flip_p: f32,
    signs: [f32; 3],
    hip: f32,
    heading0: f32,
    unwrap: f32,
    last_psi: f32,
    steps0: u32,
    last_step: u32,
    gains: [f32; 4],
    arm: [Sub; 2],
    leg: [Sub; 2],
    spine: Sub,
    head: Sub,
    grab_latch: bool,
    holding: [bool; 2],
    jerks: i32,
    first_ground: bool,
    latched: bool,
    touched: bool,
    airborne: f32,
    bump_mode: u32,
    died: bool,
    bolas_t: f32,
    bolas_sign: f32,
    bolas_first: bool,
    pedal: [bool; 2],
    kids: Kids,
    look: Look,
    scale: Scale,
}

const WINDMILL: &str = "armsWindmillAdaptive start=true armDirection=2 angSpeed=4 phase=-3.1 disableOnImpact=false bendLeftElbow=true \
                        bendRightElbow=true lean1mult=1.01 lean1offset=0 swayamount=1 angSpeedMultiplier4Dragging=1.8 maxangspeed=4 \
                        scrabblebendupper=4 scrabblebendlower=1.57 scrabblebend=2.2 scrabbleextend=0 scrabble=true armStiffness=13 amplitude=1.8";
const WRITHE: &str = "bodyWrithe start=true blendArms=0.7 blendBack=0 blendLegs=0 applyStiffness=false onFire=true elbowAmplitude=0.5 \
                      armAmplitude=2 armPeriod=1.5 rollTorqueScale=0 rollOverFlag=false";

impl YankedRdr2 {
    pub fn new(p: YankedParams) -> YankedRdr2 {
        YankedRdr2 {
            p, started: false, t: 0.0, clock: 0.0, yank: 100000.0, since_struggle: 10.0, touched_t: 0.0, ground_t: 0.0, land: -0.2,
            phase: 0.0, phase2: 0.0, seed: 0.0, ground: false, seen: false, back: false, turn_side: false, turn_thresh: 0.0,
            turn_timer: 0.0, turn_hold: 0.0, turn_go: false, flip_p: 0.0, signs: [1.0; 3], hip: 0.0, heading0: 0.0, unwrap: 0.0,
            last_psi: 0.0, steps0: 0, last_step: 0, gains: [0.0; 4], arm: [Sub::default(); 2], leg: [Sub::default(); 2],
            spine: Sub::default(), head: Sub::default(), grab_latch: false, holding: [false; 2], jerks: 0, first_ground: true,
            latched: false, touched: false, airborne: 0.0, bump_mode: 1, died: false, bolas_t: 0.0, bolas_sign: 1.0,
            bolas_first: true, pedal: [false; 2], kids: Kids::default(), look: Look::default(), scale: Scale::default(),
        }
    }

    pub fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.kids.out)
    }

    /// The rope's anchor, for a message without start (rdr2Lasso's ropePos as the rope moves).
    pub fn rope_to(&mut self, at: Vec3) {
        self.p.rope = at;
    }

    /// The rope direction's angle about up from the pelvis's r2 axis (0x2a7eb0c, our rope: anchor less spine2's
    /// position), unwrapped across +-pi outside the first call.
    fn heading(&mut self, c: &Ctx, init: bool) -> f32 {
        let up = c.up;
        let d = flat(self.p.rope - pos(c, spine_part(3)), up).normalize_or_zero();
        let r2 = flat(c.b.part(0).m.c, up).normalize_or_zero();
        let a = r2.cross(d).dot(up).atan2(r2.dot(d));
        if !init {
            let jump = a - self.last_psi;
            if jump > 0.7 * std::f32::consts::PI {
                self.unwrap -= std::f32::consts::TAU;
            } else if jump < -0.7 * std::f32::consts::PI {
                self.unwrap += std::f32::consts::TAU;
            }
        }
        self.last_psi = a;
        a
    }

    fn activate(&mut self, c: &mut Ctx) {
        self.started = true;
        if self.p.rope == Vec3::ZERO {
            // ours: no rope given, 2 m in front of the chest (the room's stand-in yanked's default)
            let f = flat(-c.m.c, c.up).normalize_or(Vec3::X);
            self.p.rope = pos(c, spine_part(4)) + f * 2.0;
        }
        (self.bolas_t, self.bolas_sign, self.bolas_first) = (0.1, 1.0, true);
        (self.yank, self.since_struggle, self.clock) = (100000.0, 10.0, 0.0);
        self.phase = 4096.0 * u(c);
        self.phase2 = 4096.0 * u(c);
        self.seed = c.ch.agent as f32;
        self.ground = false;
        for j in joints(0x1fff09) {
            c.ch.eff[j].desired = [0.0; 3];
        }
        if self.p.use_head {
            let at = if self.p.head_pos == Vec3::ZERO { self.p.rope } else { self.p.head_pos };
            self.look.to(&mut self.kids.out, at, 11.0);
        }
        for &j in SPINE().iter().chain(NECK().iter()) {
            c.ch.eff[j].oppose_gravity = 0.0;
        }
        if c.dbm.is_none() {
            self.kids.start("bodyBalance", "bodyBalance start=true".into());
        }
        self.turn_side = c.ch.random_bits() & 1 != 0;
        self.turn_thresh = ur(c, self.p.turn.0, self.p.turn.1);
        self.steps0 = c.dbm.map_or(0, |d| d.steps);
        self.gains = [self.p.arm_start.0, self.p.arm_start.1, self.p.spine_start.0, self.p.spine_start.1];
        (self.hip, self.signs, self.flip_p) = (0.0, [1.0; 3], 0.0);
        self.turn_go = c.ch.random_bits() & 1 != 0;
        self.kids.start("bodyWrithe", WRITHE.into());
        self.arm = [Sub { s: 0, t: 0.0, first: true }; 2];
        self.leg = [Sub::default(); 2];
        (self.spine, self.head) = (Sub::default(), Sub::default());
        self.bump_mode = 1;
        self.heading0 = self.heading(c, true);
    }

    pub fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if !self.started {
            self.activate(c);
        }
        self.t += dt;
        self.since_struggle = if STRUGGLE.min(1.0) >= 0.1 { -1e-6 } else { (self.since_struggle + dt).min(10.0) };
        if self.p.bolas {
            self.bolas(c, dt);
        }
        let mut ty = if (0..=2).contains(&self.p.lassoed) { self.p.lassoed } else { 1 };
        if self.p.bolas && ty == 0 {
            ty = 1;
        }
        // (type 0's balancer neck flags, bal+0x464 and ctx+0x2ed8, have no reader here)
        self.seen |= c.upright;
        let failed = c.dbm_failed || !c.upright && (self.seen || self.t > 0.5);
        if !self.ground && failed {
            self.ground = true;
            for s in self.arm.iter_mut().chain(self.leg.iter_mut()) {
                s.t = 0.0;
            }
            (self.spine.t, self.head.t) = (0.0, 0.0);
        }
        for s in self.arm.iter_mut().chain(self.leg.iter_mut()) {
            s.t -= dt;
        }
        self.spine.t -= dt;
        self.head.t -= dt;
        if !self.ground {
            self.feet(c, ty, dt);
        } else {
            self.down(c, ty, dt);
            let (st, bo) = self.p.stunned;
            if self.touched_t < st + bo {
                let x = (self.touched_t - st).clamp(0.0, bo);
                self.scale.ask(&joints(0x16ff5f), [0.5 + 0.25 * x, 1.0, 1.0], true);
            }
            if STRUGGLE < -0.01 {
                let s = ((-STRUGGLE).min(1.0) + 1.0) / 2.0;
                self.scale.ask(&joints(0x16ff5f), [s, 1.0, 1.0], true);
            }
            if c.m.b.dot(c.up) > self.p.restart_up {
                (self.back, self.ground, self.touched_t, self.touched) = (true, false, 0.0, false);
                self.out_balance();
                for s in self.arm.iter_mut().chain(self.leg.iter_mut()) {
                    s.t = 0.0;
                }
            }
        }
        self.scale.apply(c);
        if self.yank < 100000.0 {
            self.yank += dt;
        }
        self.clock += dt;
    }

    fn out_balance(&mut self) {
        self.kids.on.retain(|n| *n != "bodyBalance");
        self.kids.start("bodyBalance", "bodyBalance start=true".into());
    }

    /// The bolas's spinning couple (section 7): alternating impulses about up x bolasDir while the clock is under
    /// bolasTime.
    fn bolas(&mut self, c: &mut Ctx, dt: f32) {
        if !self.p.bolas_forces || self.clock >= self.p.bolas_time {
            return;
        }
        self.bolas_t -= dt;
        if self.bolas_t > 0.0 {
            return;
        }
        self.bolas_t += 0.065;
        let speed = c.vel.length();
        let f = match self.p.lassoed {
            0 => 2500.0,
            2 => 1000.0,
            _ => 3000.0 + (5000.0 - 3000.0) * (speed.clamp(1.0, 2.0) - 1.0),
        };
        let mut s = self.bolas_sign;
        self.bolas_sign = -self.bolas_sign;
        if self.bolas_first {
            s *= 0.5;
            self.bolas_first = false;
        }
        let j = c.up.cross(self.p.bolas_dir.normalize_or(Vec3::X)).normalize_or_zero() * (f * s / 30.0);
        // impulses (0x2a1a9f8 takes them pre-multiplied by dt), as forces over this step
        let mut give = |k: usize, x: f32| c.out.force(rp(k), j * x / dt);
        match self.p.lassoed {
            0 => { give(0, 1.0); give(4, -0.5); give(6, -0.5); }
            2 => { give(20, 1.0); give(10, -1.0); }
            _ => { give(0, 1.0); give(1, -0.25); give(2, -0.25); give(10, -0.5); give(20, 0.05); }
        }
    }

    /// The on-feet tick (section 5).
    fn feet(&mut self, c: &mut Ctx, ty: i32, dt: f32) {
        let up = c.up;
        if self.back {
            self.kids.stop("armsWindmillAdaptive");
            self.pedal_set(c, [false; 2]);
            self.back = false;
        }
        // the yank: moving over 0.5 m/s along the ground starts its clock; moving along c the feet kick out
        let h = flat(c.vel, up);
        if h.length() > 0.5 && self.yank >= 100000.0 {
            self.yank = 0.0;
        }
        let stepping = c.dbm.map_or(0, |d| d.stepping);
        if -0.0667 < self.yank && self.yank < 0.3 && h.length() > 0.5 && h.normalize().dot(c.m.c) < -0.4 {
            if stepping < 2 {
                c.ch.eff[ANKLE()[1]].desired[0] = -2.9;
            }
            if stepping == 0 || stepping == 2 {
                c.ch.eff[ANKLE()[0]].desired[0] = -2.9;
            }
        }
        // the arms: no action (5) or a grab at the rope (2)
        let mut grabbing = false;
        for i in 0..2 {
            if self.arm[i].t <= 0.0 {
                let s = if ty == 0 { 5 } else { [5, 2][pick(c, &ARM_FEET)] };
                let first = self.arm[i].first;
                let (lo, hi) = self.p.state_time;
                let t = if first { ur(c, 0.1, lo.max(0.1)) } else { ur(c, lo, hi) };
                self.arm[i] = Sub { s, t, first: false };
                grabbing |= s == 2;
            }
            if (2..=4).contains(&self.arm[i].s) && c.ch.busy[i] {
                self.arm[i].s = 0;
            }
        }
        if grabbing {
            if !self.grab_latch {
                self.head.t = 0.0;
            }
            self.grab_latch = true;
        }
        // the hip pitch from the turn's noise (owner 4, at once)
        c.out.dbm_crouch.push(Crouch::Hip(4, -self.hip.clamp(-2.0, 2.0), 100000.0));
        // (the foot friction, footFriction and +0xcc, has no per-part friction to set here)
        // the leans: the hips by the rope (slot 4) and the body (slot 6)
        let d = flat(self.p.rope - c.com, up).normalize_or_zero();
        if d != Vec3::ZERO {
            let (lo, hi) = self.p.lean_hips;
            c.out.hip_lean = Some(d * (lo + noise01(self.clock, self.phase, self.seed) * (hi - lo)));
            let (lo, hi) = self.p.lean;
            let a = if h.dot(d) < -0.4 { lo } else { lo + noise01(2.0 * self.clock, self.phase, self.seed + 0.5) * (hi - lo) };
            c.out.lean = Some((LeanMode::Dir(d), a, false));
        }
        // the legs soften with time and with each step (not while staggerFall runs)
        if !c.extras_on.contains(&"staggerFall") {
            let (a, b) = self.p.legs;
            let x = if self.p.ramp_to > 0.0 { ((self.clock - self.p.time_end) / self.p.ramp_to).clamp(0.0, 1.0) } else if self.clock >= self.p.time_end { 1.0 } else { 0.0 };
            let base = a + (b - a) * x;
            let n = c.dbm.map_or(0, |d| d.steps) as i32 - self.steps0 as i32 - self.p.steps_till;
            for i in 0..2 {
                let red = self.p.per_step * if stepping == i as u32 + 1 { n } else { n - 1 } as f32;
                let k = (base - red).max(5.0);
                for j in [HIP()[i], KNEE()[i], ANKLE()[i]] {
                    c.ch.eff[j].strength = k * k;
                }
            }
            // the arm and spine gains: the start values, then (ours [P]) a ramp to the end values
            let x = if self.p.ramp_from > 0.0 { ((self.clock - self.p.time_start) / self.p.ramp_from).clamp(0.0, 1.0) }
                    else if self.clock >= self.p.time_start { 1.0 } else { 0.0 };
            let s = [self.p.arm_start.0, self.p.arm_start.1, self.p.spine_start.0, self.p.spine_start.1];
            let e = [self.p.arm.0, self.p.arm.1, self.p.spine.0, self.p.spine.1];
            self.gains = std::array::from_fn(|k| s[k] + (e[k] - s[k]) * x);
            set_k(c, &joints(0x709), self.gains[2], self.gains[3], None);
            for i in 0..2 {
                if self.arm[i].s != 2 {
                    set_k(c, &joints([0x1c800, 0xe1000][i]), self.gains[0], self.gains[1], None);
                }
            }
        }
        // loose at the jerk
        let ls = self.p.loose[2];
        if self.yank <= self.p.loose[0] {
            let m = if self.p.stable_neck { 0xfdf08 } else { 0x1fff09 };
            self.scale.ask(&joints(m), [1.0, ls, ls], true);
        }
        if self.yank <= self.p.loose[1] {
            self.scale.ask(&joints(6), [1.0, ls, ls], true);
        }
        // (torqueexaggeratejumplimit's raised spine lean limits: Delirium's limits aren't per-step settable)
        self.turn(c, stepping, dt);
        self.grab(c);
        self.head_tick(c, false);
    }

    /// The turn decision, hip pitch and spine (section 5.1).
    fn turn(&mut self, c: &mut Ctx, stepping: u32, dt: f32) {
        let raw = self.heading(c, false);
        let psi = raw + self.unwrap + self.heading0;
        self.turn_timer -= dt;
        // ([G] the byte at RVA 0x3cc7e9c read as 0: a run-out timer re-arms only between steps)
        if self.turn_timer <= 0.0 && stepping == 0 || stepping != self.last_step {
            self.turn_timer = ur(c, 0.6, 2.0);
            if self.turn_hold < 0.0 || self.turn_side && psi > self.turn_thresh || !self.turn_side && psi < -self.turn_thresh {
                self.turn_side = !self.turn_side;
                self.turn_thresh = ur(c, self.p.turn.0, self.p.turn.1);
                self.turn_hold = 10000.0;
            }
            self.turn_go = u(c) < 0.65;
            for k in 0..3 {
                if u(c) < 1.0 - self.flip_p {
                    self.signs[k] = -self.signs[k];
                    self.flip_p += 0.05;
                }
            }
        }
        self.last_step = stepping;
        let (lo, hi) = self.p.turn;
        if self.turn_thresh > 0.01 && lo > 0.0 && hi > 0.0 && (self.turn_go || psi.abs() <= hi) {
            // [P] the turn request's heading: the pelvis's side axis on the turn's side
            let side = flat(c.b.part(0).m.b, c.up).normalize_or_zero() * if self.turn_side { 1.0 } else { -1.0 };
            c.out.dbm_turn = Some(side * 0.5);
        } else {
            c.out.dbm_turn = Some(Vec3::ZERO);
        }
        let n = (2.0 * noise01(2.0 * self.clock, self.phase, self.seed) - 1.0) * self.signs[0];
        self.hip = n * if n < 0.0 { self.p.hip_pitch.1 } else { self.p.hip_pitch.0 };
        let b = self.p.bend;
        let n1 = noise01(2.0 * self.clock, self.phase, self.seed - 0.5);
        let n2 = noise01(self.phase2, self.seed + 0.5, 2.0 * self.clock);
        let l1 = (-2.0 * b + 4.0 * b * n1) * self.signs[1];
        let l2 = (-2.0 * b + 4.0 * b * n2) * self.signs[2];
        spine_writer(c, l1, l2, l2);
    }

    /// The rope grab (section 6.4, [P]): each grabbing hand to its grab point in grabPart's frame, clawing about it.
    fn grab(&mut self, c: &mut Ctx) {
        let both = self.arm[0].s == 2 && self.arm[1].s == 2;
        let m = c.b.part(rp(self.p.grab_part)).m;
        for i in 0..2 {
            if self.arm[i].s != 2 {
                self.holding[i] = false;
                continue;
            }
            let f = self.p.claw_freq;
            let n = Vec3::new(noise01(f.x * self.clock, self.phase, self.seed + i as f32), noise01(f.y * self.clock, self.phase2, self.seed),
                              noise01(f.z * self.clock, self.seed, self.phase)) * 2.0 - Vec3::ONE;
            let mut g = self.p.grab_pos[i] + self.p.claw_amp * n;
            if both {
                g += self.p.hands_off * if i == 0 { 1.0 } else { -1.0 };
            }
            let to = m.d + m.a * g.x + m.b * g.y + m.c * g.z;
            arm_ik(c, i, to, 0.0, Vec3::ZERO);
            let (k, d) = self.p.ground[1];
            set_k(c, &joints([0x1c800, 0xe1000][i]), k, d, None);
            // ours: holding once the hand is there
            self.holding[i] = pos(c, limb(i).p[3]).distance(to) < 0.1;
        }
    }

    /// The head driver (section 5 step 10, 6.3's 0x2a78448).
    fn head_tick(&mut self, c: &mut Ctx, ground: bool) {
        let grab = self.arm.iter().any(|a| a.s == 2);
        if self.head.t <= 0.0 {
            let w = self.p.head_prob;
            self.head.s = [1, 4, 3][pick(c, &[w.x, w.y, w.z])];
            if grab && u(c) < HEAD_GRAB {
                self.head.s = 2;
            }
            self.head.t = ur(c, self.p.state_time.0, self.p.state_time.1);
        }
        if !ground && self.head.s == 4 && c.dbm.is_some_and(|d| d.stepping == 0) {
            self.head.s = 1;
        }
        if !self.p.use_head && self.head.s == 1 {
            self.head.s = 0;
        }
        if ground && self.p.animal {
            self.head.s = 6;
        }
        // [G] stableneck read as the stable-neck state
        if self.p.stable_neck {
            self.head.s = 5;
        }
        let neck: Vec<usize> = joints(0x102000);
        let (ks, kd) = if ground { (1.0 + self.p.spine.0, self.p.spine.1) } else { (11.0, 1.0) };
        let at = match self.head.s {
            6 => Some(self.p.rope),
            2 => {
                let m = c.b.part(rp(self.p.grab_part)).m;
                let g = self.p.grab_pos[1];
                Some(m.d + m.a * g.x + m.b * g.y + m.c * g.z)
            }
            1 => Some(if self.p.head_pos == Vec3::ZERO { self.p.rope } else { self.p.head_pos }),
            4 => Some(pos(c, spine_part(6)) + c.vel.normalize_or(-c.m.c)),
            _ => None,
        };
        match (self.head.s, at) {
            (_, Some(at)) => {
                set_k(c, &neck, ks, kd, None);
                self.look.to(&mut self.kids.out, at, ks);
            }
            (5, None) => {
                set_k(c, &neck, self.p.neck_k, 1.0, Some(1.0));
                for &j in &neck {
                    c.ch.eff[j].desired[2] = 0.0;
                }
            }
            _ => {
                set_k(c, &neck, 1.0 + self.p.spine.0, self.p.spine.1, None);
                for &j in &neck {
                    c.ch.eff[j].desired = [0.0; 3];
                }
                self.look.stop(&mut self.kids.out);
            }
        }
    }

    fn pedal_set(&mut self, c: &mut Ctx, legs: [bool; 2]) {
        if legs != self.pedal {
            self.pedal = legs;
            if legs == [false; 2] {
                self.kids.out.push("pedalLegs start=false".into());
            } else {
                self.kids.out.push(format!("pedalLegs start=true adaptivePedal4Dragging=true angSpeedMultiplier4Dragging=1 spread=0.3 angularSpeed=9 \
                                            speedAsymmetry=1 centreForwards=0.1 backPedal=false pedalLeftLeg={} pedalRightLeg={}", legs[0], legs[1]));
            }
        }
        if legs != [false; 2] {
            // pedalLegs' one-tick reverse flags, back-pedalling legs (state 3)
            c.out.pedal_reverse = Some([0, 1].map(|i| self.leg[i].s == 3));
        }
    }

    /// The ground tick (section 6, [P] overall).
    fn down(&mut self, c: &mut Ctx, ty: i32, dt: f32) {
        let up = c.up;
        let neck = ty == 0;
        let speed = c.vel.length();
        let all: Vec<usize> = (0..21).map(rp).collect();
        // the rope's jerk
        let d = (pos(c, spine_part(4)) - self.p.rope).normalize_or_zero();
        let vr = d.dot(c.vel);
        if !self.first_ground && vr > 5.0 {
            self.jerks = 2;
        }
        let jerk = self.jerks > 0 && vr > 0.0;
        self.jerks -= 1;
        self.first_ground = false;
        // landing: softened, ramping back up
        let r = (self.land.clamp(0.0, 0.6) / 0.6 + 0.1).min(1.0);
        if !jerk && r < 1.0 {
            self.scale.ask(&joints(0x6df5f), [r.max(0.7), r, r], true);
            self.scale.ask(&joints(0xfd800), [r.max(0.7); 3], true);
        }
        let legs_touch = c.b.doll.collided(&parts(0xf7));
        if legs_touch {
            (self.latched, self.touched) = (true, true);
        }
        if self.latched {
            self.land += dt;
            let r2 = (self.land.clamp(0.0, 0.1) * 10.0 + 0.1).min(1.0);
            self.scale.ask(&joints(0x102000), [r2; 3], true);
        }
        if !legs_touch && c.vel.dot(up) < 0.0 {
            self.airborne += dt;
        } else if legs_touch {
            self.airborne = 0.0;
        }
        if self.airborne > 0.3 {
            self.latched = false;
        }
        let spine_touch = c.b.doll.collided(&parts(0x102709));
        if self.p.animal {
            self.scale.ask(&joints(0x102000), [1.0; 3], false);
        }
        if speed < 1.0 && !spine_touch {
            if r > 0.2 {
                self.scale.ask(&joints(0x10275f), [0.2; 3], true);
            }
            self.scale.ask(&joints(0x102000), [0.2; 3], true);
        }
        if STRUGGLE > 0.4 {
            self.scale.ask(&all.iter().filter(|&&k| k > 0).map(|&k| k - 1).collect::<Vec<_>>(), [1.0; 3], false);
        }
        // (ours: no contact normals here, the ground's normal is up)
        let gn = up;
        // frictionsim: a constant sliding drag on the roped end
        let upper = if self.p.stirrup && c.extras_on.contains(&"rdr2Sit") { 0x1fff08 } else { 0x1fff09 };
        let k = if c.b.doll.collided(&parts(upper)) { self.p.fsim.0 } else if self.airborne > 0.5 { self.p.fsim.1 } else { 0.0 };
        if k > 0.0 {
            let ps: Vec<usize> = if neck {
                vec![limb(0).p[3], limb(1).p[3], spine_part(4), spine_part(3), spine_part(2), spine_part(1)]
            } else {
                [2usize, 3].iter().flat_map(|&i| [limb(i).p[1], limb(i).p[2], limb(i).p[3]]).collect()
            };
            for kk in ps {
                let v = flat(c.b.part(kk).v, up);
                if v.length() > 1e-4 {
                    c.out.force(kk, -v.normalize() * k);
                }
            }
        }
        // bumpsim
        let p = if dt > 1.0 / 60.0 { self.p.bump.1 * dt.min(1.0 / 15.0) * 60.0 } else { self.p.bump.1 };
        if self.p.bump.0 > 0.0 && speed > 1.0 && u(c) < p {
            self.bump_mode = if neck { (self.bump_mode + 2) % 3 } else { (self.bump_mode + 1) % 3 };
            let set = [[13, 20, 11, 12], [3, 8, 9, 10], [1, 4, 2, 6]][self.bump_mode as usize];
            let k = rp(set[(u(c) * 3.999) as usize]);
            let mean = c.b.doll.mass() / 21.0;
            let mut at = gn * (self.p.bump.0 * c.b.doll.part_mass(k) / (60.0 * mean));
            if self.bump_mode == 1 {
                at *= 4.0;
            }
            c.out.force(k, at / dt);
        }
        // face flags, limp, the "yanked" feedback
        let face_up = gn.dot(c.m.c) > 0.0;
        let side = -gn.dot(c.m.a);
        let limp = self.ground_t >= self.p.die
            || LIMP_RATE > 0.0 && noise01(self.clock * LIMP_RATE, self.phase, self.seed - 0.5) > 0.7
            || 0.0 < self.since_struggle && self.since_struggle < 0.3
            || self.p.bolas && self.touched_t < 0.5;
        if self.ground_t >= self.p.die && !self.died {
            self.died = true;
            c.out.send(SUCCESS(), "yanked", 0.0);
        }
        if limp {
            self.kids.stop("bodyWrithe");
        } else {
            self.kids.start("bodyWrithe", WRITHE.into());
        }
        // the arms: reach/claw (0), windmill (1), the rope (2)
        let moving = speed > 1.0;
        let mut grabbed = false;
        for i in 0..2 {
            if self.arm[i].t <= 0.0 {
                let s = if ty == 0 {
                    let w = self.p.feet_prob;
                    pick(c, &[w.x, w.y]) as u8
                } else {
                    pick(c, &ARM_DOWN) as u8
                };
                self.arm[i] = Sub { s, t: ur(c, self.p.state_time.0, self.p.state_time.1), first: false };
                grabbed |= s == 2;
            }
            if self.arm[i].s == 1 && !(self.airborne > 0.0 || face_up && moving) {
                self.arm[i].s = 0;
            }
            if limp {
                self.arm[i].s = 0;
            }
            if STRUGGLE > 0.4 && ty != 0 {
                self.arm[i].s = 2;
            }
            if c.ch.busy[i] {
                self.arm[i].s = 0;
            }
        }
        if grabbed {
            self.head.t = 0.0;
        }
        if self.arm.iter().any(|a| a.s == 1) {
            self.kids.start("armsWindmillAdaptive", WINDMILL.into());
        } else {
            self.kids.stop("armsWindmillAdaptive");
        }
        for i in 0..2 {
            if self.arm[i].s == 0 {
                self.claw(c, i, neck, face_up, limp, speed);
            }
        }
        self.grab(c);
        // the legs: kicks (0), pedalling (2), back-pedalling (3)
        for i in 0..2 {
            if self.leg[i].t <= 0.0 {
                let r = u(c) * if self.p.stirrup { 0.5 } else { 1.0 };
                let s = if ty == 0 { 0 } else if r < 0.25 { 3 } else if r < 0.5 { 2 } else { 0 };
                self.leg[i] = Sub { s, t: ur(c, 0.3, 1.0), first: false };
            }
        }
        self.pedal_set(c, [0, 1].map(|i| self.leg[i].s >= 2));
        for i in 0..2 {
            if self.leg[i].s != 0 {
                continue;
            }
            let a = if limp { 0.3 } else if face_up { 0.4 } else { self.p.leg_amp };
            let g = 2.2 * speed.clamp(0.03, 1.0);
            let h = a * noise01(g * self.clock, self.phase + 7.0 * i as f32, self.seed - 0.5);
            let (hp, kn, an) = (c.b.joint_pos(HIP()[i]), c.b.joint_pos(KNEE()[i]), c.b.joint_pos(ANKLE()[i]));
            let (la, lb) = ((kn - hp).length(), (an - kn).length().max(0.01));
            let knee = -h - (h.sin() * la / lb).clamp(-1.0, 1.0).asin();
            let e = &mut c.ch.eff[HIP()[i]].desired;
            (e[0], e[1]) = (h, -0.05);
            c.ch.eff[KNEE()[i]].desired[0] = knee;
            // (the carts' and side cases' spread legs: no carts here)
            let _ = side;
        }
        // the spine
        if self.spine.t <= 0.0 {
            let w = self.p.spine_prob;
            self.spine.s = [0, 2, 3][pick(c, &[w.x, w.y, w.z])];
            if self.arm.iter().any(|a| a.s == 2) && u(c) < 0.3 {
                self.spine.s = 3;
            }
            self.spine.t = ur(c, self.p.state_time.0, self.p.state_time.1);
        }
        if self.arm.iter().any(|a| a.s == 1) {
            self.spine.s = 1;
        }
        if limp {
            self.spine.s = 0;
        }
        if STRUGGLE > 0.4 {
            self.spine.s = 3;
        }
        if self.spine.s >= 2 {
            let grab = self.arm.iter().any(|a| a.s == 2);
            let s = speed.clamp(if grab { self.p.grab_spine_freq } else { STRUGGLE.max(0.05) }, 1.0);
            let mut a = if self.spine.s == 2 { self.p.spine_amp } else { SPINE_A[3] };
            if face_up {
                a = -a;
            }
            let l1 = a * noise01(self.phase, self.p.spine_freq * s * self.clock, self.seed - 0.9);
            let sd = self.p.side.1 * (2.0 * noise01(self.p.side.0 * s * self.clock, self.phase2, self.seed) - 1.0);
            let k = if grab { 1.5 } else { 1.0 };
            spine_writer(c, k * l1, k * sd, k * sd);
        }
        let (sk, sd) = self.p.ground[2];
        set_k(c, &joints(0x709), sk, sd, None);
        let (lk, ld) = self.p.ground[0];
        let lk = if limp { if neck { 8.0 } else { 4.0 } } else { lk };
        set_k(c, &joints(0xf6), lk, ld, None);
        // roll help ([P]: a torque about his long axis on the spine parts)
        if !limp {
            let back = up.dot(c.m.c) > 0.0;
            let mv = speed > self.p.moving;
            let k = match (back, mv) { (false, false) => self.p.roll[0], (true, false) => self.p.roll[1], (false, true) => self.p.roll[2], (true, true) => self.p.roll[3] };
            if k > 0.0 && !(self.p.roll_type == 1 && back) {
                let s = if side >= 0.0 { 1.0 } else { -1.0 };
                for kk in 1..5 {
                    c.out.torque(spine_part(kk), c.m.b * (k * s * c.b.doll.mass() * 0.25));
                }
            }
        }
        self.head_tick(c, true);
        if self.touched {
            self.touched_t += dt;
        }
        self.ground_t += dt;
    }

    /// The reach/claw (state 0): the hand to a noise-driven point about the shoulder, from the table for the case.
    fn claw(&mut self, c: &mut Ctx, i: usize, neck: bool, face_up: bool, limp: bool, speed: f32) {
        let a: [f32; 8] = match (neck, face_up, limp) {
            (true, false, true) => [0.7, 1.0, 1.0, 2.5, 0.9, -0.5, 0.0, 6.0],
            (true, true, true) => [0.6, 0.9, 1.9, 2.5, 1.2, -0.5, 0.3, 7.5],
            (true, true, false) => [0.5, 1.0, 1.0, 2.5, 2.5, -0.5, -1.0, 13.0],
            (false, true, _) => [0.5, 0.9, 1.8, 2.5, 3.0, -0.5, 0.0, 11.0],
            // ([G] the face-down twist isn't recovered: the default's 0.5)
            (false, false, _) => [0.65, 1.0, -2.0, -0.7, 2.5, 0.5, 0.0, 11.0],
            _ => [0.85, 0.85, -2.0, -2.0, 0.7, 0.5, 0.0, 7.0],
        };
        let len = arm_len(c, i);
        let m = c.b.part(spine_part(3)).m;
        let (ax, bx) = (m.a, -m.b);
        let f = a[4] * speed.clamp(0.1, 1.0);
        let o = i as f32 * 3.3;
        let r = len * (a[0] + (a[1] - a[0]) * noise01(f * self.clock, self.phase + o, self.seed - 0.5));
        let th = a[2] + (a[3] - a[2]) * noise01(self.phase2 + o, self.seed + 0.5, f * self.clock);
        let s = if i == 0 { 1.0 } else { -1.0 };
        let to = pos(c, limb(i).p[0]) + (ax * th.cos() - bx * (s * th.sin())) * r;
        arm_ik(c, i, to, a[5], Vec3::ZERO);
        set_k(c, &joints([0x1c800, 0xe1000][i]), a[7], 1.0, None);
    }

    pub fn stop(&mut self, c: &mut Ctx) {
        if !self.started {
            return;
        }
        c.out.dbm_turn = Some(Vec3::ZERO);
        c.out.dbm_crouch.push(Crouch::ReleaseHip(f32::INFINITY));
        self.look.stop(&mut self.kids.out);
        // ours: what it started stops with it
        for n in ["armsWindmillAdaptive", "bodyWrithe"] {
            self.kids.stop(n);
        }
        if self.pedal != [false; 2] {
            self.kids.out.push("pedalLegs start=false".into());
        }
    }
}

/// RDR2's spine writer: lean1, lean2 and twist spread over the four spine joints, a quarter each.
fn spine_writer(c: &mut Ctx, l1: f32, l2: f32, tw: f32) {
    for &j in SPINE().iter() {
        c.ch.eff[j].desired = [l1 * 0.25, l2 * 0.25, tw * 0.25];
    }
}

// ---- swim ----

#[derive(Clone, Copy, Debug, Default)]
struct Stroke {
    phase: u8,
    timer: f32,
    done: bool,
    reach: Vec3,
    timeout: f32,
    dist: f32,
    speed: f32,
    offset: Vec3,
    dir: Vec3,
}

/// The nearest point on an ellipse of half axes (a, b) to (x, y), outside it (0x2a248a0): Newton on the half-angle
/// tangent, at most 100 steps.
fn ellipse_nearest(x: f32, y: f32, a: f32, b: f32) -> (f32, f32) {
    let (sx, sy) = (if x < 0.0 { -1.0 } else { 1.0 }, if y < 0.0 { -1.0 } else { 1.0 });
    let (px, py) = (x.abs(), y.abs());
    let (ia, ka, kb) = (1.0 / a, a / b, b / a);
    let (cx, cy) = (px.min(a), py.min(b));
    let e = (cx * cx / (a * a) + cy * cy / (b * b)).sqrt();
    let (mut qx, mut qy) = (cx / e, cy / e);
    let mut t = ((1.0 - ia * qx) / (ia * qx + 1.0)).max(0.0).sqrt();
    for _ in 0..100 {
        let f = 2.0 * ((px - qx) * ka * qy - (py - qy) * kb * qx);
        let d = f / ((2.0 * qy * qy * ka * ka + 2.0 * qx * qx * kb * kb + 2.0 * qx * (px - qx) + 2.0 * qy * (py - qy)) * (ia * qx + 1.0) - qy / b * f);
        t = (t - d).clamp(0.0, 1.0);
        let w = 1.0 / (t * t + 1.0);
        (qx, qy) = ((1.0 - t * t) * w * a, 2.0 * t * w * b);
        if d.abs() <= 1e-7 {
            break;
        }
    }
    (qx * sx, qy * sy)
}

/// RDR2's joint limit clamp (0x2a23d60 at scale 1) on raw angles [twist, lean1, lean2], limit-centred: the twist per
/// axis, the two leans inside their ellipse (per axis when a half-range is under 0.001).
fn clamp_cone(raw: [f32; 3], limits: &[(f32, f32); 3]) -> [f32; 3] {
    let half = |k: usize| (limits[k].1 - limits[k].0) * 0.5;
    let (h1, h2, ht) = (half(0), half(1), half(2));
    let tw = raw[0].clamp(-ht, ht);
    if h1 < 0.001 || h2 < 0.001 {
        return [tw, raw[1].clamp(-h1, h1), raw[2].clamp(-h2, h2)];
    }
    let (mut l1, mut l2) = (raw[1], raw[2]);
    if ((l2 / h2).powi(2) + (l1 / h1).powi(2)).sqrt() - 1.0 > 0.0 {
        (l1, l2) = ellipse_nearest(l1, l2, h1, h2);
    }
    [tw, l1, l2]
}

/// RDR2's swim (rdr2_swim.md, read through the code): the water plane picks wade (chest under, head out: pedalLegs, the
/// windmill, headLook ahead), swim (chest and head under, the balancer, armsWindmillAdaptive, an alternating
/// three-phase arm stroke, the spine aimed along the swim direction, headLook above the chest) or plunge (chest and
/// head under, sinking faster than 2.5 m/s: foetal and writhe); none, with the chest out. As in RDR2 a state change
/// starts things and stops only what the note lists: pedalLegs and the windmill run on until swim stops.
/// RDR2's state change writes constants into the strokes (no offset, reach 0.63, 4 m/s, direction 0 0 1), so swimOffset,
/// extendDist, extendSpeed and swimDirection never reach the arms there; only the spine reads the direction.
/// Ours: the plane comes with the message (RDR2's core water state, ctx+0x10c4..+0x10c8, is worked out here); no pole for
/// the retract's IK (Delirium's IK takes none; RDR2's is the target itself); bodyBalance stands for RDR2's balancer.
pub struct Swim {
    offset: Vec3, twist: f32, dist: f32, speed: f32, close2: f32, enough2: f32, timeout: f32,
    arm: (f32, f32), leg: (f32, f32), spine: (f32, f32), surface: Vec3, normal: Vec3, dir: Vec3,
    started: bool, state: u8, strokes: [Stroke; 2], kids: Kids, look: Look,
}

const SWIM_WINDMILL: &str = "armsWindmillAdaptive start=true armDirection=-1 disableOnImpact=false setBackAngles=false leftElbowAngle=0.6 \
                             rightElbowAngle=0.6 phase=0.8 amplitude=0.8 bendLeftElbow=true bendRightElbow=true mask=ub angSpeed=4.49 \
                             lean1mult=0.9 lean1offset=1.3 elbowRate=4 armStiffness=11";

impl Swim {
    fn new(p: &mut P) -> Result<Swim, Error> {
        Ok(Swim {
            offset: p.v("swimOffset", Vec3::new(0.0, 0.0, -0.2))?, twist: p.f("IKtwist", 0.0)?, dist: p.f("extendDist", 0.63)?,
            speed: p.f("extendSpeed", 4.0)?, close2: p.f("extendCloseEnough", 0.1)?.powi(2), enough2: p.f("extendEnough", 0.1)?.powi(2),
            timeout: p.f("armPhaseTimeout", 1.0)?, arm: (p.f("armStiffness", 12.0)?, p.f("armDamping", 1.0)?),
            leg: (p.f("legStiffness", 12.0)?, p.f("legDamping", 1.0)?), spine: (p.f("spineStiffness", 12.0)?, p.f("spineDamping", 1.0)?),
            surface: p.v("surfacePoint", Vec3::ZERO)?, normal: p.v("surfaceNormal", Vec3::Z)?, dir: p.v("swimDirection", Vec3::Z)?,
            started: false, state: 0, strokes: [Stroke::default(); 2], kids: Kids::default(), look: Look::default(),
        })
    }

    /// The chest's frame (the arm limb's +0x430, [P] the spine's top part) and the recovery point's local offset.
    fn frame(c: &Ctx) -> crate::nmdbm::M34 {
        c.b.part(spine_part(4)).m
    }

    fn q(c: &Ctx, i: usize) -> Vec3 {
        let m = Self::frame(c);
        m.d + m.b * (0.25 * arm_len(c, i)) - m.c * 0.15
    }

    /// The stroke's phase change (0x2aaae80): into the reach (0), its end point, kept relative to the chest.
    fn phase_to(&mut self, c: &Ctx, i: usize, p: u8) {
        if self.strokes[i].phase == p {
            return;
        }
        let timeout = self.timeout;
        let s = &mut self.strokes[i];
        (s.phase, s.timer, s.done, s.timeout) = (p, 0.0, false, timeout);
        if p == 0 {
            let m = Self::frame(c);
            let d = s.dir;
            let a = m.b * arm_len(c, i);
            let k = a.cross(d);
            let beta = k.length().clamp(-1.0, 1.0).asin();
            let mut t = if beta <= 1.3 { d } else { Quat::from_axis_angle(k.normalize_or(Vec3::X), -1.3) * a };
            t += m.a * s.offset.x + m.b * s.offset.y + m.c * s.offset.z;
            let t = t.normalize_or_zero() * s.dist;
            s.reach = c.b.joint_pos(limb(i).e[0]) + t - m.d;
        }
        note(format_args!("swim: arm {i} phase {p}"));
    }

    /// Activate: both strokes retracting with the message's values, the right one then reaching.
    fn strokes_start(&mut self, c: &Ctx) {
        let (offset, dist, speed, timeout) = (self.offset, self.dist, self.speed, self.timeout);
        self.strokes = [Stroke { phase: 2, offset, dist, speed, timeout, dir: Vec3::Z, ..Stroke::default() }; 2];
        self.phase_to(c, 1, 0);
    }

    /// A state change's reset (inline in 0x2aa81b4): RDR2 writes constants here, not the message's values.
    fn strokes_reset(&mut self, c: &Ctx) {
        for s in &mut self.strokes {
            (s.phase, s.timer, s.done) = (2, 0.0, false);
            (s.offset, s.dist, s.speed, s.timeout, s.dir) = (Vec3::ZERO, 0.63, 4.0, 1.0, Vec3::Z);
        }
        self.phase_to(c, 1, 0);
    }

    /// One arm's stroke (0x2aabd10).
    fn stroke(&mut self, c: &mut Ctx, i: usize, dt: f32, force: bool) {
        let s = self.strokes[i];
        let timer = s.timer + dt;
        let done = s.done || timer > s.timeout;
        self.strokes[i].timer = timer;
        self.strokes[i].done = done;
        if done && !(s.phase == 2 && !force) {
            self.phase_to(c, i, (s.phase + 1) % 3);
        }
        let s = self.strokes[i];
        let m = Self::frame(c);
        let q = Self::q(c, i);
        let hand = pos(c, limb(i).p[3]);
        match s.phase {
            0 => {
                let end = m.d + s.reach;
                let e = end - q;
                let k = if e.length() > 0.0 { (s.speed * s.timer / e.length()).clamp(0.0, 1.0) } else { 1.0 };
                let to = q + e * k;
                arm_ik_drag(c, i, to, self.twist, 1.0, c.vel);
                let ed = e.normalize_or_zero();
                let off = to - hand;
                let along = off.dot(ed).max(0.0);
                let lateral = off - ed * along;
                if lateral.length_squared() < self.close2 && (hand - m.d).length_squared() > s.reach.length_squared() - self.enough2 {
                    self.strokes[i].done = true;
                }
            }
            2 => {
                arm_ik_drag(c, i, q, self.twist, 1.0, c.vel);
                // the request's hand normal: against the stroke's direction
                crate::nmshot::Shot::wrist(c, limb(i), q, -s.dir, 1.0 - 2.0 * i as f32);
                if (q - hand).length() < 0.1 {
                    self.strokes[i].done = true;
                }
            }
            _ => {}
        }
    }

    /// The spine aimed along the swim direction (0x2aa31c4): each spine joint's z axis turned onto it, inside its limits.
    fn spine_aim(&mut self, c: &mut Ctx) {
        let d = self.dir.normalize_or(Vec3::Z);
        let mut clamped = 0;
        for &j in SPINE().iter() {
            let m = c.b.eff3(j).m;
            let local = Vec3::new(m.a.dot(d), m.b.dot(d), m.c.dot(d)).normalize_or(Vec3::Z);
            let raw = crate::nmrs::angles(Quat::from_rotation_arc(Vec3::Z, local));
            let jd = &c.ch.joints[j];
            let [tw, l1, l2] = clamp_cone(raw, &jd.limits);
            clamped += (tw != raw[0] || l1 != raw[1] || l2 != raw[2]) as u32;
            c.ch.eff[j].desired = crate::nmrs::effector_angles([l1, l2, tw], &jd.limits, jd.flip);
        }
        if clamped > 0 {
            note(format_args!("swim: spine aim, {clamped} joints clamped to their cones"));
        }
    }

    fn switch(&mut self, c: &mut Ctx, to: u8) {
        if to == self.state {
            return;
        }
        self.kids.on.retain(|n| *n != "bodyBalance");
        self.kids.start("bodyBalance", "bodyBalance start=true".into());
        if self.state == 1 {
            self.kids.stop("bodyWrithe");
            self.kids.stop("bodyFoetal");
        }
        if self.state == 2 {
            self.strokes_reset(c);
        }
        note(format_args!("swim: state {} -> {to}, sinking {:.2} m/s", self.state, -self.normal.normalize_or_zero().dot(c.vel)));
        match to {
            1 => {
                self.kids.stop("armsWindmillAdaptive");
                self.kids.out.push("bodyBalance start=false".into());
                self.kids.on.retain(|n| *n != "bodyBalance");
                self.look.stop(&mut self.kids.out);
                self.kids.start("bodyFoetal", "bodyFoetal start=true stiffness=9".into());
                self.kids.start("bodyWrithe", "bodyWrithe start=true blendArms=0.5 blendBack=0.5 blendLegs=0.5".into());
            }
            2 => {
                self.kids.start("armsWindmillAdaptive", SWIM_WINDMILL.into());
                self.strokes_reset(c);
            }
            3 => {
                self.kids.start("armsWindmillAdaptive", SWIM_WINDMILL.into());
                self.kids.out.push("bodyBalance start=false".into());
                self.kids.on.retain(|n| *n != "bodyBalance");
                self.kids.start("pedalLegs", "pedalLegs start=true radius=0.25 spread=0.15".into());
            }
            _ => {}
        }
        self.state = to;
    }
}

impl Extra for Swim {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if !self.started {
            self.started = true;
            self.strokes_start(c);
            self.kids.start("bodyBalance", "bodyBalance start=true".into());
        }
        // the water plane (0x2a5dd58 keeps these): in the water is spine part 9 under it, deep the head part 20 [C]
        let n = self.normal.normalize_or_zero();
        let state = if self.surface != Vec3::ZERO && n != Vec3::ZERO {
            let under = |p: Vec3| (p - self.surface).dot(n) < 0.0;
            let in_water = under(pos(c, rp(9)));
            let deep = under(pos(c, rp(20)));
            if !in_water { 0 } else if !deep { 3 } else if -n.dot(c.vel) <= 2.5 { 2 } else { 1 }
        } else {
            0
        };
        self.switch(c, state);
        let f = if state == 3 { 0.85 } else { 1.0 };
        set_k(c, &joints(0xfd800), f * self.arm.0, self.arm.1, None);
        set_k(c, &joints(0xf6), f * self.leg.0, self.leg.1, None);
        set_k(c, &joints(0x102709), f * self.spine.0, self.spine.1, None);
        let chest = pos(c, spine_part(4));
        match state {
            2 => {
                let r2 = self.strokes[1].phase == 2;
                self.stroke(c, 0, dt, r2);
                let l2 = self.strokes[0].phase == 2;
                self.stroke(c, 1, dt, l2);
                self.spine_aim(c);
                self.look.to(&mut self.kids.out, chest + c.up, 11.0);
            }
            3 => {
                // the record's channel bit 2 (value 4) at +0x14: the twist
                for &j in WRIST().iter() {
                    c.ch.eff[j].desired[2] = 0.4;
                }
                let at = chest + c.vel.normalize_or_zero() + c.b.part(spine_part(4)).m.a * 0.5;
                self.look.to(&mut self.kids.out, at, 11.0);
            }
            _ => {}
        }
        // (ctx+0x2eb8, +0x2ebc, +0x2ec0: nothing in the NM code reads what these write, so they are for the game [C])
    }

    fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.kids.out)
    }

    fn stop(&mut self, _c: &mut Ctx) {
        note(format_args!("swim: deactivate, stopping {:?}", self.kids.on));
        for n in ["armsWindmillAdaptive", "pedalLegs", "bodyWrithe", "bodyFoetal"] {
            self.kids.stop(n);
        }
        self.look.stop(&mut self.kids.out);
    }
}

// ---- keyframe ----

/// Our parts for a joint mask: each joint's child part, and the pelvis with both hips.
fn mask_parts(m: &str) -> Vec<usize> {
    let js = crate::nmrs::Character::mask(m);
    let mut ps: Vec<usize> = js.iter().map(|j| j + 1).collect();
    if js.contains(&HIP()[0]) && js.contains(&HIP()[1]) {
        ps.insert(0, 0);
    }
    ps
}

/// RDR2's keyframe parameters (slot 0x120 table).
pub struct KeyframeRdr2 {
    src: (i32, i32),
    active: Vec<usize>,
    muscle: Vec<usize>,
    hard: Vec<usize>,
    soft: Vec<usize>,
    alt: Vec<usize>,
    w: (f32, f32),
    rate: (f32, f32),
    init: (f32, f32),
    blend: (bool, bool),
    accel: Vec3,
    ang: Vec3,
    grav: f32,
    brk: f32,
    momentum: bool,
    forces: bool,
    target_vel: bool,
    spikes: i32,
    cool: i32,
    spike_ang: Vec3,
    started: bool,
    failed: bool,
    ramp: (f32, f32),
    pose: Vec<Transform>,
    angles: Vec<[f32; 3]>,
    prev: Vec<Transform>,
    count: Vec<i32>,
    err: f32,
    hold: Vec<(usize, Option<Vec3>, Option<Quat>)>,
    logged: u8,
    seen_incoming: bool,
}

/// The shortest rotation's axis times its angle.
fn turn(q: Quat) -> Vec3 {
    (if q.w < 0.0 { -q } else { q }).to_scaled_axis()
}

fn clamp3(v: Vec3, l: Vec3) -> Vec3 {
    Vec3::new(v.x.clamp(-l.x, l.x), v.y.clamp(-l.y, l.y), v.z.clamp(-l.z, l.z))
}

/// The tracking's braking along the way to the target (twice in 0x2a1c45c, for the velocity and the turn): if he
/// comes at it faster than its own speed and can't slow in the distance left at these limits, the wanted speed along
/// the way is what he can still shed.
fn brake(want: Vec3, now: Vec3, own: Vec3, way: Vec3, dist: f32, lim: Vec3) -> Vec3 {
    let (cur, anim) = (now.dot(way), own.dot(way));
    if anim < cur {
        let t = dist / (cur - anim);
        let can = t * (lim.x * way.x.abs() + lim.y * way.y.abs() + lim.z * way.z.abs()) + anim;
        if can < cur {
            return want + way * (can - want.dot(way));
        }
    }
    want
}

impl KeyframeRdr2 {
    pub fn new(p: &mut P) -> Result<KeyframeRdr2, Error> {
        Ok(KeyframeRdr2 {
            src: (p.i("animSourceCurrent", 0)?, p.i("animSourcePrevious", 1)?),
            active: crate::nmrs::Character::mask(&p.s("activePoseMask", "na")?),
            muscle: crate::nmrs::Character::mask(&p.s("muscleKeyMask", "na")?),
            hard: mask_parts(&p.s("hardKeyMask", "fb")?),
            soft: mask_parts(&p.s("softKeyMask", "na")?),
            alt: mask_parts(&p.s("softKeyMaskAlt", "na")?),
            w: (p.f("softweight", 1.0)?.clamp(0.0, 1.0), p.f("softweightalt", 1.0)?.clamp(0.0, 1.0)),
            rate: (p.f("softweightrate", 1.0)?, p.f("softweightaltrate", 1.0)?),
            init: (p.f("softweightinitial", 0.0)?, p.f("softweightaltinitial", 0.0)?),
            blend: (p.b("blendinsoftweight", false)?, p.b("blendinsoftweightalt", false)?),
            accel: p.v("softmaxaccel", Vec3::splat(50.0))?, ang: p.v("softmaxangaccel", Vec3::splat(50.0))?,
            grav: p.f("softgravity", 0.0)?, brk: p.f("softbreakthreshold", -1.0)?,
            momentum: p.b("softpreservemomentum", false)?, forces: p.b("useForces", false)?, target_vel: p.b("usetargetvel", false)?,
            spikes: p.i("spikes", 0)?, cool: p.i("spikecooldown", 3)?, spike_ang: p.v("spikeangaccel", Vec3::splat(1000.0))?,
            started: false, failed: false, ramp: (0.0, 0.0), pose: vec![], angles: vec![], prev: vec![], count: vec![], err: 0.0,
            hold: vec![], logged: 0, seen_incoming: false,
        })
    }

    /// softKeyframe under the core: RDR2's soft keying of its mask (nothing hard keyed), RDR2's defaults otherwise.
    pub fn soft(mask: &str, accel: Option<f32>) -> KeyframeRdr2 {
        let mut k = KeyframeRdr2::new(&mut P::empty()).expect("defaults");
        (k.hard, k.soft) = (vec![], mask_parts(mask));
        if let Some(a) = accel {
            k.accel = Vec3::splat(a);
        }
        k
    }

    /// The pose tracked: the incoming animation when the agent has one in world space, else the pose at the start.
    fn targets(&mut self, c: &Ctx) -> (Vec<Transform>, Vec<[f32; 3]>) {
        let n = c.b.frames.len();
        if c.incoming.len() >= n && c.incoming.iter().any(|t| t.translation != Vec3::ZERO) {
            if trace() && self.logged & 1 != 0 && !self.seen_incoming {
                self.seen_incoming = true;
                eprintln!("keyframe: tracking the incoming buffer");
            }
            let m = crate::nmrs::incoming_rows(&c.b.doll.links, c.incoming);
            let a = (0..c.ch.eff.len()).map(|j| c.ch.incoming_angle(j, &m)).collect();
            return (c.incoming[..n].to_vec(), a);
        }
        (self.pose.clone(), self.angles.clone())
    }

    fn say(&mut self, bit: u8, s: String) {
        if trace() && self.logged & bit == 0 {
            self.logged |= bit;
            eprintln!("{s}");
        }
    }

    pub fn tick(&mut self, c: &mut Ctx, dt: f32) {
        let up = c.up;
        if !self.started {
            self.started = true;
            self.failed = false;
            self.ramp = (if self.blend.0 { self.init.0 } else { 0.0 }, if self.blend.1 { self.init.1 } else { 0.0 });
            self.pose = c.b.frames.to_vec();
            self.angles = (0..c.ch.eff.len()).map(|j| c.ch.actual(j, c.b.frames)).collect();
            self.count = vec![0; self.pose.len()];
            // activate: the foot yaw hold off and its latches let go, ZMP gravity compensation off (ctx+0x47fd, +0x4800, +0x2b54)
            (c.ch.core.foot_hold, c.ch.core.foot_unlatch, c.ch.core.zmp) = (false, true, false);
            if !self.hard.is_empty() && self.grav > 0.0 {
                for &k in self.soft.iter().chain(&self.alt) {
                    if k > 0 {
                        c.ch.eff[k - 1].oppose_gravity = 0.0;
                    }
                }
            }
            self.say(1, format!("keyframe: activate, hard {} soft {} alt {} muscle {} active {}, foot hold and ZMP off",
                                   self.hard.len(), self.soft.len(), self.alt.len(), self.muscle.len(), self.active.len()));
        }
        let (target, angles) = self.targets(c);
        let _ = self.src;
        // desired from the incoming pose on every keyed joint
        let mut m: Vec<usize> = self.active.clone();
        m.extend(self.hard.iter().chain(&self.soft).chain(&self.alt).filter(|&&k| k > 0).map(|&k| k - 1));
        for &j in &m {
            if let Some(a) = angles.get(j) {
                c.ch.eff[j].desired = *a;
            }
        }
        // the hard keyed parts: 0x2a1b1a4 sets their pose from the buffer and their velocities to nothing while the mode
        // is 3 (ours: Delirium's hard constraints on the pose)
        self.hold = self.hard.iter().filter_map(|&k| target.get(k).map(|t| (k, Some(t.translation), Some(t.rotation)))).collect();
        if !self.failed {
            if self.brk >= 0.0 && self.brk < self.err {
                self.failed = true;
                self.err = 0.0;
                c.out.send(FAILURE(), "keyframe", 0.0);
                self.say(2, format!("keyframe: break, tracking error over {}", self.brk));
            } else {
                self.soft_track(c, &target, up, dt);
                // the hard record's writer: 15 stiff on the muscle key mask, only while soft keying hasn't failed
                set_k(c, &self.muscle, 15.0, 1.0, Some(1.0));
            }
        }
        // 0x2a186d0: the muscle target of the masked joints is the pose's angles (3-DOF inside the limits), without the
        // hard keyed parts' joints. Its loop starts at joint 1, so the left hip is never overridden [C]
        let (hard, knee_elbow): (Vec<usize>, Vec<usize>) = (self.hard.iter().filter(|&&k| k > 0).map(|&k| k - 1).collect(), KNEE().iter().chain(ELBOW().iter()).copied().collect());
        let mut kept = 0;
        for &j in self.muscle.iter().filter(|j| !hard.contains(j) && **j != HIP()[0]) {
            if let Some(a) = angles.get(j) {
                let l = c.ch.limits(j);
                c.ch.eff[j].desired = if knee_elbow.contains(&j) { *a } else { [0, 1, 2].map(|k| a[k].clamp(l[2 * k], l[2 * k + 1])) };
                kept += 1;
            }
        }
        if !self.muscle.is_empty() {
            self.say(128, format!("keyframe: muscle key overrides {kept} of {} joints, failed {}", self.muscle.len(), self.failed));
        }
        self.prev = target;
    }

    /// The soft keying (0x2a1c45c): each soft part's velocity and turn driven at the pose, braked to what the limits can
    /// still shed, the accelerations clamped per axis by the part's weight (angular in the COM frame, with the spike
    /// allowance). RDR2 sets the parts' velocities unless useForces; here both are forces over the step.
    /// softpreservemomentum as the code has it: the tracked parts' mean velocity before the step added to each after [C].
    /// useForces: what the parts' forces leave of the trunk's own tracking is shared out by mass (ours [P]: the frame's
    /// rotations are the parts', not relative to their rest; the inertia is taken about the axis of the turn).
    fn soft_track(&mut self, c: &mut Ctx, target: &[Transform], up: Vec3, dt: f32) {
        let mut w = self.w.0;
        if self.blend.0 {
            self.ramp.0 = (self.ramp.0 + dt * self.rate.0).clamp(self.init.0, 1.0);
            w *= self.ramp.0;
        }
        let mut wa = self.w.1;
        if self.blend.1 {
            self.ramp.1 = (self.ramp.1 + dt * self.rate.1).clamp(self.init.1, 1.0);
            wa *= self.ramp.1;
        }
        let mut weight = vec![0.0f32; target.len()];
        for &k in &self.soft {
            if let Some(x) = weight.get_mut(k) { *x = w; }
        }
        for &k in &self.alt {
            if let Some(x) = weight.get_mut(k) { *x = wa; }
        }
        let rows = [c.m.a, c.m.b, c.m.c];
        let (mut sum, mut n, mut mass, mut mv) = (0.0, 0, 0.0, Vec3::ZERO);
        let mut push: Vec<(usize, Vec3, Vec3)> = vec![];
        let (mut last_a, mut last_w) = (Vec3::ZERO, Vec3::ZERO);
        let (mut sum_f, mut sum_t) = (Vec3::ZERO, Vec3::ZERO);
        let (mut spiked, mut braked) = (0, 0);
        let moved = self.target_vel || self.forces;
        for k in 0..target.len().min(c.b.frames.len()) {
            let wk = weight[k];
            if wk <= 0.0 || self.hard.contains(&k) {
                continue;
            }
            let part = c.b.part(k);
            let pm = c.b.doll.part_mass(k);
            (mass, mv) = (mass + pm, mv + part.v * pm);
            let now = c.b.frames[k];
            let t = target[k];
            let (lim_a, lim_w) = (self.accel * wk, self.ang * wk);
            // the linear half
            let gap = t.translation - now.translation;
            let dist = gap.length();
            let p = self.prev.get(k).copied().unwrap_or(t);
            let own = (t.translation - p.translation) / dt;
            let mut v = gap / dt;
            if moved {
                v += own;
            }
            let way = gap.normalize_or_zero();
            let v2 = brake(v, part.v, own, way, dist, lim_a);
            braked += (v2 != v) as u32;
            let g = if self.forces { 0.0 } else { self.grav };
            // the gravity share after the clamp (RDR2's fixed vector (0, -9.8, 0), read here as along his up)
            let a = clamp3((v2 - part.v) / dt, lim_a) + up * (9.8 * g);
            // the angular half
            let err = turn(t.rotation * now.rotation.inverse());
            let ang = err.length();
            let own_w = turn(t.rotation * p.rotation.inverse()) / dt;
            let mut wv = err / dt;
            if moved {
                wv += own_w;
            }
            let wv = brake(wv, part.w, own_w, err.normalize_or_zero(), ang, lim_w);
            let al = (wv - part.w) / dt;
            let lc = Vec3::new(rows[0].dot(al), rows[1].dot(al), rows[2].dot(al));
            let over = lc.x.abs() > lim_w.x || lc.y.abs() > lim_w.y || lc.z.abs() > lim_w.z;
            // the spikes: past them the counter runs up and comes round to 0, then an over-limit step may spike again
            let cnt = &mut self.count[k];
            if self.spikes <= *cnt {
                *cnt += 1;
            }
            if self.spikes + self.cool <= *cnt {
                *cnt = 0;
            }
            let lim = if *cnt < self.spikes && over {
                *cnt += 1;
                spiked += 1;
                self.spike_ang
            } else {
                lim_w
            };
            let lc = clamp3(lc, lim);
            let al = rows[0] * lc.x + rows[1] * lc.y + rows[2] * lc.z;
            (last_a, last_w) = (lim_a, lim);
            sum_f += a * pm;
            sum_t += al * c.b.doll.inertia(k, al.normalize_or(Vec3::Z));
            push.push((k, a, al));
            sum += dist;
            n += 1;
        }
        self.err = if n > 0 { sum / n as f32 } else { 0.0 };
        if push.is_empty() {
            return;
        }
        self.say(4, format!("keyframe: soft tracking {n} parts, mean error {:.3} m, weight {w:.2}/{wa:.2}", self.err));
        if braked > 0 {
            self.say(8, format!("keyframe: braking along the way on {braked} parts"));
        }
        if spiked > 0 {
            self.say(16, format!("keyframe: {spiked} parts spiked"));
        }
        let mut extra = (Vec3::ZERO, Vec3::ZERO);
        if self.forces {
            extra = self.trunk(c, target, dt, (last_a, last_w), (sum_f, sum_t), n as f32);
        } else if self.momentum {
            let mean = mv / mass.max(1e-6);
            push.iter_mut().for_each(|p| p.1 += mean / dt);
            self.say(32, format!("keyframe: momentum, {:.3} m/s added", mean.length()));
        }
        for (k, a, al) in push {
            let pm = c.b.doll.part_mass(k);
            c.out.force(k, a * pm + extra.0 * (pm / mass.max(1e-6)));
            let l = al.length();
            let tq = if l > 1e-6 { al * c.b.doll.inertia(k, al / l) } else { Vec3::ZERO };
            c.out.torque(k, tq + extra.1 * (pm / mass.max(1e-6)));
        }
    }

    /// useForces' second half of 0x2a1c45c: the trunk (RDR2's mask 0x102709) tracked as one body. The force and torque it
    /// would take, less what the parts already got, a share for each tracked part.
    fn trunk(&mut self, c: &mut Ctx, target: &[Transform], dt: f32, lim: (Vec3, Vec3), got: (Vec3, Vec3), n: f32) -> (Vec3, Vec3) {
        let trunk = parts(0x102709);
        let (mut m, mut pc, mut pi, mut v, mut wc) = (0.0, Vec3::ZERO, Vec3::ZERO, Vec3::ZERO, Vec3::ZERO);
        for &k in &trunk {
            let (pm, part) = (c.b.doll.part_mass(k), c.b.part(k));
            m += pm;
            pc += c.b.frames[k].translation * pm;
            pi += target[k].translation * pm;
            v += part.v * pm;
            wc += part.w * pm;
        }
        let m = m.max(1e-6);
        let (pc, pi, v, wc) = (pc / m, pi / m, v / m, wc / m);
        let frame = |rot: &dyn Fn(usize) -> Quat| {
            let (mut x, mut y) = (Vec3::ZERO, Vec3::ZERO);
            for &k in &trunk {
                let pm = c.b.doll.part_mass(k);
                x += rot(k) * Vec3::X * pm;
                y += rot(k) * Vec3::Y * pm;
            }
            let x = x.normalize_or(Vec3::X);
            let z = x.cross(y).normalize_or(Vec3::Z);
            Quat::from_mat3(&bevy_math::Mat3::from_cols(x, z.cross(x), z))
        };
        let (rc, ri, rp_) = (frame(&|k| c.b.frames[k].rotation), frame(&|k| target[k].rotation),
                             frame(&|k| self.prev.get(k).unwrap_or(&target[k]).rotation));
        let total: f32 = (0..c.b.frames.len()).map(|k| c.b.doll.part_mass(k)).sum();
        let a = clamp3(((pi - pc) / dt - v) / dt, lim.0);
        let force = (a * total - got.0) / n;
        let err = turn(ri * rc.inverse());
        let own = turn(ri * rp_.inverse()) / dt;
        let wv = brake(err / dt + own, wc, own, err.normalize_or_zero(), err.length(), Vec3::splat(180.0));
        let al = (wv - wc) / dt;
        let (ra, rb, rcc) = (rc * Vec3::X, rc * Vec3::Y, rc * Vec3::Z);
        let lc = clamp3(Vec3::new(ra.dot(al), rb.dot(al), rcc.dot(al)), lim.1);
        let al = ra * lc.x + rb * lc.y + rcc * lc.z;
        let axis = al.normalize_or(Vec3::Z);
        let inertia: f32 = (0..c.b.frames.len()).map(|k| {
            let r = c.b.frames[k].translation - pc;
            c.b.doll.inertia(k, axis) + c.b.doll.part_mass(k) * axis.cross(r).length_squared()
        }).sum();
        let torque = (al * inertia - got.1) / n * 0.9;
        self.say(64, format!("keyframe: useForces trunk force {:.1} N, torque {:.1} Nm", force.length(), torque.length()));
        (force, torque)
    }

    pub fn constraints(&self) -> Vec<(usize, Option<Vec3>, Option<Quat>)> {
        self.hold.clone()
    }

    /// Deactivate: hard keying off; with the core, the foot yaw hold and ZMP gravity compensation back on.
    pub fn stop(&mut self, c: &mut Ctx) {
        self.hold.clear();
        if self.started && c.ch.rdr2 {
            (c.ch.core.foot_hold, c.ch.core.zmp) = (true, true);
            note(format_args!("keyframe: deactivate, foot hold and ZMP on"));
        }
    }
}
