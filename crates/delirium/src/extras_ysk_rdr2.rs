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
    at: Option<(Vec3, (f32, f32, bool))>,
}

impl Look {
    fn to(&mut self, out: &mut Vec<String>, at: Vec3, stiffness: f32, damping: f32, always: bool) {
        match self.at {
            None => out.push(format!("headLook start=true alwaysLook={always} pos={} stiffness={stiffness} damping={damping}", v3s(at))),
            Some((was, k)) if was.distance(at) > 0.05 || k != (stiffness, damping, always) => {
                out.push(format!("headLook alwaysLook={always} pos={} stiffness={stiffness} damping={damping}", v3s(at)))
            }
            _ => return,
        }
        self.at = Some((at, (stiffness, damping, always)));
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

/// RDR2's yanked parameters (slot 0x88 table and the bolas block; RDR2's defaults, read from its default blocks).
#[derive(Clone, Copy, Debug)]
pub struct YankedParams {
    arm_start: (f32, f32), spine_start: (f32, f32), arm: (f32, f32), spine: (f32, f32),
    steps_till: i32, per_step: f32, time_end: f32, ramp_to: f32, legs: (f32, f32), loose: [f32; 3],
    spine_limit: [f32; 3], hip_pitch: (f32, f32), lean_hips: (f32, f32), lean: (f32, f32), restart_up: f32, bend: f32,
    foot_friction: f32, neck_foot_friction: f32, turn: (f32, f32),
    head_pos: Vec3, use_head: bool, head_prob: Vec3, pedal_slow: bool, leg_amp: f32, grab_spine_freq: f32, spine_freq: f32,
    spine_amp: f32, spine_a: [f32; 5], side: (f32, f32), roll: [f32; 4], moving: f32, roll_type: i32,
    ground: [(f32, f32); 3], neck_k: f32, ground_friction: f32, rope_end: f32, other_end: f32,
    pub lassoed: i32, pub grab_part: usize, grab_pos: [Vec3; 2], grab_normal: [Vec3; 2], claw_amp: Vec3, claw_freq: Vec3,
    hands_off: Vec3, head_grab: f32, arm_feet: Vec3, feet_prob: Vec3, arm_down: Vec3, right_grabs: bool, spine_prob: Vec3,
    state_time: (f32, f32), die: f32, limp_rate: f32, struggle: f32, fsim: (f32, f32), bump: (f32, f32),
    stirrup: bool, animal: bool, cart: i32, stable_neck: bool, stunned: (f32, f32),
    bolas: bool, bolas_forces: bool, bolas_share: f32, bolas_k: [f32; 4], bolas_time: f32, bolas_first: f32, bolas_period: f32,
    bolas_dir: Vec3, pub rope: Vec3, pub rope_part: usize,
}

impl YankedParams {
    /// RDR2's names and defaults. `point` and `part` (the room's stand-in rope of Delirium's own yanked) are the rope's
    /// anchor and roped part; rdr2Lasso's ropePos/ropePart likewise. timeAtStartValues, rampTimeFromStartValues and the
    /// two amplitudes at +0x138 and +0x13c are in RDR2's table and nothing in its code reads them: taken, as RDR2 takes them.
    pub fn new(p: &mut P) -> Result<YankedParams, Error> {
        let rope = if p.v("ropePos", Vec3::ZERO)? != Vec3::ZERO { p.v("ropePos", Vec3::ZERO)? } else { p.v("point", Vec3::ZERO)? };
        let part = p.i("part", 10)?;
        let rope_part = p.i("ropePart", part)?.clamp(0, 20) as usize;
        let _ = (p.f("timeAtStartValues", 0.4)?, p.f("rampTimeFromStartValues", 0.1)?);
        let _ = (p.f("spineAmp138", 0.4)?, p.f("spineAmp13c", 1.0)?);
        Ok(YankedParams {
            arm_start: (p.f("armStiffnessStart", 3.0)?, p.f("armDampingStart", 0.1)?),
            spine_start: (p.f("spineStiffnessStart", 3.0)?, p.f("spineDampingStart", 0.1)?),
            arm: (p.f("armStiffness", 13.0)?, p.f("armDamping", 0.5)?), spine: (p.f("spineStiffness", 12.0)?, p.f("spineDamping", 0.5)?),
            steps_till: p.i("stepsTillStartEnd", 1)?, per_step: p.f("perStepReduction", 1.0)?,
            time_end: p.f("timeStartEnd", 100.0)?, ramp_to: p.f("rampTimeToEndValues", 0.0)?,
            legs: (p.f("lowerBodyStiffness", 16.0)?, p.f("lowerBodyStiffnessEnd", 15.0)?),
            loose: [p.f("loosetimetorso", 0.3)?, p.f("loosetimelegs", 0.1)?, p.f("loosestrength", 0.1)?],
            spine_limit: [p.f("spinelean1limit", 0.5)?, p.f("spinelean2limit", 0.3)?, p.f("torqueexaggeratejumplimit", 0.2)?],
            hip_pitch: (p.f("hipPitchForward", 0.3)?, p.f("hipPitchBack", 0.5)?),
            lean_hips: (p.f("leanhipsmin", -0.4)?, p.f("leanhipsmax", 0.2)?), lean: (p.f("leanmin", -0.1)?, p.f("leanmax", -0.3)?),
            restart_up: p.f("restartUpness", 1.1)?, bend: p.f("spineBend", 1.0)?,
            foot_friction: p.f("footFriction", 0.3)?, neck_foot_friction: p.f("neckFootFriction", 0.1)?,
            turn: (p.f("turnThresholdMin", 0.5)?, p.f("turnThresholdMax", 1.5)?),
            head_pos: p.v("headLookPos", Vec3::ZERO)?, use_head: p.b("useHeadLook", true)?,
            head_prob: p.v("headlookprob", Vec3::splat(0.3))?, pedal_slow: p.b("pedalWhenSlow", false)?, leg_amp: p.f("legAmplitude", 0.4)?,
            grab_spine_freq: p.f("grabspinefreq", 0.5)?, spine_freq: p.f("spinefreq", 2.5)?, spine_amp: p.f("spineAmplitude", 1.0)?,
            spine_a: [p.f("spineAmpUp", 0.4)?, p.f("spineAmpUpNeck", 0.4)?, p.f("spineAmpDown", 1.0)?, p.f("spineBigAmp", 4.0)?, p.f("spineBigAmpUp", 0.4)?],
            side: (p.f("spsidefreq", 2.0)?, p.f("spsideamp", 1.0)?),
            roll: [p.f("rollHelp", 0.0)?, p.f("rollhelpback", 0.0)?, p.f("rollhelpmoving", 0.0)?, p.f("rollhelpbackmoving", 0.0)?],
            moving: p.f("movingthresh", 1.0)?, roll_type: p.i("rollType", 0)?,
            ground: [(p.f("groundLegStiffness", 11.0)?, p.f("groundLegDamping", 0.5)?), (p.f("groundArmStiffness", 11.0)?, p.f("groundArmDamping", 0.5)?),
                     (p.f("groundSpineStiffness", 11.0)?, p.f("groundSpineDamping", 0.5)?)],
            neck_k: p.f("groundneckstiffness", 11.0)?, ground_friction: p.f("groundFriction", 1.0)?,
            rope_end: p.f("ropeendfriction", 0.5)?, other_end: p.f("otherendfriction", 2.0)?,
            lassoed: p.i("lassoedtype", 0)?, grab_part: p.i("grabPart", 10)?.clamp(0, 20) as usize,
            grab_pos: [p.v("grabposl", Vec3::new(0.07, 0.05, -0.051))?, p.v("grabposr", Vec3::new(0.09, -0.05, -0.051))?],
            grab_normal: [p.v("grabnormall", Vec3::NEG_Z)?, p.v("grabnormalr", Vec3::NEG_Z)?],
            claw_amp: p.v("grabpointamp", Vec3::ZERO)?, claw_freq: p.v("grabpointfreq", Vec3::new(4.0, 5.0, 3.0))?,
            hands_off: p.v("grabhandsoffset", Vec3::ZERO)?, head_grab: p.f("headGrabProb", 0.333)?,
            arm_feet: p.v("armFeetProb", Vec3::new(0.2, 0.7, 0.0))?, feet_prob: p.v("feetprob", Vec3::new(0.5, 0.5, 0.0))?,
            arm_down: p.v("armDownProb", Vec3::new(0.55, 0.0, 0.1))?, right_grabs: p.b("rightGrabsAfter", true)?,
            spine_prob: p.v("spineprob", Vec3::new(0.6, 0.4, 0.05))?,
            state_time: (p.f("minstatetime", 1.0)?, p.f("maxstatetime", 1.2)?), die: p.f("timetodie", 15.0)?,
            limp_rate: p.f("limpRate", 0.0)?, struggle: p.f("struggle", 0.0)?,
            fsim: (p.f("frictionsim", 70.0)?, p.f("frictionsimairborne", 70.0)?), bump: (p.f("bumpsim", 400.0)?, p.f("bumpsimprob", 0.08)?),
            stirrup: p.b("draggedbystirrup", false)?, animal: p.b("draggedbyanimal", false)?, cart: p.i("DraggedByCart", 0)?,
            stable_neck: p.b("stableneck", false)?, stunned: (p.f("stunnedtime", 0.0)?, p.f("stunnedblendout", 0.0)?),
            bolas: p.b("bolas", false)?, bolas_forces: p.b("bolasForces", false)?, bolas_share: p.f("bolasHeadShare", 0.05)?,
            bolas_k: [p.f("bolasFeet", 1000.0)?, p.f("bolasSlow", 3000.0)?, p.f("bolasNeck", 2500.0)?, p.f("bolasFast", 5000.0)?],
            bolas_time: p.f("bolasTime", 0.35)?, bolas_first: p.f("bolasFirst", 0.1)?, bolas_period: p.f("bolasPeriod", 0.065)?,
            bolas_dir: p.v("bolasDir", Vec3::X)?, rope, rope_part,
        })
    }
}

/// RDR2's yanked (rdr2_yanked.md, section 10 checked against the code), the reaction to a rope: on his feet the balancer
/// leans about the rope and is sometimes asked to turn, his upper body goes loose at the jerk and the legs soften step by
/// step; once the balancer fails, the ground mode's noise-driven writhe, kicks, clawing and rope grabs, the sliding
/// friction and bumps, until timetodie.
pub struct YankedRdr2 {
    pub p: YankedParams,
    started: bool,
    first: bool,
    said: u64,
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
    alive: bool,
    back: bool,
    limp: bool,
    turn_side: bool,
    turn_thresh: f32,
    turn_timer: f32,
    turn_hold: f32,
    turn_go: bool,
    flip_p: f32,
    signs: [f32; 3],
    hip: f32,
    heading_now: f32,
    unwrap: f32,
    raw_was: f32,
    steps0: u32,
    last_step: u32,
    gains: [f32; 4],
    arm_raw: [u8; 2],
    arm: [u8; 2],
    arm_t: [f32; 2],
    leg_raw: [u8; 2],
    leg: [u8; 2],
    leg_t: [f32; 2],
    spine_raw: u8,
    spine_t: f32,
    head_raw: u8,
    head: u8,
    head_t: f32,
    neck_pose: f32,
    arm_k: f32,
    slow_latch: bool,
    grab_latch: bool,
    grab_first: u8,
    /// +0x3f6, one holding flag round the chest (the right's overwrites the left's); +0x3f8, +0x3f9 round the feet
    held_chest: bool,
    held_feet: [bool; 2],
    code: [u8; 2],
    pose: [Option<[f32; 6]>; 2],
    jerks: i32,
    latched: bool,
    latched_t: f32,
    touched: bool,
    airborne: f32,
    air_t: f32,
    bump_mode: i32,
    bolas_t: f32,
    bolas_sign: f32,
    bolas_once: bool,
    roll_t: f32,
    pedal: [bool; 2],
    /// A running sit's stirrups, left and right, alive
    pub stirrups: Option<[bool; 2]>,
    free: [bool; 2],
    fric: [f32; 21],
    writhe_mask: &'static str,
    wind_mask: &'static str,
    kids: Kids,
    look: Look,
    scale: Scale,
}

const WINDMILL: &str = "armsWindmillAdaptive start=true armDirection=2 angSpeed=4 phase=-3.1 disableOnImpact=false bendLeftElbow=true \
                        bendRightElbow=true lean1mult=1.01 lean1offset=0 swayamount=1 angSpeedMultiplier4Dragging=1.8 maxangspeed=4 \
                        scrabblebendupper=4 scrabblebendlower=1.57 scrabblebend=2.2 scrabbleextend=0 scrabble=true armStiffness=13 amplitude=1.8";
const WRITHE: &str = "bodyWrithe start=true blendArms=0.7 blendBack=0 blendLegs=0 applyStiffness=false onFire=true elbowAmplitude=0.5 \
                      armAmplitude=2 armPeriod=1.5 rollTorqueScale=0 rollOverFlag=false";
/// The left and right arm's masks in RDR2's part bits.
const ARM_BITS: [u32; 2] = [0x1c800, 0xe1000];
const TAU: f32 = std::f32::consts::TAU;

impl YankedRdr2 {
    pub fn new(p: YankedParams) -> YankedRdr2 {
        YankedRdr2 {
            p, started: false, first: true, said: 0, clock: 0.0, yank: 100000.0, since_struggle: 10.0, touched_t: 0.0, ground_t: 0.0, land: -0.2,
            phase: 0.0, phase2: 0.0, seed: 0.0, ground: false, alive: true, back: false, limp: false, turn_side: false, turn_thresh: 0.0,
            turn_timer: 0.0, turn_hold: 0.0, turn_go: false, flip_p: 0.0, signs: [1.0; 3], hip: 0.0, heading_now: 0.0, unwrap: 0.0,
            raw_was: 0.0, steps0: 0, last_step: 0, gains: [0.0; 4], arm_raw: [0; 2], arm: [0; 2], arm_t: [0.0; 2], leg_raw: [0; 2],
            leg: [0; 2], leg_t: [0.0; 2], spine_raw: 0, spine_t: 0.0, head_raw: 0, head: 0, head_t: 0.0, neck_pose: 1.0, arm_k: 13.0,
            slow_latch: false, grab_latch: false, grab_first: 0, held_chest: false, held_feet: [false; 2], code: [12; 2], pose: [None; 2], jerks: 0,
            latched: false, latched_t: 0.0, touched: false, airborne: 0.0, air_t: 0.0, bump_mode: 1, bolas_t: 0.0, bolas_sign: 1.0, bolas_once: true,
            roll_t: -0.1, pedal: [false; 2], stirrups: None, free: [false; 2], fric: [1.0; 21], writhe_mask: "lb", wind_mask: "", kids: Kids::default(), look: Look::default(),
            scale: Scale::default(),
        }
    }

    pub fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.kids.out)
    }

    /// The rope's anchor, for a message without start (rdr2Lasso's ropePos as the rope moves).
    pub fn rope_to(&mut self, at: Vec3) {
        self.p.rope = at;
    }

    /// With RDR2_TRACE: a line the first time a branch runs.
    fn ran(&mut self, bit: u32, what: &str) {
        if self.said >> bit & 1 == 0 {
            self.said |= 1 << bit;
            note(format_args!("yanked: {what}"));
        }
    }

    /// The per-part friction array (0x2a4de94, ctx+0x3570): stays as written. Ours lapses when he stops.
    fn fric_set(&mut self, bits: u32, x: f32) {
        (0..21).filter(|i| bits >> i & 1 != 0).for_each(|i| self.fric[i] = x);
    }

    /// The rope direction's angle (0x2a7eb0c, our rope: spine2's position less the anchor) from the pelvis's r2 axis,
    /// signed about up, both flattened. Outside the first call a jump past 7 pi / 10 from the last adds a turn to the unwrap.
    fn heading(&mut self, c: &Ctx, init: bool) {
        let up = c.up;
        let d = flat(pos(c, spine_part(3)) - self.p.rope, up).normalize_or(Vec3::X);
        let r2 = flat(c.b.part(0).m.c, up).normalize_or(Vec3::X);
        let x = r2.cross(d);
        let mut a = x.length().atan2(r2.dot(d));
        if x.dot(up) < 0.0 {
            a = -a;
        }
        if !init {
            let jump = self.heading_now - a;
            if jump < -2.1991148 {
                self.unwrap -= TAU;
            }
            if jump > 2.1991148 {
                self.unwrap += TAU;
            }
        }
        self.heading_now = a;
    }

    fn activate(&mut self, c: &mut Ctx) {
        self.started = true;
        if self.p.rope == Vec3::ZERO {
            // ours: no rope given, 2 m in front of the chest (the room's stand-in yanked's default)
            let f = flat(-c.m.c, c.up).normalize_or(Vec3::X);
            self.p.rope = pos(c, spine_part(4)) + f * 2.0;
        }
        (self.bolas_t, self.bolas_sign, self.bolas_once) = (self.p.bolas_first, 1.0, true);
        (self.yank, self.since_struggle, self.clock) = (100000.0, 10.0, 0.0);
        self.phase = 4096.0 * u(c);
        self.phase2 = 4096.0 * u(c);
        self.seed = c.ch.agent as f32;
        let _ = c.ch.random_bits();
        self.ground = false;
        for j in joints(0x1fff09) {
            c.ch.eff[j].desired = [0.0; 3];
        }
        if self.p.use_head {
            let at = if self.p.head_pos == Vec3::ZERO { self.p.rope } else { self.p.head_pos };
            self.look.to(&mut self.kids.out, at, 11.0, 1.0, true);
        }
        // the head, neck and spine0..2's oppose gravity: spine3 keeps its own
        for &j in SPINE()[..3].iter().chain(NECK().iter()) {
            c.ch.eff[j].oppose_gravity = 0.0;
        }
        if c.dbm.is_none() {
            self.kids.start("bodyBalance", "bodyBalance start=true".into());
        }
        self.turn_side = c.ch.random_bits() & 1 != 0;
        self.turn_thresh = ur(c, self.p.turn.0, self.p.turn.1);
        self.steps0 = c.dbm.map_or(0, |d| d.steps);
        self.gains = [self.p.arm_start.0, self.p.arm_start.1, self.p.spine_start.0, self.p.spine_start.1];
        (self.hip, self.signs) = (0.0, [1.0; 3]);
        self.turn_go = c.ch.random_bits() & 1 != 0;
        // the writhe's mask leaves the arms out unless a grab holds round the feet (see grab), and it blends only arms
        self.kids.start("bodyWrithe", format!("{WRITHE} mask=lb"));
        self.arm_raw = [0; 2];
        self.arm = [0; 2];
        self.arm_t = [0.0; 2];
        self.leg_raw = [0; 2];
        self.leg = [0; 2];
        self.leg_t = [0.0; 2];
        (self.spine_raw, self.spine_t, self.head_raw, self.head, self.head_t) = (0, 0.0, 0, 0, 0.0);
        self.bump_mode = 1;
        self.alive = true;
        self.heading(c, true);
        self.ran(0, "activate: start values, balancer asked, writhe started");
    }

    pub fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if !self.started {
            self.activate(c);
        }
        self.since_struggle = if self.p.struggle >= 0.1 { -1e-6 } else { (self.since_struggle + dt).min(10.0) };
        if self.p.bolas {
            self.bolas(c, dt);
        }
        let mut ty = if (0..=2).contains(&self.p.lassoed) { self.p.lassoed } else { 1 };
        // round the neck: the balancer's force balance (bal+0x464; ctx+0x2ed8's reader isn't known)
        if self.p.lassoed == 0 {
            c.out.dbm_force_balance = true;
            self.ran(1, "type 0: the balancer's force balance on");
        }
        if self.p.bolas && ty == 0 {
            ty = 1;
        }
        // ours: with no balancer at all he can't fail one, so past half a second that counts as down
        let failed = c.dbm_failed || c.dbm.is_none() && self.clock > 0.5;
        if !self.ground && failed {
            self.ground = true;
            self.arm_t = [0.0; 2];
            self.leg_t = [0.0; 2];
            (self.spine_t, self.head_t) = (0.0, 0.0);
            note(format_args!("yanked: down at {:.2} s (balancer failed {})", self.clock, c.dbm_failed));
        }
        for t in self.arm_t.iter_mut().chain(self.leg_t.iter_mut()) {
            *t -= dt;
        }
        self.spine_t -= dt;
        self.head_t -= dt;
        if !self.ground {
            self.feet(c, ty, dt);
        } else {
            self.down(c, ty, dt);
            let (st, bo) = self.p.stunned;
            if self.touched_t < st + bo {
                let x = (self.touched_t - st).clamp(0.0, bo);
                self.scale.ask(&joints(0x16ff5f), [0.5 + 0.25 * x, 1.0, 1.0], true);
                self.ran(2, "stunned: the scale on the body");
            }
            if self.p.struggle < -0.01 {
                let s = ((-self.p.struggle).clamp(0.0, 1.0) + 1.0) * 0.5;
                self.scale.ask(&joints(0x16ff5f), [s, 1.0, 1.0], true);
                self.ran(3, "negative struggle: weakened");
            }
            if self.alive && c.m.b.dot(c.up) > self.p.restart_up {
                (self.back, self.ground, self.touched_t, self.touched) = (true, false, 0.0, false);
                self.kids.on.retain(|n| *n != "bodyBalance");
                self.kids.start("bodyBalance", "bodyBalance start=true".into());
                self.arm_t = [0.0; 2];
                self.leg_t = [0.0; 2];
                (self.spine_t, self.head_t) = (0.0, 0.0);
                self.ran(4, "restartUpness: back on his feet");
            }
        }
        // the hands' feedback (0x2a8c800), its hash uncracked: side and code in the log. In the code the left's 9 reads
        // the shared chest flag and the right's 9 a flag nothing sets
        let codes = [
            if self.arm[0] != 2 { 0 } else if self.held_feet[0] { 8 } else if self.held_chest { 9 } else { 0 },
            if self.arm[1] != 2 { 0 } else if self.held_feet[1] { 8 } else { 0 },
        ];
        for i in 0..2 {
            if self.code[i] != codes[i] {
                self.code[i] = codes[i];
                self.kids.out.push(format!("#say yanked hand {i} {}", codes[i]));
            }
        }
        let fr: Vec<(usize, f32)> = (0..21).filter(|&i| self.fric[i] != 1.0).map(|i| (rp(i), self.fric[i])).collect();
        c.out.part_friction.extend(fr);
        self.scale.apply(c);
        self.first = false;
        self.yank += dt;
        self.clock += dt;
    }

    /// The bolas's spinning couple (0x2a7de10): alternating impulses about up x bolasDir while the clock is under
    /// bolasTime. Ours: `ctx+0x3540->+0x522c` (a physics step flag) is taken as set.
    fn bolas(&mut self, c: &mut Ctx, dt: f32) {
        if !self.p.bolas_forces || self.clock >= self.p.bolas_time {
            return;
        }
        if self.first {
            self.bolas_t = self.p.bolas_first;
        } else if self.bolas_t <= 0.0 {
            let t = c.vel.length().clamp(1.0, 2.0) - 1.0;
            let [feet, slow, neck, fast] = self.p.bolas_k;
            let f = match self.p.lassoed {
                0 => neck,
                2 => feet,
                _ => slow + (fast - slow) * t,
            };
            let mut s = self.bolas_sign;
            if self.bolas_once {
                s *= 0.5;
                self.bolas_once = false;
            }
            self.bolas_sign = -self.bolas_sign;
            let j = c.up.cross(self.p.bolas_dir.normalize_or(Vec3::X)).normalize_or_zero() * (f * s / 30.0);
            let list: Vec<(usize, f32)> = match self.p.lassoed {
                0 => vec![(0, 1.0), (4, -0.5), (6, -0.5)],
                2 => vec![(20, 1.0), (10, -1.0)],
                _ => vec![(0, 1.0), (1, -0.25), (2, -0.25), (10, -0.5), (20, self.p.bolas_share)],
            };
            for (k, x) in list {
                let kk = rp(k);
                c.out.impulse.push((kk, pos(c, kk), j * x));
            }
            self.bolas_t = self.p.bolas_period;
            self.ran(5, "bolas: a spin impulse");
        }
        self.bolas_t -= dt;
    }

    /// The on-feet tick (0x2aa9fa0).
    fn feet(&mut self, c: &mut Ctx, ty: i32, dt: f32) {
        let up = c.up;
        let was = self.arm;
        if self.back {
            self.kids.stop("armsWindmillAdaptive");
            self.pedal_set(c, [false; 2]);
            self.back = false;
        }
        // the yank: moving over 0.5 m/s along the ground starts its clock (ctx+0x4804's impulse flag: no source);
        // moving along his back at that the feet in swing kick out
        let h = flat(c.vel, up);
        if h.length() > 0.5 && self.yank >= 100000.0 {
            self.yank = 0.0;
            note(format_args!("yanked: the yank at {:.2} s, {:.2} m/s", self.clock, h.length()));
        }
        let stepping = c.dbm.map_or(0, |d| d.stepping);
        if -0.066664 < self.yank && self.yank < 0.3 && h.length() > 0.5 && h.normalize().dot(c.m.c) < -0.4 {
            if stepping < 2 {
                c.ch.eff[ANKLE()[1]].desired[0] = -2.9;
            }
            if stepping == 0 || stepping == 2 {
                c.ch.eff[ANKLE()[0]].desired[0] = -2.9;
            }
            self.ran(6, "the yank from behind: feet kick out");
        }
        // the arms: no action (5) or a grab at the rope (2), the first delay under 0.3 s
        if self.first {
            for i in 0..2 {
                self.arm_t[i] = 0.30100003 * u(c) - 0.001;
                self.arm_raw[i] = 5;
            }
        }
        for i in 0..2 {
            if self.arm_t[i] < 0.0 {
                let w = self.p.arm_feet;
                self.arm_raw[i] = if ty == 0 { 5 } else { [5, 2][pick(c, &[w.x, w.y])] };
                let (lo, hi) = self.p.state_time;
                self.arm_t[i] = ur(c, lo, hi);
                if self.first {
                    self.arm_t[i] = ur(c, 0.1, lo.max(0.1));
                }
                note(format_args!("yanked: arm {i} picks {}", self.arm_raw[i]));
            }
        }
        self.arm = self.arm_raw;
        if ty == 2 && self.p.right_grabs && self.grab_latch && self.arm_raw[0] != 2 && self.arm_raw[1] != 2 {
            self.arm[1] = 2;
            self.ran(7, "feet lassoed: the right hand grabs again");
        }
        for i in 0..2 {
            if (2..=4).contains(&self.arm[i]) && c.ch.busy[i] {
                self.arm[i] = 0;
            }
        }
        self.grab_latch |= self.arm_raw.contains(&2);
        // the hip pitch from the turn's noise (owner 4, at once)
        c.out.dbm_crouch.push(Crouch::Hip(4, -self.hip.clamp(-2.0, 2.0), 100000.0));
        // foot friction: round the neck, 0.7 at least moving fast, else neckFootFriction
        self.fric_set(0xa0, self.p.foot_friction);
        if ty == 0 {
            let x = if h.length_squared() >= 6.0 { self.p.foot_friction.max(0.7) } else { self.p.neck_foot_friction };
            self.fric_set(0xa0, x);
        }
        // the leans: the hips by the rope (slot 4) and the body (slot 6)
        let d = flat(self.p.rope - c.com, up).normalize_or(Vec3::X);
        let (lo, hi) = self.p.lean_hips;
        c.out.hip_lean = Some(d * (lo + noise01(self.phase, 2.0 * self.clock, self.seed - 0.5) * (hi - lo)));
        let (lo, hi) = self.p.lean;
        let a = if c.vel.dot(d) < -0.4 { lo } else { lo + noise01(2.0 * self.clock, self.phase, self.seed + 0.5) * (hi - lo) };
        c.out.lean = Some((LeanMode::Dir(d), a, false));
        // the legs soften with time and with each step (not while staggerFall runs)
        if !c.extras_on.contains(&"staggerFall") {
            let (a, b) = self.p.legs;
            let (end, ramp) = (self.p.time_end, self.p.ramp_to);
            let base = if self.clock >= end + ramp { b } else if self.clock >= end { a + (b - a) * (self.clock - end) / ramp } else { a };
            let done = c.dbm.map_or(0, |d| d.steps) as i32 - self.steps0 as i32;
            let n = done - self.p.steps_till;
            let less = if done > self.p.steps_till { (n - 1) as f32 * self.p.per_step } else { 0.0 };
            let more = if done >= self.p.steps_till { n as f32 * self.p.per_step } else { less };
            let (mut left, mut right) = (base - less, base - less);
            // the stance leg's by one more: the left stepping lowers bal+0x65c, the right's (0x2aaeca8: +0x658 limb 2)
            if stepping == 1 {
                right = base - more;
            }
            if stepping == 2 {
                left = base - more;
            }
            c.out.dbm_leg_each = Some([left.max(5.0), right.max(5.0)]);
            if n > 0 {
                self.ran(8, &format!("per step: the legs soften, left {left:.1} right {right:.1} after {done} steps, stepping {stepping}"));
            }
            // the arm and spine gains go straight to the end values at the first tick here, so nothing reads the start
            // values unless staggerFall runs
            self.gains = [self.p.arm.0, self.p.arm.1, self.p.spine.0, self.p.spine.1];
            set_k(c, &joints(0x709), self.gains[2], self.gains[3], None);
        }
        // loose at the jerk
        let ls = self.p.loose[2];
        if self.yank <= self.p.loose[0] {
            let m = if self.p.stable_neck { 0xfdf08 } else { 0x1fff09 };
            self.scale.ask(&joints(m), [1.0, ls, ls], true);
            self.ran(9, "loose at the jerk: torso");
        }
        if self.yank <= self.p.loose[1] {
            self.scale.ask(&joints(6), [1.0, ls, ls], true);
            self.ran(10, "loose at the jerk: legs");
        }
        // and the spine's lean limits raised for the first of it
        if self.yank <= self.p.spine_limit[2] {
            let [l1, l2, _] = self.p.spine_limit;
            for k in [3, 8, 9, 10] {
                c.out.lim_floor.push((rp(k), [l1.max(0.0), l2.max(0.0), 0.0]));
            }
            self.ran(11, "spine lean limits raised");
        }
        self.turn(c, stepping, dt);
        self.grab(c, ty);
        // the head's pick
        let now = ty < 2 && self.arm.contains(&2);
        if now && !was.contains(&2) || !now && self.head == 2 {
            self.head_t = -1.0;
        }
        if self.head_t < 0.0 {
            let w = self.p.head_prob;
            self.head_raw = [1, 4, 0][pick(c, &[w.x, w.y, w.z])];
            let (lo, hi) = self.p.state_time;
            self.head_t = ur(c, lo, hi);
            if u(c) < self.p.head_grab && now {
                self.head_raw = 2;
            }
            note(format_args!("yanked: head picks {}", self.head_raw));
        }
        self.head = self.head_raw;
        if self.head == 4 && stepping == 0 {
            self.head = 1;
        }
        if !self.p.use_head && self.head == 1 {
            self.head = 0;
        }
        self.head_drive(c, dt);
    }

    /// The turn decision, hip pitch and spine (0x2a7a3b8).
    fn turn(&mut self, c: &mut Ctx, stepping: u32, dt: f32) {
        let up = c.up;
        self.heading(c, false);
        // the rope's bearing in the COM frame: its own unwrap, added to the pelvis heading's
        let d = flat(self.p.rope - pos(c, spine_part(4)), up).normalize_or(Vec3::X);
        let (r2, r0) = (flat(c.m.c, up).normalize_or(Vec3::X), flat(c.m.a, up).normalize_or(Vec3::X));
        let raw = d.dot(r0).atan2(-d.dot(r2));
        if self.first {
            self.unwrap = 0.0;
        } else {
            let jump = self.raw_was - raw;
            if jump < -2.1991148 {
                self.unwrap -= TAU;
            }
            if jump > 2.1991148 {
                self.unwrap += TAU;
            }
        }
        self.raw_was = raw;
        let psi = raw + self.unwrap + self.heading_now;
        self.turn_timer -= dt;
        self.turn_hold -= dt;
        // the byte at RVA 0x3cc7e9c reads 0: a run-out timer counts only between steps
        if self.turn_timer < 0.0 && stepping == 0 || stepping != 0 && self.last_step != stepping {
            self.last_step = stepping;
            self.turn_timer = ur(c, 0.6, 2.0);
            if self.turn_hold < 0.0 || self.turn_side && psi > self.turn_thresh || !self.turn_side && psi < -self.turn_thresh {
                self.turn_side = !self.turn_side;
                self.turn_thresh = ur(c, self.p.turn.0, self.p.turn.1);
                self.turn_hold = ur(c, 10000.0, 10000.1);
                note(format_args!("yanked: turn side {} past {:.2}, psi {psi:.2}", self.turn_side, self.turn_thresh));
            }
            self.turn_go = u(c) < 0.65;
            for k in 0..3 {
                if self.flip_p < u(c) {
                    self.signs[k] = -self.signs[k];
                    self.flip_p += 0.05;
                }
            }
        }
        let (lo, hi) = self.p.turn;
        // asked for when +0x3f3 says so or the rope is past the limit; a request stays until the next (or his stop)
        if self.turn_thresh > 0.01 && lo > 0.0 && hi > 0.0 && (self.turn_go || !(-hi <= psi && psi <= hi)) {
            let s = if self.turn_go { if self.turn_side { 1.0 } else { -1.0 } } else if psi <= hi { 1.0 } else { -1.0 };
            let m = c.b.part(0).m;
            c.out.dbm_turn = Some((m.b * s + m.c * 0.5).normalize_or_zero() * 0.5);
            // the request block's +0x618 0.2 L: the "other foot farther" margin stays its usual while turning (its +0x614
            // -1 gates a foot-axis test Delirium's balancer doesn't turn on)
            c.out.dbm_turn_margin = Some(true);
            self.ran(45, "the turn request");
        }
        let n = (2.0 * noise01(self.phase, 2.0 * self.clock, self.seed - 0.5) - 1.0) * self.signs[0];
        // [C] the code's order, the note's was the other way round: negative takes forward's
        self.hip = n * if n < 0.0 { self.p.hip_pitch.0 } else { self.p.hip_pitch.1 };
        let b = self.p.bend;
        let n1 = noise01(2.0 * self.clock, self.phase, self.seed - 0.5);
        let n2 = noise01(self.phase2, self.seed + 0.5, 2.0 * self.clock);
        let l1 = (-2.0 * b + 4.0 * b * n1) * self.signs[1];
        let l2 = (-2.0 * b + 4.0 * b * n2) * self.signs[2];
        spine_writer(c, l1, l2, l2);
    }

    /// The rope grab (0x2a78ac0): each grabbing hand to its grab point in grabPart's frame, by the hand placement
    /// (0x2a4733c), clawing about it; holding once within 0.2 m. Round the chest with grabPart on his back the point
    /// sits on the surface of the part (half its depth, ours the capsule's radius, along -z). The mirror rig's swap of
    /// grabposl and grabposr (ctx+0x3514 = 0x11) isn't a rig here.
    fn grab(&mut self, c: &mut Ctx, ty: i32) {
        let f = self.p.claw_freq;
        let (x, y, z) = (noise01(f.x * self.clock, self.phase, self.seed - 0.5), noise01(self.phase2, self.seed + 0.5, f.y * self.clock),
                         noise01(self.seed + 0.9, f.z * self.clock, self.phase2));
        let n = Vec3::new(2.0 * x - 1.0, 2.0 * y - 1.0, 2.0 * z - if ty == 1 { 2.0 } else { 1.0 }) * self.p.claw_amp;
        let [l, r] = [self.arm[0] == 2, self.arm[1] == 2];
        match (l, r) {
            (true, false) => self.grab_first = 1,
            (false, true) => self.grab_first = 2,
            (false, false) => self.grab_first = 0,
            (true, true) if self.grab_first == 0 => self.grab_first = 1,
            _ => {}
        }
        let s = if self.grab_first == 1 { -1.0 } else { 1.0 };
        let off = if l && r { self.p.hands_off * s } else { Vec3::ZERO };
        let part = rp(self.p.grab_part);
        let m = c.b.part(part).m;
        let on_body = ty == 1 && (0x709u32 >> self.p.grab_part & 1) != 0;
        let mut reached = [false; 2];
        self.held_chest = false;
        self.held_feet = [false; 2];
        for i in 0..2 {
            if self.arm[i] != 2 {
                self.pose[i] = None;
                continue;
            }
            let side = if i == 0 { -1.0 } else { 1.0 };
            let mut g = self.p.grab_pos[i] + n + off * side;
            if on_body {
                g.z = n.z - c.b.build.parts[part].radius;
            }
            let to = m.d + m.a * g.x + m.b * g.y + m.c * g.z;
            let gn = self.p.grab_normal[i];
            let nw = (m.a * gn.x + m.b * gn.y + m.c * gn.z).normalize_or(Vec3::Z);
            let mut pose = self.pose[i];
            crate::extras_rdr2::hand_place(c, i, to, nw, &mut pose, None, 0.0);
            self.pose[i] = pose;
            let (k, d) = self.p.ground[1];
            set_k(c, &joints(ARM_BITS[i]), k, d, Some(1.0));
            self.scale.ask(&joints(ARM_BITS[i]), [1.0; 3], false);
            if self.p.bolas {
                set_k(c, &joints([0x10000, 0x80000][i]), k, d, Some(4.0));
            }
            reached[i] = pos(c, limb(i).p[3]).distance(to) < 0.2;
            self.held_chest = ty == 1 && reached[i];
            self.held_feet[i] = ty == 2 && reached[i];
            self.ran(12 + i as u32, "a hand grabs the rope");
            if reached[i] {
                self.ran(14 + i as u32, "a hand holds the rope");
            }
        }
        // the writhe reaches an arm only round the feet, with that hand on the rope
        let mask = match (self.held_feet[0], self.held_feet[1]) {
            (true, true) => "ua",
            (true, false) => "ul",
            (false, true) => "ur",
            _ => "lb",
        };
        if mask != self.writhe_mask && self.kids.on.contains(&"bodyWrithe") {
            self.writhe_mask = mask;
            self.kids.out.push(format!("bodyWrithe mask={mask}"));
            self.ran(16, "the writhe's mask follows the holding hand");
        }
    }

    /// The head driver (0x2a78448).
    fn head_drive(&mut self, c: &mut Ctx, dt: f32) {
        let neck: Vec<usize> = joints(0x102000);
        let (s0, sd) = (self.gains[2], self.gains[3]);
        let m = c.b.part(rp(self.p.grab_part)).m;
        let at = match self.head {
            2 => {
                let g = self.p.grab_pos[1];
                Some(m.d + m.a * g.x + m.b * g.y + m.c * g.z)
            }
            1 => Some(if self.p.head_pos == Vec3::ZERO { self.p.rope } else { self.p.head_pos }),
            4 => Some(pos(c, spine_part(6)) + c.vel.normalize_or(-c.m.c)),
            _ => None,
        };
        if self.head == 6 {
            set_k(c, &neck, self.p.neck_k, 1.0, Some(1.5));
            self.look.to(&mut self.kids.out, self.p.rope, 13.0, 1.0, true);
            // the neck's and head's lean1 limit raised (0x2a4e990, the literal's register [P] the same 1.5)
            c.out.lim_floor.push((rp(13), [1.5, 0.0, 0.0]));
            c.out.lim_floor.push((rp(20), [1.5, 0.0, 0.0]));
            self.ran(17, "head state 6: looks at the rope");
        } else if let Some(at) = at {
            let (k, d, ms) = if self.ground { (1.0 + s0, sd, 0.5) } else { (11.0, 1.0, 1.0) };
            self.look.to(&mut self.kids.out, at, k, d, false);
            for &j in &neck {
                c.ch.eff[j].muscle_stiffness = ms;
            }
            self.ran(18 + self.head as u32, "head state 1, 2 or 4: looks");
        } else {
            set_k(c, &neck, 1.0 + s0, sd, None);
            for &j in &neck {
                c.ch.eff[j].desired = [0.0; 3];
            }
            self.look.stop(&mut self.kids.out);
        }
        if self.head == 5 {
            // a long step (over 0.052 s) latches the stiffer neck muscle; under 0.048 s lets it go
            if dt > 0.052 {
                self.slow_latch = true;
            }
            if dt < 0.048 {
                self.slow_latch = false;
            }
            let ms = if self.slow_latch { 2.0 } else { 1.0 };
            if self.slow_latch {
                self.scale.ask(&neck, [1.0; 3], false);
            }
            self.look.stop(&mut self.kids.out);
            set_k(c, &neck, self.p.neck_k, 1.0, Some(ms));
            // the neck held to the spine limb's pose (0x2a3dff8), its amount ramping at 0.55/s with a limp rate
            let want = if self.limp { 0.0 } else { 1.0 };
            let mut a = want;
            if self.p.limp_rate > 0.0 && self.neck_pose < want {
                a = (self.neck_pose + 0.55 * dt).min(want);
            }
            self.neck_pose = a;
            let n = c.b.part(spine_part(5)).m;
            let f = n.a.dot(c.up);
            let (l1, l2) = (-n.c.dot(c.up) * a, -f.abs() * f * a);
            for &j in NECK().iter() {
                let e = &mut c.ch.eff[j].desired;
                (e[0], e[1], e[2]) = (l1, l2, 0.0);
            }
            self.ran(23, "head state 5: the stable neck");
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
            c.out.pedal_reverse = Some([0, 1].map(|i| self.leg[i] == 3));
        }
    }

    /// Contacts with anything but himself in a mask, by part: (point, normal).
    fn touching(c: &Ctx, bits: u32) -> Vec<(Vec3, Vec3)> {
        parts(bits).into_iter().filter_map(|k| c.b.doll.hit_out.get(k).copied().flatten()).map(|h| (h.0, h.1)).collect()
    }

    /// The ground tick (0x2a85930).
    fn down(&mut self, c: &mut Ctx, ty: i32, dt: f32) {
        let up = c.up;
        let neck = ty == 0;
        let speed = c.vel.length();
        let st = self.p.struggle;
        // each tick: the arm's clavicle and shoulder take no gravity, the elbow no muscle, the wrist straight
        for i in 0..2 {
            let l = limb(i);
            c.ch.eff[l.e[0]].oppose_gravity = 0.0;
            c.ch.eff[l.e[1]].oppose_gravity = 0.0;
            c.ch.eff[l.e[2]].muscle_stiffness = 0.0;
            c.ch.eff[l.e[3]].desired = [0.0; 3];
        }
        // the rope's jerk (ctx+0x100c, a kinetic measure past 50, isn't kept here: the rope's speed alone)
        let d = (pos(c, spine_part(4)) - self.p.rope).normalize_or_zero();
        let vr = d.dot(c.vel);
        if !self.first && vr > 5.0 {
            self.jerks = 2;
            note(format_args!("yanked: the rope's jerk, {vr:.1} m/s"));
        }
        let jerk = self.jerks > 0 && vr > 0.0;
        if self.jerks > 0 {
            self.jerks -= 1;
        }
        // landing: softened, ramping back up
        let r = (self.land.clamp(0.0, 0.6) * 1.6666666 + 0.1).min(1.0);
        if !jerk && r < 0.999999 {
            self.scale.ask(&joints(0x6df5f), [r.max(0.7), r, r], true);
            self.scale.ask(&joints(0xfd800), [r.max(0.7); 3], true);
            self.ran(24, "landing: the soft ramp");
        }
        let legs = Self::touching(c, 0xf7).iter().any(|h| h.1.dot(up) > 0.77);
        self.latched |= legs;
        let all = Self::touching(c, 0x1fffff);
        // [G] ctx+0x4808, written only by a game setter, read as his time off everything
        self.air_t = if all.is_empty() { self.air_t + dt } else { 0.0 };
        let air = self.air_t > 0.75 && !self.latched;
        if self.latched {
            self.latched_t += dt;
            let r2 = (self.land.clamp(0.0, 0.1) * 10.0 + 0.1).min(1.0);
            self.land += dt;
            if !jerk && r2 < 0.999999 {
                self.scale.ask(&joints(0x102000), [r2; 3], true);
            }
            if !legs && c.vel.dot(up) < 0.0 {
                self.airborne += dt;
            }
        }
        if self.p.animal {
            self.scale.ask(&joints(0x102000), [1.0; 3], false);
        }
        if !air {
            if self.airborne > 0.3 {
                (self.airborne, self.latched_t, self.latched, self.land) = (0.0, 0.0, false, -0.2);
                self.ran(25, "off the ground 0.3 s: the landing ramp again");
            }
            if !jerk && speed < 1.0 && Self::touching(c, 0x309).is_empty() {
                if r > 0.2 {
                    self.scale.ask(&joints(0x10275f), [0.2; 3], true);
                }
                self.scale.ask(&joints(0x102000), [0.2; 3], true);
                self.ran(26, "slow with his back off the ground: limp");
            }
        }
        if st > 0.4 {
            self.scale.ask(&joints(0x1fffff), [1.0; 3], false);
        }
        // the contact normals: each arm's, and the running ground normal from up and all of them
        let sum = |h: &[(Vec3, Vec3)]| h.iter().fold(Vec3::ZERO, |a, x| a + x.1);
        let gn = (up + sum(&all)).normalize_or(up);
        let (nl0, nr0) = (sum(&Self::touching(c, ARM_BITS[0])), sum(&Self::touching(c, ARM_BITS[1])));
        let nl = if nl0 != Vec3::ZERO { nl0 } else if nr0 != Vec3::ZERO { nr0 } else { gn }.normalize_or(up);
        let nr = if nr0 != Vec3::ZERO { nr0 } else if nl0 != Vec3::ZERO { nl0 } else { gn }.normalize_or(up);
        if !all.is_empty() {
            self.ran(46, &format!("the ground normal {gn:.2} from {} contacts, the arms' {nl:.2} {nr:.2}", all.len()));
        }
        // frictionsim: a constant sliding drag on the end the rope doesn't hold
        let upper = if self.p.stirrup && c.extras_on.contains(&"rdr2Sit") { 0x1fff08 } else { 0x1fff09 };
        let on_upper = !Self::touching(c, upper).is_empty();
        self.touched |= on_upper;
        let k = if on_upper { self.p.fsim.0 } else if self.touched_t > 0.5 { self.p.fsim.1 } else { 0.0 };
        if k > 0.001 {
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
            self.ran(27, "frictionsim: the drag");
        }
        // bumpsim: an impulse along the ground's normal k m / (60 m_mean) at a part, in turn over his body
        let k = self.p.bump.0;
        let p = if dt > 0.01666 { self.p.bump.1 * dt.min(0.0666) * 60.0 } else { self.p.bump.1 };
        if speed > 1.0 && u(c) < p {
            self.bump_mode += if neck { -1 } else { 1 };
            self.bump_mode = if self.bump_mode >= 3 { 0 } else if self.bump_mode < 0 { 2 } else { self.bump_mode };
            let bit = |c: &mut Ctx| c.ch.random_bits() & 1 != 0;
            let mut at = Vec3::ZERO;
            let mut kk = k;
            let (first, mut second) = (match self.bump_mode {
                0 => {
                    let mut f = if bit(c) { 20 } else { 13 };
                    if bit(c) {
                        f = if f != 13 { 12 } else { 11 };
                    }
                    f
                }
                1 => {
                    at = c.m.a * ((u(c) - 0.5) * 0.2);
                    kk = k * 4.0;
                    [3, 8, 9, 10][(c.ch.random_bits() & 3) as usize]
                }
                _ => {
                    let pair = if bit(c) { [2, 6] } else { [1, 4] };
                    pair[bit(c) as usize]
                }
            }, 0usize);
            if self.bump_mode == 0 && neck || self.bump_mode == 1 && !neck {
                let side = if bit(c) { [17, 18] } else { [14, 15] };
                second = side[bit(c) as usize];
            }
            let mean = c.b.doll.mass() / 21.0;
            let f = rp(first);
            c.out.impulse.push((f, pos(c, f) + at, gn * (kk * c.b.doll.part_mass(f) / (60.0 * mean))));
            if second != 0 {
                let s = rp(second);
                c.out.impulse.push((s, pos(c, s), gn * (k * 0.25 * c.b.doll.part_mass(s) / (60.0 * mean))));
            }
            note(format_args!("yanked: a bump, mode {} part {first} second {second}", self.bump_mode));
        }
        // the arms' picks: reach (0), windmill (1), the rope (2)
        if self.first {
            for i in 0..2 {
                self.arm_t[i] = 0.30100003 * u(c) - 0.001;
                let w = self.p.feet_prob;
                self.arm_raw[i] = pick(c, &[w.x, w.y]) as u8;
            }
        }
        for i in 0..2 {
            if self.arm_t[i] < 0.0 {
                self.arm_raw[i] = if ty == 0 {
                    let w = self.p.feet_prob;
                    pick(c, &[w.x, w.y]) as u8
                } else {
                    let w = self.p.arm_down;
                    pick(c, &[w.x, w.y, w.z]) as u8
                };
                self.arm_t[i] = ur(c, self.p.state_time.0, self.p.state_time.1);
                note(format_args!("yanked: down, arm {i} picks {}", self.arm_raw[i]));
            }
        }
        self.arm = self.arm_raw;
        // the face flags, by noise off the ground
        let mut back = gn.dot(c.m.c);
        if air {
            back = noise01(self.phase, 2.0 * self.clock, self.seed + 0.5) * 1.4 - 0.4;
        }
        let (face_down, face_up) = (back <= 0.0, back > 0.0);
        let mut side = -gn.dot(c.m.a);
        if air {
            side = 2.0 * noise01(2.0 * self.clock, self.phase, self.seed - 0.5) - 1.0;
        }
        // limp, and the "yanked" feedback once timetodie passes
        let alive = self.ground_t < self.p.die;
        let dying = self.p.limp_rate > 0.0 && noise01(self.clock * self.p.limp_rate, self.phase, self.seed - 0.5) > 0.7
            || 0.0 < self.since_struggle && self.since_struggle < 0.3
            || self.p.bolas && self.touched_t < 0.5;
        if !alive && self.alive {
            c.out.send(SUCCESS(), "yanked", 0.0);
            note(format_args!("yanked: timetodie passed at {:.2} s down", self.ground_t));
        }
        self.alive = alive;
        self.limp = !alive || dying;
        let limp = self.limp;
        if limp {
            self.kids.stop("bodyWrithe");
            self.ran(28, "limp: the writhe stops");
        } else {
            self.kids.start("bodyWrithe", format!("{WRITHE} mask={}", self.writhe_mask));
        }
        for i in 0..2 {
            if self.arm_raw[i] == 1 && (back < 0.0 || speed <= 0.8) && !air {
                self.arm[i] = 0;
            }
        }
        if ty == 2 && self.p.right_grabs && self.grab_latch && self.arm_raw[0] != 2 && self.arm_raw[1] != 2 {
            self.arm[1] = 2;
        }
        self.grab_latch |= self.arm_raw.contains(&2);
        if limp {
            self.arm = [0; 2];
        }
        if st > 0.4 && ty != 0 {
            self.arm = [2; 2];
            self.grab_latch = true;
        }
        for i in 0..2 {
            if (2..=4).contains(&self.arm[i]) && c.ch.busy[i] {
                self.arm[i] = 0;
            }
        }
        // windmill: each arm in state 1, the wrist along that side's ground normal (0x2a75cb0)
        let wind = [self.arm[0] == 1, self.arm[1] == 1];
        if wind != [false; 2] {
            self.kids.start("armsWindmillAdaptive", WINDMILL.into());
            let mask = match wind { [true, true] => "ub", [true, false] => "ul", _ => "ur" };
            if mask != self.wind_mask {
                self.wind_mask = mask;
                self.kids.out.push(format!("armsWindmillAdaptive mask={mask}"));
            }
            if self.p.limp_rate > 0.0 && self.arm_k < 13.0 {
                self.arm_k = (self.arm_k + dt).min(13.0);
                self.kids.out.push(format!("armsWindmillAdaptive armStiffness={}", self.arm_k));
            } else {
                self.arm_k = 13.0;
            }
            for i in 0..2 {
                if wind[i] {
                    let (l, s) = (limb(i), if i == 0 { 1.0 } else { -1.0 });
                    let n = if i == 0 { nl } else { nr };
                    wrist_ground(c, l, n, s);
                }
            }
            self.ran(29, "windmill: the arm, its wrist to the ground");
        } else {
            self.kids.stop("armsWindmillAdaptive");
            self.wind_mask = "";
        }
        // the reach (0): the hand to a noise-driven point about the shoulder
        let mut hand_fric = -1.0;
        if self.arm.contains(&0) {
            hand_fric = self.claw(c, neck, face_down, face_up, limp, speed, dt);
        }
        // the legs: kicks (0), pedalling (2), back-pedalling (3). Dragged by a stirrup, a foot out of its stirrup pedals
        let free = match (self.p.stirrup, self.stirrups) {
            (true, Some(alive)) => [!alive[0], !alive[1]],
            _ => [false; 2],
        };
        self.free = free;
        if self.p.stirrup {
            self.ran(48, &format!("dragged by a stirrup: sit's stirrups alive {:?}", self.stirrups));
        }
        for i in 0..2 {
            if self.leg_t[i] < 0.0 {
                let mut r = u(c);
                let mut t = ty;
                if free[i] {
                    self.ran(47, "a foot out of its stirrup: it pedals");
                    r *= 0.5;
                    t = 2;
                }
                self.leg_raw[i] = if t != 0 && r < 0.25 { 3 } else if t != 0 && r < 0.5 { 2 } else { 0 };
                self.leg_t[i] = 0.7 * u(c) + 0.3;
                note(format_args!("yanked: down, leg {i} picks {}", self.leg_raw[i]));
            }
        }
        self.leg = self.leg_raw;
        // the second leg pedals as the first
        if self.leg_raw[0] == 2 && self.leg_raw[1] == 3 {
            self.leg[1] = 2;
        } else if self.leg_raw[0] == 3 && self.leg_raw[1] == 2 {
            self.leg[1] = 3;
        }
        if free == [false; 2] && (face_up || limp || speed <= 0.8) && (limp || !self.p.pedal_slow || !face_down) && (!air || limp) {
            self.leg = [0; 2];
        }
        if self.leg.contains(&0) {
            self.kicks(c, ty, side, limp, face_down, face_up, speed);
        }
        self.pedal_set(c, [0, 1].map(|i| self.leg[i] >= 2));
        if self.pedal != [false; 2] {
            self.ran(30, "pedalLegs: running, its reverse flags set");
        }
        let (lk, ld) = self.p.ground[0];
        let lk = if limp { if neck { 8.0 } else { 4.0 } } else { lk };
        set_k(c, &joints(0xf6), lk, ld, None);
        set_k(c, &joints(0x50), lk, ld, None);
        // the carts: 5 stiff on the leg they hold
        if self.p.cart == 1 || self.p.cart == 3 {
            set_k(c, &joints(0x32), 5.0, 1.0, None);
            self.ran(31, "cart: left leg");
        }
        if self.p.cart == 2 || self.p.cart == 3 {
            set_k(c, &joints(0xc4), 5.0, 1.0, None);
            self.ran(32, "cart: right leg");
        }
        set_k(c, &joints(0xa0), lk, ld, Some(5.0));
        let holding = ty == 1 && self.arm[0] == 2 && self.held_chest;
        if !limp {
            self.roll(c, ty, face_down, face_up, holding, speed, dt);
        }
        self.friction(speed, hand_fric, neck);
        self.grab(c, ty);
        self.spine_tick(c, holding, face_down, face_up, speed, limp, neck);
        if self.head_t < 0.0 {
            self.head_raw = 5;
            self.head_t = ur(c, self.p.state_time.0, self.p.state_time.1);
        }
        if self.p.animal {
            self.head_raw = 6;
        }
        self.head = self.head_raw;
        self.head_drive(c, dt);
        if self.touched {
            self.touched_t += dt;
        }
        self.ground_t += dt;
    }

    /// The reach (state 0), both arms: returns the hand friction its case sets (-1: none).
    #[allow(clippy::too_many_arguments)]
    fn claw(&mut self, c: &mut Ctx, neck: bool, face_down: bool, face_up: bool, limp: bool, speed: f32, dt: f32) -> f32 {
        // r lo, r hi, angle lo, hi, rate, twist, hand friction, stiffness
        let a: [f32; 8] = match (neck, limp) {
            (true, true) if face_down => [0.7, 1.0, 1.0, 2.5, 0.9, -0.5, 0.0, 6.0],
            (true, true) if face_up => [0.6, 0.9, 1.9, 2.5, 1.2, -0.5, 0.3, 7.5],
            (false, false) if face_down => [0.65, 1.0, -2.0, -0.7, 2.5, 0.5, 0.0, 11.0],
            (true, false) if face_down => [0.65, 1.0, -2.0, -0.7, 2.5, 0.5, -1.0, 11.0],
            (false, false) if face_up => [0.5, 0.9, 1.8, 2.5, 3.0, -0.5, 0.0, 11.0],
            (true, false) if face_up => [0.5, 1.0, 1.0, 2.5, 2.5, -0.5, -1.0, 13.0],
            _ => [0.85, 0.85, -2.0, -2.0, 0.7, 0.5, 0.0, 7.0],
        };
        let f = if limp { speed.clamp(0.1, a[4] + 0.1) - 0.1 } else { a[4] * speed.clamp(0.1, 1.0) };
        let mut k = a[7];
        if self.p.limp_rate > 0.0 && self.arm_k < k {
            k = (self.arm_k + dt).min(k);
        }
        self.arm_k = k;
        let m = c.b.part(spine_part(3)).m;
        let (ax, bx) = (m.a, -m.b);
        let noises = [
            (noise01(f * self.clock, self.phase, self.seed - 0.5), noise01(self.phase2, self.seed + 0.5, f * self.clock)),
            (noise01(self.phase, f * self.clock, self.seed - 0.5), noise01(self.seed + 0.5, self.phase2, f * self.clock)),
        ];
        for i in 0..2 {
            if self.arm[i] != 0 {
                continue;
            }
            let len = arm_len(c, i);
            let r = len * (a[0] + (a[1] - a[0]) * noises[i].0);
            let th = a[2] + (a[3] - a[2]) * noises[i].1;
            let s = if i == 0 { -1.0 } else { 1.0 };
            let to = c.b.joint_pos(limb(i).e[1]) + (ax * th.cos() + bx * (s * th.sin())) * r;
            arm_ik(c, i, to, a[5], Vec3::ZERO);
            let l = limb(i);
            crate::nmcf::clav_overflow(c.ch, l.e[0], l.e[1]);
            set_k(c, &[l.e[0], l.e[1], l.e[2]], k, 0.5, None);
            self.ran(33, "the reach: the arm's IK");
        }
        a[6]
    }

    /// The kicks (state 0).
    #[allow(clippy::too_many_arguments)]
    fn kicks(&mut self, c: &mut Ctx, ty: i32, side: f32, limp: bool, face_down: bool, face_up: bool, speed: f32) {
        let a = if limp { 0.3 } else if face_down { self.p.leg_amp } else { 0.4 };
        let g = if limp { speed.clamp(0.0, 2.2) } else { speed.clamp(0.03, 1.0) * 2.2 };
        let w = c.m.b.dot(c.angvel).abs().min(1.0);
        let (lean2_a, lean2_b) = (0.2 - w * 0.2, 0.3 - w * 0.2);
        let spread = noise01(self.phase, 5.0 * self.clock, self.seed - 0.5) - 0.5;
        let wild = self.p.struggle > 0.1 && ty == 0;
        let mut first = (0.0, 0.0);
        let solve = |c: &Ctx, i: usize, h: f32| {
            let (hp, kn, an) = (c.b.joint_pos(HIP()[i]), c.b.joint_pos(KNEE()[i]), c.b.joint_pos(ANKLE()[i]));
            let (la, lb) = ((kn - hp).length(), (an - kn).length().max(0.01));
            (h.sin() * la / lb).clamp(-1.0, 1.0).asin()
        };
        for i in 0..2 {
            if self.leg[i] != 0 {
                continue;
            }
            let n = if i == 0 { noise01(g * self.clock, self.phase, self.seed - 0.5) } else { noise01(self.phase2, self.seed + 0.5, g * self.clock) };
            let mut h = n * a;
            let mut knee = -h - solve(c, i, h);
            if wild {
                let n1 = noise01(self.phase2, self.seed + 0.5, 10.0 * self.clock);
                let n2 = noise01(10.0 * self.clock, self.phase, self.seed - 0.5);
                (h, knee) = if i == 0 {
                    let h = 7.0 * n1 - 2.0;
                    (h, -h + (0.3 - n2 * 3.5 * 0.3))
                } else if self.leg[0] != 0 {
                    let h = 5.0 * n1;
                    (h, -h + (0.3 - n2 * 3.5 * 0.3))
                } else {
                    first
                };
                self.ran(34, "struggle: the wild kicks");
            }
            if i == 0 {
                first = (h, knee);
            }
            let mut hip = [h, -0.05, 0.0];
            let mut kn = knee;
            if wild {
                let x = if i == 0 { spread } else { -spread };
                (hip[1], hip[2]) = (x, x);
            } else if face_up {
                hip[0] = -0.1;
                kn -= 0.2;
            }
            if self.p.cart == 1 || self.p.cart == 2 || ty != 0 {
                let far = if i == 0 { side < -0.6 } else { side > 0.6 };
                let near = if i == 0 { side > 0.6 } else { side < -0.6 };
                if far {
                    hip = [0.4, lean2_a, 0.0];
                    kn = 0.0;
                    self.ran(35, "kicks: the leg on the rolled side spread");
                }
                if near {
                    hip = [-0.4, lean2_b, 0.0];
                    kn = 0.0;
                }
                // the first 0.3 s on the ground: a big kick
                if self.latched_t < 0.3 {
                    let h2 = n * 0.8 + 0.7;
                    kn = (h2 * -0.5 - solve(c, i, h2)) * 1.3;
                    hip = [h2, -0.05, 0.0];
                    self.ran(36, "kicks: landed, the big kick");
                }
            }
            c.ch.eff[HIP()[i]].desired = hip;
            c.ch.eff[KNEE()[i]].desired[0] = kn;
        }
    }

    /// The roll help (0x2a79310, with the choice of its two numbers at its call).
    #[allow(clippy::too_many_arguments)]
    fn roll(&mut self, c: &mut Ctx, ty: i32, face_down: bool, face_up: bool, holding: bool, speed: f32, dt: f32) {
        let (mut front, mut back) = if speed > self.p.moving { (self.p.roll[2], self.p.roll[3]) } else { (self.p.roll[0], self.p.roll[1]) };
        let grabbing = self.arm.contains(&2);
        if ty == 1 && (face_down && !holding || face_up && !grabbing) {
            (front, back) = (0.0, 0.0);
        }
        if self.p.struggle > 0.01 {
            (front, back) = if self.p.bolas { (2.0, 1.0) } else if ty == 0 { (3.2, 1.5) } else { (3.0, 3.0) };
        }
        let touching = !Self::touching(c, 0x709).is_empty();
        let along = (c.b.joint_pos(SPINE()[3]) - c.b.joint_pos(SPINE()[0])).normalize_or_zero();
        let (w0, w4) = (c.b.part(spine_part(0)).w, c.b.part(spine_part(4)).w);
        let w = along.dot(w4);
        if touching && w.abs() < 6.0 && along.dot(w0).abs() < 6.0 && along.dot(c.angvel).abs() < 6.0 && c.angvel.length() < 8.0 {
            let sgn = if w > 0.0 { 25.0 } else if w < 0.0 { -25.0 } else { 0.0 };
            let beat = ((self.roll_t * 1.5).sin() * 0.5 + 0.3).clamp(0.0, 1.0);
            let mut t = along * (sgn * beat);
            if speed > 1.0 && c.vel.normalize_or_zero().dot(c.m.a) > 0.5 {
                t *= 0.1;
            }
            let on_back = c.up.dot(c.m.c);
            t *= if on_back < 0.0 { back } else { front };
            match self.p.roll_type {
                1 => t *= on_back + 1.0,
                2 => t *= 1.0 - on_back,
                _ => {}
            }
            if t != Vec3::ZERO {
                for k in 1..5 {
                    c.out.torque(spine_part(k), t);
                }
                self.ran(37, "roll help: the torque");
            }
        }
        self.roll_t = if self.roll_t >= 0.0 { self.roll_t - dt } else { 2.5 * u(c) + 0.5 };
    }

    /// The friction (0x2aa7dec): moving, his body at groundFriction, the two ends of him by the rope's end and the
    /// other's, no friction at the pelvis, the hands' as asked; slow, 1 everywhere.
    fn friction(&mut self, speed: f32, hands: f32, neck: bool) {
        let g = self.p.ground_friction;
        if speed >= 1.0 {
            self.fric_set(0xfdfff, g);
            let (a, b) = (if self.p.rope_end > 0.0 { self.p.rope_end } else { 1.0 }, if self.p.other_end > 0.0 { self.p.other_end } else { 2.0 });
            let (legs, upper) = if neck { (a * g, b * g) } else { (b * g, a * g) };
            self.fric_set(0xf7, legs);
            self.fric_set(0x1fff09, upper);
            self.fric_set(1, 0.0);
            if hands > -0.001 {
                self.fric_set(0x90000, hands);
            }
            self.ran(38, &format!("friction: legs {legs:.2} upper {upper:.2} pelvis 0, hands {hands:.2}"));
        } else {
            self.fric_set(0x1fffff, 1.0);
        }
    }

    /// The spine's states (0 still, 2 and 3 writhes, 1 left to the windmill).
    #[allow(clippy::too_many_arguments)]
    fn spine_tick(&mut self, c: &mut Ctx, holding: bool, face_down: bool, face_up: bool, speed: f32, limp: bool, neck: bool) {
        if self.spine_t < 0.0 {
            let w = self.p.spine_prob;
            self.spine_raw = [0, 2, 3][pick(c, &[w.x, w.y, w.z])];
            self.spine_t = ur(c, self.p.state_time.0, self.p.state_time.1);
            note(format_args!("yanked: down, spine picks {}", self.spine_raw));
        }
        let mut s = self.spine_raw;
        if self.arm.contains(&1) {
            s = 1;
        }
        if holding && s == 0 && u(c) > 0.7 {
            s = 3;
        }
        if limp {
            s = 0;
        }
        let st = self.p.struggle;
        if st > 0.01 {
            s = 3;
        }
        if s == 2 || s == 3 {
            // +0x120 or +0x130; face up the negated +0x124 or +0x134, round the neck -+0x128; else face down +0x12c
            let [up, up_neck, down, big, big_up] = self.p.spine_a;
            let mut a = if s == 2 { self.p.spine_amp } else { big };
            if face_up {
                a = if s == 2 { -up } else { -big_up };
                if neck {
                    a = -up_neck;
                }
            }
            if !neck && face_down {
                a = down;
            }
            let lo = st.max(if holding { self.p.grab_spine_freq } else { 0.05 });
            let r = if speed >= lo { speed.min(1.0) } else { lo };
            let l1 = a * noise01(self.phase, self.p.spine_freq * r * self.clock, self.seed - 0.9);
            let sd = (2.0 * noise01(self.p.side.0 * r * self.clock, self.phase, self.seed - 0.9) - 1.0) * self.p.side.1;
            let k = if holding { 1.5 } else { 1.0 };
            spine_writer(c, k * l1, k * sd, k * sd);
            if st > 0.1 {
                // the struggle's second, larger writhe ([P] its rate: struggle times 3, plus 1)
                let lo = if face_down { -1.0 } else { -2.0 };
                let hi = if face_down { 3.0 } else { 2.0 };
                let f = st * 3.0 + 1.0;
                let amp = st + 1.0;
                let l1 = noise01(self.phase, f * self.clock, self.seed - 0.9) * (hi - lo) + lo;
                let l2 = noise01(f * self.clock, self.phase, self.seed - 0.9) * 2.0 * amp - amp;
                let sign = if l1 >= 0.0 { -1.0 } else { 1.0 };
                spine_writer(c, k * l1, k * l2, sign * k * l2);
                self.ran(39, "struggle: the second writhe");
            }
            self.ran(40 + s as u32, "the spine's writhe");
        } else if s == 0 {
            // still, bent to the side of a foot out of its stirrup
            let x = if self.free[1] { 0.5 } else if self.free[0] { -0.5 } else { 0.0 };
            spine_writer(c, 0.0, -x, x);
            self.ran(44, "the spine still");
        }
        let (sk, sd) = self.p.ground[2];
        set_k(c, &joints(0x709), sk, sd, None);
    }

    pub fn stop(&mut self, c: &mut Ctx) {
        if !self.started {
            return;
        }
        c.out.dbm_turn = Some(Vec3::ZERO);
        c.out.dbm_turn_margin = Some(false);
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

/// The wrist along a ground normal (0x2a758c0 as 0x2a75cb0 calls it: its frame flag set, limit 2.4): the twist that
/// turns the palm to `n`, clamped by how much of `n` lies across the wrist, the lean toward it split by that twist.
fn wrist_ground(c: &mut Ctx, l: crate::nmshot::Limb, n: Vec3, side: f32) {
    let w = c.b.eff3(l.e[3]);
    let (pa, pb, pc) = (w.m.a, w.m.c.cross(w.m.a), w.m.c);
    let b2 = pb * side;
    let (na, nb) = (n.dot(pa), -n.dot(b2));
    let len = (na * na + nb * nb).sqrt() + 1e-6;
    let max = len * 2.4;
    let tw = (na / len).atan2(nb / len).clamp(-max, max);
    let (s, co) = tw.sin_cos();
    let pp = pa * s - b2 * co;
    let phi = n.dot(pc).atan2(n.dot(pp));
    c.ch.eff[l.e[3]].desired = [-(phi * s), phi * co, tw];
}

/// RDR2's spine writer (0x2aa7be4): lean1, lean2 and twist spread over the four spine joints, a quarter each, within 9.9.
fn spine_writer(c: &mut Ctx, l1: f32, l2: f32, tw: f32) {
    let q = |x: f32| (x * 0.25).clamp(-9.9, 9.9);
    for &j in SPINE().iter() {
        c.ch.eff[j].desired = [q(l1), q(l2), q(tw)];
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
                crate::nmpa::share(c.ch, c.b.frames, limb(i).e[0], limb(i).e[1]);
                crate::extras_core_rdr2::trace_once("swim clavicle mode 2", || format!("arm {i}"));
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
                crate::nmpa::share(c.ch, c.b.frames, limb(i).e[0], limb(i).e[1]);
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
        let w = c.ch.core.cfg.water;
        let (surface, n) = if w.on { (w.point, w.normal.normalize_or_zero()) } else { (self.surface, self.normal.normalize_or_zero()) };
        let state = if (w.on || surface != Vec3::ZERO) && n != Vec3::ZERO {
            let under = |p: Vec3| (p - surface).dot(n) < 0.0;
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
                self.look.to(&mut self.kids.out, chest + c.up, 11.0, 1.0, true);
            }
            3 => {
                // the record's channel bit 2 (value 4) at +0x14: the twist
                for &j in WRIST().iter() {
                    c.ch.eff[j].desired[2] = 0.4;
                }
                let at = chest + c.vel.normalize_or_zero() + c.b.part(spine_part(4)).m.a * 0.5;
                self.look.to(&mut self.kids.out, at, 11.0, 1.0, true);
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
pub(crate) fn mask_parts(m: &str) -> Vec<usize> {
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
    release: Vec<(usize, Vec3, Vec3)>,
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
            hold: vec![], release: vec![], logged: 0, seen_incoming: false,
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
        // what mode 4 gives them when it ends: the pose buffers' velocities (0x2a1ec9c)
        self.release = self.hard.iter().filter_map(|&k| {
            let (t, p) = (target.get(k)?, self.prev.get(k)?);
            let (axis, angle) = (t.rotation * p.rotation.inverse()).to_axis_angle();
            let angle = if angle > std::f32::consts::PI { angle - std::f32::consts::TAU } else { angle };
            Some((k, (t.translation - p.translation) / dt.max(1e-4), axis * (angle / dt.max(1e-4))))
        }).collect();
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
        // mode 4, one step (0x2a1b1a4): the hard parts leave with the animation's velocities
        if c.ch.rdr2 && !self.release.is_empty() {
            let mut most = 0.0f32;


            for (k, v, w) in std::mem::take(&mut self.release) {
                let p = c.b.part(k);
                let dv = v - p.v;
                most = most.max(dv.length());
                c.out.impulse.push((k, c.b.doll.centre(k), dv * c.b.doll.part_mass(k)));
                let dw = w - p.w;
                let l = dw.length();
                if l > 1e-6 {
                    let i = c.b.doll.inertia(k, dw / l);
                    c.out.torque(k, dw * (i * 60.0));
                }
            }
            note(format_args!("keyframe: mode 4, the hard parts released, up to {most:.2} m/s changed"));
        }
        if self.started && c.ch.rdr2 {
            (c.ch.core.foot_hold, c.ch.core.zmp) = (true, true);
            note(format_args!("keyframe: deactivate, foot hold and ZMP on"));
        }
    }
}
