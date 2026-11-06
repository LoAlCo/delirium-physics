//! RDR2's core messages: the ones that set up the character instead of starting a behaviour (configureSoftLimit,
//! setFrictionScale, setMuscleStiffness, configureBullets, setCharacterStrength). Written from the notes
//! (`rdr2_params.md`, `rdr2_tasks.md`, `rdr2_bullet_impulse.md`). Nothing here runs without the RDR2 core; nothing in
//! the room sends them unless a set of the player's physicstasks file does (`rdr2 task`).
use crate::extras::{Extra, P};
use crate::msg::{Error, Kind, Message, Param};
use crate::nmrs::{Character, ELBOW, KNEE};
use crate::nmshot::Ctx;
use bevy_math::Vec3;

/// A message line can say start=false: on=false here, since an extra only starts with start=true.
const ON: Param = Param { name: "on", kind: Kind::Bool, used: true, doc: "false: RDR2's start=false for this record (true)" };

pub static MESSAGES: [Message; 7] = [
    Message { name: "setOpposeGravity", params: &[
        Param { name: "oppose", kind: Kind::Float, used: true, doc: "the joints' gravity opposition, this step's request (1)" },
        Param { name: "mask", kind: Kind::Str, used: true, doc: "the joints (fb)" },
    ] },
    Message { name: "configureSoftLimit", params: &[
        ON,
        Param { name: "fresh", kind: Kind::Bool, used: true, doc: "RDR2's start=true: its default stiffness and damping before these values (false)" },
        Param { name: "index", kind: Kind::Int, used: true, doc: "0, 1 the left and right elbow, 2, 3 the left and right knee (0)" },
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "k: the torque is -(k^2 e + 2 k d w) (15)" },
        Param { name: "damping", kind: Kind::Float, used: true, doc: "d (1)" },
        Param { name: "limitAngle", kind: Kind::Float, used: true, doc: "the soft limit, rad in from the joint's own end (0)" },
        Param { name: "approachDirection", kind: Kind::Int, used: true, doc: ">= 0: from the low end; < 0: from the high end (0)" },
        Param { name: "velocityScaled", kind: Kind::Bool, used: true, doc: "k scaled by the COM speed: k speed/4, speed under 4 m/s, at least 3 (false)" },
    ] },
    Message { name: "setFrictionScale", params: &[
        Param { name: "scale", kind: Kind::Float, used: true, doc: "the parts' friction multiplier (not sent: unchanged)" },
        Param { name: "mask", kind: Kind::Str, used: true, doc: "the parts (fb)" },
        Param { name: "globalMin", kind: Kind::Float, used: true, doc: "the clamp on a contact's friction, low (not sent: unchanged)" },
        Param { name: "globalMax", kind: Kind::Float, used: true, doc: "high (not sent: unchanged)" },
        Param { name: "clamp", kind: Kind::Bool, used: true, doc: "RDR2's a7180f98: the clamp is on (not sent: off; ctx+0x1058)" },
    ] },
    Message { name: "setMuscleStiffness", params: &[
        ON,
        Param { name: "muscleStiffness", kind: Kind::Float, used: true, doc: "the joints' muscle stiffness" },
        Param { name: "mask", kind: Kind::Str, used: true, doc: "the joints (fb)" },
    ] },
    Message { name: "setCharacterStrength", params: &[
        Param { name: "characterStrength", kind: Kind::Float, used: true, doc: "ctx+0x102c: scales the bullets' leak and torque share (1)" },
    ] },
    Message { name: "configureBullets", params: BULLET_PARAMS },
    Message { name: "setcharacterunderwater", params: &[
        Param { name: "Underwater", kind: Kind::Bool, used: true, doc: "ctx+0x10c4: the core's water is on" },
        Param { name: "surfacePoint", kind: Kind::Vec3, used: true, doc: "a point on the water (0 0 1)" },
        Param { name: "surfaceNormal", kind: Kind::Vec3, used: true, doc: "the water's up" },
        Param { name: "viscosity", kind: Kind::Float, used: true, doc: "a part under drags by this x its speed squared (0)" },
        Param { name: "stroke", kind: Kind::Float, used: true, doc: "a hand or foot under, moving back along the axis, pushes by this x its speed (0)" },
        Param { name: "?704d3c8f", kind: Kind::Float, used: true, doc: "the feet's share of the stroke (0)" },
        Param { name: "?17e435bb", kind: Kind::Float, used: true, doc: "the speed a stroke needs (0.73)" },
        Param { name: "?d83b0b56", kind: Kind::Int, used: true, doc: "the stroke's axis: 0 his side's, 1 the water's up, else the character's up" },
        Param { name: "?4077910a", kind: Kind::Bool, used: true, doc: "a stroke pushes at the hand or foot, not the chest" },
        Param { name: "?4158ed9b", kind: Kind::Bool, used: true, doc: "ctx+0x10c6: no reader in NM" },
    ] },
];

const fn b(name: &'static str) -> Param {
    Param { name, kind: Kind::Bool, used: true, doc: "" }
}
const fn f(name: &'static str) -> Param {
    Param { name, kind: Kind::Float, used: true, doc: "" }
}
const fn i(name: &'static str) -> Param {
    Param { name, kind: Kind::Int, used: true, doc: "" }
}
/// What the record's tick doesn't read: the counter impulse (doCounterImpulse: no set sends it true, `rdr2_core4.md`).
const fn left(name: &'static str, kind: Kind) -> Param {
    Param { name, kind, used: false, doc: "RDR2's; taken and not used here yet" }
}

static BULLET_PARAMS: &[Param] = &[
    b("extra"), b("fresh"),
    f("impulsePeriod"), f("impulseDelay"), f("impulseReductionPerShot"), f("impulseRecovery"), f("impulseMinLeakage"), b("impulseLeakageStrengthScaled"),
    b("impulseAirOn"), f("impulseAirMult"), f("impulseAirMultStart"), f("impulseAirMax"), f("impulseAirApplyAbove"),
    b("impulseOneLegOn"), f("impulseOneLegMult"), f("impulseOneLegMultStart"), f("impulseOneLegMax"), f("impulseOneLegApplyAbove"),
    f("impulseNoBalMult"), f("liftGain"),
    i("torqueMode"), i("torqueSpinMode"), i("torqueFilterMode"), b("torqueAlwaysSpine3"), f("torqueDelay"), f("torquePeriod"), f("torqueGain"), f("torqueCutoff"), f("torqueReductionPerTick"),
    f("impulseTorqueScale"), b("impulseSpreadOverParts"), b("loosenessFix"),
    f("impulseBalStabStart"), f("impulseBalStabEnd"), f("impulseBalStabMult"),
    f("impulseSpineAngStart"), f("impulseSpineAngEnd"), f("impulseSpineAngMult"),
    f("impulseVelStart"), f("impulseVelEnd"), f("impulseVelMult"),
    left("counterImpulseDelay", Kind::Float), left("counterImpulseMag", Kind::Float), left("counterAfterMagReached", Kind::Bool),
    left("doCounterImpulse", Kind::Bool), left("counterImpulse2Hips", Kind::Float),
    f("rbRatio"), f("rbLowerShare"), f("rbMoment"), f("rbMaxTwistMomentArm"), f("rbMaxBroomMomentArm"),
    f("rbRatioAirborne"), f("rbMomentAirborne"), f("rbMaxTwistMomentArmAirborne"), f("rbMaxBroomMomentArmAirborne"),
    f("rbRatioOneLeg"), f("rbMomentOneLeg"), f("rbMaxTwistMomentArmOneLeg"), f("rbMaxBroomMomentArmOneLeg"),
    b("rbPivot"), i("rbTwistAxis"),
];

/// One soft limit record (ctx+0x4810, 0x30 B each).
#[derive(Clone, Copy, Debug, Default)]
pub struct SoftRec {
    pub on: bool,
    pub stiffness: f32,
    pub damping: f32,
    pub angle: f32,
    /// +1 from the low end, -1 from the high end
    pub dir: f32,
    pub velocity_scaled: bool,
}

/// A bullet record's template (ctx's two statics; the record copies one at its start). The port's own constants until
/// a configureBullets is sent, which is the behaviour with code defaults.
#[derive(Clone, Copy, Debug)]
pub struct BulletCfg {
    pub period: f32,
    pub delay: f32,
    pub air: Option<[f32; 4]>,
    pub one_leg: Option<[f32; 4]>,
    pub no_bal: f32,
    pub lift: f32,
    pub torque_mode: i32,
    pub spin_mode: i32,
    pub filter_mode: i32,
    pub always_spine3: bool,
    pub torque_delay: f32,
    pub torque_period: f32,
    pub torque_gain: f32,
    pub torque_cutoff: f32,
    pub torque_decay: f32,
    /// the impulse's factors, each (start, end, mult) off the balance quality, the trunk's lean and his speed (+0x54..+0x74)
    pub bal_stab: [f32; 3],
    pub spine_ang: [f32; 3],
    pub vel: [f32; 3],
    /// the angular part's scale (+0x38), a share to the hit part's neighbours (+0xdd), loosenessFix (+0xdf)
    pub torque_scale: f32,
    pub spread: bool,
    pub loose_fix: bool,
    /// the rigid-body spread: on both feet, in the air, on one leg, each (ratio, moment, twist arm, broom arm) (+0x98..+0xc8)
    pub rb: [[f32; 4]; 3],
    pub rb_lower: f32,
    pub rb_pivot: bool,
    pub rb_twist_axis: i32,
}

impl Default for BulletCfg {
    fn default() -> BulletCfg {
        BulletCfg { period: 0.1, delay: 0.0, air: None, one_leg: None, no_bal: 1.0, lift: 0.0, torque_mode: 0, spin_mode: 0, filter_mode: 0, always_spine3: false,
                    torque_delay: 0.0, torque_period: 0.12, torque_gain: 4.0, torque_cutoff: 0.5, torque_decay: 0.9,
                    bal_stab: [3.0, 10.0, 1.0], spine_ang: [0.7, 0.2, 1.0], vel: [1.0, 4.0, 1.0], torque_scale: 1.0, spread: false, loose_fix: false,
                    rb: [[0.0, 1.0, 0.5, 1.0]; 3], rb_lower: 0.5, rb_pivot: false, rb_twist_axis: 0 }
    }
}

/// setFallingReaction's values (0x2a74c3c): catchFall's fields, which only that message writes (rdr2_catchfall.md 2).
#[derive(Clone, Copy, Debug)]
pub struct FallCfg {
    pub com_vel_rds: f32,
    pub inhibit_rolling: f32,
    pub change_friction_time: f32,
    pub ground_friction: f32,
    pub arm_friction: f32,
    pub friction_min: f32,
    pub friction_max: f32,
    pub k_rate: f32,
    pub k_time: f32,
    pub spine_lean: f32,
    pub success: f32,
    pub min_leg: f32,
    pub min_spine: f32,
    pub min_arm: f32,
    pub call_rds: f32,
    pub resist_rolling: f32,
}

impl Default for FallCfg {
    fn default() -> FallCfg {
        FallCfg { com_vel_rds: 2.0, inhibit_rolling: 0.2, change_friction_time: 0.2, ground_friction: 1.0, arm_friction: 0.3, friction_min: 0.0, friction_max: 9999.0, k_rate: 2.0, k_time: 10.0, spine_lean: 0.5, success: 1.0, min_leg: 3.0, min_spine: 4.5, min_arm: 7.0, call_rds: 0.0, resist_rolling: 0.0 }
    }
}

impl FallCfg {
    /// One value by the port's name; the names of the parameters the message takes for these.
    pub const NAMES: [&'static str; 16] = ["comVelRDSThresh", "inhibitRollingTime", "changeFrictionTime", "groundFriction", "armFriction", "frictionMin", "frictionMax", "stoppedStrengthDecay", "kTime", "spineLeanGain", "successStrength", "minLegStiffness", "minSpineStiffness", "minArmStiffness", "callRDS", "resistRolling"];

    pub fn set(&mut self, name: &str, v: f32) {
        match name {
            "comVelRDSThresh" => self.com_vel_rds = v,
            "inhibitRollingTime" => self.inhibit_rolling = v,
            "changeFrictionTime" => self.change_friction_time = v,
            "groundFriction" => self.ground_friction = v,
            "armFriction" => self.arm_friction = v,
            "frictionMin" => self.friction_min = v,
            "frictionMax" => self.friction_max = v,
            "stoppedStrengthDecay" => self.k_rate = v,
            "kTime" => self.k_time = v,
            "spineLeanGain" => self.spine_lean = v,
            "successStrength" => self.success = v,
            "minLegStiffness" => self.min_leg = v,
            "minSpineStiffness" => self.min_spine = v,
            "minArmStiffness" => self.min_arm = v,
            "callRDS" => self.call_rds = v,
            "resistRolling" => self.resist_rolling = v,
            _ => {}
        }
    }
}

/// The core's water (setcharacterunderwater, 0x2a4d8b8; ctx+0x1090..+0x10c6).
#[derive(Clone, Copy, Debug)]
pub struct Water {
    pub on: bool,
    pub point: Vec3,
    pub normal: Vec3,
    pub viscosity: f32,
    pub stroke: f32,
    pub feet: f32,
    pub thresh: f32,
    pub axis: i32,
    pub at_part: bool,
}

impl Default for Water {
    fn default() -> Water {
        Water { on: false, point: Vec3::Z, normal: Vec3::Z, viscosity: 0.0, stroke: 0.0, feet: 0.0, thresh: 0.73, axis: 0, at_part: false }
    }
}

/// What the core messages set: read each tick by `core_tick` and by the bullet records.
#[derive(Clone, Copy, Debug)]
pub struct CoreCfg {
    pub soft: [SoftRec; 4],
    pub soft_was: [bool; 4],
    /// the per-part friction multiplier (ctx+0x35cc)
    pub fric: [f32; 32],
    pub fric_used: bool,
    pub fric_min: f32,
    pub fric_max: f32,
    pub fric_clamp: bool,
    pub muscle: Option<(f32, [bool; 32])>,
    /// ctx+0x102c
    pub strength: f32,
    /// the bullets' templates, main and extra
    pub bullets: [BulletCfg; 2],
    /// ctx+0x28b4, +0x28bc, the recovery rate +0x28b8 (None: the port's own recovery), +0x28c8
    pub reduction: f32,
    pub min_leak: f32,
    pub recovery: Option<f32>,
    pub leak_scaled: bool,
    /// bullets' torque records running, and the sign of the last (.data 0x59af990, 0x59af93c)
    pub torques: i32,
    pub torque_sign: i32,
    pub fall: FallCfg,
    pub water: Water,
}

impl Default for CoreCfg {
    fn default() -> CoreCfg {
        CoreCfg { water: Default::default(), soft: Default::default(), soft_was: [false; 4], fric: [1.0; 32], fric_used: false, fric_min: 0.0, fric_max: 9999.0, fric_clamp: false, muscle: None, strength: 1.0,
                  bullets: Default::default(), reduction: 0.15, min_leak: 0.2, recovery: None, leak_scaled: false, torques: 0, torque_sign: 0, fall: Default::default() }
    }
}

pub fn trace(s: impl FnOnce() -> String) {
    static T: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    if *T.get_or_init(|| std::env::var_os("RDR2_TRACE").is_some()) {
        eprintln!("core: {}", s());
    }
}

/// A trace line the first time a path acts, by its name: proof that it runs, without a line every tick.
pub fn trace_once(what: &'static str, s: impl FnOnce() -> String) {
    static SEEN: std::sync::Mutex<Vec<&'static str>> = std::sync::Mutex::new(vec![]);
    static T: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    if *T.get_or_init(|| std::env::var_os("RDR2_TRACE").is_some()) {
        let mut seen = SEEN.lock().unwrap();
        if !seen.contains(&what) {
            seen.push(what);
            eprintln!("core: first {what}: {}", s());
        }
    }
}

/// The joint's parts: a joint's child is part joint+1; the pelvis goes with any mask that holds both legs.
fn part_mask(mask: &str) -> [bool; 32] {
    let mut out = [false; 32];
    let m = if mask.is_empty() { "fb" } else { mask };
    for j in Character::mask(m) {
        if j + 1 < 32 {
            out[j + 1] = true;
        }
    }
    if matches!(m, "fb" | "lb") {
        out[0] = true;
    }
    out
}

fn joint_mask(mask: &str) -> [bool; 32] {
    let mut out = [false; 32];
    for j in Character::mask(if mask.is_empty() { "fb" } else { mask }) {
        if j < 32 {
            out[j] = true;
        }
    }
    out
}

/// The core's water (0x2a5dd58): with the chest under, each hand and foot under that moves back along the axis faster than the
/// threshold pushes along it, at the chest (the feet by their share); every part under drags by its speed squared.
/// Ours: no feedback when the chest or head crosses the plane (the game's reader isn't here).
fn water_tick(c: &mut Ctx) {
    let w = c.ch.core.cfg.water;
    if !w.on {
        return;
    }
    let n = w.normal.normalize_or_zero();
    let under = |p: Vec3| (p - w.point).dot(n) < 0.0;
    let rp = crate::extras_ysk_rdr2::rp;
    let chest = rp(9);
    if under(c.b.at(chest)) && w.stroke > 0.0 {
        let axis = match w.axis {
            0 => c.m.b,
            1 => n,
            _ => c.up,
        };
        for (i, k) in [16, 19, 5, 7].map(rp).into_iter().enumerate() {
            if under(c.b.at(k)) {
                let s = -axis.dot(c.b.part(k).v - c.vel);
                if s > w.thresh {
                    let g = w.stroke * if i > 1 { w.feet } else { 1.0 } * s;
                    trace_once("water stroke", || format!("part {k}, {g:.1} N"));
                    c.out.force(if w.at_part { k } else { chest }, axis * g);
                }
            }
        }
    }
    if w.viscosity > 1.2e-7 {
        trace_once("water drag", || format!("{}", w.viscosity));
        for k in 0..c.b.frames.len() {
            if under(c.b.at(k)) {
                let v = c.b.part(k).v;
                c.out.force(k, v * (-v.length() * w.viscosity));
            }
        }
    }
}

/// Each tick with the core on: what the core messages hold (0x2a5d400, 0x2a5c4a0, the muscle record).
pub fn core_tick(c: &mut Ctx) {
    c.ch.core.bullet_seen = c.ch.core.bullet_new.take();
    water_tick(c);
    let cfg = c.ch.core.cfg;
    if cfg.fric_used {
        for (k, x) in c.out.part_friction.iter_mut() {
            *x *= cfg.fric.get(*k).copied().unwrap_or(1.0);
        }
        let parts = c.b.frames.len().min(cfg.fric.len());
        for k in 0..parts {
            if !c.out.part_friction.iter().any(|p| p.0 == k) && cfg.fric[k] != 1.0 {
                c.out.part_friction.push((k, cfg.fric[k]));
            }
        }
    }
    c.out.fric_clamp = cfg.fric_clamp.then_some((cfg.fric_min, cfg.fric_max));
    let speed = c.vel.length();
    for (n, r) in cfg.soft.iter().enumerate() {
        let joint = if n < 2 { (*ELBOW())[n] } else { (*KNEE())[n - 2] };
        if !r.on {
            if cfg.soft_was[n] {
                c.ch.eff[joint].soft[0] = Default::default();
                c.ch.core.cfg.soft_was[n] = false;
            }
            continue;
        }
        let l = c.ch.limits(joint);
        let (lo, hi) = (l[0], l[1]);
        let limit = if r.dir > 0.0 { lo + r.angle.min(hi - lo) } else { hi - r.angle.min(hi - lo) };
        let k = if r.velocity_scaled { (speed.clamp(0.0, 4.0) * 0.25 * r.stiffness).max(3.0) } else { r.stiffness };
        let mut l = crate::nmrs::SoftLimit { stiffness: k * k, damping: r.damping, enabled: true, ..Default::default() };
        // ours is active on the far side of dir; RDR2's when dir times the excess isn't positive
        l.set_limit(limit, -r.dir, false);
        c.ch.eff[joint].soft[0] = l;
        c.ch.core.cfg.soft_was[n] = true;
    }
    if let Some((s, parts)) = cfg.muscle {
        for (j, e) in c.ch.eff.iter_mut().enumerate() {
            if parts.get(j).copied().unwrap_or(false) {
                e.muscle_stiffness = s;
            }
        }
    }
}

/// A message that sets its values on the next tick and is over.
struct Once(Box<dyn FnMut(&mut Ctx) + Send + Sync>, bool);

impl Extra for Once {
    fn tick(&mut self, c: &mut Ctx, _dt: f32) {
        if c.ch.rdr2 && !self.1 {
            (self.0)(c);
        }
        self.1 = true;
    }
    fn done(&self) -> bool {
        self.1
    }
}

fn once(f: impl FnMut(&mut Ctx) + Send + Sync + 'static) -> Box<dyn Extra> {
    Box::new(Once(Box::new(f), false))
}

pub fn build(name: &str, p: &mut P) -> Option<Result<Box<dyn Extra>, Error>> {
    Some(match name {
        "configureSoftLimit" => soft_limit(p),
        "setFrictionScale" => friction_scale(p),
        "setMuscleStiffness" => muscle(p),
        "setCharacterStrength" => strength(p),
        "setOpposeGravity" => oppose(p),
        "configureBullets" => bullets(p),
        "setcharacterunderwater" => underwater(p),
        _ => return None,
    })
}

const NOT_SENT: f32 = -1e30;

fn soft_limit(p: &mut P) -> Result<Box<dyn Extra>, Error> {
    let on = p.b("on", true)?;
    let fresh = p.b("fresh", false)?;
    let idx = p.i("index", 0)?.clamp(0, 3) as usize;
    let (k, d) = (p.f("stiffness", NOT_SENT)?, p.f("damping", NOT_SENT)?);
    let angle = p.f("limitAngle", 0.0)?;
    let dir = if p.i("approachDirection", 0)? < 0 { -1.0 } else { 1.0 };
    let vs = p.b("velocityScaled", false)?;
    Ok(once(move |c| {
        let r = &mut c.ch.core.cfg.soft[idx];
        if !on {
            r.on = false;
            return;
        }
        // the registered defaults of 0x2a294d0 on a start
        if fresh || r.stiffness == 0.0 {
            (r.stiffness, r.damping) = (15.0, 1.0);
        }
        if k != NOT_SENT {
            r.stiffness = k;
        }
        if d != NOT_SENT {
            r.damping = d;
        }
        (r.angle, r.dir, r.velocity_scaled, r.on) = (angle, dir, vs, true);
        let r = *r;
        trace(|| format!("soft limit {idx}: k {} d {} angle {} dir {} velocityScaled {}", r.stiffness, r.damping, r.angle, r.dir, r.velocity_scaled));
    }))
}

fn friction_scale(p: &mut P) -> Result<Box<dyn Extra>, Error> {
    let scale = p.f("scale", NOT_SENT)?;
    let mask = p.s("mask", "fb")?;
    let (lo, hi) = (p.f("globalMin", NOT_SENT)?, p.f("globalMax", NOT_SENT)?);
    let clamp = p.b("clamp", false)?;
    Ok(once(move |c| {
        let cfg = &mut c.ch.core.cfg;
        if scale != NOT_SENT {
            let parts = part_mask(&mask);
            for (k, x) in cfg.fric.iter_mut().enumerate() {
                if parts[k] {
                    *x = scale;
                }
            }
            cfg.fric_used = true;
        }
        if lo != NOT_SENT {
            cfg.fric_min = lo;
        }
        if hi != NOT_SENT {
            cfg.fric_max = hi;
        }
        cfg.fric_clamp = clamp;
        let (min, max) = (cfg.fric_min, cfg.fric_max);
        trace(|| format!("friction scale {scale} on {mask}; clamp {clamp} {min}..{max}"));
    }))
}

fn muscle(p: &mut P) -> Result<Box<dyn Extra>, Error> {
    let on = p.b("on", true)?;
    let s = p.f("muscleStiffness", 1.0)?;
    let mask = p.s("mask", "fb")?;
    Ok(once(move |c| {
        c.ch.core.cfg.muscle = on.then(|| (s, joint_mask(&mask)));
        trace(|| format!("muscle stiffness {s} on {mask}, on {on}"));
    }))
}

/// 0x2a4f9b8: one request at priority 0x27 on the mask; the balancer's own write takes over its joints next step.
fn oppose(p: &mut P) -> Result<Box<dyn Extra>, Error> {
    let v = p.f("oppose", 1.0)?;
    let mask = p.s("mask", "fb")?;
    Ok(once(move |c| {
        let on = joint_mask(&mask);
        for (j, e) in c.ch.eff.iter_mut().enumerate() {
            if on.get(j).copied().unwrap_or(false) {
                e.oppose_gravity = v;
            }
        }
        trace(|| format!("oppose gravity {v} on {mask}"));
    }))
}

fn strength(p: &mut P) -> Result<Box<dyn Extra>, Error> {
    let s = p.f("characterStrength", 1.0)?;
    Ok(once(move |c| {
        c.ch.core.cfg.strength = s;
        trace(|| format!("character strength {s}"));
    }))
}

fn underwater(p: &mut P) -> Result<Box<dyn Extra>, Error> {
    let d = Water::default();
    let on = p.b("Underwater", d.on)?;
    let w = Water { on, point: p.v("surfacePoint", d.point)?, normal: p.v("surfaceNormal", d.normal)?, viscosity: p.f("viscosity", d.viscosity)?,
                    stroke: p.f("stroke", d.stroke)?, feet: p.f("?704d3c8f", d.feet)?, thresh: p.f("?17e435bb", d.thresh)?, axis: p.i("?d83b0b56", d.axis)?,
                    at_part: p.b("?4077910a", d.at_part)? };
    p.b("?4158ed9b", false)?;
    Ok(once(move |c| {
        c.ch.core.cfg.water = w;
        trace(|| format!("water {w:?}"));
    }))
}

fn bullets(p: &mut P) -> Result<Box<dyn Extra>, Error> {
    let extra = p.b("extra", false)?;
    let fresh = p.b("fresh", false)?;
    let mut o = |n: &str| p.f(n, NOT_SENT).map(|x| (x != NOT_SENT).then_some(x));
    let (period, delay, red, rec, leak) = (o("impulsePeriod")?, o("impulseDelay")?, o("impulseReductionPerShot")?, o("impulseRecovery")?, o("impulseMinLeakage")?);
    let air_v = [o("impulseAirMult")?, o("impulseAirMultStart")?, o("impulseAirMax")?, o("impulseAirApplyAbove")?];
    let leg_v = [o("impulseOneLegMult")?, o("impulseOneLegMultStart")?, o("impulseOneLegMax")?, o("impulseOneLegApplyAbove")?];
    let (nobal, lift) = (o("impulseNoBalMult")?, o("liftGain")?);
    let tq = [o("torqueDelay")?, o("torquePeriod")?, o("torqueGain")?, o("torqueCutoff")?, o("torqueReductionPerTick")?];
    let fac = [o("impulseBalStabStart")?, o("impulseBalStabEnd")?, o("impulseBalStabMult")?, o("impulseSpineAngStart")?, o("impulseSpineAngEnd")?, o("impulseSpineAngMult")?,
               o("impulseVelStart")?, o("impulseVelEnd")?, o("impulseVelMult")?, o("impulseTorqueScale")?];
    let rbs = [[o("rbRatio")?, o("rbMoment")?, o("rbMaxTwistMomentArm")?, o("rbMaxBroomMomentArm")?],
               [o("rbRatioAirborne")?, o("rbMomentAirborne")?, o("rbMaxTwistMomentArmAirborne")?, o("rbMaxBroomMomentArmAirborne")?],
               [o("rbRatioOneLeg")?, o("rbMomentOneLeg")?, o("rbMaxTwistMomentArmOneLeg")?, o("rbMaxBroomMomentArmOneLeg")?]];
    let rb_lower = o("rbLowerShare")?;
    let mut q = |n: &str| p.b(n, false);
    let (air_on, leg_on, scaled, spine3) = (q("impulseAirOn")?, q("impulseOneLegOn")?, q("impulseLeakageStrengthScaled")?, q("torqueAlwaysSpine3")?);
    let (spread, loose_fix, rb_pivot) = (q("impulseSpreadOverParts")?, q("loosenessFix")?, q("rbPivot")?);
    let (mode, spin, filter) = (p.i("torqueMode", -1)?, p.i("torqueSpinMode", -1)?, p.i("torqueFilterMode", -1)?);
    let rb_axis = p.i("rbTwistAxis", -1)?;
    for x in BULLET_PARAMS.iter().filter(|x| !x.used) {
        match x.kind {
            Kind::Float => drop(p.f(x.name, 0.0)?),
            Kind::Bool => drop(p.b(x.name, false)?),
            _ => drop(p.i(x.name, 0)?),
        }
    }
    Ok(once(move |c| {
        let cfg = &mut c.ch.core.cfg;
        let i = extra as usize;
        if fresh {
            cfg.bullets[i] = Default::default();
        }
        let t = &mut cfg.bullets[i];
        let set = |x: &mut f32, v: Option<f32>| if let Some(v) = v { *x = v };
        set(&mut t.period, period);
        set(&mut t.delay, delay);
        set(&mut t.no_bal, nobal);
        set(&mut t.lift, lift);
        set(&mut t.torque_delay, tq[0]);
        set(&mut t.torque_period, tq[1]);
        set(&mut t.torque_gain, tq[2]);
        set(&mut t.torque_cutoff, tq[3]);
        set(&mut t.torque_decay, tq[4]);
        let curve = |v: [Option<f32>; 4], d: Option<[f32; 4]>| {
            let d = d.unwrap_or([1.0, 100.0, 100.0, 399.0]);
            std::array::from_fn(|k| v[k].unwrap_or(d[k]))
        };
        t.air = air_on.then(|| curve(air_v, t.air));
        t.one_leg = leg_on.then(|| curve(leg_v, t.one_leg));
        t.always_spine3 = spine3;
        for (n, x) in [t.bal_stab[0], t.bal_stab[1], t.bal_stab[2], t.spine_ang[0], t.spine_ang[1], t.spine_ang[2], t.vel[0], t.vel[1], t.vel[2], t.torque_scale].into_iter().enumerate() {
            let v = fac[n].unwrap_or(x);
            match n {
                0..=2 => t.bal_stab[n] = v,
                3..=5 => t.spine_ang[n - 3] = v,
                6..=8 => t.vel[n - 6] = v,
                _ => t.torque_scale = v,
            }
        }
        for s in 0..3 {
            for n in 0..4 {
                if let Some(v) = rbs[s][n] {
                    t.rb[s][n] = v;
                }
            }
        }
        set(&mut t.rb_lower, rb_lower);
        (t.spread, t.loose_fix, t.rb_pivot) = (spread, loose_fix, rb_pivot);
        if rb_axis >= 0 {
            t.rb_twist_axis = rb_axis;
        }
        if mode >= 0 {
            t.torque_mode = mode;
        }
        if spin >= 0 {
            t.spin_mode = spin;
        }
        if filter >= 0 {
            t.filter_mode = filter;
        }
        if !extra {
            set(&mut cfg.reduction, red);
            set(&mut cfg.min_leak, leak);
            if rec.is_some() {
                cfg.recovery = rec;
            }
            cfg.leak_scaled = scaled;
        }
        let (r, m, v, t) = (cfg.reduction, cfg.min_leak, cfg.recovery, cfg.bullets[i]);
        trace(|| format!("bullets {}: reduction {r} floor {m} recovery {v:?} {t:?}", if extra { "extra" } else { "main" }));
    }))
}

/// A record's impulse factor off the balancer's feet (0x2a40518's two curves): below the start no change, up to the max
/// a line of the multiplier, above that held at the max; none past the apply limit.
pub fn curve(c: [f32; 4], mag: f32) -> f32 {
    let [mult, start, max, above] = c;
    if !(mag < above && start < mag && mag > 1e-6) {
        return 1.0;
    }
    if mag < max {
        if max - start <= 1e-6 {
            return 1.0;
        }
        ((mag - start) * mult + start) / mag
    } else {
        max / mag
    }
}

/// The torque a bullet's record starts with (0x2a2ccc8), and the share of its impulse left. Hit in the parts mask
/// 0x103f09 (pelvis, trunk, clavicles, neck, head), when the filter lets it.
pub fn bullet_torque(cfg: &mut CoreCfg, t: &BulletCfg, r: Vec3, imp: Vec3, axis: Vec3, rand_bit: bool) -> Option<(f32, f32)> {
    if t.torque_mode == 0 {
        return None;
    }
    let perp = r - axis * r.dot(axis);
    let moment = perp.cross(imp).dot(axis);
    let mut sign = if moment > 0.0 { 1 } else { -1 };
    let pass = match t.filter_mode {
        1 => cfg.torques == 0,
        2 => sign != cfg.torque_sign,
        _ => true,
    };
    if !pass {
        return None;
    }
    cfg.torques += 1;
    match t.spin_mode {
        2 if cfg.torque_sign != 0 => sign = -cfg.torque_sign,
        1 => sign = if rand_bit { 1 } else { -1 },
        _ => {}
    }
    cfg.torque_sign = sign;
    if t.torque_mode == 1 {
        let kept = (1.0 - cfg.strength) * (1.0 - t.torque_cutoff) + t.torque_cutoff;
        Some((imp.length() * (1.0 - kept) * sign as f32 * t.torque_gain, kept))
    } else {
        Some((moment * t.torque_gain, 1.0))
    }
}
