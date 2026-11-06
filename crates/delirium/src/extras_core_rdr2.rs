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

pub static MESSAGES: [Message; 5] = [
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
/// What's used by the record's tick; the rb family (a rigid-body spread of the impulse over the body) and the stability
/// and velocity factors are taken and left, see `rdr2_params.md`.
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
    left("impulseTorqueScale", Kind::Float), left("impulseSpreadOverParts", Kind::Bool), left("loosenessFix", Kind::Bool),
    left("impulseBalStabStart", Kind::Float), left("impulseBalStabEnd", Kind::Float), left("impulseBalStabMult", Kind::Float),
    left("impulseSpineAngStart", Kind::Float), left("impulseSpineAngEnd", Kind::Float), left("impulseSpineAngMult", Kind::Float),
    left("impulseVelStart", Kind::Float), left("impulseVelEnd", Kind::Float), left("impulseVelMult", Kind::Float),
    left("counterImpulseDelay", Kind::Float), left("counterImpulseMag", Kind::Float), left("counterAfterMagReached", Kind::Bool),
    left("doCounterImpulse", Kind::Bool), left("counterImpulse2Hips", Kind::Float),
    left("rbRatio", Kind::Float), left("rbLowerShare", Kind::Float), left("rbMoment", Kind::Float), left("rbMaxTwistMomentArm", Kind::Float), left("rbMaxBroomMomentArm", Kind::Float),
    left("rbRatioAirborne", Kind::Float), left("rbMomentAirborne", Kind::Float), left("rbMaxTwistMomentArmAirborne", Kind::Float), left("rbMaxBroomMomentArmAirborne", Kind::Float),
    left("rbRatioOneLeg", Kind::Float), left("rbMomentOneLeg", Kind::Float), left("rbMaxTwistMomentArmOneLeg", Kind::Float), left("rbMaxBroomMomentArmOneLeg", Kind::Float),
    left("rbPivot", Kind::Bool), left("rbTwistAxis", Kind::Int),
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
}

impl Default for BulletCfg {
    fn default() -> BulletCfg {
        BulletCfg { period: 0.1, delay: 0.0, air: None, one_leg: None, no_bal: 1.0, lift: 0.0, torque_mode: 0, spin_mode: 0, filter_mode: 0, always_spine3: false,
                    torque_delay: 0.0, torque_period: 0.12, torque_gain: 4.0, torque_cutoff: 0.5, torque_decay: 0.9 }
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
}

impl Default for CoreCfg {
    fn default() -> CoreCfg {
        CoreCfg { soft: Default::default(), soft_was: [false; 4], fric: [1.0; 32], fric_used: false, fric_min: 0.0, fric_max: 9999.0, fric_clamp: false, muscle: None, strength: 1.0,
                  bullets: Default::default(), reduction: 0.15, min_leak: 0.2, recovery: None, leak_scaled: false, torques: 0, torque_sign: 0 }
    }
}

fn trace(s: impl FnOnce() -> String) {
    static T: std::sync::OnceLock<bool> = std::sync::OnceLock::new();
    if *T.get_or_init(|| std::env::var_os("RDR2_TRACE").is_some()) {
        eprintln!("core: {}", s());
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

/// Each tick with the core on: what the core messages hold (0x2a5d400, 0x2a5c4a0, the muscle record).
pub fn core_tick(c: &mut Ctx) {
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
        "configureBullets" => bullets(p),
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

fn strength(p: &mut P) -> Result<Box<dyn Extra>, Error> {
    let s = p.f("characterStrength", 1.0)?;
    Ok(once(move |c| {
        c.ch.core.cfg.strength = s;
        trace(|| format!("character strength {s}"));
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
    let mut q = |n: &str| p.b(n, false);
    let (air_on, leg_on, scaled, spine3) = (q("impulseAirOn")?, q("impulseOneLegOn")?, q("impulseLeakageStrengthScaled")?, q("torqueAlwaysSpine3")?);
    let (mode, spin, filter) = (p.i("torqueMode", -1)?, p.i("torqueSpinMode", -1)?, p.i("torqueFilterMode", -1)?);
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
