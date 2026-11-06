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

pub static MESSAGES: [Message; 16] = [
    Message { name: "configureLimits", params: &[
        Param { name: "mask", kind: Kind::Str, used: true, doc: "the joints (fb)" },
        Param { name: "index", kind: Kind::Int, used: true, doc: "one joint, by number (-1: the mask's)" },
        Param { name: "enable", kind: Kind::Bool, used: true, doc: "false: the limits wide open this tick (true)" },
        Param { name: "?a45afe45", kind: Kind::Bool, used: true, doc: "back to the base limits (false)" },
        Param { name: "toDesired", kind: Kind::Bool, used: true, doc: "wide enough for the desired angles (false)" },
        Param { name: "toCurAnimation", kind: Kind::Bool, used: true, doc: "wide enough for the actual angles and the margin (false)" },
        Param { name: "margin", kind: Kind::Float, used: true, doc: "toCurAnimation's margin, rad (0.19635)" },
        Param { name: "lean1", kind: Kind::Float, used: true, doc: "else: a half-range at least this; a hinge's high end (1.5708)" },
        Param { name: "lean2", kind: Kind::Float, used: true, doc: "a half-range at least this (1.5708)" },
        Param { name: "twist", kind: Kind::Float, used: true, doc: "a half-range at least this; a hinge's low end at most this (1.5708)" },
    ] },
    Message { name: "configureConstraints", params: &[
        Param { name: "mask", kind: Kind::Str, used: true, doc: "RDR2's constraint config; the joints' limits take in where they are, for good (not sent by any set)" },
    ] },
    Message { name: "SoftKey", params: &[
        Param { name: "mask", kind: Kind::Str, used: true, doc: "the joints (fb)" },
        Param { name: "gravityCompensation", kind: Kind::Float, used: true, doc: "the gravity share after the clamp (0)" },
        Param { name: "?b7cd5720", kind: Kind::Vec3, used: true, doc: "the linear acceleration limit per axis (50)" },
        Param { name: "?996bb59c", kind: Kind::Vec3, used: true, doc: "the angular acceleration limit per axis (50)" },
    ] },
    Message { name: "configureSelfAvoidance", params: &[
        Param { name: "useSelfAvoidance", kind: Kind::Bool, used: true, doc: "the arms' IK targets go round the trunk (false)" },
        Param { name: "overwriteDragReduction", kind: Kind::Bool, used: true, doc: "from the target before its lead (false)" },
        Param { name: "torsoSwingFraction", kind: Kind::Float, used: true, doc: "the share of the angle to the hand the target turns by (0.75)" },
        Param { name: "maxTorsoSwingAngleRad", kind: Kind::Float, used: true, doc: "at most this (0.758)" },
        Param { name: "selfAvoidIfInSpineBoundsOnly", kind: Kind::Bool, used: true, doc: "only with the target along the trunk (false)" },
        Param { name: "selfAvoidAmount", kind: Kind::Float, used: true, doc: "the arm's length share kept out when the target is behind (0.5)" },
        Param { name: "overwriteTwist", kind: Kind::Bool, used: true, doc: "the IK's twist comes from the shoulder's angle (false)" },
        Param { name: "usePolarPathAlgorithm", kind: Kind::Bool, used: true, doc: "the polar path round a capsule instead (false)" },
        Param { name: "radius", kind: Kind::Float, used: true, doc: "the capsule's radius (0.3)" },
    ] },
    Message { name: "shotNewBullet", params: &[
        Param { name: "bodyPart", kind: Kind::Int, used: true, doc: "the part hit (the pelvis)" },
        Param { name: "localHitPointInfo", kind: Kind::Bool, used: true, doc: "hitPoint and normal are in the part's space (false)" },
        Param { name: "hitPoint", kind: Kind::Vec3, used: true, doc: "the point (the part's centre)" },
        Param { name: "normal", kind: Kind::Vec3, used: true, doc: "its normal" },
        Param { name: "?06c06365", kind: Kind::Bool, used: true, doc: "the shot's reaction takes it" },
        Param { name: "?4d7dfd89", kind: Kind::Bool, used: true, doc: "the duck's takes it" },
        Param { name: "?2f471f82", kind: Kind::Bool, used: true, doc: "the third behaviour's takes it" },
    ] },
    Message { name: "setCharacterCollisions", params: &[
        Param { name: "spin", kind: Kind::Float, used: true, doc: "the hit's spin (0)" },
        Param { name: "maxVelocity", kind: Kind::Float, used: true, doc: "only under this speed (8)" },
        Param { name: "footSlip", kind: Kind::Bool, used: true, doc: "the foot yaw hold lets go while in contact" },
        Param { name: "gsfricscale1", kind: Kind::Float, used: true, doc: "friction of mask 1's parts (8)" },
        Param { name: "gsfricscale2", kind: Kind::Float, used: true, doc: "friction of mask 2's parts (0.2)" },
        Param { name: "?b11746eb", kind: Kind::Float, used: true, doc: "under 0: the scale applies; else the friction's cap (-0.2)" },
        Param { name: "vehicleflags", kind: Kind::Str, used: false, doc: "the object types (object types aren't kept here: any moving obstacle)" },
        Param { name: "applyToAll", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "applyToSpine", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "applyToThighs", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "applyToClavicles", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "applyToUpperArms", kind: Kind::Bool, used: true, doc: "" },
    ] },
    Message { name: "setCharacterDamping", params: &[
        Param { name: "somersaultThresh", kind: Kind::Float, used: true, doc: "the contact spin limiter: the angular momentum about the COM's x before it acts (34)" },
        Param { name: "somersaultDamp", kind: Kind::Float, used: true, doc: "its gain (0)" },
        Param { name: "cartwheelThresh", kind: Kind::Float, used: true, doc: "about the COM's z (27)" },
        Param { name: "cartwheelDamp", kind: Kind::Float, used: true, doc: "its gain (0)" },
        Param { name: "vehicleCollisionTime", kind: Kind::Float, used: true, doc: "seconds it runs from a contact with an object of setCharacterCollisions' types (0)" },
        Param { name: "v2", kind: Kind::Float, used: true, doc: "only if the COM is slower than this (0)" },
    ] },
    Message { name: "spinLimit", params: &[
        Param { name: "somersaultThresh", kind: Kind::Float, used: true, doc: "?b7960dec, the shot's spin limiter: the angular momentum about the COM's x before it acts" },
        Param { name: "somersaultDamp", kind: Kind::Float, used: true, doc: "its gain" },
        Param { name: "cartwheelThresh", kind: Kind::Float, used: true, doc: "about the COM's z" },
        Param { name: "cartwheelDamp", kind: Kind::Float, used: true, doc: "its gain" },
        Param { name: "?7d54085f", kind: Kind::Float, used: true, doc: "seconds it runs from a bullet's start (0)" },
        Param { name: "?27795104", kind: Kind::Float, used: true, doc: "only if the COM is slower than this at the bullet (0.1)" },
    ] },
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
    Message { name: "quadLimbLimit", params: &[
        Param { name: "limb", kind: Kind::Int, used: false, doc: "?e35a5d1a: its handler returns at once on a biped" },
        Param { name: "type", kind: Kind::Int, used: false, doc: "" },
        Param { name: "min", kind: Kind::Float, used: false, doc: "" },
        Param { name: "max", kind: Kind::Float, used: false, doc: "" },
        Param { name: "direction", kind: Kind::Float, used: false, doc: "" },
    ] },
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
    /// stopManual (+0x90): a floor on the slope factor; over 0 it also turns the slope block on
    pub stop_manual: f32,
    /// stopOnSlopes (+0xbc): the time after which slopes count
    pub stop_on_slopes: f32,
    /// blendkneerate (+0xa4): the knees' rate
    pub knee_rate: f32,
    /// antiPropClav (+0xc1): falling backward, no clavicle
    pub anti_prop_clav: f32,
    /// ?7dad7aaf (+0xc3): the arm's swing circle offset (the ctor's 1, the message's 0)
    pub swing_circle: f32,
    /// ?dcd030d5 (ctx+0x1068): the knee and foot tweak in the core's step
    pub knee_tweak: f32,
    /// zmpgroundcollision (ctx+0x105c): the tweaked knee's limit
    pub knee_limit: f32,
    /// ?7579162f (ctx+0x1060): that knee's limit narrows back at this rate
    pub limit_rate: f32,
    /// ?4fe4ab89 (ctx+0x1064): a bent knee's foot's friction, lying down
    pub foot_friction: f32,
}

impl Default for FallCfg {
    fn default() -> FallCfg {
        FallCfg { stop_manual: 0.0, stop_on_slopes: f32::MAX, knee_rate: 10000.0, anti_prop_clav: 0.0, swing_circle: 1.0, knee_tweak: 1.0, knee_limit: 3.0, limit_rate: 1.0, foot_friction: 0.0, com_vel_rds: 2.0, inhibit_rolling: 0.2, change_friction_time: 0.2, ground_friction: 1.0, arm_friction: 0.3, friction_min: 0.0, friction_max: 9999.0, k_rate: 2.0, k_time: 10.0, spine_lean: 0.5, success: 1.0, min_leg: 3.0, min_spine: 4.5, min_arm: 7.0, call_rds: 0.0, resist_rolling: 0.0 }
    }
}

impl FallCfg {
    /// What the message's handler (0x2a74c3c) writes before its values: where the registered default isn't the ctor's.
    pub fn message_defaults() -> FallCfg {
        FallCfg { stop_on_slopes: 10000.0, swing_circle: 0.0, knee_tweak: 0.0, ..FallCfg::default() }
    }

    /// One value by the port's name; the names of the parameters the message takes for these.
    pub const NAMES: [&'static str; 25] = ["stopManual", "stopOnSlopes", "blendkneerate", "antiPropClav", "?7dad7aaf", "?dcd030d5", "zmpgroundcollision", "?7579162f", "?4fe4ab89", "comVelRDSThresh", "inhibitRollingTime", "changeFrictionTime", "groundFriction", "armFriction", "frictionMin", "frictionMax", "stoppedStrengthDecay", "kTime", "spineLeanGain", "successStrength", "minLegStiffness", "minSpineStiffness", "minArmStiffness", "callRDS", "resistRolling"];

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
            "stopManual" => self.stop_manual = v,
            "stopOnSlopes" => self.stop_on_slopes = v,
            "blendkneerate" => self.knee_rate = v,
            "antiPropClav" => self.anti_prop_clav = v,
            "?7dad7aaf" => self.swing_circle = v,
            "?dcd030d5" => self.knee_tweak = v,
            "zmpgroundcollision" => self.knee_limit = v,
            "?7579162f" => self.limit_rate = v,
            "?4fe4ab89" => self.foot_friction = v,
            _ => {}
        }
    }
}

/// The arms' IK target with self avoidance on (0x2a4b520): round the trunk. A is the top of the spine, B the pelvis, n the
/// top part's second axis negated; T the target, F the hand. Returns the new target, and the twist overwriteTwist gives.
pub fn self_avoid_target(r: &SelfAvoid, a: Vec3, b: Vec3, n: Vec3, t: Vec3, hand: Vec3, shoulder: Vec3, elbow: Vec3) -> (Vec3, Option<f32>) {
    let u = (a - b).normalize_or_zero();
    let perp = |p: Vec3, o: Vec3| { let d = p - o; d - u * d.dot(u) };
    if r.polar {
        // 0x2a36710: a path round a capsule, in the plane across it
        let len = (a - b).length();
        let along = (t - b).dot(u);
        if along < -r.radius || along > len + r.radius {
            return (t, None);
        }
        let e2 = u.cross(n).normalize_or_zero();
        let e1 = u.cross(e2).normalize_or_zero();
        let flat = |p: Vec3| { let d = p - b; bevy_math::Vec2::new(d.dot(e1), d.dot(e2)) };
        let (f2, t2) = (flat(hand), flat(t));
        let d = t2 - f2;
        let s = if d.length_squared() > 0.0 { (-f2.dot(d) / d.length_squared()).clamp(0.0, 1.0) } else { 0.0 };
        let near = f2 + d * s;
        let cross = f2.x * t2.y - f2.y * t2.x;
        let rho = near.length() * if cross < 0.0 { -1.0 } else { 1.0 };
        let dphi = t2.y.atan2(t2.x) - f2.y.atan2(f2.x);
        let lam = ((r.radius - rho) / (f2.length().min(t2.length()) - rho)).clamp(0.0, 1.0);
        let f_unit = f2.normalize_or_zero();
        let arc = f2 + f_unit * (t2.length() - f2.length()) + bevy_math::Vec2::new(-f2.y, f2.x) * dphi;
        let out = t2 * (1.0 - lam) + arc * lam;
        return (b + u * along + e1 * out.x + e2 * out.y, None);
    }
    // 0x2a36cec: turned about the axis toward the hand by a capped share (0x2a21930), then moved out or in
    let v = u.cross(n).normalize_or_zero();
    let rr = (v - u * v.dot(u)).normalize_or_zero();
    let (tp, fp) = (perp(t, a).normalize_or_zero(), perp(hand, a).normalize_or_zero());
    let phi_t = tp.dot(rr).clamp(-1.0, 1.0).acos();
    let axis = rr.cross(tp).normalize_or_zero();
    let sign = if rr.cross(fp).dot(axis) <= 0.0 { -1.0 } else { 1.0 };
    let phi_f = fp.dot(rr).clamp(-1.0, 1.0).acos() * sign;
    let turn = ((phi_f - phi_t) * r.fraction).clamp(-r.most, r.most);
    let t1 = a + bevy_math::Quat::from_axis_angle(axis, turn) * (t - a);
    let radial = perp(t1, a);
    let r1 = radial.length();
    let chain = (hand - elbow).length() + (elbow - shoulder).length();
    let along = (t - b).dot(u);
    let inside = !r.in_spine || (along >= -r.amount && along <= (a - b).length() + r.amount);
    if chain <= r1 || !inside {
        return (t, None);
    }
    let c = tp.dot(fp).clamp(-1.0, 1.0);
    let cs = perp(shoulder, a).normalize_or_zero().dot(fp).clamp(-1.0, 1.0);
    let far = if c >= 0.0 { r1 } else { chain * r.amount };
    let d = (r1 - ((1.0 - c.abs()) * perp(hand, a).length() + c.abs() * far)).abs();
    let twist = r.twist.then(|| if cs >= 0.0 { -std::f32::consts::FRAC_PI_2 } else { -std::f32::consts::FRAC_PI_4 } * cs);
    (t1 + radial.normalize_or_zero() * d, twist)
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
    pub self_avoid: SelfAvoid,
    pub spin: SpinRec,
    pub spin_hit: SpinRec,
    pub coll: CollRec,
    /// the shot's +0x3a0..+0x3ac (shotspin): the spine's limit raises, lean1, lean2, twist, and until how long after a bullet
    pub shot_spine: [f32; 4],
}

/// What configureSelfAvoidance leaves on the arm limbs (+0x3f0..+0x404), ctor's defaults.
#[derive(Clone, Copy, Debug)]
pub struct SelfAvoid {
    pub on: bool,
    pub drag: bool,
    pub fraction: f32,
    pub most: f32,
    pub in_spine: bool,
    pub amount: f32,
    pub twist: bool,
    pub polar: bool,
    pub radius: f32,
}

impl Default for SelfAvoid {
    fn default() -> SelfAvoid {
        SelfAvoid { on: false, drag: false, fraction: 0.75, most: 0.758, in_spine: false, amount: 0.5, twist: false, polar: false, radius: 0.3 }
    }
}

/// The bullet's spin limiter's record (ctx+0x2bc8), its ctor's values: off until a bullet arms it.
#[derive(Clone, Copy, Debug)]
pub struct SpinRec {
    pub lim: [f32; 2],
    pub gain: [f32; 2],
    pub time: f32,
    pub speed: f32,
    pub left: f32,
}

impl Default for SpinRec {
    fn default() -> SpinRec {
        SpinRec { lim: [2.0; 2], gain: [5.0; 2], time: 0.0, speed: 0.1, left: -0.1 }
    }
}

/// setCharacterCollisions' fields (ctx+0x10e8..+0x1110).
#[derive(Clone, Copy, Debug, Default)]
pub struct CollRec {
    pub on: bool,
    pub spin: f32,
    pub max_speed: f32,
    pub foot_slip: bool,
    pub scale: [f32; 2],
    pub cap: f32,
    pub spine: bool,
    pub thighs: bool,
    pub clavicles: bool,
    pub upper: bool,
    pub all: bool,
}

impl Default for CoreCfg {
    fn default() -> CoreCfg {
        CoreCfg { shot_spine: [-0.5, -0.3, -0.3, -0.3], self_avoid: Default::default(), spin: Default::default(), spin_hit: SpinRec { lim: [34.0, 27.0], gain: [0.0; 2], time: 0.0, speed: 0.0, left: -0.1 }, coll: Default::default(), water: Default::default(), soft: Default::default(), soft_was: [false; 4], fric: [1.0; 32], fric_used: false, fric_min: 0.0, fric_max: 9999.0, fric_clamp: false, muscle: None, strength: 1.0,
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
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        c.dt = dt;
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
        "configureLimits" => limits(p),
        "configureConstraints" => constraints(p),
        "SoftKey" => soft_key(p),
        "configureSelfAvoidance" => self_avoid(p),
        "shotNewBullet" => new_bullet(p),
        "setCharacterCollisions" => collisions(p),
        "spinLimit" => spin_limit(p),
        "setCharacterDamping" => damping(p),
        "quadLimbLimit" => limb_limit(p),
        _ => return None,
    })
}

const NOT_SENT: f32 = -1e30;

/// A trace line once per text, for paths with no state of their own to keep it in.
pub fn trace_first(s: &'static str) {
    trace_once(s, || s.to_string());
}

/// configureLimits (0x2a27f68), on one joint or a mask's. What it asks lasts this tick: the drive puts the limits back
/// to their base after it, and they narrow back at the core's rate. On a hinge lean1 is the high end and twist the low [C].
fn limits(p: &mut P) -> Result<Box<dyn Extra>, Error> {
    let mask = p.s("mask", "fb")?;
    let idx = p.i("index", -1)?;
    let (enable, base, to_desired, to_actual) = (p.b("enable", true)?, p.b("?a45afe45", false)?, p.b("toDesired", false)?, p.b("toCurAnimation", false)?);
    let (margin, l1, l2, tw) = (p.f("margin", 0.19635)?, p.f("lean1", 1.570796)?, p.f("lean2", 1.570796)?, p.f("twist", 1.570796)?);
    Ok(once(move |c| {
        let on = joint_mask(&mask);
        // (the index by RDR2's numbers: a joint is its child part less one)
        let joints: Vec<usize> = if idx >= 0 { vec![crate::extras_ysk_rdr2::rp(idx as usize + 1).saturating_sub(1)] } else { (0..c.ch.eff.len()).filter(|&j| on.get(j).copied().unwrap_or(false)).collect() };
        let n = c.ch.eff.len();
        for &j in joints.iter().filter(|&&j| j < n) {
            if !enable {
                c.ch.lim_open(j);
            } else if base {
                c.ch.lim_to_base(j);
            } else if to_desired {
                let a = c.ch.eff[j].desired;
                c.ch.lim_around(j, a, 0.0);
            } else if to_actual {
                let a = c.ch.actual(j, c.b.frames);
                c.ch.lim_around(j, a, margin);
            } else if c.ch.joints[j].three {
                c.ch.lim_raise(j, [l1, l2, tw]);
            } else {
                c.ch.lim_raise_hinge(j, tw, l1);
            }
        }
        trace(|| format!("configureLimits on {} joints: enable {enable} base {base} toDesired {to_desired} toCurAnimation {to_actual}", joints.len()));
    }))
}

/// configureConstraints' limit half (0x2a27178): each joint's limits take in where it is (slot +0x110, 1, 0), and the base
/// takes that for good (slot +0xf0, 0).
fn constraints(p: &mut P) -> Result<Box<dyn Extra>, Error> {
    let mask = p.s("mask", "fb")?;
    Ok(once(move |c| {
        let on = joint_mask(&mask);
        for j in (0..c.ch.eff.len()).filter(|&j| on.get(j).copied().unwrap_or(false)) {
            let a = c.ch.actual(j, c.b.frames);
            c.ch.lim_around(j, a, 0.0);
            c.ch.lim_commit(j);
        }
        trace(|| format!("configureConstraints: limits committed on {mask}"));
    }))
}

/// SoftKey (0x2a58364 -> 0x2a572a0 -> each limb's slot +0x58 -> each joint record's 0x2a57378): each joint's child part,
/// once per message, is driven at the incoming pose as keyframe's soft tracking drives it, without the weights, the spikes
/// or the COM frame: the velocity that closes the gap this step, braked along the way to what the limits can shed, the
/// acceleration clamped per axis, the gravity share after the clamp. Nothing without an incoming pose.
fn soft_key(p: &mut P) -> Result<Box<dyn Extra>, Error> {
    let mask = p.s("mask", "fb")?;
    let (lim_a, lim_w) = (p.v("?b7cd5720", Vec3::splat(50.0))?, p.v("?996bb59c", Vec3::splat(50.0))?);
    let grav = p.f("gravityCompensation", 0.0)?;
    Ok(once(move |c| {
        if c.incoming.len() < c.b.frames.len() || !c.incoming.iter().any(|t| t.translation != Vec3::ZERO) {
            trace(|| "SoftKey: no incoming pose, nothing".to_string());
            return;
        }
        let dt = c.dt.max(1e-4);
        let on = joint_mask(&mask);
        let up = c.up;
        let mut n = 0;
        for j in (0..c.ch.eff.len()).filter(|&j| on.get(j).copied().unwrap_or(false)) {
            let k = j + 1;
            let part = c.b.part(k);
            let now = c.b.frames[k];
            let t = c.incoming[k];
            let was = c.incoming_prev.get(k).copied().unwrap_or(t);
            let gap = t.translation - now.translation;
            let own = (t.translation - was.translation) / dt;
            let v = crate::extras_ysk_rdr2::brake(gap / dt, part.v, own, gap.normalize_or_zero(), gap.length(), lim_a);
            let a = crate::extras_ysk_rdr2::clamp3((v - part.v) / dt, lim_a) + up * (9.8 * grav);
            let err = crate::extras_ysk_rdr2::turn(t.rotation * now.rotation.inverse());
            let own_w = crate::extras_ysk_rdr2::turn(t.rotation * was.rotation.inverse()) / dt;
            let w = crate::extras_ysk_rdr2::brake(err / dt, part.w, own_w, err.normalize_or_zero(), err.length(), lim_w);
            let al = crate::extras_ysk_rdr2::clamp3((w - part.w) / dt, lim_w);
            c.out.key_vel.push((k, a * dt, al * dt));
            n += 1;
        }
        trace(|| format!("SoftKey: {n} parts driven at the incoming pose"));
    }))
}

/// configureSelfAvoidance (0x2a28ca4 -> 0x2a28a14), on both arms' limbs: read by every arm IK request (0x2a2bebc).
fn self_avoid(p: &mut P) -> Result<Box<dyn Extra>, Error> {
    let r = SelfAvoid {
        on: p.b("useSelfAvoidance", false)?, drag: p.b("overwriteDragReduction", false)?, fraction: p.f("torsoSwingFraction", 0.75)?,
        most: p.f("maxTorsoSwingAngleRad", 0.758)?, in_spine: p.b("selfAvoidIfInSpineBoundsOnly", false)?, amount: p.f("selfAvoidAmount", 0.5)?,
        twist: p.b("overwriteTwist", false)?, polar: p.b("usePolarPathAlgorithm", false)?, radius: p.f("radius", 0.3)?,
    };
    Ok(once(move |c| {
        c.ch.core.cfg.self_avoid = r;
        trace(|| format!("configureSelfAvoidance: {r:?}"));
    }))
}

/// ?b7960dec (0x2a4ce2c): the spin limiter a bullet arms (ctx+0x2bc8; 0x2a40518), `spin_limit` in the core's step.
fn spin_limit(p: &mut P) -> Result<Box<dyn Extra>, Error> {
    let rec = SpinRec {
        lim: [p.f("somersaultThresh", 2.0)?, p.f("cartwheelThresh", 2.0)?], gain: [p.f("somersaultDamp", 5.0)?, p.f("cartwheelDamp", 5.0)?],
        time: p.f("?7d54085f", 0.0)?, speed: p.f("?27795104", 0.1)?, left: -0.1,
    };
    Ok(once(move |c| {
        c.ch.core.cfg.spin = rec;
        trace(|| format!("spin limit: {rec:?}"));
    }))
}

/// setCharacterDamping (0x2a4d570): the limiter a contact arms (ctx+0x2ba0), `spin_limit` in the core's step.
fn damping(p: &mut P) -> Result<Box<dyn Extra>, Error> {
    let rec = SpinRec {
        lim: [p.f("somersaultThresh", 34.0)?, p.f("cartwheelThresh", 27.0)?], gain: [p.f("somersaultDamp", 0.0)?, p.f("cartwheelDamp", 0.0)?],
        time: p.f("vehicleCollisionTime", 0.0)?, speed: p.f("v2", 0.0)?, left: -0.1,
    };
    Ok(once(move |c| {
        c.ch.core.cfg.spin_hit = rec;
        trace(|| format!("spin limit on contact: {rec:?}"));
    }))
}

/// 0x2a1d9e4, one record: while its timer runs, slower than its speed, the angular momentum past its limits about the COM's
/// x and z axes is opposed at the pelvis, not proportional: -e gain / clamp(60 dt, 1, 60). Returns the torque.
pub fn spin_limit_step(r: &mut SpinRec, h: Vec3, a: Vec3, z: Vec3, speed: f32, dt: f32) -> Vec3 {
    if r.left <= 0.0 {
        return Vec3::ZERO;
    }
    r.left -= dt;
    if speed >= r.speed {
        return Vec3::ZERO;
    }
    let mut out = Vec3::ZERO;
    let div = (60.0 * dt).clamp(1.0, 60.0);
    for (axis, lim, gain) in [(a, r.lim[0], r.gain[0]), (z, r.lim[1], r.gain[1])] {
        let x = h.dot(axis);
        if x.abs() > lim {
            out -= axis * ((x - lim * x.signum()) * gain / div);
        }
    }
    out
}

/// setCharacterCollisions (0x2a4d0bc): held; `collisions` in the contact pass reads it.
fn collisions(p: &mut P) -> Result<Box<dyn Extra>, Error> {
    let r = CollRec {
        on: true, spin: p.f("spin", 0.0)?, max_speed: p.f("maxVelocity", 8.0)?, foot_slip: p.b("footSlip", false)?, scale: [p.f("gsfricscale1", 8.0)?, p.f("gsfricscale2", 0.2)?],
        cap: p.f("?b11746eb", -0.2)?, spine: p.b("applyToSpine", false)?, thighs: p.b("applyToThighs", false)?, clavicles: p.b("applyToClavicles", false)?, upper: p.b("applyToUpperArms", false)?, all: p.b("applyToAll", false)?,
    };
    let _ = p.s("vehicleflags", "");
    Ok(once(move |c| {
        c.ch.core.cfg.coll = r;
        trace(|| format!("setCharacterCollisions: {r:?}"));
    }))
}

/// shotNewBullet (0x2a546c8): a new bullet record for the behaviours asked (shot's 0x2ad4a10, duck's 0x2ad2bc8), without a
/// new impulse: the point from the message, in the part's space if it says so, else the part's centre.
fn new_bullet(p: &mut P) -> Result<Box<dyn Extra>, Error> {
    let part = crate::extras_ysk_rdr2::rp(p.i("bodyPart", 0)?.max(0) as usize);
    let local = p.b("localHitPointInfo", false)?;
    let at = p.get("hitPoint").is_some().then(|| p.v("hitPoint", Vec3::ZERO)).transpose()?;
    let n = p.v("normal", Vec3::Z)?;
    let (shot, duck, third) = (p.b("?06c06365", false)?, p.b("?4d7dfd89", false)?, p.b("?2f471f82", false)?);
    Ok(once(move |c| {
        let k = part.min(c.b.frames.len().saturating_sub(1));
        let f = c.b.frames[k];
        let point = match (at, local) {
            (Some(q), true) => f.translation + f.rotation * q,
            (Some(q), false) => q,
            (None, _) => f.translation,
        };
        let _ = n;
        if shot || duck || third {
            c.ch.core.bullet_new = Some((k, point, Vec3::ZERO));
            c.out.shot_bullet = shot.then_some((k, point));
        }
        trace(|| format!("shotNewBullet: part {k} at {point}, shot {shot} duck {duck}"));
    }))
}

/// ?e35a5d1a's handler returns at once on a biped: its records are the quadruped legs' (`rdr2_quadruped.md` 15.2).
fn limb_limit(_p: &mut P) -> Result<Box<dyn Extra>, Error> {
    Ok(once(move |_| trace(|| "?e35a5d1a: nothing on a biped, as the code".to_string())))
}
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
