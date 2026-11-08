//! RDR2 core: pointGun from RDR2's code (rdr2-decomp/study/rdr2_pointgun.md, gone through against the code), an extra
//! message (Delirium has no pointGun of its own: GTA IV's shot task only starts pointArm). With the core off it does
//! what Delirium's did: pointArm at the gun hand's target. Arms 0 left, 1 right.
//!
//! Ours, because Delirium has no weapons: the hands' weapon states come from `weapon` / `leftWeapon` or the character's
//! held weapon; the gun's axis in the hand is the hand's -row 1, which is what RDR2's own facing test for a pistol
//! compares it with (the game's registerWeapon matrix has no source here); weapon attachment constraints, gun mass and
//! the rifle grip's re-aim on the muzzle (no such things) aren't run, though their parameters are read. The reach request
//! (type 9) runs RDR2's two solvers, 0x2a430f4 with its chain 0x2a44338, and 0x2a43794 (usePistolIK's); pointGun is their
//! only user in the game. The hand-to-hand and rifle constraints are Delirium's grip ropes.
use crate::extras::{Extra, P};
use crate::extras_ysk_rdr2::{note, parts, rp};
use crate::msg::{Error, Kind, Message, Param};
use crate::nmdbm::M34;
use crate::nmrs::{CLAVICLE, ELBOW, SHOULDER, SPINE, WRIST};
use crate::nmshot::{chest_part, limb, Ctx};
use bevy_math::{Mat3, Quat, Vec3};

macro_rules! p {
    ($n:literal, $k:ident, $d:literal) => { Param { name: $n, kind: Kind::$k, used: true, doc: $d } };
}

pub static MESSAGES: [Message; 1] = [
    Message { name: "pointGun", params: PARAMS },
];

const PARAMS: &[Param] = &[
    p!("weapon", Int, "ours: the right hand's weapon, 0 none, 1 pistol, 2 rifle, 3 put away, 4 unusable; -1 the character's held weapon (-1)"),
    p!("leftWeapon", Int, "ours: the left hand's, likewise; -1 from the character (-1)"),
    p!("fireWeapon", Bool, "ours: with no start, a shot (the arm relaxes)"),
    p!("leftHandTarget", Vec3, "where the left hand's gun points, world (zero)"),
    p!("rightHandTarget", Vec3, "where the right hand's gun points, world (zero)"),
    p!("leftHandTargetInstance", Int, "RDR2's uncracked 1c7b6b2b: the object the left target is on, its frame and motion; -1 the world (-1)"),
    p!("rightHandTargetInstance", Int, "RDR2's uncracked 5606fc8c: likewise the right (-1)"),
    p!("leftHandParentEffector", Int, "RDR2's joint the left aim starts from; -1 its shoulder (-1)"),
    p!("rightHandParentEffector", Int, "likewise the right (-1)"),
    p!("leftHandParentOffset", Vec3, "added to it, in the chest's frame (zero)"),
    p!("rightHandParentOffset", Vec3, "(zero)"),
    p!("leadTarget", Float, "lead a moving target by this (0)"),
    p!("lefthandweapondistance", Float, "the left hand this far out; < 0 measured (-1)"),
    p!("righthandweapondistance", Float, "(-1)"),
    p!("armStiffness", Float, "a pointing arm's (14)"),
    p!("armStiffnessDetSupport", Float, "with the support hand off the gun (8)"),
    p!("armDamping", Float, "(1)"),
    p!("supportConstraint", Int, "0 none, 1 hand-to-hand constraint, 2 a pull and a constraint (1)"),
    p!("oneHandedPointing", Int, "1 a supported pistol may point alone, 2 a rifle may, 3 any; 0, 2 let a pistol arm take neutral poses while the support may connect (1)"),
    p!("constraintMinDistance", Float, "what the hand constraint shortens to, m (0.015)"),
    p!("makeConstraintDistance", Float, "the support hand this near makes it, m (0.1)"),
    p!("reduceConstraintLengthVel", Float, "m/s (1.5)"),
    p!("breakingStrength", Float, "of the hand constraint; < 0 never (-1)"),
    p!("brokenSupportTime", Float, "s without support after it breaks (1)"),
    p!("brokenToSideProb", Float, "the chance the arms go to the side meanwhile (0.5)"),
    p!("connectAfter", Float, "s pointing out of the support's reach until it may connect (0.7)"),
    p!("connectFor", Float, "s (0.55)"),
    p!("clavicleBlend", Float, "the gun arm's clavicle toward the incoming pose (0)"),
    p!("clavicleblendsupport", Float, "the support arm's, once connected (1)"),
    p!("elbowAttitude", Float, "the gun arm's elbow swing (0.3)"),
    p!("elbowattitudesupport", Float, "the support arm's (0.1)"),
    p!("useIncomingTransforms", Bool, "grip and distances from the incoming pose; off, or with none, RDR2's hard-coded grips (true)"),
    p!("measureParentOffset", Bool, "the aim's anchor moved to the hand's height in the incoming pose (true)"),
    p!("fireWeaponRelaxTime", Float, "s the arm gives after a shot (0.4)"),
    p!("fireWeaponRelaxAmount", Float, "(0.5)"),
    p!("fireWeaponRelaxDistance", Float, "the hand comes in this far, m (0.05)"),
    p!("gravityOpposition", Float, "(1)"),
    p!("gravOppDetachedSupport", Float, "(0.5)"),
    p!("massMultDetachedSupport", Float, "the gun's mass with the support off; no gun here, read only (0.1)"),
    p!("weaponStrengthDetached", Float, "RDR2's uncracked 5274f454, the gun attachment's strength with the support off (0.3)"),
    p!("weaponStrengthHolster", Float, "RDR2's uncracked c53bbf28, the attachment's strength in state 3 (0.5)"),
    p!("weaponStrengthAway", Float, "RDR2's uncracked c53bbf28, with the weapon put away; no gun here, read only (0.5)"),
    p!("constrainRifle", Bool, "the rifle hand held near the chest by a constraint (true)"),
    p!("rifleConstraintMinDistance", Float, "m (0.2)"),
    p!("supportReachScale", Float, "RDR2's uncracked 6f5df967: the support reaches this x arm length (1)"),
    p!("clampsupport", Float, "the support hand's reach, x arm length (1.5)"),
    p!("pointgun", Bool, "may point (true)"),
    p!("enableRight", Bool, "(true)"),
    p!("enableLeft", Bool, "(true)"),
    p!("disableArmCollisions", Bool, "(false)"),
    p!("disableRifleCollisions", Bool, "(false)"),
    p!("poseUnusedGunArm", Bool, "a gun arm that can't point takes a neutral pose (false)"),
    p!("poseUnusedSupportArm", Bool, "(false)"),
    p!("poseUnusedOtherArm", Bool, "(false)"),
    p!("maxAngleAcross", Float, "degrees across the body it may point (90)"),
    p!("maxAngleAway", Float, "degrees away from it (90)"),
    p!("fallingLimits", Int, "1 while falling, 2 always: the limits below too (0)"),
    p!("acrossLimit", Float, "degrees (90)"),
    p!("awayLimit", Float, "(90)"),
    p!("upLimit", Float, "(90)"),
    p!("downLimit", Float, "(45)"),
    p!("errorThreshold", Float, "rad off the aim that counts as not on it (0.3926)"),
    p!("twistoffsetrifle", Float, "the spine twist's offset for a rifle (-0.6)"),
    p!("twistoffsetpistol", Float, "for a pistol (-0.6)"),
    p!("probelength", Float, "> 0: a ray this long along the aim; a hit takes the arm neutral (-1)"),
    p!("usePistolIK", Bool, "a one-handed arm uses RDR2's pistol solver (true)"),
    p!("pistolfromhip", Bool, "the pistol arm's clavicle as a rifle's (false)"),
    p!("useSpineTwist", Bool, "(true)"),
    p!("useTurnToTarget", Bool, "the balancer turns him to the target (false)"),
    p!("stepturnwait", Float, "s between step turns (0.5)"),
    p!("stepTurnFacing", Float, "RDR2's uncracked fc04dc23: the step turn's facing cosine (0.9)"),
    p!("stepTurn", Bool, "(false)"),
    p!("useHeadLook", Bool, "(true)"),
    p!("supportpistol", Bool, "the free hand supports a pistol too (false)"),
    p!("alwaysSupport", Bool, "(false)"),
    p!("movesupport", Bool, "the grip slides to where the support can reach (false)"),
    p!("alwayspoint", Bool, "aim clamped into the limits instead of not pointing (false)"),
    p!("shoulderLeanPistol", Bool, "RDR2's uncracked 64a23851: a pointing pistol arm's shoulder lean2 limit at least 1.2 (false)"),
    p!("supportacross", Bool, "the support shoulder's limits at least 1.5 while it supports (false)"),
    p!("alwaystwist", Bool, "(false)"),
    p!("allowShotLooseness", Bool, "the arm's muscle gains only lowered (false)"),
    p!("pistolNeutralType", Int, "0, 1, 2: the pistol arm's neutral pose (0)"),
    p!("neutralPoint4Pistols", Bool, "a pistol that can't point goes to point2Connect (false)"),
    p!("neutralPoint4Rifle", Bool, "a rifle likewise (true)"),
    p!("checkNeutralPoint", Bool, "only if the limits allow it (false)"),
    p!("rifleFall", Int, "falling with a rifle: 1 the arms neutral, 2 if he'll land on his side or head (0)"),
    p!("fallingSupport", Int, "with fallingTypeSupport met: 0 support off, 2 unless the constraint holds, 3 unless gripped (1)"),
    p!("fallingTypeSupport", Int, "when falling counts: 1 always, 2 landing on his side or head, 3 in contact, 4 and 5 both ways (0)"),
    p!("point2Side", Vec3, "in the COM's frame (5,-5,-2)"),
    p!("point2Connect", Vec3, "in the COM's frame (-1,-0.9,-0.2)"),
    p!("supportAllowedProb", Float, "RDR2's uncracked 94aeaba2: the chance a pointing arm's support may stay off (1)"),
    p!("add2WeaponDistSide", Float, "(0.3)"),
    p!("add2WeaponDistConnect", Float, "(0)"),
    p!("animSource", Int, "RDR2's incoming buffer; Delirium has one: read only (2)"),
    p!("measurecontinuous", Bool, "the parent offsets and distances each step too (false)"),
    p!("measureAlongHand", Bool, "RDR2's uncracked a5fe6796: the parent offset along the hand's line instead of its height (false)"),
    p!("pistolIkArg", Float, "RDR2's uncracked 08b1dc5b: the pistol solver bends the upper arm this far off the line to the target, rad (0.05)"),
    p!("constraintStrength", Float, "the support pull (2)"),
    p!("pullRadius", Float, "RDR2's uncracked fb1749eb: the pull starts at 3 x this, m (0.1)"),
    p!("timeWarpActive", Bool, "(false)"),
    p!("timeWarpStrengthScale", Float, "(1)"),
    p!("oriStiff", Float, "a hand orientation PD (0)"),
    p!("oriDamp", Float, "(0)"),
    p!("posStiff", Float, "a forearm position PD; with a constraint, the support hand's too (0)"),
    p!("posDamp", Float, "(0)"),
];

pub fn build(name: &str, p: &mut P) -> Option<Result<Box<dyn Extra>, Error>> {
    match name {
        "pointGun" => Some(PointGun::new(p).map(|e| Box::new(e) as _)),
        _ => None,
    }
}

/// The neutral pose table (0x2ae0a70): clavicle, shoulder (lean1, lean2, twist), elbow, wrist.
fn pose(id: i32) -> Option<([f32; 3], [f32; 3], f32, [f32; 3])> {
    match id {
        2 | 3 | 8 => Some(([0.2, 0.0, 0.0], [0.53, 0.35, -0.2], 2.45, [0.0; 3])),
        4 | 11 => Some(([0.0518, -0.0399, -0.0154], [-0.0012, 0.1867, 0.5365], 0.6366, [0.1214, 0.041, -0.1965])),
        5 | 6 => Some(([-0.16, -0.12, 0.2], [0.318, 0.07, 0.583], 1.78, [-0.37, 0.68, -0.3])),
        9 | 10 => Some(([0.33, 0.04, 0.04], [0.17, 0.35, 0.31], 0.66, [0.0; 3])),
        _ => None,
    }
}

/// 0x2ad0e10's hard-coded grips, the support hand in the weapon's frame: rows, then position. [rifle, pistol]
const GRIP: [([Vec3; 3], Vec3); 2] = [
    ([Vec3::new(-0.3615, -0.3111, 0.8789), Vec3::new(-0.7789, 0.6189, -0.1014), Vec3::new(-0.5124, -0.7213, -0.4660)], Vec3::new(0.2551, 0.0161, 0.0121)),
    ([Vec3::new(-0.4808, -0.8607, -0.1676), Vec3::new(-0.7854, 0.5077, -0.3542), Vec3::new(0.3900, -0.0387, -0.9200)], Vec3::new(-0.0703, 0.0198, -0.0676)),
];

/// The gun's rows from a hand's (ours, RDR2's weapon matrix has no source): muzzle -row 1, RDR2's own facing test; the roll is
/// the one a hanging hand has with its thumb up the gun, the aim frame's row 2: on this rig the thumb is -row 2 on both hands.
fn gun_rows(h: &M34) -> [Vec3; 3] {
    [-h.b, -h.a, -h.c]
}

/// A hand's frame whose gun has these rows.
fn hand_for(gun: [Vec3; 3], d: Vec3) -> M34 {
    M34 { a: -gun[1], b: -gun[0], c: -gun[2], d }
}

fn mul(g: &[Vec3; 3], r: &[Vec3; 3]) -> [Vec3; 3] {
    std::array::from_fn(|i| r[0] * g[i].x + r[1] * g[i].y + r[2] * g[i].z)
}

fn on(r: &[Vec3; 3], d: Vec3, p: Vec3) -> Vec3 {
    d + r[0] * p.x + r[1] * p.y + r[2] * p.z
}

fn inv_rows(m: &[Vec3; 3]) -> [Vec3; 3] {
    [Vec3::new(m[0].x, m[1].x, m[2].x), Vec3::new(m[0].y, m[1].y, m[2].y), Vec3::new(m[0].z, m[1].z, m[2].z)]
}

fn quat_of(m: &M34) -> Quat {
    Quat::from_mat3(&Mat3::from_cols(m.a, m.b, m.c))
}

fn unit(v: Vec3) -> Vec3 {
    v.normalize_or_zero()
}

fn v3s(v: Vec3) -> String {
    format!("{},{},{}", v.x, v.y, v.z)
}

fn acos(x: f32) -> f32 {
    x.clamp(-1.0, 1.0).acos()
}

#[derive(Clone, Debug)]
struct Params {
    weapon: [i32; 2],
    target: [Vec3; 2],
    instance: [i32; 2],
    parent: [i32; 2],
    parent_offset: [Vec3; 2],
    lead: f32,
    hand_dist: [f32; 2],
    arm_k: f32,
    arm_k_det: f32,
    arm_d: f32,
    support_constraint: i32,
    one_handed: i32,
    con_min: f32,
    make_con: f32,
    con_vel: f32,
    breaking: f32,
    broken_time: f32,
    broken_side: f32,
    connect_after: f32,
    connect_for: f32,
    clav_blend: [f32; 2],
    elbow: [f32; 2],
    use_incoming: bool,
    measure_parent: bool,
    relax: (f32, f32, f32),
    /// +0x120, +0x124: the gun attachment's strength factors
    weapon_strength: [f32; 2],
    grav: (f32, f32),
    constrain_rifle: bool,
    rifle_min: f32,
    reach_scale: f32,
    clamp_support: f32,
    point: bool,
    enable: [bool; 2],
    no_arm_col: bool,
    no_rifle_col: bool,
    pose_unused: [bool; 3],
    max_across: f32,
    max_away: f32,
    falling_limits: i32,
    limits: [f32; 4],
    error_threshold: f32,
    twist_offset: (f32, f32),
    probe: f32,
    pistol_ik: (bool, f32),
    from_hip: bool,
    use_twist: bool,
    use_turn: bool,
    step_wait: f32,
    step_facing: f32,
    step_turn: bool,
    use_look: bool,
    support_pistol: bool,
    always_support: bool,
    move_support: bool,
    always_point: bool,
    shoulder_pistol: bool,
    support_across: bool,
    always_twist: bool,
    loose: bool,
    pistol_neutral: i32,
    neutral_point: (bool, bool, bool),
    rifle_fall: i32,
    falling_support: i32,
    falling_type: i32,
    point2side: Vec3,
    point2connect: Vec3,
    support_prob: f32,
    add_side: f32,
    add_connect: f32,
    continuous: bool,
    along_hand: bool,
    strength: f32,
    pull_r: f32,
    time_warp: (bool, f32),
    ori: (f32, f32),
    pos: (f32, f32),
}

impl Params {
    fn new(p: &mut P) -> Result<Params, Error> {
        // read only: nothing here to give them to
        let _ = (p.f("massMultDetachedSupport", 0.1)?, p.f("weaponStrengthDetached", 0.3)?, p.f("weaponStrengthAway", 0.5)?);
        let _ = p.i("animSource", 2)?;
        Ok(Params {
            weapon: [p.i("leftWeapon", -1)?, p.i("weapon", -1)?],
            target: [p.v("leftHandTarget", Vec3::ZERO)?, p.v("rightHandTarget", Vec3::ZERO)?],
            instance: [p.i("leftHandTargetInstance", -1)?, p.i("rightHandTargetInstance", -1)?],
            parent: [p.i("leftHandParentEffector", -1)?, p.i("rightHandParentEffector", -1)?],
            parent_offset: [p.v("leftHandParentOffset", Vec3::ZERO)?, p.v("rightHandParentOffset", Vec3::ZERO)?],
            lead: p.f("leadTarget", 0.0)?,
            hand_dist: [p.f("lefthandweapondistance", -1.0)?, p.f("righthandweapondistance", -1.0)?],
            arm_k: p.f("armStiffness", 14.0)?, arm_k_det: p.f("armStiffnessDetSupport", 8.0)?, arm_d: p.f("armDamping", 1.0)?,
            support_constraint: p.i("supportConstraint", 1)?, one_handed: p.i("oneHandedPointing", 1)?,
            con_min: p.f("constraintMinDistance", 0.015)?, make_con: p.f("makeConstraintDistance", 0.1)?, con_vel: p.f("reduceConstraintLengthVel", 1.5)?,
            breaking: p.f("breakingStrength", -1.0)?, broken_time: p.f("brokenSupportTime", 1.0)?, broken_side: p.f("brokenToSideProb", 0.5)?,
            connect_after: p.f("connectAfter", 0.7)?, connect_for: p.f("connectFor", 0.55)?,
            clav_blend: [p.f("clavicleBlend", 0.0)?, p.f("clavicleblendsupport", 1.0)?],
            elbow: [p.f("elbowAttitude", 0.3)?, p.f("elbowattitudesupport", 0.1)?],
            use_incoming: p.b("useIncomingTransforms", true)?, measure_parent: p.b("measureParentOffset", true)?,
            weapon_strength: [p.f("weaponStrengthDetached", 0.3)?, p.f("weaponStrengthHolster", 0.5)?],
            relax: (p.f("fireWeaponRelaxTime", 0.4)?, p.f("fireWeaponRelaxAmount", 0.5)?, p.f("fireWeaponRelaxDistance", 0.05)?),
            grav: (p.f("gravityOpposition", 1.0)?, p.f("gravOppDetachedSupport", 0.5)?),
            constrain_rifle: p.b("constrainRifle", true)?, rifle_min: p.f("rifleConstraintMinDistance", 0.2)?,
            reach_scale: p.f("supportReachScale", 1.0)?, clamp_support: p.f("clampsupport", 1.5)?,
            point: p.b("pointgun", true)?,
            enable: [p.b("enableLeft", true)?, p.b("enableRight", true)?],
            no_arm_col: p.b("disableArmCollisions", false)?, no_rifle_col: p.b("disableRifleCollisions", false)?,
            pose_unused: [p.b("poseUnusedGunArm", false)?, p.b("poseUnusedSupportArm", false)?, p.b("poseUnusedOtherArm", false)?],
            max_across: p.f("maxAngleAcross", 90.0)?.to_radians(), max_away: p.f("maxAngleAway", 90.0)?.to_radians(),
            falling_limits: p.i("fallingLimits", 0)?,
            limits: [p.f("acrossLimit", 90.0)?, p.f("awayLimit", 90.0)?, p.f("upLimit", 90.0)?, p.f("downLimit", 45.0)?],
            error_threshold: p.f("errorThreshold", 0.3926)?,
            twist_offset: (p.f("twistoffsetrifle", -0.6)?, p.f("twistoffsetpistol", -0.6)?),
            probe: p.f("probelength", -1.0)?, pistol_ik: (p.b("usePistolIK", true)?, p.f("pistolIkArg", 0.05)?), from_hip: p.b("pistolfromhip", false)?,
            use_twist: p.b("useSpineTwist", true)?, use_turn: p.b("useTurnToTarget", false)?,
            step_wait: p.f("stepturnwait", 0.5)?, step_facing: p.f("stepTurnFacing", 0.9)?, step_turn: p.b("stepTurn", false)?,
            use_look: p.b("useHeadLook", true)?,
            support_pistol: p.b("supportpistol", false)?, always_support: p.b("alwaysSupport", false)?, move_support: p.b("movesupport", false)?,
            always_point: p.b("alwayspoint", false)?, shoulder_pistol: p.b("shoulderLeanPistol", false)?, support_across: p.b("supportacross", false)?,
            always_twist: p.b("alwaystwist", false)?, loose: p.b("allowShotLooseness", false)?,
            pistol_neutral: p.i("pistolNeutralType", 0)?,
            neutral_point: (p.b("neutralPoint4Pistols", false)?, p.b("neutralPoint4Rifle", true)?, p.b("checkNeutralPoint", false)?),
            rifle_fall: p.i("rifleFall", 0)?, falling_support: p.i("fallingSupport", 1)?, falling_type: p.i("fallingTypeSupport", 0)?,
            point2side: p.v("point2Side", Vec3::new(5.0, -5.0, -2.0))?, point2connect: p.v("point2Connect", Vec3::new(-1.0, -0.9, -0.2))?,
            support_prob: p.f("supportAllowedProb", 1.0)?,
            add_side: p.f("add2WeaponDistSide", 0.3)?, add_connect: p.f("add2WeaponDistConnect", 0.0)?,
            continuous: p.b("measurecontinuous", false)?, along_hand: p.b("measureAlongHand", false)?,
            strength: p.f("constraintStrength", 2.0)?, pull_r: p.f("pullRadius", 0.1)?,
            time_warp: (p.b("timeWarpActive", false)?, p.f("timeWarpStrengthScale", 1.0)?),
            ori: (p.f("oriStiff", 0.0)?, p.f("oriDamp", 0.0)?), pos: (p.f("posStiff", 0.0)?, p.f("posDamp", 0.0)?),
        })
    }
}

/// An arm's record (R at this+0x200): field names for the note's offsets.
#[derive(Clone, Copy, Debug)]
struct Arm {
    /// +0x170 the target (the connect point or the clamped aim overwrite it), +0x180 kept as it came, +0x190 last step's
    target: Vec3,
    look: Vec3,
    prev: Vec3,
    /// +0x1a0, +0x1c0
    led: Vec3,
    vel: Vec3,
    /// +0x160 the aim's anchor, +0x1b0 the aim
    anchor: Vec3,
    aim: Vec3,
    /// +0x1e0, +0x1d0: the forearm PD's target speed and the last new target
    pd_vel: Vec3,
    pd_at: Vec3,
    /// +0x20 the hand's target frame
    goal: M34,
    /// +0xe0 the grip on the real gun hand, +0x120 on the gun hand's target, +0x60 the support request's
    grip: M34,
    alt: M34,
    req_m: M34,
    /// +0x1f4 the fire relax timer, +0x1f8 its factor
    relax_t: f32,
    relax: f32,
    /// +0x1fc the hand's distance from its anchor, +0x200 this step's
    dist: f32,
    used: f32,
    /// +0x21c the support hand from its grip, +0x210 / +0x214 the two grips from the shoulder, +0x218 kept
    d_hand: f32,
    d_grip: f32,
    d_alt: f32,
    /// +0x220 state (0 none, 3 pointing, 4 supporting, 6 pointing off the aim, 10 at the connect point, 0xb neutral), last
    /// step's, +0x228 requested, +0x22c pose
    state: i32,
    was: i32,
    req: i32,
    pose: i32,
    /// +0x235 pointing now, +0x236 near, +0x237 / +0x238 within reach, +0x23b crossed, +0x23c the gun faces the right way
    pointing: bool,
    near: bool,
    reach: bool,
    reach_kept: bool,
    crossed: bool,
    facing: bool,
}

impl Default for Arm {
    fn default() -> Arm {
        let m = M34 { a: Vec3::X, b: Vec3::Y, c: Vec3::Z, d: Vec3::ZERO };
        Arm { target: Vec3::ZERO, look: Vec3::ZERO, prev: Vec3::ZERO, led: Vec3::ZERO, vel: Vec3::ZERO, anchor: Vec3::ZERO, aim: Vec3::ZERO,
              pd_vel: Vec3::ZERO, pd_at: Vec3::ZERO, goal: m, grip: m, alt: m, req_m: m, relax_t: 0.0, relax: 1.0, dist: 0.0, used: 0.0,
              d_hand: 0.0, d_grip: 0.0, d_alt: 0.0, state: 0, was: 0, req: 0, pose: 1, pointing: false, near: false, reach: false,
              reach_kept: false, crossed: false, facing: true }
    }
}

pub struct PointGun {
    p: Params,
    arms: [Arm; 2],
    started: bool,
    /// +0x774 arm length, +0x770 the grip's ramp, +0x778 the rifle constraint's length (None: none), the hand constraint's
    arm_len: f32,
    /// the shot's hand-over module (+0x7b8)
    held_off: bool,
    ramp: f32,
    rifle_len: Option<f32>,
    hand_len: Option<f32>,
    /// +0x6d0 the grip in the weapon's frame (rows), +0x700 its position now, +0x740 measured
    grip_rows: [Vec3; 3],
    grip_pos: Vec3,
    grip_meas: Vec3,
    /// +0x760 / +0x750 the measured parent offsets
    measured: [Vec3; 2],
    /// +0x77c, +0x780 side, +0x784 broken, +0x788 / +0x78c connect, +0x790 extra distance, +0x794 turn, +0x798 look timers
    tw: f32,
    side_t: f32,
    broken_t: f32,
    conn_t: f32,
    conn_for: f32,
    extra: f32,
    turn_t: f32,
    look_t: f32,
    /// +0x79c the side pose falling, +0x7a0 the last support feedback, +0x7a8 / +0x7ac gun and support hands
    side_pose: i32,
    fed: i32,
    g: usize,
    s: usize,
    /// +0x7b0 per arm (RDR2's is one flag, set by the arm that ran last), +0x7b3 / +0x7b4 falling flags, +0x7b5 support on, +0x7b7
    probe_hit: [bool; 2],
    fall_rifle: bool,
    fall_support: bool,
    support_on: bool,
    duck_neutral: bool,
    /// +0x7b9 two-handed, +0x7ba the gun hand below the chest, +0x7bb support mode, +0x7bc / +0x7bd look draws
    two: bool,
    below: bool,
    support_mode: bool,
    look_r: [bool; 2],
    /// +0x7be connected, +0x7bf the support exists (a constraint or a pull), +0x7c0 allowed, +0x7c1
    connected: bool,
    exists: bool,
    allowed: bool,
    connected_kept: bool,
    /// +0xb4 a new target; lassoed by the chest: yanked's coupling
    new_target: bool,
    yanked: bool,
    /// the pistol solver's limit factor, with its hysteresis (RDR2's global at 0x3cc7ca8, 1.15 to start)
    pik_limit: f32,
    /// the objects: id, place, turn, velocity, spin
    objects: Vec<(i32, Vec3, Quat, Vec3, Vec3)>,
    looking: bool,
    dt: f32,
    ws: [i32; 2],
    /// R+0x40c: no more recoil on this arm until it's out
    inhibit: [f32; 2],
    ropes: Vec<(usize, Vec3, Vec3, f32, f32)>,
    pair_off: Vec<(u32, u32)>,
    block: [bool; 2],
    said: u64,
    out: Vec<String>,
    fallback: Option<bool>,
}

impl PointGun {
    fn new(p: &mut P) -> Result<PointGun, Error> {
        let fire = p.b("fireWeapon", false)?;
        let mut g = PointGun {
            p: Params::new(p)?, arms: [Arm::default(); 2], started: false, arm_len: 0.0, held_off: false, ramp: -1.0, rifle_len: None, hand_len: None,
            grip_rows: [Vec3::X, Vec3::Y, Vec3::Z], grip_pos: Vec3::ZERO, grip_meas: Vec3::ZERO, measured: [Vec3::ZERO; 2], tw: 1.0,
            side_t: 0.0, broken_t: 0.0, conn_t: 0.0, conn_for: 0.0, extra: 0.0, turn_t: 0.0, look_t: 0.0, side_pose: 1, fed: 0, g: 1, s: 0,
            probe_hit: [false; 2], fall_rifle: false, fall_support: false, support_on: true, duck_neutral: false, two: false, below: false,
            support_mode: false, look_r: [false, true], connected: false, exists: false, allowed: true, connected_kept: true,
            new_target: true, yanked: false, pik_limit: 1.15, objects: vec![], looking: false, dt: 1.0 / 60.0, ws: [0; 2], inhibit: [0.0; 2], ropes: vec![], pair_off: vec![], block: [false; 2],
            said: 0, out: vec![], fallback: None,
        };
        if fire {
            g.fire();
        }
        Ok(g)
    }

    /// A branch ran: said once, with RDR2_TRACE (proof, no effect on the physics).
    fn ran(&mut self, bit: u32, what: &str) {
        if self.said >> bit & 1 == 0 {
            self.said |= 1 << bit;
            note(format_args!("pointGun: {what}"));
        }
    }

    /// The hands' weapon states (left, right): 0 none, 1 pistol, 2 rifle, 3, 4.
    /// 0x2accc7c: fireWeapon on the arm. States 0 and 4 hold nothing to fire. Past its inhibit clock the recoil goes on the hand, or on the clavicle and the
    /// hand by split, in the hand's space (through registerWeapon's gunToHand, ctx+0x2d00), and on the other hand when it grips too. Then the
    /// arm relaxes, and a gripping other arm's four joints take the arm's stiffness by .7, .6, .5, .3. Whatever the clock, the relax restarts.
    fn fire_rdr2(&mut self, c: &mut Ctx, r: crate::nmrs::FireRec) {
        let i = r.hand;
        let state = c.ch.core.arm_state[i];
        if state == 0 || state == 4 {
            note(format_args!("fireWeapon: arm {i} state {state}, nothing to fire"));
            return;
        }
        let (l, lo) = (crate::nmshot::limb(i), crate::nmshot::limb(1 - i));
        let two = self.support_mode && self.gripped(1 - i) && self.arms[1 - i].state == 4;
        if self.inhibit[i] <= 0.0 {
            self.inhibit[i] = r.inhibit;
            let hand = c.b.part(l.p[3]);
            // the hand's matrix times the gun's (ctx+0x2d00: gunToHand), then the message's direction in the gun
            let g = c.ch.core.weapon[i].to_hand;
            let dg = g[0] * r.direction.x + g[1] * r.direction.y + g[2] * r.direction.z;
            let d = (hand.m.a * dg.x + hand.m.b * dg.y + hand.m.c * dg.z) * c.dt;
            let other = c.b.part(lo.p[3]).m.d;
            let other_gets = |f: Vec3| -> Option<Vec3> {
                if !two { None } else if r.other > 0.0 && r.other <= 1.0 { Some(f * r.other) } else if r.other < 0.0 { Some(-f * r.other) } else { None }
            };
            let hand_f = |f: Vec3| if r.other > 0.0 && r.other <= 1.0 { f * (1.0 - r.other) } else { f };
            let f = if r.at_clavicle {
                c.out.impulse.push((l.p[0], c.b.joint_pos(l.e[1]), d * (r.strength * (1.0 - r.split))));
                d * (r.strength * r.split)
            } else {
                d * r.strength
            };
            c.out.impulse.push((l.p[3], hand.m.d, hand_f(f)));
            if let Some(g) = other_gets(f) {
                c.out.impulse.push((lo.p[3], other, g));
            }
            note(format_args!("fireWeapon: recoil {:.1} N s on arm {i}, clavicle {}, the other hand {two}", f.length(), r.at_clavicle));
        }
        self.arms[i].relax_t = self.p.relax.0;
        let k = (1.0 - self.p.relax.1) * self.p.arm_k;
        c.ch.set_stiffness(k, self.p.arm_d, ["ul", "ur"][i], None);
        if two {
            crate::extras_core_rdr2::trace_first("fireWeapon: the other arm's four joints softened, .7, .6, .5, .3");
            let f = self.arms[1 - i].relax * self.p.arm_k;
            for (j, m) in lo.e.iter().zip([0.7, 0.6, 0.5, 0.3]) {
                let (s, d) = (f * m, self.p.arm_d);
                c.ch.eff[*j].strength = s * s;
                c.ch.eff[*j].damping = 2.0 * s * d;
            }
        }
    }

    fn states(&self, c: &Ctx) -> [i32; 2] {
        // a registered weapon's state, setWeaponMode's or 0x2a4e830's, over the message's
        if c.ch.core.weapon_bits != 0 {
            return c.ch.core.arm_state;
        }
        let right = match self.p.weapon[1] {
            w @ 0.. => w.min(4),
            _ if c.ch.busy[1] => if c.ch.busy[0] { 2 } else { 1 },
            _ => 0,
        };
        [self.p.weapon[0].clamp(0, 4), right]
    }

    /// 0x2ac8860: the gun hand (right unless only the left holds one) and the support hand.
    fn hands(ws: [i32; 2]) -> (usize, usize) {
        let armed = |s: i32| s != 0 && s != 4;
        if !armed(ws[1]) && armed(ws[0]) { (0, 1) } else { (1, 0) }
    }

    /// 0x2ac4620: the arm may be used.
    fn enabled(&self, i: usize) -> bool {
        self.p.enable[i] && self.ws[i] != 4
    }

    /// Falling as the arms mean it: the balancer isn't holding him, or the shot's fall to knees is on.
    fn falling(c: &Ctx) -> bool {
        !c.upright || c.ch.core.ftk.on
    }

    /// The hands' frames in the incoming pose, if it has one to use.
    fn incoming(&self, c: &Ctx) -> Option<Vec<M34>> {
        let on = self.p.use_incoming && c.incoming.len() >= c.b.frames.len() && c.incoming.iter().any(|t| t.translation != Vec3::ZERO);
        on.then(|| {
            let r = crate::nmrs::incoming_rows(&c.b.doll.links, c.incoming);
            c.incoming.iter().zip(r).map(|(t, r)| M34 { a: r[0], b: r[1], c: r[2], d: t.translation }).collect()
        })
    }

    /// 0x2ad0e10: the grip: measured from the incoming pose, else (useIncomingTransforms off, or Delirium has none) RDR2's hard-coded.
    fn measure(&mut self, c: &Ctx) {
        self.measured = [Vec3::ZERO; 2];
        let Some(m) = self.incoming(c) else {
            let k = match self.ws[self.g] { 2 => 0, 1 => 1, _ => return };
            (self.grip_rows, self.grip_meas) = GRIP[k];
            self.grip_pos = self.grip_meas;
            self.ran(1, "the grip is RDR2's hard-coded one (no incoming pose)");
            return;
        };
        self.ran(2, "the grip and hand distances measured from the incoming pose");
        if self.p.measure_parent {
            for i in 0..2 {
                let l = limb(i);
                let (hand, shoulder, chest) = (m[l.p[3]], m[l.p[1]].d, m[chest_part()]);
                let v = if self.p.along_hand {
                    let along = (shoulder - hand.d).dot(hand.b);
                    (if along > 0.1 { hand.d + hand.b * along } else { Vec3::ZERO }) - shoulder
                } else {
                    Vec3::new(0.0, 0.0, hand.d.z - shoulder.z)
                };
                self.measured[i] = Vec3::new(chest.a.dot(v), chest.b.dot(v), chest.c.dot(v));
            }
        }
        self.grip_incoming(c, &m);
        for i in 0..2 {
            if self.p.hand_dist[i] < 0.0 {
                // 0x2ad0a64: the hand from the parent joint and offset in the incoming pose
                let l = limb(i);
                let chest = m[chest_part()];
                let o = self.p.parent_offset[i] + self.measured[i];
                let j = self.parent_part(c, i);
                self.arms[i].dist = (m[l.p[3]].d - on(&[chest.a, chest.b, chest.c], m[j].d, o)).length();
            }
        }
    }

    /// 0x2ad0fe4: the support hand in the weapon's frame, in the incoming pose. (Its rifle re-aim on ctx+0x2d80, the gun's
    /// butt, has nothing to read here.)
    fn grip_incoming(&mut self, c: &Ctx, m: &[M34]) {
        let (gh, sh) = (m[limb(self.g).p[3]], m[limb(self.s).p[3]]);
        let w = gun_rows(&gh);
        let inv = inv_rows(&w);
        let srows = [sh.a, sh.b, sh.c];
        self.grip_rows = std::array::from_fn(|i| inv[0] * srows[i].x + inv[1] * srows[i].y + inv[2] * srows[i].z);
        let d = sh.d - gh.d;
        self.grip_meas = Vec3::new(d.dot(w[0]), d.dot(w[1]), d.dot(w[2]));
        self.grip_pos = self.grip_meas;
        if self.ws[self.g] == 2 {
            let chest = m[chest_part()];
            self.below = (gh.d - m[chest_part()].d).dot(chest.a) < 0.0;
            let _ = c;
        }
    }

    /// Our part of RDR2's parent joint, or the arm's upper arm.
    fn parent_part(&self, c: &Ctx, i: usize) -> usize {
        let j = self.p.parent[i];
        if (0..20).contains(&j) && c.b.frames.len() >= 21 { rp(j as usize + 1) } else { limb(i).p[1] }
    }

    /// Activate (§4).
    fn start(&mut self, c: &mut Ctx) {
        // 4.: each weapon's attachment softened (0x2a47288), state 3 at the factor (0x2a4b400)
        for i in 0..2 {
            c.ch.core.weapon[i].soften();
            if c.ch.core.arm_state[i] == 3 {
                c.ch.core.weapon[i].scale(self.p.weapon_strength[1]);
            }
            if c.ch.core.weapon[i].on {
                crate::extras_core_rdr2::trace_once("weapon soft", || format!("pointGun: the weapon's attachment softened on hand {i}: {:.2}, saved {:.2}", c.ch.core.weapon[i].strength, c.ch.core.weapon[i].saved));
            }
        }
        self.ws = self.states(c);
        (self.g, self.s) = Self::hands(self.ws);
        let l = limb(0);
        // limb +0x3a8 is the arm's length to the hand, not the wrist (rdr2_core3.md: recorded 0.626, +0x3ac 0.558 to the wrist)
        self.arm_len = (c.b.joint_pos(l.e[2]) - c.b.joint_pos(l.e[1])).length() + (c.b.joint_pos(l.e[3]) - c.b.joint_pos(l.e[2])).length()
            + (c.b.at(l.p[3]) - c.b.joint_pos(l.e[3])).length() + 0.023;
        self.allowed = c.ch.random() < self.p.support_prob;
        for i in 0..2 {
            self.anchor(c, i);
            // what RDR2 measures from the incoming pose, from his pose now when there's none
            let hand = c.b.at(limb(i).p[3]);
            self.arms[i].dist = (hand - self.arms[i].anchor).length();
        }
        self.measure(c);
        let h = c.b.part(limb(self.g).p[3]).m;
        let along = unit(h.d - c.b.joint_pos(limb(self.g).e[3]));
        note(format_args!("pointGun: start: gun axis against the hand's line {:.3}, weapon states {:?}, arm length {:.3}", (-h.b).dot(along), self.ws, self.arm_len));
        let chest = c.b.pmat(chest_part()).m;
        let (ls, rs) = (c.b.joint_pos(limb(0).e[1]), c.b.joint_pos(limb(1).e[1]));
        for i in 0..2 {
            let (h, fore) = (c.b.part(limb(i).p[3]).m, c.b.part(limb(i).p[2]).m);
            let f = |v: Vec3| [v.dot(chest.a), v.dot(chest.b), v.dot(chest.c)];
            let fingers = unit(h.d - c.b.joint_pos(limb(i).e[3]));
            let (w, d) = (c.b.eff3(limb(i).e[3]), &c.ch.joints[limb(i).e[3]]);
            note(format_args!("pointGun: start: hand {i} in the chest's rows: a {:.2?} b {:.2?} c {:.2?} line {:.2?}; forearm b {:.2?}; wrist half limits {:.2} {:.2} {:.2} mid {:.2?}",
                f(h.a), f(h.b), f(h.c), f(fingers), f(fore.b), w.half_lean1, w.half_lean2, w.half_twist, d.limits));
        }
        note(format_args!("pointGun: start: the chest's row 0 up {:.2}, row 1 toward his left {:.2}, row 2 forward {:.2}; COM rows up {:.2} {:.2} {:.2}",
            chest.a.dot(c.up), chest.b.dot(unit(ls - rs)), chest.c.dot(unit((ls - rs).cross(c.up))), c.m.a.dot(c.up), c.m.b.dot(c.up), c.m.c.dot(c.up)));
        self.started = true;
    }

    /// The aim's anchor (R+0x160): the parent joint plus the offsets in the chest's frame.
    fn anchor(&mut self, c: &Ctx, i: usize) -> Vec3 {
        let k = self.parent_part(c, i);
        let j = k.saturating_sub(1).min(c.ch.eff.len() - 1);
        let m = c.b.pmat(chest_part()).m;
        let o = self.p.parent_offset[i] + self.measured[i];
        self.arms[i].anchor = on(&[m.a, m.b, m.c], c.b.joint_pos(j), o);
        self.arms[i].anchor
    }

    /// 0x2acf8cc: may this arm point at its target; with alwayspoint and `check` off, the target clamped into the box.
    fn limits_ok(&mut self, c: &Ctx, i: usize, check: bool) -> bool {
        if !self.p.point {
            return false;
        }
        self.ran(3, "the pointing limits ran");
        let anchor = self.anchor(c, i);
        let m = c.b.pmat(chest_part()).m;
        let mut a = self.arms[i];
        a.aim = unit(a.target - anchor);
        let v = a.target - m.d;
        let flat = unit(v - m.a * v.dot(m.a));
        let theta = if flat.dot(m.b) < 0.0 { -acos(-m.c.dot(flat)) } else { acos(-m.c.dot(flat)) };
        // the slack: spine twist left, while standing
        let (mut minus, mut plus) = (0.0, 0.0);
        if self.p.use_twist && !Self::falling(c) {
            let twist: f32 = (*SPINE()).iter().map(|&j| c.ch.actual(j, c.b.frames)[2]).sum();
            (minus, plus) = ((0.5584 - twist).max(0.0), (0.5584 + twist).max(0.0));
        }
        let (hi, lo) = if i == 0 { (self.p.max_away, self.p.max_across) } else { (self.p.max_across, self.p.max_away) };
        let mut ok = theta < hi + minus && theta > -(lo + plus);
        let euler = self.p.falling_limits == 2 || (self.p.falling_limits == 1 && Self::falling(c));
        if euler {
            self.ran(4, "the falling limits' box ran");
            let [across, away, up, down] = self.p.limits;
            let (hi, lo) = if i == 0 { (away, across) } else { (across, away) };
            let (hi, lo) = (hi + minus.to_degrees(), lo + plus.to_degrees());
            let d = unit(v);
            let q = Quat::from_rotation_arc(Vec3::NEG_Z, Vec3::new(d.dot(m.a), d.dot(m.b), d.dot(m.c)));
            let [_, l1, l2] = crate::nmrs::angles(q);
            let (pitch, yaw) = (l1.to_degrees(), -l2.to_degrees());
            ok = pitch >= -up && pitch <= down && yaw >= -lo && yaw <= hi;
            if self.p.always_point && !ok && !check {
                self.ran(5, "alwayspoint clamped the aim");
                let q = crate::nmrs::rotation([0.0, pitch.clamp(-up, down).to_radians(), -yaw.clamp(-lo, hi).to_radians()]);
                let l = q * Vec3::NEG_Z;
                a.target = m.d + (m.a * l.x + m.b * l.y + m.c * l.z) * v.length();
                a.aim = unit(a.target - anchor);
                (a.look, a.led) = (a.target, a.target);
                self.arms[i] = a;
                return true;
            }
        }
        if self.p.lead > 0.0 {
            a.aim = unit(a.led - anchor);
        }
        self.arms[i] = a;
        // not behind: a fixed direction in the chest's frame
        let side = if i == 0 { 1.0 } else { -1.0 };
        let fixed = unit(m.b * (side * 0.37460658) - m.c * 0.92718387);
        !(a.aim.dot(fixed) <= -0.5 && !euler) && ok
    }

    /// 0x2ac7aac: point (3), neutral (0xb), or the side/connect choice (7).
    fn mode(&mut self, c: &Ctx, i: usize) -> i32 {
        if !self.enabled(i) {
            return 0;
        }
        if self.arms[i].facing {
            if self.fall_rifle || self.probe_hit[i] || self.duck_neutral {
                return 0xb;
            }
            if self.limits_ok(c, i, false) {
                return 3;
            }
        }
        7
    }

    /// 0x2ade8ac: may this arm take a neutral pose.
    fn neutral_allowed(&self, i: usize) -> bool {
        match self.ws[i] {
            1 => self.p.pose_unused[0] || ((self.p.one_handed == 0 || self.p.one_handed == 2) && self.conn_for > 0.0),
            2 | 3 => true,
            _ => if self.support_mode { self.p.pose_unused[1] } else { self.p.pose_unused[2] },
        }
    }

    /// 0x2ae0a70: a neutral pose written when allowed; returns the state.
    fn set_pose(&mut self, c: &mut Ctx, i: usize, id: i32, check: bool) -> i32 {
        if check && !self.neutral_allowed(i) {
            return 0;
        }
        let Some((cl, sh, el, wr)) = pose(id) else { return 0 };
        self.ran(6, &format!("neutral pose {id}"));
        let l = limb(i);
        let e = &mut c.ch.eff;
        e[l.e[0]].desired = cl;
        e[l.e[1]].desired = sh;
        e[l.e[2]].desired[0] = el;
        e[l.e[3]].desired = wr;
        self.arms[i].pose = id;
        0xb
    }

    /// 0x2ae08ac: the neutral pose for this arm; returns the state.
    fn neutral(&mut self, c: &mut Ctx, i: usize) -> i32 {
        if !self.neutral_allowed(i) {
            return 0;
        }
        let by_type = |t: i32, d: i32| match t { 0 => Some(d), 1 => Some(6), 2 => Some(0xb), _ => None };
        match self.ws[i] {
            0 => { self.set_pose(c, i, if self.ws[1 - i] == 2 { 10 } else { 0xb }, false); }
            1 => {
                let t = self.p.pistol_neutral;
                if let Some(id) = by_type(t, 2) {
                    self.set_pose(c, i, id, false);
                }
            }
            2 => {
                if self.fall_rifle {
                    self.set_pose(c, i, self.side_pose, false);
                } else if self.side_t > 0.0 && self.broken_t > 0.0 {
                    self.set_pose(c, i, 0xb, false);
                    self.side_t -= self.dt;
                } else {
                    let id = if self.arms[i].facing { by_type(self.p.pistol_neutral, 2) } else { Some(6) };
                    if let Some(id) = id {
                        self.set_pose(c, i, id, false);
                    }
                }
                if self.probe_hit[i] {
                    self.set_pose(c, i, 2, false);
                }
            }
            3 => { self.set_pose(c, i, 0xb, false); }
            _ => {}
        }
        0xb
    }

    /// 0x2adf898: the connect point (pose 7), or while both side timers run the side point (pose 0xc); it becomes the target.
    fn side_connect(&mut self, c: &Ctx, i: usize) -> bool {
        if self.ws[i] == 2 {
            self.extra = 0.0;
        }
        if !self.arms[i].facing {
            return false;
        }
        let on = (self.p.neutral_point.0 && !self.two) || (self.p.neutral_point.1 && self.two);
        if !self.neutral_allowed(i) || !on {
            return false;
        }
        let m = c.m;
        let chest = c.b.pmat(chest_part()).m;
        let flip = if i == 0 { -1.0 } else { 1.0 };
        let (t, id) = if self.side_t <= 0.0 || self.broken_t <= 0.0 {
            let q = self.p.point2connect;
            if self.ws[i] == 2 {
                self.extra = self.p.add_connect;
            }
            // the connect point is in the chest's frame: x along -row 1 (row 1 for the left arm), y row 0, z row 2
            (c.com - chest.b * (q.x * flip) + chest.a * q.y + chest.c * q.z, 7)
        } else {
            let q = self.p.point2side;
            if self.ws[i] == 2 {
                self.extra = self.p.add_side;
            }
            self.side_t -= self.dt;
            (c.com + m.a * (q.x * flip) + m.b * q.y + m.c * q.z, 0xc)
        };
        self.ran(if id == 7 { 7 } else { 8 }, if id == 7 { "the connect point" } else { "the side point" });
        self.arms[i].target = t;
        self.arms[i].pose = id;
        self.arms[i].aim = unit(t - self.arms[i].anchor);
        !self.p.neutral_point.2 || self.limits_ok(c, i, true)
    }

    /// 0x2ac6a24: the gun hand's target frame (R+0x20): the aim's anchor, the hand's distance, and the frame that turns the
    /// chest's -row 2 onto the aim (0x2acd20c) with the gun on it. `kind` 10: at the point in the target.
    fn gun_target(&mut self, c: &Ctx, i: usize, kind: i32) {
        let anchor = self.anchor(c, i);
        let mut a = self.arms[i];
        let param = self.p.hand_dist[i];
        if !self.p.use_incoming || param >= 0.0 {
            a.dist = param.clamp(0.2, self.arm_len.max(0.2));
        }
        let mut d = a.dist;
        if self.p.relax.0 > 0.0 {
            d -= a.relax_t / self.p.relax.0 * self.p.relax.2;
        }
        d = (d + self.extra).clamp(0.2, self.arm_len.max(0.2));
        let mut at = a.led;
        if kind == 10 {
            if self.two {
                d = d.min(0.92 * self.arm_len);
            }
            at = a.target;
        }
        a.used = d;
        let m = c.b.pmat(chest_part()).m;
        let q = Quat::from_rotation_arc(-m.c, unit(at - anchor).try_normalize().unwrap_or(-m.c));
        let gun = [q * -m.c, q * m.b, q * m.a];
        a.goal = hand_for(gun, anchor + a.aim * d);
        self.arms[i] = a;
    }

    /// 0x2a42ca8 (at the point, within 0.9 of the limits) or 0x2a42ebc (following the aim, from the incoming pose's, 0.99): raw (tw, l1, l2).
    fn clavicle_raw(&mut self, c: &Ctx, i: usize, x: Vec3, follow: bool, src: Option<&[M34]>) -> [f32; 3] {
        let l = limb(i);
        let e = c.b.eff3(l.e[0]);
        if !follow {
            self.ran(9, "the clavicle's reach (0x2a42ca8)");
            let to = unit(x - c.b.joint_pos(l.e[0]));
            let h = Vec3::new(e.m.a.dot(to), e.m.b.dot(to), e.m.c.dot(to));
            let raw = crate::nmrs::angles(Quat::from_rotation_arc(Vec3::Z, h.try_normalize().unwrap_or(Vec3::Z)));
            return limit(&e, raw, 0.9).0;
        }
        self.ran(10, "the rifle's clavicle (0x2a42ebc)");
        let to = unit(x - c.b.joint_pos(l.e[1]));
        let (fx, fy) = (e.m.a.dot(to), -e.m.b.dot(to));
        let [mut tw, mut l1, mut l2] = src.map_or([0.0; 3], |m| crate::nmrs::angles(rest(c, l.e[0], Some(m))));
        if fy < -0.01 {
            l1 = fy * -1.04;
        }
        if fx > 0.0 {
            if fy > 0.7 {
                l2 += 2.0 * fx * (fy - 0.7) * -2.0;
            }
            tw = fx * -0.35;
        }
        limit(&e, [tw, l1, l2], 0.99).0
    }

    /// The clavicle written (blended toward the incoming pose by `blend`, a lean of `most` of its limits after it), and the
    /// shoulder's frame it leaves (0x2a2fcac).
    fn set_clavicle(&mut self, c: &mut Ctx, i: usize, mut raw: [f32; 3], blend: f32, most: f32, src: Option<&[M34]>) -> Fr {
        let l = limb(i);
        if let (true, Some(m)) = (blend > 0.0, src) {
            self.ran(11, "the clavicle blended toward the incoming pose");
            let t = crate::nmrs::angles(rest(c, l.e[0], Some(m)));
            raw = std::array::from_fn(|k| raw[k] + (t[k] - raw[k]) * blend.min(1.0));
            raw = limit(&c.b.eff3(l.e[0]), raw, most).0;
        }
        write3(c, l.e[0], raw);
        let e = c.b.eff1(l.e[0]);
        let on_it = fr(&e.m).then(Fr { q: crate::nmrs::rotation(raw), d: Vec3::ZERO });
        on_it.then(fr(&e.child).inv().then(fr(&c.b.eff1(l.e[1]).m)))
    }

    /// The chain 0x2a44338: the elbow from the triangle in its hinge's frame, the shoulder from the source pose turned so the
    /// wrist lies on the target, then by the elbow attitude about that line. Returns the shoulder's rotation and the elbow's angle.
    fn chain(&mut self, c: &mut Ctx, i: usize, x: Vec3, attitude: f32, p: Fr, src: Option<&[M34]>) -> (Quat, f32) {
        self.ran(45, "the chain solver (0x2a44338)");
        let l = limb(i);
        let e = c.b.limb(l.e[1], l.e[2]);
        let (row, el) = ([e.a, e.b, e.c], c.b.eff1(l.e[2]));
        let d = c.b.joint_pos(l.e[3]) - el.pos;
        let v = Vec3::new(el.child.a.dot(d), el.child.b.dot(d), el.child.c.dot(d));
        let u = Vec3::new(e.a.dot(-e.d), e.b.dot(-e.d), e.c.dot(-e.d));
        let t = p.q.inverse() * (x - p.d);
        let most = on(&row, e.d, v).length();
        let mut len = t.length();
        if most < len {
            len = most - 0.001;
        }
        let aa = -2.0 * v.x * u.x - 2.0 * v.y * u.y;
        let bb = 2.0 * u.x * v.y - 2.0 * v.x * u.y;
        let cc = len * len - u.length_squared() - v.length_squared() + 2.0 * v.z * u.z;
        let root = (bb * bb + aa * aa - cc * cc).clamp(0.0, 10.0).sqrt();
        let (top, under) = (-bb - root, -aa - cc);
        let half = if top == 0.0 && under == 0.0 { 0.0 } else { (top / under).atan() };
        let bend = half + half;
        let w = on(&row, e.d, Quat::from_rotation_z(bend) * v);
        let start = rest(c, l.e[1], src);
        let wa = start * w;
        let (dt, turn) = (t.try_normalize().unwrap_or(Vec3::Z), Quat::from_rotation_arc(wa.try_normalize().unwrap_or(Vec3::Z), t.try_normalize().unwrap_or(Vec3::Z)));
        (Quat::from_axis_angle(dt, -attitude) * turn * start, bend)
    }

    /// The type-9 request's first solver (0x2a430f4): clavicle, the pistol's pull-in past the shoulder's range, the chain, the
    /// wrist on the frame the chain leaves, times its weight.
    #[allow(clippy::too_many_arguments)]
    fn reach(&mut self, c: &mut Ctx, i: usize, goal: M34, elbow: f32, blend: f32, w: f32, support: bool, hip: bool) {
        let l = limb(i);
        let src = self.incoming(c);
        let src = src.as_deref();
        let hand = c.b.part(l.p[3]).m;
        let wrist = c.b.joint_pos(l.e[3]) - hand.d;
        let mut x = on(&[goal.a, goal.b, goal.c], goal.d, Vec3::new(hand.a.dot(wrist), hand.b.dot(wrist), hand.c.dot(wrist)));
        let raw = self.clavicle_raw(c, i, goal.d, hip && !support, src);
        let p = self.set_clavicle(c, i, raw, blend, 0.9, src);
        if !hip && !support {
            let to = unit(x - p.d);
            let h = p.q.inverse() * to;
            let q = Quat::from_rotation_arc(Vec3::Z, h.try_normalize().unwrap_or(Vec3::Z));
            let (clip, past) = limit(&c.b.eff3(l.e[1]), crate::nmrs::angles(q), 0.9);
            if past > 0.0 {
                self.ran(12, "a pistol past the shoulder's range: the hand comes in");
                let (_, angle) = (crate::nmrs::rotation(clip).inverse() * q).to_axis_angle();
                let most = 2.0 * (c.b.joint_pos(l.e[2]) - c.b.joint_pos(l.e[3])).length();
                x = p.d + to * (angle.cos() * most).clamp(0.15, most.max(0.15));
            }
        }
        let (q, bend) = self.chain(c, i, x, elbow, p, src);
        write3(c, l.e[1], crate::nmrs::angles(q));
        c.ch.eff[l.e[2]].desired[0] = bend;
        // the wrist: the hand's frame on the forearm the chain leaves, times the weight
        c.ch.eff[l.e[3]].desired = [0.0; 3];
        if w > 0.0 {
            let wq = wrist_q(c, i, p.q * q, bend, &goal);
            let raw = crate::nmrs::angles(wq);
            write3(c, l.e[3], raw);
            let d = c.ch.eff[l.e[3]].desired;
            c.ch.eff[l.e[3]].desired = d.map(|x| x * w);
            self.ran(13, "the wrist turned to the target frame");
        }
    }

    /// The type-9 request's second solver (0x2a43794, usePistolIK): the upper arm aimed at the target from its neutral turn, off the
    /// line by `off` about the hinge; past the shoulder's limits turned so the hinge's plane holds the target; the elbow from
    /// the current elbow's line to it; the wrist on the frame.
    fn pistol_reach(&mut self, c: &mut Ctx, i: usize, goal: M34, blend: f32, off: f32) {
        self.ran(46, "the pistol solver (0x2a43794)");
        let l = limb(i);
        let src = self.incoming(c);
        let src = src.as_deref();
        let raw = self.clavicle_raw(c, i, goal.d, true, src);
        let p = self.set_clavicle(c, i, raw, blend, 0.99, src);
        let e = c.b.limb(l.e[1], l.e[2]);
        // (the note's 0x2a43794 steps; everything in the shoulder's parent frame)
        let d0 = (p.q.inverse() * (goal.d - c.b.joint_pos(l.e[1]))).try_normalize().unwrap_or(Vec3::Z);
        let neutral = Quat::from_rotation_x(std::f32::consts::FRAC_PI_2);
        let (ed, axis) = (neutral * e.d, neutral * e.c);
        let du = Quat::from_axis_angle(axis.try_normalize().unwrap_or(Vec3::Z), off) * unit(ed);
        let mut q = Quat::from_rotation_arc(du, d0) * neutral;
        let es = c.b.eff3(l.e[1]);
        let (clip, past) = limit(&es, crate::nmrs::angles(q), self.pik_limit);
        q = crate::nmrs::rotation(clip);
        if past <= 0.0 && self.pik_limit <= 1.15 {
            self.pik_limit = 1.1999999;
        } else if past > 0.0 && self.pik_limit >= 1.15 {
            self.pik_limit = 1.1;
        }
        if past > 0.0 {
            self.ran(47, "the pistol solver's limit: the hinge's plane turned to the target");
            let w = q * du;
            let row0 = q * e.a;
            let n1 = unit(w.cross(d0));
            let n2 = n1.cross(w);
            let mut angle = acos(n2.dot(row0));
            if n2.cross(row0).dot(w) > 0.0 {
                angle = -angle;
            }
            q = Quat::from_axis_angle(w.try_normalize().unwrap_or(Vec3::Z), angle) * q;
        }
        let raw = limit(&es, crate::nmrs::angles(q), 0.95).0;
        write3(c, l.e[1], raw);
        let q = crate::nmrs::rotation(raw);
        // the elbow: from straight, the frame on the arm just set, to the line from the elbow now
        let up = p.then(Fr { q, d: Vec3::ZERO }).then(Fr { q: e_quat(&e), d: e.d });
        let eff = c.b.eff1(l.e[2]);
        let fore = up.then(fr(&eff.child).inv()).then(fr(&c.b.part(l.p[2]).m));
        let upper = c.b.part(l.p[1]).m;
        let along = fore.at(fr(&upper).inv().at(eff.pos)) - up.d;
        let to = goal.d - c.b.joint_pos(l.e[2]);
        let hinge = c.ch.joints[l.e[2]].limits[0];
        let bend = acos(unit(along).dot(unit(to))).clamp(hinge.0.min(hinge.1), hinge.0.max(hinge.1));
        c.ch.eff[l.e[2]].desired[0] = bend;
        // the wrist, within 0.99 of its limits
        let wq = wrist_q(c, i, p.q * q, bend, &goal);
        let raw = limit(&c.b.eff3(l.e[3]), crate::nmrs::angles(wq), 0.99).0;
        write3(c, l.e[3], raw);
        self.ran(13, "the wrist turned to the target frame");
    }

    /// 0x2add8ec: the gun arm points; state 3, or 6 off the aim. (`again`: rebuild the target frame first.)
    fn point_arm(&mut self, c: &mut Ctx, i: usize, again: bool, kind: i32) -> i32 {
        if again {
            self.gun_target(c, i, kind);
        }
        let l = limb(i);
        let a = self.arms[i];
        if self.yanked {
            // lassoed by the chest (slot 0x88's +0x184 = 1): the arm straight along the aim, the clavicle and wrist zero [P]
            self.ran(14, "the yanked arm: straight along the aim");
            let side = if i == 0 { 1.0 } else { -1.0 };
            let x = c.b.joint_pos(l.e[1]) + unit(a.target - c.b.joint_pos(l.e[1])) * self.arm_len;
            crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[3], x, self.p.elbow[0], 0.0, side, 1.0, Vec3::ZERO, None);
            c.ch.eff[l.e[0]].desired = [0.0; 3];
            c.ch.eff[l.e[3]].desired = [0.0; 3];
        } else {
            self.ran(15, "the point arm's reach");
            c.out.lim_floor.push((l.p[3], [1.5, 1.5, 1.5]));
            let rifle = self.ws[i] == 2;
            let blend = if rifle && !(self.exists && self.connected) { 0.0 } else { self.p.clav_blend[0] };
            if self.p.pistol_ik.0 && !self.two {
                self.pistol_reach(c, i, a.goal, blend, self.p.pistol_ik.1);
            } else {
                self.reach(c, i, a.goal, self.p.elbow[0], blend, 1.0, false, rifle || self.p.from_hip);
            }
            if self.p.ori.0 > 0.0 && a.relax > 0.0 {
                self.ran(16, "the hand's orientation PD");
                let (hand, k, d) = (c.b.part(l.p[3]), self.p.ori.0 * a.relax * self.tw, self.p.ori.1);
                let err = (quat_of(&a.goal) * quat_of(&hand.m).inverse()).to_scaled_axis();
                // RDR2's are angular impulses per step
                c.out.torque(l.p[3], err * (k * k) - hand.w * (k * d));
            }
            if self.new_target {
                let pos = a.goal.d;
                self.arms[i].pd_vel = if a.pd_at.length_squared() >= 0.1 { (pos - a.pd_at) / self.dt.max(1e-7) } else { c.b.part(l.p[2]).v };
                self.arms[i].pd_at = pos;
            }
            if self.p.pos.0 > 0.0 && a.relax > 0.0 {
                self.ran(17, "the forearm's position PD");
                let (k, d, at) = (self.p.pos.0 * a.relax * self.tw, self.p.pos.1, c.b.joint_pos(l.e[3]));
                let v = c.b.part(l.p[2]).v - self.arms[i].pd_vel;
                let f = (a.goal.d - at) * (k * k * self.dt) - v * (2.0 * k * d * self.dt);
                c.out.impulse.push((l.p[2], at, f));
            }
        }
        let hand = c.b.part(l.p[3]).m;
        let err = acos((-hand.b).dot(self.arms[i].aim));
        if err > self.p.error_threshold { 6 } else { 3 }
    }

    /// 0x2ae0a2c.
    fn neutral_or_side(&mut self, c: &mut Ctx, i: usize) -> i32 {
        if self.side_connect(c, i) {
            self.point_arm(c, i, true, 10);
            10
        } else {
            self.neutral(c, i)
        }
    }

    /// 0x2ad2458 with the mode.
    fn dispatch(&mut self, c: &mut Ctx, i: usize, mode: i32, again: bool) -> i32 {
        match mode {
            3 => self.point_arm(c, i, again, 1),
            7 => {
                self.gun_target(c, i, 7);
                self.neutral_or_side(c, i)
            }
            10 => {
                self.point_arm(c, i, again, 10);
                10
            }
            0xb => {
                self.gun_target(c, i, 0xb);
                self.neutral(c, i)
            }
            _ => 0,
        }
    }

    /// 0x2ad0360: the support's grip on the gun hand and on its target; the distances; whether it's within reach.
    fn support_target(&mut self, c: &Ctx, g: usize, s: usize) -> bool {
        if !self.enabled(s) || !self.enabled(g) {
            self.arms[s].state = 0;
            self.hand_len = None;
            return false;
        }
        let gh = c.b.part(limb(g).p[3]).m;
        let gw = gun_rows(&gh);
        let make = |rows: &[Vec3; 3], p: Vec3, gw: &[Vec3; 3], d: Vec3| {
            let r = mul(rows, gw);
            M34 { a: r[0], b: r[1], c: r[2], d: on(gw, d, p) }
        };
        let (rows, pos) = (self.grip_rows, self.grip_pos);
        let grip = make(&rows, pos, &gw, gh.d);
        let alt = if self.arms[g].pointing {
            let goal = self.arms[g].goal;
            make(&rows, pos, &gun_rows(&goal), goal.d)
        } else {
            grip
        };
        let l = limb(s);
        let (hand, shoulder) = (c.b.at(l.p[3]), c.b.joint_pos(l.e[1]));
        let scale = self.arm_len * self.p.reach_scale;
        let mut a = self.arms[s];
        (a.grip, a.alt) = (grip, alt);
        a.d_hand = (hand - grip.d).length();
        a.d_grip = (grip.d - shoulder).length();
        a.d_alt = (alt.d - shoulder).length();
        a.near = a.d_grip < self.p.make_con * 0.5 + self.arm_len;
        a.reach = a.d_alt < scale;
        self.arms[s] = a;
        if !self.p.move_support || a.reach {
            return a.reach;
        }
        if a.d_alt - scale >= 1.2 {
            return false;
        }
        // movesupport: the grip slides along the line to where the support can reach
        self.ran(18, "movesupport slid the grip");
        let goal = self.arms[g].goal;
        let p0 = goal.d + goal.b * 0.9 + goal.c * 0.7;
        let f = p0 - shoulder;
        if f.length() < scale {
            let r = scale - 0.001;
            let d = alt.d - p0;
            let (aa, bb) = (d.length_squared(), 2.0 * d.dot(f));
            let disc = bb * bb - 4.0 * (f.length_squared() - r * r) * aa;
            let t = if disc <= 0.0 || aa <= 1e-6 { -1.0 } else { (disc.sqrt() - bb) * (0.5 / aa) };
            let mut a = self.arms[s];
            a.grip.d = (a.grip.d - gh.d) * t + gh.d;
            a.alt.d = (a.alt.d - p0) * t + p0;
            a.d_grip = (a.grip.d - shoulder).length();
            a.d_alt = (a.alt.d - shoulder).length();
            a.near = a.d_grip < self.p.make_con * 0.5 + self.arm_len;
            a.reach = a.d_alt < scale;
            a.d_hand = (hand - a.grip.d).length();
            self.arms[s] = a;
        }
        self.arms[s].reach
    }

    /// 0x2ae1d40: the support hand holds the gun.
    fn gripped(&self, s: usize) -> bool {
        let most = match self.p.support_constraint { 0 => 0.1, 2 => 0.1, _ => 0.03 };
        self.exists && self.arms[s].d_hand < most
    }

    /// 0x2ac8638: the falling modes.
    fn falling_modes(&mut self, c: &Ctx) {
        if !self.support_mode || (self.fall_rifle && self.fall_support) {
            return;
        }
        let sat = c.extras_on.contains(&"rdr2Sit");
        let falling = Self::falling(c) && !sat;
        // the attitude 0.2 s on (RDR2's 0x2a35fb8, 0x2a33350; Delirium's classes as highFall's)
        let class = || {
            let q = Quat::from_scaled_axis(c.turn_rate() * 0.2);
            crate::extras_fall::attitude([q * c.m.a, q * c.m.b, q * c.m.c], c.up)
        };
        let mut seen = None;
        if !self.fall_rifle && self.two && falling && self.p.rifle_fall != 0 {
            let set = if self.p.rifle_fall == 2 {
                let k = class();
                seen = Some(k);
                k == 2
            } else {
                true
            };
            self.fall_rifle = set;
            if set {
                self.ran(19, "rifleFall: the arms neutral");
                self.side_pose = if self.side_t > 0.0 && self.broken_t > 0.0 { 4 } else { 3 };
            }
        }
        let t = self.p.falling_type;
        if self.fall_support || !falling || t == 0 {
            return;
        }
        let mut fine = false;
        if t == 2 || t == 4 || t == 5 {
            let k = seen.unwrap_or_else(class);
            fine = k == 0 || k == 3 || k == 1;
        }
        let set = if (3..=5).contains(&t) {
            let touching = parts(0x1fff09).into_iter().any(|k| c.b.doll.hit_out.get(k).is_some_and(Option::is_some));
            if touching && t == 5 {
                fine = false;
            }
            touching && !fine
        } else {
            !fine
        };
        self.fall_support = set;
        if set {
            self.ran(20, "fallingTypeSupport met");
        }
    }

    /// 0x2acf688: the support broke: release it, and the arms wait in neutral or side poses. True while they do.
    fn support_break(&mut self, c: &mut Ctx, g: usize, s: usize) -> bool {
        let draw = |me: &mut Self, c: &mut Ctx| {
            me.broken_t = me.p.broken_time;
            if c.ch.random() <= me.p.broken_side {
                me.side_t = me.broken_t;
            }
            me.exists = false;
            me.ran(21, "the support broke");
        };
        if self.broken_t <= 0.0 {
            let snapped = self.hand_len.is_some() && c.b.doll.snapped.contains(&limb(s).p[3]);
            if snapped {
                self.hand_len = None;
                draw(self, c);
            }
            if self.exists && self.p.support_constraint == 2 && self.arms[s].d_hand > 0.17 {
                draw(self, c);
            }
        }
        if self.broken_t <= 0.0 {
            return false;
        }
        self.broken_t -= self.dt;
        self.arms[s].req = 8;
        let sup = self.neutral(c, s);
        let gun = if !self.two {
            let mut m = self.mode(c, g);
            self.arms[g].req = m;
            if m == 3 && !matches!(self.p.one_handed, 1 | 3) {
                m = 7;
            }
            self.dispatch(c, g, m, true)
        } else {
            self.arms[g].req = 8;
            if !self.fall_rifle {
                self.gun_target(c, g, 7);
                self.neutral_or_side(c, g)
            } else {
                self.gun_target(c, g, 0xb);
                self.neutral(c, g);
                0xb
            }
        };
        self.rifle_constraint(c, g);
        self.arms[s].state = sup;
        self.arms[g].state = gun;
        true
    }

    /// May the gun arm point with the support where it is (oneHandedPointing).
    fn alone_ok(&self, gun_state: i32) -> bool {
        match self.p.one_handed {
            1 => gun_state == 1 && self.p.support_pistol,
            2 => gun_state == 2,
            3 => true,
            _ => false,
        }
    }

    /// 0x2acf33c: the support shoulder or hand beyond the plane of the shoulders and spine, past the neck.
    fn crossed(&self, c: &Ctx, s: usize) -> bool {
        let (ls, rs) = (c.b.joint_pos(limb(0).e[1]), c.b.joint_pos(limb(1).e[1]));
        let pelvis = c.b.at(0);
        let spine = c.b.at(chest_part()) - pelvis;
        let across = rs - ls;
        let n = unit(across.cross(spine));
        let sh = c.b.joint_pos(limb(s).e[1]);
        let neck = c.b.at((*crate::nmrs::NECK())[0] + 1) - sh;
        let past = |v: Vec3| v.dot(n) > 0.0 && v.dot(spine) < neck.dot(spine);
        past(self.arms[s].grip.d - sh) || past(c.b.at(limb(s).p[3]) - sh)
    }

    /// 0x2adcf54: both hands on the gun.
    fn two_handed(&mut self, c: &mut Ctx, g: usize, s: usize) {
        let dt = self.dt;
        let gun_state = self.ws[g];
        let mut want = if self.ws[s] == 0 && self.enabled(s) { 7 } else { 0 };
        let mut gun = 0;
        'body: {
            if !self.enabled(g) {
                want = 0;
                break 'body;
            }
            self.falling_modes(c);
            if self.support_break(c, g, s) {
                self.connected = false;
                self.connected_kept = false;
                return;
            }
            let (mut again, mut was_pointing, mut lost) = (true, false, false);
            let mut mode = self.mode(c, g);
            self.arms[g].req = mode;
            self.gun_target(c, g, 1);
            let reach = self.support_target(c, g, s);
            let d = self.arms[s].d_hand;
            self.connected = self.exists && d < if self.connected { 0.15 } else { 0.07 };
            if mode == 3 {
                let alone = self.alone_ok(gun_state);
                let mut point = (want != 0 && self.enabled(s) && reach) || alone;
                if !self.connected_kept && self.allowed && !alone && (!point || !self.connected) {
                    point = false;
                }
                self.arms[s].reach_kept = self.arms[s].reach;
                if point {
                    again = false;
                    self.allowed = c.ch.random() < self.p.support_prob;
                    was_pointing = self.arms[g].pointing;
                    lost = !self.arms[s].reach;
                    self.arms[g].pointing = true;
                } else {
                    mode = 7;
                }
            }
            if mode == 7 && !self.neutral_allowed(g) {
                mode = 0;
            }
            if mode == 0 {
                want = 0;
                gun = self.dispatch(c, g, mode, false);
                break 'body;
            }
            if want == 0 {
                gun = self.dispatch(c, g, mode, again);
                break 'body;
            }
            if mode == 7 && self.side_connect(c, g) {
                mode = 10;
                self.gun_target(c, g, 10);
            }
            if !was_pointing {
                self.support_target(c, g, s);
            }
            let crossed = self.crossed(c, s);
            self.arms[s].crossed = crossed;
            let a = self.arms[s];
            let can = (self.exists && mode == 3 && a.reach) || (a.near && !lost);
            if (mode == 3 || self.p.always_support) && !crossed && can {
                want = 4;
            }
            let mut on = self.support_on;
            if on && self.fall_support && match self.p.falling_support { 0 => true, 2 => !self.exists, 3 => !self.gripped(s), _ => false } {
                self.ran(22, "fallingSupport: support off");
                self.support_on = false;
                on = false;
            }
            if !on || !self.enabled(s) {
                self.arms[s].req = 0;
                want = 0;
            }
            if want != 4 && mode == 3 && !self.alone_ok(gun_state) {
                mode = 7;
                again = true;
            }
            gun = self.dispatch(c, g, mode, again);
            if gun == 0xb && want == 4 {
                // the gun arm went neutral: the support follows the gun the pose will hold (0x2acd3b4)
                self.ran(23, "the support follows the neutral gun arm");
                let h = pose_frame(c, g);
                self.arms[g].goal = h;
                let r = mul(&self.grip_rows, &gun_rows(&h));
                let alt = M34 { a: r[0], b: r[1], c: r[2], d: on_rows(&gun_rows(&h), h.d, self.grip_pos) };
                let sh = c.b.joint_pos(limb(s).e[1]);
                let mut a = self.arms[s];
                a.alt = alt;
                a.d_alt = (alt.d - sh).length();
                a.reach = a.d_alt < self.arm_len;
                self.arms[s] = a;
                if !a.near || !a.reach {
                    want = 7;
                }
            }
            self.arms[s].req = want;
            if want == 4 {
                let a = self.arms[s];
                let use_grip = !self.exists || !(gun == 3 || gun == 6 || (gun == 10 && a.reach));
                let m = if use_grip { a.grip } else { a.alt };
                let w = if a.d_hand > 0.2 { 0.0 } else if a.d_hand >= 0.1 { (0.2 - a.d_hand) * 10.0 } else { 1.0 };
                let shoulder = c.b.joint_pos(limb(s).e[1]);
                let off = m.d - shoulder;
                let most = self.arm_len * self.p.clamp_support;
                let pos = shoulder + if off.length_squared() > most * most { unit(off) * most }
                    else if off.length_squared() < 0.0025 { unit(off) * 0.05 } else { off };
                let goal = M34 { d: pos, ..m };
                self.arms[s].req_m = goal;
                self.ran(24, "the support arm's request");
                let blend = if self.exists && self.connected { self.p.clav_blend[1] } else { 0.0 };
                self.reach(c, s, goal, self.p.elbow[1], blend, w, true, gun_state == 2 || self.p.from_hip);
                if w > 0.0 {
                    c.out.lim_floor.push((limb(s).p[3], [1.5, 1.5, 1.5]));
                }
            } else if want == 7 {
                want = self.neutral(c, s);
            }
        }
        self.arms[s].state = want;
        self.arms[g].state = gun;
        self.rifle_constraint(c, g);
        self.support_pull(c, g, s);
        // §5.8 the connect timers
        let held = (self.arms[g].state > 2 && self.arms[g].state != 10) || self.arms[g].req != 3 || self.conn_for > 0.0 || !self.arms[s].reach_kept;
        let (mut left, mut on) = (self.conn_for, 0.0);
        if held {
            self.conn_t = 0.0;
        } else {
            left = 0.0;
            on = dt + self.conn_t;
            self.conn_for = 0.0;
            self.conn_t = on;
        }
        if on > self.p.connect_after {
            self.ran(25, "the connect timer ran out: the support may connect");
            left = self.p.connect_for;
            self.conn_for = left;
        }
        if left >= 0.0 {
            self.conn_for = left - dt;
        }
        if self.connected {
            self.connected_kept = true;
        }
        if self.arms[s].state != 4 {
            self.connected_kept = false;
        }
    }

    /// 0x2adf5d4, 0x2ae2da8: a rope between the chest and the rifle hand, tightening to rifleConstraintMinDistance.
    fn rifle_constraint(&mut self, c: &Ctx, g: usize) {
        let st = self.arms[g].state;
        if self.ws[g] != 2 || !self.p.constrain_rifle || self.below || !matches!(st, 2 | 3 | 6 | 10) {
            self.rifle_len = None;
            return;
        }
        let h = c.b.part(limb(g).p[3]).m;
        let (a, min) = (self.arms[g], self.p.rifle_min);
        let butt = h.d + h.b * a.used;
        let at = a.anchor + a.aim * (min - 0.05);
        let len = match self.rifle_len {
            None => {
                self.ran(26, "the rifle's constraint");
                (butt - at).length()
            }
            Some(l) => (l - self.dt.clamp(0.0, 0.0166)).clamp(min, 10.0).max(0.001),
        };
        self.rifle_len = Some(len);
        self.ropes.push((limb(g).p[3], butt, at, len, -1.0));
        self.ropes.push((chest_part(), at, butt, len, -1.0));
    }

    /// 0x2ae1d84: the support's pull and hand-to-hand constraint.
    fn support_pull(&mut self, c: &mut Ctx, g: usize, s: usize) {
        if !self.exists && self.ramp > 0.0 {
            self.ramp = 0.5;
        }
        if self.ramp < 0.0 {
            self.ramp = 2.0;
        }
        self.grip_pos = self.grip_meas * self.ramp.min(1.0);
        self.ramp = (self.ramp + 2.0 * self.dt).min(2.0);
        self.exists = false;
        let (ls, lg) = (limb(s), limb(g));
        let a = self.arms[s];
        let candidate = !a.crossed && a.state == 4;
        let kind = self.p.support_constraint;
        if kind == 0 && candidate {
            self.exists = true;
        }
        if kind == 2 && candidate {
            self.exists = true;
            let r = self.p.pull_r;
            let (hs, hg) = (c.b.part(ls.p[3]), c.b.part(lg.p[3]));
            let d = a.grip.d - hs.m.d;
            let scale = if self.dt < 1.0 / 60.0 { self.dt * 60.0 } else { 1.0 };
            if d.length() < 3.0 * r {
                self.ran(27, "the support pull");
                let k = ((3.0 * r - d.length()) * (0.5 / r)).clamp(0.0, 1.0) * self.p.strength;
                let f = (d * (k * k) + (hg.v - hs.v) * (k * 0.2)) * scale;
                c.out.impulse.push((ls.p[3], hs.m.d + hs.m.a * 0.04, f));
                c.out.impulse.push((lg.p[3], a.grip.d, -f));
            }
            let d = a.alt.d - a.grip.d;
            if d.length() < 3.0 * r {
                let k = ((3.0 * r - d.length()) * (0.5 / r)).clamp(0.0, 1.0) * self.p.strength;
                let pelvis = 0;
                let (hp, hg) = (c.b.part(pelvis), c.b.part(lg.p[3]));
                let f = (d * (k * k) + (hp.v - hg.v) * (k * 0.2)) * scale;
                c.out.impulse.push((lg.p[3], a.grip.d, f));
                c.out.impulse.push((pelvis, a.alt.d, -f));
            }
        }
        let close = a.d_hand < self.p.make_con || self.hand_len.is_some();
        let breaking = if self.ramp <= 2.0 - 0.001 { -1.0 } else { self.p.breaking };
        if candidate && (kind == 1 || kind == 2) && close {
            let hand = c.b.part(ls.p[3]).m.d;
            let len = match self.hand_len {
                None => {
                    self.ran(28, "the hand-to-hand constraint made");
                    (hand - a.grip.d).length()
                }
                Some(l) => (l - self.dt * self.p.con_vel).max(self.p.con_min),
            };
            self.hand_len = Some(len);
            self.exists = true;
            self.ropes.push((ls.p[3], hand, a.grip.d, len, breaking));
            self.ropes.push((lg.p[3], a.grip.d, hand, len, breaking));
        } else {
            self.hand_len = None;
        }
        if self.p.pos.0 > 0.0 && self.hand_len.is_some() {
            self.ran(29, "the support hand's position PD");
            let (hs, k, d) = (c.b.part(ls.p[3]), self.p.pos.0, self.p.pos.1);
            let f = (a.req_m.d - hs.m.d) * (k * k * self.dt) - hs.v * (2.0 * k * d * self.dt);
            c.out.impulse.push((ls.p[3], hs.m.d, f));
        }
    }

    /// 0x2adffec: a pointing, supporting or neutral arm's gains.
    fn gains(&mut self, c: &mut Ctx, i: usize) {
        let l = limb(i);
        let a = self.arms[i];
        let (mut k, mut og) = (self.p.arm_k, self.p.grav.0);
        let st = self.ws[i];
        if st == 2 || (st == 1 && self.p.support_pistol) {
            if !(self.gripped(1 - i) && self.arms[1 - i].state == 4) {
                (k, og) = (self.p.arm_k_det, self.p.grav.1);
                self.ran(30, "the arm's gains with the support detached");
                // 0x2a4b400 with +0x120 (5274f454)
                c.ch.core.weapon[i].scale(self.p.weapon_strength[0]);
            }
        }
        let tw = self.tw;
        c.ch.set_stiffness(k * a.relax * tw, self.p.arm_d, ["ul", "ur"][i], None);
        let e = &mut c.ch.eff;
        if self.p.loose {
            for j in [l.e[0], l.e[1], l.e[2]] {
                if e[j].muscle_stiffness >= 0.5 {
                    e[j].muscle_stiffness = 0.5;
                }
            }
        } else {
            e[l.e[0]].muscle_stiffness = 0.5;
            e[l.e[1]].muscle_stiffness = 0.5;
            e[l.e[2]].muscle_stiffness = 0.5;
        }
        e[l.e[3]].muscle_stiffness = 10.0;
        if st == 2 {
            e[l.e[3]].set_stiffness(k * tw, self.p.arm_d, None);
        }
        for j in l.e.into_iter().chain(*SPINE()) {
            e[j].oppose_gravity = og;
        }
    }

    /// With RDR2_TRACE: the solvers' frames against his pose now (the shoulder's angles, the elbow's and wrist's places).
    fn check_frames(&self, c: &Ctx, i: usize) {
        let l = limb(i);
        let (es, e) = (c.b.eff1(l.e[1]), c.b.limb(l.e[1], l.e[2]));
        let d = &c.ch.joints[l.e[1]];
        let [tw, l1, l2] = crate::nmrs::angles(rest(c, l.e[1], None));
        let from = crate::nmrs::effector_angles([l1, l2, tw], &d.limits, d.flip);
        let now = c.ch.actual(l.e[1], c.b.frames);
        let sh = fr(&es.m).then(Fr { q: crate::nmrs::rotation([tw, l1, l2]), d: Vec3::ZERO });
        let el = sh.then(Fr { q: e_quat(&e), d: e.d });
        let ee = c.b.eff1(l.e[2]);
        let w = c.b.joint_pos(l.e[3]) - ee.pos;
        let v = Vec3::new(ee.child.a.dot(w), ee.child.b.dot(w), ee.child.c.dot(w));
        let bend = c.ch.actual(l.e[2], c.b.frames)[0];
        let wr = el.at(Quat::from_rotation_z(bend) * v);
        let wr2 = el.at(Quat::from_rotation_z(-bend) * v);
        note(format_args!("pointGun: frames: shoulder from {:.2?} now {:.2?} desired {:.2?}; elbow place off {:.3}; wrist place off {:.3} (other sign {:.3}), elbow {:.2} desired {:.2}; wrist desired {:.2?}",
            from, now, c.ch.eff[l.e[1]].desired, (el.d - ee.pos).length(), (wr - c.b.joint_pos(l.e[3])).length(), (wr2 - c.b.joint_pos(l.e[3])).length(), bend, c.ch.eff[l.e[2]].desired[0], c.ch.eff[l.e[3]].desired));
    }

    /// 0x2ae03b8: which of the arms' parts leave which of his parts alone.
    fn collisions(&mut self) {
        self.pair_off.clear();
        if !self.support_mode {
            return;
        }
        let (rifle, arm) = (self.two && self.p.no_rifle_col, self.p.no_arm_col);
        let mut mask = if self.g == 1 { 0x80000 } else { 0x10000 };
        let mut ignore = 0x10000;
        if self.g == 0 {
            ignore = 0x80000;
        }
        if rifle || arm {
            if self.ws[1] == 2 {
                mask |= 0x40000;
            }
            if self.ws[0] == 2 {
                mask |= 0x8000;
            }
            ignore = 0x18000;
            if rifle {
                ignore = 0x18600;
            }
            if arm {
                mask |= 0x40000;
                ignore |= 0x10275f;
            }
        }
        self.pair_off.push((mask, ignore));
        self.pair_off.push((if self.s == 0 { 0x1c800 } else { 0xe1000 }, 0x102600));
    }

    /// §5.1 inputs and the targets update 0x2ae2434.
    fn inputs(&mut self, c: &mut Ctx, dt: f32) {
        self.ws = self.states(c);
        // what the other behaviours read of the held gun (ctx+0x2ca4, +0x2ce4)
        c.ch.core.arm_state = self.ws;
        (self.g, self.s) = Self::hands(self.ws);
        self.extra = 0.0;
        // the facing tests, in the incoming pose
        let mut facing = [true; 2];
        if let Some(m) = self.incoming(c) {
            self.ran(31, "the facing tests on the incoming pose");
            let (lh, rh) = (m[limb(0).p[3]], m[limb(1).p[3]]);
            let across = unit(lh.d - rh.d);
            let t = |w: i32, h: &M34, sign: f32| match w {
                2 => (-h.b).dot(across) * sign > 0.94,
                // (a pistol's test compares the gun with the hand's -row 1: ours is that by definition)
                1 => true,
                _ => true,
            };
            facing = [t(self.ws[0], &lh, -1.0), t(self.ws[1], &rh, 1.0)];
            if !self.p.continuous {
                self.grip_incoming(c, &m);
            } else {
                self.measure(c);
            }
        }
        if let Some(r) = c.ch.core.fire.take() {
            self.fire_rdr2(c, r);
        }
        self.tw = if self.p.time_warp.0 && dt > 0.0 { ((1.0 / 30.0) / dt * self.p.time_warp.1).sqrt().max(1.0) } else { 1.0 };
        let chest = c.b.part(chest_part());
        for i in 0..2 {
            let a = &mut self.arms[i];
            (a.was, a.state, a.pointing, a.facing) = (a.state, 0, false, facing[i]);
            a.prev = a.look;
            a.target = self.p.target[i];
            let on_it = self.objects.iter().find(|o| o.0 == self.p.instance[i]);
            if let Some(o) = on_it {
                a.target = o.1 + o.2 * a.target;
            }
            a.look = a.target;
            a.vel = match on_it {
                // 0x2a37784: the instance's own point velocity
                Some(o) => o.3 + o.4.cross(a.target - o.1),
                None if a.prev.length_squared() >= 0.01 && dt > 0.0 => (a.target - a.prev) / dt,
                None => Vec3::ZERO,
            };
            if on_it.is_some() {
                note(format_args!("pointGun: arm {i} targets object {} at {:.2?}", self.p.instance[i], a.target));
            }
            a.led = a.target;
            if self.p.lead > 0.0 {
                // (RDR2 clamps the lead by its large-step ramp too, ctx+0x7b8..: Delirium's step never reaches it)
                let r = a.target - chest.m.d;
                a.led += (a.vel - chest.v - chest.w.cross(r)) * (self.p.lead * 0.016666 / self.tw);
            }
            self.inhibit[i] = (self.inhibit[i] - dt).max(0.0);
            let a = &mut self.arms[i];
            let (t, amount) = (self.p.relax.0, self.p.relax.1);
            a.relax = if t > 0.0 { (1.0 - a.relax_t / t) * amount + (1.0 - amount) } else { 1.0 };
            a.relax_t = (a.relax_t - dt).clamp(0.0, 5.0);
        }
        // the friction while standing
        if c.upright {
            if self.enabled(0) {
                c.out.part_friction.extend(parts(0x1c800).into_iter().map(|k| (k, 0.1)));
            }
            if self.enabled(1) {
                c.out.part_friction.extend(parts(0xe1000).into_iter().map(|k| (k, 0.1)));
            }
        }
    }

    /// §5.4, §5.5 the look, the spine's twist and the balancer.
    fn look(&mut self, c: &mut Ctx, dt: f32) {
        let armed = |w: i32| w != 0 && w != 4;
        let list: Vec<usize> = (0..2).filter(|&i| armed(self.ws[i]) && self.p.enable[i]).collect();
        let chest = c.b.pmat(chest_part()).m;
        let mut look = if list.is_empty() {
            chest.d - chest.a * 3.0
        } else {
            list.iter().map(|&i| self.arms[i].look).sum::<Vec3>() / list.len() as f32
        };
        let point = |a: &Arm| a.state == 3 || a.state == 6;
        let aiming = point(&self.arms[0]) || point(&self.arms[1]);
        let pointing = aiming || self.p.always_twist;
        if look.length_squared() <= 1e-4 {
            return;
        }
        let (g, s) = (self.g, self.s);
        if self.p.use_look {
            // the priority (nmhl's look): 0x12 pointing, or at the connect point with the support near and within reach; else 0xb
            let mut prio = 0xb;
            if self.support_mode {
                let (gs, ss) = (self.arms[g], self.arms[s]);
                if gs.state == 10 && gs.pose == 7 && ss.d_hand < 0.5 && ss.reach_kept {
                    prio = 0x12;
                }
            }
            if aiming {
                prio = 0x12;
            }
            if self.support_mode {
                let (gs, ss) = (self.arms[g], self.arms[s]);
                if !self.connected && ss.state == 4 && (point(&gs) || (gs.state == 10 && gs.pose == 7)) {
                    if self.look_r[1] {
                        self.ran(32, "the look at the support target");
                        (look, prio) = (ss.req_m.d, 0x12);
                    }
                } else {
                    self.look_r[1] = c.ch.random() < 0.5;
                }
                if !self.connected && gs.state == 10 && gs.pose == 7 {
                    if self.look_r[0] && self.look_t <= 2.0 {
                        self.ran(33, "the look at the support hand and its target");
                        (look, prio) = ((ss.req_m.d + c.b.at(limb(s).p[3])) * 0.5, 0x12);
                        self.look_t += dt;
                    }
                } else {
                    self.look_t = 0.0;
                    self.look_r[0] = c.ch.random() < 0.3;
                }
            }
            if !self.looking {
                self.ran(34, "headLook started");
                self.out.push(format!("headLook start=true alwaysLook=true pos={}", v3s(look)));
                self.looking = true;
            }
            c.out.head_look_prio = Some((prio, look));
        }
        let due = self.p.step_turn && self.turn_t >= self.p.step_wait;
        self.turn_t += dt;
        let turning = pointing || due;
        let step = !pointing && due;
        let mut tau = 0.0;
        if turning {
            let o = |w: i32| match w { 2 => self.p.twist_offset.0, 1 => self.p.twist_offset.1, _ => 0.0 };
            tau = o(self.ws[0]) - o(self.ws[1]);
        }
        if self.p.use_twist {
            self.ran(35, &format!("spineTwist at the look, offset {tau}"));
            c.out.spine_twist = Some((look, crate::nmst::SpineTwist::default().stiffness));
            c.out.spine_twist_offset = Some(tau);
        }
        if self.p.use_turn {
            self.ran(36, "the balancer's heading");
            let d = look - chest.d;
            c.out.dbm_turn = Some(if turning { unit(d - c.up * d.dot(c.up)) * 0.5 } else { Vec3::ZERO });
            if step {
                self.ran(37, "the step turn");
                c.out.dbm_turn_facing = Some(self.p.step_facing);
            }
        }
    }

    /// §5.6, §5.7 the joints' limits, the other behaviours and the feedback.
    fn after(&mut self, c: &mut Ctx) {
        self.new_target = false;
        let sh = *SHOULDER();
        let point = |a: &Arm| a.state == 3 || a.state == 6;
        if self.two {
            for i in 0..2 {
                if self.ws[i] == 2 {
                    c.out.lim_floor.push((sh[i] + 1, [0.0, 0.0, 1.22]));
                    self.ran(38, "the rifle shoulder's twist limit 1.22");
                }
                if self.p.support_across && self.ws[i] == 2 && self.arms[1 - i].req == 4 {
                    c.out.lim_floor.push((sh[1 - i] + 1, [1.5, 1.5, 1.5]));
                    self.ran(39, "supportacross: the support shoulder's limits");
                }
            }
        }
        if self.p.shoulder_pistol {
            for i in 0..2 {
                if self.ws[i] == 1 && point(&self.arms[i]) {
                    c.out.lim_floor.push((sh[i] + 1, [0.0, 1.2, 0.0]));
                    self.ran(40, "the pistol shoulder's lean2 limit 1.2");
                }
            }
        }
        // reachForWound leaves the arms in use (its R+0xdb)
        self.block = [self.arms[0].state > 2, self.arms[1].state > 2];
        let changed = (0..2).any(|i| self.arms[i].state != self.arms[i].was);
        let slot = delirium_data::kv!(127 int);
        if changed {
            let (l, r) = (self.arms[0], self.arms[1]);
            c.out.send(slot, "pointgunLeftState", l.state as f32);
            c.out.send(slot, "pointgunRightState", r.state as f32);
            c.out.send(slot, "pointgunLeftPose", l.pose as f32);
            c.out.send(slot, "pointgunRightPose", r.pose as f32);
            c.out.send(slot, "pointgun", self.arms[self.g].state as f32);
            note(format_args!("pointGun: states left {} right {} poses {} {} (t {:.2})", l.state, r.state, l.pose, r.pose, c.b.doll.age));
        }
        let support = if self.gripped(self.s) && self.arms[self.s].state == 4 { 2 } else { 1 };
        if support != self.fed {
            self.ran(41, "the support feedback");
            c.out.send(slot, "pointgunSupport", support as f32);
            self.fed = support;
        }
    }
}

/// A joint frame or part frame: a rotation and a position.
#[derive(Clone, Copy, Debug)]
struct Fr {
    q: Quat,
    d: Vec3,
}

impl Fr {
    fn then(self, b: Fr) -> Fr {
        Fr { q: self.q * b.q, d: self.d + self.q * b.d }
    }
    fn inv(self) -> Fr {
        let q = self.q.inverse();
        Fr { q, d: q * -self.d }
    }
    fn at(self, p: Vec3) -> Vec3 {
        self.d + self.q * p
    }
}

fn fr(m: &M34) -> Fr {
    Fr { q: quat_of(m), d: m.d }
}

fn e_quat(e: &M34) -> Quat {
    quat_of(e)
}

/// 0x2a248a0: the nearest point of the ellipse with half axes `a`, `b`, by the half-angle tangent u (Newton, up to 100 steps).
fn nearest((x, y): (f32, f32), a: f32, b: f32) -> (f32, f32) {
    let (px, py) = (x.abs(), y.abs());
    let (ia, ib) = (1.0 / a, 1.0 / b);
    let (kx, ky) = (ib * a, ia * b);
    let (mut qx, mut qy) = (px.min(a), py.min(b));
    let n = ((qx * qx) / (a * a) + (qy * qy) / (b * b)).sqrt();
    (qx, qy) = (qx / n, qy / n);
    let mut u = ((1.0 - ia * qx) / (ia * qx + 1.0)).max(0.0).sqrt();
    for _ in 0..100 {
        let g = 2.0 * ((px - qx) * kx * qy - (py - qy) * ky * qx);
        let h = (2.0 * qy * qy * kx * kx + 2.0 * qx * qx * ky * ky + 2.0 * qx * (px - qx) + 2.0 * qy * (py - qy)) * (ia * qx + 1.0) - ib * qy * g;
        let step = g / h;
        u = (u - step).clamp(0.0, 1.0);
        let w = 1.0 / (u * u + 1.0);
        (qx, qy) = ((1.0 - u * u) * w * a, 2.0 * u * w * b);
        if step.abs() <= 1e-7 {
            break;
        }
    }
    (qx * x.signum(), qy * y.signum())
}

/// 0x2a23f78: twist and leans within `f` of the joint's limits; how far the leans were past them.
fn limit(e: &crate::nmdbm::Eff3, [tw, mut l1, mut l2]: [f32; 3], f: f32) -> ([f32; 3], f32) {
    let t = f * e.half_twist;
    let (a, b) = (f * e.half_lean1, f * e.half_lean2);
    if a < 0.001 || b < 0.001 {
        l1 = l1.clamp(-a, a);
        l2 = l2.clamp(-b, b);
    }
    let past = ((l1 / a.max(1e-6)).powi(2) + (l2 / b.max(1e-6)).powi(2)).sqrt() - 1.0;
    if past > 0.0 && a >= 0.001 && b >= 0.001 {
        (l1, l2) = nearest((l1, l2), a, b);
    }
    ([tw.clamp(-t, t), l1, l2], past.max(0.0))
}

/// Raw (tw, l1, l2) from a joint's desired angles (effector_angles' inverse).
fn read3(c: &Ctx, j: usize) -> [f32; 3] {
    let (d, e) = (&c.ch.joints[j], c.ch.eff[j].desired);
    let r = |k: usize| {
        let mid = (d.limits[k].0 + d.limits[k].1) * 0.5;
        if d.flip[k] { -e[k] - mid } else { e[k] - mid }
    };
    [r(2), r(0), r(1)]
}

/// 0x2acd3b4: the hand's frame the arm's records make: joint by joint from the clavicle's parent frame.
fn pose_frame(c: &Ctx, i: usize) -> M34 {
    let l = limb(i);
    let (ec, es, ee, ew) = (c.b.eff1(l.e[0]), c.b.eff1(l.e[1]), c.b.eff1(l.e[2]), c.b.eff1(l.e[3]));
    let turn = |q: Quat| Fr { q, d: Vec3::ZERO };
    let f = fr(&ec.m).then(turn(crate::nmrs::rotation(read3(c, l.e[0])))).then(fr(&ec.child).inv().then(fr(&es.m)))
        .then(turn(crate::nmrs::rotation(read3(c, l.e[1])))).then(fr(&es.child).inv().then(fr(&ee.m)))
        .then(turn(Quat::from_rotation_z(c.ch.eff[l.e[2]].desired[0]))).then(fr(&ee.child).inv().then(fr(&ew.m)))
        .then(turn(crate::nmrs::rotation(read3(c, l.e[3])))).then(fr(&ew.child).inv().then(fr(&c.b.part(l.p[3]).m)));
    let m = Mat3::from_quat(f.q);
    M34 { a: m.x_axis, b: m.y_axis, c: m.z_axis, d: f.d }
}

/// Raw (tw, l1, l2) into the joint's desired angles.
fn write3(c: &mut Ctx, j: usize, [tw, l1, l2]: [f32; 3]) {
    let d = &c.ch.joints[j];
    c.ch.eff[j].desired = crate::nmrs::effector_angles([l1, l2, tw], &d.limits, d.flip);
}

/// A joint's rotation, child in parent, in the source pose: the incoming pose's, or his own (0x2a44338 takes either).
fn rest(c: &Ctx, j: usize, src: Option<&[M34]>) -> Quat {
    let (e, d) = (c.b.eff1(j), &c.ch.joints[j]);
    let Some(m) = src else { return quat_of(&e.m).inverse() * quat_of(&e.child) };
    let (pm, cm) = (quat_of(&c.b.part(d.parent).m).inverse() * quat_of(&e.m), quat_of(&c.b.part(d.child).m).inverse() * quat_of(&e.child));
    (quat_of(&m[d.parent]) * pm).inverse() * (quat_of(&m[d.child]) * cm)
}

/// The wrist's rotation for the hand's frame `goal`, on the arm the solver leaves: the upper arm's child frame (rotation `up`),
/// the elbow at `bend`.
fn wrist_q(c: &Ctx, i: usize, up: Quat, bend: f32, goal: &M34) -> Quat {
    let l = limb(i);
    let (el, wr) = (c.b.eff1(l.e[2]), c.b.eff1(l.e[3]));
    let e = c.b.limb(l.e[1], l.e[2]);
    let parent = up * e_quat(&e) * Quat::from_rotation_z(bend) * (quat_of(&el.child).inverse() * quat_of(&wr.m));
    let child = quat_of(goal) * (quat_of(&c.b.part(l.p[3]).m).inverse() * quat_of(&wr.child));
    parent.inverse() * child
}

/// A ray against an object's box, grown by the probe's radius.
fn ray_box(o: &crate::ragdoll::Obstacle, from: Vec3, dir: Vec3, len: f32) -> bool {
    let (q, c) = (o.rot.inverse(), o.at + o.rot * o.centre);
    let (p, d, h) = (q * (from - c), q * dir, o.half + Vec3::splat(0.05));
    let (mut lo, mut hi) = (0.0f32, len);
    for k in 0..3 {
        if d[k].abs() < 1e-6 {
            if p[k].abs() > h[k] {
                return false;
            }
        } else {
            let (a, b) = ((-h[k] - p[k]) / d[k], (h[k] - p[k]) / d[k]);
            lo = lo.max(a.min(b));
            hi = hi.min(a.max(b));
        }
    }
    lo <= hi
}

fn on_rows(r: &[Vec3; 3], d: Vec3, p: Vec3) -> Vec3 {
    on(r, d, p)
}

impl Extra for PointGun {
    fn hold(&mut self, on: bool) {
        if on != self.held_off {
            crate::extras_core_rdr2::trace(|| format!("pointGun: the shot's hand-over {}", if on { "lets it go" } else { "takes it up again" }));
        }
        self.held_off = on;
    }

    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if self.held_off {
            return;
        }
        if !c.ch.rdr2 {
            // without the core: Delirium's pointGun, pointArm at the gun hand's target
            if self.fallback.is_none() {
                let (g, _) = Self::hands(self.states(c));
                self.out.push(format!("pointArm start=true useLeftArm={} target={}", g == 0, v3s(self.p.target[g])));
                self.fallback = Some(true);
            }
            return;
        }
        if !self.started {
            self.start(c);
        }
        self.dt = dt;
        self.ropes.clear();
        self.inputs(c, dt);
        let (ws, g, s) = (self.ws, self.g, self.s);
        // §5.2 the modes
        self.two = (ws[0] == 2 && ws[1] == 0) || (ws[1] == 2 && ws[0] == 0);
        self.support_mode = self.two || (self.p.support_pistol && ((ws[0] == 1 && ws[1] == 0) || (ws[1] == 1 && ws[0] == 0)));
        if !self.two {
            self.rifle_len = None;
        }
        if !self.support_mode {
            self.hand_len = None;
        }
        // §5.3 per arm, right first
        let mut other_set = false;
        for i in [1, 0] {
            let st = if ws[i] == 2 && !self.two { 3 } else { ws[i] };
            match st {
                1 if !self.support_mode => {
                    self.ran(42, "a one-handed arm");
                    let m = self.mode(c, i);
                    self.arms[i].req = m;
                    self.arms[i].state = self.dispatch(c, i, m, true);
                    if ws[1 - i] == 0 || ws[1 - i] == 4 {
                        self.arms[1 - i].state = self.set_pose(c, 1 - i, 0xb, true);
                        other_set = true;
                    }
                }
                1 | 2 => {
                    self.ran(43, "two hands on the gun");
                    self.two_handed(c, i, 1 - i);
                }
                3 => {
                    self.ran(44, "a weapon that can't point: neutral pose 0xb");
                    // (the state stays 0: RDR2 doesn't keep this one's result, so no gains)
                    self.set_pose(c, i, 0xb, false);
                }
                _ => {
                    if !self.support_mode && !other_set {
                        self.arms[i].state = 0;
                    }
                }
            }
        }
        let _ = (g, s);
        for i in [1, 0] {
            if self.arms[i].state > 2 {
                self.gains(c, i);
            }
        }
        self.collisions();
        self.look(c, dt);
        self.after(c);
        if crate::extras_ysk_rdr2::trace() && (c.b.doll.age * 2.0) as i32 != ((c.b.doll.age - dt) * 2.0) as i32 {
            let (a, h) = (self.arms[g], c.b.at(limb(g).p[3]));
            let sa = self.arms[s];
            self.check_frames(c, g);
            note(format_args!("pointGun: at {:.1}: gun state {} off the aim {:.2} rad, hand from its target {:.3}; support state {} hand from its grip {:.3}, grip from the shoulder {:.3} of {:.3}, connected {} constraint {:?}, rifle {:?}, pairs {}",
                c.b.doll.age, a.state, acos((-c.b.part(limb(g).p[3]).m.b).dot(a.aim)), (h - a.goal.d).length(), sa.state, sa.d_hand, sa.d_alt, self.arm_len * self.p.reach_scale, self.connected, self.hand_len, self.rifle_len, c.b.doll.pairs.len()));
        }
    }

    fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.out)
    }

    /// 0x2aceffc: the rifle probe along the aim; ×1.05 while it's hitting. RDR2's sphere of radius 0.05: a ray against the world
    /// (Delirium's world has no sphere sweep), the objects' boxes grown by the radius.
    fn probe(&mut self, c: &Ctx, world: &crate::collision::Collision, obs: &[crate::ragdoll::Obstacle]) {
        if !c.ch.rdr2 || !self.started || self.p.probe <= 0.0 {
            return;
        }
        for i in 0..2 {
            let a = self.arms[i];
            let dir = unit(a.target - a.anchor);
            if dir == Vec3::ZERO {
                continue;
            }
            let len = self.p.probe * if self.probe_hit[i] { 1.05 } else { 1.0 };
            // (not the target's own object: RDR2 leaves the hand targets' instances out)
            let hit = world.cast(a.anchor, dir, len).is_some() || obs.iter().filter(|o| !self.p.instance.contains(&o.id)).any(|o| ray_box(o, a.anchor, dir, len));
            if hit && !self.probe_hit[i] {
                note(format_args!("pointGun: the probe hit (arm {i})"));
            }
            self.probe_hit[i] = hit;
        }
    }

    fn objects(&mut self, obs: &[crate::ragdoll::Obstacle]) {
        self.objects = obs.iter().map(|o| (o.id, o.at, o.rot, o.vel, o.spin)).collect();
    }

    fn ropes(&self) -> Vec<(usize, Vec3, Vec3, f32, f32)> {
        self.ropes.clone()
    }

    fn pair_off(&self) -> Vec<(u32, u32)> {
        self.pair_off.clone()
    }

    fn rfw_block(&self) -> Option<[bool; 2]> {
        (self.block != [false; 2]).then_some(self.block)
    }

    fn pg_duck_set(&mut self, on: bool) {
        if on != self.duck_neutral {
            note(format_args!("pointGun: duck's neutral arms {on}"));
        }
        self.duck_neutral = on;
    }

    fn pg_yanked_set(&mut self, on: bool) {
        self.yanked = on;
    }

    /// RDR2's controlledFall: timeWarpStrengthScale from it, and the left hand off (RDR2 turns enableLeft off when
    /// either hand's weapon state is 0; Delirium's left hand rarely holds a weapon, see `states`).
    fn pg_set(&mut self, time_warp_scale: f32) {
        self.p.time_warp.1 = time_warp_scale;
        // (controlledFall's 5: with either arm's state 0)
        if self.ws[0] == 0 || self.ws[1] == 0 {
            self.p.enable[0] = false;
            crate::extras_core_rdr2::trace_first("controlledFall: an arm in state 0: pointGun's enableLeft off");
        }
    }

    fn fire(&mut self) {
        for a in &mut self.arms {
            a.relax_t = self.p.relax.0;
        }
        note(format_args!("pointGun: fired, the arm relaxes"));
    }

    /// RDR2's message re-parses every parameter from the defaults (the handler's reload), and marks a new target.
    fn update(&mut self, p: &mut P) -> Result<(), Error> {
        let fire = p.b("fireWeapon", false)?;
        let was = self.p.target;
        self.p = Params::new(p)?;
        if self.fallback.is_some() {
            let g = if self.p.target[1] != Vec3::ZERO { 1 } else { 0 };
            self.out.push(format!("pointArm target={}", v3s(self.p.target[g])));
        }
        let _ = was;
        self.new_target = true;
        if fire {
            self.fire();
        }
        Ok(())
    }

    fn stop(&mut self, c: &mut Ctx) {
        // release: the attachments back (0x2a47208)
        for i in 0..2 {
            c.ch.core.weapon[i].restore();
        }
        if self.fallback.is_some() {
            self.out.push("pointArm start=false".into());
            return;
        }
        // 0x2ad53e8: the last feedback; the balancer's heading, spineTwist; ropes, friction, limits and masks end with this step's
        let (l, r) = (self.arms[0], self.arms[1]);
        let slot = delirium_data::kv!(127 int);
        c.out.send(slot, "pointgunEnded", if self.support_mode { 12.0 } else { 7.0 });
        note(format_args!("pointGun: stopped, states left {} right {} poses {} {}", l.state, r.state, l.pose, r.pose));
        if self.looking {
            self.out.push("headLook start=false".into());
        }
        if self.p.use_twist {
            c.out.spine_twist_stop = true;
        }
        if self.p.use_turn {
            c.out.dbm_turn = Some(Vec3::ZERO);
        }
        for i in 0..2 {
            for j in limb(i).e {
                c.ch.eff[j].oppose_gravity = 0.0;
            }
        }
        let _ = (CLAVICLE(), ELBOW(), WRIST());
    }
}
