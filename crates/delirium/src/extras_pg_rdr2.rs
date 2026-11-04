//! RDR2 core: pointGun from RDR2's code (rdr2-decomp/study/rdr2_pointgun.md), an extra message (Delirium has no
//! pointGun of its own: GTA IV's shot task only starts pointArm). With the core off it does what Delirium's did:
//! pointArm at the gun hand's target. Delirium has no weapons, weapon constraints or incoming transforms: the hands'
//! weapon states come from `weapon` (ours) or the character's held weapon, and what RDR2 measures from the incoming
//! pose is measured from his pose when it starts. Arms 0 left, 1 right.
use crate::extras::{Extra, P};
use crate::msg::{Error, Kind, Message, Param};
use crate::nmrs::SPINE;
use crate::nmshot::{chest_part, limb, Ctx};
use bevy_math::Vec3;

macro_rules! unread {
    ($($n:literal),* $(,)?) => { &[$(Param { name: $n, kind: Kind::Float, used: false, doc: "RDR2's; not read here (see the module's notes)" }),*] };
}

pub static MESSAGES: [Message; 1] = [
    Message { name: "pointGun", params: PARAMS },
];

const PARAMS: &[Param] = &[
    Param { name: "weapon", kind: Kind::Int, used: true, doc: "ours: the right hand's weapon, 0 none, 1 pistol, 2 rifle; -1 the character's held weapon (-1)" },
    Param { name: "leftHandTarget", kind: Kind::Vec3, used: true, doc: "where the left hand's gun points, world (zero)" },
    Param { name: "rightHandTarget", kind: Kind::Vec3, used: true, doc: "where the right hand's gun points, world (zero)" },
    Param { name: "leftHandParentEffector", kind: Kind::Int, used: true, doc: "the joint the left aim starts from; -1 its shoulder (-1)" },
    Param { name: "rightHandParentEffector", kind: Kind::Int, used: true, doc: "likewise the right (-1)" },
    Param { name: "leftHandParentOffset", kind: Kind::Vec3, used: true, doc: "added to it, in the chest's frame (zero)" },
    Param { name: "rightHandParentOffset", kind: Kind::Vec3, used: true, doc: "(zero)" },
    Param { name: "leadTarget", kind: Kind::Float, used: true, doc: "lead a moving target by this (0)" },
    Param { name: "lefthandweapondistance", kind: Kind::Float, used: true, doc: "the left hand this far out; < 0 as at the start (-1)" },
    Param { name: "righthandweapondistance", kind: Kind::Float, used: true, doc: "(-1)" },
    Param { name: "armStiffness", kind: Kind::Float, used: true, doc: "a pointing arm's (14)" },
    Param { name: "armStiffnessDetSupport", kind: Kind::Float, used: true, doc: "with the support hand off the gun (8)" },
    Param { name: "armDamping", kind: Kind::Float, used: true, doc: "(1)" },
    Param { name: "supportConstraint", kind: Kind::Int, used: true, doc: "0 none, 1 hand-to-hand constraint (here the pull), 2 the pull (1)" },
    Param { name: "clavicleBlend", kind: Kind::Float, used: true, doc: "the gun arm's clavicle share of the reach (0)" },
    Param { name: "clavicleblendsupport", kind: Kind::Float, used: true, doc: "the support arm's, once connected (1)" },
    Param { name: "elbowAttitude", kind: Kind::Float, used: true, doc: "the gun arm's elbow swing (0.3)" },
    Param { name: "elbowattitudesupport", kind: Kind::Float, used: true, doc: "the support arm's (0.1)" },
    Param { name: "fireWeaponRelaxTime", kind: Kind::Float, used: true, doc: "s the arm gives after a shot (0.4)" },
    Param { name: "fireWeaponRelaxAmount", kind: Kind::Float, used: true, doc: "(0.5)" },
    Param { name: "fireWeaponRelaxDistance", kind: Kind::Float, used: true, doc: "the hand comes in this far, m (0.05)" },
    Param { name: "gravityOpposition", kind: Kind::Float, used: true, doc: "(1)" },
    Param { name: "gravOppDetachedSupport", kind: Kind::Float, used: true, doc: "(0.5)" },
    Param { name: "clampsupport", kind: Kind::Float, used: true, doc: "the support hand's reach, x arm length (1.5)" },
    Param { name: "pointgun", kind: Kind::Bool, used: true, doc: "may point (true)" },
    Param { name: "enableRight", kind: Kind::Bool, used: true, doc: "(true)" },
    Param { name: "enableLeft", kind: Kind::Bool, used: true, doc: "(true)" },
    Param { name: "poseUnusedGunArm", kind: Kind::Bool, used: true, doc: "a gun arm that can't point takes a neutral pose (false)" },
    Param { name: "poseUnusedSupportArm", kind: Kind::Bool, used: true, doc: "(false)" },
    Param { name: "poseUnusedOtherArm", kind: Kind::Bool, used: true, doc: "(false)" },
    Param { name: "maxAngleAcross", kind: Kind::Float, used: true, doc: "degrees across the body it may point (90)" },
    Param { name: "maxAngleAway", kind: Kind::Float, used: true, doc: "degrees away from it (90)" },
    Param { name: "fallingLimits", kind: Kind::Int, used: true, doc: "1 while falling, 2 always: the limits below too (0)" },
    Param { name: "acrossLimit", kind: Kind::Float, used: true, doc: "degrees (90)" },
    Param { name: "awayLimit", kind: Kind::Float, used: true, doc: "(90)" },
    Param { name: "upLimit", kind: Kind::Float, used: true, doc: "(90)" },
    Param { name: "downLimit", kind: Kind::Float, used: true, doc: "(45)" },
    Param { name: "errorThreshold", kind: Kind::Float, used: true, doc: "rad off the aim that counts as not on it (0.3926)" },
    Param { name: "useSpineTwist", kind: Kind::Bool, used: true, doc: "(true)" },
    Param { name: "useTurnToTarget", kind: Kind::Bool, used: true, doc: "the balancer turns him to the target (false)" },
    Param { name: "useHeadLook", kind: Kind::Bool, used: true, doc: "(true)" },
    Param { name: "supportpistol", kind: Kind::Bool, used: true, doc: "the free hand supports a pistol too (false)" },
    Param { name: "alwaysSupport", kind: Kind::Bool, used: true, doc: "(false)" },
    Param { name: "alwayspoint", kind: Kind::Bool, used: true, doc: "aim clamped into the limits instead of not pointing (false)" },
    Param { name: "alwaystwist", kind: Kind::Bool, used: true, doc: "(false)" },
    Param { name: "pistolNeutralType", kind: Kind::Int, used: true, doc: "0, 1, 2: the pistol arm's neutral pose (0)" },
    Param { name: "neutralPoint4Pistols", kind: Kind::Bool, used: true, doc: "a pistol that can't point goes to point2Connect (false)" },
    Param { name: "neutralPoint4Rifle", kind: Kind::Bool, used: true, doc: "a rifle likewise (true)" },
    Param { name: "checkNeutralPoint", kind: Kind::Bool, used: true, doc: "only if the limits allow it (false)" },
    Param { name: "point2Connect", kind: Kind::Vec3, used: true, doc: "in the COM's frame (-1,-0.9,-0.2)" },
    Param { name: "add2WeaponDistConnect", kind: Kind::Float, used: true, doc: "(0)" },
    Param { name: "constraintStrength", kind: Kind::Float, used: true, doc: "the support pull (2)" },
    Param { name: "timeWarpActive", kind: Kind::Bool, used: true, doc: "(false)" },
    Param { name: "timeWarpStrengthScale", kind: Kind::Float, used: true, doc: "(1)" },
    Param { name: "fireWeapon", kind: Kind::Bool, used: true, doc: "ours: with no start, a shot (the arm relaxes)" },
];

/// RDR2's parameters this port doesn't read (see the module's notes); taken and ignored.
const UNREAD: &[Param] = unread!["oneHandedPointing", "constraintMinDistance", "makeConstraintDistance", "reduceConstraintLengthVel", "breakingStrength",
    "brokenSupportTime", "brokenToSideProb", "connectAfter", "connectFor", "useIncomingTransforms", "measureParentOffset",
    "massMultDetachedSupport", "constrainRifle", "rifleConstraintMinDistance", "disableArmCollisions", "disableRifleCollisions",
    "twistoffsetrifle", "twistoffsetpistol", "probelength", "usePistolIK", "pistolfromhip", "stepturnwait", "stepTurn", "movesupport",
    "supportacross", "allowShotLooseness", "rifleFall", "fallingSupport", "fallingTypeSupport", "point2Side", "add2WeaponDistSide",
    "animSource", "measurecontinuous", "oriStiff", "oriDamp", "posStiff", "posDamp"];

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

#[derive(Clone, Copy, Debug, Default)]
struct Arm {
    /// the hand's target this tick and last, and its lead
    target: Vec3,
    prev: Vec3,
    led: Vec3,
    vel: Vec3,
    /// fire relax: its timer and factor; the hand's distance from its anchor
    relax_t: f32,
    relax: f32,
    dist: f32,
    /// R+0x220 state (0 none, 3 pointing, 4 supporting, 6 pointing off the aim, 0xb neutral), last tick's
    state: i32,
    was: i32,
    pose: i32,
    /// the support arm reaches its grip
    reachable: bool,
}

pub struct PointGun {
    weapon: i32,
    target: [Vec3; 2],
    parent: [i32; 2],
    parent_offset: [Vec3; 2],
    lead: f32,
    hand_dist: [f32; 2],
    arm_k: f32,
    arm_k_det: f32,
    arm_d: f32,
    support_constraint: i32,
    clav_blend: [f32; 2],
    elbow: [f32; 2],
    relax: (f32, f32, f32),
    grav: (f32, f32),
    clamp_support: f32,
    point: bool,
    enable: [bool; 2],
    pose_unused: [bool; 3],
    max_across: f32,
    max_away: f32,
    falling_limits: i32,
    limits: [f32; 4],
    error_threshold: f32,
    use_twist: bool,
    use_turn: bool,
    use_look: bool,
    support_pistol: bool,
    always_support: bool,
    always_point: bool,
    always_twist: bool,
    pistol_neutral: i32,
    neutral_point: (bool, bool, bool),
    point2connect: Vec3,
    add_connect: f32,
    strength: f32,
    time_warp: (bool, f32),
    /// running: started under the core, the arm length, the support grip in the gun hand's frame, the connected latch
    started: bool,
    arm_len: f32,
    grip: Vec3,
    connected: bool,
    arms: [Arm; 2],
    looking: bool,
    out: Vec<String>,
    fallback: Option<bool>,
}

fn unit(v: Vec3) -> Vec3 {
    v.normalize_or_zero()
}

fn v3s(v: Vec3) -> String {
    format!("{},{},{}", v.x, v.y, v.z)
}

impl PointGun {
    fn new(p: &mut P) -> Result<PointGun, Error> {
        let deg = |x: f32| x.to_radians();
        let mut g = PointGun {
            weapon: p.i("weapon", -1)?,
            target: [p.v("leftHandTarget", Vec3::ZERO)?, p.v("rightHandTarget", Vec3::ZERO)?],
            parent: [p.i("leftHandParentEffector", -1)?, p.i("rightHandParentEffector", -1)?],
            parent_offset: [p.v("leftHandParentOffset", Vec3::ZERO)?, p.v("rightHandParentOffset", Vec3::ZERO)?],
            lead: p.f("leadTarget", 0.0)?,
            hand_dist: [p.f("lefthandweapondistance", -1.0)?, p.f("righthandweapondistance", -1.0)?],
            arm_k: p.f("armStiffness", 14.0)?, arm_k_det: p.f("armStiffnessDetSupport", 8.0)?, arm_d: p.f("armDamping", 1.0)?,
            support_constraint: p.i("supportConstraint", 1)?,
            clav_blend: [p.f("clavicleBlend", 0.0)?, p.f("clavicleblendsupport", 1.0)?],
            elbow: [p.f("elbowAttitude", 0.3)?, p.f("elbowattitudesupport", 0.1)?],
            relax: (p.f("fireWeaponRelaxTime", 0.4)?, p.f("fireWeaponRelaxAmount", 0.5)?, p.f("fireWeaponRelaxDistance", 0.05)?),
            grav: (p.f("gravityOpposition", 1.0)?, p.f("gravOppDetachedSupport", 0.5)?),
            clamp_support: p.f("clampsupport", 1.5)?,
            point: p.b("pointgun", true)?,
            enable: [p.b("enableLeft", true)?, p.b("enableRight", true)?],
            pose_unused: [p.b("poseUnusedGunArm", false)?, p.b("poseUnusedSupportArm", false)?, p.b("poseUnusedOtherArm", false)?],
            max_across: deg(p.f("maxAngleAcross", 90.0)?), max_away: deg(p.f("maxAngleAway", 90.0)?),
            falling_limits: p.i("fallingLimits", 0)?,
            limits: [deg(p.f("acrossLimit", 90.0)?), deg(p.f("awayLimit", 90.0)?), deg(p.f("upLimit", 90.0)?), deg(p.f("downLimit", 45.0)?)],
            error_threshold: p.f("errorThreshold", 0.3926)?,
            use_twist: p.b("useSpineTwist", true)?, use_turn: p.b("useTurnToTarget", false)?, use_look: p.b("useHeadLook", true)?,
            support_pistol: p.b("supportpistol", false)?, always_support: p.b("alwaysSupport", false)?,
            always_point: p.b("alwayspoint", false)?, always_twist: p.b("alwaystwist", false)?,
            pistol_neutral: p.i("pistolNeutralType", 0)?,
            neutral_point: (p.b("neutralPoint4Pistols", false)?, p.b("neutralPoint4Rifle", true)?, p.b("checkNeutralPoint", false)?),
            point2connect: p.v("point2Connect", Vec3::new(-1.0, -0.9, -0.2))?, add_connect: p.f("add2WeaponDistConnect", 0.0)?,
            strength: p.f("constraintStrength", 2.0)?,
            time_warp: (p.b("timeWarpActive", false)?, p.f("timeWarpStrengthScale", 1.0)?),
            started: false, arm_len: 0.0, grip: Vec3::ZERO, connected: false, arms: [Arm::default(); 2], looking: false,
            out: vec![], fallback: None,
        };
        g.unread(p)?;
        Ok(g)
    }

    fn unread(&mut self, p: &mut P) -> Result<(), Error> {
        for q in UNREAD {
            p.get(q.name);
        }
        if p.b("fireWeapon", false)? {
            self.fire();
        }
        Ok(())
    }

    /// The hands' weapon states (left, right): 0 none, 1 pistol, 2 rifle.
    fn states(&self, c: &Ctx) -> [i32; 2] {
        match self.weapon {
            w @ 0.. => [0, w.min(4)],
            _ if c.ch.busy[1] => [0, if c.ch.busy[0] { 2 } else { 1 }],
            _ => [0, 0],
        }
    }

    /// 0x2ac8860: the gun hand (right unless only the left holds one) and the support hand.
    fn hands(ws: [i32; 2]) -> (usize, usize) {
        let armed = |s: i32| s != 0 && s != 4;
        if !armed(ws[1]) && armed(ws[0]) { (0, 1) } else { (1, 0) }
    }

    /// Activate (§4): the arm length, the support grip and the hands' distances from his pose now.
    fn start(&mut self, c: &mut Ctx) {
        let l = limb(0);
        self.arm_len = (c.b.joint_pos(l.e[2]) - c.b.joint_pos(l.e[1])).length() + (c.b.joint_pos(l.e[3]) - c.b.joint_pos(l.e[2])).length() + 0.023;
        let (g, s) = Self::hands(self.states(c));
        let gm = c.b.part(limb(g).p[3]).m;
        let d = c.b.at(limb(s).p[3]) - gm.d;
        self.grip = Vec3::new(gm.a.dot(d), gm.b.dot(d), gm.c.dot(d));
        for i in 0..2 {
            let a = self.anchor(c, i);
            self.arms[i] = Arm { dist: (c.b.at(limb(i).p[3]) - a).length(), pose: 1, relax: 1.0, ..Arm::default() };
        }
        self.started = true;
    }

    /// 0x2ac4620: the arm may be used.
    fn enabled(&self, i: usize, ws: [i32; 2]) -> bool {
        self.enable[i] && ws[i] != 4
    }

    /// The aim's anchor: the parent joint (the shoulder) plus the offset in the chest's frame.
    fn anchor(&self, c: &Ctx, i: usize) -> Vec3 {
        let j = if self.parent[i] >= 0 && (self.parent[i] as usize) < c.ch.eff.len() { self.parent[i] as usize } else { limb(i).e[1] };
        let m = c.b.pmat(chest_part()).m;
        let o = self.parent_offset[i];
        c.b.joint_pos(j) + m.a * o.x + m.b * o.y + m.c * o.z
    }

    /// 0x2acf8cc: may this arm point along `dir` (and, with alwayspoint, the aim clamped into the limits).
    fn limits_ok(&self, c: &Ctx, i: usize, dir: Vec3) -> (bool, Vec3) {
        if !self.point {
            return (false, dir);
        }
        let m = c.b.pmat(chest_part()).m;
        let side = if i == 0 { 1.0 } else { -1.0 };
        // (+b is his left: away from the body is +b for the left arm, -b for the right)
        let (fwd, away, up) = (-m.c, m.b * side, m.a);
        let twist: f32 = (*SPINE()).iter().map(|&j| c.ch.actual(j, c.b.frames)[2]).sum();
        let slack = if self.use_twist && c.upright { (0.5584 - side * twist).max(0.0) } else { 0.0 };
        let yaw = away.dot(dir).atan2(fwd.dot(dir));
        let mut ok = -(self.max_across + slack) < yaw && yaw < self.max_away + slack && fwd.dot(dir) > -0.5;
        let mut out = dir;
        if self.falling_limits == 2 || (self.falling_limits == 1 && !c.upright) {
            let flat = (fwd.dot(dir).powi(2) + away.dot(dir).powi(2)).sqrt();
            let pitch = up.dot(dir).atan2(flat);
            let [across, aw, upl, down] = self.limits;
            let inside = -(across + slack) < yaw && yaw < aw + slack && -down < pitch && pitch < upl;
            if self.always_point && !inside {
                let (y, p) = (yaw.clamp(-(across + slack), aw + slack), pitch.clamp(-down, upl));
                out = (fwd * y.cos() + away * y.sin()) * p.cos() + up * p.sin();
                ok = true;
            } else {
                ok &= inside;
            }
        }
        (ok, out)
    }

    /// 0x2ac7aac: point (3), neutral (0xb, never here: no falling flag or probe), or the side/connect choice (7).
    fn mode(&self, c: &Ctx, i: usize, ws: [i32; 2]) -> i32 {
        if !self.enabled(i, ws) {
            return 0;
        }
        let dir = unit(self.arms[i].led - self.anchor(c, i));
        if self.limits_ok(c, i, dir).0 { 3 } else { 7 }
    }

    /// 0x2ade8ac: may this arm take a neutral pose.
    fn neutral_allowed(&self, i: usize, ws: [i32; 2], support_mode: bool) -> bool {
        match ws[i] {
            1 => self.pose_unused[0],
            2 | 3 => true,
            _ => if support_mode { self.pose_unused[1] } else { self.pose_unused[2] },
        }
    }

    /// 0x2ae08ac and 0x2ae0a70: the neutral pose, written when allowed.
    fn neutral(&mut self, c: &mut Ctx, i: usize, ws: [i32; 2], support_mode: bool) {
        let id = match ws[i] {
            0 => if ws[1 - i] == 2 { 10 } else { 11 },
            _ => match self.pistol_neutral { 1 => 6, 2 => 11, _ => 2 },
        };
        self.arms[i].state = 0xb;
        if !self.neutral_allowed(i, ws, support_mode) {
            return;
        }
        self.arms[i].pose = id;
        if let Some((cl, sh, el, wr)) = pose(id) {
            let l = limb(i);
            let e = &mut c.ch.eff;
            e[l.e[0]].desired = cl;
            e[l.e[1]].desired = sh;
            e[l.e[2]].desired[0] = el;
            e[l.e[3]].desired = wr;
        }
    }

    /// 0x2adf898: the gun arm's connect point (pose 7) in the COM's frame, if it may go there.
    fn connect_point(&self, c: &Ctx, i: usize, ws: [i32; 2], two: bool, support_mode: bool) -> Option<Vec3> {
        let on = if two { self.neutral_point.1 } else { self.neutral_point.0 };
        if !on || !self.neutral_allowed(i, ws, support_mode) {
            return None;
        }
        let m = c.m;
        let q = self.point2connect;
        let a = if i == 0 { -m.a } else { m.a };
        let t = c.com + a * q.x + m.b * q.y + m.c * q.z;
        let dir = unit(t - self.anchor(c, i));
        (!self.neutral_point.2 || self.limits_ok(c, i, dir).0).then_some(t)
    }

    /// 0x2add8ec: the gun arm reaches along the aim (mode 1 the target, 10 the connect point); its state 3, or 6 off the aim.
    fn point_arm(&mut self, c: &mut Ctx, i: usize, at: Vec3, connect: bool, two: bool) {
        let l = limb(i);
        let anchor = self.anchor(c, i);
        let (_, dir) = self.limits_ok(c, i, unit(at - anchor));
        let a = self.arms[i];
        let dist = if self.hand_dist[i] >= 0.0 { self.hand_dist[i] } else { a.dist };
        let extra = if connect { self.add_connect } else { 0.0 };
        let mut d = (dist - (a.relax_t / self.relax.0.max(1e-4)) * self.relax.2 + extra).clamp(0.2, self.arm_len.max(0.2));
        if connect && two {
            d = d.min(0.92 * self.arm_len);
        }
        let x = anchor + dir * d;
        let side = if i == 0 { 1.0 } else { -1.0 };
        // RDR2's type-9 reach request (full target matrix): Delirium's arm IK to the point, the wrist straight
        let clav = c.ch.eff[l.e[0]].desired;
        crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[3], x, self.elbow[0], 0.0, side, 1.0, Vec3::ZERO, None);
        let blend = if two && !self.connected { 0.0 } else { self.clav_blend[0] };
        self.share(c, i, blend, clav);
        c.ch.eff[l.e[3]].desired = [0.0; 3];
        // the error: the arm's line (ours, for the weapon's axis) against the aim
        let err = unit(c.b.at(l.p[3]) - anchor).dot(dir).clamp(-1.0, 1.0).acos();
        self.arms[i].state = if err > self.error_threshold { 6 } else { 3 };
    }

    /// The clavicle's share of the reach (the request's clavicle blend).
    fn share(&self, c: &mut Ctx, i: usize, blend: f32, clav: [f32; 3]) {
        let l = limb(i);
        let sh = c.ch.eff[l.e[1]].desired;
        crate::nmpa::share(c.ch, c.b.frames, l.e[0], l.e[1]);
        let (cs, ss) = (c.ch.eff[l.e[0]].desired, c.ch.eff[l.e[1]].desired);
        for k in 0..2 {
            c.ch.eff[l.e[0]].desired[k] = clav[k] + (cs[k] - clav[k]) * blend;
            c.ch.eff[l.e[1]].desired[k] = sh[k] + (ss[k] - sh[k]) * blend;
        }
        c.ch.eff[l.e[0]].desired[2] = clav[2];
    }

    /// 0x2ad2458 with the mode: point, the connect point, or neutral.
    fn dispatch(&mut self, c: &mut Ctx, i: usize, mode: i32, ws: [i32; 2], two: bool, support_mode: bool) {
        match mode {
            3 => self.point_arm(c, i, self.arms[i].led, false, two),
            7 => match self.connect_point(c, i, ws, two, support_mode) {
                Some(t) => {
                    self.arms[i].pose = 7;
                    self.point_arm(c, i, t, true, two);
                }
                None => self.neutral(c, i, ws, support_mode),
            },
            0xb => self.neutral(c, i, ws, support_mode),
            _ => self.arms[i].state = 0,
        }
    }

    /// 0x2adcf54 (in part): the gun arm points; the support hand reaches for its grip on the gun.
    fn two_handed(&mut self, c: &mut Ctx, g: usize, s: usize, ws: [i32; 2], two: bool, dt: f32) {
        let mut mode = self.mode(c, g, ws);
        let gm = c.b.part(limb(g).p[3]).m;
        let grip = gm.d + gm.a * self.grip.x + gm.b * self.grip.y + gm.c * self.grip.z;
        let ls = limb(s);
        let (hand, shoulder) = (c.b.at(ls.p[3]), c.b.joint_pos(ls.e[1]));
        let d = (hand - grip).length();
        let reachable = (grip - shoulder).length() < self.arm_len;
        self.arms[s].reachable = reachable;
        self.connected = d < if self.connected { 0.15 } else { 0.07 };
        let follow = self.enabled(s, ws) && reachable;
        if mode == 3 && !follow {
            mode = 7;
        }
        self.dispatch(c, g, mode, ws, two, true);
        let pointing = matches!(self.arms[g].state, 3 | 6);
        if (pointing || self.always_support) && (self.connected || reachable) && self.enabled(s, ws) {
            // the support request (state 4): weight from the hand's distance, the reach clamped
            let w = if d <= 0.1 { 1.0 } else if d < 0.2 { (0.2 - d) * 10.0 } else { 0.0 };
            let off = grip - shoulder;
            let most = self.arm_len * self.clamp_support;
            let x = shoulder + if off.length() > most { unit(off) * most } else { off };
            let side = if s == 0 { 1.0 } else { -1.0 };
            let clav = c.ch.eff[ls.e[0]].desired;
            crate::nmdbm::limb_ik_w(c.b, c.ch, ls.e[1], ls.e[2], ls.p[0], ls.p[3], x, self.elbow[1], 0.0, side, 1.0, Vec3::ZERO, None, w, false);
            self.share(c, s, if self.connected { self.clav_blend[1] } else { 0.0 }, clav);
            self.arms[s].state = 4;
            // the support pull (supportConstraint 2; ours for 1 too: no hand-to-hand constraint here)
            if self.support_constraint != 0 {
                let r = 0.1;
                if d < 3.0 * r {
                    let k = ((3.0 * r - d) * 0.5 / r).clamp(0.0, 1.0) * self.strength;
                    let (vg, vs) = (c.b.part(limb(g).p[3]).v, c.b.part(ls.p[3]).v);
                    let mut f = (grip - hand) * (k * k) + (vg - vs) * (0.2 * k);
                    if dt < 1.0 / 60.0 {
                        f *= 60.0 * dt;
                    }
                    let sm = c.b.part(ls.p[3]).m;
                    c.out.impulse.push((ls.p[3], sm.d + sm.a * (0.04 * side), f));
                    c.out.impulse.push((limb(g).p[3], grip, -f));
                }
            }
        } else {
            self.neutral(c, s, ws, true);
        }
    }

    /// 0x2adffec: a pointing or supporting arm's gains.
    fn gains(&self, c: &mut Ctx, i: usize, detached: bool, tw: f32) {
        let l = limb(i);
        let k = if detached { self.arm_k_det } else { self.arm_k };
        c.ch.set_stiffness(k * tw, self.arm_d, ["ul", "ur"][i], None);
        let og = if detached { self.grav.1 } else { self.grav.0 };
        let e = &mut c.ch.eff;
        e[l.e[0]].muscle_stiffness = 0.5;
        e[l.e[1]].muscle_stiffness = 0.5;
        e[l.e[2]].muscle_stiffness = 0.5;
        e[l.e[3]].muscle_stiffness = 10.0;
        for j in l.e.into_iter().chain(*SPINE()) {
            e[j].oppose_gravity = og;
        }
    }
}

impl Extra for PointGun {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if !c.ch.rdr2 {
            // without the core: Delirium's pointGun, pointArm at the gun hand's target
            if self.fallback.is_none() {
                let (g, _) = Self::hands(self.states(c));
                self.out.push(format!("pointArm start=true useLeftArm={} target={}", g == 0, v3s(self.target[g])));
                self.fallback = Some(true);
            }
            return;
        }
        if !self.started {
            self.start(c);
        }
        let ws = self.states(c);
        let (g, s) = Self::hands(ws);
        let tw = if self.time_warp.0 && dt > 0.0 { ((1.0 / 30.0) / dt * self.time_warp.1).sqrt().max(1.0) } else { 1.0 };
        // §5.1 the targets, their speed and lead; the fire relax
        let chest = c.b.part(chest_part());
        for i in 0..2 {
            let a = &mut self.arms[i];
            (a.was, a.state) = (a.state, 0);
            a.target = self.target[i];
            a.vel = if a.prev.length_squared() >= 1e-4 && dt > 0.0 { (a.target - a.prev) / dt } else { Vec3::ZERO };
            a.led = a.target;
            if self.lead > 0.0 {
                a.led += (a.vel - chest.v) * (self.lead * (1.0 / 60.0) / tw);
            }
            let amount = self.relax.1;
            a.relax = (1.0 - a.relax_t / self.relax.0.max(1e-4)) * amount + (1.0 - amount);
            a.relax_t = (a.relax_t - dt).clamp(0.0, 5.0);
        }
        // §5.2 the modes
        let two = (ws[g] == 2 && ws[s] == 0) || (ws[s] == 2 && ws[g] == 0);
        let support_mode = two || (ws[g] == 1 && ws[s] == 0 && self.support_pistol);
        if !support_mode {
            self.connected = false;
        }
        // §5.3 per arm, right first
        let mut set = [false; 2];
        for i in [1, 0] {
            if set[i] {
                continue;
            }
            let st = if ws[i] == 2 && ws[1 - i] != 0 { 3 } else { ws[i] };
            match st {
                1 if !support_mode => {
                    let m = self.mode(c, i, ws);
                    self.dispatch(c, i, m, ws, false, false);
                    set[i] = true;
                    if ws[1 - i] == 0 {
                        self.neutral(c, 1 - i, ws, false);
                        set[1 - i] = true;
                    }
                }
                1 | 2 => {
                    self.two_handed(c, i, 1 - i, ws, two, dt);
                    set = [true; 2];
                }
                3 => {
                    self.neutral(c, i, ws, support_mode);
                    set[i] = true;
                }
                _ => {}
            }
        }
        for i in 0..2 {
            if self.arms[i].state > 2 && self.arms[i].state != 0xb {
                self.gains(c, i, support_mode && !self.connected, tw);
            }
        }
        // §5.4 the look: the armed hands' targets (last tick's), or ahead of the chest
        let armed: Vec<usize> = (0..2).filter(|&i| ws[i] != 0 && ws[i] != 4 && self.enable[i]).collect();
        let look = if armed.is_empty() {
            c.b.pmat(chest_part()).m.d - c.b.pmat(chest_part()).m.a * 3.0
        } else {
            armed.iter().map(|&i| self.arms[i].prev).sum::<Vec3>() / armed.len() as f32
        };
        let pointing = (0..2).any(|i| matches!(self.arms[i].state, 3 | 6));
        if look.length() > 0.01 && self.use_look {
            if self.looking {
                self.out.push(format!("headLook pos={}", v3s(look)));
            } else {
                self.out.push(format!("headLook start=true alwaysLook=true pos={}", v3s(look)));
                self.looking = true;
            }
        }
        // §5.5 spine twist and the balancer's heading
        if self.use_twist && look.length() > 0.01 {
            c.out.spine_twist = Some((look, crate::nmst::SpineTwist::default().stiffness));
        }
        if self.use_turn {
            let d = look - chest.m.d;
            c.out.dbm_turn = Some(if pointing || self.always_twist { unit(d - c.up * d.dot(c.up)) * 0.5 } else { Vec3::ZERO });
        }
        // §5.6 bookkeeping: the targets kept, the gun arm's state change told
        for i in 0..2 {
            self.arms[i].prev = self.arms[i].target;
        }
        if self.arms[g].state != self.arms[g].was {
            c.out.send(delirium_data::kv!(127 int), "pointgun", self.arms[g].state as f32);
        }
    }

    fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.out)
    }

    /// RDR2's controlledFall: timeWarpStrengthScale from it, and the left hand off (RDR2 turns enableLeft off when
    /// either hand's weapon state is 0; Delirium's left hand never holds a weapon, see `states`).
    fn pg_set(&mut self, time_warp_scale: f32) {
        self.time_warp.1 = time_warp_scale;
        self.enable[0] = false;
    }

    fn fire(&mut self) {
        for a in &mut self.arms {
            a.relax_t = self.relax.0;
        }
    }

    fn update(&mut self, p: &mut P) -> Result<(), Error> {
        self.target = [p.v("leftHandTarget", self.target[0])?, p.v("rightHandTarget", self.target[1])?];
        if self.fallback.is_some() {
            let g = if self.target[1] != Vec3::ZERO { 1 } else { 0 };
            self.out.push(format!("pointArm target={}", v3s(self.target[g])));
        }
        for q in PARAMS.iter().chain(UNREAD) {
            p.get(q.name);
        }
        Ok(())
    }

    fn stop(&mut self, c: &mut Ctx) {
        if self.fallback.is_some() {
            self.out.push("pointArm start=false".into());
            return;
        }
        if self.looking {
            self.out.push("headLook start=false".into());
        }
        if self.use_twist {
            c.out.spine_twist_stop = true;
        }
        if self.use_turn {
            c.out.dbm_turn = Some(Vec3::ZERO);
        }
        for i in 0..2 {
            for j in limb(i).e {
                c.ch.eff[j].oppose_gravity = 0.0;
            }
        }
    }
}
