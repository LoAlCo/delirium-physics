//! RDR2 core: RDR2's `quadruped` behaviour (slot 0x130, the horse's balancer) and the quadruped body's hoof contact
//! and gravity opposition, from rdr2_quadruped.md (and rdr2_core2.md 1 for the gravity opposition). Runs only with
//! the RDR2 core on (`Horse::rdr2`); without it the horse is Delirium's own (horse.rs), unchanged.
//!
//! The horse stands by its legs: a body frame is kept level under the centre of mass (with the rider's mass when
//! one is attached), tilted by the look-ahead centre of mass's offset (gains 10 and 2, at most 0.3 rad), and each
//! hoof is aimed at its place in that frame (its target moving at most 1.5 m/s). When the look-ahead centre of mass
//! leaves the polygon of the planted hooves it steps, one leg at a time (duration = step length × steptimescale).
//! A hoof is planted when its contact bit is set with a normal within 60° of up, held 0.025 s. The states are
//! standing, stepping, falling, falling with a ground hit and down; strength falls linearly, and after
//! maxBalanceTime in support the legs on the side it leans to give way. Down it scrabbles (hoof circles a quarter
//! turn apart), writhes (random re-picked pointing of its neck), lifts its head toward its facing and gets the yaw
//! and roll torques.
//!
//! Ours, where Delirium differs or the note is a guess: there is no quadruped leg IK in the note (its solver is
//! unread), so each leg's hip turns and its knee or hock (a hinge here) bends, the bend nearest the last one, with the
//! fetlock held at its pose, to put the hoof's sole (where it met the floor as it stood) at the target. The COM matrix
//! is the barrel's middle (Spine2) at the centre of mass, with a its up, b its left and c its back (fore hooves at
//! -c). The body frame's origin is H below the COM along u (along a, as the note reads, it swung out from under the
//! COM as the body pitched), turned from a onto u (level without a push). legLen (ctx+0x2bfc) is the standing hip
//! height over the soles. RDR2's angular velocity row (+0xf70) and |ω| are the barrel's. Fewer than 3 planted hooves
//! give a margin of -1. The walk order of the step trigger (fore left, hind right, fore right, hind left) and its
//! target (the neutral place plus the look-ahead offset) are ours; the swing's lift factor is taken over 0.15 m (read
//! as metres it lifted hooves 1 m). The restep distance (+0x178, not in the handler, default unread) is 0: off.
//! Gravity opposition: RDR2's law locks a connected leg and opposes everything above it, 1/n per leg; on a horse
//! standing on legs spread fore and aft (and with its fore legs on Spine3, not the root) that pushed the hooves
//! together and launched it, so each joint here opposes the weight on its far side less the ground's push on the
//! planted hooves there, those pushes shared by the least-norm shares that put their centre under the COM (at the
//! contact points). Its cap 0.2·strength·opposeGravity takes strength as our muscle's s² times the far side's moment
//! of inertia about the joint (Delirium's muscles are frequencies, RDR2's per-character stiffness scale is unread),
//! opposeGravity 1; its support point is the planted hooves' polygon. By the same reading of strength, a planted
//! hoof's joint (fetlock, carpus) has its muscle raised by √(body's inertia / hoof's) while it stands (Delirium's
//! muscle there moves the hoof alone, and the leg folded under the body; raising the hips and knees too made the legs
//! shove the hooves along the ground). The rider (defineAttachedObject) is his mass at the saddle, and also hangs his
//! weight on the barrel, since Delirium swings him up from beside the horse and his ropes hang on world anchors.
//! Delirium has no animation driving a sound horse, so until it's first hit, lamed or falls it runs as its own caller
//! would send it: strengthdecay 0, maxBalanceTime -1, step weakening 0, and height its standing COM height (RDR2's
//! H = 1.0 x height crouched it 0.2 m); then RDR2's values. The lying timer counts while the roll amount is under 0.5
//! (the note reads "> 0.5", which would keep a still horse from ever settling). The lean side that gives way is the
//! lower one. The writhe re-picks every 1/writhespeed s and turns its direction about up and left; pointing turns
//! each joint a 1/n share; the head lift points E..H. The roll spring's "c·dir" is taken as the side tilt b·w, and its
//! mass factor (ctx+0x2c0c) as 1. The snap is Delirium's snap impulse shared over the part hit and its neighbours.
//! Our pelvis is the root: the base curve's pelvis joint (RDR2's root link) is left at its pose. Lamed legs keep
//! Delirium's weakening.
//! Left out: the leg fold poses of the roll (unread: rolling legs stand), the roll prediction, the swing's reach term
//! and flags (p144, p140), the hind-leg bend and the IK scalar (p194, p148), gait pairs (gaitstate > 0), the bias
//! timer, angVelScale's looseness, the contact class 0x1822 double count, frictionScale, the muscle stiffness
//! (0x2128bd54: Delirium's muscle has none), the stayUpright righting torque, the speed gate of the body snap, the
//! floor-joint table (the part touched and its children are used), measuresupport's averaged half-sizes, the
//! feedback record (the state is shown in `state`), the double-stance pairing of the opposition (biped joints).
use super::{Horse, LAME, LEGS, MID, WELD, WELDED};
use bevy::prelude::*;
use delirium::collision::Collision;
use delirium::ragdoll::{Capsule, Drive, Limit, Muscle, Obstacle, GRAVITY};
use std::f32::consts::{FRAC_PI_2, PI, TAU};

/// RDR2's legs, fore left, fore right, hind left, hind right (rdr2_quadruped.md 1: leg 0's ankle is horse.xml's joint
/// 18 Wrist_Left, 1's 16 Wrist_Right, 2's 8 Ankle_Left, 3's 6 Ankle_Right), as rows of `LEGS` (hind right, hind left,
/// fore right, fore left).
const LEG: [usize; 4] = [3, 2, 1, 0];
/// The barrel and neck base the behaviour pushes on: Pelvis, Spine1, Spine2, Spine3, Neck1.
const PUSHED: [usize; 5] = [0, 5, 11, 12, 15];
/// The spine/neck chain A..H (Pelvis, Spine1, Spine2, Spine3, Neck0, Neck1, Neck2, Head joints) as our parts: the
/// part each joint moves (the pelvis joint's is RDR2's root link, which hangs from our pelvis).
const CHAIN: [usize; 8] = [1, 5, 11, 12, 15, 21, 23, 24];
/// The base spine/neck curve: lean1 on A..H every tick, under everything else.
const POSTURE: [f32; 8] = [0.01, 0.19, 0.0, -0.15, -0.08, -0.19, -0.10, 0.0];
/// The writhe and head-lift mask 0x709822: A, B, C, E, F, G, H.
const WRITHE: [usize; 7] = [0, 1, 2, 4, 5, 6, 7];
/// The neck and head (E..H), pointed at the facing by the head lift.
const NECK: [usize; 4] = [4, 5, 6, 7];
/// Floor weakness joint mask 0xfff5f96e (RDR2 joints).
const FLOOR_MASK: u32 = 0xfff5_f96e;
/// The step order when walking (RDR2 legs).
const WALK: [usize; 4] = [3, 2, 0, 1];
/// The hoof contact hold, s.
const HOLD: f32 = 0.025;
/// Gravity opposition's look-ahead, s (ctx+0x2b40).
const AHEAD: f32 = 0.1;

/// Our part for RDR2's link `l` (horse.xml's part order: root, pelvis, ... ; ours has the pelvis first and the
/// neck's and head's second capsules after their firsts).
fn link_part(l: usize) -> usize {
    match l {
        0 => 1,
        1 => 0,
        2..=15 => l,
        16..=20 => l + 1,
        21 => 23,
        _ => 24,
    }
}

/// RDR2's `quadruped` settings: (name, default, min, max) in the order of its default block (0xc8 on). The names
/// RDR2's strings didn't give are their hashes; rdr2_quadruped.md 9 says what each does. Yes/no ones are 1/0.
pub const TABLE: [(&str, f32, f32, f32); 85] = [
    ("legStiffness", 15.0, 6.0, 17.0), ("legStiffnessMin", 7.0, 3.0, 30.0), ("legDamping", 1.0, 0.0, 2.0),
    ("0x2128bd54", 1.0, 0.0, 10.0), ("spineStiffness", 15.0, 6.0, 30.0), ("spinestiffnessmin", 7.0, 3.0, 30.0),
    ("spineDamping", 1.0, 0.0, 2.0), ("0xe4adac13", 4.0, 0.0, 30.0), ("pitchoffset", 0.0, -1.0, 1.0),
    ("rolloffset", 0.0, -1.0, 1.0), ("height", 1.0, 0.0, 2.0), ("strengthdecay", 0.5, 0.0, 10.0),
    ("0xd5842c66", 1.0, 0.0, 10.0), ("0x1689ff1e", 0.1, 0.0, 1.0), ("rampheighttime", -1.0, -1.0, 10.0),
    ("maxsupporttime", -1.0, -1.0, 10.0), ("frictionScale", 1.0, 0.0, 10.0), ("frictionscaleend", 1.0, 0.0, 10.0),
    ("0x1f3617d5", 0.1, 0.0, 10.0), ("maxsteplength", 0.6, 0.0, 1.0), ("steptimescale", 1.0, 0.0, 10.0),
    ("stepheightscale", 1.0, 0.0, 10.0), ("stepthreshold", 0.0, -1.0, 1.0), ("gaitphase", 0.0, 0.0, 1.0),
    ("0x6ce93cc5", 0.24, 0.0, 2.0), ("0xe6200ae9", 0.67, 0.0, 2.0), ("0x60b32079", -0.15, -2.0, 2.0),
    ("0x78748c58", 1.0, 0.0, 10.0), ("0xdcaa8fcc", 0.1, 0.0, 10.0), ("leanAgainstVelocity", 0.0, -4.0, 4.0),
    ("0x3ba69a5e", 1.0, 0.0, 1.0), ("0xbf41a9ff", 1.0, -1.0, 2.0), ("0x0a364789", 1.0, 0.0, 10.0),
    ("anklestrengthscale", 1.0, 0.0, 2.0), ("snapMag", 1.0, -5.0, 5.0), ("initialWeaknessZeroDuration", 0.0, 0.0, 10.0),
    ("initialWeaknessRampDuration", 0.4, 0.0, 10.0), ("floorWeaknessTime", 0.1, 0.0, 10.0), ("floorStrength", 0.1, 0.0, 1.0),
    ("floorJointWeaknessTime", 0.1, 0.0, 10.0), ("floorJointStrength", 0.01, 0.0, 1.0), ("footslipcompensation", 0.0, 0.0, 100.0),
    ("maxBalanceTime", 1.0, -1.0, f32::MAX), ("writhespeed", 0.5, 0.0, 10.0), ("minscrabblespeed", 3.0, 0.0, 20.0),
    ("maxscrabblespeed", 11.0, 0.0, 20.0), ("angVelScale", 1.0, 0.0, 1.0), ("rightingTorque", 0.0, 0.0, 10.0),
    ("failangle", 0.6, -1.0, 1.0), ("0xbda62333", 0.25, 0.0, 0.6), ("rollvelocity", 0.7, 0.0, 10.0),
    ("rollscale", 0.7, 0.0, 10.0), ("rollpredictiontime", 0.0, 0.0, 10.0), ("0x18b9c869", 1.0, 0.0, 10.0),
    ("0x75a875d8", 0.5, 0.0, 10.0), ("0x65b9a730", 2.0, 0.0, 10.0), ("0x84cf43be", 4.0, 0.0, 10.0),
    ("0x6ded80a2", 1.0, 0.0, 10.0), ("0x33a9d629", 0.3, 0.0, 10.0), ("0xc55d4193", 10.0, 0.0, 100.0),
    ("0x1179b8be", 2.0, 0.0, 100.0), ("lookahead", 0.5, 0.1, 10.0), ("0xcdd11768", 0.0, -100.0, 100.0),
    ("0x463334cb", 0.0, -10000.0, 10000.0), ("0xf08a586f", 0.3, 0.0, 1.57), ("0x491b9b33", 0.0, 0.0, 10000.0),
    ("0x741b990a", 0.0, 0.0, 10.0), ("gaitstate", 0.0, 0.0, 4.0), ("0x8c66dab4", -1.0, -1.0, 4.0),
    ("initialstep", -1.0, -1.0, 3.0), ("alwayslifthead", 0.0, 0.0, 1.0), ("scrabbleonside", 1.0, 0.0, 1.0),
    ("forcescrabble", 0.0, 0.0, 1.0), ("measuresupport", 0.0, 0.0, 1.0), ("newHit", 0.0, 0.0, 1.0),
    ("Snap", 0.0, 0.0, 1.0), ("Writhe", 0.0, 0.0, 1.0), ("writheonground", 1.0, 0.0, 1.0),
    ("rollonground", 1.0, 0.0, 1.0), ("0x839ff632", 0.0, 0.0, 1.0), ("step", 1.0, 0.0, 1.0),
    ("0xdf7a800d", 1.0, 0.0, 1.0), ("OnFire", 0.0, 0.0, 1.0), ("0x76e322dc", 0.0, -f32::MAX, f32::MAX),
    ("0x178", 0.0, 0.0, 10.0),
];
const LEG_STIFF: usize = 0;
const LEG_MIN: usize = 1;
const LEG_DAMP: usize = 2;
const SPINE_STIFF: usize = 4;
const SPINE_MIN: usize = 5;
const SPINE_DAMP: usize = 6;
const SPINE_DOWN: usize = 7;
const PITCH_OFF: usize = 8;
const ROLL_OFF: usize = 9;
const HEIGHT: usize = 10;
const DECAY: usize = 11;
const DECAY_DOWN: usize = 12;
const STEP_WEAKEN: usize = 13;
const RAMP_HEIGHT: usize = 14;
const MAX_SUPPORT: usize = 15;
const MAX_STEP: usize = 19;
const STEP_TIME: usize = 20;
const STEP_HEIGHT: usize = 21;
const STEP_THRESHOLD: usize = 22;
const HALF_WIDTH: usize = 24;
const HALF_LENGTH: usize = 25;
const OFFSET: usize = 26;
const ANKLE_SCALE: usize = 33;
const SNAP_MAG: usize = 34;
const WEAK_ZERO: usize = 35;
const WEAK_RAMP: usize = 36;
const FLOOR_TIME: usize = 37;
const FLOOR_STRENGTH: usize = 38;
const FLOOR_JOINT_TIME: usize = 39;
const FLOOR_JOINT_STRENGTH: usize = 40;
const FOOT_SLIP: usize = 41;
const MAX_BALANCE: usize = 42;
const WRITHE_SPEED: usize = 43;
const SCRABBLE_MIN: usize = 44;
const SCRABBLE_MAX: usize = 45;
const RIGHTING: usize = 47;
const FAIL: usize = 48;
const ROLL_VEL: usize = 50;
const ROLL_SCALE: usize = 51;
const DOWN_GATE: usize = 53;
const YAW_CAP: usize = 54;
const ROLL_CAP: usize = 55;
const ROLL_GAIN: usize = 56;
const FADE_FAST: usize = 57;
const FADE_SLOW: usize = 58;
const TILT_ROLL: usize = 59;
const TILT_PITCH: usize = 60;
const LOOKAHEAD: usize = 61;
const HIT_LEAN: usize = 62;
const HIT_PUSH: usize = 63;
const HIT_ANGLE: usize = 64;
const LIFT_FORCE: usize = 65;
const LIFT_TIME: usize = 66;
const LIFT_HEAD: usize = 70;
const SCRABBLE_SIDE: usize = 71;
const FORCE_SCRABBLE: usize = 72;
const MEASURE: usize = 73;
const NEW_HIT: usize = 74;
const SNAP: usize = 75;
const WRITHE_ON: usize = 76;
const WRITHE_GROUND: usize = 77;
const ROLL_GROUND: usize = 78;
const STEP: usize = 80;
const FACING_VEL: usize = 81;
const ON_FIRE: usize = 82;
const RESTEP: usize = 84;

/// The settings, as RDR2's message gives them.
#[derive(Clone, Debug)]
pub struct Params {
    pub v: [f32; 85],
    pub mask: String,
    pub normal: Vec3,
    pub hit_point: Vec3,
    pub bullet_vel: Vec3,
    pub local_hit: i32,
}

impl Default for Params {
    fn default() -> Self {
        Params { v: TABLE.map(|t| t.1), mask: "fb".into(), normal: Vec3::Z, hit_point: Vec3::ZERO, bullet_vel: Vec3::ZERO, local_hit: 0 }
    }
}

impl Params {
    /// Sets one by RDR2's name (or hash), clamped to its range; yes/no as 1/0 or true/false; vectors as x,y,z.
    pub fn set(&mut self, name: &str, value: &str) -> bool {
        let vec = || {
            let v: Vec<f32> = value.split(',').filter_map(|x| x.trim().parse().ok()).collect();
            (v.len() == 3).then(|| Vec3::new(v[0], v[1], v[2]))
        };
        match name.to_ascii_lowercase().as_str() {
            "angvelscalemask" => { self.mask = value.into(); return true }
            "normal" => return vec().map(|v| self.normal = v).is_some(),
            "hitpoint" => return vec().map(|v| self.hit_point = v).is_some(),
            "bulletvel" => return vec().map(|v| self.bullet_vel = v).is_some(),
            "localhitpointinfo" => return value.parse().map(|v| self.local_hit = v).is_ok(),
            _ => {}
        }
        let Some(i) = TABLE.iter().position(|t| t.0.eq_ignore_ascii_case(name)) else { return false };
        let v = match value {
            "true" => 1.0,
            "false" => 0.0,
            v => match v.parse::<f32>() {
                Ok(v) => v,
                Err(_) => return false,
            },
        };
        self.v[i] = v.clamp(TABLE[i].2, TABLE[i].3);
        true
    }

    fn on(&self, i: usize) -> bool {
        self.v[i] != 0.0
    }
}

#[derive(Clone, Copy, Default)]
struct Leg {
    /// where the hoof goes in the body frame (a up, b left, c back), and where it's aimed now (moving toward it)
    goal: Vec3,
    target: Vec3,
    stepping: bool,
    start: f32,
    end: f32,
    from: Vec3,
    to: Vec3,
    /// the step's remaining horizontal distance, kept non-increasing
    left: f32,
    /// the contact hold timer
    hold: f32,
    /// the knee's (or hock's) hinge angle last solved
    knee: f32,
}

/// The behaviour's state (B+0x250 on).
pub struct Core {
    p: Params,
    t: f32,
    since_hit: f32,
    weak_clock: f32,
    state: usize,
    entered3: bool,
    h: f32,
    unit: f32,
    leg_len: f32,
    com_h: f32,
    airborne: bool,
    free: bool,
    origin: Vec3,
    rows: [Vec3; 3],
    tilt: f32,
    sign: f32,
    flail_sign: f32,
    flail: f32,
    flail_t: f32,
    lying_t: f32,
    on_parts: bool,
    lying: bool,
    support_t: f32,
    out_t: f32,
    fs: f32,
    fs2: f32,
    fl: f32,
    legs: [Leg; 4],
    neutral: [Vec3; 4],
    next: usize,
    phase: f32,
    sigma: f32,
    writhe: Vec3,
    writhe_t: f32,
    roll: f32,
    fade: [f32; 2],
    push_lean: Vec3,
    push_force: Vec3,
    facing: Vec3,
    rng: (u32, u32),
    floor_t: f32,
    floor_part: Option<usize>,
    /// Delirium's: not hit, lamed or fallen yet (see the module's notes)
    sound: bool,
}

impl Core {
    fn rand(&mut self) -> f32 {
        let t = 0x5cdc_faa7u64 * self.rng.0 as u64 + self.rng.1 as u64;
        self.rng = (t as u32, (t >> 32) as u32);
        self.rng.0 as f32 / 4_294_967_296.0
    }

    /// The setting, with Delirium's sound-horse values for the strength decay, maxBalanceTime and step weakening.
    fn p(&self, i: usize) -> f32 {
        match i {
            DECAY | STEP_WEAKEN if self.sound => 0.0,
            MAX_BALANCE if self.sound => -1.0,
            _ => self.p.v[i],
        }
    }

    fn to_world(&self, l: Vec3) -> Vec3 {
        self.origin + self.rows[0] * l.x + self.rows[1] * l.y + self.rows[2] * l.z
    }

    fn to_local(&self, w: Vec3) -> Vec3 {
        let d = w - self.origin;
        Vec3::new(d.dot(self.rows[0]), d.dot(self.rows[1]), d.dot(self.rows[2]))
    }
}

/// A rotation's lean1, lean2 and twist in a joint's frames (as `in_limits` reads them), and back.
fn angles(pf: Quat, cf: Quat, rel: Quat) -> Vec3 {
    let r = pf.inverse() * rel * cf;
    let r = if r.w < 0.0 { -r } else { r };
    let tw = Quat::from_xyzw(0.0, 0.0, r.z, r.w).normalize();
    let sw = (r * tw.inverse()).to_scaled_axis();
    Vec3::new(sw.y, sw.x, 2.0 * tw.z.atan2(tw.w))
}

fn from_angles(pf: Quat, cf: Quat, a: Vec3) -> Quat {
    pf * Quat::from_scaled_axis(Vec3::new(a.y, a.x, 0.0)) * Quat::from_rotation_z(a.z) * cf.inverse()
}

/// Signed distance of `p` into the convex hull of `pts` in the plane normal to `up` (positive inside).
fn margin(pts: &[Vec3], p: Vec3, up: Vec3) -> f32 {
    let flat = |v: Vec3| v - up * v.dot(up);
    let c = pts.iter().map(|&v| flat(v)).sum::<Vec3>() / pts.len() as f32;
    let (x, y) = {
        let x = (flat(pts[0]) - c).normalize_or(up.any_orthonormal_vector());
        (x, up.cross(x))
    };
    let mut ring: Vec<Vec3> = pts.iter().map(|&v| flat(v)).collect();
    ring.sort_by(|a, b| (a - c).dot(y).atan2((a - c).dot(x)).total_cmp(&(b - c).dot(y).atan2((b - c).dot(x))));
    let q = flat(p);
    let mut inside = f32::MAX;
    let mut out = f32::MAX;
    let mut all_in = true;
    for i in 0..ring.len() {
        let (a, b) = (ring[i], ring[(i + 1) % ring.len()]);
        let e = b - a;
        let n = up.cross(e).normalize_or_zero();
        let d = (q - a).dot(n);
        inside = inside.min(d);
        all_in &= d >= 0.0;
        let s = ((q - a).dot(e) / e.length_squared().max(1e-9)).clamp(0.0, 1.0);
        out = out.min((a + e * s).distance(q));
    }
    if all_in { inside } else { -out }
}

impl Horse {
    fn hoof_at(&self, fr: &[Transform], l: usize, q: [Quat; 3]) -> (Vec3, Vec3) {
        let leg = LEGS[l];
        let p = self.build.parts[leg[0]].parent.unwrap_or(0);
        let (mut rot, mut at) = (fr[p].rotation, fr[p].translation);
        let mut hip = at;
        for (i, &k) in leg.iter().enumerate() {
            let d = &self.build.parts[k];
            let j = at + rot * d.anchor_parent;
            if i == 0 {
                hip = j;
            }
            rot = rot * q[i];
            at = j - rot * d.anchor;
        }
        (hip, at + rot * self.sole(l))
    }

    /// Where leg `l`'s (a `LEGS` row) hoof meets the floor as it stood, in the hoof's frame.
    fn sole(&self, l: usize) -> Vec3 {
        let low = self.build.parts.iter().zip(&self.pose).map(|(d, f)| f.translation.z - (f.rotation * Vec3::Y).z.abs() * d.half - d.radius).fold(f32::MAX, f32::min);
        let f = self.pose[LEGS[l][2]];
        f.rotation.inverse() * Vec3::new(0.0, 0.0, low - f.translation.z)
    }

    /// Leg `l`'s sole now, world.
    fn sole_at(&self, fr: &[Transform], l: usize) -> Vec3 {
        let f = fr[LEGS[l][2]];
        f.translation + f.rotation * self.sole(l)
    }

    /// Ours (the note's quadruped IK is unread): the hip, knee and fetlock turns that put leg `l`'s (a `LEGS` row)
    /// sole at `to`: the knee's hinge bent from `knee` till the hip-to-sole distance fits, the hip turned to aim it,
    /// the fetlock at its pose.
    fn leg_ik(&self, fr: &[Transform], l: usize, to: Vec3, knee: f32) -> ([Quat; 3], f32) {
        let leg = LEGS[l];
        let mut q = leg.map(|k| self.targets[k]);
        let mut t = knee;
        if let Limit::BoneHinge { pf, cf, lo, hi } = self.build.parts[leg[1]].limit {
            let (lo, hi) = (lo + 0.03, hi - 0.03);
            let at = |t: f32| {
                let mut q = q;
                q[1] = pf * Quat::from_rotation_z(t) * cf.inverse();
                let (hip, hoof) = self.hoof_at(fr, l, q);
                hoof.distance(hip)
            };
            let hip = self.hoof_at(fr, l, q).0;
            let want = to.distance(hip);
            // the bend nearest the last one (from the pose's: its own way of bending) that fits, or the nearest fit
            let pose = knee;
            let ts: Vec<f32> = (0..=32).map(|i| lo + (hi - lo) * i as f32 / 32.0).collect();
            let ds: Vec<f32> = ts.iter().map(|&t| at(t) - want).collect();
            let mut best: Option<(f32, f32, f32)> = None;
            for i in 0..32 {
                if ds[i] * ds[i + 1] <= 0.0 {
                    let mid = 0.5 * (ts[i] + ts[i + 1]);
                    if best.is_none_or(|b| (mid - pose).abs() < (0.5 * (b.0 + b.1) - pose).abs()) {
                        best = Some((ts[i], ts[i + 1], ds[i]));
                    }
                }
            }
            t = match best {
                Some((mut a, mut b, fa)) => {
                    for _ in 0..12 {
                        let m = 0.5 * (a + b);
                        if (at(m) - want) * fa > 0.0 { a = m } else { b = m }
                    }
                    0.5 * (a + b)
                }
                None => ts[(0..33).min_by(|&i, &j| ds[i].abs().total_cmp(&ds[j].abs())).unwrap_or(0)],
            };
            q[1] = pf * Quat::from_rotation_z(t) * cf.inverse();
        }
        let (hip, hoof) = self.hoof_at(fr, l, q);
        let p = self.build.parts[leg[0]].parent.unwrap_or(0);
        let rp = fr[p].rotation;
        if let (Some(u), Some(v)) = ((hoof - hip).try_normalize(), (to - hip).try_normalize()) {
            q[0] = rp.inverse() * Quat::from_rotation_arc(u, v) * rp * q[0];
        }
        (q, t)
    }

    /// The hinge angle of the knee in a relative turn.
    fn knee_angle(&self, k: usize, rel: Quat) -> f32 {
        match self.build.parts[k].limit {
            Limit::BoneHinge { pf, cf, .. } => {
                let r = pf.inverse() * rel * cf;
                let r = if r.w < 0.0 { -r } else { r };
                2.0 * r.z.atan2(r.w)
            }
            _ => 0.0,
        }
    }

    /// The COM matrix: the barrel's middle's up (a), left (b) and back (c).
    fn com_rows(&self, fr: &[Transform]) -> [Vec3; 3] {
        let a = (fr[MID].rotation * self.stand).normalize_or(Vec3::Z);
        let f = fr[MID].rotation * self.fwd;
        let c = -(f - a * a.dot(f)).normalize_or(Vec3::X);
        [a, c.cross(a), c]
    }

    /// The centre of mass and its velocity, with the attached rider's mass.
    fn com_rider(&self) -> (Vec3, Vec3, f32) {
        let (x, v) = self.doll.com();
        let m = self.doll.mass();
        match self.rider {
            Some((p, pv, rm)) if rm > 0.0 => ((x * m + p * rm) / (m + rm), (v * m + pv * rm) / (m + rm), m + rm),
            _ => (x, v, m),
        }
    }

    fn activate(&mut self, world: &Collision, fr: &[Transform]) -> Core {
        // a held horse is let go: it stands on its legs now
        if self.held {
            self.held = false;
            self.doll.unpin();
            for (k, at, imp) in std::mem::take(&mut self.pushes) {
                self.doll.impulse(k, at, imp);
            }
        }
        let p = self.r.clone();
        let rows = self.com_rows(fr);
        let (com, _, _) = self.com_rider();
        // the hips' height over the soles as it stood: legLen
        let low = self.build.parts.iter().zip(&self.pose).map(|(d, f)| f.translation.z - (f.rotation * Vec3::Y).z.abs() * d.half - d.radius).fold(f32::MAX, f32::min);
        let leg_len = LEG.iter().map(|&l| {
            let (hip, _) = self.hoof_at(&self.pose, l, LEGS[l].map(|k| self.targets[k]));
            hip.z - low
        }).sum::<f32>() / 4.0;
        // RDR2's standing height: body+0x3b4 (1.0) times height
        let unit = 1.0;
        // Delirium's sound horse (see `Core::p`) stands at its own height: the COM over its hooves' soles
        let h = unit * if self.legs.iter().all(|&l| l == 0.0) && !self.dead { (com.z - low).max(0.1) / unit } else { p.v[HEIGHT] };
        let (w, ln, o) = (p.v[HALF_WIDTH], p.v[HALF_LENGTH], p.v[OFFSET]);
        let mut neutral = [Vec3::new(0.0, w, -ln - o), Vec3::new(0.0, -w, -ln - o), Vec3::new(0.0, w, ln - o), Vec3::new(0.0, -w, ln - o)];
        let origin = com - rows[0] * h;
        let mut c = Core {
            p, t: 0.0, since_hit: 1e5, weak_clock: 1e3, state: 0, entered3: false, h, unit, leg_len, com_h: 10.0, airborne: false,
            free: false, origin, rows, tilt: 0.0, sign: 1.0, flail_sign: 2.0, flail: 0.0, flail_t: 0.0, lying_t: 0.0, on_parts: false,
            lying: false, support_t: 0.0, out_t: 0.0, fs: 1.0, fs2: 1.0, fl: 1.0, legs: [Leg::default(); 4], neutral, next: 0,
            phase: 0.0, sigma: 0.0, writhe: Vec3::ZERO, writhe_t: 0.0, roll: 0.0, fade: [1.0; 2], push_lean: Vec3::ZERO,
            push_force: Vec3::ZERO, facing: -rows[2], rng: (123, 321), floor_t: 1e3, floor_part: None, sound: true,
        };
        if c.p.on(MEASURE) {
            for (i, &l) in LEG.iter().enumerate() {
                let m = c.to_local(fr[LEGS[l][2]].translation);
                neutral[i] = Vec3::new(0.0, m.y, m.z);
            }
            c.neutral = neutral;
        }
        // the character's random numbers seed the scrabble and writhe periods
        c.rng = (0x1234_5678 ^ (self.health as u32), 321);
        c.next = (c.p.v[67] as usize).min(3);
        if c.p.v[69] >= 0.0 {
            c.next = c.p.v[69] as usize;
        }
        for (i, &l) in LEG.iter().enumerate() {
            let hoof = self.sole_at(fr, l);
            let at = c.to_local(hoof);
            c.legs[i] = Leg { goal: c.neutral[i], target: at, knee: self.knee_angle(LEGS[l][1], self.targets[LEGS[l][1]]), ..default() };
        }
        let _ = world;
        c
    }

    /// One physics step with the RDR2 core: RDR2's `quadruped` behaviour.
    pub(super) fn step_rdr2(&mut self, world: &Collision, dt: f32, obstacles: &[Obstacle], others: &[Capsule]) {
        let fr: Vec<Transform> = self.doll.frames().collect();
        let mut c = match self.core.take() {
            Some(c) => c,
            None => Box::new(self.activate(world, &fr)),
        };
        // a newHit sent as a message: its bulletVel at hitPoint, on the part nearest it
        if self.r.on(NEW_HIT) {
            self.r.v[NEW_HIT] = 0.0;
            let part = self.doll.nearest(&self.build, self.r.hit_point);
            self.rdr2_hits.push((part, self.r.hit_point, self.r.bullet_vel));
        }
        c.p = self.r.clone();
        let n = self.build.parts.len();
        let (mut force, mut torque) = (vec![Vec3::ZERO; n], vec![Vec3::ZERO; n]);
        let w = Vec3::Z;
        let g = GRAVITY();
        let rows = self.com_rows(&fr);
        let [a, b, cc] = rows;
        let (com, vel, mass) = self.com_rider();
        let omega = self.doll.vel(MID).1;
        let (speed, spin) = (vel.length(), omega.length());
        let leg_len = c.leg_len;

        // 1. timers, a new hit first
        c.t += dt;
        c.since_hit += dt;
        c.weak_clock += dt;
        c.floor_t += dt;
        for (part, at, imp) in std::mem::take(&mut self.rdr2_hits) {
            c.sound = false;
            let (zero, ramp) = (c.p.v[WEAK_ZERO], c.p.v[WEAK_RAMP]);
            if c.weak_clock >= zero + ramp {
                c.weak_clock = 0.0;
            }
            // the bullet's direction in the body's (b, c) plane, turned out to at least p1d0 from the body's line
            let (mut y, mut z) = (imp.dot(b), imp.dot(cc));
            let (lim, r) = (c.p.v[HIT_ANGLE], y.hypot(z));
            if r > 1e-6 && y.abs().atan2(z.abs()) < lim {
                (y, z) = (y.signum() * r * lim.sin(), z.signum() * r * lim.cos());
            }
            let dir = b * y + cc * z;
            let dir = (dir - w * dir.dot(w)).normalize_or_zero();
            if c.p.v[HIT_LEAN] != 0.0 {
                c.push_lean = dir * c.p.v[HIT_LEAN];
            }
            if c.p.v[HIT_PUSH] != 0.0 {
                c.push_force = dir * c.p.v[HIT_PUSH];
            }
            c.since_hit = 0.0;
            // the quadruped body's snap: the part hit and the parts on either side of it (Delirium's snap impulse)
            if c.p.on(SNAP) && imp != Vec3::ZERO {
                let mut near: Vec<usize> = vec![part];
                near.extend(self.build.parts[part].parent);
                near.extend((0..n).filter(|&k| self.build.parts[k].parent == Some(part)));
                let each = imp.normalize() * c.p.v[SNAP_MAG] * 60.0 / near.len() as f32;
                for k in near {
                    self.doll.impulse(k, at, each);
                }
            }
        }
        if self.legs.iter().any(|&l| l > 0.0) || self.dead {
            c.sound = false;
        }

        // 2. ground probe from the COM straight down 1.5 legLen; the trajectory probe along v
        match world.contact(com, -w, 1.5 * leg_len) {
            Some((d, ..)) => {
                c.airborne = false;
                c.com_h = d;
            }
            None => c.airborne = true,
        }
        c.free = false;
        if c.airborne {
            let ahead = vel.try_normalize().and_then(|d| world.contact(com, d, (speed * 1.0).max(1.5 * leg_len)));
            if ahead.is_none() {
                c.free = true;
                if c.flail_sign == 2.0 {
                    c.flail_sign = c.rand() * 4.0 - 2.0;
                }
            }
        }
        // 3. tilt: sideways lean plus a sideways turning term
        let (t1, t2) = (b.dot(w), 0.2 * omega.dot(cc));
        c.tilt = if t1.abs() < 0.1 { 0.0 } else { t1 } + if t2.abs() < 0.1 { 0.0 } else { t2 };
        c.sign = if c.tilt < 0.0 { -1.0 } else { 1.0 };
        // 4. the start lift
        if c.t < c.p.v[LIFT_TIME] && vel.dot(b) < 0.5 {
            let f = w * c.p.v[LIFT_FORCE] * (1.0 - c.t / c.p.v[LIFT_TIME]) / 3.0;
            for k in [11, 12, 15] {
                force[k] += f;
            }
        }
        // 5. the look-ahead COM, at the body frame's height
        let look = {
            let l = com + vel * c.p.v[LOOKAHEAD];
            l - w * (l - com).dot(w)
        };

        // 6. support: the pushes, the hooves' contact, the body frame
        let u = (w + c.push_lean).normalize_or(w);
        if c.state < 2 && c.push_force != Vec3::ZERO {
            let f = c.push_force - w * c.push_force.dot(w);
            for k in [5, 11, 12, 15] {
                force[k] += f;
            }
        }
        let mut planted = [false; 4];
        for (i, &l) in LEG.iter().enumerate() {
            let k = LEGS[l][2];
            let now = self.doll.touch[k] & delirium_data::kv!(7 int) != 0 && self.doll.hit[k].is_some_and(|h| h.1.dot(w) > 0.5);
            if now {
                c.legs[i].hold = HOLD;
            } else {
                c.legs[i].hold -= dt;
            }
            planted[i] = c.legs[i].hold > 0.0;
        }
        // the body frame: the COM matrix, its origin H' down along a, turned from a onto u (level, unless pushed)
        let ramp = c.p.v[RAMP_HEIGHT];
        let h = if ramp > 0.1 { c.h - 0.5 * ((c.support_t - 1.0) / ramp).min(1.0) } else { c.h };
        let level = Quat::from_rotation_arc(a, u);
        let mut fr_rows = [u, level * b, level * cc];
        let mut origin = com - u * h;
        if c.state == 0 {
            // balance by tilting: roll and pitch from the look-ahead COM's offset
            let d = look - origin;
            let bh = (fr_rows[1] - w * fr_rows[1].dot(w)).length().max(1e-3);
            let k = h / leg_len.max(0.1);
            let mut roll = ((d.dot(fr_rows[1]) / bh).clamp(-1.0, 1.0) * k).clamp(-1.0, 1.0).asin() * c.p.v[TILT_ROLL];
            let mut pitch = -((d.dot(fr_rows[2])).clamp(-1.0, 1.0) * k).clamp(-1.0, 1.0).asin() * c.p.v[TILT_PITCH];
            roll += c.p.v[ROLL_OFF];
            pitch += c.p.v[PITCH_OFF];
            let most = c.p.v[MAX_SUPPORT];
            if (most >= 0.0 && c.support_t > most) || c.com_h < 0.5 * leg_len {
                roll = 0.6 * (c.support_t - most.max(0.0)).min(0.5) * c.sign;
            }
            let (roll, pitch) = (roll.clamp(-0.3, 0.3), pitch.clamp(-0.3, 0.3));
            let r = Quat::from_axis_angle(fr_rows[2], roll) * Quat::from_axis_angle(fr_rows[1], pitch);
            let up0 = fr_rows[0];
            fr_rows = fr_rows.map(|v| r * v);
            // the lowest corner hoof stays on the ground
            let low = c.neutral.iter().map(|l| (fr_rows[0] * l.x + fr_rows[1] * l.y + fr_rows[2] * l.z).dot(up0)).fold(0.0f32, f32::min);
            origin -= up0 * low;
        }
        c.origin = origin;
        c.rows = fr_rows;
        // the support polygon of the planted hooves: the look-ahead COM's margin
        let feet: Vec<Vec3> = (0..4).filter(|&i| planted[i]).map(|i| fr[LEGS[LEG[i]][2]].translation).collect();
        let m = if a.dot(w) < 0.0 || feet.len() < 3 { -1.0 } else { margin(&feet, look, w) };

        // 7. other parts on the ground
        let hooves: Vec<usize> = LEGS.iter().map(|l| l[2]).collect();
        let touching: Vec<usize> = (0..n).filter(|k| !hooves.contains(k))
            .filter(|&k| self.doll.touch[k] & delirium_data::kv!(7 int) != 0 && self.doll.hit[k].is_some_and(|h| h.1.dot(w) > 0.5)).collect();
        c.on_parts = touching.len() >= 2;
        if c.on_parts && c.roll < 0.5 {
            c.lying_t = (c.lying_t + dt).min(0.3);
        } else {
            c.lying_t = (c.lying_t - dt).max(0.0);
        }
        c.lying = c.lying_t > 0.2;

        // 8. the state
        let up = w.dot(a);
        let fail = (c.p.v[FAIL] + if c.on_parts { 0.1 } else { 0.0 }).max(0.0);
        let was = c.state;
        let mut s = if was == 4 {
            4
        } else if up > 0.1 && m > c.p.v[STEP_THRESHOLD] {
            0
        } else if up < fail || c.airborne || c.lying {
            2
        } else if c.p.on(STEP) {
            1
        } else {
            0
        };
        if s == 2 && (was == 2 || was == 3) && !touching.is_empty() {
            s = 3;
        }
        if (s == 2 || s == 3) && speed < 0.5 && spin < 0.5 && c.lying {
            s = 4;
        }
        if s >= 2 {
            c.sound = false;
        }
        if s == 3 && was != 3 {
            c.entered3 = true;
            // the floor weakness, from the ground hit
            c.floor_t = 0.0;
            c.floor_part = touching.first().copied();
        }
        c.state = s;
        // 9. time in and out of support
        if s < 2 {
            c.support_t += dt;
        } else {
            c.support_t = 0.0;
        }
        if s == 1 {
            c.out_t += dt;
        } else {
            c.out_t = 0.0;
        }

        // 10. strength
        let (rate, floor) = if s == 4 {
            (c.p(DECAY_DOWN), c.p.v[SPINE_DOWN] / c.p.v[SPINE_STIFF])
        } else {
            (c.p(DECAY), c.p.v[SPINE_MIN] / c.p.v[SPINE_STIFF])
        };
        if !c.airborne {
            c.fs = (c.fs - rate * dt).max(floor.min(c.fs));
        }
        if s == 3 && std::mem::take(&mut c.entered3) {
            c.fs2 = c.p.v[SPINE_DOWN] / c.p.v[SPINE_STIFF];
        } else {
            c.fs2 += (c.fs - c.fs2) * rate.max(0.2);
        }
        let leg_floor = c.p.v[LEG_MIN] / c.p.v[LEG_STIFF];
        let mbt = c.p(MAX_BALANCE);
        let mut leg_f = [c.fl; 4];
        if s >= 2 || mbt < 0.0 {
            c.fl = c.fs.min(c.fl).max(leg_floor.min(c.fl));
            leg_f = [c.fl; 4];
        } else if c.support_t > mbt {
            let give = (1.0 - 0.5 * (c.support_t - mbt).min(2.0)).max(3.0 / c.p.v[LEG_STIFF]);
            // the lower side gives way: tilt > 0 is the left side up
            let side = if c.tilt > 0.0 { [1, 3] } else { [0, 2] };
            for i in side {
                leg_f[i] = give.min(c.fl);
            }
        }

        // 11. stepping
        let busy = c.legs.iter().any(|l| l.stepping);
        if (c.free || (s == 1 && c.out_t > 0.0)) && !busy {
            let i = c.next;
            c.next = WALK[i];
            let foot = c.to_local(self.sole_at(&fr, LEG[i]));
            let lead = c.to_local(look) - c.to_local(com);
            let to = c.neutral[i] + Vec3::new(0.0, lead.y, lead.z);
            let mut d = (to - foot) * 1.3;
            d.x = 0.0;
            let d = d.clamp_length_max(c.p.v[MAX_STEP]);
            self.start_step(&mut c, i, foot, foot + d);
        } else if s == 0 && c.p.on(STEP) && c.p.v[RESTEP] > 0.0 && !busy {
            let far = (0..4).map(|i| {
                let f = c.to_local(self.sole_at(&fr, LEG[i]));
                (i, Vec3::new(0.0, f.y - c.neutral[i].y, f.z - c.neutral[i].z).length(), f)
            }).max_by(|x, y| x.1.total_cmp(&y.1));
            if let Some((i, d, f)) = far.filter(|x| x.1 > c.p.v[RESTEP]) {
                let _ = d;
                let to = c.neutral[i];
                self.start_step(&mut c, i, f, to);
            }
        }
        // 12. rolling on the ground (the predicted roll's leg poses are unread); the height relaxes when falling
        c.roll = ((spin - c.p.v[ROLL_VEL]) * c.p.v[ROLL_SCALE]).clamp(0.0, 1.0);
        if (s == 2 || s == 3) && !c.free && !c.airborne {
            let to = c.unit * c.p.v[HEIGHT] - 0.8;
            c.h += (to - c.h) * 0.1;
        }

        // 13. the legs
        let mut targets = self.targets.clone();
        if s == 2 || s == 3 {
            c.sigma = 0.5 * (speed.min(1.0) + spin.min(1.0));
            c.phase += (c.p.v[SCRABBLE_MIN] + (c.p.v[SCRABBLE_MAX] - c.p.v[SCRABBLE_MIN]) * c.sigma) * dt;
        }
        let mut stance = [false; 4];
        for i in 0..4 {
            let l = LEG[i];
            // (rolling, RDR2 folds the legs between two poses, unread: they stand)
            let rolling = c.p.on(ROLL_GROUND) && spin > c.p.v[ROLL_VEL];
            let scrabble = (s == 4 && c.p.on(SCRABBLE_SIDE)) || c.p.on(FORCE_SCRABBLE) || ((s == 2 || s == 3) && !rolling && c.legs[i].stepping);
            let to = if scrabble {
                // a hoof circle below the hip, about the body's side axis, the legs a quarter turn apart
                let centre = fr[LEGS[l][0]].translation - a * (0.8 + 0.2 * c.sigma);
                let r = 0.2 + 0.18 * (1.0 - c.sigma);
                let th = c.phase + i as f32 * FRAC_PI_2;
                centre + (cc * th.cos() + a * th.sin()) * r
            } else if s < 2 && c.legs[i].stepping {
                self.swing(&mut c, i, &fr, planted[i]);
                c.to_world(c.legs[i].target)
            } else {
                // a stance leg: its target moves to its goal at 1.5 m/s
                stance[i] = true;
                let lg = &mut c.legs[i];
                let d = lg.goal - lg.target;
                lg.target += d.clamp_length_max(1.5 * dt);
                let k = LEGS[l][2];
                if c.p.v[FOOT_SLIP] > 0.0 && planted[i] && !lg.stepping {
                    let v = self.doll.vel(k).0;
                    let vh = v - w * v.dot(w);
                    let f = -vh * vh.length() * c.p.v[FOOT_SLIP];
                    if f.length_squared() <= 810_000.0 {
                        force[k] += f;
                    }
                }
                c.to_world(c.legs[i].target)
            };
            let (q, t) = self.leg_ik(&fr, l, to, c.legs[i].knee);
            c.legs[i].knee = t;
            for (j, &k) in LEGS[l].iter().enumerate() {
                targets[k] = q[j];
            }
        }

        // 14. the spine and neck: the base curve under everything, then writhe, the head, the flail
        let mut chain: Vec<Option<Vec3>> = CHAIN.iter().enumerate().map(|(i, &k)| match self.build.parts[k].limit {
            Limit::Bone { pf, cf, .. } if i > 0 => {
                let pose = angles(pf, cf, self.targets[k]);
                Some(Vec3::new(POSTURE[i], pose.y, pose.z))
            }
            _ => None,
        }).collect();
        let on_fire = c.p.on(ON_FIRE);
        if c.free || on_fire {
            c.flail_t -= dt;
            if c.flail_t <= 0.0 {
                c.flail_t = 0.3 + 0.3 * c.rand();
                c.flail_sign = -c.flail_sign;
                c.flail = c.rand() * 3.0 - 1.0;
            }
            let bend = c.flail * c.flail_sign.signum();
            for (i, x) in chain.iter_mut().enumerate().skip(1) {
                if let Some(x) = x {
                    x.x += bend / if i < 4 { 3.0 } else { 4.0 };
                }
            }
        }
        if (c.p.on(WRITHE_ON) || (c.p.on(WRITHE_GROUND) && s == 4)) && !c.free {
            c.writhe_t -= dt;
            if c.writhe_t <= 0.0 || c.writhe == Vec3::ZERO {
                c.writhe_t = 1.0 / c.p.v[WRITHE_SPEED].max(0.01);
                let (y, p) = ((c.rand() - 0.5) * PI, (c.rand() - 0.5) * PI);
                c.writhe = Quat::from_axis_angle(a, y) * Quat::from_axis_angle(b, p) * -cc;
            }
            self.point(&fr, &mut chain, &WRITHE, c.writhe, PI);
        }
        let lift = c.p.on(LIFT_HEAD) || s == 2 || s == 3 || (s == 4 && !c.p.on(WRITHE_GROUND));
        if lift && !c.free && !on_fire {
            let f = c.facing - w * c.facing.dot(w);
            if let Some(f) = f.try_normalize() {
                self.point(&fr, &mut chain, &NECK, f, 1.2);
            }
        }
        for (i, &k) in CHAIN.iter().enumerate() {
            if let (Some(x), Limit::Bone { pf, cf, .. }) = (chain[i], self.build.parts[k].limit) {
                targets[k] = from_angles(pf, cf, x);
            }
        }
        // the righting torque, lying still on its side
        let rt = c.p.v[RIGHTING];
        if rt > 1e-6 && c.on_parts && spin < 0.5 && speed < 0.5 {
            let s = -(a.dot(c.facing));
            if s > 0.0 {
                let side = a.cross(w).dot(cc).signum();
                for k in [0, 5, 11, 12] {
                    torque[k] += cc * side * s * rt;
                }
            }
        }
        // the down torques: yaw toward the facing, a roll spring about c
        if s >= 2 {
            let r = if speed < c.p.v[DOWN_GATE] && c.p.v[FADE_FAST] > c.p.v[FADE_SLOW] { c.p.v[FADE_FAST] } else { c.p.v[FADE_SLOW] };
            c.fade[0] = (c.fade[0] - r * dt).max(0.0);
            c.fade[1] = (c.fade[1] - 2.0 * r * dt).max(0.0);
            let fh = (-cc - w * (-cc).dot(w)).normalize_or_zero();
            let want = if c.p.on(FACING_VEL) && c.facing.dot(w).abs() > 0.9 { vel } else { c.facing };
            let dh = (want - w * want.dot(w)).normalize_or_zero();
            if fh != Vec3::ZERO && dh != Vec3::ZERO {
                let mut th = fh.cross(dh).dot(w).atan2(fh.dot(dh));
                if th > PI { th -= TAU; }
                if th < -PI { th += TAU; }
                if th.abs() > 0.1 {
                    let y = (10.0 * th * dt).clamp(-c.p.v[YAW_CAP], c.p.v[YAW_CAP]) / 5.0 * c.fade[1] * 700.0;
                    for k in PUSHED {
                        torque[k] += w * y;
                    }
                }
            }
            let wc = omega.dot(cc);
            let r = ((c.p.v[ROLL_GAIN] * b.dot(w) - wc) / dt).clamp(-c.p.v[ROLL_CAP], c.p.v[ROLL_CAP]) / 5.0 * 500.0 * c.fade[0];
            if r * wc < 0.0 {
                for k in PUSHED {
                    torque[k] += cc * r;
                }
            }
        }

        // the muscles
        let weak = ((c.weak_clock - c.p.v[WEAK_ZERO]) / c.p.v[WEAK_RAMP].max(1e-3)).clamp(0.0, 1.0);
        let floor_on = c.floor_t < c.p.v[FLOOR_TIME] && c.p.v[FLOOR_STRENGTH] > 0.0;
        let joint_on = c.floor_t < c.p.v[FLOOR_JOINT_TIME];
        let floor_parts: Vec<usize> = (0..22).filter(|j| FLOOR_MASK >> j & 1 != 0).map(|j| link_part(j + 1)).collect();
        let near_floor: Vec<usize> = c.floor_part.map_or(vec![], |f| std::iter::once(f).chain((0..n).filter(|&k| self.build.parts[k].parent == Some(f))).collect());
        let (lk, ld, sk, sd) = (c.p.v[LEG_STIFF], c.p.v[LEG_DAMP], c.p.v[SPINE_STIFF], c.p.v[SPINE_DAMP]);
        let dead = self.dead;
        let mut muscles: Vec<Muscle> = (0..n).map(|k| {
            let leg = LEGS.iter().position(|l| l.contains(&k));
            let (own, ratio) = self.muscles[k];
            let (mut st, mut dm) = if WELDED.contains(&k) {
                (WELD, 1.0)
            } else if dead {
                (0.0, ratio)
            } else if let Some(l) = leg {
                let i = LEG.iter().position(|&x| x == l).unwrap_or(0);
                let f = if c.legs[i].stepping { 1.0 } else { leg_f[i] };
                // the ankle of a leg not stepping, standing or stepping
                if k == LEGS[l][2] && s < 2 && !c.legs[i].stepping {
                    (lk * c.p.v[ANKLE_SCALE], ld)
                } else {
                    (f * lk, ld)
                }
            } else if CHAIN.contains(&k) {
                (c.fs2 * sk, sd)
            } else {
                (own, ratio)
            };
            if !WELDED.contains(&k) {
                st *= weak.max(0.05);
                if floor_on && floor_parts.contains(&k) {
                    st *= c.p.v[FLOOR_STRENGTH];
                }
                if joint_on && near_floor.contains(&k) {
                    st *= c.p.v[FLOOR_JOINT_STRENGTH];
                }
            }
            if let Some(l) = leg {
                st *= (1.0 - self.legs[l] / LAME).clamp(0.08, 1.0);
            }
            if dm <= 0.0 {
                dm = 1e-3;
            }
            Muscle { target: targets[k], stiffness: st, damping: dm }
        }).collect();

        // the rider (defineAttachedObject): his weight on the barrel; gravity opposition carries it
        if let Some((_, _, rm)) = self.rider {
            force[MID] -= w * rm * g;
        }
        if !dead {
            self.oppose_gravity(&fr, &planted, s < 2, &mut muscles, &mut torque);
        }
        let _ = (mass, stance);
        self.kept.force = force;
        self.kept.torque = torque;
        self.kept.muscles = muscles;
        self.core = Some(c);
        let drive: Drive = self.kept.drive();
        self.doll.step(&self.build, world, dt, drive, obstacles, others);
    }

    fn start_step(&self, c: &mut Core, i: usize, from: Vec3, to: Vec3) {
        let len = Vec3::new(0.0, to.y - from.y, to.z - from.z).length();
        let lg = &mut c.legs[i];
        lg.stepping = true;
        lg.start = c.t;
        lg.end = c.t + len.max(0.1) * c.p.v[STEP_TIME];
        lg.from = Vec3::new(0.0, from.y, from.z);
        lg.to = Vec3::new(0.0, to.y, to.z);
        lg.left = f32::MAX;
        // each step weakens the legs a little, not below 5/legStiffness
        c.fl = (c.fl - c.p(STEP_WEAKEN)).max((5.0 / c.p.v[LEG_STIFF]).min(c.fl));
    }

    /// A swing leg: along its step, lifted while far from its end; down early once past halfway, planted and near.
    fn swing(&self, c: &mut Core, i: usize, fr: &[Transform], planted: bool) {
        let hoof = c.to_local(self.sole_at(&fr, LEG[i]));
        let height = c.p.v[STEP_HEIGHT];
        let t = c.t;
        let lg = &mut c.legs[i];
        let s = ((t - lg.start) / (lg.end - lg.start).max(1e-3)).clamp(0.0, 1.0);
        let d = Vec3::new(0.0, lg.to.y - hoof.y, lg.to.z - hoof.z).length();
        lg.left = lg.left.min(d);
        let lift = (1.0 - s) * height * lg.left.min(0.15) * 6.667 * 0.15;
        lg.target = lg.from + (lg.to - lg.from) * s + Vec3::X * lift;
        if s >= 1.0 || (s > 0.5 && planted && lg.left < 0.1) {
            lg.stepping = false;
            lg.goal = lg.to;
            lg.target = Vec3::new(0.0, hoof.y, hoof.z);
        }
    }

    /// Points the chain's joints in `mask` (indices into `CHAIN`) at world direction `dir`: each turns its part's
    /// axis a 1/n share toward it, at most `most` rad, written as lean1/lean2.
    fn point(&self, fr: &[Transform], chain: &mut [Option<Vec3>], mask: &[usize], dir: Vec3, most: f32) {
        let share = 1.0 / mask.len() as f32;
        for &i in mask {
            let k = CHAIN[i];
            let (Some(x), Limit::Bone { pf, cf, .. }, Some(p)) = (chain[i], self.build.parts[k].limit, self.build.parts[k].parent) else { continue };
            let next = CHAIN.get(i + 1).copied().unwrap_or(k + 1).min(fr.len() - 1);
            let axis = (fr[next].translation - fr[k].translation).normalize_or(fr[k].rotation * Vec3::Y);
            let turn = Quat::from_rotation_arc(axis, dir);
            let (ax, ang) = turn.to_axis_angle();
            let turn = Quat::from_axis_angle(ax, (ang * share).min(most));
            let rel = fr[p].rotation.inverse() * turn * fr[k].rotation;
            let now = angles(pf, cf, rel);
            chain[i] = Some(Vec3::new(now.x, now.y, x.z));
        }
    }

    /// The core's gravity opposition (rdr2_core2.md 1) on the horse: at each joint the weight on its far side, 0.1 s
    /// ahead, less the ground's push on the planted hooves there (see the module's notes), the cap
    /// 0.2·strength·opposeGravity, torques over 5000 (hinge) or 1e4 dropped; no hoof planted, none (ZMP mode). Raises
    /// a standing planted hoof's joint's muscle (see the module's notes).
    fn oppose_gravity(&self, fr: &[Transform], planted: &[bool; 4], on_feet: bool, muscles: &mut [Muscle], out: &mut [Vec3]) {
        let n = self.build.parts.len();
        let up = Vec3::Z;
        let feet: Vec<usize> = (0..4).filter(|&i| planted[i]).collect();
        if feet.is_empty() {
            return;
        }
        let (com, _, _) = self.com_rider();
        // the support point: the COM at the hooves' height, in their polygon
        // where each planted hoof meets the ground (its contact point; its centre if none this step)
        let hooves: Vec<Vec3> = feet.iter().map(|&i| {
            let k = LEGS[LEG[i]][2];
            self.doll.hit[k].filter(|_| self.doll.touch[k] & delirium_data::kv!(7 int) != 0).map_or(fr[k].translation, |h| h.0)
        }).collect();
        let hz = hooves.iter().map(|h| h.z).sum::<f32>() / hooves.len() as f32;
        let mut sp = com.with_z(hz);
        if hooves.len() >= 3 {
            if margin(&hooves, sp, up) < 0.0 {
                let c = hooves.iter().sum::<Vec3>() / hooves.len() as f32;
                let mut best = c;
                for e in 0..hooves.len() {
                    for f in e + 1..hooves.len() {
                        let (a, b) = (hooves[e], hooves[f]);
                        let s = ((sp - a).dot(b - a) / (b - a).length_squared().max(1e-9)).clamp(0.0, 1.0);
                        let p = a + (b - a) * s;
                        if p.distance(sp) < best.distance(sp) {
                            best = p;
                        }
                    }
                }
                sp = best.with_z(hz);
            }
        } else if hooves.len() == 2 {
            let (a, b) = (hooves[0], hooves[1]);
            let s = ((sp - a).dot(b - a) / (b - a).length_squared().max(1e-9)).clamp(0.0, 1.0);
            sp = (a + (b - a) * s).with_z(hz);
        } else {
            sp = hooves[0];
        }
        let gv = -up * GRAVITY();
        let dn = (sp - com).normalize_or(-up);
        let gv = dn * gv.dot(dn);
        // subtree sums: mass, mass·x, mass·v, mass·|x|²
        let mut m: Vec<(f32, Vec3, Vec3, f32)> = (0..n).map(|k| {
            let mk = self.doll.part_mass(k);
            let x = fr[k].translation;
            (mk, x * mk, self.doll.vel(k).0 * mk, x.length_squared() * mk)
        }).collect();
        if let Some((p, v, rm)) = self.rider {
            let s = &mut m[MID];
            *s = (s.0 + rm, s.1 + p * rm, s.2 + v * rm, s.3 + p.length_squared() * rm);
        }
        let add = |m: &mut Vec<(f32, Vec3, Vec3, f32)>, to: usize, from: usize| {
            let f = m[from];
            m[to] = (m[to].0 + f.0, m[to].1 + f.1, m[to].2 + f.2, m[to].3 + f.3);
        };
        let mut full = m.clone();
        for k in (1..n).rev() {
            if let Some(p) = self.build.parts[k].parent {
                add(&mut full, p, k);
            }
        }
        // each planted hoof's share of the weight: the least-norm shares that put their centre under the COM
        let total = full[0];
        let c0 = total.1 / total.0.max(1e-6);
        let flat: Vec<Vec2> = hooves.iter().map(|h| h.truncate() - c0.truncate()).collect();
        let mut share = vec![1.0 / flat.len() as f32; flat.len()];
        if flat.len() >= 2 {
            let (mut sxx, mut sxy, mut syy, mut sx, mut sy) = (0.0, 0.0, 0.0, 0.0, 0.0);
            for f in &flat {
                (sxx, sxy, syy, sx, sy) = (sxx + f.x * f.x, sxy + f.x * f.y, syy + f.y * f.y, sx + f.x, sy + f.y);
            }
            let a = Mat3::from_cols(Vec3::new(flat.len() as f32, sx, sy), Vec3::new(sx, sxx, sxy), Vec3::new(sy, sxy, syy));
            let lam = if a.determinant().abs() > 1e-6 {
                a.inverse() * Vec3::X
            } else {
                // in a line: along it only
                let d = (flat[flat.len() - 1] - flat[0]).normalize_or(Vec2::X);
                let (n, sd, sdd) = flat.iter().fold((0.0, 0.0, 0.0), |(n, a, b), f| (n + 1.0, a + f.dot(d), b + f.dot(d) * f.dot(d)));
                let det = n * sdd - sd * sd;
                if det.abs() > 1e-6 { Vec3::new(sdd / det, -sd / det * d.x, -sd / det * d.y) } else { Vec3::new(1.0 / n, 0.0, 0.0) }
            };
            share = flat.iter().map(|f| (lam.x + lam.y * f.x + lam.z * f.y).max(0.0)).collect();
            let sum: f32 = share.iter().sum();
            share.iter_mut().for_each(|x| *x /= sum.max(1e-6));
        }
        for k in 1..n {
            let Some(p) = self.build.parts[k].parent else { continue };
            if WELDED.contains(&k) {
                continue;
            }
            let d = &self.build.parts[k];
            let pj = fr[p].translation + fr[p].rotation * d.anchor_parent;
            let (v, wv) = self.doll.vel(k);
            let vj = v + wv.cross(pj - fr[k].translation);
            // the hooves planted on this side of the joint, and the side whose inertia the cap takes
            let below: Vec<usize> = (0..feet.len()).filter(|&f| {
                let mut x = Some(LEGS[LEG[feet[f]]][2]);
                while let Some(y) = x.filter(|&y| y != k) {
                    x = self.build.parts[y].parent;
                }
                x == Some(k)
            }).collect();
            let (mm, mp, mq, m2) = full[k];
            if mm <= 0.0 {
                continue;
            }
            let at = pj + vj * AHEAD;
            let mut t = (at - (mp + mq * AHEAD) / mm).cross(gv * mm);
            for &f in &below {
                t -= (hooves[f] - at).cross(-gv * total.0 * share[f]);
            }
            // the cap: 0.2·strength, our muscle's s² times the far side's moment of inertia about the joint
            let (im, ip, i2) = if below.is_empty() { (mm, mp, m2) } else { (total.0 - mm, total.1 - mp, total.3 - m2) };
            let inertia = (i2 - 2.0 * ip.dot(pj) + im * pj.length_squared()).max(0.0);
            let s = muscles[k].stiffness;
            let cap = 0.2 * s * s * inertia;
            // a planted hoof's joint holds up the body: RDR2's strength (a torque per rad) is s² times that side's
            // inertia, so our muscle (a frequency over the joint's own inertia, the hoof's) is raised to match
            let touching = below.iter().all(|&f| self.doll.touch[LEGS[LEG[feet[f]]][2]] & delirium_data::kv!(7 int) != 0);
            if on_feet && !below.is_empty() && touching && LEGS.iter().any(|l| l[2] == k) {
                let own = (m2 - 2.0 * mp.dot(pj) + mm * pj.length_squared()).max(1e-4);
                muscles[k].stiffness *= (inertia / own).sqrt();
            }
            let t = match d.limit {
                Limit::BoneHinge { pf, .. } => {
                    let axis = fr[p].rotation * pf * Vec3::Z;
                    let th = t.dot(axis).clamp(-cap, cap);
                    if th.abs() > 5000.0 {
                        continue;
                    }
                    axis * th
                }
                _ => {
                    let t = t.clamp_length_max(cap);
                    if t.length_squared() > 1e8 {
                        continue;
                    }
                    t
                }
            };
            out[k] += t;
            out[p] -= t;
        }
    }

    /// What it's doing, for the panel and the headless check.
    pub(super) fn state_rdr2(&self) -> String {
        let Some(c) = self.core.as_ref() else { return "RDR2 quadruped (not started)".into() };
        let name = ["standing", "stepping", "falling", "falling, hit the ground", "down"][c.state];
        let planted = c.legs.iter().filter(|l| l.hold > 0.0).count();
        format!("RDR2 {name}  hooves {planted}  legs {:.0}%  spine {:.0}%  COM h {:.2}  H {:.2}{}", c.fl * 100.0, c.fs2 * 100.0, c.com_h, c.h,
                if c.legs.iter().any(|l| l.stepping) { "  stepping" } else { "" })
    }

    /// On its feet, as RDR2's quadruped tells the core (body+0x420: state < 2).
    pub(super) fn standing_rdr2(&self) -> bool {
        !self.dead && self.core.as_ref().is_none_or(|c| c.state < 2)
    }
}
