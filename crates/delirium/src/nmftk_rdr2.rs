//! RDR2 core: the fall-to-knees core and its record, written from the study note rdr2_falltoknees.md (prose and maths;
//! no game code). The record lives on the character (RDR2's ctx+0x2e20), not on a behaviour: shotFallToKnees and
//! balancerCollisionsReaction fill it, and three callers drive the same core with it: the shot's module (each tick
//! while it runs), meleeReact's hand-over, and smartFall's land-on-feet state.
//!
//! He keeps balancing, a helper force and a lean take his COM to the line through his toes, and once the balancer's
//! quality number is below ftkFallBelowStab (or he moves forward unstably) he folds: the balancer turns lenient and is
//! asked for a knee bend and a hip pitch; once a knee lands the thighs come upright and it holds him there, until he
//! has been still for up to 2 s (forced balancer fail) or the balancer fails by itself, and the normal fall takes over.
//!
//! Ours: the contact flags (RDR2's foot and part bits 2, 4, 16) are `hit_out`, a contact with anything but himself.
//! RDR2's lean channels hold their last request until released; ours lapses a moment after the last ask. Left out: the
//! fall back's pelvis roll and yaw (bal+0x584, +0x588, from the random k), because ours has the hip pitch on the hips and
//! no such pelvis targets; the balancer's forced-contact mask (Delirium has none).

use crate::nmdbm::LeanMode;
use crate::nmrs::{ANKLE, HIP, KNEE, SPINE};
use crate::nmshot::{chest_part, limb, Ctx};
use bevy_math::Vec3;

/// The foot's length and height (ctx+0x47c8, ctx+0x47c4).
const FOOT_LENGTH: f32 = 0.3;
const FOOT_HEIGHT: f32 = crate::nmdbm_rdr2::FOOT_HEIGHT;
/// The character's length scale, ctx+0x2bfc: 1.001 on the recorded ped, 1 here (rdr2_body_gap.md).
const L: f32 = 1.0;
/// The owner ids of its balancer requests: knee bend and hip pitch.
const KNEE_OWNER: u32 = 12;
const HIP_OWNER: u32 = 16;

/// RDR2's fall-to-knees record, ctx+0x2e20: what shotFallToKnees sets, then the core's state. Defaults are the
/// character constructor's, the same as the message's.
#[derive(Clone, Copy, Debug)]
pub struct Ftk {
    pub on: bool,
    pub balance_time: f32,
    pub helper_force: f32,
    pub lean_help: f32,
    pub spine_bend: f32,
    pub impact_looseness: f32,
    pub impact_time: f32,
    pub bend_rate: f32,
    pub hip_blend: f32,
    pub lunge_prob: f32,
    pub fric_mult: f32,
    pub hip_angle_fall: f32,
    pub pitch_forwards: f32,
    pub pitch_backwards: f32,
    pub fall_below_stab: f32,
    pub thresh_per_s: f32,
    pub thresh_per_step: f32,
    pub abort_threshold: f32,
    pub release_rfw: f32,
    pub arm_type: i32,
    pub release_point_gun: bool,
    pub fail_must_collide: bool,
    pub stay_upright: bool,
    pub knee_spin: bool,
    pub stiff_spine: bool,
    pub helper_on_spine: bool,
    pub always_change_fall: bool,
    pub(crate) t0: f32,
    pub(crate) steps0: u32,
    pub(crate) still: f32,
    pub(crate) impact_t: f32,
    pub(crate) theta: f32,
    pub(crate) impact: [bool; 2],
    pub knee_down: [bool; 2],
    pub(crate) fall_back: bool,
    pub(crate) sitting: bool,
    pub uneven: bool,
    pub kneeling: bool,
    pub(crate) spin_k: f32,
    pub(crate) lunge: bool,
    /// the per-part friction it set (ctx+0x3570): left then right, thigh, calf, foot. It stays until the exit
    pub(crate) fric: [f32; 6],
    /// the shot's module is running (runner #15's own flag)
    pub module_on: bool,
    /// the balancer's readings as they stood when it failed (RDR2 leaves them in the balancer)
    pub(crate) bal: (f32, u32, f32, f32),
    /// the core ran since the friction was last read
    pub(crate) ticked: bool,
    pub(crate) said: u32,
}

impl Default for Ftk {
    fn default() -> Ftk {
        Ftk { on: false, balance_time: 0.7, helper_force: 200.0, lean_help: 0.05, spine_bend: 0.0, impact_looseness: 0.5, impact_time: 0.2,
              bend_rate: 0.7, hip_blend: 0.3, lunge_prob: 0.0, fric_mult: 1.0, hip_angle_fall: 0.5, pitch_forwards: 0.1, pitch_backwards: 0.1,
              fall_below_stab: 0.5, thresh_per_s: 0.0, thresh_per_step: 0.0, abort_threshold: 2.0, release_rfw: -1.0, arm_type: 2,
              release_point_gun: false, fail_must_collide: true, stay_upright: true, knee_spin: false, stiff_spine: false, helper_on_spine: true,
              always_change_fall: false, t0: 0.0, steps0: 0, still: 0.0, impact_t: 0.0, theta: 0.5, impact: [false; 2], knee_down: [false; 2],
              fall_back: false, sitting: false, uneven: false, kneeling: false, spin_k: 0.1, lunge: true, fric: [1.0; 6], module_on: false,
              bal: (0.0, 0, 0.0, 0.0), ticked: false, said: 0 }
    }
}

impl Ftk {
    /// The record with the settings of `s` (a message's) and this one's state; the threshold follows (ctx+0x2e30).
    pub fn with_settings(&self, s: &Ftk) -> Ftk {
        Ftk { t0: self.t0, steps0: self.steps0, still: self.still, impact_t: self.impact_t, theta: s.fall_below_stab, impact: self.impact, knee_down: self.knee_down,
              fall_back: self.fall_back, sitting: self.sitting, uneven: self.uneven, kneeling: self.kneeling, spin_k: self.spin_k, lunge: self.lunge,
              fric: self.fric, module_on: self.module_on, bal: self.bal, ticked: self.ticked, said: self.said, ..*s }
    }

    /// balancerCollisionsReaction's preset (its two writes): at once, no helper, no lean, the threshold out of the way;
    /// `full` also sets the hip angle and the two pitches.
    pub fn bcr_preset(&mut self, full: bool) {
        (self.balance_time, self.theta, self.fall_below_stab, self.helper_force, self.lean_help) = (0.0, 10.0, 10.0, 0.0, 0.0);
        if full {
            (self.hip_angle_fall, self.pitch_forwards, self.pitch_backwards) = (0.0, -0.1, -0.1);
        }
        self.on = true;
    }

    /// The friction it holds, for this step's part scales, while the core runs (ours: RDR2's array stays as written
    /// until catchFall or an exit writes it, and ours has no other writer, so it lapses when the core stops).
    pub fn friction(&mut self) -> Vec<(usize, f32)> {
        if !std::mem::take(&mut self.ticked) || self.fric == [1.0; 6] {
            return vec![];
        }
        let (a, b) = (limb(2).p, limb(3).p);
        [a[1], a[2], a[3], b[1], b[2], b[3]].into_iter().zip(self.fric).collect()
    }

    /// A log line the first time a branch runs.
    fn say(&mut self, bit: u32, lines: &mut Vec<String>, line: impl FnOnce() -> String) {
        if self.said >> bit & 1 == 0 {
            self.said |= 1 << bit;
            lines.push(format!("#say fallToKnees {}", line()));
        }
    }
}

/// Init 0x2a2f888 (the shot's reset and its module's enter, smartFall's state 7): the clocks from the balancer, the
/// flags off, the lunge and the random k (three draws).
pub fn init(c: &mut Ctx) {
    let (time, steps) = c.dbm.map_or((c.ch.core.ftk.bal.0, c.ch.core.ftk.bal.1), |d| (d.time, d.steps));
    let lunge = c.ch.random() < c.ch.core.ftk.lunge_prob;
    let k = 0.1 + 0.3 * c.ch.random();
    let negative = c.ch.random_bits() & 1 != 0;
    let f = &mut c.ch.core.ftk;
    (f.t0, f.steps0, f.still, f.impact_t, f.theta, f.impact, f.knee_down) = (time, steps, 0.0, 0.0, 0.5, [false; 2], [false; 2]);
    (f.fall_back, f.sitting, f.uneven, f.kneeling, f.lunge, f.spin_k) = (false, false, false, false, lunge, if negative { -k } else { k });
    f.said = 0;
}

/// Exit 0x2a2f7b4: the lean requests go (ours lapse by themselves); with a kneel started, the balancer as it was and
/// the legs' friction 1.
pub fn exit(c: &mut Ctx) {
    let f = &mut c.ch.core.ftk;
    if f.kneeling {
        c.out.dbm_abort = Some(0.6);
        c.out.dbm_must_collide = Some(false);
        c.out.dbm_kneeling = Some(false);
        f.fric = [1.0; 6];
        c.ch.core.foot_hold = true;
    }
}

/// What the module and smartFall's state 7 copy to catchFall and rollDownStairs (their +0xcb..+0xcd, +0xf0..+0xf7).
pub fn flags(c: &mut Ctx) {
    let f = c.ch.core.ftk;
    let u = !f.uneven || f.always_change_fall;
    c.ch.core.cf_knee = [u, f.arm_type == 1, f.arm_type == 2];
    c.ch.core.rds_knee = Some(crate::nmrs::RdsKnee { on: u, lean: f.spine_bend, stiff: f.stiff_spine, poses: [f.arm_type == 1, f.arm_type == 2] });
}

/// The module's exit and smartFall's state 7's: the flags off.
pub fn flags_off(c: &mut Ctx) {
    c.ch.core.cf_knee = [false; 3];
    c.ch.core.rds_knee = None;
}

/// The shot's module (runner #15): `shot` is the shot's clock and falling flag while it runs. Starts when asked for,
/// he isn't falling and the shot's clock passed ftkBalanceTime; stops when the record says off.
pub fn module(c: &mut Ctx, shot: Option<(f32, bool)>, lines: &mut Vec<String>, dt: f32) -> bool {
    let f = c.ch.core.ftk;
    if f.module_on && (shot.is_none() || !f.on) {
        c.ch.core.ftk.module_on = false;
        exit(c);
        flags_off(c);
        lines.push("#say fallToKnees module exit".into());
    }
    let Some((time, falling)) = shot else { return false };
    if !c.ch.core.ftk.module_on && f.on && !falling && f.balance_time < time {
        c.ch.core.ftk.module_on = true;
        init(c);
        lines.push(format!("#say fallToKnees module enter at shot time {time:.2}"));
    }
    if !c.ch.core.ftk.module_on {
        return false;
    }
    core(c, lines, dt);
    let f = &mut c.ch.core.ftk;
    let mut kneeling = false;
    if f.kneeling {
        kneeling = true;
        if f.knee_down[0] || f.knee_down[1] {
            if f.release_rfw >= 0.0 {
                f.release_rfw = (f.release_rfw - dt).max(0.0);
                if f.release_rfw == 0.0 && c.extras_on.contains(&"reachForWound") {
                    lines.push("reachForWound start=false".into());
                    f.say(20, lines, || "reachForWound released".into());
                }
            }
            if f.release_point_gun && c.extras_on.contains(&"pointGun") {
                lines.push("pointGun start=false".into());
            }
        }
        flags(c);
    }
    kneeling
}

/// The core 0x2a2e0b4: one tick with the record (the same for all three callers).
pub fn core(c: &mut Ctx, lines: &mut Vec<String>, dt: f32) {
    let mut f = c.ch.core.ftk;
    let was_on = f.on;
    f.ticked = true;
    tick(&mut f, c, lines, dt);
    c.ch.core.ftk = f;
    if was_on && !f.on {
        // its own switch-off above 2 m/s
        exit(c);
    }
}

fn tick(f: &mut Ftk, c: &mut Ctx, lines: &mut Vec<String>, dt: f32) {
    let up = c.up;
    let running = c.upright;
    if let Some(d) = c.dbm {
        f.bal = (d.time, d.steps, d.quality, d.knee);
    }
    let (time, steps, q, knee_now) = f.bal;
    let v = c.vel;
    let speed = v.length();
    let (leg_l, leg_r) = (limb(2), limb(3));
    f.say(0, lines, || format!("core runs, balancer running {running}, quality {q:.3}"));
    // thighs and calves: friction 10 (0x56)
    f.fric = [10.0, 10.0, f.fric[2], 10.0, 10.0, f.fric[5]];
    f.theta = f.fall_below_stab + f.thresh_per_s * (time - f.t0) + f.thresh_per_step * (steps as f32 - f.steps0 as f32);
    if running && c.dbm.is_none_or(|d| d.stepping == 0) {
        for &j in ANKLE().iter() {
            c.ch.eff[j].set_stiffness(7.0, 1.0, Some(4.0));
        }
    }
    let flat = |x: Vec3| x - up * up.dot(x);
    let forward = c.m.c.dot(v.normalize_or_zero());
    let (ml, mr) = (c.b.part(leg_l.p[3]).m, c.b.part(leg_r.p[3]).m);
    // each foot's toe on the sole: row c runs from the toes to the heel (rdr2_falltoknees.md 3)
    let toe = |m: crate::nmdbm::M34| m.d - m.c * (FOOT_LENGTH * 0.5) - m.b * (FOOT_HEIGHT * 0.5);
    let (fl, fr) = (flat(toe(ml)), flat(toe(mr)));
    let q_at = flat(c.com + v * 0.2);
    let mid = flat((ml.d + mr.d) * 0.5);
    let n = (fr - fl).cross(up);
    let side = |x: Vec3| if (x - fr).dot(n) > 0.0 { 1 } else { -1 };
    let off = side(q_at) != side(mid) || f.knee_down[0] || f.knee_down[1];
    let hit = &c.b.doll.hit_out;
    let contact = hit[leg_l.p[3]].is_some() || hit[leg_r.p[3]].is_some();
    // the help toward the line through the toes
    let stay = crate::nmdbm_rdr2::stay_live(&c.ch.core.stay, v).use_forces;
    if !off && !f.sitting && ((q < 2.0 && stay) || q < 1.0) {
        let w = 2.0 - q.clamp(1.0, 2.0);
        let dist = ((fr + fl) * 0.5 - q_at).dot(n);
        let force = (n * (2.0 * f.helper_force * dist * w)).clamp_length_max(150.0);
        if contact {
            c.out.force(0, force);
            if f.helper_on_spine {
                c.out.force(chest_part(), force);
            }
        }
        let dir = (n * dist).normalize_or_zero();
        c.out.lean = Some((LeanMode::Dir(dir), f.lean_help, false));
        if !f.kneeling {
            c.out.hip_lean = Some(dir * (0.5 * f.lean_help));
        }
        f.say(1, lines, || format!("help: quality {q:.3}, weight {w:.2}, distance {dist:.4}, force {:.1} N, feet down {contact}", force.length()));
    }
    let stagger_on = c.extras_on.contains(&"staggerFall");
    if !f.kneeling {
        let fold = contact && (q < f.theta || (q < 0.99 && forward < -0.3 && speed > 0.2) || (off && forward < -0.3 && speed < 0.99));
        if !running && !fold && flat(v).length() > 2.0 {
            f.on = false;
            f.say(2, lines, || "switched off: failed and moving over 2 m/s".into());
        }
        if !f.on || (!fold && running) {
            return;
        }
        if running && stagger_on {
            lines.push("staggerFall start=false".into());
        }
        c.out.dbm_abort = Some(f.abort_threshold);
        c.out.dbm_must_collide = Some(f.fail_must_collide);
        c.out.dbm_kneeling = Some(true);
        f.kneeling = true;
        lines.push(format!("#say fallToKnees kneel starts: quality {q:.3}, threshold {:.3}, forward {forward:.2}, speed {speed:.2}, off the toes {off}, feet down {contact}, balancer running {running}", f.theta));
        return;
    }
    // kneeling: no gravity opposition on the legs and the lowest spine joint (through the balancer's request slots)
    for &j in HIP().iter().chain(KNEE().iter()).chain(ANKLE().iter()) {
        c.ch.eff[j].oppose_gravity = 0.0;
    }
    c.ch.eff[SPINE()[0]].oppose_gravity = 0.0;
    if running && stagger_on {
        lines.push("staggerFall start=false".into());
    }
    if !f.stay_upright {
        clear_stay(c);
        f.say(13, lines, || "StayUpright's sets cleared".into());
    }
    // still on his knees for a while: the balancer fails
    if running {
        if speed < 0.05 {
            f.still += dt;
        }
        if speed < 0.02 {
            f.still += dt;
        }
        if speed > 0.075 {
            f.still = 0.0;
        }
        if f.still > 2.0 {
            c.out.dbm_force_fail = true;
            f.say(3, lines, || "still for long enough: balancer fail".into());
        }
    }
    let pelvis = c.b.part(0).m.d;
    let knee_at = [c.b.joint_pos(KNEE()[0]), c.b.joint_pos(KNEE()[1])];
    let ankle_at = [c.b.joint_pos(ANKLE()[0]), c.b.joint_pos(ANKLE()[1])];
    if !f.sitting && speed < 0.5 && ankle_at[0].distance(pelvis) < 0.35 * L && ankle_at[1].distance(pelvis) < 0.35 * L {
        f.sitting = true;
        f.say(4, lines, || "sitting: both ankles by the pelvis".into());
    }
    // a knee is down when the foot's contact is low by the knee, or thigh or calf touch by it
    let hit = &c.b.doll.hit_out;
    for i in 0..2 {
        if f.knee_down[i] {
            continue;
        }
        let l = limb(2 + i);
        if let Some(at) = hit[l.p[3]] {
            let h = up.dot(knee_at[i] - at.0);
            if h < 0.23 * L {
                f.knee_down[i] = true;
            }
            if h < 0.33 * L && speed < 0.5 {
                f.sitting = true;
            }
        }
        // left 0.2 m, right 0.2 L, as read
        let near = if i == 0 { 0.2 } else { 0.2 * L };
        if !f.knee_down[i] && [l.p[1], l.p[2]].iter().any(|&k| hit[k].is_some_and(|at| at.0.distance(knee_at[i]) < near)) {
            f.knee_down[i] = true;
        }
        if f.knee_down[i] {
            f.say(5 + i as u32, lines, || format!("knee {i} down"));
        }
    }
    if f.sitting {
        clear_stay(c);
    }
    f.uneven = (knee_at[0].z - knee_at[1].z).abs() > 0.2 * L;
    // the feet's friction goes with how far they tip heel up
    f.fric[2] = (1.0 - f.fric_mult * up.dot(ml.c)).clamp(0.1, 1.0);
    f.fric[5] = (1.0 - f.fric_mult * up.dot(mr.c)).clamp(0.1, 1.0);
    // the requests of the balancer; moving straight back on both knees he falls back instead
    if forward >= -0.3 && forward > 0.98 && f.knee_down == [true; 2] && speed > 0.1 {
        f.fall_back = true;
        f.say(7, lines, || "falls back".into());
    } else {
        let pitch = if forward < -0.3 { f.pitch_forwards } else { f.pitch_backwards };
        use crate::nmdbm_rdr2::Crouch;
        c.out.dbm_crouch.push(Crouch::Knee(KNEE_OWNER, -0.5, f.bend_rate));
        // ours has the hip pitch's sign negated
        c.out.dbm_crouch.push(Crouch::Hip(HIP_OWNER, pitch, 100000.0));
        let rate = f.bend_rate;
        f.say(8, lines, || format!("asks the balancer: knee bend -0.5 at {rate}, hip pitch {pitch}"));
    }
    let any_knee = f.knee_down[0] || f.knee_down[1];
    if any_knee || f.sitting || knee_now <= -0.49 {
        // bal+0x464 and +0x465 (only +0x464 is read); 0x2ab9db0's forced-contact mask: none in Delirium
        c.out.dbm_force_balance = true;
        c.ch.core.foot_hold = !f.knee_spin;
        if f.knee_spin {
            c.ch.core.foot_unlatch = true;
            f.say(14, lines, || "ftkKneeSpin: the foot yaw hold lets go".into());
        }
        f.say(9, lines, || format!("holds: no new steps, knee bend {knee_now:.3}"));
    }
    // thighs come upright
    if any_knee {
        for i in 0..2 {
            if !f.lunge || f.knee_down[i] || !f.uneven {
                let a = c.ch.actual(HIP()[i], c.b.frames)[0];
                c.ch.eff[HIP()[i]].desired[0] = lerp(a, 0.0, f.hip_blend);
            }
        }
    }
    // the first landing of each knee: the upper body's muscle stiffness drops for a moment
    for i in 0..2 {
        if f.knee_down[i] && !f.impact[i] {
            f.impact_t = f.impact_time;
            f.impact[i] = true;
        }
    }
    if f.impact_t > 0.0 {
        let legs = crate::nmrs::Character::mask("lb");
        let to = 1.01 - f.impact_looseness;
        for j in 0..c.ch.eff.len() {
            if !legs.contains(&j) {
                c.ch.eff[j].muscle_stiffness = to;
            }
        }
        let left = f.impact_t;
        f.say(10, lines, || format!("impact: upper body muscle stiffness {to:.2} for {left:.2} s"));
        f.impact_t -= dt;
        if f.impact_t <= 0.0 {
            for j in 0..c.ch.eff.len() {
                c.ch.eff[j].muscle_stiffness = c.ch.joints[j].muscle_stiffness;
            }
            f.say(11, lines, || "impact over: muscle stiffness back".into());
        }
    }
    if f.fall_back {
        for i in 0..2 {
            let a = c.ch.actual(HIP()[i], c.b.frames)[0];
            c.ch.eff[HIP()[i]].desired[0] = lerp(a, f.hip_angle_fall, f.hip_blend);
        }
        c.ch.eff[SPINE()[0]].desired[1] = 0.0;
        c.ch.core.foot_hold = false;
        c.ch.core.foot_unlatch = true;
    }
    if f.knee_down == [true; 2] {
        (f.fric[2], f.fric[5]) = (0.0, 0.0);
        f.say(12, lines, || "both knees down: feet friction 0".into());
    }
}

fn lerp(a: f32, b: f32, t: f32) -> f32 {
    let t = t.clamp(0.0, 1.0);
    (1.0 - t) * a + b * t
}

/// 0x2a2b438: StayUpright's both sets lose useForces, useTorques, lastStandMode and torqueparts.
fn clear_stay(c: &mut Ctx) {
    for s in &mut c.ch.core.stay {
        (s.use_forces, s.use_torques, s.last_stand_mode, s.torque_parts) = (false, false, false, false);
    }
}
