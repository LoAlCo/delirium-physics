use crate::fneg;
use crate::Fma;
use crate::Vx;
use crate::prelude::*;
use bevy_math::Vec3Swizzles;

use crate::nmdbm::{asin, sdot, Body, M34};
use crate::nmrs::{Character, CLAVICLE, ELBOW, HIP, NECK, SHOULDER, SPINE, WRIST};

#[derive(Clone, Copy, Debug)]
pub struct Params {
    pub body_stiffness: f32,
    pub elbow: f32,
    pub shoulder: f32,
    pub damping: f32,
    pub max_steps: u32,
    pub head_look_pos: Vec3,
    pub head_look_instance_index: i32,
    /// The spine's stiffness (RDR2's spineStiffness; GTA IV's message calls it headLookAlwaysTimeout, the same
    /// number in the same place)
    pub spine_stiffness: f32,
    pub head_look_at_vel_prob: f32,
    pub somersault_angle: f32,
    pub somersault_angle_threshold: f32,
    pub side_somersault_angle: f32,
    pub side_somersault_angle_threshold: f32,
    pub ang_vel_multiplier: Vec3,
    pub ang_vel_threshold: Vec3,
    pub arms_out_on_push_multiplier: f32,
    pub arms_out_on_push_timeout: f32,
    pub returning_to_balance_arms_out: f32,
    pub arms_out_straighten_elbows: f32,
    pub arms_out_min_lean2: f32,
    /// The spine's damping (RDR2's spineDamping; GTA IV's bodyTurnTimeout)
    pub spine_damping: f32,
    pub turn_off_prob: f32,
    pub turn2_target_prob: f32,
    pub turn2_vel_prob: f32,
    pub turn_away_prob: f32,
    pub turn_left_prob: f32,
    pub turn_right_prob: f32,
    pub elbow_angle_on_contact: f32,
    pub bend_elbows_time: f32,
    pub bend_elbows_gait: f32,
    pub use_head_look: bool,
    pub arms_out_on_push: bool,
    pub use_body_turn: bool,
    /// RDR2's further parameters (read with the RDR2 core)
    pub rdr2: Rdr2Params,
}

/// bodyBalance's RDR2-only parameters (RDR2's names and defaults).
#[derive(Clone, Copy, Debug)]
pub struct Rdr2Params {
    /// shoulder lean1 offset (divided by shoulder)
    pub shoulder_l1: f32,
    /// shoulder lean2 offset
    pub shoulder_l2: f32,
    /// shoulder lean2 from the hip's lean2
    pub hip_l2_arm_l2: f32,
    pub shoulder_twist: f32,
    /// the arms swing only within this long (s) without a foot down
    pub airborne_arms: f32,
    /// arms-out ignores each arm's random on/off windows
    pub axis_weights_simulation_blocked: bool,
    /// lean away from a target moving past closer than this (m; <= 0 off)
    pub move_radius: f32,
    pub move_amount: f32,
    pub move_amount_still: f32,
    /// lean away only while bracing (no brace here: never)
    pub move_when_bracing: bool,
    /// moving backwards (his velocity within 60 degrees of his back): turn him to one side, keeping the side the
    /// first backward step chose (backwardsAutoTurn; RDR2's onFire sets it, no message here)
    pub backwards_auto_turn: bool,
    /// moving backwards: the elbows bent at least 1.5 and the spine's twist left alone (backwardsArms; likewise)
    pub backwards_arms: bool,
    /// a target moving past inside this turns him aside (turnWithBumpRadius; <= 0 off)
    pub bump_radius: f32,
    /// the upper body weak for this long from the start, unless hit, and by how much
    pub bump_time: f32,
    pub bump_weak: f32,
    /// the arms' brace: at this distance (<= 0 off), a target behind him cancels it till this long, the target raised by this
    pub brace_distance: f32,
    pub back_brace: f32,
    pub brace_offset: f32,
    /// the COM lean of a target moving past, him still (the force lean takes the moving amounts)
    pub lean_still: f32,
    /// turning to the target asks a real turn step (+0x139)
    pub turn_block: bool,
    pub gs_help: bool,
    /// the balancer gone: 0 catchFall, 1 rollDownStairs, 2 rollDownStairs till the upper body lands
    pub falling_reaction: i32,
    pub start_balancer: bool,
    /// the rest pose is the rig's zero pose
    pub zero_pose: bool,
}

impl Default for Rdr2Params {
    fn default() -> Rdr2Params {
        Rdr2Params { shoulder_l1: 0.0, shoulder_l2: 0.5, hip_l2_arm_l2: 0.3, shoulder_twist: -0.35, airborne_arms: 0.0,
                     axis_weights_simulation_blocked: true, move_radius: -1.0, move_amount: 0.3, move_amount_still: 0.3,
                     move_when_bracing: false, backwards_auto_turn: false, backwards_arms: false,
                     bump_radius: -1.0, bump_time: 0.0, bump_weak: 0.0, brace_distance: -1.0, back_brace: 0.5, brace_offset: 0.0, lean_still: 0.05,
                     turn_block: false, gs_help: false, falling_reaction: 0, start_balancer: true, zero_pose: false }
    }
}

impl Default for Params {
    fn default() -> Params {
        Params {
            body_stiffness: delirium_data::kv!(137), elbow: delirium_data::kv!(13), shoulder: 1.0, damping: delirium_data::kv!(74), max_steps: delirium_data::kv!(197 int), head_look_pos: Vec3::ZERO,
            head_look_instance_index: -1, spine_stiffness: delirium_data::kv!(31), head_look_at_vel_prob: -1.0,
            somersault_angle: 1.0, somersault_angle_threshold: delirium_data::kv!(17), side_somersault_angle: 1.0,
            side_somersault_angle_threshold: delirium_data::kv!(17), ang_vel_multiplier: Vec3::new(delirium_data::kv!(84), 1.0, delirium_data::kv!(84)),
            ang_vel_threshold: Vec3::new(delirium_data::kv!(136), delirium_data::kv!(5), delirium_data::kv!(136)), arms_out_on_push_multiplier: 0.0, arms_out_on_push_timeout: 1.0,
            returning_to_balance_arms_out: 0.0, arms_out_straighten_elbows: 0.0, arms_out_min_lean2: -delirium_data::kv!(188),
            spine_damping: 1.0, turn_off_prob: delirium_data::kv!(78), turn2_target_prob: delirium_data::kv!(60), turn2_vel_prob: delirium_data::kv!(24), turn_away_prob: delirium_data::kv!(9),
            turn_left_prob: delirium_data::kv!(187), turn_right_prob: delirium_data::kv!(187), elbow_angle_on_contact: delirium_data::kv!(186), bend_elbows_time: 0.5,
            bend_elbows_gait: delirium_data::kv!(74), use_head_look: false, arms_out_on_push: false, use_body_turn: false,
            rdr2: Rdr2Params::default(),
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Out {
    pub catch_fall: Option<[f32; 3]>,
    pub head_look: Option<(Vec3, Vec3)>,
    pub head_look_stiffness: f32,
    pub step_clamp: Option<f32>,
    pub turn: Option<Vec3>,
    /// RDR2: a lean asked of the balancer (leaning away from a target moving past)
    pub lean: Option<(crate::nmdbm::LeanMode, f32, bool)>,
    /// RDR2: this tick's turn is a real turn step (+0x139, the bump turn)
    pub turn_step: bool,
    /// RDR2: the arms' brace this tick: target, its velocity, distance, back time
    pub brace: Option<(Vec3, Vec3, f32, f32)>,
    /// RDR2, fallingReaction: rollDownStairs started if it isn't running, softened, and the fall's end (catchFall's stop)
    pub rds: bool,
    pub rds_soft: bool,
    pub rds_end: bool,
    /// RDR2: gshelp's writes on the core
    pub gs_help: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct BodyBalance {
    pub p: Params,
    arms_out: f32,
    floor: f32,
    euler: [f32; 3],
    bend: f32,
    cut: f32,
    stepping: u32,
    look_time: f32,
    look_rand: f32,
    turn_rand: f32,
    turn_time: f32,
    look_vel: bool,
    pub use_char_frame: bool,
    frame: [Vec3; 3],
    rd: Rdr2Bb,
}

/// RDR2's further state (with the RDR2 core).
#[derive(Clone, Copy, Debug, Default)]
struct Rdr2Bb {
    /// bump weakness's clock (+0x2e4); the falling reaction's two clocks (+0x2f8, +0x2fc: below -1 not yet)
    bump_t: f32,
    fall_soft: f32,
    fall_end: f32,
    /// the clock of sit's noisy elbows (+0x304)
    noise_t: f32,
    noisy_said: bool,
    /// each arm's arms-out on/off timer (off below 0) and its off time
    arm_on: [f32; 2],
    arm_off: [f32; 2],
    /// time without a foot down
    airborne: f32,
    /// the chosen turn (0 off, 1 to the target, 2 along the velocity, 3 away, 4 left, 5 right)
    turn: u8,
    /// the target's position and velocity estimate, its last move, held while it pauses, the last dt
    target: Vec3,
    target_vel: Vec3,
    moved: f32,
    paused: bool,
    last_dt: f32,
    /// the backwards turn's step counter (RDR2's reset 4, less one each backward step) and its side (true: the
    /// left foot's, toward his left)
    back_steps: i32,
    back_left: bool,
}

impl BodyBalance {
    /// NMX_POSE comparison: the turn state and target as a recording had them.
    pub fn nmx_turn(&mut self, state: u8, far: Vec3) {
        (self.rd.turn, self.turn_time, self.p.head_look_pos) = (state, 1.0, far);
    }

    pub fn activate(p: Params, ch: &mut Character, b: &Body) -> BodyBalance {
        if ch.rdr2 {
            return BodyBalance::activate_rdr2(p, ch, b);
        }
        for (e, j) in ch.eff.iter_mut().zip(&ch.joints) {
            e.reset(j);
            e.desired = [0.0; 3];
        }
        for i in (*NECK()) {
            ch.eff[i].oppose_gravity = 1.0;
        }
        for i in [(*SPINE())[0], (*SPINE())[1], (*SPINE())[2], (*CLAVICLE())[0], (*CLAVICLE())[1]] {
            ch.eff[i].oppose_gravity = 2.0;
        }
        for i in (*CLAVICLE()) {
            ch.eff[i].blend_to_zero(1.0);
        }
        let mut s = BodyBalance {
            p, arms_out: p.arms_out_on_push_timeout, floor: delirium_data::kv!(131), euler: [0.0; 3], bend: 0.0, cut: ch.random() * delirium_data::kv!(20),
            stepping: 0, look_time: 0.0, look_rand: 0.0, turn_rand: 0.0, turn_time: 0.0, look_vel: false,
            use_char_frame: false, frame: [Vec3::ZERO; 3], rd: Rdr2Bb::default(),
        };
        s.look_rand = ch.random();
        s.turn_rand = ch.random();
        s.frame = s.frame(b);
        s.euler = s.angles();
        s
    }

    pub fn deactivate(ch: &mut Character) {
        ch.stabilise = None;
    }

    fn frame(&self, b: &Body) -> [Vec3; 3] {
        if self.use_char_frame {
            let m = b.snap(Vec3::Z).m;
            [m.b, m.a, m.c]
        } else {
            let m = b.pmat(b.ch.joints[(*SPINE())[2]].child).m;
            [m.a, -m.b, m.c]
        }
    }

    fn angles(&self) -> [f32; 3] {
        let y = asin(self.frame[2].z);
        let k = 1.0 / (y as f64).cos() as f32;
        [asin(-self.frame[1].z * k), y, asin(-self.frame[2].y * k)]
    }

    #[allow(clippy::too_many_arguments)]
    pub fn tick(&mut self, ch: &mut Character, b: &Body, obstacles: &[crate::ragdoll::Obstacle], stepping: u32, failed: bool, catching: bool, dt: f32) -> Out {
        if ch.rdr2 {
            // sit narrows its mask (+0x14): what it would write on those joints doesn't happen
            let keep: Vec<_> = ch.bb_off.iter().map(|&j| (j, ch.eff[j].clone())).collect();
            let out = self.tick_rdr2(ch, b, obstacles, stepping, failed, catching, dt);
            for (j, e) in keep {
                ch.eff[j] = e;
            }
            return out;
        }
        let p = self.p;
        let mut out = Out::default();
        if failed {
            if !catching {
                ch.stabilise = None;
                let k = p.body_stiffness;
                out.catch_fall = Some([k * delirium_data::kv!(218), k * delirium_data::kv!(219), k * delirium_data::kv!(220)]);
            }
            return out;
        }
        if stepping != 0 && self.stepping != stepping {
            self.stepping = stepping;
            self.cut = ch.random() * delirium_data::kv!(20);
            if p.arms_out_on_push_timeout > delirium_data::kv!(221) && p.arms_out_on_push_timeout < delirium_data::kv!(222) {
                out.step_clamp = Some(ch.random() * delirium_data::kv!(136) + delirium_data::kv!(24));
            }
        }
        for i in (*SPINE()).into_iter().chain((*NECK())) {
            ch.eff[i].muscle_stiffness = delirium_data::kv!(12);
        }
        ch.set_stiffness(p.spine_stiffness, p.spine_damping, "ub", None);
        ch.set_stiffness(p.body_stiffness, p.damping, "ua", None);
        ch.set_stiffness(p.body_stiffness, 1.0, "un", None);
        ch.set_stiffness(delirium_data::kv!(137), 1.0, "uc", None);
        let k = p.body_stiffness * delirium_data::kv!(28);
        for i in (*ELBOW()) {
            ch.eff[i].damping = p.damping * delirium_data::kv!(20) * k * 2.0;
            ch.eff[i].strength = k * k;
        }
        let speed = (b.doll.kinetic() * 2.0).sqrt();
        let snap = b.snap(Vec3::Z);
        if p.use_head_look {
            out.head_look_stiffness = p.spine_stiffness + 1.0;
            let mut look = look_at(&p, b, obstacles);
            if stepping != 0 {
                self.look_time -= dt;
                if self.look_time < 0.0 {
                    self.look_time = ch.random() * delirium_data::kv!(60) + delirium_data::kv!(60);
                    if p.head_look_at_vel_prob < -delirium_data::kv!(134) {
                        self.p.head_look_at_vel_prob = self.look_rand;
                    }
                    self.look_vel = self.p.head_look_at_vel_prob > ch.random();
                }
                if self.look_vel {
                    let head = b.at(b.ch.joints[(*NECK())[1]].child);
                    let chest = b.doll.motion(b.ch.joints[(*SPINE())[delirium_data::kv!(0 int)]].child).0;
                    look = (snap.vel.mul_add(Vec3::splat(crate::rigid::nrm_k(snap.vel)), head), chest);
                }
            } else {
                self.look_time = 0.0;
            }
            out.head_look = Some(look);
        }
        let f19 = speed.clamp(delirium_data::kv!(134), 1.0);
        if p.use_body_turn {
            self.turn_time -= dt;
            if self.turn_time < 0.0 {
                self.turn_time = ch.random() * delirium_data::kv!(119) + delirium_data::kv!(116);
                self.turn_rand = ch.random();
            }
            let probs = [p.turn2_target_prob, p.turn2_vel_prob, p.turn_away_prob, p.turn_left_prob, p.turn_right_prob];
            let sum = probs.iter().fold(p.turn_off_prob, |s, x| s + x);
            if sum > delirium_data::kv!(72) {
                let inv = 1.0 / sum;
                let mut edge = 0.0;
                let edges = probs.map(|x| { edge = inv.fma(x, edge); edge });
                let r = self.turn_rand;
                let root = b.part(b.ch.joints[(*HIP())[0]].parent).m;
                let dir = if r <= edges[0] {
                    Some(p.head_look_pos - snap.com)
                } else if r <= edges[1] {
                    Some(snap.vel)
                } else if r <= edges[2] {
                    Some(-(p.head_look_pos - snap.com))
                } else if r <= edges[delirium_data::kv!(0 int)] {
                    Some(root.d - root.a - snap.com)
                } else if r <= edges[delirium_data::kv!(1 int)] {
                    Some(root.d + root.a - snap.com)
                } else {
                    None
                };
                if let Some(d) = dir.filter(|_| r < edges[delirium_data::kv!(1 int)]) {
                    out.turn = Some(d.normalize_or_zero());
                }
            }
        }
        if f19 < delirium_data::kv!(146) {
            out.turn = None;
        }
        for i in (*NECK()) {
            ch.eff[i].desired = [0.0; 3];
        }
        let m: M34 = snap.m;
        let side = m.b.vx(ch.up);
        let sin = |x: f32| (x as f64).sin() as f32;
        let pitch = sin(sdot(side, m.a, b"yzx")) * -2.0;
        let roll = sin(sdot(side, m.c, b"yzx")) * 2.0;
        for (i, l1) in (*SPINE()).into_iter().zip([-delirium_data::kv!(112), delirium_data::kv!(17), -delirium_data::kv!(112), delirium_data::kv!(112)]) {
            ch.eff[i].desired[0] = l1;
            ch.eff[i].desired[1] = 0.0;
        }
        let hip = |side: usize, des: bool| if des { ch.eff[(*HIP())[side]].desired[0] } else { b.ch.actual((*HIP())[side], b.frames)[0] };
        let tw = (hip(0, true) - hip(1, true)).fma(delirium_data::kv!(9), hip(0, false) - hip(1, false));
        for (i, k) in (*SPINE()).into_iter().zip([delirium_data::kv!(140), delirium_data::kv!(24), delirium_data::kv!(60), delirium_data::kv!(9)]) {
            ch.eff[i].desired[2] = tw * k;
        }
        let over = |x: f32, thr: f32| x.abs().max(thr).min(delirium_data::kv!(20)) - thr;
        let (tilt, side_tilt) = (over(pitch, p.somersault_angle_threshold), over(roll, p.side_somersault_angle_threshold));
        self.frame = self.frame(b);
        let now = self.angles();
        let inv = 1.0 / dt;
        let rate = [0, 1, 2].map(|k| -((now[k] - self.euler[k]) * inv));
        self.euler = now;
        let f5 = if rate[1] * pitch < 0.0 { p.returning_to_balance_arms_out } else { 1.0 };
        let f4 = if rate[0] * roll < 0.0 { 1.0 } else { p.returning_to_balance_arms_out };
        let spin = |x: f32, thr: f32| x.abs().max(thr).min(delirium_data::kv!(32)) - thr;
        let (t, m) = (p.ang_vel_threshold, p.ang_vel_multiplier);
        let f27 = (-(p.side_somersault_angle * f4)).fma(side_tilt, -(p.somersault_angle * f5 * tilt));
        let f28 = (-(spin(rate[1], t.x) * m.x)).fma(f5, f27);
        let f28 = (-spin(rate[2], t.y)).fma(m.y, f28);
        let f28 = (-(spin(rate[0], t.z) * m.z)).fma(f4, f28);
        let mut push = 0.0;
        if self.arms_out > 0.0 {
            self.arms_out -= dt;
            push = p.arms_out_on_push_multiplier;
        }
        if self.bend > 0.0 {
            self.bend -= dt;
        }
        if b.doll.collided(&Character::mask("ub").iter().map(|&i| b.ch.joints[i].child).collect::<Vec<_>>()) {
            self.bend = p.bend_elbows_time;
        }
        let foot = |side: usize| b.doll.touch[b.ch.joints[(*crate::nmrs::ANKLE())[side]].child];
        let down = |t: u8| t & delirium_data::kv!(7 int) != 0 && t & delirium_data::kv!(23 int) == 0;
        let grounded = down(foot(0)) || down(foot(1));
        let mut f30 = 0.0;
        for side in 0..2 {
            if ch.busy[side] || !grounded {
                continue;
            }
            let other = 1 - side;
            let act = |j: usize| b.ch.actual(j, b.frames);
            let (ho_des, ho_act, hs_act) = (ch.eff[(*HIP())[other]].desired, act((*HIP())[other]), act((*HIP())[side]));
            let x = (-ho_act[0]).fma(delirium_data::kv!(136), ho_des[0]);
            let x = (fneg(ho_act[0].fma(delirium_data::kv!(138), fneg(hs_act[0])))).fma(delirium_data::kv!(119), x);
            ch.eff[(*SHOULDER())[side]].desired[0] = x.fma(p.shoulder, act((*SHOULDER())[side])[0]);
            ch.eff[(*SHOULDER())[side]].desired[2] = -delirium_data::kv!(140);
            let mut elbow = (ho_des[0] - ho_act[0]).fma(p.elbow, act((*ELBOW())[side])[0]);
            if p.arms_out_on_push {
                elbow = p.arms_out_straighten_elbows.fma(if side == 0 { f28 } else { f30 + f28 }, elbow);
            }
            let mut cut = 0.0;
            if self.bend > 0.0 {
                let v = crate::rigid::nrm(snap.vel);
                let (fwd, lat) = (sdot(self.frame[2], v, b"zyx"), sdot(self.frame[1], v, b"zyx"));
                let toward = if side == 0 { lat > 0.0 } else { lat < 0.0 };
                if (fwd > 0.0 && toward) || fwd > 0.5 {
                    elbow = elbow.max(p.elbow_angle_on_contact);
                } else {
                    cut = self.cut;
                }
            }
            ch.eff[(*ELBOW())[side]].desired[0] = elbow.max(p.bend_elbows_gait).min(delirium_data::kv!(35));
            let hs_l2 = act((*HIP())[side])[1];
            let mut l2 = hs_l2.fma(delirium_data::kv!(24), 0.5);
            if p.arms_out_on_push {
                let max = l2 - cut;
                let c = |x: f32| x.abs().max(self.floor).min(delirium_data::kv!(31)) - self.floor;
                let (a, bb) = (c(ch.eff[(*HIP())[side]].desired[1]) * push, c(hs_l2) * push);
                let (cc, d) = (c(ch.eff[(*HIP())[other]].desired[1]) * push, c(act((*HIP())[other])[1]) * push);
                let o = cc.fma(-0.5, -(a * delirium_data::kv!(17)));
                let o = (-d).fma(0.5, o);
                let o = (-bb).fma(delirium_data::kv!(17), o);
                if side == 0 {
                    f30 = o;
                }
                l2 = (l2 + o + f28).max(p.arms_out_min_lean2).min(max);
            }
            ch.eff[(*SHOULDER())[side]].desired[1] = l2;
        }
        for i in (*WRIST()) {
            ch.eff[i].desired = [0.0; 3];
        }
        let k = (1.0 - f19).fma(delirium_data::kv!(20), delirium_data::kv!(60));
        for i in Character::mask("ub") {
            ch.eff[i].blend_to_zero(k);
        }
        ch.stabilise = Some((2.0, delirium_data::kv!(223)));
        out
    }
}

pub fn look_at(p: &Params, b: &Body, obstacles: &[crate::ragdoll::Obstacle]) -> (Vec3, Vec3) {
    let (i, x) = (p.head_look_instance_index, p.head_look_pos);
    if i >= 0 && i == b.ch.inst_id {
        return from_own(x, b);
    }
    crate::ragdoll::Obstacle::find(obstacles, i).map_or((x, Vec3::ZERO), |o| {
        let at = o.to_world(x);
        (at, o.vel_at(at))
    })
}

fn own(b: &Body) -> (Vec3, Quat) {
    let f = b.frames[0];
    (f.translation, f.rotation * crate::nmdbm::rage_link(&b.build.links, 0))
}

pub fn from_own(x: Vec3, b: &Body) -> (Vec3, Vec3) {
    let ((o, r), (v, w)) = (own(b), b.doll.motion(0));
    let at = match b.doll.inst {
        Some([a, bb, c, d]) => crate::vm4(a, bb, c, d, x),
        None => o + r * (x - crate::nmrs::cg(b.build.nm)),
    };
    let r = at - if b.doll.rigid.is_some() { b.at(0) } else { o };
    (at, (-r.yzx()).mul_add(w.zxy(), r.zxy().mul_add(w.yzx(), v)))
}

pub fn inst_place(doll: &crate::ragdoll::Ragdoll, nm: usize) -> Vec3 {
    match doll.inst {
        Some(m) => m[delirium_data::kv!(0 int)],
        None => doll.frames().next().map_or(Vec3::ZERO, |f| f.translation - f.rotation * crate::nmdbm::rage_link(&doll.links, 0) * crate::nmrs::cg(nm)),
    }
}

pub fn to_own(at: Vec3, b: &Body) -> Vec3 {
    if let Some(m) = b.doll.inst {
        return crate::rigid::to_frame(m, at);
    }
    let (o, r) = own(b);
    r.inverse() * (at - o) + crate::nmrs::cg(b.build.nm)
}

// ---- RDR2's bodyBalance (with the RDR2 core), from a study of RDR2's own (rdr2_bodybalance.md) ----

/// RDR2: the body turn only above this speed (sqrt(2E), m/s)
const RDR2_TURN_SPEED: f32 = 0.3;

/// Blend an effector's angles toward `to` by `w`.
fn blend(d: &mut [f32; 3], to: [f32; 3], w: f32) {
    for k in 0..3 {
        d[k] += (to[k] - d[k]) * w;
    }
}

impl BodyBalance {
    /// RDR2's activation: the non-leg joints' angles to 0 (their gains as they were), oppose gravity 1 on the neck
    /// and head and 2 on spine0-2 and the clavicles, the clavicles to their zero pose; each arm's arms-out window
    /// drawn; the torso frame and its angles.
    fn activate_rdr2(p: Params, ch: &mut Character, b: &Body) -> BodyBalance {
        let legs = Character::mask("lb");
        for j in (0..ch.eff.len()).filter(|j| !legs.contains(j)) {
            ch.eff[j].desired = [0.0; 3];
        }
        for i in (*NECK()) {
            ch.eff[i].oppose_gravity = 1.0;
        }
        for i in [(*SPINE())[0], (*SPINE())[1], (*SPINE())[2], (*CLAVICLE())[0], (*CLAVICLE())[1]] {
            ch.eff[i].oppose_gravity = 2.0;
        }
        for i in (*CLAVICLE()) {
            ch.eff[i].blend_to_zero(1.0);
        }
        let mut rd = Rdr2Bb { last_dt: 1.0 / 60.0, back_steps: 4, fall_soft: -2.0, fall_end: -2.0, ..Default::default() };
        // the backwards turn's first side: RDR2 draws it at every activation (bit 0 of a fresh generator state);
        // here only when backwardsAutoTurn is on, so every other bodyBalance draws as before
        if p.rdr2.backwards_auto_turn {
            ch.random();
            rd.back_left = ch.rng.0 & 1 != 0;
        }
        for i in 0..2 {
            rd.arm_on[i] = 1.4 * ch.random() - 0.7;
            rd.arm_off[i] = 0.4 * ch.random() + 0.3;
        }
        let mut s = BodyBalance {
            p, arms_out: p.arms_out_on_push_timeout, floor: 0.03, euler: [0.0; 3], bend: 0.0, cut: 0.0, stepping: 0,
            look_time: 0.0, look_rand: 0.0, turn_rand: 0.0, turn_time: 0.0, look_vel: true, use_char_frame: false,
            frame: [Vec3::ZERO; 3], rd,
        };
        s.frame = s.frame(b);
        s.euler = s.angles();
        s.cut = 0.8 * ch.random();
        s.look_rand = ch.random();
        s.rd.target = look_at(&p, b, &[]).0;
        s
    }

    /// RDR2's tick: the falling hand-over to catchFall; stiffness (spine; neck and head at the arms' stiffness;
    /// arms; clavicles 9; elbows (0.9 s)^2); the head look at its priority, sometimes along the velocity; the body
    /// turn chosen every 0.4-1 s; leaning away from a target moving past; spine lean and twist from the hips; arms
    /// out from the torso's tilt and spin, in random on/off windows per arm; the arm swing from the hips; then all
    /// but the legs blended toward a rest pose, more the slower he goes.
    #[allow(clippy::too_many_arguments)]
    fn tick_rdr2(&mut self, ch: &mut Character, b: &Body, obstacles: &[crate::ragdoll::Obstacle], stepping: u32, failed: bool, catching: bool, dt: f32) -> Out {
        let p = self.p;
        let r = p.rdr2;
        let mut out = Out::default();
        // with startbalancer off a balancer gone is none of its business (+0x3d1)
        if failed && r.start_balancer {
            if catching {
                return out;
            }
            ch.stabilise = None;
            // fallingReaction 1, 2: rollDownStairs, softer 0.4 s after the upper back lands (+0x2f8); 2 ends when any of the
            // upper body has touched (+0x2fc) [ours: mode 2's catchFall on the arms alone isn't here: the masks aren't]
            if r.falling_reaction == 1 || r.falling_reaction == 2 {
                out.rds = true;
                let upper = [(*SPINE())[2] + 1, (*SPINE())[3] + 1, (*NECK())[0] + 1, (*NECK())[1] + 1];
                if self.rd.fall_soft < -1.0 && b.doll.collided(&upper) {
                    self.rd.fall_soft = 0.0;
                }
                if self.rd.fall_soft >= 0.0 {
                    self.rd.fall_soft += dt;
                }
                out.rds_soft = self.rd.fall_soft > 0.4;
                if r.falling_reaction == 1 {
                    return out;
                }
                let body: Vec<usize> = std::iter::once(0).chain(7..b.frames.len()).collect();
                if self.rd.fall_end < -1.0 && b.doll.collided(&body) {
                    self.rd.fall_end = 0.0;
                }
                if self.rd.fall_end >= 0.0 {
                    self.rd.fall_end += dt;
                    out.rds_end = true;
                }
                if self.rd.fall_end < 0.0 {
                    return out;
                }
            }
            // fallingReaction 0: catchFall with its defaults (8.25, 10, 13) scaled by armStiffness / 9
            let k = p.body_stiffness / 9.0;
            out.catch_fall = Some([8.25 * k, 10.0 * k, 13.0 * k]);
            return out;
        }
        // gshelp, unless braceForImpact runs (it has its own)
        out.gs_help = r.gs_help;
        // bump weakness: the first bumpweaknesstime s, unless a reaction is ramping (0x2a50ff4 on all but legs and wrists)
        if !ch.bb_hit && r.bump_time > 0.0 && self.rd.bump_t < r.bump_time {
            let wrists = *crate::nmrs::WRIST();
            for j in Character::mask("ub").into_iter().chain(Character::mask("ua")).filter(|j| !wrists.contains(j)) {
                ch.core.lower_scale(j, 0.9, 1.0 - r.bump_weak, 1.0 - r.bump_weak);
            }
            crate::extras_core_rdr2::trace_once("bodyBalance: bump weakness", || format!("{} for {} s", r.bump_weak, r.bump_time));
            self.rd.bump_t += dt;
        }
        // moving backwards (only looked at with backwardsAutoTurn or backwardsArms)
        let up = ch.up;
        let snap = b.snap(Vec3::Z);
        let flat = |v: Vec3| v - up * up.dot(v);
        let back = (r.backwards_auto_turn || r.backwards_arms)
            && flat(snap.vel).normalize_or_zero().dot(flat(snap.m.c).normalize_or_zero()) > 0.5;
        if stepping != 0 && self.stepping != stepping {
            self.stepping = stepping;
            self.cut = 0.8 * ch.random();
            if back {
                self.rd.back_steps -= 1;
            }
        }
        // stiffness
        for i in (*SPINE()).into_iter().chain(*NECK()) {
            ch.eff[i].muscle_stiffness = 1.5;
        }
        ch.set_stiffness(p.spine_stiffness, p.spine_damping, "ub", None);
        ch.set_stiffness(p.body_stiffness, 1.0, "un", None);
        ch.set_stiffness(p.body_stiffness, p.damping, "ua", None);
        ch.set_stiffness(9.0, 1.0, "uc", None);
        let k = 0.9 * p.body_stiffness;
        for i in (*ELBOW()) {
            (ch.eff[i].strength, ch.eff[i].damping) = (k * k, 2.0 * k * 0.8 * p.damping);
        }
        let f = (b.doll.kinetic() * 2.0).sqrt().clamp(0.001, 1.0);
        let (target, target_vel) = ch.bb_target.take().map_or_else(|| look_at(&p, b, obstacles), |t| (t, Vec3::ZERO));
        // the head look (headLook's priority 0xc); along the velocity for a while on some steps
        if p.use_head_look {
            out.head_look_stiffness = 1.0 + p.spine_stiffness;
            let mut look = (target, target_vel);
            if stepping == 0 {
                self.look_time = 0.0;
            } else {
                self.look_time -= dt;
                if self.look_time < 0.0 {
                    self.look_time = 0.2 * ch.random() + 0.2;
                    if p.head_look_at_vel_prob < -0.001 {
                        self.p.head_look_at_vel_prob = self.look_rand;
                    }
                    self.look_vel = ch.random() < self.p.head_look_at_vel_prob;
                }
                if self.look_vel {
                    let head = b.at(b.ch.joints[(*NECK())[1]].child);
                    let chest = b.doll.motion(b.ch.joints[(*SPINE())[3]].child).0;
                    look = (head + crate::rigid::nrm(snap.vel), chest);
                }
            }
            out.head_look = Some(look);
        }
        // the body turn
        if p.use_body_turn {
            self.turn_time -= dt;
            if self.turn_time < 0.0 {
                self.turn_time = 0.6 * ch.random() + 0.4;
                let w = [p.turn_off_prob, p.turn2_target_prob, p.turn2_vel_prob, p.turn_away_prob, p.turn_left_prob, p.turn_right_prob];
                let sum: f32 = w.iter().sum();
                let u = ch.random();
                if sum > 0.0 {
                    let mut edge = 0.0;
                    self.rd.turn = w.iter().position(|x| { edge += x / sum; edge >= u }).unwrap_or(0) as u8;
                }
            }
            let d = match self.rd.turn {
                1 => {
                    out.turn_step = r.turn_block;
                    Some(target - snap.com)
                }
                2 => Some(snap.vel),
                3 => Some(snap.com - target),
                4 => Some(from_own(Vec3::new(-1.0, 0.0, 0.0), b).0 - snap.com),
                5 => Some(from_own(Vec3::new(1.0, 0.0, 0.0), b).0 - snap.com),
                _ => None,
            };
            if let Some(d) = d.filter(|_| f > RDR2_TURN_SPEED) {
                out.turn = Some(d.normalize_or_zero());
            }
        }
        // the target's velocity from its moves (kept while it pauses)
        let moved = target - self.rd.target;
        let dv = moved / dt;
        if dv.length() >= 1e-6 || self.rd.moved <= 1e-6 {
            self.rd.target_vel = moved / (dt + if self.rd.paused { self.rd.last_dt } else { 0.0 });
            self.rd.paused = false;
        } else {
            self.rd.paused = true;
        }
        (self.rd.moved, self.rd.target, self.rd.last_dt) = (dv.length(), target, dt);
        // the arms' brace on the target, raised by braceOffset (0x2a16654, run by the agent with its Ctx)
        let raised = target + up * r.brace_offset;
        if r.brace_distance > 0.0 {
            out.brace = Some((raised, self.rd.target_vel, r.brace_distance, r.back_brace));
        }
        // the bump turn: a target moving past inside turnWithBumpRadius turns him to a side, over the body turn; the side by
        // which way it goes past (+0x3d3), the same point in his own frame as the backwards turn
        let vb = self.rd.target_vel;
        let dh = flat(snap.com - raised);
        let bumped = r.bump_radius > 0.0 && vb.length() > 0.2 && dh.length() < r.bump_radius;
        if bumped {
            self.rd.back_left = up.cross(vb).dot(snap.com - raised) < 0.0;
            let x = if self.rd.back_left { -2.0 } else { 2.0 };
            let d = flat(from_own(Vec3::new(x, 2.0, 0.0), b).0 - snap.com);
            if f > RDR2_TURN_SPEED {
                out.turn = Some(d.normalize_or_zero());
                out.turn_step = true;
            }
            crate::extras_core_rdr2::trace_once("bodyBalance: bump turn", || format!("left {}", self.rd.back_left));
        }
        // the backwards turn (owner 2, over the body turn): the side from the stepping foot while the counter is 3 or
        // more ([P]: the note's "otherwise 4" read as not moving backwards), a point 2 m to that side and 2 m ahead
        // in his own frame
        if bumped {
        } else if back && r.backwards_auto_turn {
            if self.rd.back_steps >= 3 && stepping != 0 {
                (self.rd.back_left, self.rd.back_steps) = (stepping == 1, 3);
            }
            let x = if self.rd.back_left { -2.0 } else { 2.0 };
            let d = flat(from_own(Vec3::new(x, 2.0, 0.0), b).0 - snap.com);
            if f > RDR2_TURN_SPEED {
                out.turn = Some(d.normalize_or_zero());
            }
        } else {
            self.rd.back_steps = 4;
        }
        // lean away from a target moving past (the force lean only: our balancer has one lean channel)
        let vt = self.rd.target_vel;
        let d = flat((snap.com - target) - vt * 0.05);
        if vt.length() > 0.2 && !r.move_when_bracing && d.length() < r.move_radius {
            let mut side = up.cross(vt);
            if side.dot(d) < 0.0 {
                side = -side;
            }
            // RDR2 also asks something of the balancer here (+0x44c, unknown) before the still amount
            let amount = if snap.vel.length() < 1.0 { r.move_amount_still } else { r.move_amount };
            out.lean = Some((crate::nmdbm::LeanMode::Dir(side.normalize_or_zero()), amount, true));
            // the COM lean's channel (+0x760, 0.05, or leanamountstill still) has no second slot here: as far as the step, a hip lean
            let com = if snap.vel.length() < 1.0 { r.lean_still } else { 0.05 };
            crate::extras_core_rdr2::trace_once("bodyBalance: lean away", || format!("force {amount}, COM {com} (COM's channel not applied)"));
        }
        // spine and neck
        for i in (*NECK()) {
            ch.eff[i].desired = [0.0; 3];
        }
        for (k, (i, l1)) in (*SPINE()).into_iter().zip([-0.07, 0.25, -0.07, 0.07]).enumerate() {
            ch.eff[i].desired[0] = l1;
            if k > 0 {
                ch.eff[i].desired[1] = 0.0;
            }
        }
        let act = |j: usize| b.ch.actual(j, b.frames);
        let (hl, hr) = ((*HIP())[0], (*HIP())[1]);
        let tw = ((act(hl)[0] - act(hr)[0]) + 0.15 * (ch.eff[hl].desired[0] - ch.eff[hr].desired[0])).clamp(-0.8, 0.8);
        if !back {
            for (i, k) in (*SPINE()).into_iter().zip([0.35, 0.3, 0.2, 0.15]) {
                ch.eff[i].desired[2] = k * tw;
            }
        }
        // arms out: the torso's pitch and roll past their thresholds, and its spin
        let m: M34 = snap.m;
        let side = m.b.cross(up);
        let pitch = -2.0 * m.a.dot(side).sin();
        let roll = 2.0 * m.c.dot(side).sin();
        let over = |x: f32, thr: f32| x.abs().max(thr).min(0.8) - thr;
        let (tilt, side_tilt) = (over(pitch, p.somersault_angle_threshold), over(roll, p.side_somersault_angle_threshold));
        self.frame = self.frame(b);
        let now = self.angles();
        let rate = [0, 1, 2].map(|k| (now[k] - self.euler[k]) / dt);
        self.euler = now;
        let f5 = if rate[1] * pitch > 0.0 { p.returning_to_balance_arms_out } else { 1.0 };
        let f4 = if rate[0] * roll > 0.0 { 1.0 } else { p.returning_to_balance_arms_out };
        let spin = |x: f32, thr: f32| x.abs().max(thr).min(7.0) - thr;
        let (t, mu) = (p.ang_vel_threshold, p.ang_vel_multiplier);
        let arms_out = -tilt * p.somersault_angle * f5 - side_tilt * p.side_somersault_angle * f4 - mu.x * spin(rate[1], t.x) * f5
            - mu.y * spin(rate[2], t.y) - mu.z * spin(rate[0], t.z) * f4;
        let out_arm = [0, 1].map(|i| if self.rd.arm_on[i] < 0.0 { 0.0 } else { arms_out });
        let mut push = 0.0;
        if self.arms_out > 0.0 {
            self.arms_out -= dt;
            push = p.arms_out_on_push_multiplier;
        }
        if self.bend > 0.0 {
            self.bend -= dt;
        }
        let upper: Vec<usize> = Character::mask("uk").into_iter().chain(Character::mask("ua")).map(|i| b.ch.joints[i].child).collect();
        if b.doll.collided(&upper) {
            self.bend = p.bend_elbows_time;
        }
        let foot = |side: usize| b.doll.touch[b.ch.joints[(*crate::nmrs::ANKLE())[side]].child];
        let down = |t: u8| t & delirium_data::kv!(7 int) != 0 && t & delirium_data::kv!(23 int) == 0;
        self.rd.airborne = if down(foot(0)) || down(foot(1)) { 0.0 } else { self.rd.airborne + dt };
        let windows = self.rd.arm_on;
        for i in 0..2 {
            self.rd.arm_on[i] -= dt;
            if self.rd.arm_on[i] < -self.rd.arm_off[i] {
                self.rd.arm_on[i] = 0.6 * ch.random() + 0.1;
                self.rd.arm_off[i] = 0.4 * ch.random() + 0.3;
            }
        }
        // the arm swing
        let mut push_l = 0.0;
        for s in 0..2 {
            if ch.busy[s] || self.rd.airborne > r.airborne_arms {
                continue;
            }
            let o = 1 - s;
            let (ho_des, ho_act, hs_act) = (ch.eff[(*HIP())[o]].desired, act((*HIP())[o]), act((*HIP())[s]));
            let sh = (*SHOULDER())[s];
            let mut x = (ho_des[0] - 1.2 * ho_act[0]) - 0.6 * (1.1 * ho_act[0] - hs_act[0]);
            if p.shoulder > 0.1 {
                x += r.shoulder_l1 / p.shoulder;
            }
            ch.eff[sh].desired[0] = (x * p.shoulder + act(sh)[0]).clamp(-9.99, 9.99);
            ch.eff[sh].desired[2] = r.shoulder_twist;
            let pushing = p.arms_out_on_push && (windows[s] > 0.0 || r.axis_weights_simulation_blocked);
            let el = (*ELBOW())[s];
            let mut e = p.elbow * (ho_des[0] - ho_act[0]) + act(el)[0];
            if pushing {
                e += p.arms_out_straighten_elbows * if s == 0 { out_arm[0] } else { out_arm[1] + push_l };
            }
            let mut cut = 0.0;
            if self.bend > 0.0 {
                let v = crate::rigid::nrm(snap.vel);
                let (fwd, lat) = (self.frame[2].dot(v), self.frame[1].dot(v));
                let toward = if s == 0 { lat > 0.0 } else { lat < 0.0 };
                if (fwd > 0.0 && toward) || fwd > 0.5 {
                    e = e.max(p.elbow_angle_on_contact);
                } else {
                    cut = self.cut;
                }
            }
            let floor = if back && r.backwards_arms { 1.5 } else { p.bend_elbows_gait };
            ch.eff[el].desired[0] = e.max(floor).min(6.0);
            let base = hs_act[1] * r.hip_l2_arm_l2 + r.shoulder_l2;
            let mut l2 = base;
            if pushing {
                let c = |x: f32| x.abs().max(self.floor).min(10.0) - self.floor;
                let o = -push * (0.5 * c(ho_des[1]) + 0.25 * c(ch.eff[(*HIP())[s]].desired[1]) + 0.5 * c(ho_act[1]) + 0.25 * c(hs_act[1]));
                if s == 0 {
                    push_l = o;
                }
                l2 = (base + out_arm[s] + o).max(p.arms_out_min_lean2).min(base - cut);
            }
            ch.eff[sh].desired[1] = l2;
            ch.eff[(*WRIST())[s]].desired = [0.0; 3];
        }
        // toward rest, more the slower he goes: clavicles and wrists 0, elbows 0.4, shoulders lean2 0.5; spine, neck
        // and head 0
        let w = 0.2 + 0.8 * (1.0 - f);
        // blendToZeroPose: toward the rig's zero pose, every joint having one here, in place of the fixed rest below
        if r.zero_pose {
            for j in Character::mask("ub").into_iter().chain(Character::mask("ua")) {
                ch.eff[j].blend_to_zero(w);
            }
            crate::extras_core_rdr2::trace_first("bodyBalance: blend to the zero pose");
            ch.stabilise = None;
            return out;
        }
        // sit's noisy elbows (+0x150, set by a riding sit, cleared here)
        self.rd.noise_t += dt;
        let noisy = std::mem::take(&mut ch.bb_noisy);
        if noisy && !self.rd.noisy_said && crate::extras_ysk_rdr2::trace() {
            self.rd.noisy_said = true;
            eprintln!("bodyBalance: sit's noisy elbows, {} joints out of its mask", ch.bb_off.len());
        }
        for s in 0..2 {
            let te = if noisy {
                let (a, b) = (2.0 * self.rd.noise_t, 0.0);
                0.4 + if s == 0 { crate::nmfl::noise01(a, b, 0.0) } else { crate::nmfl::noise01(b, a, 0.0) }
            } else {
                0.4
            };
            blend(&mut ch.eff[(*CLAVICLE())[s]].desired, [0.0; 3], w);
            blend(&mut ch.eff[(*WRIST())[s]].desired, [0.0; 3], w);
            blend(&mut ch.eff[(*SHOULDER())[s]].desired, [0.0, 0.5, 0.0], w);
            let e = &mut ch.eff[(*ELBOW())[s]].desired[0];
            *e += (te - *e) * w;
        }
        for i in Character::mask("ub") {
            blend(&mut ch.eff[i].desired, [0.0; 3], w);
        }
        // RDR2's bodyBalance has no stayUpright step (rdr2_bodybalance.md): GTA IV's stabiliser stays off
        ch.stabilise = None;
        out
    }
}

