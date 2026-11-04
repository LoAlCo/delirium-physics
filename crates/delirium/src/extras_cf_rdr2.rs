//! RDR2 core: controlledFall (rdr2-decomp/study/rdr2_controlledfall.md, new in RDR2: no GTA IV or GTA V behaviour of
//! that name) as an extra message, and RDR2's onFire (rdr2_onfire.md), which the onFire extra (extras_rdr2.rs) runs
//! instead of Delirium's own while the core is on. Both from clean-room studies of RDR2's code; neither does anything
//! different without the core (controlledFall does nothing, onFire stays Delirium's).
//!
//! Delirium has no per-part friction scale, so the friction both set (controlledFall's feet 0.4, onFire's hands and
//! arms) is left out; RDR2's stiffness, length and torque scales (S, L, T) are 1 here.
use crate::extras::{Extra, P};
use crate::msg::{Error, Kind, Message, Param};
use crate::nmdbm_rdr2::Crouch;
use crate::nmrs::{NECK, SPINE};
use crate::nmshot::{limb, spine_parts, Ctx};
use bevy_math::{Quat, Vec3};

pub static MESSAGES: [Message; 1] = [
    Message { name: "controlledFall", params: &[
        Param { name: "state", kind: Kind::Int, used: false, doc: "RDR2's; read by nothing in its behaviour (0)" },
        Param { name: "TARGET", kind: Kind::Vec3, used: true, doc: "the point his steered part's front (its -c axis) turns toward, world (zero)" },
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "the turn's gain: spring stiffness^2, damping stiffness x damping (15)" },
        Param { name: "damping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "recovery", kind: Kind::Float, used: true, doc: "while the shot runs, the gain times its strength scale times this, at most 1 (1.5)" },
        Param { name: "timeWarpStrengthScale", kind: Kind::Float, used: true, doc: "with timeWarpActive, the gain times sqrt((1/30)/dt x this); also pointGun's (1)" },
        Param { name: "part", kind: Kind::Int, used: true, doc: "the steered part, ours 0..20 (-1: RDR2's part 9, Spine2). RDR2 takes a part handle" },
        Param { name: "timeWarpActive", kind: Kind::Bool, used: true, doc: "(false)" },
        Param { name: "scalePitch", kind: Kind::Float, used: true, doc: "the gain's share for the pitch and roll axes (0.5)" },
        Param { name: "dragScale", kind: Kind::Float, used: true, doc: "share of his COM's acceleration cancelled at the part at the start (0.5)" },
        Param { name: "dragTime", kind: Kind::Float, used: true, doc: "s that share fades to nothing over (0.5)" },
        Param { name: "groundBoundHandle", kind: Kind::Int, used: false, doc: "RDR2's; read by nothing in its behaviour (0)" },
    ] },
];

pub fn build(name: &str, p: &mut P) -> Option<Result<Box<dyn Extra>, Error>> {
    match name {
        "controlledFall" => Some(ControlledFall::new(p).map(|e| Box::new(e) as _)),
        _ => None,
    }
}

/// Event feedback (controlledFall's contact).
#[allow(non_snake_case)]
#[inline]
fn EVENT() -> u8 { delirium_data::kv!(127 int) }

/// RDR2's stiffness, length and torque scales (ctx+0x2c64, +0x2bfc, +0x2c44): 1 here.
const S: f32 = 1.0;
const L: f32 = 1.0;
const T: f32 = 1.0;

/// RDR2's balancer request rate that means "at once".
const NOW: f32 = 1e5;

// ---- controlledFall (slot 0xc8) ----

/// RDR2's controlledFall: one part (Spine2 by default) is turned so its front faces TARGET and its side axis stays
/// level, by three angular impulses about its own axes (yaw stiffness x the time warp, pitch and roll scalePitch of
/// that, all times the shot's recovery while the shot runs); at the start a fading share (dragScale over dragTime) of
/// the whole body's COM acceleration is cancelled at that part, slowing the start of the fall. It writes pointGun's
/// timeWarpStrengthScale and turns its left hand off, and sends "controlledFall" feedback every step the trunk,
/// neck or head touches something. It owns no limbs, never asks the balancer anything and never stops by itself.
/// Without the RDR2 core it does nothing.
pub struct ControlledFall {
    target: Vec3,
    stiffness: f32,
    damping: f32,
    recovery: f32,
    tws: f32,
    part: i32,
    time_warp: bool,
    scale_pitch: f32,
    drag_scale: f32,
    drag_time: f32,
    /// time since it started, the time-warp gain, the recovery factor, the drag share now
    t: f32,
    w: f32,
    r: f32,
    drag: f32,
    /// the COM velocity last step (RDR2 keeps its COM acceleration by difference; ours from the first step on)
    last_vel: Option<Vec3>,
    pg: Option<f32>,
}

impl ControlledFall {
    fn new(p: &mut P) -> Result<ControlledFall, Error> {
        let mut s = ControlledFall {
            target: Vec3::ZERO, stiffness: 15.0, damping: 1.0, recovery: 1.5, tws: 1.0, part: -1, time_warp: false,
            scale_pitch: 0.5, drag_scale: 0.5, drag_time: 0.5, t: 0.0, w: 1.0, r: 1.0, drag: 0.0, last_vel: None, pg: None,
        };
        s.parse(p)?;
        Ok(s)
    }

    /// The message over the values it has (RDR2's handler: defaults first on a start, which `new` gives).
    fn parse(&mut self, p: &mut P) -> Result<(), Error> {
        let _ = (p.i("state", 0)?, p.i("groundBoundHandle", 0)?);
        self.target = p.v("TARGET", self.target)?;
        self.stiffness = p.f("stiffness", self.stiffness)?;
        self.damping = p.f("damping", self.damping)?;
        self.recovery = p.f("recovery", self.recovery)?;
        self.tws = p.f("timeWarpStrengthScale", self.tws)?;
        self.part = p.i("part", self.part)?;
        self.time_warp = p.b("timeWarpActive", self.time_warp)?;
        self.scale_pitch = p.f("scalePitch", self.scale_pitch)?;
        self.drag_scale = p.f("dragScale", self.drag_scale)?;
        self.drag_time = p.f("dragTime", self.drag_time)?;
        Ok(())
    }

    /// The steered part: RDR2's default (a zero handle) is the character's own part 9, Spine2.
    fn part(&self, c: &Ctx) -> usize {
        match self.part {
            k @ 0.. if (k as usize) < c.b.build.parts.len() => k as usize,
            _ => spine_parts()[3],
        }
    }

    /// The orientation controller (0x2aec424): the yaw error of the target in the part's (b, -c) plane about a, the
    /// pitch error about b, the roll error (a's height) about c, each a PD impulse along that axis. RDR2 applies them
    /// as angular impulses with the world dt in them ([P]: 0x2a1e410 not opened); here a torque over this step.
    fn orient(&self, c: &mut Ctx, k: usize, dt: f32) {
        use std::f32::consts::PI;
        let part = c.b.part(k);
        let (a, b, cc) = (part.m.a, part.m.b, part.m.c);
        let d = (self.target - part.m.d).normalize_or_zero();
        let (x, y, z) = (a.dot(d), b.dot(d), cc.dot(d));
        let asin = |v: f32| v.clamp(-1.0, 1.0).asin();
        let yaw = if z < 0.0 { asin(y) } else if y > 0.0 { PI - asin(y) } else { -PI - asin(y) };
        let pitch = if z < 0.0 { -asin(x) } else { asin(x) };
        let roll = c.up.dot(a);
        let s = self.stiffness * self.r;
        let (ka, kb) = (self.w * s, self.scale_pitch * self.w * s);
        let (dd, om) = (self.damping, part.w);
        let j = a * (ka * dt * (s * yaw - dd * om.dot(a))) + b * (kb * dt * (s * pitch - dd * om.dot(b)))
            + cc * (kb * dt * (s * roll - dd * om.dot(cc)));
        c.out.torque(k, j / dt);
    }
}

impl Extra for ControlledFall {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        self.pg = None;
        if !c.ch.rdr2 || dt <= 0.0 {
            return;
        }
        let k = self.part(c);
        // 1 the time warp, 2 the recovery from the shot's strength scale
        self.w = if self.time_warp { ((1.0 / 30.0) / dt * self.tws).sqrt() } else { 1.0 };
        self.r = if c.shot_on { (c.shot_scale * self.recovery).clamp(0.0, 1.0) } else { 1.0 };
        // 3 the turn
        self.orient(c, k, dt);
        // 4 the fall's drag: a fading share of the body's COM acceleration (gravity included) cancelled at the part
        // (ours: dragTime 0 drags nothing)
        self.drag = if self.drag_time > 0.0 { (1.0 - self.t.clamp(0.0, self.drag_time) / self.drag_time) * self.drag_scale } else { 0.0 };
        let acc = self.last_vel.map_or(Vec3::ZERO, |v| (c.vel - v) / dt);
        self.last_vel = Some(c.vel);
        let j = -c.b.doll.mass() * acc * self.drag * dt;
        if j != Vec3::ZERO {
            c.out.impulse.push((k, c.b.at(k), j));
        }
        // 5 pointGun's timeWarpStrengthScale (and its left hand off: extras.rs applies both)
        self.pg = Some(self.tws * self.r);
        // 6 feedback while the trunk, neck or head touches (RDR2's mask 0x102709)
        if c.b.doll.collided(&spine_parts()) {
            c.out.send(EVENT(), "controlledFall", 0.0);
        }
        self.t += dt;
    }

    fn update(&mut self, p: &mut P) -> Result<(), Error> {
        self.parse(p)
    }

    fn pg_coupling(&self) -> Option<f32> {
        self.pg
    }
}

// ---- onFire (slot 0xe0) ----

/// onFire's parameters, RDR2's defaults (GTA V's set).
#[derive(Clone, Copy, Debug)]
pub(crate) struct FireParams {
    stagger_time: f32,
    stagger_lean_rate: f32,
    back: f32,
    fwd: f32,
    /// writhe blends (arms, spine, legs) on his feet, and otherwise
    stand: [f32; 3],
    pose: [f32; 3],
    roll_torque_scale: f32,
    predict_time: f32,
    roll_over: bool,
    max_roll_over_time: f32,
    roll_over_radius: f32,
}

impl Default for FireParams {
    fn default() -> FireParams {
        FireParams { stagger_time: 2.5, stagger_lean_rate: 0.9, back: 0.4, fwd: 0.5, stand: [0.4, 0.7, 0.2], pose: [0.7, 0.55, 0.5],
                     roll_torque_scale: 25.0, predict_time: 0.1, roll_over: true, max_roll_over_time: 8.0, roll_over_radius: 2.0 }
    }
}

impl FireParams {
    /// The message over `d`, unclamped (as RDR2 takes them).
    pub(crate) fn parse(p: &mut P, d: FireParams) -> Result<FireParams, Error> {
        Ok(FireParams {
            stagger_time: p.f("staggerTime", d.stagger_time)?, stagger_lean_rate: p.f("staggerLeanRate", d.stagger_lean_rate)?,
            back: p.f("stumbleMaxLeanBack", d.back)?, fwd: p.f("stumbleMaxLeanForward", d.fwd)?,
            stand: [p.f("armsWindmillWritheBlend", d.stand[0])?, p.f("spineStumbleWritheBlend", d.stand[1])?, p.f("legsStumbleWritheBlend", d.stand[2])?],
            pose: [p.f("armsPoseWritheBlend", d.pose[0])?, p.f("spinePoseWritheBlend", d.pose[1])?, p.f("legsPoseWritheBlend", d.pose[2])?],
            roll_torque_scale: p.f("rollTorqueScale", d.roll_torque_scale)?, predict_time: p.f("predictTime", d.predict_time)?,
            roll_over: p.b("rollOverFlag", d.roll_over)?, max_roll_over_time: p.f("maxRollOverTime", d.max_roll_over_time)?,
            roll_over_radius: p.f("rollOverRadius", d.roll_over_radius)?,
        })
    }
}

/// RDR2's onFire. Its children: bodyBalance (backwards arms and auto-turn, no body turn or head look) for
/// staggerTime or until the balance fails, RDR2's armsWindmill (adaptive, arms a quarter turn apart) while he is
/// upright, and RDR2's bodyWrithe (its onFire byte) throughout. Each step it classifies his attitude predictTime
/// ahead (upright, upside down, front down, front up, on either side); upright, the balancer's hip pitch wanders
/// between random lean targets within stumbleMaxLeanBack..Forward at staggerLeanRate (the knees bending with it),
/// and for the last 0.3 s of staggerTime leans against his motion; down, the writhe takes other values and he holds a
/// lying pose for the side he is on, and the balancer's hips and knees are let go. Never stops by itself.
///
/// The windmill and the writhe are run here (nmwm.rs, nmbw.rs), not as their own behaviours; under the RDR2 core's
/// limb requests their writes are onFire's (slot 0xe0), written in RDR2's priority order (writhe, then the lying
/// pose, then the windmill) so the same one wins among the three.
#[derive(Clone, Debug)]
pub(crate) struct OnFireRdr2 {
    pub(crate) p: FireParams,
    /// the attitude class (+0x84), time running (+0x88), the stumble's lean target (+0x8c)
    class: u8,
    t: f32,
    lean_target: f32,
    started: bool,
    /// bodyBalance started here and not yet stopped
    bb: bool,
    /// the windmill (its parameters, its state, started this step)
    wm: Option<(crate::nmwm::Params, crate::nmwm::Windmill, bool)>,
    bw: Option<crate::nmbw::Writhe>,
}

impl OnFireRdr2 {
    pub(crate) fn new(p: FireParams) -> OnFireRdr2 {
        OnFireRdr2 { p, class: 0, t: 0.0, lean_target: 0.0, started: false, bb: false, wm: None, bw: None }
    }

    /// Activate (§3): the children, the first lean target, bodyBalance reset and started with onFire's settings, and
    /// the balancer's leg stiffness 10 unless balancerCollisionsReaction holds them. (RDR2 also zeroes the balancer's
    /// ankleEquilibrium, its default here.)
    fn activate(&mut self, c: &mut Ctx) {
        self.children(c);
        let p = self.p;
        self.lean_target = c.ch.random() * (p.back + p.fwd) - p.back;
        let mut b = crate::nmbb::Params::default();
        (b.use_body_turn, b.use_head_look, b.rdr2.backwards_auto_turn, b.rdr2.backwards_arms) = (false, false, true, true);
        (b.spine_stiffness, b.spine_damping, b.elbow, b.shoulder, b.body_stiffness) = (10.0 * S, 0.9, 1.2, 0.8, 10.0 * S);
        c.out.bb_start = Some(b);
        self.bb = true;
        if c.ch.bcr.is_none_or(|r| r.0 == 0 || r.0 == 6) {
            c.out.dbm_leg_stiffness = Some(10.0 * S);
        }
    }

    fn stop_bb(&mut self, c: &mut Ctx) {
        if std::mem::take(&mut self.bb) {
            c.out.bb_stop = true;
        }
    }

    /// The children update (§3.1), at activation and every step.
    fn children(&mut self, c: &mut Ctx) {
        let p = self.p;
        if self.t > p.stagger_time || !c.upright {
            self.stop_bb(c);
        }
        if self.wm.is_none() && self.class == 0 {
            let w = crate::nmwm::Params { speed: [1.2; 2], elbow_min: [0.9; 2], radius: [[0.8 * L, 0.4 * L]; 2], phase_offset: 90.0,
                                          adaptive_mode: 1, ..Default::default() };
            self.wm = Some((w, crate::nmwm::Windmill::start(c, &w), true));
        }
        let w = self.bw.get_or_insert_with(|| {
            let mut w = crate::nmbw::Writhe::new(c.ch);
            (w.back_stiffness, w.leg_stiffness, w.leg_period, w.arm_damping, w.back_damping) = (14.0 * S, 10.0 * S, 0.7, 0.2, 0.5);
            (w.leg_damping, w.back_period, w.mask, w.back_amplitude, w.leg_amplitude, w.knee_amplitude) = (0.3, 1.2, "fb", 3.2, 1.1, 1.0);
            let r = &mut w.rdr2;
            (r.lean1_blend_factor, r.lean2_blend_factor, r.shoulder_lean1, r.shoulder_lean2, r.apply_stiffness) = (0.0, 0.0, 0.7, 0.4, true);
            w.activate(c.ch);
            w
        });
        [w.rdr2.blend_arms, w.rdr2.blend_back, w.rdr2.blend_legs] = p.pose;
        w.roll_over = p.roll_over;
        (w.rdr2.roll_torque_scale, w.rdr2.max_roll_over_time, w.rdr2.roll_over_radius) = (T * p.roll_torque_scale, p.max_roll_over_time, p.roll_over_radius);
        if self.class != 0 {
            self.wm = None;
            self.stop_bb(c);
        }
    }

    /// The attitude class of his COM frame turned on by his spin predictTime ahead (ours, as highFall's: RDR2's
    /// prediction 0x2a35fb8 wasn't re-read): rows a side, b up, c back.
    fn classify(&self, c: &Ctx) -> u8 {
        let q = Quat::from_scaled_axis(c.angvel * self.p.predict_time);
        let up = c.up;
        let (beta, alpha, gamma) = ((q * c.m.b).dot(up), -(q * c.m.a).dot(up), -(q * c.m.c).dot(up));
        let mut class = if gamma >= 0.25 { 3 } else { 2 };
        if alpha.abs() - gamma.abs() > 0.15 {
            class = if alpha < 0.0 { 4 } else { 5 };
        }
        let (b, a, g) = (beta.abs(), alpha.abs(), gamma.abs());
        if b > 0.8 || (b > g && b > a && g < 0.75 && a < 0.75 && b > 0.75) {
            class = if beta > 0.0 { 0 } else { 1 };
        }
        class
    }

    /// The standing lean (§4.1), while bodyBalance runs: the hip pitch (owner 5) wanders toward random targets in
    /// [-back, forward], or in the last 0.3 s of staggerTime leans against his motion; the knees bend with it (owner
    /// 3) except during a step. Our hip pitch is RDR2's negated.
    fn stand_lean(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.p;
        let Some(d) = c.dbm else { return };
        let x0 = -d.hip_pitch;
        if self.t > p.stagger_time - 0.3 {
            let s1 = c.angvel.dot(c.m.a).clamp(-3.0, 3.0);
            let s2 = c.vel.dot(c.m.c).clamp(-3.0, 3.0);
            let sp = spine_parts();
            let e = (c.b.part(sp[4]).m.d - c.b.part(sp[0]).m.d).normalize_or_zero();
            let lean = (-0.7 * e.dot(c.m.c) - 0.25 * s2 - 0.25 * s1).clamp(-0.7, 0.5);
            c.out.dbm_crouch.push(Crouch::Hip(5, -lean.clamp(-2.0, 2.0), NOW));
            // RDR2 then lets go of the balancer's COM lean channel (owner-less release); onFire never sets a lean here
        } else {
            let (step, to) = (dt * p.stagger_lean_rate * 60.0, self.lean_target);
            let x = if x0 < to { x0 + step } else { x0 - step };
            let reached = if x0 < to { x >= to } else { x <= to };
            let x = x.max(-p.back).min(p.fwd);
            if reached {
                self.lean_target = c.ch.random() * (p.fwd + p.back) - p.back;
            }
            c.out.dbm_crouch.push(Crouch::Hip(5, -x.clamp(-2.0, 2.0), NOW));
        }
        let k = if d.stepping != 0 { 0.0 } else { (-0.25 * x0.abs()).max(-0.4) };
        c.out.dbm_crouch.push(Crouch::Knee(3, k, NOW));
    }

    /// The lying poses (§4.2): spine (through the spine writer, a quarter on each joint), neck and head, the arms
    /// (shoulder, clavicle, elbow) and the legs (hip, knee) for classes 2..5.
    fn pose(&self, c: &mut Ctx) {
        let e = &mut c.ch.eff;
        let trunk = |e: &mut [crate::nmrs::Effector], s: f32, n: f32| {
            for &j in SPINE().iter() {
                e[j].desired = [(0.25 * s).clamp(-9.9, 9.9), 0.0, 0.0];
            }
            e[NECK()[0]].desired = [n, 0.0, 0.0];
            e[NECK()[1]].desired = [n, 0.0, 0.0];
        };
        let arm = |e: &mut [crate::nmrs::Effector], i: usize| {
            let l = limb(i);
            e[l.e[1]].desired = [1.5, 1.0, 0.5];
            e[l.e[0]].desired = [0.0, -1.0, -1.0];
            e[l.e[2]].desired[0] = 1.5;
        };
        let leg = |e: &mut [crate::nmrs::Effector], i: usize, bent: bool| {
            let l = limb(2 + i);
            e[l.e[1]].desired = [if bent { 1.0 } else { 0.0 }, 0.0, 0.0];
            e[l.e[2]].desired[0] = if bent { -1.0 } else { 0.0 };
        };
        match self.class {
            2 => {
                trunk(e, -2.0, -1.0);
                arm(e, 0);
                arm(e, 1);
            }
            3 => {
                leg(e, 0, true);
                leg(e, 1, true);
                trunk(e, 2.0, 1.0);
            }
            4 => {
                leg(e, 0, false);
                leg(e, 1, true);
                trunk(e, 0.0, 0.0);
                arm(e, 0);
            }
            5 => {
                leg(e, 0, true);
                leg(e, 1, false);
                trunk(e, 0.0, 0.0);
                arm(e, 1);
            }
            _ => {}
        }
    }

    /// Tick (§4).
    pub(crate) fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if !self.started {
            self.started = true;
            self.activate(c);
        }
        let p = self.p;
        self.class = self.classify(c);
        self.children(c);
        let bb = self.bb && c.dbm.is_some();
        if let Some(w) = self.bw.as_mut() {
            (w.elbow_amplitude, w.arm_period, w.arm_amplitude, w.rdr2.on_fire) = (0.5, 0.5, 2.3, true);
            (w.leg_period, w.leg_amplitude, w.arm_stiffness) = (0.7, 1.1, 13.0 * S);
            match self.class {
                0 => {
                    (w.elbow_amplitude, w.arm_period, w.arm_amplitude, w.rdr2.on_fire, w.arm_stiffness) = (0.0, 1.0, 4.0, false, 12.0 * S);
                    [w.rdr2.blend_arms, w.rdr2.blend_back, w.rdr2.blend_legs] = p.stand;
                }
                2 => {
                    (w.elbow_amplitude, w.arm_period, w.leg_period, w.leg_amplitude, w.arm_stiffness) = (1.0, 1.6, 1.3, 1.4, 16.0 * S);
                }
                _ => {}
            }
        }
        if self.class == 0 && bb {
            self.stand_lean(c, dt);
        }
        if self.class != 0 {
            c.out.dbm_crouch.extend([Crouch::ReleaseKnee(NOW), Crouch::ReleaseHip(NOW)]);
        }
        self.t += dt;
        // the children's writes, lowest priority first: bodyWrithe (0xe8), onFire's pose (0xe0), armsWindmill (0x58;
        // one started this step first ticks next step, as its slot comes before onFire's)
        if let Some(w) = self.bw.as_mut() {
            w.tick(c, dt);
        }
        self.pose(c);
        if let Some((wp, w, new)) = self.wm.as_mut() {
            if !std::mem::take(new) {
                w.tick(c, wp, dt);
            }
        }
    }

    /// Deactivate (§5): the balancer's knees and hips let go, the windmill and bodyBalance stopped, the writhe too.
    pub(crate) fn stop(&mut self, c: &mut Ctx) {
        c.out.dbm_crouch.extend([Crouch::ReleaseKnee(NOW), Crouch::ReleaseHip(NOW)]);
        self.wm = None;
        self.stop_bb(c);
        if let Some(mut w) = self.bw.take() {
            w.deactivate(c.ch);
        }
    }
}
