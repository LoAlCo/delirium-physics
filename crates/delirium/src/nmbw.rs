use crate::fneg;
use bevy_math::Vec3;
use std::f32::consts::{PI, TAU};

use crate::nmfl::noise01;
use crate::nmrs::{Character, NECK, SPINE};
use crate::nmshot::{limb, Ctx};
use crate::Fma;

#[derive(Clone, Debug)]
pub struct Writhe {
    pub arm_stiffness: f32,
    pub back_stiffness: f32,
    pub leg_stiffness: f32,
    pub arm_damping: f32,
    pub back_damping: f32,
    pub leg_damping: f32,
    pub arm_period: f32,
    pub back_period: f32,
    pub leg_period: f32,
    pub arm_amplitude: f32,
    pub back_amplitude: f32,
    pub leg_amplitude: f32,
    pub elbow_amplitude: f32,
    pub knee_amplitude: f32,
    pub roll_over: bool,
    pub mask: &'static str,
    pub(crate) t: f32,
    pub(crate) t_noise: f32,
    pub(crate) noise_at: f32,
    pub(crate) roll_rate: f32,
    pub(crate) roll_force: f32,
    pub(crate) roll_timer: f32,
    /// RDR2's further parameters (read with the RDR2 core)
    pub rdr2: Rdr2Params,
    rd: Rdr2State,
    pub active: bool,
}

/// RDR2's bodyWrithe parameters that GTA IV's hasn't, with RDR2's defaults.
#[derive(Clone, Copy, Debug)]
pub struct Rdr2Params {
    /// request weights of the arms', back's and legs' targets
    pub blend_arms: f32,
    pub blend_back: f32,
    pub blend_legs: f32,
    pub apply_stiffness: bool,
    pub on_fire: bool,
    pub shoulder_lean1: f32,
    pub shoulder_lean2: f32,
    pub lean1_blend_factor: f32,
    pub lean2_blend_factor: f32,
    pub roll_torque_scale: f32,
    pub max_roll_over_time: f32,
    pub roll_over_radius: f32,
}

impl Default for Rdr2Params {
    fn default() -> Rdr2Params {
        Rdr2Params { blend_arms: 1.0, blend_back: 1.0, blend_legs: 1.0, apply_stiffness: true, on_fire: false, shoulder_lean1: 0.7,
                     shoulder_lean2: 0.4, lean1_blend_factor: 0.0, lean2_blend_factor: 0.0, roll_torque_scale: 150.0, max_roll_over_time: 8.0,
                     roll_over_radius: 2.0 }
    }
}

/// RDR2's writhe state: its two noise seeds, where the roll began, its sense and timers.
#[derive(Clone, Copy, Debug)]
struct Rdr2State {
    seeds: (f32, f32),
    start: Vec3,
    dir: f32,
    flip: f32,
    roll_t: f32,
    back_down: bool,
    started: bool,
}

impl Default for Rdr2State {
    fn default() -> Rdr2State {
        Rdr2State { seeds: (123.0, 321.0), start: Vec3::ZERO, dir: 1.0, flip: 1.5, roll_t: 0.0, back_down: false, started: false }
    }
}

impl Writhe {
    pub fn new(ch: &mut Character) -> Writhe {
        Writhe { arm_stiffness: delirium_data::kv!(195), back_stiffness: delirium_data::kv!(195), leg_stiffness: delirium_data::kv!(195), arm_damping: 0.5, back_damping: 0.5,
                 leg_damping: 0.5, arm_period: 1.0, back_period: 1.0, leg_period: 1.0, arm_amplitude: 1.0, back_amplitude: 1.0,
                 leg_amplitude: 1.0, elbow_amplitude: 1.0, knee_amplitude: 1.0, roll_over: false, mask: "fb", t: 0.0, t_noise: 0.0,
                 noise_at: delirium_data::kv!(238) * ch.random(), roll_rate: delirium_data::kv!(12), roll_force: 1.0, roll_timer: delirium_data::kv!(12), rdr2: Rdr2Params::default(), rd: Rdr2State::default(), active: false }
    }

    fn stiffness(&self, ch: &mut Character) {
        ch.set_stiffness(self.arm_stiffness, self.arm_damping, "ua", None);
        ch.set_stiffness(self.back_stiffness, self.back_damping, "uk", None);
        ch.set_stiffness(self.leg_stiffness, self.leg_damping, "lb", None);
    }

    pub fn activate(&mut self, ch: &mut Character) {
        if self.active {
            return;
        }
        (self.t, self.active) = (0.0, true);
        if ch.rdr2 {
            return;
        }
        let keep = ch.lock(self.mask);
        self.start(ch);
        ch.unlock(self.mask, keep);
    }

    /// RDR2's start (with the RDR2 core, from the tick, which knows where he is): three random noise draws, and with
    /// applyStiffness the rig's gains and the three stiffnesses (the back's muscle 0.75); the targets are kept.
    fn start_rdr2(&mut self, c: &mut Ctx) {
        let ch = &mut *c.ch;
        self.rd = Rdr2State { start: c.com, started: true, ..Default::default() };
        self.t_noise = 4000.0 * ch.random();
        self.rd.seeds = (4000.0 * ch.random(), 4000.0 * ch.random());
        if let Ok(v) = std::env::var("NMX_WR") { let x: Vec<f32> = v.split(' ').filter_map(|s| s.parse().ok()).collect(); if x.len() == 3 { self.t_noise = x[0]; self.rd.seeds = (x[1], x[2]); } } // NMXHOOK
        if self.rdr2.apply_stiffness {
            for j in Character::mask(self.mask) {
                ch.eff[j].reset(&ch.joints[j]);
            }
            // (the back's muscle 0.75 in the pooled request, priority -10, doesn't reach the joint: recorded on all three shots,
            // the spine's muscle on that tick is the rig's 1.5, under the writhe's strengths. Why isn't read, [P])
            self.stiffness_rdr2(ch, None);
        }
    }

    /// RDR2: arms, legs and back each within the mask; the back's muscle stiffness when given.
    fn stiffness_rdr2(&self, ch: &mut Character, back_ms: Option<f32>) {
        let mask = Character::mask(self.mask);
        for (m, s, d, ms) in [("ua", self.arm_stiffness, self.arm_damping, None), ("lb", self.leg_stiffness, self.leg_damping, None),
                              ("uk", self.back_stiffness, self.back_damping, back_ms)] {
            for j in Character::mask(m).into_iter().filter(|j| mask.contains(j)) {
                ch.eff[j].set_stiffness(s, d, ms);
            }
        }
    }

    fn start(&mut self, ch: &mut Character) {
        self.t_noise = delirium_data::kv!(238) * ch.random();
        for j in Character::mask("fb") {
            ch.eff[j].reset(&ch.joints[j]);
            ch.eff[j].desired = [0.0; 3];
        }
        for j in (*SPINE()) {
            ch.eff[j].muscle_stiffness = delirium_data::kv!(18);
        }
        self.stiffness(ch);
    }

    pub fn deactivate(&mut self, ch: &mut Character) {
        if self.active {
            *self = Writhe::new(ch);
        }
    }

    pub fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if !self.active {
            return;
        }
        if c.ch.rdr2 {
            return self.tick_rdr2(c, dt);
        }
        let keep = c.ch.lock(self.mask);
        self.stiffness(c.ch);
        self.writhe(c, dt);
        c.ch.unlock(self.mask, keep);
        self.t_noise = dt.fma(delirium_data::kv!(74), self.t_noise);
        self.t += (noise01(self.t_noise, self.noise_at, delirium_data::kv!(139)) + delirium_data::kv!(18)) * dt;
    }

    fn writhe(&mut self, c: &mut Ctx, dt: f32) {
        let ch = &mut *c.ch;
        let s = self.t * delirium_data::kv!(24);
        let ph = |n: f32| n.fma(TAU, -PI);
        let (sin, cos) = (|x: f32| (x as f64).sin() as f32, |x: f32| (x as f64).cos() as f32);
        let agent = ch.agent as f32;
        let p16 = ph(noise01(s, self.noise_at, agent));
        let p15 = ph(noise01(s, s, agent));
        let p24 = ph(noise01(s, self.noise_at, s));
        let p23 = ph(noise01(s, s, self.noise_at));
        let p22 = ph(noise01(s, agent, s));
        let a = self.t / self.arm_period * TAU;
        let amp = noise01(self.noise_at, a, a) * self.arm_amplitude;
        let cl = |x: f32| x.clamp(-1.0, 1.0);
        let arms = [(0, [cl(sin(a + p16) * amp * delirium_data::kv!(20)), cl(cos(a + p15) * amp * 2.0), cl(fneg(p24))], sin(a + p22)),
                    (1, [cl(sin(a + p24) * amp * delirium_data::kv!(20)), cl(cos(a + p23) * amp * 2.0), cl(fneg(p22))], cos(a + p23))];
        for (i, x, elbow) in arms {
            let l = limb(i);
            for k in 0..delirium_data::kv!(0 int) {
                ch.set_relative(l.e[0], k, x[k] * delirium_data::kv!(17));
            }
            ch.eff[l.e[0]].blend_to_zero(0.5);
            for k in 0..delirium_data::kv!(0 int) {
                ch.set_relative(l.e[1], k, x[k]);
            }
            ch.set_relative(l.e[2], 0, cl(elbow * amp * self.elbow_amplitude));
        }
        let (sn, cs) = (sin(a + p16), cos(a + p23));
        for (j, k) in [((*NECK())[0], -1.0), ((*NECK())[1], 1.0)] {
            ch.set_relative(j, 0, sn * k);
            ch.set_relative(j, 1, cs * k);
            ch.set_relative(j, 2, sn * k);
        }
        let b = self.t / self.back_period * TAU;
        let back = [self.back_amplitude * sin((b + p23) * delirium_data::kv!(20)) * delirium_data::kv!(17), p24 * delirium_data::kv!(9), self.back_amplitude * sin(b + p16) * delirium_data::kv!(239 f32)];
        for j in (*SPINE()) {
            ch.eff[j].desired = back.map(|x| x * delirium_data::kv!(17));
        }
        let g = self.t / self.leg_period * delirium_data::kv!(240);
        let la = self.leg_amplitude;
        let (l, r) = (limb(2), limb(delirium_data::kv!(0 int)));
        ch.eff[l.e[1]].desired[0] = sin(g + p22 + 2.0) * la * 0.5;
        ch.eff[l.e[2]].desired[0] = (self.knee_amplitude * cos(g + p15)).fma(delirium_data::kv!(28), -0.5);
        ch.eff[l.e[1]].desired[1] = (-(sin(g * delirium_data::kv!(119)) * la)).fma(delirium_data::kv!(60), -delirium_data::kv!(78));
        ch.eff[l.e[1]].desired[2] = sin(g.fma(delirium_data::kv!(241), 1.0)) * la * delirium_data::kv!(24);
        ch.eff[r.e[1]].desired[0] = cos(g + p23) * la * -0.5;
        ch.eff[r.e[2]].desired[0] = (-(self.knee_amplitude * sin(g + p24))).fma(delirium_data::kv!(28), -0.5);
        ch.eff[r.e[1]].desired[1] = (-(cos(g * delirium_data::kv!(119)) * la)).fma(delirium_data::kv!(60), -delirium_data::kv!(78));
        ch.eff[r.e[1]].desired[2] = sin(g.fma(delirium_data::kv!(242), -1.0)) * la * delirium_data::kv!(24);
        if !self.roll_over {
            return;
        }
        let up = crate::nmdbm::vnorm(c.b.joint_pos((*SPINE())[delirium_data::kv!(0 int)]) - c.b.joint_pos((*SPINE())[0]));
        let k = (sin(self.roll_rate * self.t) + 1.0) * 0.5 - delirium_data::kv!(74);
        let f = up * (k.clamp(0.0, 1.0) * self.roll_force * delirium_data::kv!(243));
        let down = c.b.doll.collided(&[r.p[delirium_data::kv!(0 int)], l.p[delirium_data::kv!(0 int)], delirium_data::kv!(104 int), 0]);
        if crate::nmdbm::slen(c.angvel, b"yxz") < delirium_data::kv!(25) && down {
            for k in [delirium_data::kv!(81 int), delirium_data::kv!(104 int), delirium_data::kv!(23 int), delirium_data::kv!(22 int)] {
                c.out.torque(k, f);
            }
        }
        if self.roll_timer < 0.0 {
            self.roll_force = -self.roll_force;
            self.roll_timer = c.ch.random() * delirium_data::kv!(33) + 0.5;
        } else {
            self.roll_timer -= dt;
        }
    }

    /// RDR2's tick: the stiffnesses again (no muscle), the writhe, then the clock: noise time at 0.7/s and the writhe's
    /// time at 0.75 to 1.75 of real time, wandering by noise.
    fn tick_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        if !self.rd.started {
            self.start_rdr2(c);
        }
        let keep = c.ch.lock(self.mask);
        if self.rdr2.apply_stiffness {
            self.stiffness_rdr2(c.ch, None);
        }
        self.writhe_rdr2(c, dt);
        c.ch.unlock(self.mask, keep);
        self.t_noise += 0.7 * dt;
        self.t += (noise01(self.t_noise, self.rd.seeds.0, 16.0) + 0.75) * dt;
    }

    /// RDR2's writhe: five noise phases from its two seeds; arms and neck as fractions of each joint's range, back
    /// and legs in radians, each part's targets a request weighted by blendArms / blendBack / blendLegs; the roll over.
    fn writhe_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.rdr2;
        let (a_seed, b_seed) = self.rd.seeds;
        let s = 0.3 * self.t;
        let ph = |n: f32| TAU * n - PI;
        let (p1, p2, p3) = (ph(noise01(s, a_seed, b_seed)), ph(noise01(a_seed, s, b_seed)), ph(noise01(s, b_seed, a_seed)));
        let (p4, p5) = (ph(noise01(b_seed, s, a_seed)), ph(noise01(a_seed, b_seed, s)));
        let a = TAU * self.t / self.arm_period;
        let amp = noise01(a_seed, a, a) * self.arm_amplitude;
        let cl = |x: f32| x.clamp(-1.0, 1.0);
        let ch = &mut *c.ch;
        // the share x of the joint's range as an angle
        let rel = |ch: &Character, j: usize, k: usize, x: f32| {
            let l = ch.limits(j);
            ((l[2 * k + 1] - l[2 * k]) * 0.5).fma(x, (l[2 * k + 1] + l[2 * k]) * 0.5)
        };
        // a request of weight w over what is there
        let put = |ch: &mut Character, j: usize, x: [f32; 3], n: usize, w: f32| {
            for k in 0..n {
                let d = &mut ch.eff[j].desired[k];
                *d += w.clamp(0.0, 1.0) * (x[k] - *d);
            }
        };
        let arms = [(0, [cl(0.8 * amp * (a + p1).sin()), cl(2.0 * amp * (a + p2).cos()), cl(-p3)], cl(amp * self.elbow_amplitude * (a + p5).sin())),
                    (1, [cl(0.8 * amp * (a + p3).sin()), cl(2.0 * amp * (a + p4).cos()), cl(-p5)], cl(amp * self.elbow_amplitude * (a + p4).cos()))];
        let wa = p.blend_arms;
        for (i, x, e) in arms {
            let l = limb(i);
            let clav = [0, 1, 2].map(|k| rel(ch, l.e[0], k, 0.25 * x[k]));
            // the clavicle, then the whole arm blended half way to its zero pose, in the one request
            let mut req: Vec<crate::nmrs::Effector> = l.e.iter().map(|&j| ch.eff[j]).collect();
            req[0].desired = clav;
            for r in req.iter_mut() {
                r.blend_to_zero(0.5);
            }
            put(ch, l.e[0], req[0].desired, 3, wa);
            put(ch, l.e[3], req[3].desired, 3, wa);
            let mut sh = [rel(ch, l.e[1], 0, x[0]), rel(ch, l.e[1], 1, x[1]), rel(ch, l.e[1], 2, if p.on_fire { 0.0 } else { x[2] })];
            if p.on_fire {
                if p.lean1_blend_factor > 0.01 {
                    let k = p.lean1_blend_factor.clamp(0.0, 1.0);
                    sh[0] = (1.0 - k) * sh[0] + k * p.shoulder_lean1;
                }
                if p.lean2_blend_factor > 0.01 {
                    let k = p.lean2_blend_factor.clamp(0.0, 1.0);
                    sh[1] = (1.0 - k) * sh[1] + k * p.shoulder_lean2;
                }
            }
            put(ch, l.e[1], sh, 3, wa);
            put(ch, l.e[2], [rel(ch, l.e[2], 0, e), 0.0, 0.0], 1, wa);
        }
        let (s1, c4) = ((a + p1).sin(), (a + p4).cos());
        let wb = p.blend_back;
        for (j, k) in [((*NECK())[0], -1.0), ((*NECK())[1], 1.0)] {
            let x = [rel(ch, j, 0, k * s1), rel(ch, j, 1, k * c4), rel(ch, j, 2, k * s1)];
            put(ch, j, x, 3, wb);
        }
        let b = TAU * self.t / self.back_period;
        let back = [0.25 * self.back_amplitude * (0.8 * (b + p4)).sin(), 0.15 * p3, 0.405 * self.back_amplitude * (b + p1).sin()];
        for j in (*SPINE()) {
            put(ch, j, back.map(|x| (0.25 * x).clamp(-9.9, 9.9)), 3, wb);
        }
        let g = 2.4 * PI * self.t / self.leg_period;
        let (la, ka, wl) = (self.leg_amplitude, self.knee_amplitude, p.blend_legs);
        let (l, r) = (limb(2), limb(3));
        put(ch, l.e[1], [0.5 * la * (g + p5).sin(), -0.1 - 0.2 * la * (0.6 * g).sin(), 0.3 * la * (0.72 * g + 1.0).sin()], 3, wl);
        put(ch, l.e[2], [0.9 * ka * (g + p2).cos() - 0.5, 0.0, 0.0], 1, wl);
        put(ch, r.e[1], [-0.5 * la * (g + p4).cos(), -0.1 - 0.2 * la * (0.6 * g).cos(), 0.3 * la * (0.52 * g - 1.0).sin()], 3, wl);
        put(ch, r.e[2], [-0.5 - 0.9 * ka * (g + p3).sin(), 0.0, 0.0], 1, wl);
        if self.roll_over {
            self.roll_rdr2(c, dt);
        }
    }

    /// RDR2's roll over: with the back on the ground, a torque about the spine's axis on the four spine parts above the
    /// pelvis, its sense the spin's; on fire it fades with the time and distance since the back first touched.
    fn roll_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.rdr2;
        let up = c.up;
        let sp = [0, delirium_data::kv!(22 int), delirium_data::kv!(23 int), delirium_data::kv!(104 int), delirium_data::kv!(81 int)];
        let back: Vec<usize> = sp.iter().copied().chain([delirium_data::kv!(113 int), delirium_data::kv!(55 int)]).collect();
        if p.on_fire {
            self.rd.back_down |= c.b.doll.collided(&back);
            if !self.rd.back_down {
                (self.rd.roll_t, self.rd.start) = (0.0, c.com);
            }
        }
        let down = c.b.doll.collided(&sp);
        let unit = |v: Vec3| v.try_normalize().unwrap_or(Vec3::ZERO);
        let u = unit(c.b.joint_pos((*SPINE())[3]) - c.b.joint_pos((*SPINE())[0]));
        let mut w8 = c.b.part(sp[4]).w.dot(u);
        if p.on_fire {
            let flat = |v: Vec3| v - up * up.dot(v);
            let (k, h) = (unit(up.cross(flat(c.m.b))), unit(flat(c.vel)));
            if k != Vec3::ZERO && h != Vec3::ZERO {
                w8 = k.dot(h);
            }
        }
        let sign = if w8 > 0.0 { 1.0 } else if w8 < 0.0 { -1.0 } else { 0.0 };
        self.rd.dir = if p.on_fire && self.rd.back_down { self.rd.dir } else { sign };
        let (w7, w3) = (c.angvel.dot(u), c.b.part(0).w.dot(u));
        if down && w8.abs() < 10.0 && w3.abs() < 10.0 && w7.abs() < 10.0 {
            let f = if p.on_fire {
                let dist = (self.rd.start - c.com).length();
                let (r, tm) = (p.roll_over_radius.max(0.01), p.max_roll_over_time.max(0.01));
                self.rd.roll_t = self.rd.roll_t.min(tm);
                (1.0 - self.rd.roll_t / tm) * (1.0 - dist.min(r) / r) * self.rd.dir * p.roll_torque_scale
            } else {
                (0.5 * (1.5 * self.t).sin() - 0.2).clamp(0.0, 1.0) * self.rd.dir * p.roll_torque_scale
            };
            for k in [sp[4], sp[3], sp[2], sp[1]] {
                c.out.torque(k, u * f);
            }
            // limp where an arm or leg rests on something, so the roll carries it over
            if !p.on_fire && p.roll_torque_scale > 0.001 {
                for (i, m) in [(0, "ul"), (1, "ur"), (2, "ll"), (3, "lr")] {
                    if c.b.doll.collided(&limb(i).p[if i < 2 { 0 } else { 1 }..]) {
                        c.ch.set_stiffness(7.0, self.arm_damping, m, None);
                    }
                }
            }
        }
        if p.on_fire {
            self.rd.roll_t += dt;
        } else if self.rd.flip < 0.0 {
            self.rd.dir = -self.rd.dir;
            self.rd.flip = 0.5 + 2.5 * c.ch.random();
        } else {
            self.rd.flip -= dt;
        }
    }
}

