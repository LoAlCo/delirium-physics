use crate::fneg;
use crate::nmdbm::{sdot, slen};
use crate::Vx;
use crate::prelude::Vec3;
use crate::nmrs::{Character, NECK, SPINE};
use crate::nmshot::{limb, Ctx};
use crate::rigid::{nrm, q_scaled, qrot};
use crate::Fma;

#[allow(non_snake_case)]
#[inline]
fn SPINE3() -> usize { delirium_data::kv!(81 int) }
#[allow(non_snake_case)]
#[inline]
fn NECK_PART() -> usize { delirium_data::kv!(113 int) }

#[derive(Clone, Debug)]
pub struct RollUp {
    pub stiffness: f32,
    pub use_arm_to_slow_down: f32,
    pub arm_reach_amount: f32,
    pub leg_push: f32,
    pub asymmetrical_legs: f32,
    pub arm_reach_length: f32,
    pub hips_from_vel: bool,
    pub mask: &'static str,
    /// RDR2's parameters (used with the RDR2 core): the damping ratio and muscle stiffness everywhere, how long the
    /// roll speed stays under rollVelForSuccess before success, and the roll speed's measure
    pub rdr2: Rdr2Params,
    /// RDR2: the smoothed roll amount
    smooth: f32,
    t: f32,
    shoulder: f32,
    damping: f32,
    reach: [f32; 2],
    still: f32,
    roll: f32,
    asym: f32,
    rate: f32,
    phase: f32,
    pub active: bool,
}

impl Default for RollUp {
    fn default() -> RollUp {
        RollUp { stiffness: delirium_data::kv!(31), use_arm_to_slow_down: delirium_data::kv!(80), arm_reach_amount: delirium_data::kv!(179), leg_push: 0.0, asymmetrical_legs: 0.0,
                 arm_reach_length: delirium_data::kv!(119), hips_from_vel: false, mask: "fb", t: 0.0, shoulder: 0.0, damping: 0.0, reach: [0.0; 2], still: 0.0,
                 roll: 0.0, asym: 0.0, rate: 0.0, phase: 0.0, active: false, rdr2: Rdr2Params::default(), smooth: 0.0 }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Rdr2Params {
    pub damping: f32,
    pub muscle_stiffness: f32,
    pub no_roll_time_before_success: f32,
    pub roll_vel_for_success: f32,
    pub roll_vel_linear_contribution: f32,
    pub velocity_scale: f32,
    pub velocity_offset: f32,
}

impl Default for Rdr2Params {
    fn default() -> Rdr2Params {
        Rdr2Params { damping: 0.5, muscle_stiffness: 1.0, no_roll_time_before_success: 0.5, roll_vel_for_success: 0.2,
                     roll_vel_linear_contribution: 1.0, velocity_scale: 0.2, velocity_offset: 2.0 }
    }
}

/// RDR2: the arms reach for the ground only slower than this (m/s), and the head lines up with up below this roll speed.
const RDR2_REACH_SPEED: f32 = 5.0;
/// RDR2: below this speed (m/s) the arms settle at a softer stiffness.
const RDR2_SETTLED: f32 = 0.3;

impl RollUp {
    /// RDR2's leg asymmetry draw: (sign - (0.6 r - 0.3)) x asymmetricalLegs, negative for r of 0.5 or more.
    fn roll_asym_rdr2(&mut self, ch: &mut Character) {
        let r = ch.random();
        let sign = if 2.0 * r - 1.0 >= 0.0 { -1.0 } else { 1.0 };
        self.asym = (sign - (0.6 * r - 0.3)) * self.asymmetrical_legs;
    }

    fn roll_asym(&mut self, ch: &mut Character) {
        let x = 2.0 * ch.random() - 1.0;
        self.asym = x.signum() * self.asymmetrical_legs - self.asymmetrical_legs * x * delirium_data::kv!(24);
    }

    pub fn activate(&mut self, ch: &mut Character) {
        if self.active {
            return;
        }
        (self.t, self.active) = (0.0, true);
        let keep = ch.lock(self.mask);
        self.start(ch);
        ch.unlock(self.mask, keep);
    }

    fn start(&mut self, ch: &mut Character) {
        if ch.rdr2 {
            return self.start_rdr2(ch);
        }
        (self.roll, self.still, self.damping) = (0.0, 0.0, 0.5);
        for j in 0..ch.eff.len() {
            ch.eff[j].reset(&ch.joints[j]);
            ch.eff[j].desired = [0.0; 3];
        }
        for e in ch.eff.iter_mut() {
            e.muscle_stiffness *= delirium_data::kv!(18);
        }
        for i in 2..delirium_data::kv!(1 int) {
            ch.eff[limb(i).e[delirium_data::kv!(0 int)]].muscle_stiffness *= 2.0;
        }
        ch.set_stiffness(self.stiffness, self.damping, "fb", None);
        ch.set_stiffness(self.stiffness + 2.0, 1.0, "ua", None);
        ch.set_stiffness(self.stiffness + 2.0, 1.0, "uc", None);
        self.reach = [0.0; 2];
        self.roll_asym(ch);
        self.rate = delirium_data::kv!(528) + delirium_data::kv!(529) * ch.random();
        self.phase = delirium_data::kv!(528) * ch.random();
        self.shoulder = if ch.mode == 1 { ch.random() - 1.0 } else { 0.0 };
    }

    pub fn deactivate(&mut self) {
        if self.active {
            *self = RollUp::default();
        }
    }

    pub fn tick(&mut self, c: &mut Ctx, brace: bool, dt: f32) {
        self.t += dt;
        if !self.active {
            return;
        }
        let keep = c.ch.lock(self.mask);
        if c.ch.rdr2 {
            self.roll_rdr2(c, brace, dt);
        } else {
            self.roll(c, brace, dt);
        }
        c.ch.unlock(self.mask, keep);
    }

    fn roll(&mut self, c: &mut Ctx, brace: bool, dt: f32) {
        let (m, w, v, up) = (c.m, c.angvel, c.vel, c.up);
        let spin = (sdot(m.b, w, b"zyx") * delirium_data::kv!(187)).clamp(-1.0, 1.0);
        let speed = slen(w, b"zyx") + slen(v, b"yxz");
        let slow = ((self.use_arm_to_slow_down.abs() - sdot(up, m.c, b"zyx")) - 1.0).clamp(0.0, delirium_data::kv!(31));
        let back = -sdot(c.b.part(SPINE3()).v, m.c, b"zyx");
        let c2 = qrot(q_scaled(w * delirium_data::kv!(78)), m.c, false);
        let u = sdot(up, c2, b"zyx");
        let push = u * self.leg_push * -0.5;
        let (mut arm_l, mut arm_r) = (1.0, 1.0);
        if self.use_arm_to_slow_down * spin > 0.0 {
            arm_l = (-spin.abs()).fma(slow, 1.0);
        } else {
            arm_r = (-spin.abs()).fma(slow, 1.0);
        }
        let sp = speed * 0.5;
        let k = self.roll.fma(2.0, sp.clamp(0.0, 1.0)) * delirium_data::kv!(530);
        self.roll = k;
        if sp < delirium_data::kv!(60) {
            self.still += dt;
            if self.still > 0.5 {
                c.out.send(delirium_data::kv!(55 int), "Roll Up", 0.0);
            }
        }
        let slack = if brace { (slen(v, b"yxz").clamp(delirium_data::kv!(60), delirium_data::kv!(531)) - delirium_data::kv!(60)) * 0.5 } else { 1.0 };
        let s = (fneg(fneg(slack).fma(k, 1.0))).fma(delirium_data::kv!(64), self.stiffness).clamp(1.0, delirium_data::kv!(139));
        c.ch.set_stiffness(s, self.damping, "fb", None);
        if u > 0.0 && (u as f64) < delirium_data::kv!(78) && (slen(w, b"yxz") as f64) > delirium_data::kv!(532) {
            self.roll_asym(c.ch);
        }
        let hip = if self.hips_from_vel { (delirium_data::kv!(24) - back).clamp(0.0, 1.0) } else { delirium_data::kv!(486) };
        let a = self.asym;
        let ch = &mut *c.ch;
        let (l, r) = (limb(2), limb(delirium_data::kv!(0 int)));
        ch.eff[l.e[1]].desired[0] = ((hip - push) - delirium_data::kv!(187)).fma(k, a) + delirium_data::kv!(187);
        ch.eff[r.e[1]].desired[0] = ((hip - push) - delirium_data::kv!(187)).fma(k, -a) + delirium_data::kv!(187);
        let knee = (-push).fma(delirium_data::kv!(5), delirium_data::kv!(231));
        let sin = |x: f32| (x as f64).sin() as f32;
        let rt = self.rate * self.t;
        let (sl, sr) = (sin(self.rate.fma(self.t, self.phase)), sin(rt.fma(delirium_data::kv!(533), self.phase) + delirium_data::kv!(5)));
        ch.eff[l.e[2]].desired[0] = (fneg(sl).fma(delirium_data::kv!(18), fneg(knee)) + 0.5).fma(k, fneg(a)) - 0.5;
        ch.eff[r.e[2]].desired[0] = (fneg(sr).fma(delirium_data::kv!(18), fneg(knee)) + 0.5).fma(k, a) - 0.5;
        let spread = if ch.mode == 2 { delirium_data::kv!(78) } else { delirium_data::kv!(140) };
        let (s4, s3) = (sin(rt.fma(delirium_data::kv!(116), self.phase)), sin(rt.fma(delirium_data::kv!(24), self.phase)));
        ch.eff[l.e[1]].desired[1] = (-((s4 + 1.0) * spread)).fma(0.5, a.fma(-0.5, -((delirium_data::kv!(80) - arm_l) * delirium_data::kv!(24))));
        ch.eff[r.e[1]].desired[1] = (-((s3 + 1.0) * spread)).fma(0.5, a.fma(0.5, -((delirium_data::kv!(179) - arm_r) * delirium_data::kv!(24))));
        for j in (*SPINE()) {
            ch.eff[j].desired = [(k + 0.5) * delirium_data::kv!(17), 0.0, a * delirium_data::kv!(17)];
        }
        ch.eff[(*SPINE())[0]].desired[0] = k * delirium_data::kv!(140);
        ch.eff[(*NECK())[1]].desired[0] = k.fma(delirium_data::kv!(24), delirium_data::kv!(60));
        ch.eff[(*NECK())[0]].desired[0] = k.fma(delirium_data::kv!(24), delirium_data::kv!(60));
        for (i, arm) in [(0, arm_l), (1, arm_r)] {
            let e = &mut ch.eff[limb(i).e[1]];
            e.desired = [arm.fma(delirium_data::kv!(66), self.shoulder), -0.5, -1.0];
            ch.eff[limb(i).e[2]].desired[0] = k.fma(delirium_data::kv!(534), delirium_data::kv!(12));
        }
        ch.eff[l.e[delirium_data::kv!(0 int)]].desired[0] = -a - delirium_data::kv!(12);
        ch.eff[r.e[delirium_data::kv!(0 int)]].desired[0] = a - delirium_data::kv!(12);
        if sp < delirium_data::kv!(64) {
            let n = c.b.pmat(NECK_PART()).m;
            for j in (*NECK()) {
                ch.eff[j].desired[0] = fneg(sdot(n.c, up, b"xyz"));
                ch.eff[j].desired[1] = fneg(sdot(n.a, up, b"xyz"));
            }
        }
        for (i, side, mask) in [(0, 1.0, "ul"), (1, -1.0, "ur")] {
            let out;
            (out, self.reach[i]) = self.arm(c, i, side, self.reach[i]);
            let s = ((-out).fma(2.0, delirium_data::kv!(84)).fma(k - 1.0, out * delirium_data::kv!(5)) + self.stiffness).clamp(0.5, delirium_data::kv!(25));
            c.ch.set_stiffness(s, 1.0, mask, None);
        }
    }

    /// RDR2's activation: the rig's gains (opposeGravity off), then the body, and the arms and clavicles 2 stiffer.
    fn start_rdr2(&mut self, ch: &mut Character) {
        let p = self.rdr2;
        (self.still, self.smooth, self.damping) = (0.0, 1.0, p.damping);
        for j in 0..ch.eff.len() {
            ch.eff[j].reset(&ch.joints[j]);
        }
        let ms = Some(p.muscle_stiffness);
        ch.set_stiffness(self.stiffness, p.damping, "uk", ms);
        ch.set_stiffness(self.stiffness, p.damping, "lb", ms);
        ch.set_stiffness(self.stiffness + 2.0, p.damping, "ua", ms);
        ch.set_stiffness(self.stiffness + 2.0, 1.0, "uc", None);
        self.reach = [0.0; 2];
        self.roll_asym_rdr2(ch);
        // drawn as GTA IV's pedalling rate and phase, which RDR2 no longer reads
        self.rate = std::f32::consts::TAU + 2.0 * std::f32::consts::TAU * ch.random();
        self.phase = std::f32::consts::TAU * ch.random();
        self.shoulder = if ch.mode == 1 { ch.random() - 1.0 } else { 0.0 };
    }

    /// RDR2's tick: a roll speed from the linear and angular speeds, smoothed; legs, spine and arms from it, the spin's
    /// push and how far the spin is off the body's side axis.
    fn roll_rdr2(&mut self, c: &mut Ctx, brace: bool, dt: f32) {
        let p = self.rdr2;
        let (m, w, up) = (c.m, c.angvel, c.up);
        let (v, spin_speed) = (c.vel.length(), w.length());
        let roll = ((v * p.roll_vel_linear_contribution + spin_speed - p.velocity_offset) * p.velocity_scale).max(0.0);
        let spin = (w.dot(m.b) * 0.125).clamp(-1.0, 1.0);
        // 0 rolling about the body's side axis
        let off = 1.0 - if spin_speed > 1e-6 { (w / spin_speed).dot(m.b).abs() } else { m.b.x };
        let slow = (self.use_arm_to_slow_down.abs() - up.dot(m.c) - 1.0).clamp(0.0, 10.0);
        let (mut arm_l, mut arm_r) = (1.0, 1.0);
        if spin * self.use_arm_to_slow_down > 0.0 {
            arm_l = 1.0 - spin.abs() * slow;
        } else {
            arm_r = 1.0 - spin.abs() * slow;
        }
        let back = c.b.part(SPINE3()).v.dot(m.c);
        let u = up.dot(qrot(q_scaled(w * 0.1), m.c, false));
        let push = -0.5 * u * self.leg_push;
        self.smooth = 0.75 * self.smooth + 0.25 * roll.clamp(0.0, 1.0);
        let k = self.smooth.clamp(0.0, 1.0);
        if roll < p.roll_vel_for_success {
            self.still += dt;
        }
        if self.still > p.no_roll_time_before_success {
            c.out.send(delirium_data::kv!(55 int), "Roll Up", 0.0);
        }
        let slack = if brace { (v.clamp(0.2, 2.2) - 0.2) * 0.5 } else { 1.0 };
        let s = (self.stiffness - (1.0 - slack * k) * 5.0).clamp(1.0, 16.0);
        let ms = Some(p.muscle_stiffness);
        let ch = &mut *c.ch;
        ch.set_stiffness(s, p.damping, "fb", ms);
        let (l, r) = (limb(2), limb(3));
        for leg in [l, r] {
            ch.eff[leg.e[3]].set_stiffness(s, p.damping + 1.0, Some(2.0));
        }
        if u > 0.0 && u < 0.1 && spin_speed > 2.7 {
            self.roll_asym_rdr2(ch);
        }
        let a = self.asym;
        let (al, ar) = (if a >= 0.0 { a } else { 0.4 * a }, if a > 0.0 { -a } else { -0.4 * a });
        let hip = if self.hips_from_vel { (back + 0.3).clamp(0.0, 1.0) } else { 1.6 };
        let hips = hip + (0.3 - hip) * (push + 0.5) + 0.9 * off;
        ch.eff[l.e[1]].desired = [hips + al, -0.0, 0.0];
        ch.eff[r.e[1]].desired = [hips + ar, 0.0, 0.0];
        let knee = 1.05 - 0.9 * push + 1.2 * off;
        ch.eff[l.e[2]].desired[0] = -al - knee;
        ch.eff[r.e[2]].desired[0] = -ar - knee;
        let curl = 1.5 * push + 0.75 + off;
        for j in (*SPINE()) {
            ch.eff[j].desired = [(0.25 * curl).clamp(-9.9, 9.9), 0.0, 0.0];
        }
        ch.eff[(*SPINE())[0]].desired[0] = 0.35 * k;
        for j in (*NECK()) {
            ch.eff[j].desired[0] = 0.3 * k + 0.2;
        }
        for (i, arm) in [(0, arm_l), (1, arm_r)] {
            let a = limb(i);
            ch.eff[a.e[0]].desired = [0.0; 3];
            ch.eff[a.e[1]].desired = [1.57 * arm + self.shoulder, k - 0.5, 0.5 - 0.5 * k];
            ch.eff[a.e[2]].desired[0] = k + 1.0;
            ch.eff[a.e[3]].desired = [0.0; 3];
        }
        for leg in [l, r] {
            ch.eff[leg.e[3]].desired[0] = 0.4 * k - 0.4;
        }
        if roll < RDR2_REACH_SPEED {
            let n = c.b.pmat(NECK_PART()).m;
            let (d0, d2) = (n.a.dot(up), n.c.dot(up));
            for j in (*NECK()) {
                ch.eff[j].desired[0] = -d2;
                ch.eff[j].desired[1] = -d0.abs() * d0;
            }
        }
        for (i, side, mask) in [(0, 1.0, "ul"), (1, -1.0, "ur")] {
            let mut out = 0.0;
            if v < RDR2_REACH_SPEED {
                (out, self.reach[i]) = self.arm_rdr2(c, i, side, self.reach[i]);
            }
            let sa = (((4.0 - 2.0 * out) * (k - 1.0) + 3.0 * out) + self.stiffness).clamp(0.5, 20.0);
            c.ch.set_stiffness(sa, p.damping, mask, ms);
        }
        // nearly still (RDR2 measures against the ground's speed; ours stands still)
        if v < RDR2_SETTLED {
            let t = 1.0 - (RDR2_SETTLED - v) / 0.6;
            c.ch.set_stiffness(((self.stiffness - 4.0) * t).clamp(2.0, 5.0), p.damping, "ua", ms);
        }
        c.ch.set_stiffness(10.0, 1.0, "uw", Some(5.0));
    }

    /// RDR2's arm reach: as GTA IV's, but its lead is up x the spine's spin and the clavicle stays where it is.
    fn arm_rdr2(&self, c: &mut Ctx, i: usize, side: f32, reach: f32) -> (f32, f32) {
        let l = limb(i);
        let (m, up) = (c.m, c.up);
        let at = c.b.eff3(l.e[1]).m.d;
        let s3 = c.b.pmat(SPINE3());
        let rise = up.dot(s3.v);
        let mut out = if rise > 0.0 { (1.0 - 0.5 * rise).clamp(0.0, 1.0) } else { 1.0 };
        let dir = nrm(s3.v + (m.b - m.c) - up * 2.45);
        let dir = nrm(dir * 0.5 + at + up.vx(s3.w) * 0.0125 - at);
        let target = at + dir * self.arm_reach_length;
        let d = nrm(s3.m.b * side - s3.m.c * 2.0).dot(dir);
        let open = ((d - (1.0 - self.arm_reach_amount)) / 0.3).min(1.0);
        let reach = if open > reach { open } else { ((d - (0.7 - self.arm_reach_amount)) / 0.3).max(0.0).min(reach) };
        out *= reach;
        if out > 0.0 {
            let vel = c.b.part(l.p[0]).v;
            crate::nmdbm::limb_ik_w(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[3], target, 0.0, 1.0, side, 1.0, vel, None, out, false);
            c.ch.eff[l.e[3]].desired[1] = -0.8 * out;
        }
        (out, reach)
    }

    fn arm(&self, c: &mut Ctx, i: usize, side: f32, reach: f32) -> (f32, f32) {
        let l = limb(i);
        let (m, up) = (c.m, c.up);
        let at = c.b.eff3(l.e[1]).m.d;
        let s3 = c.b.pmat(SPINE3());
        let mut out = 1.0;
        let rise = sdot(up, s3.v, b"zyx");
        if rise > 0.0 {
            out = fneg(rise).fma(0.5, 1.0).clamp(0.0, 1.0);
        }
        let dir = nrm(up.mul_add(Vec3::splat(-delirium_data::kv!(535)), s3.v) + (m.b - m.c));
        let p = s3.w.vx(up).mul_add(Vec3::splat(delirium_data::kv!(536)), dir.mul_add(Vec3::splat(0.5), at));
        let dir = nrm(p - at);
        let target = dir.mul_add(Vec3::splat(self.arm_reach_length), at);
        let n = nrm(s3.m.c * -2.0 + s3.m.b * side);
        let d = sdot(n, dir, b"zyx");
        let hi = ((d - ((delirium_data::kv!(80) - self.arm_reach_amount) - delirium_data::kv!(24))) * delirium_data::kv!(537)).min(1.0);
        let lo = ((d - (delirium_data::kv!(74) - self.arm_reach_amount)) * delirium_data::kv!(537)).max(0.0);
        let reach = reach.max(hi).min(lo);
        out *= reach;
        if out > 0.0 {
            let vel = c.b.part(l.p[0]).v;
            crate::nmdbm::limb_ik_w(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[delirium_data::kv!(0 int)], target, 0.0, 1.0, side, 1.0, vel, None, out, false);
            crate::nmcf::clav_follow(c.ch, l.e[0], l.e[1]);
            c.ch.eff[l.e[delirium_data::kv!(0 int)]].desired[1] = out * -delirium_data::kv!(20);
        }
        (out, reach)
    }
}

