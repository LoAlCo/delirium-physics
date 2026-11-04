use bevy_math::{Quat, Vec3};
use crate::Vd;
use crate::Vx;

use crate::nmdbm::M34;
use crate::nmfl::noise01;
use crate::Fma;
use crate::nmshot::{limb, Ctx};
use crate::nmrs::{ANKLE, ELBOW, HIP, KNEE, SHOULDER, SPINE};

/// RDR2's further pedalLegs parameters (read with the RDR2 core).
#[derive(Clone, Copy, Debug)]
pub struct Rdr2Params {
    pub drag_reduction: f32,
    pub centre_sideways: f32,
    pub ellipse: f32,
    pub spread: f32,
    pub hula: bool,
}

impl Default for Rdr2Params {
    fn default() -> Rdr2Params {
        Rdr2Params { drag_reduction: 0.25, centre_sideways: 0.0, ellipse: 1.0, spread: 0.0, hula: false }
    }
}

/// RDR2's pedalLegs message defaults (a start under the RDR2 core reloads them before the message's values).
pub fn rdr2_message() -> crate::nm::PedalLegs {
    crate::nm::PedalLegs { leg_stiffness: 10.0, radius: 0.25, angular_speed: 10.0, pedal_offset: 0.0, speed_asymmetry: 8.0, back_pedal: false,
                           adaptive: false, drag_multiplier: 0.3, left: true, right: true, radius_variance: 0.4, leg_angle_variance: 0.5,
                           centre_forwards: 0.0, centre_up: 0.0, rdr2: Rdr2Params::default(), ..crate::nm::PedalLegs::default() }
}

#[derive(Clone, Debug)]
pub struct Pedal {
    pub radius: f32,
    pub radius_variance: f32,
    pub random_seed: u32,
    pub pedal_offset: f32,
    pub leg_stiffness: f32,
    pub angular_speed: f32,
    pub speed_asymmetry: f32,
    pub leg_angle_variance: f32,
    pub drag_multiplier: f32,
    pub centre_forwards: f32,
    pub centre_up: f32,
    pub left: bool,
    pub right: bool,
    pub back_pedal: bool,
    pub adaptive: bool,
    t: f32,
    t2: f32,
    damping: f32,
    speed: [f32; 2],
    axis: [Vec3; 2],
    at: [Vec3; 2],
    centre: [Vec3; 2],
    pub active: bool,
    pub rdr2: Rdr2Params,
    /// RDR2 core: started by a message under the core, so RDR2's pedalLegs (braceForImpact's GTA IV pedalling stays
    /// GTA IV's)
    pub r2: bool,
    /// RDR2: each leg's phase, the leg length, the hula latch and the one-tick reverse flags
    theta: [f32; 2],
    leg: f32,
    hula: bool,
    reverse: [bool; 2],
}

impl Default for Pedal {
    fn default() -> Pedal {
        Pedal { radius: delirium_data::kv!(17), radius_variance: delirium_data::kv!(116), random_seed: delirium_data::kv!(197 int), pedal_offset: delirium_data::kv!(9), leg_stiffness: delirium_data::kv!(31), angular_speed: delirium_data::kv!(31),
                speed_asymmetry: delirium_data::kv!(65), leg_angle_variance: 0.5, drag_multiplier: delirium_data::kv!(24), centre_forwards: 0.0, centre_up: 0.0,
                left: true, right: true, back_pedal: false, adaptive: false, t: 0.0, t2: 0.0, damping: 0.5, speed: [0.0; 2],
                axis: [Vec3::ZERO; 2], at: [Vec3::ZERO; 2], centre: [Vec3::ZERO; 2], active: false,
                rdr2: Rdr2Params::default(), r2: false, theta: [0.0; 2], leg: 0.0, hula: false, reverse: [false; 2] }
    }
}

fn apply(m: &M34, v: Vec3) -> Vec3 {
    crate::vm4(m.a, m.b, m.c, Vec3::ZERO, v) + m.d
}

impl Pedal {
    /// A message's values.
    pub fn take(&mut self, m: &crate::nm::PedalLegs) {
        (self.leg_stiffness, self.radius, self.angular_speed) = (m.leg_stiffness, m.radius, m.angular_speed);
        (self.pedal_offset, self.speed_asymmetry, self.back_pedal) = (m.pedal_offset, m.speed_asymmetry, m.back_pedal);
        (self.adaptive, self.drag_multiplier) = (m.adaptive, m.drag_multiplier);
        (self.left, self.right, self.radius_variance, self.random_seed) = (m.left, m.right, m.radius_variance, m.random_seed);
        (self.leg_angle_variance, self.centre_forwards, self.centre_up) = (m.leg_angle_variance, m.centre_forwards, m.centre_up);
        self.rdr2 = m.rdr2;
    }

    /// The running values, for a message that changes some of them.
    pub fn held(&self, m: &mut crate::nm::PedalLegs) {
        (m.leg_stiffness, m.radius, m.angular_speed) = (self.leg_stiffness, self.radius, self.angular_speed);
        (m.pedal_offset, m.speed_asymmetry, m.back_pedal) = (self.pedal_offset, self.speed_asymmetry, self.back_pedal);
        (m.adaptive, m.drag_multiplier) = (self.adaptive, self.drag_multiplier);
        (m.left, m.right, m.radius_variance, m.random_seed) = (self.left, self.right, self.radius_variance, self.random_seed);
        (m.leg_angle_variance, m.centre_forwards, m.centre_up) = (self.leg_angle_variance, self.centre_forwards, self.centre_up);
        m.rdr2 = self.rdr2;
    }

    fn stiffness(&self, c: &mut Ctx) {
        for i in 2..delirium_data::kv!(1 int) {
            let l = limb(i);
            c.ch.eff[l.e[1]].set_stiffness(self.leg_stiffness, self.damping, None);
            c.ch.eff[l.e[2]].set_stiffness(self.leg_stiffness, self.damping, None);
            c.ch.eff[l.e[delirium_data::kv!(0 int)]].set_stiffness(self.leg_stiffness * delirium_data::kv!(12), self.damping, None);
        }
    }

    fn speeds(&mut self, w: f32, agent: f32) {
        let a = (noise01(self.t, agent, self.t2) - 0.5) * 2.0;
        let b = (noise01(self.t2, agent, self.t) - 0.5) * 2.0;
        self.speed = [a, b].map(|n| match n.fma(self.speed_asymmetry, w) {
            s if s < 0.0 => delirium_data::kv!(60),
            s => s,
        });
    }

    fn centres(&mut self, c: &Ctx) {
        let r = self.radius + (noise01(self.t + delirium_data::kv!(31), self.t2, c.ch.agent as f32) - 0.5) * self.radius_variance - delirium_data::kv!(518);
        let p = c.b.part(0).m;
        for (i, k) in [(0, -self.pedal_offset - self.centre_up), (1, self.pedal_offset - self.centre_up)] {
            let m = c.b.eff3(limb(i + 2).e[1]).m;
            let v = p.c.mul_add(Vec3::splat(-self.centre_forwards), p.a * r);
            self.centre[i] = c.up.mul_add(Vec3::splat(k), Vec3::new(m.a.vd(v), m.b.vd(v), m.c.vd(v)));
        }
    }

    pub fn activate(&mut self, c: &mut Ctx) {
        if self.active {
            return;
        }
        self.active = true;
        if self.r2 {
            self.activate_rdr2(c);
            return;
        }
        self.t = delirium_data::kv!(215) * c.ch.random();
        self.t2 = delirium_data::kv!(215) * c.ch.random();
        self.radius = self.radius.clamp(0.0, delirium_data::kv!(10));
        self.centres(c);
        self.axis = [Vec3::NEG_Y; 2];
        self.speeds(self.angular_speed, c.ch.agent as f32);
        let n = -c.b.part(0).m.c;
        let l = limb(delirium_data::kv!(0 int));
        let foot = c.b.part(l.p[delirium_data::kv!(0 int)]).m.d - apply(&c.b.eff3(l.e[1]).m, self.centre[1]);
        let r = if foot.vd(n) >= 0.0 { self.radius } else { -self.radius };
        self.at = [Vec3::new(r, 0.0, 0.0), Vec3::new(-r, 0.0, 0.0)];
        self.stiffness(c);
    }

    pub fn deactivate(&mut self) {
        if self.active {
            *self = Pedal::default();
        }
    }

    pub fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if !self.active {
            return;
        }
        if self.r2 {
            self.tick_rdr2(c, dt);
            return;
        }
        self.t += dt;
        self.stiffness(c);
        self.centres(c);
        let (mut back, mut legs) = (self.back_pedal, [self.left, self.right]);
        let mut w = self.angular_speed;
        if self.adaptive {
            let s = c.b.part(delirium_data::kv!(22 int));
            let v = s.v;
            w = (v.x.fma(v.x, v.y.fma(v.y, v.z * v.z)).sqrt() / self.radius * self.drag_multiplier).clamp(0.0, delirium_data::kv!(195));
            if crate::nmdbm::sdot(crate::rigid::nrm(v).vx(c.up), s.m.b, b"zyx") > 0.0 {
                back = !self.back_pedal;
            }
            if crate::nmdbm::sdot(c.up, s.m.a, b"zyx") < -delirium_data::kv!(116) {
                legs = [false; 2];
            }
        }
        self.speeds(w, c.ch.agent as f32);
        for i in 0..2 {
            if !legs[i] {
                continue;
            }
            let yaw = (noise01(c.ch.agent as f32, self.t * 0.5, self.t2) - 0.5) * self.leg_angle_variance;
            let h = (self.speed[i] * dt * 0.5) as f64;
            let (s, co) = (h.sin() as f32, h.cos() as f32);
            let q = Quat::from_xyzw(self.axis[i].x * s, self.axis[i].y * s, self.axis[i].z * s, co);
            self.at[i] = crate::rigid::qrot(q, self.at[i], back);
            let l = limb(i + 2);
            let t = apply(&c.b.eff3(l.e[1]).m, self.at[i] + self.centre[i]);
            let side = if i == 0 { 1.0 } else { -1.0 };
            crate::nmdbm::limb_ik_w(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[delirium_data::kv!(0 int)], t, yaw, 0.0, side, -1.0, Vec3::ZERO, None, 1.0, true);
        }
    }

    /// RDR2's one-tick reverse flags (+0xb5/+0xb6), as yanked writes them before this tick.
    pub fn set_reverse(&mut self, r: [bool; 2]) {
        self.reverse = r;
    }

    /// RDR2's speeds: each leg's angular speed noised by speedAsymmetry, a negative one floored at 0.2 rad/s.
    fn speeds_rdr2(&mut self, w: f32, agent: f32) {
        let n = [2.0 * noise01(self.t, agent, self.t2) - 1.0, 2.0 * noise01(self.t2, agent, self.t) - 1.0];
        self.speed = n.map(|n| match w + self.speed_asymmetry * n {
            s if s < 0.0 => 0.2,
            s => s,
        });
    }

    /// RDR2's activate (rdr2_pedallegs.md section 3): the noise time and seed (t2), a random start phase with the
    /// right leg half a turn on (in step with hula), and the leg length from the left leg's joints.
    fn activate_rdr2(&mut self, c: &mut Ctx) {
        self.t = 5000.0 * c.ch.random();
        self.t2 = 5000.0 * c.ch.random();
        let phase = std::f32::consts::TAU * c.ch.random();
        self.speeds_rdr2(self.angular_speed, c.ch.agent as f32);
        self.hula = self.rdr2.hula;
        self.theta = [phase, if self.hula { phase } else { phase + std::f32::consts::PI }];
        let (h, k, a) = (c.b.joint_pos(HIP()[0]), c.b.joint_pos(KNEE()[0]), c.b.joint_pos(ANKLE()[0]));
        self.leg = (k - h).length() + (a - k).length() + 0.07;
        self.reverse = [false; 2];
    }

    /// RDR2's tick (section 4): leg stiffness, the radius noise, adaptive pedalling against the drag, each leg's phase
    /// and its foot target, and with hula the upper body's counter-motion. The one-tick reverse flags are yanked's
    /// (its back-pedalling legs, `set_reverse`), cleared after each tick.
    fn tick_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.rdr2;
        if p.hula != self.hula {
            self.theta[1] = self.theta[0] + if p.hula { 0.0 } else { std::f32::consts::PI };
            self.hula = p.hula;
        }
        self.t += dt;
        // the left hip as commanded last step (the hula's counter-motion follows it)
        let hip = c.ch.eff[HIP()[0]].desired;
        for i in 0..2 {
            c.ch.eff[HIP()[i]].set_stiffness(self.leg_stiffness, 0.5, None);
            c.ch.eff[KNEE()[i]].set_stiffness(self.leg_stiffness, 0.5, None);
            c.ch.eff[ANKLE()[i]].set_stiffness(1.5 * self.leg_stiffness, 0.5, None);
        }
        let agent = c.ch.agent as f32;
        let r = self.radius + self.radius_variance * (noise01(self.t + 10.0, self.t2, agent) - 0.5);
        let (mut legs, mut back, mut w) = ([self.left, self.right], self.back_pedal, self.angular_speed);
        if self.adaptive {
            // the spine part's velocity against the ground's (ours: the ground is still)
            let s = c.b.part(SPINE()[0] + 1);
            let n = s.v.length();
            let dir = if n > 0.0 { s.v / n } else { Vec3::ZERO };
            let q = n / self.radius * self.drag_multiplier;
            w = if q < 0.0 { 0.0 } else { self.angular_speed.min(q) };
            if c.up.cross(dir).dot(s.m.b) < 0.0 {
                back = !self.back_pedal;
            }
            if s.m.a.dot(c.up) < -0.4 {
                legs = [false; 2];
            }
        }
        self.speeds_rdr2(w, agent);
        for i in 0..2 {
            if legs[i] {
                let sign = if back { -1.0 } else { 1.0 } * if self.reverse[i] { -1.0 } else { 1.0 };
                self.theta[i] += sign * self.speed[i] * dt;
                self.leg_rdr2(c, i, if i == 0 { self.pedal_offset } else { -self.pedal_offset }, r);
            }
        }
        if p.hula {
            let (h1, h2) = (hip[0], hip[1]);
            let q = |x: f32| (0.25 * x).clamp(-9.9, 9.9);
            for j in *SPINE() {
                c.ch.eff[j].desired = [q(h1), q(-h2), q(-h1)];
            }
            for i in 0..2 {
                c.ch.eff[KNEE()[i]].desired[0] = -h1;
                c.ch.eff[SHOULDER()[i]].desired[0] = h1;
            }
            c.ch.eff[SHOULDER()[0]].desired[1] = h2 + 0.5;
            c.ch.eff[SHOULDER()[1]].desired[1] = h2 + 0.8;
            c.ch.eff[ELBOW()[0]].desired[0] = (h1 + 0.5).max(0.2);
            c.ch.eff[ELBOW()[1]].desired[0] = (h1 + 0.2).max(0.2);
        }
        self.reverse = [false; 2];
    }

    /// One leg's foot target (section 4.1): a point on an ellipse of size radius in the pelvis's (trunk axis,
    /// forward) plane, centred r - L (plus the offsets) along the trunk axis from the hip; with hula a circle in the
    /// (side, forward) plane at the hip's height. Reached by Delirium's leg IK, its twist as the IK's yaw (the IK
    /// request's velocity feed and dragReduction have no counterpart there).
    fn leg_rdr2(&self, c: &mut Ctx, i: usize, offset: f32, r: f32) {
        let p = self.rdr2;
        let (a, b) = if p.ellipse >= 0.0 { (1.0, p.ellipse) } else { (-p.ellipse, 1.0) };
        let (s, co) = self.theta[i].sin_cos();
        let big = self.radius;
        let x = if p.hula {
            Vec3::new(offset + self.centre_up, a * s * big, b * co * big - self.centre_forwards)
        } else {
            let side = if i == 0 { p.spread } else { -p.spread };
            Vec3::new(r - (1.0 - a) * big - self.leg + offset + self.centre_up + a * s * big, side - p.centre_sideways,
                      b * co * big - self.centre_forwards)
        };
        let m = c.b.part(0).m;
        let l = limb(i + 2);
        let t = c.b.joint_pos(l.e[1]) + m.a * x.x + m.b * x.y + m.c * x.z;
        let twist = self.leg_angle_variance * (noise01(c.ch.agent as f32, 0.5 * self.t, self.t2) - 0.5);
        let side = if i == 0 { 1.0 } else { -1.0 };
        crate::nmdbm::limb_ik_w(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[3], t, twist, 0.0, side, -1.0, Vec3::ZERO, None, 1.0, true);
    }
}
