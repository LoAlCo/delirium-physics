use bevy_math::Vec3;
use crate::Vd;
use crate::Vx;

use crate::nmru::RollUp;
use crate::nmshot::{limb, Ctx};

#[derive(Clone, Debug)]
pub struct RollDownStairs {
    pub stiffness: f32,
    pub damping: f32,
    pub forcemag: f32,
    #[allow(dead_code)]
    pub asymmetry: f32,
    pub asymmetrical_legs: f32,
    pub use_arm_to_slow_down: f32,
    pub arm_reach_amount: f32,
    pub use_zero_pose: bool,
    pub spin_when_in_air: bool,
    pub leg_push: f32,
    pub arm_reach_length: f32,
    pub stiffness_decay_time: f32,
    pub stiffness_decay_target: f32,
    pub use_custom_roll_dir: bool,
    pub z_axis_spin_reduction: f32,
    pub custom_roll_dir: Vec3,
    pub helper_only: bool,
    /// RDR2's further parameters (read with the RDR2 core)
    pub rdr2: Rdr2Params,
    mass: f32,
    dir: Vec3,
    sum: Vec3,
    t: f32,
    slope: f32,
    /// RDR2: the foetal legs stay on once the legs were high
    latch: bool,
    pub active: bool,
}

/// RDR2's rollDownStairs parameters that GTA IV's hasn't, with RDR2's defaults.
#[derive(Clone, Copy, Debug)]
pub struct Rdr2Params {
    pub muscle_stiffness: f32,
    pub use_relative_velocity: bool,
    pub max_ang_vel_around_frontward_axis: f32,
    pub cheating_torques: bool,
    pub mag_of_torque_to_roll: f32,
    pub max_ang_vel: f32,
    pub min_ang_vel: f32,
    pub align: bool,
    pub delay_to_align_body: f32,
    pub mag_of_torque_to_align: f32,
    pub airborne_reduction: f32,
    pub foetal_legs: bool,
    pub movement_legs_in_foetal_position: f32,
    pub limit_spin_reduction: bool,
    pub target_linear_velocity: f32,
    pub target_linear_velocity_decay_time: f32,
}

impl Default for Rdr2Params {
    fn default() -> Rdr2Params {
        Rdr2Params { muscle_stiffness: 1.0, use_relative_velocity: false, max_ang_vel_around_frontward_axis: 2.0, cheating_torques: false,
                     mag_of_torque_to_roll: 50.0, max_ang_vel: 5.0, min_ang_vel: 0.5, align: false, delay_to_align_body: 0.2,
                     mag_of_torque_to_align: 50.0, airborne_reduction: 0.85, foetal_legs: false, movement_legs_in_foetal_position: 1.3,
                     limit_spin_reduction: false, target_linear_velocity: 1.0, target_linear_velocity_decay_time: 0.5 }
    }
}

impl Default for RollDownStairs {
    fn default() -> RollDownStairs {
        RollDownStairs { stiffness: delirium_data::kv!(117), damping: delirium_data::kv!(179), forcemag: delirium_data::kv!(132), asymmetry: 0.0, asymmetrical_legs: 0.0,
                         use_arm_to_slow_down: -delirium_data::kv!(192), arm_reach_amount: delirium_data::kv!(179), use_zero_pose: false, spin_when_in_air: false,
                         leg_push: 1.0, arm_reach_length: delirium_data::kv!(116), stiffness_decay_time: -1.0, stiffness_decay_target: delirium_data::kv!(137),
                         use_custom_roll_dir: false, z_axis_spin_reduction: 0.0, custom_roll_dir: Vec3::ZERO, helper_only: false, rdr2: Rdr2Params::default(),
                         mass: -1.0, dir: Vec3::ZERO, sum: Vec3::ZERO, t: 0.0, slope: 0.0, latch: false, active: false }
    }
}

fn safe_normalize(v: Vec3) -> Vec3 {
    if v.abs().max_element() > delirium_data::kv!(4) { crate::nmdbm::vnorm(v) } else { Vec3::ZERO }
}

/// RDR2's unit vector: zero below 1e-6 long.
fn unit(v: Vec3) -> Vec3 {
    let l = v.length();
    if l > 1e-6 { v / l } else { Vec3::ZERO }
}

fn flat(v: Vec3, up: Vec3) -> Vec3 {
    v - up * up.dot(v)
}

/// RDR2's spine parts, pelvis to chest (the five its roll helpers push).
fn spine_parts() -> [usize; 5] {
    [0, delirium_data::kv!(22 int), delirium_data::kv!(23 int), delirium_data::kv!(104 int), delirium_data::kv!(81 int)]
}

/// Every part but the legs' (RDR2's upper-body contact mask).
fn upper_parts(c: &Ctx) -> Vec<usize> {
    let legs: Vec<usize> = (2..4).flat_map(|i| limb(i).p[1..].to_vec()).collect();
    (0..c.b.build.parts.len()).filter(|k| !legs.contains(k)).collect()
}

/// RDR2: the upper body is on something and a foot is no more than 0.8 m under the head.
fn legs_high(c: &Ctx) -> bool {
    let up = c.up;
    let head = up.dot(c.b.part(delirium_data::kv!(55 int)).m.d);
    let feet = [2, 3].map(|i| up.dot(c.b.part(limb(i).p[3]).m.d));
    c.b.doll.collided(&upper_parts(c)) && feet[0].max(feet[1]) >= head - 0.8
}

impl RollDownStairs {
    /// RDR2 core: sit writes these while it runs (rollDownStairs +0x5c, and +0xa0 in the seat's frame).
    pub fn live(&mut self, force: f32, dir: Vec3) {
        self.forcemag = force;
        self.custom_roll_dir = dir;
        self.dir = dir;
    }

    /// RDR2's defaults (its handler's with no message).
    pub fn rdr2_defaults() -> RollDownStairs {
        RollDownStairs { stiffness: 11.0, damping: 1.4, forcemag: 0.55, asymmetrical_legs: 0.0, use_arm_to_slow_down: -1.9, arm_reach_amount: 1.4,
                         use_zero_pose: false, spin_when_in_air: false, leg_push: 1.0, arm_reach_length: 0.4, stiffness_decay_time: -1.0,
                         stiffness_decay_target: 9.0, use_custom_roll_dir: false, z_axis_spin_reduction: 0.0, custom_roll_dir: Vec3::Z,
                         helper_only: false, ..Default::default() }
    }

    pub fn toward(dir: Vec3) -> RollDownStairs {
        RollDownStairs { use_custom_roll_dir: true, custom_roll_dir: dir, ..Default::default() }
    }

    pub fn activate(&mut self, c: &mut Ctx, roll_up: &mut RollUp) {
        if self.active {
            return;
        }
        self.active = true;
        if c.ch.rdr2 {
            return self.activate_rdr2(c, roll_up);
        }
        if !self.helper_only {
            let r = roll_up;
            (r.use_arm_to_slow_down, r.arm_reach_amount, r.stiffness) = (self.use_arm_to_slow_down, self.arm_reach_amount, self.stiffness);
            (r.leg_push, r.arm_reach_length, r.asymmetrical_legs) = (self.leg_push, self.arm_reach_length, self.asymmetrical_legs);
            r.activate(c.ch);
        }
        c.ch.set_stiffness(self.stiffness, self.damping, "fb", None);
        self.dir = safe_normalize(if self.use_custom_roll_dir { self.custom_roll_dir } else { c.vel });
        self.slope = self.dir.vd(c.up).clamp(-1.0, 1.0).asin();
        self.sum = -c.up;
        self.t = 0.0;
        if self.mass < 0.0 {
            self.mass = c.b.doll.mass();
        }
    }

    pub fn deactivate(&mut self, roll_up: &mut RollUp) {
        if self.active {
            roll_up.deactivate();
            *self = RollDownStairs::default();
        }
    }

    /// `roll_up`: the bodyRollUp RDR2's keeps running under it; `brace`: braceForImpact is running.
    pub fn tick(&mut self, c: &mut Ctx, roll_up: &mut RollUp, brace: bool, dt: f32) {
        if !self.active {
            return;
        }
        if c.ch.rdr2 {
            return self.tick_rdr2(c, roll_up, brace, dt);
        }
        self.t += dt;
        if !self.helper_only && self.stiffness_decay_time > 0.0 {
            let k = (self.t / self.stiffness_decay_time).min(1.0);
            c.ch.set_stiffness(crate::Fma::fma(1.0 - k, self.stiffness, self.stiffness_decay_target * k), self.damping, "fb", None);
        }
        self.sum = safe_normalize(self.sum + c.vel);
        self.dir = safe_normalize(if self.use_custom_roll_dir { self.custom_roll_dir } else { self.sum });
        self.sum *= delirium_data::kv!(64);
        self.slope = self.dir.vd(c.up).clamp(-1.0, 1.0).asin();
        if self.use_zero_pose && !self.helper_only {
            let k = crate::Fma::fma(-crate::nmdbm::slen(c.angvel, b"yxz").clamp(0.0, delirium_data::kv!(31)), delirium_data::kv!(78), 1.0);
            for e in c.ch.eff.iter_mut() {
                e.blend_to_zero(k);
            }
        }
        self.roll_force(c);
    }

    fn roll_force(&self, c: &mut Ctx) {
        let all: Vec<usize> = (0..c.b.build.parts.len()).collect();
        let mut touching = c.b.doll.collided(&all);
        let (w, up) = (c.angvel, c.up);
        let (spin, speed) = (crate::nmdbm::slen(w, b"yxz"), crate::nmdbm::slen(c.vel, b"yxz"));
        let fwd = (-spin).mul_add(delirium_data::kv!(140), speed).clamp(-1.0, 1.0);
        let axis = if self.slope.abs() > delirium_data::kv!(179) && !self.use_custom_roll_dir {
            if spin > 0.5 { -safe_normalize(w - up * up.vd(w)) } else { c.m.a }
        } else {
            safe_normalize(self.dir.vx(up))
        };
        let mut f = -delirium_data::kv!(79) * self.forcemag;
        if !touching && self.spin_when_in_air {
            touching = true;
            f = (1.0 - (spin * delirium_data::kv!(78)).clamp(0.0, 1.0)) * f * delirium_data::kv!(9);
        }
        let k = (fwd.abs() * (1.0 / self.mass) * f).clamp(-2.0, 2.0);
        if fwd > 0.0 && speed > 0.5 && touching {
            for p in [0, delirium_data::kv!(22 int), delirium_data::kv!(23 int), delirium_data::kv!(104 int), delirium_data::kv!(81 int)] {
                c.out.torque(p, axis * (c.b.doll.part_mass(p) * k * delirium_data::kv!(31)));
            }
            if self.z_axis_spin_reduction > 0.0 {
                crate::nmcf::spin_reduction(c, self.z_axis_spin_reduction);
            }
        }
    }

    /// RDR2: bodyRollUp's values from ours (the eight it copies every tick); `mask` the joints it may drive.
    fn configure(&self, r: &mut RollUp, mask: &'static str) {
        (r.use_arm_to_slow_down, r.arm_reach_amount, r.stiffness) = (self.use_arm_to_slow_down, self.arm_reach_amount, self.stiffness);
        (r.rdr2.damping, r.rdr2.muscle_stiffness) = (self.damping, self.rdr2.muscle_stiffness);
        (r.leg_push, r.arm_reach_length, r.asymmetrical_legs, r.mask) = (self.leg_push, self.arm_reach_length, self.asymmetrical_legs, mask);
    }

    /// RDR2's start: bodyRollUp reset to its defaults, configured and started under us, the whole body's stiffness,
    /// the foetal legs if the legs are already high, the roll direction and the slope along it.
    fn activate_rdr2(&mut self, c: &mut Ctx, ru: &mut RollUp) {
        let p = self.rdr2;
        if !self.helper_only {
            ru.rdr2 = Default::default();
            (ru.hips_from_vel, ru.mask) = (false, "fb");
            self.configure(ru, "fb");
            ru.activate(c.ch);
        }
        c.ch.set_stiffness(self.stiffness, self.damping, "fb", Some(p.muscle_stiffness));
        (self.t, self.mass) = (0.0, c.b.doll.mass());
        self.dir = unit(if self.use_custom_roll_dir { self.custom_roll_dir } else { c.vel });
        if p.foetal_legs {
            if legs_high(c) || self.latch {
                self.latch = true;
                c.ch.set_stiffness(10.0, self.damping, "lb", None);
                self.foetal(c, ru);
            } else {
                ru.mask = "fb";
            }
        }
        self.slope = self.dir.dot(c.up).clamp(-1.0, 1.0).asin();
        self.sum = -c.up;
    }

    /// RDR2's tick, step by step as its: the legs-high and frontward-spin checks, the align torque, the stiffness decay,
    /// the direction, the zero pose, bodyRollUp kept running and configured, the foetal legs, then the helper torques
    /// and the wrists.
    fn tick_rdr2(&mut self, c: &mut Ctx, ru: &mut RollUp, brace: bool, dt: f32) {
        let p = self.rdr2;
        let (up, w, v) = (c.up, c.angvel, c.vel);
        let high = (p.cheating_torques || p.foetal_legs) && !self.latch && legs_high(c);
        let spinning = (p.cheating_torques || p.align) && p.max_ang_vel_around_frontward_axis > 0.0 && self.frontward_spin(c, high);
        if p.align && self.t > p.delay_to_align_body {
            let n = unit(up.cross(self.dir));
            if n.dot(w).abs() >= p.min_ang_vel && !spinning {
                self.align(c, n, p.mag_of_torque_to_align);
            }
        }
        self.t += dt;
        let ms = Some(p.muscle_stiffness);
        if !self.helper_only && self.stiffness_decay_time > 1e-6 {
            let f = (self.t / self.stiffness_decay_time).min(1.0);
            let s = (1.0 - f) * self.stiffness + f * self.stiffness_decay_target;
            c.ch.set_stiffness(if s < 0.0 { 0.0 } else { s.min(20.0) }, self.damping, "fb", ms);
        }
        self.sum = unit(self.sum + v);
        self.dir = unit(if self.use_custom_roll_dir { self.custom_roll_dir } else { self.sum });
        self.sum *= 5.0;
        self.slope = self.dir.dot(up).clamp(-1.0, 1.0).asin();
        if self.use_zero_pose && !self.helper_only {
            let k = 1.0 - 0.1 * w.length().clamp(0.0, 10.0);
            for e in c.ch.eff.iter_mut() {
                e.blend_to_zero(k);
            }
        }
        if !self.helper_only {
            if !ru.active {
                ru.rdr2 = Default::default();
                (ru.hips_from_vel, ru.mask) = (false, "fb");
                self.configure(ru, "fb");
                ru.activate(c.ch);
            }
            self.configure(ru, "fb");
        }
        if p.foetal_legs {
            if !high && !self.latch {
                ru.mask = "fb";
            } else {
                self.latch = true;
                c.ch.set_stiffness(10.0, self.damping, "lb", None);
                self.foetal(c, ru);
            }
        }
        if p.cheating_torques {
            if !spinning && (high || self.latch) && self.dir.dot(v) > 0.0 {
                // (as the roll helper's, the axis rolls him forward down the slope: GTA IV's sign, see roll_rdr2)
                let axis = unit(up.cross(self.dir));
                let s = axis.dot(w).abs();
                if p.min_ang_vel < s && s < p.max_ang_vel {
                    let sp = spine_parts();
                    for (i, k) in [(1, 1.0), (2, 1.0), (3, 0.6), (4, 0.6)] {
                        c.out.torque(sp[i], axis * (p.mag_of_torque_to_roll * k));
                    }
                }
            }
        } else {
            let decay = p.target_linear_velocity_decay_time;
            let tv = if decay > 1e-6 { p.target_linear_velocity * (1.0 - self.t.clamp(0.0, decay) / decay) } else { p.target_linear_velocity };
            // the ground stands still here, so the floor-relative speed is the speed
            let speed = if brace && p.use_relative_velocity { self.custom_roll_dir.length() } else { v.length() };
            let mut sr = self.z_axis_spin_reduction;
            if p.limit_spin_reduction {
                let b = c.m.b;
                sr *= (1.0 - unit(v).dot(b).abs()) * (1.0 - up.dot(b).abs()) * unit(w).dot(b).abs();
            }
            self.roll_rdr2(c, speed, tv, sr);
        }
        // fall to knees' overlay (+0xf0, +0xf4..+0xf7)
        if let Some(k) = c.ch.core.rds_knee.filter(|k| k.on) {
            use crate::nmrs::{ELBOW, HIP, KNEE, SHOULDER, SPINE};
            let st = self.stiffness;
            for i in 0..2 {
                c.ch.eff[HIP()[i]].desired[0] = -0.1;
                c.ch.eff[HIP()[i]].strength = st * st;
                c.ch.eff[KNEE()[i]].desired[0] = [-0.5, -0.9][i];
            }
            for &j in SPINE().iter() {
                c.ch.eff[j].desired[0] = k.lean;
                if k.stiff {
                    c.ch.eff[j].strength = (st + 1.0) * (st + 1.0);
                }
            }
            let poses = if k.poses[0] {
                Some([[0.0, -0.5, -0.7], [0.0, -0.9, -0.7]])
            } else if k.poses[1] {
                Some([[0.0, 0.9, 0.7], [0.0, 0.7, 0.7]])
            } else {
                None
            };
            if let Some(a) = poses {
                for i in 0..2 {
                    c.ch.eff[SHOULDER()[i]].desired = a[i];
                    c.ch.eff[ELBOW()[i]].desired[0] = [0.9, 0.6][i];
                }
            }
        }
        c.ch.set_stiffness(8.0, 1.0, "uw", Some(5.0));
    }

    /// RDR2: spinning about the body's front-back axis faster than maxAngVelAroundFrontwardAxis with the spine lying
    /// within 1.15 rad of the roll direction.
    fn frontward_spin(&self, c: &Ctx, high: bool) -> bool {
        let up = c.up;
        let mut f = unit(-c.b.pmat(0).m.c);
        if high {
            let s = unit(flat(self.dir.cross(up), up));
            f = s.cross(self.dir);
        }
        let spin = c.angvel.dot(f).abs();
        let sp = spine_parts();
        let p = unit(flat(c.b.part(sp[4]).m.d - c.b.part(sp[0]).m.d, up));
        let q = unit(flat(self.dir, up));
        let a = p.dot(q).abs().clamp(-1.0, 1.0).acos();
        spin > self.rdr2.max_ang_vel_around_frontward_axis && a < 1.15
    }

    /// RDR2's align torque: turns the spine to lie along the roll axis n (a log roll).
    fn align(&self, c: &mut Ctx, n: Vec3, mag: f32) {
        use std::f32::consts::FRAC_PI_2;
        let sp = spine_parts();
        let p = unit(c.b.part(sp[4]).m.d - c.b.part(sp[0]).m.d);
        let mut k = unit(p.cross(n));
        let mut th = n.dot(p).clamp(-1.0, 1.0).acos();
        let w = c.angvel.dot(k);
        let th2 = if (th - FRAC_PI_2).abs() < 0.2 { th + 0.1 * w } else { th };
        if th2 >= FRAC_PI_2 {
            k = -k;
        }
        if th > FRAC_PI_2 {
            th = std::f32::consts::PI - th;
        }
        let m = (0.5470459 * th - 0.1312910 * w.abs()).clamp(0.0, 1.0) * mag;
        for k2 in sp {
            c.out.torque(k2, unit(k) * (0.2 * m));
        }
    }

    /// RDR2's foetal legs: bodyRollUp leaves the legs to us, the hips bent and the knees folded by how far each foot is
    /// above the pelvis (across the roll direction with a custom one).
    fn foetal(&self, c: &mut Ctx, ru: &mut RollUp) {
        ru.mask = "ub";
        let up = c.up;
        let n = if self.use_custom_roll_dir && self.custom_roll_dir.length() > 0.7 { self.dir.cross(up).cross(self.dir) } else { up };
        let pelvis = c.b.part(0).m.d;
        let m = self.rdr2.movement_legs_in_foetal_position;
        for i in 0..2 {
            let leg = limb(2 + i);
            let h = n.dot(c.b.part(leg.p[3]).m.d - pelvis) * m;
            c.ch.eff[leg.e[1]].desired = [1.6 - 0.15 * h, -0.15 - 0.15 * h, -0.15 - 0.15 * h];
            c.ch.eff[leg.e[2]].desired[0] = h - 2.0;
        }
    }

    /// RDR2's roll helper: a roll torque on the five spine parts by how much faster than its spin he moves (with the
    /// target speed as a floor), weaker in the air, then the spin about up damped.
    fn roll_rdr2(&self, c: &mut Ctx, speed: f32, target: f32, sr: f32) {
        let all: Vec<usize> = (0..c.b.build.parts.len()).collect();
        let mut touching = c.b.doll.collided(&all);
        let (w, up) = (c.angvel, c.up);
        let spin = w.length();
        let vv = speed.max(target);
        let e = vv - 0.35 * spin;
        if e <= 0.0 || vv <= 0.5 {
            return;
        }
        let e = e.min(1.0);
        // RDR2's note reads up x d with a negative gain; GTA IV's (and Delirium's) is d x up with the same gain, which
        // rolls him forward, down the slope. We keep GTA IV's sign [P].
        let axis = if self.slope.abs() <= 1.4 || self.use_custom_roll_dir {
            unit(self.dir.cross(up))
        } else if spin > 0.5 {
            -unit(flat(w, up))
        } else {
            c.m.a
        };
        let mut g = -50.0 * self.forcemag;
        if !touching && self.spin_when_in_air {
            g *= (1.0 - self.rdr2.airborne_reduction) * (1.0 - (0.1 * spin).clamp(0.0, 1.0));
            touching = true;
        }
        let t = (g / self.mass * e).clamp(-2.0, 2.0);
        if touching {
            for k in spine_parts() {
                c.out.torque(k, axis * (10.0 * t * c.b.doll.part_mass(k)));
            }
            if sr > 0.0 {
                crate::nmcf::spin_reduction(c, sr);
            }
        }
    }
}
