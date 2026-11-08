use std::f32::consts::TAU;

use crate::nmrs::{Character, NECK, SPINE};
use crate::nmshot::{limb, Ctx};
use crate::Fma;

#[derive(Clone, Debug)]
pub struct Windmill {
    pub body_stiffness: f32,
    pub arm_stiffness: f32,
    pub period: f32,
    pub amplitude: f32,
    pub phase: f32,
    pub disable_on_impact: bool,
    t: f32,
    elbow: [(f32, f32, f32, f32); 2],
    pub active: bool,
    hit: [bool; 2],
    going: [bool; 2],
    /// RDR2 core: RDR2's parameters, and whether this run is RDR2's (started by the message with the core on)
    pub rdr2: Rdr2Params,
    pub r2: bool,
}

impl Default for Windmill {
    fn default() -> Windmill {
        Windmill { body_stiffness: delirium_data::kv!(117), arm_stiffness: delirium_data::kv!(202), period: 1.0, amplitude: delirium_data::kv!(119), phase: 0.0, disable_on_impact: true,
                   t: 0.0, elbow: [(0.0, 0.0, 0.0, 0.0); 2], active: false, hit: [false; 2], going: [false; 2], rdr2: Rdr2Params::default(), r2: false }
    }
}

/// armsWindmillAdaptive's RDR2 parameters, RDR2's names and defaults (rdr2_windmill.md B.2). Those GTA IV's message
/// shares (bodyStiffness, armStiffness, amplitude, phase, disableOnImpact) are kept here too so RDR2's defaults hold
/// until sent. Elbow angles and lean pairs are per arm / (mult, offset).
#[derive(Clone, Copy, Debug)]
pub struct Rdr2Params {
    pub body_stiffness: f32,
    pub arm_stiffness: f32,
    pub ang_speed: f32,
    pub amplitude: f32,
    pub clavicle_scale: f32,
    pub clavicle_bias: f32,
    pub phase: f32,
    pub elbow_angle: [f32; 2],
    pub lean1: (f32, f32),
    pub lean2: (f32, f32),
    pub elbow_rate: f32,
    pub sway: f32,
    pub drag_mult: f32,
    pub max_ang_speed: f32,
    pub scrabble_upper: f32,
    pub scrabble_lower: f32,
    pub scrabble_bend: f32,
    pub scrabble_extend: f32,
    pub arm_direction: i32,
    pub mask: &'static str,
    pub disable_on_impact: bool,
    pub set_back_angles: bool,
    pub use_ang_mom: bool,
    pub bend_elbow: [bool; 2],
    pub scrabble: bool,
}

impl Default for Rdr2Params {
    fn default() -> Rdr2Params {
        Rdr2Params { body_stiffness: 11.0, arm_stiffness: 14.14, ang_speed: 6.28, amplitude: 0.6, clavicle_scale: 1.0, clavicle_bias: 0.15,
                     phase: 0.0, elbow_angle: [-1.0; 2], lean1: (1.0, 0.0), lean2: (1.0, 0.0), elbow_rate: 1.0, sway: 0.0, drag_mult: 1.0,
                     max_ang_speed: 4.0, scrabble_upper: 4.0, scrabble_lower: 1.57, scrabble_bend: 2.2, scrabble_extend: 0.0, arm_direction: 0,
                     mask: "ub", disable_on_impact: false, set_back_angles: true, use_ang_mom: false, bend_elbow: [false; 2], scrabble: false }
    }
}

/// RDR2's elbow floor (15 degrees) and the rate it is pushed back up at (rad/s).
const RDR2_ELBOW_FLOOR: (f32, f32) = (0.2618, 15.0);

impl Windmill {
    fn stiffness(&self, ch: &mut Character) {
        ch.set_stiffness(self.body_stiffness, 0.5, "ub", None);
        ch.set_stiffness(self.arm_stiffness, 1.0, "ua", None);
    }

    pub fn activate(&mut self, ch: &mut Character) {
        if self.active {
            return;
        }
        self.active = true;
        if self.r2 {
            return self.activate_rdr2(ch);
        }
        self.t = delirium_data::kv!(221) * ch.random();
        for i in 0..2 {
            let l = ch.limits(limb(i).e[2]);
            self.elbow[i] = (l[0], l[1], delirium_data::kv!(222), delirium_data::kv!(30));
        }
        (self.hit, self.going) = ([false; 2], [true; 2]);
        self.stiffness(ch);
    }

    pub fn deactivate(&mut self) {
        if self.active {
            *self = Windmill::default();
        }
    }

    pub fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if !self.active {
            return;
        }
        if self.r2 {
            return self.tick_rdr2(c, dt);
        }
        for i in 0..2 {
            if self.disable_on_impact && !self.hit[i] && c.b.doll.collided(&[limb(i).p[delirium_data::kv!(0 int)]]) {
                self.hit[i] = true;
            }
            if self.disable_on_impact && self.hit[i] {
                self.going[i] = false;
            }
        }
        let ch = &mut *c.ch;
        if !self.going[0] && !self.going[1] {
            for e in ch.eff.iter_mut() {
                (e.strength, e.damping) = (e.strength * delirium_data::kv!(223), e.damping * delirium_data::kv!(223));
            }
            return;
        }
        self.stiffness(ch);
        self.t += dt;
        let w = TAU / self.period * if crate::nmdbm::sdot(c.angvel, c.m.a, b"yzx") >= 0.0 { -1.0 } else { 1.0 };
        let x = w * self.t;
        let amp = self.amplitude;
        for (i, at) in [(0, x), (1, self.phase + x)] {
            if !self.going[i] {
                continue;
            }
            let l = limb(i);
            let (s, co) = ((at as f64).sin() as f32, (at as f64).cos() as f32);
            ch.eff[l.e[0]].desired[0] = -(s.fma(delirium_data::kv!(18), delirium_data::kv!(17)) * amp);
            ch.eff[l.e[0]].desired[1] = amp.fma(co, -delirium_data::kv!(9));
            ch.eff[l.e[1]].desired[0] = -(amp * s) - delirium_data::kv!(24);
            ch.eff[l.e[1]].desired[1] = amp * co;
            let (lo, hi, min, rate) = self.elbow[i];
            let now = ch.actual(l.e[2], c.b.frames)[0];
            if now < min {
                ch.eff[l.e[2]].desired[0] = rate.fma(dt, now).clamp(lo, hi);
            }
        }
        let sin = (x as f64).sin() as f32;
        for j in (*SPINE()) {
            ch.eff[j].desired = [0.0, 0.0, sin * delirium_data::kv!(116) * delirium_data::kv!(17)];
        }
        ch.eff[(*NECK())[0]].desired[2] = sin * -delirium_data::kv!(24);
    }
}

impl Windmill {
    /// RDR2's activate: a random start phase (5000 U rad), both arms on, the masked joints at bodyStiffness.
    fn activate_rdr2(&mut self, ch: &mut Character) {
        self.t = 5000.0 * ch.random();
        (self.hit, self.going) = ([false; 2], [true; 2]);
        ch.set_stiffness(self.rdr2.body_stiffness, 0.5, self.rdr2.mask, None);
    }

    /// RDR2 core: a behaviour that owns the running windmill writes its speed and arm flags each tick (0x2a7af24).
    pub fn set_live(&mut self, ang_speed: f32, on: [bool; 2]) {
        self.rdr2.ang_speed = ang_speed;
        self.going = [on[0] && !self.hit[0], on[1] && !self.hit[1]];
    }

    /// RDR2's armsWindmillAdaptive (rdr2_windmill.md B.4): the arms circle at angSpeed rad/s the way his spin about
    /// his side says (or armDirection), the clavicle and shoulder swinging by amplitude with lean mults and offsets,
    /// the elbow kept above 15 degrees and drawn to its elbowAngle at elbowRate, bent on the forward half with
    /// bendLeftElbow / bendRightElbow; scrabble drives the phase from how fast his trunk moves and bends the elbows in
    /// a window of it. setBackAngles twists the spine and neck with it. An arm whose hand touches stops for good
    /// (disableOnImpact); with both stopped nothing is written. Only the mask's joints are written.
    fn tick_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.rdr2;
        for i in 0..2 {
            if p.disable_on_impact && c.b.doll.collided(&[limb(i).p[3]]) {
                (self.hit[i], self.going[i]) = (true, false);
            }
        }
        if !self.going.contains(&true) {
            return;
        }
        let keep = c.ch.lock(p.mask);
        c.ch.set_stiffness(p.body_stiffness, 0.5, "us", None);
        c.ch.set_stiffness(p.body_stiffness, 0.5, "un", None);
        c.ch.set_stiffness(p.arm_stiffness, 1.0, "ua", None);
        let d = if p.arm_direction != 0 {
            p.arm_direction as f32
        } else {
            let mut ws = c.angvel.dot(c.m.a);
            if p.use_ang_mom {
                // his angular momentum about his side, less a posture term from how far his back faces up
                ws = ang_mom(c).dot(c.m.a);
                let g = 6.8 * c.m.c.dot(c.up).clamp(-1.0, 1.0).asin();
                ws = if ws.abs() > 12.0 || ws * g < 0.0 { ws - g } else { -g };
            }
            if ws < 0.0 { 1.0 } else { -1.0 }
        };
        if p.scrabble {
            // (the floor's velocity: none in Delirium's world)
            let v = c.b.part((*SPINE())[0] + 1).v.length();
            self.t += dt * (4.0 * p.drag_mult * v).clamp(0.0, p.max_ang_speed);
        } else {
            self.t += d * p.ang_speed * dt;
        }
        let forward = d == 1.0 || p.scrabble;
        for i in 0..2 {
            if self.going[i] {
                self.arm_rdr2(c, i, if i == 0 { self.t } else { self.t + p.phase }, forward, dt);
            }
        }
        if p.set_back_angles {
            // RDR2's spine writer: a quarter on each spine joint
            let s = self.t.sin();
            let q = |x: f32| (0.25 * x).clamp(-9.9, 9.9);
            for j in *SPINE() {
                c.ch.eff[j].desired = [0.0, q(-s * p.sway), q(0.4 * s)];
            }
            c.ch.eff[(*NECK())[0]].desired[2] = -0.3 * s;
        }
        c.ch.unlock(p.mask, keep);
    }

    /// One arm at phase `th` (RDR2's 0x2aee93c).
    fn arm_rdr2(&self, c: &mut Ctx, i: usize, th: f32, forward: bool, dt: f32) {
        let p = &self.rdr2;
        let l = limb(i);
        let (s, co) = th.sin_cos();
        let a = p.amplitude * p.clavicle_scale;
        let tw = if p.scrabble { -1.0 } else { 0.0 };
        c.ch.eff[l.e[0]].desired = [(-0.25 - 0.75 * s) * a, a * co - p.clavicle_bias, tw];
        let (mut l1, mut l2) = (p.lean1.1 - (p.amplitude * p.lean1.0 * s + 0.3), p.amplitude * p.lean2.0 * co + p.lean2.1);
        if p.scrabble {
            (l1, l2) = (l1.clamp(0.0, 10.0), l2.clamp(-10.0, 10.0));
        }
        c.ch.eff[l.e[1]].desired = [l1, l2, tw];
        let el = l.e[2];
        let lim = c.ch.limits(el);
        let (lo, hi) = (lim[0], lim[1]);
        let e = p.elbow_angle[i];
        let mut now = c.ch.actual(el, c.b.frames)[0];
        let (floor, rate) = RDR2_ELBOW_FLOOR;
        if now < floor {
            now = (now + rate * dt).clamp(lo, hi);
            c.ch.eff[el].desired[0] = now;
        }
        if e > -0.01 {
            c.ch.eff[el].desired[0] = (now + 2.0 * p.elbow_rate * dt * (e - now)).clamp(lo, hi);
        }
        if !p.scrabble {
            if p.bend_elbow[i] && forward {
                c.ch.eff[el].desired[0] = if s >= 0.0 { 2.0 * s } else { e.max(0.2) };
            }
        } else {
            let t = th % TAU;
            c.ch.eff[el].desired[0] = if t > p.scrabble_upper || t < p.scrabble_lower { p.scrabble_bend } else { p.scrabble_extend };
        }
    }
}

/// His angular momentum about his centre of mass, the parts taken as point masses (Delirium keeps no whole-body
/// angular momentum; RDR2's ctx+0xf80 includes the parts' own spin).
fn ang_mom(c: &Ctx) -> bevy_math::Vec3 {
    (0..c.b.frames.len()).map(|k| {
        let q = c.b.part(k);
        (q.m.d - c.com).cross(q.v - c.vel) * c.b.doll.part_mass(k)
    }).sum()
}

