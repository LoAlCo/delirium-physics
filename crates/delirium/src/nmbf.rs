#![allow(dead_code)]
use crate::nmrs::{Character, ANKLE, CLAVICLE, ELBOW, HIP, KNEE, NECK, SHOULDER, SPINE, WRIST};
use crate::nmshot::limb;

#[derive(Clone, Debug)]
pub struct Foetal {
    pub stiffness: f32,
    pub damping_factor: f32,
    pub asymmetry: f32,
    pub back_twist: f32,
    pub random_seed: u32,
    pub mask: &'static str,
    pub active: bool,
}

impl Default for Foetal {
    fn default() -> Foetal {
        Foetal { stiffness: delirium_data::kv!(137), damping_factor: delirium_data::kv!(179), asymmetry: 0.0, back_twist: 0.0, random_seed: delirium_data::kv!(203 int), mask: "fb", active: false }
    }
}

struct Rng(u32, u32);

impl Rng {
    fn new(s: u32) -> Rng {
        Rng(s + (s == 0) as u32, s.rotate_left(delirium_data::kv!(54 int)) ^ s)
    }

    fn next(&mut self) -> f32 {
        let n = self.0 as u64 * delirium_data::kv!(198 int) + self.1 as u64;
        (self.0, self.1) = (n as u32, (n >> delirium_data::kv!(7 int)) as u32);
        (self.0 & delirium_data::kv!(196 int)) as f32 * delirium_data::kv!(230)
    }
}

impl Foetal {
    /// RDR2's defaults (a start under the RDR2 core reloads them before the message's values; randomSeed isn't
    /// RDR2's).
    pub fn rdr2() -> Foetal {
        Foetal { stiffness: 9.0, damping_factor: 1.0, asymmetry: 0.0, back_twist: 0.0, mask: "fb", ..Foetal::default() }
    }

    pub fn activate(&mut self, ch: &mut Character) {
        if !self.active {
            self.active = true;
            self.pose(ch);
        }
    }

    pub fn deactivate(&mut self) {
        if self.active {
            *self = Foetal::default();
        }
    }

    pub fn tick(&mut self, ch: &mut Character) {
        if self.active {
            self.pose(ch);
        }
    }

    fn pose(&self, ch: &mut Character) {
        if ch.rdr2 {
            self.curl_rdr2(ch);
            return;
        }
        let keep = ch.lock(self.mask);
        self.curl(ch);
        ch.unlock(self.mask, keep);
    }

    fn curl(&self, ch: &mut Character) {
        let mut r = Rng::new(self.random_seed);
        if self.asymmetry == 0.0 {
            ch.set_stiffness(self.stiffness, self.damping_factor, "fb", None);
        } else {
            let a = self.asymmetry * delirium_data::kv!(24);
            let (s0, s1) = ((1.0 - self.asymmetry * 0.5) * self.stiffness, (a + 1.0) * self.stiffness);
            let (d0, d1) = ((1.0 - a) * self.damping_factor, (a + 1.0) * self.damping_factor);
            for e in ch.eff.iter_mut() {
                let s = r.next() * (s1 - s0) + s0;
                let d = r.next() * (d1 - d0) + d0;
                (e.strength, e.damping) = (s * s, d * s * 2.0);
            }
        }
        let twist = r.next() * self.back_twist;
        for i in 2..delirium_data::kv!(1 int) {
            let l = limb(i);
            ch.eff[l.e[1]].desired[0] = 2.0;
            ch.eff[l.e[2]].desired[0] = -2.0;
        }
        for j in (*SPINE()) {
            ch.eff[j].desired = [0.5, 0.0, twist * delirium_data::kv!(17)];
        }
        ch.eff[(*NECK())[0]].desired[0] = 0.5;
        for i in [1, 0] {
            let l = limb(i);
            ch.eff[l.e[1]].desired[2] = -delirium_data::kv!(24);
            ch.eff[l.e[1]].desired[0] = delirium_data::kv!(231);
            ch.eff[l.e[2]].desired[0] = 2.0;
        }
        for i in 2..delirium_data::kv!(1 int) {
            let e = &mut ch.eff[limb(i).e[delirium_data::kv!(0 int)]];
            (e.strength, e.damping, e.desired[0]) = (delirium_data::kv!(25), delirium_data::kv!(64), -1.0);
        }
    }

    /// RDR2's bodyFoetal (rdr2_foetal.md), at the start and every tick: the draws come from a copy of the character's
    /// generator (its state isn't advanced, so the pattern is the same every tick). The mask limits only the even
    /// stiffness; the asymmetric stiffness and the pose are written whatever the mask.
    fn curl_rdr2(&self, ch: &mut Character) {
        let kept = ch.rng;
        let (k, d, a) = (self.stiffness, self.damping_factor, self.asymmetry);
        if a == 0.0 {
            ch.set_stiffness(k, d, self.mask, None);
        } else {
            let (s0, s1, d0) = ((1.0 - 0.5 * a) * k, (1.0 + 0.3 * a) * k, (1.0 - 0.3 * a) * d);
            let draw = |ch: &mut Character| {
                let (u1, u2) = (ch.random(), ch.random());
                (s0 + u2 * (s1 - s0), d0 + u1 * (d - d0))
            };
            let limbs = [CLAVICLE()[0], SHOULDER()[0], ELBOW()[0], WRIST()[0], CLAVICLE()[1], SHOULDER()[1], ELBOW()[1], WRIST()[1],
                         HIP()[0], KNEE()[0], ANKLE()[0], HIP()[1], KNEE()[1], ANKLE()[1]];
            for j in limbs {
                let (s, dd) = draw(ch);
                ch.eff[j].set_stiffness(s, dd, None);
            }
            // the spine's draws go in raw (strength s, damping d), as compiled
            for j in *SPINE() {
                let (s, dd) = draw(ch);
                (ch.eff[j].strength, ch.eff[j].damping) = (s, dd);
            }
        }
        let twist = self.back_twist * ch.random();
        for i in 0..2 {
            ch.eff[HIP()[i]].desired[0] = 2.0;
            ch.eff[KNEE()[i]].desired[0] = -2.0;
            let e = &mut ch.eff[ANKLE()[i]];
            (e.strength, e.damping, e.desired[0]) = (20.0, 5.0, -1.0);
            ch.eff[SHOULDER()[i]].desired[2] = -0.3;
            ch.eff[SHOULDER()[i]].desired[0] = 1.57;
            ch.eff[ELBOW()[i]].desired[0] = 2.0;
        }
        // the spine writer: a quarter of (2, 0, twist) on each spine joint
        for j in *SPINE() {
            ch.eff[j].desired = [0.5, 0.0, (0.25 * twist).clamp(-9.9, 9.9)];
        }
        ch.eff[NECK()[0]].desired[0] = 0.5;
        ch.rng = kept;
    }
}
