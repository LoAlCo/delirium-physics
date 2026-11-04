use crate::prelude::*;
use crate::Vd;

use crate::nmdbm::Body;
use crate::nmrs::{Character, CLAVICLE, SPINE};

#[derive(Clone, Debug)]
pub struct SpineTwist {
    pub pos: Vec3,
    pub vel: Vec3,
    pub stiffness: f32,
    pub twist: f32,
    pub active: bool,
    pub arms: bool,
    pub always: bool,
    pub twisting: bool,
    /// RDR2 core: rad added to the measured twist (RDR2 turns the pelvis frame by it before measuring)
    pub offset: f32,
    range: [(f32, f32); 4],
    sum: (f32, f32),
    inv: (f32, f32),
}

impl Default for SpineTwist {
    fn default() -> SpineTwist {
        SpineTwist { pos: Vec3::ZERO, vel: Vec3::ZERO, stiffness: delirium_data::kv!(21), twist: 0.0, active: false, arms: false, always: false,
                     twisting: false, offset: 0.0, range: [(0.0, 0.0); 4], sum: (0.0, 0.0), inv: (0.0, 0.0) }
    }
}

impl SpineTwist {
    pub fn activate(&mut self, ch: &mut Character) {
        if self.active {
            return;
        }
        (self.active, self.twist, self.twisting) = (true, 0.0, true);
        if ch.rdr2 {
            return self.activate_rdr2(ch);
        }
        for (k, j) in (*SPINE()).into_iter().enumerate() {
            let l = ch.limits(j);
            let f = if k == 0 { delirium_data::kv!(517) } else { delirium_data::kv!(140 f32) + 1.0 };
            self.range[k] = (l[delirium_data::kv!(1 int)] * f, l[delirium_data::kv!(2 int)] * f);
        }
        let r = &self.range;
        self.sum = (((r[2].0 + r[1].0) + r[0].0) + r[delirium_data::kv!(0 int)].0, ((r[delirium_data::kv!(0 int)].1 + r[2].1) + r[1].1) + r[0].1);
        self.inv = (1.0 / self.sum.0, 1.0 / self.sum.1);
        ch.set_stiffness(self.stiffness, 2.0, "us", None);
    }

    pub fn deactivate(&mut self) {
        if self.active {
            *self = SpineTwist::default();
        }
    }

    pub fn tick(&mut self, ch: &mut Character, b: &Body) {
        if !self.active {
            return;
        }
        if ch.rdr2 {
            return self.tick_rdr2(ch, b);
        }
        let p = b.pmat(0);
        let t = (self.vel - p.v).mul_add(Vec3::splat(delirium_data::kv!(78)), self.pos);
        let n = crate::rigid::nrm(t - p.m.d);
        self.twist = crate::nmdbm::atan2(p.m.b.vd(n), -p.m.c.vd(n));
        let lim = delirium_data::kv!(254);
        if !(self.twist < lim && self.twist > -lim) && !self.always {
            self.twisting = false;
            return;
        }
        self.twisting = true;
        let up = crate::nmdbm::sdot(ch.block.unwrap_or_else(|| b.block()).m.b, ch.up, b"zyx").abs();
        let x = (up * self.twist).max(self.sum.0).min(self.sum.1);
        for (k, j) in (*SPINE()).into_iter().enumerate() {
            let share = if x >= 0.0 { self.range[k].1 * self.inv.1 } else { self.range[k].0 * self.inv.0 };
            ch.eff[j].desired[2] = share * x;
        }
        if self.arms {
            ch.eff[(*CLAVICLE())[1]].desired[0] = x;
            ch.eff[(*CLAVICLE())[0]].desired[0] = -x;
        }
    }

    /// RDR2's activate (rdr2_spinetwist.md section 3): every spine joint may twist 35 % past its limits, the ranges
    /// summed per side. No stiffness: RDR2's spineTwist sets none.
    fn activate_rdr2(&mut self, ch: &mut Character) {
        for (k, j) in (*SPINE()).into_iter().enumerate() {
            let l = ch.limits(j);
            self.range[k] = (l[4] * (1.0 + RDR2_EXCESS), l[5] * (1.0 + RDR2_EXCESS));
        }
        let r = &self.range;
        self.sum = (r[0].0 + r[1].0 + r[2].0 + r[3].0, r[0].1 + r[1].1 + r[2].1 + r[3].1);
        self.inv = (1.0 / self.sum.0, 1.0 / self.sum.1);
    }

    /// RDR2's tick (rdr2_spinetwist.md section 4): the twist to the led target in the pelvis frame keeps its side
    /// while twisting once the target is within 0.1 pi of straight behind; scaled by how upright the trunk is, it
    /// twists only inside +-0.8 pi (or always), shared over the spine joints by their ranges, the clavicles' twist
    /// channel turned with it (right +, left -). Not twisting leaves the spine alone.
    fn tick_rdr2(&mut self, ch: &mut Character, b: &Body) {
        use std::f32::consts::PI;
        let p = b.pmat(0);
        let t = (self.vel - p.v) * 0.1 + self.pos;
        let n = crate::rigid::nrm(t - p.m.d);
        let mut th = crate::nmdbm::atan2(p.m.b.vd(n), -p.m.c.vd(n)) + self.offset;
        if self.twisting && !(-0.9 * PI..=0.9 * PI).contains(&th) {
            th = if self.twist < 0.0 { -th.abs() } else { th.abs() };
        }
        self.twist = th;
        let up = crate::nmdbm::sdot(ch.block.unwrap_or_else(|| b.block()).m.b, ch.up, b"zyx").abs();
        let g = up * th;
        if !(g > -0.8 * PI && g < 0.8 * PI) && !self.always {
            self.twisting = false;
            return;
        }
        self.twisting = true;
        let x = g.max(self.sum.0).min(self.sum.1);
        for (k, j) in (*SPINE()).into_iter().enumerate() {
            let share = if x < 0.0 { self.range[k].0 * self.inv.0 } else { self.range[k].1 * self.inv.1 };
            ch.eff[j].desired[2] = share * x;
        }
        if self.arms {
            ch.eff[(*CLAVICLE())[1]].desired[2] = x;
            ch.eff[(*CLAVICLE())[0]].desired[2] = -x;
        }
    }
}

/// RDR2's spineTwist limit excess: each spine joint may twist this much past its limits.
const RDR2_EXCESS: f32 = 0.35;

