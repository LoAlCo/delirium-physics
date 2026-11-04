//! RDR2's armsWindmill (with the RDR2 core), from a study of RDR2's code (rdr2_windmill.md part A): each hand is
//! driven round an ellipse fixed to the chest, the right one's circle mirrored from the left one's. Used standalone
//! (the armsWindmill message, extras_held.rs) and by the shot's default arms (nmshot.rs), which grows the circles.

use crate::nmshot::{limb, Ctx};
use crate::prelude::*;

use std::f32::consts::{FRAC_PI_2, TAU};

/// armsWindmill's parameters, RDR2's names and defaults: per arm (left, right) the circle's three turns in rad
/// (RDR2's "normal" is Euler angles), its centre in the chest's frame, its two radii (m) and its speed (turns a
/// second); shoulder and elbow (stiffness, damping), phaseOffset (degrees), IKtwist, angVelThreshold / angVelGain,
/// the elbow floors, mirrorMode, adaptiveMode, forceSync, useLeft / useRight, disableOnImpact. dragReduction is read
/// but Delirium's limb IK has nothing for it.
#[derive(Clone, Copy, Debug)]
pub struct Params {
    pub normal: [Vec3; 2],
    pub centre: [Vec3; 2],
    pub radius: [[f32; 2]; 2],
    pub speed: [f32; 2],
    pub shoulder: (f32, f32),
    pub elbow: (f32, f32),
    pub phase_offset: f32,
    pub drag_reduction: f32,
    pub ik_twist: f32,
    pub ang_vel_threshold: f32,
    pub ang_vel_gain: f32,
    pub elbow_min: [f32; 2],
    pub mirror_mode: i32,
    pub adaptive_mode: i32,
    pub force_sync: bool,
    pub use_arm: [bool; 2],
    pub disable_on_impact: bool,
}

impl Default for Params {
    fn default() -> Params {
        Params {
            normal: [Vec3::new(0.0, 0.2, 0.2), Vec3::new(0.0, -0.2, -0.2)],
            centre: [Vec3::new(0.0, 0.5, -0.1), Vec3::new(0.0, -0.5, -0.1)],
            radius: [[0.75; 2]; 2],
            speed: [1.0; 2],
            shoulder: (12.0, 1.0),
            elbow: (12.0, 1.0),
            phase_offset: 0.0,
            drag_reduction: 0.2,
            ik_twist: 0.0,
            ang_vel_threshold: 0.1,
            ang_vel_gain: 1.0,
            elbow_min: [0.5; 2],
            mirror_mode: 1,
            adaptive_mode: 0,
            force_sync: true,
            use_arm: [true; 2],
            disable_on_impact: true,
        }
    }
}

/// armsWindmill's state: each arm's phase, the radius additions (to radius1 and radius2, per arm), the speed, shoulder and elbow
/// stiffness and phase (degrees) additions (all written only by the shot), and whether each hand is still clear of
/// contact (disableOnImpact: once it touches, that arm stops for good).
#[derive(Clone, Copy, Debug, Default)]
pub struct Windmill {
    pub theta: [f32; 2],
    pub delta: [[f32; 2]; 2],
    pub speed_add: [f32; 2],
    pub ks_add: f32,
    pub ke_add: f32,
    pub phase_add: f32,
    pub live: [bool; 2],
}

pub(crate) fn wrap_2pi(x: f32) -> f32 {
    x.rem_euclid(TAU)
}

/// Arm `i`'s circle: its frame rows (r1 and r2 span the ellipse) and its centre. The frame is the chest's turned a
/// quarter about its Z, then by the turns about Y, Z and X; the centre is in the unturned chest frame. With
/// mirrorMode the right circle is the left one's with the centre's y negated (the chest is a trunk part) and, for
/// mirrorMode 1, its frame reflected across the chest's y. (RDR2's part references are unsolved names: always the
/// chest here, RDR2's default.)
fn circle(c: &Ctx, p: &Params, i: usize) -> ([Vec3; 3], Vec3) {
    let chest = c.b.part(crate::nmshot::chest_part()).m;
    let src = if i == 1 && p.mirror_mode != 0 { 0 } else { i };
    let n = p.normal[src];
    let mut f = [chest.a, chest.b, chest.c];
    let turn = |f: &mut [Vec3; 3], a: usize, b: usize, ang: f32| {
        let (s, co) = ang.sin_cos();
        let (x, y) = (f[a], f[b]);
        (f[a], f[b]) = (x * co + y * s, y * co - x * s);
    };
    turn(&mut f, 0, 1, FRAC_PI_2);
    turn(&mut f, 2, 0, n.y);
    turn(&mut f, 0, 1, n.z);
    turn(&mut f, 1, 2, n.x);
    let mut centre = p.centre[src];
    if i == 1 && p.mirror_mode != 0 {
        centre.y = -centre.y;
        if p.mirror_mode == 1 {
            for row in f.iter_mut() {
                *row -= chest.b * (2.0 * row.dot(chest.b));
            }
        }
    }
    (f, crate::vm4(chest.a, chest.b, chest.c, chest.d, centre))
}

impl Windmill {
    /// RDR2's activate: each arm's phase from where its hand is on its circle, both hands clear.
    pub fn start(c: &Ctx, p: &Params) -> Windmill {
        let theta = [0, 1].map(|i| {
            let (f, o) = circle(c, p, i);
            let d = c.b.part(limb(i).p[3]).m.d - o;
            wrap_2pi(f[2].dot(d).atan2(f[1].dot(d)))
        });
        Windmill { theta, live: [true; 2], ..Default::default() }
    }

    /// RDR2's tick: the speeds (adaptiveMode 1..3 from his spin about his side), the arms' stiffness, forceSync
    /// keeping the right arm phaseOffset behind the left by speeding one arm's step up and the other's down by up to
    /// half, and each live arm's hand on its circle by IK with the elbow kept above its floor (approached at 5/s) and
    /// the wrist straight.
    pub fn tick(&mut self, c: &mut Ctx, p: &Params, dt: f32) {
        let ws = -c.angvel.dot(c.m.a);
        let (mut ks, mut kd) = (p.shoulder.0 + self.ks_add, p.shoulder.1);
        let ke = p.elbow.0 + self.ke_add;
        let speed = match p.adaptive_mode {
            0 => [p.speed[0] + self.speed_add[0], p.speed[1] + self.speed_add[1]],
            1 => {
                let s = if ws < 0.0 { 1.0 } else { -1.0 };
                [p.speed[0].abs() * s, p.speed[1].abs() * s]
            }
            _ => [if ws.abs() > p.ang_vel_threshold { ws * p.ang_vel_gain } else { 0.0 }; 2],
        };
        if p.adaptive_mode >= 3 {
            (ks, kd) = ((10.0 * ws.abs()).clamp(0.001, 15.0), ws.abs().clamp(0.1, 1.0));
        }
        for i in 0..2 {
            if p.use_arm[i] {
                let l = limb(i);
                c.ch.eff[l.e[0]].set_stiffness(ks, kd, None);
                c.ch.eff[l.e[1]].set_stiffness(ks, kd, None);
                c.ch.eff[l.e[2]].set_stiffness(ke, p.elbow.1, None);
            }
        }
        // forceSync (RDR2 compares leftSpeed's sign with itself, so only the two arms being used gate it)
        let mut step = [dt; 2];
        if p.force_sync && p.use_arm == [true; 2] {
            let (tl, tr) = (self.theta[0], self.theta[1]);
            let phi = wrap_2pi(tr - (p.phase_offset + self.phase_add).to_radians());
            let (mut d1, mut d2) = if phi > tl { (phi - tl, TAU - phi + tl) } else { (TAU - tl + phi, tl - phi) };
            if p.speed[0] < 0.0 && p.speed[1] < 0.0 {
                (d1, d2) = (d2, d1);
            }
            let m = (0.5 * d1.min(d2)).clamp(0.0, 0.5);
            step = if d1 > d2 { [dt * (1.0 - m), dt * (1.0 + m)] } else { [dt * (1.0 + m), dt * (1.0 - m)] };
        }
        for i in 0..2 {
            let l = limb(i);
            if self.live[i] && p.disable_on_impact && c.b.doll.collided(&[l.p[3]]) {
                self.live[i] = false;
            }
            if !(p.use_arm[i] && (self.live[i] || !p.disable_on_impact)) {
                continue;
            }
            let (f, o) = circle(c, p, i);
            self.theta[i] = wrap_2pi(self.theta[i] + TAU * step[i] * speed[i]);
            let (r, d) = (p.radius[i], self.delta[i]);
            let (s, co) = self.theta[i].sin_cos();
            let at = o + f[1] * ((r[0] + d[0]) * co) + f[2] * ((r[1] + d[1]) * s);            let side = if i == 0 { 1.0 } else { -1.0 };
            crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[3], at, p.ik_twist, 0.0, side, 1.0, Vec3::ZERO, None);
            let a = c.ch.actual(l.e[2], c.b.frames)[0];
            if a < p.elbow_min[i] {
                let floor = a + (p.elbow_min[i] - a) * (5.0 * dt).clamp(0.0, 1.0);
                let e = &mut c.ch.eff[l.e[2]].desired[0];
                *e = e.max(floor);
            }
            c.ch.eff[l.e[3]].desired = [0.0; 3];
        }
    }
}
