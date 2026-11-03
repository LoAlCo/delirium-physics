use crate::fneg;
use std::f32::consts::FRAC_PI_2;
use crate::Vx;

use crate::prelude::*;

use crate::nmdbm::{acos, sdot, slen, Body};
use crate::rigid::{nrm, q_axis, q_scaled, qrot};
use crate::Fma;
use crate::nmrs::{Character, NECK, SPINE};

#[allow(non_snake_case)]
#[inline]
fn SPINE3() -> usize { delirium_data::kv!(81 int) }
#[allow(non_snake_case)]
#[inline]
fn HEAD() -> usize { delirium_data::kv!(55 int) }
#[allow(non_snake_case)]
#[inline]
fn NECK_PART() -> usize { delirium_data::kv!(113 int) }

#[derive(Clone, Debug)]
pub struct HeadLook {
    pub pos: Vec3,
    pub vel: Vec3,
    pub stiffness: f32,
    pub damping: f32,
    pub always_look: bool,
    pub always_eyes_horizontal: bool,
    pub active: bool,
    pub visible: bool,
    pub cone: f32,
    pub len: f32,
    pub local: Option<Vec3>,
    /// RDR2 (keepHeadAwayFromGround): lean the neck and head away from the ground instead of looking. bodyRollUp
    /// sets it every tick, catchFall once down; it lasts until headLook is reset.
    pub keep_head_away: bool,
    /// RDR2 (twistSpine): the twist the neck and head can't take goes to spine3 and spine2.
    pub twist_spine: bool,
    pub(crate) rd: Rdr2Hl,
}

/// RDR2's per-tick channels and rate limiters (with the RDR2 core).
#[derive(Clone, Debug, Default)]
pub(crate) struct Rdr2Hl {
    /// this tick's target priority (0: none yet); a target is taken at this priority or higher; cleared each tick
    prio: u8,
    /// head/neck and spine-twist speeds (rad/s) with the priority that asked; used at the target's priority
    speed: Option<(u8, f32)>,
    twist_speed: Option<(u8, f32)>,
    /// a re-snapshot asked at this priority
    resnap: Option<u8>,
    /// limiter states and targets: neck and head (lean1, lean2, twist), spine3 and spine2 twist
    neck: [f32; 3],
    head: [f32; 3],
    neck_t: [f32; 3],
    head_t: [f32; 3],
    spine: [f32; 2],
    spine_t: [f32; 2],
    /// the neck/head and the spine twist are being driven
    engaged: bool,
    twisting: bool,
}

/// RDR2's look priorities (headLook's target setter): the highest each tick wins.
pub const PRIO_BODY_BALANCE: u8 = 0xc;
pub const PRIO_SHOT: u8 = 0xd;
pub const PRIO_CATCH_FALL: u8 = 0x14;
pub const PRIO_MESSAGE: u8 = 0x17;
/// RDR2's "at once" speed
const INSTANT: f32 = 1e5;

impl Default for HeadLook {
    fn default() -> HeadLook {
        HeadLook { pos: Vec3::ZERO, vel: Vec3::ZERO, stiffness: delirium_data::kv!(31), damping: 1.0, always_look: false, always_eyes_horizontal: true,
                   active: false, visible: false, cone: 0.0, len: 0.0, local: None, keep_head_away: false, twist_spine: true, rd: Rdr2Hl::default() }
    }
}

fn split(x: f32, a: f32, b: f32, c: f32, d: f32) -> (f32, f32) {
    let (l, h) = (1.0 / (a + c), 1.0 / (b + d));
    if x >= 0.0 { (h * b * x, h * d * x) } else { (l * a * x, l * c * x) }
}

/// RDR2's rate limiter: each axis moves toward its target by at most `rate`·dt.
fn step<const N: usize>(state: &mut [f32; N], target: [f32; N], rate: f32, dt: f32) {
    let most = rate * dt;
    for k in 0..N {
        state[k] += (target[k] - state[k]).clamp(-most, most);
    }
}

impl HeadLook {
    pub fn activate(&mut self, ch: &mut Character, b: &Body) {
        if self.active {
            return;
        }
        self.active = true;
        self.cone = std::f32::consts::FRAC_PI_3;
        self.visible = false;
        self.len = slen(b.at(HEAD()) - b.at(SPINE3()), b"xyz");
        if ch.rdr2 {
            // RDR2: the limiters start where the neck, head and spine twist are, aiming at 0
            self.snapshot(b, true);
            let r = &mut self.rd;
            (r.neck_t, r.head_t, r.spine_t, r.engaged, r.twisting) = ([0.0; 3], [0.0; 3], [0.0; 2], false, false);
            if self.stiffness > 0.0 {
                ch.set_stiffness(self.stiffness, self.damping, "un", None);
            }
            return;
        }
        ch.set_stiffness(self.stiffness, self.damping, "un", None);
    }

    pub fn deactivate(&mut self) {
        if self.active {
            *self = HeadLook::default();
        }
    }

    /// RDR2's target setter: taken when `prio` is at least this tick's (a later caller wins a tie). A target within
    /// 1 cm of the world origin is refused. Returns whether it was taken.
    pub fn look(&mut self, prio: u8, pos: Vec3, vel: Vec3, eyes_horizontal: bool) -> bool {
        let prio = if prio > PRIO_MESSAGE { 0 } else { prio };
        if prio < self.rd.prio || pos.abs().max_element() < 0.01 {
            return false;
        }
        (self.pos, self.vel, self.local, self.always_eyes_horizontal, self.rd.prio) = (pos, vel, None, eyes_horizontal, prio);
        true
    }

    /// RDR2: the neck/head speed (rad/s) this tick, used if `prio` is the target's.
    pub fn speed(&mut self, prio: u8, v: f32) {
        if self.rd.speed.is_none_or(|(p, _)| prio >= p) {
            self.rd.speed = Some((prio, v));
        }
    }

    /// RDR2: the spine-twist speed (rad/s) this tick, used if `prio` is the target's.
    pub fn twist_speed(&mut self, prio: u8, v: f32) {
        if self.rd.twist_speed.is_none_or(|(p, _)| prio >= p) {
            self.rd.twist_speed = Some((prio, v));
        }
    }

    /// RDR2: restart the limiters from the current pose this tick, if `prio` is the target's.
    pub fn resnap(&mut self, prio: u8) {
        if self.rd.resnap.is_none_or(|p| prio >= p) {
            self.rd.resnap = Some(prio);
        }
    }

    fn snapshot(&mut self, b: &Body, spine: bool) {
        let (neck, head) = ((*NECK())[0], (*NECK())[1]);
        (self.rd.neck, self.rd.head) = (b.ch.actual(neck, b.frames), b.ch.actual(head, b.frames));
        if spine {
            let (s3, s2) = ((*SPINE())[3], (*SPINE())[2]);
            self.rd.spine = [b.ch.actual(s3, b.frames)[2], b.ch.actual(s2, b.frames)[2]];
        }
    }

    pub fn tick(&mut self, ch: &mut Character, b: &Body, dt: f32) {
        if !self.active {
            return;
        }
        if ch.rdr2 {
            return self.tick_rdr2(ch, b, dt);
        }
        if let Some(x) = self.local {
            (self.pos, self.vel) = crate::nmbb::from_own(x, b);
        }
        ch.set_stiffness(self.stiffness, self.damping, "un", None);
        let Some((twist, lean1, lean2, over)) = self.solve(ch, b) else {
            return;
        };
        let (neck, head) = ((*NECK())[0], (*NECK())[1]);
        let (nl, hl) = (ch.limits(neck), ch.limits(head));
        let (s2, s3) = ((*SPINE())[2], (*SPINE())[delirium_data::kv!(0 int)]);
        let (l2, l3) = (ch.limits(s2), ch.limits(s3));
        let x3 = over.fma(delirium_data::kv!(8), ch.eff[s3].desired[2]);
        ch.eff[s3].desired[2] = x3.max(l3[delirium_data::kv!(1 int)] * delirium_data::kv!(517)).min(l3[delirium_data::kv!(2 int)] * delirium_data::kv!(517));
        let x2 = over.fma(delirium_data::kv!(140), ch.eff[s2].desired[2]);
        ch.eff[s2].desired[2] = x2.max(l2[delirium_data::kv!(1 int)] * delirium_data::kv!(517)).min(l2[delirium_data::kv!(2 int)] * delirium_data::kv!(517));
        for (n, x, lo, hi) in [(2, twist, delirium_data::kv!(1 int), delirium_data::kv!(2 int)), (0, lean1, 0, 1), (1, lean2, 2, delirium_data::kv!(0 int))] {
            let (xn, xh) = split(x, nl[lo], nl[hi], hl[lo], hl[hi]);
            ch.eff[neck].desired[n] = xn.clamp(-delirium_data::kv!(137), delirium_data::kv!(137));
            ch.eff[head].desired[n] = xh.clamp(-delirium_data::kv!(137), delirium_data::kv!(137));
        }
    }

    /// The look's angles for the neck and head together (twist, lean1, lean2) and the twist they can't take (for
    /// the spine); None when the target is out of sight and alwaysLook is off.
    fn solve(&mut self, ch: &Character, b: &Body) -> Option<(f32, f32, f32, f32)> {
        let (neck, head) = ((*NECK())[0], (*NECK())[1]);
        let strength = (ch.eff[head].strength + ch.eff[neck].strength) * 0.5;
        let damping = (ch.eff[head].damping + ch.eff[neck].damping) * delirium_data::kv!(17);
        let (nl, hl) = (ch.limits(neck), ch.limits(head));
        let sp = b.pmat(SPINE3());
        let m = sp.m;
        let q = q_scaled(m.b * -delirium_data::kv!(516));
        let (a, c) = (qrot(q, m.a, false), qrot(q, m.c, false));
        let h = a.mul_add(Vec3::splat(self.len), b.at(SPINE3()));
        let t = (((self.pos - m.d).vx(sp.w) - sp.v) + self.vel).mul_add(Vec3::splat(damping / strength), self.pos);
        let dir = nrm(t - m.d);
        let x = sdot(dir, a, b"zyx");
        let k = if x > 0.0 { FRAC_PI_2 - self.cone } else { self.cone - FRAC_PI_2 };
        self.visible = sdot(c, dir, b"zyx") < (k.fma(x, self.cone) as f64).cos() as f32;
        if !self.visible && !self.always_look {
            return None;
        }
        let mut n = nrm((t - h) * -1.0);
        let p = nrm(a.vx(n).vx(a));
        let tw = acos(sdot(p, c, b"zyx"));
        let max = (nl[delirium_data::kv!(2 int)] + hl[delirium_data::kv!(2 int)]) * delirium_data::kv!(136);
        let mut over = 0.0;
        if tw > max {
            over = max - tw;
            if sdot(m.b, p, b"zyx") < 0.0 {
                over *= -1.0;
            }
            n = qrot(q_axis(a, -over), n, false);
        }
        let up = ch.up;
        let ua = sdot(up, a, b"zyx");
        let s = if ua * ua <= delirium_data::kv!(17) && !self.always_eyes_horizontal {
            a.vx(n)
        } else if sdot(up, n, b"zyx").abs() < delirium_data::kv!(28) {
            up.vx(n)
        } else {
            (Vec3::ONE - up) * b.pmat(HEAD()).m.a
        };
        let s = nrm(s);
        let w = nrm(n.vx(s));
        let r = q_scaled(nrm(w.vx(a)) * acos(sdot(w, a, b"zyx")));
        let n2 = nrm(qrot(r, n, false));
        let sign = |x: f32| if x < 0.0 { -1.0 } else { 1.0 };
        let twist = acos(sdot(n2, c, b"zyx")) * sign(-sdot(n2, m.b, b"zyx"));
        let u1 = nrm(fneg(w.vx(m.b).vx(m.b)));
        let lean1 = acos(sdot(u1, a, b"zyx")) * sign(-sdot(u1, c, b"zyx"));
        let u2 = nrm(fneg(w.vx(c).vx(c)));
        let lean2 = acos(sdot(u2, a, b"zyx")) * sign(sdot(u2, m.b, b"zyx"));
        Some((twist, lean1, lean2, over))
    }

    /// RDR2's tick: the highest-priority target this tick, through rate limiters (at once unless a speed was asked
    /// at the target's priority); the neck and head each take their share clamped to their own limits; the twist
    /// they can't take goes half each to spine3 and spine2 (within 1.35 their limits). Without a target in sight
    /// the neck and head go back to 0 and stop being written within 0.01 rad. keepHeadAwayFromGround replaces the
    /// look. Every priority, speed and re-snapshot lasts this tick only.
    fn tick_rdr2(&mut self, ch: &mut Character, b: &Body, dt: f32) {
        let (neck, head) = ((*NECK())[0], (*NECK())[1]);
        let prio = self.rd.prio;
        if self.rd.resnap == Some(prio) {
            self.snapshot(b, self.twist_spine);
        }
        if self.stiffness > 0.0 {
            ch.set_stiffness(self.stiffness, self.damping, "un", None);
        }
        if self.keep_head_away {
            // lean away from the ground (RDR2 reads a normal smartFall keeps; ours is up), gain 0.5
            let (m, n) = (b.pmat(NECK_PART()).m, ch.up);
            let (d0, d2) = (m.a.dot(n), m.c.dot(n));
            for j in [neck, head] {
                ch.eff[j].desired[0] = -0.5 * d2;
                ch.eff[j].desired[1] = -0.5 * d0.abs() * d0;
            }
            self.relax_twist(ch);
        } else {
            if let Some(x) = self.local {
                (self.pos, self.vel) = crate::nmbb::from_own(x, b);
            }
            let at = |s: Option<(u8, f32)>| s.filter(|&(p, _)| p == prio).map_or(INSTANT, |(_, v)| v);
            let (rate, twist_rate) = (at(self.rd.speed), at(self.rd.twist_speed));
            let found = if self.local.is_none() && self.pos.abs().max_element() < 0.01 { None } else { self.solve(ch, b) };
            match found {
                None => {
                    self.relax_twist(ch);
                    if self.rd.engaged {
                        (self.rd.neck_t, self.rd.head_t, self.rd.engaged) = ([0.0; 3], [0.0; 3], false);
                        self.snapshot(b, false);
                    }
                    let r = &self.rd;
                    let off = (0..3).any(|k| (r.head_t[k] - r.head[k]).abs() >= 0.01 || (r.neck_t[k] - r.neck[k]).abs() >= 0.01);
                    if off {
                        self.drive(ch, rate, dt);
                    }
                }
                Some((twist, lean1, lean2, over)) => {
                    if !self.rd.engaged {
                        self.snapshot(b, false);
                    }
                    // each angle split by the limits on its side, each share clamped to its own joint's limits;
                    // the neck's share drives the neck (RDR2's solver read as storing it in the head's output: see
                    // the commit, the rig's limits make that a misread)
                    let (nl, hl) = (ch.limits(neck), ch.limits(head));
                    for (n, x) in [(0, lean1), (1, lean2), (2, twist)] {
                        let (lo, hi) = (2 * n, 2 * n + 1);
                        let (xn, xh) = split(x, nl[lo], nl[hi], hl[lo], hl[hi]);
                        self.rd.neck_t[n] = xn.max(nl[lo]).min(nl[hi]);
                        self.rd.head_t[n] = xh.max(hl[lo]).min(hl[hi]);
                    }
                    self.drive(ch, rate, dt);
                    self.rd.engaged = true;
                    if self.twist_spine {
                        self.spine(ch, b, over, twist_rate, dt);
                    } else {
                        self.relax_twist(ch);
                    }
                }
            }
        }
        let r = &mut self.rd;
        (r.prio, r.speed, r.twist_speed, r.resnap) = (0, None, None, None);
    }

    /// Step the neck and head limiters and write their angles.
    fn drive(&mut self, ch: &mut Character, rate: f32, dt: f32) {
        let r = &mut self.rd;
        step(&mut r.neck, r.neck_t, rate, dt);
        step(&mut r.head, r.head_t, rate, dt);
        (ch.eff[(*NECK())[0]].desired, ch.eff[(*NECK())[1]].desired) = (r.neck, r.head);
    }

    /// RDR2's leftover twist: half each to spine3 and spine2 within 1.35 their twist limits, rate limited; back to 0
    /// while there is none.
    fn spine(&mut self, ch: &mut Character, b: &Body, over: f32, rate: f32, dt: f32) {
        let (s3, s2) = ((*SPINE())[3], (*SPINE())[2]);
        if !self.rd.twisting {
            self.rd.spine = [b.ch.actual(s3, b.frames)[2], b.ch.actual(s2, b.frames)[2]];
        }
        if over.abs() <= f32::EPSILON {
            if !self.rd.twisting {
                return;
            }
            self.rd.spine_t = [0.0; 2];
        } else {
            for (k, j) in [s3, s2].into_iter().enumerate() {
                let l = ch.limits(j);
                self.rd.spine_t[k] = (0.5 * over).max(1.35 * l[4]).min(1.35 * l[5]);
            }
            self.rd.twisting = true;
        }
        let r = &mut self.rd;
        step(&mut r.spine, r.spine_t, rate, dt);
        (ch.eff[s3].desired[2], ch.eff[s2].desired[2]) = (r.spine[0], r.spine[1]);
    }

    /// RDR2: let go of the spine twist (written to 0 once).
    fn relax_twist(&mut self, ch: &mut Character) {
        if self.rd.twisting {
            let (s3, s2) = ((*SPINE())[3], (*SPINE())[2]);
            (ch.eff[s3].desired[2], ch.eff[s2].desired[2]) = (0.0, 0.0);
            let r = &mut self.rd;
            (r.spine, r.spine_t, r.twisting) = ([0.0; 2], [0.0; 2], false);
        }
    }
}
