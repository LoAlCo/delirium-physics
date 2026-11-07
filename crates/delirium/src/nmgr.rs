use crate::fneg;
use bevy_math::{Mat3, Quat, Vec3};
use crate::ragdoll::Obstacle;
use crate::Vd;
use crate::Vx;

use crate::nmdbm::Dbm;
use crate::nmhl::HeadLook;
use crate::nmrs::Character;
use crate::nmshot::{limb, Ctx, Limb};
use crate::nmst::SpineTwist;
use crate::Fma;

#[allow(non_snake_case)]
#[inline]
fn SPINE3() -> usize { delirium_data::kv!(81 int) }

#[derive(Clone, Copy, Debug)]
pub struct GrabParams {
    pub justbrace: bool,
    pub linegrab: bool,
    pub surface_grab: bool,
    pub use_right: bool,
    pub use_left: bool,
    pub dont_let_go: bool,
    pub pos: [Vec3; 4],
    pub normal_r: Vec3,
    pub normal_l: Vec3,
    pub grab_strength: f32,
    pub arm_stiffness: f32,
    pub grab_dist: f32,
    pub reach_angle: f32,
    pub one_side_reach_angle: f32,
    pub body_stiffness: f32,
    pub pull_up_time: f32,
    pub pull_up_strength: [f32; 2],
    pub grab_hold_max_timer: f32,
    pub max_reach: f32,
    pub hold_stiffness: f32,
    pub instance: i32,
    pub part: i32,
    /// RDR2 core only (rdr2_grab.md §2): a hand holding a weapon may still grab (and asks to drop it within this)
    pub drop_weapon: bool,
    pub drop_weapon_distance: f32,
    /// the hold's hand pull strength
    pub sticky_hands: f32,
    /// the balancer's heading: 0 none, 1 toward the grab, 2 away
    pub turn_to_target: i32,
    /// the hands keep colliding while they reach
    pub hands_collide: bool,
    /// pos3/pos4 (with normalR2/normalL2) are a second pair, taken when nearer the chest
    pub points_x4: bool,
    pub normal_r2: Vec3,
    pub normal_l2: Vec3,
    /// the held hand's wrist turn torque gain; a hand let go past this wrist angle
    pub orientation_scale: f32,
    pub max_wrist_angle: f32,
    /// with a hand held: headLook and spineTwist at targetForHeadLook, and the one-hand pull-up
    pub head_look_to_target: bool,
    pub look_at_grab: bool,
    pub head_look_target: Vec3,
}

impl Default for GrabParams {
    fn default() -> GrabParams {
        GrabParams { justbrace: false, linegrab: false, surface_grab: false, use_right: false, use_left: false,
                     dont_let_go: false, pos: [Vec3::ZERO; 4], normal_r: Vec3::ZERO, normal_l: Vec3::ZERO, grab_strength: -1.0,
                     arm_stiffness: delirium_data::kv!(126), grab_dist: 1.0, reach_angle: delirium_data::kv!(178), one_side_reach_angle: delirium_data::kv!(179), body_stiffness: delirium_data::kv!(117),
                     pull_up_time: 1.0, pull_up_strength: [0.0; 2], grab_hold_max_timer: delirium_data::kv!(26), max_reach: delirium_data::kv!(74),
                     hold_stiffness: delirium_data::kv!(84), instance: -1, part: 0, drop_weapon: false, drop_weapon_distance: 0.3,
                     sticky_hands: 4.0, turn_to_target: 1, hands_collide: false, points_x4: false, normal_r2: Vec3::ZERO, normal_l2: Vec3::ZERO,
                     orientation_scale: 1.0, max_wrist_angle: std::f32::consts::PI, head_look_to_target: false, look_at_grab: true,
                     head_look_target: Vec3::ZERO }
    }
}

impl GrabParams {
    /// RDR2's defaults (its start reloads them; rdr2_grab.md §2).
    pub fn rdr2() -> GrabParams {
        GrabParams { arm_stiffness: 14.0, grab_dist: 1.0, reach_angle: 2.8, one_side_reach_angle: 1.4, body_stiffness: 11.0,
                     pull_up_time: 1.0, pull_up_strength: [0.0; 2], grab_hold_max_timer: 100.0, max_reach: 0.7, grab_strength: -1.0,
                     ..GrabParams::default() }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Grip {
    pub broken: bool,
    pub local: Vec3,
    pub at: Option<Vec3>,
    pub to: Vec3,
    pub len: f32,
    on: Vec3,
}

#[derive(Clone, Debug)]
pub struct Grab {
    pub p: GrabParams,
    pub active: bool,
    pub grip: [Option<Grip>; 2],
    older: usize,
    target: [Vec3; 2],
    normal: [Vec3; 2],
    held: [f32; 2],
    len: [f32; 2],
    told: [bool; 2],
    grabbing: bool,
    look_ahead: bool,
    frame: [Vec3; 4],
    pub contact_off: u32,
    /// RDR2 core: grab's own state (rdr2_grab.md §3)
    r: R2,
}

impl Default for Grab {
    fn default() -> Grab {
        Grab { p: GrabParams::default(), active: false, grip: [None; 2], older: 0, target: [Vec3::ZERO; 2], normal: [Vec3::ZERO; 2],
               held: [0.0; 2], len: [0.0; 2], told: [false; 2], grabbing: false, look_ahead: true, frame: ID, contact_off: 0, r: R2::default() }
    }
}

fn on_line(a: Vec3, b: Vec3, x: Vec3) -> Vec3 {
    let d = a - b;
    let n = crate::rigid::nrm(d);
    let len = crate::nmdbm::slen(d, b"yzx");
    let t = crate::nmdbm::sdot(x - b, n, b"yzx");
    let t = if t >= 0.0 { t } else { 0.0 };
    let t = if t - len >= 0.0 { len } else { t };
    n.mul_add(Vec3::splat(t), b)
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    let k = |p: f32, q: f32, r: f32, s: f32| fneg(p.mul_add(q, -(r * s)));
    Vec3::new(k(a.z, b.y, a.y, b.z), k(a.x, b.z, a.z, b.x), k(a.y, b.x, a.x, b.y))
}

fn on_quad(q: &[Vec3; 4], justbrace: bool, x: Vec3, edge: f32) -> (Vec3, Vec3) {
    use crate::nmdbm::{sdot, slen};
    use crate::rigid::nrm;
    let e = [on_line(q[0], q[1], x), on_line(q[1], q[2], x), on_line(q[2], q[delirium_data::kv!(0 int)], x), on_line(q[delirium_data::kv!(0 int)], q[0], x)];
    let n = cross(nrm(q[0] - q[1]), nrm(q[2] - q[1])) * -1.0;
    let rel = x - q[0];
    let h = sdot(n, rel, b"zxy");
    let p = (rel - n * h) + q[0];
    let d = [slen(p - e[0], b"xzy"), slen(p - e[1], b"yzx"), slen(p - e[2], b"yzx"), slen(p - e[delirium_data::kv!(0 int)], b"yzx")];
    let (mut best, mut at, mut ends) = (d[0], e[0], (q[0], q[1]));
    for (k, ab) in [(1, (q[1], q[2])), (2, (q[2], q[delirium_data::kv!(0 int)])), (delirium_data::kv!(0 int), (q[delirium_data::kv!(0 int)], q[0]))] {
        if d[k] < best {
            (best, at, ends) = (d[k], e[k], ab);
        }
    }
    let u = nrm(ends.1 - ends.0);
    let w = nrm(at - p);
    let out = sdot(cross(w, u), n, b"zxy") < 0.0;
    let p = if out || (!justbrace && best < edge) { at } else { p };
    (p, if h < 0.0 { n * -1.0 } else { n })
}

fn apart(this: &mut Vec3, other: Vec3, a: Vec3, b: Vec3) {
    let u = crate::rigid::nrm(other - *this);
    let end = if crate::nmdbm::sdot(crate::rigid::nrm(a - b), u, b"zyx") > 0.0 { b } else { a };
    let len = crate::nmdbm::slen(other - end, b"yzx");
    *this -= u * if len - delirium_data::kv!(9) >= 0.0 { delirium_data::kv!(9) } else { len };
}

const ID: [Vec3; 4] = [Vec3::X, Vec3::Y, Vec3::Z, Vec3::ZERO];

impl Grab {
    pub fn place(&mut self, obstacles: &[Obstacle]) {
        let (at, m) = Obstacle::find(obstacles, self.p.instance).map_or((Vec3::ZERO, Mat3::IDENTITY), |o| match self.p.part {
            k @ 1.. => o.component(k as usize),
            _ => (o.at, o.turn.unwrap_or_else(|| Mat3::from_quat(o.rot))),
        });
        self.frame = [m.x_axis, m.y_axis, m.z_axis, at];
        for i in 0..2 {
            if let Some(on) = self.grip[i].map(|g| g.on) {
                let to = self.to_world(on);
                self.grip[i].as_mut().unwrap().to = to;
            }
        }
    }

    fn to_world(&self, x: Vec3) -> Vec3 {
        let [a, b, c, d] = self.frame;
        crate::vm4(a, b, c, d, x)
    }

    pub(crate) fn to_local(&self, x: Vec3) -> Vec3 {
        let f = self.frame;
        let r = |v: Vec3, w: f32| [v.x, v.y, v.z, w];
        let [a, b, c, d] = delirium_data::lift::inverse_affine([r(f[0], 0.0), r(f[1], 0.0), r(f[2], 0.0), r(f[delirium_data::kv!(0 int)], 1.0)])
            .map_or(f, |m| m.map(Vec3::from));
        crate::vm4(a, b, c, d, x)
    }

    fn dir_world(&self, n: Vec3) -> Vec3 {
        let [a, b, c, _] = self.frame;
        crate::vm4(a, b, c, Vec3::ZERO, n)
    }

    pub fn hold_points(&mut self, f: impl Fn(usize, Vec3) -> Vec3) {
        for (i, g) in self.grip.iter_mut().enumerate() {
            if let Some(g) = g.as_mut() {
                if let Some(at) = g.at.take() {
                    g.local = f(limb(i).p[delirium_data::kv!(0 int)], at);
                }
            }
        }
    }

    pub fn grips(&self) -> Vec<(usize, Vec3, Vec3, f32, f32)> {
        [self.older, 1 - self.older].into_iter().filter_map(|i| self.grip[i].filter(|g| !g.broken).map(|g| (limb(i).p[delirium_data::kv!(0 int)], g.local, g.to, g.len, self.p.grab_strength))).collect()
    }

    pub fn snapped(&mut self, parts: &[usize]) {
        for i in 0..2 {
            if let Some(g) = self.grip[i].as_mut().filter(|_| parts.contains(&limb(i).p[delirium_data::kv!(0 int)])) {
                g.broken = true;
            }
        }
    }

    pub fn activate(&mut self, c: &mut Ctx, head: &mut HeadLook, st: &mut SpineTwist, dbm: Option<&mut Dbm>) {
        if self.active {
            return;
        }
        if c.ch.rdr2 {
            return self.activate_rdr2(c, head, st, dbm);
        }
        (self.active, self.grip, self.look_ahead) = (true, [None; 2], true);
        for (i, on) in [(0, self.p.use_left), (1, self.p.use_right)] {
            if on && !self.p.justbrace {
                self.contact_off |= 1 << limb(i).p[delirium_data::kv!(0 int)];
            }
        }
        self.tick(c, head, st, dbm, 0.0);
    }

    pub fn deactivate(&mut self, ch: &mut Character, head: &mut HeadLook, st: &mut SpineTwist, dbm: Option<&mut Dbm>) {
        if !self.active {
            return;
        }
        if ch.rdr2 {
            return self.deactivate_rdr2(ch, head, st, dbm);
        }
        head.deactivate();
        st.deactivate();
        if let Some(d) = dbm {
            (d.pv.fixed_vel, d.pv.vel) = (false, self.target[1]);
        }
        *self = Grab::default();
        ch.set_stiffness(delirium_data::kv!(31), 1.0, "fb", Some(1.0));
    }

    pub fn tick(&mut self, c: &mut Ctx, head: &mut HeadLook, st: &mut SpineTwist, dbm: Option<&mut Dbm>, dt: f32) {
        if !self.active {
            return;
        }
        if c.ch.rdr2 {
            return self.tick_rdr2(c, head, st, dbm, dt);
        }
        let p = self.p;
        let (mut left, mut right) = (p.use_left, p.use_right);
        if left || right {
            self.decide(c, &mut left, &mut right);
        } else {
            self.grabbing = false;
        }
        if self.grabbing && (left || right) {
            for (i, on, a, b, n) in [(1, right, p.pos[0], p.pos[1], p.normal_r), (0, left, p.pos[1], p.pos[0], p.normal_l)] {
                if on && self.grip[i].is_none() {
                    let l = limb(i);
                    (self.target[i], self.normal[i]) = self.aim(c, a, b, n, self.to_local(c.b.part(l.p[delirium_data::kv!(0 int)]).m.d), c.b.joint_pos(l.e[1]));
                }
            }
            if p.linegrab && (self.target[0] - self.target[1]).length() < delirium_data::kv!(9) {
                let (l, r) = (self.target[0], self.target[1]);
                if left && self.grip[0].is_none() {
                    apart(&mut self.target[0], r, p.pos[0], p.pos[1]);
                } else if right && self.grip[1].is_none() {
                    apart(&mut self.target[1], l, p.pos[1], p.pos[0]);
                }
            }
            let w = [0, 1].map(|i| (self.to_world(self.target[i]), self.dir_world(self.normal[i])));
            for (i, on) in [(0, left), (1, right)] {
                if on && !p.justbrace {
                    self.contact_off |= 1 << limb(i).p[delirium_data::kv!(0 int)];
                }
                if on {
                    self.arm(c, i, w[i], ["ul", "ur"][i], [-1.0, 1.0][i], dt);
                }
            }
            let mid = match (left, right) {
                (true, true) => (w[0].0 + w[1].0) * 0.5,
                (true, false) => w[0].0,
                _ => w[1].0,
            };
            let t = self.held[0].max(self.held[1]);
            if t > delirium_data::kv!(486) {
                self.look_off(c, head);
            } else if t < delirium_data::kv!(68) || head.active {
                if !head.active {
                    head.activate(c.ch, c.b);
                }
                (head.pos, head.local, head.always_look, head.stiffness, head.damping) = (mid, None, true, p.body_stiffness, delirium_data::kv!(136));
                head.vel = Vec3::ZERO;
            }
            if !st.active {
                st.activate(c.ch);
            }
            (st.pos, st.stiffness, st.always, st.arms) = (mid, p.body_stiffness, true, false);
            if if p.justbrace { left } else { left && right } {
                if let Some(d) = dbm {
                    let v = mid - c.com;
                    (d.pv.fixed_vel, d.pv.vel) = (true, v * delirium_data::lift::vrsq(v.vd(v)));
                }
            }
        } else {
            self.look_off(c, head);
            st.deactivate();
            c.out.send(delirium_data::kv!(127 int), "GrabNotGrabbing", 0.0);
            if let Some(d) = dbm {
                (d.pv.fixed_vel, d.pv.vel) = (false, self.target[1]);
            }
        }
        for (i, on) in [(1, right), (0, left)] {
            if !on && self.grip[i].take().is_some() && p.grab_hold_max_timer > 0.0 {
                self.held[i] = p.grab_hold_max_timer + delirium_data::kv!(60);
            }
            if !on {
                self.contact_off &= !(1 << limb(i).p[delirium_data::kv!(0 int)]);
            }
        }
        if !p.use_left {
            self.held[0] = 0.0;
        }
        if !p.use_right {
            self.held[1] = 0.0;
        }
    }

    fn look_off(&mut self, c: &Ctx, head: &mut HeadLook) {
        if self.look_ahead {
            self.look_ahead = false;
            let m = c.b.pmat(SPINE3()).m;
            head.local = Some(crate::nmbb::to_own((m.b * delirium_data::kv!(116) - m.c) + c.b.at(SPINE3()), c.b));
        }
    }

    fn decide(&mut self, c: &Ctx, left: &mut bool, right: &mut bool) {
        let p = self.p;
        let s3 = c.b.part(SPINE3()).m;
        let at = self.to_local(s3.d);
        let dist = if p.justbrace { p.grab_dist } else { p.grab_dist * delirium_data::kv!(13) };
        let g = if p.linegrab {
            on_line(p.pos[0], p.pos[1], at)
        } else if p.surface_grab {
            on_quad(&p.pos, p.justbrace, at, 0.0).0
        } else {
            match (p.use_left, p.use_right) {
                (true, true) => (p.pos[0] + p.pos[1]) * 0.5,
                (true, false) => p.pos[1],
                (false, true) => p.pos[0],
                _ => at,
            }
        };
        let (g, at) = (self.to_world(g), s3.d);
        let k = if p.justbrace { delirium_data::kv!(505) } else { delirium_data::kv!(10) };
        let d = g - at;
        let len = d.length();
        let closing = (c.vel.vd(d.normalize_or_zero()) * k).max(0.0).min(len);
        let left_over = len - closing;
        self.grabbing = left_over < if self.grabbing { dist + delirium_data::kv!(60) } else { dist };
        let v = g - s3.d;
        let local = Vec3::new(s3.a.vd(v), s3.b.vd(v), s3.c.vd(v));
        let ang = local.y.atan2(-local.z);
        if ang < p.reach_angle && ang > -p.reach_angle {
            *right = if ang > p.reach_angle - p.one_side_reach_angle { false } else { p.use_right };
            *left = if ang < p.one_side_reach_angle - p.reach_angle { false } else { p.use_left };
        } else {
            (self.grabbing, *left, *right) = (false, false, false);
        }
        if p.dont_let_go && self.grip.iter().any(Option::is_some) {
            (self.grabbing, *right, *left) = (true, p.use_right, p.use_left);
        }
        if p.grab_hold_max_timer > 0.0 {
            if self.held[1] > p.grab_hold_max_timer {
                *right = false;
            }
            if self.held[0] > p.grab_hold_max_timer {
                *left = false;
            }
        }
        *left &= !c.ch.busy[0];
        *right &= !c.ch.busy[1];
    }

    fn aim(&self, _c: &Ctx, a: Vec3, b: Vec3, normal: Vec3, hand: Vec3, shoulder: Vec3) -> (Vec3, Vec3) {
        let p = self.p;
        let none = normal == Vec3::ZERO;
        let shoulder = self.to_local(shoulder);
        let (mut t, mut n) = (a, crate::rigid::nrm(if none { shoulder - a } else { normal }));
        if p.linegrab {
            t = on_line(a, b, hand);
            if none {
                let d = crate::rigid::nrm(a - b);
                n = fneg(crate::rigid::nrm(shoulder - t).vx(d).vx(d));
            }
        }
        if p.surface_grab {
            (t, n) = on_quad(&p.pos, p.justbrace, hand, delirium_data::kv!(20));
        }
        (t, n)
    }

    fn arm(&mut self, c: &mut Ctx, i: usize, (w, n): (Vec3, Vec3), mask: &str, side: f32, dt: f32) {
        let p = self.p;
        let l = limb(i);
        let pull = p.pull_up_strength[i] * delirium_data::kv!(5);
        let speed = crate::nmdbm::slen(c.vel, b"yxz");
        let dir = crate::rigid::nrm(c.vel);
        let at = c.b.joint_pos(l.e[1]);
        let k = (speed.clamp(delirium_data::kv!(64), delirium_data::kv!(25)) - delirium_data::kv!(64)) * (1.0 / delirium_data::kv!(30));
        let wc = w - c.com;
        let mut t = c.com + dir.mul_add(Vec3::splat(-((crate::nmdbm::sdot(dir, wc, b"yxz") * k) * -0.5)), wc);
        let all: Vec<usize> = (0..c.b.build.parts.len()).collect();
        let touching = c.b.doll.collided(&all);
        if !touching {
            t = c.up.mul_add(Vec3::splat(k), t);
        }
        let a = speed.clamp(0.0, 1.0);
        let reach = crate::nmdbm::slen(t - at, b"xzy").fma(1.0 - (1.0 - a), p.max_reach.clamp(delirium_data::kv!(78), delirium_data::kv!(5)) * (1.0 - a));
        let t = crate::rigid::nrm(t - at).mul_add(Vec3::splat(reach), at);
        let yaw = reach.clamp(0.0, delirium_data::kv!(506));
        let ik_side = if i == 0 { 1.0 } else { -1.0 };
        if self.grip[i].is_none() {
            c.ch.set_stiffness(p.arm_stiffness, 1.0, mask, Some(1.0));
            crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[delirium_data::kv!(0 int)], t, yaw, delirium_data::kv!(18), ik_side, 1.0, Vec3::ZERO, None);
            crate::nmpa::share(c.ch, c.b.frames, l.e[0], l.e[1]);
            let hand = c.b.at(l.p[delirium_data::kv!(0 int)]);
            if crate::nmdbm::sdot(n, crate::rigid::nrm(hand - at), b"zyx") > delirium_data::kv!(124) {
                c.ch.eff[l.e[delirium_data::kv!(0 int)]].desired[2] = 0.0;
                c.ch.eff[l.e[delirium_data::kv!(0 int)]].desired[0] = 0.0;
            } else {
                crate::nmshot::Shot::wrist(c, l, t, n, ik_side);
            }
            let strength = if p.justbrace { 0.0 } else { delirium_data::kv!(84) };
            c.ch.set_relative(l.e[delirium_data::kv!(0 int)], 1, -1.0);
            self.pull(c, i, l, w, side, strength, !p.justbrace, dt);
            return;
        }
        self.held[i] += dt;
        let mut arm_k = p.hold_stiffness;
        if touching {
            arm_k *= delirium_data::kv!(255);
        }
        let top = p.pull_up_strength[i] * delirium_data::kv!(137);
        let s = (top / (p.pull_up_time + delirium_data::kv!(51)) * self.held[i] + delirium_data::kv!(64)).max(delirium_data::kv!(64)).min(top);
        if !self.told[i] {
            self.told[i] = true;
            c.out.send(delirium_data::kv!(55 int), ["LeftArmGrab", "RightArmGrab"][i], 0.0);
        }
        c.ch.set_stiffness(s, 0.5, mask, Some(delirium_data::kv!(32)));
        let wr = &mut c.ch.eff[l.e[delirium_data::kv!(0 int)]];
        (wr.muscle_stiffness, wr.strength, wr.damping) = (delirium_data::kv!(25), delirium_data::kv!(507), delirium_data::kv!(136));
        self.pull(c, i, l, w, side, arm_k, !p.justbrace, dt);
        for j in [l.e[delirium_data::kv!(0 int)], l.e[1]] {
            c.ch.eff[j].desired = c.ch.actual(j, c.b.frames);
        }
        self.turn_hand(c, l, n, side, dt);
        if pull > 0.0 {
            c.ch.eff[l.e[2]].desired[0] = pull;
            c.ch.eff[l.e[1]].desired[0] = (pull * delirium_data::kv!(74)).clamp(0.0, 2.0);
            c.ch.eff[l.e[0]].desired[0] = pull * delirium_data::kv!(60);
        }
        let legs = |k: usize| c.b.doll.collided(&limb(k).p[1..]);
        let trunk = if !legs(2) && !legs(delirium_data::kv!(0 int)) {
            c.ch.set_stiffness(delirium_data::kv!(84), 1.0, "us", Some(1.0));
            delirium_data::kv!(32)
        } else {
            c.ch.set_stiffness(delirium_data::kv!(31), 1.0, "us", Some(1.0));
            delirium_data::kv!(31)
        };
        c.ch.set_stiffness(trunk, 1.0, "uc", Some(1.0));
    }

    #[allow(clippy::too_many_arguments)]
    fn pull(&mut self, c: &mut Ctx, i: usize, l: Limb, w: Vec3, side: f32, strength: f32, can_grip: bool, dt: f32) {
        #[allow(non_snake_case)]
        #[inline]
        fn DIST() -> f32 { delirium_data::kv!(60) }
        let hand = c.b.part(l.p[delirium_data::kv!(0 int)]);
        let at = hand.m.d - hand.m.a * (side * delirium_data::kv!(118));
        let g = w - at;
        let d = g.y.mul_add(g.y, g.z.mul_add(g.z, g.x * g.x)).sqrt();
        if d < delirium_data::kv!(5) * DIST() && self.grip[i].is_none() {
            let k = ((delirium_data::kv!(5) * DIST() - d) / (2.0 * DIST())).clamp(0.0, 1.0) * strength;
            let mut imp = (Vec3::ZERO - hand.v).mul_add(Vec3::splat(k * delirium_data::kv!(60)), (w - at) * (k * k));
            if dt < 1.0 / delirium_data::kv!(103) {
                imp *= dt * delirium_data::kv!(103);
            }
            c.out.impulse.push((l.p[delirium_data::kv!(0 int)], c.b.joint_pos(l.e[delirium_data::kv!(0 int)]), imp));
        }
        if can_grip && d < DIST() && self.grip[i].is_none() {
            self.len[i] = d;
            if self.grip[1 - i].is_none() {
                self.older = i;
            }
            self.grip[i] = Some(Grip { broken: false, local: Vec3::ZERO, at: Some(at), to: w, len: d, on: crate::rigid::grip_local(self.frame, None, w) });
        } else if let Some(g) = self.grip[i].as_mut() {
            self.len[i] = (self.len[i] - dt * delirium_data::kv!(119)).max(0.0).min(d);
            g.len = self.len[i];
        }
    }

    fn turn_hand(&self, c: &mut Ctx, l: Limb, n: Vec3, k: f32, dt: f32) {
        let a = c.b.part(l.p[delirium_data::kv!(0 int)]).m.a * k;
        let cos = n.x.mul_add(a.x, n.y.mul_add(a.y, n.z * a.z));
        let ang = if cos <= -1.0 { std::f32::consts::PI } else if cos >= 1.0 { 0.0 } else { delirium_data::mathrt::acos(cos as f64) as f32 };
        if let Some(axis) = unit_axis(a.vx(n)).filter(|_| dt > 0.0) {
            c.out.torque(l.p[delirium_data::kv!(0 int)], axis * (ang / (dt * delirium_data::kv!(103))));
        }
    }
}

fn unit_axis(v: Vec3) -> Option<Vec3> {
    (v.vd(v) > delirium_data::kv!(50)).then(|| crate::rigid::nrm(v))
}

// ---------------------------------------------------------------------------------------------------------------
// RDR2 core: grab from RDR2's code (rdr2-decomp/study/rdr2_grab.md). Arms 0 left, 1 right; points and normals in
// the grabbed instance's frame (Delirium's instanceIndex for RDR2's boundhandle).

/// RDR2's stiffness scale (ctx+0x2c64): 1 here.
const S: f32 = 1.0;
/// RDR2's look priority for grab's headLook target
const PRIO_GRAB: u8 = 10;
/// The surface grab's edge snap (RDR2's value wasn't recovered): GTA IV's.
#[allow(non_snake_case)]
#[inline]
fn SNAP() -> f32 { delirium_data::kv!(20) }

#[derive(Clone, Copy, Debug, Default)]
struct R2 {
    /// activated under the core
    on: bool,
    /// per arm: the grab point and normal this tick (+0x150..+0x180), and the hand's anchor (+0x190/+0x1a0)
    point: [Vec3; 2],
    normal: [Vec3; 2],
    anchor: [Vec3; 2],
    /// hold timers (+0x1b0/+0x1b4) and the times after which a hand may grip again (+0x1b8/+0x1bc)
    hold: [f32; 2],
    regrab: [f32; 2],
    /// the behaviour's clock, this tick's reach limit, the one-hand hang timer
    clock: f32,
    reach: f32,
    hang: f32,
    /// a hand is held (edge-detected for feedback), grabbing last tick, the first tick, the pull-up latched
    held: [bool; 2],
    grabbing: bool,
    first: bool,
    latched: bool,
    /// the balancer's heading is ours (owner 6)
    turning: bool,
}

fn unit(v: Vec3) -> Vec3 {
    v.normalize_or_zero()
}

impl Grab {
    fn activate_rdr2(&mut self, c: &mut Ctx, head: &mut HeadLook, st: &mut SpineTwist, dbm: Option<&mut Dbm>) {
        (self.active, self.grip) = (true, [None; 2]);
        self.r = R2 { on: true, first: true, reach: 0.7, ..R2::default() };
        self.tick_rdr2(c, head, st, dbm, 0.0);
    }

    /// RDR2's release and deactivate (§6): both grips go, the hands collide again, headLook and spineTwist stop,
    /// the balancer's heading is let go, arms and spine at 10.
    fn deactivate_rdr2(&mut self, ch: &mut Character, head: &mut HeadLook, st: &mut SpineTwist, dbm: Option<&mut Dbm>) {
        head.deactivate();
        st.deactivate();
        if let Some(d) = dbm.filter(|_| self.r.turning) {
            d.pv.fixed_vel = false;
        }
        *self = Grab::default();
        ch.set_stiffness(10.0 * S, 1.0, "ua", Some(1.0));
        ch.set_stiffness(10.0 * S, 1.0, "us", Some(1.0));
    }

    fn tick_rdr2(&mut self, c: &mut Ctx, head: &mut HeadLook, st: &mut SpineTwist, dbm: Option<&mut Dbm>, dt: f32) {
        if !self.r.on {
            self.r = R2 { on: true, first: true, reach: 0.7, ..R2::default() };
        }
        let p = self.p;
        self.r.reach = p.max_reach;
        // §4.1 which arms
        let (mut l, mut r) = (p.use_left, p.use_right);
        if l || r {
            let both = self.grip[0].is_some() && self.grip[1].is_some();
            if !(p.dont_let_go && both && l && r) {
                self.decide_rdr2(c, &mut l, &mut r);
            }
            if p.dont_let_go {
                l |= self.grip[0].is_some();
                r |= self.grip[1].is_some();
            }
        }
        if p.grab_hold_max_timer > 0.0 {
            r &= p.grab_hold_max_timer >= self.r.hold[1];
            l &= p.grab_hold_max_timer >= self.r.hold[0];
        }
        if !l && !r {
            if self.r.grabbing {
                self.look_off_rdr2(c, head);
                st.deactivate();
                self.grip = [None; 2];
                self.r.held = [false; 2];
            }
            self.r.grabbing = false;
            c.out.send(EVENT(), "GrabNotGrabbing", 0.0);
            return self.end_rdr2(c, dt);
        }
        self.r.grabbing = true;
        // §4.2 the grab point and normal of each arm
        let chest = self.to_local(c.b.at(SPINE3()));
        let second = p.points_x4 && (chest - p.pos[2]).length_squared() <= (chest - p.pos[0]).length_squared();
        for (i, on) in [(1, r), (0, l)] {
            if !on {
                continue;
            }
            let shoulder = self.to_local(c.b.joint_pos(limb(i).e[1]));
            let (a, b, n) = match (i, second) {
                (1, false) => (p.pos[0], p.pos[1], p.normal_r),
                (1, true) => (p.pos[2], p.pos[3], p.normal_r2),
                (_, false) => (p.pos[1], p.pos[0], p.normal_l),
                (_, true) => (p.pos[3], p.pos[2], p.normal_l2),
            };
            (self.r.point[i], self.r.normal[i]) = self.resolve_rdr2(a, b, n, shoulder);
        }
        if p.linegrab && (self.r.point[0] - self.r.point[1]).length() < 0.15 {
            let (lp, rp) = (self.r.point[0], self.r.point[1]);
            if self.grip[0].is_none() {
                self.r.point[0] = apart_rdr2(lp, rp, p.pos[0], p.pos[1]);
            } else if self.grip[1].is_none() {
                self.r.point[1] = apart_rdr2(rp, lp, p.pos[0], p.pos[1]);
            }
        }
        // §4.4 the weapon drop: the game drops it on this feedback
        let w = [0, 1].map(|i| self.to_world(self.r.point[i]));
        let hand = [0, 1].map(|i| c.b.at(limb(i).p[3]));
        if self.grip.iter().all(Option::is_none) && p.drop_weapon {
            let d = [0, 1].map(|i| (hand[i] - w[i]).length());
            if c.ch.busy[0] && l && d[0] < p.drop_weapon_distance && p.drop_weapon_distance < d[1] {
                c.out.send(EVENT(), "grabDropLeftWeapon", 0.0);
            }
            if c.ch.busy[1] && r && d[1] < p.drop_weapon_distance && p.drop_weapon_distance < d[0] {
                c.out.send(EVENT(), "grabDropRightWeapon", 0.0);
            }
        }
        // §4.5 the arms, left first
        let mut target = [Vec3::ZERO; 2];
        for (i, on) in [(0, l), (1, r)] {
            let ll = limb(i);
            if !on {
                if self.grip[i].take().is_some() || self.r.held[i] {
                    c.out.send(FAILURE(), ["LeftArmGrab", "RightArmGrab"][i], 0.0);
                }
                self.r.held[i] = false;
                if p.grab_hold_max_timer > 0.0 {
                    self.r.hold[i] = p.grab_hold_max_timer + 0.3;
                }
                self.contact_off &= !((1 << ll.p[3]) | (1 << ll.p[2]));
                continue;
            }
            if !p.justbrace && !p.hands_collide {
                self.contact_off |= 1 << ll.p[3];
            }
            if self.grip[i].is_none() {
                self.r.anchor[i] = self.r.point[i];
            }
            target[i] = self.to_world(self.r.anchor[i]);
            let may = if i == 0 { !((r && self.crossed(c)) || self.r.clock < self.r.regrab[0]) } else { self.r.regrab[1] <= self.r.clock };
            let n = self.dir_world(self.r.normal[i]);
            self.arm_rdr2(c, i, target[i], n, may, dt);
        }
        if !p.use_left {
            self.r.hold[0] = 0.0;
        }
        if !p.use_right {
            self.r.hold[1] = 0.0;
        }
        // §4.6 a broken grip is let go, and that hand may not grip again for a second (no self-grabs here)
        for i in 0..2 {
            if self.grip[i].is_some_and(|g| g.broken) {
                self.grip[i] = None;
                self.r.held[i] = false;
                self.r.regrab[i] = self.r.clock + 1.0;
            }
        }
        // §4.7 head look, spine twist, pull-up
        let t = match (l, r) {
            (true, true) => (target[0] + target[1]) * 0.5,
            (true, false) => target[0],
            _ => target[1],
        };
        let held = self.r.held;
        if !p.head_look_to_target || !(held[0] || held[1]) {
            let most = self.r.hold[0].max(self.r.hold[1]);
            if most > 1.6 {
                self.look_off_rdr2(c, head);
            } else if p.look_at_grab && (most < 0.05 || head.active) {
                look_on(c, head, t, p.body_stiffness);
            }
            twist_on(c.ch, st, t, p.head_look_to_target);
        } else {
            look_on(c, head, p.head_look_target, p.body_stiffness);
            c.ch.set_stiffness(p.body_stiffness, 1.0, "us", None);
            twist_on(c.ch, st, p.head_look_target, p.head_look_to_target);
            if held[0] != held[1] {
                self.pull_up(c, if held[0] { 0 } else { 1 }, t, dt);
            }
        }
        // §4.8 the balancer's heading (owner 6)
        if p.turn_to_target != 0 {
            if let Some(d) = dbm {
                let dir = unit(t - c.com) * if p.turn_to_target == 2 { -1.0 } else { 1.0 };
                (d.pv.fixed_vel, d.pv.vel) = (true, dir * 0.5);
                self.r.turning = true;
            }
        }
        self.end_rdr2(c, dt);
    }

    /// §4.9: the clock, and both wrists' muscle stiffness.
    fn end_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        self.r.clock += dt;
        self.r.first = false;
        for i in 0..2 {
            c.ch.eff[limb(i).e[3]].muscle_stiffness = 2.0;
        }
    }

    /// RDR2's headLook off: only on the first tick, a look ahead of the chest (in his own frame).
    fn look_off_rdr2(&mut self, c: &Ctx, head: &mut HeadLook) {
        if self.r.first {
            let at = c.b.at(SPINE3()) + c.m.b * 0.4 - c.m.c;
            if head.look(PRIO_GRAB, at, Vec3::ZERO, true) {
                head.local = Some(crate::nmbb::to_own(at, c.b));
            }
        }
    }

    /// §4.2 (0x2ae29d0): an arm's grab point and normal from its pair, facing the shoulder by default.
    fn resolve_rdr2(&self, a: Vec3, b: Vec3, n: Vec3, shoulder: Vec3) -> (Vec3, Vec3) {
        let p = self.p;
        let none = n == Vec3::ZERO;
        let n0 = unit(if none { shoulder - a } else { n });
        if p.linegrab {
            let t = on_line(a, b, shoulder);
            let d = unit(a - b);
            return (t, if none { -n0.cross(d).cross(d) } else { n0 });
        }
        if p.surface_grab {
            return on_quad(&p.pos, p.justbrace, shoulder, SNAP());
        }
        (a, n0)
    }

    /// §4.3 (0x2ac88b8): which arms reach, by distance (less the approach) and the angle across the chest.
    fn decide_rdr2(&self, c: &Ctx, left: &mut bool, right: &mut bool) {
        let p = self.p;
        let s3 = c.b.pmat(SPINE3()).m;
        let cl = self.to_local(s3.d);
        let d2 = |x: Vec3, q: Vec3| (q - x).length_squared();
        let (pr, pl) = if p.linegrab {
            let q = on_line(p.pos[0], p.pos[1], cl);
            (q, q)
        } else if p.surface_grab {
            let q = on_quad(&p.pos, p.justbrace, cl, SNAP()).0;
            (q, q)
        } else if p.use_left && p.use_right {
            let pr = if p.points_x4 && d2(p.pos[2], cl) < d2(p.pos[0], cl) { p.pos[2] } else { p.pos[0] };
            let pl = if p.points_x4 && d2(p.pos[3], pr) < d2(p.pos[1], pr) { p.pos[3] } else { p.pos[1] };
            (pr, pl)
        } else {
            // (one hand: the comparison is the inverse of the tick's, as RDR2 reads)
            let pr = if p.points_x4 && d2(p.pos[2], cl) >= d2(p.pos[0], cl) { p.pos[2] } else { p.pos[0] };
            let pl = if p.points_x4 && d2(p.pos[3], cl) >= d2(p.pos[1], cl) { p.pos[3] } else { p.pos[1] };
            (pr, pl)
        };
        let (pr, pl) = (self.to_world(pr), self.to_world(pl));
        let gate = p.grab_dist + if self.r.grabbing { 0.2 } else { 0.0 };
        let near = |from: Vec3, to: Vec3| {
            let v = to - from;
            let d = v.length();
            d - (0.45 * unit(v).dot(c.vel)).clamp(0.0, d) < gate
        };
        // the angle round the chest (which chest-frame components RDR2 takes is [P]: Delirium's GTA IV ones)
        let angle = |to: Vec3| {
            let v = to - s3.d;
            s3.b.dot(v).atan2(-s3.c.dot(v))
        };
        let (ra, os) = (p.reach_angle, p.one_side_reach_angle);
        let cone = |th: f32| -ra < th && th < ra;
        if p.use_left && p.use_right {
            let (thr, thl) = (angle(pr), angle(pl));
            *right = near(c.b.joint_pos(limb(1).e[1]), pr) && cone(thr) && thr <= ra - os;
            *left = near(c.b.joint_pos(limb(0).e[1]), pl) && cone(thl) && thl >= os - ra;
        } else {
            let q = if p.use_right { pr } else { pl };
            let th = angle(q);
            if !near(s3.d, q) || !cone(th) {
                (*left, *right) = (false, false);
            } else {
                *right = p.use_right && th <= ra - os;
                *left = p.use_left && th >= os - ra;
            }
        }
        // weapon gates: a hand holding something (Delirium's busy hands, a weapon state not 0) only with dropWeaponIfNecessary; state 4 never
        let st = c.ch.core.arm_state;
        if !p.drop_weapon {
            *left &= !c.ch.busy[0] && st[0] == 0;
            *right &= !c.ch.busy[1] && st[1] == 0;
        }
        *left &= st[0] != 4;
        *right &= st[1] != 4;
        if st != [0, 0] {
            crate::extras_core_rdr2::trace_once("grab weapon", || format!("grab: weapon states {st:?}: left {left} right {right}"));
        }
    }

    /// 0x2acdd8c [P]: with a grip, the hands crossed or bunched against the grab points.
    fn crossed(&self, c: &Ctx) -> bool {
        if self.grip.iter().all(Option::is_none) {
            return false;
        }
        let up = c.up;
        let g = self.to_world(self.r.point[1]) - self.to_world(self.r.point[0]);
        let gv = g.dot(up);
        let gh = g - up * gv;
        if gh.length() < 0.1 {
            return true;
        }
        let hv = (c.b.at(limb(1).p[3]) - c.b.at(limb(0).p[3])).dot(up);
        let e = c.b.joint_pos(limb(0).e[2]) - c.b.joint_pos(limb(1).e[2]);
        (gv > 0.0) != (hv > 0.0) && gv.abs() > 0.05 && ((e.dot(gh) > 0.0 && gh.length() > 0.05) || gv < 0.3)
    }

    /// §5 (0x2ad13ac): one arm toward its target.
    fn arm_rdr2(&mut self, c: &mut Ctx, i: usize, w: Vec3, n: Vec3, may: bool, dt: f32) {
        let p = self.p;
        let l = limb(i);
        let (mask, side) = (["ul", "ur"][i], if i == 0 { 1.0 } else { -1.0 });
        let up = c.up;
        let hand = c.b.part(l.p[3]);
        let m = hand.m;
        let shoulder = c.b.joint_pos(l.e[1]);
        let all: Vec<usize> = (0..c.b.build.parts.len()).collect();
        let touching = c.b.doll.collided(&all);
        // §5.1 the target: led by the COM's speed, lifted while nothing touches, off the surface by the hand's turn
        let k = (c.vel.length().clamp(5.0, 20.0) - 5.0) / 15.0;
        let vh = unit(c.vel);
        let mut t = w + vh * (0.5 * k * (w - c.com).dot(vh));
        if !touching {
            t += up * k;
        }
        let mut o = if i == 0 { (m.a.dot(n).clamp(0.2, 1.0) - 0.2) * 0.2 } else { -0.04 - 0.2 * m.a.dot(n).clamp(-1.0, -0.2) };
        o += 0.2 * ((m.b.dot(n).clamp(0.2, 1.0) - 0.2) + (m.c.dot(n).abs().clamp(0.2, 1.0) - 0.2));
        o = o.min(0.2);
        if unit(t - m.d).dot(n) < 0.0 {
            o += (0.5 * (t - m.d).length()).min(0.1);
        }
        t += n * o;
        let reach = (t - shoulder).length().clamp(0.0, self.r.reach);
        let x = shoulder + unit(t - shoulder) * reach;
        let delta = reach.clamp(0.0, 0.71);
        let at = m.d + m.a * (0.04 * side);
        if self.grip[i].is_none() {
            // §5.2 reaching: Delirium's arm IK for RDR2's IK request, the hand turned to the normal
            c.ch.set_stiffness(p.arm_stiffness, 1.0, mask, Some(1.0));
            crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[3], x, delta, 0.0, side, 1.0, Vec3::ZERO, None);
            crate::nmpa::share(c.ch, c.b.frames, l.e[0], l.e[1]);
            if unit(m.d - shoulder).dot(n) <= 0.85 {
                crate::nmshot::Shot::wrist(c, l, x, n, side);
            } else {
                let e = &mut c.ch.eff[l.e[3]];
                (e.desired[0], e.desired[2]) = (0.0, 0.0);
            }
            c.ch.eff[l.e[3]].desired[1] = -1.0;
            // RDR2's reach calls the hand pull with radius 0 (no pull, no grip); ours keeps Delirium's grip within
            // GTA IV's distance so he still holds on
            let d = (w - at).length();
            if !p.justbrace && may && d < delirium_data::kv!(60) {
                self.len[i] = d;
                if self.grip[1 - i].is_none() {
                    self.older = i;
                }
                self.grip[i] = Some(Grip { broken: false, local: Vec3::ZERO, at: Some(at), to: w, len: d, on: crate::rigid::grip_local(self.frame, None, w) });
            }
        } else {
            // §5.3 held
            self.r.hold[i] += dt;
            let s = p.sticky_hands * if touching { 0.025 } else { 1.0 };
            let pu = p.pull_up_strength[i];
            let k = (5.0 + 9.0 * pu / (p.pull_up_time + 0.01) * self.r.hold[i]).min(9.0 * pu).max(5.0);
            c.ch.set_stiffness(k * S, 0.5, mask, Some(7.0));
            let wr = &mut c.ch.eff[l.e[3]];
            (wr.muscle_stiffness, wr.strength, wr.damping) = (20.0, 144.0 * S * S, 1.2 * S);
            for j in l.e {
                c.ch.eff[j].desired = c.ch.actual(j, c.b.frames);
            }
            // the hand pull (radius 0.25)
            let g = t - at;
            let d = g.length();
            if d < 0.75 {
                let kk = ((0.75 - d) * 2.0).clamp(0.0, 1.0) * s;
                let mut f = (if d > 0.3 { unit(g) * 0.3 } else { g }) * (kk * kk) - hand.v * (0.2 * kk);
                if dt < 1.0 / 60.0 {
                    f *= 60.0 * dt;
                }
                c.out.impulse.push((l.p[3], at, f));
            }
            let dg = (w - at).length();
            if let Some(gr) = self.grip[i].as_mut() {
                self.len[i] = (self.len[i] - 0.6 * dt).max(0.0).min(dg);
                gr.len = self.len[i];
            }
            // the wrist turned to the normal; past maxWristAngle the hand lets go
            let a = m.a * side;
            let cs = a.dot(n);
            let th = if cs <= -1.0 { std::f32::consts::PI } else if cs >= 1.0 { 0.0 } else { cs.acos() };
            if let Some(axis) = unit_axis(a.vx(n)) {
                c.out.torque(l.p[3], axis * (th * p.orientation_scale / (60.0 * dt).max(1.0)));
            }
            if th > p.max_wrist_angle {
                self.grip[i] = None;
            }
            if pu > 0.0 && !self.r.held[1 - i] {
                c.ch.eff[l.e[2]].desired[0] = 3.0 * pu;
                c.ch.eff[l.e[1]].desired[0] = (2.1 * pu).clamp(0.0, 2.0);
                c.ch.eff[l.e[0]].desired[0] = 0.6 * pu;
            }
            if !p.head_look_to_target {
                let legs = c.b.doll.collided(&limb(2).p[1..]) || c.b.doll.collided(&limb(3).p[1..]);
                let k = if legs { 10.0 * S } else { 4.0 * S };
                c.ch.set_stiffness(k, 1.0, "us", Some(1.0));
                c.ch.set_stiffness(k, 1.0, "uc", Some(1.0));
            }
        }
        // §5.4 grip feedback
        match (self.grip[i].is_some(), self.r.held[i]) {
            (true, false) => {
                self.r.held[i] = true;
                c.out.send(SUCCESS(), ["LeftArmGrab", "RightArmGrab"][i], 0.0);
            }
            (false, true) => {
                self.r.held[i] = false;
                c.out.send(FAILURE(), ["LeftArmGrab", "RightArmGrab"][i], 0.0);
            }
            _ => {}
        }
    }

    /// §4.7 the pull-up with one hand held: the clavicle stiffened and, once he hangs high enough (or latched), the
    /// hand turned to face along the wall by an orientation PD.
    fn pull_up(&mut self, c: &mut Ctx, i: usize, t: Vec3, dt: f32) {
        let p = self.p;
        let l = limb(i);
        let up = c.up;
        c.ch.eff[l.e[0]].set_stiffness((1.5 * p.body_stiffness).clamp(3.0 * S, 20.0 * S), 1.0, None);
        self.r.hang += dt;
        let hand = c.b.at(l.p[3]);
        let off = hand - c.com;
        let flat = off - up * off.dot(up);
        let pu = p.pull_up_strength[i];
        // (RDR2's fourth gate reads ctx+0xfa0, not named: taken as the COM's angular speed)
        let gate = hand.dot(up) > c.com.dot(up) + 1.0 - 0.4 * pu && self.r.hang > 0.5 && flat.length() < 0.5 && c.angvel.length() < 2.0;
        if !(gate || self.r.latched) {
            return;
        }
        self.r.latched = true;
        (self.p.max_wrist_angle, self.p.orientation_scale) = (std::f32::consts::PI, 0.0);
        let d = if i == 1 { c.com - t } else { t - c.com };
        let f = unit(d - up * d.dot(up));
        let mut rows = [f, -up, f.cross(-up)];
        if pu > 0.0 {
            let q = Quat::from_axis_angle(up, if i == 0 { -2.0944 } else { 2.0944 } * pu * pu);
            rows = rows.map(|v| q * v);
        }
        // the orientation PD (k 5, d 1), its impulse taken as a change of the hand's angular velocity
        let target = Quat::from_mat3(&Mat3::from_cols(rows[0], rows[1], rows[2]));
        let hm = c.b.part(l.p[3]);
        let now = Quat::from_mat3(&Mat3::from_cols(unit(hm.m.a), unit(hm.m.b), unit(hm.m.c)));
        let (axis, mut ang) = (target * now.inverse()).to_axis_angle();
        if ang > std::f32::consts::PI {
            ang -= std::f32::consts::TAU;
        }
        if dt > 0.0 && axis.length_squared() > 0.5 {
            let (k, damp) = (5.0, 1.0);
            let dw = axis * (k * k * dt * ang) - hm.w * (k * damp * dt);
            if dw.length_squared() > 1e-12 {
                let inertia = c.b.doll.inertia(l.p[3], unit(dw));
                c.out.torque(l.p[3], dw * (inertia / dt));
            }
        }
        c.ch.eff[l.e[3]].desired[2] = 0.0;
    }
}

/// RDR2's look on (0x2ac7bac): headLook started if it isn't, at grab's priority, stiffness bodyStiffness, damping 1.2.
fn look_on(c: &mut Ctx, head: &mut HeadLook, at: Vec3, stiffness: f32) {
    if !head.active {
        *head = HeadLook { stiffness, damping: 1.2, always_look: true, ..HeadLook::default() };
        head.activate(c.ch, c.b);
    }
    (head.stiffness, head.damping, head.always_look) = (stiffness, 1.2, true);
    head.look(PRIO_GRAB, at, Vec3::ZERO, true);
}

/// RDR2's spineTwist on (0x2ac7d3c): started if it isn't, toward `at`.
fn twist_on(ch: &mut Character, st: &mut SpineTwist, at: Vec3, arms: bool) {
    if !st.active {
        *st = SpineTwist::default();
        st.activate(ch);
    }
    (st.pos, st.always, st.arms) = (at, true, arms);
}

/// 0x2ad2180: a hand's point moved away from the other's along the line, by at most 0.15.
fn apart_rdr2(this: Vec3, other: Vec3, a: Vec3, b: Vec3) -> Vec3 {
    let u = unit(this - other);
    let end = if unit(a - b).dot(u) > 0.0 { a } else { b };
    this + u * (end - this).length().min(0.15)
}

#[allow(non_snake_case)]
#[inline]
fn EVENT() -> u8 { delirium_data::kv!(127 int) }
#[allow(non_snake_case)]
#[inline]
fn FAILURE() -> u8 { delirium_data::kv!(54 int) }
#[allow(non_snake_case)]
#[inline]
fn SUCCESS() -> u8 { delirium_data::kv!(55 int) }

