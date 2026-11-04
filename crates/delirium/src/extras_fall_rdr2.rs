//! RDR2's teeter, highFall and balancerCollisionsReaction, with the RDR2 core, from clean-room studies of RDR2's
//! behaviours (rdr2-decomp/study/rdr2_teeter.md, rdr2_highfall.md, rdr2_bcr.md). Teeter and balancerCollisionsReaction
//! are the RDR2 paths of their extras; highFall runs as an extra only with the RDR2 core (msg.rs sends it here), GTA
//! IV's highFall (nmhf.rs) otherwise.
use crate::extras::{Extra, P};
use crate::extras_fall::{attitude, orient_pd, unit, v3s, CHEST};
use crate::msg::{Error, Value};
use crate::nmdbm_rdr2::Crouch;
use crate::nmrs::{ANKLE, CLAVICLE, ELBOW, HIP, KNEE, NECK, SHOULDER, SPINE};
use crate::nmshot::{limb, Ctx};
use bevy_math::{Quat, Vec3};

const TAU: f32 = std::f32::consts::TAU;

/// Event feedback (teeter's states, highFall's state value), and the failure and success ones.
#[allow(non_snake_case)]
#[inline]
fn EVENT() -> u8 { delirium_data::kv!(127 int) }
#[allow(non_snake_case)]
#[inline]
fn FAILURE() -> u8 { delirium_data::kv!(54 int) }
#[allow(non_snake_case)]
#[inline]
fn SUCCESS() -> u8 { delirium_data::kv!(55 int) }

fn all_parts(c: &Ctx) -> Vec<usize> {
    (0..c.b.build.parts.len()).collect()
}

/// The pelvis, spine, neck and head parts (ours [G] for RDR2's contact masks 0x102709 and 0x1fff08 less the limbs).
fn trunk_parts() -> Vec<usize> {
    std::iter::once(0).chain(SPINE().iter().chain(NECK().iter()).map(|&j| j + 1)).collect()
}

fn leg_length(c: &Ctx) -> f32 {
    (c.b.joint_pos((*HIP())[0]) - c.b.joint_pos((*ANKLE())[0])).length().max(0.1)
}

// ---- teeter ----

/// RDR2's teeter parameters, RDR2's defaults where none is given. edgehandle is read and unused (no instances here).
#[derive(Clone, Copy, Debug)]
pub struct TtParams {
    left: Vec3,
    right: Vec3,
    /// an edge was given (else ours: one in front of him)
    given: bool,
    exclusion: bool,
    head: bool,
    high_fall: bool,
    towards: bool,
    away: bool,
    writhe: bool,
    force: bool,
    pre: f32,
    scale: f32,
    zone: f32,
}

impl TtParams {
    pub fn read(p: &mut P) -> Result<TtParams, Error> {
        let f = |p: &mut P, n: &str, d: f32| -> Result<f32, Error> { p.get(n).map_or(Ok(d), Value::f) };
        let b = |p: &mut P, n: &str, d: bool| -> Result<bool, Error> { p.get(n).map_or(Ok(d), Value::b) };
        let l = p.get("edgeLeft").map(Value::v).transpose()?;
        let r = p.get("edgeRight").map(Value::v).transpose()?;
        p.get("edgehandle").map(Value::i).transpose()?;
        f(p, "teeterTime", 1.0)?;
        Ok(TtParams {
            given: l.is_some() || r.is_some(), left: l.unwrap_or(Vec3::new(39.47, 38.89, 21.12)), right: r.unwrap_or(Vec3::new(39.47, 39.89, 21.12)),
            exclusion: b(p, "useExclusionZone", true)?, head: b(p, "useHeadLook", true)?, high_fall: b(p, "callHighFall", true)?,
            towards: b(p, "leantowards", false)?, away: b(p, "leanAway", true)?, writhe: b(p, "writhefall", true)?,
            force: b(p, "forceBalance", true)?, pre: f(p, "preTeeterTime", 0.5)?, scale: f(p, "leanAwayScale", 0.5)?,
            zone: f(p, "exclusionZone", 0.2)?,
        })
    }
}

/// RDR2's armsWindmill as teeter runs it: both circles from the chest at angles and centre (0, 0.5, -0.1) (the right
/// one mirrored, mirrorMode 1), radii 0.5 and 0.75 leg lengths, 1.5 turns a second either way, forceSync with a
/// random phaseOffset (degrees), each arm stopping for good once its hand touches something.
#[derive(Clone, Copy, Debug)]
struct Windmill {
    theta: [f32; 2],
    live: [bool; 2],
    phase: f32,
    radii: [[f32; 2]; 2],
    speed: [f32; 2],
    shoulder: f32,
}

const WM_ANGLES: Vec3 = Vec3::new(0.0, 0.5, -0.1);
const WM_CENTRE: Vec3 = Vec3::new(0.0, 0.5, -0.1);
const WM_ELBOW: f32 = 12.0;
const WM_ELBOW_MIN: f32 = 0.5;

impl Windmill {
    /// The circle's frame rows and origin: the chest's matrix turned by pi/2 about Z, then by the angles about Y, Z
    /// and X; the centre in the unturned frame; the right circle reflected across the chest's row 1.
    fn circle(c: &Ctx, i: usize) -> ([Vec3; 3], Vec3) {
        let m = c.b.part(CHEST()).m;
        let mut f = [m.a, m.b, m.c];
        let turn = |f: &mut [Vec3; 3], p: usize, q: usize, ang: f32| {
            let (s, co) = ang.sin_cos();
            let (x, y) = (f[p], f[q]);
            (f[p], f[q]) = (x * co + y * s, y * co - x * s);
        };
        turn(&mut f, 0, 1, std::f32::consts::FRAC_PI_2);
        turn(&mut f, 2, 0, WM_ANGLES.y);
        turn(&mut f, 0, 1, WM_ANGLES.z);
        turn(&mut f, 1, 2, WM_ANGLES.x);
        let mut k = WM_CENTRE;
        if i == 1 {
            k.y = -k.y;
            for row in f.iter_mut() {
                *row -= m.b * (2.0 * row.dot(m.b));
            }
        }
        (f, m.d + m.a * k.x + m.b * k.y + m.c * k.z)
    }

    fn new(c: &mut Ctx, leg: f32) -> Windmill {
        let theta = [0, 1].map(|i| {
            let (f, o) = Self::circle(c, i);
            let d = c.b.part(limb(i).p[3]).m.d - o;
            f[2].dot(d).atan2(f[1].dot(d)).rem_euclid(TAU)
        });
        Windmill { theta, live: [true; 2], phase: 200.0 * c.ch.random() - 100.0, radii: [[0.5 * leg, 0.75 * leg]; 2], speed: [1.5; 2], shoulder: 11.0 }
    }

    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        for i in 0..2 {
            let l = limb(i);
            c.ch.eff[l.e[0]].set_stiffness(self.shoulder, 1.0, None);
            c.ch.eff[l.e[1]].set_stiffness(self.shoulder, 1.0, None);
            c.ch.eff[l.e[2]].set_stiffness(WM_ELBOW, 1.0, None);
        }
        // forceSync: the right arm kept phaseOffset behind the left, each one's step sped up or slowed by up to half
        let (tl, tr) = (self.theta[0], self.theta[1]);
        let phi = (tr - self.phase.to_radians()).rem_euclid(TAU);
        let (mut d1, mut d2) = if phi > tl { (phi - tl, TAU - phi + tl) } else { (TAU - tl + phi, tl - phi) };
        if self.speed[0] < 0.0 && self.speed[1] < 0.0 {
            (d1, d2) = (d2, d1);
        }
        let m = (0.5 * d1.min(d2)).clamp(0.0, 0.5);
        let step = if d1 > d2 { [dt * (1.0 - m), dt * (1.0 + m)] } else { [dt * (1.0 + m), dt * (1.0 - m)] };
        for i in 0..2 {
            let l = limb(i);
            if self.live[i] && c.b.doll.collided(&[l.p[3]]) {
                self.live[i] = false;
            }
            if !self.live[i] {
                continue;
            }
            let (f, o) = Self::circle(c, i);
            self.theta[i] = (self.theta[i] + TAU * step[i] * self.speed[i]).rem_euclid(TAU);
            let (s, co) = self.theta[i].sin_cos();
            let p = o + f[1] * (self.radii[i][0] * co) + f[2] * (self.radii[i][1] * s);
            let side = if i == 0 { 1.0 } else { -1.0 };
            crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[3], p, 0.0, 0.0, side, 1.0, Vec3::ZERO, None);
            let a = c.ch.actual(l.e[2], c.b.frames)[0];
            if a < WM_ELBOW_MIN {
                let floor = a + (WM_ELBOW_MIN - a) * (5.0 * dt).clamp(0.0, 1.0);
                let e = &mut c.ch.eff[l.e[2]].desired[0];
                *e = e.max(floor);
            }
            c.ch.eff[l.e[3]].desired = [0.0; 3];
        }
    }
}

/// RDR2's teeter (with the RDR2 core): from an edge (left and right ends; its normal up x (right - left) is the safe
/// side), it leans him away from it (or toward it) through the balancer's leans and a longer prediction, turns him
/// along it once he is within 0.2 m (the balancer's turn and spineTwist), pitches his hips once per teeter episode
/// with the spine bending against them, windmills his arms near it, forces the balance while his COM is past it,
/// keeps the balancer's steps off it, looks at it, and once the balancer has given up past the edge hands the fall
/// to highFall (feet clear and nothing touching) or writhes.
pub struct TeeterRdr2 {
    p: TtParams,
    first: bool,
    /// the edge: point, normal, edge vector
    at: Vec3,
    n: Vec3,
    e: Vec3,
    d: f32,
    approach: f32,
    tte: f32,
    fwd: f32,
    feet: [f32; 2],
    /// the windmill's clock (+0x1ac), the teeter hold (+0x1b0), the turn side, the hip pitch amplitude, the episode
    /// clock, the lean-toward ramp
    wm_t: f32,
    hold: f32,
    side: f32,
    amp: f32,
    timer: f32,
    ramp: f32,
    failed_last: bool,
    teetering: bool,
    forcing: bool,
    hf_called: bool,
    over: bool,
    turned: bool,
    writhe_on: bool,
    wm: Option<Windmill>,
    flags: [bool; 9],
    prev: [bool; 9],
    pub out: Vec<String>,
}

impl TeeterRdr2 {
    pub fn new(p: TtParams) -> TeeterRdr2 {
        TeeterRdr2 { p, first: true, at: Vec3::ZERO, n: Vec3::X, e: Vec3::Y, d: 1.0, approach: 1.0, tte: 1.0, fwd: 0.0, feet: [1.0; 2], wm_t: 0.0,
                     hold: 0.0, side: 0.0, amp: 1.6, timer: 0.0, ramp: 0.0, failed_last: false, teetering: false, forcing: false, hf_called: false,
                     over: true, turned: false, writhe_on: false, wm: None, flags: [false; 9], prev: [false; 9], out: vec![] }
    }

    /// The edge from its two ends: point left, edge right - left, normal up x edge ((1, 0, 0) if degenerate).
    fn edge(&mut self, up: Vec3) {
        (self.at, self.e) = (self.p.left, self.p.right - self.p.left);
        let n = up.cross(self.e);
        self.n = if n.length_squared() > 1e-12 { n.normalize() } else { Vec3::X };
    }

    /// New edge ends sent while it runs (RDR2 rebuilds the edge when they change).
    pub fn update(&mut self, p: &mut P) -> Result<(), Error> {
        let l = p.get("edgeLeft").map(Value::v).transpose()?;
        let r = p.get("edgeRight").map(Value::v).transpose()?;
        if let Some(l) = l {
            self.p.left = l;
        }
        if let Some(r) = r {
            self.p.right = r;
        }
        if l.is_some() || r.is_some() {
            self.p.given = true;
            self.first = true;
        }
        Ok(())
    }

    fn begin(&mut self, c: &mut Ctx) {
        self.first = false;
        if !self.p.given {
            // ours: no edge given, one 0.5 m in front of his feet, 1 m wide, his side the safe one (RDR2's default is
            // a fixed place in its world)
            let cl = *CLAVICLE();
            let fwd = unit((c.b.part(cl[0] + 1).m.d - c.b.part(cl[1] + 1).m.d).cross(c.up));
            let fwd = unit(fwd - c.up * c.up.dot(fwd));
            let side = c.up.cross(fwd);
            let foot = c.b.part(limb(2).p[3]).m.d;
            let at = c.com - c.up * c.up.dot(c.com - foot) + fwd * 0.5;
            (self.p.left, self.p.right) = (at - side * 0.5, at + side * 0.5);
            self.p.given = true;
        }
        self.edge(c.up);
        if self.hold == 0.0 {
            self.hold = 0.8 + 1.1 * c.ch.random();
        }
    }

    pub fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if self.first {
            self.begin(c);
        }
        self.flags = [false; 9];
        let up = c.up;
        let flat = |x: Vec3| x - up * up.dot(x);
        // measurements
        let com_e = c.com + up * (up.dot(self.at) - up.dot(c.com));
        self.d = (com_e - self.at).dot(self.n);
        self.approach = -flat(c.vel).dot(self.n);
        self.tte = if self.d < 0.0 { 0.0 } else if self.approach <= 0.0 { 10.0 } else { self.d / self.approach };
        let hips = [0, 1].map(|i| c.b.joint_pos((*HIP())[i]));
        self.fwd = unit(hips[0] - hips[1]).cross(up).dot(c.vel);
        self.feet = [2, 3].map(|i| flat(c.b.part(limb(i).p[3]).m.d - self.at).dot(self.n));
        let failed = !c.upright;
        if failed && !self.failed_last {
            c.out.dbm_crouch.push(Crouch::Hip(6, 0.0, f32::INFINITY));
        }
        if !failed {
            self.teeter(c, dt);
        } else {
            self.fall(c);
        }
        // the balancer: force balance while past the edge (latched), and the exclusion plane
        c.out.dbm_force_balance |= self.forcing;
        if self.p.exclusion {
            c.out.dbm_plane = Some((self.at, self.n, -1.0, self.p.zone));
        }
        self.arms(c, failed, dt);
        self.failed_last = failed;
        // feedback: each flag set this tick that wasn't last tick
        for i in 0..9 {
            if self.flags[i] && !self.prev[i] {
                c.out.send(EVENT(), "teeter", i as f32);
            }
        }
        self.prev = self.flags;
    }

    /// RDR2's teeter logic, while the balancer holds him: the prediction boost, the leans, head look, turn along the
    /// edge, the teeter episode and force balance.
    fn teeter(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.p;
        let (d, approach, n) = (self.d, self.approach, self.n);
        let a = (approach - 1.0).clamp(0.0, 3.0) / 3.0;
        let boost = if d > 0.0 && (d < 1.5 || self.feet[0] < 1.0 || self.feet[1] < 1.0) { 0.3 * a } else { 0.0 };
        c.out.dbm_lead_add = boost;
        // leans: the COM lean (RDR2's priority 8) and the hip lean (6)
        let l = a * p.scale;
        let mut away = false;
        let tail = if p.towards && d < 4.0 {
            let f = (1.5 - 0.5 * approach).clamp(0.0, 2.0) * 0.08;
            // (read as written: about 0.0005 a tick)
            self.ramp = if approach >= 0.5 { 0.0 } else { self.ramp + 0.0005 * dt + 0.0005 };
            let k = -(f + self.ramp);
            c.out.lean = Some((crate::nmdbm::LeanMode::Dir(n), k, false));
            c.out.hip_lean = Some(n * k);
            true
        } else if p.away && d < 1.0 {
            self.flags[1] = true;
            if l > 0.001 {
                c.out.lean = Some((crate::nmdbm::LeanMode::Dir(n), l, false));
                c.out.hip_lean = Some(n * l);
                away = true;
            }
            l < 0.1
        } else {
            false
        };
        if tail && d < 0.7 && approach > 0.2 {
            c.out.lean = Some((crate::nmdbm::LeanMode::Dir(n), 0.1, false));
            c.out.hip_lean = None;
        }
        self.look(c);
        // ours [P]: the spine part's row RDR2 reads is taken as the back axis (row c)
        let back = c.b.part(SPINE()[1] + 1).m.c;
        if d < 0.2 {
            self.flags[3] = true;
            if self.side == 0.0 {
                self.side = if back.dot(self.e) < 0.0 { 1.0 } else { -1.0 };
            }
            let e = self.e * self.side;
            c.out.dbm_turn = Some(e * 0.5);
            self.turned = true;
            c.out.spine_twist = Some((c.com + e, crate::nmst::SpineTwist::default().stiffness));
        }
        // the teeter episode: the hips pitch once (to A, back to 0 after 0.5 s) at a random rate, the spine against it
        let window = 1.4 / self.amp.abs() + 0.2;
        if self.teetering && self.hold + 0.5 + window < self.timer {
            self.teetering = false;
        }
        if d < 0.2 || self.teetering {
            if !self.teetering {
                self.amp = -0.8 - 0.8 * c.ch.random();
                let hips = [0, 1].map(|i| c.b.joint_pos((*HIP())[i]));
                if (hips[1] - hips[0]).dot(self.e) < 0.0 {
                    self.amp = -self.amp;
                }
                self.timer = 0.0;
                let r = c.ch.random();
                // our hip pitch is RDR2's negated
                c.out.dbm_crouch.push(Crouch::Hip(6, -self.amp.clamp(-2.0, 2.0), 2.0 + 2.0 * r));
                self.wm_t -= 1.0;
            } else if self.timer > 0.5 {
                c.out.dbm_crouch.push(Crouch::Hip(6, 0.0, 1.4));
            }
            let pitch = -c.dbm.map_or(0.0, |b| b.hip_pitch);
            let s = pitch * -0.2 / self.amp.abs();
            for &j in &SPINE()[..4] {
                c.ch.eff[j].desired[0] = s;
            }
            self.teetering = true;
            self.timer += dt;
        }
        if p.force && d < 0.0 {
            self.forcing = true;
        } else if self.forcing && d > 0.15 {
            self.forcing = false;
        }
        if self.teetering && self.timer <= self.hold + 0.5 + window {
            self.flags[4] = true;
        }
        c.out.dbm_taper = Some(!(boost > 0.0 || away));
    }

    /// RDR2's head look: one second ahead when not about to reach the edge, else along the edge from his COM, at his
    /// left foot's height.
    fn look(&mut self, c: &mut Ctx) {
        let up = c.up;
        if !self.p.head || self.tte >= 1.0 {
            // (headLook isn't started here: only its target, if it runs)
            self.out.push(format!("headLook pos={} alwaysLook=false", v3s(c.com + c.vel)));
            return;
        }
        let com_e = c.com + up * (up.dot(self.at) - up.dot(c.com));
        let eh = unit(self.e);
        let mut q = self.at + eh * (com_e - self.at).dot(eh);
        let dot = c.b.part(SPINE()[1] + 1).m.c.dot(eh);
        let s = if self.fwd > 0.0 { if dot > 0.0 { 1.0 } else { -1.0 } } else { -dot };
        q += eh * (0.5 * s);
        let foot = c.b.part(limb(2).p[3]).m.d;
        q += up * (up.dot(foot) - up.dot(q));
        self.out.push(format!("headLook start=true alwaysLook=true pos={} damping=1 stiffness=11", v3s(q)));
    }

    /// RDR2's fall logic, once the balancer has given up: past the edge with his feet clear and nothing touching he
    /// goes to highFall; otherwise bodyWrithe while nothing touches.
    fn fall(&mut self, c: &mut Ctx) {
        // (balancerCollisionsReaction's hand-over, +0x1d1, isn't seen from here)
        self.over = self.d <= 0.0;
        if self.d > 0.0 {
            self.flags[5] = true;
        }
        if !self.over {
            return;
        }
        let touching = c.b.doll.collided(&all_parts(c));
        if !touching && self.feet[0] < -0.1 && self.feet[1] < -0.1 && !self.hf_called && self.p.high_fall {
            self.hf_called = true;
            self.wm = None;
            self.writhe_on = false;
            self.out.extend(["bodyWrithe start=false", "catchFall start=false", "bodyBalance start=false"].map(String::from));
            self.flags[8] = true;
            // RDR2's highFall settings as teeter writes them (read as written)
            self.out.push("highFall start=true pdStrength=0 pdDamping=0 forwardRoll=false bodydamping=1 aimAngleBase=1 bodyStiffness=11 \
                           catchfalltime=0.3 crashOrLandCutOff=0.868 armAngSpeed=7.85 armAmplitude=2 legRadius=0.4 legAngSpeed=7.85 \
                           orientateBodyToFallDirection=false armsUp=0.18 startplayerset=-0.02 footVelCompScale=0.05 sideD=-1 \
                           fowardOffsetOfLegIK=0.2 legL=0.8 catchFallCutOff=0.878 legStrength=12 ignorWorldCollisions=false".into());
        }
        if !self.hf_called {
            self.flags[7] = true;
            if self.p.writhe {
                if self.writhe_on && touching {
                    self.writhe_on = false;
                    self.out.push("bodyWrithe start=false".into());
                } else if !self.writhe_on && !touching {
                    self.writhe_on = true;
                    self.out.push("bodyWrithe start=true armStiffness=13 backStiffness=13 legStiffness=11 armDamping=0.3 backDamping=0.3 \
                                   legDamping=0.3 armPeriod=1.5 backPeriod=3 legPeriod=1 armAmplitude=0.8 backAmplitude=2 legAmplitude=1.5 \
                                   rollOverFlag=false blendArms=0.5 blendBack=0.5 blendLegs=0.5 mask=fb".into());
                }
            }
        }
    }

    /// RDR2's arms: the windmill near the edge (or heading for it fast), while the trunk touches nothing; once failed,
    /// an arm past the edge that still windmills is left out of catchFall's and bodyRollUp's arms.
    fn arms(&mut self, c: &mut Ctx, failed: bool, dt: f32) {
        let fast = self.approach > 1.0 && self.tte <= self.p.pre;
        let free = !c.b.doll.collided(&trunk_parts());
        let on = (self.d < 0.4 || fast) && self.wm_t < 0.8 && (!failed || (self.over && !self.hf_called)) && free;
        if !on {
            if !self.hf_called {
                self.wm = None;
            }
        } else {
            self.wm_t += dt;
            let leg = leg_length(c);
            let mut wm = match self.wm {
                Some(w) => w,
                None => Windmill::new(c, leg),
            };
            if failed && !self.failed_last {
                wm.phase = 60.0;
                wm.radii[0] = [0.75 * leg; 2];
            }
            let hips = [0, 1].map(|i| c.b.joint_pos((*HIP())[i]));
            let h = unit(hips[1] - hips[0]);
            let w = h.dot(c.angvel);
            let rev = (h.dot(self.e) < 0.0 && !(w < -1.5 && self.fwd > 0.5)) || (w > 1.5 && self.fwd < 0.5);
            wm.speed = [if rev { -1.5 } else { 1.5 }; 2];
            wm.shoulder = if failed { 13.0 } else { (2.0 * self.approach.abs() + 11.0).min(15.0) };
            wm.tick(c, dt);
            self.wm = Some(wm);
        }
        if failed {
            if let Some(wm) = self.wm {
                for i in 0..2 {
                    let arm = c.b.part(limb(i).p[1]).m.d;
                    let off = arm - self.at;
                    if (off - c.up * c.up.dot(off)).dot(self.n) < -0.2 && wm.live[i] {
                        c.ch.exclude_arms[i] = true;
                    }
                }
            }
        }
    }

    /// RDR2's deactivate: the exclusion plane and leans lapse, the turn and hip pitch are let go, and if he went over,
    /// bodyWrithe, headLook, spineTwist and the windmill stop.
    pub fn stop(&mut self, c: &mut Ctx) {
        if self.turned {
            c.out.dbm_turn = Some(Vec3::ZERO);
        }
        c.out.dbm_crouch.push(Crouch::ReleaseHip(f32::INFINITY));
        if self.over {
            if self.writhe_on {
                self.out.push("bodyWrithe start=false".into());
            }
            self.out.push("headLook start=false".into());
            c.out.spine_twist_stop = true;
            self.wm = None;
        }
    }
}

// ---- highFall ----

/// RDR2's highFall parameters, RDR2's defaults where none is given (GTA IV's message names and RDR2's).
#[derive(Clone, Copy, Debug)]
struct HfParams {
    body_stiffness: f32,
    body_damping: f32,
    catch_fall_time: f32,
    crash_or_land_cut_off: f32,
    pd_strength: f32,
    pd_damping: f32,
    catch_fall_cut_off: f32,
    arms_up: f32,
    leg_radius: f32,
    leg_ang_speed: f32,
    leg_asymmetry: f32,
    arms2legs_phase: f32,
    arms2legs_sync: i32,
    arm_ang_speed: f32,
    arm_amplitude: f32,
    arm_phase: f32,
    forward_offset: f32,
    leg_strength: f32,
    balance: bool,
    orientate_to_fall: bool,
    alan_rickman: bool,
    orientate_twist: bool,
    orientate_max: f32,
    forward_roll: bool,
    zero_pose_roll: bool,
    ignore_world: bool,
    adaptive_circling: bool,
    hula: bool,
    min_speed_for_brace: f32,
    max_recoverable: f32,
    landing_normal: f32,
}

impl HfParams {
    fn read(p: &mut P) -> Result<HfParams, Error> {
        let f = |p: &mut P, n: &str, d: f32| -> Result<f32, Error> { p.get(n).map_or(Ok(d), Value::f) };
        let b = |p: &mut P, n: &str, d: bool| -> Result<bool, Error> { p.get(n).map_or(Ok(d), Value::b) };
        let i = |p: &mut P, n: &str, d: i32| -> Result<i32, Error> { p.get(n).map_or(Ok(d), |v| v.i().or_else(|_| v.b().map(i32::from))) };
        // read by RDR2's handler but not by highFall, or GTA IV's own
        for n in ["aimAngleBase", "startplayerset", "footVelCompScale", "legL", "sideD", "armPeriod", "landCutOffVelocity", "fowardVelRotation",
                  "fowardVelCutOff"] {
            f(p, n, 0.0)?;
        }
        for n in ["useZeroPose", "armBendElbows"] {
            b(p, n, true)?;
        }
        let roll = b(p, "fowardRoll", false)?;
        let offset = f(p, "fowardOffsetOfLegIK", 0.0)?;
        Ok(HfParams {
            body_stiffness: f(p, "bodyStiffness", 11.0)?, body_damping: f(p, "bodydamping", 1.0)?, catch_fall_time: f(p, "catchfalltime", 0.3)?,
            crash_or_land_cut_off: f(p, "crashOrLandCutOff", 0.868)?, pd_strength: f(p, "pdStrength", 0.0)?, pd_damping: f(p, "pdDamping", 1.0)?,
            catch_fall_cut_off: f(p, "catchFallCutOff", 0.878)?, arms_up: f(p, "armsUp", -3.1)?, leg_radius: f(p, "legRadius", 0.4)?,
            leg_ang_speed: f(p, "legAngSpeed", 7.85)?, leg_asymmetry: f(p, "legAsymmetry", 4.0)?, arms2legs_phase: f(p, "arms2LegsPhase", 0.0)?,
            arms2legs_sync: i(p, "arms2LegsSync", 1)?, arm_ang_speed: f(p, "armAngSpeed", 7.85)?, arm_amplitude: f(p, "armAmplitude", 2.0)?,
            arm_phase: f(p, "armPhase", 3.1)?, forward_offset: f(p, "forwardOffsetOfLegIK", offset)?, leg_strength: f(p, "legStrength", 12.0)?,
            balance: b(p, "balance", true)?, orientate_to_fall: b(p, "orientateBodyToFallDirection", false)?,
            alan_rickman: b(p, "alanRickman", false)?, orientate_twist: b(p, "orientateTwist", true)?, orientate_max: f(p, "orientateMax", 300.0)?,
            forward_roll: b(p, "forwardRoll", roll)?, zero_pose_roll: b(p, "zeroPoseRoll", false)?, ignore_world: b(p, "ignorWorldCollisions", false)?,
            adaptive_circling: b(p, "adaptiveCircling", true)?, hula: b(p, "hula", true)?, min_speed_for_brace: f(p, "minSpeedForBrace", 10.0)?,
            max_recoverable: f(p, "maxSpeedForRecoverableFall", 15.0)?, landing_normal: f(p, "landingNormal", 0.6)?,
        })
    }
}

const HF_NAMES: [&str; 8] = ["airborne", "crash", "brace", "unrecoverable", "fast fall", "forward roll", "hard landing", "land on feet"];

/// RDR2's highFall (with the RDR2 core): smartFall's older sibling, an eight-state machine (airborne, crash, brace,
/// unrecoverable, fast fall, forward roll, hard landing, land on feet) that runs the fall by starting and
/// configuring other behaviours, from a ground probe along five seconds of the velocity, the time to impact, the
/// attitude he will land in and whether the fall is recoverable.
pub struct HighFall {
    p: HfParams,
    state: u8,
    /// T, the time with the upper body touching (state 0), the predicted impact time (in T), the state clocks (forward
    /// roll / land on feet, and the hard landing's), the smoothed speed, the hula timer
    t: f32,
    touch_t: f32,
    impact: f32,
    t110: f32,
    s6: f32,
    smooth: f32,
    hula: f32,
    leg_len: f32,
    land_up: f32,
    class: u8,
    class_mid: u8,
    class_now: u8,
    fb_state: i32,
    contact: bool,
    pd: bool,
    good: bool,
    latch: bool,
    not_upright: bool,
    recoverable: bool,
    forward: bool,
    flat: bool,
    start: Vec3,
    ground: Vec3,
    /// state 3's short probe down: the ground's normal . up when it hits
    down: Option<f32>,
    back_pedal: bool,
    seen_balancer: bool,
    first: bool,
    on: Vec<&'static str>,
    out: Vec<String>,
}

impl HighFall {
    pub fn new(p: &mut P) -> Result<HighFall, Error> {
        Ok(HighFall { p: HfParams::read(p)?, state: 0, t: 0.0, touch_t: 0.0, impact: 50.0, t110: 0.0, s6: 0.0, smooth: 0.0, hula: -0.1, leg_len: 0.0,
                      land_up: 0.0, class: 3, class_mid: 3, class_now: 3, fb_state: 0, contact: false, pd: true, good: false, latch: false,
                      not_upright: false, recoverable: true, forward: false, flat: false, start: Vec3::ZERO, ground: Vec3::Z, down: None,
                      back_pedal: false, seen_balancer: false, first: true, on: vec![], out: vec![] })
    }

    fn ensure(&mut self, name: &'static str, line: String) {
        if !self.on.contains(&name) {
            self.on.push(name);
            self.out.push(line);
        }
    }

    /// Sent every tick (its parameters each time), and so known to be running.
    fn keep(&mut self, name: &'static str, line: String) {
        if !self.on.contains(&name) {
            self.on.push(name);
        }
        self.out.push(line);
    }

    fn halt(&mut self, name: &'static str) {
        if self.on.contains(&name) {
            self.on.retain(|&s| s != name);
            self.out.push(format!("{name} start=false"));
        }
    }

    /// The "highFall" feedback, once per activation (+0x10 read as failure, +0x18 as success [P]).
    fn feedback(&mut self, c: &mut Ctx, failure: bool) {
        if !self.latch {
            self.latch = true;
            c.out.send(if failure { FAILURE() } else { SUCCESS() }, "highFall", 0.0);
        }
    }

    fn catch_fall_line(&self) -> String {
        let b = self.p.body_stiffness;
        format!("catchFall start=true legsStiffness={} torsoStiffness={} armsStiffness={}", 0.75 * b, 0.909 * b, 0.8462 * b)
    }

    /// RDR2's ground probe (state 0): from the COM along 5 x the velocity; the time to impact from the height over the
    /// hit (less the leg length when upright), and the attitude he will land in.
    fn ground_probe(&mut self, c: &Ctx, world: &crate::collision::Collision, obs: &[crate::ragdoll::Obstacle]) {
        let p = self.p;
        let up = c.up;
        let seg = c.vel * 5.0;
        let len = seg.length();
        match (len > 1e-4).then(|| crate::nmcf::cast(world, obs, c.com, seg / len, len)).flatten() {
            Some((t, n, _)) => {
                let hit = c.com + seg / len * t;
                (self.land_up, self.ground) = (n.dot(up), n);
                self.flat = self.land_up > p.landing_normal;
                let mut d = up.dot(c.com - hit).abs();
                if up.dot(c.m.b) >= p.crash_or_land_cut_off && d > self.leg_len {
                    d -= self.leg_len;
                }
                let (vz, g) = (-c.vel.dot(up).abs(), crate::ragdoll::GRAVITY().abs().max(1.0));
                self.impact = self.t + ((2.0 * g * d + vz * vz).sqrt() + vz) / g;
            }
            None => (self.impact, self.ground) = (self.t + 30.0, up),
        }
        // (ours: the body turned on at its spin, at most a second ahead)
        let q = Quat::from_scaled_axis(c.angvel * (self.impact - self.t).clamp(0.0, 1.0));
        self.class = attitude([q * c.m.a, q * c.m.b, q * c.m.c], self.ground);
        self.good = self.class == 0;
    }

    fn tick_inner(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.p;
        let up = c.up;
        if self.first {
            self.first = false;
            self.start = c.com;
            self.out.push("#say highFall (RDR2) airborne".into());
            self.enter(c, 0);
        }
        if self.t > 0.0 {
            self.contact = c.b.doll.collided(&all_parts(c));
        }
        self.not_upright = up.dot(c.m.b) < p.catch_fall_cut_off;
        let v = if c.vel.length_squared() > 1e-6 { c.vel.normalize() } else { Vec3::X };
        self.forward = v.dot(-c.m.c) > 0.2;
        self.class_now = attitude([c.m.a, c.m.b, c.m.c], self.ground);
        if !matches!(self.state, 1 | 6 | 7) {
            c.ch.set_stiffness(p.body_stiffness, p.body_damping, "fb", None);
        }
        if self.pd && p.pd_strength > 0.001 {
            orient_pd(c, dt, (p.pd_strength, p.pd_damping, p.orientate_max), p.alan_rickman, p.orientate_to_fall, p.orientate_twist, self.start);
        }
        self.t += dt;
        let next = self.update(c, dt);
        if let Some(s) = next.filter(|&s| s != self.state) {
            self.exit(c, self.state);
            self.state = s;
            self.out.push(format!("#say highFall (RDR2) {}  impact in {:.2} s", HF_NAMES[s as usize], self.impact - self.t));
            self.enter(c, s);
        }
        let fb = match self.state {
            0 | 2 => 5,
            1 | 3 => 1,
            4 | 5 => 4,
            6 if self.s6 > 0.0 => 6,
            _ => 0,
        };
        if fb != self.fb_state {
            self.fb_state = fb;
            c.out.send(EVENT(), "highFall", fb as f32);
        }
    }

    fn enter(&mut self, _c: &mut Ctx, s: u8) {
        match s {
            0 => (self.pd, self.touch_t) = (true, 0.0),
            1 => self.contact = false,
            3 => self.halt("pedalLegs"),
            4 => {
                self.on.retain(|&n| n != "catchFall");
                let l = self.catch_fall_line();
                self.ensure("catchFall", l);
            }
            6 => {
                self.s6 = 1.0;
                self.halt("bodyFoetal");
                self.halt("pedalLegs");
                self.on.retain(|&n| n != "catchFall");
                let l = self.catch_fall_line();
                self.ensure("catchFall", l);
            }
            _ => {}
        }
    }

    fn exit(&mut self, _c: &mut Ctx, s: u8) {
        let stop: &[&'static str] = match s {
            0 => {
                self.pd = false;
                &["headLook", "pedalLegs", "catchFall", "armsWindmillAdaptive", "bodyWrithe"]
            }
            1 => &["bodyFoetal", "pedalLegs"],
            2 => &["pedalLegs", "catchFall"],
            3 => &["bodyWrithe", "bodyFoetal"],
            4 => &["catchFall"],
            5 => &["rollDownStairs"],
            _ => &[],
        };
        for &n in stop {
            self.halt(n);
        }
    }

    /// The state's update, and the state it asks for.
    fn update(&mut self, c: &mut Ctx, dt: f32) -> Option<u8> {
        let p = self.p;
        let up = c.up;
        let vh = c.vel - up * up.dot(c.vel);
        let speed = c.vel.length();
        let late = self.t - 0.25 > self.impact;
        match self.state {
            0 => {
                self.airborne(c, dt);
                let legs: Vec<usize> = (2..4).flat_map(|i| limb(i).p[1..].to_vec()).collect();
                let upper: Vec<usize> = (1..c.b.build.parts.len()).filter(|k| !legs.contains(k)).collect();
                self.touch_t = if c.b.doll.collided(&upper) { self.touch_t + dt } else { 0.0 };
                let impact = self.touch_t > 0.3 || self.impact - p.catch_fall_time - 0.15 < self.t;
                if impact {
                    self.recoverable = speed < p.max_recoverable;
                    // (ours: as smartFall's sibling, an unrecoverable impact wins over the crash requests)
                    if !self.recoverable {
                        return Some(3);
                    }
                }
                let crash = impact && !(self.good && (!self.contact || p.ignore_world));
                if !crash {
                    return (impact && self.flat).then_some(2);
                }
                if !self.forward {
                    return Some(1);
                }
                match self.class {
                    1 => Some(6),
                    2 => {
                        let q = Quat::from_scaled_axis(c.angvel * (0.5 * (self.impact - self.t)).clamp(0.0, 1.0));
                        self.class_mid = attitude([q * c.m.a, q * c.m.b, q * c.m.c], self.ground);
                        Some(if self.class_mid == 2 && self.class_now == 2 { 6 } else { 1 })
                    }
                    _ => Some(1),
                }
            }
            1 => {
                self.crash(c);
                if self.contact { Some(6) } else if late { Some(0) } else { None }
            }
            2 => {
                self.brace(c);
                if self.contact {
                    Some(if p.forward_roll { 5 } else if self.not_upright { 6 } else { 7 })
                } else if late {
                    Some(0)
                } else {
                    None
                }
            }
            3 => {
                let mut r = None;
                if self.down.is_some_and(|u| u > 0.7) && self.contact && self.not_upright && speed < 1.0 {
                    r = Some(6);
                } else if vh.length() > p.min_speed_for_brace && self.class != 3 && self.forward && !self.flat {
                    r = Some(4);
                } else {
                    self.ensure("bodyFoetal", "bodyFoetal start=true".into());
                    self.ensure("bodyWrithe", "bodyWrithe start=true blendArms=0.5 blendBack=0.5 blendLegs=0.5".into());
                }
                if late && !self.contact {
                    r = Some(0);
                }
                r
            }
            4 => {
                if vh.length() > p.min_speed_for_brace && self.class != 3 && self.forward && !self.flat && !late {
                    self.catch(c);
                    None
                } else {
                    Some(3)
                }
            }
            5 => {
                self.roll(c, dt);
                self.latch.then_some(6)
            }
            6 => {
                if self.s6 > 0.0 {
                    self.s6 -= dt;
                }
                self.catch(c);
                None
            }
            7 => self.land(c, dt).then_some(6),
            _ => None,
        }
    }

    /// S0: head look below and ahead, pedalling legs (adaptive circling, hula) and windmilling arms.
    fn airborne(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.p;
        let b = p.body_stiffness;
        let (up, v) = (c.up, c.vel);
        let look = c.com - c.m.c + up * c.m.c.dot(up) - up * 2.0;
        self.keep("headLook", format!("headLook start=true alwaysLook=true pos={} vel={} stiffness={b} damping={}", v3s(look), v3s(v), p.body_damping));
        if p.adaptive_circling {
            let pitch = {
                let a = c.m.c.z.clamp(-1.0, 1.0).asin();
                if c.m.b.z < 0.0 { if a > 0.0 { std::f32::consts::PI - a } else if a < 0.0 { -std::f32::consts::PI - a } else { a } } else { a }
            };
            // (RDR2 reads the angular momentum about the side axis; ours, the spin about it [G], as smartFall's)
            let l = c.m.a.dot(c.angvel);
            let e = if l > 12.0 || l < -12.0 || pitch.signum() != l.signum() { l - pitch } else { -pitch };
            if e < -2.2 {
                self.back_pedal = false;
            } else if e > 2.2 {
                self.back_pedal = true;
            }
        }
        if p.hula {
            if c.m.c.z < -0.2 && self.hula < 0.0 && c.m.b.z < 0.0 && c.angvel.length() < 7.0 {
                self.hula = 1.0;
            } else if (c.m.c.z > 0.2 || c.m.b.z > 0.0) && self.hula < 0.5 {
                self.hula = -0.1;
            }
            if self.hula > 0.0 {
                self.hula -= dt;
            }
        } else {
            self.hula = -0.1;
        }
        let pedal = if self.hula > 0.0 {
            "radius=1 radiusVariance=0 angularSpeed=8 centreUp=-0.25 legAngleVariance=0 backPedal=true speedAsymmetry=0".to_string()
        } else {
            format!("radius={} angularSpeed={} centreUp=-0.1 radiusVariance=0.4 legAngleVariance=0.5 speedAsymmetry={} backPedal={}",
                    p.leg_radius, p.leg_ang_speed, p.leg_asymmetry, self.back_pedal)
        };
        self.keep("pedalLegs", format!("pedalLegs start=true pedalLeftLeg=true pedalRightLeg=true legStiffness={b} adaptivePedal4Dragging=false \
                                        pedalOffset=0 centreForwards=0 {pedal}"));
        let phase = p.arm_phase - if p.arms2legs_sync != 0 { p.arms2legs_phase } else { 0.0 };
        self.keep("armsWindmillAdaptive", format!("armsWindmillAdaptive start=true bodyStiffness={b} armStiffness={} period={} amplitude={} \
                                                   phase={phase} disableOnImpact=false", b * 14.0 / 11.0, TAU / p.arm_ang_speed.max(0.01), p.arm_amplitude));
    }

    /// S1: curled up, the legs pedalling slowly under him.
    fn crash(&mut self, c: &mut Ctx) {
        self.ensure("bodyFoetal", "bodyFoetal start=true stiffness=10 mask=ub".into());
        self.ensure("pedalLegs", "pedalLegs start=true pedalLeftLeg=true pedalRightLeg=true radius=0.2 angularSpeed=3 backPedal=false \
                                  legStiffness=9 radiusVariance=0.2 adaptivePedal4Dragging=false speedAsymmetry=1 pedalOffset=0.05".into());
        let k = 0.95 - c.up.dot(c.m.b).clamp(0.85, 0.95);
        self.out.push(format!("pedalLegs start=true centreForwards={} centreUp={}", 1.5 * k - 0.1, 3.0 * k + 0.05));
        if c.vel.length() < 0.05 {
            self.feedback(c, true);
        }
    }

    /// S2: catchFall early (armsUp -2 or lower, landing any way but feet first), else the arms up over the head or
    /// out in front of the chest; the legs pedal under him.
    fn brace(&mut self, c: &mut Ctx) {
        let p = self.p;
        let b = p.body_stiffness;
        let up = c.up;
        let mut ik = None;
        if p.arms_up > -3.0 || self.class != 0 {
            if p.arms_up <= -2.0 {
                let l = self.catch_fall_line();
                self.ensure("catchFall", l);
            } else {
                let head = c.b.part(delirium_data::kv!(55 int)).m;
                let r = head.d + unit(up) * p.arms_up + unit(head.a) * 0.3;
                ik = Some([r - unit(head.a) * 0.6, r]);
            }
        } else {
            let chest = c.b.part(CHEST()).m;
            let at = |y: f32| chest.d + chest.a * 0.43 + chest.b * y - chest.c * 0.32;
            ik = Some([at(0.38), at(-0.38)]);
        }
        if let Some(t) = ik {
            c.ch.eff[(*NECK())[0]].desired[0] = 0.4;
            c.ch.eff[(*NECK())[1]].desired[0] = 0.3;
            for (i, side) in [(0, 1.0), (1, -1.0)] {
                let l = limb(i);
                let vel = c.b.part(l.p[0]).v;
                crate::nmdbm::limb_ik_w(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[3], t[i], 0.0, 1.0, side, 1.0, vel, None, 1.0, false);
            }
        }
        for &j in (*KNEE()).iter().chain((*HIP()).iter()) {
            c.ch.eff[j].muscle_stiffness = 0.5;
        }
        self.ensure("pedalLegs", format!("pedalLegs start=true pedalLeftLeg=true pedalRightLeg=true radius={} angularSpeed={} backPedal=false \
                                          legStiffness={b} adaptivePedal4Dragging=false speedAsymmetry=4 pedalOffset=0 centreForwards={} centreUp=0 \
                                          radiusVariance=0.1", p.leg_radius, p.leg_ang_speed, p.forward_offset));
    }

    /// S4 and S6: catchFall, soft all over when not balancing.
    fn catch(&mut self, c: &mut Ctx) {
        let l = self.catch_fall_line();
        self.ensure("catchFall", l);
        if !self.p.balance {
            c.ch.set_stiffness(6.0, 1.0, "fb", Some(0.5));
        }
        if c.vel.length() < 0.075 {
            self.feedback(c, true);
        }
    }

    /// S5: tucked in, stiffening down from bodyStiffness over the roll, rollDownStairs' helper torques under it.
    fn roll(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.p;
        self.t110 += dt;
        let ch = &mut *c.ch;
        for &j in &(*SPINE()) {
            ch.eff[j].desired[0] = 1.0;
        }
        for (i, e) in [(0, 1.6), (1, 1.4)] {
            ch.eff[(*CLAVICLE())[i]].desired = [0.0; 3];
            ch.eff[(*SHOULDER())[i]].desired = [2.0, -2.0, -0.8];
            ch.eff[(*ELBOW())[i]].desired[0] = e;
        }
        ch.eff[(*HIP())[0]].desired[0] = 2.25;
        ch.eff[(*HIP())[1]].desired[0] = 2.3;
        for &j in &(*NECK()) {
            ch.eff[j].desired = [3.0, 0.0, 0.0];
        }
        self.smooth = 0.9 * self.smooth + 0.1 * c.vel.length();
        ch.set_stiffness((p.body_stiffness - 12.0 * self.t110).clamp(8.0, 12.0), p.body_damping, "fb", None);
        let ub = c.up.dot(c.m.b);
        ch.eff[(*KNEE())[0]].desired[0] = ub - 1.75;
        ch.eff[(*KNEE())[1]].desired[0] = ub - 1.65;
        if p.zero_pose_roll {
            for e in ch.eff.iter_mut() {
                e.blend_to_zero(1.0);
            }
        }
        self.ensure("rollDownStairs", "rollDownStairs start=true forcemag=1 spinWhenInAir=false onlyApplyHelperForces=true".into());
        if self.t110 > 1.6 {
            self.feedback(c, true);
        }
    }

    /// S7: bodyBalance with RDR2's landing settings, the hips crouching then rising over the first half second and
    /// the legs strengthening; true when the balancer has given up.
    fn land(&mut self, c: &mut Ctx, dt: f32) -> bool {
        let p = self.p;
        let b = p.body_stiffness;
        if !self.on.contains(&"bodyBalance") {
            c.out.dbm_taper = Some(false);
            // (ours [P]: RDR2's look target is (0, 0, -4) in his own frame; here 4 m below his COM, where he lands)
            let look = c.com - c.up * 4.0;
            let turn = c.ch.random() < 0.5;
            self.ensure("bodyBalance", format!("bodyBalance start=true useHeadLook=true headLookPos={} spineStiffness=10 spineDamping=1 \
                                                useBodyTurn={turn} somersaultAngleThreshold=0.25 sideSomersaultAngleThreshold=0.25 \
                                                angVelThreshold=1.2,3,1.2 bodyStiffness={} elbow=1.2 angVelMultiplier=2.5,0.9,2.5 shoulder=0.9 \
                                                returningToBalanceArmsOut=0.1 armsOutStraightenElbows=0 headLookAtVelProb=0.9 turnOffProb=0.05 \
                                                turn2TargetProb=0.05 turn2VelProb=0.7 turnAwayProb=0 turnLeftProb=0.1 turnRightProb=0.1 damping=0.7 \
                                                somersaultAngle=1 sideSomersaultAngle=1 armsOutOnPush=true armsOutOnPushMultiplier=1 \
                                                armsOutOnPushTimeout=1 armsOutMinLean2=-10 elbowAngleOnContact=0.7 bendElbowsTime=0 bendElbowsGait=0.7",
                                               v3s(look), 9.0 / 11.0 * b));
            (self.t110, self.seen_balancer) = (0.0, false);
        }
        self.t110 += dt;
        self.seen_balancer |= c.dbm.is_some();
        if !p.balance {
            c.ch.set_stiffness(6.0, 1.0, "fb", Some(0.5));
            for &j in (*HIP()).iter().chain((*KNEE()).iter()) {
                c.ch.eff[j].set_stiffness(6.0, 1.0, Some(0.5));
            }
        } else if c.dbm.is_some() {
            c.out.dbm_taper = Some(false);
            let t = (self.t110 - 0.5).clamp(-0.5, 0.0);
            // owner 2, below 3; our hip pitch is RDR2's negated
            c.out.dbm_crouch.push(Crouch::Hip(2, -t, f32::INFINITY));
            let k = (((self.t110 + 1.0).powi(2) - 0.5) * p.leg_strength.clamp(1.0, 12.0) / 12.0).clamp(0.5, 1.0);
            c.out.dbm_knee_scale = Some(k);
        }
        // the balancer has given up (ours: once it ran, or after half a second without one)
        let failed = !c.upright && (self.seen_balancer || self.t110 > 0.5);
        if c.vel.length() < 0.15 {
            self.feedback(c, failed);
        }
        failed
    }
}

impl Extra for HighFall {
    fn probe(&mut self, c: &Ctx, world: &crate::collision::Collision, obs: &[crate::ragdoll::Obstacle]) {
        if self.leg_len == 0.0 {
            let (h, k, a) = (c.b.joint_pos((*HIP())[0]), c.b.joint_pos((*KNEE())[0]), c.b.joint_pos((*ANKLE())[0]));
            self.leg_len = (h - k).length() + (k - a).length();
        }
        match self.state {
            0 => self.ground_probe(c, world, obs),
            3 => self.down = crate::nmcf::cast(world, obs, c.com, -c.up, 0.6).map(|(_, n, _)| n.dot(c.up)),
            _ => {}
        }
    }

    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        self.tick_inner(c, dt);
    }

    /// RDR2's deactivate: taperKneeStrength back on, the leg strength scale 1, the hip crouch let go, the state left,
    /// and everything it runs stopped.
    fn stop(&mut self, c: &mut Ctx) {
        c.out.dbm_taper = Some(true);
        c.out.dbm_knee_scale = Some(1.0);
        c.out.dbm_crouch.push(Crouch::ReleaseHip(f32::INFINITY));
        self.exit(c, self.state);
        for n in std::mem::take(&mut self.on) {
            self.out.push(format!("{n} start=false"));
        }
    }

    fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.out)
    }
}

// ---- balancerCollisionsReaction ----

/// How far ahead the wall probes look, in seconds of his horizontal speed (RDR2's unnamed 0.5), and the speed they
/// need to run.
const BCR_PROBE_TIME: f32 = 0.5;
const BCR_PROBE_SPEED: f32 = 0.5;
/// An impact this far (rad) off head-on is a glance.
const BCR_GLANCE: f32 = 0.6;
/// Ours [G] (RDR2's part contact probes aren't read): a part's contact is on the wall within this of its plane and
/// with its normal this close to the wall's.
const BCR_ON_WALL: f32 = 0.1;
const BCR_NORMAL: f32 = 0.7;

#[derive(Clone, Copy, Debug)]
struct BcrParams {
    /// objectBehindVictim: the wall given (point, normal)
    behind: Option<(Vec3, Vec3)>,
    steps: i32,
    zone: f32,
    leg_red: f32,
    slump_red: f32,
    slump_rate: f32,
    spin_time: f32,
    mass: f32,
    volume: f32,
    rebound_mode: i32,
    forward_mode: i32,
    to_forward: f32,
    rebound: f32,
    brace: bool,
}

/// A probe's hit: the point, the wall's normal there (out of it), the distance along the probe and the object's
/// velocity at the point.
#[derive(Clone, Copy, Debug)]
struct BcrHit {
    at: Vec3,
    n: Vec3,
    dist: f32,
    vel: Vec3,
}

/// RDR2's balancerCollisionsReaction (with the RDR2 core), from a partial study of RDR2's code
/// (rdr2-decomp/study/rdr2_bcr.md): while nothing is met, two sphere probes look half a second ahead along his
/// horizontal velocity for a wall (moving objects both light and small don't count); the wall becomes a plane the
/// balancer keeps his predicted COM off (exclusionZone, more once he has hit it); a part touching it is the impact,
/// which publishes impactLegStiffReduction for staggerFall, bends the knees through the balancer (owner 8) and turns
/// the head to the wall. RDR2's states: 0 looking, 1 impact, 2 moving off it, 4 slump, 5 glance (side-on, back to 1
/// after glanceSpinTime), 6 come off the wall (reboundMode decides), 12 finished; he slumps after
/// numStepsTillSlump steps from the impact, timeToForward after a forward impact, or once the balancer gives up.
pub struct BcrRdr2 {
    p: BcrParams,
    state: u8,
    hits: [Option<BcrHit>; 2],
    speed: f32,
    /// the wall: a point on it and its horizontal normal toward him; the object's velocity; time to reach it
    wall: Option<(Vec3, Vec3)>,
    wall_vel: Vec3,
    tte: f32,
    /// the impact latched (+0x27b) and this phase's (+0x27c), the time since, the balancer's steps then
    impacted: bool,
    phase: bool,
    t_hit: f32,
    steps_at: u32,
    /// still balancing (+0x62c), and how long something other than his feet has touched since it stopped
    balancing: bool,
    nonfoot: Option<f32>,
    slump_t: f32,
    started: bool,
    look: Option<Vec3>,
}

impl BcrRdr2 {
    pub fn new(p: &mut P) -> Result<BcrRdr2, Error> {
        let f = |p: &mut P, n: &str, d: f32| -> Result<f32, Error> { p.get(n).map_or(Ok(d), Value::f) };
        let b = |p: &mut P, n: &str, d: bool| -> Result<bool, Error> { p.get(n).map_or(Ok(d), Value::b) };
        let i = |p: &mut P, n: &str, d: i32| -> Result<i32, Error> { p.get(n).map_or(Ok(d), Value::i) };
        let v = |p: &mut P, n: &str| -> Result<Vec3, Error> { p.get(n).map_or(Ok(Vec3::ZERO), Value::v) };
        let (at, n) = (v(p, "objectBehindVictimPos")?, v(p, "objectBehindVictimNormal")?);
        let p = BcrParams {
            behind: b(p, "objectBehindVictim", false)?.then_some((at, n)),
            steps: i(p, "numStepsTillSlump", 3)?, zone: f(p, "exclusionZone", 0.2)?, leg_red: f(p, "impactLegStiffReduction", 3.0)?,
            slump_red: f(p, "slumpLegStiffReduction", 1.0)?, slump_rate: f(p, "slumpLegStiffRate", 8.0)?,
            spin_time: f(p, "glanceSpinTime", 0.5)?, mass: f(p, "ignoreColMassBelow", 20.0)?, volume: f(p, "ignoreColVolumeBelow", 0.1)?,
            rebound_mode: i(p, "reboundMode", 0)?, forward_mode: i(p, "forwardMode", 0)?, to_forward: f(p, "timeToForward", 0.5)?,
            rebound: f(p, "reboundForce", 0.0)?, brace: b(p, "braceWall", false)?,
        };
        Ok(BcrRdr2 { p, state: 0, hits: [None; 2], speed: 0.0, wall: None, wall_vel: Vec3::ZERO, tte: 0.0, impacted: false, phase: false,
                     t_hit: 0.0, steps_at: 0, balancing: true, nonfoot: None, slump_t: 0.0, started: false, look: None })
    }

    /// The probes (state 0, moving faster than 0.5 m/s): spheres of 0.25 m from the pelvis and 0.1 m from the chest
    /// along his horizontal velocity for half a second of it.
    pub fn probe(&mut self, c: &Ctx, world: &crate::collision::Collision, obs: &[crate::ragdoll::Obstacle]) {
        self.hits = [None; 2];
        if self.state != 0 {
            return;
        }
        let vh = c.vel - c.up * c.up.dot(c.vel);
        self.speed = vh.length();
        if self.speed <= BCR_PROBE_SPEED {
            return;
        }
        let (d, len) = (vh / self.speed, BCR_PROBE_TIME * self.speed);
        for (i, (k, r)) in [(0, 0.25), (crate::nmshot::chest_part(), 0.1)].into_iter().enumerate() {
            self.hits[i] = self.sweep(world, obs, c.b.at(k), d, r, len);
        }
    }

    /// The nearest hit of a sphere swept from `from` along `d`: the world, and the objects but those both lighter
    /// than ignoreColMassBelow and smaller than ignoreColVolumeBelow (fixed ones always count; ours: an object's box
    /// grown by the radius stands for the sphere).
    fn sweep(&self, world: &crate::collision::Collision, obs: &[crate::ragdoll::Obstacle], from: Vec3, d: Vec3, r: f32, len: f32) -> Option<BcrHit> {
        let (to, pad) = (from + d * len, Vec3::splat(r));
        let tris: Vec<[Vec3; 3]> = world.tris_near(from.min(to) - pad, from.max(to) + pad).into_iter().map(|t| t.0).collect();
        let mut best = crate::collision::sphere_sweep(&tris, from, d, r, len)
            .map(|(t, q)| BcrHit { at: q, n: unit(from + d * t - q), dist: t, vel: Vec3::ZERO });
        for ob in obs {
            if ob.inv_mass > 0.0 && 1.0 / ob.inv_mass < self.p.mass && 8.0 * ob.half.x * ob.half.y * ob.half.z < self.p.volume {
                continue;
            }
            let inv = ob.rot.inverse();
            if let Some((t, nl)) = crate::nmcf::ray_box(inv * (from - ob.centre), inv * d, ob.half + pad) {
                if t <= len && best.is_none_or(|b| t < b.dist) {
                    let n = ob.rot * nl;
                    let q = from + d * t - n * r;
                    best = Some(BcrHit { at: q, n, dist: t, vel: ob.vel_at(q) });
                }
            }
        }
        best
    }

    pub fn tick(&mut self, c: &mut Ctx, dt: f32, out: &mut Vec<String>) {
        let up = c.up;
        let flat = |v: Vec3| unit(v - up * up.dot(v));
        if !self.started {
            self.started = true;
            // ours: RDR2 has objectBehindVictimPos/Normal, but where it reads them isn't found
            if let Some((at, n)) = self.p.behind {
                let n = flat(if n == Vec3::ZERO { c.com - at } else { n });
                self.wall = (n != Vec3::ZERO).then_some((at, n));
            }
        }
        let was = self.state;
        if self.impacted {
            self.t_hit += dt;
        }
        // balancing while the balancer hasn't given up; after that, how long anything but his feet has touched
        self.balancing &= c.upright;
        if !self.balancing {
            let feet = [limb(2).p[3], limb(3).p[3]];
            let parts: Vec<usize> = all_parts(c).into_iter().filter(|k| !feet.contains(k)).collect();
            if self.nonfoot.is_some() || c.b.doll.collided(&parts) {
                self.nonfoot = Some(self.nonfoot.unwrap_or(0.0) + dt);
            }
        }
        // the wall from the probes: the chest's hit if it has one, else the pelvis's, its normal made horizontal
        if self.state == 0 {
            if let Some(h0) = self.hits[0] {
                let h = self.hits[1].unwrap_or(h0);
                let n = flat(h.n);
                if n != Vec3::ZERO {
                    (self.wall, self.wall_vel, self.tte) = (Some((h.at, n)), h.vel, h.dist / self.speed.max(1e-6));
                }
            }
        }
        self.machine(c, out);
        let speed = (c.vel - up * up.dot(c.vel)).length();
        if let Some((at, n)) = self.wall {
            // the balancer's exclusion plane (BCR's wins over teeter's once he has hit it)
            let off = match self.state {
                5 | 10 => 0.4,
                6 => 0.1,
                _ => self.p.zone + if self.impacted { (0.2 * speed).min(0.5) } else { 0.0 },
            };
            if self.impacted || c.out.dbm_plane.is_none() {
                c.out.dbm_plane = Some((at, n, -1.0, off));
            }
            // coming up to it with braceWall: look at it while it is more than half a second away
            let bracing = self.p.brace && self.balancing && (self.p.rebound_mode != 2 || self.state == 0);
            if !self.impacted && self.state != 12 && bracing && self.tte > 0.5 {
                self.look_at(at, out);
            }
            if self.impacted {
                // the head to the wall at chest height, while he stands (ours: RDR2 waits for catchFall and
                // bodyRollUp to be idle)
                if c.upright {
                    let chest = c.b.at(crate::nmshot::chest_part());
                    self.look_at(at + up * up.dot(chest - at), out);
                }
                // the knee bend: the spine root's height over the lower foot less 0.1, at most 0.35, at 4/s
                if self.phase && c.dbm.is_some() {
                    let low = [2, 3].map(|i| up.dot(c.b.at(limb(i).p[3]))).into_iter().fold(f32::MAX, f32::min);
                    let k = (up.dot(c.b.joint_pos((*SPINE())[0])) - low - 0.1).clamp(0.0, 0.35);
                    c.out.dbm_crouch.push(Crouch::Knee(8, -k, 4.0));
                }
            }
        }
        // the slump once the balancer has given up (unless he has been down on something more than 0.3 s)
        if !self.balancing && !matches!(self.state, 0 | 12) && self.nonfoot.unwrap_or(0.0) <= 0.3 {
            self.state = 4;
        }
        if self.state == 4 {
            self.slump_t += dt;
            if c.dbm.is_some() {
                self.slump(c);
            }
        }
        c.ch.bcr = Some((self.state, self.impacted, if self.impacted { self.p.leg_red } else { 0.0 }));
        if self.state != was {
            out.push(format!("#say bcr state {}", self.state));
        }
    }

    /// RDR2's state machine (0x2aeaa50), as far as it is read.
    fn machine(&mut self, c: &mut Ctx, out: &mut Vec<String>) {
        let steps = c.dbm.map_or(0, |d| d.steps);
        // the slump by steps since the impact (ours [P]: counted only after it)
        if self.impacted && steps.saturating_sub(self.steps_at) as i64 >= self.p.steps as i64 {
            self.state = 4;
            if (self.p.rebound_mode == 0 && !c.shot_on) || self.p.rebound_mode == 2 {
                return;
            }
        }
        if !self.impacted && !self.balancing {
            self.state = 12;
            return;
        }
        if matches!(self.state, 6 | 12) {
            return;
        }
        let Some((at, n)) = self.wall else { return };
        let w = -n;
        if !self.impacted {
            let hit = (0..c.b.doll.hit.len()).filter_map(|k| c.b.doll.hit[k])
                .find(|&(q, cn, _)| (q - at).dot(n).abs() < BCR_ON_WALL && cn.dot(n).abs() > BCR_NORMAL);
            if let Some((q, _, _)) = hit {
                (self.impacted, self.phase, self.state, self.t_hit, self.steps_at) = (true, true, 1, 0.0, steps);
                // the impact angle: his velocity against the wall's inward direction (ours [P] on the vectors)
                let v = unit((c.vel - self.wall_vel) - c.up * c.up.dot(c.vel - self.wall_vel));
                if v != Vec3::ZERO && w.dot(v).clamp(-1.0, 1.0).acos() > BCR_GLANCE {
                    self.state = 5;
                }
                self.wall = Some((q, n));
            }
        }
        if matches!(self.state, 5 | 10) && self.t_hit > self.p.spin_time {
            self.state = 1;
        }
        let at = self.wall.map_or(at, |w| w.0);
        if self.state == 1 {
            // facing the wall (ours: his forward axis is the trunk's back axis negated)
            if (-c.m.c).dot(w) >= 0.0 {
                let chest = crate::nmshot::chest_part();
                let e = (c.b.at(chest) - at).dot(n);
                if self.p.rebound > 0.0 && e > 0.0 && e < 0.4 {
                    c.out.impulse.push((chest, c.b.at(chest), n * self.p.rebound));
                    for i in 0..2 {
                        let k = limb(i).p[1];
                        c.out.impulse.push((k, c.b.at(k), n * (-0.25 * self.p.rebound)));
                    }
                }
                // forwardMode 1 while shot: RDR2's fall-to-knees preset instead ([G], left out: he stays)
                if self.t_hit > 0.1 && self.t_hit > self.p.to_forward && !(self.p.forward_mode == 1 && c.shot_on) {
                    self.state = 4;
                }
            } else {
                self.state = 2;
            }
        }
        // coming off the wall: moving away from it faster than 0.2 m/s with the COM 0.4 m out
        if self.impacted && !matches!(self.state, 5 | 10) && (c.vel - self.wall_vel).dot(n) > 0.2 && (c.com - at).dot(n) > 0.4 {
            self.state = 6;
            match self.p.rebound_mode {
                0 if !c.shot_on => self.state = 4,
                2 => self.state = 4,
                3 => self.restart(c, out),
                _ => {}
            }
        }
    }

    /// reboundMode 3: back to looking for a wall; catchFall and bodyRollUp stopped if he is still balancing; the
    /// knee bend let go.
    fn restart(&mut self, c: &mut Ctx, out: &mut Vec<String>) {
        (self.state, self.impacted, self.phase, self.t_hit, self.wall, self.tte, self.slump_t) = (0, false, false, 0.0, None, 0.0, 0.0);
        c.out.dbm_crouch.push(Crouch::ReleaseKnee(1.0));
        if self.balancing {
            out.extend(["catchFall start=false".to_string(), "bodyRollUp start=false".to_string()]);
        }
    }

    /// The slump's leg weakening (ours [G]: RDR2's slump core isn't read): the legs' stiffness from the balancer
    /// less slumpLegStiffReduction, reached at slumpLegStiffRate a second.
    fn slump(&self, c: &mut Ctx) {
        let r = (self.p.slump_rate * self.slump_t).min(self.p.slump_red).max(0.0);
        for i in 0..2 {
            for j in [(*HIP())[i], (*KNEE())[i], (*ANKLE())[i]] {
                let e = &mut c.ch.eff[j];
                let s = e.strength.max(0.0).sqrt();
                if s > 1e-6 {
                    let k = (s - r).max(0.0) / s;
                    (e.strength, e.damping) = (e.strength * k * k, e.damping * k);
                }
            }
        }
    }

    fn look_at(&mut self, at: Vec3, out: &mut Vec<String>) {
        if self.look.is_none_or(|l| l.distance(at) > 0.05) {
            self.look = Some(at);
            out.push(format!("headLook start=true alwaysLook=true pos={}", v3s(at)));
        }
    }

    /// Deactivate: impactLegStiffReduction withdrawn, the knee bend let go.
    pub fn stop(&mut self, c: &mut Ctx) {
        c.ch.bcr = None;
        c.out.dbm_crouch.push(Crouch::ReleaseKnee(1.0));
    }
}
