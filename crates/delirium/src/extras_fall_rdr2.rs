//! RDR2's teeter, highFall and fallOverWall, with the RDR2 core, from clean-room studies of RDR2's behaviours
//! (rdr2-decomp/study/rdr2_teeter.md, rdr2_highfall.md, rdr2_fallwall.md). Teeter is the RDR2 path of the teeter
//! extra; highFall and fallOverWall run as extras only with the RDR2 core (msg.rs sends them here), GTA IV's
//! (nmhf.rs, nmfow.rs) otherwise.
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

// ---- fallOverWall ----

/// RDR2's fallOverWall parameters, RDR2's defaults where none is given. wallhandle, forceAngleAbort (unread by RDR2)
/// and stepExclusionZone (balancerCollisionsReaction's, not modelled) are read and unused, and so are GTA IV's own
/// fallOverPoint, fallOverEdge and useSpecifiedPoint: the wall is the segment fallOverWallEndA-EndB in the world.
#[derive(Clone, Copy, Debug)]
pub struct FowParams {
    end_a: Vec3,
    end_b: Vec3,
    damping: f32,
    body_stiffness: f32,
    mag_of_force: f32,
    force_time_out: f32,
    min_leg_height: f32,
    max_dist: f32,
    max_force_dist: f32,
    /// the twist torque's range, 45 and 85 times bodyTwist
    twist: (f32, f32),
    max_twist: f32,
    move_arms: bool,
    move_legs: bool,
    bend_spine: bool,
    leaning: f32,
    leg_twist: f32,
    leg_amp: f32,
    spin: f32,
    angle_dir: f32,
    max_ang_vel: f32,
    adapt: bool,
    max_wall_height: f32,
    success_dist: f32,
    rolling_back: f32,
    rolling_potential: f32,
}

/// RDR2's leg swing noise rate (an unnamed parameter, hash 0x31a7053c, default 2).
const FOW_LEG_RATE: f32 = 2.0;

/// RDR2's fallOverWall arm pose per arm: elbow, clavicle (lean1, lean2), shoulder (lean1, lean2, twist).
const FOW_ARMS: [(f32, [f32; 2], [f32; 3]); 2] = [
    (1.12, [-0.02269, -1.45534], [0.14776, -1.43301, -1.73880]),
    (1.024, [-0.04673, -1.65534], [0.11821, -1.52854, -1.79447]),
];

impl FowParams {
    fn read(p: &mut P) -> Result<FowParams, Error> {
        let f = |p: &mut P, n: &str, d: f32| -> Result<f32, Error> { p.get(n).map_or(Ok(d), Value::f) };
        let b = |p: &mut P, n: &str, d: bool| -> Result<bool, Error> { p.get(n).map_or(Ok(d), Value::b) };
        let v = |p: &mut P, n: &str| -> Result<Vec3, Error> { p.get(n).map_or(Ok(Vec3::ZERO), Value::v) };
        p.get("wallhandle").map(Value::i).transpose()?;
        f(p, "forceAngleAbort", -0.2)?;
        f(p, "stepExclusionZone", 0.5)?;
        v(p, "fallOverPoint")?;
        v(p, "fallOverEdge")?;
        b(p, "useSpecifiedPoint", false)?;
        let bt = f(p, "bodyTwist", 0.54)?;
        Ok(FowParams {
            end_a: v(p, "fallOverWallEndA")?, end_b: v(p, "fallOverWallEndB")?, damping: f(p, "damping", 0.5)?,
            body_stiffness: f(p, "bodyStiffness", 9.0)?, mag_of_force: f(p, "magOfForce", 0.5)?, force_time_out: f(p, "forceTimeOut", 2.0)?,
            min_leg_height: f(p, "minLegHeight", 0.4)?, max_dist: f(p, "maxDistanceFromPelToHitPoint", 0.25)?,
            max_force_dist: f(p, "maxForceDist", 0.8)?, twist: (45.0 * bt, 85.0 * bt), max_twist: f(p, "maxTwist", std::f32::consts::PI)?,
            move_arms: b(p, "moveArms", true)?, move_legs: b(p, "moveLegs", true)?, bend_spine: b(p, "bendSpine", true)?,
            leaning: f(p, "leaningAngleThreshold", 180.0)?, leg_twist: f(p, "legtwist", 1.0)?, leg_amp: f(p, "legamp", 1.0)?,
            spin: f(p, "zAxisSpinReduction", 0.0)?, angle_dir: f(p, "angleDirWithWallNormal", 180.0)?, max_ang_vel: f(p, "maxAngVel", 2.0)?,
            adapt: b(p, "adaptForcesToLowWall", false)?, max_wall_height: f(p, "maxWallHeight", -1.0)?,
            success_dist: f(p, "distanceToSendSuccessMessage", -1.0)?, rolling_back: f(p, "rollingBackThr", 0.5)?,
            rolling_potential: f(p, "rollingPotential", 0.3)?,
        })
    }
}

/// RDR2's fallOverWall (rdr2_fallwall.md), with the RDR2 core (msg.rs sends it here; GTA IV's is nmfow.rs). The wall
/// is the segment A-B; its normal n (horizontal, up x d) points to the side the pelvis starts on, and W is the
/// wall's top above the pelvis. Until a contact latches it (pelvis or low spine at once; a thigh, hand, forearm or
/// the third spine part only within maxDistanceFromPelToHitPoint of the wall, horizontally) and the start tests pass
/// (feet out from the pelvis by minLegHeight, the lean and approach angle thresholds, maxWallHeight), it only
/// watches. Then by the approach speed against rollingPotential: state 2, going over, or 1, not enough momentum.
/// In state 2: whole-body stiffness, the fixed arm pose, the spine bent by the pelvis' tilt to the wall, the legs
/// swung up by noise-driven IK, forces turning each spine part about the wall's edge (forceTimeOut, maxForceDist,
/// maxAngVel) and the twist torque that keeps him rolling the way he twists. States 3 over, 4 stalled, 5 fell back;
/// fallOverWallState on each change and the fallOverWall feedback (1, 4, 5 failure, 3 success) every tick.
pub struct FallOverWall {
    p: FowParams,
    started: bool,
    a: Vec3,
    b: Vec3,
    d: Vec3,
    n: Vec3,
    contact: Vec3,
    w: Vec3,
    wp: Vec3,
    /// the time in state 2, the twist about the spine, the approach speed (and whether it is known)
    t2: f32,
    twist: f32,
    approach: Option<f32>,
    /// noise offsets (the third also the noise time)
    noise: [f32; 3],
    state: u8,
    feet: [bool; 2],
    latched: bool,
    past: bool,
    over: bool,
    rolling_back: bool,
    speed: f32,
    upright: f32,
    /// angular momentum about the COM
    l: Vec3,
}

impl FallOverWall {
    pub fn new(p: &mut P) -> Result<FallOverWall, Error> {
        Ok(FallOverWall { p: FowParams::read(p)?, started: false, a: Vec3::ZERO, b: Vec3::ZERO, d: Vec3::ZERO, n: Vec3::ZERO,
                          contact: Vec3::ZERO, w: Vec3::ZERO, wp: Vec3::ZERO, t2: 0.0, twist: 0.0, approach: None, noise: [0.0; 3],
                          state: 0, feet: [false; 2], latched: false, past: false, over: false, rolling_back: false, speed: 0.0,
                          upright: 0.0, l: Vec3::ZERO })
    }

    /// The wall's top above `p`: p's horizontal foot on the line A-B (not clamped to the segment), at the line's
    /// height there (A's plus the slope along it).
    fn top(&self, p: Vec3, up: Vec3) -> Vec3 {
        let flat = |v: Vec3| v - up * up.dot(v);
        let (ah, e) = (flat(self.a), flat(self.b) - flat(self.a));
        let len = e.length();
        if len <= 1e-6 {
            return flat(p) + up * up.dot(self.a);
        }
        let along = (flat(p) - ah).dot(e / len);
        ah + e / len * along + up * (up.dot(self.a) + (up.dot(self.b) - up.dot(self.a)) / len * along)
    }

    /// The distance from `p` to the segment A-B.
    fn to_segment(&self, p: Vec3) -> f32 {
        let e = self.b - self.a;
        let l2 = e.length_squared();
        let t = if l2 > 1e-12 { ((p - self.a).dot(e) / l2).clamp(0.0, 1.0) } else { 0.0 };
        (p - (self.a + e * t)).length()
    }

    /// The contact search and the start tests, while nothing has latched.
    fn search(&mut self, c: &mut Ctx, pel: Vec3, up: Vec3) {
        let p = self.p;
        let sp = crate::nmshot::spine_parts();
        let flat = |v: Vec3| v - up * up.dot(v);
        let record = |c: &Ctx, k: usize| c.b.doll.hit[k].map_or((c.b.part(k).m.d, Vec3::X), |(at, n, _)| (at, n));
        let mut found = [sp[0], sp[1], sp[2]].into_iter().find(|&k| c.b.doll.collided(&[k])).map(|k| record(c, k));
        if found.is_none() && flat(self.wp).length() < p.max_dist {
            found = [limb(2).p[1], limb(3).p[1], limb(0).p[3], limb(1).p[3], limb(1).p[2], limb(0).p[2], sp[3]].into_iter()
                .find(|&k| c.b.doll.collided(&[k])).map(|k| (self.w, record(c, k).1));
        }
        let Some((at, normal)) = found else { return };
        (self.latched, self.contact) = (true, at);
        if self.approach.is_none() {
            // the upper body's speed towards the wall (the ground still)
            let v: Vec3 = sp[2..].iter().map(|&k| c.b.part(k).v).sum();
            self.approach = Some(-0.2 * self.n.dot(v));
        }
        let feet = [limb(2).p[3], limb(3).p[3]].map(|k| c.b.part(k).m.d);
        let lf = feet.iter().map(|&f| flat(f - pel).length()).fold(0.0, f32::max);
        let nc = if normal.dot(up).abs() > 0.7 { self.n } else { normal };
        let lean = up.dot(self.n.cross(c.b.joint_pos(crate::nmshot::spine_joints()[4]) - pel).normalize_or_zero()).abs();
        let c1 = if p.leaning > 90.0 { 1.0 } else { p.leaning.to_radians().sin() };
        let c2 = if p.angle_dir > 90.0 { 1.0 } else { p.angle_dir.to_radians().cos().abs() };
        let toward = flat(c.vel).normalize_or_zero().dot(self.n) < c2;
        let low = p.max_wall_height < 0.0 || up.dot(self.a).min(up.dot(self.b)) - up.dot(feet[0]).min(up.dot(feet[1])) < p.max_wall_height;
        if lf > p.min_leg_height && lean < c1 && toward && low {
            // (the bend values are re-taken in state 2; RDR2 also draws a random bit here that nothing reads)
            c.ch.random();
        } else {
            self.latched = false;
        }
    }

    fn active(&mut self, c: &mut Ctx, dt: f32) {
        let (p, up) = (self.p, c.up);
        let sp = crate::nmshot::spine_parts();
        c.ch.set_stiffness(p.body_stiffness, p.damping, "fb", None);
        if c.m.b.dot(up) < 0.1 {
            crate::nmcf::spin_reduction(c, p.spin);
        }
        if p.move_arms {
            for (i, &(elbow, cl, sh)) in FOW_ARMS.iter().enumerate() {
                let l = limb(i);
                c.ch.eff[l.e[2]].desired[0] = elbow;
                (c.ch.eff[l.e[0]].desired[0], c.ch.eff[l.e[0]].desired[1]) = (cl[0], cl[1]);
                c.ch.eff[l.e[1]].desired = sh;
            }
        }
        if p.bend_spine {
            let pm = c.b.part(sp[0]).m;
            let (s170, s174) = (-pm.c.dot(self.n), -pm.a.dot(self.n));
            crate::extras_rdr2::spine_split(c, -2.0 * s170, s174);
        }
        if p.move_legs {
            self.noise[2] += dt;
            let (t, r, a, id) = (self.noise[2], FOW_LEG_RATE, p.leg_amp, c.ch.agent as f32);
            let noise = crate::nmfl::noise01;
            let n1 = noise(r * t, self.noise[0], id - 0.5);
            let n2 = noise(self.noise[1], id + 0.5, r * t);
            let n3 = noise(self.noise[0], r * t, id - 0.5);
            let n4 = noise(id + 0.5, self.noise[1], r * t);
            let u = self.upright.abs();
            let chest = c.b.part(sp[4]).m;
            let (r0, r1, r2) = (chest.a, chest.b, chest.c);
            let tau = (1.0 - u) * p.leg_twist * 1.7 * up.dot(r1);
            let lg = 1.016498 * (c.b.joint_pos((*KNEE())[0]) - c.b.joint_pos((*HIP())[0])).length();
            let kl = -0.98377 * (0.6 + 0.3 * (u - tau)).clamp(0.6, 0.99);
            let kr = -0.98377 * (0.6 + 0.3 * (u + tau)).clamp(0.6, 0.99);
            let to = [
                r0 * kl + r1 * (0.4 * n1 - 0.2) + r2 * (1.25 * (1.6 * a * n2 - 0.8 * a)),
                r0 * kr - r1 * (0.4 * n3 - 0.2) + r2 * (1.6 * a * n4 - 0.8 * a),
            ];
            for (k, side) in [(0usize, 1.0f32), (1, -1.0)] {
                let l = limb(k + 2);
                let touching = c.b.doll.collided(&[l.p[3]]);
                if !touching || !self.feet[k] {
                    let target = c.b.joint_pos(l.e[1]) + to[k] * lg;
                    crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[3], target, 0.0, 1.0, side, -1.0, Vec3::ZERO, None);
                }
                c.ch.eff[l.e[1]].desired[2] = 0.0;
                if !touching {
                    self.feet[k] = true;
                }
            }
        }
        if self.t2 < p.force_time_out {
            let mass = c.b.doll.mass().max(1e-3);
            for (k, wt) in [(sp[3], 0.8), (sp[4], 0.75), (sp[5], 0.5), (sp[6], 0.25), (sp[0], 1.0), (sp[1], 1.0), (sp[2], 0.9)] {
                let pos = c.b.part(k).m.d;
                let rel = pos - self.contact;
                if rel.length() >= p.max_force_dist {
                    continue;
                }
                let mut f = (60.0 * p.mag_of_force * c.b.doll.part_mass(k) / mass).clamp(-5.5, 5.5) * wt;
                if p.adapt {
                    f *= 0.2 + 0.5333 * (1.5 - self.to_segment(pos)).clamp(0.0, 1.5);
                }
                f *= 60.0;
                if p.max_ang_vel <= 0.0 || self.l.dot(self.d).abs() < p.max_ang_vel {
                    c.out.force(k, rel.normalize_or_zero().cross(self.d) * f);
                }
            }
        }
        if self.twist.abs() < p.max_twist {
            let se = crate::nmshot::spine_joints();
            let s = (c.b.joint_pos(se[3]) - c.b.joint_pos(se[0])).normalize_or_zero();
            self.twist += c.angvel.dot(s) * dt;
            let slow = |w: Vec3| w.dot(s).abs() < 4.0;
            if c.b.doll.collided(&sp[1..5]) && self.twist.abs() < p.max_twist && s.dot(up).abs() < 0.5
                && slow(c.b.part(sp[4]).w) && slow(c.b.part(sp[0]).w) && slow(c.angvel)
            {
                let sign = if self.twist > 0.0 { 1.0 } else if self.twist < 0.0 { -1.0 } else { 0.0 };
                let t = s * ((p.twist.0 + c.ch.random() * (p.twist.1 - p.twist.0)) * sign);
                for (k, g) in [(sp[3], 1.0), (sp[4], 1.0), (sp[1], 0.6), (sp[2], 0.6), (sp[0], -2.0)] {
                    c.out.torque(k, t * g);
                }
            }
        }
    }
}

impl Extra for FallOverWall {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        let (p, up) = (self.p, c.up);
        let pel = c.b.part(crate::nmshot::spine_parts()[0]).m.d;
        (self.a, self.b) = (p.end_a, p.end_b);
        if !self.started {
            // activate: the noise offsets, the wall's direction and normal (towards the pelvis' side)
            self.started = true;
            self.noise = [4000.0 * c.ch.random(), 4000.0 * c.ch.random(), 4000.0 * c.ch.random()];
            self.d = (self.a - self.b).normalize_or_zero();
            self.n = up.cross(self.d).normalize_or_zero();
            if (self.top(pel, up) - pel).dot(self.n) > 0.0 {
                (self.n, self.d) = (-self.n, -self.d);
            }
            c.out.send(EVENT(), "fallOverWallState", 0.0);
        }
        // the analysis
        self.w = self.top(pel, up);
        self.wp = self.w - pel;
        self.past = self.wp.dot(self.n) > 0.0;
        self.speed = (c.vel.length() + c.angvel.length()) * 0.5;
        if self.state == 2 {
            self.t2 += dt;
        }
        self.over = self.past && (pel - self.contact).dot(up) < -0.1;
        self.upright = up.dot(c.m.b);
        self.l = (0..c.b.build.parts.len()).map(|k| {
            let q = c.b.part(k);
            (q.m.d - c.com).cross(q.v - c.vel) * c.b.doll.part_mass(k)
        }).sum();
        let along = self.d.dot(self.l);
        self.rolling_back = along < 0.0 && along.abs() > p.rolling_back;
        if !self.latched {
            self.search(c, pel, up);
        }
        // the state
        let was = self.state;
        let back = (self.rolling_back || self.wp.length() > 0.5) && !self.past;
        self.state = match self.state {
            0 if self.latched => if self.approach.unwrap_or(0.0) > p.rolling_potential { 2 } else { 1 },
            2 if self.over => 3,
            2 if self.speed < 0.2 => 4,
            2 if back => 5,
            4 if self.over => 3,
            4 if back => 5,
            s => s,
        };
        if self.state != was {
            c.out.send(EVENT(), "fallOverWallState", self.state as f32);
        }
        match self.state {
            1 | 4 | 5 => c.out.send(FAILURE(), "fallOverWall", 0.0),
            3 if p.success_dist < 0.0 || self.wp.length() > p.success_dist => c.out.send(SUCCESS(), "fallOverWall", 0.0),
            _ => {}
        }
        if self.latched {
            self.contact = self.w;
            // the balancer's force balance, from the start (RDR2 sets it once; here it lasts while this runs)
            c.out.dbm_force_balance |= p.move_legs;
            if self.state == 2 {
                self.active(c, dt);
            }
        }
    }
}
