//! RDR2's teeter, highFall, balancerCollisionsReaction and fallOverWall, with the RDR2 core, from clean-room studies
//! of RDR2's behaviours (rdr2-decomp/study/rdr2_teeter.md, rdr2_highfall.md, rdr2_bcr.md, rdr2_fallwall.md). Teeter and
//! balancerCollisionsReaction are the RDR2 paths of their extras; highFall and fallOverWall run as extras only with the
//! RDR2 core (msg.rs sends them here), GTA IV's (nmhf.rs, nmfow.rs) otherwise.
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
    side_d: f32,
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
        for n in ["aimAngleBase", "startplayerset", "footVelCompScale", "legL", "armPeriod", "landCutOffVelocity", "fowardVelRotation",
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
            arm_phase: f(p, "armPhase", 3.1)?, forward_offset: f(p, "forwardOffsetOfLegIK", offset)?,
            side_d: f(p, "sideD", 0.2)?, leg_strength: f(p, "legStrength", 12.0)?,
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
        // pedalLegs' ellipse from the circling error (RDR2's pedalLegs default, 1, without adaptive circling)
        let mut ellipse = 1.0;
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
            ellipse = (e.abs() / 2.2).min(1.0);
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
            format!("hula=true radius=1 radiusVariance=0 angularSpeed=8 centreUp=-0.25 legAngleVariance=0 backPedal=true speedAsymmetry=0 ellipse={ellipse}")
        } else {
            format!("hula=false radius={} angularSpeed={} centreUp=-0.1 radiusVariance=0.4 legAngleVariance=0.5 speedAsymmetry={} backPedal={} \
                     ellipse={ellipse}", p.leg_radius, p.leg_ang_speed, p.leg_asymmetry, self.back_pedal)
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
                                  legStiffness=9 radiusVariance=0.2 adaptivePedal4Dragging=false speedAsymmetry=1 pedalOffset=0.05 \
                                  centresideways=-0.2 ellipse=-0.1".into());
        // set on the running pedalLegs (a start would reload its defaults under the core)
        let k = 0.95 - c.up.dot(c.m.b).clamp(0.85, 0.95);
        self.out.push(format!("pedalLegs centreForwards={} centreUp={}", 1.5 * k - 0.1, 3.0 * k + 0.05));
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
                                          radiusVariance=0.1 centresideways={}", p.leg_radius, p.leg_ang_speed, p.forward_offset, p.side_d));
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
/// The parts whose contacts make the impact (the list 0x2aeaa50 walks, in Delirium's part order: thighs, pelvis, spine,
/// clavicles, neck, head), and how far above the lower foot the contact has to be (0x2af9f74): no calf, foot or arm counts.
const BCR_PARTS: [usize; 11] = [1, 4, 0, 7, 8, 9, 10, 17, 13, 11, 12];
const BCR_ABOVE_FOOT: f32 = 0.1;

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
    stable: f32,
    react: f32,
    spin_mag: f32,
    spin_turn: f32,
    spin_decay: f32,
    spin_reduce: f32,
    spin_min: f32,
    slump_mode: i32,
    sit_start: f32,
    sit_rate: f32,
    fling: f32,
    arm_brace: bool,
    drape: bool,
    high: bool,
    weak_zero: f32,
    weak_ramp: f32,
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

/// RDR2's balancerCollisionsReaction (with the RDR2 core), from RDR2's code (rdr2-decomp/study/rdr2_bcr.md, §10 on):
/// while nothing is met, two sphere probes look half a second ahead along his horizontal velocity for a wall (moving
/// objects both light and small don't count); the wall becomes a plane the balancer keeps his predicted COM off
/// (exclusionZone, more once he has hit it); a part touching it is the impact, which publishes impactLegStiffReduction
/// for staggerFall, bends the knees through the balancer (owner 8) and turns the head from the wall. RDR2's states: 0
/// looking, 1 impact, 2 moving off it, 3 still against it, 4 slump, 5 glance (side-on, spinning, back to 1 after
/// glanceSpinTime), 6 come off the wall (reboundMode decides), 8/9 over a low wall, 10 glance over one, 12 finished.
/// Ours or left out: see rdr2_bcr.md §12.
pub struct BcrRdr2 {
    p: BcrParams,
    state: u8,
    hits: [Option<BcrHit>; 2],
    speed: f32,
    /// the wall: a point on it and its horizontal normal toward him; the object's velocity; time to reach it
    wall: Option<(Vec3, Vec3)>,
    wall_vel: Vec3,
    tte: f32,
    /// which side of the plane he is on (+0x300): -1 unless he was moving off it at the impact
    side: f32,
    /// the impact latched (+0x27b) and this phase's (+0x27c), the time since, the balancer's steps then
    impacted: bool,
    phase: bool,
    t_hit: f32,
    steps_at: u32,
    /// still balancing (+0x62c), and how long something other than his feet has touched since it stopped
    balancing: bool,
    nonfoot: Option<f32>,
    started: bool,
    look: Option<Vec3>,
    /// what the wall is (+0x274): 0 nothing above the contact, 1 a full wall, 2 a low wall's top, 3 not known
    kind: u8,
    /// the probe down the wall's face from above the contact (0x2af9c64, RDR2's is asynchronous): from, along; its hit
    second: Option<(Vec3, Vec3)>,
    second_hit: Option<Option<Vec3>>,
    /// contacts low (+0x27d) and high (+0x27e), the wall judged low (+0x27f), their normals' sums (+0x5e0, +0x5d0)
    touch_low: bool,
    touch_high: bool,
    low: bool,
    sum: [Vec3; 2],
    /// his velocity relative to the object at the impact (+0x5c0), his velocity last tick (+0x5b0)
    rel: Vec3,
    /// time in states 2 and 3 (+0x620)
    t_stable: f32,
    /// the weakness ramp (+0x618)
    weak: f32,
    /// the over-a-low-wall record (+0x4d0): time, its random delay drawn, he got back on his feet (+0x4f2)
    over_t: f32,
    over_drawn: bool,
    recovered: bool,
    /// fallOverWall was started from here (+0x633); rollDownStairs and bodyRollUp too (ours: RDR2 reads whether they run)
    fow_mine: bool,
    rds_on: bool,
    roll_on: bool,
    slump: Option<SlumpRec>,
    fling: Option<BcrFling>,
    /// arms near the wall (+0x473/+0x474), sliding along it (+0x471/+0x472), sinking (+0x470)
    near: [bool; 2],
    slide: [bool; 2],
    sink: bool,
}

/// The slump core's record (0x2a55ddc, +0x478): its clock, the last kinetic energy, the legs' stiffness ramps (hip L, R,
/// knee L, R). The friction ramps aren't here: Delirium's friction is per material, not per part.
#[derive(Clone, Copy, Debug)]
struct SlumpRec {
    t: f32,
    ke: f32,
    legs: [f32; 4],
}

/// fling2 (+0x390, block defaults 0x2a3b4e8, start 0x2a2d5dc): per arm whether it flings, the drawn angle and length,
/// when it relaxes, whether the hand has stopped; the clock.
#[derive(Clone, Copy, Debug)]
struct BcrFling {
    on: [bool; 2],
    angle: [f32; 2],
    length: [f32; 2],
    relax: [f32; 2],
    stopped: [bool; 2],
    hand: [Vec3; 2],
    t: f32,
}
const FL_TIME: f32 = 0.5;
const FL_BEFORE: f32 = 0.1;

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
            rebound: f(p, "reboundForce", 0.0)?, brace: b(p, "braceWall", false)?, stable: f(p, "stable2SlumpTime", 1.0)?,
            react: f(p, "reactTime", 0.3)?, spin_mag: f(p, "glanceSpinMag", 50.0)?, spin_turn: f(p, "glanceSpinTurn", 40.0)?,
            spin_decay: f(p, "glanceSpinDecayMult", 0.3)?, spin_reduce: f(p, "glanceSpinReduce", 1.5)?, spin_min: 0.3,
            slump_mode: i(p, "slumpMode", 1)?, sit_start: f(p, "sitstarttime", 0.3)?, sit_rate: f(p, "sitrate", 1.0)?,
            fling: f(p, "fling2prob", 0.5)?, arm_brace: b(p, "armBrace", true)?, drape: b(p, "fallOverWallDrape", true)?,
            high: b(p, "fallOverHighWalls", false)?, weak_zero: f(p, "impactWeaknessZeroDuration", 0.2)?,
            weak_ramp: f(p, "impactWeaknessRampDuration", 0.01)?,
        };
        Ok(BcrRdr2 { p, state: 0, hits: [None; 2], speed: 0.0, wall: None, wall_vel: Vec3::ZERO, tte: 0.0, side: -1.0, impacted: false, phase: false,
                     t_hit: 0.0, steps_at: 0, balancing: true, nonfoot: None, started: false, look: None, kind: 3, second: None, second_hit: None,
                     touch_low: false, touch_high: false, low: false, sum: [Vec3::ZERO; 2], rel: Vec3::ZERO, t_stable: 0.0, weak: 1.0005,
                     over_t: 0.0, over_drawn: false, recovered: false, fow_mine: false, rds_on: false, roll_on: false, slump: None, fling: None, near: [false; 2], slide: [false; 2], sink: false })
    }

    /// The probes (state 0, moving faster than 0.5 m/s): spheres of 0.25 m from the pelvis and 0.1 m from the chest
    /// along his horizontal velocity for half a second of it. And the probe down the wall's face, when it is asked.
    pub fn probe(&mut self, c: &Ctx, world: &crate::collision::Collision, obs: &[crate::ragdoll::Obstacle]) {
        if let Some((from, d)) = self.second.take() {
            self.second_hit = Some(self.sweep(world, obs, from, d, 0.001, 1.0).map(|h| h.at));
        }
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
                    // the wall's type (0x2aee3a8): 2 with the pelvis's hit alone, else by where the chest's lies
                    self.kind = self.hits[1].map_or(2, |h1| if (h1.at - h0.at).dot(flat(h0.n)) < -0.1 { 2 } else { 1 });
                }
            }
        }
        let fow_on = c.extras_on.contains(&"fallOverWall");
        self.machine(c, dt, fow_on, out);
        let speed = (c.vel - up * up.dot(c.vel)).length();
        if let Some((at, n)) = self.wall {
            // the balancer's exclusion plane (BCR's wins over teeter's once he has hit it)
            let off = match self.state {
                5 | 10 => 0.4,
                6 => 0.1,
                _ => self.p.zone + if self.impacted { (0.2 * speed).min(0.5) } else { 0.0 },
            };
            if self.impacted || c.out.dbm_plane.is_none() {
                c.out.dbm_plane = Some((at, n, self.side, off));
            }
            if !self.impacted && self.state != 12 {
                // moving along his facing faster than 0.5 m/s toward it: his feet wider (0x2af9888)
                let fwd = flat(-c.m.c);
                if fwd.dot(c.vel) > 0.5 {
                    c.out.dbm_extra_feet = Some(0.2);
                }
                // coming up to it with braceWall: the head to it while more than half a second away, nearer it the
                // spine turned from it (0x2ae9ae4 decides; the brace service it runs isn't here, see §12)
                let bracing = self.p.brace && self.balancing && (self.state == 0 || self.kind != 2);
                if bracing && self.tte >= 0.5 {
                    self.look_at(at, out);
                }
            }
            if self.impacted {
                // reactTime: reachForWound lets both hands go
                if self.t_hit < self.p.react && self.balancing {
                    c.out.rfw_release = [true; 2];
                }
                // the head from the wall: the normal, from the chest (ours: RDR2 waits for catchFall and bodyRollUp to
                // be idle)
                if c.upright {
                    let chest = c.b.at(crate::nmshot::chest_part());
                    self.look_at(chest + n, out);
                }
                // on the impact's tick the knees are held at the bend they have (0x2ab617c, at most 0.35), not bent further
                if self.phase {
                    if let Some(d) = c.dbm {
                        let zp = c.b.part(0).m.d.z;
                        let zf = [2, 3].map(|i| c.b.part(limb(i).p[3]).m.d.z);
                        let k = if d.leg > 0.1 { (0.79235 * (0.99 - (zp - zf[0]).max(zp - zf[1]) / d.leg)).clamp(0.0, 0.35) } else { 0.0 };
                        c.out.dbm_crouch.push(Crouch::Knee(8, -k, f32::INFINITY));
                    }
                }
                let free = c.upright && self.nonfoot.is_none();
                if free {
                    self.fling_or_brace(c, at, n, dt);
                }
                // the weakness ramp (+0x618): nothing for impactWeaknessZeroDuration, then up to 1; the shot's strength
                // reads it (its looseness amount, while the amount is no less than the ramp)
                if self.weak < 1.0 && self.t_hit >= self.p.weak_zero {
                    self.weak = (self.weak + dt / self.p.weak_ramp.max(1e-4)).min(1.0);
                }
            }
        }
        // the slump once the balancer has given up (unless he has been down on something more than 0.3 s)
        if !self.balancing && !matches!(self.state, 0 | 12) && self.nonfoot.unwrap_or(0.0) <= 0.3 && c.ch.fow_state.is_none_or(|s| s == 0 || s == 4) {
            self.state = 4;
        }
        if matches!(self.state, 4 | 7) && c.dbm.is_some() {
            self.slump_core(c, dt);
        }
        // he has come down on both feet off fallOverWall: back to balancing, state 2 (step 15)
        let feet = c.b.doll.collided(&[limb(2).p[3]]) && c.b.doll.collided(&[limb(3).p[3]]);
        if c.ch.fow_state.is_some_and(|s| s > 1) && !self.balancing && !fow_on && feet {
            out.extend(["rollDownStairs start=false", "catchFall start=false", "smartFall start=false", "bodyRollUp start=false"].map(String::from));
            c.out.dbm = Some(true);
            c.out.dbm_taper = Some(true);
            (self.state, self.balancing, self.recovered, self.nonfoot) = (2, true, true, None);
            c.ch.fow_state = None;
        }
        if self.fow_mine && !matches!(self.state, 8 | 9) {
            self.fow_mine = false;
            out.push("fallOverWall start=false".into());
        }
        c.ch.bcr = Some((self.state, self.impacted, if self.impacted { self.p.leg_red } else { 0.0 }, self.weak));
        if self.state != was {
            out.push(format!("#say bcr state {}", self.state));
        }
    }

    /// RDR2's state machine (0x2aeaa50).
    fn machine(&mut self, c: &mut Ctx, dt: f32, fow_on: bool, out: &mut Vec<String>) {
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
        // the probe down the wall's face has answered (0x2af96cc): on the plane a full wall, off it a low wall's top
        if let Some(hit) = self.second_hit.take() {
            self.kind = match (hit, self.wall) {
                (Some(q), Some((at, n))) => if (q - at).dot(n).abs() < 0.02 { 1 } else { 2 },
                _ => 0,
            };
        }
        self.phase = false;
        let up = c.up;
        let over_fow = self.kind == 0 && matches!(self.state, 8 | 9) && fow_on;
        let (mut contact, mut height) = (None, 0.0);
        if (!self.touch_high && !self.low) || over_fow {
            // the contacts (0x2af9f74): a listed part's above the lower foot, on anything but himself; the first five
            // (six with fallOverHighWalls) are low, the rest high
            let low_foot = [2, 3].map(|i| up.dot(c.b.at(limb(i).p[3]))).into_iter().fold(f32::MAX, f32::min);
            let lows = if self.p.high { 6 } else { 5 };
            for (i, &k) in BCR_PARTS.iter().enumerate() {
                if let Some((q, cn)) = c.b.doll.hit_out[k] {
                    let h = up.dot(q) - low_foot;
                    if h > BCR_ABOVE_FOOT {
                        let high = i >= lows;
                        self.sum[high as usize] += cn;
                        if high {
                            self.touch_high = true;
                        } else {
                            self.touch_low = true;
                        }
                        (contact, height) = (Some(q), h);
                    }
                }
            }
            // over a wall fallOverWall holds, a higher contact moves its top
            if let (true, Some(q), Some((at, n))) = (over_fow, contact, self.wall) {
                let d = q - at;
                if up.dot(q) > up.dot(at) || (up.dot(d).abs() < 0.01 && n.dot(d) < 0.0) {
                    self.wall = Some((q, n));
                    out.push(format!("fallOverWall fallOverWallEndA={} fallOverWallEndB={}", v3s(q + n.cross(up)), v3s(q)));
                }
            }
        }
        if !self.impacted {
            if let Some(q) = contact {
                let s = self.sum[0] + self.sum[1];
                let n = unit(s - up * up.dot(s));
                (self.impacted, self.phase, self.state, self.t_hit, self.steps_at) = (true, true, 1, 0.0, steps);
                self.rel = c.vel - self.wall_vel;
                // the impact angle: his velocity against the wall's inward direction, past 0.6 rad a glance
                let v = unit(self.rel - up * up.dot(self.rel));
                if (-n).dot(v).clamp(-1.0, 1.0).acos() > BCR_GLANCE {
                    self.state = 5;
                }
                self.side = if self.rel.dot(n) <= 0.0 { -1.0 } else { 1.0 };
                self.wall = Some((q, n));
                self.weak = 0.01;
                // a high contact is a full wall; a low one asks what is above it (0x2af9c64): from above the wall's face
                // (1.35 m over the lower foot with one down, else at the upper spine), down and into it
                if self.touch_high {
                    self.kind = 1;
                } else {
                    let (feet, zf) = ([2, 3].map(|i| c.b.doll.collided(&[limb(i).p[3]])), [2, 3].map(|i| up.dot(c.b.at(limb(i).p[3]))));
                    let h = if feet[0] || feet[1] { zf[0].min(zf[1]) + 1.35 } else { up.dot(c.b.at((*SPINE())[2] + 1)) };
                    self.second = Some((q + up * (h - up.dot(q)) + n * 0.05, unit(-n - up)));
                }
            }
        }
        let Some((at, n)) = self.wall else { return };
        let w = -n;
        let in_glance = matches!(self.state, 5 | 10);
        if in_glance && self.t_hit > self.p.spin_time {
            self.state = 1;
        }
        let chest = c.b.at(crate::nmshot::chest_part());
        if (self.touch_high || self.kind == 1) && self.state == 1 {
            self.touch_high = true;
            // facing the wall (ours: his forward axis is the trunk's back axis negated)
            if (-c.m.c).dot(w) >= 0.0 {
                let e = (chest - at).dot(n);
                if self.p.rebound > 0.0 && self.side * e < 0.0 && e.abs() < 0.4 {
                    let chest_k = crate::nmshot::chest_part();
                    c.out.impulse.push((chest_k, chest, n * self.p.rebound));
                    for i in 0..2 {
                        let k = limb(i).p[1];
                        c.out.impulse.push((k, c.b.at(k), n * (-0.3 * self.p.rebound)));
                    }
                }
                if self.t_hit > 0.1 && self.t_hit > self.p.to_forward {
                    if self.p.forward_mode == 1 && c.shot_on {
                        self.to_knees(out);
                    } else {
                        self.state = 4;
                    }
                }
            } else {
                self.state = 2;
            }
        }
        // coming off the wall: moving away from it faster than 0.2 m/s with the COM 0.4 m out
        if self.impacted && !matches!(self.state, 5 | 10) {
            let v = unit(c.vel - self.wall_vel);
            if v.dot(n) > 0.2 && self.side * (c.com - at - n * 0.4).dot(n) < 0.0 {
                self.state = 6;
                match self.p.rebound_mode {
                    0 if !c.shot_on => self.state = 4,
                    0 => self.to_knees(out),
                    // reboundMode 1: the balancer's time limit is now (bal+0x6d0 = its time balancing)
                    1 => c.out.dbm_time_up = true,
                    2 => self.state = 4,
                    3 => return self.restart(c, out),
                    _ => {}
                }
            }
        }
        // a low wall: low contacts only, then a full wall's parts past the plane, a low one's at once
        if self.touch_low && !self.touch_high && !self.low && !self.recovered {
            let past = |p: Vec3| self.side * (p - at).dot(n) > 0.0;
            let by_type = self.kind == 0 || self.kind == 2;
            let arms = (0..2).any(|i| past(c.b.at(limb(i).p[3])) || past(c.b.joint_pos(limb(i).e[1])));
            if by_type || past(c.b.at((*SPINE())[1] + 1)) || past(c.b.at((*SPINE())[2] + 1)) || past(chest) || arms {
                self.low = true;
            }
            if self.low {
                if self.state == 5 {
                    if contact.is_some() && height > 0.4 {
                        self.state = 10;
                    }
                } else if matches!(self.state, 1 | 2) {
                    self.over_low_wall(c, at, n, fow_on, out);
                }
            }
        }
        if self.state == 10 && !self.balancing && self.low {
            self.over_low_wall(c, at, n, fow_on, out);
        }
        if self.t_hit < self.p.spin_time && matches!(self.state, 5 | 10) {
            self.glance_spin(c, n);
        }
        if matches!(self.state, 8 | 9) {
            self.over_supervise(c, fow_on, dt, out);
        }
        // still against it (the balancer's fail test writes 3 below its stable speeds, ours the same speeds as its still
        // test); stable2SlumpTime from moving off it
        if self.state == 2 && c.vel.length() < 0.2 && c.angvel.length() < 0.5 {
            self.state = 3;
        }
        if matches!(self.state, 2 | 3) {
            self.t_stable += dt;
        }
        if self.state == 3 && self.t_stable >= self.p.stable {
            self.state = 4;
        }
    }

    /// RDR2's "fall to knees" settings in the character's shot record: at once, no helper force, no lean.
    fn to_knees(&mut self, out: &mut Vec<String>) {
        out.push("shotFallToKnees start=true ftkBalanceTime=0 ftkHelperForce=0 ftkLeanHelp=0".into());
    }

    /// Over a low wall (0x2aec2f0): state 8 or 9 by which way his hips cross it, and with fallOverWallDrape fallOverWall
    /// takes it (0x2afbf58).
    fn over_low_wall(&mut self, c: &mut Ctx, at: Vec3, n: Vec3, fow_on: bool, out: &mut Vec<String>) {
        let along = n.cross(c.up);
        let hips = c.b.joint_pos(limb(3).e[1]) - c.b.joint_pos(limb(2).e[1]);
        self.state = 8 + (along.dot(hips) < 0.0) as u8;
        if self.p.drape && !fow_on {
            out.push(format!("fallOverWall start=true fallOverWallEndA={} fallOverWallEndB={} magOfForce=0.7 forceTimeOut=3 bodyStiffness=9 bodyTwist=0.9 maxTwist={} moveArms=false moveLegs=true legtwist=1 legamp=1 zAxisSpinReduction=0.2",
                             v3s(at + along), v3s(at), std::f32::consts::PI));
            self.fow_mine = true;
        }
    }

    /// While over a low wall (0x2a2d260, called through 0x2aed504).
    fn over_supervise(&mut self, c: &mut Ctx, fow_on: bool, dt: f32, out: &mut Vec<String>) {
        let touching = c.b.doll.collided(&[crate::nmshot::chest_part(), limb(0).p[3], limb(1).p[3]]);
        if !fow_on {
            self.over_t += dt;
            if self.over_t > 0.3 && self.balancing {
                (self.state, self.recovered) = (2, true);
                out.extend(["rollDownStairs start=false", "catchFall start=false"].map(String::from));
            }
            if self.state == 8 {
                if touching && !self.over_drawn {
                    self.over_drawn = true;
                    self.over_t += c.ch.random() * 0.2;
                }
                if !self.rds_on {
                    self.rds_on = true;
                    let legs = 0.2 + c.ch.random() * 0.6;
                    out.push(format!("rollDownStairs start=true stiffness=10 forcemag=1 asymmetricalLegs={legs} m_useArmToSlowDown=-0.9 m_armReachAmount=1.4 spinWhenInAir=true m_legPush=0.2 armReachLength=0.6"));
                }
            }
        } else {
            let s = c.ch.fow_state.unwrap_or(0);
            if self.kind != 1 && (s != 1 && s != 7) {
                if s == 4 {
                    self.state = 4;
                    (self.recovered, self.nonfoot) = (true, None);
                    out.push("fallOverWall start=false".into());
                } else if self.over_drawn {
                    out.push("fallOverWall start=false".into());
                }
            } else {
                self.state = 2;
                self.recovered = true;
                out.push("fallOverWall start=false".into());
                if self.balancing {
                    out.extend(["catchFall start=false", "bodyRollUp start=false"].map(String::from));
                }
            }
        }
        if !self.balancing && !fow_on {
            (self.state, self.recovered) = (2, true);
        }
        if (self.state == 9 || fow_on) && !self.roll_on {
            self.roll_on = true;
            out.push("bodyRollUp start=true".into());
        }
    }

    /// The glance's spin, states 5 and 10 (part of 0x2aeaa50): against his speed along the wall, decaying with the time
    /// since the impact, the way his velocity crosses the normal; about the spine on the spine's parts, or (glanceSpinTurn
    /// > 0, RDR2's default) as a turn asked of the balancer (ours [P]: dbm_turn on the record's heading, not its force).
    fn glance_spin(&mut self, c: &mut Ctx, n: Vec3) {
        let along = (c.vel - n * n.dot(c.vel)).length();
        let g = if along > self.p.spin_min { (self.p.spin_reduce * along).min(1.0) } else { 0.0 };
        let sign = if self.rel.cross(n).dot(c.up) < 0.0 { -1.0 } else { 1.0 };
        let fall = ((self.p.spin_time - self.t_hit * self.p.spin_decay) / self.p.spin_time.max(1e-4)).max(0.0) * g;
        let s = *SPINE();
        let axis = unit(c.b.joint_pos(s[3]) - c.b.joint_pos(s[0]));
        let t = axis * (self.p.spin_mag * fall * sign);
        if self.state == 10 {
            c.out.torque(s[0] + 1, t);
            c.out.torque(s[1] + 1, t * 0.5);
            c.out.torque(0, t * 0.5);
        } else if self.p.spin_turn > 0.0 {
            c.out.dbm_turn = Some(unit(c.m.b * sign + c.m.c * 0.5) * 0.5);
        } else {
            c.out.torque(s[2] + 1, t);
            c.out.torque(s[1] + 1, t * 0.5);
            c.out.torque(s[3] + 1, t * 0.5);
            c.out.torque(s[0] + 1, t * 0.25);
        }
    }

    /// After the impact: fling2 on the first tick if a draw passes and he hit faster than 1.25 m/s, while it runs the
    /// flinging arms; else the arms brace on the wall.
    fn fling_or_brace(&mut self, c: &mut Ctx, at: Vec3, n: Vec3, dt: f32) {
        if self.phase {
            let on = [c.ch.random() < self.p.fling, c.ch.random() < self.p.fling];
            if self.rel.length() > 1.25 && (on[0] || on[1]) {
                let g = 1.5924 * arm_length(c, 0);
                let mut r = || c.ch.random();
                let (a0, a1, l0, l1) = (r(), r(), r(), r());
                let hand = [limb(0).p[3], limb(1).p[3]].map(|k| c.b.at(k));
                self.fling = Some(BcrFling { on, angle: [-1.5 + 2.5 * a0, -1.5 + 2.5 * a1], length: [(0.25 + 0.3 * l0) * g, (0.25 + 0.35 * l1) * g],
                                             relax: [FL_BEFORE + 0.5; 2], stopped: [false; 2], hand, t: 0.0 });
            }
        }
        if self.fling.is_some() {
            return self.fling_tick(c, dt);
        }
        if !self.p.arm_brace {
            return;
        }
        // the chest faces the wall (0x2af2848): the flags, with their hysteresis
        if (-c.m.c).dot(n) >= 0.0 {
            return;
        }
        let vt = c.vel - n * n.dot(c.vel);
        let side_v = vt.dot(c.m.b);
        let shoulder = |c: &Ctx, i: usize| c.b.joint_pos(limb(i).e[1]);
        for i in 0..2 {
            let reach = shoulder(c, i) + c.b.doll.vel(limb(i).p[3]).0 * 0.1 - at;
            self.near[i] = reach.dot(n) < if self.near[i] { 0.5 } else { 0.4 };
        }
        let hys = |on: bool| 0.2 - if on { 0.05 } else { 0.0 };
        let slide = [side_v > hys(self.slide[0]), -side_v > hys(self.slide[1])];
        self.sink = -vt.dot(c.up) > hys(self.sink) && !slide[0] && !slide[1];
        self.slide = slide;
        for i in 0..2 {
            if self.near[i] {
                let idle = !self.sink && !self.slide[i];
                self.brace_arm(c, i, at, n, if idle { 0.15 } else { 1.0 });
                if idle {
                    c.ch.eff[(*crate::nmrs::ELBOW())[i]].desired[0] = 1.4;
                }
            }
        }
    }

    /// One arm on the wall (0x2ae9f5c): the hand direction whose reach meets the plane from the shoulder's distance
    /// (capped at `cap`), turned out by 0.5 rad to his side. Ours [P]: RDR2 turns that into clavicle, shoulder, elbow and
    /// wrist rotations in their joint frames (limited to 0.8 rad); here the limb IK takes the same hand point.
    fn brace_arm(&mut self, c: &mut Ctx, i: usize, at: Vec3, n: Vec3, cap: f32) {
        let l = limb(i);
        let s = c.b.joint_pos(l.e[1]);
        let d = (s - at).dot(n).min(cap);
        if d < 0.0 {
            return;
        }
        let len = arm_length(c, i).max(1e-3);
        let out_side = 1.0 - 2.0 * i as f32;
        let tangent = unit(n.cross(c.up)) * if unit(n.cross(c.up)).dot(c.m.b) * out_side >= 0.0 { 1.0 } else { -1.0 };
        let v0 = Quat::from_axis_angle(n, -0.5 * out_side) * tangent;
        let perp = unit(v0.cross(n));
        let a = ((d - 0.05) / len).clamp(-1.0, 1.0).acos();
        let h = Quat::from_axis_angle(perp, std::f32::consts::FRAC_PI_2 - a) * v0;
        if h.is_finite() && perp != Vec3::ZERO {
            crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[3], s + h * len, 0.0, 0.0, out_side, 1.0, Vec3::ZERO, None);
        }
    }

    /// fling2's arms (0x2a5c0d4, 0x2a2fd20), after 0.1 s until 0.5 s: stiff at first, relaxed 0.5 s on or once the hand
    /// has stopped; the hand to the drawn angle off the spine axis (against the hip line) and length from the shoulder.
    fn fling_tick(&mut self, c: &mut Ctx, dt: f32) {
        let Some(mut f) = self.fling else { return };
        f.t += dt;
        if f.t > FL_BEFORE && f.t < FL_TIME {
            let s = *SPINE();
            let spine = unit(c.b.at(crate::nmshot::chest_part()) - c.b.at(0));
            let hips = unit(c.b.joint_pos(limb(3).e[1]) - c.b.joint_pos(limb(2).e[1]));
            let _ = s;
            for i in 0..2 {
                if !f.on[i] {
                    continue;
                }
                let l = limb(i);
                let (stiff, damp, muscle, og) = if f.relax[i] < f.t { (7.0, 0.7, None, 0.0) } else { (12.0, 0.7, (i == 0).then_some(0.95), 0.5) };
                c.ch.set_stiffness(stiff, damp, if i == 0 { "ul" } else { "ur" }, muscle);
                let e = &mut c.ch.eff[l.e[2]];
                e.set_stiffness(stiff * 0.75, damp * 0.75, None);
                for j in [l.e[0], l.e[1], l.e[2]] {
                    c.ch.eff[j].oppose_gravity = og;
                }
                let hand = c.b.at(l.p[3]);
                if (hand - f.hand[i]).length() / dt.max(1e-4) < 1.0 && f.t > FL_BEFORE + 0.1 {
                    f.stopped[i] = true;
                    f.relax[i] = f.t;
                }
                f.hand[i] = hand;
                let side = hips * (1.0 - 2.0 * i as f32);
                let shoulder = c.b.joint_pos(l.e[1]);
                let to = shoulder + (spine * f.angle[i].cos() - side * f.angle[i].sin()) * f.length[i];
                let upper = unit(c.b.joint_pos(l.e[2]) - shoulder);
                let x = upper.dot(side);
                let twist = (2.5 * (upper.dot(spine) - x.max(0.0)) + 0.5).clamp(-1.7, 0.2);
                crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[3], to, twist, 0.0, 1.0 - 2.0 * i as f32, 1.0, Vec3::ZERO, None);
                c.ch.eff[l.e[3]].desired = [0.0, 0.0, 0.24];
            }
        }
        self.fling = (f.t < FL_TIME && (f.on[0] || f.on[1])).then_some(f);
    }

    /// reboundMode 3: the reset (0x2aedfb4), back to looking for a wall; catchFall and bodyRollUp stopped if he is still
    /// balancing; the knee bend and leg reduction let go.
    fn restart(&mut self, c: &mut Ctx, out: &mut Vec<String>) {
        let p = self.p;
        let balancing = self.balancing;
        self.look = None;
        (self.state, self.impacted, self.phase, self.t_hit, self.wall, self.tte, self.slump) = (0, false, false, 0.0, None, 0.0, None);
        (self.kind, self.side, self.touch_low, self.touch_high, self.low, self.sum, self.weak) = (3, -1.0, false, false, false, [Vec3::ZERO; 2], 1.0005);
        (self.t_stable, self.over_t, self.over_drawn, self.fling, self.near, self.slide, self.sink) = (0.0, 0.0, false, None, [false; 2], [false; 2], false);
        self.p = p;
        c.out.dbm_crouch.push(Crouch::ReleaseKnee(1.0));
        if balancing {
            out.extend(["catchFall start=false".to_string(), "bodyRollUp start=false".to_string()]);
        }
    }

    /// The slump (0x2a55ddc, from RDR2's code): the leg under the COM collapses first (the other, one time in five, by a
    /// random part of it), then every stiffness falls at slumpLegStiffRate (while his kinetic energy falls below 0.4;
    /// slumpMode 1 always, 2 while it falls at all); no new steps; after sitstarttime the legs blend to the sit pose at
    /// sitrate. The friction ramps aren't here: Delirium's friction is per material.
    fn slump_core(&mut self, c: &mut Ctx, dt: f32) {
        c.out.dbm_force_balance = true;
        let (hip, knee, ankle) = (*HIP(), *KNEE(), *ANKLE());
        let stiff = |c: &Ctx, j: usize| c.ch.eff[j].strength.max(0.0).sqrt();
        let ke = 0.5 * c.vel.length_squared();
        let mut r = self.slump.unwrap_or_else(|| {
            let mut legs = [stiff(c, hip[0]), stiff(c, hip[1]), stiff(c, knee[0]), stiff(c, knee[1])];
            let (fl, fr) = (c.b.at(limb(2).p[3]), c.b.at(limb(3).p[3]));
            let d = fr - fl;
            // the right leg (1) if the COM is nearer the right foot
            let under = ((fr - c.com).dot(d) <= (c.com - fl).dot(d)) as usize;
            legs[under] -= self.p.slump_red;
            legs[2 + under] -= self.p.slump_red;
            if c.ch.random() > 0.8 {
                let other = 1 - under;
                legs[other] -= c.ch.random() * self.p.slump_red;
                legs[2 + other] -= c.ch.random() * self.p.slump_red;
            }
            SlumpRec { t: 0.0, ke, legs }
        });
        r.t += dt;
        if (ke <= r.ke && ke < 0.4) || self.p.slump_mode == 1 || (self.p.slump_mode == 2 && ke <= r.ke) {
            r.legs = r.legs.map(|s| s - dt * self.p.slump_rate);
        }
        r.ke = ke;
        for i in 0..2 {
            let k = stiff(c, knee[i]).min(r.legs[2 + i]);
            let h = stiff(c, hip[i]).min(r.legs[i]);
            c.ch.eff[ankle[i]].set_stiffness(1.0, 1.0, None);
            c.ch.eff[knee[i]].set_stiffness(k.max(1.0), 0.9, None);
            c.ch.eff[hip[i]].set_stiffness(k.max(h).max(0.0), 1.0, None);
            if self.p.sit_rate > 0.0 && r.t > self.p.sit_start {
                let w = ((r.t - self.p.sit_start) * self.p.sit_rate).clamp(0.0, 1.0);
                let e = &mut c.ch.eff[knee[i]].desired[0];
                *e += (-1.8 - *e) * w;
                // ours: our hip sign is RDR2's negated
                let e = &mut c.ch.eff[hip[i]].desired[0];
                *e += (-1.5 - *e) * w;
            }
        }
        self.slump = Some(r);
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

fn arm_length(c: &Ctx, i: usize) -> f32 {
    let l = limb(i);
    let (s, e, w) = (c.b.joint_pos(l.e[1]), c.b.joint_pos(l.e[2]), c.b.joint_pos(l.e[3]));
    (e - s).length() + (w - e).length() + (c.b.at(l.p[3]) - w).length()
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
        c.ch.fow_state = Some(self.state as i32);
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
