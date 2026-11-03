//! RDR2 core, balancer part (experimental, opt-in through `Agent::rdr2_core`): what Red Dead Redemption 2's dynamic
//! balancer adds to GTA IV's, written from a study of how it behaves (descriptions and its parameter tables; no game
//! code). GTA IV's balancer (nmdbm.rs) still does the work; this changes, when on:
//! - failing: more fail tests (feet giving out at 0.5 m, the ankles' support at 2.4), and step, time and stillness
//!   limits that no longer drop him at once but drain his legs' strength so he gives way, one leg first;
//! - pelvis height from leg length and how far apart the feet are;
//! - a step up onto something higher lifts the swing foot more for that step;
//! - ankle balance: the planted ankles' balancing pitch fades out once the foot has tilted off flat;
//! - the swing ankle doesn't oppose gravity, and its pitch clamp.
//! Not here yet: slopes, running, crouch start, the injured-leg limp, obstacle avoidance, lean channels, the turn
//! step, hip pitch on failing, the sideways cheat force.

/// configureBalance's RDR2 parameters this port uses (RDR2's names and defaults).
#[derive(Clone, Copy, Debug)]
pub struct Params {
    pub max_steps: u32,
    pub max_balance_time: f32,
    /// s standing still before the timed fail (its name isn't known)
    pub max_still_time: f32,
    pub balance_indefinitely: bool,
    pub giveup_height: f32,
    pub fall_type: i32,
    pub fall_mult: f32,
    pub step_height_inc4_step: f32,
    pub ankle_balance: bool,
    pub start_flat: f32,
    pub end_flat: f32,
}

impl Default for Params {
    fn default() -> Params {
        Params { max_steps: 100, max_balance_time: 50.0, max_still_time: 50.0, balance_indefinitely: false, giveup_height: 0.5,
                 fall_type: 0, fall_mult: 1.0, step_height_inc4_step: 0.1, ankle_balance: true, start_flat: 0.1, end_flat: 0.3 }
    }
}

impl Params {
    /// Sets one parameter by its RDR2 name; false when the name isn't one of these.
    pub fn set(&mut self, name: &str, v: f32) -> bool {
        match name {
            "maxSteps" => self.max_steps = v as u32,
            "maxBalanceTime" => self.max_balance_time = v,
            "maxStillTime" => self.max_still_time = v,
            "balanceIndefinitely" => self.balance_indefinitely = v != 0.0,
            "giveupHeight" => self.giveup_height = v,
            "fallType" => self.fall_type = v as i32,
            "fallMult" => self.fall_mult = v,
            "stepHeightInc4Step" => self.step_height_inc4_step = v,
            "ankleBalance" => self.ankle_balance = v != 0.0,
            "startFlat" => self.start_flat = v,
            "endFlat" => self.end_flat = v,
            _ => return false,
        }
        true
    }
}

/// The ankles' support test: fail when 2.4 x the COM's height over them is less than its distance from them.
pub const ANKLE_ABORT: f32 = 2.4;
/// Floor of the draining leg and hip strength (x the character's stiffness scale, 1 here).
const LEG_FLOOR: f32 = 2.0;
const HIP_FLOOR: f32 = 4.0;
/// Below this squared angular speed he counts as still.
const STILL: f32 = 0.25;
/// Step up: the swing foot's ground is this much over the planted one's.
const STEP_UP: f32 = 0.1;
const SWING_ANKLE_CLAMP: f32 = 9.7;

#[derive(Clone, Copy, Debug)]
pub struct Rdr2 {
    pub p: Params,
    pub time: f32,
    pub still: f32,
    pub steps: u32,
    /// when the timed fail started (balance time)
    pub failing: Option<f32>,
    /// in [-0.15, 0.15]: which hip folds first
    pub asym: f32,
}

impl Rdr2 {
    pub fn new(p: Params, random: f32) -> Rdr2 {
        Rdr2 { p, time: 0.0, still: 0.0, steps: 0, failing: None, asym: (random * 2.0 - 1.0) * 0.15 }
    }

    /// Timers and the timed fail; `stepped` when a step was committed this tick.
    pub fn update(&mut self, dt: f32, angvel2: f32, stepped: bool) {
        self.time += dt;
        self.still = if angvel2 < STILL { self.still + dt } else { 0.0 };
        self.steps += stepped as u32;
        if self.failing.is_none() && !self.p.balance_indefinitely
            && (self.time >= self.p.max_balance_time || self.still >= self.p.max_still_time || self.steps >= self.p.max_steps)
        {
            self.failing = Some(self.time);
        }
    }

    /// Leg (knees, ankles) and per-side hip stiffness while failing, from the balancer's leg stiffness.
    pub fn drain(&self, k: f32) -> Option<(f32, [f32; 2])> {
        let t = self.time - self.failing?;
        // fallType 2: keep balancing at full strength (1, keep stepping, and 3, slump, drain as 0 for now)
        if self.p.fall_type == 2 {
            return None;
        }
        let m = self.p.fall_mult;
        let leg = (k / (t * m * 0.6 + 1.3)).max(LEG_FLOOR);
        let hip = |a: f32| (k / ((0.4 + a) * m * t + 1.1)).max(HIP_FLOOR);
        Some((leg, [hip(-self.asym), hip(self.asym)]))
    }

    pub fn giveup_height(&self) -> f32 {
        self.p.giveup_height
    }

    /// Pelvis height over the feet: leg reach `l` (straightness x length) shortened by the feet's spread `d`.
    pub fn pelvis_height(l: f32, d: f32) -> f32 {
        (l * l - (0.5625 * l * l).min(0.25 * d * d)).sqrt()
    }

    /// Extra swing height when the stepping foot's ground is higher than the planted foot's.
    pub fn step_up(&self, swing_ground: f32, planted_ground: f32) -> f32 {
        if swing_ground - planted_ground > STEP_UP { self.p.step_height_inc4_step } else { 0.0 }
    }

    /// Share of the ankle's balancing pitch to keep, from how far the foot is off flat (rad).
    pub fn ankle_weight(&self, off_flat: f32) -> f32 {
        if !self.p.ankle_balance {
            return 1.0;
        }
        let (a, b) = (self.p.start_flat, self.p.end_flat);
        (1.0 - (off_flat.abs() - a) / (b - a)).clamp(0.0, 1.0)
    }

    pub fn swing_ankle_clamp() -> f32 {
        SWING_ANKLE_CLAMP
    }
}
