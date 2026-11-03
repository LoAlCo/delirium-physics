//! RDR2 core, balancer part (experimental, opt-in through `Agent::rdr2_core`): what Red Dead Redemption 2's dynamic
//! balancer adds to GTA IV's, written from a study of how it behaves (descriptions and its parameter tables; no game
//! code). GTA IV's balancer (nmdbm.rs) still does the work; this changes, when on:
//! - failing: more fail tests (feet giving out at 0.5 m, the ankles' support at 2.4), and step, time and stillness
//!   limits that no longer drop him at once but drain his legs' strength so he gives way, one leg first;
//! - pelvis height from leg length and how far apart the feet are;
//! - a step up onto something higher lifts the swing foot more for that step;
//! - ankle balance: the planted ankles' balancing pitch fades out once the foot has tilted off flat;
//! - the swing ankle doesn't oppose gravity, and its pitch clamp;
//! - slopes: on ground steeper than slopeMinDot he re-steps less (legsApartRestep at least 0.7), doesn't taper his
//!   knees, keeps his planted feet slopeMoveFoot down the slope (the direction is our reading) and lowers his pelvis
//!   by the feet's height difference beyond a step up's only if slopeDrop1/2 are set (RDR2 has 0.7 / 1.8 for two
//!   unnamed values there, but how it uses them isn't known and our reading made him fall, so off by default);
//! - step rules as parameters: legsApartRestep, legsTogetherRestep, stepDecisionThreshold, stepWait,
//!   changeStepTime, alwaysStepWithFarthest (RDR2's defaults leave all but legsApartRestep off).
//! - running: at runVel or faster (RDR2's default 100 m/s leaves it off; the game sets it for running peds) the legs
//!   use legStiffnessRun, the swing foot stepHeightRun, the knees taperKneeStrengthRun, and the swing leg goes half as
//!   stiff;
//! - the injured-leg limp (from shotInjuredLeg): the hurt leg, while planted, bends by
//!   (1 - 2.5 (clamp(feet apart, 0.6, 1) - 0.6)) x the injury, more on a short stance (the injury taken as
//!   limpScale x legInjury, our 0.25).
//! Not here yet: stepIfInSupport (its speed threshold isn't known), more foot friction on slopes, running, crouch
//! start, the injured-leg limp, obstacle avoidance, lean channels, the turn step, hip pitch on failing, the sideways
//! cheat force.

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
    pub slope_min_dot: f32,
    pub no_slope_time: f32,
    pub slope_move_foot: f32,
    /// pelvis drop per metre of the feet's height difference on a slope, both feet down / one. RDR2 has 0.7 / 1.8 there,
    /// but neither their names nor how it uses them are known, and our reading made him fall on 20-26 degree ramps, so
    /// it is off unless set (slopeDrop2 / slopeDrop1)
    pub slope_drop: [f32; 2],
    pub legs_apart_restep: f32,
    pub legs_together_restep: f32,
    pub step_decision_threshold: f32,
    pub step_wait: f32,
    pub change_step_time: f32,
    pub always_step_with_farthest: bool,
    pub run_vel: f32,
    pub leg_stiffness_run: f32,
    pub step_height_run: f32,
    pub taper_knee_strength_run: bool,
    /// share of shotInjuredLeg's legInjury the limp takes as its amount. Ours (0.25): RDR2's own source of the amount
    /// isn't known, and the whole legInjury (0.3) took a quarter-metre off the leg and dropped him
    pub limp_scale: f32,
}

impl Default for Params {
    fn default() -> Params {
        Params { max_steps: 100, max_balance_time: 50.0, max_still_time: 50.0, balance_indefinitely: false, giveup_height: 0.5,
                 fall_type: 0, fall_mult: 1.0, step_height_inc4_step: 0.1, ankle_balance: true, start_flat: 0.1, end_flat: 0.3,
                 slope_min_dot: 0.05, no_slope_time: 0.0, slope_move_foot: 0.07, slope_drop: [0.0, 0.0], legs_apart_restep: 0.2,
                 legs_together_restep: 1.0, step_decision_threshold: 0.0, step_wait: -1.0, change_step_time: -1.0,
                 always_step_with_farthest: false, run_vel: 100.0, leg_stiffness_run: 13.0, step_height_run: 0.6,
                 taper_knee_strength_run: false, limp_scale: 0.25 }
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
            "slopeMinDot" => self.slope_min_dot = v,
            "noSlopeTime" => self.no_slope_time = v,
            "slopeMoveFoot" => self.slope_move_foot = v,
            "slopeDrop2" => self.slope_drop[0] = v,
            "slopeDrop1" => self.slope_drop[1] = v,
            "legsApartRestep" => self.legs_apart_restep = v,
            "legsTogetherRestep" => self.legs_together_restep = v,
            "stepDecisionThreshold" => self.step_decision_threshold = v,
            "stepWait" => self.step_wait = v,
            "changeStepTime" => self.change_step_time = v,
            "alwaysStepWithFarthest" => self.always_step_with_farthest = v != 0.0,
            "runVel" => self.run_vel = v,
            "legStiffnessRun" => self.leg_stiffness_run = v,
            "stepHeightRun" => self.step_height_run = v,
            "taperKneeStrengthRun" => self.taper_knee_strength_run = v != 0.0,
            "limpScale" => self.limp_scale = v,
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
/// On a slope he re-steps for feet at least this far apart.
const SLOPE_RESTEP: f32 = 0.7;
/// stepWait applies below this speed (m/s), and is 0.1 s shorter above it.
const STEP_WAIT_SPEED: f32 = 0.7;
/// The pelvis never sinks below this over the feet on a slope.
const SLOPE_FLOOR: f32 = 0.2;
/// Running: the swing leg's stiffness share.
pub const RUN_SWING: f32 = 0.5;

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
    pub on_slope: bool,
    /// s since the last step ended (large before the first), and the current step's age
    pub since_step: f32,
    pub step_age: f32,
    pub speed: f32,
    pub running: bool,
    /// the injured leg (0 left, 1 right) and how badly (0..1), from shotInjuredLeg
    pub limp: (usize, f32),
}

impl Rdr2 {
    pub fn new(p: Params, random: f32) -> Rdr2 {
        Rdr2 { p, time: 0.0, still: 0.0, steps: 0, failing: None, asym: (random * 2.0 - 1.0) * 0.15, on_slope: false,
               since_step: 100000.0, step_age: 0.0, speed: 0.0, running: false, limp: (0, 0.0) }
    }

    /// Timers and the timed fail; `stepped` when a step was committed this tick.
    pub fn update(&mut self, dt: f32, angvel2: f32, stepped: bool, stepping: bool) {
        self.time += dt;
        (self.since_step, self.step_age) = if stepping { (0.0, self.step_age + dt) } else { (self.since_step + dt, 0.0) };
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

    /// The slope flag from the ground's normal under the deciding foot (the planted one, or the higher one).
    pub fn slope(&mut self, normal_up: f32) {
        self.on_slope = normal_up < 1.0 - self.p.slope_min_dot && self.time >= self.p.no_slope_time;
    }

    /// Feet at least this far apart (beyond their minimum spread) make him step though the COM is supported.
    pub fn legs_apart(&self) -> f32 {
        if self.on_slope { self.p.legs_apart_restep.max(SLOPE_RESTEP) } else { self.p.legs_apart_restep }
    }

    /// May a new step start now? stepWait holds the next one back when he's slow.
    pub fn may_step(&self) -> bool {
        let wait = if self.speed < STEP_WAIT_SPEED { self.p.step_wait } else { self.p.step_wait - 0.1 };
        wait <= 0.0 || self.since_step > wait
    }

    /// changeStepTime: a step this old may be re-decided (the other foot may take over).
    pub fn redecide(&self) -> bool {
        self.p.change_step_time > 0.0 && self.step_age > self.p.change_step_time
    }

    /// How much shorter the injured planted leg gets (a share of the leg), from the feet's spread.
    pub fn limp_bend(&self, i: usize, feet_apart: f32) -> f32 {
        if self.limp.0 != i || self.limp.1 <= 0.0 {
            return 0.0;
        }
        (1.0 - 2.5 * (feet_apart.clamp(0.6, 1.0) - 0.6)) * self.limp.1 * self.p.limp_scale
    }

    /// Pelvis height on a slope: lowered by the feet's height difference beyond a step up's.
    pub fn slope_height(&self, h: f32, feet_dz: f32, both: bool) -> f32 {
        if !self.on_slope {
            return h;
        }
        // only a difference beyond a step up's (our reading: RDR2's own use of these two isn't known)
        (h - (feet_dz.abs() - STEP_UP).max(0.0) * self.p.slope_drop[if both { 0 } else { 1 }]).max(SLOPE_FLOOR.min(h))
    }
}
