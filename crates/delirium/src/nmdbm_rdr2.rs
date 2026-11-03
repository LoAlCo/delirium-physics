//! RDR2 core, balancer part (experimental, opt-in through `Agent::rdr2_core`): what Red Dead Redemption 2's dynamic
//! balancer adds to GTA IV's, written from a study of how it behaves (descriptions and its parameter tables; no game
//! code). GTA IV's balancer (nmdbm.rs) still does the work; this changes, when on:
//! - failing: more fail tests (feet giving out at 0.5 m, the ankles' support at 2.4), and step, time and stillness
//!   limits that no longer drop him at once but drain his legs' strength so he gives way, one leg first;
//! - pelvis height from leg length and how far apart the feet are, over the support centre;
//! - a step up onto something higher lifts the swing foot more for that step;
//! - ankle balance: the planted ankles' balancing pitch fades out once the foot has tilted off flat;
//! - the swing ankle doesn't oppose gravity, and its pitch clamp;
//! - slopes: on ground steeper than slopeMinDot he re-steps less (legsApartRestep at least 0.7) and doesn't taper
//!   his knees. Between steps the higher foot's target moves up the slope by slopeMoveFoot times how much the slope
//!   tilts along the feet. His pelvis sits between the feet at 0.5 - tan^3 of their rise over their spread from the
//!   higher one, and sinks by 0.7 (between steps) or 1.8 (stepping downhill) of how far it is above what a leg
//!   reaches to both feet, never below 0.2 m (slopeDrop2 / slopeDrop1);
//! - step rules as parameters: legsApartRestep, legsTogetherRestep, stepDecisionThreshold, stepWait (only below
//!   0.7 m/s),
//!   changeStepTime, alwaysStepWithFarthest (RDR2's defaults leave all but legsApartRestep off).
//! - running: at runVel or faster (RDR2's default 100 m/s leaves it off; the game sets it for running peds) the legs
//!   use legStiffnessRun, the swing foot stepHeightRun, the knees taperKneeStrengthRun, and the swing leg goes half as
//!   stiff;
//! - the injured-leg limp (from shotInjuredLeg): the hurt leg, while planted, bends by
//!   (1 - 2.5 (clamp(feet apart, 0.6, 1) - 0.6)) x the injury, more on a short stance (the injury taken as
//!   limpScale x legInjury, our 0.25).
//! - the owned crouch request: a knee bend (leg straightness) and hip pitch that behaviours ask the balancer for
//!   instead of bending the legs themselves (duck 9, shot from behind 6, in the guts 7, the injured leg's sag 6, the
//!   balancer's own fail 14: RDR2's owner ids, which are also priorities, the highest wins). Each owner asks every
//!   step it wants it (ours: RDR2 keeps it until its owner releases it, but stopAllBehaviours would leave it held
//!   here). On failing, rampHipPitchOnFail pitches the hips 0.4 forward at 0.6 rad/s (RDR2's -0.4, its sign read as
//!   its knee bend's: negative bends more);
//! - maxFootSpeed as the swing leg's reach (GTA IV's fixed 5); fallType 1: no new step once failing;
//! - stepIfInSupport: moving, a foot that lost the ground may step though the COM is over his feet (the farther-foot
//!   test stays masked there, except when turning); off, nothing steps while the COM is over his feet;
//! - ankle balance: each planted ankle aims its foot at the contact normal tilted (at most 0.2 rad) toward the COM
//!   0.4 s ahead (speed capped at 0.3), by ankleMult x its share (1/distance^5 of the feet from the COM 0.2 s ahead)
//!   x the fade, plus ankleOffset, in place of the pelvis control's ankle term; the pelvis control loses its roll
//!   integrator and lowers its pitch target by clamp(0.26 v, 0, 0.2) on one foot and clamp(0.4 v, -0.03, 0.2) + 0.03
//!   on both once he has stepped (v his backward speed) (0.2 v / 0.7 v capped at 0.5 without ankle balance).
//!   Its twist the same way: the twist laying the foot on the ground, plus ankleTwistMult x share x a fade
//!   (startFlatTwist / endFlatTwist) of the difference to the tilted normal's, mirrored on the right foot. Only
//!   below 1 m/s; faster, the planted ankles lie flat;
//! - the turn step (when a behaviour asks the balancer to face a way): hip twist up to 1.9 rad, split with the ankle
//!   (0.3 of it on a planted leg, 0.8 swinging), each leg's twist within minLegTwist/maxLegTwist (drawn +-0.3/0.2
//!   per step), and a 0.17 +- 0.07 lift on every step. Every third step landing, and when he turns off a heading he
//!   was facing (within acos 0.9), he pauses -0.2 to 0.8 s first; a step landing ends it for that tick;
//! - minKneeAngle (a swing knee limit; never acts at RDR2's -0.5) and fallReduceGravityComp (oppose gravity drains
//!   with the hips: hips r, knees and ankles 2r);
//! - off unless set (as RDR2's defaults leave them): the predictionTime, stepClampScale and liftFeet variances drawn
//!   for each step, leanAgainstVelocity, standUp, extraFeetApart, the feetApart randomisation (feetApartMin/Max
//!   above feetApartVel) with stopWaddling, the sideways cheat force (sidewaysCheatForce over sidewaysThresh) and
//!   the 0.06 m alternation bias while moving sideways.
//! - the lean channels: the COM lean tilts his up as before, a hip lean (hipsLean*) tilts that up again for the
//!   pelvis frame the legs reach for (instead of our paired push on the hips and chest), applied with a foot down;
//!   the force lean pushes as before. A channel's latest lean wins (our messages carry no priority);
//! - crouch start (crouch, off by default): starting crouched, deeper than crouchBendThresh (up to
//!   crouchBendMax), he keeps the knees bent (at most 0.5) and the hips as they were, letting them go at
//!   crouchStandRate and crouchHipRate, for crouchTime. The crouch depth is (0.99 - COM height over the lower foot /
//!   (leg length - 0.0375)) x 0.79235, the leg length taken as his standing pelvis height (ours).
//! Not here: extra foot friction on slopes (where RDR2 reads it isn't known, and Delirium's friction is fixed per
//! part), obstacle and other-leg avoidance, lean priorities, hipError (nothing at its default 0), the turn step's
//! flat-foot test, and fallType 3 (slump: it only sets balancerCollisionsReaction's state, so here it is fallType 0).

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
    /// on a slope, the share of the pelvis's height over what a leg reaches to both feet it gives up, between steps /
    /// stepping downhill (slopeDrop2 / slopeDrop1: RDR2's two unnamed values)
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
    /// behaviours' joint writes resolved as RDR2's prioritised limb requests (limbreq.rs); off: the last write wins
    pub limb_requests: bool,
    /// limb requests resolved highest slot first (the other reading of RDR2's queue order)
    pub limb_order_reversed: bool,
    /// the swing leg solver's reach: the target at most 2 maxFootSpeed / k from the foot, k the hip stiffness (GTA IV's
    /// fixed 5, a parameter in RDR2)
    pub max_foot_speed: f32,
    pub ramp_hip_pitch_on_fail: bool,
    pub prediction_time: f32,
    pub prediction_time_variance: f32,
    pub step_clamp_scale_variance: f32,
    pub lift_feet: f32,
    pub lift_feet_variance: f32,
    pub lean_against_velocity: f32,
    pub stand_up: bool,
    pub extra_feet_apart: f32,
    pub feet_apart_min: f32,
    pub feet_apart_max: f32,
    pub feet_apart_vel: f32,
    /// 1: no left step width when fast, 2: no right
    pub stop_waddling: i32,
    pub sideways_cheat_force: f32,
    pub sideways_thresh: f32,
    /// moving, a foot that lost the ground may step again though the COM is over his feet (the farther-foot test stays
    /// masked there, as in GTA IV); off: no step at all while the COM is over his feet
    pub step_if_in_support: bool,
    pub ankle_offset: f32,
    pub ankle_mult: f32,
    pub min_knee_angle: f32,
    pub fall_reduce_gravity_comp: bool,
    pub min_leg_twist: f32,
    pub max_leg_twist: f32,
    /// ankle balance: also keep the pelvis control's ankle term (ours, for comparison: RDR2's ankle formula replaces it)
    pub ankle_keep_pelvis: bool,
    pub crouch: bool,
    pub crouch_bend_thresh: f32,
    pub crouch_bend_max: f32,
    pub crouch_time: f32,
    pub crouch_stand_rate: f32,
    pub crouch_hip_rate: f32,
    /// ankle balance's twist: the window off flat and the gain, as the pitch's
    pub start_flat_twist: f32,
    pub end_flat_twist: f32,
    pub ankle_twist_mult: f32,
}

impl Default for Params {
    fn default() -> Params {
        Params { max_steps: 100, max_balance_time: 50.0, max_still_time: 50.0, balance_indefinitely: false, giveup_height: 0.5,
                 fall_type: 0, fall_mult: 1.0, step_height_inc4_step: 0.1, ankle_balance: true, start_flat: 0.1, end_flat: 0.3,
                 slope_min_dot: 0.05, no_slope_time: 0.0, slope_move_foot: 0.07, slope_drop: [0.7, 1.8], legs_apart_restep: 0.2,
                 legs_together_restep: 1.0, step_decision_threshold: 0.0, step_wait: -1.0, change_step_time: -1.0,
                 always_step_with_farthest: false, run_vel: 100.0, leg_stiffness_run: 13.0, step_height_run: 0.6,
                 taper_knee_strength_run: false, limp_scale: 0.25, limb_requests: true, limb_order_reversed: false,
                 max_foot_speed: 5.0, ramp_hip_pitch_on_fail: true, prediction_time: 0.2, prediction_time_variance: 0.0,
                 step_clamp_scale_variance: 0.0, lift_feet: 0.07, lift_feet_variance: 0.0, lean_against_velocity: 0.0,
                 stand_up: false, extra_feet_apart: 0.0, feet_apart_min: 1.0, feet_apart_max: 1.0, feet_apart_vel: 100.0,
                 stop_waddling: 0, sideways_cheat_force: 0.0, sideways_thresh: 0.8,
                 step_if_in_support: true, ankle_offset: -0.12, ankle_mult: 25.0, min_knee_angle: -0.5,
                 fall_reduce_gravity_comp: false, min_leg_twist: -1.9, max_leg_twist: 0.2, ankle_keep_pelvis: false,
                 crouch: false, crouch_bend_thresh: 0.3, crouch_bend_max: 0.55, crouch_time: 0.5, crouch_stand_rate: 0.4,
                 crouch_hip_rate: 0.25, start_flat_twist: 0.05, end_flat_twist: 0.15, ankle_twist_mult: 20.0 }
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
            "limbRequests" => self.limb_requests = v != 0.0,
            "limbOrderReversed" => self.limb_order_reversed = v != 0.0,
            "maxFootSpeed" => self.max_foot_speed = v,
            "rampHipPitchOnFail" => self.ramp_hip_pitch_on_fail = v != 0.0,
            "predictionTime" => self.prediction_time = v,
            "predictionTimeVariance" => self.prediction_time_variance = v,
            "stepClampScaleVariance" => self.step_clamp_scale_variance = v,
            "liftFeet" => self.lift_feet = v,
            "liftFeetVariance" => self.lift_feet_variance = v,
            "leanAgainstVelocity" => self.lean_against_velocity = v,
            "standUp" => self.stand_up = v != 0.0,
            "extraFeetApart" => self.extra_feet_apart = v,
            "feetApartMin" => self.feet_apart_min = v,
            "feetApartMax" => self.feet_apart_max = v,
            "feetApartVel" => self.feet_apart_vel = v,
            "stopWaddling" => self.stop_waddling = v as i32,
            "sidewaysCheatForce" => self.sideways_cheat_force = v,
            "sidewaysThresh" => self.sideways_thresh = v,
            "stepIfInSupport" => self.step_if_in_support = v != 0.0,
            "ankleOffset" => self.ankle_offset = v,
            "ankleMult" => self.ankle_mult = v,
            "minKneeAngle" => self.min_knee_angle = v,
            "fallReduceGravityComp" => self.fall_reduce_gravity_comp = v != 0.0,
            "minLegTwist" => self.min_leg_twist = v,
            "maxLegTwist" => self.max_leg_twist = v,
            "ankleKeepPelvis" => self.ankle_keep_pelvis = v != 0.0,
            "crouch" => self.crouch = v != 0.0,
            "crouchBendThresh" => self.crouch_bend_thresh = v,
            "crouchBendMax" => self.crouch_bend_max = v,
            "crouchTime" => self.crouch_time = v,
            "crouchStandRate" => self.crouch_stand_rate = v,
            "crouchHipRate" => self.crouch_hip_rate = v,
            "startFlatTwist" => self.start_flat_twist = v,
            "endFlatTwist" => self.end_flat_twist = v,
            "ankleTwistMult" => self.ankle_twist_mult = v,
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
/// stepWait applies below this speed (m/s); above it the wait is FAST_STEP_WAIT, which never holds a step back.
const STEP_WAIT_SPEED: f32 = 0.7;
const FAST_STEP_WAIT: f32 = -0.1;
/// The turn step: facing the heading this closely (cosine) he doesn't turn; the pause's range (s).
const TURN_FACING: f32 = 0.9;
const TURN_PAUSE: (f32, f32) = (-0.2, 0.8);
/// On a slope: how far the pelvis point leans from the feet's midpoint toward RDR2's (+0x27c, unnamed; 1 = all the way).
const SLOPE_PELVIS_BLEND: f32 = 1.0;
/// The pelvis never sinks below this over the feet on a slope.
const SLOPE_FLOOR: f32 = 0.2;
/// Running: the swing leg's stiffness share.
pub const RUN_SWING: f32 = 0.5;
/// The balancer's own owner id for its crouch request when failing, and the hip pitch it asks for (rad, ours: RDR2's
/// -0.4 with its knee bend's sign, negative bending more) and how fast.
const FAIL_OWNER: u32 = 14;
const FAIL_HIP_PITCH: f32 = 0.4;
const FAIL_HIP_RATE: f32 = 0.6;
/// Moving sideways: the other foot's lead in the step decision after each step (m), and the speed (squared) it
/// takes to count.
const ALTERNATE: f32 = 0.06;
/// The turn step: hip twist limit (rad), the ankle's share of a leg's twist (planted, swinging), the lift and its
/// variance, the leg-twist limits' variances.
pub const TURN_TWIST: f32 = 1.9;
pub const TURN_ANKLE: [f32; 2] = [0.3, 0.8];
const TURN_LIFT: (f32, f32) = (0.17, 0.07);
const LEG_TWIST_VAR: (f32, f32) = (0.3, 0.2);
/// Ankle balance: the COM look-ahead (s), its speed cap, the largest tilt of the aimed normal, the feet's share power.
const ANKLE_AHEAD: f32 = 0.4;
const ANKLE_SPEED: f32 = 0.3;
pub const ANKLE_TILT: f32 = 0.2;
const SIDEWAYS_SPEED2: f32 = 0.25;

/// RDR2's crouch request the balancer owns: a knee bend (added to leg straightness) and a hip pitch (rad, flexing
/// the hips), each followed at `rate` (rad or straightness per second; infinite when the asker ramps it itself).
#[derive(Clone, Copy, Debug, Default)]
pub struct Owned {
    pub owner: u32,
    pub knee: f32,
    pub hip: f32,
    target: (f32, f32, f32),
    asked: Option<(u32, f32, f32, f32)>,
}

impl Owned {
    /// One owner's ask this step; the highest owner id wins.
    pub fn ask(&mut self, owner: u32, knee: f32, hip: f32, rate: f32) {
        if self.asked.is_none_or(|a| owner >= a.0) && (knee != 0.0 || hip != 0.0) {
            self.asked = Some((owner, knee, hip, rate));
        }
    }

    fn step(&mut self, dt: f32) {
        let (owner, k, h, rate) = self.asked.take().unwrap_or((0, 0.0, 0.0, f32::INFINITY));
        (self.owner, self.target) = (owner, (k, h, rate));
        let follow = |x: f32, to: f32| if rate.is_finite() { x + (to - x).clamp(-rate * dt, rate * dt) } else { to };
        (self.knee, self.hip) = (follow(self.knee, k), follow(self.hip, h));
    }
}

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
    /// the crouch request: knee bend and hip pitch, by owner
    pub owned: Owned,
    rng: u32,
    /// this step's draws: prediction time, step clamp change, lift, step width scale
    pub prediction: f32,
    pub clamp_add: f32,
    pub lift: f32,
    pub width_scale: f32,
    /// the swing foot's target last tick (foot, where), for maxFootSpeed
    pub prev_swing: Option<(usize, bevy_math::Vec3)>,
    pub sideways: bool,
    /// the foot that stepped last (0 left, 1 right)
    pub last_foot: usize,
    /// a behaviour asked the balancer to face a way (RDR2's turn step), and this step's leg-twist limits and lift
    pub turning: bool,
    pub leg_twist: (f32, f32),
    pub turn_lift: f32,
    /// the turn step's pause (s; it turns only once this runs out), whether he faced the heading last time, and
    /// whether a step landed last tick
    pause: f32,
    aligned: bool,
    pub landed: bool,
    /// crouch start: knee bend and hip pitch it holds (letting them go), and until when
    pub crouching: Option<(f32, f32, f32)>,
    started: bool,
}

impl Rdr2 {
    pub fn new(p: Params, random: f32) -> Rdr2 {
        Rdr2 { p, time: 0.0, still: 0.0, steps: 0, failing: None, asym: (random * 2.0 - 1.0) * 0.15, on_slope: false,
               since_step: 100000.0, step_age: 0.0, speed: 0.0, running: false, limp: (0, 0.0), owned: Owned::default(),
               rng: (random * 4294967295.0) as u32 | 1, prediction: p.prediction_time, clamp_add: 0.0, lift: p.lift_feet,
               width_scale: 1.0, prev_swing: None, sideways: false, last_foot: 0, turning: false,
               leg_twist: (p.min_leg_twist, p.max_leg_twist), turn_lift: TURN_LIFT.0, pause: 0.0, aligned: false, landed: false, crouching: None,
               started: false }
    }

    /// Timers and the timed fail; `stepped` when a step was committed this tick.
    pub fn update(&mut self, dt: f32, angvel2: f32, stepped: bool, stepping: bool) {
        self.time += dt;
        (self.since_step, self.step_age) = if stepping { (0.0, self.step_age + dt) } else { (self.since_step + dt, 0.0) };
        self.still = if angvel2 < STILL { self.still + dt } else { 0.0 };
        self.steps += stepped as u32;
        if stepped {
            self.draw();
        }
        self.crouch_step(dt);
        if self.failing.is_some() && self.p.ramp_hip_pitch_on_fail {
            self.owned.ask(FAIL_OWNER, 0.0, FAIL_HIP_PITCH, FAIL_HIP_RATE);
        }
        self.owned.step(dt);
        if self.failing.is_none() && !self.p.balance_indefinitely
            && (self.time >= self.p.max_balance_time || self.still >= self.p.max_still_time || self.steps >= self.p.max_steps)
        {
            self.failing = Some(self.time);
        }
    }

    /// Leg (knees, ankles) and per-side hip stiffness while failing, from the balancer's leg stiffness.
    pub fn drain(&self, k: f32) -> Option<(f32, [f32; 2])> {
        let t = self.time - self.failing?;
        // fallType 2: keep balancing at full strength (1, keep stepping, and 3, slump, drain as 0: slump is a state of
        // balancerCollisionsReaction, which RDR2 sets and nothing here reads)
        if self.p.fall_type == 2 {
            return None;
        }
        let m = self.p.fall_mult;
        let leg = (k / (t * m * 0.6 + 1.3)).max(LEG_FLOOR);
        let hip = |a: f32| (k / ((0.4 + a) * m * t + 1.1)).max(HIP_FLOOR);
        Some((leg, [hip(-self.asym), hip(self.asym)]))
    }

    /// Uniform in [-1, 1].
    fn rand(&mut self) -> f32 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 17;
        self.rng ^= self.rng << 5;
        (self.rng as f32 / u32::MAX as f32) * 2.0 - 1.0
    }

    /// RDR2 draws these for each new step (all fixed with its defaults).
    /// The turn step: `want` a heading asked of him, `facing` how far he faces it (cosine). Every third step
    /// landing, and when he turns off a heading he was facing, he waits a random pause (-0.2 to 0.8 s) first.
    pub fn turn(&mut self, dt: f32, want: bool, facing: f32) {
        self.pause -= dt;
        if std::mem::take(&mut self.landed) && self.pause <= 0.0 && self.steps > 1 && self.steps % 3 == 0 {
            self.pause = self.turn_pause();
        }
        self.turning = false;
        if !want {
            return;
        }
        if facing >= TURN_FACING {
            self.aligned = true;
            return;
        }
        if self.aligned && self.pause <= 0.0 {
            self.pause = self.turn_pause();
        }
        if self.pause <= 0.0 {
            (self.turning, self.aligned) = (true, false);
        }
    }

    fn turn_pause(&mut self) -> f32 {
        TURN_PAUSE.0 + (TURN_PAUSE.1 - TURN_PAUSE.0) * (self.rand() + 1.0) * 0.5
    }

    fn draw(&mut self) {
        let p = self.p;
        self.prediction = p.prediction_time + p.prediction_time_variance * self.rand();
        self.clamp_add = p.step_clamp_scale_variance * self.rand();
        self.lift = p.lift_feet + p.lift_feet_variance * self.rand();
        self.leg_twist = (p.min_leg_twist + LEG_TWIST_VAR.0 * self.rand(), p.max_leg_twist + LEG_TWIST_VAR.1 * self.rand());
        self.turn_lift = TURN_LIFT.0 + TURN_LIFT.1 * self.rand();
        let u = (self.rand() + 1.0) * 0.5;
        self.width_scale = p.feet_apart_min + (p.feet_apart_max - p.feet_apart_min) * u;
    }

    /// Do the variances or leanAgainstVelocity change the prediction time from GTA IV's?
    pub fn own_prediction(&self) -> bool {
        self.p.prediction_time_variance != 0.0 || self.p.lean_against_velocity > 0.0 || self.p.prediction_time != 0.2
    }

    /// leanAgainstVelocity: the lean (against the horizontal velocity `v`) and the added prediction time.
    pub fn lean_against(&self, v: bevy_math::Vec3) -> (bevy_math::Vec3, f32) {
        let l = self.p.lean_against_velocity;
        if l <= 0.0 {
            return (bevy_math::Vec3::ZERO, 0.0);
        }
        let f = (v.length() - 1.0).clamp(0.0, 3.0) / 3.0;
        (-v.normalize_or_zero() * l * f * 0.5, l * f * 0.3)
    }

    /// Each foot's step width from the minimum spread `min` (GTA IV's: min x 0.3 + 0.08).
    pub fn widths(&self, min: f32, speed: f32) -> [f32; 2] {
        let w = (min + self.p.extra_feet_apart).mul_add(0.3, 0.08);
        if speed <= self.p.feet_apart_vel {
            return [w; 2];
        }
        let w = w * self.width_scale;
        [if self.p.stop_waddling == 1 { 0.0 } else { w }, if self.p.stop_waddling == 2 { 0.0 } else { w }]
    }

    /// Moving sideways: `along_side` is the velocity direction along the pelvis's side axis.
    pub fn sideways(&mut self, stepping: bool, speed2: f32, along_side: f32) {
        self.sideways = !stepping && self.p.sideways_cheat_force.abs() >= 0.01 && speed2 > SIDEWAYS_SPEED2
            && along_side.abs() > self.p.sideways_thresh;
    }

    /// The ankles' shares of the balance: 1/d^5 of each foot's distance from the COM 0.2 s ahead.
    pub fn ankle_shares(d: [f32; 2]) -> [f32; 2] {
        let w = d.map(|d| (1.0 / (d + 1e-10)).powi(5));
        let sum = w[0] + w[1];
        if sum.is_finite() && sum > 0.0 { [w[0] / sum, w[1] / sum] } else { [0.5; 2] }
    }

    /// The normal an ankle aims at: `n` tilted toward the COM `ahead` of the support `centre`, at most ANKLE_TILT.
    pub fn ankle_aim(n: bevy_math::Vec3, up: bevy_math::Vec3, com: bevy_math::Vec3, vel: bevy_math::Vec3, centre: bevy_math::Vec3) -> bevy_math::Vec3 {
        let v = if vel.length() > ANKLE_SPEED { vel.normalize() * ANKLE_SPEED } else { vel };
        let d = (com + v * ANKLE_AHEAD - centre).normalize_or_zero();
        let axis = up.cross(d);
        if axis.length_squared() < 1e-12 {
            return n;
        }
        let angle = up.dot(d).clamp(-1.0, 1.0).acos().min(ANKLE_TILT);
        bevy_math::Quat::from_axis_angle(axis.normalize(), angle) * n
    }

    /// minKneeAngle: a swing knee at `angle` (moving at `speed`) straighter than -minKneeAngle is held back.
    pub fn knee_limit(&self, target: f32, angle: f32, speed: f32, dt: f32) -> f32 {
        let m = self.p.min_knee_angle;
        if angle > -m { target.min(angle - speed.clamp(0.0, 5.0) - 20.0 * dt * (angle + m)) } else { target }
    }

    /// fallReduceGravityComp: the hips' oppose-gravity share while failing (knees and ankles get twice it).
    pub fn gravity_share(&self) -> Option<f32> {
        let t = self.time - self.failing?;
        self.p.fall_reduce_gravity_comp.then(|| 1.0 / ((0.4 + self.asym) * self.p.fall_mult * t + 1.1))
    }

    /// Crouch start, at his first step: `h` the COM's height over the lower foot, `leg` the leg length, `hip` the hips'
    /// pitch now.
    pub fn crouch_start(&mut self, h: f32, leg: f32, hip: f32) {
        if std::mem::replace(&mut self.started, true) || !self.p.crouch {
            return;
        }
        let c = (0.99 - h / (leg - 0.5 * 0.075)) * 0.79235;
        if c > self.p.crouch_bend_thresh && c < self.p.crouch_bend_max {
            self.crouching = Some((-c.min(0.5), hip.clamp(-1.5, 1.5), self.p.crouch_time));
        }
    }

    /// Crouch start each step: the held bends let go at their rates; asks the crouch request (owner 1).
    pub fn crouch_step(&mut self, dt: f32) {
        let Some((k, h, left)) = self.crouching else { return };
        let (sr, hr) = (self.p.crouch_stand_rate * dt, self.p.crouch_hip_rate * dt);
        let (k, h, left) = ((k + sr).min(0.0), if h > 0.0 { (h - hr).max(0.0) } else { (h + hr).min(0.0) }, left - dt);
        self.crouching = (left > 0.0).then_some((k, h, left));
        self.owned.ask(1, k, h, f32::INFINITY);
    }

    pub fn alternate() -> f32 {
        ALTERNATE
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
        let wait = if self.speed < STEP_WAIT_SPEED { self.p.step_wait } else { FAST_STEP_WAIT };
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

    /// On a slope, between steps: how far the higher foot's target moves, `n` its ground's normal, `apart` from the
    /// lower foot to it. Up the slope, by slopeMoveFoot times how much the slope tilts along the feet.
    pub fn slope_foot(&self, up: bevy_math::Vec3, n: bevy_math::Vec3, apart: bevy_math::Vec3) -> bevy_math::Vec3 {
        let u = apart.normalize_or_zero();
        let (u, up) = (u - n * u.dot(n), up - n * up.dot(n));
        u * (self.p.slope_move_foot * up.dot(u))
    }

    /// On a slope: the point the pelvis stands over (None: the support centre) and its height `h` over it, `l` the
    /// leg's reach, `hi` / `lo` the higher and lower foot.
    pub fn slope_pelvis(&self, up: bevy_math::Vec3, l: f32, h: f32, hi: bevy_math::Vec3, lo: bevy_math::Vec3, stepping: bool, vel: bevy_math::Vec3) -> (Option<bevy_math::Vec3>, f32) {
        let rise = up.dot(hi - lo);
        // the lower foot raised to the higher's height
        let level = lo + up * (bevy_math::Vec3::Z.dot(hi) - up.dot(lo));
        let across = level - hi;
        let d = across.length();
        let along = across.dot(lo - hi);
        let base = (d > 1e-6 && along > 1e-6).then(|| {
            // 0.5 - (rise / spread)^3 of the way from the higher foot, or halfway outside [0, 0.5]
            let x = (level + hi) * 0.5 + up * (rise * rise / d);
            let t = (x - hi).dot(lo - hi) / along;
            let t = if (0.0..=0.5).contains(&t) { t } else { 0.5 };
            hi + (lo - hi) * (0.5 - SLOPE_PELVIS_BLEND * (0.5 - t))
        });
        // stepping, he only sinks moving toward the lower foot
        if stepping && vel.dot(lo - hi) <= 0.0 {
            return (base, h);
        }
        let reach = (l * l - 0.25 * (1.5 * l).min(d).powi(2)).sqrt() - 0.5 * rise;
        let over = if h >= reach { h.min(h - reach) } else { 0.0 };
        (base, (h - over * self.p.slope_drop[stepping as usize]).max(SLOPE_FLOOR))
    }

    /// Share of the ankle balance's twist to keep, from how far the foot is twisted off flat (rad).
    pub fn ankle_twist_weight(&self, off_flat: f32) -> f32 {
        let (a, b) = (self.p.start_flat_twist, self.p.end_flat_twist);
        (1.0 - (off_flat.abs() - a) / (b - a)).clamp(0.0, 1.0)
    }
}
