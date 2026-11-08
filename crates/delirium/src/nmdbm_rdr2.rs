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
//!   (leg length - 0.0375)) x 0.79235, the leg length taken as his standing pelvis height (ours). Both shins lying
//!   (more than acos 0.2 off up) veto it, and the thighs' turn-out (up to 0.35) becomes the legs' twist rest offset;
//! - both feet down and further apart than legsApartMax (2) + their minimum spread + extraFeetApart, the feet are
//!   drawn in toward the COM predicted ahead, the one nearer it moving most;
//! - the stance ankle's pitch and twist from the actual knee and foot (off the ankle balance and moving forward: from
//!   the knee predicted from the hip's desired angles to the foot target); in stagger mode, a swing foot moving
//!   against its own third axis has its ankle's shin term add its damping part instead of taking it off;
//! - braceForImpact's car: the predicted COM clipped to the car's box widened by 0.05 m (in place of an exclusion
//!   plane);
//! - stayUpright's record (rdr2_core3.md 2): the force scaled by tanh of the COM speed, trunk share on pelvis and spine
//!   top, the righting torque, and the fall assist: 0.7 M g up at the COM while he is falling, moving and the balancer
//!   is younger than antigravtime. Its dt is the true NM dt, which sums to real time: the recorder samples once per
//!   game frame and misses one of the three NM steps per 1/30 s (ctx+0x47b8 += dt advances 0.0203 s on the samples
//!   reading dt 0.0102), so a 1/60 s tick needs the code's literal force, no 0.69 factor;
//! Not here: extra foot friction on slopes (where RDR2 reads it isn't known, and Delirium's friction is fixed per
//! part), obstacle and other-leg avoidance, lean priorities, hipError (nothing at its default 0), the turn step's
//! flat-foot test, and fallType 3 (slump: it only sets balancerCollisionsReaction's state, so here it is fallType 0).

/// configureBalance's RDR2 parameters this port uses (RDR2's names and defaults).
#[derive(Clone, Copy, Debug)]
pub struct Params {
    /// movingFloor (+0x2ac) and what reads the floor's acceleration (ctx+0xfe0): leanAcc, hipLeanAcc, leanAccMax, resistAcc, resistAccMax
    pub moving_floor: bool,
    /// ctx+0x1048, ctx+0x104c: friction scale on the feet with the balancer running, and with staggerFall too
    pub foot_friction: [f32; 2],
    pub lean_acc: f32,
    pub hip_lean_acc: f32,
    pub lean_acc_max: f32,
    pub resist_acc: f32,
    pub resist_acc_max: f32,
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
    /// The pitch integrator's centre (bal+0x28c; GTA IV's pitch centre, which GTA IV's standing balance sets itself)
    pub ankle_equilibrium: f32,
    pub legs_apart_restep: f32,
    /// both feet down and further apart than this (plus their minimum spread and extraFeetApart), they are drawn in
    /// toward the predicted COM
    pub legs_apart_max: f32,
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
    /// ctx+0x103c, written straight into the character by this message: self contacts shallower than this aren't recorded (0x2a39ee0)
    pub ignore_depth: f32,
    /// behaviours' joint writes resolved as RDR2's prioritised limb requests (limbreq.rs); off: the last write wins
    pub limb_requests: bool,
    /// limb requests resolved highest slot first: RDR2's queue order (read: a stable insertion sort, highest priority, then
    /// highest sub-priority, first; the first entries dominate). Off: lowest slot first, for comparison
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
    /// the planted feet's yaw hold gain while the balancer runs (the core's 60 before), s^2 with s = 7.7 (the message's
    /// footSlipStiffness is s, its handler keeps s^2 and 2s), the damping being 2s: RDR2 writes it every step, and its
    /// running value (+0x758) in run mode; here this one in both
    pub foot_slip_stiffness: f32,
    /// standing (below 1 m/s, not staggering), the stance ankle aims from the actual knee to the actual foot on the
    /// renormalised contact normal (RDR2's +0x9c state test is left out)
    pub flatter_static_feet: bool,
    /// the timed fail starts once the trunk leans back past this (bal+0x6fc)
    pub backwards_lean_cutoff: f32,
    /// below this COM speed, and the rotation below its own, the stable feedback (bal+0x6e0)
    pub stable_lin: f32,
    /// (bal+0x6e4)
    pub stable_rot: f32,
    /// the fail tests only set a flag (bal+0x6d9)
    pub ignore_failure: bool,
    /// a failing test waits for a part off the legs to touch something (bal+0x6d8)
    pub fail_must_collide: bool,
    /// below this speed the twist and the sideways force take the way he faces, not the velocity (bal+0x2b0)
    pub use_com_dir_turn_vel_thresh: f32,
    /// feeds the stance legs the pelvis error ahead, and eases the single support pitch rule (bal+0x700)
    pub hip_error: f32,
    /// the same when running (bal+0x74c)
    pub hip_error_run: f32,
    /// the most that feed turns the pelvis frame, rad (bal+0x2c8)
    pub hip_error_max: f32,
    /// the desired pelvis frame is turned against the pelvis's spin (bal+0x2fb)
    pub damp_pelvis: bool,
    /// a swing foot moving down lies flat with no gravity opposition on its ankle (bal+0x2f9)
    pub flatten_down_swing: bool,
    /// swing ankles aim the foot flat (bal+0x6f0)
    pub flatter_swing_feet: bool,
    /// RDR2's 2bc0e7e9: the same when running (bal+0x748)
    pub flatter_swing_feet_run: bool,
    /// off, or running: a planted leading foot that lost the ground swings (bal+0x6e8)
    pub airborne_step: bool,
    /// running: how far a foot may be behind him, moving backward (bal+0x5c4)
    pub run_back: f32,
    /// the same moving forward (bal+0x5cc)
    pub run_back_fwd: f32,
    /// running: how far the other foot must be ahead of him, moving backward; the code's doubtful name (bal+0x5c8)
    pub run_ahead: f32,
    /// the same moving forward (bal+0x5d0)
    pub run_ahead_fwd: f32,
    /// turning: s since the last step before a new one (bal+0x2a4)
    pub step_turn_wait: f32,
    /// no new step this long into the balance (bal+0x6a4)
    pub dont_step_time: f32,
    /// the swing hip's damping share (bal+0x660)
    pub swing_damping_l: f32,
    /// (bal+0x664)
    pub swing_damping_r: f32,
    /// hips this, knees twice it, spine 0 this (bal+0x668)
    pub oppose_gravity_legs: f32,
    /// ankles twice this (bal+0x66c)
    pub oppose_gravity_ankles: f32,
    /// running's footSlipStiffness; the message squares it (bal+0x758)
    pub foot_slip_stiffness_run: f32,
    /// the swing target is kept off the other leg (bal+0x2b5)
    pub avoid_leg: bool,
    /// the corridor's half width, moving backward (bal+0x5bc)
    pub avoid_foot_width: f32,
    /// RDR2's c5ebfd7b: moving forward (bal+0x5c0)
    pub avoid_foot_width_fwd: f32,
    /// (bal+0x2bc)
    pub avoid_feedback: f32,
    /// RDR2's 428c2b6f (bal+0x714)
    pub turn_lift_feet: f32,
    /// RDR2's cfe51361 (bal+0x718)
    pub turn_lift_feet_variance: f32,
    /// RDR2's 96c82edf (bal+0x720)
    pub min_leg_twist_variance: f32,
    /// RDR2's 5831d084 (bal+0x728)
    pub max_leg_twist_variance: f32,
    /// RDR2's 747a3a5c (bal+0x27c)
    pub slope_pelvis_blend: f32,
    /// (bal+0x298)
    pub stagger_fail_height: f32,
}

impl Default for Params {
    fn default() -> Params {
        Params { foot_friction: [1.0, 1.0], moving_floor: false, lean_acc: 0.0, hip_lean_acc: 0.5, lean_acc_max: 5.0, resist_acc: 0.0, resist_acc_max: 3.0, max_steps: 100, max_balance_time: 50.0, max_still_time: 50.0, balance_indefinitely: false, giveup_height: 0.5,
                 fall_type: 0, fall_mult: 1.0, step_height_inc4_step: 0.1, ankle_balance: true, start_flat: 0.1, end_flat: 0.3,
                 slope_min_dot: 0.05, no_slope_time: 0.0, slope_move_foot: 0.07, slope_drop: [0.7, 1.8], ankle_equilibrium: 0.0, legs_apart_restep: 0.2,
                 legs_apart_max: 2.0, legs_together_restep: 1.0, step_decision_threshold: 0.0, step_wait: -1.0, change_step_time: -1.0,
                 always_step_with_farthest: false, run_vel: 100.0, leg_stiffness_run: 13.0, step_height_run: 0.6,
                 taper_knee_strength_run: false, limp_scale: 1.0, ignore_depth: -100.0, limb_requests: true, limb_order_reversed: true,
                 max_foot_speed: 5.0, ramp_hip_pitch_on_fail: true, prediction_time: 0.2, prediction_time_variance: 0.0,
                 step_clamp_scale_variance: 0.0, lift_feet: 0.07, lift_feet_variance: 0.0, lean_against_velocity: 0.0,
                 stand_up: false, extra_feet_apart: 0.0, feet_apart_min: 1.0, feet_apart_max: 1.0, feet_apart_vel: 100.0,
                 stop_waddling: 0, sideways_cheat_force: 0.0, sideways_thresh: 0.8,
                 step_if_in_support: true, ankle_offset: -0.12, ankle_mult: 25.0, min_knee_angle: -0.5,
                 fall_reduce_gravity_comp: false, min_leg_twist: -1.9, max_leg_twist: 0.2, ankle_keep_pelvis: false,
                 crouch: false, crouch_bend_thresh: 0.3, crouch_bend_max: 0.55, crouch_time: 0.5, crouch_stand_rate: 0.4,
                 crouch_hip_rate: 0.25, start_flat_twist: 0.05, end_flat_twist: 0.15, ankle_twist_mult: 20.0,
                 foot_slip_stiffness: 59.29, flatter_static_feet: false,
                 backwards_lean_cutoff: 1.1, stable_lin: 0.25, stable_rot: 0.25, ignore_failure: false, fail_must_collide: false, use_com_dir_turn_vel_thresh: 0.0, hip_error: 0.0, hip_error_run: 0.0, hip_error_max: 0.9, damp_pelvis: true, flatten_down_swing: false, flatter_swing_feet: false, flatter_swing_feet_run: true, airborne_step: true, run_back: 0.65, run_back_fwd: 0.65, run_ahead: -0.1, run_ahead_fwd: 0.05, step_turn_wait: -1.0, dont_step_time: 0.0, swing_damping_l: 1.0, swing_damping_r: 1.0, oppose_gravity_legs: 1.0, oppose_gravity_ankles: 1.0, foot_slip_stiffness_run: 161.29, avoid_leg: false, avoid_foot_width: 0.1, avoid_foot_width_fwd: 0.1, avoid_feedback: 0.6, turn_lift_feet: 0.17, turn_lift_feet_variance: 0.07, min_leg_twist_variance: 0.3, max_leg_twist_variance: 0.2, slope_pelvis_blend: 1.0, stagger_fail_height: 0.65 }
    }
}

impl Params {
    /// configureBalance's value for one of these, as the message sends it.
    pub fn set_message(&mut self, name: &str, v: f32) -> bool {
        self.set(name, if matches!(name, "footSlipStiffness" | "footSlipStiffnessRun") { v * v } else { v })
    }

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
            "ankleEquilibrium" => self.ankle_equilibrium = v,
            "sidewaysmovefeet" | "predictionTimeHip" | "footslipdamping" => {}
            "footFriction" => self.foot_friction[0] = v,
            "footFrictionStagger" => self.foot_friction[1] = v,
            "movingFloor" => self.moving_floor = v != 0.0,
            "leanAcc" => self.lean_acc = v,
            "hipLeanAcc" => self.hip_lean_acc = v,
            "leanAccMax" => self.lean_acc_max = v,
            "resistAcc" => self.resist_acc = v,
            "resistAccMax" => self.resist_acc_max = v,
            "legsApartRestep" => self.legs_apart_restep = v,
            "legsApartMax" => self.legs_apart_max = v,
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
            "ignoredepth" => self.ignore_depth = v,
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
            "footSlipStiffness" => self.foot_slip_stiffness = v,
            "flatterStaticFeet" => self.flatter_static_feet = v != 0.0,
            "backwardsLeanCutoff" => self.backwards_lean_cutoff = v,
            "stableLinSpeedThresh" => self.stable_lin = v,
            "stableRotSpeedThresh" => self.stable_rot = v,
            "ignoreFailure" => self.ignore_failure = v != 0.0,
            "failMustCollide" => self.fail_must_collide = v != 0.0,
            "useComDirTurnVelThresh" => self.use_com_dir_turn_vel_thresh = v,
            "hipError" => self.hip_error = v,
            "hipErrorRun" => self.hip_error_run = v,
            "hipErrorMax" => self.hip_error_max = v,
            "dampPelvis" => self.damp_pelvis = v != 0.0,
            "flattenDownSwing" => self.flatten_down_swing = v != 0.0,
            "flatterSwingFeet" => self.flatter_swing_feet = v != 0.0,
            "flatterSwingFeetRun" => self.flatter_swing_feet_run = v != 0.0,
            "airborneStep" => self.airborne_step = v != 0.0,
            "runningThresh" => self.run_back = v,
            "runningThreshB" => self.run_back_fwd = v,
            "clavicleCrouchingAccumulated" => self.run_ahead = v,
            "clavicleCrouchingAccumulatedB" => self.run_ahead_fwd = v,
            "stepTurnWait" => self.step_turn_wait = v,
            "dontStepTime" => self.dont_step_time = v,
            "leftLegSwingDamping" => self.swing_damping_l = v,
            "rightLegSwingDamping" => self.swing_damping_r = v,
            "opposeGravityLegs" => self.oppose_gravity_legs = v,
            "opposeGravityAnkles" => self.oppose_gravity_ankles = v,
            "footSlipStiffnessRun" => self.foot_slip_stiffness_run = v,
            "avoidLeg" => self.avoid_leg = v != 0.0,
            "avoidFootWidth" => self.avoid_foot_width = v,
            "avoidFootWidthB" => self.avoid_foot_width_fwd = v,
            "avoidFeedback" => self.avoid_feedback = v,
            "turnLiftFeet" => self.turn_lift_feet = v,
            "turnLiftFeetVariance" => self.turn_lift_feet_variance = v,
            "minLegTwistVariance" => self.min_leg_twist_variance = v,
            "maxLegTwistVariance" => self.max_leg_twist_variance = v,
            "slopePelvisBlend" => self.slope_pelvis_blend = v,
            "staggerFailHeight" => self.stagger_fail_height = v,
            _ => return false,
        }
        true
    }
}

macro_rules! stay_fields {
    (bool: $(($bn:literal, $b:ident)),*; f32: $(($fn:literal, $f:ident, $d:expr)),*) => {
        /// RDR2's StayUpright record (ctx+0x29a0 + side x 0xd0; the template's values, rdr2_core3.md 2.1): the message
        /// copies this over a set, then its parameters. All off until a message sets useForces or useTorques.
        #[derive(Clone, Copy, Debug)]
        pub struct StayUpright {
            $(pub $b: bool,)*
            $(pub $f: f32,)*
            pub torque_strength: bevy_math::Vec3,
            pub torque_damping: bevy_math::Vec3,
        }

        impl Default for StayUpright {
            fn default() -> StayUpright {
                StayUpright { $($b: false,)* $($f: $d,)*
                              torque_strength: bevy_math::Vec3::ZERO, torque_damping: bevy_math::Vec3::new(0.5, 0.5, 0.0) }
            }
        }

        impl StayUpright {
            /// One float or flag (1/0) by its RDR2 name; false when the name isn't one of them.
            pub fn set(&mut self, name: &str, v: f32) -> bool {
                match name {
                    $($bn => self.$b = v != 0.0,)*
                    $($fn => self.$f = v,)*
                    _ => return false,
                }
                true
            }

            pub fn get(&self, name: &str) -> Option<f32> {
                match name {
                    $($bn => Some(self.$b as u8 as f32),)*
                    $($fn => Some(self.$f),)*
                    _ => None,
                }
            }

            pub fn is_flag(name: &str) -> bool {
                matches!(name, $($bn)|*)
            }
        }
    };
}

// Names by joaat of the message's hashes, but for leanRelative (49e2f424) and switchSpeed (ab3a861c), which are ours.
// lastStand*, stepUpHelp are carried and do nothing here (stayUpAcc* reads in nm.rs): off or without effect on flat still ground.
stay_fields! {
    bool: ("useForces", use_forces), ("useTorques", use_torques), ("lastStandMode", last_stand_mode), ("velocityBased", velocity_based),
          ("torqueOnlyInAir", torque_only_in_air), ("leanRelative", lean_relative), ("torqueparts", torque_parts),
          ("turnTowardsBullets", turn_towards_bullets);
    f32: ("switchSpeed", switch_speed, 100.0), ("lastStandSinkRate", last_stand_sink_rate, 0.3), ("lastStandHorizDamping", last_stand_horiz_damping, 0.4),
         ("lastStandMaxTime", last_stand_max_time, 0.4), ("forceStrength", force_strength, 3.0), ("forceDamping", force_damping, -1.0),
         ("forceFeetMult", force_feet_mult, 1.0), ("forceSpine3Share", force_spine3_share, 0.3), ("forceLeanReduction", force_lean_reduction, 1.0),
         ("forceInAirShare", force_in_air_share, 0.5), ("forceMin", force_min, -1.0), ("forceMax", force_max, -1.0),
         ("forceSaturationVel", force_saturation_vel, 4.0), ("forceThresholdVel", force_threshold_vel, 0.5),
         ("torqueSaturationVel", torque_saturation_vel, 4.0), ("torqueThresholdVel", torque_threshold_vel, 2.5),
         ("torqueleanreduction", torque_lean_reduction, 0.0), ("torqueleanmin", lean_min, 0.5), ("torqueleanmax", lean_max, 1.1), ("torquelean", lean, 0.0),
         ("supportPosition", support_position, 2.0), ("noSupportForceMult", no_support_force_mult, 1.0), ("stepUpHelp", step_up_help, 0.0),
         ("stayUpAcc", stay_up_acc, 0.7), ("stayUpAccMax", stay_up_acc_max, 5.0),
         ("antigrav", antigrav, 0.0), ("antigravtime", antigrav_time, 0.5), ("antigravspeed", antigrav_speed, 1.5)
}

impl StayUpright {
    /// A velocityBased window: 0 below the threshold, tanh up to the saturation speed (1 for an empty window).
    pub fn window(speed: f32, threshold: f32, saturation: f32) -> f32 {
        if saturation > threshold {
            (3.0 / (saturation - threshold) * (speed - threshold)).tanh().max(0.0)
        } else if speed >= threshold {
            1.0
        } else {
            0.0
        }
    }

    /// The fall assist's force up (0x2abba6d..0x2abbb39), this balancer tick: the balancer younger than antigravtime,
    /// the horizontal speed over antigravspeed, the COM moving down. The caller applies it to the pelvis at the COM,
    /// for dt, as the code does: force x dt, no further scaling.
    pub fn fall_assist(&self, age: f32, vel: bevy_math::Vec3, mass: f32) -> f32 {
        if self.antigrav_time > age && vel.truncate().length() > self.antigrav_speed && vel.z < 0.0 {
            mass * 9.81 * self.antigrav
        } else {
            0.0
        }
    }
}

/// What the character's StayUpright sets choose from (0x2a58f10): the slow set below the fast set's switch speed.
pub fn stay_live(sets: &[StayUpright; 2], vel: bevy_math::Vec3) -> StayUpright {
    sets[(vel.truncate().length() >= sets[1].switch_speed) as usize]
}

/// The ankles' support test: fail when 2.4 x the COM's height over them is less than its distance from them.
pub const ANKLE_ABORT: f32 = 2.4;
/// The ankle's height above the sole (the character's foot block, ctx+0x47c4): half of it lifts each probed contact
/// and is added to the leg scale (bal+0x220, bal+0x210).
pub const FOOT_HEIGHT: f32 = 0.075;
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
/// Stagger mode: the feet may be this much further apart before he re-steps.
const STAGGER_RESTEP: f32 = 0.3;
/// stepWait applies below this speed (m/s); above it the wait is FAST_STEP_WAIT, which never holds a step back.
const STEP_WAIT_SPEED: f32 = 0.7;
const FAST_STEP_WAIT: f32 = -0.1;
/// The turn step: facing the heading this closely (cosine) he doesn't turn; the pause's range (s).
const TURN_FACING: f32 = 0.9;
const TURN_PAUSE: (f32, f32) = (-0.2, 0.8);
/// The pelvis never sinks below this over the feet on a slope.
const SLOPE_FLOOR: f32 = 0.2;
/// Running, with the planted foot off the ground: the stepping leg's hip and knee stiffness, damping and muscle (0x2abc4ec).
pub const RUN_SWING: (f32, f32, f32) = (15.0, 0.5, 6.0);
/// How long a foot made to give way waits (bal+0x1f4, 1, times 0.1 s), how far its target drops (0.4 of the probe's
/// 1 m) and how the other leg's hip and ankle are held up while it does, per second left (0x2aaeed8).
pub const GIVE_WAY: (f32, f32, f32, f32) = (0.1, 0.4, 24.0, -30.0);
/// Staggering, the thighs' meeting only counts while going less than this along the pelvis.
pub const GIVE_WAY_ALONG: f32 = 0.4;
/// Running above this speed, staggering included, the legs meeting doesn't count.
pub const GIVE_WAY_SPEED: f32 = 2.1;
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
/// Ankle balance: the COM look-ahead (s), its speed cap, the largest tilt of the aimed normal, the feet's share power.
const ANKLE_AHEAD: f32 = 0.4;
const ANKLE_SPEED: f32 = 0.3;
pub const ANKLE_TILT: f32 = 0.2;
const SIDEWAYS_SPEED2: f32 = 0.25;
/// The runtime reset's current-step age (bal+0x204) and leg-twist limits (+0x394 / +0x398, until the first step's draw).
const RESET_STEP_AGE: f32 = 0.3;
const RESET_LEG_TWIST: (f32, f32) = (-1.9, 0.6);
/// braceForImpact's car box: its half extents across and along widened by this (m) for the balancer.
pub const CAR_MARGIN: f32 = 0.05;
/// Crouch start: a shin this close to lying (cosine of its angle to up) on both legs vetoes it; the turn-out kept, at most.
const LYING_SHIN: f32 = 0.2;
const TURN_OUT_MAX: f32 = 0.35;

/// One channel of RDR2's owned crouch request: a target the balancer follows at a rate (infinite: at once). The
/// highest id asking this step owns it; the owner resets every step but the target stays until someone asks again
/// or its asker lets go (back to 0 at a rate).
#[derive(Clone, Copy, Debug, Default)]
pub struct Chan {
    pub owner: u32,
    pub value: f32,
    target: f32,
    rate: f32,
    asked: Option<(u32, f32, f32)>,
}

impl Chan {
    pub fn ask(&mut self, owner: u32, target: f32, rate: f32) {
        if self.asked.is_none_or(|a| owner >= a.0) {
            self.asked = Some((owner, target, rate));
        }
    }

    /// Letting go: back to 0 at this rate (unless someone asks this step).
    pub fn release(&mut self, rate: f32) {
        if self.asked.is_none() {
            self.asked = Some((0, 0.0, rate));
        }
    }

    fn step(&mut self, dt: f32) {
        self.owner = 0;
        if let Some((owner, to, rate)) = self.asked.take() {
            (self.owner, self.target, self.rate) = (owner, to, rate);
        }
        let (to, rate) = (self.target, self.rate);
        self.value = if rate.is_finite() { self.value + (to - self.value).clamp(-rate * dt, rate * dt) } else { to };
    }
}

/// RDR2's crouch request the balancer owns: a knee bend (added to leg straightness) and a hip pitch (rad, flexing the
/// hips), each with its own owner (RDR2's owner ids double as priorities).
#[derive(Clone, Copy, Debug, Default)]
pub struct Owned {
    pub knee: Chan,
    pub hip: Chan,
}

impl Owned {
    /// One owner's ask this step on both channels (a channel asked 0 is left alone).
    pub fn ask(&mut self, owner: u32, knee: f32, hip: f32, rate: f32) {
        if knee != 0.0 {
            self.knee.ask(owner, knee, rate);
        }
        if hip != 0.0 {
            self.hip.ask(owner, hip, rate);
        }
    }

    fn step(&mut self, dt: f32) {
        self.knee.step(dt);
        self.hip.step(dt);
    }
}

/// A behaviour's request of the balancer's crouch: (owner, target, rate) on the knee bend or hip pitch, or letting go.
#[derive(Clone, Copy, Debug)]
pub enum Crouch {
    Knee(u32, f32, f32),
    Hip(u32, f32, f32),
    ReleaseKnee(f32),
    ReleaseHip(f32),
}

/// The limp's balancer fields, set by configureShotInjuredLeg (ctor's values: no shift, scales 1, the byte set).
#[derive(Clone, Copy, Debug)]
pub struct LimpCfg { pub bias: f32, pub width: f32, pub width_injured: f32, pub both: bool }

impl Default for LimpCfg {
    fn default() -> LimpCfg {
        LimpCfg { bias: 0.0, width: 1.0, width_injured: 1.0, both: true }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct Rdr2 {
    /// bal+0x2fc, the shot's side lever, and bal+0x454, the step count it was given at: while no step has been taken since, it biases the foot choice
    pub lever: (f32, u32),
    /// yanked's turn block: the "other foot farther" margin stays 0.2 leg lengths while turning
    pub turn_margin: bool,
    pub p: Params,
    pub time: f32,
    pub still: f32,
    /// his standing leg straightness and the leg scale (ctx+0x34d0, bal+0x210), for duck's knee measure
    pub straight: (f32, f32),
    pub steps: u32,
    /// extraSteps given: taken off the step counter maxSteps is tested against (0x2a63cbc)
    pub extra_steps: u32,
    /// flattenDownSwing's flag for each leg this tick (bal+0x342)
    pub down_swing: [bool; 2],
    /// the up this tick, for the step decision
    pub up_now: bevy_math::Vec3,
    /// when the timed fail started (balance time)
    pub failing: Option<f32>,
    /// the fail test's ratios (bal+0x6c, bal+0x70): off over height at the contacts and at the ankles
    pub fail_ratio: (f32, f32),
    /// in [-0.15, 0.15]: which hip folds first
    pub asym: f32,
    pub on_slope: bool,
    /// s since the last step ended (large before the first), and the current step's age
    pub since_step: f32,
    pub step_age: f32,
    pub speed: f32,
    /// bal+0x339: staggering, or at runVel or faster (0x2aba1cc). The foot rules go by this.
    pub running: bool,
    /// At runVel or faster: only then the run values (legStiffnessRun, hipErrorRun, ...) replace the walking ones.
    pub run_fast: bool,
    /// bal+0x324, +0x328: since a foot was made to give way in the air, for how long it's kept down and the other held up.
    pub wait: [f32; 2],
    /// bal+0x334: the legs met, and the step gives way (0x2aba1cc, 0x2ab32bc).
    pub give_way: bool,
    /// bal+0x349: a staggering leg is pushing off (state 3 or 4).
    pub push_off: bool,
    /// bal+0x338: staggerFall's fall has begun; no running anti-crossing.
    pub falling: bool,
    /// the injured leg (0 left, 1 right) and how badly (0..1), from shotInjuredLeg
    pub limp: (usize, f32),
    /// configureShotInjuredLeg's injuredbias, limpwidth, limpwidthinjured and both-legs byte (bal+0x5dc, +0x5a0, +0x5a4, +0xbeb)
    pub limp_cfg: LimpCfg,
    /// each leg's lift multiplier (bal+0x23c, +0x240; below 0 off): the swing target's greatest reach from the hip, by the leg's length
    pub lift_mult: [f32; 2],
    /// RDR2's forced step: this foot (0 left, 1 right) steps next, for one step from the step count it was asked at;
    /// and whether to drop the step in progress first
    pub forced: Option<(usize, u32)>,
    pub cancel_step: bool,
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
    pub last_foot: Option<usize>,
    /// a behaviour asked the balancer to face a way (RDR2's turn step), and this step's leg-twist limits and lift
    pub turning: bool,
    /// the heading asked this tick is bodyBalance's, which asks no turn step
    pub heading_only: bool,
    /// this step's turn request: how closely he must face the heading to count as facing it (RDR2's 0.9 unless the
    /// requester says otherwise), and whether a behaviour forbids new steps (RDR2's force balance)
    pub turn_facing: Option<f32>,
    pub force_balance: bool,
    /// the balance quality (bal+0x68): 5.5556 x the least distance of the COM 0.3 s ahead from each foot 0.3 s ahead and
    /// their middle, over the COM's height above the lower foot; it rises at once and falls by a quarter a tick
    pub quality: f32,
    /// balanceAbortThreshold as fall to knees wrote it (bal+0x294), over the configured one
    pub abort: Option<f32>,
    /// failMustCollide (bal+0x6d8): failing waits for a part off the legs to touch something
    pub must_collide: bool,
    /// bal+0x33a, kneeling: the pelvis control's both-feet terms are off and there's no stable feedback
    pub kneeling: bool,
    /// fall to knees' fail now (0x2ab4b7c)
    pub force_fail: bool,
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
    /// bal+0x468: the first tick after a reset
    pub first_tick: bool,
    /// the legs' twist rest offset (bal+0x22c): the pelvis control relaxes the leg twists toward it, less GTA IV's base;
    /// a crouch start keeps the thighs' turn-out here
    pub twist_rest: f32,
    /// braceForImpact's car this step (bal+0x1b0..+0x1e0): its box's corners, widened by CAR_MARGIN, in order round it
    pub car: Option<[bevy_math::Vec3; 4]>,
    /// staggerFall's stagger mode this step (bal+0xbe2): feet further apart before a re-step, the alternation bias, the
    /// sideways test, its fail height, and no ankle balance on the stance foot
    pub stagger: bool,
    /// staggerFall's prediction time this step (bal+0x6bc over the base), in place of the balancer's own
    pub lead: Option<f32>,
    /// teeter's addition to the prediction time this step (bal+0x6bc with +0x6c0 set)
    pub lead_add: f32,
    /// taperKneeStrength (bal+0x654): off while teeter boosts or leans away, and in highFall's landing on the feet
    pub taper: bool,
    /// teeter's exclusion plane this step (bal+0x33c): point, horizontal normal, side sign, offset
    pub plane: Option<(bevy_math::Vec3, bevy_math::Vec3, f32, f32)>,
}

impl Rdr2 {
    pub fn new(p: Params, random: f32) -> Rdr2 {
        Rdr2 { lever: (0.0, 0), p, turn_margin: false, time: 0.0, still: 0.0, straight: (0.99, 1.0), steps: 0, extra_steps: 0, down_swing: [false; 2], up_now: bevy_math::Vec3::Z, failing: None, asym: (random * 2.0 - 1.0) * 0.15, on_slope: false,
               since_step: 100000.0, step_age: RESET_STEP_AGE, speed: 0.0, running: false, run_fast: false, wait: [0.0; 2], give_way: false, push_off: false, falling: false, limp: (0, 0.0), limp_cfg: LimpCfg::default(), lift_mult: [-1.0; 2], forced: None, cancel_step: false, owned: Owned::default(),
               rng: (random * 4294967295.0) as u32 | 1, prediction: p.prediction_time, clamp_add: 0.0, lift: p.lift_feet,
               width_scale: 1.0, prev_swing: None, sideways: false, last_foot: None, turning: false, heading_only: false,
               leg_twist: RESET_LEG_TWIST, turn_lift: p.turn_lift_feet, turn_facing: None, force_balance: false, fail_ratio: (0.0, 0.0), quality: 0.0, abort: None, must_collide: false, kneeling: false, force_fail: false, pause: 0.0, aligned: false, landed: false, crouching: None,
               started: false, first_tick: true, twist_rest: 0.0, car: None, stagger: false, lead: None, lead_add: 0.0, taper: true, plane: None }
    }

    /// The balance quality's tick (the fail test's tail, 0x2abda8c), each foot's position and velocity.
    pub fn measure(&mut self, up: bevy_math::Vec3, com: bevy_math::Vec3, vel: bevy_math::Vec3, feet: [(bevy_math::Vec3, bevy_math::Vec3); 2]) {
        let h = up.dot(com) - up.dot(feet[0].0).min(up.dot(feet[1].0));
        let at = com + vel * 0.3;
        let (l, r) = (feet[0].0 + feet[0].1 * 0.3, feet[1].0 + feet[1].1 * 0.3);
        let d = |x: bevy_math::Vec3| {
            let o = x - at;
            (o - up * up.dot(o)).length()
        };
        let x = d(l).min(d(r)).min(d((l + r) * 0.5)) * (5.5555553 / h.max(0.001));
        self.quality = if x <= self.quality { (x - self.quality) * 0.25 + self.quality } else { x };
    }

    /// configureBalance's extraSteps and extraTime (0x2a63cbc): they give back steps and time, never below none used.
    pub fn extra(&mut self, name: &str, v: f32) {
        match name {
            "extraSteps" if v > 0.0 => {
                crate::extras_core_rdr2::trace(|| format!("balancer: {v} extra steps at {} steps", self.steps));
                self.extra_steps = (self.extra_steps + v as u32).min(self.steps);
            }
            "extraTime" if v >= 0.0 => {
                crate::extras_core_rdr2::trace(|| format!("balancer: {v} s extra at {:.2} s", self.time));
                self.time = (self.time - v).max(0.0);
            }
            _ => {}
        }
    }

    /// The backward lean (0x2abda8c): how far the trunk's top, its two sides and its direction go behind the pelvis, along
    /// the pelvis's third axis off `up`. Past backwardsLeanCutoff the timed fail starts. Two of the three are metres and
    /// one a sine, as in the code.
    pub fn back_lean(&mut self, up: bevy_math::Vec3, pelvis: &crate::nmdbm::M34, top: &crate::nmdbm::M34) {
        let back = (pelvis.c - up * pelvis.c.dot(up)).normalize_or_zero();
        let o = top.d - pelvis.d;
        let x = (o + top.b * 0.16).dot(back).max(o.normalize_or_zero().dot(back)).max((o - top.b * 0.16).dot(back));
        if x > self.p.backwards_lean_cutoff && self.failing.is_none() {
            self.failing = Some(self.time);
            crate::extras_core_rdr2::trace(|| format!("balancer: leaning back {x:.2} past {} at {:.2} s", self.p.backwards_lean_cutoff, self.time));
        }
    }

    /// hipError or its running value.
    pub fn hip_error(&self) -> f32 {
        if self.run_fast { self.p.hip_error_run } else { self.p.hip_error }
    }

    /// Do this leg's swing ankle aim flat (0x2aae0b4)? Moving sideways, flatterSwingFeet or a foot moving down.
    pub fn flat_swing(&self, i: usize) -> bool {
        self.sideways || self.down_swing[i] || self.wait[1 - i] > 0.0 || if self.run_fast { self.p.flatter_swing_feet_run } else { self.p.flatter_swing_feet }
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
        // the car is the brace's this step: placement has used it
        self.car = None;
        if self.failing.is_some() && self.p.ramp_hip_pitch_on_fail {
            self.owned.ask(FAIL_OWNER, 0.0, FAIL_HIP_PITCH, FAIL_HIP_RATE);
        }
        self.owned.step(dt);
        if self.failing.is_none() && !self.p.balance_indefinitely
            && (self.time >= self.p.max_balance_time || self.still >= self.p.max_still_time || self.steps.saturating_sub(self.extra_steps) >= self.p.max_steps)
        {
            self.failing = Some(self.time);
            crate::extras_core_rdr2::trace(|| format!("balancer: timed fail at {:.2} s, {} steps of {}", self.time, self.steps, self.p.max_steps));
        }
    }

    /// Leg (knees, ankles) and per-side hip stiffness while failing, from the balancer's leg stiffness.
    pub fn drain(&self, k: f32) -> Option<(f32, [f32; 2])> {
        let t = self.time - self.failing?;
        // the whole ramp-down sits under balanceIndefinitely off (0x2abc4ec)
        if self.p.balance_indefinitely {
            return None;
        }
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
        if facing >= self.turn_facing.take().unwrap_or(TURN_FACING) {
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
        self.leg_twist = (p.min_leg_twist + p.min_leg_twist_variance * self.rand(), p.max_leg_twist + p.max_leg_twist_variance * self.rand());
        self.turn_lift = p.turn_lift_feet + p.turn_lift_feet_variance * self.rand();
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

    /// The exclusion plane's clamp of a predicted point (RDR2's placement with bal+0x33c set: the offset alone):
    /// e = (q - point - offset n).n; if side e > 0 the point is moved back onto the offset plane.
    pub fn exclude(&self, q: bevy_math::Vec3) -> bevy_math::Vec3 {
        let Some((at, n, side, off)) = self.plane else { return q };
        let e = (q - at - n * off).dot(n);
        if side * e > 0.0 { q - n * e } else { q }
    }

    /// Each foot's step width from the minimum spread `min` (GTA IV's: min x 0.3 + 0.08).
    pub fn widths(&self, min: f32, speed: f32, stepping: u32) -> [f32; 2] {
        let w = (min + self.p.extra_feet_apart).mul_add(0.3, 0.08);
        let base = if speed <= self.p.feet_apart_vel {
            [w; 2]
        } else {
            let w = w * self.width_scale;
            [if self.p.stop_waddling == 1 { 0.0 } else { w }, if self.p.stop_waddling == 2 { 0.0 } else { w }]
        };
        self.limp_widths(base, w, stepping)
    }

    /// The limp (bal+0x244 over 0, in stage 1): no step, both as they were; stepping, both at limpwidthinjured, and when the
    /// healthy foot steps its own at limpwidth (with the both-legs byte, both).
    fn limp_widths(&self, base: [f32; 2], w: f32, stepping: u32) -> [f32; 2] {
        if self.limp.1 <= 0.0 || stepping == 0 {
            return base;
        }
        let (h, c) = (self.limp.0, self.limp_cfg);
        let mut out = [c.width_injured * w; 2];
        // the hurt leg is 0 left: the healthy foot steps as 2; hurt right, as 1
        if stepping == if h == 0 { 2 } else { 1 } {
            out[1 - h] = c.width * w;
            if c.both {
                out[h] = c.width * w;
            }
        }
        crate::extras_core_rdr2::trace_once("balancer: limp widths", || format!("{out:?} of {w}, leg {h} hurt, foot {stepping} stepping"));
        out
    }

    /// The support centre (bal+0x420): the middle, limping away from the hurt leg by injuredbias.
    pub fn support(&self, l: bevy_math::Vec3, r: bevy_math::Vec3) -> bevy_math::Vec3 {
        if self.limp.1 <= 0.0 || self.limp_cfg.bias == 0.0 {
            return (l + r) * 0.5;
        }
        let w = if self.limp.0 == 0 { 0.5 + self.limp_cfg.bias } else { 0.5 - self.limp_cfg.bias };
        crate::extras_core_rdr2::trace_once("balancer: limp bias", || format!("support at {w} from the left foot"));
        l + (r - l) * w
    }

    /// Moving sideways: `along_side` is the velocity direction along the pelvis's side axis.
    pub fn sideways(&mut self, speed2: f32, along_side: f32) {
        self.sideways = (self.stagger || self.p.sideways_cheat_force.abs() >= 0.01) && speed2 > SIDEWAYS_SPEED2
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

    /// 0x2ab617c: how deep a crouch goes, from `h` the pelvis's world height over the lower foot's and `straight` ctx+0x34d0
    /// (the same sum as the start's).
    pub fn crouch_depth(h: f32, leg: f32, straight: f32) -> f32 {
        let l = leg - 0.5 * FOOT_HEIGHT;
        if l > 0.1 { (straight - h / l) * 0.79235 } else { 0.0 }
    }

    /// Crouch start (activate 0x2ab728c; here at his first tick): `h` the pelvis's world height over the lower foot's,
    /// `leg` the leg scale, `straight` his standing leg straightness, `hip` the hips' pitch now; `legs` whether both
    /// shins lie (which vetoes it) and the thighs' turn-out, asked only when it would start. The turn-out (clamped to
    /// 0.35) becomes the legs' twist rest offset.
    pub fn crouch_start(&mut self, h: f32, leg: f32, straight: f32, hip: f32, legs: impl FnOnce() -> (bool, f32)) {
        if std::mem::replace(&mut self.started, true) || !self.p.crouch {
            return;
        }
        let c = (straight - h / (leg - 0.5 * FOOT_HEIGHT)) * 0.79235;
        if c > self.p.crouch_bend_thresh && c < self.p.crouch_bend_max {
            let (lying, turn_out) = legs();
            if lying {
                return;
            }
            self.crouching = Some((-c.min(0.5), hip.clamp(-1.5, 1.5), self.p.crouch_time));
            crate::extras_core_rdr2::trace_once("crouch depth", || format!("crouch start: depth {c:.3}, between {} and {}", self.p.crouch_bend_thresh, self.p.crouch_bend_max));
            let t = turn_out.clamp(0.0, TURN_OUT_MAX);
            if t > 0.0 {
                self.twist_rest = t;
            }
        }
    }

    /// Crouch start's veto: both shins (knee to ankle joint) further than acos 0.2 off `up`, lying or kneeling.
    pub fn legs_lying(up: bevy_math::Vec3, shins: [(bevy_math::Vec3, bevy_math::Vec3); 2]) -> bool {
        shins.iter().all(|&(knee, ankle)| (knee - ankle).normalize_or_zero().dot(up) < LYING_SHIN)
    }

    /// The thighs' mean angle about the pelvis's long axis `a`, from its side axis `b`, less a right angle: each
    /// thigh's side axis `t[i].0` (mirrored on the right; its third axis when the side axis lies within acos 0.95 of
    /// `a`) taken off `a`, then its angle to `b`.
    pub fn turn_out(a: bevy_math::Vec3, b: bevy_math::Vec3, t: [(bevy_math::Vec3, bevy_math::Vec3); 2]) -> f32 {
        let angle = |x: bevy_math::Vec3| crate::nmdbm::acos((x - a * x.dot(a)).dot(b));
        let (l1, l2) = t[0];
        let l = if l1.dot(a).abs() > 0.95 { -l2 } else { l1 };
        let (r1, r2) = t[1];
        let r = if a.dot(-r1).abs() > 0.95 { r2 } else { -r1 };
        (angle(l) + angle(r)) * 0.5 - std::f32::consts::FRAC_PI_2
    }

    /// The legsApartMax pull: both feet down and further apart than `max`, each foot moves toward the predicted COM
    /// `p` (at its own height) by the excess times the other foot's share of their distances from it.
    pub fn pull_in(a: &mut bevy_math::Vec3, b: &mut bevy_math::Vec3, max: f32, p: bevy_math::Vec3, up: bevy_math::Vec3) {
        let d = (*b - *a).length();
        if d <= max {
            return;
        }
        let (pa, pb) = (p + up * up.dot(*a - p), p + up * up.dot(*b - p));
        let (da, db) = ((pa - *a).length().max(0.002), (pb - *b).length().max(0.002));
        let e = (d - max) / (da + db);
        *a += (pa - *a).normalize_or_zero() * (e * db);
        *b += (pb - *b).normalize_or_zero() * (e * da);
    }

    /// braceForImpact's car (bal+0x346): the predicted COM `q`, on the line from the COM `p`, clipped to the car box
    /// (world x, y). Both faces crossed ahead of the COM: the nearer if q is past it; the COM inside the box: the
    /// nearest way out, even behind it; otherwise q stays.
    pub fn clip_car(&self, p: bevy_math::Vec3, q: bevy_math::Vec3) -> bevy_math::Vec3 {
        let Some(c) = self.car else { return q };
        let cross = |a: bevy_math::Vec2, b: bevy_math::Vec2| a.x * b.y - a.y * b.x;
        let d = (q - p).truncate();
        let mut hit = [(0.0f32, q); 2];
        let mut n = 0;
        for i in 0..4 {
            let (v, e) = (c[i].truncate(), (c[(i + 1) % 4] - c[i]).truncate());
            let den = cross(e, d);
            if den.abs() <= 1e-6 {
                continue;
            }
            let w = v - p.truncate();
            let (s, t) = (cross(d, w) / den, cross(e, w) / den);
            if s > 0.0 && s <= 1.0 {
                hit[n] = (t, (v + e * s).extend(q.z));
                n += 1;
                if n == 2 {
                    break;
                }
            }
        }
        if n < 2 {
            return q;
        }
        let [(t0, x0), (t1, x1)] = hit;
        if t0 > 0.0 && t1 > 0.0 {
            if t0 < 1.0 || t1 < 1.0 { if t0 < t1 { x0 } else { x1 } } else { q }
        } else if (t0 > 0.0 && t1 < 0.0) || (t0 < 0.0 && t1 > 0.0) {
            if t0.abs() < t1.abs() { x0 } else { x1 }
        } else {
            q
        }
    }

    /// The car box's corners for the balancer, round it: `centre`, the car's across / along axes and half extents.
    pub fn car_box(centre: bevy_math::Vec3, ux: bevy_math::Vec3, uy: bevy_math::Vec3, half: bevy_math::Vec2) -> [bevy_math::Vec3; 4] {
        let (x, y) = (ux * (half.x + CAR_MARGIN), uy * (half.y + CAR_MARGIN));
        [centre + x + y, centre - x + y, centre - x - y, centre + x - y]
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
        let r = if self.on_slope { self.p.legs_apart_restep.max(SLOPE_RESTEP) } else { self.p.legs_apart_restep };
        if self.stagger { r + STAGGER_RESTEP } else { r }
    }

    /// May a new step start now? stepWait holds the next one back when he's slow.
    pub fn may_step(&self) -> bool {
        // a turn step waits for stepTurnWait (0x2ab32bc)
        if self.turning && self.p.step_turn_wait > 0.0 && self.since_step <= self.p.step_turn_wait {
            crate::extras_core_rdr2::trace_once("stepTurnWait", || format!("{:.2} of {}", self.since_step, self.p.step_turn_wait));
            return false;
        }
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
            hi + (lo - hi) * (0.5 - self.p.slope_pelvis_blend * (0.5 - t))
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
