#![allow(clippy::all)]
use crate::collision::Collision;
use crate::nm::Agent;
use crate::prelude::*;
use crate::ragdoll::Build;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Kind {
    Float,
    Int,
    Bool,
    Vec3,
    Str,
}

#[derive(Clone, Debug, PartialEq)]
pub enum Value {
    Float(f32),
    Int(i32),
    Bool(bool),
    Vec3(Vec3),
    Str(String),
}

impl Value {
    pub(crate) fn f(&self) -> Result<f32, Error> {
        match self {
            Value::Float(x) => Ok(*x),
            Value::Int(i) => Ok(*i as f32),
            _ => Err(Error::Type),
        }
    }
    pub(crate) fn i(&self) -> Result<i32, Error> {
        match self {
            Value::Int(i) => Ok(*i),
            Value::Float(x) if x.fract() == 0.0 => Ok(*x as i32),
            Value::Bool(b) => Ok(*b as i32),
            _ => Err(Error::Type),
        }
    }
    pub(crate) fn b(&self) -> Result<bool, Error> {
        match self {
            Value::Bool(b) => Ok(*b),
            Value::Int(i) => Ok(*i != 0),
            _ => Err(Error::Type),
        }
    }
    pub(crate) fn v(&self) -> Result<Vec3, Error> {
        match self {
            Value::Vec3(v) => Ok(*v),
            _ => Err(Error::Type),
        }
    }
    pub(crate) fn s(&self) -> Result<&str, Error> {
        match self {
            Value::Str(s) => Ok(s),
            _ => Err(Error::Type),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    Message,
    Param,
    Type,
}

pub struct Param {
    pub name: &'static str,
    pub kind: Kind,
    pub used: bool,
    pub doc: &'static str,
}

pub struct Message {
    pub name: &'static str,
    pub params: &'static [Param],
}

pub static MESSAGES: [Message; 30] = [
    Message { name: "activePose", params: &[
        Param { name: "mask", kind: Kind::Str, used: true, doc: "" },
    ] },
    Message { name: "applyImpulse", params: &[
        Param { name: "partIndex", kind: Kind::Int, used: true, doc: "" },
        Param { name: "hitPoint", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "impulse", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "equalizeAmount", kind: Kind::Float, used: true, doc: "" },
    ] },
    Message { name: "armsWindmillAdaptive", params: &[
        Param { name: "bodyStiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "amplitude", kind: Kind::Float, used: true, doc: "" },
        Param { name: "phase", kind: Kind::Float, used: true, doc: "" },
        Param { name: "disableOnImpact", kind: Kind::Bool, used: true, doc: "+144 0x40: an arm stops once its hand hits something (init's true; the game never sends it)." },
        Param { name: "period", kind: Kind::Float, used: true, doc: "" },
        Param { name: "armStiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "angSpeed", kind: Kind::Float, used: true, doc: "RDR2 core only: the phase's speed, rad/s (6.28)" },
        Param { name: "claviclescale", kind: Kind::Float, used: true, doc: "RDR2 core only: the clavicles' swing, of amplitude (1)" },
        Param { name: "claviclebias", kind: Kind::Float, used: true, doc: "RDR2 core only: the clavicles' lean2 offset (0.15)" },
        Param { name: "leftElbowAngle", kind: Kind::Float, used: true, doc: "RDR2 core only: the left elbow drawn to this, rad; below -0.01 off (-1)" },
        Param { name: "rightElbowAngle", kind: Kind::Float, used: true, doc: "RDR2 core only: the right elbow likewise (-1)" },
        Param { name: "lean1mult", kind: Kind::Float, used: true, doc: "RDR2 core only: the shoulders' lean1 swing, of amplitude (1)" },
        Param { name: "lean1offset", kind: Kind::Float, used: true, doc: "RDR2 core only: added to the shoulders' lean1 (0)" },
        Param { name: "lean2mult", kind: Kind::Float, used: true, doc: "RDR2 core only: the shoulders' lean2 swing, of amplitude (1)" },
        Param { name: "lean2offset", kind: Kind::Float, used: true, doc: "RDR2 core only: added to the shoulders' lean2 (0)" },
        Param { name: "elbowRate", kind: Kind::Float, used: true, doc: "RDR2 core only: how fast the elbows go to their angle (1)" },
        Param { name: "swayamount", kind: Kind::Float, used: true, doc: "RDR2 core only: with setBackAngles, the spine's lean2 sway (0)" },
        Param { name: "angSpeedMultiplier4Dragging", kind: Kind::Float, used: true, doc: "RDR2 core only: with scrabble, phase speed per m/s of the trunk (1)" },
        Param { name: "maxangspeed", kind: Kind::Float, used: true, doc: "RDR2 core only: with scrabble, the phase's top speed, rad/s (4)" },
        Param { name: "scrabblebendupper", kind: Kind::Float, used: true, doc: "RDR2 core only: with scrabble, the elbows bend past this phase, rad (4)" },
        Param { name: "scrabblebendlower", kind: Kind::Float, used: true, doc: "RDR2 core only: and before this one (1.57)" },
        Param { name: "scrabblebend", kind: Kind::Float, used: true, doc: "RDR2 core only: the elbows then (2.2)" },
        Param { name: "scrabbleextend", kind: Kind::Float, used: true, doc: "RDR2 core only: the elbows otherwise (0)" },
        Param { name: "armDirection", kind: Kind::Int, used: true, doc: "RDR2 core only: the way the arms turn, 0 from his spin (0)" },
        Param { name: "mask", kind: Kind::Str, used: true, doc: "RDR2 core only: the joints it drives (ub)" },
        Param { name: "setBackAngles", kind: Kind::Bool, used: true, doc: "RDR2 core only: the spine and neck twist with the arms (true)" },
        Param { name: "useAngMom", kind: Kind::Bool, used: true, doc: "RDR2 core only: the way from his angular momentum and how far his back faces up (false)" },
        Param { name: "bendLeftElbow", kind: Kind::Bool, used: true, doc: "RDR2 core only: the left elbow bends on the forward half (false)" },
        Param { name: "bendRightElbow", kind: Kind::Bool, used: true, doc: "RDR2 core only: the right elbow likewise (false)" },
        Param { name: "scrabble", kind: Kind::Bool, used: true, doc: "RDR2 core only: the phase from how fast his trunk moves, the elbows bent in a window of it (false)" },
    ] },
    Message { name: "bodyBalance", params: &[
        Param { name: "armStiffness", kind: Kind::Float, used: true, doc: "RDR2's name for what stiffens the arms and the neck (9)" },
        Param { name: "armDamping", kind: Kind::Float, used: true, doc: "RDR2's name for the arms' damping (0.7)" },
        Param { name: "bodyStiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "headLookAtVelProb", kind: Kind::Float, used: true, doc: "" },
        Param { name: "armsOutStraightenElbows", kind: Kind::Float, used: true, doc: "" },
        Param { name: "bendElbowsTime", kind: Kind::Float, used: true, doc: "" },
        Param { name: "elbow", kind: Kind::Float, used: true, doc: "" },
        Param { name: "angVelMultiplier", kind: Kind::Vec3, used: true, doc: "angVelMultiplier and angVelThreshold, x y z." },
        Param { name: "angVelThreshold", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "returningToBalanceArmsOut", kind: Kind::Float, used: true, doc: "" },
        Param { name: "somersaultAngleThreshold", kind: Kind::Float, used: true, doc: "" },
        Param { name: "armsOutMinLean2", kind: Kind::Float, used: true, doc: "" },
        Param { name: "sideSomersaultAngleThreshold", kind: Kind::Float, used: true, doc: "" },
        Param { name: "bendElbowsGait", kind: Kind::Float, used: true, doc: "" },
        Param { name: "armsOutOnPush", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "elbowAngleOnContact", kind: Kind::Float, used: true, doc: "" },
        Param { name: "bodyTurnTimeout", kind: Kind::Float, used: true, doc: "The spine's damping (RDR2 calls it spineDamping): GTA IV's name for it." },
        Param { name: "turnOffProb", kind: Kind::Float, used: true, doc: "" },
        Param { name: "turn2VelProb", kind: Kind::Float, used: true, doc: "" },
        Param { name: "damping", kind: Kind::Float, used: true, doc: "" },
        Param { name: "headLookPos", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "sideSomersaultAngle", kind: Kind::Float, used: true, doc: "" },
        Param { name: "headLookAlwaysTimeout", kind: Kind::Float, used: true, doc: "The spine's stiffness (RDR2 calls it spineStiffness): GTA IV's name for it." },
        Param { name: "turnAwayProb", kind: Kind::Float, used: true, doc: "" },
        Param { name: "shoulder", kind: Kind::Float, used: true, doc: "" },
        Param { name: "turnRightProb", kind: Kind::Float, used: true, doc: "" },
        Param { name: "turnLeftProb", kind: Kind::Float, used: true, doc: "" },
        Param { name: "somersaultAngle", kind: Kind::Float, used: true, doc: "" },
        Param { name: "armsOutOnPushMultiplier", kind: Kind::Float, used: true, doc: "" },
        Param { name: "turn2TargetProb", kind: Kind::Float, used: true, doc: "" },
        Param { name: "useBodyTurn", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "useHeadLook", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "headLookInstanceIndex", kind: Kind::Int, used: true, doc: "" },
        Param { name: "armsOutOnPushTimeout", kind: Kind::Float, used: true, doc: "" },
        Param { name: "maxSteps", kind: Kind::Int, used: true, doc: "" },
        Param { name: "spineStiffness", kind: Kind::Float, used: true, doc: "RDR2's name for headLookAlwaysTimeout (the same number)" },
        Param { name: "spineDamping", kind: Kind::Float, used: true, doc: "RDR2's name for bodyTurnTimeout (the same number)" },
        Param { name: "shoulderL1", kind: Kind::Float, used: true, doc: "RDR2 core only: shoulder lean1 offset (0)" },
        Param { name: "shoulderL2", kind: Kind::Float, used: true, doc: "RDR2 core only: shoulder lean2 offset (0.5)" },
        Param { name: "hipL2ArmL2", kind: Kind::Float, used: true, doc: "RDR2 core only: shoulder lean2 per hip lean2 (0.3)" },
        Param { name: "shoulderTwist", kind: Kind::Float, used: true, doc: "RDR2 core only (-0.35)" },
        Param { name: "airbornearms", kind: Kind::Float, used: true, doc: "RDR2 core only: the arms swing within this long (s) without a foot down (0)" },
        Param { name: "axisweightssimulationblocked", kind: Kind::Bool, used: true, doc: "RDR2 core only: arms out ignores each arm's on/off windows (true)" },
        Param { name: "moveRadius", kind: Kind::Float, used: true, doc: "RDR2 core only: lean away from a target moving past closer than this (-1: off)" },
        Param { name: "turnWithBumpRadius", kind: Kind::Float, used: true, doc: "RDR2 core: +0xf8: a target moving past inside this turns him aside (-1: off)" },
        Param { name: "bumpweaknesstime", kind: Kind::Float, used: true, doc: "RDR2 core: +0xfc: s from the start the upper body goes weak, unless hit (0)" },
        Param { name: "bumpweakness", kind: Kind::Float, used: true, doc: "RDR2 core: +0x100: how weak (0)" },
        Param { name: "braceDistance", kind: Kind::Float, used: true, doc: "RDR2 core: +0x104: the target this near, the arms brace (-1: off)" },
        Param { name: "timeToBackwardsBrace", kind: Kind::Float, used: true, doc: "RDR2 core: +0x110: s braced before a target behind him counts (0.5)" },
        Param { name: "braceOffset", kind: Kind::Float, used: true, doc: "RDR2 core: +0x124: the target raised by this for the brace and the moves (0)" },
        Param { name: "leanamountstill", kind: Kind::Float, used: true, doc: "RDR2 core: +0x134: the COM lean of a target moving past, him still (0.05)" },
        Param { name: "?01d67b58", kind: Kind::Bool, used: true, doc: "RDR2 core: +0x139: turning to the target asks a turn step of the balancer (false)" },
        Param { name: "gshelp", kind: Kind::Bool, used: true, doc: "RDR2 core: +0x13a: setCharacterCollisions' friction override on, its yaw hold off (false)" },
        Param { name: "fallingReaction", kind: Kind::Int, used: true, doc: "RDR2 core: +0x13c: the balancer gone: 0 catchFall, 1 rollDownStairs, 2 rollDownStairs till the upper body lands (0)" },
        Param { name: "startbalancer", kind: Kind::Bool, used: true, doc: "RDR2 core: +0x140: bodyBalance starts the balancer and reacts to its failure (true)" },
        Param { name: "blendToZeroPose", kind: Kind::Bool, used: true, doc: "RDR2 core: +0xf6: the rest it blends to is the rig's zero pose (false)" },
        Param { name: "moveAmount", kind: Kind::Float, used: true, doc: "RDR2 core only (0.3)" },
        Param { name: "moveamountstill", kind: Kind::Float, used: true, doc: "RDR2 core only: moveAmount when nearly still (0.3)" },
        Param { name: "moveWhenBracing", kind: Kind::Bool, used: true, doc: "RDR2 core only: lean away only while bracing (no bracing here) (false)" },
    ] },
    Message { name: "bodyFoetal", params: &[
        Param { name: "mask", kind: Kind::Str, used: true, doc: "+80: the effectors it may drive (the rest locked while it runs)." },
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "+68 stiffness, +72 damping, +64 asymmetry, +60 the back's random twist, +76 the seed." },
        Param { name: "asymmetry", kind: Kind::Float, used: true, doc: "" },
        Param { name: "randomSeed", kind: Kind::Int, used: true, doc: "" },
        Param { name: "dampingFactor", kind: Kind::Float, used: true, doc: "" },
        Param { name: "backTwist", kind: Kind::Float, used: true, doc: "" },
    ] },
    Message { name: "bodyRelax", params: &[
        Param { name: "mask", kind: Kind::Str, used: true, doc: "" },
        Param { name: "holdPose", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "relaxation", kind: Kind::Float, used: true, doc: "" },
        Param { name: "damping", kind: Kind::Float, used: true, doc: "" },
    ] },
    Message { name: "bodyRollUp", params: &[
        Param { name: "mask", kind: Kind::Str, used: true, doc: "" },
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "legPush", kind: Kind::Float, used: true, doc: "" },
        Param { name: "armReachAmount", kind: Kind::Float, used: true, doc: "" },
        Param { name: "useArmToSlowDown", kind: Kind::Float, used: true, doc: "" },
        Param { name: "asymmetricalLegs", kind: Kind::Float, used: true, doc: "" },
        Param { name: "damping", kind: Kind::Float, used: true, doc: "RDR2 core only: damping ratio everywhere (0.5)" },
        Param { name: "muscleStiffness", kind: Kind::Float, used: true, doc: "RDR2 core only (1)" },
        Param { name: "noRollTimeBeforeSuccess", kind: Kind::Float, used: true, doc: "RDR2 core only: s under rollVelForSuccess before success (0.5)" },
        Param { name: "rollVelForSuccess", kind: Kind::Float, used: true, doc: "RDR2 core only (0.2)" },
        Param { name: "rollVelLinearContribution", kind: Kind::Float, used: true, doc: "RDR2 core only: the linear speed's weight in the roll speed (1)" },
        Param { name: "velocityScale", kind: Kind::Float, used: true, doc: "RDR2 core only: roll speed = (v x lin + |w| - offset) x scale (0.2)" },
        Param { name: "velocityOffset", kind: Kind::Float, used: true, doc: "RDR2 core only (2)" },
    ] },
    Message { name: "bodyWrithe", params: &[
        Param { name: "armStiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "legStiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "legPeriod", kind: Kind::Float, used: true, doc: "" },
        Param { name: "elbowAmplitude", kind: Kind::Float, used: true, doc: "" },
        Param { name: "legAmplitude", kind: Kind::Float, used: true, doc: "" },
        Param { name: "kneeAmplitude", kind: Kind::Float, used: true, doc: "" },
        Param { name: "backDamping", kind: Kind::Float, used: true, doc: "" },
        Param { name: "backPeriod", kind: Kind::Float, used: true, doc: "" },
        Param { name: "rollOverFlag", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "backAmplitude", kind: Kind::Float, used: true, doc: "" },
        Param { name: "backStiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "armDamping", kind: Kind::Float, used: true, doc: "" },
        Param { name: "legDamping", kind: Kind::Float, used: true, doc: "" },
        Param { name: "mask", kind: Kind::Str, used: true, doc: "" },
        Param { name: "armAmplitude", kind: Kind::Float, used: true, doc: "" },
        Param { name: "armPeriod", kind: Kind::Float, used: true, doc: "" },
        Param { name: "blendArms", kind: Kind::Float, used: true, doc: "RDR2 core only: the arms' targets' weight over what else drives them (1)" },
        Param { name: "blendBack", kind: Kind::Float, used: true, doc: "RDR2 core only: the spine's and neck's (1)" },
        Param { name: "blendLegs", kind: Kind::Float, used: true, doc: "RDR2 core only: the legs' (1)" },
        Param { name: "applyStiffness", kind: Kind::Bool, used: true, doc: "RDR2 core only: set the three stiffnesses (true)" },
        Param { name: "onFire", kind: Kind::Bool, used: true, doc: "RDR2 core only: rolls from where his back first lands, fading with time and distance (false)" },
        Param { name: "shoulderLean1", kind: Kind::Float, used: true, doc: "RDR2 core only: on fire, the shoulders' lean1 blended toward this (0.7)" },
        Param { name: "shoulderLean2", kind: Kind::Float, used: true, doc: "RDR2 core only (0.4)" },
        Param { name: "lean1BlendFactor", kind: Kind::Float, used: true, doc: "RDR2 core only (0)" },
        Param { name: "lean2BlendFactor", kind: Kind::Float, used: true, doc: "RDR2 core only (0)" },
        Param { name: "rollTorqueScale", kind: Kind::Float, used: true, doc: "RDR2 core only: the roll over's torque (150)" },
        Param { name: "maxRollOverTime", kind: Kind::Float, used: true, doc: "RDR2 core only: on fire, s (8)" },
        Param { name: "rollOverRadius", kind: Kind::Float, used: true, doc: "RDR2 core only: on fire, m (2)" },
    ] },
    Message { name: "braceForImpact", params: &[
        Param { name: "bodyStiffness", kind: Kind::Float, used: true, doc: "+20 bodyStiffness, +24 pos, +36 instanceIndex, +40 braceDistance, +44 targetPredictionTime, +48 reachAbsorbtionTime, +52 look, +64 grabStrength, +68 grabDistance, +72 grabReachAngle, +76 grabHoldTimer, +80 the car's top speed to grab (hash unnamed), +84 how long it braces at a target behind it (hash unnamed), +88 grabDontLetGo, +92 legStiffness." },
        Param { name: "look", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "grabReachAngle", kind: Kind::Float, used: true, doc: "" },
        Param { name: "grabHoldTimer", kind: Kind::Float, used: true, doc: "" },
        Param { name: "braceDistance", kind: Kind::Float, used: true, doc: "" },
        Param { name: "targetPredictionTime", kind: Kind::Float, used: true, doc: "" },
        Param { name: "grabDistance", kind: Kind::Float, used: true, doc: "" },
        Param { name: "pos", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "reachAbsorbtionTime", kind: Kind::Float, used: true, doc: "" },
        Param { name: "grabDontLetGo", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "legStiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "grabStrength", kind: Kind::Float, used: true, doc: "" },
        Param { name: "instanceIndex", kind: Kind::Int, used: true, doc: "" },
        Param { name: "hash_4f7c0f0b", kind: Kind::Float, used: true, doc: "" },
    ] },
    Message { name: "catchFall", params: &[
        Param { name: "useHeadLook", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "armsStiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "torsoStiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "legsStiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "forwardMaxArmOffset", kind: Kind::Float, used: true, doc: "" },
        Param { name: "zAxisSpinReduction", kind: Kind::Float, used: true, doc: "" },
        Param { name: "backwardsMinArmOffset", kind: Kind::Float, used: true, doc: "" },
    ] },
    Message { name: "configureBalance", params: &[
        Param { name: "ignoredepth", kind: Kind::Float, used: true, doc: "RDR2 core: ctx+0x103c: self contacts shallower than this don't count as touching (-100)" },
        Param { name: "backwardsLeanCutoff", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "stableLinSpeedThresh", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "stableRotSpeedThresh", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "ignoreFailure", kind: Kind::Bool, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "failMustCollide", kind: Kind::Bool, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "useComDirTurnVelThresh", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "hipError", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "hipErrorRun", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "hipErrorMax", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "dampPelvis", kind: Kind::Bool, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "flattenDownSwing", kind: Kind::Bool, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "flatterSwingFeet", kind: Kind::Bool, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "flatterSwingFeetRun", kind: Kind::Bool, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "airborneStep", kind: Kind::Bool, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "runningThresh", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "runningThreshB", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "clavicleCrouchingAccumulated", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "clavicleCrouchingAccumulatedB", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "stepTurnWait", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "dontStepTime", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "leftLegSwingDamping", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "rightLegSwingDamping", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "opposeGravityLegs", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "opposeGravityAnkles", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "footSlipStiffnessRun", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "avoidLeg", kind: Kind::Bool, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "avoidFootWidth", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "avoidFootWidthB", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "avoidFeedback", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "turnLiftFeet", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "turnLiftFeetVariance", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "minLegTwistVariance", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "maxLegTwistVariance", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "slopePelvisBlend", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "staggerFailHeight", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "extraSteps", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "extraTime", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "maxStillTime", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "legStiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "stepHeight", kind: Kind::Float, used: true, doc: "" },
        Param { name: "stepClampScale", kind: Kind::Float, used: true, doc: "" },
        Param { name: "balanceAbortThreshold", kind: Kind::Float, used: true, doc: "" },
        Param { name: "taperKneeStrength", kind: Kind::Bool, used: true, doc: "DBM+0xBFC bit 0x10: on from init; the game never sends it, but highFall clears it for good." },
        Param { name: "maxBalanceTime", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "giveupHeight", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "fallMult", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "stepHeightInc4Step", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "startFlat", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "endFlat", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "slopeMinDot", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "noSlopeTime", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "slopeMoveFoot", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "ankleEquilibrium", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "movingFloor", kind: Kind::Bool, used: true, doc: "RDR2 core only: leans into the floor's acceleration ctx+0xfe0 (test hook RDR2_FLOOR_ACC=x,y,z: nothing moves under him in the room)" },
        Param { name: "leanAcc", kind: Kind::Float, used: true, doc: "RDR2 core only: with movingFloor, COM's lean per g (0)" },
        Param { name: "hipLeanAcc", kind: Kind::Float, used: true, doc: "RDR2 core only: with movingFloor, the hips' lean per g (0.5)" },
        Param { name: "leanAccMax", kind: Kind::Float, used: true, doc: "RDR2 core only: with movingFloor, the most acceleration leaned into (5)" },
        Param { name: "resistAcc", kind: Kind::Float, used: true, doc: "RDR2 core only: with movingFloor, force per mass on all but the legs (0)" },
        Param { name: "resistAccMax", kind: Kind::Float, used: true, doc: "RDR2 core only: with movingFloor, its most acceleration (3)" },
        Param { name: "legsApartRestep", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "legsApartMax", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "legsTogetherRestep", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "stepDecisionThreshold", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "stepWait", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "changeStepTime", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "runVel", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "legStiffnessRun", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "stepHeightRun", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "maxFootSpeed", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "predictionTime", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "predictionTimeVariance", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "stepClampScaleVariance", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "liftFeet", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "liftFeetVariance", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "leanAgainstVelocity", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "extraFeetApart", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "feetApartMin", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "feetApartMax", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "feetApartVel", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "sidewaysCheatForce", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "sidewaysThresh", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "ankleOffset", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "ankleMult", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "minKneeAngle", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "minLegTwist", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "maxLegTwist", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "crouchBendThresh", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "crouchBendMax", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "crouchTime", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "crouchStandRate", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "crouchHipRate", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "startFlatTwist", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "endFlatTwist", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "ankleTwistMult", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "footSlipStiffness", kind: Kind::Float, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "balanceIndefinitely", kind: Kind::Bool, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "ankleBalance", kind: Kind::Bool, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "alwaysStepWithFarthest", kind: Kind::Bool, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "taperKneeStrengthRun", kind: Kind::Bool, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "rampHipPitchOnFail", kind: Kind::Bool, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "standUp", kind: Kind::Bool, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "stepIfInSupport", kind: Kind::Bool, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "fallReduceGravityComp", kind: Kind::Bool, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "crouch", kind: Kind::Bool, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "flatterStaticFeet", kind: Kind::Bool, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "maxSteps", kind: Kind::Int, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "fallType", kind: Kind::Int, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "stopWaddling", kind: Kind::Int, used: true, doc: "RDR2 core only: the balancer's, as RDR2 names it" },
        Param { name: "reset", kind: Kind::Bool, used: true, doc: "RDR2 core only: back to RDR2's defaults first" },
    ] },
    Message { name: "configureBalanceReset", params: &[
    ] },
    Message { name: "defineAttachedObject", params: &[
        Param { name: "partIndex", kind: Kind::Int, used: true, doc: "" },
        Param { name: "worldPos", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "objectMass", kind: Kind::Float, used: true, doc: "" },
    ] },
    Message { name: "fallOverWall", params: &[
        Param { name: "bodyTwist", kind: Kind::Float, used: true, doc: "" },
        Param { name: "useSpecifiedPoint", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "maxDistanceFromPelToHitPoint", kind: Kind::Float, used: true, doc: "" },
        Param { name: "fallOverEdge", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "fallOverPoint", kind: Kind::Vec3, used: true, doc: "+32 fallOverPoint, +48 fallOverEdge, +64 damping, +68 bodyStiffness, +72 magOfForce, +76 the time it pushes (1, no message field), +80 how far it tips before it's done (-0.2, none), +84 maxDistanceFromPelToHitPoint, +88 / +92 the twist's range (bodyTwist x 45 / x 85, clamped 0..1), +96 useSpecifiedPoint." },
        Param { name: "magOfForce", kind: Kind::Float, used: true, doc: "" },
        Param { name: "bodyStiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "damping", kind: Kind::Float, used: true, doc: "" },
        Param { name: "fallOverWallEndA", kind: Kind::Vec3, used: false, doc: "RDR2 core only: one end of the wall's top edge, world (zero)" },
        Param { name: "fallOverWallEndB", kind: Kind::Vec3, used: false, doc: "RDR2 core only: its other end (zero)" },
        Param { name: "wallhandle", kind: Kind::Int, used: false, doc: "RDR2 core only: the wall's instance (0, the world; not read: the ends are in the world here)" },
        Param { name: "forceTimeOut", kind: Kind::Float, used: false, doc: "RDR2 core only: s of going over that the forces push (2)" },
        Param { name: "forceAngleAbort", kind: Kind::Float, used: false, doc: "RDR2 core only: (-0.2; not read by RDR2)" },
        Param { name: "minLegHeight", kind: Kind::Float, used: false, doc: "RDR2 core only: to start, a foot this far from the pelvis, across (0.4)" },
        Param { name: "maxForceDist", kind: Kind::Float, used: false, doc: "RDR2 core only: parts this close to the wall's top are pushed (0.8)" },
        Param { name: "stepExclusionZone", kind: Kind::Float, used: false, doc: "RDR2 core only: balancerCollisionsReaction's (0.5; not modelled)" },
        Param { name: "maxTwist", kind: Kind::Float, used: false, doc: "RDR2 core only: the twist torque stops once he has twisted this far, rad (pi)" },
        Param { name: "moveArms", kind: Kind::Bool, used: false, doc: "RDR2 core only: the arm pose (true)" },
        Param { name: "moveLegs", kind: Kind::Bool, used: false, doc: "RDR2 core only: the legs swing up; no new steps (true)" },
        Param { name: "bendSpine", kind: Kind::Bool, used: false, doc: "RDR2 core only: the spine bends over the wall (true)" },
        Param { name: "leaningAngleThreshold", kind: Kind::Float, used: false, doc: "RDR2 core only: to start, leaning sideways at most this, degrees (180: any)" },
        Param { name: "legtwist", kind: Kind::Float, used: false, doc: "RDR2 core only: the legs' swing apart with his roll (1)" },
        Param { name: "legamp", kind: Kind::Float, used: false, doc: "RDR2 core only: the legs' swing size (1)" },
        Param { name: "zAxisSpinReduction", kind: Kind::Float, used: false, doc: "RDR2 core only: (0)" },
        Param { name: "angleDirWithWallNormal", kind: Kind::Float, used: false, doc: "RDR2 core only: to start, moving at least this close to the wall's way, degrees (180: any)" },
        Param { name: "maxAngVel", kind: Kind::Float, used: false, doc: "RDR2 core only: the forces stop while he turns along the wall faster than this (2; 0: never)" },
        Param { name: "adaptForcesToLowWall", kind: Kind::Bool, used: false, doc: "RDR2 core only: forces weaker away from the wall (false)" },
        Param { name: "maxWallHeight", kind: Kind::Float, used: false, doc: "RDR2 core only: to start, the wall at most this above his feet, m (-1: any)" },
        Param { name: "distanceToSendSuccessMessage", kind: Kind::Float, used: false, doc: "RDR2 core only: success once this far from the wall's top (-1: at once)" },
        Param { name: "rollingBackThr", kind: Kind::Float, used: false, doc: "RDR2 core only: (0.5)" },
        Param { name: "rollingPotential", kind: Kind::Float, used: false, doc: "RDR2 core only: the approach speed to go over, m/s (0.3)" },
    ] },
    Message { name: "grab", params: &[
        Param { name: "instanceIndex", kind: Kind::Int, used: true, doc: "the obstacle (a step's car box, by its id) the points, normals and grips are in the frame of; -1 the world" },
        Param { name: "bodyStiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "armStiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "grabStrength", kind: Kind::Float, used: true, doc: "+132 grabStrength, +136 armStiffness, +140 grabDist, +144 reachAngle, +148 oneSideReachAngle, +152 bodyStiffness, +156 pullUpTime, +160 / +164 pullUpStrengthLeft / Right, +168 grabHoldMaxTimer; +0x124 reachDistance (hash 0x34629c68): how far the arm reaches out when moving slowly; +0x128 the arm's stiffness while held (4, no message field)." },
        Param { name: "useLeft", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "normalR", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "pos3", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "useRight", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "pos1", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "pos2", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "pos4", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "surfaceGrab", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "oneSideReachAngle", kind: Kind::Float, used: true, doc: "" },
        Param { name: "normalL", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "instancePartIndex", kind: Kind::Int, used: true, doc: "+300: from 1, that child of the obstacle's composite (its frame, ) instead of the obstacle's" },
        Param { name: "linegrab", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "grabDist", kind: Kind::Float, used: true, doc: "" },
        Param { name: "dontLetGo", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "justbrace", kind: Kind::Bool, used: true, doc: "+20 justbrace, +21 linegrab, +22 surfaceGrab, +23 useRight, +24 useLeft, +25 dontLetGo." },
        Param { name: "reachAngle", kind: Kind::Float, used: true, doc: "" },
        Param { name: "pullUpStrengthRight", kind: Kind::Float, used: true, doc: "" },
        Param { name: "pullUpTime", kind: Kind::Float, used: true, doc: "" },
        Param { name: "pullUpStrengthLeft", kind: Kind::Float, used: true, doc: "" },
        Param { name: "grabHoldMaxTimer", kind: Kind::Float, used: true, doc: "" },
        Param { name: "reachDistance", kind: Kind::Float, used: true, doc: "" },
        Param { name: "justBrace", kind: Kind::Bool, used: true, doc: "RDR2's name for justbrace" },
        Param { name: "useLineGrab", kind: Kind::Bool, used: true, doc: "RDR2's name for linegrab" },
        Param { name: "grabDistance", kind: Kind::Float, used: true, doc: "RDR2's name for grabDist" },
        Param { name: "maxReachDistance", kind: Kind::Float, used: true, doc: "RDR2's name for reachDistance" },
        Param { name: "dropWeaponIfNecessary", kind: Kind::Bool, used: true, doc: "RDR2 core only: a busy hand may still grab (false)" },
        Param { name: "dropWeaponDistance", kind: Kind::Float, used: true, doc: "RDR2 core only: asks to drop it this close (0.3)" },
        Param { name: "stickyHands", kind: Kind::Float, used: true, doc: "RDR2 core only: the held hand's pull (4)" },
        Param { name: "turnToTarget", kind: Kind::Int, used: true, doc: "RDR2 core only: the balancer's heading, 0 none, 1 toward, 2 away (1)" },
        Param { name: "handsCollide", kind: Kind::Bool, used: true, doc: "RDR2 core only: the hands collide while reaching (false)" },
        Param { name: "pointsX4grab", kind: Kind::Bool, used: true, doc: "RDR2 core only: pos3/pos4 a second pair, taken when nearer the chest (false)" },
        Param { name: "normalL2", kind: Kind::Vec3, used: true, doc: "RDR2 core only: the second pair's left normal (zero: facing the shoulder)" },
        Param { name: "normalR2", kind: Kind::Vec3, used: true, doc: "RDR2 core only: and its right (zero)" },
        Param { name: "orientationConstraintScale", kind: Kind::Float, used: true, doc: "RDR2 core only: the held wrist's turn torque (1)" },
        Param { name: "maxWristAngle", kind: Kind::Float, used: true, doc: "RDR2 core only: a hand lets go past this wrist angle, rad (pi)" },
        Param { name: "useHeadLookToTarget", kind: Kind::Bool, used: true, doc: "RDR2 core only: with a hand held, look and twist to targetForHeadLook and pull up on one hand (false)" },
        Param { name: "lookAtGrab", kind: Kind::Bool, used: true, doc: "RDR2 core only: headLook at the grab (true)" },
        Param { name: "targetForHeadLook", kind: Kind::Vec3, used: true, doc: "RDR2 core only (zero)" },
        Param { name: "move2Radius", kind: Kind::Float, used: false, doc: "RDR2 core only: (0; not read by RDR2)" },
    ] },
    Message { name: "headLook", params: &[
        Param { name: "keepHeadAwayFromGround", kind: Kind::Bool, used: true, doc: "RDR2 core: lean the neck and head away from the ground instead of looking" },
        Param { name: "twistSpine", kind: Kind::Bool, used: true, doc: "RDR2 core: the leftover twist spreads to two spine joints (true)" },
        Param { name: "instanceIndex", kind: Kind::Int, used: false, doc: "a game object index (the SDK has no level objects: positions are world coordinates)" },
        Param { name: "pos", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "alwaysEyesHorizontal", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "alwaysLook", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "vel", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "damping", kind: Kind::Float, used: true, doc: "" },
    ] },
    Message { name: "highFall", params: &[
        Param { name: "bodyStiffness", kind: Kind::Float, used: true, doc: "+20 bodyStiffness, +24 bodydamping, +28 catchfalltime, +32 crashOrLandCutOff (landCutOffVelocity lands here too), +36 pdStrength, +40 catchFallCutOff, +44 armsUp, +48 legRadius, +52 legAngSpeed, +56 armPeriod, +60 armAmplitude, +64 aimAngleBase, +68 fowardVelRotation, +72 footVelCompScale, +76 legL, +80 sideD, +84 fowardOffsetOfLegIK, +88 fowardVelCutOff, +92 legStrength, +96 orientateBodyToFallDirection, +97 fowardRoll, +98 the zero pose rolling (hash 0xf411b4e7, unnamed), +99 ignorWorldCollisions. (useZeroPose is in the handler's table but stored nowhere.)" },
        Param { name: "armAmplitude", kind: Kind::Float, used: true, doc: "" },
        Param { name: "armPeriod", kind: Kind::Float, used: true, doc: "" },
        Param { name: "pdStrength", kind: Kind::Float, used: true, doc: "" },
        Param { name: "useZeroPose", kind: Kind::Bool, used: false, doc: "the game handler stores it nowhere" },
        Param { name: "armsUp", kind: Kind::Float, used: true, doc: "" },
        Param { name: "legRadius", kind: Kind::Float, used: true, doc: "" },
        Param { name: "bodydamping", kind: Kind::Float, used: true, doc: "" },
        Param { name: "legAngSpeed", kind: Kind::Float, used: true, doc: "" },
        Param { name: "crashOrLandCutOff", kind: Kind::Float, used: true, doc: "" },
        Param { name: "fowardOffsetOfLegIK", kind: Kind::Float, used: true, doc: "" },
        Param { name: "catchfalltime", kind: Kind::Float, used: true, doc: "" },
        Param { name: "landCutOffVelocity", kind: Kind::Float, used: true, doc: "" },
        Param { name: "ignorWorldCollisions", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "fowardRoll", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "orientateBodyToFallDirection", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "aimAngleBase", kind: Kind::Float, used: true, doc: "" },
        Param { name: "fowardVelRotation", kind: Kind::Float, used: true, doc: "" },
        Param { name: "sideD", kind: Kind::Float, used: true, doc: "" },
        Param { name: "catchFallCutOff", kind: Kind::Float, used: true, doc: "" },
        Param { name: "footVelCompScale", kind: Kind::Float, used: true, doc: "" },
        Param { name: "legL", kind: Kind::Float, used: true, doc: "" },
        Param { name: "fowardVelCutOff", kind: Kind::Float, used: true, doc: "" },
        Param { name: "legStrength", kind: Kind::Float, used: true, doc: "" },
        Param { name: "zeroPoseRoll", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "pdDamping", kind: Kind::Float, used: false, doc: "RDR2 core only: the orientation PD's damping (1)" },
        Param { name: "armAngSpeed", kind: Kind::Float, used: false, doc: "RDR2 core only: the windmill's speed, rad/s (7.85)" },
        Param { name: "armPhase", kind: Kind::Float, used: false, doc: "RDR2 core only: the windmill's phase (3.1)" },
        Param { name: "armBendElbows", kind: Kind::Bool, used: false, doc: "RDR2 core only: (true; not done: the windmill has no elbow bend)" },
        Param { name: "legAsymmetry", kind: Kind::Float, used: false, doc: "RDR2 core only: pedalLegs speedAsymmetry (4)" },
        Param { name: "arms2LegsPhase", kind: Kind::Float, used: false, doc: "RDR2 core only: the windmill's phase offset from the legs (0)" },
        Param { name: "arms2LegsSync", kind: Kind::Int, used: false, doc: "RDR2 core only: 1: that offset every tick, 2: at the start (1)" },
        Param { name: "startplayerset", kind: Kind::Float, used: false, doc: "RDR2 core only: (not read by RDR2)" },
        Param { name: "balance", kind: Kind::Bool, used: false, doc: "RDR2 core only: bodyBalance when he lands on his feet (true)" },
        Param { name: "alanRickman", kind: Kind::Bool, used: false, doc: "RDR2 core only: the orientation PD keeps him along where he started (false)" },
        Param { name: "orientateTwist", kind: Kind::Bool, used: false, doc: "RDR2 core only: the orientation PD turns him about up too (true)" },
        Param { name: "orientateMax", kind: Kind::Float, used: false, doc: "RDR2 core only: the orientation PD's torque clamp (300)" },
        Param { name: "forwardRoll", kind: Kind::Bool, used: false, doc: "RDR2 core only: RDR2's name of fowardRoll (false)" },
        Param { name: "forwardOffsetOfLegIK", kind: Kind::Float, used: false, doc: "RDR2 core only: RDR2's unnamed pedalLegs centreForwards in the brace, GTA IV's fowardOffsetOfLegIK (0)" },
        Param { name: "adaptiveCircling", kind: Kind::Bool, used: false, doc: "RDR2 core only: the legs pedal backwards when he pitches forward (true)" },
        Param { name: "hula", kind: Kind::Bool, used: false, doc: "RDR2 core only: the legs circle wide when he falls face up (true)" },
        Param { name: "minSpeedForBrace", kind: Kind::Float, used: false, doc: "RDR2 core only: above this sideways speed the unrecoverable fall catches itself, m/s (10)" },
        Param { name: "maxSpeedForRecoverableFall", kind: Kind::Float, used: false, doc: "RDR2 core only: faster than this at impact he can't recover, m/s (15)" },
        Param { name: "landingNormal", kind: Kind::Float, used: false, doc: "RDR2 core only: ground flatter than this (normal.up) counts as a landing (0.6)" },
    ] },
    Message { name: "incomingTransforms", params: &[
    ] },
    Message { name: "leanInDirection", params: &[
        Param { name: "leanAmount", kind: Kind::Float, used: true, doc: "" },
        Param { name: "applyAsForce", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "dir", kind: Kind::Vec3, used: true, doc: "" },
    ] },
    Message { name: "leanToPosition", params: &[
        Param { name: "leanAmount", kind: Kind::Float, used: true, doc: "" },
        Param { name: "applyAsForce", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "pos", kind: Kind::Vec3, used: true, doc: "" },
    ] },
    Message { name: "leanTowardsObject", params: &[
        Param { name: "instanceIndex", kind: Kind::Int, used: false, doc: "a game object index (the SDK has no level objects: positions are world coordinates)" },
        Param { name: "leanAmount", kind: Kind::Float, used: true, doc: "" },
        Param { name: "applyAsForce", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "offset", kind: Kind::Vec3, used: true, doc: "" },
    ] },
    Message { name: "pedalLegs", params: &[
        Param { name: "legStiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "pedalRightLeg", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "speedAsymmetry", kind: Kind::Float, used: true, doc: "" },
        Param { name: "pedalLeftLeg", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "radius", kind: Kind::Float, used: true, doc: "" },
        Param { name: "angularSpeed", kind: Kind::Float, used: true, doc: "" },
        Param { name: "pedalOffset", kind: Kind::Float, used: true, doc: "" },
        Param { name: "angSpeedMultiplier4Dragging", kind: Kind::Float, used: true, doc: "" },
        Param { name: "adaptivePedal4Dragging", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "backPedal", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "radiusVariance", kind: Kind::Float, used: true, doc: "" },
        Param { name: "centreUp", kind: Kind::Float, used: true, doc: "" },
        Param { name: "centreForwards", kind: Kind::Float, used: true, doc: "" },
        Param { name: "legAngleVariance", kind: Kind::Float, used: true, doc: "" },
        Param { name: "randomSeed", kind: Kind::Int, used: true, doc: "" },
        Param { name: "dragReduction", kind: Kind::Float, used: false, doc: "RDR2 core only: the IK's drag reduction (0.25; Delirium's leg IK has none)" },
        Param { name: "centresideways", kind: Kind::Float, used: true, doc: "RDR2 core only: the circles' centre moved sideways, m (0)" },
        Param { name: "ellipse", kind: Kind::Float, used: true, doc: "RDR2 core only: the circle's height over its length, -1..1; negative: its length over its height (1)" },
        Param { name: "spread", kind: Kind::Float, used: true, doc: "RDR2 core only: the feet's circles apart sideways, m (0)" },
        Param { name: "hula", kind: Kind::Bool, used: true, doc: "RDR2 core only: level circles at the hips' height, the upper body swaying against them (false)" },
    ] },
    Message { name: "pointArm", params: &[
        Param { name: "instanceIndex", kind: Kind::Int, used: true, doc: "" },
        Param { name: "useLeftArm", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "twist", kind: Kind::Float, used: true, doc: "" },
        Param { name: "target", kind: Kind::Vec3, used: true, doc: "+32 target, +48 twist, +52 armStraightness, +56 armStiffness, +60 armDamping, +64 instanceIndex, +68 pointSwingLimit, +72 useLeftArm, +73 useZeroPoseWhenNotPointing." },
        Param { name: "pointSwingLimit", kind: Kind::Float, used: true, doc: "" },
        Param { name: "armStraightness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "useZeroPoseWhenNotPointing", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "armDamping", kind: Kind::Float, used: true, doc: "" },
        Param { name: "armStiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "useRightArm", kind: Kind::Bool, used: true, doc: "RDR2 core only: point the right arm (under the core useLeftArm points the left; both may) (false)" },
        Param { name: "targetLeft", kind: Kind::Vec3, used: true, doc: "RDR2 core only: where the left arm points, world (zero)" },
        Param { name: "targetRight", kind: Kind::Vec3, used: true, doc: "RDR2 core only: where the right arm points, world (zero)" },
        Param { name: "twistLeft", kind: Kind::Float, used: true, doc: "RDR2 core only (0.3)" },
        Param { name: "twistRight", kind: Kind::Float, used: true, doc: "RDR2 core only (0.3)" },
        Param { name: "armStraightnessLeft", kind: Kind::Float, used: true, doc: "RDR2 core only: the hand 0.7 m times this from the shoulder (0.8)" },
        Param { name: "armStraightnessRight", kind: Kind::Float, used: true, doc: "RDR2 core only (0.8)" },
        Param { name: "armStiffnessLeft", kind: Kind::Float, used: true, doc: "RDR2 core only (15)" },
        Param { name: "armStiffnessRight", kind: Kind::Float, used: true, doc: "RDR2 core only (15)" },
        Param { name: "armDampingLeft", kind: Kind::Float, used: true, doc: "RDR2 core only (1)" },
        Param { name: "armDampingRight", kind: Kind::Float, used: true, doc: "RDR2 core only (1)" },
        Param { name: "pointSwingLimitLeft", kind: Kind::Float, used: true, doc: "RDR2 core only: past this clavicle swing (of its range) the arm stops pointing (1.5)" },
        Param { name: "pointSwingLimitRight", kind: Kind::Float, used: true, doc: "RDR2 core only (1.5)" },
        Param { name: "instanceHandleLeft", kind: Kind::Int, used: false, doc: "RDR2 core only: a game object the target is relative to (the SDK has no level objects: world coordinates)" },
        Param { name: "instanceHandleRight", kind: Kind::Int, used: false, doc: "RDR2 core only: as instanceHandleLeft" },
        Param { name: "useZeroPoseWhenNotPointingLeft", kind: Kind::Bool, used: true, doc: "RDR2 core only (false)" },
        Param { name: "useZeroPoseWhenNotPointingRight", kind: Kind::Bool, used: true, doc: "RDR2 core only (false)" },
    ] },
    Message { name: "rollDownStairs", params: &[
        Param { name: "forcemag", kind: Kind::Float, used: true, doc: "" },
        Param { name: "armReachLength", kind: Kind::Float, used: true, doc: "" },
        Param { name: "m_legPush", kind: Kind::Float, used: true, doc: "" },
        Param { name: "useCustomRollDir", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "stiffnessDecayTarget", kind: Kind::Float, used: true, doc: "" },
        Param { name: "spinWhenInAir", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "m_armReachAmount", kind: Kind::Float, used: true, doc: "" },
        Param { name: "customRollDir", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "stiffnessDecayTime", kind: Kind::Float, used: true, doc: "" },
        Param { name: "m_useArmToSlowDown", kind: Kind::Float, used: true, doc: "" },
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "The message's: +20 stiffness, +24 damping, +28 forcemag, +32 asymmetry, +36 asymmetricalLegs, +40 m_useArmToSlowDown, +44 m_armReachAmount, +48 useZeroPose, +49 spinWhenInAir, +52 m_legPush, +60 armReachLength, +64 stiffnessDecayTime, +68 stiffnessDecayTarget, +72 useCustomRollDir, +76 zAxisSpinReduction, +80 customRollDir." },
        Param { name: "damping", kind: Kind::Float, used: true, doc: "" },
        Param { name: "useZeroPose", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "zAxisSpinReduction", kind: Kind::Float, used: true, doc: "" },
        Param { name: "asymmetricalLegs", kind: Kind::Float, used: true, doc: "" },
        Param { name: "asymmetry", kind: Kind::Float, used: true, doc: "" },
        Param { name: "muscleStiffness", kind: Kind::Float, used: true, doc: "RDR2 core only (1)" },
        Param { name: "onlyApplyHelperForces", kind: Kind::Bool, used: true, doc: "RDR2 core only: the helper torques without bodyRollUp (false)" },
        Param { name: "useRelativeVelocity", kind: Kind::Bool, used: true, doc: "RDR2 core only: with braceForImpact running, the speed is customRollDir's length (false)" },
        Param { name: "maxAngVelAroundFrontwardAxis", kind: Kind::Float, used: true, doc: "RDR2 core only: spinning faster about the front-back axis turns the cheating and align torques off, rad/s (2)" },
        Param { name: "applyNewRollingCheatingTorques", kind: Kind::Bool, used: true, doc: "RDR2 core only: a fixed roll torque once the legs are high, instead of the roll helper (false)" },
        Param { name: "magOfTorqueToRoll", kind: Kind::Float, used: true, doc: "RDR2 core only (50)" },
        Param { name: "maxAngVel", kind: Kind::Float, used: true, doc: "RDR2 core only: the cheating torque only below this roll speed, rad/s (5)" },
        Param { name: "minAngVel", kind: Kind::Float, used: true, doc: "RDR2 core only: and above this (0.5)" },
        Param { name: "applyHelPerTorqueToAlign", kind: Kind::Bool, used: true, doc: "RDR2 core only: turn the spine along the roll axis (false)" },
        Param { name: "delayToAlignBody", kind: Kind::Float, used: true, doc: "RDR2 core only, s (0.2)" },
        Param { name: "magOfTorqueToAlign", kind: Kind::Float, used: true, doc: "RDR2 core only (50)" },
        Param { name: "airborneReduction", kind: Kind::Float, used: true, doc: "RDR2 core only: the roll torque in the air times 1 minus this, with spinWhenInAir (0.85)" },
        Param { name: "applyFoetalToLegs", kind: Kind::Bool, used: true, doc: "RDR2 core only: fold the legs in once they are high (false)" },
        Param { name: "movementLegsInFoetalPosition", kind: Kind::Float, used: true, doc: "RDR2 core only (1.3)" },
        Param { name: "limitSpinReduction", kind: Kind::Bool, used: true, doc: "RDR2 core only: zAxisSpinReduction only while spinning about the body's up axis (false)" },
        Param { name: "targetLinearVelocity", kind: Kind::Float, used: true, doc: "RDR2 core only: the roll torque acts as if at least this fast, m/s (1)" },
        Param { name: "targetLinearVelocityDecayTime", kind: Kind::Float, used: true, doc: "RDR2 core only: that floor fades to 0 over this, s (0.5)" },
    ] },
    Message { name: "setStiffness", params: &[
        Param { name: "bodyStiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "mask", kind: Kind::Str, used: true, doc: "" },
        Param { name: "damping", kind: Kind::Float, used: true, doc: "" },
    ] },
    Message { name: "shot", params: &[
        Param { name: "looseness4Stagger", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "useCStrModulation", kind: Kind::Bool, used: true, doc: "RDR2 core only" },
        Param { name: "PointGun", kind: Kind::Bool, used: true, doc: "RDR2 core: +0x289: the module that lets pointGun go when he moves backward and takes it up again (false)" },
        Param { name: "cStrUpperMin", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "cStrUpperMax", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "cStrLowerMin", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "cStrLowerMax", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "bulletProofVest", kind: Kind::Bool, used: true, doc: "RDR2 core only" },
        Param { name: "alwaysResetLooseness", kind: Kind::Bool, used: true, doc: "RDR2 core only" },
        Param { name: "alwaysResetNeckLooseness", kind: Kind::Bool, used: true, doc: "RDR2 core only" },
        Param { name: "spineBlendExagCPain", kind: Kind::Bool, used: true, doc: "RDR2 core only" },
        Param { name: "floorWeaknessTime", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "floorStrength", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "floorJointWeaknessTime", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "floorJointStrength", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "angVelScale", kind: Kind::Float, used: true, doc: "RDR2 core: +0xc8: at the shot's start the parts in the mask have their angular velocity times this (1)" },
        Param { name: "angVelScaleMask", kind: Kind::Str, used: true, doc: "RDR2 core: +0xcc: those parts, a mask by name (fb)" },
        Param { name: "looseness4Fall", kind: Kind::Float, used: true, doc: "RDR2 core: +0x128: over 0, the shot's strength scale times this while it falls (0x2a8c4b8) (0)" },
        Param { name: "flingTime", kind: Kind::Float, used: false, doc: "RDR2's fling: its start needs a field only its own enter fills, so it doesn't start" },
        Param { name: "deathTime", kind: Kind::Float, used: false, doc: "RDR2's: gated on two dying flags Delirium doesn't have" },
        Param { name: "spineBlendPose", kind: Kind::Int, used: false, doc: "RDR2's 9de396e2: a pose index for the spine blend, not here" },
        Param { name: "allowInjuredLeg", kind: Kind::Bool, used: false, doc: "RDR2: read by the room when it layers the game's sets" },
        Param { name: "allowInjuredArm", kind: Kind::Bool, used: false, doc: "RDR2: read by the room when it layers the game's sets" },
        Param { name: "exagDuration", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "exagMag", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "exagTwistMag", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "exagUndoRate", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "cpainStart", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "cpainRate", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "cpainDuration", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "cpainMag", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "cpainTwistMag", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "cpainUndoRate", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "armStiffness", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "spineDamping", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "initialNeckStiffness", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "initialNeckDamping", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "neckStiffness", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "neckDamping", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "kMultOnLoose", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "kMult4Legs", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "initialWeaknessRampDuration", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "initialWeaknessZeroDuration", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "initialNeckRampDuration", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "initialNeckDuration", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "spineBlendZero", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "minArmsLooseness", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "minLegsLooseness", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "fallingReaction", kind: Kind::Int, used: true, doc: "RDR2 core only: 0 roll up, 1 catch fall, 4 catch fall then writhe (2, 3 roll up here)" },
        Param { name: "bracedSideSpinMult", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "shockSpinMaxTwistVel", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "shockSpinAirMult", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "shockSpin1FootMult", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "shockSpinScaleByLeverArm", kind: Kind::Bool, used: true, doc: "RDR2 core only" },
        Param { name: "fling2", kind: Kind::Bool, used: true, doc: "RDR2 core only" },
        Param { name: "fling2Left", kind: Kind::Bool, used: true, doc: "RDR2 core only" },
        Param { name: "fling2Right", kind: Kind::Bool, used: true, doc: "RDR2 core only" },
        Param { name: "fling2OverrideStagger", kind: Kind::Bool, used: true, doc: "RDR2 core only" },
        Param { name: "fling2TimeBefore", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "fling2Time", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "fling2MStiffL", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "fling2MStiffR", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "fling2StiffL", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "fling2StiffR", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "fling2DampL", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "fling2DampR", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "fling2RelaxTimeL", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "fling2RelaxTimeR", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "fling2AngleMinL", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "fling2AngleMaxL", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "fling2AngleMinR", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "fling2AngleMaxR", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "fling2LengthMinL", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "fling2LengthMaxL", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "fling2LengthMinR", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "fling2LengthMaxR", kind: Kind::Float, used: true, doc: "RDR2 core only" },
        Param { name: "enableDefaultArms", kind: Kind::Bool, used: true, doc: "RDR2 core only (RDR2's shotConfigureArms): the default arms (true)" },
        Param { name: "brace", kind: Kind::Bool, used: true, doc: "RDR2 core only: free arms reach towards the ground the way he moves, else the guard pose (true)" },
        Param { name: "useArmsWindmill", kind: Kind::Bool, used: true, doc: "RDR2 core only: arms windmill when thrown back fast (true)" },
        Param { name: "alwaysReachTime", kind: Kind::Float, used: true, doc: "RDR2 core only: s a hand holds the wound before releaseWound may let it go (0.3)" },
        Param { name: "AWSpeedMult", kind: Kind::Float, used: true, doc: "RDR2 core only: windmill speed per m/s (1)" },
        Param { name: "AWRadiusMult", kind: Kind::Float, used: true, doc: "RDR2 core only: windmill radius per m/s (1)" },
        Param { name: "AWStiffnessAdd", kind: Kind::Float, used: true, doc: "RDR2 core only: windmill stiffness added with the radius (4)" },
        Param { name: "releaseWound", kind: Kind::Int, used: true, doc: "RDR2 core only: let a reaching hand go when moving fast, 0 never, 1 with brace, 2 always (1)" },
        Param { name: "bodyStiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "grabHoldTime", kind: Kind::Float, used: true, doc: "" },
        Param { name: "bodyPart", kind: Kind::Int, used: true, doc: "" },
        Param { name: "loosenessAmount", kind: Kind::Float, used: true, doc: "" },
        Param { name: "shockSpinDecayMult", kind: Kind::Float, used: true, doc: "" },
        Param { name: "timeBeforeReachForWound", kind: Kind::Float, used: true, doc: "" },
        Param { name: "headLookAtWoundMaxTimer", kind: Kind::Float, used: true, doc: "" },
        Param { name: "reachForWound", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "spinePainTwistMultiplier", kind: Kind::Float, used: true, doc: "" },
        Param { name: "useHeadLook", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "shockSpinMin", kind: Kind::Float, used: true, doc: "" },
        Param { name: "stableHandsAndNeck", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "maxSteps", kind: Kind::Int, used: true, doc: "" },
        Param { name: "shockSpinMax", kind: Kind::Float, used: true, doc: "" },
        Param { name: "timeBeforeCollapseWoundLeg", kind: Kind::Float, used: true, doc: "" },
        Param { name: "melee", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "shockSpinScalePerComponent", kind: Kind::Float, used: true, doc: "" },
        Param { name: "randomizeShockSpinDirection", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "useCatchFallOnFall", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "crouching", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "headLookAtWoundMinTimer", kind: Kind::Float, used: true, doc: "" },
        Param { name: "hitPoint", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "shockSpinLiftForceMult", kind: Kind::Float, used: true, doc: "" },
        Param { name: "chickenArms", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "fling", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "normal", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "spinePainTime", kind: Kind::Float, used: true, doc: "" },
        Param { name: "headLookAtHeadPosMinTimer", kind: Kind::Float, used: true, doc: "" },
        Param { name: "bulletVel", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "addShockSpin", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "spinePainMultiplier", kind: Kind::Float, used: true, doc: "" },
        Param { name: "headLookAtHeadPosMaxTimer", kind: Kind::Float, used: true, doc: "" },
        Param { name: "alwaysAddShockSpin", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "flingWidth", kind: Kind::Float, used: true, doc: "+0x64 flingWidth: fling's clavicle lift." },
        Param { name: "headLook", kind: Kind::Vec3, used: true, doc: "+16 headLook: the point useHeadLook looks at." },
    ] },
    Message { name: "shotRelax", params: &[
        Param { name: "relaxPeriod", kind: Kind::Float, used: true, doc: "" },
        Param { name: "relaxPeriodUpper", kind: Kind::Float, used: true, doc: "RDR2 core: +0x6fc: s the upper body's strength falls over (2)" },
        Param { name: "relaxPeriodLower", kind: Kind::Float, used: true, doc: "RDR2 core: +0x704: s the lower body's falls over (0.4)" },
    ] },
    Message { name: "stayUpright", params: &[
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "dampingScale", kind: Kind::Float, used: true, doc: "" },
        Param { name: "side", kind: Kind::Int, used: true, doc: "RDR2 core: which set, 0 slow or 1 fast (both)" },
        Param { name: "useForces", kind: Kind::Bool, used: true, doc: "RDR2 core: the force (false)" },
        Param { name: "useTorques", kind: Kind::Bool, used: true, doc: "RDR2 core: the righting torque (false)" },
        Param { name: "lastStandMode", kind: Kind::Bool, used: true, doc: "RDR2 core: carried, not applied (false)" },
        Param { name: "velocityBased", kind: Kind::Bool, used: true, doc: "RDR2 core: force and torque scaled by tanh of the COM speed (false)" },
        Param { name: "torqueOnlyInAir", kind: Kind::Bool, used: true, doc: "RDR2 core: the torque only with no foot down (false)" },
        Param { name: "leanRelative", kind: Kind::Bool, used: true, doc: "RDR2 core: the torque's frame turned by his lean (false)" },
        Param { name: "torqueparts", kind: Kind::Bool, used: true, doc: "RDR2 core: the torque spread over every part by inertia (false)" },
        Param { name: "turnTowardsBullets", kind: Kind::Bool, used: true, doc: "RDR2 core: a yaw torque on the trunk toward where the last bullet came from (false)" },
        Param { name: "switchSpeed", kind: Kind::Float, used: true, doc: "RDR2 core: the fast set's: at this horizontal speed it takes over (100)" },
        Param { name: "lastStandSinkRate", kind: Kind::Float, used: true, doc: "RDR2 core: carried, not applied (0.3)" },
        Param { name: "lastStandHorizDamping", kind: Kind::Float, used: true, doc: "RDR2 core: carried, not applied (0.4)" },
        Param { name: "lastStandMaxTime", kind: Kind::Float, used: true, doc: "RDR2 core: carried, not applied (0.4)" },
        Param { name: "forceStrength", kind: Kind::Float, used: true, doc: "RDR2 core: the force's gain (3)" },
        Param { name: "forceDamping", kind: Kind::Float, used: true, doc: "RDR2 core: its damping's share, negative: 0.2 (-1)" },
        Param { name: "forceFeetMult", kind: Kind::Float, used: true, doc: "RDR2 core: the feet's share (1)" },
        Param { name: "forceSpine3Share", kind: Kind::Float, used: true, doc: "RDR2 core: the spine top's part of the trunk's share, the rest on the pelvis (0.3)" },
        Param { name: "forceLeanReduction", kind: Kind::Float, used: true, doc: "RDR2 core: how much his lean takes it off (1)" },
        Param { name: "forceInAirShare", kind: Kind::Float, used: true, doc: "RDR2 core: one foot down, that foot's share (0.5)" },
        Param { name: "forceMin", kind: Kind::Float, used: true, doc: "RDR2 core: unused, as RDR2 passes it (-1)" },
        Param { name: "forceMax", kind: Kind::Float, used: true, doc: "RDR2 core: unused, as RDR2 passes it (-1)" },
        Param { name: "forceSaturationVel", kind: Kind::Float, used: true, doc: "RDR2 core: velocityBased: m/s (4)" },
        Param { name: "forceThresholdVel", kind: Kind::Float, used: true, doc: "RDR2 core: velocityBased: off below this, m/s (0.5)" },
        Param { name: "torqueSaturationVel", kind: Kind::Float, used: true, doc: "RDR2 core: velocityBased: m/s (4)" },
        Param { name: "torqueThresholdVel", kind: Kind::Float, used: true, doc: "RDR2 core: velocityBased: off below this, m/s (2.5)" },
        Param { name: "torqueleanreduction", kind: Kind::Float, used: true, doc: "RDR2 core: how much leaning past leanmin takes the torque off (0)" },
        Param { name: "torqueleanmin", kind: Kind::Float, used: true, doc: "RDR2 core: rad (0.5)" },
        Param { name: "torqueleanmax", kind: Kind::Float, used: true, doc: "RDR2 core: rad (1.1)" },
        Param { name: "torquelean", kind: Kind::Float, used: true, doc: "RDR2 core: carried, not applied (0)" },
        Param { name: "supportPosition", kind: Kind::Float, used: true, doc: "RDR2 core: both feet this far behind the COM, noSupportForceMult applies, m (2)" },
        Param { name: "noSupportForceMult", kind: Kind::Float, used: true, doc: "RDR2 core: (1)" },
        Param { name: "stepUpHelp", kind: Kind::Float, used: true, doc: "RDR2 core: carried, not applied (0)" },
        Param { name: "stayUpAcc", kind: Kind::Float, used: true, doc: "RDR2 core: carried, not applied (0.7)" },
        Param { name: "stayUpAccMax", kind: Kind::Float, used: true, doc: "RDR2 core: carried, not applied (5)" },
        Param { name: "antigrav", kind: Kind::Float, used: true, doc: "RDR2 core: fall assist: the share of his weight pushed up while falling (0)" },
        Param { name: "antigravtime", kind: Kind::Float, used: true, doc: "RDR2 core: fall assist: only while the balancer is younger than this, s (0.5)" },
        Param { name: "antigravspeed", kind: Kind::Float, used: true, doc: "RDR2 core: fall assist: only over this horizontal speed, m/s (1.5)" },
        Param { name: "torqueStrength", kind: Kind::Vec3, used: true, doc: "RDR2 core: the torque's stiffness about the two tilts and yaw (0,0,0)" },
        Param { name: "torqueDamping", kind: Kind::Vec3, used: true, doc: "RDR2 core: its damping (0.5,0.5,0)" },
    ] },
    Message { name: "stopAllBehaviours", params: &[
    ] },
    Message { name: "upperBodyFlinch", params: &[
        Param { name: "handDistanceFrontBack", kind: Kind::Float, used: true, doc: "" },
        Param { name: "noiseScale", kind: Kind::Float, used: true, doc: "" },
        Param { name: "handDistanceVertical", kind: Kind::Float, used: true, doc: "" },
        Param { name: "useRightArm", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "handDistanceLeftRight", kind: Kind::Float, used: true, doc: "" },
        Param { name: "rightHanded", kind: Kind::Bool, used: true, doc: "" },
        Param { name: "turnTowards", kind: Kind::Int, used: true, doc: "" },
        Param { name: "backBendAmount", kind: Kind::Float, used: true, doc: "" },
        Param { name: "newHit", kind: Kind::Bool, used: true, doc: "+68: restart the flinch's timing; set only when sent (else the behaviour's own stands)." },
        Param { name: "leftHanded", kind: Kind::Bool, used: true, doc: "+32 / +33: stored, read by nothing sends one of them, a coin flip; both arms still flinch)." },
        Param { name: "bodyDamping", kind: Kind::Float, used: true, doc: "" },
        Param { name: "useLeftArm", kind: Kind::Bool, used: true, doc: "+61 / +60: the arms that flinch." },
        Param { name: "pos", kind: Kind::Vec3, used: true, doc: "" },
        Param { name: "bodyStiffness", kind: Kind::Float, used: true, doc: "" },
        Param { name: "protectHeadToggle", kind: Kind::Bool, used: true, doc: "RDR2 core only: always turned away, covering the head (false)" },
        Param { name: "applyStiffness", kind: Kind::Bool, used: true, doc: "RDR2 core only: set the upper body's stiffness (true)" },
        Param { name: "dontBraceHead", kind: Kind::Bool, used: true, doc: "RDR2 core only: no head cover when turned away, no fixed first pose (false)" },
        Param { name: "headLookAwayFromTarget", kind: Kind::Bool, used: true, doc: "RDR2 core only: now and then look away to the side (false)" },
        Param { name: "useHeadLook", kind: Kind::Bool, used: true, doc: "RDR2 core only: run headLook (true)" },
    ] },
];

#[derive(Clone, Debug)]
pub struct Impulse {
    pub part_index: i32,
    pub hit_point: Option<Vec3>,
    pub impulse: Vec3,
    pub equalize_amount: f32,
}
impl Default for Impulse {
    fn default() -> Impulse {
        Impulse { part_index: 0, hit_point: None, impulse: Vec3::new(0.0, -delirium_data::kv!(79), 0.0), equalize_amount: 0.0 }
    }
}

#[derive(Clone, Debug)]
pub struct BodyRelax {
    pub mask: Option<String>,
    pub hold_pose: bool,
    pub relaxation: f32,
    pub damping: Option<f32>,
}
impl Default for BodyRelax {
    fn default() -> BodyRelax {
        BodyRelax { mask: None, hold_pose: false, relaxation: delirium_data::kv!(79), damping: None }
    }
}

#[derive(Clone, Debug)]
pub struct Attached {
    pub part_index: i32,
    pub world_pos: Vec3,
    pub object_mass: f32,
}
impl Default for Attached {
    fn default() -> Attached {
        Attached { part_index: -1, world_pos: Vec3::ZERO, object_mass: 0.0 }
    }
}

#[derive(Clone, Debug)]
pub struct HeadLookMsg {
    pub keep_head_away: bool,
    pub twist_spine: bool,
    pub pos: Vec3,
    pub vel: Vec3,
    pub stiffness: f32,
    pub damping: f32,
    pub always_look: bool,
    pub always_eyes_horizontal: bool,
}
impl Default for HeadLookMsg {
    fn default() -> HeadLookMsg {
        HeadLookMsg { keep_head_away: false, twist_spine: true, pos: Vec3::ZERO, vel: Vec3::ZERO, stiffness: delirium_data::kv!(31), damping: 1.0, always_look: false, always_eyes_horizontal: true }
    }
}

#[derive(Clone, Debug)]
pub struct Lean {
    pub lean_amount: f32,
    pub apply_as_force: bool,
    pub dir: Vec3,
}
impl Default for Lean {
    fn default() -> Lean {
        Lean { lean_amount: delirium_data::kv!(60), apply_as_force: false, dir: Vec3::Z }
    }
}

#[derive(Clone, Debug)]
pub struct LeanTo {
    pub lean_amount: f32,
    pub apply_as_force: bool,
    pub pos: Vec3,
}
impl Default for LeanTo {
    fn default() -> LeanTo {
        LeanTo { lean_amount: delirium_data::kv!(60), apply_as_force: false, pos: Vec3::ZERO }
    }
}

#[derive(Clone, Debug)]
pub struct LeanTowards {
    pub lean_amount: f32,
    pub apply_as_force: bool,
    pub offset: Vec3,
}
impl Default for LeanTowards {
    fn default() -> LeanTowards {
        LeanTowards { lean_amount: delirium_data::kv!(60), apply_as_force: false, offset: Vec3::ZERO }
    }
}

#[derive(Clone, Debug)]
pub struct Stiffness {
    pub body_stiffness: f32,
    pub damping: f32,
    pub mask: String,
}
impl Default for Stiffness {
    fn default() -> Stiffness {
        Stiffness { body_stiffness: delirium_data::kv!(21), damping: 1.0, mask: "fb".into() }
    }
}

#[derive(Clone, Debug)]
pub struct ActivePose {
    pub mask: Option<String>,
}
impl Default for ActivePose {
    fn default() -> ActivePose {
        ActivePose { mask: None }
    }
}

#[derive(Clone, Debug)]
pub struct ShotRelax {
    pub relax_period: f32,
    /// RDR2 core: the game's two periods, when the message gives them (below 0: not given)
    pub upper: f32,
    pub lower: f32,
}
impl Default for ShotRelax {
    fn default() -> ShotRelax {
        ShotRelax { relax_period: 2.0, upper: -1.0, lower: -1.0 }
    }
}

#[derive(Clone, Debug)]
pub struct StayUpright {
    pub stiffness: f32,
    pub damping_scale: f32,
    /// RDR2 core: this message's record, the template under its parameters, and the set it fills (both without one)
    pub rdr2: crate::nmdbm_rdr2::StayUpright,
    pub side: Option<i32>,
}
impl Default for StayUpright {
    fn default() -> StayUpright {
        StayUpright { stiffness: delirium_data::kv!(5), damping_scale: 0.5, rdr2: Default::default(), side: None }
    }
}

#[derive(Clone, Default)]
pub struct State {
    pub active_pose: ActivePose,
    pub apply_impulse: Impulse,
    pub arms_windmill_adaptive: crate::nm::ArmsWindmill,
    pub body_balance: crate::nmbb::Params,
    pub body_foetal: crate::nmbf::Foetal,
    pub body_relax: BodyRelax,
    pub body_roll_up: crate::nm::BodyRollUp,
    pub body_writhe: crate::nm::BodyWrithe,
    pub brace_for_impact: crate::nmbr::BraceParams,
    pub catch_fall: crate::nmcf::Params,
    pub configure_balance: crate::nm::ConfigureBalance,
    pub define_attached_object: Attached,
    pub fall_over_wall: crate::nmfow::FallOverWallParams,
    pub grab: crate::nmgr::GrabParams,
    pub head_look: HeadLookMsg,
    pub high_fall: crate::nmhf::HighFallParams,
    pub lean_in_direction: Lean,
    pub lean_to_position: LeanTo,
    pub lean_towards_object: LeanTowards,
    pub pedal_legs: crate::nm::PedalLegs,
    pub point_arm: crate::nmpa::PointArmParams,
    pub roll_down_stairs: crate::nmrds::RollDownStairs,
    pub set_stiffness: Stiffness,
    pub shot: crate::nmshot::Params,
    pub shot_relax: ShotRelax,
    pub stay_upright: StayUpright,
    pub upper_body_flinch: crate::nm::UpperBodyFlinch,
}

fn intern(s: &str) -> &'static str {
    static SEEN: std::sync::Mutex<Vec<&'static str>> = std::sync::Mutex::new(Vec::new());
    let mut seen = SEEN.lock().unwrap();
    if let Some(&k) = seen.iter().find(|k| **k == s) {
        return k;
    }
    let k: &'static str = Box::leak(s.into());
    seen.push(k);
    k
}

pub fn find(name: &str) -> Option<&'static Message> {
    MESSAGES.iter().chain(crate::extras::messages()).find(|m| m.name == name)
}

pub fn send(a: &mut Agent, build: &Build, world: &Collision, st: &mut State, msg: &str, params: &[(&str, Value)]) -> Result<usize, Error> {
    if let Some(r) = a.extras.send(msg, params) {
        return r;
    }
    // RDR2 core: RDR2's highFall, from this message's parameters over RDR2's defaults (its start reloads them)
    if msg == "highFall" && a.rdr2_core.is_some() {
        return a.extras.send_named("highFall", params);
    }
    // and its fallOverWall
    if msg == "fallOverWall" && a.rdr2_core.is_some() {
        return a.extras.send_named("fallOverWall", params);
    }
    let m = find(msg).ok_or(Error::Message)?;
    let start = params.iter().find(|(n, _)| *n == "start").map(|(_, v)| v.b()).transpose()?;
    let one_shot = matches!(msg, "activePose" | "applyImpulse" | "bodyRelax" | "defineAttachedObject" | "leanInDirection"
                                 | "leanToPosition" | "leanTowardsObject" | "setStiffness" | "shotRelax");
    // RDR2 core: grab's start reloads RDR2's defaults
    if start == Some(true) && msg == "grab" && a.rdr2_core.is_some() {
        st.grab = crate::nmgr::GrabParams::rdr2();
    }
    if start == Some(true) && matches!(msg, "bodyBalance" | "shot" | "pointArm") {
        reset(st, msg);
        // RDR2 core: pointArm's own two-arm parameters
        if msg == "pointArm" && a.rdr2_core.is_some() {
            st.point_arm.rdr2 = Some(Default::default());
        }
    } else if start == Some(true) && a.rdr2_core.is_some() && matches!(msg, "pedalLegs" | "bodyFoetal") {
        // RDR2 core: a start reloads RDR2's defaults before the message's values
        match msg {
            "pedalLegs" => st.pedal_legs = crate::nmpd::rdr2_message(),
            _ => st.body_foetal = crate::nmbf::Foetal::rdr2(),
        }
    } else if !one_shot {
        a.held_params(st, msg);
    }
    // RDR2 core: a start reloads RDR2's registered defaults where they aren't GTA IV's
    if start == Some(true) && a.rdr2_core.is_some() {
        for (n, v) in rdr2_start_defaults(msg) {
            set(st, msg, n, &v)?;
        }
    }
    // RDR2 core: StayUpright's message copies the template over the set, then its parameters
    if msg == "stayUpright" && a.rdr2_core.is_some() {
        (st.stay_upright.rdr2, st.stay_upright.side) = (Default::default(), None);
    }
    let has = |n: &str| params.iter().any(|(p, _)| *p == n);
    let ang_vel = has("angVelMultiplier") && has("angVelThreshold");
    let mut unused = 0;
    // RDR2's handler (0x2a63cbc): start=true is the registered defaults, then this message's values
    if msg == "configureBalance" && start == Some(true) && a.rdr2_core.is_some() {
        st.configure_balance = Default::default();
        a.configure_balance_rdr2_reset();
    }
    for (name, v) in params {
        if *name == "start" || (msg == "bodyBalance" && !ang_vel && matches!(*name, "angVelMultiplier" | "angVelThreshold")) {
            continue;
        }
        let prm = m.params.iter().find(|p| p.name == *name).ok_or(Error::Param)?;
        // RDR2's configureBalance: the balancer's parameters, nothing without the RDR2 core
        if msg == "configureBalance" && !matches!(*name, "legStiffness" | "stepHeight" | "stepClampScale" | "balanceAbortThreshold" | "taperKneeStrength") {
            if a.rdr2_core.is_some() {
                let x = match v { Value::Bool(b) => *b as i32 as f32, _ => v.f()? };
                if *name == "reset" {
                    if x != 0.0 {
                        a.configure_balance_rdr2_reset();
                    }
                } else {
                    a.configure_balance_rdr2(prm.name, x);
                }
            }
            continue;
        }
        if !prm.used {
            match prm.kind {
                Kind::Float => drop(v.f()?),
                Kind::Int => drop(v.i()?),
                Kind::Bool => drop(v.b()?),
                Kind::Vec3 => drop(v.v()?),
                Kind::Str => drop(v.s()?),
            }
            unused += 1;
            continue;
        }
        set(st, msg, name, v)?;
    }
    let on = start == Some(true);
    match msg {
        "activePose" => a.active_pose(st.active_pose.mask.as_deref()),
        "incomingTransforms" => if let Some(s) = start { a.incoming_state = if s { 2 } else { delirium_data::kv!(1 int) } },
        "applyImpulse" => { let p = &st.apply_impulse; a.doll.equalized_impulse(p.part_index.max(0) as usize, p.impulse, p.hit_point, p.equalize_amount); }
        "bodyRelax" => { let p = &st.body_relax; a.body_relax_full(p.relaxation, p.damping, p.mask.as_deref(), p.hold_pose); }
        "configureBalance" => a.configure_balance(st.configure_balance.clone()),
        "configureBalanceReset" => {
            st.configure_balance = Default::default();
            a.configure_balance(Default::default());
            if a.rdr2_core.is_some() {
                a.configure_balance_rdr2_reset();
            }
        }
        "defineAttachedObject" => { let p = &st.define_attached_object; if p.part_index >= 0 { a.define_attached_object(p.part_index as usize, p.world_pos, p.object_mass); } }
        "leanInDirection" => { let p = &st.lean_in_direction; a.lean_in_direction(p.dir, p.lean_amount, f32::INFINITY, p.apply_as_force); }
        "leanToPosition" => { let p = &st.lean_to_position; a.lean_to(crate::nmdbm::LeanMode::Pos(p.pos), p.lean_amount, p.apply_as_force); }
        "leanTowardsObject" => { let p = &st.lean_towards_object; a.lean_to(crate::nmdbm::LeanMode::Object(p.offset, -1), p.lean_amount, p.apply_as_force); }
        "setStiffness" => { let p = &st.set_stiffness; a.set_stiffness_mask(p.body_stiffness, p.damping, &p.mask); }
        "shotRelax" => a.shot_relax_rdr2(&st.shot_relax),
        "stopAllBehaviours" => a.stop_all_behaviours(),
        // RDR2 core: a message with RDR2's parameters, or a stop, is RDR2's; GTA IV's two alone stay GTA IV's stabiliser
        "stayUpright" if a.rdr2_core.is_some() && (start == Some(false) || params.iter().any(|(n, _)| !matches!(*n, "start" | "stiffness" | "dampingScale"))) => {
            let p = &st.stay_upright;
            a.stay_upright_rdr2(start != Some(false), p.side.map(|i| i.max(0) as usize), p.rdr2);
        }
        "stayUpright" => { let p = &st.stay_upright; a.stay_upright(start != Some(false), p.stiffness, p.damping_scale); }
        _ if start.is_none() => {
            a.update_params(build, world, st, msg, params.iter().any(|(n, _)| *n == "bodyPart"));
            st.upper_body_flinch.new_hit = None;
        }
        "armsWindmillAdaptive" => if on { a.arms_windmill(st.arms_windmill_adaptive.clone()) } else if a.rdr2_core.is_some() { a.stop_windmill() },
        "bodyBalance" => if on { a.body_balance_full(build, world, st.body_balance) } else { a.body_balance_stop() },
        "bodyFoetal" => a.body_foetal(on.then(|| st.body_foetal.clone())),
        "bodyRollUp" => if on { a.body_roll_up(st.body_roll_up.clone()) } else { a.stop_roll_up() },
        "bodyWrithe" => if on { a.body_writhe(st.body_writhe.clone()) } else if a.rdr2_core.is_some() { a.stop_writhe() },
        "braceForImpact" => if on { a.brace_for_impact(build, st.brace_for_impact.clone()) },
        "catchFall" => if on { a.catch_fall_full(build, st.catch_fall.clone()) } else { a.stop_catch_fall() },
        "fallOverWall" => if on {
            a.fall_over_wall(st.fall_over_wall.clone());
            a.fow_rdr2 = Some(params.iter().map(|(n, v)| (n.to_string(), v.clone())).collect());
        },
        "grab" => a.grab(build, on.then(|| st.grab.clone())),
        "headLook" => { let p = &st.head_look;
            if a.rdr2_core.is_some() {
                (a.nm_head.keep_head_away, a.nm_head.twist_spine) = (p.keep_head_away, p.twist_spine);
            }
            a.head_look(build, on.then_some((p.pos, p.vel)), p.stiffness, p.damping, p.always_look, p.always_eyes_horizontal); }
        "highFall" => if on {
            a.high_fall(st.high_fall.clone());
            a.hf_rdr2 = Some(params.iter().map(|(n, v)| (n.to_string(), v.clone())).collect());
        },
        "pedalLegs" => if on { a.pedal_legs(st.pedal_legs.clone()) } else { a.stop_pedal() },
        "pointArm" => a.point_arm(on.then_some(st.point_arm)),
        "rollDownStairs" => if on { a.roll_down_stairs(st.roll_down_stairs.clone()) } else { a.stop_roll_down_stairs() },
        "shot" => if on {
            a.shot_start(st.shot);
            a.nm_hit_hold = params.iter().any(|(n, _)| *n == "bodyPart");
        },
        "upperBodyFlinch" => if on { a.upper_body_flinch(build, st.upper_body_flinch.clone()); st.upper_body_flinch.new_hit = None; },
        _ => {}
    }
    if one_shot {
        reset(st, msg);
    }
    Ok(unused)
}

/// What RDR2 registers for these where the message's defaults here are GTA IV's.
fn rdr2_start_defaults(msg: &str) -> Vec<(&'static str, Value)> {
    use Value::{Bool as B, Float as F};
    match msg {
        "bodyBalance" => vec![("elbow", F(0.9)), ("armsOutOnPush", B(true)), ("armsOutOnPushMultiplier", F(1.0)), ("armsOutOnPushTimeout", F(1.1)),
                              ("bendElbowsTime", F(0.3)), ("useBodyTurn", B(true))],
        "catchFall" => vec![("legsStiffness", F(8.25)), ("armsStiffness", F(13.0)), ("torsoStiffness", F(10.0))],
        "braceForImpact" => vec![("reachAbsorbtionTime", F(0.15)), ("braceDistance", F(0.5)), ("targetPredictionTime", F(0.45))],
        _ => vec![],
    }
}

/// The parameter audit: a start's RDR2 defaults on these values.
#[cfg(feature = "param-audit")]
pub fn audit_start(st: &mut State, msg: &str) {
    for (n, v) in rdr2_start_defaults(msg) {
        let _ = set(st, msg, n, &v);
    }
}

fn reset(st: &mut State, msg: &str) {
    match msg {
        "activePose" => st.active_pose = Default::default(),
        "applyImpulse" => st.apply_impulse = Default::default(),
        "bodyBalance" => st.body_balance = Default::default(),
        "bodyRelax" => st.body_relax = Default::default(),
        "defineAttachedObject" => st.define_attached_object = Default::default(),
        "leanInDirection" => st.lean_in_direction = Default::default(),
        "leanToPosition" => st.lean_to_position = Default::default(),
        "leanTowardsObject" => st.lean_towards_object = Default::default(),
        "pointArm" => st.point_arm = Default::default(),
        "setStiffness" => st.set_stiffness = Default::default(),
        "shot" => st.shot = Default::default(),
        "shotRelax" => st.shot_relax = Default::default(),
        _ => {}
    }
}

fn set(st: &mut State, msg: &str, name: &str, v: &Value) -> Result<(), Error> {
    match (msg, name) {
        ("activePose", "mask") => { let p = &mut st.active_pose; p.mask = Some(v.s()?.to_string()); }
        ("applyImpulse", "partIndex") => { let p = &mut st.apply_impulse; p.part_index = v.i()?; }
        ("applyImpulse", "hitPoint") => { let p = &mut st.apply_impulse; p.hit_point = Some(v.v()?); }
        ("applyImpulse", "impulse") => { let p = &mut st.apply_impulse; p.impulse = v.v()?; }
        ("applyImpulse", "equalizeAmount") => { let p = &mut st.apply_impulse; p.equalize_amount = v.f()?; }
        ("armsWindmillAdaptive", "bodyStiffness") => { let p = &mut st.arms_windmill_adaptive; p.body_stiffness = v.f()?; p.rdr2.body_stiffness = p.body_stiffness; }
        ("armsWindmillAdaptive", "amplitude") => { let p = &mut st.arms_windmill_adaptive; p.amplitude = v.f()?; p.rdr2.amplitude = p.amplitude; }
        ("armsWindmillAdaptive", "phase") => { let p = &mut st.arms_windmill_adaptive; p.phase = v.f()?; p.rdr2.phase = p.phase; }
        ("armsWindmillAdaptive", "disableOnImpact") => { let p = &mut st.arms_windmill_adaptive; p.disable_on_impact = v.b()?; p.rdr2.disable_on_impact = p.disable_on_impact; }
        ("armsWindmillAdaptive", "period") => { let p = &mut st.arms_windmill_adaptive; p.period = v.f()?; }
        ("armsWindmillAdaptive", "armStiffness") => { let p = &mut st.arms_windmill_adaptive; p.arm_stiffness = v.f()?; p.body_stiffness = p.arm_stiffness; p.rdr2.arm_stiffness = p.arm_stiffness; }
        ("armsWindmillAdaptive", "angSpeed") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.ang_speed = v.f()?; }
        ("armsWindmillAdaptive", "claviclescale") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.clavicle_scale = v.f()?; }
        ("armsWindmillAdaptive", "claviclebias") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.clavicle_bias = v.f()?; }
        ("armsWindmillAdaptive", "leftElbowAngle") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.elbow_angle[0] = v.f()?; }
        ("armsWindmillAdaptive", "rightElbowAngle") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.elbow_angle[1] = v.f()?; }
        ("armsWindmillAdaptive", "lean1mult") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.lean1.0 = v.f()?; }
        ("armsWindmillAdaptive", "lean1offset") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.lean1.1 = v.f()?; }
        ("armsWindmillAdaptive", "lean2mult") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.lean2.0 = v.f()?; }
        ("armsWindmillAdaptive", "lean2offset") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.lean2.1 = v.f()?; }
        ("armsWindmillAdaptive", "elbowRate") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.elbow_rate = v.f()?; }
        ("armsWindmillAdaptive", "swayamount") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.sway = v.f()?; }
        ("armsWindmillAdaptive", "angSpeedMultiplier4Dragging") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.drag_mult = v.f()?; }
        ("armsWindmillAdaptive", "maxangspeed") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.max_ang_speed = v.f()?; }
        ("armsWindmillAdaptive", "scrabblebendupper") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.scrabble_upper = v.f()?; }
        ("armsWindmillAdaptive", "scrabblebendlower") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.scrabble_lower = v.f()?; }
        ("armsWindmillAdaptive", "scrabblebend") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.scrabble_bend = v.f()?; }
        ("armsWindmillAdaptive", "scrabbleextend") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.scrabble_extend = v.f()?; }
        ("armsWindmillAdaptive", "armDirection") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.arm_direction = v.i()?; }
        ("armsWindmillAdaptive", "mask") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.mask = intern(v.s()?); }
        ("armsWindmillAdaptive", "setBackAngles") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.set_back_angles = v.b()?; }
        ("armsWindmillAdaptive", "useAngMom") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.use_ang_mom = v.b()?; }
        ("armsWindmillAdaptive", "bendLeftElbow") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.bend_elbow[0] = v.b()?; }
        ("armsWindmillAdaptive", "bendRightElbow") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.bend_elbow[1] = v.b()?; }
        ("armsWindmillAdaptive", "scrabble") => { let p = &mut st.arms_windmill_adaptive; p.rdr2.scrabble = v.b()?; }
        ("bodyBalance", "bodyStiffness") => { let p = &mut st.body_balance; p.body_stiffness = v.f()?; }
        ("bodyBalance", "headLookAtVelProb") => { let p = &mut st.body_balance; p.head_look_at_vel_prob = v.f()?; }
        ("bodyBalance", "armsOutStraightenElbows") => { let p = &mut st.body_balance; p.arms_out_straighten_elbows = v.f()?; }
        ("bodyBalance", "bendElbowsTime") => { let p = &mut st.body_balance; p.bend_elbows_time = v.f()?; }
        ("bodyBalance", "armStiffness") => { let p = &mut st.body_balance; p.body_stiffness = v.f()?; }
        ("bodyBalance", "armDamping") => { let p = &mut st.body_balance; p.damping = v.f()?; }
        ("bodyBalance", "elbow") => { let p = &mut st.body_balance; p.elbow = v.f()?; }
        ("bodyBalance", "angVelMultiplier") => { let p = &mut st.body_balance; p.ang_vel_multiplier = v.v()?; }
        ("bodyBalance", "angVelThreshold") => { let p = &mut st.body_balance; p.ang_vel_threshold = v.v()?; }
        ("bodyBalance", "returningToBalanceArmsOut") => { let p = &mut st.body_balance; p.returning_to_balance_arms_out = v.f()?; }
        ("bodyBalance", "somersaultAngleThreshold") => { let p = &mut st.body_balance; p.somersault_angle_threshold = v.f()?; }
        ("bodyBalance", "armsOutMinLean2") => { let p = &mut st.body_balance; p.arms_out_min_lean2 = v.f()?; }
        ("bodyBalance", "sideSomersaultAngleThreshold") => { let p = &mut st.body_balance; p.side_somersault_angle_threshold = v.f()?; }
        ("bodyBalance", "bendElbowsGait") => { let p = &mut st.body_balance; p.bend_elbows_gait = v.f()?; }
        ("bodyBalance", "armsOutOnPush") => { let p = &mut st.body_balance; p.arms_out_on_push = v.b()?; }
        ("bodyBalance", "elbowAngleOnContact") => { let p = &mut st.body_balance; p.elbow_angle_on_contact = v.f()?; }
        ("bodyBalance", "bodyTurnTimeout") => { let p = &mut st.body_balance; p.spine_damping = v.f()?; }
        ("bodyBalance", "turnOffProb") => { let p = &mut st.body_balance; p.turn_off_prob = v.f()?; }
        ("bodyBalance", "turn2VelProb") => { let p = &mut st.body_balance; p.turn2_vel_prob = v.f()?; }
        ("bodyBalance", "damping") => { let p = &mut st.body_balance; p.damping = v.f()?; }
        ("bodyBalance", "headLookPos") => { let p = &mut st.body_balance; p.head_look_pos = v.v()?; }
        ("bodyBalance", "sideSomersaultAngle") => { let p = &mut st.body_balance; p.side_somersault_angle = v.f()?; }
        ("bodyBalance", "headLookAlwaysTimeout") => { let p = &mut st.body_balance; p.spine_stiffness = v.f()?; }
        ("bodyBalance", "turnAwayProb") => { let p = &mut st.body_balance; p.turn_away_prob = v.f()?; }
        ("bodyBalance", "shoulder") => { let p = &mut st.body_balance; p.shoulder = v.f()?; }
        ("bodyBalance", "turnRightProb") => { let p = &mut st.body_balance; p.turn_right_prob = v.f()?; }
        ("bodyBalance", "turnLeftProb") => { let p = &mut st.body_balance; p.turn_left_prob = v.f()?; }
        ("bodyBalance", "somersaultAngle") => { let p = &mut st.body_balance; p.somersault_angle = v.f()?; }
        ("bodyBalance", "armsOutOnPushMultiplier") => { let p = &mut st.body_balance; p.arms_out_on_push_multiplier = v.f()?; }
        ("bodyBalance", "turn2TargetProb") => { let p = &mut st.body_balance; p.turn2_target_prob = v.f()?; }
        ("bodyBalance", "useBodyTurn") => { let p = &mut st.body_balance; p.use_body_turn = v.b()?; }
        ("bodyBalance", "useHeadLook") => { let p = &mut st.body_balance; p.use_head_look = v.b()?; }
        ("bodyBalance", "headLookInstanceIndex") => { let p = &mut st.body_balance; p.head_look_instance_index = v.i()?; }
        ("bodyBalance", "armsOutOnPushTimeout") => { let p = &mut st.body_balance; p.arms_out_on_push_timeout = v.f()?; }
        ("bodyBalance", "spineStiffness") => { let p = &mut st.body_balance; p.spine_stiffness = v.f()?; }
        ("bodyBalance", "spineDamping") => { let p = &mut st.body_balance; p.spine_damping = v.f()?; }
        ("bodyBalance", "shoulderL1") => { let p = &mut st.body_balance; p.rdr2.shoulder_l1 = v.f()?; }
        ("bodyBalance", "shoulderL2") => { let p = &mut st.body_balance; p.rdr2.shoulder_l2 = v.f()?; }
        ("bodyBalance", "hipL2ArmL2") => { let p = &mut st.body_balance; p.rdr2.hip_l2_arm_l2 = v.f()?; }
        ("bodyBalance", "shoulderTwist") => { let p = &mut st.body_balance; p.rdr2.shoulder_twist = v.f()?; }
        ("bodyBalance", "airbornearms") => { let p = &mut st.body_balance; p.rdr2.airborne_arms = v.f()?; }
        ("bodyBalance", "axisweightssimulationblocked") => { let p = &mut st.body_balance; p.rdr2.axis_weights_simulation_blocked = v.b()?; }
        ("bodyBalance", "moveRadius") => { let p = &mut st.body_balance; p.rdr2.move_radius = v.f()?; }
        ("bodyBalance", "blendToZeroPose") => { let p = &mut st.body_balance; p.rdr2.zero_pose = v.b()?; }
        ("bodyBalance", "startbalancer") => { let p = &mut st.body_balance; p.rdr2.start_balancer = v.b()?; }
        ("bodyBalance", "fallingReaction") => { let p = &mut st.body_balance; p.rdr2.falling_reaction = v.i()?; }
        ("bodyBalance", "gshelp") => { let p = &mut st.body_balance; p.rdr2.gs_help = v.b()?; }
        ("bodyBalance", "?01d67b58") => { let p = &mut st.body_balance; p.rdr2.turn_block = v.b()?; }
        ("bodyBalance", "leanamountstill") => { let p = &mut st.body_balance; p.rdr2.lean_still = v.f()?; }
        ("bodyBalance", "braceOffset") => { let p = &mut st.body_balance; p.rdr2.brace_offset = v.f()?; }
        ("bodyBalance", "timeToBackwardsBrace") => { let p = &mut st.body_balance; p.rdr2.back_brace = v.f()?; }
        ("bodyBalance", "braceDistance") => { let p = &mut st.body_balance; p.rdr2.brace_distance = v.f()?; }
        ("bodyBalance", "bumpweakness") => { let p = &mut st.body_balance; p.rdr2.bump_weak = v.f()?; }
        ("bodyBalance", "bumpweaknesstime") => { let p = &mut st.body_balance; p.rdr2.bump_time = v.f()?; }
        ("bodyBalance", "turnWithBumpRadius") => { let p = &mut st.body_balance; p.rdr2.bump_radius = v.f()?; }
        ("bodyBalance", "moveAmount") => { let p = &mut st.body_balance; p.rdr2.move_amount = v.f()?; }
        ("bodyBalance", "moveamountstill") => { let p = &mut st.body_balance; p.rdr2.move_amount_still = v.f()?; }
        ("bodyBalance", "moveWhenBracing") => { let p = &mut st.body_balance; p.rdr2.move_when_bracing = v.b()?; }
        ("bodyBalance", "maxSteps") => { let p = &mut st.body_balance; p.max_steps = v.i()?.max(0) as u32; }
        ("bodyFoetal", "mask") => { let p = &mut st.body_foetal; p.mask = intern(v.s()?); }
        ("bodyFoetal", "stiffness") => { let p = &mut st.body_foetal; p.stiffness = v.f()?; }
        ("bodyFoetal", "asymmetry") => { let p = &mut st.body_foetal; p.asymmetry = v.f()?; }
        ("bodyFoetal", "randomSeed") => { let p = &mut st.body_foetal; p.random_seed = v.i()?.max(0) as u32; }
        ("bodyFoetal", "dampingFactor") => { let p = &mut st.body_foetal; p.damping_factor = v.f()?; }
        ("bodyFoetal", "backTwist") => { let p = &mut st.body_foetal; p.back_twist = v.f()?; }
        ("bodyRelax", "mask") => { let p = &mut st.body_relax; p.mask = Some(v.s()?.to_string()); }
        ("bodyRelax", "holdPose") => { let p = &mut st.body_relax; p.hold_pose = v.b()?; }
        ("bodyRelax", "relaxation") => { let p = &mut st.body_relax; p.relaxation = v.f()?; }
        ("bodyRelax", "damping") => { let p = &mut st.body_relax; p.damping = Some(v.f()?); }
        ("bodyRollUp", "mask") => { let p = &mut st.body_roll_up; p.mask = Some(intern(v.s()?)); }
        ("bodyRollUp", "stiffness") => { let p = &mut st.body_roll_up; p.stiffness = v.f()?; }
        ("bodyRollUp", "legPush") => { let p = &mut st.body_roll_up; p.leg_push = v.f()?; }
        ("bodyRollUp", "armReachAmount") => { let p = &mut st.body_roll_up; p.arm_reach_amount = v.f()?; }
        ("bodyRollUp", "useArmToSlowDown") => { let p = &mut st.body_roll_up; p.use_arm_to_slow_down = v.f()?; }
        ("bodyRollUp", "asymmetricalLegs") => { let p = &mut st.body_roll_up; p.asymmetrical_legs = v.f()?; }
        ("bodyRollUp", "damping") => { let p = &mut st.body_roll_up; p.rdr2.damping = v.f()?; }
        ("bodyRollUp", "muscleStiffness") => { let p = &mut st.body_roll_up; p.rdr2.muscle_stiffness = v.f()?; }
        ("bodyRollUp", "noRollTimeBeforeSuccess") => { let p = &mut st.body_roll_up; p.rdr2.no_roll_time_before_success = v.f()?; }
        ("bodyRollUp", "rollVelForSuccess") => { let p = &mut st.body_roll_up; p.rdr2.roll_vel_for_success = v.f()?; }
        ("bodyRollUp", "rollVelLinearContribution") => { let p = &mut st.body_roll_up; p.rdr2.roll_vel_linear_contribution = v.f()?; }
        ("bodyRollUp", "velocityScale") => { let p = &mut st.body_roll_up; p.rdr2.velocity_scale = v.f()?; }
        ("bodyRollUp", "velocityOffset") => { let p = &mut st.body_roll_up; p.rdr2.velocity_offset = v.f()?; }
        ("bodyWrithe", "armStiffness") => { let p = &mut st.body_writhe; p.arm_stiffness = v.f()?; }
        ("bodyWrithe", "legStiffness") => { let p = &mut st.body_writhe; p.leg_stiffness = v.f()?; }
        ("bodyWrithe", "legPeriod") => { let p = &mut st.body_writhe; p.leg_period = v.f()?; }
        ("bodyWrithe", "elbowAmplitude") => { let p = &mut st.body_writhe; p.elbow_amplitude = v.f()?; }
        ("bodyWrithe", "legAmplitude") => { let p = &mut st.body_writhe; p.leg_amplitude = v.f()?; }
        ("bodyWrithe", "kneeAmplitude") => { let p = &mut st.body_writhe; p.knee_amplitude = v.f()?; }
        ("bodyWrithe", "backDamping") => { let p = &mut st.body_writhe; p.back_damping = v.f()?; }
        ("bodyWrithe", "backPeriod") => { let p = &mut st.body_writhe; p.back_period = v.f()?; }
        ("bodyWrithe", "rollOverFlag") => { let p = &mut st.body_writhe; p.roll_over = v.b()?; }
        ("bodyWrithe", "backAmplitude") => { let p = &mut st.body_writhe; p.back_amplitude = v.f()?; }
        ("bodyWrithe", "backStiffness") => { let p = &mut st.body_writhe; p.back_stiffness = v.f()?; }
        ("bodyWrithe", "armDamping") => { let p = &mut st.body_writhe; p.arm_damping = v.f()?; }
        ("bodyWrithe", "legDamping") => { let p = &mut st.body_writhe; p.leg_damping = v.f()?; }
        ("bodyWrithe", "mask") => { let p = &mut st.body_writhe; p.mask = intern(v.s()?); }
        ("bodyWrithe", "armAmplitude") => { let p = &mut st.body_writhe; p.arm_amplitude = v.f()?; }
        ("bodyWrithe", "armPeriod") => { let p = &mut st.body_writhe; p.arm_period = v.f()?; }
        ("bodyWrithe", "blendArms") => { let p = &mut st.body_writhe; p.rdr2.blend_arms = v.f()?; }
        ("bodyWrithe", "blendBack") => { let p = &mut st.body_writhe; p.rdr2.blend_back = v.f()?; }
        ("bodyWrithe", "blendLegs") => { let p = &mut st.body_writhe; p.rdr2.blend_legs = v.f()?; }
        ("bodyWrithe", "applyStiffness") => { let p = &mut st.body_writhe; p.rdr2.apply_stiffness = v.b()?; }
        ("bodyWrithe", "onFire") => { let p = &mut st.body_writhe; p.rdr2.on_fire = v.b()?; }
        ("bodyWrithe", "shoulderLean1") => { let p = &mut st.body_writhe; p.rdr2.shoulder_lean1 = v.f()?; }
        ("bodyWrithe", "shoulderLean2") => { let p = &mut st.body_writhe; p.rdr2.shoulder_lean2 = v.f()?; }
        ("bodyWrithe", "lean1BlendFactor") => { let p = &mut st.body_writhe; p.rdr2.lean1_blend_factor = v.f()?; }
        ("bodyWrithe", "lean2BlendFactor") => { let p = &mut st.body_writhe; p.rdr2.lean2_blend_factor = v.f()?; }
        ("bodyWrithe", "rollTorqueScale") => { let p = &mut st.body_writhe; p.rdr2.roll_torque_scale = v.f()?; }
        ("bodyWrithe", "maxRollOverTime") => { let p = &mut st.body_writhe; p.rdr2.max_roll_over_time = v.f()?; }
        ("bodyWrithe", "rollOverRadius") => { let p = &mut st.body_writhe; p.rdr2.roll_over_radius = v.f()?; }
        ("braceForImpact", "bodyStiffness") => { let p = &mut st.brace_for_impact; p.body_stiffness = v.f()?; }
        ("braceForImpact", "look") => { let p = &mut st.brace_for_impact; p.look = v.v()?; }
        ("braceForImpact", "grabReachAngle") => { let p = &mut st.brace_for_impact; p.grab_reach_angle = v.f()?; }
        ("braceForImpact", "grabHoldTimer") => { let p = &mut st.brace_for_impact; p.grab_hold_timer = v.f()?; }
        ("braceForImpact", "braceDistance") => { let p = &mut st.brace_for_impact; p.brace_distance = v.f()?; }
        ("braceForImpact", "targetPredictionTime") => { let p = &mut st.brace_for_impact; p.target_prediction_time = v.f()?; }
        ("braceForImpact", "grabDistance") => { let p = &mut st.brace_for_impact; p.grab_distance = v.f()?; }
        ("braceForImpact", "pos") => { let p = &mut st.brace_for_impact; p.pos = v.v()?; }
        ("braceForImpact", "reachAbsorbtionTime") => { let p = &mut st.brace_for_impact; p.reach_absorbtion_time = v.f()?; }
        ("braceForImpact", "grabDontLetGo") => { let p = &mut st.brace_for_impact; p.grab_dont_let_go = v.b()?; }
        ("braceForImpact", "legStiffness") => { let p = &mut st.brace_for_impact; p.leg_stiffness = v.f()?; }
        ("braceForImpact", "grabStrength") => { let p = &mut st.brace_for_impact; p.grab_strength = v.f()?; }
        ("braceForImpact", "instanceIndex") => { let p = &mut st.brace_for_impact; p.instance_index = v.i()?; }
        ("braceForImpact", "hash_4f7c0f0b") => { let p = &mut st.brace_for_impact; p.behind_time = v.f()?; }
        ("catchFall", "useHeadLook") => { let p = &mut st.catch_fall; p.use_head_look = v.b()?; }
        ("catchFall", "armsStiffness") => { let p = &mut st.catch_fall; p.arms_stiffness = v.f()?; }
        ("catchFall", "torsoStiffness") => { let p = &mut st.catch_fall; p.torso_stiffness = v.f()?; }
        ("catchFall", "legsStiffness") => { let p = &mut st.catch_fall; p.legs_stiffness = v.f()?; }
        ("catchFall", "forwardMaxArmOffset") => { let p = &mut st.catch_fall; p.forward_max_arm_offset = v.f()?; }
        ("catchFall", "zAxisSpinReduction") => { let p = &mut st.catch_fall; p.z_axis_spin_reduction = v.f()?; }
        ("catchFall", "backwardsMinArmOffset") => { let p = &mut st.catch_fall; p.backwards_min_arm_offset = v.f()?; }
        ("configureBalance", "legStiffness") => { let p = &mut st.configure_balance; p.leg_stiffness = v.f()?; }
        ("configureBalance", "stepHeight") => { let p = &mut st.configure_balance; p.step_height = v.f()?; }
        ("configureBalance", "stepClampScale") => { let p = &mut st.configure_balance; p.step_clamp_scale = v.f()?; }
        ("configureBalance", "balanceAbortThreshold") => { let p = &mut st.configure_balance; p.balance_abort_threshold = v.f()?; }
        ("configureBalance", "taperKneeStrength") => { let p = &mut st.configure_balance; p.taper_knee_strength = v.b()?; }
        ("defineAttachedObject", "partIndex") => { let p = &mut st.define_attached_object; p.part_index = v.i()?; }
        ("defineAttachedObject", "worldPos") => { let p = &mut st.define_attached_object; p.world_pos = v.v()?; }
        ("defineAttachedObject", "objectMass") => { let p = &mut st.define_attached_object; p.object_mass = v.f()?; }
        ("fallOverWall", "bodyTwist") => { let p = &mut st.fall_over_wall; p.body_twist(v.f()?); }
        ("fallOverWall", "useSpecifiedPoint") => { let p = &mut st.fall_over_wall; p.use_specified_point = v.b()?; }
        ("fallOverWall", "maxDistanceFromPelToHitPoint") => { let p = &mut st.fall_over_wall; p.max_distance_from_pel_to_hit_point = v.f()?; }
        ("fallOverWall", "fallOverEdge") => { let p = &mut st.fall_over_wall; p.fall_over_edge = v.v()?; }
        ("fallOverWall", "fallOverPoint") => { let p = &mut st.fall_over_wall; p.fall_over_point = v.v()?; }
        ("fallOverWall", "magOfForce") => { let p = &mut st.fall_over_wall; p.mag_of_force = v.f()?; }
        ("fallOverWall", "bodyStiffness") => { let p = &mut st.fall_over_wall; p.body_stiffness = v.f()?; }
        ("fallOverWall", "damping") => { let p = &mut st.fall_over_wall; p.damping = v.f()?; }
        ("grab", "bodyStiffness") => { let p = &mut st.grab; p.body_stiffness = v.f()?; }
        ("grab", "instanceIndex") => { let p = &mut st.grab; p.instance = v.i()?; }
        ("grab", "instancePartIndex") => { let p = &mut st.grab; p.part = v.i()?; }
        ("grab", "armStiffness") => { let p = &mut st.grab; p.arm_stiffness = v.f()?; }
        ("grab", "grabStrength") => { let p = &mut st.grab; p.grab_strength = v.f()?; }
        ("grab", "useLeft") => { let p = &mut st.grab; p.use_left = v.b()?; }
        ("grab", "normalR") => { let p = &mut st.grab; p.normal_r = v.v()?; }
        ("grab", "pos3") => { let p = &mut st.grab; p.pos[2] = v.v()?; }
        ("grab", "useRight") => { let p = &mut st.grab; p.use_right = v.b()?; }
        ("grab", "pos1") => { let p = &mut st.grab; p.pos[0] = v.v()?; }
        ("grab", "pos2") => { let p = &mut st.grab; p.pos[1] = v.v()?; }
        ("grab", "pos4") => { let p = &mut st.grab; p.pos[delirium_data::kv!(0 int)] = v.v()?; }
        ("grab", "surfaceGrab") => { let p = &mut st.grab; p.surface_grab = v.b()?; }
        ("grab", "oneSideReachAngle") => { let p = &mut st.grab; p.one_side_reach_angle = v.f()?; }
        ("grab", "normalL") => { let p = &mut st.grab; p.normal_l = v.v()?; }
        ("grab", "linegrab") => { let p = &mut st.grab; p.linegrab = v.b()?; }
        ("grab", "grabDist") => { let p = &mut st.grab; p.grab_dist = v.f()?; }
        ("grab", "dontLetGo") => { let p = &mut st.grab; p.dont_let_go = v.b()?; }
        ("grab", "justbrace") => { let p = &mut st.grab; p.justbrace = v.b()?; }
        ("grab", "reachAngle") => { let p = &mut st.grab; p.reach_angle = v.f()?; }
        ("grab", "pullUpStrengthRight") => { let p = &mut st.grab; p.pull_up_strength[1] = v.f()?; }
        ("grab", "pullUpTime") => { let p = &mut st.grab; p.pull_up_time = v.f()?; }
        ("grab", "pullUpStrengthLeft") => { let p = &mut st.grab; p.pull_up_strength[0] = v.f()?; }
        ("grab", "grabHoldMaxTimer") => { let p = &mut st.grab; p.grab_hold_max_timer = v.f()?; }
        ("grab", "reachDistance") => { let p = &mut st.grab; p.max_reach = v.f()?; }
        ("grab", "justBrace") => { let p = &mut st.grab; p.justbrace = v.b()?; }
        ("grab", "useLineGrab") => { let p = &mut st.grab; p.linegrab = v.b()?; }
        ("grab", "grabDistance") => { let p = &mut st.grab; p.grab_dist = v.f()?; }
        ("grab", "maxReachDistance") => { let p = &mut st.grab; p.max_reach = v.f()?; }
        ("grab", "dropWeaponIfNecessary") => { let p = &mut st.grab; p.drop_weapon = v.b()?; }
        ("grab", "dropWeaponDistance") => { let p = &mut st.grab; p.drop_weapon_distance = v.f()?; }
        ("grab", "stickyHands") => { let p = &mut st.grab; p.sticky_hands = v.f()?; }
        ("grab", "turnToTarget") => { let p = &mut st.grab; p.turn_to_target = v.i()?; }
        ("grab", "handsCollide") => { let p = &mut st.grab; p.hands_collide = v.b()?; }
        ("grab", "pointsX4grab") => { let p = &mut st.grab; p.points_x4 = v.b()?; }
        ("grab", "normalL2") => { let p = &mut st.grab; p.normal_l2 = v.v()?; }
        ("grab", "normalR2") => { let p = &mut st.grab; p.normal_r2 = v.v()?; }
        ("grab", "orientationConstraintScale") => { let p = &mut st.grab; p.orientation_scale = v.f()?; }
        ("grab", "maxWristAngle") => { let p = &mut st.grab; p.max_wrist_angle = v.f()?; }
        ("grab", "useHeadLookToTarget") => { let p = &mut st.grab; p.head_look_to_target = v.b()?; }
        ("grab", "lookAtGrab") => { let p = &mut st.grab; p.look_at_grab = v.b()?; }
        ("grab", "targetForHeadLook") => { let p = &mut st.grab; p.head_look_target = v.v()?; }
        ("headLook", "pos") => { let p = &mut st.head_look; p.pos = v.v()?; }
        ("headLook", "stiffness") => { let p = &mut st.head_look; p.stiffness = v.f()?; }
        ("headLook", "keepHeadAwayFromGround") => { let p = &mut st.head_look; p.keep_head_away = v.b()?; }
        ("headLook", "twistSpine") => { let p = &mut st.head_look; p.twist_spine = v.b()?; }
        ("headLook", "alwaysEyesHorizontal") => { let p = &mut st.head_look; p.always_eyes_horizontal = v.b()?; }
        ("headLook", "alwaysLook") => { let p = &mut st.head_look; p.always_look = v.b()?; }
        ("headLook", "vel") => { let p = &mut st.head_look; p.vel = v.v()?; }
        ("headLook", "damping") => { let p = &mut st.head_look; p.damping = v.f()?; }
        ("highFall", "bodyStiffness") => { let p = &mut st.high_fall; p.body_stiffness = v.f()?; }
        ("highFall", "armAmplitude") => { let p = &mut st.high_fall; p.arm_amplitude = v.f()?; }
        ("highFall", "armPeriod") => { let p = &mut st.high_fall; p.arm_period = v.f()?; }
        ("highFall", "pdStrength") => { let p = &mut st.high_fall; p.pd_strength = v.f()?; }
        ("highFall", "armsUp") => { let p = &mut st.high_fall; p.arms_up = v.f()?; }
        ("highFall", "legRadius") => { let p = &mut st.high_fall; p.leg_radius = v.f()?; }
        ("highFall", "bodydamping") => { let p = &mut st.high_fall; p.bodydamping = v.f()?; }
        ("highFall", "legAngSpeed") => { let p = &mut st.high_fall; p.leg_ang_speed = v.f()?; }
        ("highFall", "crashOrLandCutOff") => { let p = &mut st.high_fall; p.crash_or_land_cut_off = v.f()?; }
        ("highFall", "fowardOffsetOfLegIK") => { let p = &mut st.high_fall; p.foward_offset_of_leg_ik = v.f()?; }
        ("highFall", "catchfalltime") => { let p = &mut st.high_fall; p.catchfalltime = v.f()?; }
        ("highFall", "landCutOffVelocity") => { let p = &mut st.high_fall; p.crash_or_land_cut_off = v.f()?; }
        ("highFall", "ignorWorldCollisions") => { let p = &mut st.high_fall; p.ignor_world_collisions = v.b()?; }
        ("highFall", "fowardRoll") => { let p = &mut st.high_fall; p.foward_roll = v.b()?; }
        ("highFall", "orientateBodyToFallDirection") => { let p = &mut st.high_fall; p.orientate_body_to_fall_direction = v.b()?; }
        ("highFall", "aimAngleBase") => { let p = &mut st.high_fall; p.aim_angle_base = v.f()?; }
        ("highFall", "fowardVelRotation") => { let p = &mut st.high_fall; p.foward_vel_rotation = v.f()?; }
        ("highFall", "sideD") => { let p = &mut st.high_fall; p.side_d = v.f()?; }
        ("highFall", "catchFallCutOff") => { let p = &mut st.high_fall; p.catch_fall_cut_off = v.f()?; }
        ("highFall", "footVelCompScale") => { let p = &mut st.high_fall; p.foot_vel_comp_scale = v.f()?; }
        ("highFall", "legL") => { let p = &mut st.high_fall; p.leg_l = v.f()?; }
        ("highFall", "fowardVelCutOff") => { let p = &mut st.high_fall; p.foward_vel_cut_off = v.f()?; }
        ("highFall", "legStrength") => { let p = &mut st.high_fall; p.leg_strength = v.f()?; }
        ("highFall", "zeroPoseRoll") => { let p = &mut st.high_fall; p.zero_pose_roll = v.b()?; }
        ("leanInDirection", "leanAmount") => { let p = &mut st.lean_in_direction; p.lean_amount = v.f()?; }
        ("leanInDirection", "applyAsForce") => { let p = &mut st.lean_in_direction; p.apply_as_force = v.b()?; }
        ("leanInDirection", "dir") => { let p = &mut st.lean_in_direction; p.dir = v.v()?; }
        ("leanToPosition", "leanAmount") => { let p = &mut st.lean_to_position; p.lean_amount = v.f()?; }
        ("leanToPosition", "applyAsForce") => { let p = &mut st.lean_to_position; p.apply_as_force = v.b()?; }
        ("leanToPosition", "pos") => { let p = &mut st.lean_to_position; p.pos = v.v()?; }
        ("leanTowardsObject", "leanAmount") => { let p = &mut st.lean_towards_object; p.lean_amount = v.f()?; }
        ("leanTowardsObject", "applyAsForce") => { let p = &mut st.lean_towards_object; p.apply_as_force = v.b()?; }
        ("leanTowardsObject", "offset") => { let p = &mut st.lean_towards_object; p.offset = v.v()?; }
        ("pedalLegs", "legStiffness") => { let p = &mut st.pedal_legs; p.leg_stiffness = v.f()?; }
        ("pedalLegs", "pedalRightLeg") => { let p = &mut st.pedal_legs; p.right = v.b()?; }
        ("pedalLegs", "speedAsymmetry") => { let p = &mut st.pedal_legs; p.speed_asymmetry = v.f()?; }
        ("pedalLegs", "pedalLeftLeg") => { let p = &mut st.pedal_legs; p.left = v.b()?; }
        ("pedalLegs", "radius") => { let p = &mut st.pedal_legs; p.radius = v.f()?; }
        ("pedalLegs", "angularSpeed") => { let p = &mut st.pedal_legs; p.angular_speed = v.f()?; }
        ("pedalLegs", "pedalOffset") => { let p = &mut st.pedal_legs; p.pedal_offset = v.f()?; }
        ("pedalLegs", "angSpeedMultiplier4Dragging") => { let p = &mut st.pedal_legs; p.drag_multiplier = v.f()?; }
        ("pedalLegs", "adaptivePedal4Dragging") => { let p = &mut st.pedal_legs; p.adaptive = v.b()?; }
        ("pedalLegs", "backPedal") => { let p = &mut st.pedal_legs; p.back_pedal = v.b()?; }
        ("pedalLegs", "radiusVariance") => { let p = &mut st.pedal_legs; p.radius_variance = v.f()?; }
        ("pedalLegs", "centreUp") => { let p = &mut st.pedal_legs; p.centre_up = v.f()?; }
        ("pedalLegs", "centreForwards") => { let p = &mut st.pedal_legs; p.centre_forwards = v.f()?; }
        ("pedalLegs", "legAngleVariance") => { let p = &mut st.pedal_legs; p.leg_angle_variance = v.f()?; }
        ("pedalLegs", "randomSeed") => { let p = &mut st.pedal_legs; p.random_seed = v.i()?.max(0) as u32; }
        ("pedalLegs", "centresideways") => { let p = &mut st.pedal_legs; p.rdr2.centre_sideways = v.f()?; }
        ("pedalLegs", "ellipse") => { let p = &mut st.pedal_legs; p.rdr2.ellipse = v.f()?; }
        ("pedalLegs", "spread") => { let p = &mut st.pedal_legs; p.rdr2.spread = v.f()?; }
        ("pedalLegs", "hula") => { let p = &mut st.pedal_legs; p.rdr2.hula = v.b()?; }
        ("pointArm", "instanceIndex") => { let p = &mut st.point_arm; p.instance_index = v.i()?; }
        ("pointArm", "useLeftArm") => { let p = &mut st.point_arm; p.use_left_arm = v.b()?; if let Some(r) = p.rdr2.as_mut() { r[0].use_arm = p.use_left_arm; } }
        ("pointArm", "targetLeft") => { let p = &mut st.point_arm; p.rdr2.get_or_insert_with(Default::default)[0].target = v.v()?; }
        ("pointArm", "targetRight") => { let p = &mut st.point_arm; p.rdr2.get_or_insert_with(Default::default)[1].target = v.v()?; }
        ("pointArm", "twistLeft") => { let p = &mut st.point_arm; p.rdr2.get_or_insert_with(Default::default)[0].twist = v.f()?; }
        ("pointArm", "twistRight") => { let p = &mut st.point_arm; p.rdr2.get_or_insert_with(Default::default)[1].twist = v.f()?; }
        ("pointArm", "armStraightnessLeft") => { let p = &mut st.point_arm; p.rdr2.get_or_insert_with(Default::default)[0].arm_straightness = v.f()?; }
        ("pointArm", "armStraightnessRight") => { let p = &mut st.point_arm; p.rdr2.get_or_insert_with(Default::default)[1].arm_straightness = v.f()?; }
        ("pointArm", "armStiffnessLeft") => { let p = &mut st.point_arm; p.rdr2.get_or_insert_with(Default::default)[0].arm_stiffness = v.f()?; }
        ("pointArm", "armStiffnessRight") => { let p = &mut st.point_arm; p.rdr2.get_or_insert_with(Default::default)[1].arm_stiffness = v.f()?; }
        ("pointArm", "armDampingLeft") => { let p = &mut st.point_arm; p.rdr2.get_or_insert_with(Default::default)[0].arm_damping = v.f()?; }
        ("pointArm", "armDampingRight") => { let p = &mut st.point_arm; p.rdr2.get_or_insert_with(Default::default)[1].arm_damping = v.f()?; }
        ("pointArm", "pointSwingLimitLeft") => { let p = &mut st.point_arm; p.rdr2.get_or_insert_with(Default::default)[0].point_swing_limit = v.f()?; }
        ("pointArm", "pointSwingLimitRight") => { let p = &mut st.point_arm; p.rdr2.get_or_insert_with(Default::default)[1].point_swing_limit = v.f()?; }
        ("pointArm", "useRightArm") => { let p = &mut st.point_arm; p.rdr2.get_or_insert_with(Default::default)[1].use_arm = v.b()?; }
        ("pointArm", "useZeroPoseWhenNotPointingLeft") => { let p = &mut st.point_arm; p.rdr2.get_or_insert_with(Default::default)[0].use_zero_pose = v.b()?; }
        ("pointArm", "useZeroPoseWhenNotPointingRight") => { let p = &mut st.point_arm; p.rdr2.get_or_insert_with(Default::default)[1].use_zero_pose = v.b()?; }
        ("pointArm", "twist") => { let p = &mut st.point_arm; p.twist = v.f()?; }
        ("pointArm", "target") => { let p = &mut st.point_arm; p.target = v.v()?; }
        ("pointArm", "pointSwingLimit") => { let p = &mut st.point_arm; p.point_swing_limit = v.f()?; }
        ("pointArm", "armStraightness") => { let p = &mut st.point_arm; p.arm_straightness = v.f()?; }
        ("pointArm", "useZeroPoseWhenNotPointing") => { let p = &mut st.point_arm; p.use_zero_pose_when_not_pointing = v.b()?; }
        ("pointArm", "armDamping") => { let p = &mut st.point_arm; p.arm_damping = v.f()?; }
        ("pointArm", "armStiffness") => { let p = &mut st.point_arm; p.arm_stiffness = v.f()?; }
        ("rollDownStairs", "forcemag") => { let p = &mut st.roll_down_stairs; p.forcemag = v.f()?; }
        ("rollDownStairs", "armReachLength") => { let p = &mut st.roll_down_stairs; p.arm_reach_length = v.f()?; }
        ("rollDownStairs", "m_legPush") => { let p = &mut st.roll_down_stairs; p.leg_push = v.f()?; }
        ("rollDownStairs", "useCustomRollDir") => { let p = &mut st.roll_down_stairs; p.use_custom_roll_dir = v.b()?; }
        ("rollDownStairs", "stiffnessDecayTarget") => { let p = &mut st.roll_down_stairs; p.stiffness_decay_target = v.f()?; }
        ("rollDownStairs", "spinWhenInAir") => { let p = &mut st.roll_down_stairs; p.spin_when_in_air = v.b()?; }
        ("rollDownStairs", "m_armReachAmount") => { let p = &mut st.roll_down_stairs; p.arm_reach_amount = v.f()?; }
        ("rollDownStairs", "customRollDir") => { let p = &mut st.roll_down_stairs; p.custom_roll_dir = v.v()?; }
        ("rollDownStairs", "stiffnessDecayTime") => { let p = &mut st.roll_down_stairs; p.stiffness_decay_time = v.f()?; }
        ("rollDownStairs", "m_useArmToSlowDown") => { let p = &mut st.roll_down_stairs; p.use_arm_to_slow_down = v.f()?; }
        ("rollDownStairs", "stiffness") => { let p = &mut st.roll_down_stairs; p.stiffness = v.f()?; }
        ("rollDownStairs", "damping") => { let p = &mut st.roll_down_stairs; p.damping = v.f()?; }
        ("rollDownStairs", "useZeroPose") => { let p = &mut st.roll_down_stairs; p.use_zero_pose = v.b()?; }
        ("rollDownStairs", "zAxisSpinReduction") => { let p = &mut st.roll_down_stairs; p.z_axis_spin_reduction = v.f()?; }
        ("rollDownStairs", "asymmetricalLegs") => { let p = &mut st.roll_down_stairs; p.asymmetrical_legs = v.f()?; }
        ("rollDownStairs", "asymmetry") => { let p = &mut st.roll_down_stairs; p.asymmetry = v.f()?; }
        ("rollDownStairs", "muscleStiffness") => { let p = &mut st.roll_down_stairs; p.rdr2.muscle_stiffness = v.f()?; }
        ("rollDownStairs", "onlyApplyHelperForces") => { let p = &mut st.roll_down_stairs; p.helper_only = v.b()?; }
        ("rollDownStairs", "useRelativeVelocity") => { let p = &mut st.roll_down_stairs; p.rdr2.use_relative_velocity = v.b()?; }
        ("rollDownStairs", "maxAngVelAroundFrontwardAxis") => { let p = &mut st.roll_down_stairs; p.rdr2.max_ang_vel_around_frontward_axis = v.f()?; }
        ("rollDownStairs", "applyNewRollingCheatingTorques") => { let p = &mut st.roll_down_stairs; p.rdr2.cheating_torques = v.b()?; }
        ("rollDownStairs", "magOfTorqueToRoll") => { let p = &mut st.roll_down_stairs; p.rdr2.mag_of_torque_to_roll = v.f()?; }
        ("rollDownStairs", "maxAngVel") => { let p = &mut st.roll_down_stairs; p.rdr2.max_ang_vel = v.f()?; }
        ("rollDownStairs", "minAngVel") => { let p = &mut st.roll_down_stairs; p.rdr2.min_ang_vel = v.f()?; }
        ("rollDownStairs", "applyHelPerTorqueToAlign") => { let p = &mut st.roll_down_stairs; p.rdr2.align = v.b()?; }
        ("rollDownStairs", "delayToAlignBody") => { let p = &mut st.roll_down_stairs; p.rdr2.delay_to_align_body = v.f()?; }
        ("rollDownStairs", "magOfTorqueToAlign") => { let p = &mut st.roll_down_stairs; p.rdr2.mag_of_torque_to_align = v.f()?; }
        ("rollDownStairs", "airborneReduction") => { let p = &mut st.roll_down_stairs; p.rdr2.airborne_reduction = v.f()?; }
        ("rollDownStairs", "applyFoetalToLegs") => { let p = &mut st.roll_down_stairs; p.rdr2.foetal_legs = v.b()?; }
        ("rollDownStairs", "movementLegsInFoetalPosition") => { let p = &mut st.roll_down_stairs; p.rdr2.movement_legs_in_foetal_position = v.f()?; }
        ("rollDownStairs", "limitSpinReduction") => { let p = &mut st.roll_down_stairs; p.rdr2.limit_spin_reduction = v.b()?; }
        ("rollDownStairs", "targetLinearVelocity") => { let p = &mut st.roll_down_stairs; p.rdr2.target_linear_velocity = v.f()?; }
        ("rollDownStairs", "targetLinearVelocityDecayTime") => { let p = &mut st.roll_down_stairs; p.rdr2.target_linear_velocity_decay_time = v.f()?; }
        ("setStiffness", "bodyStiffness") => { let p = &mut st.set_stiffness; p.body_stiffness = v.f()?; }
        ("setStiffness", "mask") => { let p = &mut st.set_stiffness; p.mask = v.s()?.to_string(); }
        ("setStiffness", "damping") => { let p = &mut st.set_stiffness; p.damping = v.f()?; }
        ("shot", "bodyStiffness") => { let p = &mut st.shot; p.body_stiffness = v.f()?; }
        ("shot", "exagDuration") => { let p = &mut st.shot; p.rdr2.exag_duration = v.f()?; }
        ("shot", "exagMag") => { let p = &mut st.shot; p.rdr2.exag_mag = v.f()?; }
        ("shot", "exagTwistMag") => { let p = &mut st.shot; p.rdr2.exag_twist_mag = v.f()?; }
        ("shot", "exagUndoRate") => { let p = &mut st.shot; p.rdr2.exag_undo_rate = v.f()?; }
        ("shot", "cpainStart") => { let p = &mut st.shot; p.rdr2.cpain_start = v.f()?; }
        ("shot", "cpainRate") => { let p = &mut st.shot; p.rdr2.cpain_rate = v.f()?; }
        ("shot", "cpainDuration") => { let p = &mut st.shot; p.rdr2.cpain_duration = v.f()?; }
        ("shot", "cpainMag") => { let p = &mut st.shot; p.rdr2.cpain_mag = v.f()?; }
        ("shot", "cpainTwistMag") => { let p = &mut st.shot; p.rdr2.cpain_twist_mag = v.f()?; }
        ("shot", "cpainUndoRate") => { let p = &mut st.shot; p.rdr2.cpain_undo_rate = v.f()?; }
        ("shot", "armStiffness") => { let p = &mut st.shot; p.rdr2.arm_stiffness = v.f()?; }
        ("shot", "spineDamping") => { let p = &mut st.shot; p.rdr2.spine_damping = v.f()?; }
        ("shot", "initialNeckStiffness") => { let p = &mut st.shot; p.rdr2.initial_neck_stiffness = v.f()?; }
        ("shot", "initialNeckDamping") => { let p = &mut st.shot; p.rdr2.initial_neck_damping = v.f()?; }
        ("shot", "neckStiffness") => { let p = &mut st.shot; p.rdr2.neck_stiffness = v.f()?; }
        ("shot", "neckDamping") => { let p = &mut st.shot; p.rdr2.neck_damping = v.f()?; }
        ("shot", "kMultOnLoose") => { let p = &mut st.shot; p.rdr2.k_mult_on_loose = v.f()?; }
        ("shot", "kMult4Legs") => { let p = &mut st.shot; p.rdr2.k_mult_4_legs = v.f()?; }
        ("shot", "initialWeaknessRampDuration") => { let p = &mut st.shot; p.rdr2.initial_weakness_ramp = v.f()?; }
        ("shot", "initialWeaknessZeroDuration") => { let p = &mut st.shot; p.rdr2.initial_weakness_zero = v.f()?; }
        ("shot", "initialNeckRampDuration") => { let p = &mut st.shot; p.rdr2.initial_neck_ramp = v.f()?; }
        ("shot", "initialNeckDuration") => { let p = &mut st.shot; p.rdr2.initial_neck_duration = v.f()?; }
        ("shot", "spineBlendZero") => { let p = &mut st.shot; p.rdr2.spine_blend_zero = v.f()?; }
        ("shot", "minArmsLooseness") => { let p = &mut st.shot; p.rdr2.min_arms_looseness = v.f()?; }
        ("shot", "minLegsLooseness") => { let p = &mut st.shot; p.rdr2.min_legs_looseness = v.f()?; }
        ("shot", "looseness4Stagger") => { let p = &mut st.shot; p.rdr2.looseness4_stagger = v.f()?; }
        ("shot", "angVelScale") => { let p = &mut st.shot; p.rdr2.ang_vel_scale = v.f()?; }
        ("shot", "angVelScaleMask") => { let p = &mut st.shot; p.rdr2.ang_vel_mask = v.s()?.as_bytes().first().copied().zip(v.s()?.as_bytes().get(1).copied()).map_or(*b"fb", |(a, b)| [a, b]); }
        ("shot", "useCStrModulation") => { let p = &mut st.shot; p.rdr2.use_cstr = v.b()?; }
        ("shot", "looseness4Fall") => { let p = &mut st.shot; p.rdr2.looseness4_fall = v.f()?; }
        ("shot", "PointGun") => { let p = &mut st.shot; p.rdr2.point_gun = v.b()?; }
        ("shot", "cStrUpperMin") => { let p = &mut st.shot; p.rdr2.cstr_upper_min = v.f()?; }
        ("shot", "cStrUpperMax") => { let p = &mut st.shot; p.rdr2.cstr_upper_max = v.f()?; }
        ("shot", "cStrLowerMin") => { let p = &mut st.shot; p.rdr2.cstr_lower_min = v.f()?; }
        ("shot", "cStrLowerMax") => { let p = &mut st.shot; p.rdr2.cstr_lower_max = v.f()?; }
        ("shot", "bulletProofVest") => { let p = &mut st.shot; p.rdr2.bullet_proof_vest = v.b()?; }
        ("shot", "alwaysResetLooseness") => { let p = &mut st.shot; p.rdr2.always_reset_looseness = v.b()?; }
        ("shot", "alwaysResetNeckLooseness") => { let p = &mut st.shot; p.rdr2.always_reset_neck = v.b()?; }
        ("shot", "spineBlendExagCPain") => { let p = &mut st.shot; p.rdr2.spine_blend_exag_cpain = v.b()?; }
        ("shot", "floorWeaknessTime") => { let p = &mut st.shot; p.rdr2.floor_weakness_time = v.f()?; }
        ("shot", "floorStrength") => { let p = &mut st.shot; p.rdr2.floor_strength = v.f()?; }
        ("shot", "floorJointWeaknessTime") => { let p = &mut st.shot; p.rdr2.floor_joint_time = v.f()?; }
        ("shot", "floorJointStrength") => { let p = &mut st.shot; p.rdr2.floor_joint_strength = v.f()?; }
        ("shot", "fallingReaction") => { let p = &mut st.shot; p.rdr2.falling_reaction = Some(v.i()?); }
        ("shot", "bracedSideSpinMult") => { let p = &mut st.shot; p.rdr2.braced_side_spin_mult = v.f()?; }
        ("shot", "shockSpinMaxTwistVel") => { let p = &mut st.shot; p.rdr2.shock_spin_max_twist_vel = v.f()?; }
        ("shot", "shockSpinAirMult") => { let p = &mut st.shot; p.rdr2.shock_spin_air_mult = v.f()?; }
        ("shot", "shockSpin1FootMult") => { let p = &mut st.shot; p.rdr2.shock_spin_one_foot_mult = v.f()?; }
        ("shot", "shockSpinScaleByLeverArm") => { let p = &mut st.shot; p.rdr2.shock_spin_scale_by_lever_arm = v.b()?; }
        ("shot", "fling2") => { let p = &mut st.shot; p.rdr2.fling2 = v.b()?; }
        ("shot", "fling2Left") => { let p = &mut st.shot; p.rdr2.fling2_side[0] = v.b()?; }
        ("shot", "fling2Right") => { let p = &mut st.shot; p.rdr2.fling2_side[1] = v.b()?; }
        ("shot", "fling2OverrideStagger") => { let p = &mut st.shot; p.rdr2.fling2_override_stagger = v.b()?; }
        ("shot", "fling2TimeBefore") => { let p = &mut st.shot; p.rdr2.fling2_before = v.f()?; }
        ("shot", "fling2Time") => { let p = &mut st.shot; p.rdr2.fling2_time = v.f()?; }
        ("shot", "fling2MStiffL") => { let p = &mut st.shot; p.rdr2.fling2_mstiff[0] = v.f()?; }
        ("shot", "fling2MStiffR") => { let p = &mut st.shot; p.rdr2.fling2_mstiff[1] = v.f()?; }
        ("shot", "fling2StiffL") => { let p = &mut st.shot; p.rdr2.fling2_stiff[0] = v.f()?; }
        ("shot", "fling2StiffR") => { let p = &mut st.shot; p.rdr2.fling2_stiff[1] = v.f()?; }
        ("shot", "fling2DampL") => { let p = &mut st.shot; p.rdr2.fling2_damp[0] = v.f()?; }
        ("shot", "fling2DampR") => { let p = &mut st.shot; p.rdr2.fling2_damp[1] = v.f()?; }
        ("shot", "fling2RelaxTimeL") => { let p = &mut st.shot; p.rdr2.fling2_relax[0] = v.f()?; }
        ("shot", "fling2RelaxTimeR") => { let p = &mut st.shot; p.rdr2.fling2_relax[1] = v.f()?; }
        ("shot", "fling2AngleMinL") => { let p = &mut st.shot; p.rdr2.fling2_angle[0][0] = v.f()?; }
        ("shot", "fling2AngleMaxL") => { let p = &mut st.shot; p.rdr2.fling2_angle[0][1] = v.f()?; }
        ("shot", "fling2AngleMinR") => { let p = &mut st.shot; p.rdr2.fling2_angle[1][0] = v.f()?; }
        ("shot", "fling2AngleMaxR") => { let p = &mut st.shot; p.rdr2.fling2_angle[1][1] = v.f()?; }
        ("shot", "fling2LengthMinL") => { let p = &mut st.shot; p.rdr2.fling2_length[0][0] = v.f()?; }
        ("shot", "fling2LengthMaxL") => { let p = &mut st.shot; p.rdr2.fling2_length[0][1] = v.f()?; }
        ("shot", "fling2LengthMinR") => { let p = &mut st.shot; p.rdr2.fling2_length[1][0] = v.f()?; }
        ("shot", "fling2LengthMaxR") => { let p = &mut st.shot; p.rdr2.fling2_length[1][1] = v.f()?; }
        ("shot", "enableDefaultArms") => { let p = &mut st.shot; p.rdr2.default_arms = v.b()?; }
        ("shot", "brace") => { let p = &mut st.shot; p.rdr2.brace = v.b()?; }
        ("shot", "useArmsWindmill") => { let p = &mut st.shot; p.rdr2.use_arms_windmill = v.b()?; }
        ("shot", "alwaysReachTime") => { let p = &mut st.shot; p.rdr2.always_reach_time = v.f()?; }
        ("shot", "AWSpeedMult") => { let p = &mut st.shot; p.rdr2.aw_speed_mult = v.f()?; }
        ("shot", "AWRadiusMult") => { let p = &mut st.shot; p.rdr2.aw_radius_mult = v.f()?; }
        ("shot", "AWStiffnessAdd") => { let p = &mut st.shot; p.rdr2.aw_stiffness_add = v.f()?; }
        ("shot", "releaseWound") => { let p = &mut st.shot; p.rdr2.release_wound = v.i()?; }
        ("shot", "grabHoldTime") => { let p = &mut st.shot; p.grab_hold_time = v.f()?; }
        ("shot", "bodyPart") => { let p = &mut st.shot; p.body_part = v.i()?.max(0) as usize; }
        ("shot", "loosenessAmount") => { let p = &mut st.shot; p.looseness_amount = v.f()?; }
        ("shot", "shockSpinDecayMult") => { let p = &mut st.shot; p.shock_spin_decay_mult = v.f()?; }
        ("shot", "timeBeforeReachForWound") => { let p = &mut st.shot; p.time_before_reach_for_wound = v.f()?; }
        ("shot", "headLookAtWoundMaxTimer") => { let p = &mut st.shot; p.head_look_at_wound_max_timer = v.f()?; }
        ("shot", "reachForWound") => { let p = &mut st.shot; p.reach_for_wound = v.b()?; }
        ("shot", "spinePainTwistMultiplier") => { let p = &mut st.shot; p.spine_pain_twist_multiplier = v.f()?; }
        ("shot", "useHeadLook") => { let p = &mut st.shot; p.use_head_look = v.b()?; }
        ("shot", "shockSpinMin") => { let p = &mut st.shot; p.shock_spin_min = v.f()?; }
        ("shot", "stableHandsAndNeck") => { let p = &mut st.shot; p.stable_hands_and_neck = v.b()?; }
        ("shot", "maxSteps") => { let p = &mut st.shot; p.max_steps = v.i()?; }
        ("shot", "shockSpinMax") => { let p = &mut st.shot; p.shock_spin_max = v.f()?; }
        ("shot", "timeBeforeCollapseWoundLeg") => { let p = &mut st.shot; p.time_before_collapse_wound_leg = v.f()?; }
        ("shot", "melee") => { let p = &mut st.shot; p.melee = v.b()?; }
        ("shot", "shockSpinScalePerComponent") => { let p = &mut st.shot; p.shock_spin_scale_per_component = v.f()?; }
        ("shot", "randomizeShockSpinDirection") => { let p = &mut st.shot; p.randomize_shock_spin_direction = v.b()?; }
        ("shot", "useCatchFallOnFall") => { let p = &mut st.shot; p.use_catch_fall_on_fall = v.b()?; }
        ("shot", "crouching") => { let p = &mut st.shot; p.crouching = v.b()?; }
        ("shot", "headLookAtWoundMinTimer") => { let p = &mut st.shot; p.head_look_at_wound_min_timer = v.f()?; }
        ("shot", "hitPoint") => { let p = &mut st.shot; p.hit_point = v.v()?; }
        ("shot", "shockSpinLiftForceMult") => { let p = &mut st.shot; p.shock_spin_lift_force_mult = v.f()?; }
        ("shot", "chickenArms") => { let p = &mut st.shot; p.chicken_arms = v.b()?; }
        ("shot", "fling") => { let p = &mut st.shot; p.fling = v.b()?; }
        ("shot", "normal") => { let p = &mut st.shot; p.normal = v.v()?; }
        ("shot", "spinePainTime") => { let p = &mut st.shot; p.spine_pain_time = v.f()?; }
        ("shot", "headLookAtHeadPosMinTimer") => { let p = &mut st.shot; p.head_look_at_head_pos_min_timer = v.f()?; }
        ("shot", "bulletVel") => { let p = &mut st.shot; p.bullet_vel = v.v()?; }
        ("shot", "addShockSpin") => { let p = &mut st.shot; p.add_shock_spin = v.b()?; }
        ("shot", "spinePainMultiplier") => { let p = &mut st.shot; p.spine_pain_multiplier = v.f()?; }
        ("shot", "headLookAtHeadPosMaxTimer") => { let p = &mut st.shot; p.head_look_at_head_pos_max_timer = v.f()?; }
        ("shot", "alwaysAddShockSpin") => { let p = &mut st.shot; p.always_add_shock_spin = v.b()?; }
        ("shot", "flingWidth") => { let p = &mut st.shot; p.fling_width = v.f()?; }
        ("shot", "headLook") => { let p = &mut st.shot; p.head_look = v.v()?; }
        ("shotRelax", "relaxPeriod") => { let p = &mut st.shot_relax; p.relax_period = v.f()?; }
        ("shotRelax", "relaxPeriodUpper") => { let p = &mut st.shot_relax; p.upper = v.f()?; }
        ("shotRelax", "relaxPeriodLower") => { let p = &mut st.shot_relax; p.lower = v.f()?; }
        ("stayUpright", "stiffness") => { let p = &mut st.stay_upright; p.stiffness = v.f()?; }
        ("stayUpright", "dampingScale") => { let p = &mut st.stay_upright; p.damping_scale = v.f()?; }
        ("stayUpright", "side") => st.stay_upright.side = Some(v.i()?),
        ("stayUpright", "torqueStrength") => st.stay_upright.rdr2.torque_strength = v.v()?,
        ("stayUpright", "torqueDamping") => st.stay_upright.rdr2.torque_damping = v.v()?,
        ("stayUpright", n) => {
            let x = if crate::nmdbm_rdr2::StayUpright::is_flag(n) { v.b()? as u8 as f32 } else { v.f()? };
            if !st.stay_upright.rdr2.set(n, x) {
                return Err(Error::Param);
            }
        }
        ("upperBodyFlinch", "handDistanceFrontBack") => { let p = &mut st.upper_body_flinch; p.hand_distance_front_back = v.f()?; }
        ("upperBodyFlinch", "noiseScale") => { let p = &mut st.upper_body_flinch; p.noise_scale = v.f()?; }
        ("upperBodyFlinch", "handDistanceVertical") => { let p = &mut st.upper_body_flinch; p.hand_distance_vertical = v.f()?; }
        ("upperBodyFlinch", "useRightArm") => { let p = &mut st.upper_body_flinch; p.use_right_arm = v.b()?; }
        ("upperBodyFlinch", "handDistanceLeftRight") => { let p = &mut st.upper_body_flinch; p.hand_distance_left_right = v.f()?; }
        ("upperBodyFlinch", "rightHanded") => { let p = &mut st.upper_body_flinch; p.right_handed = v.b()?; }
        ("upperBodyFlinch", "turnTowards") => { let p = &mut st.upper_body_flinch; p.turn_towards = v.i()?; }
        ("upperBodyFlinch", "backBendAmount") => { let p = &mut st.upper_body_flinch; p.back_bend_amount = v.f()?; }
        ("upperBodyFlinch", "newHit") => { let p = &mut st.upper_body_flinch; p.new_hit = Some(v.b()?); }
        ("upperBodyFlinch", "leftHanded") => { let p = &mut st.upper_body_flinch; p.left_handed = v.b()?; }
        ("upperBodyFlinch", "bodyDamping") => { let p = &mut st.upper_body_flinch; p.body_damping = v.f()?; }
        ("upperBodyFlinch", "useLeftArm") => { let p = &mut st.upper_body_flinch; p.use_left_arm = v.b()?; }
        ("upperBodyFlinch", "pos") => { let p = &mut st.upper_body_flinch; p.pos = v.v()?; }
        ("upperBodyFlinch", "bodyStiffness") => { let p = &mut st.upper_body_flinch; p.body_stiffness = v.f()?; }
        ("upperBodyFlinch", "protectHeadToggle") => { let p = &mut st.upper_body_flinch; p.protect_head_toggle = v.b()?; }
        ("upperBodyFlinch", "applyStiffness") => { let p = &mut st.upper_body_flinch; p.apply_stiffness = v.b()?; }
        ("upperBodyFlinch", "dontBraceHead") => { let p = &mut st.upper_body_flinch; p.dont_brace_head = v.b()?; }
        ("upperBodyFlinch", "headLookAwayFromTarget") => { let p = &mut st.upper_body_flinch; p.head_look_away = v.b()?; }
        ("upperBodyFlinch", "useHeadLook") => { let p = &mut st.upper_body_flinch; p.use_head_look = v.b()?; }
        _ => return Err(Error::Param),
    }
    Ok(())
}

pub fn get(st: &State, msg: &str, name: &str) -> Option<Value> {
    Some(match (msg, name) {
        ("activePose", "mask") => { let p = &st.active_pose; Value::Str(p.mask.clone().unwrap_or_else(|| "fb".into())) }
        ("applyImpulse", "partIndex") => { let p = &st.apply_impulse; Value::Int(p.part_index) }
        ("applyImpulse", "hitPoint") => { let p = &st.apply_impulse; Value::Vec3(p.hit_point.unwrap_or(Vec3::ZERO)) }
        ("applyImpulse", "impulse") => { let p = &st.apply_impulse; Value::Vec3(p.impulse) }
        ("applyImpulse", "equalizeAmount") => { let p = &st.apply_impulse; Value::Float(p.equalize_amount) }
        ("armsWindmillAdaptive", "bodyStiffness") => { let p = &st.arms_windmill_adaptive; Value::Float(p.body_stiffness) }
        ("armsWindmillAdaptive", "amplitude") => { let p = &st.arms_windmill_adaptive; Value::Float(p.amplitude) }
        ("armsWindmillAdaptive", "phase") => { let p = &st.arms_windmill_adaptive; Value::Float(p.phase) }
        ("armsWindmillAdaptive", "disableOnImpact") => { let p = &st.arms_windmill_adaptive; Value::Bool(p.disable_on_impact) }
        ("armsWindmillAdaptive", "period") => { let p = &st.arms_windmill_adaptive; Value::Float(p.period) }
        ("armsWindmillAdaptive", "armStiffness") => { let p = &st.arms_windmill_adaptive; Value::Float(p.arm_stiffness) }
        ("bodyBalance", "bodyStiffness") => { let p = &st.body_balance; Value::Float(p.body_stiffness) }
        ("bodyBalance", "headLookAtVelProb") => { let p = &st.body_balance; Value::Float(p.head_look_at_vel_prob) }
        ("bodyBalance", "armsOutStraightenElbows") => { let p = &st.body_balance; Value::Float(p.arms_out_straighten_elbows) }
        ("bodyBalance", "bendElbowsTime") => { let p = &st.body_balance; Value::Float(p.bend_elbows_time) }
        ("bodyBalance", "armStiffness") => { let p = &st.body_balance; Value::Float(p.body_stiffness) }
        ("bodyBalance", "armDamping") => { let p = &st.body_balance; Value::Float(p.damping) }
        ("bodyBalance", "elbow") => { let p = &st.body_balance; Value::Float(p.elbow) }
        ("bodyBalance", "angVelMultiplier") => { let p = &st.body_balance; Value::Vec3(p.ang_vel_multiplier) }
        ("bodyBalance", "angVelThreshold") => { let p = &st.body_balance; Value::Vec3(p.ang_vel_threshold) }
        ("bodyBalance", "returningToBalanceArmsOut") => { let p = &st.body_balance; Value::Float(p.returning_to_balance_arms_out) }
        ("bodyBalance", "somersaultAngleThreshold") => { let p = &st.body_balance; Value::Float(p.somersault_angle_threshold) }
        ("bodyBalance", "armsOutMinLean2") => { let p = &st.body_balance; Value::Float(p.arms_out_min_lean2) }
        ("bodyBalance", "sideSomersaultAngleThreshold") => { let p = &st.body_balance; Value::Float(p.side_somersault_angle_threshold) }
        ("bodyBalance", "bendElbowsGait") => { let p = &st.body_balance; Value::Float(p.bend_elbows_gait) }
        ("bodyBalance", "armsOutOnPush") => { let p = &st.body_balance; Value::Bool(p.arms_out_on_push) }
        ("bodyBalance", "elbowAngleOnContact") => { let p = &st.body_balance; Value::Float(p.elbow_angle_on_contact) }
        ("bodyBalance", "bodyTurnTimeout") => { let p = &st.body_balance; Value::Float(p.spine_damping) }
        ("bodyBalance", "turnOffProb") => { let p = &st.body_balance; Value::Float(p.turn_off_prob) }
        ("bodyBalance", "turn2VelProb") => { let p = &st.body_balance; Value::Float(p.turn2_vel_prob) }
        ("bodyBalance", "damping") => { let p = &st.body_balance; Value::Float(p.damping) }
        ("bodyBalance", "headLookPos") => { let p = &st.body_balance; Value::Vec3(p.head_look_pos) }
        ("bodyBalance", "sideSomersaultAngle") => { let p = &st.body_balance; Value::Float(p.side_somersault_angle) }
        ("bodyBalance", "headLookAlwaysTimeout") => { let p = &st.body_balance; Value::Float(p.spine_stiffness) }
        ("bodyBalance", "turnAwayProb") => { let p = &st.body_balance; Value::Float(p.turn_away_prob) }
        ("bodyBalance", "shoulder") => { let p = &st.body_balance; Value::Float(p.shoulder) }
        ("bodyBalance", "turnRightProb") => { let p = &st.body_balance; Value::Float(p.turn_right_prob) }
        ("bodyBalance", "turnLeftProb") => { let p = &st.body_balance; Value::Float(p.turn_left_prob) }
        ("bodyBalance", "somersaultAngle") => { let p = &st.body_balance; Value::Float(p.somersault_angle) }
        ("bodyBalance", "armsOutOnPushMultiplier") => { let p = &st.body_balance; Value::Float(p.arms_out_on_push_multiplier) }
        ("bodyBalance", "turn2TargetProb") => { let p = &st.body_balance; Value::Float(p.turn2_target_prob) }
        ("bodyBalance", "useBodyTurn") => { let p = &st.body_balance; Value::Bool(p.use_body_turn) }
        ("bodyBalance", "useHeadLook") => { let p = &st.body_balance; Value::Bool(p.use_head_look) }
        ("bodyBalance", "headLookInstanceIndex") => { let p = &st.body_balance; Value::Int(p.head_look_instance_index) }
        ("bodyBalance", "armsOutOnPushTimeout") => { let p = &st.body_balance; Value::Float(p.arms_out_on_push_timeout) }
        ("bodyBalance", "maxSteps") => { let p = &st.body_balance; Value::Int(p.max_steps as i32) }
        ("bodyBalance", "spineStiffness") => { let p = &st.body_balance; Value::Float(p.spine_stiffness) }
        ("bodyBalance", "spineDamping") => { let p = &st.body_balance; Value::Float(p.spine_damping) }
        ("bodyBalance", "shoulderL1") => { let p = &st.body_balance; Value::Float(p.rdr2.shoulder_l1) }
        ("bodyBalance", "shoulderL2") => { let p = &st.body_balance; Value::Float(p.rdr2.shoulder_l2) }
        ("bodyBalance", "hipL2ArmL2") => { let p = &st.body_balance; Value::Float(p.rdr2.hip_l2_arm_l2) }
        ("bodyBalance", "shoulderTwist") => { let p = &st.body_balance; Value::Float(p.rdr2.shoulder_twist) }
        ("bodyBalance", "airbornearms") => { let p = &st.body_balance; Value::Float(p.rdr2.airborne_arms) }
        ("bodyBalance", "axisweightssimulationblocked") => { let p = &st.body_balance; Value::Bool(p.rdr2.axis_weights_simulation_blocked) }
        ("bodyBalance", "moveRadius") => { let p = &st.body_balance; Value::Float(p.rdr2.move_radius) }
        ("bodyBalance", "blendToZeroPose") => { let p = &st.body_balance; Value::Bool(p.rdr2.zero_pose) }
        ("bodyBalance", "startbalancer") => { let p = &st.body_balance; Value::Bool(p.rdr2.start_balancer) }
        ("bodyBalance", "fallingReaction") => { let p = &st.body_balance; Value::Int(p.rdr2.falling_reaction) }
        ("bodyBalance", "gshelp") => { let p = &st.body_balance; Value::Bool(p.rdr2.gs_help) }
        ("bodyBalance", "?01d67b58") => { let p = &st.body_balance; Value::Bool(p.rdr2.turn_block) }
        ("bodyBalance", "leanamountstill") => { let p = &st.body_balance; Value::Float(p.rdr2.lean_still) }
        ("bodyBalance", "braceOffset") => { let p = &st.body_balance; Value::Float(p.rdr2.brace_offset) }
        ("bodyBalance", "timeToBackwardsBrace") => { let p = &st.body_balance; Value::Float(p.rdr2.back_brace) }
        ("bodyBalance", "braceDistance") => { let p = &st.body_balance; Value::Float(p.rdr2.brace_distance) }
        ("bodyBalance", "bumpweakness") => { let p = &st.body_balance; Value::Float(p.rdr2.bump_weak) }
        ("bodyBalance", "bumpweaknesstime") => { let p = &st.body_balance; Value::Float(p.rdr2.bump_time) }
        ("bodyBalance", "turnWithBumpRadius") => { let p = &st.body_balance; Value::Float(p.rdr2.bump_radius) }
        ("bodyBalance", "moveAmount") => { let p = &st.body_balance; Value::Float(p.rdr2.move_amount) }
        ("bodyBalance", "moveamountstill") => { let p = &st.body_balance; Value::Float(p.rdr2.move_amount_still) }
        ("bodyBalance", "moveWhenBracing") => { let p = &st.body_balance; Value::Bool(p.rdr2.move_when_bracing) }
        ("bodyFoetal", "mask") => { let p = &st.body_foetal; Value::Str(p.mask.to_string()) }
        ("bodyFoetal", "stiffness") => { let p = &st.body_foetal; Value::Float(p.stiffness) }
        ("bodyFoetal", "asymmetry") => { let p = &st.body_foetal; Value::Float(p.asymmetry) }
        ("bodyFoetal", "randomSeed") => { let p = &st.body_foetal; Value::Int(p.random_seed as i32) }
        ("bodyFoetal", "dampingFactor") => { let p = &st.body_foetal; Value::Float(p.damping_factor) }
        ("bodyFoetal", "backTwist") => { let p = &st.body_foetal; Value::Float(p.back_twist) }
        ("bodyRelax", "mask") => { let p = &st.body_relax; Value::Str(p.mask.clone().unwrap_or_else(|| "fb".into())) }
        ("bodyRelax", "holdPose") => { let p = &st.body_relax; Value::Bool(p.hold_pose) }
        ("bodyRelax", "relaxation") => { let p = &st.body_relax; Value::Float(p.relaxation) }
        ("bodyRelax", "damping") => { let p = &st.body_relax; Value::Float(p.damping.unwrap_or(1.0)) }
        ("bodyRollUp", "mask") => { let p = &st.body_roll_up; Value::Str(p.mask.unwrap_or("").to_string()) }
        ("bodyRollUp", "stiffness") => { let p = &st.body_roll_up; Value::Float(p.stiffness) }
        ("bodyRollUp", "legPush") => { let p = &st.body_roll_up; Value::Float(p.leg_push) }
        ("bodyRollUp", "armReachAmount") => { let p = &st.body_roll_up; Value::Float(p.arm_reach_amount) }
        ("bodyRollUp", "useArmToSlowDown") => { let p = &st.body_roll_up; Value::Float(p.use_arm_to_slow_down) }
        ("bodyRollUp", "asymmetricalLegs") => { let p = &st.body_roll_up; Value::Float(p.asymmetrical_legs) }
        ("bodyWrithe", "armStiffness") => { let p = &st.body_writhe; Value::Float(p.arm_stiffness) }
        ("bodyWrithe", "legStiffness") => { let p = &st.body_writhe; Value::Float(p.leg_stiffness) }
        ("bodyWrithe", "legPeriod") => { let p = &st.body_writhe; Value::Float(p.leg_period) }
        ("bodyWrithe", "elbowAmplitude") => { let p = &st.body_writhe; Value::Float(p.elbow_amplitude) }
        ("bodyWrithe", "legAmplitude") => { let p = &st.body_writhe; Value::Float(p.leg_amplitude) }
        ("bodyWrithe", "kneeAmplitude") => { let p = &st.body_writhe; Value::Float(p.knee_amplitude) }
        ("bodyWrithe", "backDamping") => { let p = &st.body_writhe; Value::Float(p.back_damping) }
        ("bodyWrithe", "backPeriod") => { let p = &st.body_writhe; Value::Float(p.back_period) }
        ("bodyWrithe", "rollOverFlag") => { let p = &st.body_writhe; Value::Bool(p.roll_over) }
        ("bodyWrithe", "backAmplitude") => { let p = &st.body_writhe; Value::Float(p.back_amplitude) }
        ("bodyWrithe", "backStiffness") => { let p = &st.body_writhe; Value::Float(p.back_stiffness) }
        ("bodyWrithe", "armDamping") => { let p = &st.body_writhe; Value::Float(p.arm_damping) }
        ("bodyWrithe", "legDamping") => { let p = &st.body_writhe; Value::Float(p.leg_damping) }
        ("bodyWrithe", "mask") => { let p = &st.body_writhe; Value::Str(p.mask.to_string()) }
        ("bodyWrithe", "armAmplitude") => { let p = &st.body_writhe; Value::Float(p.arm_amplitude) }
        ("bodyWrithe", "armPeriod") => { let p = &st.body_writhe; Value::Float(p.arm_period) }
        ("braceForImpact", "bodyStiffness") => { let p = &st.brace_for_impact; Value::Float(p.body_stiffness) }
        ("braceForImpact", "look") => { let p = &st.brace_for_impact; Value::Vec3(p.look) }
        ("braceForImpact", "grabReachAngle") => { let p = &st.brace_for_impact; Value::Float(p.grab_reach_angle) }
        ("braceForImpact", "grabHoldTimer") => { let p = &st.brace_for_impact; Value::Float(p.grab_hold_timer) }
        ("braceForImpact", "braceDistance") => { let p = &st.brace_for_impact; Value::Float(p.brace_distance) }
        ("braceForImpact", "targetPredictionTime") => { let p = &st.brace_for_impact; Value::Float(p.target_prediction_time) }
        ("braceForImpact", "grabDistance") => { let p = &st.brace_for_impact; Value::Float(p.grab_distance) }
        ("braceForImpact", "pos") => { let p = &st.brace_for_impact; Value::Vec3(p.pos) }
        ("braceForImpact", "reachAbsorbtionTime") => { let p = &st.brace_for_impact; Value::Float(p.reach_absorbtion_time) }
        ("braceForImpact", "grabDontLetGo") => { let p = &st.brace_for_impact; Value::Bool(p.grab_dont_let_go) }
        ("braceForImpact", "legStiffness") => { let p = &st.brace_for_impact; Value::Float(p.leg_stiffness) }
        ("braceForImpact", "grabStrength") => { let p = &st.brace_for_impact; Value::Float(p.grab_strength) }
        ("braceForImpact", "instanceIndex") => { let p = &st.brace_for_impact; Value::Int(p.instance_index) }
        ("braceForImpact", "hash_4f7c0f0b") => { let p = &st.brace_for_impact; Value::Float(p.behind_time) }
        ("catchFall", "useHeadLook") => { let p = &st.catch_fall; Value::Bool(p.use_head_look) }
        ("catchFall", "armsStiffness") => { let p = &st.catch_fall; Value::Float(p.arms_stiffness) }
        ("catchFall", "torsoStiffness") => { let p = &st.catch_fall; Value::Float(p.torso_stiffness) }
        ("catchFall", "legsStiffness") => { let p = &st.catch_fall; Value::Float(p.legs_stiffness) }
        ("catchFall", "forwardMaxArmOffset") => { let p = &st.catch_fall; Value::Float(p.forward_max_arm_offset) }
        ("catchFall", "zAxisSpinReduction") => { let p = &st.catch_fall; Value::Float(p.z_axis_spin_reduction) }
        ("catchFall", "backwardsMinArmOffset") => { let p = &st.catch_fall; Value::Float(p.backwards_min_arm_offset) }
        ("configureBalance", "legStiffness") => { let p = &st.configure_balance; Value::Float(p.leg_stiffness) }
        ("configureBalance", "stepHeight") => { let p = &st.configure_balance; Value::Float(p.step_height) }
        ("configureBalance", "stepClampScale") => { let p = &st.configure_balance; Value::Float(p.step_clamp_scale) }
        ("configureBalance", "balanceAbortThreshold") => { let p = &st.configure_balance; Value::Float(p.balance_abort_threshold) }
        ("configureBalance", "taperKneeStrength") => { let p = &st.configure_balance; Value::Bool(p.taper_knee_strength) }
        ("defineAttachedObject", "partIndex") => { let p = &st.define_attached_object; Value::Int(p.part_index) }
        ("defineAttachedObject", "worldPos") => { let p = &st.define_attached_object; Value::Vec3(p.world_pos) }
        ("defineAttachedObject", "objectMass") => { let p = &st.define_attached_object; Value::Float(p.object_mass) }
        ("fallOverWall", "bodyTwist") => { let p = &st.fall_over_wall; Value::Float(p.twist_max / delirium_data::kv!(189)) }
        ("fallOverWall", "useSpecifiedPoint") => { let p = &st.fall_over_wall; Value::Bool(p.use_specified_point) }
        ("fallOverWall", "maxDistanceFromPelToHitPoint") => { let p = &st.fall_over_wall; Value::Float(p.max_distance_from_pel_to_hit_point) }
        ("fallOverWall", "fallOverEdge") => { let p = &st.fall_over_wall; Value::Vec3(p.fall_over_edge) }
        ("fallOverWall", "fallOverPoint") => { let p = &st.fall_over_wall; Value::Vec3(p.fall_over_point) }
        ("fallOverWall", "magOfForce") => { let p = &st.fall_over_wall; Value::Float(p.mag_of_force) }
        ("fallOverWall", "bodyStiffness") => { let p = &st.fall_over_wall; Value::Float(p.body_stiffness) }
        ("fallOverWall", "damping") => { let p = &st.fall_over_wall; Value::Float(p.damping) }
        ("grab", "bodyStiffness") => { let p = &st.grab; Value::Float(p.body_stiffness) }
        ("grab", "instanceIndex") => { let p = &st.grab; Value::Int(p.instance) }
        ("grab", "instancePartIndex") => { let p = &st.grab; Value::Int(p.part) }
        ("grab", "armStiffness") => { let p = &st.grab; Value::Float(p.arm_stiffness) }
        ("grab", "grabStrength") => { let p = &st.grab; Value::Float(p.grab_strength) }
        ("grab", "useLeft") => { let p = &st.grab; Value::Bool(p.use_left) }
        ("grab", "normalR") => { let p = &st.grab; Value::Vec3(p.normal_r) }
        ("grab", "pos3") => { let p = &st.grab; Value::Vec3(p.pos[2]) }
        ("grab", "useRight") => { let p = &st.grab; Value::Bool(p.use_right) }
        ("grab", "pos1") => { let p = &st.grab; Value::Vec3(p.pos[0]) }
        ("grab", "pos2") => { let p = &st.grab; Value::Vec3(p.pos[1]) }
        ("grab", "pos4") => { let p = &st.grab; Value::Vec3(p.pos[delirium_data::kv!(0 int)]) }
        ("grab", "surfaceGrab") => { let p = &st.grab; Value::Bool(p.surface_grab) }
        ("grab", "oneSideReachAngle") => { let p = &st.grab; Value::Float(p.one_side_reach_angle) }
        ("grab", "normalL") => { let p = &st.grab; Value::Vec3(p.normal_l) }
        ("grab", "linegrab") => { let p = &st.grab; Value::Bool(p.linegrab) }
        ("grab", "grabDist") => { let p = &st.grab; Value::Float(p.grab_dist) }
        ("grab", "dontLetGo") => { let p = &st.grab; Value::Bool(p.dont_let_go) }
        ("grab", "justbrace") => { let p = &st.grab; Value::Bool(p.justbrace) }
        ("grab", "reachAngle") => { let p = &st.grab; Value::Float(p.reach_angle) }
        ("grab", "pullUpStrengthRight") => { let p = &st.grab; Value::Float(p.pull_up_strength[1]) }
        ("grab", "pullUpTime") => { let p = &st.grab; Value::Float(p.pull_up_time) }
        ("grab", "pullUpStrengthLeft") => { let p = &st.grab; Value::Float(p.pull_up_strength[0]) }
        ("grab", "grabHoldMaxTimer") => { let p = &st.grab; Value::Float(p.grab_hold_max_timer) }
        ("grab", "reachDistance") => { let p = &st.grab; Value::Float(p.max_reach) }
        ("grab", "justBrace") => { let p = &st.grab; Value::Bool(p.justbrace) }
        ("grab", "useLineGrab") => { let p = &st.grab; Value::Bool(p.linegrab) }
        ("grab", "grabDistance") => { let p = &st.grab; Value::Float(p.grab_dist) }
        ("grab", "maxReachDistance") => { let p = &st.grab; Value::Float(p.max_reach) }
        ("grab", "dropWeaponIfNecessary") => { let p = &st.grab; Value::Bool(p.drop_weapon) }
        ("grab", "dropWeaponDistance") => { let p = &st.grab; Value::Float(p.drop_weapon_distance) }
        ("grab", "stickyHands") => { let p = &st.grab; Value::Float(p.sticky_hands) }
        ("grab", "turnToTarget") => { let p = &st.grab; Value::Int(p.turn_to_target) }
        ("grab", "handsCollide") => { let p = &st.grab; Value::Bool(p.hands_collide) }
        ("grab", "pointsX4grab") => { let p = &st.grab; Value::Bool(p.points_x4) }
        ("grab", "normalL2") => { let p = &st.grab; Value::Vec3(p.normal_l2) }
        ("grab", "normalR2") => { let p = &st.grab; Value::Vec3(p.normal_r2) }
        ("grab", "orientationConstraintScale") => { let p = &st.grab; Value::Float(p.orientation_scale) }
        ("grab", "maxWristAngle") => { let p = &st.grab; Value::Float(p.max_wrist_angle) }
        ("grab", "useHeadLookToTarget") => { let p = &st.grab; Value::Bool(p.head_look_to_target) }
        ("grab", "lookAtGrab") => { let p = &st.grab; Value::Bool(p.look_at_grab) }
        ("grab", "targetForHeadLook") => { let p = &st.grab; Value::Vec3(p.head_look_target) }
        ("headLook", "pos") => { let p = &st.head_look; Value::Vec3(p.pos) }
        ("headLook", "stiffness") => { let p = &st.head_look; Value::Float(p.stiffness) }
        ("headLook", "keepHeadAwayFromGround") => { let p = &st.head_look; Value::Bool(p.keep_head_away) }
        ("headLook", "twistSpine") => { let p = &st.head_look; Value::Bool(p.twist_spine) }
        ("headLook", "alwaysEyesHorizontal") => { let p = &st.head_look; Value::Bool(p.always_eyes_horizontal) }
        ("headLook", "alwaysLook") => { let p = &st.head_look; Value::Bool(p.always_look) }
        ("headLook", "vel") => { let p = &st.head_look; Value::Vec3(p.vel) }
        ("headLook", "damping") => { let p = &st.head_look; Value::Float(p.damping) }
        ("highFall", "bodyStiffness") => { let p = &st.high_fall; Value::Float(p.body_stiffness) }
        ("highFall", "armAmplitude") => { let p = &st.high_fall; Value::Float(p.arm_amplitude) }
        ("highFall", "armPeriod") => { let p = &st.high_fall; Value::Float(p.arm_period) }
        ("highFall", "pdStrength") => { let p = &st.high_fall; Value::Float(p.pd_strength) }
        ("highFall", "armsUp") => { let p = &st.high_fall; Value::Float(p.arms_up) }
        ("highFall", "legRadius") => { let p = &st.high_fall; Value::Float(p.leg_radius) }
        ("highFall", "bodydamping") => { let p = &st.high_fall; Value::Float(p.bodydamping) }
        ("highFall", "legAngSpeed") => { let p = &st.high_fall; Value::Float(p.leg_ang_speed) }
        ("highFall", "crashOrLandCutOff") => { let p = &st.high_fall; Value::Float(p.crash_or_land_cut_off) }
        ("highFall", "fowardOffsetOfLegIK") => { let p = &st.high_fall; Value::Float(p.foward_offset_of_leg_ik) }
        ("highFall", "catchfalltime") => { let p = &st.high_fall; Value::Float(p.catchfalltime) }
        ("highFall", "landCutOffVelocity") => { let p = &st.high_fall; Value::Float(p.crash_or_land_cut_off) }
        ("highFall", "ignorWorldCollisions") => { let p = &st.high_fall; Value::Bool(p.ignor_world_collisions) }
        ("highFall", "fowardRoll") => { let p = &st.high_fall; Value::Bool(p.foward_roll) }
        ("highFall", "orientateBodyToFallDirection") => { let p = &st.high_fall; Value::Bool(p.orientate_body_to_fall_direction) }
        ("highFall", "aimAngleBase") => { let p = &st.high_fall; Value::Float(p.aim_angle_base) }
        ("highFall", "fowardVelRotation") => { let p = &st.high_fall; Value::Float(p.foward_vel_rotation) }
        ("highFall", "sideD") => { let p = &st.high_fall; Value::Float(p.side_d) }
        ("highFall", "catchFallCutOff") => { let p = &st.high_fall; Value::Float(p.catch_fall_cut_off) }
        ("highFall", "footVelCompScale") => { let p = &st.high_fall; Value::Float(p.foot_vel_comp_scale) }
        ("highFall", "legL") => { let p = &st.high_fall; Value::Float(p.leg_l) }
        ("highFall", "fowardVelCutOff") => { let p = &st.high_fall; Value::Float(p.foward_vel_cut_off) }
        ("highFall", "legStrength") => { let p = &st.high_fall; Value::Float(p.leg_strength) }
        ("highFall", "zeroPoseRoll") => { let p = &st.high_fall; Value::Bool(p.zero_pose_roll) }
        ("leanInDirection", "leanAmount") => { let p = &st.lean_in_direction; Value::Float(p.lean_amount) }
        ("leanInDirection", "applyAsForce") => { let p = &st.lean_in_direction; Value::Bool(p.apply_as_force) }
        ("leanInDirection", "dir") => { let p = &st.lean_in_direction; Value::Vec3(p.dir) }
        ("leanToPosition", "leanAmount") => { let p = &st.lean_to_position; Value::Float(p.lean_amount) }
        ("leanToPosition", "applyAsForce") => { let p = &st.lean_to_position; Value::Bool(p.apply_as_force) }
        ("leanToPosition", "pos") => { let p = &st.lean_to_position; Value::Vec3(p.pos) }
        ("leanTowardsObject", "leanAmount") => { let p = &st.lean_towards_object; Value::Float(p.lean_amount) }
        ("leanTowardsObject", "applyAsForce") => { let p = &st.lean_towards_object; Value::Bool(p.apply_as_force) }
        ("leanTowardsObject", "offset") => { let p = &st.lean_towards_object; Value::Vec3(p.offset) }
        ("pedalLegs", "legStiffness") => { let p = &st.pedal_legs; Value::Float(p.leg_stiffness) }
        ("pedalLegs", "pedalRightLeg") => { let p = &st.pedal_legs; Value::Bool(p.right) }
        ("pedalLegs", "speedAsymmetry") => { let p = &st.pedal_legs; Value::Float(p.speed_asymmetry) }
        ("pedalLegs", "pedalLeftLeg") => { let p = &st.pedal_legs; Value::Bool(p.left) }
        ("pedalLegs", "radius") => { let p = &st.pedal_legs; Value::Float(p.radius) }
        ("pedalLegs", "angularSpeed") => { let p = &st.pedal_legs; Value::Float(p.angular_speed) }
        ("pedalLegs", "pedalOffset") => { let p = &st.pedal_legs; Value::Float(p.pedal_offset) }
        ("pedalLegs", "angSpeedMultiplier4Dragging") => { let p = &st.pedal_legs; Value::Float(p.drag_multiplier) }
        ("pedalLegs", "adaptivePedal4Dragging") => { let p = &st.pedal_legs; Value::Bool(p.adaptive) }
        ("pedalLegs", "backPedal") => { let p = &st.pedal_legs; Value::Bool(p.back_pedal) }
        ("pedalLegs", "radiusVariance") => { let p = &st.pedal_legs; Value::Float(p.radius_variance) }
        ("pedalLegs", "centreUp") => { let p = &st.pedal_legs; Value::Float(p.centre_up) }
        ("pedalLegs", "centreForwards") => { let p = &st.pedal_legs; Value::Float(p.centre_forwards) }
        ("pedalLegs", "legAngleVariance") => { let p = &st.pedal_legs; Value::Float(p.leg_angle_variance) }
        ("pedalLegs", "randomSeed") => { let p = &st.pedal_legs; Value::Int(p.random_seed as i32) }
        ("pedalLegs", "centresideways") => { let p = &st.pedal_legs; Value::Float(p.rdr2.centre_sideways) }
        ("pedalLegs", "ellipse") => { let p = &st.pedal_legs; Value::Float(p.rdr2.ellipse) }
        ("pedalLegs", "spread") => { let p = &st.pedal_legs; Value::Float(p.rdr2.spread) }
        ("pedalLegs", "hula") => { let p = &st.pedal_legs; Value::Bool(p.rdr2.hula) }
        ("pointArm", "instanceIndex") => { let p = &st.point_arm; Value::Int(p.instance_index) }
        ("pointArm", "useLeftArm") => { let p = &st.point_arm; Value::Bool(p.use_left_arm) }
        ("pointArm", "targetLeft") => { let p = &st.point_arm; Value::Vec3(p.rdr2.unwrap_or_default()[0].target) }
        ("pointArm", "targetRight") => { let p = &st.point_arm; Value::Vec3(p.rdr2.unwrap_or_default()[1].target) }
        ("pointArm", "twistLeft") => { let p = &st.point_arm; Value::Float(p.rdr2.unwrap_or_default()[0].twist) }
        ("pointArm", "twistRight") => { let p = &st.point_arm; Value::Float(p.rdr2.unwrap_or_default()[1].twist) }
        ("pointArm", "armStraightnessLeft") => { let p = &st.point_arm; Value::Float(p.rdr2.unwrap_or_default()[0].arm_straightness) }
        ("pointArm", "armStraightnessRight") => { let p = &st.point_arm; Value::Float(p.rdr2.unwrap_or_default()[1].arm_straightness) }
        ("pointArm", "armStiffnessLeft") => { let p = &st.point_arm; Value::Float(p.rdr2.unwrap_or_default()[0].arm_stiffness) }
        ("pointArm", "armStiffnessRight") => { let p = &st.point_arm; Value::Float(p.rdr2.unwrap_or_default()[1].arm_stiffness) }
        ("pointArm", "armDampingLeft") => { let p = &st.point_arm; Value::Float(p.rdr2.unwrap_or_default()[0].arm_damping) }
        ("pointArm", "armDampingRight") => { let p = &st.point_arm; Value::Float(p.rdr2.unwrap_or_default()[1].arm_damping) }
        ("pointArm", "pointSwingLimitLeft") => { let p = &st.point_arm; Value::Float(p.rdr2.unwrap_or_default()[0].point_swing_limit) }
        ("pointArm", "pointSwingLimitRight") => { let p = &st.point_arm; Value::Float(p.rdr2.unwrap_or_default()[1].point_swing_limit) }
        ("pointArm", "useRightArm") => { let p = &st.point_arm; Value::Bool(p.rdr2.unwrap_or_default()[1].use_arm) }
        ("pointArm", "useZeroPoseWhenNotPointingLeft") => { let p = &st.point_arm; Value::Bool(p.rdr2.unwrap_or_default()[0].use_zero_pose) }
        ("pointArm", "useZeroPoseWhenNotPointingRight") => { let p = &st.point_arm; Value::Bool(p.rdr2.unwrap_or_default()[1].use_zero_pose) }
        ("pointArm", "twist") => { let p = &st.point_arm; Value::Float(p.twist) }
        ("pointArm", "target") => { let p = &st.point_arm; Value::Vec3(p.target) }
        ("pointArm", "pointSwingLimit") => { let p = &st.point_arm; Value::Float(p.point_swing_limit) }
        ("pointArm", "armStraightness") => { let p = &st.point_arm; Value::Float(p.arm_straightness) }
        ("pointArm", "useZeroPoseWhenNotPointing") => { let p = &st.point_arm; Value::Bool(p.use_zero_pose_when_not_pointing) }
        ("pointArm", "armDamping") => { let p = &st.point_arm; Value::Float(p.arm_damping) }
        ("pointArm", "armStiffness") => { let p = &st.point_arm; Value::Float(p.arm_stiffness) }
        ("rollDownStairs", "forcemag") => { let p = &st.roll_down_stairs; Value::Float(p.forcemag) }
        ("rollDownStairs", "armReachLength") => { let p = &st.roll_down_stairs; Value::Float(p.arm_reach_length) }
        ("rollDownStairs", "m_legPush") => { let p = &st.roll_down_stairs; Value::Float(p.leg_push) }
        ("rollDownStairs", "useCustomRollDir") => { let p = &st.roll_down_stairs; Value::Bool(p.use_custom_roll_dir) }
        ("rollDownStairs", "stiffnessDecayTarget") => { let p = &st.roll_down_stairs; Value::Float(p.stiffness_decay_target) }
        ("rollDownStairs", "spinWhenInAir") => { let p = &st.roll_down_stairs; Value::Bool(p.spin_when_in_air) }
        ("rollDownStairs", "m_armReachAmount") => { let p = &st.roll_down_stairs; Value::Float(p.arm_reach_amount) }
        ("rollDownStairs", "customRollDir") => { let p = &st.roll_down_stairs; Value::Vec3(p.custom_roll_dir) }
        ("rollDownStairs", "stiffnessDecayTime") => { let p = &st.roll_down_stairs; Value::Float(p.stiffness_decay_time) }
        ("rollDownStairs", "m_useArmToSlowDown") => { let p = &st.roll_down_stairs; Value::Float(p.use_arm_to_slow_down) }
        ("rollDownStairs", "stiffness") => { let p = &st.roll_down_stairs; Value::Float(p.stiffness) }
        ("rollDownStairs", "damping") => { let p = &st.roll_down_stairs; Value::Float(p.damping) }
        ("rollDownStairs", "useZeroPose") => { let p = &st.roll_down_stairs; Value::Bool(p.use_zero_pose) }
        ("rollDownStairs", "zAxisSpinReduction") => { let p = &st.roll_down_stairs; Value::Float(p.z_axis_spin_reduction) }
        ("rollDownStairs", "asymmetricalLegs") => { let p = &st.roll_down_stairs; Value::Float(p.asymmetrical_legs) }
        ("rollDownStairs", "asymmetry") => { let p = &st.roll_down_stairs; Value::Float(p.asymmetry) }
        ("setStiffness", "bodyStiffness") => { let p = &st.set_stiffness; Value::Float(p.body_stiffness) }
        ("setStiffness", "mask") => { let p = &st.set_stiffness; Value::Str(p.mask.clone()) }
        ("setStiffness", "damping") => { let p = &st.set_stiffness; Value::Float(p.damping) }
        ("shot", "bodyStiffness") => { let p = &st.shot; Value::Float(p.body_stiffness) }
        ("shot", "grabHoldTime") => { let p = &st.shot; Value::Float(p.grab_hold_time) }
        ("shot", "bodyPart") => { let p = &st.shot; Value::Int(p.body_part as i32) }
        ("shot", "loosenessAmount") => { let p = &st.shot; Value::Float(p.looseness_amount) }
        ("shot", "shockSpinDecayMult") => { let p = &st.shot; Value::Float(p.shock_spin_decay_mult) }
        ("shot", "timeBeforeReachForWound") => { let p = &st.shot; Value::Float(p.time_before_reach_for_wound) }
        ("shot", "headLookAtWoundMaxTimer") => { let p = &st.shot; Value::Float(p.head_look_at_wound_max_timer) }
        ("shot", "reachForWound") => { let p = &st.shot; Value::Bool(p.reach_for_wound) }
        ("shot", "spinePainTwistMultiplier") => { let p = &st.shot; Value::Float(p.spine_pain_twist_multiplier) }
        ("shot", "useHeadLook") => { let p = &st.shot; Value::Bool(p.use_head_look) }
        ("shot", "shockSpinMin") => { let p = &st.shot; Value::Float(p.shock_spin_min) }
        ("shot", "stableHandsAndNeck") => { let p = &st.shot; Value::Bool(p.stable_hands_and_neck) }
        ("shot", "maxSteps") => { let p = &st.shot; Value::Int(p.max_steps) }
        ("shot", "shockSpinMax") => { let p = &st.shot; Value::Float(p.shock_spin_max) }
        ("shot", "timeBeforeCollapseWoundLeg") => { let p = &st.shot; Value::Float(p.time_before_collapse_wound_leg) }
        ("shot", "melee") => { let p = &st.shot; Value::Bool(p.melee) }
        ("shot", "shockSpinScalePerComponent") => { let p = &st.shot; Value::Float(p.shock_spin_scale_per_component) }
        ("shot", "randomizeShockSpinDirection") => { let p = &st.shot; Value::Bool(p.randomize_shock_spin_direction) }
        ("shot", "useCatchFallOnFall") => { let p = &st.shot; Value::Bool(p.use_catch_fall_on_fall) }
        ("shot", "crouching") => { let p = &st.shot; Value::Bool(p.crouching) }
        ("shot", "headLookAtWoundMinTimer") => { let p = &st.shot; Value::Float(p.head_look_at_wound_min_timer) }
        ("shot", "hitPoint") => { let p = &st.shot; Value::Vec3(p.hit_point) }
        ("shot", "shockSpinLiftForceMult") => { let p = &st.shot; Value::Float(p.shock_spin_lift_force_mult) }
        ("shot", "chickenArms") => { let p = &st.shot; Value::Bool(p.chicken_arms) }
        ("shot", "fling") => { let p = &st.shot; Value::Bool(p.fling) }
        ("shot", "normal") => { let p = &st.shot; Value::Vec3(p.normal) }
        ("shot", "spinePainTime") => { let p = &st.shot; Value::Float(p.spine_pain_time) }
        ("shot", "headLookAtHeadPosMinTimer") => { let p = &st.shot; Value::Float(p.head_look_at_head_pos_min_timer) }
        ("shot", "bulletVel") => { let p = &st.shot; Value::Vec3(p.bullet_vel) }
        ("shot", "addShockSpin") => { let p = &st.shot; Value::Bool(p.add_shock_spin) }
        ("shot", "spinePainMultiplier") => { let p = &st.shot; Value::Float(p.spine_pain_multiplier) }
        ("shot", "headLookAtHeadPosMaxTimer") => { let p = &st.shot; Value::Float(p.head_look_at_head_pos_max_timer) }
        ("shot", "alwaysAddShockSpin") => { let p = &st.shot; Value::Bool(p.always_add_shock_spin) }
        ("shot", "flingWidth") => { let p = &st.shot; Value::Float(p.fling_width) }
        ("shot", "headLook") => { let p = &st.shot; Value::Vec3(p.head_look) }
        ("shotRelax", "relaxPeriod") => { let p = &st.shot_relax; Value::Float(p.relax_period) }
        ("shotRelax", "relaxPeriodUpper") => { let p = &st.shot_relax; Value::Float(p.upper) }
        ("shotRelax", "relaxPeriodLower") => { let p = &st.shot_relax; Value::Float(p.lower) }
        ("stayUpright", "stiffness") => { let p = &st.stay_upright; Value::Float(p.stiffness) }
        ("stayUpright", "dampingScale") => { let p = &st.stay_upright; Value::Float(p.damping_scale) }
        ("stayUpright", "side") => Value::Int(st.stay_upright.side.unwrap_or(-1)),
        ("stayUpright", "torqueStrength") => Value::Vec3(st.stay_upright.rdr2.torque_strength),
        ("stayUpright", "torqueDamping") => Value::Vec3(st.stay_upright.rdr2.torque_damping),
        ("stayUpright", n) => {
            let x = st.stay_upright.rdr2.get(n)?;
            if crate::nmdbm_rdr2::StayUpright::is_flag(n) { Value::Bool(x != 0.0) } else { Value::Float(x) }
        }
        ("upperBodyFlinch", "handDistanceFrontBack") => { let p = &st.upper_body_flinch; Value::Float(p.hand_distance_front_back) }
        ("upperBodyFlinch", "noiseScale") => { let p = &st.upper_body_flinch; Value::Float(p.noise_scale) }
        ("upperBodyFlinch", "handDistanceVertical") => { let p = &st.upper_body_flinch; Value::Float(p.hand_distance_vertical) }
        ("upperBodyFlinch", "useRightArm") => { let p = &st.upper_body_flinch; Value::Bool(p.use_right_arm) }
        ("upperBodyFlinch", "handDistanceLeftRight") => { let p = &st.upper_body_flinch; Value::Float(p.hand_distance_left_right) }
        ("upperBodyFlinch", "rightHanded") => { let p = &st.upper_body_flinch; Value::Bool(p.right_handed) }
        ("upperBodyFlinch", "turnTowards") => { let p = &st.upper_body_flinch; Value::Int(p.turn_towards) }
        ("upperBodyFlinch", "backBendAmount") => { let p = &st.upper_body_flinch; Value::Float(p.back_bend_amount) }
        ("upperBodyFlinch", "newHit") => { let p = &st.upper_body_flinch; Value::Bool(p.new_hit.unwrap_or(true)) }
        ("upperBodyFlinch", "leftHanded") => { let p = &st.upper_body_flinch; Value::Bool(p.left_handed) }
        ("upperBodyFlinch", "bodyDamping") => { let p = &st.upper_body_flinch; Value::Float(p.body_damping) }
        ("upperBodyFlinch", "useLeftArm") => { let p = &st.upper_body_flinch; Value::Bool(p.use_left_arm) }
        ("upperBodyFlinch", "pos") => { let p = &st.upper_body_flinch; Value::Vec3(p.pos) }
        ("upperBodyFlinch", "bodyStiffness") => { let p = &st.upper_body_flinch; Value::Float(p.body_stiffness) }
        ("upperBodyFlinch", "protectHeadToggle") => { let p = &st.upper_body_flinch; Value::Bool(p.protect_head_toggle) }
        ("upperBodyFlinch", "applyStiffness") => { let p = &st.upper_body_flinch; Value::Bool(p.apply_stiffness) }
        ("upperBodyFlinch", "dontBraceHead") => { let p = &st.upper_body_flinch; Value::Bool(p.dont_brace_head) }
        ("upperBodyFlinch", "headLookAwayFromTarget") => { let p = &st.upper_body_flinch; Value::Bool(p.head_look_away) }
        ("upperBodyFlinch", "useHeadLook") => { let p = &st.upper_body_flinch; Value::Bool(p.use_head_look) }
        _ => return None,
    })
}

