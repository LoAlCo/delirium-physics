use crate::extras::{Extra, P};
use crate::msg::{Error, Kind, Message, Param, Value};
use crate::nmrs::{ANKLE, CLAVICLE, ELBOW, HIP, KNEE, NECK, SHOULDER, SPINE};
use crate::nmshot::Ctx;
use bevy_math::Vec3;

pub static MESSAGES: [Message; 7] = [
    Message { name: "teeter", params: &[
        Param { name: "edgeLeft", kind: Kind::Vec3, used: true, doc: "one end of the drop's edge (both zero: 0.5 m in front of his feet, 1 m wide)" },
        Param { name: "edgeRight", kind: Kind::Vec3, used: true, doc: "its other end" },
        Param { name: "reactDistance", kind: Kind::Float, used: true, doc: "COM to edge where he starts teetering, m (0.5)" },
        Param { name: "restartDistance", kind: Kind::Float, used: true, doc: "COM back from the edge that counts as recovered, m (0.8)" },
        Param { name: "useExclusionZone", kind: Kind::Bool, used: true, doc: "RDR2 core: the balancer keeps the COM's prediction exclusionZone on his side of the edge (true); not done without it" },
        Param { name: "useHeadLook", kind: Kind::Bool, used: true, doc: "look at the edge, then where he falls (true)" },
        Param { name: "callHighFall", kind: Kind::Bool, used: true, doc: "highFall when he goes over (true)" },
        Param { name: "leanAway", kind: Kind::Bool, used: true, doc: "lean away from the edge (true)" },
        Param { name: "preTeeterTime", kind: Kind::Float, used: true, doc: "settle time before teetering, s (0.2)" },
        Param { name: "leanAwayTime", kind: Kind::Float, used: true, doc: "how long the lean away lasts, s (0.8)" },
        Param { name: "teeterTime", kind: Kind::Float, used: true, doc: "how long he teeters before it resolves, s (2)" },
        Param { name: "leanAwayScale", kind: Kind::Float, used: true, doc: "how hard he leans away (0.1)" },
        Param { name: "edgehandle", kind: Kind::Int, used: false, doc: "RDR2 core: the object the edge moves with (none here: the edge is in the world)" },
        Param { name: "leantowards", kind: Kind::Bool, used: true, doc: "RDR2 core only: lean toward the drop instead (false)" },
        Param { name: "writhefall", kind: Kind::Bool, used: true, doc: "RDR2 core only: writhe while falling clear with no highFall (true)" },
        Param { name: "forceBalance", kind: Kind::Bool, used: true, doc: "RDR2 core only: no new steps while the COM is past the edge (true)" },
        Param { name: "exclusionZone", kind: Kind::Float, used: true, doc: "RDR2 core only: m on his side of the edge the balancer's prediction keeps to (0.2)" },
    ] },
    Message { name: "stumble", params: &[
        Param { name: "legsStiffness", kind: Kind::Float, used: true, doc: "legs, damping 0.5 (12)" },
        Param { name: "torsoStiffness", kind: Kind::Float, used: true, doc: "the upper body under the arms (10)" },
        Param { name: "armsStiffness", kind: Kind::Float, used: true, doc: "arms are held at this minus 1 (10)" },
        Param { name: "armReduceSpeed", kind: Kind::Float, used: false, doc: "(not done: catchFall has no such param, 0.2)" },
        Param { name: "wristMS", kind: Kind::Float, used: false, doc: "(not done: no hand mask, 15)" },
        Param { name: "feetMS", kind: Kind::Float, used: false, doc: "(not done: no foot mask, 15)" },
        Param { name: "armTwist", kind: Kind::Float, used: true, doc: "how far the braced arms turn out, of the joint range (0.3)" },
        Param { name: "fallMask", kind: Kind::Str, used: false, doc: "(not done: the arms always brace)" },
        Param { name: "forwardMaxArmOffset", kind: Kind::Float, used: true, doc: "arms' reach forward at full lean, of the joint range (0.9)" },
        Param { name: "backwardsMinArmOffset", kind: Kind::Float, used: true, doc: "arms' reach when upright, of the joint range (0.2)" },
        Param { name: "useArmsBrace", kind: Kind::Bool, used: true, doc: "arms brace forward (true)" },
        Param { name: "grabRadius2", kind: Kind::Float, used: false, doc: "(not done: no grabbing, 0.5)" },
        Param { name: "leanRate", kind: Kind::Float, used: true, doc: "how fast the lean moves to leanTowards, per second (0.5)" },
        Param { name: "maxLeanBack", kind: Kind::Float, used: true, doc: "most he leans back (0.1)" },
        Param { name: "maxLeanForward", kind: Kind::Float, used: true, doc: "most he leans forward (0.3)" },
        Param { name: "leanTowards", kind: Kind::Float, used: true, doc: "the lean he is heading for, + forward (0.2)" },
        Param { name: "zAxisSpinReduction", kind: Kind::Float, used: true, doc: "damping of his spin about up, of weight x metre per rad/s (0.5)" },
        Param { name: "dampPelvis", kind: Kind::Float, used: true, doc: "damping of the pelvis' spin, same units (0)" },
        Param { name: "pitchInContact", kind: Kind::Float, used: false, doc: "(not done: 0)" },
        Param { name: "twistSpine", kind: Kind::Bool, used: true, doc: "the spine twists as he steps (true)" },
        Param { name: "useHeadLook", kind: Kind::Bool, used: true, doc: "look where he is heading (true)" },
        Param { name: "staggerTime", kind: Kind::Float, used: true, doc: "seconds of stumbling before it counts as lost, s (1.5)" },
        Param { name: "dropVal", kind: Kind::Float, used: true, doc: "COM drop from where he began that counts as down, m (0.4)" },
        Param { name: "different", kind: Kind::Bool, used: true, doc: "the two arms brace differently (false)" },
    ] },
    Message { name: "staggerFall", params: &[
        Param { name: "armStiffness", kind: Kind::Float, used: true, doc: "arm muscle stiffness once ramped (6)" },
        Param { name: "armDamping", kind: Kind::Float, used: true, doc: "arm damping once ramped (0.5)" },
        Param { name: "spineStiffness", kind: Kind::Float, used: true, doc: "spine stiffness once ramped (8)" },
        Param { name: "spineDamping", kind: Kind::Float, used: true, doc: "spine damping once ramped (0.5)" },
        Param { name: "armStiffnessStart", kind: Kind::Float, used: true, doc: "arm stiffness at the start (12)" },
        Param { name: "armDampingStart", kind: Kind::Float, used: true, doc: "arm damping at the start (1)" },
        Param { name: "spineStiffnessStart", kind: Kind::Float, used: true, doc: "spine stiffness at the start (12)" },
        Param { name: "spineDampingStart", kind: Kind::Float, used: true, doc: "spine damping at the start (1)" },
        Param { name: "timeAtStartValues", kind: Kind::Float, used: true, doc: "s the start gains are held (0.5)" },
        Param { name: "rampTimeFromStartValues", kind: Kind::Float, used: true, doc: "s to ramp from them to steady (1)" },
        Param { name: "lowerBodyStiffness", kind: Kind::Float, used: true, doc: "leg stiffness (12)" },
        Param { name: "strikeReduction", kind: Kind::Float, used: true, doc: "RDR2 core: a released stance leg's hip takes this off and its knee twice it, going forward (1)" },
        Param { name: "strikereductionb", kind: Kind::Float, used: true, doc: "RDR2 core: the same going back (1)" },
        Param { name: "lowerbodystiffnessb", kind: Kind::Float, used: true, doc: "RDR2 core: all the leg stiffness times this going back (1)" },
        Param { name: "lowerBodyStiffnessEnd", kind: Kind::Float, used: true, doc: "leg stiffness at the end of the wind-down (3)" },
        Param { name: "staggerStepProb", kind: Kind::Float, used: true, doc: "chance a step is a stagger, pushed off (0.5)" },
        Param { name: "stepsTillStartEnd", kind: Kind::Int, used: true, doc: "steps before the wind-down (6)" },
        Param { name: "timeStartEnd", kind: Kind::Float, used: true, doc: "s before the wind-down, whichever comes first (6)" },
        Param { name: "rampTimeToEndValues", kind: Kind::Float, used: true, doc: "s to ramp to the end gains (2)" },
        Param { name: "predictionTime", kind: Kind::Float, used: true, doc: "RDR2 core: the balancer's prediction time, s, going forward (0.15); not done without it" },
        Param { name: "predictiontimeb", kind: Kind::Float, used: true, doc: "RDR2 core: the same going back (0.15)" },
        Param { name: "perStepReduction1", kind: Kind::Float, used: true, doc: "gains times this each step, floor 0.2 (0.95)" },
        Param { name: "pushOffDist", kind: Kind::Float, used: true, doc: "m a stagger step pushes off (0.1); RDR2 core: COM this far past the stance foot, going forward, before the knee pushes (0.28)" },
        Param { name: "pushoffdistb", kind: Kind::Float, used: true, doc: "RDR2 core: the same going back (0.23)" },
        Param { name: "pushoffknee", kind: Kind::Float, used: true, doc: "RDR2 core: the stance knee pushes when straighter than this, rad (-0.25)" },
        Param { name: "forcebalancethresh", kind: Kind::Float, used: true, doc: "RDR2 core: slow and the COM this steeply over both feet (cosine): no new steps (1)" },
        Param { name: "timeToFall", kind: Kind::Float, used: true, doc: "RDR2 core: s before fallType acts (2)" },
        Param { name: "fallType", kind: Kind::Int, used: true, doc: "RDR2 core: 0 fail the balance when a knee push ends, 1 no new steps, 2 (not done), 3-5 the balancer's lean in the step wait (4)" },
        Param { name: "firingInputOutput", kind: Kind::Bool, used: true, doc: "RDR2 core: hands the fall to catchFall even while the shot runs (false)" },
        Param { name: "maxPushoffVel", kind: Kind::Float, used: true, doc: "its top speed, m/s (1)" },
        Param { name: "leanInDirRate", kind: Kind::Float, used: true, doc: "lean amount rise, per s (1)" },
        Param { name: "leanInDirMaxF", kind: Kind::Float, used: true, doc: "lean cap going forward (0.3)" },
        Param { name: "leanInDirMaxB", kind: Kind::Float, used: true, doc: "lean cap going back (0.15)" },
        Param { name: "leanHipsMaxF", kind: Kind::Float, used: true, doc: "hip lean going forward, of the joint range (0.3)" },
        Param { name: "leanHipsMaxB", kind: Kind::Float, used: true, doc: "hip lean going back (0.1)" },
        Param { name: "lean2multF", kind: Kind::Float, used: true, doc: "hip lean multiplier forward (1)" },
        Param { name: "lean2multB", kind: Kind::Float, used: true, doc: "hip lean multiplier back (1)" },
        Param { name: "hipBendMult", kind: Kind::Float, used: true, doc: "hip bend, 0.3 of range, times this (1)" },
        Param { name: "spineBendMult", kind: Kind::Float, used: true, doc: "spine bend times this (1)" },
        Param { name: "alwaysBendForwards", kind: Kind::Bool, used: true, doc: "bend forward even going back (false)" },
        Param { name: "headLookPos", kind: Kind::Vec3, used: true, doc: "where he looks (2 m ahead)" },
        Param { name: "headLookInstanceIndex", kind: Kind::Int, used: false, doc: "a game object index, not done (-1)" },
        Param { name: "headLookAtVelProb", kind: Kind::Float, used: true, doc: "chance a step looks along his velocity; above 1 always, -1 coin flip (0.5)" },
        Param { name: "turnOffProb", kind: Kind::Float, used: true, doc: "weight of this body turn (0.3)" },
        Param { name: "turn2TargetProb", kind: Kind::Float, used: true, doc: "weight of this body turn (0.1)" },
        Param { name: "turn2VelProb", kind: Kind::Float, used: true, doc: "weight of this body turn (0.3)" },
        Param { name: "turnAwayProb", kind: Kind::Float, used: true, doc: "weight of this body turn (0.1)" },
        Param { name: "turnLeftProb", kind: Kind::Float, used: true, doc: "weight of this body turn (0.1)" },
        Param { name: "turnRightProb", kind: Kind::Float, used: true, doc: "weight of this body turn (0.1)" },
        Param { name: "useBodyTurn", kind: Kind::Bool, used: true, doc: "body turns on (true)" },
        Param { name: "useHeadLook", kind: Kind::Bool, used: true, doc: "head look on (true)" },
    ] },
    Message { name: "smartFall", params: &[
        Param { name: "bodyStiffness", kind: Kind::Float, used: true, doc: "body stiffness while airborne (10)" },
        Param { name: "bodydamping", kind: Kind::Float, used: true, doc: "its damping ratio (0.6)" },
        Param { name: "pdStrength", kind: Kind::Float, used: false, doc: "(not done: 0)" },
        Param { name: "pdDamping", kind: Kind::Float, used: false, doc: "(not done: 0)" },
        Param { name: "catchfalltime", kind: Kind::Float, used: true, doc: "catchFall runs this long after a land, s (1.5)" },
        Param { name: "catchFallCutOff", kind: Kind::Float, used: true, doc: "speed under which a landing counts settled, m/s (0.3)" },
        Param { name: "crashOrLandCutOff", kind: Kind::Float, used: true, doc: "ends this long after the landing at the latest, s (4)" },
        Param { name: "armsUp", kind: Kind::Float, used: true, doc: "arms raised by this, of the joint range (0.6)" },
        Param { name: "armAngSpeed", kind: Kind::Float, used: true, doc: "arm circling speed, rad/s (8)" },
        Param { name: "armAmplitude", kind: Kind::Float, used: true, doc: "arm circle size, of the joint range (0.7)" },
        Param { name: "armPhase", kind: Kind::Float, used: true, doc: "right arm behind the left, radians (3.14)" },
        Param { name: "armBendElbows", kind: Kind::Float, used: true, doc: "elbow bend, of the joint range (0.3)" },
        Param { name: "legRadius", kind: Kind::Float, used: true, doc: "pedalLegs radius, m (0.15)" },
        Param { name: "legAngSpeed", kind: Kind::Float, used: true, doc: "pedalLegs angular speed, rad/s (8)" },
        Param { name: "legAsymmetry", kind: Kind::Float, used: true, doc: "pedalLegs speed asymmetry (0)" },
        Param { name: "legL", kind: Kind::Bool, used: true, doc: "the left leg pedals too (true)" },
        Param { name: "sideD", kind: Kind::Float, used: false, doc: "(not done: 0)" },
        Param { name: "forwardOffsetOfLegIK", kind: Kind::Float, used: true, doc: "pedalLegs centreForwards, m (0)" },
        Param { name: "legStrength", kind: Kind::Float, used: true, doc: "pedalLegs leg stiffness (8)" },
        Param { name: "arms2LegsPhase", kind: Kind::Float, used: false, doc: "(not done: 0)" },
        Param { name: "arms2LegsSync", kind: Kind::Bool, used: false, doc: "(not done: false)" },
        Param { name: "adaptiveCircling", kind: Kind::Bool, used: false, doc: "(not done: false)" },
        Param { name: "hula", kind: Kind::Float, used: false, doc: "(not done: 0)" },
        Param { name: "aimAngleBase", kind: Kind::Float, used: false, doc: "(not done: 0)" },
        Param { name: "forwardVelRotation", kind: Kind::Float, used: false, doc: "(not done: 0)" },
        Param { name: "footVelCompScale", kind: Kind::Float, used: false, doc: "(not done: 0)" },
        Param { name: "orientateBodyToFallDirection", kind: Kind::Bool, used: true, doc: "turn to face the way he is falling (true)" },
        Param { name: "orientateTwist", kind: Kind::Float, used: true, doc: "spine twist into that turn, of the joint range (0.3)" },
        Param { name: "orientateMax", kind: Kind::Float, used: true, doc: "the turn is clamped to this angle, radians (0.8)" },
        Param { name: "minSpeedForBrace", kind: Kind::Float, used: true, doc: "landing faster than this he rolls rather than lands, m/s (4)" },
        Param { name: "maxSpeedForRecoverableFall", kind: Kind::Float, used: true, doc: "landing faster than this is a crash, m/s (12)" },
        Param { name: "landingNormal", kind: Kind::Vec3, used: true, doc: "the surface he lands on (no ground sense here: given; 0 0 1)" },
        Param { name: "balance", kind: Kind::Bool, used: true, doc: "bodyBalance on a land (true)" },
        Param { name: "forwardRoll", kind: Kind::Bool, used: true, doc: "a roll is bodyRollUp even on a slope (false)" },
        Param { name: "useZeroPose_withForwardRoll", kind: Kind::Bool, used: false, doc: "(not done: false)" },
        Param { name: "ignorWorldCollisions", kind: Kind::Bool, used: false, doc: "(not done: false)" },
        Param { name: "alanRickman", kind: Kind::Bool, used: true, doc: "limp, no flail: straight down (false)" },
        Param { name: "rdsForceMag", kind: Kind::Float, used: true, doc: "rollDownStairs forcemag on a crash down a slope (2)" },
        Param { name: "tumble", kind: Kind::Float, used: true, doc: "forward somersault in the air, rad/s, about the way he is going (0: none)" },
        Param { name: "rdsTargetLinearVelocity", kind: Kind::Float, used: false, doc: "(not done: 0)" },
        Param { name: "rdsTargetLinearVelocityDecayTime", kind: Kind::Float, used: false, doc: "(not done: 0)" },
        Param { name: "initialState", kind: Kind::Int, used: true, doc: "RDR2 core only: the state it starts in, 0 airborne .. 6 hard landing .. 8 settle (0)" },
        Param { name: "stopRollingTime", kind: Kind::Float, used: true, doc: "RDR2 core only (2)" },
        Param { name: "splatWhenStopped", kind: Kind::Float, used: true, doc: "RDR2 core only: after a hard landing, lie still below this speed, m/s (0: off)" },
        Param { name: "cfZAxisSpinReduction", kind: Kind::Float, used: true, doc: "RDR2 core only: catchFall's zAxisSpinReduction (0.5)" },
        Param { name: "teeter", kind: Kind::Bool, used: true, doc: "RDR2 core only: teeter at the edge when landing on his feet, from initialState 7 (false)" },
        Param { name: "teeterOffset", kind: Kind::Float, used: true, doc: "RDR2 core only: the edge this far ahead, m (0.3)" },
        Param { name: "rdsForceVelThreshold", kind: Kind::Float, used: true, doc: "RDR2 core only (10)" },
    ] },
    Message { name: "balancerCollisionsReaction", params: &[
        Param { name: "objectBehindVictimPos", kind: Kind::Vec3, used: true, doc: "the obstacle behind him (zero: a hip-high wall 0.4 m behind)" },
        Param { name: "objectBehindVictimNormal", kind: Kind::Vec3, used: true, doc: "its normal, toward him (zero: from the contact)" },
        Param { name: "numStepsTillSlump", kind: Kind::Int, used: true, doc: "steps allowed before forcing a slump, 0.3 s each (3)" },
        Param { name: "stable2SlumpTime", kind: Kind::Float, used: true, doc: "still this long after impact: slump anyway, s (1.0)" },
        Param { name: "exclusionZone", kind: Kind::Float, used: true, doc: "RDR2 core: the balancer keeps his predicted COM this far off the wall, m (0.2); not done without it" },
        Param { name: "footFrictionMultStart", kind: Kind::Float, used: false, doc: "(not done: 1)" },
        Param { name: "footFrictionMultRate", kind: Kind::Float, used: false, doc: "(not done: 0)" },
        Param { name: "backFrictionMultStart", kind: Kind::Float, used: false, doc: "(not done: 1)" },
        Param { name: "backFrictionMultRate", kind: Kind::Float, used: false, doc: "(not done: 0)" },
        Param { name: "impactLegStiffReduction", kind: Kind::Float, used: true, doc: "leg stiffness dropped on impact, of its value (0.3)" },
        Param { name: "slumpLegStiffReduction", kind: Kind::Float, used: true, doc: "leg stiffness dropped when slumping, of its value (0.8)" },
        Param { name: "slumpLegStiffRate", kind: Kind::Float, used: true, doc: "slump drop reached at this per second (1.0)" },
        Param { name: "reactTime", kind: Kind::Float, used: true, doc: "contact gathered this long before it reacts, s (0.25)" },
        Param { name: "impactExagTime", kind: Kind::Float, used: true, doc: "impact exaggerated for this long, s (0.2)" },
        Param { name: "glanceSpinTime", kind: Kind::Float, used: true, doc: "glance-spin lasts, s (0.5)" },
        Param { name: "glanceSpinMag", kind: Kind::Float, used: true, doc: "glance-spin torque, of weight x metre / 2 (0.3)" },
        Param { name: "glanceSpinDecayMult", kind: Kind::Float, used: true, doc: "its decay per 0.1 s (0.8)" },
        Param { name: "objectBehindVictim", kind: Kind::Bool, used: true, doc: "RDR2 core only: the wall is objectBehindVictimPos/Normal from the start, not found by probing ahead (false)" },
        Param { name: "ignoreColMassBelow", kind: Kind::Float, used: true, doc: "RDR2 core only: a moving object lighter than this (kg) and smaller than ignoreColVolumeBelow is no wall (20)" },
        Param { name: "ignoreColVolumeBelow", kind: Kind::Float, used: true, doc: "RDR2 core only: m^3 (0.1)" },
        Param { name: "reboundMode", kind: Kind::Int, used: true, doc: "RDR2 core only: coming off the wall, 0 slump unless shot, 1 keep going, 2 slump, 3 start over (0)" },
        Param { name: "forwardMode", kind: Kind::Int, used: true, doc: "RDR2 core only: 1: no slump on a forward impact while shot (0)" },
        Param { name: "timeToForward", kind: Kind::Float, used: true, doc: "RDR2 core only: s after a forward impact before he slumps (0.5)" },
        Param { name: "reboundForce", kind: Kind::Float, used: true, doc: "RDR2 core only: push off the wall on a forward impact, on the chest (upper arms -1/4) each tick (0)" },
        Param { name: "braceWall", kind: Kind::Bool, used: true, doc: "RDR2 core only: look at the wall coming up to it (false)" },
        Param { name: "glanceSpinReduce", kind: Kind::Float, used: true, doc: "RDR2 core only: the spin's share per m/s along the wall (1.5)" },
        Param { name: "glanceSpinTurn", kind: Kind::Float, used: true, doc: "RDR2 core only: over 0 the glance asks the balancer to turn, 0 torques the spine (40)" },
        Param { name: "slumpMode", kind: Kind::Int, used: true, doc: "RDR2 core only: stiffness falls while kinetic energy falls below 0.4; 1 always, 2 while it falls at all (1)" },
        Param { name: "sitstarttime", kind: Kind::Float, used: true, doc: "RDR2 core only: s into the slump before the legs go to the sit pose (0.3)" },
        Param { name: "sitrate", kind: Kind::Float, used: true, doc: "RDR2 core only: blend to the sit pose per s, 0 none (1)" },
        Param { name: "fling2prob", kind: Kind::Float, used: true, doc: "RDR2 core only: each arm's chance to fling at an impact over 1.25 m/s (0.5)" },
        Param { name: "armBrace", kind: Kind::Bool, used: true, doc: "RDR2 core only: arms near the wall brace on it (true)" },
        Param { name: "bracePredictionTime", kind: Kind::Float, used: true, doc: "RDR2 core only: how far ahead the brace service predicts him, s (0.7)" },
        Param { name: "minBraceTime", kind: Kind::Float, used: true, doc: "RDR2 core only: s a brace lasts once begun (0.3)" },
        Param { name: "timeToBackwardsBrace", kind: Kind::Float, used: true, doc: "RDR2 core only: s before a brace at a wall behind him is allowed (0.5)" },
        Param { name: "reachAbsorbtionTime", kind: Kind::Float, used: true, doc: "RDR2 core only: how much the arms shorten per m/s closing (0.15)" },
        Param { name: "braceDistance", kind: Kind::Float, used: true, doc: "RDR2 core only: brace within this of the wall, m (2)" },
        Param { name: "bracearmstiffness", kind: Kind::Float, used: true, doc: "RDR2 core only: the bracing arm's stiffness (12)" },
        Param { name: "staggerStopTime", kind: Kind::Float, used: true, doc: "RDR2 core only: staggerFall stops with bodyBalance or the shot on when the wall is nearer in time, s (0)" },
        Param { name: "snap", kind: Kind::Bool, used: true, doc: "RDR2 core only: the shot's snap on the impact (false)" },
        Param { name: "snapDirectionRandomness", kind: Kind::Float, used: true, doc: "RDR2 core only (0.3)" },
        Param { name: "snapHipType", kind: Kind::Int, used: true, doc: "RDR2 core only (0)" },
        Param { name: "snapDuration", kind: Kind::Float, used: true, doc: "RDR2 core only (0)" },
        Param { name: "snapLeftLeg", kind: Kind::Bool, used: true, doc: "RDR2 core only" },
        Param { name: "snapRightLeg", kind: Kind::Bool, used: true, doc: "RDR2 core only" },
        Param { name: "snapLeftArm", kind: Kind::Bool, used: true, doc: "RDR2 core only" },
        Param { name: "snapRightArm", kind: Kind::Bool, used: true, doc: "RDR2 core only" },
        Param { name: "snapSpine", kind: Kind::Bool, used: true, doc: "RDR2 core only" },
        Param { name: "snapNeck", kind: Kind::Bool, used: true, doc: "RDR2 core only" },
        Param { name: "snapPhasedLegs", kind: Kind::Bool, used: true, doc: "RDR2 core only" },
        Param { name: "fallOverWallDrape", kind: Kind::Bool, used: true, doc: "RDR2 core only: over a low wall fallOverWall takes him (true)" },
        Param { name: "fallOverHighWalls", kind: Kind::Bool, used: true, doc: "RDR2 core only: the lower spine's contact counts as low (false)" },
        Param { name: "impactWeaknessZeroDuration", kind: Kind::Float, used: true, doc: "RDR2 core only: s after the impact before the weakness ramps (0.2)" },
        Param { name: "impactWeaknessRampDuration", kind: Kind::Float, used: true, doc: "RDR2 core only: s for it to ramp to 1 (0.01)" },
    ] },
    Message { name: "injuredOnGround", params: &[
        Param { name: "numInjuries", kind: Kind::Int, used: true, doc: "wounds to reach for, 0 to 2 (0: one on the left of the belly); RDR2 core: one, 0 the belly" },
        Param { name: "injury1LocalPosition", kind: Kind::Vec3, used: true, doc: "wound 1, in its body part's frame, m (zero)" },
        Param { name: "injury1LocalNormal", kind: Kind::Vec3, used: true, doc: "its outward normal, the hand stops this far off (zero); RDR2 core: not read, the position's direction" },
        Param { name: "injury1Component", kind: Kind::Int, used: true, doc: "its body part index (7, the lower spine)" },
        Param { name: "injury2LocalPosition", kind: Kind::Vec3, used: true, doc: "wound 2, as wound 1 (zero); RDR2 core: not read" },
        Param { name: "injury2LocalNormal", kind: Kind::Vec3, used: true, doc: "its outward normal (zero); RDR2 core: not read" },
        Param { name: "injury2Component", kind: Kind::Int, used: true, doc: "its body part index (7, the lower spine); RDR2 core: not read" },
        Param { name: "attackerPos", kind: Kind::Vec3, used: true, doc: "where he looks (zero: 2 m beyond his head, along the ground); RDR2 core: what the trapped push braces against (he looks at the wound)" },
        Param { name: "dontReachWithLeft", kind: Kind::Bool, used: true, doc: "the left hand stays off the wounds (false)" },
        Param { name: "dontReachWithRight", kind: Kind::Bool, used: true, doc: "the right hand stays off the wounds (false)" },
        Param { name: "strongRollForce", kind: Kind::Bool, used: true, doc: "roll about twice as hard (false); not RDR2's" },
        Param { name: "relaxTimeMin", kind: Kind::Float, used: true, doc: "RDR2 core only: rest between rolls, s, at least (3)" },
        Param { name: "relaxTimeMax", kind: Kind::Float, used: true, doc: "RDR2 core only: and at most (5)" },
        Param { name: "useConstraints", kind: Kind::Bool, used: true, doc: "RDR2 core only: pin the chest and pelvis lying on his back (false)" },
        Param { name: "trapped", kind: Kind::Bool, used: true, doc: "RDR2 core only: push against what is towards attackerPos instead of reaching for the wound (false)" },
        Param { name: "attackerInstance", kind: Kind::Int, used: true, doc: "RDR2 core only: the object attackerPos is given in (-1: the world)" },
        Param { name: "noRollOnBack", kind: Kind::Bool, used: true, doc: "RDR2 core only: no rolling while lying on his back (false; RDR2's +0xe4, its name not solved)" },
        Param { name: "rollDuration", kind: Kind::Float, used: true, doc: "RDR2 core only: a roll's duration, s (5; RDR2's +0xa0, its table name doubtful)" },
        Param { name: "rollRandomAbove", kind: Kind::Float, used: true, doc: "RDR2 core only: a draw above this makes the duration random, 0.2..3 s (0.3; +0xa4)" },
        Param { name: "rollTorqueMax", kind: Kind::Float, used: true, doc: "RDR2 core only: the roll torque's cap (30; +0xa8)" },
        Param { name: "rollTorqueStart", kind: Kind::Float, used: true, doc: "RDR2 core only: the roll torque at a roll's start, 0: 20 for the first roll (0; +0xac)" },
        Param { name: "rollTorqueRate", kind: Kind::Float, used: true, doc: "RDR2 core only: the roll torque's ramp rate (60; +0xb0)" },
    ] },
    Message { name: "learnedCrawl", params: &[
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "(10)" },
        Param { name: "damping", kind: Kind::Float, used: true, doc: "(0.6)" },
        Param { name: "bLearn", kind: Kind::Bool, used: false, doc: "(not done: the crawl cycle is ours, nothing learned)" },
        Param { name: "numFrames2Learn", kind: Kind::Int, used: false, doc: "(not done: the crawl cycle is ours, nothing learned)" },
        Param { name: "animIndex", kind: Kind::Int, used: false, doc: "(not done: the crawl cycle is ours, nothing learned)" },
        Param { name: "learnFromAnimPlayback", kind: Kind::Bool, used: false, doc: "(not done: the crawl cycle is ours, nothing learned)" },
        Param { name: "inputSequenceSize", kind: Kind::Int, used: false, doc: "(not done: the crawl cycle is ours, nothing learned)" },
        Param { name: "speed", kind: Kind::Float, used: true, doc: "crawl cycles a second (0.8)" },
        Param { name: "targetPosition", kind: Kind::Vec3, used: true, doc: "where he crawls to (zero: 3 m ahead of him)" },
        Param { name: "yawOffset", kind: Kind::Float, used: true, doc: "rad, heading this much off the target (0)" },
        Param { name: "useSpine3Thing", kind: Kind::Bool, used: false, doc: "(not done)" },
        Param { name: "useRollBoneCompensation", kind: Kind::Bool, used: false, doc: "(not done)" },
        Param { name: "useTwister", kind: Kind::Bool, used: true, doc: "the spine twists toward the heading (true)" },
    ] },
];

pub fn build(name: &str, p: &mut P) -> Option<Result<Box<dyn Extra>, Error>> {
    fn b<T: Extra + 'static>(e: Result<T, Error>) -> Option<Result<Box<dyn Extra>, Error>> {
        Some(e.map(|e| Box::new(e) as _))
    }
    match name {
        "teeter" => b(Teeter::new(p)),
        "stumble" => b(Stumble::new(p)),
        "staggerFall" => b(StaggerFall::new(p)),
        "smartFall" => b(SmartFall::new(p)),
        // only the RDR2 core sends highFall to the extras (msg.rs)
        "highFall" => b(crate::extras_fall_rdr2::HighFall::new(p)),
        // likewise fallOverWall
        "fallOverWall" => b(crate::extras_fall_rdr2::FallOverWall::new(p)),
        "balancerCollisionsReaction" => b(Collide::new(p)),
        "injuredOnGround" => b(Injured::new(p)),
        "learnedCrawl" => b(Crawl::new(p)),
        _ => None,
    }
}

#[allow(non_snake_case)]
#[inline]
pub(crate) fn CHEST() -> usize { (*SPINE())[delirium_data::kv!(0 int)] + 1 }
const TAU: f32 = std::f32::consts::TAU;

fn weight(c: &Ctx) -> f32 {
    c.b.doll.mass() * delirium_data::kv!(115)
}

fn flat(v: Vec3) -> Vec3 {
    Vec3::new(v.x, v.y, 0.0).normalize_or_zero()
}

fn travel(c: &Ctx, d: Vec3) -> Vec3 {
    if c.vel.truncate().length() > delirium_data::kv!(116) { flat(c.vel) } else { flat(d) }
}

fn windmill(c: &mut Ctx, t: f32, amp: f32) {
    for i in 0..2 {
        let ph = t + i as f32 * TAU * 0.5;
        c.ch.set_relative((*SHOULDER())[i], 0, amp * ph.sin());
        c.ch.set_relative((*SHOULDER())[i], 1, amp * ph.cos());
        c.ch.set_relative((*ELBOW())[i], 0, -delirium_data::kv!(24));
    }
}

pub struct Teeter {
    ends: (Vec3, Vec3),
    react: f32,
    restart: f32,
    head: bool,
    high_fall: bool,
    lean: bool,
    pre: f32,
    lean_time: f32,
    teeter_time: f32,
    lean_scale: f32,
    normal: Vec3,
    state: u8,
    t: f32,
    out: Vec<String>,
    /// RDR2's, with the RDR2 core: its parameters and state
    rp: crate::extras_fall_rdr2::TtParams,
    rd: Option<Box<crate::extras_fall_rdr2::TeeterRdr2>>,
}
const PRE: u8 = 0;
const TEETERING: u8 = 1;
const FALLING: u8 = 2;
#[allow(non_snake_case)]
#[inline]
fn RECOVERED() -> u8 { delirium_data::kv!(0 int) }
impl Teeter {
    fn new(p: &mut P) -> Result<Teeter, Error> {
        p.b("useExclusionZone", false)?;
        let rp = crate::extras_fall_rdr2::TtParams::read(p)?;
        Ok(Teeter { rp, rd: None, ends: (p.v("edgeLeft", Vec3::ZERO)?, p.v("edgeRight", Vec3::ZERO)?), react: p.f("reactDistance", 0.5)?,
                    restart: p.f("restartDistance", delirium_data::kv!(20))?, head: p.b("useHeadLook", true)?, high_fall: p.b("callHighFall", true)?,
                    lean: p.b("leanAway", true)?, pre: p.f("preTeeterTime", delirium_data::kv!(60))?, lean_time: p.f("leanAwayTime", delirium_data::kv!(20))?,
                    teeter_time: p.f("teeterTime", 2.0)?, lean_scale: p.f("leanAwayScale", delirium_data::kv!(78))?, normal: Vec3::ZERO, state: PRE,
                    t: 0.0, out: vec![] })
    }

    fn to(&mut self, state: u8, name: &str) {
        (self.state, self.t) = (state, 0.0);
        self.out.push(format!("#say teeter {name}"));
    }

    fn look(&mut self, p: Vec3) {
        if self.head {
            self.out.push(format!("headLook start=true pos={},{},{} stiffness={} damping=1", p.x, p.y, p.z, delirium_data::kv!(117)));
        }
    }
}
impl Extra for Teeter {
    fn update(&mut self, p: &mut P) -> Result<(), Error> {
        if let Some(rd) = self.rd.as_mut() {
            rd.update(p)?;
        }
        Ok(())
    }

    fn stop(&mut self, c: &mut Ctx) {
        if let Some(rd) = self.rd.as_mut() {
            rd.stop(c);
            self.out.append(&mut rd.out);
        }
    }

    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if c.ch.rdr2 {
            let rp = self.rp;
            let rd = self.rd.get_or_insert_with(|| Box::new(crate::extras_fall_rdr2::TeeterRdr2::new(rp)));
            rd.tick(c, dt);
            self.out.append(&mut rd.out);
            return;
        }
        let (com, vel) = (c.com, c.vel);
        if self.normal == Vec3::ZERO {
            if self.ends.0 == self.ends.1 {
                let cl = (*crate::nmrs::CLAVICLE());
                let fwd = flat((c.b.part(cl[0] + 1).m.d - c.b.part(cl[1] + 1).m.d).cross(Vec3::Z));
                let side = Vec3::Z.cross(fwd);
                let at = Vec3::new(com.x, com.y, c.b.part((*ANKLE())[0] + 1).m.d.z) + fwd * 0.5;
                self.ends = (at + side * 0.5, at - side * 0.5);
            }
            let n = flat(Vec3::Z.cross(self.ends.1 - self.ends.0));
            self.normal = if (self.ends.0 - com).dot(n) > 0.0 { n } else { -n };
        }
        let (a, b) = self.ends;
        let com2edge = (a - com).dot(self.normal);
        let vel2edge = vel.dot(self.normal);
        self.t += dt;
        if self.state < FALLING && (com2edge < 0.0 || com.z < a.z - delirium_data::kv!(24)) {
            self.to(FALLING, "falling");
            self.look(com + vel);
            self.out.push("leanInDirection dir=0,0,1 leanAmount=0".into());
            if self.high_fall {
                self.out.push("highFall start=true".into());
            }
            return;
        }
        match self.state {
            PRE if self.t >= self.pre && com2edge < self.react => {
                self.to(TEETERING, "teetering");
                let u = ((com - a).dot(b - a) / (b - a).length_squared().max(delirium_data::kv!(50))).clamp(0.0, 1.0);
                self.look(a + (b - a) * u);
                if self.lean {
                    let d = -self.normal;
                    self.out.push(format!("leanInDirection dir={},{},0 leanAmount={}", d.x, d.y, self.lean_scale));
                }
            }
            TEETERING => {
                if self.lean && self.t >= self.lean_time && self.t - dt < self.lean_time {
                    self.out.push("leanInDirection dir=0,0,1 leanAmount=0".into());
                }
                let urgency = (1.0 - com2edge / self.react.max(delirium_data::kv!(51)) + vel2edge.max(0.0)).clamp(delirium_data::kv!(24), 1.0);
                c.ch.set_stiffness(delirium_data::kv!(21), 0.5, "ua", None);
                windmill(c, self.t * delirium_data::kv!(65), delirium_data::kv!(28) * urgency);
                for &j in &(*SPINE()) {
                    c.ch.set_relative(j, 2, delirium_data::kv!(24) * urgency * (self.t * delirium_data::kv!(64)).sin());
                }
                if self.t >= self.teeter_time && com2edge > self.restart {
                    self.to(RECOVERED(), "recovered");
                    if self.head {
                        self.out.push("headLook start=false".into());
                    }
                }
            }
            _ => {}
        }
    }

    fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.out)
    }

    fn done(&self) -> bool {
        self.rd.is_none() && self.state >= FALLING
    }
}

pub struct Stumble {
    legs: f32, torso: f32, arms_k: f32, twist: f32, fwd_off: f32, back_off: f32, brace: bool,
    rate: f32, max_back: f32, max_fwd: f32, towards: f32, z_spin: f32, damp_pelvis: f32, twist_spine: bool,
    head: bool, stagger: f32, drop: f32, different: bool,
    lean: f32, sent: f32, z0: f32, state: u8, t: f32, out: Vec<String>,
}
impl Stumble {
    fn new(p: &mut P) -> Result<Stumble, Error> {
        p.f("armReduceSpeed", delirium_data::kv!(60))?; p.f("wristMS", delirium_data::kv!(30))?; p.f("feetMS", delirium_data::kv!(30))?; p.s("fallMask", "")?;
        p.f("grabRadius2", 0.5)?; p.f("pitchInContact", 0.0)?;
        Ok(Stumble { legs: p.f("legsStiffness", delirium_data::kv!(21))?, torso: p.f("torsoStiffness", delirium_data::kv!(31))?, arms_k: p.f("armsStiffness", delirium_data::kv!(31))?,
                     twist: p.f("armTwist", delirium_data::kv!(24))?, fwd_off: p.f("forwardMaxArmOffset", delirium_data::kv!(28))?,
                     back_off: p.f("backwardsMinArmOffset", delirium_data::kv!(60))?, brace: p.b("useArmsBrace", true)?,
                     rate: p.f("leanRate", 0.5)?, max_back: p.f("maxLeanBack", delirium_data::kv!(78))?, max_fwd: p.f("maxLeanForward", delirium_data::kv!(24))?,
                     towards: p.f("leanTowards", delirium_data::kv!(60))?, z_spin: p.f("zAxisSpinReduction", 0.5)?,
                     damp_pelvis: p.f("dampPelvis", 0.0)?, twist_spine: p.b("twistSpine", true)?,
                     head: p.b("useHeadLook", true)?, stagger: p.f("staggerTime", delirium_data::kv!(12))?, drop: p.f("dropVal", delirium_data::kv!(116))?,
                     different: p.b("different", false)?, lean: 0.0, sent: 0.0, z0: f32::NAN, state: 0, t: 0.0, out: vec![] })
    }

    fn end(&mut self, state: u8, name: &str) {
        self.state = state;
        self.out.push(format!("#say stumble {name}"));
        if self.head {
            self.out.push("headLook start=false".into());
        }
        if state == 2 {
            self.out.push(format!("catchFall start=true useHeadLook={} armsStiffness={} torsoStiffness={} legsStiffness={} \
                                   forwardMaxArmOffset={} backwardsMinArmOffset={} zAxisSpinReduction={}",
                                  self.head, self.arms_k, self.torso, self.legs, self.fwd_off, self.back_off, self.z_spin));
        }
        self.out.push("leanInDirection dir=0,0,1 leanAmount=0".into());
    }
}
impl Extra for Stumble {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        let w = weight(c);
        let d = travel(c, -Vec3::Y);
        let first = self.t == 0.0;
        if first {
            self.z0 = c.com.z;
            self.out.push("#say stumble start".into());
            self.out.push(format!("bodyBalance start=true bodyStiffness={} useHeadLook=false", self.torso));
            if self.head {
                let p = c.com + d * 2.0;
                self.out.push(format!("headLook start=true pos={},{},{} stiffness={} damping=1", p.x, p.y, p.z, delirium_data::kv!(117)));
            }
        }
        self.t += dt;
        if self.state != 0 {
            return;
        }
        if c.com.z < self.z0 - self.drop || self.t >= self.stagger {
            return self.end(2, "down");
        }
        if self.t > 0.5 && c.vel.truncate().length() < delirium_data::kv!(24) {
            return self.end(1, "recovered");
        }
        c.ch.set_stiffness(self.torso, 0.5, "ub", None);
        c.ch.set_stiffness((self.arms_k - 1.0).max(0.0), 0.5, "ua", None);
        c.ch.set_stiffness(self.legs, 0.5, "lb", None);
        let goal = self.towards.clamp(-self.max_back, self.max_fwd);
        self.lean += (goal - self.lean).clamp(-self.rate * dt, self.rate * dt);
        if (self.lean - self.sent).abs() > delirium_data::kv!(118) {
            self.sent = self.lean;
            self.out.push(format!("leanInDirection dir={},{},0 leanAmount={}", d.x, d.y, self.lean));
        }
        c.out.torque(0, -Vec3::Z * (c.angvel.z * self.z_spin * w * delirium_data::kv!(78)));
        if self.damp_pelvis > 0.0 {
            c.out.torque(0, -c.angvel * (self.damp_pelvis * w * delirium_data::kv!(78)));
        }
        if self.brace {
            let u = (self.lean / self.max_fwd.max(delirium_data::kv!(51))).clamp(0.0, 1.0);
            let reach = self.back_off + (self.fwd_off - self.back_off) * u;
            for i in 0..2 {
                let s = if i == 0 { 1.0 } else { -1.0 };
                let r = if self.different && i == 1 { reach * delirium_data::kv!(119) } else { reach };
                c.ch.set_relative((*SHOULDER())[i], 0, r);
                c.ch.set_relative((*SHOULDER())[i], 1, delirium_data::kv!(24));
                c.ch.set_relative((*SHOULDER())[i], 2, s * self.twist);
                c.ch.set_relative((*ELBOW())[i], 0, -delirium_data::kv!(60));
            }
        }
        if self.twist_spine {
            for &j in &(*SPINE()) {
                c.ch.set_relative(j, 2, delirium_data::kv!(60) * (self.t * delirium_data::kv!(84)).sin());
            }
        }
    }

    fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.out)
    }

    fn done(&self) -> bool {
        self.state != 0
    }
}

pub struct StaggerFall {
    arm: [f32; 2],
    arm_start: [f32; 2],
    spine: [f32; 2],
    spine_start: [f32; 2],
    hold: f32,
    ramp: f32,
    legs: f32,
    legs_end: f32,
    prob: f32,
    steps_end: f32,
    time_end: f32,
    ramp_end: f32,
    reduce: f32,
    push_dist: f32,
    push_vel: f32,
    lean_rate: f32,
    lean_max: [f32; 2],
    hips_max: [f32; 2],
    lean2: [f32; 2],
    hip_bend: f32,
    spine_bend: f32,
    always_fwd: bool,
    head_pos: Vec3,
    head_vel_prob: f32,
    head: bool,
    turn: String,
    t: f32,
    t_end: f32,
    steps: u32,
    side: f32,
    push: f32,
    state: u8,
    seed: u32,
    out: Vec<String>,
    /// RDR2's, with the RDR2 core: its parameters and state
    rp: StgParams,
    rd: Option<StgRdr2>,
}
impl StaggerFall {
    fn new(p: &mut P) -> Result<StaggerFall, Error> {
        let arm = [p.f("armStiffness", delirium_data::kv!(35))?, p.f("armDamping", 0.5)?];
        let spine = [p.f("spineStiffness", delirium_data::kv!(65))?, p.f("spineDamping", 0.5)?];
        let arm_start = [p.f("armStiffnessStart", delirium_data::kv!(21))?, p.f("armDampingStart", 1.0)?];
        let spine_start = [p.f("spineStiffnessStart", delirium_data::kv!(21))?, p.f("spineDampingStart", 1.0)?];
        let (hold, ramp) = (p.f("timeAtStartValues", 0.5)?, p.f("rampTimeFromStartValues", 1.0)?);
        let (legs, legs_end) = (p.f("lowerBodyStiffness", delirium_data::kv!(21))?, p.f("lowerBodyStiffnessEnd", delirium_data::kv!(5))?);
        let prob = p.f("staggerStepProb", 0.5)?;
        let steps_end = p.i("stepsTillStartEnd", delirium_data::kv!(6 int))? as f32;
        let (time_end, ramp_end) = (p.f("timeStartEnd", delirium_data::kv!(35))?, p.f("rampTimeToEndValues", 2.0)?);
        p.f("predictionTime", delirium_data::kv!(24))?;
        let reduce = p.f("perStepReduction1", delirium_data::kv!(19))?;
        let (push_dist, push_vel) = (p.f("pushOffDist", delirium_data::kv!(78))?, p.f("maxPushoffVel", 1.0)?);
        let lean_rate = p.f("leanInDirRate", 1.0)?;
        let lean_max = [p.f("leanInDirMaxF", delirium_data::kv!(24))?, p.f("leanInDirMaxB", delirium_data::kv!(9))?];
        let hips_max = [p.f("leanHipsMaxF", delirium_data::kv!(24))?, p.f("leanHipsMaxB", delirium_data::kv!(78))?];
        let lean2 = [p.f("lean2multF", 1.0)?, p.f("lean2multB", 1.0)?];
        let (hip_bend, spine_bend) = (p.f("hipBendMult", 1.0)?, p.f("spineBendMult", 1.0)?);
        let always_fwd = p.b("alwaysBendForwards", false)?;
        let head_pos = p.v("headLookPos", Vec3::ZERO)?;
        p.i("headLookInstanceIndex", -1)?;
        let head_vel_prob = p.f("headLookAtVelProb", 0.5)?;
        let turn_off = p.f("turnOffProb", delirium_data::kv!(24))?;
        let (t2t, t2v) = (p.f("turn2TargetProb", delirium_data::kv!(78))?, p.f("turn2VelProb", delirium_data::kv!(24))?);
        let (away, left, right) = (p.f("turnAwayProb", delirium_data::kv!(78))?, p.f("turnLeftProb", delirium_data::kv!(78))?, p.f("turnRightProb", delirium_data::kv!(78))?);
        let body_turn = p.b("useBodyTurn", true)?;
        let head = p.b("useHeadLook", true)?;
        let turn = format!("turnOffProb={turn_off} turn2TargetProb={t2t} turn2VelProb={t2v} turnAwayProb={away} turnLeftProb={left} \
                            turnRightProb={right} useBodyTurn={body_turn}");
        Ok(StaggerFall { arm, arm_start, spine, spine_start, hold, ramp, legs, legs_end, prob, steps_end, time_end, ramp_end, reduce,
                         push_dist, push_vel, lean_rate, lean_max, hips_max, lean2, hip_bend, spine_bend, always_fwd, head_pos,
                         head_vel_prob, head, turn, t: 0.0, t_end: 0.0, steps: 0, side: 0.0, push: 0.0, state: 0, seed: delirium_data::kv!(120 int),
                         out: vec![], rp: StgParams::read(p)?, rd: None })
    }

    fn rand(&mut self) -> f32 {
        self.seed = self.seed.wrapping_mul(delirium_data::kv!(121 int)).wrapping_add(delirium_data::kv!(122 int));
        (self.seed >> delirium_data::kv!(23 int)) as f32 / (1u32 << delirium_data::kv!(107 int)) as f32
    }
}
impl Extra for StaggerFall {
    fn stop(&mut self, _c: &mut Ctx) {
        if let Some(rd) = self.rd.as_ref() {
            if self.rp.head {
                self.out.push("headLook start=false".into());
            }
            if rd.handed {
                self.out.push("catchFall start=false".into());
                self.out.push("rollDownStairs start=false".into());
            }
        }
    }

    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if c.ch.rdr2 {
            let rp = self.rp;
            let rd = self.rd.get_or_insert_with(|| StgRdr2::new(c, rp, &mut self.out));
            rd.tick(c, &rp, &mut self.out, dt);
            return;
        }
        let cl = (*crate::nmrs::CLAVICLE());
        let fwd = flat((c.b.part(cl[0] + 1).m.d - c.b.part(cl[1] + 1).m.d).cross(Vec3::Z));
        let dir = travel(c, fwd);
        if self.t == 0.0 {
            self.out.push(format!("bodyBalance start=true bodyStiffness={} useHeadLook=false {}", self.legs, self.turn));
            self.out.push("#say staggerFall staggering".into());
        }
        self.t += dt;
        let s = (c.b.part((*ANKLE())[0] + 1).m.d - c.b.part((*ANKLE())[1] + 1).m.d).dot(dir);
        let stepped = s.abs() > delirium_data::kv!(68) && self.side != 0.0 && s.signum() != self.side;
        if s.abs() > delirium_data::kv!(68) {
            self.side = s.signum();
        }
        let fwd_dot = dir.dot(fwd);
        let k = (fwd_dot < 0.0) as usize;
        if stepped {
            self.steps += 1;
            if self.rand() < self.prob {
                self.push = delirium_data::kv!(9);
            }
            let amount = (self.lean_rate * self.t).min(self.lean_max[k]);
            self.out.push(format!("leanInDirection dir={},{},{} leanAmount={amount}", dir.x, dir.y, dir.z));
            if self.head {
                let v = self.head_vel_prob;
                let at_vel = v > 1.0 || (v < 0.0 && self.rand() < 0.5) || (v > 0.0 && self.rand() < v);
                let at = if self.head_pos == Vec3::ZERO { c.com + dir * 2.0 } else { self.head_pos };
                self.out.push(if at_vel {
                    format!("headLook start=true vel={},{},{} stiffness={} damping=1", c.vel.x, c.vel.y, c.vel.z, delirium_data::kv!(117))
                } else {
                    format!("headLook start=true pos={},{},{} stiffness={} damping=1", at.x, at.y, at.z, delirium_data::kv!(117))
                });
            }
        }
        if self.state == 0 && (self.steps as f32 >= self.steps_end || self.t >= self.time_end) {
            (self.state, self.t_end) = (1, 0.0);
            self.out.push("#say staggerFall winding down".into());
        }
        if self.state == 1 {
            self.t_end += dt;
        }
        let u = ((self.t - self.hold) / self.ramp.max(dt)).clamp(0.0, 1.0);
        let e = if self.state == 1 { (self.t_end / self.ramp_end.max(dt)).min(1.0) } else { 0.0 };
        let fat = self.reduce.powi(self.steps as i32).max(delirium_data::kv!(60));
        let mix = |a: [f32; 2], b: [f32; 2], i: usize| (a[i] + (b[i] - a[i]) * u) * fat * (1.0 - e) + e;
        c.ch.set_stiffness(mix(self.arm_start, self.arm, 0), mix(self.arm_start, self.arm, 1), "ua", None);
        c.ch.set_stiffness(mix(self.spine_start, self.spine, 0), mix(self.spine_start, self.spine, 1), "us", None);
        c.ch.set_stiffness(self.legs + (self.legs_end - self.legs) * e, 0.5, "lb", None);
        for j in (*NECK()).into_iter().chain((*SPINE())[..delirium_data::kv!(0 int)].iter().copied()).chain(cl) {
            c.ch.eff[j].oppose_gravity = 0.0;
        }
        let sign = if self.always_fwd || fwd_dot >= 0.0 { 1.0 } else { -1.0 };
        let lean = (self.lean_rate * self.t).min(1.0);
        let hips = (delirium_data::kv!(24) * self.hip_bend + self.hips_max[k] * self.lean2[k] * lean).min(1.0);
        for i in 0..2 {
            c.ch.set_relative((*HIP())[i], 0, -sign * hips);
        }
        for &j in &(*SPINE())[..delirium_data::kv!(0 int)] {
            c.ch.set_relative(j, 0, sign * (delirium_data::kv!(24) * self.spine_bend).min(1.0));
        }
        if self.push > 0.0 {
            self.push -= dt;
            let v = (self.push_dist / delirium_data::kv!(9)).min(self.push_vel);
            c.out.force(0, dir * (c.b.doll.mass() * v / delirium_data::kv!(9)));
        }
        if (c.dbm_failed && self.t > 0.5) || self.t_end > self.ramp_end + delirium_data::kv!(84) {
            self.state = 2;
            self.out.push("#say staggerFall fallen".into());
            for j in (*NECK()).into_iter().chain((*SPINE())[..delirium_data::kv!(0 int)].iter().copied()).chain(cl).chain((*ANKLE())).chain((*KNEE())).chain((*HIP())) {
                c.ch.eff[j].oppose_gravity = 1.0;
            }
            if self.head {
                self.out.push("headLook start=false".into());
            }
        }
    }

    fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.out)
    }

    fn done(&self) -> bool {
        self.state == 2
    }
}

/// RDR2's staggerFall parameters (RDR2's names and defaults; its unnamed values are consts below). Read with every
/// staggerFall message, used with the RDR2 core.
#[derive(Clone, Copy, Debug)]
struct StgParams {
    /// arm and spine (stiffness, damping) once ramped, and at the start
    arm: [f32; 2],
    spine: [f32; 2],
    arm_start: [f32; 2],
    spine_start: [f32; 2],
    hold: f32,
    ramp: f32,
    steps_end: i32,
    legs: f32,
    /// lowerBodyStiffnessEnd, timeStartEnd, rampTimeToEndValues: the legs' ramp in time
    legs_end: [f32; 3],
    /// strikeReduction forward and back; perStepReduction1; all the leg values times this going back
    strike: [f32; 2],
    per_step: f32,
    legs_back: f32,
    prediction: [f32; 2],
    lean_rate: f32,
    /// per direction (forward, back): the COM and hip lean noise's (min, max, frequency), the total lean's (min, max),
    /// the extra hip lean per step past stepsTillStartEnd
    com: [[f32; 3]; 2],
    hip: [[f32; 3]; 2],
    total: [[f32; 2]; 2],
    bonus: [f32; 2],
    push_dist: [f32; 2],
    push_knee: f32,
    push_vel: f32,
    force_thresh: f32,
    head_vel_prob: f32,
    head: bool,
    time_to_fall: f32,
    fall_type: i32,
    firing: bool,
    upper_body: bool,
}

impl StgParams {
    fn read(p: &mut P) -> Result<StgParams, Error> {
        let arm = [p.f("armStiffness", 11.0)?, p.f("armDamping", 1.0)?];
        let spine = [p.f("spineStiffness", 11.0)?, p.f("spineDamping", 0.7)?];
        let arm_start = [p.f("armStiffnessStart", 8.0)?, p.f("armDampingStart", 0.1)?];
        let spine_start = [p.f("spineStiffnessStart", 3.0)?, p.f("spineDampingStart", 0.1)?];
        let (hold, ramp) = (p.f("timeAtStartValues", 0.0)?, p.f("rampTimeFromStartValues", 0.0)?);
        p.f("staggerStepProb", 0.0)?;
        let steps_end = p.i("stepsTillStartEnd", 100)?;
        let legs = p.f("lowerBodyStiffness", 17.0)?;
        let legs_end = [p.f("lowerBodyStiffnessEnd", 8.0)?, p.f("timeStartEnd", 100.0)?, p.f("rampTimeToEndValues", 0.0)?];
        let strike = [p.f("strikeReduction", 1.0)?, p.f("strikereductionb", 1.0)?];
        let (per_step, legs_back) = (p.f("perStepReduction1", 0.0)?, p.f("lowerbodystiffnessb", 1.0)?);
        let prediction = [p.f("predictionTime", 0.15)?, p.f("predictiontimeb", 0.15)?];
        let lean_rate = p.f("leanInDirRate", 1.0)?;
        let com = [[0.4, p.f("leanInDirMaxF", 0.4)?, 4.0], [0.3, p.f("leanInDirMaxB", 0.5)?, 4.0]];
        let hip = [[0.1, p.f("leanHipsMaxF", 0.4)?, 4.0], [-0.2, p.f("leanHipsMaxB", 0.2)?, 6.0]];
        p.f("lean2multF", -1.0)?;
        p.f("lean2multB", -1.0)?;
        let push_dist = [p.f("pushOffDist", 0.28)?, p.f("pushoffdistb", 0.23)?];
        let (push_knee, push_vel) = (p.f("pushoffknee", -0.25)?, p.f("maxPushoffVel", 1.2)?);
        let force_thresh = p.f("forcebalancethresh", 1.0)?;
        let head_vel_prob = p.f("headLookAtVelProb", 1.0)?;
        let head = p.b("useHeadLook", true)?;
        let (time_to_fall, fall_type) = (p.f("timeToFall", 2.0)?, p.i("fallType", 4)?);
        let firing = p.b("firingInputOutput", false)?;
        let upper_body = p.b("upperBodyReaction", true)?;
        Ok(StgParams { arm, spine, arm_start, spine_start, hold, ramp, steps_end, legs, legs_end, strike, per_step, legs_back, prediction, lean_rate, com, hip,
                       total: STG_TOTAL, bonus: STG_BONUS, push_dist, push_knee, push_vel, force_thresh, head_vel_prob, head,
                       time_to_fall, fall_type, firing, upper_body })
    }
}

/// RDR2's unnamed staggerFall values: the total lean's (min, max) forward and back, the extra hip lean per step past
/// stepsTillStartEnd forward and back, the knee and hip muscle stiffness (the ankles' is 4), the leg stiffness floors
/// (knee and hip, ankle), the COM-height weakening (from, to in leg lengths, and the least it leaves).
const STG_TOTAL: [[f32; 2]; 2] = [[-10.0, 10.0], [0.4, 0.5]];
const STG_BONUS: [f32; 2] = [0.0, 0.1];
const STG_LEG_MUSCLE: f32 = 0.7;
const STG_ANKLE_MUSCLE: f32 = 4.0;
const STG_FLOOR: [f32; 2] = [5.0, 7.0];
const STG_WEAK: [f32; 3] = [0.65, 0.75, 0.3];
/// Below this speed (m/s) the lean goes along the body's back axis, not the velocity; below STG_SLOW the balance may be
/// forced (no new steps).
const STG_DIR_SPEED: f32 = 0.6;
const STG_SLOW: f32 = 0.65;
/// How long a foot counts as just stepped (s), and the head look's stiffness.
const STG_JUST: f32 = 1.0 / 15.0;
const STG_HEAD: f32 = 11.0;

/// RDR2's staggerFall (with the RDR2 core), from a study of RDR2's code (rdr2-decomp/study/rdr2_stagger.md): the
/// balancer runs in stagger mode and staggerFall drives its COM and hip leans with noise between per-direction limits;
/// it counts the balancer's steps, lets the stance leg stop opposing gravity or pushes it off with the knee, sets the
/// legs' stiffness (weaker as the COM drops) and ramps the spine and arms; once the balancer has given up it hands
/// the fall to catchFall (and rollDownStairs), unless the shot is running.
struct StgRdr2 {
    t: f32,
    seed: f32,
    last: u32,
    steps: i32,
    just: [f32; 2],
    leg: [u8; 2],
    force: bool,
    latch: bool,
    fell: bool,
    handed: bool,
    leg_len: f32,
}

impl StgRdr2 {
    /// Activation: opposeGravity off on the lower spine, neck and clavicles, the head look, the noise seed.
    fn new(c: &mut Ctx, p: StgParams, out: &mut Vec<String>) -> StgRdr2 {
        let seed = 4000.0 * c.ch.random();
        for j in (*SPINE()).iter().take(3).chain(NECK().iter()).chain(CLAVICLE().iter()).copied() {
            c.ch.eff[j].oppose_gravity = 0.0;
        }
        // RDR2 starts the balancer itself; here a balancer is asked for when none runs (ours)
        if c.dbm.is_none() && c.upright {
            out.push("bodyBalance start=true".into());
        }
        if p.head {
            out.push(format!("headLook start=true vel={},{},{} stiffness={STG_HEAD} damping=1", c.vel.x, c.vel.y, c.vel.z));
        }
        let leg_len = (c.b.joint_pos((*HIP())[0]) - c.b.joint_pos((*ANKLE())[0])).length().max(0.1);
        StgRdr2 { t: 0.0, seed, last: 0, steps: 0, just: [0.0; 2], leg: [0; 2], force: false, latch: false, fell: false, handed: false,
                  leg_len }
    }

    fn foot(c: &Ctx, i: usize) -> usize {
        c.ch.joints[(*ANKLE())[i]].child
    }

    fn touching(c: &Ctx, i: usize) -> bool {
        c.b.doll.touch[Self::foot(c, i)] & delirium_data::kv!(7 int) != 0
    }

    fn tick(&mut self, c: &mut Ctx, p: &StgParams, out: &mut Vec<String>, dt: f32) {
        let Some(bal) = c.dbm.filter(|_| c.upright) else {
            // the balancer has given up: hand the fall over, unless the shot is running and takes it
            if !self.handed && (p.firing || !c.shot_on) {
                self.handed = true;
                self.hand_over(c, out);
            }
            return;
        };
        c.out.dbm_stagger = true;
        let up = c.up;
        let hz = |v: Vec3| v - up * v.dot(up);
        let (v, back_axis) = (c.vel, c.m.c);
        let vn = v.normalize_or_zero();
        let backward = back_axis.dot(vn) > 0.0;
        let k = backward as usize;
        let speed = hz(v).length();
        let feet = [0, 1].map(|i| c.b.part(Self::foot(c, i)).m.d);
        // no new steps when slow with the COM steeply over both feet
        if bal.stepping == 0 {
            self.force = speed < STG_SLOW && feet.iter().all(|&f| up.dot((c.com - f).normalize_or_zero()) > p.force_thresh);
        }
        c.out.dbm_force_balance = self.force || (p.fall_type == 1 && self.fell);
        c.out.dbm_lead = Some(p.prediction[k]);
        // the balancer's steps
        if bal.stepping != 0 && bal.stepping != self.last {
            self.steps += 1;
            self.just[bal.stepping as usize - 1] = STG_JUST;
            if p.head && (p.head_vel_prob >= 1.0 || c.ch.random() < p.head_vel_prob) {
                out.push(format!("headLook start=true vel={},{},{} stiffness={STG_HEAD} damping=1", v.x, v.y, v.z));
            }
        }
        self.last = bal.stepping;
        for j in &mut self.just {
            *j -= dt;
        }
        // the leans: 3-D noise between each direction's limits, the total clamped, the COM's ramped in
        let t = self.t;
        let [cn, cx, cf] = p.com[k];
        let [hn, hx, hf] = p.hip[k];
        let hf = if self.force { 0.1 } else { hf };
        let mut com = cn + (cx - cn) * crate::nmfl::noise01(t * cf, self.seed, -0.5);
        let mut hip = hn + (hx - hn) * crate::nmfl::noise01(self.seed, t * hf, -0.5);
        let [lo, hi] = p.total[k];
        let total = com + hip;
        if total > hi + 1e-6 {
            (com, hip) = (com * hi / total, hip * hi / total);
        } else if total < lo {
            if com > 0.0 && hip > 0.0 {
                (com, hip) = (com * lo / total, hip * lo / total);
            } else if com <= 0.0 {
                com = lo - hip;
            } else {
                hip = lo - com;
            }
        }
        com *= (t * p.lean_rate).clamp(0.0, 1.0);
        let dir = if speed < STG_DIR_SPEED {
            hz(back_axis).normalize_or_zero() * if backward { 1.0 } else { -1.0 }
        } else {
            hz(v).normalize_or_zero()
        };
        let bonus = if self.steps > p.steps_end { (self.steps - p.steps_end - 1) as f32 * p.bonus[k] } else { 0.0 };
        // ours: the side factor from the body's back axis (the note leaves which row of the spine root it is)
        let f = hz(back_axis).normalize_or_zero().dot(hz(v).normalize_or_zero()).abs();
        c.out.hip_lean = Some(dir * (hip + bonus) * f);
        c.out.lean = Some((crate::nmdbm::LeanMode::Dir(dir), (com + bonus) * f, false));
        // the spine twists against the hips' swing (going forward)
        if !backward {
            let a = [0, 1].map(|i| c.ch.actual((*HIP())[i], c.b.frames)[0]);
            let d = [0, 1].map(|i| c.ch.eff[(*HIP())[i]].desired[0]);
            let l = ((a[0] - a[1]) + 0.15 * (d[0] - d[1])).clamp(-0.8, 0.8);
            for (&j, w) in (*SPINE()).iter().zip([0.35, 0.3, 0.2, 0.15]) {
                c.ch.eff[j].desired[2] = w * l;
            }
        }
        // the upper body's ramp from the start values
        let lerp = |a: [f32; 2], b: [f32; 2], u: f32| [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u];
        let u = if t > p.hold + 1e-6 { if t > p.hold + p.ramp { 1.0 } else { (t - p.hold) / p.ramp } } else { 0.0 };
        // (the arms' pair ramps too; no reader of it was found: `arms` has its own)
        let spine = lerp(p.spine_start, p.spine, u);
        // the legs' base: lowerBodyStiffness, ramped to its End in time (0x2a9e8a4)
        let [end, t_end, r_end] = p.legs_end;
        let base = if t >= t_end + r_end - 1e-6 {
            end
        } else if t >= t_end {
            p.legs + (end - p.legs) * (t - t_end) / r_end
        } else {
            p.legs
        };
        self.t += dt;
        // the legs: the stance leg stops opposing gravity, or pushes off with the knee once the COM is past it
        let prev = self.leg;
        self.leg = [0; 2];
        match bal.stepping {
            1 => self.leg[1] = 2,
            2 => self.leg[0] = 2,
            _ => {}
        }
        let at = if backward { [0, 1].map(|i| c.b.joint_pos((*ANKLE())[i])) } else { feet };
        let past = [0, 1].map(|i| hz(c.com - at[i]).dot(hz(v)));
        let rise = c.b.part(CHEST()).v.dot(up);
        let fall_time = self.t > p.time_to_fall && c.out.dbm_force_balance;
        for i in 0..2 {
            let o = 1 - i;
            if !(fall_time || (bal.stepping as usize == o + 1 && !bal.planted[o])) {
                continue;
            }
            let touch = Self::touching(c, i);
            if past[i] <= p.push_dist[k] || !touch || rise >= p.push_vel {
                if touch {
                    for j in [(*ANKLE())[i], (*KNEE())[i], (*HIP())[i]] {
                        c.ch.eff[j].oppose_gravity = 0.0;
                    }
                    self.leg[i] = 1;
                }
            } else {
                self.leg[i] = 3;
                let knee = &mut c.ch.eff[(*KNEE())[i]].desired[0];
                if *knee > p.push_knee {
                    self.leg[i] = 4;
                    *knee += (25.0 * (*knee - p.push_knee)).clamp(0.0, 2.5);
                }
            }
            if self.just[o] >= 0.0 && past[o] > 0.2 {
                self.leg[o] = 5;
            }
        }
        c.out.dbm_push_off = self.leg.iter().any(|&l| l == 3 || l == 4);
        // leg stiffness (0x2a9e8a4). (hip, knee, ankle) of a leg with no state: base - 1, base - 3, base - 1; a stance leg, the
        // other foot stepping: base, base, base + 2; once released (it stopped opposing gravity): the hip less the strike
        // reduction, the knee less twice it, the ankle base. Each less the steps past stepsTillStartEnd's reduction.
        // Ours: the balancer's fail-ramp caps aren't here (its own ramp lowers the legs after this), nor fallreduction
        // (0 in every set; it needs two balancer fields not traced).
        let past_end = |n: i32| if self.steps > p.steps_end + n { p.per_step * (self.steps - p.steps_end - n) as f32 } else { 0.0 };
        let strike = p.strike[k];
        let red = c.ch.bcr.map_or(0.0, |b| b.2);
        let [w0, w1, wmin] = STG_WEAK;
        // COM height over the contact points in leg lengths, a clamped line (0x9a0434), squared
        let h = feet.iter().map(|&f| up.dot(c.com - f)).fold(f32::MIN, f32::max) / self.leg_len;
        let g = ((h - w0) / (w1 - w0)).clamp(0.0, 1.0);
        let w = wmin + (1.0 - wmin) * g * g;
        for i in 0..2 {
            let (hip, knee, ankle) = if self.leg[i] == 1 {
                let r = past_end(1);
                (base - strike - r, base - 2.0 * strike - r, base - r)
            } else if self.leg[i] != 0 {
                let r = past_end(0);
                (base - r, base - r, base + 2.0 - r)
            } else {
                let r = past_end(1);
                (base - 1.0 - r, base - 3.0 - r, base - 1.0 - r)
            };
            let scale = if backward { p.legs_back } else { 1.0 };
            let (mut hip, mut knee) = (((hip * scale) - red).max(STG_FLOOR[0]), ((knee * scale) - red).max(STG_FLOOR[0]));
            let ankle = ((ankle * scale) - red).max(STG_FLOOR[1]);
            // the stance leg's, both when neither steps, weaker as the COM drops: not the ankle
            if w < 1.0 && bal.stepping as usize != i + 1 {
                (hip, knee) = (hip * w, knee * w);
            }
            c.ch.eff[(*KNEE())[i]].set_stiffness(knee, 0.9, Some(STG_LEG_MUSCLE));
            c.ch.eff[(*ANKLE())[i]].set_stiffness(ankle, 1.0, Some(STG_ANKLE_MUSCLE));
            c.ch.eff[(*HIP())[i]].set_stiffness(hip, 1.0, Some(STG_LEG_MUSCLE));
        }
        c.ch.set_stiffness(spine[0], spine[1], "us", None);
        c.ch.set_stiffness(spine[0], spine[1], "un", None);
        self.arms(c, p);
        // fall types (timeToFall)
        let elapsed = self.t > p.time_to_fall;
        let leaves = |s: u8| (0..2).any(|i| prev[i] == s && self.leg[i] != s);
        if !self.fell {
            match p.fall_type {
                0 if elapsed && leaves(4) => {
                    self.fell = true;
                    c.out.dbm = Some(false);
                }
                1 | 2 if elapsed => self.fell = true,
                3 => self.latch = true,
                4 if elapsed && leaves(1) => self.latch = true,
                5 if elapsed && leaves(4) => self.latch = true,
                _ => {}
            }
        }
        c.out.dbm_falling = elapsed && self.latch;
    }

    /// RDR2's stagger arms (0x2a7af24, upperBodyReaction): while the chest moves fast and not backward, the arms' pose is
    /// left to the brace helper (0x2a7741c, an IK reach not ported: here they keep what lower behaviours ask); otherwise
    /// the arms are held by how far the chest is from the pelvis, plus their relative speed. armsWindmillAdaptive, which it
    /// starts going back fast without the shot's brace, isn't started here.
    fn arms(&mut self, c: &mut Ctx, p: &StgParams) {
        if !p.upper_body {
            return;
        }
        let up = c.up;
        let sp = crate::nmshot::spine_parts();
        let (top, pel, mid) = (c.b.part(sp[4]), c.b.part(sp[0]), c.b.part(sp[2]));
        let mut vc = (top.v + pel.v) * 0.3;
        // lying on the back the speed along the chest's axis doesn't count
        if up.dot(mid.m.c) > 0.8 {
            vc -= mid.m.c * vc.length();
        }
        let spd = vc.length();
        let back = vc.normalize_or_zero().dot(top.m.c);
        let side = c.vel.normalize_or_zero().dot(c.m.a);
        let mut free = [true; 2];
        if (back < 0.2 || (back < 0.7 && side.abs() > 0.7)) && spd > 0.5 {
            // each arm by a draw until he moves fast, then both; going sideways only the arm on that side
            let (mut l, mut r) = (true, true);
            if spd < 1.2 {
                l = 0.6 < c.ch.random() * (spd + 0.5) - 0.4;
                r = 0.6 < c.ch.random() * (spd + 0.5) - 0.4;
            }
            if side < -0.7 {
                (l, r) = (true, false);
            }
            free = if side <= 0.7 { [!l, !r] } else { [true, false] };
        }
        if back > 0.2 && spd > 0.5 && !c.shot_on {
            free = [false; 2];
        }
        let d = (top.m.d - pel.m.d) + (top.v - pel.v) * 0.6;
        let a = (d.length() - 0.15).clamp(0.0, 1.0);
        for i in 0..2 {
            if !free[i] {
                continue;
            }
            let l = crate::nmshot::limb(i);
            for j in l.e {
                c.ch.eff[j].set_stiffness(a * 3.0 + 7.0, 1.0, None);
            }
            let m = c.b.eff3(l.e[1]).m;
            // as read: the two arms' frames aren't mirrored in this term
            let y = (4.0 * 0.707 * if i == 0 { (m.c - m.b).dot(d) } else { (m.b + m.c).dot(d) }).clamp(-8.0, 8.0);
            let (s, co) = (4.0 * m.a.dot(d)).sin_cos();
            let rr = co + y - 0.5 * a;
            let (ls, lc) = (c.ch.limits(l.e[1]), c.ch.limits(l.e[0]));
            for j in [l.e[0], l.e[1]] {
                c.ch.eff[j].oppose_gravity = a;
            }
            let twist = if i == 0 { 0.0 } else { -0.1 };
            c.ch.eff[l.e[1]].desired = [(0.5 * (ls[0] + ls[1]) - s) * a, (0.5 * (ls[2] + ls[3]) + rr) * a + (1.0 - a) * 0.2, twist];
            c.ch.eff[l.e[0]].desired = [0.0, if i == 0 { (rr + 0.5 * (lc[2] + lc[3])) * a } else { 0.0 }, 0.0];
            c.ch.eff[l.e[3]].desired = [0.0; 3];
            c.ch.eff[l.e[2]].desired[0] = if i == 0 { (1.0 - a) * 0.7 + a } else { 0.8 };
        }
        // slowing to a stop the upper body blends to its zero pose
        let f = (2.0 * c.b.doll.kinetic()).sqrt().clamp(0.001, 1.0);
        for i in 0..2 {
            for j in crate::nmshot::limb(i).e {
                c.ch.eff[j].blend_to_zero(1.0 - f);
            }
        }
        for &j in SPINE().iter().chain(NECK().iter()) {
            c.ch.eff[j].blend_to_zero(1.0 - f);
        }
    }

    /// The balancer gave up: the legs held where they are and soft, catchFall, and rollDownStairs lying face up.
    fn hand_over(&mut self, c: &mut Ctx, out: &mut Vec<String>) {
        for i in 0..2 {
            let (h, kn) = ((*HIP())[i], (*KNEE())[i]);
            let a = c.ch.actual(h, c.b.frames);
            let d = c.ch.eff[h].desired;
            c.ch.eff[h].desired = [a[0], d[1].min(0.0), d[2].min(0.0)];
            c.ch.eff[kn].desired[0] = c.ch.actual(kn, c.b.frames)[0];
        }
        c.ch.set_stiffness(7.0, 1.0, "lb", None);
        out.push("catchFall start=true legsStiffness=8.25 torsoStiffness=10 armsStiffness=13".into());
        if c.m.c.dot(c.up) < -0.8 {
            out.push(format!("rollDownStairs start=true stiffness=7 forcemag=0.4 asymmetricalLegs={} m_useArmToSlowDown=-0.9 \
                              m_armReachAmount=1.4 spinWhenInAir=true m_legPush=0.2 armReachLength=0.6", 0.2 + 0.6 * c.ch.random()));
        }
    }
}

pub struct SmartFall {
    stiff: (f32, f32),
    catch_time: f32,
    cutoff: f32,
    end_time: f32,
    arms: (f32, f32, f32, f32, f32),
    legs: String,
    orient: (bool, f32, f32),
    brace: f32,
    max: f32,
    normal: Vec3,
    balance: bool,
    fwd_roll: bool,
    limp: bool,
    rds: f32,
    tumble: f32,
    axis: Vec3,
    z0: f32,
    state: u8,
    t: f32,
    peak: f32,
    air: bool,
    started: Vec<&'static str>,
    out: Vec<String>,
    /// RDR2's, with the RDR2 core: its parameters and machine
    rp: SfParams,
    rd: Option<Box<SfRdr2>>,
}
const SF_AIR: u8 = 0;
const SF_LAND: u8 = 1;
const SF_ROLL: u8 = 2;
#[allow(non_snake_case)]
#[inline]
fn SF_CRASH() -> u8 { delirium_data::kv!(0 int) }
#[allow(non_snake_case)]
#[inline]
fn SF_DONE() -> u8 { delirium_data::kv!(1 int) }
impl SmartFall {
    fn new(p: &mut P) -> Result<SmartFall, Error> {
        let stiff = (p.f("bodyStiffness", delirium_data::kv!(31))?, p.f("bodydamping", delirium_data::kv!(119))?);
        let (catch_time, cutoff, end_time) = (p.f("catchfalltime", delirium_data::kv!(12))?, p.f("catchFallCutOff", delirium_data::kv!(24))?, p.f("crashOrLandCutOff", delirium_data::kv!(84))?);
        let arms = (p.f("armsUp", delirium_data::kv!(119))?, p.f("armAngSpeed", delirium_data::kv!(65))?, p.f("armAmplitude", delirium_data::kv!(74))?, p.f("armPhase", delirium_data::kv!(123))?,
                    p.f("armBendElbows", delirium_data::kv!(24))?);
        let legs = format!("pedalLegs start=true radius={} angularSpeed={} speedAsymmetry={} pedalLeftLeg={} pedalRightLeg=true \
                            centreForwards={} legStiffness={}", p.f("legRadius", delirium_data::kv!(9))?, p.f("legAngSpeed", delirium_data::kv!(65))?,
                           p.f("legAsymmetry", 0.0)?, p.b("legL", true)?, p.f("forwardOffsetOfLegIK", 0.0)?, p.f("legStrength", delirium_data::kv!(65))?);
        let orient = (p.b("orientateBodyToFallDirection", true)?, p.f("orientateTwist", delirium_data::kv!(24))?, p.f("orientateMax", delirium_data::kv!(20))?);
        let (brace, max) = (p.f("minSpeedForBrace", delirium_data::kv!(84))?, p.f("maxSpeedForRecoverableFall", delirium_data::kv!(21))?);
        let normal = p.v("landingNormal", Vec3::Z)?.normalize_or(Vec3::Z);
        let (balance, fwd_roll, limp) = (p.b("balance", true)?, p.b("forwardRoll", false)?, p.b("alanRickman", false)?);
        let rp = SfParams::read(p)?;
        let (rds, tumble) = (p.f("rdsForceMag", 2.0)?, p.f("tumble", 0.0)?);
        for n in ["pdStrength", "pdDamping", "sideD", "arms2LegsPhase", "hula", "aimAngleBase", "forwardVelRotation", "footVelCompScale",
                  "rdsTargetLinearVelocity", "rdsTargetLinearVelocityDecayTime"] {
            p.f(n, 0.0)?;
        }
        for n in ["arms2LegsSync", "adaptiveCircling", "useZeroPose_withForwardRoll", "ignorWorldCollisions"] {
            p.b(n, false)?;
        }
        Ok(SmartFall { stiff, catch_time, cutoff, end_time, arms, legs, orient, brace, max, normal, balance, fwd_roll, limp, rds,
                       tumble, axis: Vec3::ZERO, z0: 0.0, state: SF_AIR, t: 0.0, peak: 0.0, air: false, started: vec![], out: vec![], rp, rd: None })
    }

    fn to(&mut self, state: u8, name: &str) {
        (self.state, self.t) = (state, 0.0);
        self.out.push(format!("#say smartFall {name}"));
    }

    fn start(&mut self, name: &'static str, line: String) {
        self.out.push(line);
        self.started.push(name);
    }

    fn stop(&mut self, name: &'static str) {
        if self.started.contains(&name) {
            self.started.retain(|&s| s != name);
            self.out.push(format!("{name} start=false"));
        }
    }
}
impl Extra for SmartFall {
    fn probe(&mut self, c: &Ctx, world: &crate::collision::Collision, obs: &[crate::ragdoll::Obstacle]) {
        if c.ch.rdr2 {
            let rp = self.rp;
            self.rd.get_or_insert_with(|| Box::new(SfRdr2::new(rp))).probe(c, world, obs);
        }
    }

    fn stop(&mut self, c: &mut Ctx) {
        // RDR2's deactivate: fall to knees' exit
        if c.ch.rdr2 && self.rd.as_ref().is_some_and(|r| r.state == 7) {
            crate::nmftk_rdr2::exit(c);
            crate::nmftk_rdr2::flags_off(c);
        }
        if let Some(rd) = self.rd.as_mut() {
            for n in std::mem::take(&mut rd.on) {
                self.out.push(format!("{n} start=false"));
            }
        }
    }

    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if c.ch.rdr2 {
            let rp = self.rp;
            let rd = self.rd.get_or_insert_with(|| Box::new(SfRdr2::new(rp)));
            rd.tick(c, dt);
            self.out.append(&mut rd.out);
            return;
        }
        self.t += dt;
        let w = weight(c);
        if self.state == SF_AIR {
            self.peak = self.peak.max(-c.vel.z);
            if self.peak < delirium_data::kv!(12) {
                return;
            }
            if !self.air {
                self.air = true;
                self.t = 0.0;
                self.out.push("#say smartFall airborne".into());
                let cl = (*crate::nmrs::CLAVICLE());
                let fwd = flat((c.b.part(cl[0] + 1).m.d - c.b.part(cl[1] + 1).m.d).cross(Vec3::Z));
                let d = if c.vel.truncate().length() > delirium_data::kv!(24) { flat(c.vel) } else { fwd };
                self.axis = Vec3::Z.cross(d).normalize_or_zero();
                self.z0 = c.com.z;
                if !self.limp {
                    self.start("pedalLegs", self.legs.clone());
                }
            }
            if self.limp {
                c.ch.set_stiffness(2.0, delirium_data::kv!(20), "fb", None);
            } else {
                c.ch.set_stiffness(self.stiff.0, self.stiff.1, "fb", None);
                let (up, rate, amp, phase, bend) = self.arms;
                for i in 0..2 {
                    let ph = self.t * rate + i as f32 * phase;
                    c.ch.set_relative((*SHOULDER())[i], 0, (up + amp * ph.sin()).clamp(-1.0, 1.0));
                    c.ch.set_relative((*SHOULDER())[i], 1, (amp * ph.cos()).clamp(-1.0, 1.0));
                    c.ch.set_relative((*ELBOW())[i], 0, -bend);
                }
                let (on, twist, max) = self.orient;
                if on && c.vel.truncate().length() > 0.5 {
                    let cl = (*crate::nmrs::CLAVICLE());
                    let fwd = flat((c.b.part(cl[0] + 1).m.d - c.b.part(cl[1] + 1).m.d).cross(Vec3::Z));
                    let d = flat(c.vel);
                    let a = (fwd.cross(d).z.atan2(fwd.dot(d)) / max.max(delirium_data::kv!(51))).clamp(-1.0, 1.0);
                    c.out.torque(0, Vec3::Z * (w * (delirium_data::kv!(9) * a - delirium_data::kv!(118) * c.angvel.z)));
                    for &j in &(*SPINE()) {
                        c.ch.set_relative(j, 2, twist * a);
                    }
                }
            }
            if self.tumble != 0.0 && self.axis != Vec3::ZERO {
                c.out.torque(0, self.axis * (w * delirium_data::kv!(78) * (self.tumble - c.angvel.dot(self.axis))));
            }
            let all: Vec<usize> = (0..c.b.build.parts.len()).collect();
            if self.t > delirium_data::kv!(9) && c.b.doll.collided(&all) && (self.tumble == 0.0 || c.com.z < self.z0 - 1.0 && c.vel.z > -delirium_data::kv!(119) * self.peak) {
                self.out.push(format!("#say smartFall landing speed={:.1}", self.peak));
                let d = flat(-self.normal);
                let slope = self.normal.z < delirium_data::kv!(124);
                if self.limp || self.peak > self.max {
                    self.to(SF_CRASH(), "crash");
                    if slope {
                        self.start("rollDownStairs", format!("rollDownStairs start=true forcemag={} useCustomRollDir=true customRollDir={},{},{}",
                                                             self.rds, d.x, d.y, d.z));
                    }
                    self.out.push(format!("catchFall start=true useHeadLook=true armsStiffness={} torsoStiffness={} legsStiffness={}                                    forwardMaxArmOffset={} backwardsMinArmOffset={} zAxisSpinReduction=0.5", delirium_data::kv!(31), delirium_data::kv!(31), delirium_data::kv!(21), delirium_data::kv!(28), delirium_data::kv!(60)));
                } else if self.peak > self.brace {
                    self.to(SF_ROLL, "roll");
                    if slope && !self.fwd_roll {
                        self.start("rollDownStairs", format!("rollDownStairs start=true useCustomRollDir=true customRollDir={},{},{}", d.x, d.y, d.z));
                    } else {
                        self.start("bodyRollUp", "bodyRollUp start=true".into());
                    }
                } else {
                    self.to(SF_LAND, "land");
                    self.start("catchFall", "catchFall start=true".into());
                    if self.balance {
                        self.start("bodyBalance", "bodyBalance start=true".into());
                    }
                }
                self.stop("pedalLegs");
            }
        } else {
            if self.state == SF_LAND && self.t >= self.catch_time {
                self.stop("catchFall");
            }
            if self.t > 0.5 && (c.vel.length() < self.cutoff || self.t >= self.end_time) {
                self.to(SF_DONE(), "settled");
                for s in std::mem::take(&mut self.started) {
                    self.out.push(format!("{s} start=false"));
                }
            }
        }
    }

    fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.out)
    }

    fn done(&self) -> bool {
        self.state == SF_DONE()
    }
}

pub struct Collide {
    pos: Vec3, normal: Vec3,
    steps: i32, stable: f32, leg_red: f32, slump_red: f32, slump_rate: f32,
    react: f32, exag: f32, spin_t: f32, spin_mag: f32, spin_decay: f32,
    given: bool, virt: bool, first: bool, state: u8, t: f32, calm: f32, hit_t: f32,
    ub_n: Vec3, lb_n: Vec3, ub: bool, lb: bool, glance: f32, fall: bool, out: Vec<String>,
    /// the RDR2 core's balancerCollisionsReaction (extras_fall_rdr2.rs)
    rd: crate::extras_fall_rdr2::BcrRdr2,
}
const BAL_NORMAL: u8 = 0;
const BAL_IMPACT: u8 = 1;
const BAL_REBOUND: u8 = 2;
const BAL_DRAPE: u8 = 3;
const BAL_DRAPE_FWD: u8 = 4;
const BAL_SLUMP: u8 = 5;
const BAL_END: u8 = 7;
#[allow(non_snake_case)]
#[inline]
fn BCR_STEP() -> f32 { delirium_data::kv!(24) }
const BCR_NAMES: [&str; 8] = ["normal", "impact", "rebound", "drape", "drapeForward", "slump", "trip", "end"];
impl Collide {
    fn new(p: &mut P) -> Result<Collide, Error> {
        p.f("exclusionZone", 0.0)?; p.f("footFrictionMultStart", 1.0)?; p.f("footFrictionMultRate", 0.0)?;
        p.f("backFrictionMultStart", 1.0)?; p.f("backFrictionMultRate", 0.0)?;
        let rd = crate::extras_fall_rdr2::BcrRdr2::new(p)?;
        Ok(Collide { pos: p.v("objectBehindVictimPos", Vec3::ZERO)?, normal: p.v("objectBehindVictimNormal", Vec3::ZERO)?,
                     steps: p.i("numStepsTillSlump", delirium_data::kv!(0 int))?, stable: p.f("stable2SlumpTime", 1.0)?,
                     leg_red: p.f("impactLegStiffReduction", delirium_data::kv!(24))?, slump_red: p.f("slumpLegStiffReduction", delirium_data::kv!(20))?,
                     slump_rate: p.f("slumpLegStiffRate", 1.0)?, react: p.f("reactTime", delirium_data::kv!(17))?,
                     exag: p.f("impactExagTime", delirium_data::kv!(60))?, spin_t: p.f("glanceSpinTime", 0.5)?,
                     spin_mag: p.f("glanceSpinMag", delirium_data::kv!(24))?, spin_decay: p.f("glanceSpinDecayMult", delirium_data::kv!(20))?,
                     given: false, virt: false, first: true, state: BAL_NORMAL, t: 0.0, calm: 0.0, hit_t: 0.0,
                     ub_n: Vec3::ZERO, lb_n: Vec3::ZERO, ub: false, lb: false, glance: 0.0, fall: false, out: vec![], rd })
    }

    fn to(&mut self, state: u8) {
        (self.state, self.t) = (state, 0.0);
        self.out.push(format!("#say bcr {}", BCR_NAMES[state as usize]));
    }

    fn catch_fall(&mut self) {
        if !self.fall {
            self.fall = true;
            self.out.push("catchFall start=true useHeadLook=true armsStiffness=10 torsoStiffness=10 legsStiffness=12 \
                           forwardMaxArmOffset=0.9 backwardsMinArmOffset=0.2 zAxisSpinReduction=0.5".into());
        }
    }
}
impl Extra for Collide {
    fn probe(&mut self, c: &Ctx, world: &crate::collision::Collision, obs: &[crate::ragdoll::Obstacle]) {
        if c.ch.rdr2 {
            self.rd.probe(c, world, obs);
        }
    }

    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if c.ch.rdr2 {
            return self.rd.tick(c, dt, &mut self.out);
        }
        let w = weight(c);
        let (com, vel) = (c.com, c.vel);
        let cl = (*crate::nmrs::CLAVICLE());
        let facing = flat((c.b.frames[cl[0] + 1].translation - c.b.frames[cl[1] + 1].translation).cross(Vec3::Z));
        if self.first {
            self.first = false;
            self.given = self.pos != Vec3::ZERO;
            self.virt = !self.given;
            if self.virt {
                self.pos = com - facing * delirium_data::kv!(116);
                self.normal = facing;
            } else if self.normal == Vec3::ZERO {
                self.normal = flat(com - self.pos);
            }
            self.normal = flat(self.normal);
            self.out.push("#say bcr start".into());
        }
        self.t += dt;
        self.hit_t += dt;
        let n_obj = self.normal;
        let top = c.b.frames[(*ANKLE())[0] + 1].translation.z + delirium_data::kv!(28);
        let (lower, upper) = ([0, (*SPINE())[0] + 1, (*SPINE())[1] + 1], [(*SPINE())[2] + 1, (*SPINE())[delirium_data::kv!(0 int)] + 1]);
        let (mut lb, mut ub, mut n) = (false, false, Vec3::ZERO);
        let real: Vec<usize> = lower.iter().chain(&upper).copied().filter(|&k| c.b.doll.collided(&[k])).collect();
        if !real.is_empty() {
            self.virt = false;
            lb = real.iter().any(|k| lower.contains(k));
            ub = real.iter().any(|k| upper.contains(k));
            let away = -real.iter().map(|&k| c.b.frames[k].translation - com).fold(Vec3::ZERO, |a, b| a + b);
            n = if self.given { n_obj }
                else if away.truncate().length() > delirium_data::kv!(68) { flat(away) }
                else if vel.truncate().length() > delirium_data::kv!(24) { -flat(vel) } else { facing };
        } else if self.virt {
            for &k in &lower {
                let p = c.b.frames[k].translation;
                let pen = delirium_data::kv!(125) - (p - self.pos).dot(n_obj);
                if pen > 0.0 && p.z < top {
                    lb = true;
                    n = n_obj;
                    c.out.force(k, n_obj * (w * (pen * delirium_data::kv!(27) + delirium_data::kv!(24) * (-vel.dot(n_obj)).max(0.0))));
                }
            }
        }
        if lb || ub {
            if self.state == BAL_NORMAL {
                self.to(BAL_IMPACT);
                self.ub_n = Vec3::ZERO;
                self.lb_n = Vec3::ZERO;
                (self.ub, self.lb, self.hit_t) = (false, false, 0.0);
                self.normal = if n == Vec3::ZERO { n_obj } else { n };
                let vt = vel - self.normal * vel.dot(self.normal);
                let side = self.normal.cross(vt).z;
                self.glance = if vt.truncate().length() > delirium_data::kv!(78) { side.signum() * self.spin_mag } else { 0.0 };
            }
            if self.state == BAL_IMPACT {
                self.ub |= ub;
                self.lb |= lb;
                if ub { self.ub_n += n; }
                if lb { self.lb_n += n; }
            }
        }
        let f_exag = if self.state == BAL_IMPACT { (1.0 - self.t / self.exag.max(dt)).clamp(0.0, 1.0) } else { 0.0 };
        let still = vel.truncate().length() < delirium_data::kv!(24);
        self.calm = if still { self.calm + dt } else { 0.0 };
        let leg = |c: &mut Ctx, k: f32| c.ch.set_stiffness(delirium_data::kv!(126) * k.max(delirium_data::kv!(68)), delirium_data::kv!(119), "lb", None);
        match self.state {
            BAL_NORMAL => {
                if self.virt && self.t < delirium_data::kv!(17) {
                    c.out.force(CHEST(), -self.normal * (w * delirium_data::kv!(119)));
                }
                if self.t > 2.0 && self.t - dt <= 2.0 {
                    self.to(BAL_END);
                }
            }
            BAL_IMPACT => {
                leg(c, 1.0 - self.leg_red);
                c.out.force(CHEST(), self.normal * (w * delirium_data::kv!(24) * f_exag));
                if self.t < self.spin_t {
                    self.glance *= self.spin_decay.powf(dt * delirium_data::kv!(31));
                    c.out.torque(0, Vec3::Z * (self.glance * w * 0.5));
                }
                for i in 0..2 {
                    c.ch.set_relative((*SHOULDER())[i], 1, 0.5);
                    c.ch.set_relative((*ELBOW())[i], 0, -delirium_data::kv!(24));
                }
                if self.t >= self.react {
                    let acc = flat(self.ub_n + self.lb_n);
                    if acc != Vec3::ZERO && !self.given {
                        self.normal = acc;
                    }
                    let s = if self.ub && self.lb { BAL_SLUMP } else if self.lb {
                        if facing.dot(self.normal) > 0.0 { BAL_DRAPE } else { BAL_DRAPE_FWD }
                    } else { BAL_REBOUND };
                    self.to(s);
                    self.calm = 0.0;
                    if s != BAL_DRAPE && s != BAL_DRAPE_FWD {
                        self.catch_fall();
                    }
                }
            }
            BAL_REBOUND => {
                leg(c, 1.0 - self.leg_red * 0.5);
                if self.t > delirium_data::kv!(116) && still {
                    self.to(BAL_END);
                    self.out.push("catchFall start=false".into());
                } else if self.hit_t > self.steps as f32 * BCR_STEP() {
                    self.to(BAL_SLUMP);
                }
            }
            BAL_DRAPE | BAL_DRAPE_FWD => {
                let dir = if self.state == BAL_DRAPE { 1.0 } else { -1.0 };
                leg(c, 1.0 - self.leg_red);
                c.ch.set_stiffness(delirium_data::kv!(5), delirium_data::kv!(119), "ub", None);
                c.out.force(CHEST(), -self.normal * (w * delirium_data::kv!(17) * dir));
                for &j in &(*SPINE()) {
                    c.ch.set_relative(j, 0, -delirium_data::kv!(24) * dir);
                }
                if self.t >= self.steps as f32 * BCR_STEP() || self.calm >= self.stable {
                    self.to(BAL_SLUMP);
                    self.catch_fall();
                }
            }
            BAL_SLUMP => {
                leg(c, 1.0 - self.slump_red * (self.slump_rate * self.t).min(1.0));
                if (self.t > delirium_data::kv!(78) && c.dbm_failed) || self.t > delirium_data::kv!(12) {
                    self.to(BAL_END);
                }
            }
            _ => {}
        }
    }

    fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.out)
    }

    fn done(&self) -> bool {
        self.state == BAL_END
    }

    fn stop(&mut self, c: &mut Ctx) {
        if c.ch.rdr2 {
            return self.rd.stop(c);
        }
        if !matches!(self.state, BAL_NORMAL | BAL_END) {
            self.catch_fall();
        }
    }
}

pub struct Injured {
    n: i32,
    wounds: [(Vec3, Vec3, usize); 2],
    attacker: Vec3,
    no_reach: [bool; 2],
    strong: bool,
    t: f32,
    pedal_cd: f32,
    pedalling: bool,
    state: u8,
    out: Vec<String>,
    /// the RDR2 core's settings and state
    rd: IogRdr2,
}
const LYING: [&str; 5] = ["upright", "face down", "face up", "on left side", "on right side"];
impl Injured {
    fn new(p: &mut P) -> Result<Injured, Error> {
        let mut w = [(Vec3::ZERO, Vec3::ZERO, (*SPINE())[0] + 1); 2];
        for (i, w) in w.iter_mut().enumerate() {
            let k = i + 1;
            *w = (p.v(&format!("injury{k}LocalPosition"), Vec3::ZERO)?, p.v(&format!("injury{k}LocalNormal"), Vec3::ZERO)?,
                  p.i(&format!("injury{k}Component"), (*SPINE())[0] as i32 + 1)?.clamp(0, delirium_data::kv!(127 int)) as usize);
        }
        Ok(Injured { n: p.i("numInjuries", 0)?.clamp(0, 2), wounds: w, attacker: p.v("attackerPos", Vec3::ZERO)?,
                     no_reach: [p.b("dontReachWithLeft", false)?, p.b("dontReachWithRight", false)?],
                     strong: p.b("strongRollForce", false)?, t: 0.0, pedal_cd: delirium_data::kv!(12), pedalling: false, state: 0, out: vec![],
                     rd: IogRdr2 { relax: (p.f("relaxTimeMin", 3.0)?, p.f("relaxTimeMax", 5.0)?), use_constraints: p.b("useConstraints", false)?,
                                   trapped: p.b("trapped", false)?, attacker_ob: p.i("attackerInstance", -1)?, no_roll_back: p.b("noRollOnBack", false)?,
                                   roll: (p.f("rollDuration", 5.0)?, p.f("rollRandomAbove", 0.3)?, p.f("rollTorqueMax", 30.0)?, p.f("rollTorqueStart", 0.0)?, p.f("rollTorqueRate", 60.0)?),
                                   foot_fric: [1.0; 2], ..Default::default() } })
    }
}
impl Extra for Injured {
    fn probe(&mut self, c: &Ctx, world: &crate::collision::Collision, obs: &[crate::ragdoll::Obstacle]) {
        if c.ch.rdr2 {
            self.probe_rdr2(c, world, obs);
        }
    }

    /// RDR2's two constraints are distance limits to where the part was (0x2a29ba8: type 1, maximum 0.3 and 0.25 m, the
    /// strength field 10000 [P]), so they are ropes here.
    fn ropes(&self) -> Vec<(usize, Vec3, Vec3, f32, f32)> {
        self.rd.pins.iter().map(|&(k, now, at, len)| (k, now, at, len, 10000.0)).collect()
    }

    fn rfw_block(&self) -> Option<[bool; 2]> {
        self.rd.rfw_block
    }

    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if c.ch.rdr2 {
            return self.tick_rdr2(c, dt);
        }
        let first = self.t == 0.0;
        self.t += dt;
        let t = self.t;
        if first {
            self.out.push("bodyBalance start=false".into());
        }
        let up = c.ch.up.normalize_or_zero();
        let pelvis = c.b.part(0).m.d;
        let spine = (c.b.part((*NECK())[0] + 1).m.d - pelvis).normalize_or_zero();
        let side = (c.b.part((*CLAVICLE())[0] + 1).m.d - c.b.part((*CLAVICLE())[1] + 1).m.d).normalize_or_zero();
        let front = side.cross(spine);
        let state = if spine.dot(up) > delirium_data::kv!(74) { 0 }
            else if front.dot(up).abs() > delirium_data::kv!(119) { if front.dot(up) < 0.0 { 1 } else { 2 } }
            else if side.dot(up) > 0.0 { delirium_data::kv!(1 int) } else { delirium_data::kv!(0 int) };
        let w = weight(c);
        let g = 1.0 - 0.5 * (t / delirium_data::kv!(65)).min(1.0);
        if state != self.state {
            self.state = state;
            self.out.push(format!("#say injuredOnGround {}", LYING[state as usize]));
            if state != 0 {
                let a = if self.attacker != Vec3::ZERO { self.attacker } else {
                    pelvis + (spine - up * spine.dot(up)).normalize_or_zero() * 2.0
                };
                self.out.push(format!("headLook start=true pos={},{},{} stiffness={} damping=1 alwaysLook=true", a.x, a.y, a.z, delirium_data::kv!(117)));
            }
        }
        c.ch.set_stiffness(delirium_data::kv!(5), delirium_data::kv!(119), "fb", None);
        for i in 0..2 {
            if !self.pedalling {
                c.ch.set_relative((*HIP())[i], 0, -delirium_data::kv!(119) * g);
                c.ch.set_relative((*KNEE())[i], 0, delirium_data::kv!(74) * g);
            }
        }
        for &j in &(*SPINE()) {
            c.ch.set_relative(j, 0, delirium_data::kv!(116) * g);
        }
        if state == 0 {
            return;
        }
        self.pedal_cd -= dt;
        if self.pedal_cd <= 0.0 {
            self.pedalling = !self.pedalling;
            self.pedal_cd = if self.pedalling { 2.0 } else { delirium_data::kv!(12) };
            self.out.push(if self.pedalling { format!("pedalLegs start=true legStiffness={}", delirium_data::kv!(32)) } else { "pedalLegs start=false".into() });
        }
        let (cyc, u) = (((t - 1.0) / delirium_data::kv!(128)).floor(), ((t - 1.0) % delirium_data::kv!(128)) / delirium_data::kv!(74));
        if t > 1.0 && u < 1.0 {
            let s = if cyc as i32 % 2 == 0 { 1.0 } else { -1.0 };
            let torque = spine * (s * w * (if self.strong { delirium_data::kv!(129) } else { delirium_data::kv!(130) }) * g * (u * std::f32::consts::PI).sin());
            c.out.torque(0, torque);
            c.out.torque(CHEST(), torque);
        }
        let brace = (t > 2.0 && (t - 2.0) % delirium_data::kv!(84) < delirium_data::kv!(119)).then(|| ((t - 2.0) / delirium_data::kv!(84)) as usize % 2);
        let wounds: Vec<Vec3> = if self.n == 0 {
            vec![c.b.part((*SPINE())[0] + 1).m.d + side * delirium_data::kv!(125)]
        } else {
            self.wounds[..self.n as usize].iter().map(|&(p, n, k)| {
                let m = c.b.part(k).m;
                crate::vm4(m.a, m.b, m.c, m.d, p) + crate::vm4(m.a, m.b, m.c, Vec3::ZERO, n).normalize_or_zero() * delirium_data::kv!(131)
            }).collect()
        };
        let mut target: [Option<Vec3>; 2] = [None, None];
        for &wd in &wounds {
            let free = |i: usize| !self.no_reach[i] && target[i].is_none();
            let d = |i: usize| (c.b.part((*SHOULDER())[i] + 1).m.d - wd).length();
            let pick = match (free(0), free(1)) {
                (true, true) => Some(if d(0) <= d(1) { 0 } else { 1 }),
                (true, false) => Some(0),
                (false, true) => Some(1),
                _ => None,
            };
            if let Some(i) = pick {
                target[i] = Some(wd);
            }
        }
        for i in 0..2 {
            let braced = brace == Some(i);
            let Some(at) = (if braced { Some(c.b.part((*SHOULDER())[i] + 1).m.d - up * 0.5) } else { target[i] }) else { continue };
            let l = crate::nmshot::limb(i);
            let k = if braced { delirium_data::kv!(126) } else { delirium_data::kv!(21) * g };
            c.ch.eff[l.e[1]].set_stiffness(k, delirium_data::kv!(119), None);
            c.ch.eff[l.e[2]].set_stiffness(k, delirium_data::kv!(119), None);
            crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[delirium_data::kv!(0 int)], at, 0.0, 0.0, 1.0 - 2.0 * i as f32, 1.0,
                                  Vec3::ZERO, None);
        }
    }

    fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.out)
    }
}

/// injuredOnGround's RDR2-core settings (relaxTimeMin / Max, useConstraints, trapped) and state: started, the time,
/// the roll's direction (0 none, 1, 2), the arms' reach mode (3 both), the wound-reach timer and its hold, the trapped
/// push's timer, its start value and rest, the relax timer, roll time and duration, the pedal timer, the roll torque,
/// whether a tick has run, the pins (part, where it is, where it was pinned, the distance), the arms' search poses, which
/// helpers it has started (pedalLegs, reachForWound, headLook), the trapped push's probe hit (point, direction), the brace
/// service's record, the feet's friction, what reachForWound's hands are told, the first-time log marks, the wrist's
/// floor, the object attackerPos is in, no roll on his back (+0xe4) and the roll's parameters (+0xa0..+0xb0).
#[derive(Clone, Default)]
struct IogRdr2 {
    relax: (f32, f32), use_constraints: bool, trapped: bool,
    started: bool, time: f32, dir: u8, mode: u8, reach_t: f32, reach_hold: f32, push_t: f32, push_start: f32, push_rest: f32,
    relax_t: f32, roll_t: f32, roll_dur: f32, pedal_t: f32, torque: f32, ticked: bool,
    pins: Vec<(usize, Vec3, Vec3, f32)>, pose: [Option<[f32; 6]>; 2], pedalling: bool, rfw: bool, looking: bool, hit: Option<(Vec3, Vec3)>,
    brace: crate::extras_fall_rdr2::BcrBrace, foot_fric: [f32; 2], rfw_block: Option<[bool; 2]>, said: u32,
    attacker_ob: i32, no_roll_back: bool, roll: (f32, f32, f32, f32, f32),
}

/// The trapped push's brace record (the constructor's defaults at +0x2c0, 0.5 and 11 from the push).
const IOG_BRACE: crate::extras_fall_rdr2::BraceRecord = crate::extras_fall_rdr2::BraceRecord {
    distance: 0.5, stiffness: 11.0, predict: 0.45, min_brace: 0.3, back_time: 1.0, absorb: 0.15 };
/// The default wound's depth in front of the spine (ours: RDR2 takes half the part's bound's x extent and half of
/// body+0x98, which wasn't identified; this is reachForWound's surface offset for the trunk).
const IOG_WOUND_DEPTH: f32 = 0.18;

/// Delirium's parts as RDR2's masks: a leg (0xf6), an arm (0xfd800), the pelvis and lowest spine part (0x9), the trunk
/// (0x709) and with the neck (0x2709).
fn iog_leg(k: usize) -> bool {
    (1..=6).contains(&k)
}
fn iog_arm(k: usize) -> bool {
    (13..=20).contains(&k)
}
fn iog_trunk(k: usize) -> bool {
    k == 0 || (7..=10).contains(&k)
}

impl Injured {
    /// The trapped push's probe, before the tick: from the chest 0.7 m towards attackerPos.
    fn probe_rdr2(&mut self, c: &Ctx, world: &crate::collision::Collision, obs: &[crate::ragdoll::Obstacle]) {
        let r = &mut self.rd;
        r.hit = None;
        if !r.trapped || !r.started || r.push_t <= 0.0 {
            return;
        }
        let chest = c.b.part((*SPINE())[3] + 1).m.d;
        // attackerPos in its object (0x2a21218 [P])
        let attacker = obs.iter().find(|o| o.id == r.attacker_ob && r.attacker_ob >= 0).map_or(self.attacker, |o| o.at + o.rot * self.attacker);
        if r.attacker_ob >= 0 && r.said >> 15 & 1 == 0 {
            r.said |= 1 << 15;
            self.out.push(format!("#say iog attackerPos in object {}: {attacker}", r.attacker_ob));
        }
        let r = &mut self.rd;
        let e = (attacker - chest).normalize_or_zero();
        if let Some((t, _, _)) = crate::nmcf::cast(world, obs, chest, e, 0.7) {
            r.hit = Some((chest + e * t, e));
        }
    }

    /// RDR2's injuredOnGround (rdr2_injuredground.md): lying hurt, he rests, then rolls towards his side or back by a
    /// torque about his body's long axis that ramps up while the roll is slow, his spine twisting and the far leg
    /// pushing; on his side the upper spine is flattened. He pedals his legs (not face down, not rolling, not a
    /// wounded leg), one or both hands hold the wound (a leg wound through reachForWound, on and off) while he looks at
    /// it, free arms rest bent, and with `trapped` he braces against what is towards attackerPos through the core brace
    /// service. Arm friction 0.5, the pushing foot's 2, and floors on the shoulders' and wrists' limit half-ranges. Every
    /// joint's stiffness is capped at 9. Ours: Delirium's arm search for RDR2's hand placement. Left out: the Rage
    /// joint's +0x30 = 3 at activate (only that write was found, no reader).
    fn tick_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        // the joint stiffness cap (RDR2's tick runs it on the joints before this frame's requests land)
        for e in c.ch.eff.iter_mut() {
            if e.strength.max(0.0).sqrt() > 9.0 {
                e.set_stiffness(9.0, 1.0, None);
            }
        }
        if !self.rd.started {
            self.start_rdr2(c);
        }
        crate::nmcf::spin_reduction(c, 0.5);
        // the shoulders' limit half-ranges at least (lean1, lean2, twist) (0x2a4eaa0: read in 0x2a21828's half-range)
        for i in 0..2 {
            c.out.lim_floor.push((SHOULDER()[i] + 1, [1.3, 0.9, 2.0]));
        }
        self.rd.rfw_block = None;
        self.rd.time += dt;
        // the log: last tick's friction and floors reached the body, a shoulder went past the rig's limit
        if let (Some(&f), Some(&g)) = (c.b.doll.fric.get(13), c.b.doll.lim_floor.get(SHOULDER()[0] + 1)) {
            self.say(11, format!("#say iog body has arm friction {f} and shoulder limit floors {g:?}"));
        }
        for i in 0..2 {
            let (a, l) = (c.ch.actual(SHOULDER()[i], c.b.frames), c.ch.limits(SHOULDER()[i]));
            if a[2] < l[4] - 0.05 || a[2] > l[5] + 0.05 {
                self.say(12, format!("#say iog shoulder {i} twist {:.2} past the rig's {:.2}..{:.2}", a[2], l[4], l[5]));
            }
        }
        // pose analysis: up made square to the body's long axis
        let (a, b, cc, up) = (c.m.a, c.m.b, c.m.c, c.up);
        let s = b.cross(up).normalize_or_zero();
        let u = s.cross(b).normalize_or_zero();
        let prone = u.dot(cc) > 0.5;
        let supine = u.dot(cc) < -0.6;
        let sigma = a.dot(u);
        let side = (sigma > 0.5, sigma < -0.5);
        // the base pose
        let q2 = sigma * sigma;
        for (j, x) in [(SPINE()[3], 0.4 * q2), (SPINE()[2], 0.4 * q2), (SPINE()[1], 0.2 * q2), (NECK()[0], 0.2 * q2)] {
            c.ch.eff[j].desired[0] = x;
        }
        for i in 0..2 {
            c.ch.eff[HIP()[i]].desired[0] = 1.0;
            c.ch.eff[KNEE()[i]].desired[0] = -1.0;
            (c.ch.eff[SHOULDER()[i]].desired[0], c.ch.eff[SHOULDER()[i]].desired[2]) = (0.785, -0.15);
            c.ch.eff[ELBOW()[i]].desired[0] = 1.0;
            let k = &mut c.ch.eff[ANKLE()[i]];
            (k.strength, k.damping, k.desired[0]) = (20.0, 5.0, -0.5);
        }
        // (the note had this as useConstraints: the tick tests +0xe4, and the pins inside the roll are +0xe2's)
        if !(self.rd.no_roll_back && supine) {
            self.roll_rdr2(c, dt, (s, u, sigma), prone, supine, side);
        } else {
            self.say(9, "#say iog no roll on his back".into());
        }
        self.rd.pins.iter_mut().for_each(|p| p.1 = c.b.part(p.0).m.d);
        let mut fric: Vec<(usize, f32)> = (13..=20).map(|k| (k, 0.5)).collect();
        fric.extend([(3, self.rd.foot_fric[0]), (6, self.rd.foot_fric[1])]);
        c.out.part_friction = fric;
        self.pedal_rdr2(c, dt);
        let mut used = [false; 2];
        if !(self.rd.trapped && self.push_rdr2(c, dt, &mut used)) {
            self.reach_rdr2(c, dt, &mut used);
        }
        self.arms_rdr2(c, prone, sigma, &mut used);
        // the head looks at the wound
        if self.n > 0 && !prone {
            let (k, local) = (self.wounds[0].2, self.wounds[0].0);
            let m = c.b.part(k).m;
            let x = crate::vm4(m.a, m.b, m.c, m.d, local);
            if !self.rd.looking {
                self.rd.looking = true;
                self.out.push(format!("headLook start=true alwaysLook=true pos={},{},{}", x.x, x.y, x.z));
            } else {
                self.out.push(format!("headLook pos={},{},{}", x.x, x.y, x.z));
            }
        }
        self.rd.ticked = true;
    }

    /// RDR2's activate: the reach and trapped-push timers, the first roll's duration and torque, both arms.
    fn start_rdr2(&mut self, c: &mut Ctx) {
        let r = &mut self.rd;
        r.started = true;
        r.mode = 3;
        r.time = 0.0;
        r.reach_t = if r.trapped { c.ch.random() + 0.5 } else { 2.0 * c.ch.random() - 0.5 };
        r.reach_hold = c.ch.random() + 0.5;
        r.push_t = c.ch.random() + 2.0;
        r.push_start = r.push_t;
        r.push_rest = c.ch.random() + 2.5;
        (r.roll_dur, r.relax_t, r.roll_t) = (r.roll.0, 0.0, 0.0);
        if c.ch.random() > r.roll.1 {
            r.roll_dur = 0.2 + 2.8 * c.ch.random();
        }
        (r.pedal_t, r.torque) = (0.0, -20.0);
    }

    /// A log line the first time a branch runs (which branches ran in a scene: `RDR2_SEND` scenes read them).
    fn say(&mut self, bit: u32, line: String) {
        if self.rd.said >> bit & 1 == 0 {
            self.rd.said |= 1 << bit;
            self.out.push(line);
        }
    }

    /// RDR2's roll: resting, the body stiff 5 (neck and head 10) and on a side the upper spine flattened; when the
    /// rest runs out a roll starts the way his side says, for its duration (or until he is over, side value past
    /// 0.9), with its torque about the long axis ramping while the roll is slow, the spine twisting, and the leg
    /// on the far side pushed by IK while the near hip opens.
    #[allow(clippy::too_many_arguments)]
    fn roll_rdr2(&mut self, c: &mut Ctx, dt: f32, (s, u, sigma): (Vec3, Vec3, f32), prone: bool, supine: bool, side: (bool, bool)) {
        let (a, b) = (c.m.a, c.m.b);
        let chest = (*SPINE())[3] + 1;
        // the pins: on his back with the chest and pelvis touching something other than himself (bit 0x10)
        if self.rd.use_constraints && self.rd.pins.is_empty() && supine && c.b.doll.hit_out[chest].is_some() && c.b.doll.hit_out[0].is_some() {
            let (pc, pp) = (c.b.part(chest).m.d, c.b.part(0).m.d);
            self.rd.pins = vec![(chest, pc, pc, 0.3), (0, pp, pp, 0.25)];
            self.say(0, format!("#say iog pins chest {pc} pelvis {pp}"));
        }
        if self.rd.dir == 0 {
            self.say(1, format!("#say iog resting sigma {sigma:.2} side {side:?}"));
        }
        let r = &mut self.rd;
        if r.dir == 0 {
            c.ch.set_stiffness(5.0, 1.0, "fb", None);
            // mask 0x600, bit = joint index + 1: the top two spine joints [C] (the note took them for the neck and head)
            for j in [SPINE()[2], SPINE()[3]] {
                c.ch.eff[j].set_stiffness(10.0, 1.0, None);
            }
            if side.0 || side.1 {
                let t = a.cross(if side.0 { u } else { -u });
                for (k, f) in [(1, 20.0), (2, 40.0), (3, 60.0)] {
                    c.out.torque((*SPINE())[k] + 1, t * f);
                }
                self.say(2, format!("#say iog flatten on a side sigma {sigma:.2}"));
            }
            let r = &mut self.rd;
            r.relax_t -= dt;
            if r.relax_t >= 0.0 {
                return;
            }
            r.dir = if side.0 { 2 } else if side.1 { 1 } else if sigma < 0.0 { 2 } else { 1 };
            let span = r.relax.1 - r.relax.0;
            r.relax_t = r.relax.0 + c.ch.random() * span;
            (r.roll_dur, r.roll_t) = (r.roll.0, 0.0);
            if c.ch.random() > r.roll.1 {
                r.roll_dur = 0.2 + 2.8 * c.ch.random();
            }
            if prone {
                r.roll_dur = r.roll_dur.max(1.2);
            }
            if r.roll_dur < 2.0 {
                r.relax_t = 0.5 * r.relax.0 + c.ch.random() * 0.5 * span;
            }
            r.torque = if r.roll.3 == 0.0 && r.torque < 0.0 { 20.0 } else { r.roll.3 };
            let _ = c.ch.random();
            let line = format!("#say iog roll {} starts, {:.2} s, sigma {sigma:.2}, prone {prone}", r.dir, r.roll_dur);
            self.out.push(line);
            return;
        }
        r.roll_t += dt;
        let tau = r.roll_t;
        if tau > r.roll_dur {
            r.dir = 0;
            self.out.push(format!("#say iog roll ends on its time, sigma {sigma:.2}"));
            return;
        }
        c.ch.set_stiffness(7.0, 1.0, "fb", None);
        c.ch.set_stiffness(5.0, 1.0, "ua", None);
        let w = c.angvel.dot(b);
        let rate = r.roll.4;
        r.torque += if (r.dir == 2 && w > -1.46) || (r.dir == 1 && w < 1.46) { 0.5 * rate * dt } else { -3.5 * rate * dt };
        r.torque = r.torque.clamp(0.0, r.roll.2);
        let (k1, k2, k3, q0) = ((10.0 * tau).clamp(3.0, 9.0), (10.0 * tau).clamp(3.0, 5.5), (10.0 * tau).clamp(7.0, 11.0), (5.0 * tau).clamp(0.0, 5.0));
        c.ch.set_stiffness(k3, 1.0, "us", None);
        // dir 1 pushes with the right leg and opens the left hip; dir 2 the mirror
        let (sg, push, open) = if r.dir == 1 { (1.0, 3, 2) } else { (-1.0, 2, 3) };
        for &j in SPINE().iter() {
            c.ch.eff[j].desired[2] = sg * q0;
        }
        if sg * w < 2.1 {
            for k in [0, (*SPINE())[0] + 1, (*SPINE())[1] + 1, (*SPINE())[2] + 1] {
                c.out.torque(k, b * (sg * r.torque));
            }
        }
        if !prone {
            let near = if r.dir == 1 { side.1 } else { side.0 };
            let q = if near && !supine { -0.3 } else { 0.1 * ((8.0 * r.time).cos() - 1.0) - 0.4 };
            let l = crate::nmshot::limb(push);
            let target = c.b.part(0).m.d - s * (0.3 * sg) - b * 0.4 + u * q;
            crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[3], target, 0.0, 0.0, if push == 2 { 1.0 } else { -1.0 }, 1.0, Vec3::ZERO, None);
            c.ch.set_stiffness(k1, 1.0, if push == 2 { "ll" } else { "lr" }, None);
            // the pushing foot's friction 2 (masks 0x80, 0x20); nothing but the deactivate puts it back
            r.foot_fric[push - 2] = 2.0;
            let f = if r.dir == 1 { 2.0 * sigma.clamp(-0.9, -0.4) + 1.8 } else { 1.8 - 2.0 * sigma.clamp(0.4, 0.9) };
            let o = crate::nmshot::limb(open);
            let lerp = |x: f32, y: f32| x + (y - x) * f;
            c.ch.eff[o.e[1]].desired = [lerp(0.0, 0.2), lerp(-0.5, -1.0), lerp(0.0, -0.4)];
            // the knee: lerp(-0.5, -1, f) (0x2a20d48 clamps f)
            c.ch.eff[o.e[2]].desired[0] = -0.5 - 0.5 * f.clamp(0.0, 1.0);
            c.ch.set_stiffness(k2, 1.0, if open == 2 { "ll" } else { "lr" }, None);
        }
        if (r.dir == 1 && sigma > 0.9) || (r.dir == 2 && sigma < -0.9) {
            r.dir = 0;
            self.out.push(format!("#say iog roll over at sigma {sigma:.2}, {tau:.2} s"));
        }
    }

    /// RDR2's pedalling: pedalLegs runs while he isn't rolling or face down, sparing a wounded leg, its backPedal,
    /// radiusVariance and legAngleVariance drawn again every 1..3 s. (Its other parameters stay as last sent: Delirium
    /// keeps a message's settings, RDR2 resets them.)
    fn pedal_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        let r = &mut self.rd;
        if r.dir != 0 || c.m.c.dot(c.up) > 0.5 {
            if r.pedalling {
                r.pedalling = false;
                self.say(3, "#say iog pedal stopped".into());
                self.out.push("pedalLegs start=false".into());
            }
            return;
        }
        let k = self.wounds[0].2;
        if !r.pedalling {
            r.pedalling = true;
            let (back, rv, av) = (c.ch.random() > 0.5, c.ch.random(), c.ch.random());
            let _ = c.ch.random();
            self.out.push(format!("pedalLegs start=true pedalLeftLeg={} pedalRightLeg={} legStiffness=4 backPedal={back} radiusVariance={rv} \
                                   legAngleVariance={av} angularSpeed=2 centreUp=0.1 centreForwards=0.1", !(1..=3).contains(&k), !(4..=6).contains(&k)));
            r.pedal_t = 1.0 + 2.0 * c.ch.random();
            self.say(4, "#say iog pedal started".into());
            return;
        }
        r.pedal_t -= dt;
        if r.pedal_t < 0.0 {
            r.pedal_t = 1.0 + 2.0 * c.ch.random();
            let (back, rv, av) = (c.ch.random() > 0.5, c.ch.random(), c.ch.random());
            self.say(5, "#say iog pedal redrawn".into());
            self.out.push(format!("pedalLegs backPedal={back} radiusVariance={rv} legAngleVariance={av} angularSpeed=2"));
        }
    }

    /// RDR2's trapped push: on for 2..3 s, off for 2.5..3.5 s; while on and something is within 0.7 m of the chest
    /// towards attackerPos, the core brace service braces on it (target nearer as the push runs out, stiffness 11). Returns
    /// whether it acted (then there is no wound reach), the bracing arms marked used.
    fn push_rdr2(&mut self, c: &mut Ctx, dt: f32, used: &mut [bool; 2]) -> bool {
        let r = &mut self.rd;
        r.push_t -= dt;
        if r.push_t < -r.push_rest {
            r.push_t = 2.0 + c.ch.random();
            r.push_start = r.push_t;
            r.push_rest = 2.5 + c.ch.random();
        }
        if r.push_t <= 0.0 {
            return false;
        }
        let Some((hit, e)) = r.hit else { return false };
        let f = if r.push_t >= r.push_start - 0.7 { 1.0 } else { r.push_t / (r.push_start - 0.7) };
        let at = hit + e * ((1.0 - f) * 0.25 - 0.2);
        let arms = crate::extras_fall_rdr2::brace_core(c, &mut r.brace, IOG_BRACE, at, Vec3::ZERO, dt);
        for i in 0..2 {
            used[i] |= arms[i];
        }
        self.say(6, format!("#say iog trapped push at {at} arms {arms:?}"));
        if arms != [false; 2] {
            self.say(10, format!("#say iog trapped brace arms {arms:?} at {at}"));
        }
        true
    }

    /// RDR2's wound reach: one wound (by default on the belly, an arm's or the pelvis's moved to the spine, one on the
    /// back of the trunk brought round to its front); the hands hold it a little apart (Delirium's arm search for
    /// RDR2's hand placement, without its pull), an arm whose shoulder is within 0.22 m of it left off; a leg wound
    /// goes to reachForWound, on and off on random timers. The clavicles' lean2 is zeroed. (0x2a355a0, which moves a wound
    /// up to a part in the mask ctx+0x34e8, changes nothing here: that mask is all parts unless a weapon constraint sets it.)
    fn reach_rdr2(&mut self, c: &mut Ctx, dt: f32, used: &mut [bool; 2]) {
        // the wrists' limit half-ranges at least (lean1, lean2, twist); lean2's floor +0x3dc is 0.6 for good
        for i in 0..2 {
            c.out.lim_floor.push((crate::nmrs::WRIST()[i] + 1, [0.5236, 0.6, 1.2]));
        }
        let spine1 = (*SPINE())[1] + 1;
        let (mut local, _, mut k) = self.wounds[0];
        if self.n == 0 || iog_arm(k) {
            (self.n, k, local) = (1, spine1, Vec3::new(0.0, 0.0, -IOG_WOUND_DEPTH));
        }
        if k == 0 || k == (*SPINE())[0] + 1 {
            k = spine1;
        }
        if (iog_trunk(k) || k == 11) && local.z > 0.0 {
            local.z = -local.z;
        }
        let normal = local.normalize_or_zero();
        self.wounds[0] = (local, normal, k);
        let m = c.b.part(k).m;
        let x = crate::vm4(m.a, m.b, m.c, m.d, local);
        let r = &mut self.rd;
        if !r.ticked {
            if self.no_reach[0] && r.mode == 1 {
                r.mode = 2;
            }
            if self.no_reach[1] && r.mode == 2 {
                r.mode = 1;
            }
            for i in 0..2 {
                if (c.b.joint_pos((*SHOULDER())[i]) - x).length() < 0.22 {
                    self.no_reach[i] = true;
                    self.out.push(format!("#say iog arm {i}'s shoulder is within 0.22 of the wound: stays off"));
                }
            }
        }
        let leg = iog_leg(k);
        let mut leg_reach = false;
        if leg {
            r.reach_t -= dt;
            leg_reach = r.reach_t > 0.0;
            if r.reach_t < -r.reach_hold {
                r.reach_t = 0.5 + c.ch.random();
                r.reach_hold = if r.trapped { 0.2 + 0.3 * c.ch.random() } else { 0.5 + c.ch.random() };
            }
        } else {
            let rolling = r.dir != 0;
            for i in 0..2 {
                let mode_ok = if i == 0 { r.mode == 1 || r.mode == 3 } else { r.mode == 2 || r.mode == 3 };
                if self.no_reach[i] || !(mode_ok || rolling) || c.ch.busy[i] {
                    r.pose[i] = None;
                    continue;
                }
                let mut at = local;
                if r.mode == 3 && iog_trunk(k) {
                    at.x += if i == 0 { 0.05 } else { -0.05 };
                }
                let t = crate::vm4(m.a, m.b, m.c, m.d, at);
                let n = crate::vm4(m.a, m.b, m.c, Vec3::ZERO, normal);
                crate::extras_rdr2::hand_place(c, i, t, n, &mut r.pose[i], None, 0.0);
                used[i] = true;
            }
        }
        if !leg_reach && r.rfw {
            r.rfw = false;
            self.out.push("reachForWound start=false".into());
        } else if leg_reach && !r.rfw {
            r.rfw = true;
            self.out.push(format!("reachForWound start=true hitPart={k} hitPoint={},{},{} allowInjuredThighReach=true allowInjuredLowerLegReach=true \
                                   reachWithOneHand=false reachFalling=1 reachOnFloor=1 timeBeforeReachForWound=0 legStiffness=5 armStiffness=6 \
                                   spineStiffness=8", x.x, x.y, x.z));
        }
        // the hands reachForWound must let go (an arm used here or that shouldn't reach, R+0xdb), and those it has on it
        if leg_reach {
            let rolling = r.dir != 0;
            let want = [!self.no_reach[0] && (r.mode == 1 || r.mode == 3 || rolling), !self.no_reach[1] && (r.mode == 2 || r.mode == 3 || rolling)];
            r.rfw_block = Some([used[0] || !want[0], used[1] || !want[1]]);
            let on = c.rfw.map_or([false; 2], |v| v.reached);
            *used = [used[0] || on[0], used[1] || on[1]];
            if on != [false; 2] {
                self.say(16, format!("#say iog reachForWound has hands on the leg wound {on:?}, told to let go {:?}", self.rd.rfw_block));
            }
            self.say(7, format!("#say iog leg wound on part {k}: reachForWound, hands on it {on:?}"));
        } else if !leg {
            self.say(8, format!("#say iog hands {used:?} on the wound on part {k}, normal {}", normal));
        }
        for i in 0..2 {
            c.ch.eff[(*CLAVICLE())[i]].desired[1] = 0.0;
        }
    }

    /// RDR2's arm stiffness and idle arms: arms stiff 10 (wrist muscle 10), the arm on the side he rolls towards
    /// softened through the end of the roll; face down and rolling, a hand caught above the back is pulled in; an
    /// arm with nothing to do rests bent (8, damping 0.8).
    fn arms_rdr2(&mut self, c: &mut Ctx, prone: bool, sigma: f32, used: &mut [bool; 2]) {
        let dir = self.rd.dir;
        let s = [
            if dir == 1 { 4.0 - 6.666667 * (sigma.clamp(-1.0, -0.85) + 0.85) * 6.0 } else { 10.0 },
            if dir == 2 { 4.0 + (6.666667 * sigma.clamp(0.85, 1.0) - 5.666667) * 6.0 } else { 10.0 },
        ];
        for i in [1, 0] {
            let l = crate::nmshot::limb(i);
            for j in l.e {
                c.ch.eff[j].set_stiffness(s[i], 1.0, Some(1.0));
            }
            c.ch.eff[l.e[3]].muscle_stiffness = 10.0;
        }
        if prone && dir != 0 {
            let leg = (c.b.joint_pos(HIP()[0]) - c.b.joint_pos(KNEE()[0])).length() + (c.b.joint_pos(KNEE()[0]) - c.b.joint_pos(ANKLE()[0])).length();
            let i = if dir == 2 { 0 } else { 1 };
            let l = crate::nmshot::limb(i);
            let back = c.b.part((*SPINE())[2] + 1).m.d;
            let above = |p: Vec3| (p - back).dot(c.m.c) > 0.1 * leg;
            if above(c.b.part(l.p[3]).m.d) || above(c.b.joint_pos(l.e[2])) {
                for j in l.e {
                    c.ch.eff[j].set_stiffness(6.4, 0.64, Some(0.5));
                }
                c.ch.eff[l.e[1]].desired = [1.0, 1.0, 0.0];
                c.ch.eff[l.e[2]].desired[0] = 0.2;
                used[i] = true;
                self.say(13, format!("#say iog prone, rolling: arm {i} above the back pulled in"));
            }
        }
        for i in 0..2 {
            if used[i] || c.ch.busy[i] {
                continue;
            }
            let l = crate::nmshot::limb(i);
            for j in l.e {
                c.ch.eff[j].set_stiffness(8.0, 0.8, Some(0.5));
            }
            c.ch.eff[l.e[0]].desired = [0.0; 3];
            c.ch.eff[l.e[1]].desired = [0.5, 1.0, 1.0];
            c.ch.eff[l.e[2]].desired[0] = 0.6;
            self.say(14, format!("#say iog arm {i} idle"));
        }
    }
}

const DOWN: f32 = 1.0;

pub struct Crawl {
    k: (f32, f32),
    speed: f32,
    target: Vec3,
    yaw: f32,
    twister: bool,
    t: f32,
    best: (f32, f32),
    out: Vec<String>,
    over: bool,
}

impl Crawl {
    fn new(p: &mut P) -> Result<Crawl, Error> {
        let _ = (p.b("bLearn", false)?, p.i("numFrames2Learn", 0)?, p.i("animIndex", 0)?, p.b("learnFromAnimPlayback", false)?,
                 p.i("inputSequenceSize", 0)?, p.b("useSpine3Thing", false)?, p.b("useRollBoneCompensation", false)?);
        Ok(Crawl { k: (p.f("stiffness", delirium_data::kv!(31))?, p.f("damping", delirium_data::kv!(119))?), speed: p.f("speed", delirium_data::kv!(20))?, target: p.v("targetPosition", Vec3::ZERO)?,
                   yaw: p.f("yawOffset", 0.0)?, twister: p.b("useTwister", true)?, t: 0.0, best: (f32::MAX, 0.0), out: vec![],
                   over: false })
    }
}

impl Extra for Crawl {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        let pelvis = c.b.part(0).m.d;
        let front = flat(c.b.part((*NECK())[1] + 1).m.d - pelvis);
        if self.t == 0.0 {
            self.out.push("bodyBalance start=false".into());
        }
        self.t += dt;
        let w = weight(c);
        c.ch.set_stiffness(self.k.0, self.k.1, "fb", None);
        for (k, h) in [(0, 0.5), (CHEST(), delirium_data::kv!(10))] {
            let p = c.b.part(k);
            c.out.force(k, Vec3::Z * ((h - p.m.d.z) * w * delirium_data::kv!(84) - p.v.z * w * delirium_data::kv!(116)).clamp(0.0, w * 0.5));
        }
        if self.t < DOWN {
            for i in 0..2 {
                c.ch.set_relative((*HIP())[i], 0, -0.5);
                c.ch.set_relative((*KNEE())[i], 0, delirium_data::kv!(74));
                c.ch.set_relative((*SHOULDER())[i], 0, -delirium_data::kv!(119));
            }
            return;
        }
        if self.target == Vec3::ZERO {
            self.target = pelvis + front * delirium_data::kv!(5);
        }
        if self.best.0 == f32::MAX {
            self.best.1 = self.t;
        }
        let to = self.target - pelvis;
        let dist = Vec3::new(to.x, to.y, 0.0).length();
        if dist < 0.5 {
            self.out.push("#say learnedCrawl success".into());
            self.over = true;
            return;
        }
        if dist < self.best.0 - delirium_data::kv!(60) {
            self.best = (dist, self.t);
        } else if self.t - self.best.1 > delirium_data::kv!(84) {
            self.out.push("#say learnedCrawl failure".into());
            self.over = true;
            return;
        }
        let (s, r) = (self.yaw.sin(), self.yaw.cos());
        let d = flat(to);
        let heading = Vec3::new(d.x * r - d.y * s, d.x * s + d.y * r, 0.0);
        let ph = (self.t - DOWN) * self.speed * TAU;
        for i in 0..2 {
            let a = ph + i as f32 * TAU * 0.5;
            let l = ph + (1 - i) as f32 * TAU * 0.5;
            let arm = crate::nmshot::limb(i);
            let reach = c.b.part(arm.p[0]).m.d + heading * (delirium_data::kv!(24) * a.sin()) - Vec3::Z * (delirium_data::kv!(132) - delirium_data::kv!(125) * a.cos().max(0.0));
            crate::nmdbm::limb_ik(c.b, c.ch, arm.e[1], arm.e[2], arm.p[0], arm.p[delirium_data::kv!(0 int)], reach, 0.0, 0.0, 1.0 - 2.0 * i as f32, 1.0,
                                  Vec3::ZERO, None);
            c.ch.set_relative((*HIP())[i], 0, -0.5 + delirium_data::kv!(116) * l.sin());
            c.ch.set_relative((*KNEE())[i], 0, delirium_data::kv!(119) + delirium_data::kv!(24) * l.cos());
        }
        let turn = front.cross(heading).z.clamp(-1.0, 1.0);
        if self.twister {
            for j in (*SPINE()) {
                c.ch.eff[j].desired[2] = turn * delirium_data::kv!(9);
            }
        }
        c.out.torque(0, Vec3::Z * (turn * w * delirium_data::kv!(9)));
        c.out.force(0, heading * (w * delirium_data::kv!(130)));
        c.out.force(CHEST(), heading * (w * delirium_data::kv!(133)));
    }

    fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.out)
    }

    fn done(&self) -> bool {
        self.over
    }
}

/// RDR2's smartFall parameters, with RDR2's defaults where none is given (GTA IV's message, read with the RDR2 core).
#[derive(Clone, Copy, Debug)]
struct SfParams {
    body_stiffness: f32,
    body_damping: f32,
    catch_fall_time: f32,
    crash_or_land_cut_off: f32,
    pd_strength: f32,
    pd_damping: f32,
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
    max_recoverable: f32,
    landing_normal: f32,
    rds_force_mag: f32,
    rds_tlv: f32,
    rds_tlv_decay: f32,
    rds_force_vel: f32,
    initial_state: u8,
    stop_rolling_time: f32,
    teeter: bool,
    teeter_offset: f32,
    cf_spin: f32,
    splat: f32,
}

impl SfParams {
    fn read(p: &mut P) -> Result<SfParams, Error> {
        let f = |p: &mut P, n: &str, d: f32| -> Result<f32, Error> { p.get(n).map_or(Ok(d), Value::f) };
        let b = |p: &mut P, n: &str, d: bool| -> Result<bool, Error> { p.get(n).map_or(Ok(d), Value::b) };
        // GTA IV's message has these as numbers where RDR2's are flags, or a flag where RDR2's is a number
        let fb = |p: &mut P, n: &str, d: bool| -> Result<bool, Error> { p.get(n).map_or(Ok(d), |v| v.f().map(|x| x != 0.0)) };
        let bi = |p: &mut P, n: &str, d: i32| -> Result<i32, Error> { p.get(n).map_or(Ok(d), |v| v.b().map(i32::from)) };
        Ok(SfParams {
            body_stiffness: f(p, "bodyStiffness", 11.0)?, body_damping: f(p, "bodydamping", 1.0)?, catch_fall_time: f(p, "catchfalltime", 0.3)?,
            crash_or_land_cut_off: f(p, "crashOrLandCutOff", 0.868)?, pd_strength: f(p, "pdStrength", 0.0)?, pd_damping: f(p, "pdDamping", 1.0)?,
            arms_up: f(p, "armsUp", -3.1)?, leg_radius: f(p, "legRadius", 0.4)?, leg_ang_speed: f(p, "legAngSpeed", 7.85)?,
            leg_asymmetry: f(p, "legAsymmetry", 4.0)?, arms2legs_phase: f(p, "arms2LegsPhase", 0.0)?, arms2legs_sync: bi(p, "arms2LegsSync", 1)?,
            arm_ang_speed: f(p, "armAngSpeed", 7.85)?, arm_amplitude: f(p, "armAmplitude", 2.0)?, arm_phase: f(p, "armPhase", 3.1)?,
            forward_offset: f(p, "forwardOffsetOfLegIK", 0.0)?, balance: b(p, "balance", true)?,
            orientate_to_fall: b(p, "orientateBodyToFallDirection", false)?, alan_rickman: b(p, "alanRickman", false)?,
            orientate_twist: fb(p, "orientateTwist", true)?, orientate_max: f(p, "orientateMax", 300.0)?, forward_roll: b(p, "forwardRoll", false)?,
            zero_pose_roll: b(p, "useZeroPose_withForwardRoll", false)?, ignore_world: b(p, "ignorWorldCollisions", false)?,
            adaptive_circling: b(p, "adaptiveCircling", true)?, hula: fb(p, "hula", true)?,
            max_recoverable: f(p, "maxSpeedForRecoverableFall", 15.0)?, landing_normal: 0.6, rds_force_mag: f(p, "rdsForceMag", 0.8)?,
            rds_tlv: f(p, "rdsTargetLinearVelocity", 1.0)?, rds_tlv_decay: f(p, "rdsTargetLinearVelocityDecayTime", 0.5)?,
            rds_force_vel: f(p, "rdsForceVelThreshold", 10.0)?, initial_state: p.get("initialState").map_or(Ok(0), Value::i)?.clamp(0, 8) as u8,
            stop_rolling_time: f(p, "stopRollingTime", 2.0)?, teeter: b(p, "teeter", false)?, teeter_offset: f(p, "teeterOffset", 0.3)?,
            cf_spin: f(p, "cfZAxisSpinReduction", 0.5)?, splat: f(p, "splatWhenStopped", 0.0)?,
        })
    }
}

/// RDR2's smartFall (with the RDR2 core): a nine-state machine that runs the fall by starting and configuring other
/// behaviours, from ground probes, the time to impact, the body's attitude and whether the fall is survivable.
struct SfRdr2 {
    p: SfParams,
    state: u8,
    /// logged that state 7 runs fall to knees
    ftk_said: bool,
    /// the state's own clocks: rolling (5) and on the feet (7), and the hard landing's (6)
    t200: f32,
    t21c: f32,
    smooth: f32,
    /// time since the start
    t: f32,
    hula: f32,
    /// state 0: time with the upper body touching
    touching: f32,
    first_floor: f32,
    peak: Vec3,
    start: Vec3,
    leg_len: f32,
    /// the feedback was sent (once per latch)
    sent: bool,
    pd: bool,
    height: f32,
    tti: f32,
    land_up: f32,
    landing_flat: bool,
    class: u8,
    class_now: u8,
    class_mid: u8,
    ground: Vec3,
    contact: bool,
    recoverable: bool,
    failed: bool,
    settled: bool,
    back_pedal: bool,
    first: bool,
    on: Vec<&'static str>,
    out: Vec<String>,
}

pub(crate) fn v3s(v: Vec3) -> String {
    format!("{},{},{}", v.x, v.y, v.z)
}

pub(crate) fn unit(v: Vec3) -> Vec3 {
    let l = v.length();
    if l > 1e-6 { v / l } else { Vec3::ZERO }
}

/// The landing attitude: 0 feet down, 1 front down, 2 side or head down, 3 back down (ours [G]: RDR2's classifier
/// isn't read; its states use 1 and 2 as the falls catchFall takes).
pub(crate) fn attitude(a: [Vec3; 3], n: Vec3) -> u8 {
    let (b, c) = (a[1], a[2]);
    if b.dot(n) > 0.5 { 0 } else if c.dot(n) > 0.5 { 1 } else if c.dot(n) < -0.5 { 3 } else { 2 }
}

const SF_NAMES: [&str; 9] = ["airborne", "tumbling", "crash", "brace", "unrecoverable", "forward roll", "hard landing", "land on feet", "settle"];

impl SfRdr2 {
    fn new(p: SfParams) -> SfRdr2 {
        SfRdr2 { p, state: p.initial_state, t200: 0.0, t21c: 0.0, smooth: 0.0, t: 0.0, hula: -0.1, touching: 0.0, first_floor: -1.0, peak: Vec3::ZERO,
                 start: Vec3::ZERO, leg_len: 0.9, sent: false, pd: true, height: 5.0, tti: 30.0, land_up: 0.0, landing_flat: false, class: 3,
                 class_now: 3, class_mid: 3, ground: Vec3::Z, contact: false, recoverable: true, failed: false, settled: false, back_pedal: false, first: true,
                 on: vec![], out: vec![], ftk_said: false }
    }

    fn ensure(&mut self, name: &'static str, line: String) {
        if !self.on.contains(&name) {
            self.on.push(name);
            self.out.push(line);
        }
    }

    fn stop(&mut self, name: &'static str) {
        if self.on.contains(&name) {
            self.on.retain(|&s| s != name);
            self.out.push(format!("{name} start=false"));
        }
    }

    fn feedback(&mut self, c: &mut Ctx, failure: bool) {
        if !self.sent {
            self.sent = true;
            c.out.send(if failure { delirium_data::kv!(54 int) } else { delirium_data::kv!(55 int) }, "smartFall", 0.0);
        }
    }

    /// RDR2's ground probes: the height under the COM (5 m down) and where a 1 s ballistic guess meets the ground, from
    /// which the time to impact and the attitude he will land in.
    fn probe(&mut self, c: &Ctx, world: &crate::collision::Collision, obs: &[crate::ragdoll::Obstacle]) {
        let up = c.up;
        if self.state == 7 {
            self.height = 1.0;
        } else {
            match crate::nmcf::cast(world, obs, c.com, -up, 5.0) {
                Some((t, n, _)) => (self.height, self.ground) = (t, n),
                None => self.height = 5.0,
            }
        }
        if self.state == 7 {
            self.tti = 30.0;
            return;
        }
        let seg = c.vel - up * 9.81;
        let len = seg.length();
        // (no hit: the ground is further than a second away; ours, RDR2's probe keeps its last result)
        let Some((t, n, _)) = (len > 1e-4).then(|| crate::nmcf::cast(world, obs, c.com, seg / len, len)).flatten() else {
            self.tti = 30.0;
            return;
        };
        let hit = c.com + seg / len * t;
        (self.land_up, self.landing_flat) = (n.dot(up), n.dot(up) > self.p.landing_normal);
        let mut d = up.dot(c.com - hit).abs();
        if c.m.b.dot(up) > self.p.crash_or_land_cut_off && d > self.leg_len {
            d -= self.leg_len;
        }
        let (vz, g) = (-c.vel.dot(up).abs(), crate::ragdoll::GRAVITY().abs().max(1.0));
        self.tti = ((2.0 * g * d + vz * vz).sqrt() + vz) / g;
        let q = bevy_math::Quat::from_scaled_axis(c.angvel * self.tti.min(1.0));
        self.class = attitude([q * c.m.a, q * c.m.b, q * c.m.c], n);
    }

    fn begin(&mut self, c: &mut Ctx) {
        (self.start, self.recoverable, self.first) = (c.com, true, false);
        let (h, k, a) = (c.b.joint_pos((*HIP())[0]), c.b.joint_pos((*KNEE())[0]), c.b.joint_pos((*ANKLE())[0]));
        self.leg_len = (h - k).length() + (k - a).length();
        self.peak = c.vel;
        self.first_floor = -1.0;
        self.out.push(format!("#say smartFall (RDR2) {}", SF_NAMES[self.state as usize]));
        self.enter(c, self.state);
    }

    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if self.first {
            self.begin(c);
        }
        let p = self.p;
        let (up, v) = (c.up, c.vel);
        let b = p.body_stiffness;
        if !matches!(self.state, 2 | 6 | 7 | 8) {
            // (RDR2 scales by the health, which is full here)
            c.ch.set_stiffness((2.0 * b).max(5.0).min(b), p.body_damping, "fb", None);
        }
        let upright = up.dot(c.m.b) > p.crash_or_land_cut_off;
        // the situation
        let h = v - up * up.dot(v);
        let down = -up.dot(v);
        let share = down / (down.abs() + h.length()).max(1e-6);
        let fwdish = unit(v).dot(-c.m.c) > -0.2;
        self.class_now = attitude([c.m.a, c.m.b, c.m.c], self.ground);
        let qm = bevy_math::Quat::from_scaled_axis(c.angvel * (0.5 * self.tti).min(1.0));
        self.class_mid = attitude([qm * c.m.a, qm * c.m.b, qm * c.m.c], self.ground);
        if v.length() > self.peak.length() {
            self.peak = v;
        }
        let all: Vec<usize> = (0..c.b.build.parts.len()).collect();
        let touch = c.b.doll.collided(&all);
        if !self.first || matches!(p.initial_state, 6 | 7) {
            self.contact = touch;
            if !touch {
                self.ground = up;
            }
        }
        let floor = self.ground.dot(up) > 0.7;
        if self.contact && floor && self.first_floor < 0.0 && self.state != 7 {
            self.first_floor = self.t;
        }
        if self.state == 0 {
            let legs: Vec<usize> = (2..4).flat_map(|i| crate::nmshot::limb(i).p[1..].to_vec()).collect();
            let upper: Vec<usize> = (1..c.b.build.parts.len()).filter(|k| !legs.contains(k)).collect();
            self.touching = if c.b.doll.collided(&upper) { self.touching + dt } else { 0.0 };
        }
        let impact = (self.state == 0 && self.touching > 0.3) || self.tti < p.catch_fall_time + 0.15;
        let contact = self.contact && !p.ignore_world;
        let f302 = upright && self.land_up > 0.7;
        let (o1, o2, o5) = (self.class == 1, self.class == 2, self.class_now == 2 && self.class_mid == 2);
        let forward = fwdish && matches!(self.class, 1 | 2);
        let land = contact || (impact && !f302);
        let slope = 10.0 * (1.0 - self.ground.dot(up)).clamp(0.0, 0.1);
        self.recoverable &= (1.0 - slope) * v.dot(up) > -p.max_recoverable;
        let high = !impact && self.tti > 0.4 && !contact && self.height > 1.5;
        let vertical = share > 0.3;
        let dead = false;
        self.orientate(c, dt);
        self.t += dt;
        // the state machine: the update, then the transitions it asks for
        self.update(c, dt);
        let r = self.recoverable;
        let air = |vertical: bool| if vertical { 0 } else { 1 };
        let not_land = |s: &SfRdr2| if impact && s.landing_flat { Some(3) } else if vertical { None } else { Some(1) };
        let next: Option<u8> = match self.state {
            5 if self.sent => Some(6),
            0 if land && dead => Some(8),
            0 if impact && !r => Some(4),
            0 if land => {
                if forward && (o1 || (o2 && o5)) { Some(6) } else if forward && (!o2 || o5) { not_land(self) } else { Some(2) }
            }
            0 => not_land(self),
            1 if land && dead => Some(8),
            1 if impact && !r => Some(4),
            1 if land => Some(if forward { 6 } else { 2 }),
            1 if impact && self.landing_flat => Some(3),
            1 if vertical => Some(0),
            2 if contact => Some(if upright { 7 } else { 6 }),
            2 if o5 => Some(6),
            2 if !r => Some(4),
            2 if self.tti > 0.4 && self.height > 1.5 && vertical => Some(0),
            2 if self.height > 1.5 && !vertical => Some(1),
            3 | 4 | 5 | 6 | 7 | 8 if high => Some(air(vertical)),
            3 if !r => Some(4),
            3 if !contact => None,
            3 if dead => Some(8),
            3 => Some(if p.forward_roll { 5 } else if upright { 7 } else { 6 }),
            4 if !contact => None,
            4 if !dead && r => Some(if upright { 7 } else { 6 }),
            4 => Some(8),
            6 if (p.splat > 0.0 && v.length() < p.splat) || dead || (contact && !r) => Some(8),
            7 if self.failed && (dead || !r) => Some(8),
            7 if self.failed && !self.on.contains(&"catchFall") && !self.on.contains(&"rollDownStairs") => Some(0),
            _ => None,
        };
        if let Some(s) = next.filter(|&s| s != self.state) {
            self.exit(c, self.state);
            self.state = s;
            self.out.push(format!("#say smartFall (RDR2) {}  height {:.2} m, impact in {:.2} s", SF_NAMES[s as usize], self.height, self.tti));
            self.enter(c, s);
        }
    }

    /// RDR2's orientation PD on the pelvis, while airborne: toward the fall's direction (or the heading he has), or for
    /// alanRickman along where he started.
    fn orientate(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.p;
        if !self.pd || p.pd_strength <= 0.001 {
            return;
        }
        orient_pd(c, dt, (p.pd_strength, p.pd_damping, p.orientate_max), p.alan_rickman, p.orientate_to_fall, p.orientate_twist, self.start);
    }

    fn enter(&mut self, c: &mut Ctx, s: u8) {
        let p = self.p;
        let b = p.body_stiffness;
        match s {
            0 => {
                self.pd = true;
                (self.peak, self.touching, self.first_floor) = (Vec3::ZERO, 0.0, -1.0);
            }
            1 => self.pd = false,
            2 => {
                self.contact = false;
                self.ensure("bodyFoetal", "bodyFoetal start=true stiffness=10 mask=ub".into());
                self.ensure("pedalLegs", "pedalLegs start=true pedalLeftLeg=true pedalRightLeg=true radius=0.2 angularSpeed=3 backPedal=false \
                                          legStiffness=9 radiusVariance=0.2 adaptivePedal4Dragging=false speedAsymmetry=1 pedalOffset=0.05".into());
            }
            4 => {
                self.stop("pedalLegs");
                self.ensure("bodyFoetal", "bodyFoetal start=true".into());
                self.ensure("bodyWrithe", "bodyWrithe start=true blendArms=0.5 blendBack=0.5 blendLegs=0.5".into());
            }
            6 => {
                for n in ["bodyWrithe", "bodyFoetal", "pedalLegs"] {
                    self.stop(n);
                }
                self.t21c = 0.0;
                self.on.retain(|&n| n != "catchFall");
                self.ensure("catchFall", format!("catchFall start=true legsStiffness={} torsoStiffness={} armsStiffness={} zAxisSpinReduction={}",
                                                 0.75 * b, 10.0 / 11.0 * b, 13.0 / 11.0 * b, p.cf_spin));
            }
            7 => {
                self.ensure("bodyBalance", "bodyBalance start=true".into());
                (self.t200, self.failed) = (0.0, false);
                // fall to knees' init on the character's record
                crate::nmftk_rdr2::init(c);
            }
            8 => self.settled = false,
            _ => {}
        }
        let _ = c;
    }

    fn exit(&mut self, c: &mut Ctx, s: u8) {
        match s {
            0 => {
                self.pd = false;
                for n in ["headLook", "pedalLegs", "catchFall", "armsWindmillAdaptive", "bodyWrithe"] {
                    self.stop(n);
                }
            }
            1 | 4 => {
                self.stop("bodyWrithe");
                self.stop("bodyFoetal");
            }
            2 => {
                self.stop("bodyFoetal");
                self.stop("pedalLegs");
            }
            3 => {
                self.stop("pedalLegs");
                self.stop("catchFall");
            }
            5 => self.stop("rollDownStairs"),
            6 => {
                self.stop("catchFall");
                self.t21c = 0.0;
            }
            7 => {
                self.stop("teeter");
                self.stop("bodyBalance");
                crate::nmftk_rdr2::exit(c);
                crate::nmftk_rdr2::flags_off(c);
            }
            _ => {}
        }
        let _ = c;
    }

    fn update(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.p;
        let b = p.body_stiffness;
        let (up, v) = (c.up, c.vel);
        match self.state {
            0 => {
                self.sent = false;
                let look = c.com - c.m.c + up * c.m.c.dot(up) - up * 2.0;
                self.out.push(format!("headLook start=true alwaysLook=true pos={} vel={} stiffness={b} damping={}", v3s(look), v3s(v), p.body_damping));
                self.on.retain(|&n| n != "headLook");
                self.on.push("headLook");
                // adaptive circling: pedal backwards (and the arms the other way) when pitching forward, from the pitch
                // and the angular momentum about the side axis
                if p.adaptive_circling {
                    let pitch = {
                        let a = c.m.c.z.clamp(-1.0, 1.0).asin();
                        if c.m.b.z < 0.0 { if a >= 0.0 { std::f32::consts::PI - a } else { -std::f32::consts::PI - a } } else { a }
                    };
                    // (RDR2 reads the angular momentum about the side axis; ours, the spin about it [G])
                    let l = c.m.a.dot(c.angvel);
                    let e = if l.abs() > 12.0 || pitch.signum() != l.signum() { l - pitch } else { -pitch };
                    if e < -2.2 {
                        self.back_pedal = false;
                    } else if e > 2.2 {
                        self.back_pedal = true;
                    }
                }
                let back = self.back_pedal;
                // hula: legs circle wide and back when he falls face up and slowly
                if p.hula {
                    if c.m.c.z < -0.2 && c.m.b.z < 0.0 && c.angvel.length() < 7.0 && self.hula < 0.0 {
                        self.hula = 1.0;
                    } else if (c.m.c.z > 0.2 || c.m.b.z > 0.0) && (0.0..0.5).contains(&self.hula) {
                        self.hula = -0.1;
                    }
                    if self.hula > 0.0 {
                        self.hula -= dt;
                    }
                }
                let pedal = if self.hula > 0.0 {
                    "radius=1 radiusVariance=0 angularSpeed=8 centreUp=-0.25 legAngleVariance=0 backPedal=true speedAsymmetry=0".to_string()
                } else {
                    format!("radius={} angularSpeed={} centreUp=-0.1 radiusVariance=0.4 legAngleVariance=0.5 speedAsymmetry={} backPedal={back}",
                            p.leg_radius, p.leg_ang_speed, p.leg_asymmetry)
                };
                self.on.retain(|&n| n != "pedalLegs");
                self.on.push("pedalLegs");
                self.out.push(format!("pedalLegs start=true pedalLeftLeg=true pedalRightLeg=true legStiffness={b} adaptivePedal4Dragging=false \
                                       pedalOffset=0 centreForwards=0 {pedal}"));
                let phase = p.arm_phase - if p.arms2legs_sync != 0 { p.arms2legs_phase } else { 0.0 };
                self.on.retain(|&n| n != "armsWindmillAdaptive");
                self.on.push("armsWindmillAdaptive");
                self.out.push(format!("armsWindmillAdaptive start=true bodyStiffness={b} armStiffness={} period={} amplitude={} phase={phase} \
                                       disableOnImpact=false", b * 14.0 / 11.0, std::f32::consts::TAU / p.arm_ang_speed.max(0.01), p.arm_amplitude));
            }
            1 => {
                self.sent = false;
                self.ensure("bodyFoetal", "bodyFoetal start=true".into());
                self.ensure("bodyWrithe", "bodyWrithe start=true armStiffness=15 armPeriod=0.6 backPeriod=0.6 legPeriod=0.6 blendArms=1 \
                                           blendBack=1 blendLegs=0.5 armAmplitude=3".into());
            }
            2 => {
                let k = 0.95 - up.dot(c.m.b).clamp(0.85, 0.95);
                self.out.push(format!("pedalLegs start=true centreForwards={} centreUp={}", 1.5 * k - 0.1, 3.0 * k + 0.05));
                if v.length() < 0.05 {
                    self.feedback(c, false);
                }
            }
            3 => self.brace(c),
            4 => {}
            5 => self.roll(c, dt),
            6 => {
                self.t21c += dt;
                if !p.balance {
                    c.ch.set_stiffness(6.0, 1.0, "fb", Some(0.5));
                }
                if v.length() < 0.075 {
                    self.feedback(c, false);
                }
            }
            7 => {
                self.ensure("bodyBalance", "bodyBalance start=true".into());
                if p.teeter && p.initial_state == 7 && !self.on.contains(&"teeter") {
                    let h = unit(v - up * up.dot(v));
                    let feet = [2, 3].map(|i| c.b.part(crate::nmshot::limb(i).p[3]).m.d);
                    let at = c.com + h * p.teeter_offset;
                    let at = at - up * up.dot(at) + up * up.dot((feet[0] + feet[1]) * 0.5);
                    // RDR2's order: edgeLeft on the -up x v side, so the edge's normal (up x (right - left)) faces him
                    let s = unit(h.cross(up));
                    self.ensure("teeter", format!("teeter start=true edgeLeft={} edgeRight={} preTeeterTime=1 teeterTime=0.5 leanAwayScale=0.5 \
                                                   callHighFall=false", v3s(at + s), v3s(at - s)));
                }
                self.t200 += dt;
                if !p.balance {
                    c.ch.set_stiffness(6.0, 1.0, "ub", Some(0.5));
                }
                // the balancer's failure, seen from outside (ours: smartFall reads the balancer's own flag)
                let legs: Vec<usize> = (2..4).flat_map(|i| crate::nmshot::limb(i).p[1..].to_vec()).collect();
                let trunk: Vec<usize> = (0..c.b.build.parts.len()).filter(|k| !legs.contains(k)).collect();
                self.failed |= up.dot(c.m.b) < 0.5 || c.b.doll.collided(&trunk);
                // fall to knees, if the character's record says so; catchFall and rollDownStairs get its flags
                if c.ch.core.ftk.on {
                    if !self.ftk_said {
                        self.ftk_said = true;
                        self.out.push("#say smartFall land on feet: runs fall to knees".into());
                    }
                    crate::nmftk_rdr2::core(c, &mut self.out, dt);
                    crate::nmftk_rdr2::flags(c);
                }
                if v.length() < 0.15 {
                    let f = self.failed;
                    self.feedback(c, f);
                }
            }
            8 => {
                if v.length() <= 1.0 {
                    for n in ["bodyRollUp", "bodyWrithe", "bodyFoetal", "pedalLegs", "catchFall"] {
                        self.stop(n);
                    }
                    if !self.settled {
                        self.settled = true;
                        self.lie_still(c);
                    }
                } else {
                    self.ensure("bodyRollUp", "bodyRollUp start=true".into());
                }
            }
            _ => {}
        }
    }

    /// RDR2's brace before impact: catchFall early when falling any way but feet first (with arms up at -2 or lower),
    /// else the arms up in front of the head (feet first), the legs pedalling under him.
    fn brace(&mut self, c: &mut Ctx) {
        let p = self.p;
        let b = p.body_stiffness;
        let up = c.up;
        let mut ik = None;
        if p.arms_up > -3.0 || self.class != 0 {
            if p.arms_up <= -2.0 {
                self.ensure("catchFall", format!("catchFall start=true legsStiffness={} torsoStiffness={} armsStiffness={} zAxisSpinReduction={}",
                                                 0.75 * b, 10.0 / 11.0 * b, 13.0 / 11.0 * b, p.cf_spin));
            } else {
                let head = c.b.part(delirium_data::kv!(55 int)).m;
                let r = head.d + up * p.arms_up + unit(head.a) * 0.3;
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
                let l = crate::nmshot::limb(i);
                let vel = c.b.part(l.p[0]).v;
                crate::nmdbm::limb_ik_w(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[3], t[i], 0.0, 1.0, side, 1.0, vel, None, 1.0, false);
            }
        }
        for &j in (*KNEE()).iter().chain((*HIP()).iter()) {
            c.ch.eff[j].muscle_stiffness = 0.5;
        }
        self.ensure("pedalLegs", format!("pedalLegs start=true pedalLeftLeg=true pedalRightLeg=true radius={} angularSpeed={} legStiffness={b} \
                                          speedAsymmetry=4 centreForwards={} centreUp=0 radiusVariance=0.1", p.leg_radius, p.leg_ang_speed, p.forward_offset));
    }

    /// RDR2's forward roll: tucked in, stiffening down from 12 over the roll, rollDownStairs' helper torques under it.
    fn roll(&mut self, c: &mut Ctx, dt: f32) {
        let p = self.p;
        self.t200 += dt;
        let ch = &mut *c.ch;
        for &j in &(*SPINE()) {
            ch.eff[j].desired[0] = 1.0;
        }
        for (i, e) in [(0, 1.6), (1, 1.4)] {
            ch.eff[(*SHOULDER())[i]].desired = [2.0, -2.0, -0.8];
            ch.eff[(*ELBOW())[i]].desired[0] = e;
        }
        ch.eff[(*HIP())[0]].desired[0] = 2.25;
        ch.eff[(*HIP())[1]].desired[0] = 2.3;
        for &j in &(*NECK()) {
            ch.eff[j].desired[0] = 3.0;
        }
        self.smooth = 0.9 * self.smooth + 0.1 * c.vel.length();
        ch.set_stiffness((p.body_stiffness - 12.0 * self.t200).clamp(8.0, 12.0), p.body_damping, "fb", None);
        let ub = c.up.dot(c.m.b);
        ch.eff[(*KNEE())[0]].desired[0] = ub - 1.75;
        ch.eff[(*KNEE())[1]].desired[0] = ub - 1.65;
        if p.zero_pose_roll {
            for e in ch.eff.iter_mut() {
                e.blend_to_zero(1.0);
            }
        }
        self.ensure("rollDownStairs", "rollDownStairs start=true forcemag=1 spinWhenInAir=false onlyApplyHelperForces=true".into());
        if self.t200 > 1.6 {
            self.feedback(c, false);
        }
    }

    /// RDR2's settle, once nearly still: the pose held where it is, elbows and hips eased, one arm out.
    fn lie_still(&mut self, c: &mut Ctx) {
        let ch = &mut *c.ch;
        for j in 0..ch.eff.len() {
            ch.eff[j].desired = ch.actual(j, c.b.frames);
        }
        for &j in &(*ELBOW()) {
            ch.eff[j].desired[0] = 1.0;
        }
        for &j in &(*HIP()) {
            ch.eff[j].desired[1] = -0.7;
        }
        // which arm by the side he lies on [P]
        let n = self.ground;
        let s = c.m.c.dot(n) * unit(c.up.cross(n)).dot(c.m.b);
        let (a, o) = if s >= 0.0 { (0, 1) } else { (1, 0) };
        ch.eff[(*SHOULDER())[a]].desired[1] = -0.2;
        ch.eff[(*SHOULDER())[o]].desired[1] = -0.5;
        ch.eff[(*ELBOW())[o]].desired[0] = 0.0;
        // (RDR2's stiffness here wasn't read: ours, the hard landing's limp values)
        ch.set_stiffness(6.0, 1.0, "fb", Some(0.5));
    }
}

/// RDR2's orientation PD on the pelvis (smartFall and highFall): toward the fall's direction (or the heading he has),
/// or for alanRickman along where he started; (strength, damping, torque clamp).
pub(crate) fn orient_pd(c: &mut Ctx, dt: f32, (s, damping, max): (f32, f32, f32), alan_rickman: bool, to_fall: bool, twist: bool, start: Vec3) {
    let up = c.up;
    let pel = c.b.pmat(0).m;
    let (axis, angle) = if alan_rickman {
        let a = (c.com - start).cross(-pel.c);
        (unit(a), a.length().min(1.0).asin())
    } else {
        let h = c.vel - up * up.dot(c.vel);
        let fwd_speed = -h.dot(c.m.c);
        let (side, fwd) = if fwd_speed.abs() < 1.0 || !to_fall {
            let a = unit(c.m.a - up * up.dot(c.m.a));
            (a, up.cross(a))
        } else {
            let s = unit(-h);
            (s, s.cross(up))
        };
        if twist {
            let target = bevy_math::Quat::from_mat3(&bevy_math::Mat3::from_cols(side, up, -fwd));
            let now = bevy_math::Quat::from_mat3(&bevy_math::Mat3::from_cols(unit(pel.a), unit(pel.b), unit(pel.c)));
            let (ax, mut an) = (target * now.inverse()).to_axis_angle();
            if an > std::f32::consts::PI {
                an -= std::f32::consts::TAU;
            }
            (ax, an)
        } else {
            let a = (-pel.c).cross(fwd);
            (unit(a), a.length().min(1.0).asin())
        }
    };
    let t = (260.0 * s * angle - 80.0 * s * damping * axis.dot(c.angvel)).clamp(-max, max) / (60.0 * dt).clamp(1.0, 60.0);
    c.out.torque(0, axis * t);
}
