use bevy_math::Vec3;

use crate::extras::{Extra, P};
use crate::msg::{Error, Kind, Message, Param};
use crate::nmrs::{Character, ANKLE, CLAVICLE, ELBOW, HIP, KNEE, NECK, SHOULDER, SPINE, WRIST};
use crate::nmshot::Ctx;
use bevy_math::Quat;

#[allow(non_snake_case)]
#[inline]
fn CHEST() -> usize { delirium_data::kv!(81 int) }
#[allow(non_snake_case)]
#[inline]
fn HANDS() -> &'static [usize; 2] { static V: std::sync::OnceLock<[usize; 2]> = std::sync::OnceLock::new(); let f = || [delirium_data::kv!(54 int), delirium_data::kv!(127 int)]; if delirium_data::kv::open() { V.get_or_init(f) } else { Box::leak(Box::new(f())) } }
#[allow(non_snake_case)]
#[inline]
fn FEET() -> &'static [usize; 2] { static V: std::sync::OnceLock<[usize; 2]> = std::sync::OnceLock::new(); let f = || [delirium_data::kv!(0 int), delirium_data::kv!(6 int)]; if delirium_data::kv::open() { V.get_or_init(f) } else { Box::leak(Box::new(f())) } }

pub static MESSAGES: [Message; 15] = [
    Message { name: "yanked", params: &[
        Param { name: "point", kind: Kind::Vec3, used: true, doc: "world point he is pulled toward (zero: 2 m in front of the chest)" },
        Param { name: "part", kind: Kind::Int, used: true, doc: "the part pulled (10, the chest)" },
        Param { name: "force", kind: Kind::Float, used: true, doc: "pull, in body weights (1; 0: no pull, just the reaction)" },
        Param { name: "pull", kind: Kind::Float, used: true, doc: "seconds the pull lasts (0.2)" },
        Param { name: "armStiffness", kind: Kind::Float, used: true, doc: "(11)" },
        Param { name: "armDamping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "spineStiffness", kind: Kind::Float, used: true, doc: "(11)" },
        Param { name: "spineDamping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "armStiffnessStart", kind: Kind::Float, used: true, doc: "arms while the yank lands (3)" },
        Param { name: "armDampingStart", kind: Kind::Float, used: true, doc: "(0.1)" },
        Param { name: "spineStiffnessStart", kind: Kind::Float, used: true, doc: "(3)" },
        Param { name: "spineDampingStart", kind: Kind::Float, used: true, doc: "(0.1)" },
        Param { name: "timeAtStartValues", kind: Kind::Float, used: true, doc: "s at the start values (0.4)" },
        Param { name: "rampTimeFromStartValues", kind: Kind::Float, used: true, doc: "s ramping from them (0.1)" },
        Param { name: "stepsTillStartEnd", kind: Kind::Int, used: true, doc: "RDR2 core: steps before perStepReduction starts (1)" },
        Param { name: "timeStartEnd", kind: Kind::Float, used: true, doc: "s until the ramp to the end values (3)" },
        Param { name: "rampTimeToEndValues", kind: Kind::Float, used: true, doc: "s of that ramp (1)" },
        Param { name: "lowerBodyStiffness", kind: Kind::Float, used: true, doc: "legs (12)" },
        Param { name: "lowerBodyStiffnessEnd", kind: Kind::Float, used: true, doc: "legs at the end (8)" },
        Param { name: "perStepReduction", kind: Kind::Float, used: true, doc: "RDR2 core: leg stiffness lost a step (1)" },
        Param { name: "hipPitchForward", kind: Kind::Float, used: true, doc: "hip pitch when pulled forward, of the range (0.6)" },
        Param { name: "hipPitchBack", kind: Kind::Float, used: true, doc: "when pulled back (1)" },
        Param { name: "spineBend", kind: Kind::Float, used: true, doc: "spine bend with the pull, of the range (0.7)" },
        Param { name: "footFriction", kind: Kind::Float, used: true, doc: "RDR2 core: the feet on his feet (0.3)" },
        Param { name: "comVelRDSThresh", kind: Kind::Float, used: true, doc: "COM speed, m/s, past which he goes down (4)" },
        Param { name: "turnThresholdMin", kind: Kind::Float, used: true, doc: "RDR2 core: least bearing he turns at (0.5)" },
        Param { name: "turnThresholdMax", kind: Kind::Float, used: true, doc: "RDR2 core: most, past it he is asked to turn (1.5)" },
        Param { name: "hulaPeriod", kind: Kind::Float, used: true, doc: "s per hip/spine sway (0.25)" },
        Param { name: "hipAmplitude", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "spineAmplitude", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "minRelaxPeriod", kind: Kind::Float, used: true, doc: "shortest s between relax moments (0.3)" },
        Param { name: "maxRelaxPeriod", kind: Kind::Float, used: true, doc: "longest (1.5)" },
        Param { name: "rollHelp", kind: Kind::Float, used: true, doc: "roll push on the ground, of his weight (0.5)" },
        Param { name: "groundLegStiffness", kind: Kind::Float, used: true, doc: "(11)" },
        Param { name: "groundArmStiffness", kind: Kind::Float, used: true, doc: "(11)" },
        Param { name: "groundSpineStiffness", kind: Kind::Float, used: true, doc: "(11)" },
        Param { name: "groundLegDamping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "groundArmDamping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "groundSpineDamping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "groundFriction", kind: Kind::Float, used: true, doc: "RDR2 core: down, moving (1)" },
        Param { name: "useHeadLook", kind: Kind::Bool, used: true, doc: "look at headLookPos (true)" },
        Param { name: "headLookPos", kind: Kind::Vec3, used: true, doc: "(zero: the pull point)" },
        Param { name: "headLookInstanceIndex", kind: Kind::Int, used: true, doc: "(-1)" },
        Param { name: "headLookAtVelProb", kind: Kind::Float, used: false, doc: "(not done)" },
        Param { name: "ropePos", kind: Kind::Vec3, used: true, doc: "RDR2 core: the rope's anchor, world (zero: point; both zero: 2 m in front of his chest)" },
        Param { name: "ropePart", kind: Kind::Int, used: true, doc: "RDR2 core: the part the rope is on (part)" },
        Param { name: "loosetimetorso", kind: Kind::Float, used: true, doc: "RDR2 core: s after the yank his upper body is loose (0.3)" },
        Param { name: "loosetimelegs", kind: Kind::Float, used: true, doc: "RDR2 core: and his legs (0.1)" },
        Param { name: "loosestrength", kind: Kind::Float, used: true, doc: "RDR2 core: the damping and muscle scale while loose (0.1)" },
        Param { name: "spinelean1limit", kind: Kind::Float, used: true, doc: "RDR2 core: the spine's lean limits raised at the jerk, lean1 (0.5)" },
        Param { name: "spinelean2limit", kind: Kind::Float, used: true, doc: "RDR2 core: lean2 (0.3)" },
        Param { name: "torqueexaggeratejumplimit", kind: Kind::Float, used: true, doc: "RDR2 core: s of it after the yank (0.2)" },
        Param { name: "leanhipsmin", kind: Kind::Float, used: true, doc: "RDR2 core: the hips' lean toward the rope, least (-0.4)" },
        Param { name: "leanhipsmax", kind: Kind::Float, used: true, doc: "RDR2 core: most (0.2)" },
        Param { name: "leanmin", kind: Kind::Float, used: true, doc: "RDR2 core: the body's lean toward the rope, least (-0.1)" },
        Param { name: "leanmax", kind: Kind::Float, used: true, doc: "RDR2 core: most (-0.3)" },
        Param { name: "restartUpness", kind: Kind::Float, used: true, doc: "RDR2 core: down, his uprightness above which he's back on his feet (1.1: never)" },
        Param { name: "headlookprob", kind: Kind::Vec3, used: true, doc: "RDR2 core: the head's chances of headLookPos, along his motion, still (0.3,0.3,0.3)" },
        Param { name: "legAmplitude", kind: Kind::Float, used: true, doc: "RDR2 core: down, the legs' kicks (0.4)" },
        Param { name: "grabspinefreq", kind: Kind::Float, used: true, doc: "RDR2 core: down, the spine's least writhe rate while grabbing (0.5)" },
        Param { name: "spinefreq", kind: Kind::Float, used: true, doc: "RDR2 core: down, the spine's writhe rate (2.5)" },
        Param { name: "spsidefreq", kind: Kind::Float, used: true, doc: "RDR2 core: down, the side writhe's rate (2)" },
        Param { name: "spsideamp", kind: Kind::Float, used: true, doc: "RDR2 core: and size (1)" },
        Param { name: "rollhelpback", kind: Kind::Float, used: true, doc: "RDR2 core: down, the roll torque on his back (0)" },
        Param { name: "rollhelpmoving", kind: Kind::Float, used: true, doc: "RDR2 core: moving (0)" },
        Param { name: "rollhelpbackmoving", kind: Kind::Float, used: true, doc: "RDR2 core: on his back, moving (0)" },
        Param { name: "movingthresh", kind: Kind::Float, used: true, doc: "RDR2 core: m/s above which he's moving (1)" },
        Param { name: "rollType", kind: Kind::Int, used: true, doc: "RDR2 core: 1: no roll help on his back (0)" },
        Param { name: "groundneckstiffness", kind: Kind::Float, used: true, doc: "RDR2 core: down, the stable neck (11)" },
        Param { name: "ropeendfriction", kind: Kind::Float, used: true, doc: "RDR2 core: down, moving: friction at the rope's end of him (0.5)" },
        Param { name: "otherendfriction", kind: Kind::Float, used: true, doc: "RDR2 core: at the other end (2)" },
        Param { name: "lassoedtype", kind: Kind::Int, used: true, doc: "RDR2 core: 0 round the neck, 1 the chest and arms, 2 the feet (0)" },
        Param { name: "grabPart", kind: Kind::Int, used: true, doc: "RDR2 core: the part whose frame the rope grabs are in (10)" },
        Param { name: "grabposl", kind: Kind::Vec3, used: true, doc: "RDR2 core: the left hand's grab in that frame (0.07,0.05,-0.051)" },
        Param { name: "grabnormall", kind: Kind::Vec3, used: true, doc: "RDR2 core: the left palm's normal in that frame (0,0,-1)" },
        Param { name: "grabposr", kind: Kind::Vec3, used: true, doc: "RDR2 core: the right hand's (0.09,-0.05,-0.051)" },
        Param { name: "grabnormalr", kind: Kind::Vec3, used: true, doc: "RDR2 core: the right's (0,0,-1)" },
        Param { name: "grabpointamp", kind: Kind::Vec3, used: true, doc: "RDR2 core: the grabs clawing about it, m (zero)" },
        Param { name: "grabpointfreq", kind: Kind::Vec3, used: true, doc: "RDR2 core: its rates (4,5,3)" },
        Param { name: "grabhandsoffset", kind: Kind::Vec3, used: true, doc: "RDR2 core: the hands apart when both grab (zero)" },
        Param { name: "feetprob", kind: Kind::Vec3, used: true, doc: "RDR2 core: down, round the neck: the arms' chances of reach, windmill (0.5,0.5,0)" },
        Param { name: "spineprob", kind: Kind::Vec3, used: true, doc: "RDR2 core: down: the spine's chances of still, writhe, big writhe (0.6,0.4,0.05)" },
        Param { name: "minstatetime", kind: Kind::Float, used: true, doc: "RDR2 core: s a pick lasts, least (1)" },
        Param { name: "maxstatetime", kind: Kind::Float, used: true, doc: "RDR2 core: most (1.2)" },
        Param { name: "timetodie", kind: Kind::Float, used: true, doc: "RDR2 core: s down before he goes limp, with the feedback yanked (15)" },
        Param { name: "frictionsim", kind: Kind::Float, used: true, doc: "RDR2 core: down, the sliding drag on the roped end, N (70)" },
        Param { name: "frictionsimairborne", kind: Kind::Float, used: true, doc: "RDR2 core: off the ground over 0.5 s (70)" },
        Param { name: "bumpsim", kind: Kind::Float, used: true, doc: "RDR2 core: down, a bump off the ground (400)" },
        Param { name: "bumpsimprob", kind: Kind::Float, used: true, doc: "RDR2 core: its chance a step at 60 Hz (0.08)" },
        Param { name: "draggedbystirrup", kind: Kind::Bool, used: true, doc: "RDR2 core: (false)" },
        Param { name: "draggedbyanimal", kind: Kind::Bool, used: true, doc: "RDR2 core: the head looks at the rope (false)" },
        Param { name: "DraggedByCart", kind: Kind::Int, used: true, doc: "RDR2 core: 1 left leg, 2 right leg, 3 both held 5 stiff (0)" },
        Param { name: "stableneck", kind: Kind::Bool, used: true, doc: "RDR2 core: the jerk leaves his neck stiff (false)" },
        Param { name: "stunnedtime", kind: Kind::Float, used: true, doc: "RDR2 core: s limp once down (0)" },
        Param { name: "stunnedblendout", kind: Kind::Float, used: true, doc: "RDR2 core: s coming out of it (0)" },
        Param { name: "bolas", kind: Kind::Bool, used: true, doc: "RDR2 core: a bolas (false)" },
        Param { name: "bolasForces", kind: Kind::Bool, used: true, doc: "RDR2 core: its spinning impulses (false)" },
        Param { name: "bolasTime", kind: Kind::Float, used: true, doc: "RDR2 core: s they last (0.35)" },
        Param { name: "bolasDir", kind: Kind::Vec3, used: true, doc: "RDR2 core: the spin's axis source (1,0,0)" },
        Param { name: "neckFootFriction", kind: Kind::Float, used: true, doc: "RDR2 core: round the neck, the feet moving slowly (+0xcc, 0.1)" },
        Param { name: "pedalWhenSlow", kind: Kind::Bool, used: true, doc: "RDR2 core: down, face down, the legs may pedal when slow (+0x110, false)" },
        Param { name: "spineAmpUp", kind: Kind::Float, used: true, doc: "RDR2 core: down, face up, the writhe (+0x124, 0.4)" },
        Param { name: "spineAmpUpNeck", kind: Kind::Float, used: true, doc: "RDR2 core: and round the neck (+0x128, 0.4)" },
        Param { name: "spineAmpDown", kind: Kind::Float, used: true, doc: "RDR2 core: down, face down, not round the neck (+0x12c, 1)" },
        Param { name: "spineBigAmp", kind: Kind::Float, used: true, doc: "RDR2 core: down, the big writhe (+0x130, 4)" },
        Param { name: "spineBigAmpUp", kind: Kind::Float, used: true, doc: "RDR2 core: and face up (+0x134, 0.4)" },
        Param { name: "spineAmp138", kind: Kind::Float, used: true, doc: "RDR2 core: in RDR2's table, no reader (+0x138, 0.4)" },
        Param { name: "spineAmp13c", kind: Kind::Float, used: true, doc: "RDR2 core: in RDR2's table, no reader (+0x13c, 1)" },
        Param { name: "headGrabProb", kind: Kind::Float, used: true, doc: "RDR2 core: the head's chance of looking at a grab (+0x210, 0.333)" },
        Param { name: "armFeetProb", kind: Kind::Vec3, used: true, doc: "RDR2 core: on his feet, the arms' chances of nothing, the rope (+0x220, 0.2,0.7,0)" },
        Param { name: "armDownProb", kind: Kind::Vec3, used: true, doc: "RDR2 core: down, the arms' chances of reach, windmill, the rope (+0x240, 0.55,0,0.1)" },
        Param { name: "rightGrabsAfter", kind: Kind::Bool, used: true, doc: "RDR2 core: round the feet, the right hand grabs again after a grab (+0x250, true)" },
        Param { name: "limpRate", kind: Kind::Float, used: true, doc: "RDR2 core: down, how fast a noise makes him limp, 0 never (+0x27c, 0)" },
        Param { name: "struggle", kind: Kind::Float, used: true, doc: "RDR2 core: down, -1 weak .. 1 hard (+0x280, 0)" },
        Param { name: "bolasHeadShare", kind: Kind::Float, used: true, doc: "RDR2 core: the impulse on the head, chest lassoed (+0x2b4, 0.05)" },
        Param { name: "bolasFeet", kind: Kind::Float, used: true, doc: "RDR2 core: the impulse round the feet (+0x2b8, 1000)" },
        Param { name: "bolasSlow", kind: Kind::Float, used: true, doc: "RDR2 core: round the chest, slow (+0x2bc, 3000)" },
        Param { name: "bolasNeck", kind: Kind::Float, used: true, doc: "RDR2 core: round the neck (+0x2c0, 2500)" },
        Param { name: "bolasFast", kind: Kind::Float, used: true, doc: "RDR2 core: round the chest, fast (+0x2c4, 5000)" },
        Param { name: "bolasFirst", kind: Kind::Float, used: true, doc: "RDR2 core: s before the first (+0x2cc, 0.1)" },
        Param { name: "bolasPeriod", kind: Kind::Float, used: true, doc: "RDR2 core: s between them (+0x2d0, 0.065)" },
    ] },
    Message { name: "dragged", params: &[
        Param { name: "armStiffness", kind: Kind::Float, used: true, doc: "(11)" },
        Param { name: "armDamping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "armMuscleStiffness", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "radiusTolerance", kind: Kind::Float, used: true, doc: "how near the rope a hand grabs it, m (0.25)" },
        Param { name: "lengthTolerance", kind: Kind::Float, used: true, doc: "slack before the rope is taut, m (0.1)" },
        Param { name: "ropePos", kind: Kind::Vec3, used: true, doc: "the rope's anchor (zero: 2 m in front of his chest)" },
        Param { name: "ropeAttachedToInstance", kind: Kind::Int, used: false, doc: "(not done)" },
        Param { name: "ropedBodyPart", kind: Kind::Int, used: true, doc: "the part it's tied to (-1: none, the hands grab it)" },
        Param { name: "ropeTaut", kind: Kind::Bool, used: false, doc: "(not done)" },
        Param { name: "playerControl", kind: Kind::Bool, used: true, doc: "grabLeft / grabRight pick the hands (false)" },
        Param { name: "grabLeft", kind: Kind::Bool, used: true, doc: "(true)" },
        Param { name: "grabRight", kind: Kind::Bool, used: true, doc: "(true)" },
        Param { name: "reach", kind: Kind::Float, used: true, doc: "how far from the chest a hand reaches, m (0.7)" },
        Param { name: "armTwist", kind: Kind::Float, used: true, doc: "(0)" },
        Param { name: "direction", kind: Kind::Vec3, used: true, doc: "room: which way the anchor moves off (zero: the way he faces)" },
        Param { name: "speed", kind: Kind::Float, used: true, doc: "room: its speed, m/s (1.2)" },
        Param { name: "delay", kind: Kind::Float, used: true, doc: "room: s before it moves (0.7)" },
        Param { name: "force", kind: Kind::Float, used: true, doc: "the rope's most pull, body weights (0.6)" },
    ] },
    Message { name: "dangle", params: &[
        Param { name: "doGrab", kind: Kind::Bool, used: true, doc: "reach up for what he hangs from (false)" },
        Param { name: "grabFrequency", kind: Kind::Float, used: true, doc: "how often, 0 to 1 (0.5)" },
        Param { name: "point", kind: Kind::Vec3, used: true, doc: "room: where his hands are held (zero: 1 m over his head, his feet off the floor)" },
        Param { name: "hold", kind: Kind::Bool, used: true, doc: "room: hold them there (true; false: something else holds him)" },
    ] },
    Message { name: "carried", params: &[
        Param { name: "point", kind: Kind::Vec3, used: true, doc: "the room's stand-in for the carrier's shoulder, his waist held there (zero: 0.6 m over his pelvis)" },
        Param { name: "hold", kind: Kind::Bool, used: true, doc: "hold him at point (true; false: just the stiffnesses, for when something else carries him)" },
        Param { name: "trunkStiffness", kind: Kind::Float, used: true, doc: "spine (5)" },
        Param { name: "legStiffness", kind: Kind::Float, used: true, doc: "left leg while moving (2)" },
        Param { name: "armStiffness", kind: Kind::Float, used: true, doc: "(1.5)" },
        Param { name: "clavicleStiffness", kind: Kind::Float, used: true, doc: "(4)" },
        Param { name: "neckStiffness", kind: Kind::Float, used: true, doc: "(7)" },
        Param { name: "stillLegStiffness", kind: Kind::Float, used: true, doc: "right leg, and the left once he's still (4)" },
    ] },
    Message { name: "buoyancy", params: &[
        Param { name: "buoyancy", kind: Kind::Float, used: true, doc: "up-force on a part under, of its weight (1.1)" },
        Param { name: "chestBuoyancy", kind: Kind::Float, used: true, doc: "extra on the chest, tenths of his weight (0.5)" },
        Param { name: "damping", kind: Kind::Float, used: true, doc: "water drag per second (3)" },
        Param { name: "surfacePoint", kind: Kind::Vec3, used: true, doc: "a point on the water (zero: his chest when it starts)" },
        Param { name: "surfaceNormal", kind: Kind::Vec3, used: true, doc: "the water's up (0 0 1)" },
        Param { name: "righting", kind: Kind::Bool, used: true, doc: "turn him face up (true)" },
        Param { name: "rightingStrength", kind: Kind::Float, used: true, doc: "of his weight, as torque (0.3)" },
        Param { name: "rightingTime", kind: Kind::Float, used: true, doc: "s it lasts, easing off (2)" },
        Param { name: "footBuoyancy", kind: Kind::Float, used: true, doc: "RDR2 core: times buoyancy on the shins and feet (0.1)" },
    ] },
    Message { name: "animPose", params: &[
        Param { name: "pose", kind: Kind::Str, used: true, doc: "joint:a,b,c;joint:a,b,c; angles in radians (empty: hold the pose he is in)" },
        Param { name: "mask", kind: Kind::Str, used: true, doc: "the joints it may drive (fb)" },
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "(12)" },
        Param { name: "damping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "muscleStiffness", kind: Kind::Float, used: true, doc: "(-1: the joints' own)" },
        Param { name: "gravityCompensation", kind: Kind::Float, used: true, doc: "gravity opposition (0)" },
        Param { name: "stiffnessLeftArm", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "dampingLeftArm", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "muscleStiffnessLeftArm", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "gravCompLeftArm", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "stiffnessRightArm", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "dampingRightArm", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "muscleStiffnessRightArm", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "gravCompRightArm", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "stiffnessSpine", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "dampingSpine", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "muscleStiffnessSpine", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "gravCompSpine", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "stiffnessLeftLeg", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "dampingLeftLeg", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "muscleStiffnessLeftLeg", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "gravCompLeftLeg", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "stiffnessRightLeg", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "dampingRightLeg", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "muscleStiffnessRightLeg", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "gravCompRightLeg", kind: Kind::Float, used: true, doc: "(-1: the global one)" },
        Param { name: "useZMPGravityCompensation", kind: Kind::Bool, used: false, doc: "(not done: false)" },
        Param { name: "overideHeadlook", kind: Kind::Bool, used: true, doc: "leave the neck alone (false)" },
        Param { name: "overidePointArm", kind: Kind::Bool, used: true, doc: "leave the left arm alone (false)" },
        Param { name: "overidePointGun", kind: Kind::Bool, used: true, doc: "leave the right arm alone (false)" },
        Param { name: "connectedLeftHand", kind: Kind::Int, used: true, doc: "nonzero: anchored, its limb at half stiffness (0)" },
        Param { name: "connectedRightHand", kind: Kind::Int, used: true, doc: "nonzero: anchored, its limb at half stiffness (0)" },
        Param { name: "connectedLeftFoot", kind: Kind::Int, used: true, doc: "nonzero: anchored, its limb at half stiffness (0)" },
        Param { name: "connectedRightFoot", kind: Kind::Int, used: true, doc: "nonzero: anchored, its limb at half stiffness (0)" },
        Param { name: "animSource", kind: Kind::Int, used: false, doc: "(not done: our pose only)" },
        Param { name: "dampenSideMotionInstanceIndex", kind: Kind::Int, used: false, doc: "(not done)" },
        Param { name: "vDampenPlanePoint", kind: Kind::Vec3, used: false, doc: "(not done)" },
        Param { name: "vDampenPlaneNormal", kind: Kind::Vec3, used: true, doc: "brakes motion along it (zero: off)" },
        Param { name: "effectorMask", kind: Kind::Str, used: true, doc: "RDR2 core: the joints it drives (fb; mask is read when this isn't sent)" },
        Param { name: "stiffnessLeftAnkle", kind: Kind::Float, used: true, doc: "RDR2 core (-1: leave it)" },
        Param { name: "dampingLeftAnkle", kind: Kind::Float, used: true, doc: "RDR2 core (1)" },
        Param { name: "muscleStiffnessLeftAnkle", kind: Kind::Float, used: true, doc: "RDR2 core (-1: none)" },
        Param { name: "stiffnessRightAnkle", kind: Kind::Float, used: true, doc: "RDR2 core (-1: leave it)" },
        Param { name: "dampingRightAnkle", kind: Kind::Float, used: true, doc: "RDR2 core (1)" },
        Param { name: "muscleStiffnessRightAnkle", kind: Kind::Float, used: true, doc: "RDR2 core (-1: none)" },
        Param { name: "dampenSideMotionInstanceHandle", kind: Kind::Int, used: false, doc: "RDR2 core: an obstacle's id, or -1 the world (ours), holds his COM near the plane through it and his side axis at the start (0: off)" },
    ] },
    Message { name: "armsWindmill", params: &[
        Param { name: "?5e7f11cc", kind: Kind::Int, used: false, doc: "RDR2: an object handle for the left circle: the chest here" },
        Param { name: "?7ec5afa3", kind: Kind::Int, used: false, doc: "RDR2: an object handle for the right circle: the chest here" },
        Param { name: "leftNormal", kind: Kind::Vec3, used: true, doc: "the circle's plane normal, his frame: x forward, y left, z up (0 1 0); RDR2 core: the circle's turns about the chest's X, Y, Z, rad (0 0.2 0.2)" },
        Param { name: "leftCentre", kind: Kind::Vec3, used: true, doc: "the circle's centre from the part, his frame (0.15 0.25 0.05); RDR2 core: in the chest's frame (0 0.5 -0.1)" },
        Param { name: "leftPartID", kind: Kind::Int, used: true, doc: "the part the circle is from (10, the chest); RDR2 core: always the chest" },
        Param { name: "leftRadius1", kind: Kind::Float, used: true, doc: "m (0.45); RDR2 core: 0.75" },
        Param { name: "leftRadius2", kind: Kind::Float, used: true, doc: "m (0.45); RDR2 core: 0.75" },
        Param { name: "leftSpeed", kind: Kind::Float, used: true, doc: "revolutions a second (1)" },
        Param { name: "rightNormal", kind: Kind::Vec3, used: true, doc: "the circle's plane normal, his frame: x forward, y left, z up (0 -1 0); RDR2 core: as the left (0 -0.2 -0.2)" },
        Param { name: "rightCentre", kind: Kind::Vec3, used: true, doc: "the circle's centre from the part, his frame (0.15 -0.25 0.05); RDR2 core: (0 -0.5 -0.1)" },
        Param { name: "rightPartID", kind: Kind::Int, used: true, doc: "the part the circle is from (10, the chest); RDR2 core: always the chest" },
        Param { name: "rightRadius1", kind: Kind::Float, used: true, doc: "m (0.45); RDR2 core: 0.75" },
        Param { name: "rightRadius2", kind: Kind::Float, used: true, doc: "m (0.45); RDR2 core: 0.75" },
        Param { name: "rightSpeed", kind: Kind::Float, used: true, doc: "revolutions a second (1)" },
        Param { name: "radius1Delta", kind: Kind::Float, used: true, doc: "adaptive, a second (0); not RDR2's" },
        Param { name: "radius2Delta", kind: Kind::Float, used: true, doc: "(0)" },
        Param { name: "speedDelta", kind: Kind::Float, used: true, doc: "(0)" },
        Param { name: "shoulderStiffness", kind: Kind::Float, used: true, doc: "(12)" },
        Param { name: "shoulderDamping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "elbowStiffness", kind: Kind::Float, used: true, doc: "(12)" },
        Param { name: "elbowDamping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "shoulderStiffnessDelta", kind: Kind::Float, used: true, doc: "adaptive, a second (0); not RDR2's" },
        Param { name: "elbowStiffnessDelta", kind: Kind::Float, used: true, doc: "(0)" },
        Param { name: "phaseOffset", kind: Kind::Float, used: true, doc: "the right arm behind the left, radians (3.14); RDR2 core: degrees, only kept by forceSync (0)" },
        Param { name: "dragReduction", kind: Kind::Float, used: true, doc: "(not done: Delirium's limb IK has no drag reduction)" },
        Param { name: "IKtwist", kind: Kind::Float, used: true, doc: "shoulder twist in the IK (0)" },
        Param { name: "angVelThreshold", kind: Kind::Float, used: true, doc: "adaptive: body spin, rad/s, before it speeds up (0.1)" },
        Param { name: "angVelGain", kind: Kind::Float, used: true, doc: "adaptive: faster by this per rad/s over (1)" },
        Param { name: "leftElbowMin", kind: Kind::Float, used: true, doc: "least elbow bend, rad (0.3); RDR2 core: approached at 5/s (0.5)" },
        Param { name: "rightElbowMin", kind: Kind::Float, used: true, doc: "(0.3); RDR2 core: (0.5)" },
        Param { name: "mirrorMode", kind: Kind::Int, used: true, doc: "1: the right arm goes round the other way (0); RDR2 core: 1 the right circle the left's mirrored, 2 its centre only, 0 its own (1)" },
        Param { name: "adaptiveMode", kind: Kind::Int, used: true, doc: "speed tracks his spin (false); RDR2 core: 1 the turn's sign from his spin, 2 its speed, 3 and the shoulder stiffness (0)" },
        Param { name: "forceSync", kind: Kind::Bool, used: true, doc: "no phase offset (false); RDR2 core: keep the right arm phaseOffset behind the left (true)" },
        Param { name: "useLeft", kind: Kind::Bool, used: true, doc: "(true)" },
        Param { name: "useRight", kind: Kind::Bool, used: true, doc: "(true)" },
        Param { name: "disableOnImpact", kind: Kind::Bool, used: true, doc: "a hand stops once it hits something (true)" },
    ] },
    Message { name: "spineTwist", params: &[
        Param { name: "pos", kind: Kind::Vec3, used: true, doc: "the point his chest turns to (zero: off to his left, a little behind)" },
        Param { name: "offset", kind: Kind::Float, used: true, doc: "rad, turn this much past it (0)" },
        Param { name: "velX", kind: Kind::Float, used: true, doc: "the point's speed, to lead it (0)" },
        Param { name: "velY", kind: Kind::Float, used: true, doc: "(0)" },
        Param { name: "velZ", kind: Kind::Float, used: true, doc: "(0)" },
        Param { name: "spineTwistExcess", kind: Kind::Float, used: true, doc: "each spine joint may twist this much past its range (0.35)" },
        Param { name: "spStrength", kind: Kind::Float, used: true, doc: "the spine's stiffness (12)" },
        Param { name: "spDamping", kind: Kind::Float, used: true, doc: "(2)" },
        Param { name: "twistClavicles", kind: Kind::Bool, used: true, doc: "the clavicles turn with it (true)" },
        Param { name: "allwaysTwist", kind: Kind::Bool, used: true, doc: "keep twisting even when it's right behind him (false)" },
    ] },
    Message { name: "wave", params: &[
        Param { name: "right", kind: Kind::Bool, used: true, doc: "wave the right hand (true; false: the left)" },
        Param { name: "time", kind: Kind::Float, used: true, doc: "s of waving (2.25)" },
        Param { name: "rate", kind: Kind::Float, used: true, doc: "waves a second (2)" },
        Param { name: "size", kind: Kind::Float, used: true, doc: "how far the hand sways each side, m (0.18)" },
    ] },
    Message { name: "breathe", params: &[
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "the spine's stiffness while breathing (10)" },
        Param { name: "effectorMask", kind: Kind::Str, used: true, doc: "the joints that breathe (us: the spine)" },
        Param { name: "breathingScale", kind: Kind::Float, used: true, doc: "how deep, 0 none (1)" },
    ] },
    Message { name: "softKeyframe", params: &[
        Param { name: "followMultiplier", kind: Kind::Float, used: true, doc: "how hard each part chases its place, 1/s^2 (10)" },
        Param { name: "maxAcceleration", kind: Kind::Float, used: true, doc: "the chase's most, m/s^2 (20)" },
        Param { name: "goSlowerWhenWeaker", kind: Kind::Bool, used: true, doc: "a weak joint's part chases less (true)" },
        Param { name: "yawOffset", kind: Kind::Float, used: true, doc: "rad, the pose turned about up (0)" },
        Param { name: "targetPosition", kind: Kind::Vec3, used: true, doc: "where the pose's pelvis is (zero: wherever his pelvis is)" },
        Param { name: "mask", kind: Kind::Str, used: true, doc: "the joints whose parts follow (fb)" },
    ] },
    Message { name: "splitBody", params: &[
        Param { name: "aimTarget", kind: Kind::Vec3, used: true, doc: "the point the gun aims at (zero: straight ahead)" },
        Param { name: "activePoseMask", kind: Kind::Str, used: true, doc: "joints held in their start pose (uk: spine and neck)" },
        Param { name: "hardConstraintMask", kind: Kind::Str, used: true, doc: "joints whose parts are pinned where they start (none)" },
        Param { name: "softConstraintMask", kind: Kind::Str, used: true, doc: "joints whose parts are eased back to where they start (none)" },
        Param { name: "collisionMask", kind: Kind::Str, used: true, doc: "kept, not used (fb)" },
        Param { name: "gravityCompensationMask", kind: Kind::Str, used: true, doc: "joints that hold up against gravity (ub)" },
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "the pose's stiffness (12)" },
        Param { name: "damping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "controllerStiffness", kind: Kind::Float, used: true, doc: "the aiming arm's stiffness (14)" },
        Param { name: "controllerDamping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "leftHandWeaponIndex", kind: Kind::Int, used: true, doc: "the left hand's gun, -1 none (-1)" },
        Param { name: "rightHandWeaponIndex", kind: Kind::Int, used: true, doc: "the right hand's gun, -1 none (0)" },
        Param { name: "constrainHands", kind: Kind::Bool, used: true, doc: "the support hand pinned on the grip (true)" },
        Param { name: "controlHandOrientation", kind: Kind::Bool, used: true, doc: "the support hand turns with the gun hand (true)" },
        Param { name: "leftHandStatus", kind: Kind::Int, used: true, doc: "0 free, 1 on the gun, 2 supporting (2)" },
        Param { name: "rightHandStatus", kind: Kind::Int, used: true, doc: "(1)" },
        Param { name: "adjustJointLimits", kind: Kind::Bool, used: true, doc: "soft limits on the elbows, a steadier aim (true)" },
        Param { name: "handStable", kind: Kind::Bool, used: true, doc: "the wrists stiffer (false)" },
        Param { name: "handStableStiff", kind: Kind::Float, used: true, doc: "(14)" },
        Param { name: "handStableDamp", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "recoilRelax", kind: Kind::Bool, used: true, doc: "the arms give on each shot (true)" },
        Param { name: "recoilRelaxAmount", kind: Kind::Float, used: true, doc: "how much, 0..1 (0.5)" },
        Param { name: "recoilRelaxTime", kind: Kind::Float, used: true, doc: "s back to stiff (0.3)" },
        Param { name: "breathingScale", kind: Kind::Float, used: true, doc: "the breathing sway, 0 none (1)" },
        Param { name: "fireWeapon", kind: Kind::Bool, used: true, doc: "send alone while it runs: a shot" },
    ] },
    Message { name: "shootDodge", params: &[
        Param { name: "rightArmTarget", kind: Kind::Vec3, used: true, doc: "the right arm aims here (zero: not aiming)" },
        Param { name: "leftArmTarget", kind: Kind::Vec3, used: true, doc: "the left arm aims here (zero: not aiming)" },
        Param { name: "targetValid", kind: Kind::Bool, used: true, doc: "aim at all (true)" },
        Param { name: "collisionPoint", kind: Kind::Vec3, used: true, doc: "kept, not used" },
        Param { name: "collisionNormal", kind: Kind::Vec3, used: true, doc: "kept, not used" },
        Param { name: "collisionObjectVelocity", kind: Kind::Vec3, used: true, doc: "kept, not used" },
        Param { name: "collisionIndex", kind: Kind::Int, used: true, doc: "kept, not used" },
        Param { name: "collisionTime", kind: Kind::Float, used: true, doc: "kept, not used" },
        Param { name: "collisionTimeThreshold", kind: Kind::Float, used: true, doc: "s touching that ends it (0.2)" },
        Param { name: "collisionAccelThreshold", kind: Kind::Float, used: true, doc: "m/s^2 of a hit that ends it (40)" },
        Param { name: "collisionVelThreshold", kind: Kind::Float, used: true, doc: "m/s lost in a step that ends it (2)" },
        Param { name: "collisionReaction", kind: Kind::Int, used: true, doc: "after: 0 catchFall, 1 bodyRollUp, 2 bodyBalance, 3 nothing (0)" },
        Param { name: "trunkStiffness", kind: Kind::Float, used: true, doc: "(12)" },
        Param { name: "trunkDamping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "legStiffness", kind: Kind::Float, used: true, doc: "(10)" },
        Param { name: "legDamping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "legTaper", kind: Kind::Float, used: true, doc: "each joint down the leg this times softer (0.8)" },
        Param { name: "aimingArmStiffness", kind: Kind::Float, used: true, doc: "(14)" },
        Param { name: "aimingArmDamping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "notAimingArmStiffness", kind: Kind::Float, used: true, doc: "(6)" },
        Param { name: "notAimingArmDamping", kind: Kind::Float, used: true, doc: "(0.7)" },
        Param { name: "notAimingArmTaper", kind: Kind::Float, used: true, doc: "each joint down the arm this times softer (0.7)" },
        Param { name: "headStiffness", kind: Kind::Float, used: true, doc: "(10)" },
        Param { name: "headDamping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "wristMuscleStiffness", kind: Kind::Float, used: true, doc: "the aiming wrist's muscle stiffness (10)" },
        Param { name: "oriStiff", kind: Kind::Float, used: true, doc: "holding the dive's attitude, 1/s^2 (30)" },
        Param { name: "oriDamp", kind: Kind::Float, used: true, doc: "1/s (8)" },
        Param { name: "oriSplit", kind: Kind::Float, used: true, doc: "its share on the pelvis, the rest on the chest (0.5)" },
        Param { name: "supportedHeightThreshold", kind: Kind::Float, used: true, doc: "kept, not used" },
        Param { name: "legNoiseScale", kind: Kind::Float, used: true, doc: "the legs' drift, rad (0.05)" },
        Param { name: "breathingScale", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "armAnimTask", kind: Kind::Bool, used: true, doc: "kept, not used (no animation stream)" },
        Param { name: "leftArmAnimTaskParent", kind: Kind::Int, used: true, doc: "kept, not used" },
        Param { name: "rightArmAnimTaskParent", kind: Kind::Int, used: true, doc: "kept, not used" },
        Param { name: "state", kind: Kind::Int, used: true, doc: "where it starts: 0 enter, 1 hold (0)" },
    ] },
    Message { name: "armHang", params: &[
        Param { name: "constraintPosition", kind: Kind::Vec3, used: true, doc: "the world point his hands hang on, carried with the object after (zero: between them where they are)" },
        Param { name: "targetPosition", kind: Kind::Vec3, used: true, doc: "where the arm reaches and he looks (zero: the anchor)" },
        Param { name: "instanceIndex", kind: Kind::Int, used: true, doc: "the object hung from, -1 the world (-1)" },
        Param { name: "boundIndex", kind: Kind::Int, used: true, doc: "its part, from 1 (0: the object as a whole)" },
        Param { name: "letGo", kind: Kind::Int, used: true, doc: "sent without start while hanging: that hand lets go, 0 left, 1 right (both: he drops)" },
    ] },
    Message { name: "crushReaction", params: &[
        Param { name: "effectorMask", kind: Kind::Str, used: true, doc: "the joints that react, an arm only if all of it is in (ub)" },
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "(12)" },
        Param { name: "damping", kind: Kind::Float, used: true, doc: "(1)" },
        Param { name: "obstacleID", kind: Kind::Int, used: true, doc: "the object crushing him, -1 none (-1)" },
        Param { name: "flinchMode", kind: Kind::Int, used: true, doc: "0 cover the head, 1 brace the arms at it (0)" },
        Param { name: "useInjuries", kind: Kind::Bool, used: true, doc: "a hurt arm doesn't cover or brace (false)" },
        Param { name: "obstaclePosition", kind: Kind::Vec3, used: true, doc: "room: where the thing crushing him is without an obstacleID (zero: 1 m over his head)" },
    ] },
];

pub fn build(name: &str, p: &mut P) -> Option<Result<Box<dyn Extra>, Error>> {
    fn b<E: Extra + 'static>(e: Result<E, Error>) -> Option<Result<Box<dyn Extra>, Error>> {
        Some(e.map(|e| Box::new(e) as _))
    }
    match name {
        "yanked" => b(Yanked::new(p)),
        "dragged" => b(Dragged::new(p)),
        "dangle" => b(Dangle::new(p)),
        "carried" => b(Carried::new(p)),
        "buoyancy" => b(Buoyancy::new(p)),
        "animPose" => b(AnimPose::new(p)),
        "armsWindmill" => b(ArmsWindmill::new(p)),
        "spineTwist" => b(Twist::new(p)),
        "wave" => b(Wave::new(p)),
        "breathe" => b(Breathe::new(p)),
        "softKeyframe" => b(SoftKeyframe::new(p)),
        "splitBody" => b(SplitBody::new(p)),
        "shootDodge" => b(ShootDodge::new(p)),
        "armHang" => b(ArmHang::new(p)),
        "crushReaction" => b(CrushReaction::new(p)),
        _ => None,
    }
}

fn pos(c: &Ctx, k: usize) -> Vec3 {
    c.b.part(k).m.d
}

fn weight(c: &Ctx) -> f32 {
    c.b.doll.mass() * delirium_data::kv!(115)
}

fn facing(c: &Ctx) -> Vec3 {
    let up = c.ch.up.normalize_or_zero();
    let f = (pos(c, (*CLAVICLE())[0] + 1) - pos(c, (*CLAVICLE())[1] + 1)).cross(up);
    (f - up * f.dot(up)).normalize_or_zero()
}

fn spring(c: &mut Ctx, part: usize, to: Vec3, k: f32, d: f32, cap: f32) {
    let f = (to - pos(c, part)) * k - c.b.part(part).v * d;
    c.out.force(part, f.clamp_length_max(cap));
}

fn parse_pose(s: &str) -> Vec<(usize, [f32; 3])> {
    s.split(';').filter_map(|e| {
        let (j, a) = e.split_once(':')?;
        let mut a = a.split(',').map(|x| x.trim().parse::<f32>().unwrap_or(0.0));
        Some((j.trim().parse().ok().filter(|&j| j < delirium_data::kv!(127 int))?, [a.next()?, a.next().unwrap_or(0.0), a.next().unwrap_or(0.0)]))
    }).collect()
}

pub struct Yanked {
    point: Vec3,
    part: usize,
    force: f32,
    pull: f32,
    arm: [f32; 4],
    spine: [f32; 4],
    start_time: f32,
    ramp_from: f32,
    time_end: f32,
    ramp_to: f32,
    legs: (f32, f32),
    pitch: (f32, f32),
    bend: f32,
    thresh: f32,
    hula: (f32, f32, f32),
    relax: (f32, f32),
    roll: f32,
    ground: [f32; 6],
    head: bool,
    head_pos: Vec3,
    head_index: i32,
    t: f32,
    next_relax: f32,
    down: bool,
    still: f32,
    finished: bool,
    out: Vec<String>,
    /// RDR2 core: RDR2's yanked, run in this one's place
    r2: Box<crate::extras_ysk_rdr2::YankedRdr2>,
}

impl Yanked {
    fn new(p: &mut P) -> Result<Yanked, Error> {
        let _ = (p.i("stepsTillStartEnd", 0)?, p.f("perStepReduction", 0.0)?, p.f("footFriction", 1.0)?);
        let _ = (p.f("turnThresholdMin", 0.0)?, p.f("turnThresholdMax", 0.0)?, p.f("groundFriction", 1.0)?);
        let _ = p.f("headLookAtVelProb", 0.0)?;
        let r2 = Box::new(crate::extras_ysk_rdr2::YankedRdr2::new(crate::extras_ysk_rdr2::YankedParams::new(p)?));
        Ok(Yanked {
            r2,
            point: p.v("point", Vec3::ZERO)?,
            part: p.i("part", CHEST() as i32)?.clamp(0, delirium_data::kv!(127 int)) as usize,
            force: p.f("force", 1.0)?,
            pull: p.f("pull", delirium_data::kv!(60))?,
            arm: [p.f("armStiffnessStart", delirium_data::kv!(5))?, p.f("armDampingStart", delirium_data::kv!(78))?, p.f("armStiffness", delirium_data::kv!(117))?, p.f("armDamping", 1.0)?],
            spine: [p.f("spineStiffnessStart", delirium_data::kv!(5))?, p.f("spineDampingStart", delirium_data::kv!(78))?, p.f("spineStiffness", delirium_data::kv!(117))?,
                    p.f("spineDamping", 1.0)?],
            start_time: p.f("timeAtStartValues", delirium_data::kv!(116))?,
            ramp_from: p.f("rampTimeFromStartValues", delirium_data::kv!(78))?,
            time_end: p.f("timeStartEnd", delirium_data::kv!(5))?,
            ramp_to: p.f("rampTimeToEndValues", 1.0)?,
            legs: (p.f("lowerBodyStiffness", delirium_data::kv!(21))?, p.f("lowerBodyStiffnessEnd", delirium_data::kv!(65))?),
            pitch: (p.f("hipPitchForward", delirium_data::kv!(119))?, p.f("hipPitchBack", 1.0)?),
            bend: p.f("spineBend", delirium_data::kv!(74))?,
            thresh: p.f("comVelRDSThresh", delirium_data::kv!(84))?,
            hula: (p.f("hulaPeriod", delirium_data::kv!(17))?, p.f("hipAmplitude", 1.0)?, p.f("spineAmplitude", 1.0)?),
            relax: (p.f("minRelaxPeriod", delirium_data::kv!(24))?, p.f("maxRelaxPeriod", delirium_data::kv!(12))?),
            roll: p.f("rollHelp", 0.5)?,
            ground: [p.f("groundLegStiffness", delirium_data::kv!(117))?, p.f("groundLegDamping", 1.0)?, p.f("groundArmStiffness", delirium_data::kv!(117))?,
                     p.f("groundArmDamping", 1.0)?, p.f("groundSpineStiffness", delirium_data::kv!(117))?, p.f("groundSpineDamping", 1.0)?],
            head: p.b("useHeadLook", true)?,
            head_pos: p.v("headLookPos", Vec3::ZERO)?,
            head_index: p.i("headLookInstanceIndex", -1)?,
            t: 0.0,
            next_relax: 0.0,
            down: false,
            still: 0.0,
            finished: false,
            out: vec![],
        })
    }

    fn end(&mut self, why: &str) {
        self.finished = true;
        self.out.push(format!("#say yanked {why}"));
        if self.head {
            self.out.push("headLook start=false".into());
        }
    }
}

fn ramp(a: f32, b: f32, t: f32, t0: f32, time: f32) -> f32 {
    a + (b - a) * ((t - t0) / time.max(delirium_data::kv!(134))).clamp(0.0, 1.0)
}

impl Extra for Yanked {
    fn yanked_stirrups_set(&mut self, alive: Option<[bool; 2]>) {
        self.r2.stirrups = alive;
    }

    /// RDR2's pointGun reads yanked's lassoedtype (+0x184 = 1: the arm straight along the aim).
    fn pg_yanked(&self) -> Option<bool> {
        Some(self.r2.p.lassoed == 1)
    }

    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if c.ch.rdr2 {
            self.r2.tick(c, dt);
            // ours: the room's stand-in rope pulls as it does without the core (RDR2's rope is the game's)
            if self.t < self.pull && self.force > 0.0 {
                let w = weight(c);
                spring(c, self.part, self.r2.p.rope, w * delirium_data::kv!(5), w, w * self.force);
            }
            self.t += dt;
            return;
        }
        if self.t == 0.0 {
            if self.point == Vec3::ZERO {
                self.point = pos(c, CHEST()) + facing(c) * 2.0;
            }
            if self.head_pos == Vec3::ZERO {
                self.head_pos = self.point;
            }
            self.out.push("bodyBalance start=true".into());
            if self.head {
                let h = self.head_pos;
                self.out.push(format!("headLook start=true instanceIndex={} pos={},{},{} stiffness={} damping=1", self.head_index,
                                      h.x, h.y, h.z, delirium_data::kv!(117)));
            }
            self.out.push("#say yanked pulled".into());
            self.next_relax = self.relax.1;
        }
        self.t += dt;
        let t = self.t;
        let w = weight(c);
        let to = self.point - pos(c, self.part);
        let dir = Vec3::new(to.x, to.y, 0.0).normalize_or_zero();
        if t < self.pull && self.force > 0.0 {
            spring(c, self.part, self.point, w * delirium_data::kv!(5), w, w * self.force);
        }
        let feet = pos(c, (*FEET())[0]).z.min(pos(c, (*FEET())[1]).z);
        if !self.down && (c.vel.length() > self.thresh || c.com.z - feet < delirium_data::kv!(10)) {
            self.down = true;
            self.out.push("#say yanked ground".into());
        }
        let speed = c.vel.length();
        self.still = if speed < delirium_data::kv!(24) { self.still + dt } else { 0.0 };
        if self.down {
            let g = self.ground;
            c.ch.set_stiffness(g[0], g[1], "lb", None);
            c.ch.set_stiffness(g[2], g[delirium_data::kv!(0 int)], "ua", None);
            c.ch.set_stiffness(g[delirium_data::kv!(1 int)], g[delirium_data::kv!(2 int)], "uk", None);
            c.out.torque(0, Vec3::Z.cross(dir) * (w * self.roll * delirium_data::kv!(78)));
            if self.still > 1.0 {
                self.end("settled");
            }
            return;
        }
        let k = |v: &[f32; 4]| {
            let up = ramp(v[0], v[2], t, self.start_time, self.ramp_from);
            (up, ramp(v[1], v[delirium_data::kv!(0 int)], t, self.start_time, self.ramp_from))
        };
        let ((ak, ad), (sk, sd)) = (k(&self.arm), k(&self.spine));
        let lk = ramp(self.legs.0, self.legs.1, t, self.time_end, self.ramp_to);
        self.next_relax -= dt;
        if self.next_relax < 0.0 {
            let r = c.ch.random();
            self.next_relax = self.relax.0 + (self.relax.1 - self.relax.0) * r;
        }
        let relax = if self.next_relax < delirium_data::kv!(9) { delirium_data::kv!(135) } else { 1.0 };
        c.ch.set_stiffness(ak * relax, ad, "ua", None);
        c.ch.set_stiffness(sk * relax, sd, "uk", None);
        c.ch.set_stiffness(lk, 1.0, "lb", None);
        let along = dir.dot(facing(c));
        let fade = if t < self.pull { 1.0 } else { ramp(1.0, 0.0, t, self.pull, 1.0) };
        let pitch = if along > 0.0 { self.pitch.0 } else { self.pitch.1 } * along * fade;
        let sway = (t / self.hula.0.max(delirium_data::kv!(68)) * std::f32::consts::TAU).sin() * fade;
        c.ch.set_relative((*SPINE())[0], 0, 0.5 * pitch);
        c.ch.set_relative((*SPINE())[0], 1, delirium_data::kv!(24) * self.hula.1 * sway);
        for &j in &(*SPINE())[1..] {
            c.ch.set_relative(j, 0, -delirium_data::kv!(17) * self.bend * along * fade);
            c.ch.set_relative(j, 1, -delirium_data::kv!(24) * self.hula.2 * sway);
        }
        if t > self.time_end + self.ramp_to && self.still > 0.5 {
            self.end("recovered");
        }
    }

    fn lines(&mut self) -> Vec<String> {
        let mut v = std::mem::take(&mut self.out);
        v.extend(self.r2.lines());
        v
    }

    fn done(&self) -> bool {
        self.finished
    }

    fn stop(&mut self, c: &mut Ctx) {
        self.r2.stop(c);
    }
}

pub struct Dragged {
    arm: [f32; 3],
    radius_tol: f32,
    length_tol: f32,
    anchor: Vec3,
    part: i32,
    player: bool,
    want: [bool; 2],
    reach: f32,
    twist: f32,
    dir: Vec3,
    speed: f32,
    delay: f32,
    force: f32,
    length: f32,
    grabbed: [Option<f32>; 2],
    pulled: bool,
    t: f32,
    out: Vec<String>,
}

impl Dragged {
    fn new(p: &mut P) -> Result<Dragged, Error> {
        let _ = (p.i("ropeAttachedToInstance", -1)?, p.b("ropeTaut", false)?);
        Ok(Dragged {
            arm: [p.f("armStiffness", delirium_data::kv!(117))?, p.f("armDamping", 1.0)?, p.f("armMuscleStiffness", 1.0)?],
            radius_tol: p.f("radiusTolerance", delirium_data::kv!(17))?,
            length_tol: p.f("lengthTolerance", delirium_data::kv!(78))?,
            anchor: p.v("ropePos", Vec3::ZERO)?,
            part: p.i("ropedBodyPart", -1)?.clamp(-1, delirium_data::kv!(127 int)),
            player: p.b("playerControl", false)?,
            want: [p.b("grabLeft", true)?, p.b("grabRight", true)?],
            reach: p.f("reach", delirium_data::kv!(74))?,
            twist: p.f("armTwist", 0.0)?,
            dir: p.v("direction", Vec3::ZERO)?,
            speed: p.f("speed", delirium_data::kv!(136))?,
            delay: p.f("delay", delirium_data::kv!(74))?,
            force: p.f("force", delirium_data::kv!(119))?,
            length: 0.0,
            grabbed: [None; 2],
            pulled: false,
            t: 0.0,
            out: vec![],
        })
    }
}

impl Extra for Dragged {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        let chest = pos(c, CHEST());
        if self.t == 0.0 {
            self.out.push("bodyBalance start=true".into());
            if self.dir == Vec3::ZERO {
                self.dir = facing(c);
            }
            self.dir = Vec3::new(self.dir.x, self.dir.y, 0.0).normalize_or(Vec3::NEG_Y);
            if self.anchor == Vec3::ZERO {
                self.anchor = chest + self.dir * 2.0 - Vec3::Z * delirium_data::kv!(60);
            }
            self.length = match self.part {
                -1 => (self.anchor - chest).length() - delirium_data::kv!(24),
                k => (self.anchor - pos(c, k as usize)).length(),
            };
        }
        self.t += dt;
        if self.t > self.delay {
            self.anchor += self.dir * (self.speed * dt);
        }
        let w = weight(c);
        let cap = self.force * w;
        let along = |s: f32| self.anchor - self.dir * s;
        let mut taut = false;
        if self.part >= 0 {
            let k = self.part as usize;
            let over = (self.anchor - pos(c, k)).length() - self.length - self.length_tol;
            if over > 0.0 {
                taut = true;
                spring(c, k, self.anchor, w * delirium_data::kv!(5), w, cap);
            }
        }
        if self.pulled {
            c.ch.set_stiffness(delirium_data::kv!(5), delirium_data::kv!(119), "fb", None);
        }
        for i in 0..2 {
            if !if self.player { self.want[i] } else { self.part < 0 } {
                continue;
            }
            let (l, hand) = (crate::nmshot::limb(i), (*HANDS())[i]);
            match self.grabbed[i] {
                Some(s) => {
                    let to = along(s);
                    if (to - pos(c, hand)).length() > self.length_tol {
                        taut = true;
                    }
                    spring(c, hand, to, w * delirium_data::kv!(35), w, cap * 0.5);
                    c.ch.eff[l.e[1]].set_stiffness(self.arm[0], self.arm[1], Some(self.arm[2]));
                    c.ch.eff[l.e[2]].set_stiffness(self.arm[0], self.arm[1], Some(self.arm[2]));
                }
                None => {
                    let h = pos(c, hand);
                    let s = (self.anchor - h).dot(self.dir).clamp(0.0, self.length);
                    let to = along(s);
                    if (to - h).length() < self.radius_tol {
                        self.grabbed[i] = Some(s);
                        self.out.push(format!("#say dragged grabbed {}", ["left", "right"][i]));
                        continue;
                    }
                    let target = chest + (to - chest).clamp_length_max(self.reach);
                    c.ch.eff[l.e[1]].set_stiffness(self.arm[0], self.arm[1], Some(self.arm[2]));
                    c.ch.eff[l.e[2]].set_stiffness(self.arm[0], self.arm[1], Some(self.arm[2]));
                    crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[delirium_data::kv!(0 int)], target, self.twist, 0.0,
                                          1.0 - 2.0 * i as f32, 1.0, Vec3::ZERO, None);
                }
            }
        }
        if taut != self.pulled {
            self.pulled = taut;
            self.out.push(format!("pedalLegs start={taut}"));
            if taut {
                self.out.push("bodyBalance start=false".into());
            }
        }
    }

    fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.out)
    }
}

pub struct Dangle {
    grab: bool,
    freq: f32,
    point: Vec3,
    hold: bool,
    t: f32,
    pedal: (bool, f32),
    hang: (bool, f32),
    reach: f32,
    reaching: bool,
    out: Vec<String>,
}

impl Dangle {
    fn new(p: &mut P) -> Result<Dangle, Error> {
        Ok(Dangle { grab: p.b("doGrab", false)?, freq: p.f("grabFrequency", 0.5)?.clamp(0.0, 1.0), point: p.v("point", Vec3::ZERO)?,
                    hold: p.b("hold", true)?, t: 0.0, pedal: (false, 1.0), hang: (false, 2.0), reach: 1.0, reaching: false,
                    out: vec![] })
    }

    fn wait(c: &mut Ctx) -> f32 {
        1.0 + delirium_data::kv!(5) * c.ch.random()
    }
}

impl Extra for Dangle {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if self.point == Vec3::ZERO {
            self.point = pos(c, (*NECK())[1] + 1) + Vec3::Z * 1.0;
            self.out.push("bodyBalance start=false".into());
        }
        self.t += dt;
        let w = weight(c);
        if self.hold {
            for (i, k) in (*HANDS()).into_iter().enumerate() {
                let at = self.point + Vec3::Z.cross(facing(c)) * if i == 0 { delirium_data::kv!(9) } else { -delirium_data::kv!(9) };
                spring(c, k, at, w * 2.0, w * delirium_data::kv!(24), w * delirium_data::kv!(24));
            }
            spring(c, CHEST(), self.point - Vec3::Z * delirium_data::kv!(18), w * delirium_data::kv!(35), w * delirium_data::kv!(12), w * delirium_data::kv!(136));
        }
        for i in 0..2 {
            for (j, s) in [((*CLAVICLE())[i], delirium_data::kv!(137)), ((*SHOULDER())[i], delirium_data::kv!(65)), ((*ELBOW())[i], delirium_data::kv!(32)), (crate::nmshot::limb(i).e[delirium_data::kv!(0 int)], delirium_data::kv!(35))] {
                c.ch.eff[j].set_stiffness(s, s / delirium_data::kv!(31), None);
            }
        }
        c.ch.set_stiffness(delirium_data::kv!(5), delirium_data::kv!(24), "us", None);
        c.ch.set_stiffness(delirium_data::kv!(5), delirium_data::kv!(24), "lb", None);
        self.hang.1 -= dt;
        if self.hang.1 < 0.0 {
            self.hang = (!self.hang.0, Self::wait(c));
        }
        if self.hang.0 {
            let s = (self.t * delirium_data::kv!(33)).sin() * delirium_data::kv!(116);
            for j in (*HIP()) {
                c.ch.set_relative(j, 0, s);
            }
        }
        self.pedal.1 -= dt;
        if self.pedal.1 < 0.0 {
            self.pedal = (!self.pedal.0, Self::wait(c));
            self.out.push(format!("pedalLegs start={}", self.pedal.0));
        }
        if !self.grab {
            return;
        }
        self.reach -= dt;
        if self.reach < 0.0 {
            self.reaching = !self.reaching;
            self.reach = if self.reaching { 1.0 } else { Self::wait(c) * (1.0 + delirium_data::kv!(5) * (1.0 - self.freq)) };
            let p = self.point;
            self.out.push(match self.reaching {
                true => format!("headLook start=true pos={},{},{} stiffness={} damping=1", p.x, p.y, p.z, delirium_data::kv!(117)),
                false => "headLook start=false".into(),
            });
        }
        if self.reaching {
            for i in 0..2 {
                let l = crate::nmshot::limb(i);
                let at = self.point + Vec3::Z * delirium_data::kv!(9);
                crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[delirium_data::kv!(0 int)], at, 0.0, 0.0, 1.0 - 2.0 * i as f32, 1.0,
                                      Vec3::ZERO, None);
            }
        }
    }

    fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.out)
    }
}

pub struct Carried { point: Vec3, hold: bool, s: [f32; 6] }

impl Carried {
    fn new(p: &mut P) -> Result<Carried, Error> {
        Ok(Carried { point: p.v("point", Vec3::ZERO)?, hold: p.b("hold", true)?,
                     s: [p.f("trunkStiffness", delirium_data::kv!(64))?, p.f("legStiffness", 2.0)?, p.f("armStiffness", delirium_data::kv!(12))?, p.f("clavicleStiffness", delirium_data::kv!(84))?,
                         p.f("neckStiffness", delirium_data::kv!(32))?, p.f("stillLegStiffness", delirium_data::kv!(84))?] })
    }
}

impl Extra for Carried {
    fn tick(&mut self, c: &mut Ctx, _dt: f32) {
        if self.point == Vec3::ZERO {
            self.point = pos(c, 0) + Vec3::Z * delirium_data::kv!(119);
        }
        let [trunk, leg, arm, clav, neck, still_leg] = self.s;
        let still = c.b.part(0).v.length() < delirium_data::kv!(24);
        c.ch.set_stiffness(trunk, 0.5, "us", None);
        c.ch.set_stiffness(if still { still_leg } else { leg }, 0.5, "ll", None);
        c.ch.set_stiffness(still_leg, 0.5, "lr", None);
        c.ch.set_stiffness(arm, 0.5, "ua", None);
        c.ch.set_stiffness(clav, 0.5, "uc", None);
        c.ch.set_stiffness(neck, 1.0, "un", None);
        if self.hold {
            let w = weight(c);
            spring(c, 0, self.point, w * delirium_data::kv!(38), w * delirium_data::kv!(64), w * 2.0);
        }
    }
}

pub struct Buoyancy {
    buoyancy: f32,
    chest: f32,
    damping: f32,
    point: Vec3,
    normal: Vec3,
    righting: bool,
    strength: f32,
    time: f32,
    t: f32,
    /// RDR2 core: the message read with RDR2's defaults (rdr2_buoyancy.md section 2)
    rdr2: BuoyancyRdr2,
}

/// RDR2's buoyancy parameters: buoyancy, chestBuoyancy, footBuoyancy, damping, the plane (point, normal),
/// righting, rightingStrength (rightingTime is parsed by RDR2 but never read).
struct BuoyancyRdr2 { buoyancy: f32, chest: f32, foot: f32, damping: f32, point: Vec3, normal: Vec3, righting: bool, strength: f32 }

impl Buoyancy {
    fn new(p: &mut P) -> Result<Buoyancy, Error> {
        let rdr2 = BuoyancyRdr2 { buoyancy: p.f("buoyancy", 1.0)?, chest: p.f("chestBuoyancy", 8.0)?, foot: p.f("footBuoyancy", 0.1)?,
                                  damping: p.f("damping", 40.0)?, point: p.v("surfacePoint", Vec3::ZERO)?, normal: p.v("surfaceNormal", Vec3::Z)?,
                                  righting: p.b("righting", true)?, strength: p.f("rightingStrength", 25.0)? };
        Ok(Buoyancy { buoyancy: p.f("buoyancy", delirium_data::kv!(138))?, chest: p.f("chestBuoyancy", 0.5)?, damping: p.f("damping", delirium_data::kv!(5))?,
                      point: p.v("surfacePoint", Vec3::ZERO)?, normal: p.v("surfaceNormal", Vec3::Z)?.normalize_or(Vec3::Z),
                      righting: p.b("righting", true)?, strength: p.f("rightingStrength", delirium_data::kv!(24))?, time: p.f("rightingTime", 2.0)?,
                      t: 0.0, rdr2 })
    }

    /// RDR2's tick (rdr2_buoyancy.md section 4). Each part whose lowest point (centre less its half-size along the
    /// normal) is under the plane gets lift 9810 x volume x (depth / full height, at most 1) x buoyancy (x
    /// chestBuoyancy on the upper two spine parts, x footBuoyancy on the shins and feet) along the normal and a
    /// mass-free drag of -damping x its velocity. Righting: with the third spine part (RDR2's part 9) wet, a torque
    /// about up x its axis 0, (angle - 0.2) x rightingStrength, ramping in over 2 s from when it got wet.
    /// The core's water plane when setcharacterunderwater has it on, else (ours) the message's plane or its defaults; the
    /// part's half-size and volume are from its capsule or box (RDR2 reads its bound's); RDR2's
    /// "vector x dt at the part" is taken as a force.
    fn tick_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        let w = c.ch.core.cfg.water;
        if w.on {
            (self.rdr2.point, self.rdr2.normal) = (w.point, w.normal);
        }
        let r = &self.rdr2;
        let n = r.normal.normalize_or_zero();
        let spine = crate::nmshot::spine_parts();
        let (chest, nine) = ([spine[3], spine[4]], spine[3]);
        let (l, rl) = (crate::nmshot::limb(2), crate::nmshot::limb(3));
        let feet = [l.p[2], l.p[3], rl.p[2], rl.p[3]];
        let mut wet9 = false;
        for (k, d) in c.b.build.parts.iter().enumerate() {
            let (half, vol) = match d.boxed {
                Some(e) => (2.0 * e.min_element(), 8.0 * e.x * e.y * e.z),
                None => (d.radius, std::f32::consts::PI * d.radius * d.radius * (2.0 * d.half + 4.0 / 3.0 * d.radius)),
            };
            let p = c.b.part(k);
            let h = (r.point - (p.m.d - n * half)).dot(n);
            if h <= 0.0 {
                continue;
            }
            let full = 2.0 * half;
            let mut b = r.buoyancy;
            if chest.contains(&k) {
                b *= r.chest;
            }
            if feet.contains(&k) {
                b *= r.foot;
            }
            let lift = 9810.0 * vol * (h.min(full) / full) * b;
            c.out.force(k, n * lift);
            c.out.force(k, p.v * -r.damping);
            if k == nine {
                wet9 = true;
                if r.righting {
                    let a = p.m.a;
                    let w = c.ch.up.normalize_or_zero();
                    let x = w.cross(a);
                    let th = x.length().min(1.0).asin();
                    self.t += dt;
                    let ramp = 0.5 * self.t.clamp(0.0, 2.0);
                    if th - 0.2 > 0.0 {
                        c.out.torque(k, x.normalize_or_zero() * ((th - 0.2) * r.strength * ramp));
                    }
                }
            }
        }
        if !wet9 {
            self.t = 0.0;
        }
    }
}

impl Extra for Buoyancy {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if c.ch.rdr2 {
            return self.tick_rdr2(c, dt);
        }
        if self.point == Vec3::ZERO {
            self.point = pos(c, CHEST());
        }
        let n = self.normal;
        let mut wet = false;
        for k in 0..delirium_data::kv!(111 int) {
            let (p, m) = (c.b.part(k), c.b.doll.part_mass(k));
            let under = ((self.point - p.m.d).dot(n) / delirium_data::kv!(60)).clamp(0.0, 1.0);
            if under > 0.0 {
                wet = true;
                let chest = if k == CHEST() { self.chest * weight(c) * delirium_data::kv!(78) } else { 0.0 };
                c.out.force(k, (n * (m * delirium_data::kv!(115) * self.buoyancy + chest) - p.v * (m * self.damping)) * under);
            }
        }
        if !wet {
            return;
        }
        self.t += dt;
        c.ch.set_stiffness(delirium_data::kv!(35), delirium_data::kv!(20), "fb", None);
        if self.righting && self.t < self.time {
            let spine = (pos(c, CHEST()) - pos(c, 0)).normalize_or_zero();
            let front = (pos(c, (*CLAVICLE())[0] + 1) - pos(c, (*CLAVICLE())[1] + 1)).cross(spine).normalize_or_zero();
            let ease = 1.0 - self.t / self.time.max(delirium_data::kv!(51));
            let tq = front.cross(n) * (self.strength * weight(c) * ease);
            c.out.torque(CHEST(), tq);
            c.out.torque(0, tq);
        }
    }
}

pub struct AnimPose {
    pose: Vec<(usize, [f32; 3])>,
    mask: String,
    global: [f32; 4],
    regions: [[f32; 4]; 5],
    over: [bool; 3],
    connected: [bool; 4],
    plane: Vec3,
    frozen: bool,
    /// RDR2 core: the message read with RDR2's names and defaults (rdr2_animpose.md section 2)
    rdr2: AnimPoseRdr2,
    r2_on: bool,
}

/// RDR2's animPose parameters. Limbs in RDR2's order: left arm, right arm, spine, left leg, right leg, left ankle,
/// right ankle; gravComp for the first five; connected hands and feet: 1 on, 0 off, -1 by contact, -2 untouched.
struct AnimPoseRdr2 {
    global: [f32; 4],
    mask: String,
    over: [bool; 3],
    limbs: [[f32; 3]; 7],
    grav: [f32; 5],
    connected: [i32; 4],
    source: i32,
    zmp: bool,
    /// dampenSideMotionInstanceHandle: an obstacle's id, -1 the world (ours), 0 none
    handle: i32,
    /// the instance's place now, from the obstacles
    instance: Option<(Vec3, Quat)>,
    /// the side-motion plane, in the instance's frame: a point and a normal (+0xf0, +0x100)
    plane: Option<(Vec3, Vec3)>,
    /// the incoming pose last step (RDR2's second buffer, ctx+0x3638)
    prev: Vec<bevy_transform::components::Transform>,
    logged: u8,
}

/// RDR2's animPose limb masks, in its parameter order (the spine's stiffness mask takes the neck and head, its
/// gravity mask the spine joints only).
const RDR2_LIMBS: [&str; 7] = ["ul", "ur", "uk", "ll", "lr", "la", "ra"];

fn rdr2_limb(m: &str) -> Vec<usize> {
    match m {
        "la" => vec![(*ANKLE())[0]],
        "ra" => vec![(*ANKLE())[1]],
        _ => Character::mask(m),
    }
}

const REGIONS: [(&str, &str); 5] = [("LeftArm", "ul"), ("RightArm", "ur"), ("Spine", "uk"), ("LeftLeg", "ll"), ("RightLeg", "lr")];

impl AnimPose {
    fn new(p: &mut P) -> Result<AnimPose, Error> {
        let global = [p.f("stiffness", delirium_data::kv!(21))?, p.f("damping", 1.0)?, p.f("muscleStiffness", -1.0)?, p.f("gravityCompensation", 0.0)?];
        let mut regions = [[-1.0; 4]; 5];
        for (r, (name, _)) in regions.iter_mut().zip(REGIONS) {
            for (x, what) in r.iter_mut().zip(["stiffness", "damping", "muscleStiffness", "gravComp"]) {
                *x = p.f(&format!("{what}{name}"), -1.0)?;
            }
        }
        let rdr2 = Self::rdr2(p)?;
        let _ = (p.b("useZMPGravityCompensation", false)?, p.i("animSource", 0)?, p.i("dampenSideMotionInstanceIndex", -1)?);
        let _ = p.v("vDampenPlanePoint", Vec3::ZERO)?;
        Ok(AnimPose {
            rdr2,
            r2_on: false,
            pose: parse_pose(&p.s("pose", "")?),
            mask: p.s("mask", "fb")?,
            global,
            regions,
            over: [p.b("overideHeadlook", false)?, p.b("overidePointArm", false)?, p.b("overidePointGun", false)?],
            connected: [p.i("connectedLeftHand", 0)? != 0, p.i("connectedRightHand", 0)? != 0, p.i("connectedLeftFoot", 0)? != 0,
                        p.i("connectedRightFoot", 0)? != 0],
            plane: p.v("vDampenPlaneNormal", Vec3::ZERO)?.normalize_or_zero(),
            frozen: false,
        })
    }

    fn rdr2(p: &mut P) -> Result<AnimPoseRdr2, Error> {
        let global = [p.f("stiffness", -1.0)?, p.f("damping", 1.0)?, p.f("muscleStiffness", -1.0)?, p.f("gravityCompensation", -1.0)?];
        let mask = match p.s("effectorMask", "")? {
            m if m.is_empty() => p.s("mask", "fb")?,
            m => m,
        };
        let mut limbs = [[-1.0, 1.0, -1.0]; 7];
        for (l, name) in limbs.iter_mut().zip(["LeftArm", "RightArm", "Spine", "LeftLeg", "RightLeg", "LeftAnkle", "RightAnkle"]) {
            *l = [p.f(&format!("stiffness{name}"), -1.0)?, p.f(&format!("damping{name}"), 1.0)?, p.f(&format!("muscleStiffness{name}"), -1.0)?];
        }
        let mut grav = [-1.0; 5];
        for (g, name) in grav.iter_mut().zip(["LeftArm", "RightArm", "Spine", "LeftLeg", "RightLeg"]) {
            *g = p.f(&format!("gravComp{name}"), -1.0)?;
        }
        let zmp = p.b("useZMPGravityCompensation", true)?;
        let handle = p.i("dampenSideMotionInstanceHandle", 0)?;
        Ok(AnimPoseRdr2 {
            global, mask, limbs, grav, zmp, handle, instance: None, plane: None, prev: vec![], logged: 0,
            over: [p.b("overideHeadlook", false)?, p.b("overidePointArm", false)?, p.b("overidePointGun", false)?],
            connected: [p.i("connectedLeftHand", 0)?, p.i("connectedRightHand", 0)?, p.i("connectedLeftFoot", -2)?, p.i("connectedRightFoot", -2)?],
            source: p.i("animSource", 0)?,
        })
    }

    /// RDR2's tick (rdr2_animpose.md section 4, biped): the side-motion spring; the global stiffness on effectorMask (if
    /// >= 0); the joints of effectorMask driven to the incoming animation (animSource 0 now, 1 last step's; RDR2's limb
    /// handler tests each joint's bit, and writes nothing without the buffer [C]); the global gravity opposition on
    /// effectorMask (if >= 0); each limb's own stiffness on its fixed mask (even outside effectorMask); a connected hand
    /// or foot (1, or -1 with its end part touching something other than himself) gets its wrist or ankle at (16, 1,
    /// muscle 12); each limb's own gravity opposition. The overrides make headLook and pointArm yield (nm.rs); animPose
    /// drives those joints; nothing in the NM code reads overidePointGun [C, by scan]. Ours: animSource 2 has no buffer here;
    /// the instance is an obstacle by id, or -1 for the world. The ZMP gravity-compensation flag and the connected hands
    /// and feet go to the core's gravity opposition (nmdbm.rs `oppose_gravity`).
    fn tick_rdr2(&mut self, c: &mut Ctx) {
        self.r2_on = true;
        let r = &mut self.rdr2;
        let say = |r: &mut AnimPoseRdr2, bit: u8, s: String| {
            if crate::extras_ysk_rdr2::trace() && r.logged & bit == 0 {
                r.logged |= bit;
                eprintln!("animPose: {s}");
            }
        };
        if let Some((at, rot)) = r.instance {
            // the plane through his COM and side axis when it started, carried by the instance
            let (point, normal) = *r.plane.get_or_insert_with(|| (rot.inverse() * (c.com - at), (rot.inverse() * c.m.a).normalize_or_zero()));
            let n = (rot * normal).normalize_or_zero();
            let d = (c.com - (at + rot * point)).dot(n);
            if d.abs() > 0.1 {
                let f = n * (-300.0 * d);
                for k in crate::extras_ysk_rdr2::parts(0x102709) {
                    c.out.force(k, f);
                }
                say(r, 1, format!("side motion spring, off the plane {d:.3} m, {:.0} N a part", f.length()));
            }
        }
        let mine = Character::mask(&r.mask);
        let [s, d, ms, g] = r.global;
        if s >= 0.0 {
            for &j in &mine {
                c.ch.eff[j].set_stiffness(s, d, (ms >= 0.0).then_some(ms));
            }
        }
        if (0..2).contains(&r.source) {
            let buffer: &[bevy_transform::components::Transform] = if r.source == 0 { c.incoming } else { &r.prev };
            if !buffer.is_empty() {
                let m = crate::nmrs::incoming_rows(&c.b.doll.links, buffer);
                for &j in &mine {
                    c.ch.eff[j].desired = c.ch.incoming_angle(j, &m);
                }
                say(r, 2, format!("{} joints from animation buffer {}", mine.len(), r.source));
            }
        }
        r.prev = c.incoming.to_vec();
        if g >= -1e-5 {
            c.ch.core.zmp = r.zmp;
            for &j in &mine {
                c.ch.eff[j].oppose_gravity = g;
            }
        }
        for i in [0, 1, 3, 4, 5, 6, 2] {
            let [s, d, ms] = r.limbs[i];
            if s >= 0.0 {
                for j in rdr2_limb(RDR2_LIMBS[i]) {
                    c.ch.eff[j].set_stiffness(s, d, (ms >= 0.0).then_some(ms));
                }
            }
        }
        let connected = r.connected;
        for (i, &v) in connected.iter().enumerate() {
            // limbs 0..3 are the left and right arm, then the left and right leg: the hand or foot is the last part.
            // -1: bit 2 and (4 or 0x10) of the part's contact flags, touching something other than himself
            let l = crate::nmshot::limb(i);
            if v == 1 || (v == -1 && c.b.doll.hit_out[l.p[3]].is_some()) {
                c.ch.eff[l.e[3]].set_stiffness(16.0, 1.0, Some(12.0));
                say(r, if v == 1 { 4 } else { 8 }, format!("limb {i} connected, value {v}: its end joint at 16"));
            }
        }
        if r.grav.iter().any(|&g| g >= -1e-4) {
            c.ch.core.zmp = r.zmp;
        }
        // the connected flags: -1 takes the hand or foot from contact at the next gravity pass, 0 / 1 set it
        // (a hand stays so after), anything else leaves it (a foot is from contact every step anyway)
        for (i, &v) in r.connected.iter().enumerate() {
            match (i, v) {
                (0 | 1, -1) => c.ch.core.hands_auto[i] = true,
                (0 | 1, 0 | 1) => (c.ch.core.hands[i], c.ch.core.hands_auto[i]) = (v == 1, false),
                (_, 0 | 1) => c.ch.core.feet[i - 2] = Some(v == 1),
                _ => {}
            }
        }
        for (i, m) in ["ul", "ur", "us", "ll", "lr"].into_iter().enumerate() {
            if r.grav[i] >= -1e-5 {
                for j in Character::mask(m) {
                    c.ch.eff[j].oppose_gravity = r.grav[i];
                }
            }
        }
    }
}

impl Extra for AnimPose {
    fn objects(&mut self, obs: &[crate::ragdoll::Obstacle]) {
        let h = self.rdr2.handle;
        self.rdr2.instance = match h {
            0 => None,
            -1 => Some((Vec3::ZERO, Quat::IDENTITY)),
            _ => obs.iter().find(|o| o.id == h).map(|o| (o.at, o.rot)),
        };
    }

    fn stop(&mut self, c: &mut Ctx) {
        // RDR2's deactivate: gravity opposition off on effectorMask, and the ZMP flag back on
        if self.r2_on {
            for j in Character::mask(&self.rdr2.mask) {
                c.ch.eff[j].oppose_gravity = 0.0;
            }
            c.ch.core.zmp = true;
        }
    }

    fn anim_pose_yield(&self) -> Option<[bool; 3]> {
        let r = &self.rdr2;
        let mine = Character::mask(&r.mask);
        let owns = |m: Vec<usize>| m.iter().any(|j| mine.contains(j));
        self.r2_on.then(|| [r.over[0] && owns((*NECK()).to_vec()), r.over[1] && owns(Character::mask("ul")), r.over[1] && owns(Character::mask("ur"))])
    }

    fn tick(&mut self, c: &mut Ctx, _dt: f32) {
        if c.ch.rdr2 {
            return self.tick_rdr2(c);
        }
        let mine = Character::mask(&self.mask);
        if !self.frozen {
            self.frozen = true;
            if self.pose.is_empty() {
                self.pose = mine.iter().map(|&j| (j, c.ch.actual(j, c.b.frames))).collect();
            }
        }
        let mut skip = vec![];
        for (o, joints) in self.over.iter().zip([&(*NECK())[..], &Character::mask("ul"), &Character::mask("ur")]) {
            if *o {
                skip.extend_from_slice(joints);
            }
        }
        let connected = [self.connected[0], self.connected[1], false, self.connected[2], self.connected[delirium_data::kv!(0 int)]];
        for (i, (_, m)) in REGIONS.iter().enumerate() {
            let v: Vec<f32> = (0..delirium_data::kv!(1 int)).map(|k| if self.regions[i][k] < 0.0 { self.global[k] } else { self.regions[i][k] }).collect();
            let s = if connected[i] { v[0] * 0.5 } else { v[0] };
            for j in Character::mask(m) {
                if skip.contains(&j) || !mine.contains(&j) {
                    continue;
                }
                let ms = if (*WRIST()).contains(&j) || (*ANKLE()).contains(&j) { Some(delirium_data::kv!(139)) } else { (v[2] >= 0.0).then_some(v[2]) };
                c.ch.eff[j].set_stiffness(s, v[1], ms);
                c.ch.eff[j].oppose_gravity = v[delirium_data::kv!(0 int)];
            }
        }
        for &(j, a) in &self.pose {
            if !skip.contains(&j) {
                c.ch.eff[j].desired = a;
            }
        }
        if self.plane != Vec3::ZERO {
            for k in 0..delirium_data::kv!(111 int) {
                let v = c.b.part(k).v.dot(self.plane);
                c.out.force(k, self.plane * (-v * c.b.doll.part_mass(k) * delirium_data::kv!(31)));
            }
        }
    }
}

#[derive(Clone, Copy)]
struct Circle { normal: Vec3, centre: Vec3, part: usize, r1: f32, r2: f32, speed: f32 }

pub struct ArmsWindmill {
    arm: [Circle; 2],
    deltas: [f32; 5],
    gains: [f32; 4],
    phase: f32,
    twist: f32,
    spin: (f32, f32),
    elbow_min: [f32; 2],
    mirror: i32,
    adaptive: bool,
    use_arm: [bool; 2],
    disable_on_impact: bool,
    angle: [f32; 2],
    hit: [bool; 2],
    /// the RDR2 core's parameters (RDR2's defaults) and state (nmwm.rs)
    rdr2: crate::nmwm::Params,
    r2: Option<crate::nmwm::Windmill>,
}

impl ArmsWindmill {
    fn new(p: &mut P) -> Result<ArmsWindmill, Error> {
        let mut circle = |side: &str, y: f32| -> Result<Circle, Error> {
            Ok(Circle { normal: p.v(&format!("{side}Normal"), Vec3::new(0.0, y, 0.0))?.normalize_or(Vec3::Y),
                        centre: p.v(&format!("{side}Centre"), Vec3::new(delirium_data::kv!(9), delirium_data::kv!(17) * y, delirium_data::kv!(68)))?,
                        part: p.i(&format!("{side}PartID"), CHEST() as i32)?.clamp(0, delirium_data::kv!(127 int)) as usize,
                        r1: p.f(&format!("{side}Radius1"), delirium_data::kv!(10))?, r2: p.f(&format!("{side}Radius2"), delirium_data::kv!(10))?,
                        speed: p.f(&format!("{side}Speed"), 1.0)? })
        };
        let arm = [circle("left", 1.0)?, circle("right", -1.0)?];
        let rdr2 = Self::rdr2(p)?;
        let _ = p.f("dragReduction", 0.0)?;
        let force_sync = p.b("forceSync", false)?;
        let phase = p.f("phaseOffset", std::f32::consts::PI)?;
        Ok(ArmsWindmill {
            arm,
            deltas: [p.f("radius1Delta", 0.0)?, p.f("radius2Delta", 0.0)?, p.f("speedDelta", 0.0)?, p.f("shoulderStiffnessDelta", 0.0)?,
                     p.f("elbowStiffnessDelta", 0.0)?],
            gains: [p.f("shoulderStiffness", delirium_data::kv!(21))?, p.f("shoulderDamping", 1.0)?, p.f("elbowStiffness", delirium_data::kv!(21))?, p.f("elbowDamping", 1.0)?],
            phase: if force_sync { 0.0 } else { phase },
            twist: p.f("IKtwist", 0.0)?,
            spin: (p.f("angVelThreshold", delirium_data::kv!(78))?, p.f("angVelGain", 1.0)?),
            elbow_min: [p.f("leftElbowMin", delirium_data::kv!(24))?, p.f("rightElbowMin", delirium_data::kv!(24))?],
            mirror: p.i("mirrorMode", 0)?,
            adaptive: p.b("adaptiveMode", false)?,
            use_arm: [p.b("useLeft", true)?, p.b("useRight", true)?],
            disable_on_impact: p.b("disableOnImpact", true)?,
            angle: [0.0; 2],
            hit: [false; 2],
            rdr2,
            r2: None,
        })
    }

    /// The message read with RDR2's names and defaults (rdr2_windmill.md A.2).
    fn rdr2(p: &mut P) -> Result<crate::nmwm::Params, Error> {
        let d = crate::nmwm::Params::default();
        let side = |p: &mut P, s: &str, i: usize| -> Result<(Vec3, Vec3, [f32; 2], f32), Error> {
            Ok((p.v(&format!("{s}Normal"), d.normal[i])?, p.v(&format!("{s}Centre"), d.centre[i])?,
                [p.f(&format!("{s}Radius1"), d.radius[i][0])?, p.f(&format!("{s}Radius2"), d.radius[i][1])?], p.f(&format!("{s}Speed"), d.speed[i])?))
        };
        let (l, r) = (side(p, "left", 0)?, side(p, "right", 1)?);
        Ok(crate::nmwm::Params {
            normal: [l.0, r.0], centre: [l.1, r.1], radius: [l.2, r.2], speed: [l.3, r.3],
            shoulder: (p.f("shoulderStiffness", d.shoulder.0)?, p.f("shoulderDamping", d.shoulder.1)?),
            elbow: (p.f("elbowStiffness", d.elbow.0)?, p.f("elbowDamping", d.elbow.1)?),
            phase_offset: p.f("phaseOffset", d.phase_offset)?, drag_reduction: p.f("dragReduction", d.drag_reduction)?,
            ik_twist: p.f("IKtwist", d.ik_twist)?, ang_vel_threshold: p.f("angVelThreshold", d.ang_vel_threshold)?,
            ang_vel_gain: p.f("angVelGain", d.ang_vel_gain)?,
            elbow_min: [p.f("leftElbowMin", d.elbow_min[0])?, p.f("rightElbowMin", d.elbow_min[1])?],
            mirror_mode: p.i("mirrorMode", d.mirror_mode)?, adaptive_mode: p.i("adaptiveMode", d.adaptive_mode)?,
            force_sync: p.b("forceSync", d.force_sync)?, use_arm: [p.b("useLeft", true)?, p.b("useRight", true)?],
            disable_on_impact: p.b("disableOnImpact", d.disable_on_impact)?,
        })
    }
}

impl Extra for ArmsWindmill {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        // RDR2 core: RDR2's armsWindmill (nmwm.rs), started on the first tick
        if c.ch.rdr2 {
            let p = self.rdr2;
            let w = self.r2.get_or_insert_with(|| crate::nmwm::Windmill::start(c, &p));
            w.tick(c, &p, dt);
            return;
        }
        self.r2 = None;
        let over = if self.adaptive { (c.angvel.length() - self.spin.0).max(0.0) } else { 0.0 };
        if over > 0.0 {
            for a in &mut self.arm {
                (a.r1, a.r2, a.speed) = (a.r1 + self.deltas[0] * dt, a.r2 + self.deltas[1] * dt, a.speed + self.deltas[2] * dt);
            }
            self.gains[0] += self.deltas[delirium_data::kv!(0 int)] * dt;
            self.gains[2] += self.deltas[delirium_data::kv!(1 int)] * dt;
        }
        let fwd = facing(c);
        let up = c.ch.up.normalize_or(Vec3::Z);
        let left = up.cross(fwd);
        let world = |v: Vec3| fwd * v.x + left * v.y + up * v.z;
        for i in 0..2 {
            if self.disable_on_impact && c.b.doll.collided(&[(*HANDS())[i]]) {
                self.hit[i] = true;
            }
            if !self.use_arm[i] || self.hit[i] {
                continue;
            }
            let a = self.arm[i];
            let dir = if i == 1 && self.mirror == 1 { -1.0 } else { 1.0 };
            self.angle[i] += dir * a.speed * (1.0 + over * self.spin.1) * std::f32::consts::TAU * dt;
            let th = self.angle[i] - if i == 1 { self.phase } else { 0.0 };
            let n = world(a.normal).normalize_or(left);
            let u = n.cross(if n.dot(up).abs() > delirium_data::kv!(28) { fwd } else { up }).normalize();
            let v = n.cross(u);
            let target = pos(c, a.part) + world(a.centre) + u * (a.r1 * th.cos()) + v * (a.r2 * th.sin());
            let l = crate::nmshot::limb(i);
            c.ch.eff[l.e[1]].set_stiffness(self.gains[0], self.gains[1], None);
            c.ch.eff[l.e[2]].set_stiffness(self.gains[2], self.gains[delirium_data::kv!(0 int)], None);
            crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[delirium_data::kv!(0 int)], target, self.twist, 0.0, 1.0 - 2.0 * i as f32, 1.0,
                                  Vec3::ZERO, None);
            let e = &mut c.ch.eff[l.e[2]].desired[0];
            *e = e.max(self.elbow_min[i]);
        }
    }
}

pub struct Twist {
    pos: Vec3,
    offset: f32,
    vel: Vec3,
    excess: f32,
    k: (f32, f32),
    clav: bool,
    always: bool,
    twisting: Option<bool>,
    out: Vec<String>,
    /// RDR2 core: RDR2's spineTwist (nmst.rs), started on the first tick
    r2: Option<crate::nmst::SpineTwist>,
}

impl Twist {
    fn new(p: &mut P) -> Result<Twist, Error> {
        Ok(Twist { pos: p.v("pos", Vec3::ZERO)?, offset: p.f("offset", 0.0)?,
                   vel: Vec3::new(p.f("velX", 0.0)?, p.f("velY", 0.0)?, p.f("velZ", 0.0)?), excess: p.f("spineTwistExcess", delirium_data::kv!(140))?,
                   k: (p.f("spStrength", delirium_data::kv!(21))?, p.f("spDamping", 2.0)?), clav: p.b("twistClavicles", true)?,
                   always: p.b("allwaysTwist", false)?, twisting: None, out: vec![], r2: None })
    }

    /// RDR2 core: RDR2's spineTwist has no message (other behaviours drive it), so this one drives it with the
    /// message's target, its velocity (led, not moved: RDR2 moves no target), offset, twistClavicles and
    /// allwaysTwist. RDR2's limit excess is a fixed 0.35 and it sets no stiffness, so spineTwistExcess, spStrength
    /// and spDamping are not used.
    fn tick_rdr2(&mut self, c: &mut Ctx) {
        let (pos, vel, offset, arms, always) = (self.pos, self.vel, self.offset, self.clav, self.always);
        let st = self.r2.get_or_insert_with(Default::default);
        (st.pos, st.vel, st.offset, st.arms, st.always) = (pos, vel, offset, arms, always);
        st.activate(c.ch);
        st.tick(c.ch, c.b);
        let on = st.twisting;
        if self.twisting != Some(on) {
            self.twisting = Some(on);
            self.out.push(format!("#say spineTwist {}", if on { "twisting" } else { "stopped" }));
        }
    }
}

impl Extra for Twist {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if self.pos == Vec3::ZERO {
            self.pos = pos(c, CHEST()) + Vec3::Z.cross(facing(c)) * delirium_data::kv!(12) - facing(c) * 0.5;
        }
        if c.ch.rdr2 {
            return self.tick_rdr2(c);
        }
        self.pos += self.vel * dt;
        let p = c.b.part(0);
        let n = (self.pos + (self.vel - p.v) * delirium_data::kv!(78) - p.m.d).normalize_or_zero();
        let twist = p.m.b.dot(n).atan2(-p.m.c.dot(n)) + self.offset;
        let on = self.always || twist.abs() < delirium_data::kv!(20) * std::f32::consts::PI;
        if self.twisting != Some(on) {
            self.twisting = Some(on);
            self.out.push(format!("#say spineTwist {}", if on { "twisting" } else { "stopped" }));
        }
        if !on {
            return;
        }
        c.ch.set_stiffness(self.k.0, self.k.1, "us", None);
        let range = (*SPINE()).map(|j| {
            let l = c.ch.limits(j);
            (l[delirium_data::kv!(1 int)] * (1.0 + self.excess), l[delirium_data::kv!(2 int)] * (1.0 + self.excess))
        });
        let sum = range.iter().fold((0.0, 0.0), |s, r| (s.0 + r.0, s.1 + r.1));
        let up = (pos(c, CHEST()) - pos(c, 0)).normalize_or_zero().z.abs();
        let x = (up * twist).clamp(sum.0, sum.1);
        for (k, j) in (*SPINE()).into_iter().enumerate() {
            c.ch.eff[j].desired[2] = if x >= 0.0 { range[k].1 / sum.1 } else { range[k].0 / sum.0 } * x;
        }
        if self.clav {
            c.ch.eff[(*CLAVICLE())[1]].desired[0] = x;
            c.ch.eff[(*CLAVICLE())[0]].desired[0] = -x;
        }
    }

    fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.out)
    }
}

pub struct Wave { side: usize, time: f32, rate: f32, size: f32, t: f32, out: Vec<String> }

impl Wave {
    fn new(p: &mut P) -> Result<Wave, Error> {
        Ok(Wave { side: p.b("right", true)? as usize, time: p.f("time", delirium_data::kv!(141))?, rate: p.f("rate", 2.0)?, size: p.f("size", delirium_data::kv!(142))?,
                  t: 0.0, out: vec![] })
    }
}

#[allow(non_snake_case)]
#[inline]
fn WAVE_UP() -> f32 { delirium_data::kv!(24) }
#[allow(non_snake_case)]
#[inline]
fn WAVE_DOWN() -> f32 { delirium_data::kv!(24) }

impl Extra for Wave {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if self.t == 0.0 {
            self.out.push("#say wave Hi!".into());
        }
        self.t += dt;
        let i = self.side;
        let l = crate::nmshot::limb(i);
        let (f, left) = (facing(c), Vec3::Z.cross(facing(c)));
        let out = if i == 0 { left } else { -left };
        let shoulder = pos(c, l.p[0]);
        let to = if self.t < WAVE_UP() + self.time {
            let sway = (self.t * self.rate * std::f32::consts::TAU).sin() * self.size * ((self.t / WAVE_UP()).min(1.0));
            shoulder + out * (delirium_data::kv!(140) + sway) + Vec3::Z * delirium_data::kv!(10) * (self.t / WAVE_UP()).min(1.0) + f * delirium_data::kv!(68)
        } else {
            shoulder + out * delirium_data::kv!(8)
        };
        c.ch.set_stiffness(delirium_data::kv!(21), 1.0, ["ul", "ur"][i], None);
        crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[delirium_data::kv!(0 int)], to, 0.0, 0.0, 1.0 - 2.0 * i as f32, 1.0, Vec3::ZERO, None);
    }

    fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.out)
    }

    fn done(&self) -> bool {
        self.t > WAVE_UP() + self.time + WAVE_DOWN()
    }
}

pub struct Breathe { s: f32, joints: Vec<usize>, scale: f32, t: f32, last: f32, wrote: Vec<f32> }

#[allow(non_snake_case)]
#[inline]
fn BREATH() -> f32 { delirium_data::kv!(84) }
#[allow(non_snake_case)]
#[inline]
fn BREATH_LEAN() -> f32 { delirium_data::kv!(131) }

impl Breathe {
    fn new(p: &mut P) -> Result<Breathe, Error> {
        Ok(Breathe { s: p.f("stiffness", delirium_data::kv!(31))?, joints: Character::mask(&p.s("effectorMask", "us")?), scale: p.f("breathingScale", 1.0)?,
                     t: 0.0, last: 0.0, wrote: vec![] })
    }
}

impl Extra for Breathe {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        self.t += dt;
        let lean = -BREATH_LEAN() * self.scale * (0.5 - 0.5 * (self.t * std::f32::consts::TAU / BREATH()).cos());
        self.wrote.resize(self.joints.len(), f32::NAN);
        for (&j, w) in self.joints.iter().zip(&mut self.wrote) {
            c.ch.eff[j].set_stiffness(self.s, 1.0, None);
            let d = &mut c.ch.eff[j].desired[0];
            *d += if *d == *w { lean - self.last } else { lean };
            *w = *d;
        }
        c.ch.eff[(*NECK())[0]].set_stiffness(self.s * delirium_data::kv!(28), 1.0, None);
        self.last = lean;
    }
}

pub struct SoftKeyframe { follow: f32, most: f32, slower: bool, yaw: f32, target: Vec3, parts: Vec<usize>, pose: Vec<Vec3>,
                          r2: Option<Box<crate::extras_ysk_rdr2::KeyframeRdr2>> }

impl SoftKeyframe {
    fn new(p: &mut P) -> Result<SoftKeyframe, Error> {
        let (follow, most, slower, yaw, target) = (p.f("followMultiplier", delirium_data::kv!(31))?, p.f("maxAcceleration", delirium_data::kv!(25))?,
                                                   p.b("goSlowerWhenWeaker", true)?, p.f("yawOffset", 0.0)?, p.v("targetPosition", Vec3::ZERO)?);
        let mask = p.s("mask", "fb")?;
        let parts = Character::mask(&mask).into_iter().map(|j| j + 1).collect();
        // RDR2 core: RDR2's keyframe soft keying of the mask, maxAcceleration its clamp when given
        let r2 = Some(Box::new(crate::extras_ysk_rdr2::KeyframeRdr2::soft(&mask, p.get("maxAcceleration").is_some().then_some(most))));
        Ok(SoftKeyframe { follow, most, slower, yaw, target, parts, pose: vec![], r2 })
    }
}

impl Extra for SoftKeyframe {
    fn stop(&mut self, c: &mut Ctx) {
        if let Some(k) = self.r2.as_mut() {
            k.stop(c);
        }
    }

    fn tick(&mut self, c: &mut Ctx, _dt: f32) {
        if c.ch.rdr2 && let Some(k) = self.r2.as_mut() {
            return k.tick(c, _dt);
        }
        let pelvis = pos(c, 0);
        if self.pose.is_empty() {
            self.pose = self.parts.iter().map(|&k| pos(c, k) - pelvis).collect();
        }
        let (anchor, v0) = if self.target == Vec3::ZERO { (pelvis, c.b.part(0).v) } else { (self.target, Vec3::ZERO) };
        let turn = bevy_math::Quat::from_rotation_z(self.yaw);
        for (&k, &off) in self.parts.iter().zip(&self.pose) {
            let gap = anchor + turn * off - pos(c, k);
            let mut a = (gap * self.follow - (c.b.part(k).v - v0) * 2.0 * self.follow.max(0.0).sqrt()).clamp_length_max(self.most);
            if self.slower {
                a *= (c.ch.eff[k - 1].strength.max(0.0).sqrt() / delirium_data::kv!(21)).min(1.0);
            }
            c.out.force(k, a * c.b.doll.part_mass(k));
        }
    }
}

fn aim(c: &mut Ctx, i: usize, at: Vec3, s: f32, d: f32) {
    let shoulder = pos(c, crate::nmshot::limb(i).p[0]);
    let dir = if at == Vec3::ZERO { facing(c) } else { (at - shoulder).normalize_or(facing(c)) };
    reach(c, i, shoulder + dir * delirium_data::kv!(132), s, d);
}

fn reach(c: &mut Ctx, i: usize, at: Vec3, s: f32, d: f32) {
    let l = crate::nmshot::limb(i);
    c.ch.set_stiffness(s, d, ["ul", "ur"][i], None);
    crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[delirium_data::kv!(0 int)], at, 0.0, 0.0, 1.0 - 2.0 * i as f32, 1.0, Vec3::ZERO, None);
}

fn v3(p: Vec3) -> String {
    format!("{},{},{}", p.x, p.y, p.z)
}

pub struct SplitBody {
    aim: Vec3,
    pose_mask: Vec<usize>,
    hard: Vec<usize>,
    soft: SoftKeyframe,
    grav: Vec<usize>,
    s: (f32, f32),
    ctrl: (f32, f32),
    gun: Option<usize>,
    support: Option<usize>,
    constrain: bool,
    orient: bool,
    limits: bool,
    stable: Option<(f32, f32)>,
    recoil: Option<(f32, f32)>,
    breathe: Breathe,
    pose: Vec<(usize, [f32; 3])>,
    hold: Vec<(usize, Vec3, Quat)>,
    turn: Quat,
    relax: f32,
    kick: bool,
    cons: Vec<(usize, Option<Vec3>, Option<Quat>)>,
    start: Vec<Vec3>,
    t: f32,
}

impl SplitBody {
    fn new(p: &mut P) -> Result<SplitBody, Error> {
        let aim = p.v("aimTarget", Vec3::ZERO)?;
        let pose_mask = Character::mask(&p.s("activePoseMask", "uk")?);
        let parts = |m: String| Character::mask(&m).into_iter().map(|j| j + 1).collect::<Vec<_>>();
        let hard = if p.s("hardConstraintMask", "")?.is_empty() { vec![] } else { parts(p.s("hardConstraintMask", "")?) };
        let soft_parts = if p.s("softConstraintMask", "")?.is_empty() { vec![] } else { parts(p.s("softConstraintMask", "")?) };
        let _ = p.s("collisionMask", "fb")?;
        let grav = Character::mask(&p.s("gravityCompensationMask", "ub")?);
        let (s, ctrl) = ((p.f("stiffness", delirium_data::kv!(21))?, p.f("damping", 1.0)?), (p.f("controllerStiffness", delirium_data::kv!(126))?, p.f("controllerDamping", 1.0)?));
        let weapon = [p.i("leftHandWeaponIndex", -1)?, p.i("rightHandWeaponIndex", 0)?];
        let status = [p.i("leftHandStatus", 2)?, p.i("rightHandStatus", 1)?];
        let gun = (0..2).find(|&i| status[i] == 1 && weapon[i] >= 0);
        let support = gun.and_then(|g| (status[1 - g] == 2).then_some(1 - g));
        let (constrain, orient, limits) = (p.b("constrainHands", true)?, p.b("controlHandOrientation", true)?, p.b("adjustJointLimits", true)?);
        let stable = p.b("handStable", false)?.then_some(()).map(|()| (0.0, 0.0));
        let stable_g = (p.f("handStableStiff", delirium_data::kv!(126))?, p.f("handStableDamp", 1.0)?);
        let recoil = p.b("recoilRelax", true)?;
        let recoil_g = (p.f("recoilRelaxAmount", 0.5)?, p.f("recoilRelaxTime", delirium_data::kv!(24))?);
        let breathe = Breathe { s: s.0, joints: Character::mask("us"), scale: p.f("breathingScale", 1.0)?, t: 0.0, last: 0.0, wrote: vec![] };
        let _ = p.b("fireWeapon", false)?;
        Ok(SplitBody { aim, pose_mask, hard, soft: SoftKeyframe { follow: delirium_data::kv!(31), most: delirium_data::kv!(25), slower: true, yaw: 0.0, target: Vec3::ZERO, parts: soft_parts, pose: vec![], r2: None },
                       grav, s, ctrl, gun, support, constrain, orient, limits, stable: stable.map(|_| stable_g), recoil: recoil.then_some(recoil_g),
                       breathe, pose: vec![], hold: vec![], turn: Quat::IDENTITY, relax: 0.0, kick: false, cons: vec![], start: vec![], t: 0.0 })
    }

    fn hand(i: usize) -> usize {
        crate::nmshot::limb(i).p[delirium_data::kv!(0 int)]
    }
}

#[allow(non_snake_case)]
#[inline]
fn GRIP_ON() -> f32 { delirium_data::kv!(116) }
#[allow(non_snake_case)]
#[inline]
fn GRIP_W() -> f32 { delirium_data::kv!(38) }
#[allow(non_snake_case)]
#[inline]
fn GRIP_MOST() -> f32 { delirium_data::kv!(15) }

impl Extra for SplitBody {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        let f = c.b.frames;
        if self.pose.is_empty() {
            self.pose = self.pose_mask.iter().map(|&j| (j, c.ch.actual(j, f))).collect();
            self.hold = self.hard.iter().map(|&k| (k, f[k].translation, f[k].rotation)).collect();
            if let (Some(g), Some(h)) = (self.gun, self.support) {
                self.turn = f[Self::hand(g)].rotation.inverse() * f[Self::hand(h)].rotation;
            }
            self.start = (0..2).map(|i| f[Self::hand(i)].translation).collect();
        }
        self.t += dt;
        let give = match self.recoil {
            Some((amount, time)) if self.relax > 0.0 => 1.0 - amount * (self.relax / time.max(delirium_data::kv!(134))),
            _ => 1.0,
        };
        self.relax = (self.relax - dt).max(0.0);
        for &(j, a) in &self.pose {
            c.ch.eff[j].set_stiffness(self.s.0 * give, self.s.1, None);
            c.ch.eff[j].desired = a;
            c.ch.eff[j].oppose_gravity = if self.grav.contains(&j) { 1.0 } else { 0.0 };
        }
        self.breathe.tick(c, dt);
        self.cons = self.hold.iter().map(|&(k, x, q)| (k, Some(x), Some(q))).collect();
        self.soft.tick(c, dt);
        let Some(g) = self.gun else { return };
        aim(c, g, self.aim, self.ctrl.0 * give, self.ctrl.1);
        if self.limits {
            for e in (*ELBOW()) {
                let at = c.ch.eff[e].desired[0];
                let mut l = crate::nmrs::SoftLimit { stiffness: delirium_data::kv!(15), damping: 1.0, enabled: true, ..Default::default() };
                l.set_limit(at + delirium_data::kv!(9) * at.signum(), at.signum(), false);
                c.ch.eff[e].soft[0] = l;
            }
        }
        if let Some((s, d)) = self.stable {
            c.ch.eff[(*WRIST())[g]].set_stiffness(s * give, d, None);
        }
        let gh = Self::hand(g);
        if std::mem::take(&mut self.kick) {
            let back = -facing(c);
            c.out.force(gh, (Vec3::Z + back).normalize() * (2.0 / dt));
        }
        if let (Some(h), true) = (self.support, self.constrain) {
            let across = Vec3::Z.cross(facing(c)) * if h == 0 { 1.0 } else { -1.0 };
            let grip = f[gh].translation + across * delirium_data::kv!(112) - Vec3::Z * delirium_data::kv!(131);
            let on = (self.t / GRIP_ON()).min(1.0);
            let hand = Self::hand(h);
            let to = self.start[h].lerp(grip, on);
            reach(c, h, to, self.ctrl.0 * give, self.ctrl.1);
            let (m, w) = (c.b.doll.part_mass(hand), GRIP_W());
            let (a, b) = (c.b.part(hand), c.b.part(gh));
            let pull = ((to - a.m.d) * (m * w * w) - (a.v - b.v) * (2.0 * m * w)).clamp_length_max(GRIP_MOST());
            c.out.force(hand, pull);
            c.out.force(gh, -pull);
            if self.orient {
                let to = f[hand].rotation.slerp(f[gh].rotation * self.turn, on);
                let i = m * delirium_data::kv!(143);
                let twist = (short_turn(to * f[hand].rotation.inverse()) * (i * w * w) - (a.w - b.w) * (2.0 * i * w)).clamp_length_max(GRIP_MOST() * delirium_data::kv!(68));
                c.out.torque(hand, twist);
                c.out.torque(gh, -twist);
            }
        }
    }

    fn constraints(&self) -> Vec<(usize, Option<Vec3>, Option<Quat>)> {
        self.cons.clone()
    }

    fn fire(&mut self) {
        self.kick = true;
        if let Some((_, time)) = self.recoil {
            self.relax = time;
        }
    }

    fn stop(&mut self, c: &mut Ctx) {
        if self.limits {
            for e in (*ELBOW()) {
                c.ch.eff[e].soft[0].set_enabled(false);
            }
        }
    }
}

pub struct ShootDodge {
    targets: [Vec3; 2],
    valid: bool,
    ends: (f32, f32, f32),
    reaction: i32,
    trunk: (f32, f32),
    legs: (f32, f32, f32),
    aiming: (f32, f32),
    loose: (f32, f32, f32),
    head: (f32, f32),
    wrist_ms: f32,
    ori: (f32, f32, f32),
    noise: f32,
    breathe: Breathe,
    state: u8,
    t: f32,
    touching: f32,
    last_vel: Option<Vec3>,
    attitude: Quat,
    leg_pose: Vec<(usize, [f32; 3])>,
    done: bool,
    out: Vec<String>,
}

const SD_ENTER: u8 = 0;
const SD_HOLD: u8 = 1;

impl ShootDodge {
    fn new(p: &mut P) -> Result<ShootDodge, Error> {
        let targets = [p.v("leftArmTarget", Vec3::ZERO)?, p.v("rightArmTarget", Vec3::ZERO)?];
        let valid = p.b("targetValid", true)?;
        for n in ["collisionPoint", "collisionNormal", "collisionObjectVelocity"] {
            p.v(n, Vec3::ZERO)?;
        }
        let _ = (p.i("collisionIndex", -1)?, p.f("collisionTime", 0.0)?, p.f("supportedHeightThreshold", 0.0)?);
        let _ = (p.b("armAnimTask", false)?, p.i("leftArmAnimTaskParent", -1)?, p.i("rightArmAnimTaskParent", -1)?);
        let ends = (p.f("collisionTimeThreshold", delirium_data::kv!(60))?, p.f("collisionAccelThreshold", delirium_data::kv!(11))?, p.f("collisionVelThreshold", 2.0)?);
        Ok(ShootDodge {
            targets, valid, ends, reaction: p.i("collisionReaction", 0)?,
            trunk: (p.f("trunkStiffness", delirium_data::kv!(21))?, p.f("trunkDamping", 1.0)?),
            legs: (p.f("legStiffness", delirium_data::kv!(31))?, p.f("legDamping", 1.0)?, p.f("legTaper", delirium_data::kv!(20))?),
            aiming: (p.f("aimingArmStiffness", delirium_data::kv!(126))?, p.f("aimingArmDamping", 1.0)?),
            loose: (p.f("notAimingArmStiffness", delirium_data::kv!(35))?, p.f("notAimingArmDamping", delirium_data::kv!(74))?, p.f("notAimingArmTaper", delirium_data::kv!(74))?),
            head: (p.f("headStiffness", delirium_data::kv!(31))?, p.f("headDamping", 1.0)?),
            wrist_ms: p.f("wristMuscleStiffness", delirium_data::kv!(31))?,
            ori: (p.f("oriStiff", delirium_data::kv!(27))?, p.f("oriDamp", delirium_data::kv!(65))?, p.f("oriSplit", 0.5)?.clamp(0.0, 1.0)),
            noise: p.f("legNoiseScale", delirium_data::kv!(68))?,
            breathe: Breathe { s: delirium_data::kv!(21), joints: Character::mask("us"), scale: p.f("breathingScale", 1.0)?, t: 0.0, last: 0.0, wrote: vec![] },
            state: if p.i("state", 0)? == 1 { SD_HOLD } else { SD_ENTER },
            t: 0.0, touching: 0.0, last_vel: None, attitude: Quat::IDENTITY, leg_pose: vec![], done: false, out: vec![],
        })
    }

    fn aiming_arm(&self) -> Option<usize> {
        if !self.valid { None } else if self.targets[1] != Vec3::ZERO { Some(1) } else if self.targets[0] != Vec3::ZERO { Some(0) } else { None }
    }
}

impl Extra for ShootDodge {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if self.done {
            return;
        }
        self.t += dt;
        let f = c.b.frames;
        if self.state == SD_ENTER || self.leg_pose.is_empty() {
            self.attitude = f[0].rotation;
            self.leg_pose = Character::mask("lb").into_iter().map(|j| (j, c.ch.actual(j, f))).collect();
            let look = self.aiming_arm().map_or(pos(c, delirium_data::kv!(55 int)) + facing(c) * delirium_data::kv!(64), |i| self.targets[i]);
            self.out.push(format!("headLook start=true pos={},{},{} stiffness={} damping={}", look.x, look.y, look.z, self.head.0, self.head.1));
            self.out.push("#say shootDodge hold".into());
            self.state = SD_HOLD;
        }
        c.ch.set_stiffness(self.trunk.0, self.trunk.1, "us", None);
        c.ch.set_stiffness(self.head.0, self.head.1, "un", None);
        let (ls, ld, lt) = self.legs;
        for (k, (j, a)) in self.leg_pose.iter().enumerate() {
            let down = [(*HIP()), (*KNEE()), (*ANKLE())].iter().position(|set| set.contains(j)).unwrap_or(0) as i32;
            c.ch.eff[*j].set_stiffness(ls * lt.powi(down), ld, None);
            let drift = self.noise * (self.t * delirium_data::kv!(66) + k as f32 * delirium_data::kv!(144)).sin();
            c.ch.eff[*j].desired = [a[0] + drift, a[1], a[2]];
        }
        let arm = self.aiming_arm();
        for i in 0..2 {
            if Some(i) == arm {
                aim(c, i, self.targets[i], self.aiming.0, self.aiming.1);
                c.ch.eff[(*WRIST())[i]].muscle_stiffness = self.wrist_ms;
            } else {
                let (s, d, taper) = self.loose;
                for (n, &j) in [(*CLAVICLE())[i], (*SHOULDER())[i], (*ELBOW())[i], (*WRIST())[i]].iter().enumerate() {
                    c.ch.eff[j].set_stiffness(s * taper.powi(n.saturating_sub(1) as i32), d, None);
                }
            }
        }
        for (k, share) in [(0, self.ori.2), (delirium_data::kv!(81 int), 1.0 - self.ori.2)] {
            let err = short_turn(self.attitude * f[0].rotation.inverse());
            let t = (err * self.ori.0 - c.b.part(0).w * self.ori.1) * (c.b.doll.part_mass(k) * delirium_data::kv!(78) * share);
            c.out.torque(k, t);
        }
        self.breathe.tick(c, dt);
        let touching = c.b.doll.collided(&(0..delirium_data::kv!(111 int)).collect::<Vec<_>>());
        self.touching = if touching { self.touching + dt } else { 0.0 };
        let (accel, lost) = self.last_vel.map_or((0.0, 0.0), |v: Vec3| ((c.vel - v).length() / dt, v.length() - c.vel.length()));
        self.last_vel = Some(c.vel);
        if touching && (accel > self.ends.1 || lost > self.ends.2 || self.touching > self.ends.0) {
            self.out.push("headLook start=false".into());
            match self.reaction {
                0 => self.out.push("catchFall start=true".into()),
                1 => self.out.push("bodyRollUp start=true".into()),
                2 => self.out.push("bodyBalance start=true".into()),
                _ => {}
            }
            self.out.push("#say shootDodge end".into());
            self.done = true;
        }
    }

    fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.out)
    }

    fn done(&self) -> bool {
        self.done
    }
}

pub struct ArmHang {
    anchor: Vec3,
    target: Vec3,
    inst: (i32, i32),
    frame: Option<(Vec3, Quat)>,
    local: Option<[Vec3; 2]>,
    start: Vec<Vec3>,
    across: Vec3,
    t: f32,
    cons: Vec<(usize, Option<Vec3>, Option<Quat>)>,
    out: Vec<String>,
    free: [bool; 2],
}

const HANG_ON: f32 = 0.5;

impl ArmHang {
    fn new(p: &mut P) -> Result<ArmHang, Error> {
        let (anchor, target) = (p.v("constraintPosition", Vec3::ZERO)?, p.v("targetPosition", Vec3::ZERO)?);
        let inst = (p.i("instanceIndex", -1)?, p.i("boundIndex", 0)?);
        Ok(ArmHang { anchor, target, inst, frame: None, local: None, start: vec![], across: Vec3::X, t: 0.0, cons: vec![], out: vec![], free: [false; 2] })
    }
}

impl Extra for ArmHang {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if self.start.is_empty() {
            self.start = (*HANDS()).iter().map(|&k| pos(c, k)).collect();
            self.across = Vec3::Z.cross(facing(c));
            if self.anchor == Vec3::ZERO {
                self.anchor = (self.start[0] + self.start[1]) * 0.5;
            }
            let to = v3(if self.target == Vec3::ZERO { self.anchor } else { self.target });
            self.out.extend(["bodyBalance start=false".into(), format!("pointArm start=true useLeftArm=false target={to}"),
                             format!("headLook start=true pos={to} stiffness={} damping=1", delirium_data::kv!(31)), format!("spineTwist start=true pos={to}"),
                             "pedalLegs start=true".into()]);
        }
        self.t += dt;
        let on = (self.t / HANG_ON).min(1.0);
        let mut pins = [0, 1].map(|i| self.anchor + self.across * (delirium_data::kv!(9) - delirium_data::kv!(24) * i as f32));
        if let Some((at, q)) = self.frame {
            let local = *self.local.get_or_insert_with(|| pins.map(|p| q.inverse() * (p - at)));
            pins = local.map(|l| at + q * l);
            (self.anchor, self.across) = ((pins[0] + pins[1]) * 0.5, (pins[0] - pins[1]).normalize_or(self.across));
        }
        self.cons = (0..2).filter(|&i| !self.free[i]).map(|i| ((*HANDS())[i], Some(self.start[i].lerp(pins[i], on)), None)).collect();
        for i in (0..2).filter(|&i| !self.free[i]) {
            reach(c, i, self.start[i].lerp(pins[i], on), delirium_data::kv!(65), delirium_data::kv!(20));
        }
        c.ch.set_stiffness(delirium_data::kv!(5), delirium_data::kv!(24), "us", None);
    }

    fn constraints(&self) -> Vec<(usize, Option<Vec3>, Option<Quat>)> {
        self.cons.clone()
    }

    fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.out)
    }

    fn let_go(&mut self, hand: usize) {
        self.free[hand] = true;
    }

    fn done(&self) -> bool {
        self.free == [true; 2]
    }

    fn stop(&mut self, _c: &mut Ctx) {
        self.out.extend(["pointArm", "headLook", "spineTwist", "pedalLegs"].map(|n| format!("{n} start=false")));
    }

    fn objects(&mut self, obs: &[crate::ragdoll::Obstacle]) {
        if let Some(o) = crate::ragdoll::Obstacle::find(obs, self.inst.0) {
            self.frame = Some(match self.inst.1 {
                k @ 1.. => { let (at, m) = o.component(k as usize); (at, Quat::from_mat3(&m)) }
                _ => (o.at, o.rot),
            });
        }
    }
}

pub struct CrushReaction {
    mask: Vec<usize>,
    s: (f32, f32),
    mode: i32,
    id: i32,
    ob: Option<crate::ragdoll::Obstacle>,
    use_hurt: bool,
    obstacle: Vec3,
    started: bool,
    fell: bool,
    out: Vec<String>,
}

impl CrushReaction {
    fn new(p: &mut P) -> Result<CrushReaction, Error> {
        let mask = Character::mask(&p.s("effectorMask", "ub")?);
        let s = (p.f("stiffness", delirium_data::kv!(21))?, p.f("damping", 1.0)?);
        let (id, use_hurt) = (p.i("obstacleID", -1)?, p.b("useInjuries", false)?);
        Ok(CrushReaction { mask, s, mode: p.i("flinchMode", 0)?, id, ob: None, use_hurt, obstacle: p.v("obstaclePosition", Vec3::ZERO)?,
                           started: false, fell: false, out: vec![] })
    }
}

impl Extra for CrushReaction {
    fn tick(&mut self, c: &mut Ctx, _dt: f32) {
        let head = pos(c, (*NECK())[1] + 1);
        if let Some(o) = self.ob {
            let local = o.rot.inverse() * (pos(c, CHEST()) - o.centre);
            self.obstacle = o.centre + o.rot * local.clamp(-o.half, o.half);
        }
        if !self.started {
            self.started = true;
            if self.obstacle == Vec3::ZERO {
                self.obstacle = head + Vec3::Z;
            }
            let look = if self.mode == 1 { self.obstacle } else { pos(c, CHEST()) + facing(c) - Vec3::Z * delirium_data::kv!(12) };
            self.out.extend(["bodyBalance start=true".into(),
                             format!("upperBodyFlinch start=true useLeftArm=false useRightArm=false pos={}", v3(self.obstacle)),
                             format!("headLook start=true pos={} stiffness={} damping={}", v3(look), self.s.0, self.s.1)]);
        }
        for &j in &self.mask {
            c.ch.eff[j].set_stiffness(self.s.0, self.s.1, None);
        }
        let across = Vec3::Z.cross(facing(c));
        for i in 0..2 {
            let arm = [(*CLAVICLE())[i], (*SHOULDER())[i], (*ELBOW())[i], (*WRIST())[i]];
            if (self.use_hurt && arm.iter().any(|&j| c.ch.eff[j].injury > 0.0)) || !arm.iter().all(|j| self.mask.contains(j)) {
                continue;
            }
            let at = if self.mode == 1 {
                let shoulder = pos(c, crate::nmshot::limb(i).p[0]);
                let gap = self.obstacle - shoulder;
                shoulder + gap.normalize_or(Vec3::Z) * gap.length().min(delirium_data::kv!(119))
            } else {
                head + across * (delirium_data::kv!(125) - delirium_data::kv!(145) * i as f32) + Vec3::Z * delirium_data::kv!(78) + facing(c) * delirium_data::kv!(146)
            };
            reach(c, i, at, self.s.0, self.s.1);
        }
        if c.dbm_failed && !self.fell {
            self.fell = true;
            self.out.push("catchFall start=true".into());
        }
    }

    fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.out)
    }

    fn stop(&mut self, _c: &mut Ctx) {
        self.out.extend(["upperBodyFlinch", "headLook", "bodyBalance"].map(|n| format!("{n} start=false")));
        if self.fell {
            self.out.push("catchFall start=false".into());
        }
    }

    fn objects(&mut self, obs: &[crate::ragdoll::Obstacle]) {
        self.ob = crate::ragdoll::Obstacle::find(obs, self.id).copied();
    }
}

fn short_turn(q: Quat) -> Vec3 {
    (if q.w < 0.0 { -q } else { q }).to_scaled_axis()
}

