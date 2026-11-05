//! Behaviours from Red Dead Redemption 2's Euphoria, written from descriptions of how they behave (see the RDR2
//! Euphoria notes); none of RDR2's code, so what each does is Delirium's reading of it, not RDR2's own. Most parameter
//! names are RDR2's; some are Delirium's own, where the behaviour needs a setting RDR2's table doesn't show (rdr2Sit's
//! seat and stirrups, wallCollision's wall, keyframe's hold, the injured limbs' looseness and the like). A default is
//! RDR2's own value (as RDR2's descriptor table gives it) wherever the parameter means the same here: times in seconds,
//! distances, shares 0..1, bends and rates. Where RDR2's value is on a scale of its own (its forces, torques and
//! "amount" multipliers: shotFromBehind's and shotInGuts' amounts, shotSpin's turn and torque, shotInjuredArm's hipYaw,
//! reachForWound's armStiffness, wallCollision's snapMag), the default is Delirium's, in the units its doc gives.
//! rdr2Bolas and setFallingReaction's hands-and-knees settings are Delirium's own throughout. Any of them can be sent
//! as parameters. They are extra messages: nothing here runs unless one is sent with start=true, so every existing
//! behaviour is unchanged.
use crate::extras::{Extra, P};
use crate::msg::{Error, Kind, Message, Param, Value};
use crate::nmdbm_rdr2::Crouch;
use crate::nmdbm::LeanMode;
use crate::nmrs::{ANKLE, ELBOW, HIP, KNEE, NECK, SPINE};
use crate::ragdoll::GRAVITY;
use crate::nmshot::Ctx;
use bevy_math::Vec3;

pub static MESSAGES: [Message; 28] = [
    Message { name: "applyBulletImpulse", params: &[
        Param { name: "partIndex", kind: Kind::Int, used: true, doc: "the part hit (0)" },
        Param { name: "impulse", kind: Kind::Vec3, used: true, doc: "the bullet's impulse, N s, world (zero)" },
        Param { name: "hitPoint", kind: Kind::Vec3, used: true, doc: "where it hit, world (unset: the part's centre)" },
        Param { name: "equalizeAmount", kind: Kind::Float, used: true, doc: "0: the impulse as given, 1: scaled by the part's mass over the mean part's (RDR2's shot sends 0) (0)" },
        Param { name: "extraShare", kind: Kind::Float, used: true, doc: "a share also given to the trunk (Spine0): less than 0 adds |share| of it there on top, 0..1 splits it between the trunk and the part hit, more than 1 none. RDR2's shot sends minus COMImpulseScale on trunk hits (0)" },
    ] },
    Message { name: "rdr2Sit", params: &[
        Param { name: "seatPos", kind: Kind::Vec3, used: true, doc: "where his pelvis sits, world; send rdr2Sit again without start to move it with the horse (zero: where his pelvis is)" },
        Param { name: "seatUp", kind: Kind::Vec3, used: true, doc: "the seat's up, world (0,0,1). RDR2 gives the seat as a part of its horse with seatuplocal and seatrightlocal in that part's frame" },
        Param { name: "seatForward", kind: Kind::Vec3, used: true, doc: "which way the horse faces, world (zero: his facing)" },
        Param { name: "stirrupL", kind: Kind::Vec3, used: true, doc: "where his left foot goes, world (zero: 0.55 m under the seat, 0.26 m to the side)" },
        Param { name: "stirrupR", kind: Kind::Vec3, used: true, doc: "his right foot (zero: likewise)" },
        Param { name: "mountTime", kind: Kind::Float, used: true, doc: "s he takes to get from where he is into the seat, swung up over it (0.6). RDR2 core: Delirium's, the game's mounting before RDR2's sit starts (RDR2 has none: 0)" },
        Param { name: "constrain", kind: Kind::Int, used: true, doc: "without the core: non-zero holds his pelvis in the seat (1; 0: only his pose, he sits on whatever's under him). RDR2 core: 0 none, 1 his pelvis held where it is (the world), 2 his pelvis and shins held to the seat (RDR2's 0)" },
        Param { name: "constrainstrength", kind: Kind::Float, used: true, doc: "how hard his hips are held square to the seat (40)" },
        Param { name: "constrainstrengthstirrup", kind: Kind::Float, used: true, doc: "how hard his feet are held in the stirrups: a spring of 10 times this, N per m (40; 0: they aren't)" },
        Param { name: "hippitchstart", kind: Kind::Float, used: true, doc: "his hips tip back (less than 0) or forward, -1..1 of half a radian (-0.5)" },
        Param { name: "hippitchramp", kind: Kind::Float, used: true, doc: "s they take to get there (0.25)" },
        Param { name: "straightnessstart", kind: Kind::Float, used: true, doc: "his back slouches (less than 0) or arches, -1..1 (-0.1)" },
        Param { name: "straightnessramp", kind: Kind::Float, used: true, doc: "s it takes (0.35)" },
        Param { name: "failangle", kind: Kind::Float, used: true, doc: "he's thrown once his body leans this far from the seat's up, a cosine (0.71)" },
        Param { name: "seatfailangle", kind: Kind::Float, used: true, doc: "or the seat tips this far from the sky's up (0.71)" },
        Param { name: "softweight", kind: Kind::Float, used: true, doc: "how hard his chest is held upright in the seat, as RDR2 holds riders to their animation, 0..1 (0.8; slumped: none)" },
        Param { name: "Slump", kind: Kind::Bool, used: true, doc: "he sits slumped, his back and arms loose, the reins let go; leaning doesn't throw him (false)" },
        Param { name: "drag", kind: Kind::Bool, used: true, doc: "thrown, a foot stays caught in its stirrup and he's dragged by it (false)" },
        Param { name: "dragTime", kind: Kind::Float, used: true, doc: "s until the foot comes free (10)" },
        Param { name: "dragreach", kind: Kind::Bool, used: true, doc: "dragged, his arms reach back toward the caught foot (true)" },
        Param { name: "dragreachdelay", kind: Kind::Float, used: true, doc: "s after he's thrown that he starts reaching (0.2)" },
        Param { name: "dragstiffness", kind: Kind::Float, used: true, doc: "his stiffness while dragged (9)" },
        Param { name: "knockSpeed", kind: Kind::Float, used: true, doc: "a hit that moves his chest this fast against his hips knocks him off, m/s (4; 0: none)" },
        Param { name: "throwSpeed", kind: Kind::Float, used: true, doc: "thrown, how hard he's flung off to the side he leans to, and up, m/s (3)" },
        Param { name: "throw", kind: Kind::Bool, used: true, doc: "sent without start: throw him now, as if he'd failed (false)" },
        Param { name: "posefailangle", kind: Kind::Float, used: true, doc: "RDR2 core: he fails once his trunk is this far from the animation's, rad (0.71; no incoming animation: never)" },
        Param { name: "standfailangle", kind: Kind::Float, used: true, doc: "RDR2 core: failing with his shins or feet on the ground, he lands on his feet (bodyBalance) if his COM is above them by at least this cosine (0.71)" },
        Param { name: "rolldelay", kind: Kind::Float, used: true, doc: "RDR2 core: s a dead rider waits before he rolls off (0.3)" },
        Param { name: "rollspeedthreshold", kind: Kind::Float, used: true, doc: "RDR2 core: a dead rider rolls off only if he moves slower than this against the seat, m/s (2)" },
        Param { name: "healthfaildelay", kind: Kind::Float, used: true, doc: "RDR2 core: s his health is out before it counts, with healthFailNow off (0)" },
        Param { name: "health", kind: Kind::Float, used: true, doc: "RDR2 core, Delirium's: his health as RDR2's NM reads it (its character strength); 0 or less fails him; may be sent again without start (1)" },
        Param { name: "healthFailNow", kind: Kind::Bool, used: true, doc: "RDR2 core, RDR2's uncracked cd51e744: health fails him at once, healthfaildelay not waited for (true)" },
        Param { name: "slump", kind: Kind::Bool, used: true, doc: "RDR2 core: a dead rider on a stopped horse folds over the saddle (false; Slump sets it too)" },
        Param { name: "fallingReaction", kind: Kind::Int, used: true, doc: "RDR2 core: falling off, 0 bodyRollUp, 1 catchFall, 2 rollDownStairs, 3 smartFall, other none (0)" },
        Param { name: "dragreaction", kind: Kind::Int, used: true, doc: "RDR2 core: dragged, 0 bodyRollUp and pedalLegs, 1 yanked on the caught foot, other bodyFoetal (1)" },
        Param { name: "neverreleaseconstraint", kind: Kind::Bool, used: true, doc: "RDR2 core: the seat's constraints never break and no fail lets them go (false)" },
        Param { name: "constraintoworld", kind: Kind::Bool, used: true, doc: "RDR2 core: constrain 2's anchors in the world, moved with the seat (false; Delirium's are always so, the horse being no body his solver holds him to)" },
        Param { name: "pelvisLengthStart", kind: Kind::Float, used: true, doc: "RDR2 core, RDR2's uncracked dface7f3: the pelvis constraint's slack at the start, m (0.2)" },
        Param { name: "pelvisLength", kind: Kind::Float, used: true, doc: "RDR2 core, uncracked 7eb6c0a8: its slack once tightened, m (0.05)" },
        Param { name: "pelvisRate", kind: Kind::Float, used: true, doc: "RDR2 core, uncracked 66b2fa7e: how fast it tightens, m/s (0.1)" },
        Param { name: "orientLengthStart", kind: Kind::Float, used: true, doc: "RDR2 core, uncracked ffb7714b: how far the pelvis may turn from the seat at the start, rad (0.2)" },
        Param { name: "orientLength", kind: Kind::Float, used: true, doc: "RDR2 core, uncracked 4356388b: once tightened, rad (0)" },
        Param { name: "orientRate", kind: Kind::Float, used: true, doc: "RDR2 core, uncracked 7a822fad: rad/s (0.1)" },
        Param { name: "stirrupLengthStart", kind: Kind::Float, used: true, doc: "RDR2 core, uncracked 1d0ffcf0: the stirrups' slack at the start, m (0.2)" },
        Param { name: "stirrupLength", kind: Kind::Float, used: true, doc: "RDR2 core, uncracked 47195a12: once tightened, m (0.05)" },
        Param { name: "stirrupRate", kind: Kind::Float, used: true, doc: "RDR2 core, uncracked 1d24def6: m/s (0.1)" },
        Param { name: "dragSlack", kind: Kind::Float, used: true, doc: "RDR2 core, uncracked bf94ac99: extra stirrup slack while dragged, m (0)" },
        Param { name: "seatBoxSide", kind: Kind::Float, used: true, doc: "RDR2 core, uncracked 002d1ca5: how far his pelvis may be from the seat sideways and still be in it, m (0.3)" },
        Param { name: "seatBoxLength", kind: Kind::Float, used: true, doc: "RDR2 core, uncracked 95df888f: and along the horse, m (0.75)" },
        Param { name: "rdsforcemagstart", kind: Kind::Float, used: true, doc: "RDR2 core: rolling off, rollDownStairs' force at first (0.5)" },
        Param { name: "rdsforcemagramp", kind: Kind::Float, used: true, doc: "RDR2 core: s it ramps over (0.05)" },
        Param { name: "rdsForceMag", kind: Kind::Float, used: true, doc: "RDR2 core: its force after the ramp, and fallingReaction 2's (2)" },
        Param { name: "rollDir", kind: Kind::Vec3, used: true, doc: "RDR2 core, uncracked e60ff12e: fallingReaction 2's rollDownStairs customRollDir (0,0,1)" },
        Param { name: "useRollDir", kind: Kind::Bool, used: true, doc: "RDR2 core, uncracked d51b7ce2: rollDownStairs' useCustomRollDir; rolling off, roll to the seat's left (false: the side his COM goes)" },
        Param { name: "rdsMinAngVel", kind: Kind::Float, used: true, doc: "RDR2 core, uncracked ff905c00: rollDownStairs' minAngVel (0.3)" },
        Param { name: "dragReachChance", kind: Kind::Float, used: true, doc: "RDR2 core, uncracked 4005b366: dragged, he reaches for the caught foot while a noise is under this (0.5)" },
        Param { name: "hardKeyMask", kind: Kind::Str, used: true, doc: "RDR2 core: with constrain 0, parts keyframed to the riding animation (keyframe) (na)" },
        Param { name: "softKeyMask", kind: Kind::Str, used: true, doc: "RDR2 core: soft-keyed at softweight (na)" },
        Param { name: "softKeyMaskAlt", kind: Kind::Str, used: true, doc: "RDR2 core: soft-keyed at softweightalt (na)" },
        Param { name: "softweightalt", kind: Kind::Float, used: true, doc: "RDR2 core: 0..1 (0.8)" },
        Param { name: "spinefollowanims", kind: Kind::Bool, used: true, doc: "RDR2 core: his spine leans toward the animation's trunk (true; no incoming animation: toward the sky)" },
        Param { name: "seatuplocal", kind: Kind::Vec3, used: false, doc: "RDR2's seat up in the seat part's frame ((0,-1,0); here seatUp, world)" },
        Param { name: "seatrightlocal", kind: Kind::Vec3, used: false, doc: "RDR2's ((1,0,0); here seatForward, world)" },
        Param { name: "activePoseMask", kind: Kind::Str, used: false, doc: "RDR2's: parts driven to the incoming pose (fb)" },
        Param { name: "softbreakthreshold", kind: Kind::Float, used: false, doc: "RDR2's (0.15)" },
        Param { name: "softmaxaccel", kind: Kind::Vec3, used: false, doc: "RDR2's (50,50,50)" },
        Param { name: "softmaxangaccel", kind: Kind::Vec3, used: false, doc: "RDR2's (50,50,50)" },
        Param { name: "stateridingtarget", kind: Kind::Bool, used: false, doc: "RDR2's (false)" },
        Param { name: "animSource", kind: Kind::Int, used: false, doc: "RDR2's (0)" },
        Param { name: "constraintsfollowanims", kind: Kind::Bool, used: false, doc: "RDR2's (false)" },
        Param { name: "pelvisradiusscale", kind: Kind::Float, used: false, doc: "RDR2's (1)" },
        Param { name: "thighradiusscale", kind: Kind::Float, used: false, doc: "RDR2's (1)" },
    ] },
    Message { name: "keyframe", params: &[
        Param { name: "blendTime", kind: Kind::Float, used: true, doc: "s to blend into the neutral stand (0.3)" },
        Param { name: "holdTime", kind: Kind::Float, used: true, doc: "s it then holds it (0.5)" },
        Param { name: "settle", kind: Kind::Float, used: true, doc: "how hard the upper body's motion is damped meanwhile, 1/s (3)" },
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "spine and neck stiffness; the arms 0.8 of it (11)" },
        Param { name: "animSourceCurrent", kind: Kind::Int, used: true, doc: "RDR2 core: the incoming animation buffer (Delirium has one: the agent's incoming pose, or the pose at the start) (0)" },
        Param { name: "animSourcePrevious", kind: Kind::Int, used: true, doc: "RDR2 core: the previous buffer (here last step's pose) (1)" },
        Param { name: "activePoseMask", kind: Kind::Str, used: true, doc: "RDR2 core: joints given the pose's angles (na)" },
        Param { name: "muscleKeyMask", kind: Kind::Str, used: true, doc: "RDR2 core, RDR2's uncracked 00751d32: joints held at the pose's angles, 15 stiff (na)" },
        Param { name: "hardKeyMask", kind: Kind::Str, used: true, doc: "RDR2 core: parts the game keyframes; here held on the pose by hard constraints (fb)" },
        Param { name: "softKeyMask", kind: Kind::Str, used: true, doc: "RDR2 core: parts velocity-tracked to the pose at softweight (na)" },
        Param { name: "softKeyMaskAlt", kind: Kind::Str, used: true, doc: "RDR2 core: at softweightalt (na)" },
        Param { name: "softweight", kind: Kind::Float, used: true, doc: "RDR2 core: 0..1 (1)" },
        Param { name: "softweightalt", kind: Kind::Float, used: true, doc: "RDR2 core: 0..1 (1)" },
        Param { name: "softweightrate", kind: Kind::Float, used: true, doc: "RDR2 core, RDR2's uncracked 3387d79d: softweight's blend-in rate, 1/s (1)" },
        Param { name: "softweightaltrate", kind: Kind::Float, used: true, doc: "RDR2 core, RDR2's uncracked 3a9e42b6 (1)" },
        Param { name: "softweightinitial", kind: Kind::Float, used: true, doc: "RDR2 core: the blend-in's start (0)" },
        Param { name: "softweightaltinitial", kind: Kind::Float, used: true, doc: "RDR2 core (0)" },
        Param { name: "blendinsoftweight", kind: Kind::Bool, used: true, doc: "RDR2 core: blend softweight in (false)" },
        Param { name: "blendinsoftweightalt", kind: Kind::Bool, used: true, doc: "RDR2 core (false)" },
        Param { name: "softmaxaccel", kind: Kind::Vec3, used: true, doc: "RDR2 core: the tracking's acceleration clamp per axis, m/s^2 at weight 1 (50,50,50)" },
        Param { name: "softmaxangaccel", kind: Kind::Vec3, used: true, doc: "RDR2 core: the angular one, in the COM frame (50,50,50)" },
        Param { name: "softgravity", kind: Kind::Float, used: true, doc: "RDR2 core, RDR2's uncracked cae6fd66: the share of gravity the tracking takes up (0)" },
        Param { name: "softbreakthreshold", kind: Kind::Float, used: true, doc: "RDR2 core: mean tracking error, m, past which soft keying stops with keyframe failure (-1: never)" },
        Param { name: "softpreservemomentum", kind: Kind::Bool, used: true, doc: "RDR2 core (false)" },
        Param { name: "useForces", kind: Kind::Bool, used: true, doc: "RDR2 core: forces, with the pose's own velocity (false)" },
        Param { name: "usetargetvel", kind: Kind::Bool, used: true, doc: "RDR2 core: add the pose's own velocity (false)" },
        Param { name: "spikes", kind: Kind::Int, used: true, doc: "RDR2 core: steps of large angular acceleration allowed (0)" },
        Param { name: "spikecooldown", kind: Kind::Int, used: true, doc: "RDR2 core, RDR2's uncracked e87ea3df: steps after them (3)" },
        Param { name: "spikeangaccel", kind: Kind::Vec3, used: true, doc: "RDR2 core: the clamp during a spike (1000,1000,1000)" },
    ] },
    Message { name: "rdr2Balance", params: &[
        Param { name: "runningThresh", kind: Kind::Float, used: true, doc: "his speed along the ground above which he balances as running, m/s (0.65)" },
        Param { name: "legStiffnessRun", kind: Kind::Float, used: true, doc: "hips and knees at least this stiff when running (13; 0: off)" },
        Param { name: "startFlat", kind: Kind::Float, used: true, doc: "a foot this little above the other (m) is turned fully flat (0.1)" },
        Param { name: "endFlat", kind: Kind::Float, used: true, doc: "from this high above the other (m) it isn't (0.3)" },
        Param { name: "flatStiffness", kind: Kind::Float, used: true, doc: "the turn's strength, N m per rad (12; 0: off)" },
    ] },
    Message { name: "setFallingReaction", params: &[
        Param { name: "handsAndKnees", kind: Kind::Bool, used: true, doc: "end a forward fall on all fours (true; false: nothing happens)" },
        Param { name: "hkHoldTime", kind: Kind::Float, used: true, doc: "s he stays on hands and knees (1.5)" },
        Param { name: "stoppedStrengthDecay", kind: Kind::Float, used: true, doc: "then his strength fades by this per s (1)" },
        Param { name: "minArmStiffness", kind: Kind::Float, used: true, doc: "arms' stiffness on all fours (14)" },
        Param { name: "minLegStiffness", kind: Kind::Float, used: true, doc: "legs' stiffness (12)" },
        Param { name: "minSpineStiffness", kind: Kind::Float, used: true, doc: "spine and neck stiffness (9)" },
        Param { name: "hkHeadAvoid", kind: Kind::Bool, used: true, doc: "keep the head up off the ground (true)" },
        Param { name: "pushForward", kind: Kind::Float, used: true, doc: "a push on the chest tipping him forwards first, N (0)" },
    ] },
    Message { name: "wallCollision", params: &[
        Param { name: "objectBehindVictimPos", kind: Kind::Vec3, used: true, doc: "a point on the wall he ran into, world" },
        Param { name: "objectBehindVictimNormal", kind: Kind::Vec3, used: true, doc: "the wall's normal, out of it towards him (zero: nothing happens)" },
        Param { name: "solidWall", kind: Kind::Bool, used: true, doc: "make the wall solid for his body (true; false when the world already has it)" },
        Param { name: "wallStiffness", kind: Kind::Float, used: true, doc: "that wall's spring, 1/s^2 (400)" },
        Param { name: "wallHalfWidth", kind: Kind::Float, used: true, doc: "the wall reaches this far either side of the point, m (0: on and on)" },
        Param { name: "slumpMode", kind: Kind::Int, used: true, doc: "0: slump when his back meets it, 1: always slump, 2: never (0)" },
        Param { name: "braceWall", kind: Kind::Bool, used: true, doc: "facing it: both hands on the wall (true); false: just a rebound" },
        Param { name: "braceDistance", kind: Kind::Float, used: true, doc: "brace when his chest is within this of it, m (2)" },
        Param { name: "braceArmStiffness", kind: Kind::Float, used: true, doc: "the bracing arms' stiffness (12)" },
        Param { name: "reactTime", kind: Kind::Float, used: true, doc: "s the impact weakens his legs (0.3)" },
        Param { name: "impactLegStiffReduction", kind: Kind::Float, used: true, doc: "leg stiffness lost at the impact (3)" },
        Param { name: "snapMag", kind: Kind::Float, used: true, doc: "the snap away from the wall at the impact, as shotSnap's (0.3)" },
        Param { name: "sitStartTime", kind: Kind::Float, used: true, doc: "slumping: s before he lets go of balance (0.3)" },
        Param { name: "sitRate", kind: Kind::Float, used: true, doc: "slumping: how fast his legs give, per s (1)" },
        Param { name: "glanceSpinMag", kind: Kind::Float, used: true, doc: "side on: the spin away from the wall, N m (50)" },
        Param { name: "glanceSpinTime", kind: Kind::Float, used: true, doc: "side on: s that spin decays over (0.5)" },
        Param { name: "timeToForward", kind: Kind::Float, used: true, doc: "s the hands hold the wall before he pushes off it (0.5)" },
        Param { name: "reboundForce", kind: Kind::Float, used: true, doc: "the push off the wall, N (0: none)" },
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "his stiffness (10)" },
    ] },
    Message { name: "reachForWound", params: &[
        Param { name: "hitPoint", kind: Kind::Vec3, used: true, doc: "the wound, world (zero: in front of the part)" },
        Param { name: "hitPart", kind: Kind::Int, used: true, doc: "the part it is on, 0..20 (-1: the belly)" },
        Param { name: "bulletVel", kind: Kind::Vec3, used: true, doc: "the bullet's velocity (RDR2 core: from behind, he reaches for where it came out)" },
        Param { name: "timeBeforeReachForWound", kind: Kind::Float, used: true, doc: "s before reaching (0.2)" },
        Param { name: "grabHoldTime", kind: Kind::Float, used: true, doc: "s the hand holds the wound (2)" },
        Param { name: "reachWithOneHand", kind: Kind::Bool, used: true, doc: "one hand for a trunk wound (true); false: both" },
        Param { name: "allowInjuredThighReach", kind: Kind::Bool, used: true, doc: "reach for a thigh, bending down to it (true)" },
        Param { name: "allowInjuredLowerLegReach", kind: Kind::Bool, used: true, doc: "reach for a shin or foot, bending further (false)" },
        Param { name: "armStiffness", kind: Kind::Float, used: true, doc: "the reaching arm's stiffness (10)" },
        Param { name: "bend2spine", kind: Kind::Float, used: true, doc: "forward bend of the spine for a leg wound, rad (0.7)" },
        Param { name: "bend2hip", kind: Kind::Float, used: true, doc: "hip bend for a leg wound, rad (0.6)" },
        Param { name: "bend2leg", kind: Kind::Float, used: true, doc: "knee bend for a leg wound, rad (0.6)" },
        Param { name: "spineTwistMag", kind: Kind::Float, used: true, doc: "twist of the spine round to a trunk wound, rad (0.2)" },
        Param { name: "bendTimeUp", kind: Kind::Float, used: true, doc: "s the bend comes in over (0.4)" },
        Param { name: "bendTimeDown", kind: Kind::Float, used: true, doc: "s it goes over after (0.5)" },
        Param { name: "spineStiffness", kind: Kind::Float, used: true, doc: "trunk stiffness while bending (12)" },
        Param { name: "reachFalling", kind: Kind::Int, used: true, doc: "RDR2 core: reach while falling, 0 never, 1 always, 2 only for a hit while upright (0)" },
        Param { name: "reachOnFloor", kind: Kind::Int, used: true, doc: "RDR2 core: reach while lying, as reachFalling (0)" },
        Param { name: "reachFallingWithOneHand", kind: Kind::Int, used: true, doc: "RDR2 core: falling, 1 the left hand only, 2 the right only, else both (3)" },
        Param { name: "newHitReset", kind: Kind::Bool, used: true, doc: "RDR2 core: a new bullet stops a reach and starts the wait again (true)" },
        Param { name: "legRateUp", kind: Kind::Float, used: true, doc: "RDR2 core: rate the knee straightens after a leg wound, per s (0.35)" },
        Param { name: "hipRateUp", kind: Kind::Float, used: true, doc: "RDR2 core: rate the hips straighten after, per s (0.25)" },
        Param { name: "spineRateUp", kind: Kind::Float, used: true, doc: "RDR2 core: rate the spine straightens after, rad/s (0.3)" },
        Param { name: "legRateDown", kind: Kind::Float, used: true, doc: "RDR2 core: rate the knee bends down to a leg wound, per s (0.4)" },
        Param { name: "hipRateDown", kind: Kind::Float, used: true, doc: "RDR2 core: rate the hips bend, per s (0.4)" },
        Param { name: "spineRateDown", kind: Kind::Float, used: true, doc: "RDR2 core: rate the spine bends, rad/s (0.4)" },
        Param { name: "bendBeforeReach", kind: Kind::Float, used: true, doc: "RDR2 core: s he bends before the hand goes to a leg wound (0.1)" },
        Param { name: "spineTwistTime", kind: Kind::Float, used: true, doc: "RDR2 core: s the twist holds before it eases off (0.5)" },
        Param { name: "legStiffness", kind: Kind::Float, used: true, doc: "RDR2 core: his legs' stiffness bending for a leg wound on the ground (10)" },
    ] },
    Message { name: "shotFromBehind", params: &[
        Param { name: "sfbSpineAmount", kind: Kind::Float, used: true, doc: "how far the spine arches back, rad over the whole spine (0.5)" },
        Param { name: "sfbNeckAmount", kind: Kind::Float, used: true, doc: "how far the neck goes back, rad (0.3)" },
        Param { name: "sfbHipAmount", kind: Kind::Float, used: true, doc: "how far the hips push forward, rad (0.25)" },
        Param { name: "sfbKneeAmount", kind: Kind::Float, used: true, doc: "how far the knees give, rad (0.15)" },
        Param { name: "sfbPeriod", kind: Kind::Float, used: true, doc: "s the arch swells and goes over (0.7)" },
        Param { name: "sfbArmsOnset", kind: Kind::Float, used: true, doc: "s before the arms go loose (0)" },
        Param { name: "sfbKneesOnset", kind: Kind::Float, used: true, doc: "s before the knees give (0)" },
        Param { name: "sfbNoiseGain", kind: Kind::Float, used: true, doc: "tremble in the arch, 0..2 (0)" },
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "trunk stiffness while arching (10)" },
    ] },
    Message { name: "shotInGuts", params: &[
        Param { name: "sigSpineAmount", kind: Kind::Float, used: true, doc: "how far the spine hunches forward, rad over the whole spine (0.6)" },
        Param { name: "sigNeckAmount", kind: Kind::Float, used: true, doc: "how far the neck bends forward, rad (0.25)" },
        Param { name: "sigKneeAmount", kind: Kind::Float, used: true, doc: "how far the knees bend, rad; the hips flex by half (0.15)" },
        Param { name: "sigPeriod", kind: Kind::Float, used: true, doc: "s he stays folded (1.5)" },
        Param { name: "sigBlend", kind: Kind::Float, used: true, doc: "s it eases off over after (0.5)" },
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "trunk stiffness (10)" },
    ] },
    Message { name: "shotInjuredLeg", params: &[
        Param { name: "bodyPart", kind: Kind::Int, used: true, doc: "the part hit: 1..3 left leg, 4..6 right (-1: leftLeg says)" },
        Param { name: "leftLeg", kind: Kind::Bool, used: true, doc: "the left leg, when bodyPart doesn't say (true)" },
        Param { name: "legLiftTime", kind: Kind::Float, used: true, doc: "s the leg jerks up for (0: none)" },
        Param { name: "legInjuryLift", kind: Kind::Float, used: true, doc: "how far it jerks up: hip flex, rad; the knee bends twice that (0)" },
        Param { name: "legInjuryTime", kind: Kind::Float, used: true, doc: "s the leg stays weak (0.4)" },
        Param { name: "legInjury", kind: Kind::Float, used: true, doc: "share of the leg's strength lost, 0..1 (0.3)" },
        Param { name: "legInjuryHipPitch", kind: Kind::Float, used: true, doc: "hip flex on the weak leg, rad (0)" },
        Param { name: "legInjurySpineBend", kind: Kind::Float, used: true, doc: "forward bend of the spine meanwhile, rad (0.1)" },
        Param { name: "legLimpBend", kind: Kind::Float, used: true, doc: "knee bend on the weak leg, rad (0)" },
        Param { name: "legLean", kind: Kind::Float, used: true, doc: "push sagging him towards the weak leg, N (15)" },
        Param { name: "legSag", kind: Kind::Float, used: true, doc: "how far the balancer sinks him meanwhile, m (0.04)" },
        Param { name: "legBlend", kind: Kind::Float, used: true, doc: "s it fades over after (0.5)" },
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "the leg's stiffness when well (10)" },
    ] },
    Message { name: "shotInjuredArm", params: &[
        Param { name: "bodyPart", kind: Kind::Int, used: true, doc: "the part hit: 13..16 left arm, 17..20 right (-1: leftArm says)" },
        Param { name: "leftArm", kind: Kind::Bool, used: true, doc: "the left arm, when bodyPart doesn't say (true)" },
        Param { name: "bulletDir", kind: Kind::Vec3, used: true, doc: "the bullet's direction (zero: from in front)" },
        Param { name: "injuredArmTime", kind: Kind::Float, used: true, doc: "s the shoulder is knocked round for (0.25)" },
        Param { name: "hipYaw", kind: Kind::Float, used: true, doc: "torque turning the trunk with the bullet, N m (80)" },
        Param { name: "hipRoll", kind: Kind::Float, used: true, doc: "torque dropping the hit shoulder, N m (0: none)" },
        Param { name: "armLoose", kind: Kind::Float, used: true, doc: "share of the arm's stiffness lost, 0..1 (0.7)" },
        Param { name: "armLooseTime", kind: Kind::Float, used: true, doc: "s the arm hangs loose (1)" },
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "the arm's stiffness when well (10)" },
    ] },
    Message { name: "meleeReact", params: &[
        Param { name: "hitPoint", kind: Kind::Vec3, used: true, doc: "where the blow landed, world (zero)" },
        Param { name: "hitNormal", kind: Kind::Vec3, used: true, doc: "the body's surface normal there, outwards; the blow pushes against it (zero: no push or snap)" },
        Param { name: "hitPart", kind: Kind::Int, used: true, doc: "the part hit, 0..20 (-1: none; the RDR2 core: the pelvis)" },
        Param { name: "csLooseness", kind: Kind::Float, used: true, doc: "how loose the upper body goes, 0..1 (0.4)" },
        Param { name: "csPeriod", kind: Kind::Float, used: true, doc: "s it stays loose (0.3)" },
        Param { name: "Lean", kind: Kind::Float, used: true, doc: "push leaning him away from the blow, N (0: none; the RDR2 core: the balancer's lean amount)" },
        Param { name: "leanPeriod", kind: Kind::Float, used: true, doc: "s of that push (1)" },
        Param { name: "Flinch", kind: Kind::Bool, used: true, doc: "flinch: a small forward hunch (true)" },
        Param { name: "flinchPeriod", kind: Kind::Float, used: true, doc: "s of the flinch (0.7)" },
        Param { name: "armsType", kind: Kind::Int, used: true, doc: "0: arms as they are, 1: hands up to guard the face while flinching (1; the RDR2 core: 1 throws the arms out with a jolt, default 0)" },
        Param { name: "snap", kind: Kind::Bool, used: true, doc: "snap the hit part's joints along the blow (true)" },
        Param { name: "snapMag", kind: Kind::Float, used: true, doc: "that snap's size, as shotSnap's (1)" },
        Param { name: "rfw", kind: Kind::Bool, used: true, doc: "reach for the spot hit afterwards (true; the RDR2 core: false)" },
        Param { name: "rfwDelay", kind: Kind::Float, used: true, doc: "s before reaching (0.2)" },
        Param { name: "rfwPeriod", kind: Kind::Float, used: true, doc: "s of reaching (1)" },
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "upper body stiffness (10; not under the RDR2 core, whose meleeReact has none)" },
        Param { name: "hitNew", kind: Kind::Bool, used: true, doc: "RDR2 core: a new blow this step, its clocks start from it (false; RDR2's own meleeReact does nothing until one comes)" },
        Param { name: "hitMag", kind: Kind::Float, used: true, doc: "RDR2 core: the flinch's bend and the balancer's hip and knee gain (1)" },
        Param { name: "hitLocal", kind: Kind::Bool, used: true, doc: "RDR2 core: hitPoint is in the hit part's frame (true, RDR2's default; false: world)" },
        Param { name: "flinchAbortTime", kind: Kind::Float, used: true, doc: "RDR2 core: no flinch when balancerCollisionsReaction's wall is this close in time (0.5)" },
        Param { name: "leanAbortTime", kind: Kind::Float, used: true, doc: "RDR2 core: no Lean when it is this close (0.5)" },
        Param { name: "rfwNotInBcr", kind: Kind::Bool, used: true, doc: "RDR2 core: no reach while balancerCollisionsReaction is in a state other than 0 (false; RDR2's +0xf1, its name not solved)" },
        Param { name: "rfwExcludeCatchFall", kind: Kind::Bool, used: true, doc: "RDR2 core: catchFall leaves the reaching arms alone (false; RDR2's +0xfc, its name not solved)" },
        Param { name: "useHeadLook", kind: Kind::Bool, used: true, doc: "RDR2 core: look at the spot hit while reaching for it (false)" },
    ] },
    Message { name: "shotFallToKnees", params: &[
        Param { name: "ftkBalanceTime", kind: Kind::Float, used: true, doc: "s he keeps balancing before sinking (0.1)" },
        Param { name: "ftkBendRate", kind: Kind::Float, used: true, doc: "how fast knees and hips bend, rad/s (3)" },
        Param { name: "ftkHelperForce", kind: Kind::Float, used: true, doc: "forward push on the pelvis while sinking, N (0)" },
        Param { name: "ftkSpineBend", kind: Kind::Float, used: true, doc: "forward bend of the whole spine while sinking, rad (0.1)" },
        Param { name: "ftkStiffness", kind: Kind::Float, used: true, doc: "legs and spine stiffness (9)" },
        Param { name: "ftkKneelTime", kind: Kind::Float, used: true, doc: "s he stays on his knees (2)" },
        Param { name: "ftkKneelHip", kind: Kind::Float, used: true, doc: "hip bend while kneeling, rad; more leans him forwards (0.3)" },
        Param { name: "ftkLeanHelp", kind: Kind::Float, used: true, doc: "pull of his centre over his knees while kneeling, N per m, damped (1500)" },
        Param { name: "ftkOnKneesArmType", kind: Kind::Int, used: true, doc: "0: arms as they are, 1: arms hang (1)" },
        Param { name: "ftkPitchForwards", kind: Kind::Bool, used: true, doc: "topple forwards (true) or back (false) at the end" },
        Param { name: "ftkPitchForce", kind: Kind::Float, used: true, doc: "push on the chest that tips him over, N (300)" },
        Param { name: "fallToKnees", kind: Kind::Bool, used: true, doc: "RDR2 core: the shot's module on (true here, as sending the message means it; RDR2's default false)" },
        Param { name: "ftkImpactLooseness", kind: Kind::Float, used: true, doc: "RDR2 core: upper body muscle stiffness 1.01 less this when a knee lands (0.5)" },
        Param { name: "ftkImpactLoosenessTime", kind: Kind::Float, used: true, doc: "RDR2 core: for this long, s (0.2)" },
        Param { name: "ftkHipBlend", kind: Kind::Float, used: true, doc: "RDR2 core: how fast the thighs come upright, a blend a tick (0.3)" },
        Param { name: "ftkLungeProb", kind: Kind::Float, used: true, doc: "RDR2 core: chance only the landed knee's thigh comes upright (0)" },
        Param { name: "ftkFricMult", kind: Kind::Float, used: true, doc: "RDR2 core: foot friction lost per tip of the heel up (1)" },
        Param { name: "ftkHipAngleFall", kind: Kind::Float, used: true, doc: "RDR2 core: hip lean falling back, rad (0.5)" },
        Param { name: "ftkPitchForwardsRad", kind: Kind::Float, used: true, doc: "RDR2 core: RDR2's ftkPitchForwards, the hip pitch moving forward, rad; here apart, as Delirium's own is a flag (0.1)" },
        Param { name: "ftkPitchBackwards", kind: Kind::Float, used: true, doc: "RDR2 core: hip pitch unless moving forward, rad (0.1)" },
        Param { name: "ftkFallBelowStab", kind: Kind::Float, used: true, doc: "RDR2 core: folds below this balance quality (0.5)" },
        Param { name: "ftkFallBelowStabPerSecond", kind: Kind::Float, used: true, doc: "RDR2 core: it grows by this per s balancing (0; hash 4aca51a5, the name is ours)" },
        Param { name: "ftkFallBelowStabPerStep", kind: Kind::Float, used: true, doc: "RDR2 core: and per step (0; hash 6c4220c1, the name is ours)" },
        Param { name: "ftkBalanceAbortThreshold", kind: Kind::Float, used: true, doc: "RDR2 core: the balancer's while kneeling (2)" },
        Param { name: "ftkReleaseReachForWound", kind: Kind::Float, used: true, doc: "RDR2 core: s after a knee lands reachForWound stops, below 0 never (-1)" },
        Param { name: "ftkReachForWound", kind: Kind::Bool, used: false, doc: "RDR2 core: nothing reads it (true)" },
        Param { name: "ftkReleasePointGun", kind: Kind::Bool, used: true, doc: "RDR2 core: pointGun stops when a knee lands (false)" },
        Param { name: "ftkFailMustCollide", kind: Kind::Bool, used: true, doc: "RDR2 core: the balancer's while kneeling (true)" },
        Param { name: "ftkStayUpright", kind: Kind::Bool, used: true, doc: "RDR2 core: false, StayUpright goes off when kneeling (true)" },
        Param { name: "ftkKneeSpin", kind: Kind::Bool, used: true, doc: "RDR2 core: the foot yaw hold lets go on his knees (false)" },
        Param { name: "ftkStiffSpine", kind: Kind::Bool, used: true, doc: "RDR2 core: rollDownStairs stiffens the spine (false)" },
        Param { name: "ftkHelperForceOnSpine", kind: Kind::Bool, used: true, doc: "RDR2 core: the help pushes the chest too (true)" },
        Param { name: "ftkAlwaysChangeFall", kind: Kind::Bool, used: true, doc: "RDR2 core: catchFall's kneel flag though the knees are uneven (false)" },
    ] },
    Message { name: "shotSpin", params: &[
        Param { name: "spin", kind: Kind::Float, used: true, doc: "the hit's spin, m (none: from hitPoint and bulletDir: the lever arm about his centre across the bullet)" },
        Param { name: "hitPoint", kind: Kind::Vec3, used: true, doc: "where the bullet hit, world (zero)" },
        Param { name: "bulletDir", kind: Kind::Vec3, used: true, doc: "the bullet's direction (zero)" },
        Param { name: "shotSpinMin", kind: Kind::Float, used: true, doc: "spins smaller than this (m) do nothing (0.02)" },
        Param { name: "shotSpinTime", kind: Kind::Float, used: true, doc: "s the heading swings round over, with the yaw torque on (0.3)" },
        Param { name: "shotSpinSettle", kind: Kind::Float, used: true, doc: "s it then holds the new heading; the feet stay planted, so he eases part way back after (0.3)" },
        Param { name: "shotSpinTwistMult", kind: Kind::Float, used: true, doc: "rad of turn per m of spin (10)" },
        Param { name: "shotSpinTwist", kind: Kind::Float, used: true, doc: "most turn, rad; stops once he has turned this far (1.5)" },
        Param { name: "shotSpinTwistVel", kind: Kind::Float, used: true, doc: "stops while he already turns faster than this, rad/s (6)" },
        Param { name: "shotSpinTorque", kind: Kind::Float, used: true, doc: "yaw torque on pelvis and trunk per rad off the heading, N m (150)" },
        Param { name: "shotSpinTorqueMax", kind: Kind::Float, used: true, doc: "most yaw torque, N m (300)" },
        Param { name: "stepTurnType", kind: Kind::Int, used: true, doc: "after the spin: 0 nothing, 1 step round to face attackerPos (0)" },
        Param { name: "stepTurnWait", kind: Kind::Float, used: true, doc: "s after the spin before the step turn (0.3)" },
        Param { name: "stepTurnTime", kind: Kind::Float, used: true, doc: "s the step turn may take (2)" },
        Param { name: "attackerPos", kind: Kind::Vec3, used: true, doc: "where the shooter is, world (zero)" },
    ] },
    Message { name: "duck", params: &[
        Param { name: "magMin", kind: Kind::Float, used: true, doc: "least share of each joint's remaining range the duck bends into, 0..1 (0.2)" },
        Param { name: "magMax", kind: Kind::Float, used: true, doc: "most share (1); the share is random between the two" },
        Param { name: "lsRate", kind: Kind::Float, used: true, doc: "lower spine's bend rate, rad/s (0.4)" },
        Param { name: "usRate", kind: Kind::Float, used: true, doc: "upper spine's bend rate, rad/s (0.4)" },
        Param { name: "headRate", kind: Kind::Float, used: true, doc: "neck's bend rate, rad/s (0.4)" },
        Param { name: "spineTwistRate", kind: Kind::Float, used: true, doc: "spine's side-lean and twist rate, rad/s (0.5)" },
        Param { name: "neckTwistRate", kind: Kind::Float, used: true, doc: "neck's side-lean and twist rate, rad/s (0.5)" },
        Param { name: "timeBeforeDuck", kind: Kind::Float, used: true, doc: "s before the duck starts (0)" },
        Param { name: "duckTime", kind: Kind::Float, used: true, doc: "s spent bending in; then it unbends (0.5)" },
        Param { name: "blendTime", kind: Kind::Float, used: true, doc: "s over which the duck fades back out after duckTime (0.5)" },
        Param { name: "leanTwistStart", kind: Kind::Bool, used: true, doc: "also lean sideways and twist away (true)" },
        Param { name: "bendForward", kind: Kind::Bool, used: true, doc: "bend forwards (true) or back (false)" },
        Param { name: "duckLeft", kind: Kind::Bool, used: true, doc: "lean and twist to the left (false: right; RDR2 core: true)" },
        Param { name: "crouch", kind: Kind::Float, used: true, doc: "how much the balancer bends the legs while ducking (0.05; 0: no crouch; not the RDR2 core's)" },
        Param { name: "stiffness", kind: Kind::Float, used: true, doc: "spine and neck stiffness while ducking (10; not the RDR2 core's, which sets none)" },
        Param { name: "newHit", kind: Kind::Bool, used: true, doc: "RDR2 core only: duck now (a duck runs only on a new hit; one within duckTime of the last waits for it) (false)" },
        Param { name: "legBendMin", kind: Kind::Float, used: true, doc: "RDR2 core only: the knees' dip, least (0.1)" },
        Param { name: "legBendMax", kind: Kind::Float, used: true, doc: "RDR2 core only: the knees' dip; negative: random between legBendMin and its size (-0.25)" },
        Param { name: "legBendRate", kind: Kind::Float, used: true, doc: "RDR2 core only: the knees' recovery rate (0.2)" },
        Param { name: "hipPitchMin", kind: Kind::Float, used: true, doc: "RDR2 core only: the hips' dip, least (0)" },
        Param { name: "hipPitchMax", kind: Kind::Float, used: true, doc: "RDR2 core only: the hips' dip; negative: random between hipPitchMin and its size (-0.25)" },
        Param { name: "hipPitchRate", kind: Kind::Float, used: true, doc: "RDR2 core only: the hips' recovery rate (0.2)" },
        Param { name: "dontLookTime", kind: Kind::Float, used: true, doc: "RDR2 core only: s the neck stays the duck's before headLook may have it (0)" },
        Param { name: "pointGunType", kind: Kind::Int, used: true, doc: "RDR2 core only: 1, 2: tell pointGun a duck is on (Delirium has no pointGun behaviour) (0)" },
        Param { name: "duckExag", kind: Kind::Bool, used: true, doc: "RDR2 core only: (RDR2 reads it nowhere) (false)" },
    ] },
    Message { name: "shotSnap", params: &[
        Param { name: "snapMag", kind: Kind::Float, used: true, doc: "the kick's size; negative kicks the other way (0.4)" },
        Param { name: "snapDelay", kind: Kind::Float, used: true, doc: "s before the kick (0)" },
        Param { name: "snapDuration", kind: Kind::Float, used: true, doc: "s the kick is spread over; 0: all at once (0)" },
        Param { name: "unSnapInterval", kind: Kind::Float, used: true, doc: "s after the kick before the counter-kick (0.01)" },
        Param { name: "unSnapRatio", kind: Kind::Float, used: true, doc: "the counter-kick's size against the kick's, back the other way; 0: none (0.7)" },
        Param { name: "snapDirectionRandomness", kind: Kind::Float, used: true, doc: "0: about the axis across the bullet, 1: a random axis (0.3)" },
        Param { name: "snapMovingMult", kind: Kind::Float, used: true, doc: "kick times this when moving on his feet (1)" },
        Param { name: "snapBalancingMult", kind: Kind::Float, used: true, doc: "when standing still on his feet (1)" },
        Param { name: "snapAirborneMult", kind: Kind::Float, used: true, doc: "when nothing touches the ground (1)" },
        Param { name: "snapMovingThresh", kind: Kind::Float, used: true, doc: "m/s above which he is moving (1)" },
        Param { name: "snapUseBulletDir", kind: Kind::Bool, used: true, doc: "kick about the axis across the bullet (true); false: about his own side axis" },
        Param { name: "bulletDir", kind: Kind::Vec3, used: true, doc: "the bullet's direction (zero: coming from in front)" },
        Param { name: "snapHitPart", kind: Kind::Bool, used: true, doc: "also kick the joints either side of the part hit (true)" },
        Param { name: "bodyPart", kind: Kind::Int, used: true, doc: "the part hit, 0..20 (-1: none)" },
        Param { name: "snapHipType", kind: Kind::Int, used: true, doc: "legs: 0 no kick, 1 about the kick's axis, 2 about the bullet's direction (0)" },
        Param { name: "snapLeftLeg", kind: Kind::Bool, used: true, doc: "(false)" },
        Param { name: "snapRightLeg", kind: Kind::Bool, used: true, doc: "(false)" },
        Param { name: "snapLeftArm", kind: Kind::Bool, used: true, doc: "(false)" },
        Param { name: "snapRightArm", kind: Kind::Bool, used: true, doc: "(false)" },
        Param { name: "snapSpine", kind: Kind::Bool, used: true, doc: "(true)" },
        Param { name: "snapNeck", kind: Kind::Bool, used: true, doc: "(true)" },
        Param { name: "snapPhasedLegs", kind: Kind::Bool, used: true, doc: "the right leg kicks the other way (true)" },
    ] },
    Message { name: "rdr2Lasso", params: &[
        Param { name: "lassoedtype", kind: Kind::Int, used: true, doc: "where the rope is: 0 round his neck (his hands claw at it), 1 round his chest and arms (arms pinned, he wriggles), 2 round his feet (legs held together, his arms reach out past his head clawing at the ground) (0)" },
        Param { name: "grabPart", kind: Kind::Int, used: true, doc: "round the neck: the part his hands claw at (11, the neck; -1: they don't)" },
        Param { name: "grabposl", kind: Kind::Vec3, used: true, doc: "where the left hand claws, from that part: forward, left, up (0.07,0.05,-0.051)" },
        Param { name: "grabposr", kind: Kind::Vec3, used: true, doc: "the right hand (0.09,-0.05,-0.051)" },
        Param { name: "grabpointamp", kind: Kind::Vec3, used: true, doc: "how far the hands claw about it, m (zero: they hold)" },
        Param { name: "grabpointfreq", kind: Kind::Vec3, used: true, doc: "how fast, Hz per axis (4,5,3)" },
        Param { name: "legAmplitude", kind: Kind::Float, used: true, doc: "down, how hard the legs kick (0.4)" },
        Param { name: "spinefreq", kind: Kind::Float, used: true, doc: "down, the spine writhes forwards and back this often, Hz (2.5)" },
        Param { name: "spineAmplitude", kind: Kind::Float, used: true, doc: "that much (1)" },
        Param { name: "spsidefreq", kind: Kind::Float, used: true, doc: "and side to side this often, Hz (2)" },
        Param { name: "spsideamp", kind: Kind::Float, used: true, doc: "that much (1)" },
        Param { name: "groundLegStiffness", kind: Kind::Float, used: true, doc: "down (11)" },
        Param { name: "groundArmStiffness", kind: Kind::Float, used: true, doc: "down (11)" },
        Param { name: "groundSpineStiffness", kind: Kind::Float, used: true, doc: "down (11)" },
        Param { name: "groundneckstiffness", kind: Kind::Float, used: true, doc: "down (11)" },
        Param { name: "groundLegDamping", kind: Kind::Float, used: true, doc: "down (0.5)" },
        Param { name: "groundArmDamping", kind: Kind::Float, used: true, doc: "down (0.5)" },
        Param { name: "groundSpineDamping", kind: Kind::Float, used: true, doc: "down (0.5)" },
        Param { name: "bumpsim", kind: Kind::Float, used: true, doc: "down, a bump off the ground now and then, N for a step (400; 0: none)" },
        Param { name: "bumpsimprob", kind: Kind::Float, used: true, doc: "chance of a bump each tenth of a second (0.08)" },
        Param { name: "stunnedtime", kind: Kind::Float, used: true, doc: "s he lies stunned, limp, once down (0)" },
        Param { name: "stunnedblendout", kind: Kind::Float, used: true, doc: "s coming out of it (0)" },
        Param { name: "groundFriction", kind: Kind::Float, used: false, doc: "the ground's grip on him (1)" },
        Param { name: "ropeendfriction", kind: Kind::Float, used: false, doc: "the roped part's grip (0.5)" },
        Param { name: "otherendfriction", kind: Kind::Float, used: false, doc: "the other end's grip (2)" },
        Param { name: "ropePos", kind: Kind::Vec3, used: true, doc: "RDR2 core: the rope's anchor, world; send again without start as it moves (zero: 2 m in front of his chest)" },
        Param { name: "ropePart", kind: Kind::Int, used: true, doc: "RDR2 core: the part the rope is on (10)" },
    ] },
    Message { name: "rdr2Bolas", params: &[
        Param { name: "bindStrength", kind: Kind::Float, used: true, doc: "how hard the bolas holds his ankles together, N per m (900)" },
        Param { name: "fallTime", kind: Kind::Float, used: true, doc: "s before his balance gives (0.3)" },
        Param { name: "struggleTime", kind: Kind::Float, used: true, doc: "s he struggles on the ground before he's free (5; 0: until stopped)" },
        Param { name: "struggle", kind: Kind::Float, used: true, doc: "how hard he struggles, 0..1 (0.7)" },
    ] },
    Message { name: "leanRandom", params: LEAN_RANDOM },
    Message { name: "hipsLeanInDirection", params: &[
        Param { name: "leanAmount", kind: Kind::Float, used: true, doc: "how far the hips go; less than 0 away (0.5)" },
        Param { name: "dir", kind: Kind::Vec3, used: true, doc: "which way, world (zero: none)" },
    ] },
    Message { name: "hipsLeanToPosition", params: &[
        Param { name: "leanAmount", kind: Kind::Float, used: true, doc: "how far the hips go; less than 0 away (0.2)" },
        Param { name: "pos", kind: Kind::Vec3, used: true, doc: "towards this point, world" },
    ] },
    Message { name: "hipsLeanTowardsObject", params: &[
        Param { name: "leanAmount", kind: Kind::Float, used: true, doc: "how far the hips go; less than 0 away (0.5)" },
        Param { name: "offset", kind: Kind::Vec3, used: true, doc: "towards this point, world (as leanTowardsObject)" },
        Param { name: "instanceIndex", kind: Kind::Int, used: false, doc: "the object (-1)" },
        Param { name: "boundIndex", kind: Kind::Int, used: false, doc: "its part (0)" },
    ] },
    Message { name: "hipsLeanRandom", params: HIPS_LEAN_RANDOM },
    Message { name: "forceLeanInDirection", params: &[
        Param { name: "leanAmount", kind: Kind::Float, used: true, doc: "the push, times his weight; less than 0 away (0.2)" },
        Param { name: "dir", kind: Kind::Vec3, used: true, doc: "which way, world (zero: none)" },
        Param { name: "bodyPart", kind: Kind::Int, used: true, doc: "the part pushed, 0..20 (0, the pelvis)" },
    ] },
    Message { name: "forceLeanToPosition", params: &[
        Param { name: "leanAmount", kind: Kind::Float, used: true, doc: "the push, times his weight; less than 0 away (0.2)" },
        Param { name: "pos", kind: Kind::Vec3, used: true, doc: "towards this point, world" },
        Param { name: "bodyPart", kind: Kind::Int, used: true, doc: "the part pushed, 0..20 (0, the pelvis)" },
    ] },
    Message { name: "forceLeanTowardsObject", params: &[
        Param { name: "leanAmount", kind: Kind::Float, used: true, doc: "the push, times his weight; less than 0 away (0.2)" },
        Param { name: "offset", kind: Kind::Vec3, used: true, doc: "towards this point, world (as leanTowardsObject)" },
        Param { name: "instanceIndex", kind: Kind::Int, used: false, doc: "the object (-1)" },
        Param { name: "boundIndex", kind: Kind::Int, used: false, doc: "its part (0)" },
        Param { name: "bodyPart", kind: Kind::Int, used: true, doc: "the part pushed, 0..20 (0, the pelvis)" },
    ] },
    Message { name: "forceLeanRandom", params: FORCE_LEAN_RANDOM },
    Message { name: "onFire", params: &[
        Param { name: "staggerTime", kind: Kind::Float, used: true, doc: "s he staggers on his feet before he goes down (2.5)" },
        Param { name: "staggerLeanRate", kind: Kind::Float, used: true, doc: "how fast his stagger's lean swings, 0..1 (0.9)" },
        Param { name: "stumbleMaxLeanBack", kind: Kind::Float, used: true, doc: "most he leans back while staggering (0.4)" },
        Param { name: "stumbleMaxLeanForward", kind: Kind::Float, used: true, doc: "most he leans forward while staggering (0.5)" },
        Param { name: "armsWindmillWritheBlend", kind: Kind::Float, used: true, doc: "on his feet, arms: 0 windmill, 1 writhe (0.4)" },
        Param { name: "spineStumbleWritheBlend", kind: Kind::Float, used: true, doc: "on his feet, spine: 0 held, 1 writhes (0.7)" },
        Param { name: "legsStumbleWritheBlend", kind: Kind::Float, used: true, doc: "on his feet, legs: 0 the balancer's, 1 writhe (0.2)" },
        Param { name: "armsPoseWritheBlend", kind: Kind::Float, used: true, doc: "down, arms: 0 curled to his face, 1 writhe (0.7)" },
        Param { name: "spinePoseWritheBlend", kind: Kind::Float, used: true, doc: "down, spine: 0 curled, 1 writhes (0.55)" },
        Param { name: "legsPoseWritheBlend", kind: Kind::Float, used: true, doc: "down, legs: 0 drawn up, 1 writhe (0.5)" },
        Param { name: "rollTorqueScale", kind: Kind::Float, used: true, doc: "down, how hard he rolls to put the flames out (25)" },
        Param { name: "predictTime", kind: Kind::Float, used: true, doc: "s ahead his stagger leans against where he's heading (0.1)" },
        Param { name: "rollOverFlag", kind: Kind::Bool, used: true, doc: "down, he rolls back and forth (true)" },
        Param { name: "maxRollOverTime", kind: Kind::Float, used: true, doc: "s he keeps rolling (8)" },
        Param { name: "rollOverRadius", kind: Kind::Float, used: true, doc: "m from where he fell he may roll (2)" },
    ] },
];

const LEAN_RANDOM: &[Param] = &[
    Param { name: "leanAmountMin", kind: Kind::Float, used: true, doc: "least lean (0.2)" },
    Param { name: "leanAmountMax", kind: Kind::Float, used: true, doc: "most lean (0.2)" },
    Param { name: "changeTimeMin", kind: Kind::Float, used: true, doc: "s before a new way, at least (0.5)" },
    Param { name: "changeTimeMax", kind: Kind::Float, used: true, doc: "at most (1)" },
];

const HIPS_LEAN_RANDOM: &[Param] = &[
    Param { name: "leanAmountMin", kind: Kind::Float, used: true, doc: "least lean of the hips (0.3)" },
    Param { name: "leanAmountMax", kind: Kind::Float, used: true, doc: "most lean of the hips (0.4)" },
    Param { name: "changeTimeMin", kind: Kind::Float, used: true, doc: "s before a new way, at least (2)" },
    Param { name: "changeTimeMax", kind: Kind::Float, used: true, doc: "at most (4)" },
];

const FORCE_LEAN_RANDOM: &[Param] = &[
    Param { name: "leanAmountMin", kind: Kind::Float, used: true, doc: "least push, times his weight (0.3)" },
    Param { name: "leanAmountMax", kind: Kind::Float, used: true, doc: "most push, times his weight (0.4)" },
    Param { name: "changeTimeMin", kind: Kind::Float, used: true, doc: "s before a new way, at least (2)" },
    Param { name: "changeTimeMax", kind: Kind::Float, used: true, doc: "at most (4)" },
    Param { name: "bodyPart", kind: Kind::Int, used: true, doc: "the part pushed, 0..20 (0, the pelvis)" },
];

pub fn build(name: &str, p: &mut P) -> Option<Result<Box<dyn Extra>, Error>> {
    match name {
        "duck" => Some(Duck::new(p).map(|e| Box::new(e) as _)),
        "pointGun" => crate::extras_pg_rdr2::build(name, p),
        "controlledFall" => crate::extras_cf_rdr2::build(name, p),
        "shotSnap" => Some(Snap::new(p).map(|e| Box::new(e) as _)),
        "shotSpin" => Some(Spin::new(p).map(|e| Box::new(e) as _)),
        "shotFallToKnees" => Some(Knees::new(p).map(|e| Box::new(e) as _)),
        "meleeReact" => Some(Melee::new(p).map(|e| Box::new(e) as _)),
        "shotFromBehind" => Some(FromBehind::new(p).map(|e| Box::new(e) as _)),
        "shotInGuts" => Some(InGuts::new(p).map(|e| Box::new(e) as _)),
        "shotInjuredLeg" => Some(InjuredLeg::new(p).map(|e| Box::new(e) as _)),
        "shotInjuredArm" => Some(InjuredArm::new(p).map(|e| Box::new(e) as _)),
        "reachForWound" => Some(ReachForWound::new(p).map(|e| Box::new(e) as _)),
        "wallCollision" => Some(WallCollision::new(p).map(|e| Box::new(e) as _)),
        "setFallingReaction" => Some(FallingReaction::new(p).map(|e| Box::new(e) as _)),
        "rdr2Balance" => Some(BalanceRdr2::new(p).map(|e| Box::new(e) as _)),
        "keyframe" => Some(Keyframe::new(p).map(|e| Box::new(e) as _)),
        "onFire" => Some(OnFire::new(p).map(|e| Box::new(e) as _)),
        "rdr2Lasso" => Some(Lasso::new(p).map(|e| Box::new(e) as _)),
        "rdr2Bolas" => Some(Bolas::new(p).map(|e| Box::new(e) as _)),
        "rdr2Sit" => Some(Sit::new(p).map(|e| Box::new(e) as _)),
        "applyBulletImpulse" => Some(BulletImpulse::new(p).map(|e| Box::new(e) as _)),
        "leanRandom" | "hipsLeanInDirection" | "hipsLeanToPosition" | "hipsLeanTowardsObject" | "hipsLeanRandom" | "forceLeanInDirection"
        | "forceLeanToPosition" | "forceLeanTowardsObject" | "forceLeanRandom" => Some(Lean::new(name, p).map(|e| Box::new(e) as _)),
        _ => None,
    }
}

/// One rate-limited channel: a joint axis easing from where it was towards its duck target and back.
struct Channel { joint: usize, axis: usize, rate: f32, start: f32, goal: f32, x: f32 }

/// RDR2's DUCK: a short, fast flinch away from danger. After timeBeforeDuck the spine and neck bend into a random
/// share of what's left of each joint's range (forwards or back, and optionally sideways and twisting away), each at
/// its own rate, for duckTime; the legs crouch a little through the balancer; then it all fades out over blendTime.
pub struct Duck {
    mag: [f32; 2], rates: [f32; 5], wait: f32, time: f32, blend: f32, twist: bool, forward: bool, left: bool,
    crouch: f32, stiffness: f32, t: f32, channels: Vec<Channel>,
    /// RDR2 core: RDR2's duck (rdr2_duck.md), run instead while the core is on
    r: DuckRdr2,
    core: bool,
}

impl Duck {
    fn new(p: &mut P) -> Result<Duck, Error> {
        Ok(Duck {
            mag: [p.f("magMin", 0.2)?.clamp(0.0, 1.0), p.f("magMax", 1.0)?.clamp(0.0, 1.0)],
            rates: [p.f("lsRate", 0.4)?, p.f("usRate", 0.4)?, p.f("headRate", 0.4)?, p.f("spineTwistRate", 0.5)?, p.f("neckTwistRate", 0.5)?],
            wait: p.f("timeBeforeDuck", 0.0)?.max(0.0), time: p.f("duckTime", 0.5)?.max(0.0), blend: p.f("blendTime", 0.5)?.max(0.01),
            twist: p.b("leanTwistStart", true)?, forward: p.b("bendForward", true)?, left: p.b("duckLeft", false)?,
            crouch: p.f("crouch", 0.05)?.max(0.0), stiffness: p.f("stiffness", 10.0)?,
            t: 0.0, channels: vec![], r: DuckRdr2::default().parse(p)?, core: false,
        })
    }

    /// Picks each channel's goal from the joints' current angles and limits.
    fn aim(&mut self, c: &mut Ctx) {
        let mag = self.mag[0] + (self.mag[1] - self.mag[0]) * c.ch.random();
        let (s, n) = (*SPINE(), *NECK());
        // (joint, axis, rate, direction): axis 0 lean forwards/back, 1 lean sideways, 2 twist
        // a spine or neck joint's lean1 bends forwards as it rises, so forwards is towards its upper limit
        let bend = if self.forward { 1.0 } else { -1.0 };
        let side = if self.left { -1.0 } else { 1.0 };
        let mut want: Vec<(usize, usize, f32, f32)> = vec![
            (s[0], 0, self.rates[0], bend), (s[1], 0, self.rates[0], bend),
            (s[2], 0, self.rates[1], bend), (s[3], 0, self.rates[1], bend),
            (n[0], 0, self.rates[2], bend), (n[1], 0, self.rates[2], bend),
        ];
        if self.twist {
            for &j in &s {
                want.extend([(j, 1, self.rates[3], side), (j, 2, self.rates[3], side)]);
            }
            for &j in &n {
                want.extend([(j, 1, self.rates[4], side), (j, 2, self.rates[4], side)]);
            }
        }
        self.channels = want.into_iter().map(|(joint, axis, rate, dir)| {
            let now = c.ch.actual(joint, c.b.frames)[axis];
            let l = c.ch.limits(joint);
            let room = if dir > 0.0 { (l[2 * axis + 1] - now).max(0.0) } else { (l[2 * axis] - now).min(0.0) };
            Channel { joint, axis, rate, start: now, goal: room * mag, x: 0.0 }
        }).collect();
    }
}

impl Extra for Duck {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if c.ch.rdr2 {
            self.core = true;
            self.r.tick(c, dt);
            return;
        }
        self.t += dt;
        let t = self.t - self.wait;
        if t < 0.0 {
            return;
        }
        if self.channels.is_empty() {
            self.aim(c);
        }
        let bending = t <= self.time;
        // weight: full while bending, fading to nothing over blendTime after
        let w = if bending { 1.0 } else { (1.0 - (t - self.time) / self.blend).clamp(0.0, 1.0) };
        for ch in &mut self.channels {
            let target = if bending { ch.goal } else { 0.0 };
            let step = ch.rate * dt;
            ch.x += (target - ch.x).clamp(-step, step);
            let e = &mut c.ch.eff[ch.joint];
            if ch.axis == 0 && w > 0.0 {
                e.set_stiffness(self.stiffness, 1.0, None);
            }
            let d = &mut e.desired[ch.axis];
            *d += (ch.start + ch.x - *d) * w;
        }
        if self.crouch > 0.0 {
            c.out.dbm_leg_offset = Some(-self.crouch * w);
            c.out.dbm_owned.push((9, -self.crouch * w, 0.0));
        }
    }

    fn done(&self) -> bool {
        !self.core && self.t > self.wait + self.time + self.blend
    }

    /// RDR2 core: a start reloads the defaults and the state (no end routine for the old duck), then a newHit ducks.
    fn hit(&mut self, p: &mut P) -> Option<Result<(), Error>> {
        self.core.then(|| Duck::new(p).map(|d| *self = Duck { core: true, ..d }))
    }

    /// RDR2 core: values sent without start replace the running ones; a newHit ducks.
    fn update(&mut self, p: &mut P) -> Result<(), Error> {
        if self.core {
            self.r = self.r.clone().parse(p)?;
        }
        Ok(())
    }

    fn stop(&mut self, c: &mut Ctx) {
        if self.core {
            self.r.end(c);
            return;
        }
        if self.crouch > 0.0 {
            c.out.dbm_leg_offset = Some(0.0);
        }
    }

    /// RDR2 core: duck's requests rank at headLook's id, not its own.
    fn marks(&self) -> Vec<(u32, Vec<usize>)> {
        self.r.marks.map_or(vec![], |[s, n]| vec![(s, SPINE().to_vec()), (n, NECK().to_vec())])
    }

    /// RDR2 core: pointGun's +0x7b7 while it runs.
    fn pg_duck(&self) -> Option<bool> {
        self.core.then_some(self.r.gun)
    }
}

/// One of RDR2's duck ramps: a goal, where it is now and its rate.
#[derive(Clone, Copy)]
struct Ramp { goal: f32, current: f32, rate: f32 }

/// The fixed ranges RDR2's duck measures each group's room against, in its ramp order: spine0+1 lean1, spine2+3
/// lean1, spine lean2, spine twist, neck+head lean1, neck+head lean2, neck+head twist.
const DUCK_RANGE: [f32; 7] = [0.7, 0.71, 0.688, 0.571, 0.87, 0.7, 1.135];

/// RDR2's duck (rdr2_duck.md). Nothing happens until a newHit: then (after any earlier duck's duckTime) the spine and
/// neck groups jump from where they are by a granted share of the room left in each (a random budget split three
/// ways), the balancer's knees and hips dip at once and recover at their rates; after timeBeforeDuck the posture eases
/// back towards (never past) where it was for duckTime, then to nothing while the duck's weight fades over blendTime.
/// It never ends by itself. Its requests rank at headLook's id (0x1d, not duck's own): the spine just above headLook, the
/// neck just above until dontLookTime, then just below [C]. pointGunType 1 or 2 turns a running pointGun's arms neutral
/// while ducking. Ours: the requests' weight as a blend of each joint's desired angle towards the duck's; the balancer's
/// ownership is per step, so the dip is asked this step and its recovery the next.
#[derive(Clone)]
#[allow(dead_code)]
struct DuckRdr2 {
    mag: [f32; 2], leg_bend: [f32; 2], leg_rate: f32, hip_pitch: [f32; 2], hip_rate: f32,
    head_rate: f32, us_rate: f32, ls_rate: f32, neck_twist_rate: f32, spine_twist_rate: f32,
    wait: f32, time: f32, blend: f32, dont_look: f32, point_gun: i32, twist: bool, forward: bool, exag: bool, new_hit: bool,
    left: bool,
    ramps: [Ramp; 7],
    /// the pre-hit sums, in ramp order
    pre: [f32; 7],
    /// from -timeBeforeDuck at a hit; 100000 while no duck has run
    timer: f32,
    started: bool,
    /// the balancer's dips are ours to release
    owns: bool,
    /// the knee and hip targets to recover to, asked the step after the dip
    recover: Option<(f32, f32)>,
    /// the spine's and the neck's request ranks this step, once it has written them
    marks: Option<[u32; 2]>,
    traced: bool,
    /// pointGun's +0x7b7 this step
    gun: bool,
}

impl Default for DuckRdr2 {
    fn default() -> DuckRdr2 {
        DuckRdr2 { mag: [0.2, 1.0], leg_bend: [0.1, -0.25], leg_rate: 0.2, hip_pitch: [0.0, -0.25], hip_rate: 0.2, head_rate: 0.4, us_rate: 0.4,
                   ls_rate: 0.4, neck_twist_rate: 0.5, spine_twist_rate: 0.5, wait: 0.0, time: 0.5, blend: 0.5, dont_look: 0.0, point_gun: 0,
                   twist: true, forward: true, exag: false, new_hit: false, left: true,
                   ramps: [Ramp { goal: 0.0, current: 0.0, rate: 100000.0 }; 7], pre: [0.0; 7], timer: 100000.0, started: false, owns: false,
                   recover: None, marks: None, traced: false, gun: false }
    }
}

/// RDR2's random three-way split of a budget: None when the demands fit (each channel gets its full room), else each
/// channel's base less its demand plus its random share of the budget.
fn duck_split(c: &mut Ctx, budget: f32, base: [f32; 3], demand: [f32; 3], weight: [f32; 3]) -> Option<[f32; 3]> {
    if budget == 0.0 {
        return Some(std::array::from_fn(|k| base[k] - demand[k]));
    }
    // (a negative budget makes the shares negative: they fit, as in RDR2)
    let phi: [f32; 3] = std::array::from_fn(|k| demand[k] * weight[k] / budget);
    if phi.iter().sum::<f32>() <= 1.0 {
        return None;
    }
    const ORDERS: [[usize; 3]; 6] = [[0, 1, 2], [0, 2, 1], [1, 0, 2], [1, 2, 0], [2, 0, 1], [2, 1, 0]];
    let [f, s, t] = ORDERS[(c.ch.random_bits() % 6) as usize];
    let mut p = [0.0; 3];
    let (lo, hi) = ((1.0 - phi[s] - phi[t]).max(0.0), phi[f].min(1.0));
    p[f] = lo + c.ch.random() * (hi - lo);
    let left = 1.0 - p[f];
    let (lo, hi) = ((left - phi[t]).max(0.0), phi[s].min(left));
    p[s] = lo + c.ch.random() * (hi - lo);
    p[t] = left - p[s];
    Some(std::array::from_fn(|k| base[k] - demand[k] + p[k] * budget / weight[k]))
}

impl DuckRdr2 {
    /// The message's values over these (RDR2's defaults, or the running ones for a message without start).
    fn parse(mut self, p: &mut P) -> Result<DuckRdr2, Error> {
        self.mag = [p.f("magMin", self.mag[0])?, p.f("magMax", self.mag[1])?];
        self.leg_bend = [p.f("legBendMin", self.leg_bend[0])?, p.f("legBendMax", self.leg_bend[1])?];
        self.hip_pitch = [p.f("hipPitchMin", self.hip_pitch[0])?, p.f("hipPitchMax", self.hip_pitch[1])?];
        (self.leg_rate, self.hip_rate) = (p.f("legBendRate", self.leg_rate)?, p.f("hipPitchRate", self.hip_rate)?);
        (self.head_rate, self.us_rate, self.ls_rate) = (p.f("headRate", self.head_rate)?, p.f("usRate", self.us_rate)?, p.f("lsRate", self.ls_rate)?);
        self.neck_twist_rate = p.f("neckTwistRate", self.neck_twist_rate)?;
        self.spine_twist_rate = p.f("spineTwistRate", self.spine_twist_rate)?;
        (self.wait, self.time, self.blend) = (p.f("timeBeforeDuck", self.wait)?, p.f("duckTime", self.time)?, p.f("blendTime", self.blend)?);
        (self.dont_look, self.point_gun) = (p.f("dontLookTime", self.dont_look)?, p.i("pointGunType", self.point_gun)?);
        (self.twist, self.forward, self.exag) = (p.b("leanTwistStart", self.twist)?, p.b("bendForward", self.forward)?, p.b("duckExag", self.exag)?);
        (self.new_hit, self.left) = (p.b("newHit", self.new_hit)?, p.b("duckLeft", self.left)?);
        if self.blend <= 0.01 {
            self.blend = 0.0999;
        }
        Ok(self)
    }

    /// A new hit (section 3), unless the last duck is still within its duckTime (then it waits).
    fn hit(&mut self, c: &mut Ctx) {
        if self.time >= self.timer {
            return;
        }
        (self.timer, self.started, self.owns, self.new_hit) = (-self.wait, true, true, false);
        let sigma = if self.left { 1.0 } else { -1.0 };
        let mu = self.mag[0] + c.ch.random() * (self.mag[1] - self.mag[0]);
        let (s, n) = (*SPINE(), *NECK());
        let a = |j: usize| c.ch.actual(j, c.b.frames);
        let (sp, nk) = (s.map(a), n.map(a));
        let sum = |v: &[[f32; 3]], k: usize| v.iter().map(|x| x[k]).sum::<f32>();
        self.pre = [sp[0][0] + sp[1][0], sp[2][0] + sp[3][0], sum(&sp, 1), sum(&sp, 2), sum(&nk, 0), sum(&nk, 1), sum(&nk, 2)];
        let signed: [f32; 7] = std::array::from_fn(|k| if matches!(k, 2 | 3 | 5 | 6) { sigma * self.pre[k] } else { self.pre[k] });
        let room: [f32; 7] = std::array::from_fn(|k| (DUCK_RANGE[k] - signed[k].max(0.0)) / DUCK_RANGE[k]);
        let [ra, rb, rd, re, rc, rf, rg] = room;
        let m = (mu * (ra + rb + rc + (rd + re + rf + rg) * 0.5)).max(0.5);
        let mut factor = [1.0; 7];
        if let Some(x) = duck_split(c, m, [3.0, 2.0, 2.0], [ra + rb + rc, rd + rf, re + rg], [1.0, 0.5, 0.5]) {
            let one = [1.0; 3];
            let f1 = duck_split(c, x[0], one, [ra, rb, rc], one).unwrap_or(one);
            let f2 = duck_split(c, x[1], [1.0, 1.0, 0.0], [rd, rf, 0.0], one).unwrap_or(one);
            let f3 = duck_split(c, x[2], [1.0, 1.0, 0.0], [re, rg, 0.0], one).unwrap_or(one);
            factor = [f1[0], f1[1], f2[0], f3[0], f1[2], f2[1], f3[1]];
        }
        for k in 0..7 {
            let grant = factor[k] - (1.0 - room[k]);
            let x = if grant > 1e-4 && signed[k] <= 0.0 { 0.0 } else { signed[k] };
            let side = if matches!(k, 2 | 3 | 5 | 6) { sigma } else { 1.0 };
            self.ramps[k].current = side * (x + grant);
            // what it eases back towards is that floored posture, in the joints' own sign [C]
            self.pre[k] = side * x;
        }
        self.traced = false;
        let x = self.ramps.map(|r| (r.current * 100.0).round() / 100.0);
        crate::extras_ysk_rdr2::note(format_args!("duck: new hit, budget {m:.2}, ducked {x:?}, posture {:?}", self.pre.map(|p| (p * 100.0).round() / 100.0)));
        // the knees: their bend now (from the pelvis's height over the lower foot), dipped and recovering to it
        let zp = c.b.part(0).m.d.z;
        let zf = [2, 3].map(|i| c.b.part(crate::nmshot::limb(i).p[3]).m.d.z);
        let (straight, scale) = c.dbm.map_or((0.99, 0.0), |d| d.straight);
        let leg = scale - 0.5 * crate::nmdbm_rdr2::FOOT_HEIGHT;
        let k0 = if leg > 0.1 { (0.79235 * (straight - (zp - zf[0]).max(zp - zf[1]) / leg)).clamp(0.0, 0.35) } else { 0.0 };
        let [lo, hi] = self.leg_bend;
        let dip = if hi >= 0.0 { hi } else { lo + c.ch.random() * (hi.abs() - lo) };
        let k = (dip + k0).clamp(0.0, 0.35);
        // the hips: their pitch now from the pelvis (RDR2's sign; ours is the other way), dipped and recovering
        let (pm, up) = (c.b.part(0).m, c.up);
        let now = if pm.b.dot(up).abs() >= 0.8 { 0.0 } else {
            (std::f32::consts::FRAC_PI_2 - (-pm.c.dot(up)).clamp(-1.0, 1.0).acos()).clamp(-1.2, 1.2)
        };
        let h0 = now.clamp(-1.5, 1.5);
        let [lo, hi] = self.hip_pitch;
        let eta = if hi >= 0.0 { hi } else { lo + c.ch.random() * (hi.abs() - lo) };
        let pitch = (h0 - eta).clamp(-1.5, 1.5);
        crate::extras_ysk_rdr2::note(format_args!("duck: knees {k0:.3} dip to {k:.3}, hips {h0:.3} dip to {pitch:.3}, straight {straight:.4} leg {leg:.3}"));
        // owner 9 for the knees; the hips only when nobody owns them (owner 0 loses to any asker)
        c.out.dbm_crouch.extend([Crouch::Knee(9, -k, f32::INFINITY), Crouch::Hip(0, -pitch.clamp(-3.5, 3.5), f32::INFINITY)]);
        self.recover = Some((-k0, -h0.clamp(-2.0, 2.0)));
    }

    fn release(&mut self, c: &mut Ctx) {
        self.owns = false;
        c.out.dbm_crouch.extend([Crouch::ReleaseKnee(self.leg_rate), Crouch::ReleaseHip(self.hip_rate)]);
    }

    /// RDR2's tick (section 4).
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        self.marks = None;
        if let Some((k, h)) = self.recover.take().filter(|_| self.owns) {
            c.out.dbm_crouch.extend([Crouch::Knee(9, k, self.leg_rate), Crouch::Hip(0, h, self.hip_rate)]);
        }
        if self.new_hit {
            self.hit(c);
        }
        let t = self.timer;
        self.gun = t < self.time + self.blend && matches!(self.point_gun, 1 | 2);
        if t >= self.time + self.blend {
            self.end(c);
        } else {
            if t > 0.0 {
                let hold = t <= self.time;
                let rates = [self.ls_rate, self.us_rate, self.spine_twist_rate, self.spine_twist_rate, self.head_rate, self.neck_twist_rate,
                             self.neck_twist_rate];
                for k in 0..7 {
                    let lean1 = matches!(k, 0 | 1 | 4);
                    let pre = self.pre[k];
                    let r = &mut self.ramps[k];
                    r.goal = if hold && (lean1 || self.twist) { pre } else { 0.0 };
                    r.rate = rates[k];
                    r.current += (r.goal - r.current).clamp(-r.rate * dt, r.rate * dt);
                    if hold && lean1 {
                        r.current = r.current.max(pre);
                    } else if hold && self.twist {
                        r.current = if self.left { r.current.max(pre) } else { r.current.min(pre) };
                    }
                }
                let w = if hold { 1.0 } else { ((self.time + self.blend - t) / self.blend).max(0.0) };
                let kappa = if self.forward { 0.5 } else { -0.5 };
                if self.mag[0].abs() > 1.2e-7 && self.mag[1].abs() > 1.2e-7 {
                    let x = self.ramps.map(|r| r.current);
                    let mut put = |j: usize, k: usize, v: f32| {
                        let d = &mut c.ch.eff[j].desired[k];
                        *d += (v - *d) * w;
                    };
                    for (i, j) in SPINE().iter().copied().enumerate() {
                        put(j, 0, kappa * if i < 2 { x[0] } else { x[1] });
                        put(j, 1, 0.25 * x[2]);
                        put(j, 2, 0.25 * x[3]);
                    }
                    for j in NECK().iter().copied() {
                        put(j, 0, kappa * x[4]);
                        put(j, 1, 0.5 * x[5]);
                        put(j, 2, 0.5 * x[6]);
                    }
                    // id 0x1d's rank (slot 0xf8), flag +1 / -1 either side of headLook's
                    self.marks = Some([0xf9, if t <= self.dont_look { 0xf9 } else { 0xf7 }]);
                    if !self.traced || (t > self.dont_look && t - dt <= self.dont_look) {
                        self.traced = true;
                        crate::extras_ysk_rdr2::note(format_args!("duck: requests at {t:.2} s, weight {w:.2}, neck rank {:#x}", if t <= self.dont_look { 0xf9 } else { 0xf7 }));
                    }
                }
            }
            if t > self.time && self.owns {
                self.release(c);
            }
        }
        self.timer += dt;
    }

    /// RDR2's end of a duck (section 5): the balancer let go; stopped mid-duck, the spine's angles zeroed; the neck's
    /// too unless headLook runs and dontLookTime is over.
    fn end(&mut self, c: &mut Ctx) {
        if !std::mem::replace(&mut self.started, false) {
            return;
        }
        if self.timer > self.time && self.owns {
            self.release(c);
        }
        if self.timer > 0.0 {
            if self.timer < self.time + self.blend {
                for j in SPINE().iter().copied() {
                    c.ch.eff[j].desired = [0.0; 3];
                }
            }
            let neck = self.timer < self.dont_look || !c.head_look_on;
            if neck {
                for j in NECK().iter().copied() {
                    c.ch.eff[j].desired = [0.0; 3];
                }
            }
            crate::extras_ysk_rdr2::note(format_args!("duck: end at {:.2} s, neck zeroed {neck}, headLook on {}", self.timer, c.head_look_on));
        }
    }
}

/// Each Delirium part's parent part (joint j joins part j + 1 to this).
const PARENT: [usize; 21] = [0, 0, 1, 2, 0, 4, 5, 0, 7, 8, 9, 10, 11, 10, 13, 14, 15, 10, 17, 18, 19];
/// Delirium's own scale: a kick of size 1 turns a part by about this much angular velocity per kg of the part.
const KICK: f32 = 0.3;

/// RDR2's shot snap: a sharp kick across chosen joints at the hit, then a smaller counter-kick back. The kick turns
/// each joint about one axis (across the bullet's direction, or his side axis), sized by his state (moving,
/// standing, airborne) and by joint (spine hardest, neck lightest), spread over snapDuration (0: a single step).
pub struct Snap {
    mag: f32, delay: f32, duration: f32, interval: f32, ratio: f32, random: f32, mult: [f32; 3], moving: f32,
    use_dir: bool, dir: Vec3, part: i32, hip: i32, limbs: [bool; 6], phased: bool,
    t: f32, axis: Option<Vec3>, kicked: f32, countered: f32,
    /// the hit part RDR2 kicks round (snapHitPart is off by default there)
    rdr2_part: i32,
}

impl Snap {
    fn new(p: &mut P) -> Result<Snap, Error> {
        let mag = p.f("snapMag", 0.4)?;
        let (delay, duration) = (p.f("snapDelay", 0.0)?.max(0.0), p.f("snapDuration", 0.0)?.max(0.0));
        let (interval, ratio) = (p.f("unSnapInterval", 0.01)?.max(0.0), p.f("unSnapRatio", 0.7)?);
        let random = p.f("snapDirectionRandomness", 0.3)?.clamp(0.0, 1.0);
        let mult = [p.f("snapMovingMult", 1.0)?, p.f("snapBalancingMult", 1.0)?, p.f("snapAirborneMult", 1.0)?];
        let moving = p.f("snapMovingThresh", 1.0)?;
        let (use_dir, dir) = (p.b("snapUseBulletDir", true)?, p.v("bulletDir", Vec3::ZERO)?);
        let given = p.get("snapHitPart").map(Value::b).transpose()?;
        let hit = given.unwrap_or(true);
        let part = p.i("bodyPart", -1)?;
        let rdr2_part = if given.unwrap_or(false) { part } else { -1 };
        let hip = p.i("snapHipType", 0)?;
        let limbs = [p.b("snapLeftLeg", false)?, p.b("snapRightLeg", false)?, p.b("snapLeftArm", false)?,
                     p.b("snapRightArm", false)?, p.b("snapSpine", true)?, p.b("snapNeck", true)?];
        Ok(Snap { mag, delay, duration, interval, ratio, random, mult, moving, use_dir, dir, part: if hit { part } else { -1 },
                  hip, limbs, phased: p.b("snapPhasedLegs", true)?, t: 0.0, axis: None, kicked: 0.0, countered: 0.0, rdr2_part })
    }

    /// The share of a kick (spread over the duration) that falls in this step; all of it when the duration is 0.
    fn share(&self, since: f32, dt: f32, done: f32) -> f32 {
        if self.duration <= 0.0 {
            return 1.0 - done;
        }
        (((since + dt).min(self.duration) - since.max(0.0)) / self.duration).clamp(0.0, 1.0 - done)
    }

    /// Kicks joint `j` (the one driving part j + 1) about `axis` by `amount`: equal and opposite on its two parts.
    fn kick(c: &mut Ctx, j: usize, axis: Vec3, amount: f32, dt: f32) {
        let (child, parent) = (j + 1, PARENT[j + 1]);
        let t = axis * (amount * KICK * c.b.doll.part_mass(child) / dt);
        c.out.torque(child, t);
        c.out.torque(parent, -t);
    }

    fn apply(&mut self, c: &mut Ctx, m: f32, dt: f32) {
        let up = c.ch.up.normalize_or(Vec3::Z);
        let side = (c.b.part(13).m.d - c.b.part(17).m.d).normalize_or(Vec3::X);
        // left shoulder minus right, crossed this way round, points the way he faces
        let forward = side.cross(up).normalize_or(Vec3::Y);
        let dir = if self.dir == Vec3::ZERO { -forward } else { self.dir.normalize_or(-forward) };
        let (use_dir, random) = (self.use_dir, self.random);
        let r = Vec3::new(c.ch.random(), c.ch.random(), c.ch.random()) * 2.0 - 1.0;
        let axis = *self.axis.get_or_insert_with(|| {
            let a = if use_dir { up.cross(dir).normalize_or(side) } else { side };
            a.lerp(r.normalize_or(a), random).normalize_or(a)
        });
        // his state: nothing touching (airborne), only the feet (moving or standing), more than the feet (as it is)
        let feet = c.b.doll.collided(&[3, 6]);
        let other = c.b.doll.collided(&[0, 1, 2, 4, 5, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]);
        let horizontal = (c.vel - up * c.vel.dot(up)).length();
        let m = if !feet && !other {
            m * self.mult[2]
        } else if !other {
            m * self.mult[if horizontal > self.moving { 0 } else { 1 }]
        } else {
            m
        };
        let (s, n) = (*SPINE(), *NECK());
        if (0..21).contains(&self.part) {
            let k = self.part as usize;
            for j in [k.checked_sub(1), (k < 20).then_some(k)].into_iter().flatten() {
                Self::kick(c, j, axis, m * 15.0, dt);
            }
        }
        if self.limbs[4] {
            for (i, &j) in s.iter().enumerate() {
                Self::kick(c, j, axis, m * if i == 3 { 8.0 } else { 15.0 }, dt);
            }
        }
        if self.limbs[5] {
            for &j in &n {
                Self::kick(c, j, axis, m, dt);
            }
        }
        let mix = (up + dir).normalize_or(up);
        for (on, sign, clavicle, shoulder) in [(self.limbs[2], 1.0, 12, 13), (self.limbs[3], -1.0, 16, 17)] {
            if on {
                Self::kick(c, shoulder, mix, m * 4.0 * sign, dt);
                Self::kick(c, clavicle, up, m * 4.0 * sign, dt);
            }
        }
        if self.hip > 0 {
            let leg_axis = if self.hip >= 2 { dir } else { axis };
            for (on, hip, sign) in [(self.limbs[0], 0, 1.0), (self.limbs[1], 3, if self.phased { -1.0 } else { 1.0 })] {
                if on {
                    Self::kick(c, hip, leg_axis, m * 15.0 * sign, dt);
                }
            }
        }
    }
}

impl Extra for Snap {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        let since = self.t - self.delay;
        self.t += dt;
        if since + dt <= 0.0 {
            return;
        }
        let k = self.share(since, dt, self.kicked);
        if k > 0.0 {
            self.kicked += k;
            if c.ch.rdr2 {
                self.apply_rdr2(c, self.mag * k);
            } else {
                self.apply(c, self.mag * k, dt);
            }
        }
        let back = since - self.duration - self.interval;
        if self.ratio != 0.0 && back + dt > 0.0 {
            let k = self.share(back, dt, self.countered);
            if k > 0.0 {
                self.countered += k;
                if c.ch.rdr2 {
                    self.apply_rdr2(c, -self.ratio * self.mag * k);
                } else {
                    self.apply(c, -self.ratio * self.mag * k, dt);
                }
            }
        }
    }

    fn done(&self) -> bool {
        self.kicked >= 0.999 && (self.ratio == 0.0 || self.countered >= 0.999)
    }
}

/// RDR2's shot spin and step turn. A hit off the body's centre spins him round: the balancer is turned towards a
/// heading that swings round by the hit's spin (its lever arm about his centre, across the bullet, times
/// shotSpinTwistMult, at most shotSpinTwist), and a damped yaw torque on the pelvis and trunk pulls him onto it and
/// holds him there while it settles; it stops early when he's already turning faster than shotSpinTwistVel. Then, with stepTurnType 1, after stepTurnWait he steps
/// round to face attackerPos.
pub struct Spin {
    spin: Option<f32>, hit: Vec3, dir: Vec3, min: f32, time: f32, settle: f32, mult: f32, max_twist: f32,
    max_vel: f32, torque: f32, torque_max: f32, step_type: i32, step_wait: f32, step_time: f32, attacker: Vec3,
    t: f32, heading: Option<(Vec3, f32)>, turned: f32,
    /// RDR2's values (its defaults: twist mult 20, gates 9 rad / 20 rad/s / 100, torque 10 up to 20 outside 0.2 rad)
    rdr2: SpinRdr2, lever: Option<f32>, spin_on: bool,
}

#[derive(Clone, Copy)]
struct SpinRdr2 { twist_mult: f32, twist: f32, twist_vel: f32, twist_mom: f32, torque: f32, torque_max: f32, tol: f32, align: f32, step_offset: f32 }

impl Spin {
    fn new(p: &mut P) -> Result<Spin, Error> {
        let spin = p.f("spin", f32::NAN)?;
        Ok(Spin {
            spin: (!spin.is_nan()).then_some(spin), hit: p.v("hitPoint", Vec3::ZERO)?, dir: p.v("bulletDir", Vec3::ZERO)?,
            min: p.f("shotSpinMin", 0.02)?.max(0.0), time: p.f("shotSpinTime", 0.3)?.max(0.0), settle: p.f("shotSpinSettle", 0.3)?.max(0.0),
            mult: p.f("shotSpinTwistMult", 10.0)?, max_twist: p.f("shotSpinTwist", 1.5)?.abs(), max_vel: p.f("shotSpinTwistVel", 6.0)?.abs(),
            torque: p.f("shotSpinTorque", 150.0)?, torque_max: p.f("shotSpinTorqueMax", 300.0)?.abs(),
            step_type: p.i("stepTurnType", 0)?, step_wait: p.f("stepTurnWait", 0.3)?.max(0.0), step_time: p.f("stepTurnTime", 2.0)?.max(0.0),
            attacker: p.v("attackerPos", Vec3::ZERO)?, t: 0.0, heading: None, turned: 0.0,
            rdr2: SpinRdr2 {
                twist_mult: p.f("shotSpinTwistMult", 20.0)?, twist: p.f("shotSpinTwist", 9.0)?, twist_vel: p.f("shotSpinTwistVel", 20.0)?,
                twist_mom: p.f("shotSpinTwistMom", 100.0)?, torque: p.f("shotSpinTorque", 10.0)?, torque_max: p.f("shotSpinTorqueMax", 20.0)?,
                tol: p.f("shotSpinTorqueTol", 0.2)?, align: 0.9, step_offset: p.f("stepTurnOffset", 0.0)?,
            },
            lever: None, spin_on: p.b("shotSpin", true)?,
        })
    }

    fn facing(c: &Ctx, up: Vec3) -> Vec3 {
        let side = c.b.part(13).m.d - c.b.part(17).m.d;
        let f = side.cross(up);
        (f - up * f.dot(up)).normalize_or(Vec3::Y)
    }
}

impl Extra for Spin {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if c.ch.rdr2 {
            return self.tick_rdr2(c, dt);
        }
        self.t += dt;
        let up = c.ch.up.normalize_or(Vec3::Z);
        let yaw_rate = c.angvel.dot(up);
        self.turned += yaw_rate * dt;
        let (spin, hit, dir, mult, max_twist) = (self.spin, self.hit, self.dir, self.mult, self.max_twist);
        let (start, total) = *self.heading.get_or_insert_with(|| {
            // the spin: the hit's lever arm about his centre, across the bullet, about up
            let s = spin.unwrap_or_else(|| (hit - c.com).cross(dir.normalize_or_zero()).dot(up));
            (Self::facing(c, up), (s * mult).clamp(-max_twist, max_twist))
        });
        let spinning = self.t <= self.time + self.settle;
        if spinning && total.abs() > self.min * mult && self.turned.abs() <= max_twist + 0.5 && yaw_rate.abs() < self.max_vel {
            // the heading swings round to start + total over shotSpinTime, then holds while it settles: a yaw torque
            // on pelvis and trunk towards it (shotSpinTorque per rad off), damped by his turn rate so the planted
            // feet don't spring him back, and the balancer told to face it
            let a = total * (self.t / self.time.max(dt)).min(1.0);
            let heading = bevy_math::Quat::from_axis_angle(up, a) * start;
            let now = Self::facing(c, up);
            let off = now.cross(heading).dot(up).atan2(now.dot(heading));
            let tq = (self.torque * off - self.torque * 0.3 * yaw_rate).clamp(-self.torque_max, self.torque_max);
            for k in [0, 7, 8] {
                c.out.torque(k, up * tq / 3.0);
            }
            c.out.dbm_turn = Some(heading * 0.5);
            return;
        }
        let step_from = self.time + self.settle + self.step_wait;
        if self.step_type > 0 && self.t > step_from && self.t <= step_from + self.step_time {
            let to = self.attacker - c.com;
            let to = (to - up * to.dot(up)).normalize_or_zero();
            if to != Vec3::ZERO && Self::facing(c, up).dot(to) < 0.98 {
                c.out.dbm_turn = Some(to * 0.5);
                return;
            }
        }
        c.out.dbm_turn = Some(Vec3::ZERO);
    }

    fn done(&self) -> bool {
        let end = self.time + self.settle + if self.step_type > 0 { self.step_wait + self.step_time } else { 0.0 };
        self.t > end
    }

    fn stop(&mut self, c: &mut Ctx) {
        c.out.dbm_turn = Some(Vec3::ZERO);
    }
}

/// RDR2's fall to knees (a shot module there): after balancing for ftkBalanceTime the balancer lets go and he sinks:
/// knees bend at ftkBendRate, hips flex, the spine bends forward by ftkSpineBend and a helper push on the pelvis
/// tips him forwards over his feet. When both shins touch the ground he is kneeling: thighs come upright under him and
/// a lean help keeps his centre over his knees for ftkKneelTime; then he topples forwards (or back) and goes limp.
pub struct Knees {
    balance: f32, bend_rate: f32, helper: f32, spine_bend: f32, stiffness: f32, kneel_time: f32, lean_help: f32,
    forwards: bool, pitch: f32, arms_down: bool, kneel_hip: f32,
    t: f32, phase: u8, since: f32, knee: [f32; 2], hip: [f32; 2],
    /// With the RDR2 core the message only writes the character's record (shotFallToKnees 0x2a535a8; each message
    /// starts from the defaults), and the shot's module runs it (nmftk_rdr2.rs).
    rdr2: crate::nmftk_rdr2::Ftk, written: bool,
}

impl Knees {
    fn new(p: &mut P) -> Result<Knees, Error> {
        let d = crate::nmftk_rdr2::Ftk::default();
        // RDR2's ftkPitchForwards is a float, Delirium's own a flag
        let pitch_forwards = match p.get("ftkPitchForwards") {
            Some(Value::Bool(_)) | None => d.pitch_forwards,
            Some(v) => v.f()?,
        };
        let pitch_forwards = p.f("ftkPitchForwardsRad", pitch_forwards)?;
        let rdr2 = crate::nmftk_rdr2::Ftk {
            on: p.b("fallToKnees", true)?, balance_time: p.f("ftkBalanceTime", d.balance_time)?, helper_force: p.f("ftkHelperForce", d.helper_force)?,
            lean_help: p.f("ftkLeanHelp", d.lean_help)?, spine_bend: p.f("ftkSpineBend", d.spine_bend)?,
            impact_looseness: p.f("ftkImpactLooseness", d.impact_looseness)?, impact_time: p.f("ftkImpactLoosenessTime", d.impact_time)?,
            bend_rate: p.f("ftkBendRate", d.bend_rate)?, hip_blend: p.f("ftkHipBlend", d.hip_blend)?, lunge_prob: p.f("ftkLungeProb", d.lunge_prob)?,
            fric_mult: p.f("ftkFricMult", d.fric_mult)?, hip_angle_fall: p.f("ftkHipAngleFall", d.hip_angle_fall)?, pitch_forwards,
            pitch_backwards: p.f("ftkPitchBackwards", d.pitch_backwards)?, fall_below_stab: p.f("ftkFallBelowStab", d.fall_below_stab)?,
            thresh_per_s: p.f("ftkFallBelowStabPerSecond", d.thresh_per_s)?, thresh_per_step: p.f("ftkFallBelowStabPerStep", d.thresh_per_step)?,
            abort_threshold: p.f("ftkBalanceAbortThreshold", d.abort_threshold)?, release_rfw: p.f("ftkReleaseReachForWound", d.release_rfw)?,
            arm_type: p.i("ftkOnKneesArmType", d.arm_type)?, release_point_gun: p.b("ftkReleasePointGun", d.release_point_gun)?,
            fail_must_collide: p.b("ftkFailMustCollide", d.fail_must_collide)?, stay_upright: p.b("ftkStayUpright", d.stay_upright)?,
            knee_spin: p.b("ftkKneeSpin", d.knee_spin)?, stiff_spine: p.b("ftkStiffSpine", d.stiff_spine)?,
            helper_on_spine: p.b("ftkHelperForceOnSpine", d.helper_on_spine)?, always_change_fall: p.b("ftkAlwaysChangeFall", d.always_change_fall)?,
            ..d
        };
        p.b("ftkReachForWound", true)?;
        Ok(Knees {
            rdr2, written: false,
            balance: p.f("ftkBalanceTime", 0.1)?.max(0.0), bend_rate: p.f("ftkBendRate", 3.0)?.abs(),
            helper: p.f("ftkHelperForce", 0.0)?, spine_bend: p.f("ftkSpineBend", 0.1)?, stiffness: p.f("ftkStiffness", 9.0)?,
            kneel_time: p.f("ftkKneelTime", 2.0)?.max(0.0), lean_help: p.f("ftkLeanHelp", 1500.0)?,
            forwards: p.get("ftkPitchForwards").map_or(Ok(true), |v| if matches!(v, Value::Bool(_)) { v.b() } else { Ok(true) })?, pitch: p.f("ftkPitchForce", 300.0)?, arms_down: p.i("ftkOnKneesArmType", 1)? == 1, kneel_hip: p.f("ftkKneelHip", 0.3)?,
            t: 0.0, phase: 0, since: 0.0, knee: [0.0; 2], hip: [0.0; 2],
        })
    }

    fn forward(c: &Ctx, up: Vec3) -> Vec3 {
        let f = (c.b.part(13).m.d - c.b.part(17).m.d).cross(up);
        (f - up * f.dot(up)).normalize_or(Vec3::Y)
    }

    /// Moves joint j's axis-k target towards `to` by at most `step`, from where it is now the first time.
    fn ease(c: &mut Ctx, j: usize, k: usize, to: f32, step: f32) {
        let now = c.ch.eff[j].desired[k];
        c.ch.eff[j].desired[k] = now + (to - now).clamp(-step, step);
    }
}

impl Extra for Knees {
    fn lines(&mut self) -> Vec<String> {
        if self.written { vec!["#say shotFallToKnees wrote the character's record".into()] } else { vec![] }
    }

    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if c.ch.rdr2 {
            // only the settings: what the core keeps of its own stays, and the threshold follows (ctx+0x2e30)
            let f = &mut c.ch.core.ftk;
            let keep = *f;
            *f = keep.with_settings(&self.rdr2);
            self.written = true;
            return;
        }
        self.t += dt;
        self.since += dt;
        let up = c.ch.up.normalize_or(Vec3::Z);
        let fwd = Self::forward(c, up);
        let (knees, hips, spine) = (*KNEE(), *HIP(), *SPINE());
        match self.phase {
            0 => {
                if self.t >= self.balance {
                    // let go of the balancer; start from the legs' current angles
                    c.out.dbm = Some(false);
                    for s in 0..2 {
                        self.knee[s] = c.ch.actual(knees[s], c.b.frames)[0];
                        self.hip[s] = c.ch.actual(hips[s], c.b.frames)[0];
                        c.ch.eff[knees[s]].desired[0] = self.knee[s];
                        c.ch.eff[hips[s]].desired[0] = self.hip[s];
                    }
                    (self.phase, self.since) = (1, 0.0);
                }
            }
            1 => {
                // sinking: knees towards their deepest bend, hips flexing, spine forwards, a push over the feet
                let step = self.bend_rate * dt;
                c.ch.set_stiffness(self.stiffness, 1.0, "lb", None);
                c.ch.set_stiffness(self.stiffness, 1.0, "us", None);
                for s in 0..2 {
                    let kl = c.ch.limits(knees[s]);
                    let deep = if kl[0].abs() > kl[1].abs() { kl[0] } else { kl[1] } * 0.9;
                    Self::ease(c, knees[s], 0, deep, step);
                    let hl = c.ch.limits(hips[s]);
                    Self::ease(c, hips[s], 0, (hl[1] * 0.2).min(0.4), step);
                }
                for &j in &spine {
                    Self::ease(c, j, 0, self.spine_bend / 4.0, step);
                }
                c.out.force(0, fwd * self.helper);
                // on his knees once they are near the ground (measured from his feet; shin contacts are too patchy)
                let ground = c.b.part(3).m.d.dot(up).min(c.b.part(6).m.d.dot(up)) - 0.05;
                let knee = |t: usize, s: usize| (c.b.part(t).m.d + c.b.part(s).m.d).dot(up) * 0.5 - ground;
                let down = knee(1, 2) < 0.2 && knee(4, 5) < 0.2;
                // as the knees come down, start holding his centre over them so he doesn't pitch on
                if knee(1, 2).max(knee(4, 5)) < 0.35 {
                    let knees_at = (c.b.part(1).m.d + c.b.part(2).m.d + c.b.part(4).m.d + c.b.part(5).m.d) * 0.25;
                    let off = knees_at - c.com;
                    let vel = c.vel - up * c.vel.dot(up);
                    let pull = (off - up * off.dot(up)) * self.lean_help - vel * self.lean_help * 0.2;
                    c.out.force(0, pull * 0.5);
                    c.out.force(10, pull * 0.5);
                }
                if down || self.since > 1.5 {
                    (self.phase, self.since) = (2, 0.0);
                }
            }
            2 => {
                // kneeling: thighs upright under him, spine straight, his centre held over the knees
                let step = self.bend_rate * dt;
                c.ch.set_stiffness(self.stiffness, 1.0, "lb", None);
                c.ch.set_stiffness(self.stiffness, 1.0, "us", None);
                // hips held a little flexed and stiffer, so the trunk stays up over the knees
                for s in 0..2 {
                    Self::ease(c, hips[s], 0, self.kneel_hip, step);
                    c.ch.eff[hips[s]].set_stiffness(self.stiffness * 2.5, 1.0, None);
                }
                for &j in &spine {
                    Self::ease(c, j, 0, 0.0, step);
                    c.ch.eff[j].set_stiffness(self.stiffness * 1.5, 1.0, None);
                }
                if self.arms_down {
                    c.ch.set_stiffness(self.stiffness * 0.5, 1.0, "ua", None);
                }
                // the knees: between each thigh and shin; hold his centre over them
                let knees_at = (c.b.part(1).m.d + c.b.part(2).m.d + c.b.part(4).m.d + c.b.part(5).m.d) * 0.25;
                let off = knees_at - c.com;
                let vel = c.vel - up * c.vel.dot(up);
                let pull = (off - up * off.dot(up)) * self.lean_help - vel * self.lean_help * 0.2;
                c.out.force(0, pull * 0.5);
                c.out.force(10, pull * 0.5);
                if self.since > self.kneel_time {
                    (self.phase, self.since) = (3, 0.0);
                }
            }
            _ => {
                // topple: go loose and tip forwards (or back)
                c.ch.set_stiffness(self.stiffness * 0.3, 1.0, "fb", None);
                if self.since < 0.3 {
                    c.out.force(10, if self.forwards { fwd } else { -fwd } * self.pitch);
                }
            }
        }
    }

    fn done(&self) -> bool {
        self.written || (self.phase == 3 && self.since > 1.0)
    }
}

impl Snap {
    /// A snap at a hit part along `dir`, with Delirium's other defaults (used by meleeReact).
    fn at(mag: f32, part: i32, dir: Vec3) -> Snap {
        Snap { mag, delay: 0.0, duration: 0.0, interval: 0.05, ratio: 0.5, random: 0.25, mult: [1.0; 3], moving: 1.0,
               use_dir: true, dir, part, hip: 0, limbs: [false, false, false, false, true, true], phased: true,
               t: 0.0, axis: None, kicked: 0.0, countered: 0.0, rdr2_part: part }
    }
}

/// RDR2's melee reaction, built like its shot from smaller pieces: the upper body goes loose for csPeriod (by
/// csLooseness), a push leans him away from the blow for leanPeriod, he flinches (a small forward hunch, and with
/// armsType 1 both hands come up in front of his face) for flinchPeriod, the hit part's joints snap along the blow,
/// and after rfwDelay one hand reaches for the spot hit for rfwPeriod. With the RDR2 core, RDR2's own meleeReact
/// (`tick_rdr2`).
pub struct Melee {
    point: Vec3, push: Vec3, part: i32, loose: f32, loose_time: f32, lean: f32, lean_time: f32, flinch: bool,
    flinch_time: f32, guard: bool, rfw: bool, rfw_delay: f32, rfw_time: f32, stiffness: f32,
    snap: Option<Snap>, t: f32, offset: Option<Vec3>,
    rdr2: MelRdr2, r2: Option<Box<Mel2>>, look: Option<Vec3>, out: Vec<String>, said: u32,
}

/// RDR2's meleeReact parameters, over RDR2's defaults (its own hitPart, armsType and rfw defaults).
#[derive(Clone, Copy)]
struct MelRdr2 {
    hit_new: bool, hit_mag: f32, point: Vec3, normal: Vec3, part: usize, local: bool, cs_period: f32, cs_loose: f32,
    snap: bool, snap_mag: f32, flinch: bool, flinch_period: f32, rfw: bool, rfw_delay: f32, rfw_period: f32,
    lean: f32, lean_period: f32, head_look: bool, arms_type: i32, flinch_abort: f32, lean_abort: f32, rfw_not_bcr: bool, rfw_excl_cf: bool,
}

/// RDR2's meleeReact state: T (the time since the last blow, 10 until one comes), the COM velocity last tick, the
/// blow from behind, each reaching hand's pose, the failed balancer's joint scales (what was there, what we wrote),
/// and whether the head look was started this reach.
struct Mel2 {
    t: f32, prev_vel: Vec3, back: bool, pose: [Option<[f32; 6]>; 2], scaled: Vec<Option<([f32; 3], [f32; 3])>>, looked: bool,
}

impl Melee {
    fn new(p: &mut P) -> Result<Melee, Error> {
        let point = p.v("hitPoint", Vec3::ZERO)?;
        let normal = p.v("hitNormal", Vec3::ZERO)?;
        let given_part = p.get("hitPart").map(Value::i).transpose()?;
        let part = given_part.unwrap_or(-1);
        let (snap, snap_mag) = (p.b("snap", true)?, p.f("snapMag", 1.0)?);
        let push = -normal.normalize_or_zero();
        let (cs_loose, cs_period) = (p.f("csLooseness", 0.4)?, p.f("csPeriod", 0.3)?);
        let (lean, lean_period) = (p.f("Lean", 0.0)?, p.f("leanPeriod", 1.0)?);
        let (flinch, flinch_period) = (p.b("Flinch", true)?, p.f("flinchPeriod", 0.7)?);
        let arms = p.get("armsType").map(Value::i).transpose()?;
        let rfw = p.get("rfw").map(Value::b).transpose()?;
        let (rfw_delay, rfw_period) = (p.f("rfwDelay", 0.2)?, p.f("rfwPeriod", 1.0)?);
        let (flinch_abort, lean_abort) = (p.f("flinchAbortTime", 0.5)?, p.f("leanAbortTime", 0.5)?);
        let rdr2 = MelRdr2 {
            hit_new: p.b("hitNew", false)?, hit_mag: p.f("hitMag", 1.0)?, point, normal,
            part: given_part.filter(|k| (1..21).contains(k)).unwrap_or(0) as usize, local: p.b("hitLocal", true)?, cs_period, cs_loose,
            snap, snap_mag, flinch, flinch_period, rfw: rfw.unwrap_or(false), rfw_delay, rfw_period, lean, lean_period,
            head_look: p.b("useHeadLook", false)?, arms_type: arms.unwrap_or(0), flinch_abort, lean_abort,
            rfw_not_bcr: p.b("rfwNotInBcr", false)?, rfw_excl_cf: p.b("rfwExcludeCatchFall", false)?,
        };
        Ok(Melee {
            point, push, part, loose: cs_loose.clamp(0.0, 1.0), loose_time: cs_period.max(0.0),
            lean, lean_time: lean_period.max(0.0),
            flinch, flinch_time: flinch_period.max(0.0), guard: arms.unwrap_or(1) == 1,
            rfw: rfw.unwrap_or(true), rfw_delay: rfw_delay.max(0.0), rfw_time: rfw_period.max(0.0),
            stiffness: p.f("stiffness", 10.0)?,
            snap: (snap && push != Vec3::ZERO).then(|| Snap::at(snap_mag, part, push)), t: 0.0, offset: None,
            rdr2, r2: None, look: None, out: vec![], said: 0,
        })
    }

    /// A log line the first time a branch runs.
    fn say(&mut self, bit: u32, line: impl FnOnce() -> String) {
        if self.said >> bit & 1 == 0 {
            self.said |= 1 << bit;
            self.out.push(format!("#say melee {}", line()));
        }
    }

    /// RDR2's meleeReact (rdr2_melee.md). At its start: the desired angles off the legs zeroed, opposeGravity 1 on
    /// every joint. Each step (T the time since the last blow): within csPeriod the whole body is loosened by the
    /// shot's strength applier (L = csLooseness T/csPeriod + 1 - csLooseness, base stiffness 12) while the balancer
    /// runs, else the joint strengths are scaled by L; the flinch bends spine and neck by the blow's geometry (fading
    /// with 1 - T/flinchPeriod) and asks the balancer for a hip pitch and knee bend; on a new blow one snap of the
    /// hit part's joints; armsType 1 throws the arms out by the COM's jolt; rfw places the free hands 0.1 m off the
    /// spot hit; Lean asks the balancer to lean away from the blow. balancerCollisionsReaction's wall, known or hit, stops the
    /// flinch and the lean within their abort times. Then the hand-over to the fall-to-knees core (nmftk_rdr2.rs).
    fn tick_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        let r = self.rdr2;
        let mut s = match self.r2.take() {
            Some(s) => s,
            None => {
                let legs: Vec<usize> = crate::nmrs::Character::mask("lb");
                for j in 0..c.ch.eff.len() {
                    if !legs.contains(&j) {
                        c.ch.eff[j].desired = [0.0; 3];
                    }
                    c.ch.eff[j].oppose_gravity = 1.0;
                }
                Box::new(Mel2 { t: 10.0, prev_vel: c.vel, back: false, pose: [None; 2], scaled: vec![], looked: false })
            }
        };
        let up = c.up;
        let stagger = c.extras_on.contains(&"staggerFall");
        // balancerCollisionsReaction: a wall known or hit, and the time to it
        let (bcr, t_bcr) = c.ch.bcr_probe.unwrap_or((false, f32::MAX));
        // the hit in the world: the point through the hit part's frame when local, the normal as given
        let pm = c.b.part(r.part).m;
        let at = if r.local { crate::vm4(pm.a, pm.b, pm.c, pm.d, r.point) } else { r.point };
        let n = r.normal;
        if r.hit_new {
            self.out.push(format!("#say melee blow on part {}, time since the last {:.2}", r.part, s.t));
            s.t = 0.0;
            s.back = c.m.c.dot(n) > 0.0;
        }
        let t = s.t;
        // the pelvis frame: sv up the spine, a1 and a2 across it
        let sp = crate::nmshot::spine_parts();
        let p0 = c.b.part(sp[0]).m;
        let sv = (c.b.part(sp[4]).m.d - p0.d).normalize_or(up);
        let a1 = p0.c.cross(sv).normalize_or_zero();
        let a2 = sv.cross(a1).normalize_or_zero();
        let rel = at - p0.d;
        let (h, y1, y2) = (rel.dot(sv), rel.dot(a1), rel.dot(a2));
        let nl = Vec3::new(n.dot(sv), n.dot(a1), n.dot(a2));
        // the looseness window
        if t >= r.cs_period || r.cs_period <= 1e-6 {
            s.scaled.clear();
            if !c.upright && r.cs_period <= t && t <= r.cs_period + dt {
                for j in 0..c.ch.eff.len() {
                    let jd = c.ch.joints[j].clone();
                    c.ch.eff[j].reset(&jd);
                }
                self.say(0, || "joints reset as the looseness ends, balancer failed".into());
            }
        } else {
            let f = t / r.cs_period;
            self.say(if c.upright { 1 } else { 2 }, || format!("looseness window, balancer running {}", c.upright));
            if c.upright {
                crate::nmshot::apply_strength(c, &crate::nmshot::Strength {
                    k: f, body_stiffness: 12.0, arm_stiffness: 12.0, spine_damping: 1.0, neck: (12.0, 1.0), upper: 1.0, lower: 1.0,
                    looseness: c.ch.bcr.map_or(r.cs_loose, |b| if b.3 <= r.cs_loose { b.3 } else { r.cs_loose }), k_mult_on_loose: 0.0, k_mult_4_legs: 0.3, min_legs_looseness: 0.1, min_arms_looseness: 0.1,
                    stable_hands_and_neck: true, balancing: true }, dt);
            } else {
                let l = (f * r.cs_loose + 1.0 - r.cs_loose).clamp(0.1, 1.0);
                scale_joints(c, &mut s.scaled, l);
            }
        }
        // the flinch: spine and neck bent by the blow's geometry, the balancer's hip pitch and knee bend
        let k = r.hit_mag;
        if bcr && t_bcr <= r.flinch_abort && r.flinch && t < r.flinch_period {
            self.say(3, || format!("flinch aborted by the wall, time to it {t_bcr:.2}"));
        }
        if r.flinch && !(bcr && t_bcr <= r.flinch_abort) && r.flinch_period > 1e-5 && t < r.flinch_period && h > -0.15 {
            let g = 1.0 - t / r.flinch_period;
            let (mut a, mut b) = (nl.z.clamp(-1.0, 1.0).asin(), -nl.y.clamp(-1.0, 1.0).asin());
            if h < 0.4 {
                (a, b) = (-a, -b);
            }
            let v = Vec3::new(0.0, y1, y2);
            let u = nl.normalize_or_zero();
            let w = v - u * v.dot(u);
            let tw = if w.y > 0.0 { 10.0 * w.length() } else { -10.0 * w.length() };
            let q = |x: f32, share: f32| (share * x).clamp(-9.9, 9.9);
            for (joints, share) in [(&SPINE()[..], 0.25), (&NECK()[..], 0.5)] {
                for &j in joints {
                    let to = [q(k * a, share), q(k * b, share), q(tw, share)];
                    for (d, x) in c.ch.eff[j].desired.iter_mut().zip(to) {
                        *d += (x - *d) * g;
                    }
                }
            }
            self.say(4, || format!("flinch h {h:.2} A {a:.2} B {b:.2} twist {tw:.2}"));
            if !stagger && bcr {
                c.out.dbm_crouch.push(Crouch::Hip(8, 0.0, 4.0));
                c.out.dbm_crouch.push(Crouch::ReleaseKnee(4.0));
                self.say(5, || "flinch: hip and knee let go, a wall is known".into());
            } else if !stagger {
                let pitch = 2.0 * k * a1.cross(up).dot(n) * g * (h - 0.4);
                c.out.dbm_crouch.push(Crouch::Hip(8, -pitch.clamp(-2.0, 2.0), 1e5));
                c.out.dbm_crouch.push(Crouch::Knee(5, 0.3 * pitch, 1.0));
                self.say(6, || format!("flinch: balancer hip pitch {pitch:.2}"));
            }
        } else if !stagger {
            c.out.dbm_crouch.push(Crouch::Hip(8, 0.0, 4.0));
            c.out.dbm_crouch.push(Crouch::ReleaseKnee(4.0));
        }
        // the snap: one call on the blow's step, the hit part's joints only
        if r.hit_new && r.snap {
            let mut snap = Snap { mag: 0.0, delay: 0.0, duration: 0.0, interval: 0.0, ratio: 0.0, random: 0.0, mult: [1.0; 3], moving: 1.0,
                                  use_dir: true, dir: -n, part: r.part as i32, hip: 0, limbs: [false; 6], phased: false, t: 0.0, axis: None,
                                  kicked: 0.0, countered: 0.0, rdr2_part: r.part as i32 };
            snap.apply_rdr2(c, if h > 0.4 { -r.snap_mag } else { r.snap_mag });
            self.say(7, || format!("snap part {} h {h:.2}", r.part));
        }
        // armsType 1: the arms thrown out sideways by the COM's jolt
        if r.arms_type == 1 {
            let mut acc = (c.vel - s.prev_vel) / 60.0;
            if dt > 1e-6 {
                acc /= dt;
            }
            let a = acc.length();
            if a > 0.03 {
                let th = (a.min(0.5) - 0.03) * 3.34043;
                let ax = c.b.part(sp[4]).m.c.normalize_or(up);
                self.say(8, || format!("arms thrown by the jolt, acc {a:.3} theta {th:.2}"));
                for (i, sign) in [(0usize, -1.0f32), (1, 1.0)] {
                    let l = crate::nmshot::limb(i);
                    let to = c.b.joint_pos(l.e[1]) + bevy_math::Quat::from_axis_angle(ax, sign * th) * -sv;
                    crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[3], to, 0.0, 2.0, 1.0 - 2.0 * i as f32, 1.0, Vec3::ZERO, None);
                }
            }
        }
        // rfw: the free hands 0.1 m off the spot hit, unless a leg or (from behind) the trunk was hit
        let reach = !(r.rfw_not_bcr && c.ch.bcr.is_some_and(|b| b.0 != 0)) && r.rfw && r.rfw_delay < t && t < r.rfw_delay + r.rfw_period && c.upright
            && !(1..=6).contains(&r.part) && (!s.back || matches!(r.part, 11 | 12));
        if r.rfw_not_bcr && r.rfw && c.ch.bcr.is_some_and(|b| b.0 != 0) && r.rfw_delay < t && t < r.rfw_delay + r.rfw_period {
            self.say(13, || format!("no reach: balancerCollisionsReaction in state {}", c.ch.bcr.map_or(0, |b| b.0)));
        }
        // p13 0.05: a further 0.05 leg lengths in along the normal (0x2a4733c's skin offset)
        let leg = |c: &Ctx| c.b.joint_pos(HIP()[0]).distance(c.b.joint_pos(KNEE()[0])) + c.b.joint_pos(KNEE()[0]).distance(c.b.joint_pos(ANKLE()[0]));
        let off = 0.1 - 0.05 * leg(c);
        for i in 0..2 {
            let own = if i == 0 { 13..=16 } else { 17..=20 };
            let l = crate::nmshot::limb(i);
            if reach && !own.contains(&r.part) && c.b.joint_pos(l.e[1]).distance(at) >= 0.1 {
                hand_place(c, i, at + n * off, n, &mut s.pose[i], None, 0.0);
                if r.rfw_excl_cf {
                    c.ch.cf_exclude[i] = true;
                }
                self.say(9, || format!("reach: arm {i} to {at}"));
            } else {
                s.pose[i] = None;
            }
        }
        // useHeadLook: its target every tick of the reach
        if reach && r.head_look {
            if s.looked {
                self.out.push(format!("headLook pos={},{},{}", at.x, at.y, at.z));
            } else {
                self.look = Some(at);
            }
            self.say(10, || "head look at the spot hit".into());
        }
        s.looked = reach && r.head_look;
        // Lean: the balancer leans him away from the blow, unless the wall is close
        if !(bcr && t_bcr <= r.lean_abort) && !stagger && r.lean != 0.0 && t < r.lean_period {
            c.out.lean = Some((LeanMode::Dir(-n), r.lean, false));
            self.say(11, || format!("lean amount {}", r.lean));
        } else if bcr && t_bcr <= r.lean_abort && r.lean != 0.0 && t < r.lean_period {
            self.say(12, || format!("lean aborted by the wall, time to it {t_bcr:.2}"));
        }
        // the hand-over to fall to knees (0x2a2e0b4, with the character's record), once the flinch is done and the balancer
        // has failed, or if he's already kneeling
        let flinch_over = !r.flinch || t > r.flinch_period;
        let leaning = c.dbm.is_some_and(|d| d.lean_on);
        let f = c.ch.core.ftk;
        if f.on && (f.kneeling || (flinch_over && !leaning && c.vel.length() < 0.7 && c.ch.bcr.is_none_or(|b| b.0 == 0) && !c.upright)) {
            self.say(14, || format!("hands over to fall to knees, kneeling {}, balancer running {}", f.kneeling, c.upright));
            crate::nmftk_rdr2::core(c, &mut self.out, dt);
        }
        self.rdr2.hit_new = false;
        s.prev_vel = c.vel;
        s.t += dt;
        self.r2 = Some(s);
    }
}

/// RDR2's scale setter for meleeReact's failed balancer: every joint but the ankles and wrists at `l` of its
/// strength (`l` squared of the stiffness term), damping and muscle stiffness, as a scale for this step (what another
/// behaviour wrote is scaled; our own last write is scaled again from what was there before it).
fn scale_joints(c: &mut Ctx, scaled: &mut Vec<Option<([f32; 3], [f32; 3])>>, l: f32) {
    let skip: Vec<usize> = ANKLE().iter().chain(crate::nmrs::WRIST().iter()).copied().collect();
    scaled.resize(c.ch.eff.len(), None);
    for (j, e) in c.ch.eff.iter_mut().enumerate() {
        if skip.contains(&j) {
            continue;
        }
        let now = [e.strength, e.damping, e.muscle_stiffness];
        let base = match scaled[j] {
            Some((base, wrote)) if wrote == now => base,
            _ => now,
        };
        let to = [base[0] * l * l, base[1] * l, base[2] * l];
        (e.strength, e.damping, e.muscle_stiffness) = (to[0], to[1], to[2]);
        scaled[j] = Some((base, to));
    }
}

impl Extra for Melee {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if c.ch.rdr2 {
            return self.tick_rdr2(c, dt);
        }
        self.t += dt;
        let up = c.ch.up.normalize_or(Vec3::Z);
        let side = (c.b.part(13).m.d - c.b.part(17).m.d).normalize_or(Vec3::X);
        let fwd = side.cross(up).normalize_or(Vec3::Y);
        // the spot hit, carried along with the part it is on
        let part = (0..21).contains(&self.part).then_some(self.part as usize);
        let (point, part_pos) = (self.point, part.map(|k| c.b.part(k).m.d));
        let offset = *self.offset.get_or_insert_with(|| part_pos.map_or(Vec3::ZERO, |q| point - q));
        let wound = part_pos.map(|q| q + offset);
        if let Some(s) = self.snap.as_mut() {
            if !s.done() {
                s.tick(c, dt);
            }
        }
        if self.t < self.loose_time {
            c.ch.set_stiffness(self.stiffness * (1.0 - self.loose), 1.0, "ub", None);
        }
        if self.t < self.lean_time && self.push != Vec3::ZERO {
            let push = (self.push - up * self.push.dot(up)).normalize_or_zero() * self.lean;
            c.out.force(10, push * 0.7);
            c.out.force(0, push * 0.3);
        }
        let flinching = self.flinch && self.t < self.flinch_time;
        if flinching {
            for &j in SPINE() {
                let d = &mut c.ch.eff[j].desired[0];
                *d += (0.08 - *d) * 0.2;
            }
        }
        for i in 0..2 {
            let l = crate::nmshot::limb(i);
            let to = if flinching && self.guard {
                // guard: hands up in front of the face, a little either side
                let head = c.b.part(12).m.d;
                Some(head + fwd * 0.28 - up * 0.08 + side * if i == 0 { 0.1 } else { -0.1 })
            } else if self.rfw && self.t > self.rfw_delay && self.t < self.rfw_delay + self.rfw_time {
                // reach for the wound: the hand on the far side for an arm hit, else the near one
                let k = part.unwrap_or(10);
                let hand = if (13..17).contains(&k) { 1 } else if (17..21).contains(&k) { 0 } else if (point - c.com).dot(side) > 0.0 { 0 } else { 1 };
                wound.filter(|_| hand == i && part.is_some())
            } else {
                None
            };
            if let Some(to) = to {
                let s = if flinching && self.guard { self.stiffness * 1.5 } else { self.stiffness };
                c.ch.set_stiffness(s, 1.0, ["ul", "ur"][i], None);
                crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[delirium_data::kv!(0 int)], to, 0.0, 0.0, 1.0 - 2.0 * i as f32, 1.0, Vec3::ZERO, None);
            }
        }
    }

    fn done(&self) -> bool {
        // RDR2's runs until it is stopped
        if self.r2.is_some() {
            return false;
        }
        let end = self.loose_time.max(self.lean_time).max(self.flinch_time).max(if self.rfw { self.rfw_delay + self.rfw_time } else { 0.0 });
        self.t > end && self.snap.as_ref().is_none_or(Snap::done)
    }

    /// The RDR2 core's useHeadLook: headLook at the spot hit when the reach starts.
    fn lines(&mut self) -> Vec<String> {
        let mut out = std::mem::take(&mut self.out);
        out.extend(self.look.take().map(|p| format!("headLook start=true pos={},{},{}", p.x, p.y, p.z)));
        out
    }

    /// The RDR2 core: a message while it runs is taken in (a new blow with hitNew), its clock and state kept.
    fn hit(&mut self, p: &mut P) -> Option<Result<(), Error>> {
        let s = self.r2.take()?;
        match Melee::new(p) {
            Ok(n) => {
                *self = Melee { r2: Some(s), said: self.said, ..n };
                Some(Ok(()))
            }
            Err(e) => {
                self.r2 = Some(s);
                Some(Err(e))
            }
        }
    }

    /// The RDR2 core's deactivate: the hip pitch let go at once (unless staggerFall runs), the knee bend at 4.
    fn stop(&mut self, c: &mut Ctx) {
        if self.r2.is_none() || !c.ch.rdr2 {
            return;
        }
        if !c.extras_on.contains(&"staggerFall") {
            c.out.dbm_crouch.push(Crouch::ReleaseHip(1e5));
        }
        c.out.dbm_crouch.push(Crouch::ReleaseKnee(4.0));
    }
}

/// A set of joint-axis bends laid over whatever the balancer, the shot or another behaviour wants, scaled by a
/// weight each tick: they go into the joints' offsets (added where the joints are driven, cleared every tick), so
/// they stack with anything else and never build up.
struct Bends { ch: Vec<(usize, usize, f32)> }

impl Bends {
    fn new(ch: Vec<(usize, usize, f32)>) -> Bends {
        Bends { ch }
    }

    fn apply(&mut self, c: &mut Ctx, w: f32, stiffness: f32) {
        for &(j, k, amt) in &self.ch {
            c.ch.eff[j].offset[k] += amt * w;
            if w > 0.0 && k == 0 && stiffness > 0.0 {
                c.ch.eff[j].set_stiffness(stiffness, 1.0, None);
            }
        }
    }
}

/// Which way a knee bends: towards whichever of its limits is further from straight.
fn knee_dir(c: &Ctx, j: usize) -> f32 {
    let l = c.ch.limits(j);
    if l[0].abs() > l[1].abs() { -1.0 } else { 1.0 }
}

/// Drives a joint axis to `x` of its range (-1 one limit, 0 the middle, 1 the other), through its offset.
fn drive(c: &mut Ctx, j: usize, k: usize, x: f32) {
    let l = c.ch.limits(j);
    let (lo, hi) = (l[2 * k], l[2 * k + 1]);
    let to = (hi - lo) * 0.5 * x.clamp(-1.0, 1.0) + (hi + lo) * 0.5;
    let e = &mut c.ch.eff[j];
    e.offset[k] += to - e.desired[k];
}

/// The side a part is on: 0 left (leg 1..=3, arm 13..=16), 1 right (leg 4..=6, arm 17..=20), None otherwise.
fn part_side(part: i32) -> Option<usize> {
    match part {
        1..=3 | 13..=16 => Some(0),
        4..=6 | 17..=20 => Some(1),
        _ => None,
    }
}

/// A quick rise, a short hold and a slower return: 0 before `onset`, 1 a quarter of the way through `period`,
/// held to halfway, back to 0 at its end.
fn swell(t: f32, onset: f32, period: f32) -> f32 {
    let x = (t - onset) / period.max(0.01);
    if !(0.0..1.0).contains(&x) { 0.0 } else if x < 0.25 { x / 0.25 } else if x < 0.5 { 1.0 } else { (1.0 - x) / 0.5 }
}

/// RDR2's shot from behind: a bullet in the back arches him. The spine bends back by sfbSpineAmount (spread over its
/// joints), the neck by sfbNeckAmount, the hips push forward (sfbHipAmount) and the knees give a little
/// (sfbKneeAmount, after sfbKneesOnset); after sfbArmsOnset the arms go loose so they fling with it. It swells over
/// sfbPeriod; sfbNoiseGain adds a tremble.
pub struct FromBehind {
    period: f32, arms_onset: f32, knees_onset: f32, noise: f32, stiffness: f32, amounts: [f32; 4],
    t: f32, trunk: Option<Bends>, knees: Option<Bends>, rdr2: SfbRdr2,
}

/// RDR2's from-behind parameters (its defaults: spine 4, neck 1, hips 1, knees 0.05, force balance 0.3 s).
#[derive(Clone, Copy)]
struct SfbRdr2 { period: f32, amounts: [f32; 4], arms_onset: f32, knees_onset: f32, noise: f32, force_balance: f32 }

impl FromBehind {
    fn new(p: &mut P) -> Result<FromBehind, Error> {
        Ok(FromBehind {
            amounts: [p.f("sfbSpineAmount", 0.5)?, p.f("sfbNeckAmount", 0.3)?, p.f("sfbHipAmount", 0.25)?, p.f("sfbKneeAmount", 0.15)?],
            period: p.f("sfbPeriod", 0.7)?.max(0.05), arms_onset: p.f("sfbArmsOnset", 0.0)?.max(0.0),
            knees_onset: p.f("sfbKneesOnset", 0.0)?.max(0.0), noise: p.f("sfbNoiseGain", 0.0)?.max(0.0),
            stiffness: p.f("stiffness", 10.0)?, t: 0.0, trunk: None, knees: None,
            rdr2: SfbRdr2 {
                period: p.f("sfbPeriod", 0.7)?, arms_onset: p.f("sfbArmsOnset", 0.0)?, knees_onset: p.f("sfbKneesOnset", 0.0)?,
                noise: p.f("sfbNoiseGain", 0.0)?, force_balance: p.f("sfbForceBalancePeriod", 0.3)?,
                amounts: [p.f("sfbSpineAmount", 4.0)?, p.f("sfbNeckAmount", 1.0)?, p.f("sfbHipAmount", 1.0)?, p.f("sfbKneeAmount", 0.05)?],
            },
        })
    }
}

impl Extra for FromBehind {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if c.ch.rdr2 {
            return self.tick_rdr2(c, dt);
        }
        self.t += dt;
        let [spine, neck, hip, knee] = self.amounts;
        let trunk = self.trunk.get_or_insert_with(|| {
            let mut v: Vec<_> = SPINE().iter().map(|&j| (j, 0, -spine / 4.0)).collect();
            v.extend(NECK().iter().map(|&j| (j, 0, -neck / 2.0)));
            if !c.ch.rdr2 {
                v.extend(HIP().iter().map(|&j| (j, 0, -hip)));
            }
            Bends::new(v)
        });
        let w = swell(self.t, 0.0, self.period);
        let w = if self.noise > 0.0 && w > 0.0 { w * (1.0 + self.noise * 0.2 * (c.ch.random() - 0.5)) } else { w };
        trunk.apply(c, w, self.stiffness);
        let wk = swell(self.t, self.knees_onset, self.period);
        if c.ch.rdr2 {
            // RDR2's from-behind module asks the balancer (owner 6) to bend the knees and pitch the hips
            c.out.dbm_owned.push((6, -knee * wk, -hip * w));
        } else {
            let knees = self.knees.get_or_insert_with(|| Bends::new(KNEE().iter().map(|&j| (j, 0, knee * knee_dir(c, j))).collect()));
            knees.apply(c, wk, 0.0);
        }
        if self.t > self.arms_onset && self.t < self.arms_onset + self.period * 0.5 {
            c.ch.set_stiffness(self.stiffness * 0.4, 1.0, "ua", None);
        }
    }

    fn done(&self) -> bool {
        self.t > self.period + self.knees_onset.max(self.arms_onset)
    }

    fn stop(&mut self, c: &mut Ctx) {
        // RDR2 lets the hips and knees go at 1 rad/s
        if c.ch.rdr2 {
            c.out.dbm_crouch.extend([Crouch::ReleaseHip(1.0), Crouch::ReleaseKnee(1.0)]);
        }
    }
}

/// RDR2's shot in the guts: he folds over the wound. The spine hunches forward by sigSpineAmount and the neck by
/// sigNeckAmount and the knees bend a little (sigKneeAmount). It comes in over a fifth of a second, holds for
/// sigPeriod and eases off over sigBlend. (The hands are reachForWound's, with reachWithOneHand false.)
pub struct InGuts {
    spine: f32, neck: f32, knee: f32, period: f32, blend: f32, stiffness: f32, t: f32, bends: Option<Bends>, rdr2: SigRdr2,
}

/// RDR2's in-the-guts parameters (its defaults: spine 2, neck 1, hips 1, knees 0.05 over 2 s, no force balance).
#[derive(Clone, Copy)]
struct SigRdr2 { spine: f32, neck: f32, hip: f32, knee: f32, period: f32, knees_onset: f32, force_balance: f32 }

impl InGuts {
    fn new(p: &mut P) -> Result<InGuts, Error> {
        Ok(InGuts {
            spine: p.f("sigSpineAmount", 0.6)?, neck: p.f("sigNeckAmount", 0.25)?, knee: p.f("sigKneeAmount", 0.15)?,
            period: p.f("sigPeriod", 1.5)?.max(0.0), blend: p.f("sigBlend", 0.5)?.max(0.05), stiffness: p.f("stiffness", 10.0)?,
            t: 0.0, bends: None,
            rdr2: SigRdr2 {
                spine: p.f("sigSpineAmount", 2.0)?, neck: p.f("sigNeckAmount", 1.0)?, hip: p.f("sigHipAmount", 1.0)?,
                knee: p.f("sigKneeAmount", 0.05)?, period: p.f("sigPeriod", 2.0)?, knees_onset: p.f("sigKneesOnset", 0.0)?,
                force_balance: p.f("sigForceBalancePeriod", 0.0)?,
            },
        })
    }

    fn weight(&self) -> f32 {
        if self.t < 0.2 { self.t / 0.2 } else if self.t < 0.2 + self.period { 1.0 } else { (1.0 - (self.t - 0.2 - self.period) / self.blend).max(0.0) }
    }
}

impl Extra for InGuts {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if c.ch.rdr2 {
            return self.tick_rdr2(c, dt);
        }
        self.t += dt;
        let w = self.weight();
        let (spine, neck, knee) = (self.spine, self.neck, self.knee);
        let bends = self.bends.get_or_insert_with(|| {
            let mut v: Vec<_> = SPINE().iter().map(|&j| (j, 0, spine / 4.0)).collect();
            v.extend(NECK().iter().map(|&j| (j, 0, neck / 2.0)));
            if !c.ch.rdr2 {
                v.extend(KNEE().iter().map(|&j| (j, 0, knee * knee_dir(c, j))));
                v.extend(HIP().iter().map(|&j| (j, 0, knee * 0.5)));
            }
            Bends::new(v)
        });
        bends.apply(c, w, self.stiffness);
        if c.ch.rdr2 {
            // RDR2's in-guts module asks the balancer (owner 7)
            c.out.dbm_owned.push((7, -knee * w, knee * 0.5 * w));
        }
    }

    fn done(&self) -> bool {
        self.t > 0.2 + self.period + self.blend
    }

    fn stop(&mut self, c: &mut Ctx) {
        // RDR2 lets the hips and knees go at once
        if c.ch.rdr2 {
            c.out.dbm_crouch.extend([Crouch::ReleaseHip(f32::INFINITY), Crouch::ReleaseKnee(f32::INFINITY)]);
        }
    }
}

/// RDR2's injured leg: a hit leg first jerks up (hip and knee flex by legInjuryLift over legLiftTime), then for
/// legInjuryTime it is weak (its strength down by legInjury, the knee bent by legLimpBend, the hip by
/// legInjuryHipPitch) while the spine bends forward (legInjurySpineBend) and he sinks (legSag) and sags towards it
/// (legLean); then it
/// fades over legBlend. The leg is bodyPart's (1..=6), or leftLeg's.
pub struct InjuredLeg {
    side: usize, lift_time: f32, lift: f32, time: f32, injury: f32, hip_pitch: f32, spine_bend: f32, limp: f32,
    lean: f32, sag: f32, blend: f32, stiffness: f32, t: f32, lift_b: Option<Bends>, limp_b: Option<Bends>,
    rdr2: LegRdr2, spine: f32, forced: f32, left: f32, started: bool,
}

/// RDR2's injured-leg parameters (its defaults: the hurt foot steps at once; after 0.3 s, for 0.4 s, the leg 0.3
/// loose, the spine bending to 0.1 at 0.5 rad/s; no limp).
#[derive(Clone, Copy)]
struct LegRdr2 {
    force_step: bool, collapse: f32, bend_rate: f32, bend_rate_spine: f32, straighten: f32, straighten_rate: f32,
    straighten_rate_spine: f32,
}

impl InjuredLeg {
    fn new(p: &mut P) -> Result<InjuredLeg, Error> {
        let part = p.i("bodyPart", -1)?;
        let left = p.b("leftLeg", true)?;
        Ok(InjuredLeg {
            side: part_side(part).unwrap_or(if left { 0 } else { 1 }),
            lift_time: p.f("legLiftTime", 0.0)?.max(0.0), lift: p.f("legInjuryLift", 0.0)?,
            time: p.f("legInjuryTime", 0.4)?.max(0.0), injury: p.f("legInjury", 0.3)?.clamp(0.0, 1.0),
            hip_pitch: p.f("legInjuryHipPitch", 0.0)?, spine_bend: p.f("legInjurySpineBend", 0.1)?, limp: p.f("legLimpBend", 0.0)?,
            lean: p.f("legLean", 15.0)?, sag: p.f("legSag", 0.04)?.max(0.0), blend: p.f("legBlend", 0.5)?.max(0.05), stiffness: p.f("stiffness", 10.0)?,
            t: 0.0, lift_b: None, limp_b: None,
            rdr2: LegRdr2 {
                force_step: p.b("legForceStep", true)?, collapse: p.f("timeBeforeCollapseWoundLeg", 0.3)?,
                bend_rate: p.f("limpBendRate", 0.5)?, bend_rate_spine: p.f("limpBendRateSpine", 0.5)?,
                straighten: p.f("limpStraightenMult", 0.2)?, straighten_rate: p.f("limpStraightenRate", 0.7)?,
                straighten_rate_spine: p.f("limpStraightenRateSpine", 0.7)?,
            },
            spine: 0.0, forced: 0.0, left: 0.0, started: false,
        })
    }
}

impl Extra for InjuredLeg {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if c.ch.rdr2 {
            return self.tick_rdr2(c, dt);
        }
        self.t += dt;
        let s = self.side;
        // the RDR2 core's balancer limps on the hurt leg while it's weak (ignored by GTA IV's)
        let weak = self.t > self.lift_time && self.t < self.lift_time + self.time + self.blend;
        c.out.dbm_limp = Some((s, if weak { self.injury } else { 0.0 }));
        let (hip, knee) = (HIP()[s], KNEE()[s]);
        let (lift, limp, pitch, bend) = (self.lift, self.limp, self.hip_pitch, self.spine_bend);
        let kd = knee_dir(c, knee);
        let lift_b = self.lift_b.get_or_insert_with(|| Bends::new(vec![(hip, 0, lift), (knee, 0, 2.0 * lift * kd)]));
        lift_b.apply(c, swell(self.t, 0.0, self.lift_time * 2.0), 0.0);
        // the RDR2 core's balancer bends the hurt knee (its limp) and pitches the hips when asked; GTA IV's doesn't
        let rdr2 = c.ch.rdr2;
        let limp_b = self.limp_b.get_or_insert_with(|| {
            let mut v = if rdr2 { vec![] } else { vec![(knee, 0, limp * kd), (hip, 0, pitch)] };
            v.extend(SPINE().iter().map(|&j| (j, 0, bend / 4.0)));
            Bends::new(v)
        });
        let t = self.t - self.lift_time;
        let w = if t < 0.0 { 0.0 } else if t < 0.15 { t / 0.15 } else if t < self.time { 1.0 } else { (1.0 - (t - self.time) / self.blend).max(0.0) };
        limp_b.apply(c, w, 0.0);
        if rdr2 {
            c.out.dbm_owned.push((6, -self.sag * w, pitch * w));
        }
        if w > 0.0 {
            // the weak leg: its strength down by legInjury (stiffness by its square root)
            let k = (1.0 - self.injury * w).max(0.05).sqrt();
            c.ch.set_stiffness(self.stiffness * k, 1.0, ["ll", "lr"][s], None);
            // the balancer sinks him a little onto the weak leg
            if self.sag > 0.0 {
                c.out.dbm_leg_offset = Some(-self.sag * w);
            }
            if self.lean != 0.0 {
                let up = c.ch.up.normalize_or(Vec3::Z);
                let foot = c.b.part([3, 6][s]).m.d - c.com;
                let towards = (foot - up * foot.dot(up)).normalize_or_zero();
                c.out.force(10, towards * self.lean * w);
            }
        }
    }

    fn done(&self) -> bool {
        if self.started {
            return self.left <= 0.0 && self.t > self.rdr2.collapse;
        }
        self.t > self.lift_time + self.time + self.blend
    }

    fn stop(&mut self, c: &mut Ctx) {
        if self.started {
            // RDR2's release: the hip pitch at once, the spine, the leg's looseness, the limp and the forced foot
            c.out.dbm_crouch.push(Crouch::ReleaseHip(f32::INFINITY));
            c.out.dbm_force_step = Some(None);
            for &j in SPINE().iter() {
                c.ch.eff[j].desired[0] = 0.0;
            }
            for j in [HIP()[self.side], KNEE()[self.side], ANKLE()[self.side]] {
                c.ch.eff[j].injury = 0.0;
            }
        }
        if self.sag > 0.0 {
            c.out.dbm_leg_offset = Some(0.0);
        }
        c.out.dbm_limp = Some((self.side, 0.0));
    }
}

impl InjuredLeg {
    /// RDR2's injured leg: the hurt foot steps at once (for 0.2 s); after timeBeforeCollapseWoundLeg, for
    /// legInjuryTime, the leg goes legInjury loose, the balancer pitches the hips (legInjuryHipPitch, owner 7, at
    /// limpBendRate) and the spine bends toward legInjurySpineBend on every joint at limpBendRateSpine. With
    /// legLimpBend over 0 the balancer limps: half as loose and the straighten share while the leg swings, all of it
    /// while it carries him. (Its lift, legLiftTime, is off by default and not here.)
    fn tick_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        let s = self.side;
        let r = self.rdr2;
        if !self.started {
            self.started = true;
            self.left = self.time;
            if r.force_step {
                c.out.dbm_force_step = Some(Some(s));
                self.forced = 0.2;
            }
            self.spine = SPINE().iter().map(|&j| c.ch.actual(j, c.b.frames)[0]).sum::<f32>() / 4.0;
        }
        if self.forced > 0.0 {
            self.forced -= dt;
            if self.forced <= 0.0 {
                c.out.dbm_force_step = Some(None);
            }
        }
        let leg = [HIP()[s], KNEE()[s], ANKLE()[s]];
        let injured = self.t > r.collapse && self.left > 0.0 && self.time > 0.0;
        c.out.dbm_limp = Some((s, 0.0));
        if injured {
            let (mut loose, mut pitch, mut prate, mut bend, mut brate) =
                (self.injury, self.hip_pitch, r.bend_rate, self.spine_bend, r.bend_rate_spine);
            if self.limp > 0.0 {
                c.out.dbm_limp = Some((s, self.limp));
                // while the hurt leg carries him (its foot down, the other's up) all of it, else the straighten share
                let feet = [c.b.doll.collided(&[3]), c.b.doll.collided(&[6])];
                if !(feet[s] && !feet[1 - s]) {
                    (loose, pitch, prate, bend, brate) = (0.5 * self.injury, r.straighten * self.hip_pitch, r.straighten_rate,
                                                          r.straighten * self.spine_bend, r.straighten_rate_spine);
                }
            }
            for j in leg {
                c.ch.eff[j].injury = loose;
            }
            c.out.dbm_crouch.push(Crouch::Hip(7, -pitch.clamp(-2.0, 2.0), prate));
            self.spine += (bend - self.spine).clamp(-brate * dt, brate * dt);
            for &j in SPINE().iter() {
                c.ch.eff[j].desired[0] = self.spine;
            }
            self.left -= dt;
        } else {
            c.out.dbm_crouch.push(Crouch::ReleaseHip(f32::INFINITY));
            for j in leg {
                c.ch.eff[j].injury = 0.0;
            }
        }
        self.t += dt;
    }
}

/// RDR2's injured arm: a hit arm knocks that shoulder back and down. For injuredArmTime a yaw torque (hipYaw, N m)
/// turns the trunk with the bullet and a roll torque (hipRoll, N m) drops the hit shoulder, fading out; the arm hangs
/// loose (its stiffness down by armLoose) for armLooseTime. The arm is bodyPart's (13..=20), or leftArm's.
pub struct InjuredArm {
    side: usize, dir: Vec3, time: f32, yaw: f32, roll: f32, loose: f32, loose_time: f32, stiffness: f32, t: f32,
    rdr2: ArmRdr2,
}

/// RDR2's injured-arm parameters (its defaults: 0.25 s; a forced step and a turn below 0.8 m/s; full effect below
/// 1 m/s, none above 5).
#[derive(Clone, Copy)]
struct ArmRdr2 { force_step: bool, step_turn: bool, vel_scales: bool, vel_start: f32, vel_end: f32, vel_force_step: f32 }

impl InjuredArm {
    fn new(p: &mut P) -> Result<InjuredArm, Error> {
        let part = p.i("bodyPart", -1)?;
        let left = p.b("leftArm", true)?;
        Ok(InjuredArm {
            side: part_side(part).unwrap_or(if left { 0 } else { 1 }), dir: p.v("bulletDir", Vec3::ZERO)?,
            time: p.f("injuredArmTime", 0.25)?.max(0.0), yaw: p.f("hipYaw", 80.0)?, roll: p.f("hipRoll", 0.0)?,
            loose: p.f("armLoose", 0.7)?.clamp(0.0, 1.0), loose_time: p.f("armLooseTime", 1.0)?.max(0.0),
            stiffness: p.f("stiffness", 10.0)?, t: 0.0,
            rdr2: ArmRdr2 {
                force_step: p.b("forceStep", true)?, step_turn: p.b("stepTurn", true)?, vel_scales: p.b("velScales", true)?,
                vel_start: p.f("velMultiplierStart", 1.0)?, vel_end: p.f("velMultiplierEnd", 5.0)?,
                vel_force_step: p.f("velForceStep", 0.8)?,
            },
        })
    }
}

impl InjuredArm {
    /// RDR2's injured arm, for injuredArmTime: both clavicles thrown to their limits (the hit side one way, the other
    /// the other, by a hit from in front or behind), a step with the same-side foot and a turn of the balancer across
    /// the bullet, all only when slow (below velForceStep). Its velocity scale (velMultiplierStart..End) only scales its pelvis yaw and roll offsets (hipYaw, hipRoll), which aren't here.
    fn tick_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        let r = self.rdr2;
        let first = self.t == 0.0;
        self.t += dt;
        if self.t > self.time {
            if self.t - dt <= self.time {
                c.out.dbm_turn = Some(Vec3::ZERO);
                c.out.dbm_force_step = Some(None);
            }
            return;
        }
        let up = c.up;
        let v = c.vel.length();
        let back = { let b = c.m.c; b - up * b.dot(up) };
        let dir = self.dir - up * self.dir.dot(up);
        let front = back.dot(dir) >= 0.0;
        if r.force_step && v <= r.vel_force_step {
            if first {
                c.out.dbm_force_step = Some(Some(self.side));
            }
        } else {
            c.out.dbm_force_step = Some(None);
        }
        let (own, other) = (crate::nmshot::limb(self.side).e[0], crate::nmshot::limb(1 - self.side).e[0]);
        let (lo, hi) = (c.ch.limits(own), c.ch.limits(other));
        c.ch.eff[own].desired[0] = if front { lo[0] } else { lo[1] };
        c.ch.eff[own].desired[1] = lo[2];
        c.ch.eff[other].desired[0] = if front { hi[1] } else { hi[0] };
        c.ch.eff[other].desired[1] = if front { hi[3] } else { hi[2] };
        if r.step_turn && (!r.vel_scales || v <= r.vel_force_step) {
            let d = if self.side == 0 { dir } else { -dir };
            let heading = d.normalize_or_zero().cross(up);
            if heading != Vec3::ZERO {
                c.out.dbm_turn = Some(heading * 0.5);
            }
        }
    }
}

impl Extra for InjuredArm {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if c.ch.rdr2 {
            return self.tick_rdr2(c, dt);
        }
        self.t += dt;
        let up = c.ch.up.normalize_or(Vec3::Z);
        let r = c.b.part([13, 17][self.side]).m.d - c.com;
        if self.t < self.time {
            let w = 1.0 - self.t / self.time.max(0.01);
            // yaw: the way that carries the hit shoulder along the bullet (from in front if none given)
            let side = (c.b.part(13).m.d - c.b.part(17).m.d).normalize_or(Vec3::X);
            let push = if self.dir == Vec3::ZERO { up.cross(side) } else { self.dir };
            let push = (push - up * push.dot(up)).normalize_or_zero();
            let yaw = up * r.cross(push).dot(up).signum() * self.yaw * w;
            // roll: the way that drops the hit shoulder
            let roll = r.cross(-up).normalize_or_zero() * self.roll * w;
            c.out.torque(10, (yaw + roll) * 0.7);
            c.out.torque(0, (yaw + roll) * 0.3);
        }
        if self.t < self.loose_time {
            c.ch.set_stiffness(self.stiffness * (1.0 - self.loose), 1.0, ["ul", "ur"][self.side], None);
        }
    }

    fn done(&self) -> bool {
        self.t > self.time.max(self.loose_time)
    }
}

/// A point and a direction carried along with a part: kept in the part's own frame.
#[derive(Clone, Copy)]
struct OnPart { part: usize, pos: Vec3, dir: Vec3 }

impl OnPart {
    fn new(c: &Ctx, part: usize, at: Vec3, dir: Vec3) -> OnPart {
        let m = c.b.part(part).m;
        let local = |v: Vec3| Vec3::new(v.dot(m.a), v.dot(m.b), v.dot(m.c));
        OnPart { part, pos: local(at - m.d), dir: local(dir) }
    }

    fn now(&self, c: &Ctx) -> (Vec3, Vec3) {
        let m = c.b.part(self.part).m;
        let world = |v: Vec3| m.a * v.x + m.b * v.y + m.c * v.z;
        (world(self.pos) + m.d, world(self.dir).normalize_or_zero())
    }
}

/// Clavicle (lean1, lean2), shoulder (lean1, lean2, twist) and elbow angles, inside their limits, that put the hand
/// nearest `target`: a coordinate search over the body's own forward kinematics from its current pose, starting at
/// `start` and kept close to it, so the answer moves smoothly from tick to tick. (A two-bone solve can't find poses
/// that need the clavicle and the shoulder's twist, such as a hand on the chest.) Returns the angles and how far
/// the hand would be from the target.
fn arm_solve(c: &Ctx, l: crate::nmshot::Limb, target: Vec3, start: [f32; 6]) -> ([f32; 6], f32) {
    let (cl, sh, el, hand) = (l.e[0], l.e[1], l.e[2], l.p[delirium_data::kv!(0 int)]);
    let mut ang: Vec<[f32; 3]> = (0..c.ch.joints.len()).map(|j| c.ch.actual(j, c.b.frames)).collect();
    let root = c.b.frames[0];
    let base = c.ch.pose(c.b.build, root, &ang)[hand].translation;
    let now = c.b.frames[hand].translation;
    let (lc, ls, le) = (c.ch.limits(cl), c.ch.limits(sh), c.ch.limits(el));
    let lo = [lc[0], lc[2], ls[0], ls[2], ls[4], le[0]];
    let hi = [lc[1], lc[3], ls[1], ls[3], ls[5], le[1]];
    let twist = ang[cl][2];
    let mut hand_at = |x: [f32; 6]| {
        ang[cl] = [x[0], x[1], twist];
        ang[sh] = [x[2], x[3], x[4]];
        ang[el] = [x[5], 0.0, 0.0];
        c.ch.pose(c.b.build, root, &ang)[hand].translation - base + now
    };
    let mut cost = |x: [f32; 6]| {
        let keep: f32 = (0..6).map(|k| (x[k] - start[k]).powi(2)).sum();
        (hand_at(x) - target).length_squared() + 0.002 * keep
    };
    let mut x: [f32; 6] = std::array::from_fn(|k| start[k].clamp(lo[k], hi[k]));
    let mut best = cost(x);
    for step in [0.25, 0.12, 0.06, 0.03] {
        for _ in 0..3 {
            let mut moved = false;
            for k in 0..6 {
                for d in [step, -step] {
                    let mut y = x;
                    y[k] = (y[k] + d).clamp(lo[k], hi[k]);
                    let f = cost(y);
                    if f < best {
                        (best, x, moved) = (f, y, true);
                    }
                }
            }
            if !moved {
                break;
            }
        }
    }
    let miss = (hand_at(x) - target).length();
    (x, miss)
}

/// RDR2's reach for wound, a behaviour of its own there: after timeBeforeReachForWound one hand (or, without
/// reachWithOneHand, both) goes to the wound and holds it for grabHoldTime, the palm on the body and pulled onto
/// it, while the body bends towards the wound: a little hunch and a twist round to it for the trunk, and for a
/// thigh (allowInjuredThighReach) the hips, knees and spine bend down so the hand gets there. The bend comes in over
/// bendTimeUp and goes over bendTimeDown. The hand is the far one for an arm wound, the same side's for a leg, the
/// nearer one for the trunk.
pub struct ReachForWound {
    point: Vec3, part: i32, bullet: Vec3, wait: f32, hold: f32, one_hand: bool, thigh: bool, lower_leg: bool, arm_stiffness: f32,
    spine_bend: f32, hip_bend: f32, knee_bend: f32, twist: f32, up: f32, down: f32, stiffness: f32,
    t: f32, wound: Option<OnPart>, hands: [bool; 2], sign: [f32; 2], bends: Option<Bends>, arm: [Option<[f32; 6]>; 2],
    rdr2: RfwRdr2, r2: Option<Box<Rfw2>>,
}

/// reachForWound's RDR2-core settings (RDR2's defaults; its armStiffness is -10, which stiffens the arm only on the
/// ground). Rates are [knee, hip, spine].
#[derive(Clone, Copy)]
struct RfwRdr2 {
    falling: i32, floor: i32, falling_one: i32, new_hit_reset: bool, rate_up: [f32; 3], rate_down: [f32; 3],
    bend_before: f32, twist_time: f32, leg_stiffness: f32, arm_stiffness: f32,
}

impl ReachForWound {
    fn new(p: &mut P) -> Result<ReachForWound, Error> {
        let arm_given = p.get("armStiffness").is_some();
        let arm_stiffness = p.f("armStiffness", 10.0)?;
        Ok(ReachForWound {
            point: p.v("hitPoint", Vec3::ZERO)?, part: p.i("hitPart", -1)?, bullet: p.v("bulletVel", Vec3::ZERO)?,
            wait: p.f("timeBeforeReachForWound", 0.2)?.max(0.0), hold: p.f("grabHoldTime", 2.0)?.max(0.0),
            one_hand: p.b("reachWithOneHand", true)?, thigh: p.b("allowInjuredThighReach", true)?,
            lower_leg: p.b("allowInjuredLowerLegReach", false)?, arm_stiffness,
            spine_bend: p.f("bend2spine", 0.7)?, hip_bend: p.f("bend2hip", 0.6)?, knee_bend: p.f("bend2leg", 0.6)?,
            twist: p.f("spineTwistMag", 0.2)?, up: p.f("bendTimeUp", 0.4)?.max(0.01), down: p.f("bendTimeDown", 0.5)?.max(0.01),
            stiffness: p.f("spineStiffness", 12.0)?,
            t: 0.0, wound: None, hands: [false; 2], sign: [1.0; 2], bends: None, arm: [None; 2],
            rdr2: RfwRdr2 {
                falling: p.i("reachFalling", 0)?, floor: p.i("reachOnFloor", 0)?, falling_one: p.i("reachFallingWithOneHand", 3)?,
                new_hit_reset: p.b("newHitReset", true)?,
                rate_up: [p.f("legRateUp", 0.35)?, p.f("hipRateUp", 0.25)?, p.f("spineRateUp", 0.3)?],
                rate_down: [p.f("legRateDown", 0.4)?, p.f("hipRateDown", 0.4)?, p.f("spineRateDown", 0.4)?],
                bend_before: p.f("bendBeforeReach", 0.1)?, twist_time: p.f("spineTwistTime", 0.5)?,
                leg_stiffness: p.f("legStiffness", 10.0)?, arm_stiffness: if arm_given { arm_stiffness } else { -10.0 },
            },
            r2: None,
        })
    }

    /// The hands to use and the body's bend, once, from where the wound is.
    fn aim(&mut self, c: &mut Ctx) {
        let part = self.part as usize;
        let up = c.ch.up.normalize_or(Vec3::Z);
        let side = (c.b.part(13).m.d - c.b.part(17).m.d).normalize_or(Vec3::X);
        let fwd = side.cross(up).normalize_or(Vec3::Y);
        let q = c.b.part(part).m.d;
        let point = if self.point == Vec3::ZERO { q + fwd * if (7..=10).contains(&part) { 0.18 } else { 0.12 } } else { self.point };
        // the body's surface there: out from the part's axis (for the trunk, out from the spine)
        let out = point - q;
        let out = (out - up * out.dot(up)).normalize_or(fwd);
        self.wound = Some(OnPart::new(c, part, point, out));
        let leg = (1..=6).contains(&part);
        let lower = matches!(part, 2 | 3 | 5 | 6);
        let arm = (13..=20).contains(&part);
        let left = (point - c.com).dot(side) > 0.0;
        self.hands = if arm {
            if part_side(self.part) == Some(0) { [false, true] } else { [true, false] }
        } else if leg {
            let ok = if lower { self.lower_leg } else { self.thigh };
            let s = part_side(self.part).unwrap_or(0);
            [ok && s == 0, ok && s == 1]
        } else if self.one_hand {
            [left, !left]
        } else {
            [true, true]
        };
        let (sb, hb, kb, tw) = (self.spine_bend, self.hip_bend, self.knee_bend, self.twist);
        let mut v: Vec<(usize, usize, f32)> = vec![];
        if leg && self.hands.contains(&true) {
            // bend down to the leg: hips and knees (both legs) and the spine forwards
            let more = if lower { 1.5 } else { 1.0 };
            v.extend(SPINE().iter().map(|&j| (j, 0, sb * more / 4.0)));
            v.extend(HIP().iter().map(|&j| (j, 0, hb * more)));
            v.extend(KNEE().iter().map(|&j| (j, 0, kb * more * knee_dir(c, j))));
        } else if !arm {
            // a hunch over the wound and a twist round to it
            v.extend(SPINE().iter().map(|&j| (j, 0, 0.15 / 4.0)));
        }
        if tw != 0.0 && !leg {
            let to = if left { 1.0 } else { -1.0 };
            v.extend(SPINE().iter().map(|&j| (j, 2, tw * to / 4.0)));
        }
        self.bends = Some(Bends::new(v));
    }

    /// One hand to the wound: the clavicle forward, the arm by IK to just off the wound, the palm turned onto it,
    /// and a pull once close (as Delirium's shot reaches).
    fn reach(&mut self, c: &mut Ctx, i: usize, t: Vec3, n: Vec3, dt: f32) {
        let l = crate::nmshot::limb(i);
        let part = self.wound.map_or(0, |w| w.part);
        for j in [l.e[0], l.e[1], l.e[2]] {
            c.ch.eff[j].oppose_gravity = 1.0;
        }
        c.ch.set_stiffness(self.arm_stiffness, 1.0, ["ul", "ur"][i], None);
        // aim just off the wound, closing in as the hand gets near
        let hand = c.b.part(l.p[delirium_data::kv!(0 int)]).m.d;
        let mut dh = hand - t;
        let s = dh.dot(n);
        dh -= n * if s < 0.0 { s } else { 0.0 };
        // and from a little above while far, so the hand comes round the hip rather than into it
        let up = c.ch.up.normalize_or(Vec3::Z);
        let t2 = t + n * (dh.length() * 0.5 - 0.05).min(0.25) + up * (dh.length() * 0.5).min(0.15);
        let start = self.arm[i].unwrap_or_else(|| {
            let [k, a, e] = [0, 1, 2].map(|n| c.ch.actual(l.e[n], c.b.frames));
            [k[0], k[1], a[0], a[1], a[2], e[0]]
        });
        let (mut x, mut miss) = arm_solve(c, l, t2, start);
        if self.arm[i].is_none() && miss > 0.08 {
            // the search only finds the nearest pose, so the first time also try from the arm raised forwards and
            // across (once: after that it follows on from where it got to)
            for (sl1, stw) in [(0.6, 0.0), (1.2, -0.6), (0.9, 0.4)] {
                let (y, m) = arm_solve(c, l, t2, [start[0], start[1], sl1, start[3], stw, 1.5]);
                if m + 0.02 < miss {
                    (x, miss) = (y, m);
                }
            }
        }
        self.arm[i] = Some(x);
        (c.ch.eff[l.e[0]].desired[0], c.ch.eff[l.e[0]].desired[1]) = (x[0], x[1]);
        c.ch.eff[l.e[1]].desired = [x[2], x[3], x[4]];
        c.ch.eff[l.e[2]].desired[0] = x[5];
        let side = if i == 0 { 1.0 } else { -1.0 };
        if crate::nmshot::Shot::wrist(c, l, t, n, side) < 0.0 && part != 12 {
            self.sign[i] = -self.sign[i];
        }
        // the pull: once within 0.3 m, a growing impulse draws the palm onto the wound (and the body back onto it)
        let h = c.b.part(l.p[delirium_data::kv!(0 int)]);
        let at = h.m.d - h.m.a * (self.sign[i] * 0.02);
        let d = (t - at).length();
        if d < 0.3 {
            let k = ((0.3 - d) / 0.2).clamp(0.0, 1.0) * 4.0;
            let mut imp = (t - at) * k * k + (c.b.part(part).v - h.v) * (k * 0.2);
            if dt < 1.0 / 60.0 {
                imp *= dt * 60.0;
            }
            let wrist = c.b.joint_pos(l.e[3]);
            c.out.impulse.push((l.p[delirium_data::kv!(0 int)], wrist, imp));
            c.out.impulse.push((part, t, -imp));
        }
    }
}

impl Extra for ReachForWound {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if c.ch.rdr2 {
            return self.tick_rdr2(c, dt);
        }
        self.r2 = None;
        self.t += dt;
        if !(0..21).contains(&self.part) {
            self.part = 8;
        }
        if self.wound.is_none() {
            self.aim(c);
        }
        let t = self.t - self.wait;
        let w = if t < 0.0 { 0.0 } else if t < self.hold { (t / self.up).min(1.0) } else { (1.0 - (t - self.hold) / self.down).max(0.0) };
        if let Some(b) = self.bends.as_mut() {
            b.apply(c, w, self.stiffness);
        }
        if t >= 0.0 && t < self.hold {
            let (at, n) = self.wound.expect("aimed").now(c);
            // two hands: side by side over the wound, one a little higher, so they don't fight for the spot
            let both = self.hands == [true, true];
            let up = c.ch.up.normalize_or(Vec3::Z);
            let across = n.cross(up).normalize_or_zero();
            for i in 0..2 {
                if self.hands[i] && !c.ch.busy[i] {
                    let s = if i == 0 { 1.0 } else { -1.0 };
                    let at = if both { at + across * (0.07 * s * across.dot(c.b.part(13).m.d - c.b.part(17).m.d).signum()) + up * (0.03 * s) } else { at };
                    self.reach(c, i, at, n, dt);
                }
            }
        }
    }

    fn rfw_view(&self) -> Option<crate::nmshot::RfwView> {
        let s = self.r2.as_ref()?;
        Some(crate::nmshot::RfwView { reached: [0, 1].map(|i| s.reaching[i] && s.act[i].reached), on: s.act.map(|a| a.clock - self.wait),
                                      since_stop: s.since_stop })
    }

    fn rfw_release(&mut self, hands: [bool; 2]) {
        if let Some(s) = self.r2.as_mut() {
            s.release = hands;
        }
    }

    fn done(&self) -> bool {
        if let Some(s) = &self.r2 {
            return s.idle(self.wait + self.hold);
        }
        self.t > self.wait + self.hold + self.down
    }

    /// The RDR2 core's bullet intake: a new start while it runs is a new wound for its arms, with the message's
    /// settings.
    fn hit(&mut self, p: &mut P) -> Option<Result<(), Error>> {
        let mut s = self.r2.take()?;
        match ReachForWound::new(p) {
            Ok(n) => {
                s.hits.push((n.point, n.part, n.bullet));
                *self = ReachForWound { r2: Some(s), ..n };
                Some(Ok(()))
            }
            Err(e) => {
                self.r2 = Some(s);
                Some(Err(e))
            }
        }
    }
}

/// RDR2's rate limiter: `current` follows `target` at `rate` per s.
#[derive(Clone, Copy, Default)]
struct Limiter { target: f32, current: f32, rate: f32 }

impl Limiter {
    /// One step; returns how far it moved.
    fn step(&mut self, dt: f32) -> f32 {
        let d = (self.target - self.current).clamp(-self.rate * dt, self.rate * dt);
        self.current += d;
        d
    }
}

/// A wound one hand goes to: where (carried with its part), the leg it is on (0 left, 1 right), an arm or the head.
#[derive(Clone, Copy)]
struct Wound { on: OnPart, leg: Option<usize>, arm: bool, head: bool }

/// RDR2's arm record (one active and one pending per arm): its wound, the hand's distance from it and how fast that
/// closes, the stuck timer, the reach clock, the leg-wound bend it asks, the hunch rhythm, the hand on the wound (on),
/// a wound not yet started (fresh) and placed this frame (reached); the palm side and arm pose are Delirium's.
#[derive(Clone, Copy)]
struct ArmRec {
    wound: Option<Wound>, dist: f32, rate: f32, stuck: f32, clock: f32, demand: f32, hunch_t: f32, hunch: Limiter,
    late: bool, on: bool, fresh: bool, reached: bool, sign: f32, pose: Option<[f32; 6]>,
}

impl Default for ArmRec {
    fn default() -> ArmRec {
        ArmRec { wound: None, dist: 1000.0, rate: 0.0, stuck: 0.0, clock: 0.0, demand: 0.0, hunch_t: 0.0, hunch: Limiter::default(),
                 late: false, on: false, fresh: false, reached: false, sign: 1.0, pose: None }
    }
}

/// reachForWound's RDR2-core state: the arm records and which arms are in a reach, bullets not yet taken in, the
/// spine twist (its timer, side and limiter), the spine bend for a leg wound and its release, the hunch rhythm's
/// alternation, whether a hit came while down, and his leg length; the hands the shot's releaseWound lets go this
/// step, and the time since each hand's reach last ended (both for the shot's default arms).
#[derive(Clone, Default)]
struct Rfw2 {
    act: [ArmRec; 2], pend: [Option<ArmRec>; 2], reaching: [bool; 2], hits: Vec<(Vec3, i32, Vec3)>,
    twist_t: f32, twist_left: bool, twist: Limiter, bend: Limiter, releasing: bool, toggle: bool, toggle_at: f32,
    hit_down: bool, len: f32, release: [bool; 2], since_stop: [f32; 2],
}

impl Rfw2 {
    /// Nothing left to do: no arm in a reach or waiting for one, no bend to let go of, and no reach ended in the last
    /// 0.4 s (the shot's default arms still read that). (RDR2's behaviour never stops itself; idle, it does nothing.)
    fn idle(&self, window: f32) -> bool {
        !self.reaching.contains(&true) && self.pend.iter().all(Option::is_none) && !self.releasing && self.hits.is_empty()
            && self.act.iter().all(|a| !(a.fresh && a.clock <= window)) && self.since_stop.iter().all(|&t| t >= 0.4)
    }
}

/// RDR2's pull: strength 8 at the start of a reach falling to 4 over 0.3 s; the recoil (the hand first pushed away
/// from the wound by 0.4 of the way, for 0.06 s, at strength 2).
const RFW_PULL: (f32, f32, f32) = (8.0, 4.0, 0.3);
const RFW_RECOIL: (f32, f32, f32) = (0.4, 0.06, 2.0);
/// RDR2's per-character grab strength (ctx+0x2c3c) that scales the head-wound pull: its value isn't known (ours: 1,
/// which makes RDR2's relax-period pull of 0.5 of it match GTA IV's 0.5).
const RFW_GRAB: f32 = 1.0;
/// RDR2's on-the-ground leg bend gains (knee, hip), per unit of bend2leg / bend2hip.
const RFW_GROUND_LEG: (f32, f32) = (6.425121, 5.437921);

impl ReachForWound {
    /// RDR2's reach for wound: each arm runs its own reach. A bullet fills each chosen hand's pending record (taken
    /// at once by a hand not reaching; newHitReset stops a running reach so it restarts on the new wound). A hand
    /// starts once timeBeforeReachForWound has passed, reaches until grabHoldTime more, and stops early if its arm
    /// gets busy or the hand is stuck (not closing on the wound for 0.18 s). While reaching: the pull (8 falling to 4,
    /// within three times its radius), the hunch rhythm once the hand is on the wound, the spine twist round to a
    /// trunk wound (spineTwistMag on every spine joint for spineTwistTime), and for a leg wound out of reach a bend
    /// down to it: upright through the balancer's knee (owner 11) and hip (owner 12) requests, on the ground by the
    /// legs themselves. On the ground it reaches only as reachFalling / reachOnFloor allow, and then has catchFall and
    /// bodyRollUp leave that arm alone.
    fn tick_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        let mut s = self.r2.take().unwrap_or_else(|| {
            let len = |a: usize, b: usize| (c.b.joint_pos(a) - c.b.joint_pos(b)).length();
            Box::new(Rfw2 { hits: vec![(self.point, self.part, self.bullet)], toggle: true, len: len(HIP()[0], KNEE()[0]) + len(KNEE()[0], ANKLE()[0]),
                            since_stop: [100.0; 2], ..Default::default() })
        });
        for (point, part, bullet) in std::mem::take(&mut s.hits) {
            let (point, part) = crate::nmshot::hit_record(c, point, part, bullet);
            self.intake(c, &mut s, point, part);
        }
        let window = self.wait + self.hold;
        for a in s.act.iter_mut() {
            a.clock += dt;
            a.demand = 0.0;
        }
        for t in s.since_stop.iter_mut() {
            *t += dt;
        }
        for i in 0..2 {
            let free = !c.ch.busy[i];
            let a = s.act[i];
            if s.reaching[i] && (a.fresh || a.clock > window || !free) {
                self.stop_arm(c, &mut s, i);
            }
            let a = s.act[i];
            if !s.reaching[i] && a.fresh && a.clock > self.wait && a.clock < window && free {
                let a = &mut s.act[i];
                (a.rate, a.stuck, a.dist, a.on, a.fresh) = (0.0, 0.0, 1000.0, false, false);
                s.bend.current = SPINE().iter().map(|&j| c.ch.actual(j, c.b.frames)[0]).sum::<f32>() * 0.25;
                s.reaching[i] = true;
            }
            // a hand the shot lets go of (releaseWound) is left alone this step
            if s.reaching[i] && !s.release[i] {
                self.update_arm(c, &mut s, i, dt);
            }
        }
        s.release = [false; 2];
        self.bend_rdr2(c, &mut s, dt);
        self.r2 = Some(s);
    }

    /// A bullet in `part` at `point`: the hands it calls for, as pending wounds. An arm's own wound isn't reached by
    /// that arm; a leg wound only with allowInjuredThighReach / allowInjuredLowerLegReach. Which hand for a trunk wound
    /// is Delirium's (the nearer, with reachWithOneHand): RDR2 decides by zones whose test isn't decompiled. Two hands
    /// on the trunk are spread 0.05 leg lengths either side.
    fn intake(&mut self, c: &mut Ctx, s: &mut Rfw2, point: Vec3, part: i32) {
        let part = if (0..21).contains(&part) { part as usize } else { 8 };
        s.hit_down |= !c.upright;
        if self.rdr2.new_hit_reset {
            for a in s.act.iter_mut() {
                a.clock = self.hold + 1e5 + self.wait;
            }
        }
        s.pend = [None; 2];
        let up = c.up;
        let side = (c.b.part(13).m.d - c.b.part(17).m.d).normalize_or(Vec3::X);
        let fwd = side.cross(up).normalize_or(Vec3::Y);
        let q = c.b.part(part).m.d;
        let point = if point == Vec3::ZERO { q + fwd * if (7..=10).contains(&part) { 0.18 } else { 0.12 } } else { point };
        let out = point - q;
        let out = (out - up * out.dot(up)).normalize_or(fwd);
        let leg = (1..=6).contains(&part);
        let arm = (13..=20).contains(&part);
        let head = part == 11 || part == 12;
        let mut ok = [true; 2];
        if arm {
            ok[part_side(part as i32).unwrap_or(0)] = false;
        }
        if leg && !if matches!(part, 2 | 3 | 5 | 6) { self.lower_leg } else { self.thigh } {
            ok = [false; 2];
        }
        if self.one_hand && ok == [true; 2] {
            let left = if leg { part_side(part as i32) == Some(0) } else { (point - c.com).dot(side) > 0.0 };
            ok = [left, !left];
        }
        let spread = ok == [true; 2] && !leg && !arm && !head;
        for i in (0..2).filter(|&i| ok[i]) {
            let at = if spread { point + side * (0.05 * s.len * if i == 0 { 1.0 } else { -1.0 }) } else { point };
            let w = Wound { on: OnPart::new(c, part, at, out), leg: if leg { part_side(part as i32) } else { None }, arm, head };
            let rec = ArmRec { wound: Some(w), fresh: true, ..Default::default() };
            if s.reaching[i] {
                s.pend[i] = Some(rec);
            } else {
                s.act[i] = rec;
            }
        }
        if ok.contains(&true) {
            (s.toggle_at, s.toggle, s.twist_t, s.twist_left, s.twist) = (0.0, true, 0.0, ok[0], Limiter::default());
        }
    }

    /// RDR2's stop: the wrist straight, a leg wound's knee and hip requests let go at the "up" rates and the spine
    /// released, the pending wound taken, and the arm held where it is.
    fn stop_arm(&mut self, c: &mut Ctx, s: &mut Rfw2, i: usize) {
        let l = crate::nmshot::limb(i);
        c.ch.eff[l.e[3]].desired = [0.0; 3];
        if s.act[i].wound.is_some_and(|w| w.leg.is_some()) {
            c.out.dbm_crouch.push(Crouch::ReleaseKnee(self.rdr2.rate_up[0]));
            c.out.dbm_crouch.push(Crouch::Hip(12, 0.0, self.rdr2.rate_up[1]));
            s.releasing = true;
        }
        s.act[i].on = false;
        if let Some(p) = s.pend[i].take() {
            s.act[i] = p;
        }
        s.act[i].reached = false;
        for j in [l.e[0], l.e[1], l.e[2]] {
            c.ch.eff[j].desired = c.ch.actual(j, c.b.frames);
        }
        s.reaching[i] = false;
        s.since_stop[i] = 0.0;
    }

    /// One frame of one hand's reach (RDR2's update).
    fn update_arm(&mut self, c: &mut Ctx, s: &mut Rfw2, i: usize, dt: f32) {
        let r = self.rdr2;
        let Some(w) = s.act[i].wound else { return };
        let l = crate::nmshot::limb(i);
        let hand = c.b.part(l.p[delirium_data::kv!(0 int)]).m.d;
        let (wp, n) = w.on.now(c);
        let since = s.act[i].clock - self.wait;
        let len = s.len;
        let (p0, p1, ramp) = RFW_PULL;
        let mut strength = if since > 0.0 {
            let f = (since / ramp).min(1.0);
            (p0 * (1.0 - f) + p1 * f).max(0.0)
        } else {
            p1
        };
        // on the ground: only as reachFalling (or, lying, reachOnFloor) allows; then catchFall and bodyRollUp leave
        // this arm alone (RDR2's one-hand-by-orientation rule needs its predicted body orientation: both hands here)
        if !c.upright {
            let body: Vec<usize> = std::iter::once(0).chain(7..21).collect();
            let lying = c.b.doll.collided(&body);
            let allow = if lying { r.floor } else { r.falling };
            if !(allow == 1 || allow == 2 && !s.hit_down) {
                return;
            }
            if !lying && (r.falling_one == 1 && i == 1 || r.falling_one == 2 && i == 0) {
                return;
            }
            c.ch.exclude_arms[i] = true;
        }
        // a leg wound out of the upper arm's reach asks a bend down to it, and the hand waits bendBeforeReach
        let mut radius = 0.3;
        if w.leg.is_some() {
            let root = c.b.joint_pos(l.e[1]);
            let upper = (c.b.joint_pos(l.e[2]) - root).length();
            if (wp - root).length() > upper {
                let mut d = (-c.up.dot(wp - root) - upper).clamp(0.0, 0.4);
                if d < 0.4 {
                    d = 0.875 * d + 0.05;
                }
                if !c.upright {
                    d *= 1.1;
                }
                s.act[i].demand = d;
            }
            if since < r.bend_before {
                return;
            }
            if c.upright {
                radius = 0.1;
            }
        }
        let gap = (wp - hand).length();
        let near = if w.leg.is_some() { 0.3 } else { 0.1 };
        let mut share = if gap > near * len { 1.0 - (gap - near * len).min(0.2 * len) * 5.0 / len } else { 1.0 };
        // the hunch rhythm, once the hand is on the wound: the upper spine curls (-0.4) and lets go, then, alone or
        // ahead, alternates slowly
        if gap < 0.12 {
            let t = s.act[i].hunch_t + dt;
            s.act[i].hunch_t = t;
            let mut h = s.act[i].hunch;
            let mut k = -0.4;
            if t < 0.5 {
                (h.target, h.rate) = (if s.act[i].late { -0.2 } else { -0.4 }, 2.0);
            } else {
                (h.target, h.rate) = (0.0, 1.0);
                s.act[i].late = true;
            }
            if t > 1.2 && (!s.reaching[1 - i] || s.act[1 - i].hunch_t <= t) {
                k = -0.8;
                if t >= s.toggle_at {
                    s.toggle = !s.toggle;
                    s.toggle_at = t + if s.toggle { 1.3 } else { 0.8 };
                    s.act[1 - i].hunch_t = t;
                }
                (h.target, h.rate) = if s.toggle { (0.0, 0.15) } else { (-0.4, 0.2) };
            }
            h.step(dt);
            s.act[i].hunch = h;
            for &j in &SPINE()[2..] {
                c.ch.eff[j].desired[0] = k * h.current;
            }
        }
        let hunch = s.act[i].hunch.current;
        // a head or neck wound: the clavicle follows the neck and head, and a softer pull
        let mut clav = None;
        if w.head && w.leg.is_none() {
            let mut sum = [0.0; 3];
            for &j in NECK().iter() {
                let a = c.ch.actual(j, c.b.frames);
                for m in 0..3 {
                    sum[m] += a[m];
                }
            }
            let mut cl = [0.5 * sum[2], 0.5 * sum[1], 0.5 * sum[0]];
            cl[i] = -cl[i];
            clav = Some(cl);
            (strength, share) = (6.0 * RFW_GRAB, 1.0);
        }
        // the target, just off the skin; the recoil first pushes the hand away from it
        let mut t = wp + n * (len * if w.arm { 0.0 } else { 0.05 });
        let (ra, rt, rs) = RFW_RECOIL;
        if since < rt {
            t = hand + (wp - hand) * (-ra * (1.0 - since / rt));
            strength = rs;
        }
        if !c.upright || r.arm_stiffness > 0.0 {
            c.ch.set_stiffness(r.arm_stiffness.abs(), 1.0, ["ul", "ur"][i], None);
        }
        self.place_rdr2(c, s, i, t, n, clav, hunch);
        pull_rdr2(c, l, t, w.on.part, (strength, radius, share), s.act[i].sign, dt);
        s.act[i].on = (hand - wp).length() < 0.3 * len;
        s.act[i].reached = true;
        // the spine twist round to a trunk wound, on every spine joint
        if w.leg.is_none() && !w.arm && !w.head && self.twist > 0.0 {
            s.twist_t += dt;
            let side = if s.twist_left { 1.0 } else { -1.0 };
            (s.twist.target, s.twist.rate) = if s.twist_t < r.twist_time { (side * self.twist, 0.5) } else { (0.0, 0.3) };
            let moved = s.twist.step(dt);
            if moved != 0.0 || s.twist.current.abs() > 0.01 {
                for &j in SPINE().iter() {
                    c.ch.eff[j].desired[2] = s.twist.current;
                }
            }
        }
        // the hand stuck short of the wound for 0.18 s ends the reach
        let a = &mut s.act[i];
        a.rate = (a.dist - gap) / dt.max(1e-4);
        a.dist = gap;
        a.stuck = if gap > 0.11 * len && a.rate > -0.3 && a.rate < 0.2 { a.stuck + dt } else { 0.0 };
        if a.stuck > 0.18 {
            a.clock = self.wait + self.hold + 0.1;
        }
    }

    /// The hand to `t` (`hand_place`), the clavicle with RDR2's hunch terms (or following the neck for a head wound);
    /// a flipped palm side kept for the pull.
    #[allow(clippy::too_many_arguments)]
    fn place_rdr2(&mut self, c: &mut Ctx, s: &mut Rfw2, i: usize, t: Vec3, n: Vec3, clav: Option<[f32; 3]>, hunch: f32) {
        if hand_place(c, i, t, n, &mut s.act[i].pose, clav, hunch) {
            s.act[i].sign = -s.act[i].sign;
        }
    }

    /// The leg-wound bend, from the largest demand: the spine (a limiter on every joint's forward bend) and, upright,
    /// the balancer's knee (owner 11) and hip (owner 12) requests; on the ground the hurt leg's own hip and knee
    /// (RDR2's gains 6.425121 and 5.437921) and a stiffer spine. With no demand the spine lets go at spineRateUp.
    fn bend_rdr2(&mut self, c: &mut Ctx, s: &mut Rfw2, dt: f32) {
        let r = self.rdr2;
        let demand = |leg: usize| s.act.iter().filter(|a| a.wound.is_some_and(|w| w.leg == Some(leg))).map(|a| a.demand).fold(0.0, f32::max);
        let (dl, dr) = (demand(0), demand(1));
        let d = dl.max(dr);
        if d <= 0.0 {
            if s.releasing {
                (s.bend.target, s.bend.rate) = (0.0, r.rate_up[2]);
                s.bend.step(dt);
                for &j in SPINE().iter() {
                    c.ch.eff[j].desired[0] = s.bend.current;
                }
                s.releasing = s.bend.current.abs() >= 0.01;
            }
            return;
        }
        (s.bend.target, s.bend.rate, s.releasing) = (d * self.spine_bend, r.rate_down[2], false);
        s.bend.step(dt);
        for &j in SPINE().iter() {
            c.ch.eff[j].desired[0] = s.bend.current;
        }
        if c.upright {
            c.out.dbm_crouch.push(Crouch::Knee(11, -(d * self.knee_bend), r.rate_down[0]));
            c.out.dbm_crouch.push(Crouch::Hip(12, (d * self.hip_bend).clamp(-2.0, 2.0), r.rate_down[1]));
            return;
        }
        c.ch.set_stiffness(self.stiffness, 1.0, "us", Some(2.0));
        let (gk, gh) = RFW_GROUND_LEG;
        for (leg, dd) in [(0, dl), (1, dr)] {
            if dd > 0.0 {
                let (hip, knee) = (HIP()[leg], KNEE()[leg]);
                for j in [hip, knee] {
                    c.ch.eff[j].set_stiffness(r.leg_stiffness, 1.0, Some(0.5));
                }
                c.ch.eff[knee].desired[0] = dd * self.knee_bend * gk * knee_dir(c, knee);
                c.ch.eff[hip].desired[0] = dd * self.hip_bend * gh;
            }
        }
    }
}

/// RDR2's hand placement (0x2a4733c) without its pull: hand `i` to `t` on a surface facing `n`, by Delirium's arm
/// search (RDR2's IK request needs its limb IK solver, not decompiled), from a little off and above while far,
/// carried on from `pose`; the clavicle with the hunch terms (or `clav`); the palm onto the target within 0.4 m, else
/// the wrist straight. Returns true when the palm side flipped (reachForWound's pull reads it).
#[allow(clippy::too_many_arguments)]
pub(crate) fn hand_place(c: &mut Ctx, i: usize, t: Vec3, n: Vec3, pose: &mut Option<[f32; 6]>, clav: Option<[f32; 3]>, hunch: f32) -> bool {
    let l = crate::nmshot::limb(i);
    for j in [l.e[0], l.e[1], l.e[2]] {
        c.ch.eff[j].oppose_gravity = 1.0;
    }
    let hand = c.b.part(l.p[delirium_data::kv!(0 int)]).m.d;
    let mut dh = hand - t;
    let sn = dh.dot(n);
    dh -= n * if sn < 0.0 { sn } else { 0.0 };
    let t2 = t + n * (dh.length() * 0.5 - 0.05).min(0.25) + c.up * (dh.length() * 0.5).min(0.15);
    let start = pose.unwrap_or_else(|| {
        let [k, a, e] = [0, 1, 2].map(|m| c.ch.actual(l.e[m], c.b.frames));
        [k[0], k[1], a[0], a[1], a[2], e[0]]
    });
    let (mut x, mut miss) = arm_solve(c, l, t2, start);
    if pose.is_none() && miss > 0.08 {
        for (sl1, stw) in [(0.6, 0.0), (1.2, -0.6), (0.9, 0.4)] {
            let (y, m) = arm_solve(c, l, t2, [start[0], start[1], sl1, start[3], stw, 1.5]);
            if m + 0.02 < miss {
                (x, miss) = (y, m);
            }
        }
    }
    *pose = Some(x);
    match clav {
        Some(cl) => c.ch.eff[l.e[0]].desired = cl,
        None => (c.ch.eff[l.e[0]].desired[0], c.ch.eff[l.e[0]].desired[1]) = (x[0] - 0.5 * hunch, x[1] + hunch),
    }
    c.ch.eff[l.e[1]].desired = [x[2], x[3], x[4]];
    c.ch.eff[l.e[2]].desired[0] = x[5];
    if (hand - t).length() < 0.4 {
        let side = if i == 0 { 1.0 } else { -1.0 };
        crate::nmshot::Shot::wrist(c, l, t, n, side) < 0.0 && clav.is_none()
    } else {
        c.ch.eff[l.e[3]].desired = [0.0; 3];
        false
    }
}

/// RDR2's hand pull: within three times `radius` of the target a spring (its pull capped at 0.3 m) and a match of
/// the wound's velocity, `strength` times how far inside; `share` of it on the hand, the rest on the forearm, and the
/// opposite on the wound's part. The hand's point is 0.04 m across the palm (which side: Delirium's wrist rule).
fn pull_rdr2(c: &mut Ctx, l: crate::nmshot::Limb, t: Vec3, part: usize, (strength, radius, share): (f32, f32, f32), sign: f32, dt: f32) {
    let hand_k = l.p[delirium_data::kv!(0 int)];
    let h = c.b.part(hand_k);
    let at = h.m.d - h.m.a * (sign * 0.04);
    let d = (t - at).length();
    if d >= 3.0 * radius {
        return;
    }
    let k = ((3.0 * radius - d) * 0.5 / radius).clamp(0.0, 1.0) * strength;
    let to = if d > 0.3 { (t - at) * (0.3 / d) } else { t - at };
    let mut f = to * (k * k) + (c.b.part(part).v - h.v) * (0.2 * k);
    if dt < 1.0 / 60.0 {
        f *= 60.0 * dt;
    }
    let fore = l.p[2];
    c.out.impulse.push((hand_k, c.b.joint_pos(l.e[3]), f * share));
    c.out.impulse.push((fore, c.b.part(fore).m.d, f * (1.0 - share)));
    c.out.impulse.push((part, t, -f));
}


/// What a balancerCollisionsReaction does with the wall, from how he meets it.
#[derive(Clone, Copy, PartialEq, Debug)]
enum WallMode { Slump, Brace, Glance, Rebound }

/// RDR2's balancer collisions reaction (wallCollision here: Delirium's GTA V balancerCollisionsReaction, the slump
/// alone, keeps that name): he has run or stumbled into a wall (objectBehindVictimPos, with
/// objectBehindVictimNormal pointing out of it towards him). The wall is made solid for his body here (Delirium's world
/// has only the ground): a spring pushes any part back out of its plane. At the impact (reactTime) his legs weaken
/// by impactLegStiffReduction and he snaps away from it. Then, by how he meets it:
/// - back to it (or slumpMode 1): he slumps, letting go of balance after sitStartTime and sliding down it to sit, legs
///   weakening at sitRate, his back pressed to it and his feet sliding out;
/// - facing it (with braceWall, within braceDistance): both hands go to the wall at chest height and hold it with
///   braceArmStiffness, then after timeToForward he pushes himself off it (reboundForce);
/// - side on: a glancing spin away from it (glanceSpinMag, decaying over glanceSpinTime), the near hand on the wall.
/// slumpMode 2 never slumps.
pub struct WallCollision {
    at: Vec3, normal: Vec3, solid: bool, slump_mode: i32, brace: bool, brace_dist: f32, brace_stiffness: f32,
    react: f32, impact_legs: f32, sit_start: f32, sit_rate: f32, glance: f32, glance_time: f32, rebound: f32,
    forward_time: f32, stiffness: f32, snap_mag: f32, wall_k: f32, half_width: f32,
    t: f32, mode: Option<WallMode>, snap: Option<Snap>, arm: [Option<[f32; 6]>; 2], legs: f32,
}

impl WallCollision {
    fn new(p: &mut P) -> Result<WallCollision, Error> {
        Ok(WallCollision {
            at: p.v("objectBehindVictimPos", Vec3::ZERO)?, normal: p.v("objectBehindVictimNormal", Vec3::ZERO)?,
            solid: p.b("solidWall", true)?, slump_mode: p.i("slumpMode", 0)?, brace: p.b("braceWall", true)?,
            brace_dist: p.f("braceDistance", 2.0)?, brace_stiffness: p.f("braceArmStiffness", 12.0)?,
            react: p.f("reactTime", 0.3)?.max(0.0), impact_legs: p.f("impactLegStiffReduction", 3.0)?.max(0.0),
            sit_start: p.f("sitStartTime", 0.3)?.max(0.0), sit_rate: p.f("sitRate", 1.0)?.max(0.01),
            glance: p.f("glanceSpinMag", 50.0)?, glance_time: p.f("glanceSpinTime", 0.5)?.max(0.01),
            rebound: p.f("reboundForce", 0.0)?, forward_time: p.f("timeToForward", 0.5)?.max(0.0),
            stiffness: p.f("stiffness", 10.0)?, snap_mag: p.f("snapMag", 0.3)?, wall_k: p.f("wallStiffness", 400.0)?,
            half_width: p.f("wallHalfWidth", 0.0)?.max(0.0),
            t: 0.0, mode: None, snap: None, arm: [None; 2], legs: 0.0,
        })
    }

    /// The wall's plane, horizontal normal out towards him.
    fn plane(&self, up: Vec3) -> (Vec3, Vec3) {
        let n = self.normal - up * self.normal.dot(up);
        (self.at, n.normalize_or_zero())
    }

    /// Keeps his parts on his side of the wall: a damped spring per part on how far it is inside (and, with a
    /// wallHalfWidth, only along that much of it either side of the point given).
    fn wall(&self, c: &mut Ctx, at: Vec3, n: Vec3) {
        let along = c.ch.up.normalize_or(Vec3::Z).cross(n).normalize_or_zero();
        for k in 0..c.b.build.parts.len().min(21) {
            let pd = &c.b.build.parts[k];
            let q = c.b.part(k);
            if self.half_width > 0.0 && (q.m.d - at).dot(along).abs() > self.half_width {
                continue;
            }
            let reach = pd.radius + pd.half * 0.5;
            let inside = reach - (q.m.d - at).dot(n);
            if inside > 0.0 {
                let vn = q.v.dot(n);
                let a = self.wall_k * inside - 2.0 * self.wall_k.sqrt() * vn.min(0.0);
                c.out.force(k, n * pd.mass * a.max(0.0));
            }
        }
    }

    /// One hand flat on the wall in front of its shoulder (a little lower), as reachForWound places hands.
    fn hand_on_wall(&mut self, c: &mut Ctx, i: usize, at: Vec3, n: Vec3, up: Vec3) {
        let l = crate::nmshot::limb(i);
        let sh = c.b.part(l.p[0]).m.d;
        let on = sh - n * (sh - at).dot(n) - up * 0.15;
        let target = on + n * 0.04;
        let start = self.arm[i].unwrap_or_else(|| {
            let [k, a] = [0, 1].map(|j| c.ch.actual(l.e[j], c.b.frames));
            [k[0], k[1], 0.9, a[1], a[2], 0.4]
        });
        let (x, _) = arm_solve(c, l, target, start);
        self.arm[i] = Some(x);
        (c.ch.eff[l.e[0]].desired[0], c.ch.eff[l.e[0]].desired[1]) = (x[0], x[1]);
        c.ch.eff[l.e[1]].desired = [x[2], x[3], x[4]];
        c.ch.eff[l.e[2]].desired[0] = x[5];
        c.ch.set_stiffness(self.brace_stiffness, 1.0, ["ul", "ur"][i], None);
        let side = if i == 0 { 1.0 } else { -1.0 };
        crate::nmshot::Shot::wrist(c, l, on, n, side);
    }
}

impl Extra for WallCollision {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        self.t += dt;
        let up = c.ch.up.normalize_or(Vec3::Z);
        let (at, n) = self.plane(up);
        if n == Vec3::ZERO {
            self.t = f32::INFINITY;
            return;
        }
        let side = (c.b.part(13).m.d - c.b.part(17).m.d).normalize_or(Vec3::X);
        let fwd = side.cross(up).normalize_or(Vec3::Y);
        let mode = *self.mode.get_or_insert_with(|| {
            let facing = -fwd.dot(n);
            let dist = (c.b.part(10).m.d - at).dot(n);
            if self.slump_mode == 1 || (self.slump_mode == 0 && facing < -0.5) {
                WallMode::Slump
            } else if facing > 0.5 {
                if self.brace && dist < self.brace_dist { WallMode::Brace } else { WallMode::Rebound }
            } else {
                WallMode::Glance
            }
        });
        if self.solid {
            self.wall(c, at, n);
        }
        // the impact: legs weaken, a snap away from the wall
        if self.t < self.react {
            c.ch.set_stiffness((self.stiffness - self.impact_legs).max(2.0), 1.0, "lb", None);
        }
        let snap = self.snap.get_or_insert_with(|| Snap::at(self.snap_mag, 10, n));
        if !snap.done() {
            snap.tick(c, dt);
        }
        match mode {
            WallMode::Slump => {
                // he sags back onto the wall: pelvis and chest drawn to it (a damped spring down to touching), the
                // spine held straight so the back lies on it
                for k in [0, 10] {
                    let pd = &c.b.build.parts[k];
                    let q = c.b.part(k);
                    let gap = (q.m.d - at).dot(n) - (pd.radius + 0.02);
                    if gap > 0.0 {
                        let a = 60.0 * gap.min(0.4) + 12.0 * q.v.dot(n).max(0.0);
                        c.out.force(k, -n * pd.mass * a);
                    }
                }
                for &j in SPINE() {
                    Knees::ease(c, j, 0, 0.0, 2.0 * dt);
                }
                let t = self.t - self.sit_start;
                if t < 0.0 {
                    return;
                }
                c.out.dbm = Some(false);
                // legs give at sitRate; the hips and knees fold into a sit, the back stays against the wall
                self.legs = (self.legs + dt * self.sit_rate).min(1.0);
                let w = self.legs;
                let k = (self.stiffness * (1.0 - 0.6 * w)).max(3.0);
                c.ch.set_stiffness(k, 1.0, "lb", None);
                c.ch.set_stiffness(self.stiffness * 0.8, 1.0, "us", None);
                for s in 0..2 {
                    Knees::ease(c, HIP()[s], 0, 1.3 * w, 3.0 * dt);
                    Knees::ease(c, KNEE()[s], 0, 1.4 * w * knee_dir(c, KNEE()[s]), 3.0 * dt);
                }
                c.ch.set_stiffness(self.stiffness * 0.5, 1.0, "ua", None);
                // sitting up against it: the chest kept over the pelvis along the wall and held up above it (his
                // back's friction on the wall, which Delirium's virtual wall doesn't have)
                let along = up.cross(n).normalize_or_zero();
                let (pel, chest) = (c.b.part(0), c.b.part(10));
                let off = (chest.m.d - pel.m.d).dot(along);
                let rise = (chest.m.d - pel.m.d).dot(up);
                let cm = c.b.build.parts[10].mass;
                let side = -along * (150.0 * off + 20.0 * (chest.v - pel.v).dot(along));
                let lift = up * (150.0 * (0.42 - rise).max(0.0) - 15.0 * chest.v.dot(up).min(0.0));
                c.out.force(10, (side + lift) * cm * w);
                // sliding down it; the feet slide out from it
                for f in [3, 6] {
                    c.out.force(f, n * 4.0 * c.b.build.parts[f].mass * w);
                }
            }
            WallMode::Brace => {
                if self.t < self.forward_time {
                    for i in 0..2 {
                        if !c.ch.busy[i] {
                            self.hand_on_wall(c, i, at, n, up);
                        }
                    }
                } else if self.t < self.forward_time + 0.25 {
                    // push off: arms straighten against it, a shove on the chest away from it
                    c.out.force(10, n * self.rebound);
                    c.out.force(0, n * self.rebound * 0.4);
                }
            }
            WallMode::Rebound => {
                if self.t < 0.25 {
                    c.out.force(10, n * self.rebound);
                }
            }
            WallMode::Glance => {
                // turn away from the wall: the way that swings his facing towards its normal
                let sign = up.cross(fwd).dot(n).signum();
                let tq = self.glance * (-self.t / self.glance_time).exp();
                c.out.torque(10, up * sign * tq * 0.7);
                c.out.torque(0, up * sign * tq * 0.3);
                if self.t < self.forward_time {
                    let near = if (at - c.com).dot(side) > 0.0 { 0 } else { 1 };
                    if !c.ch.busy[near] {
                        self.hand_on_wall(c, near, at, n, up);
                    }
                }
            }
        }
    }

    fn done(&self) -> bool {
        match self.mode {
            Some(WallMode::Slump) => self.t > self.sit_start + 6.0,
            Some(WallMode::Brace) => self.t > self.forward_time + 0.6,
            Some(_) => self.t > self.forward_time.max(self.glance_time * 3.0).max(self.react),
            None => self.t.is_infinite(),
        }
    }
}

/// RDR2's falling reaction with handsAndKnees (its setfallingreaction, folded into catchFall there): a forward fall
/// ends on all fours. The balancer lets go and, as he goes down, the hips and knees fold (about a right angle each)
/// and the hands reach for the ground under the shoulders. Once hands and knees are down he holds there for
/// hkHoldTime, the trunk level and the head up, the arms at least minArmStiffness; then his strength fades
/// (stoppedStrengthDecay per second) and he sinks onto his front. pushForward tips him over first (testing, or a
/// stumble forwards).
pub struct FallingReaction {
    on: bool, hold: f32, decay: f32, arm_k: f32, leg_k: f32, spine_k: f32, push: f32, head_up: bool,
    t: f32, phase: u8, since: f32, ground: Option<f32>, arm: [Option<[f32; 6]>; 2], strength: f32,
}

impl FallingReaction {
    fn new(p: &mut P) -> Result<FallingReaction, Error> {
        Ok(FallingReaction {
            on: p.b("handsAndKnees", true)?, hold: p.f("hkHoldTime", 1.5)?.max(0.0), decay: p.f("stoppedStrengthDecay", 1.0)?.max(0.01),
            arm_k: p.f("minArmStiffness", 14.0)?, leg_k: p.f("minLegStiffness", 12.0)?, spine_k: p.f("minSpineStiffness", 9.0)?,
            push: p.f("pushForward", 0.0)?, head_up: p.b("hkHeadAvoid", true)?,
            t: 0.0, phase: 0, since: 0.0, ground: None, arm: [None; 2], strength: 1.0,
        })
    }

    /// The pose: hips and knees folded by `w` of a right angle, spine level, neck up, the hands to the ground under
    /// the shoulders, stiffness by `k` of the set floors.
    fn pose(&mut self, c: &mut Ctx, w: f32, k: f32, ground: f32, up: Vec3, fwd: Vec3, dt: f32) {
        c.ch.set_stiffness(self.leg_k * k, 1.0, "lb", None);
        c.ch.set_stiffness(self.spine_k * k, 1.0, "us", None);
        c.ch.set_stiffness(self.spine_k * k, 1.0, "un", None);
        for s in 0..2 {
            Knees::ease(c, HIP()[s], 0, 1.5 * w, 6.0 * dt);
            Knees::ease(c, KNEE()[s], 0, 1.6 * w * knee_dir(c, KNEE()[s]), 6.0 * dt);
        }
        for &j in SPINE() {
            Knees::ease(c, j, 0, 0.0, 3.0 * dt);
        }
        if self.head_up {
            for &j in NECK() {
                Knees::ease(c, j, 0, -0.35 * w, 3.0 * dt);
            }
        }
        for i in 0..2 {
            if c.ch.busy[i] {
                continue;
            }
            let l = crate::nmshot::limb(i);
            // under the shoulder (the clavicle's outer end, roughly), a little ahead of it, on the ground
            let sh = c.b.part(l.p[0]).m.d;
            let mid = (c.b.part(13).m.d + c.b.part(17).m.d) * 0.5;
            let out = sh + (sh - mid) * 0.6;
            let below = out - up * ((out.dot(up) - ground) - 0.04) + fwd * 0.08;
            let start = self.arm[i].unwrap_or_else(|| {
                let [kk, a] = [0, 1].map(|j| c.ch.actual(l.e[j], c.b.frames));
                [kk[0], kk[1], 0.6, a[1], a[2], 0.2]
            });
            let (x, _) = arm_solve(c, l, below, start);
            self.arm[i] = Some(x);
            (c.ch.eff[l.e[0]].desired[0], c.ch.eff[l.e[0]].desired[1]) = (x[0], x[1]);
            c.ch.eff[l.e[1]].desired = [x[2], x[3], x[4]];
            c.ch.eff[l.e[2]].desired[0] = x[5];
            c.ch.set_stiffness(self.arm_k * k, 1.0, ["ul", "ur"][i], None);
        }
    }
}

impl FallingReaction {
    /// What his arms and knees carry on all fours (NaturalMotion uses such helpers too: joint strength alone
    /// can't hold a body up): the upper body's weight held at the chest over the hands, the lower body's at the
    /// pelvis over the knees, each a damped spring to its height, by `k` - each only while hands (or knees) touch.
    fn support(&self, c: &mut Ctx, ground: f32, up: Vec3, k: f32) {
        let mass = |r: std::ops::RangeInclusive<usize>| r.map(|i| c.b.build.parts[i].mass).sum::<f32>();
        let (upper, lower) = (mass(7..=20), mass(0..=6));
        let hands_at = (c.b.part(16).m.d + c.b.part(20).m.d) * 0.5;
        let knees_at = (c.b.part(2).m.d + c.b.part(5).m.d) * 0.5;
        // only what the hands and knees really rest on: a helper with nothing under it would hold him up in the
        // air (a slope down, a ledge, stairs)
        let hands = c.b.doll.collided(&[15, 16, 19, 20]);
        let knees = c.b.doll.collided(&[1, 2, 4, 5]);
        for (part, over, height, m, on) in [(10, hands_at, 0.6, upper, hands), (0, knees_at, 0.55, lower * 0.6, knees)] {
            if !on {
                continue;
            }
            let q = c.b.part(part);
            let off = over - q.m.d;
            let flat = off - up * off.dot(up);
            let vflat = q.v - up * q.v.dot(up);
            let h = q.m.d.dot(up) - ground;
            let lift = 4.9 + 90.0 * (height - h).clamp(-0.2, 0.3) - 10.0 * q.v.dot(up);
            c.out.force(part, (flat * 20.0 - vflat * 5.0 + up * lift.max(0.0)) * m * k);
        }
        // the knees drawn in under the hips (the legs trail out behind him otherwise)
        let under = c.b.part(0).m.d;
        for shin in [2, 5] {
            let q = c.b.part(shin);
            let off = under - q.m.d;
            let flat = off - up * off.dot(up);
            let vflat = q.v - up * q.v.dot(up);
            c.out.force(shin, (flat * 25.0 - vflat * 5.0) * c.b.build.parts[shin].mass * 2.0 * k);
        }
    }
}

impl Extra for FallingReaction {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if !self.on {
            return;
        }
        self.t += dt;
        self.since += dt;
        let up = c.ch.up.normalize_or(Vec3::Z);
        let side = (c.b.part(13).m.d - c.b.part(17).m.d).normalize_or(Vec3::X);
        let fwd = side.cross(up).normalize_or(Vec3::Y);
        let ground = *self.ground.get_or_insert_with(|| c.b.part(3).m.d.dot(up).min(c.b.part(6).m.d.dot(up)) - 0.06);
        c.out.dbm = Some(false);
        if self.t < 0.3 && self.push != 0.0 {
            c.out.force(10, fwd * self.push);
        }
        let h = |c: &Ctx, k: usize| c.b.part(k).m.d.dot(up) - ground;
        match self.phase {
            0 => {
                // going down: fold in and reach for the ground
                let w = (self.t / 0.25).min(1.0);
                self.pose(c, w, 1.0, ground, up, fwd, dt);
                // arms and knees take the landing once he is low
                if h(c, 10) < 0.75 {
                    self.support(c, ground, up, 0.7);
                }
                let hands = h(c, 16).max(h(c, 20)) < 0.12;
                let knees = h(c, 2).min(h(c, 5)) < 0.25;
                if (hands && knees) || self.since > 1.5 {
                    (self.phase, self.since) = (1, 0.0);
                }
            }
            1 => {
                // on all fours: the pose held, his weight on hands and knees
                self.pose(c, 1.0, 1.0, ground, up, fwd, dt);
                self.support(c, ground, up, 1.0);
                if self.since > self.hold {
                    (self.phase, self.since) = (2, 0.0);
                }
            }
            _ => {
                // strength fades: he sinks down onto his front
                self.strength = (self.strength - self.decay * dt).max(0.15);
                let k = self.strength;
                self.pose(c, 1.0, k, ground, up, fwd, dt);
            }
        }
    }

    fn done(&self) -> bool {
        !self.on || (self.phase == 2 && self.strength <= 0.15 && self.since > 0.5)
    }
}

/// RDR2's balancer additions, laid over Delirium's balancer while he balances (it stays on till stopped):
/// - running balance: above runningThresh (m/s, his centre's speed along the ground) the hips and knees are at least
///   legStiffnessRun, so a running stumble stays a stumble rather than folding;
/// - feet flattening: a foot coming down is turned flat to the ground (its sole square to up), fully from startFlat
///   above the other foot down, not at all from endFlat up (as RDR2's startFlat / endFlat), by a muscle-like torque
///   pair between foot and shin (flatStiffness, N m per rad).
pub struct BalanceRdr2 {
    run_thresh: f32, run_k: f32, start_flat: f32, end_flat: f32, flat_k: f32,
}

impl BalanceRdr2 {
    fn new(p: &mut P) -> Result<BalanceRdr2, Error> {
        Ok(BalanceRdr2 {
            run_thresh: p.f("runningThresh", 0.65)?, run_k: p.f("legStiffnessRun", 13.0)?,
            start_flat: p.f("startFlat", 0.1)?.max(0.0), end_flat: p.f("endFlat", 0.3)?, flat_k: p.f("flatStiffness", 12.0)?.max(0.0),
        })
    }
}

impl Extra for BalanceRdr2 {
    fn tick(&mut self, c: &mut Ctx, _dt: f32) {
        if c.dbm_failed {
            return;
        }
        let up = c.ch.up.normalize_or(Vec3::Z);
        let speed = (c.vel - up * c.vel.dot(up)).length();
        if speed > self.run_thresh && self.run_k > 0.0 {
            let (s, d) = (self.run_k * self.run_k, 2.0 * self.run_k);
            for j in [HIP()[0], KNEE()[0], HIP()[1], KNEE()[1]] {
                let e = &mut c.ch.eff[j];
                (e.strength, e.damping) = (e.strength.max(s), e.damping.max(d));
            }
        }
        if self.flat_k > 0.0 {
            for (foot, shin, other) in [(3, 2, 6), (6, 5, 3)] {
                let f = c.b.part(foot);
                let h = (f.m.d - c.b.part(other).m.d).dot(up);
                let span = (self.end_flat - self.start_flat).max(0.01);
                let w = ((self.end_flat - h) / span).clamp(0.0, 1.0);
                if w <= 0.0 {
                    continue;
                }
                // the sole's normal is the foot's b axis (the balancer squares it to the ground the same way)
                let err = f.m.b.normalize_or(up).cross(up);
                let spin = f.w - c.b.part(shin).w;
                let t = (err * self.flat_k - spin * self.flat_k * 0.1) * w;
                c.out.torque(foot, t);
                c.out.torque(shin, -t);
            }
        }
    }
}

/// RDR2's keyframe behaviour, used here to blend a balancing body into a neutral stand before it is handed back to
/// animation: over blendTime the spine and neck ease straight, the hands go to his sides, and the parts' motion is
/// damped (settle, 1/s), so the animation picks up from a pose close to its own; then it holds that for holdTime.
/// The legs stay the balancer's.
pub struct Keyframe {
    blend: f32, hold: f32, settle: f32, stiffness: f32,
    t: f32, start: Option<Vec<(usize, [f32; 3])>>, arm: [Option<[f32; 6]>; 2],
    /// RDR2 core: RDR2's keyframe (the parameters' RDR2 half), run in this one's place
    r2: Box<crate::extras_ysk_rdr2::KeyframeRdr2>,
}

impl Keyframe {
    fn new(p: &mut P) -> Result<Keyframe, Error> {
        Ok(Keyframe {
            blend: p.f("blendTime", 0.3)?.max(0.02), hold: p.f("holdTime", 0.5)?.max(0.0), settle: p.f("settle", 3.0)?.max(0.0),
            stiffness: p.f("stiffness", 11.0)?, t: 0.0, start: None, arm: [None; 2],
            r2: Box::new(crate::extras_ysk_rdr2::KeyframeRdr2::new(p)?),
        })
    }
}

impl Extra for Keyframe {
    fn constraints(&self) -> Vec<(usize, Option<Vec3>, Option<bevy_math::Quat>)> {
        self.r2.constraints()
    }

    fn stop(&mut self, c: &mut Ctx) {
        self.r2.stop(c);
    }

    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if c.ch.rdr2 {
            return self.r2.tick(c, dt);
        }
        self.t += dt;
        let x = (self.t / self.blend).min(1.0);
        let w = x * x * (3.0 - 2.0 * x);
        let up = c.ch.up.normalize_or(Vec3::Z);
        let side = (c.b.part(13).m.d - c.b.part(17).m.d).normalize_or(Vec3::X);
        let fwd = side.cross(up).normalize_or(Vec3::Y);
        // spine and neck: from where they were towards straight
        let start = self.start.get_or_insert_with(|| SPINE().iter().chain(NECK().iter()).map(|&j| (j, c.ch.eff[j].desired)).collect());
        for &(j, d0) in start.iter() {
            c.ch.eff[j].desired = d0.map(|a| a * (1.0 - w));
            c.ch.eff[j].set_stiffness(self.stiffness, 1.0, None);
        }
        // hands down at his sides
        let pelvis = c.b.part(0).m.d;
        for i in 0..2 {
            if c.ch.busy[i] {
                continue;
            }
            let l = crate::nmshot::limb(i);
            let s = if i == 0 { 1.0 } else { -1.0 };
            let target = pelvis + side * (0.24 * s) - up * 0.06 + fwd * 0.03;
            let st = self.arm[i].unwrap_or_else(|| {
                let [k, a, e] = [0, 1, 2].map(|n| c.ch.actual(l.e[n], c.b.frames));
                [k[0], k[1], a[0], a[1], a[2], e[0]]
            });
            let (xs, _) = arm_solve(c, l, target, st);
            self.arm[i] = Some(xs);
            let blend = |d: &mut f32, to: f32| *d += (to - *d) * w;
            {
                let e0 = &mut c.ch.eff[l.e[0]].desired;
                blend(&mut e0[0], xs[0]);
                blend(&mut e0[1], xs[1]);
            }
            for k in 0..3 {
                blend(&mut c.ch.eff[l.e[1]].desired[k], xs[2 + k]);
            }
            blend(&mut c.ch.eff[l.e[2]].desired[0], xs[5]);
            c.ch.set_stiffness(self.stiffness * 0.8, 1.0, ["ul", "ur"][i], None);
        }
        // settle: the upper body's motion relative to the pelvis damped out
        if self.settle > 0.0 {
            let pv = c.b.part(0).v;
            for k in 7..21 {
                let q = c.b.part(k);
                c.out.force(k, -(q.v - pv) * c.b.build.parts[k].mass * self.settle * w);
            }
        }
    }

    fn done(&self) -> bool {
        self.t > self.blend + self.hold
    }
}

/// One message to start: its name and parameters (start=true among them).
pub type Start = (&'static str, Vec<(&'static str, crate::msg::Value)>);

/// The RDR2 reactions option, for a game or the playroom that wants them on its hits: which of the messages above a
/// hit starts, and how often. Chances are 0..1. Nothing calls this by itself; a caller that leaves the option off
/// gets Delirium's reactions unchanged.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Reactions {
    /// every shot: shotSnap, the hit part jerked along the bullet and snapped back
    pub snap: bool,
    /// chance a shot in the trunk spins him round (shotSpin)
    pub spin: f32,
    /// chance a shot in the belly, pelvis or legs drops him to his knees (shotFallToKnees; not with a spin)
    pub knees: f32,
    /// where he is hit matters: shotFromBehind (the back), shotInGuts (the belly), shotInjuredLeg, shotInjuredArm
    pub hit_location: bool,
    /// chance a hand goes to the wound (reachForWound; not the head, neck, hands or feet; both hands for the belly)
    pub reach: f32,
    /// a melee blow: meleeReact instead of the usual flinch
    pub melee: bool,
    /// rdr2Balance on from the first hit: stiffer legs when running, feet set down flat
    pub balance: bool,
}

impl Default for Reactions {
    fn default() -> Reactions {
        Reactions { snap: true, spin: 0.25, knees: 0.15, hit_location: true, reach: 0.7, melee: true, balance: true }
    }
}

impl Reactions {
    /// What a bullet starts, sent after the shot itself: `part` hit at `at`, travelling along `dir`; `fwd` is the
    /// way he faces ((left clavicle - right clavicle) x up).
    pub fn on_shot(&self, rand: &mut delirium_data::rng::Rand, part: usize, at: Vec3, dir: Vec3, fwd: Vec3) -> Vec<Start> {
        self.on_shot_core(rand, part, at, dir, fwd, false)
    }

    /// As `on_shot`; with the RDR2 core (`core`) only what RDR2's shot task was recorded starting: reachForWound, and
    /// staggerFall for a shot in a thigh or shin (with RDR2's defaults, which the RDR2 core reads).
    pub fn on_shot_core(&self, rand: &mut delirium_data::rng::Rand, part: usize, at: Vec3, dir: Vec3, fwd: Vec3, core: bool) -> Vec<Start> {
        use crate::msg::Value::{Bool as B, Float as F, Int as I, Vec3 as V};
        let mut out: Vec<Start> = vec![];
        let mut go = |name, mut ps: Vec<_>| {
            ps.insert(0, ("start", B(true)));
            out.push((name, ps));
        };
        // under the RDR2 core the balancer itself runs RDR2's running mode and flat feet (nmdbm_rdr2.rs); this extra is
        // Delirium's stand-in for them, so it stays off
        if self.balance && !core {
            go("rdr2Balance", vec![]);
        }
        if core {
            // RDR2's shot task, as recorded (pistol on a standing townsman, chest, back and leg, five recordings): the
            // shot's snap, shotspin, shockSpin, fromBehind, inGuts, injured-limb, fallToKnees and headLook modules all
            // stay off; reachForWound runs on every shot and staggerFall on a leg shot
            if matches!(part, 1 | 2 | 4 | 5) {
                go("staggerFall", vec![]);
            }
            if !matches!(part, 3 | 6 | 11 | 12 | 16 | 20) {
                go("reachForWound", vec![("hitPoint", V(at)), ("hitPart", I(part as i32)), ("bulletVel", V(dir)), ("reachWithOneHand", B(true)),
                                         ("timeBeforeReachForWound", F(rand.float_in_range(0.1, 0.3))), ("grabHoldTime", F(rand.float_in_range(1.5, 3.0)))]);
            }
            return out;
        }
        if self.snap {
            go("shotSnap", vec![("bulletDir", V(dir)), ("bodyPart", I(part as i32)), ("snapMag", F(rand.float_in_range(0.3, 0.5)))]);
        }
        // Delirium's parts: 1-3 / 4-6 the legs, 7-10 the spine (7-8 the belly), 13-16 / 17-20 the arms
        let (trunk, low) = ((7..=10).contains(&part), part <= 8);
        let spun = trunk && rand.float_in_range(0.0, 1.0) < self.spin;
        let mut knees = false;
        if spun {
            go("shotSpin", vec![("hitPoint", V(at)), ("bulletDir", V(dir))]);
        } else if low && rand.float_in_range(0.0, 1.0) < self.knees {
            knees = true;
            go("shotFallToKnees", vec![("ftkPitchForwards", B(rand.float_in_range(0.0, 1.0) < 0.6))]);
        }
        let mut gut = false;
        if self.hit_location {
            let (f, d) = (fwd.with_z(0.0).normalize_or_zero(), dir.with_z(0.0).normalize_or_zero());
            if (trunk || part == 0) && f.dot(d) > 0.4 {
                go("shotFromBehind", vec![("sfbSpineAmount", F(rand.float_in_range(0.4, 0.6))), ("sfbNeckAmount", F(rand.float_in_range(0.2, 0.4)))]);
            } else if part == 7 || part == 8 {
                gut = true;
                go("shotInGuts", vec![("sigSpineAmount", F(rand.float_in_range(0.45, 0.7))), ("sigPeriod", F(rand.float_in_range(1.0, 2.0)))]);
            } else if !knees && matches!(part, 1 | 2 | 4 | 5) {
                go("shotInjuredLeg", vec![("bodyPart", I(part as i32)), ("legInjuryTime", F(rand.float_in_range(1.0, 2.0)))]);
            } else if matches!(part, 14..=16 | 18..=20) {
                go("shotInjuredArm", vec![("bodyPart", I(part as i32)), ("bulletDir", V(dir))]);
            }
        }
        if !matches!(part, 3 | 6 | 11 | 12 | 16 | 20) && rand.float_in_range(0.0, 1.0) < self.reach {
            go("reachForWound", vec![("hitPoint", V(at)), ("hitPart", I(part as i32)), ("reachWithOneHand", B(!gut)),
                                     ("timeBeforeReachForWound", F(rand.float_in_range(0.1, 0.3))), ("grabHoldTime", F(rand.float_in_range(1.5, 3.0)))]);
        }
        out
    }

    /// What a melee blow starts (instead of a flinch): `part` hit at `at` by a blow along `dir`. Softer than the
    /// message's defaults, so a punch staggers him and rarely drops him. With the RDR2 core, RDR2's meleeReact: a new
    /// blow (hitNew) at a world point, and no Lean (there the balancer's lean amount, RDR2's default none).
    pub fn on_melee(&self, rand: &mut delirium_data::rng::Rand, part: usize, at: Vec3, dir: Vec3, core: bool) -> Vec<Start> {
        use crate::msg::Value::{Bool as B, Float as F, Int as I, Vec3 as V};
        if !self.melee {
            return vec![];
        }
        let mut out: Vec<Start> = vec![];
        if self.balance && !core {
            out.push(("rdr2Balance", vec![("start", B(true))]));
        }
        if core {
            out.push(("meleeReact", vec![("start", B(true)), ("hitNew", B(true)), ("hitPoint", V(at)), ("hitLocal", B(false)),
                                         ("hitNormal", V(-dir.normalize_or_zero())), ("hitPart", I(part as i32)),
                                         ("snapMag", F(rand.float_in_range(0.15, 0.22))), ("csLooseness", F(0.35))]));
            return out;
        }
        out.push(("meleeReact", vec![("start", B(true)), ("hitPoint", V(at)), ("hitNormal", V(-dir.normalize_or_zero())), ("hitPart", I(part as i32)),
                                     ("Lean", F(rand.float_in_range(40.0, 55.0))), ("snapMag", F(rand.float_in_range(0.15, 0.22))), ("csLooseness", F(0.35))]));
        out
    }
}

/// How hard the hips leans push, m/s² per unit of leanAmount on his whole mass (Delirium's own).
const HIPS_LEAN: f32 = 4.0;
/// onFire's roll: torque per unit of rollTorqueScale per kg (Delirium's own).
const ROLL: f32 = 0.075;

/// What a lean moves: his balance, his hips, or a push on one part.
#[derive(Clone, Copy, PartialEq)]
enum LeanBy {
    Balance,
    Hips,
    Push(usize),
}

/// Which way a lean goes.
#[derive(Clone, Copy)]
enum LeanWay {
    Dir(Vec3),
    Pos(Vec3),
    Random { amount: [f32; 2], change: [f32; 2], goal: Vec3, now: Vec3, left: f32 },
}

/// The lean family (RDR2's, as GTA V's): leanRandom, hipsLean* and forceLean*, each in a direction, to a position,
/// towards a point or at random. The plain one leans him through the balancer, as leanInDirection does; the hips ones
/// push his pelvis that way and his chest the other, so his hips swing out over his feet; the force ones push bodyPart
/// that way, leanAmount times his weight, while he's on his feet. A negative leanAmount goes away. The random ones pick a new way (and an amount between leanAmountMin
/// and leanAmountMax) every changeTimeMin..changeTimeMax s. They run until stopped.
pub struct Lean {
    by: LeanBy,
    way: LeanWay,
    amount: f32,
}

impl Lean {
    fn new(name: &str, p: &mut P) -> Result<Lean, Error> {
        let by = if name.starts_with("hips") {
            LeanBy::Hips
        } else if name.starts_with("force") {
            LeanBy::Push(p.i("bodyPart", 0)?.clamp(0, 20) as usize)
        } else {
            LeanBy::Balance
        };
        if name.ends_with("Random") {
            let (lo, hi, t0, t1) = if by == LeanBy::Balance { (0.2, 0.2, 0.5, 1.0) } else { (0.3, 0.4, 2.0, 4.0) };
            let amount = [p.f("leanAmountMin", lo)?, p.f("leanAmountMax", hi)?];
            let change = [p.f("changeTimeMin", t0)?.max(0.05), p.f("changeTimeMax", t1)?.max(0.05)];
            return Ok(Lean { by, way: LeanWay::Random { amount, change, goal: Vec3::ZERO, now: Vec3::ZERO, left: 0.0 }, amount: 0.0 });
        }
        let amount = p.f("leanAmount", if by == LeanBy::Hips && !name.ends_with("ToPosition") { 0.5 } else { 0.2 })?;
        let way = if name.ends_with("InDirection") {
            LeanWay::Dir(p.v("dir", Vec3::ZERO)?)
        } else if name.ends_with("ToPosition") {
            LeanWay::Pos(p.v("pos", Vec3::ZERO)?)
        } else {
            let _ = (p.i("instanceIndex", -1)?, p.i("boundIndex", 0)?);
            LeanWay::Pos(p.v("offset", Vec3::ZERO)?)
        };
        Ok(Lean { by, way, amount })
    }
}

impl Extra for Lean {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        let up = c.up;
        let flat = |v: Vec3| (v - up * v.dot(up)).normalize_or_zero();
        let (dir, amount) = match &mut self.way {
            LeanWay::Dir(d) => (flat(*d), self.amount),
            LeanWay::Pos(p) => (flat(*p - c.com), self.amount),
            LeanWay::Random { amount, change, goal, now, left } => {
                *left -= dt;
                if *left <= 0.0 {
                    let a = c.ch.random() * std::f32::consts::TAU;
                    let side = flat(up.any_orthonormal_vector());
                    let size = amount[0] + (amount[1] - amount[0]) * c.ch.random();
                    *goal = (side * a.cos() + up.cross(side) * a.sin()) * size;
                    *left = change[0] + (change[1] - change[0]) * c.ch.random();
                }
                // glide to the new lean over half the shortest change, so a reversal doesn't throw him over
                *now += (*goal - *now) * (dt / (change[0] * 0.5)).min(1.0);
                (now.normalize_or_zero(), now.length())
            }
        };
        // the pushes stop once he's down: they lean him, they don't drag him along the floor
        let up_now = c.b.part(0).m.d.dot(up) > 0.55 && !c.dbm_failed;
        if dir == Vec3::ZERO || (self.by != LeanBy::Balance && !up_now) {
            return;
        }
        match self.by {
            LeanBy::Balance => {
                let mode = match self.way {
                    LeanWay::Pos(p) => LeanMode::Pos(p),
                    _ => LeanMode::Dir(dir),
                };
                c.out.lean = Some((mode, amount, false));
            }
            // the RDR2 core's balancer takes the hip lean itself, tilting the pelvis frame the legs reach for
            LeanBy::Hips if c.ch.rdr2 => c.out.hip_lean = Some(dir * amount),
            LeanBy::Hips => {
                // the hips one way and the chest the other: the hips swing out over his feet without pushing him off
                let f = dir * amount * c.b.doll.mass() * HIPS_LEAN;
                c.out.force(0, f);
                c.out.force(10, -f);
            }
            LeanBy::Push(k) => c.out.force(k, dir * amount * c.b.doll.mass() * GRAVITY()),
        }
    }
}

/// RDR2's onFire: on his feet for staggerTime he staggers, his lean swinging forwards and back (up to
/// stumbleMaxLeanForward / stumbleMaxLeanBack, at staggerLeanRate) and against where he's heading (predictTime), his
/// arms flailing between a windmill and a writhe (armsWindmillWritheBlend), his spine and legs joining in by their
/// blends. Then (or once his balance goes) the balancer lets go; down, he writhes between curled up and thrashing (the
/// *PoseWritheBlend values) and, with rollOverFlag, rolls back and forth (rollTorqueScale) for maxRollOverTime while
/// within rollOverRadius of where he fell. It runs until stopped. With the RDR2 core it runs RDR2's own onFire
/// instead (extras_cf_rdr2.rs).
pub struct OnFire {
    stagger: f32, lean_rate: f32, back: f32, fwd: f32, up_blend: [f32; 3], down_blend: [f32; 3], roll_k: f32, predict: f32,
    roll: bool, roll_time: f32, radius: f32, t: f32, phase: f32, lean: f32, goal: f32, next: f32, down: Option<(f32, Vec3)>,
    /// RDR2 core: the message's values as RDR2 takes them, and RDR2's onFire once it runs
    rp: crate::extras_cf_rdr2::FireParams,
    r: Option<crate::extras_cf_rdr2::OnFireRdr2>,
}

impl OnFire {
    fn new(p: &mut P) -> Result<OnFire, Error> {
        let u = |x: f32| x.clamp(0.0, 1.0);
        Ok(OnFire {
            stagger: p.f("staggerTime", 2.5)?.max(0.0), lean_rate: u(p.f("staggerLeanRate", 0.9)?),
            back: p.f("stumbleMaxLeanBack", 0.4)?.max(0.0), fwd: p.f("stumbleMaxLeanForward", 0.5)?.max(0.0),
            up_blend: [u(p.f("armsWindmillWritheBlend", 0.4)?), u(p.f("spineStumbleWritheBlend", 0.7)?), u(p.f("legsStumbleWritheBlend", 0.2)?)],
            down_blend: [u(p.f("armsPoseWritheBlend", 0.7)?), u(p.f("spinePoseWritheBlend", 0.55)?), u(p.f("legsPoseWritheBlend", 0.5)?)],
            roll_k: p.f("rollTorqueScale", 25.0)?.max(0.0), predict: p.f("predictTime", 0.1)?.max(0.0),
            roll: p.b("rollOverFlag", true)?, roll_time: p.f("maxRollOverTime", 8.0)?.max(0.0), radius: p.f("rollOverRadius", 2.0)?.max(0.0),
            t: 0.0, phase: 0.0, lean: 0.0, goal: 0.0, next: 0.0, down: None,
            rp: crate::extras_cf_rdr2::FireParams::parse(p, Default::default())?, r: None,
        })
    }
}

impl Extra for OnFire {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        use std::f32::consts::PI;
        if c.ch.rdr2 {
            let p = self.rp;
            self.r.get_or_insert_with(|| crate::extras_cf_rdr2::OnFireRdr2::new(p)).tick(c, dt);
            return;
        }
        self.t += dt;
        self.phase += dt * 2.0 * PI / 0.45;
        let (ph, up) = (self.phase, c.up);
        let pelvis = c.b.part(0).m.d;
        if self.down.is_none() && (self.t > self.stagger || c.dbm_failed || pelvis.dot(up) < 0.55) {
            self.down = Some((self.t, c.com));
        }
        let ground = self.down.is_some();
        if ground {
            c.out.dbm = Some(false);
            c.ch.set_stiffness(10.0, 1.0, "fb", None);
        }
        // arms: thrown about across most of their range, on his feet a windmill (or a writhe), down a writhe (or
        // curled in towards his face); the elbows bend and straighten as they go
        for i in 0..2 {
            let l = crate::nmshot::limb(i);
            let (sh, el) = (l.e[1], l.e[2]);
            let s = i as f32 * PI;
            let ed = knee_dir(c, el);
            let writhe = [0.85 * (1.7 * ph + s).sin(), 0.8 * (1.3 * ph + 2.0 * s + 0.5).sin(), 0.5 * (1.1 * ph + s).sin(), 0.3 + 0.6 * (2.3 * ph + s).sin()];
            // the windmill: big circles, arms near straight, the writhe laid over it rather than shrinking it
            let windmill = [(ph + s).sin(), 0.6 * (ph + s).cos(), 0.0, -0.6];
            let wu = self.up_blend[0] * 0.5;
            let stand: [f32; 4] = std::array::from_fn(|k| windmill[k] * (1.0 - wu) + writhe[k] * wu);
            let wd = self.down_blend[0];
            let lying: [f32; 4] = std::array::from_fn(|k| [0.5, 0.0, 0.0, 0.8][k] * (1.0 - wd) + writhe[k] * wd);
            // he keeps windmilling as he goes down, into the ground's writhe over a moment
            let g = self.down.map_or(0.0, |(t0, _)| ((self.t - t0 - 0.8) / 0.6).clamp(0.0, 1.0));
            let mix = |k: usize| stand[k] * (1.0 - g) + lying[k] * g;
            if g < 0.5 && self.up_blend[0] < 1.0 {
                // the windmill: the hand goes round a big circle in front of him, up past his head and down, the
                // arms half a turn apart; the writhe then shakes it about
                let at = c.b.part(l.p[0]).m.d;
                let clav = |i: usize| c.b.part(crate::nmshot::limb(i).p[0]).m.d;
                let face = (clav(0) - clav(1)).cross(up);
                let face = (face - up * face.dot(up)).normalize_or_zero();
                let a = self.t * 2.0 * PI / 0.8 + s;
                let to = at + face * (0.15 + 0.5 * a.cos()) + up * (0.6 * a.sin() + 0.1);
                for j in [l.e[0], sh, el] {
                    c.ch.eff[j].oppose_gravity = 1.0;
                }
                crate::nmdbm::limb_ik(c.b, c.ch, sh, el, l.p[0], l.p[delirium_data::kv!(0 int)], to, 0.0, 0.0, 1.0 - 2.0 * i as f32, 1.0, Vec3::ZERO, None);
                for k in 0..3 {
                    c.ch.eff[sh].offset[k] += writhe[k] * wu * 0.6;
                }
            } else {
                for k in 0..3 {
                    drive(c, sh, k, mix(k));
                }
                drive(c, el, 0, mix(3) * ed);
            }
            c.ch.set_stiffness(15.0, 0.8, ["ul", "ur"][i], None);
        }
        // spine: hunched forward (more when down), and a writhe
        let w = if ground { self.down_blend[1] } else { self.up_blend[1] };
        let curl = if ground { 0.6 } else { 0.35 };
        let n = SPINE().len() as f32;
        for (k, &j) in SPINE().iter().enumerate() {
            let o = k as f32;
            let writhe = [0.5 * (1.3 * ph + o).sin(), 0.4 * (0.9 * ph + 1.0 + o).sin(), 0.6 * (1.1 * ph + o * 0.5).sin()];
            for a in 0..3 {
                c.ch.eff[j].offset[a] += ((if a == 0 { curl } else { 0.0 }) * (1.0 - w * 0.5) + writhe[a] * w) / n;
            }
        }
        // legs: the balancer's (on his feet) or drawn up (down), and a kicking writhe
        let w = if ground { self.down_blend[2] } else { self.up_blend[2] };
        for s in 0..2 {
            let (hip, knee) = (HIP()[s], KNEE()[s]);
            let kd = knee_dir(c, knee);
            let o = s as f32 * 2.0;
            let big = if ground { 1.0 } else { 0.35 };
            let writhe = [big * 0.9 * (1.9 * ph + o).sin(), big * 0.4 * (1.4 * ph + o + 1.0).sin(), big * (0.8 + 0.8 * (2.1 * ph + o).sin())];
            let posed = if ground { [1.0, 0.0, 1.6] } else { [0.0, 0.0, 0.0] };
            c.ch.eff[hip].offset[0] += posed[0] * (1.0 - w) + writhe[0] * w;
            c.ch.eff[hip].offset[1] += posed[1] * (1.0 - w) + writhe[1] * w;
            c.ch.eff[knee].offset[0] += (posed[2] * (1.0 - w) + writhe[2] * w) * kd;
        }
        match self.down {
            None => {
                // stagger: the lean swings between back and forward, and against where he's heading
                self.next -= dt;
                if self.next <= 0.0 {
                    self.goal = -self.back + (self.fwd + self.back) * c.ch.random();
                    self.next = 0.5 + 0.7 * c.ch.random();
                }
                self.lean += (self.goal - self.lean) * (self.lean_rate * 4.0 * dt).min(1.0);
                let clav = |i: usize| c.b.part(crate::nmshot::limb(i).p[0]).m.d;
                let face = (clav(0) - clav(1)).cross(up);
                let face = (face - up * face.dot(up)).normalize_or_zero();
                let v = face * self.lean - (c.vel - up * c.vel.dot(up)) * self.predict;
                if v.length() > 1e-3 {
                    c.out.lean = Some((LeanMode::Dir(v.normalize()), v.length(), false));
                }
            }
            Some((t0, at)) => {
                // down: roll back and forth along his length to put the flames out
                let since = self.t - t0;
                let d = c.com - at;
                if self.roll && since > 0.5 && since < self.roll_time && (d - up * d.dot(up)).length() < self.radius {
                    let along = c.b.part(12).m.d - pelvis;
                    let along = (along - up * along.dot(up)).normalize_or_zero();
                    let sign = if (since / 1.2) as i32 % 2 == 0 { 1.0 } else { -1.0 };
                    let tq = along * sign * self.roll_k * c.b.doll.mass() * ROLL;
                    c.out.torque(0, tq * 0.5);
                    c.out.torque(10, tq * 0.5);
                }
            }
        }
    }

    /// RDR2 core: deactivate (its children stopped, the balancer's hips and knees let go).
    fn stop(&mut self, c: &mut Ctx) {
        if let Some(r) = self.r.as_mut() {
            r.stop(c);
        }
    }

    /// RDR2 core: a start while it runs reloads the defaults and the message's values; it keeps running.
    fn hit(&mut self, p: &mut P) -> Option<Result<(), Error>> {
        let r = self.r.as_mut()?;
        Some(crate::extras_cf_rdr2::FireParams::parse(p, Default::default()).map(|x| (r.p, self.rp) = (x, x)))
    }

    /// RDR2 core: values sent without start replace the running ones.
    fn update(&mut self, p: &mut P) -> Result<(), Error> {
        if let Some(r) = self.r.as_mut() {
            r.p = crate::extras_cf_rdr2::FireParams::parse(p, r.p)?;
            self.rp = r.p;
        }
        Ok(())
    }
}

/// RDR2's lassoed reactions (its yanked's, while dragged on a rope; Delirium's `dragged`, with ropedBodyPart, does the
/// pulling). By lassoedtype: round the neck (0), round the chest and arms (1: arms pinned to his sides, more wriggle),
/// round the feet (2: legs held straight, arms reaching past his head clawing at the ground). On his feet nothing;
/// down, round the neck his hands claw at the rope where it's round grabPart (grabposl / grabposr from
/// it, clawing about by grabpointamp at grabpointfreq), his legs kick (legAmplitude), his spine writhes forwards and
/// back (spinefreq, spineAmplitude) and side to side (spsidefreq, spsideamp), at the ground stiffnesses, and the ground
/// bumps him now and then (bumpsim, bumpsimprob). With stunnedtime he first lies limp that long, coming out of it over
/// stunnedblendout. It runs until stopped. Its defaults are RDR2's.
pub struct Lasso {
    kind: i32, part: i32, grab: [Vec3; 2], claw: Vec3, claw_freq: Vec3, legs: f32, spine: [f32; 4], stiff: [(f32, f32); 4],
    bump: (f32, f32), stunned: (f32, f32), t: f32, down: Option<f32>, bump_t: f32,
    /// RDR2 core: RDR2's yanked from this message's values, run in this one's place
    r2: Box<crate::extras_ysk_rdr2::YankedRdr2>,
}

impl Lasso {
    fn new(p: &mut P) -> Result<Lasso, Error> {
        let _ = (p.f("groundFriction", 1.0)?, p.f("ropeendfriction", 0.5)?, p.f("otherendfriction", 2.0)?);
        let (leg_d, arm_d, spine_d) = (p.f("groundLegDamping", 0.5)?, p.f("groundArmDamping", 0.5)?, p.f("groundSpineDamping", 0.5)?);
        let mut y = crate::extras_ysk_rdr2::YankedParams::new(p)?;
        y.lassoed = p.i("lassoedtype", 0)?.clamp(0, 2);
        if p.i("grabPart", 11)? >= 0 {
            y.grab_part = p.i("grabPart", 11)?.clamp(0, 20) as usize;
        }
        y.rope_part = p.i("ropePart", 10)?.clamp(0, 20) as usize;
        Ok(Lasso {
            r2: Box::new(crate::extras_ysk_rdr2::YankedRdr2::new(y)),
            kind: p.i("lassoedtype", 0)?.clamp(0, 2),
            part: p.i("grabPart", 11)?.clamp(-1, 20),
            grab: [p.v("grabposl", Vec3::new(0.07, 0.05, -0.051))?, p.v("grabposr", Vec3::new(0.09, -0.05, -0.051))?],
            claw: p.v("grabpointamp", Vec3::ZERO)?, claw_freq: p.v("grabpointfreq", Vec3::new(4.0, 5.0, 3.0))?,
            legs: p.f("legAmplitude", 0.4)?.max(0.0),
            spine: [p.f("spinefreq", 2.5)?.max(0.0), p.f("spineAmplitude", 1.0)?.max(0.0), p.f("spsidefreq", 2.0)?.max(0.0), p.f("spsideamp", 1.0)?.max(0.0)],
            stiff: [(p.f("groundLegStiffness", 11.0)?, leg_d), (p.f("groundArmStiffness", 11.0)?, arm_d),
                    (p.f("groundSpineStiffness", 11.0)?, spine_d), (p.f("groundneckstiffness", 11.0)?, spine_d)],
            bump: (p.f("bumpsim", 400.0)?.max(0.0), p.f("bumpsimprob", 0.08)?.clamp(0.0, 1.0)),
            stunned: (p.f("stunnedtime", 0.0)?.max(0.0), p.f("stunnedblendout", 0.0)?.max(0.0)),
            t: 0.0, down: None, bump_t: 0.0,
        })
    }
}

impl Extra for Lasso {
    fn lines(&mut self) -> Vec<String> {
        self.r2.lines()
    }

    fn update(&mut self, p: &mut P) -> Result<(), Error> {
        let at = p.v("ropePos", Vec3::ZERO)?;
        if at != Vec3::ZERO {
            self.r2.rope_to(at);
        }
        Ok(())
    }

    fn stop(&mut self, c: &mut Ctx) {
        self.r2.stop(c);
    }

    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        use std::f32::consts::TAU;
        if c.ch.rdr2 {
            return self.r2.tick(c, dt);
        }
        self.t += dt;
        let up = c.up;
        let pelvis = c.b.part(0).m.d;
        if self.down.is_none() && pelvis.dot(up) < 0.45 {
            self.down = Some(self.t);
        }
        let Some(t0) = self.down else { return };
        let since = self.t - t0;
        // stunned: limp, then coming out of it
        let (st, blend) = self.stunned;
        let w = if since < st { 0.0 } else if blend > 0.0 { ((since - st) / blend).min(1.0) } else { 1.0 };
        let scale = |k: f32| 1.0 + (k - 1.0) * w;
        for (mask, (k, d)) in ["ll", "lr"].iter().map(|m| (m, self.stiff[0])).chain(["ul", "ur"].iter().map(|m| (m, self.stiff[1])))
            .chain(std::iter::once((&"us", self.stiff[2]))).chain(std::iter::once((&"un", self.stiff[3]))) {
            c.ch.set_stiffness(scale(k), d, mask, None);
        }
        if w == 0.0 {
            return;
        }
        // the spine writhes, forwards and back and side to side
        let n = SPINE().len() as f32;
        let [f1, a1, f2, a2] = self.spine;
        // arms pinned, he wriggles side to side more
        let wriggle = if self.kind == 1 { 1.8 } else { 1.0 };
        let bend = 0.3 * a1 * (TAU * f1 * since).sin() * w;
        let side = 0.25 * a2 * wriggle * (TAU * f2 * since + 1.0).sin() * w;
        for &j in SPINE().iter() {
            c.ch.eff[j].offset[0] += bend / n;
            c.ch.eff[j].offset[1] += side / n;
        }
        // the legs kick, out of step; tied at the feet they're held together, straight
        for s in 0..2 {
            if self.kind == 2 {
                let knee = KNEE()[s];
                let kd = knee_dir(c, knee);
                drive(c, knee, 0, -0.9 * kd);
                continue;
            }
            let (hip, knee) = (HIP()[s], KNEE()[s]);
            let kd = knee_dir(c, knee);
            let ph = TAU * 1.8 * since + s as f32 * std::f32::consts::PI;
            c.ch.eff[hip].offset[0] += self.legs * 1.2 * ph.sin() * w;
            c.ch.eff[knee].offset[0] += self.legs * (1.0 + 1.5 * (ph + 0.6).sin()).max(0.0) * kd * w;
        }
        let clav = |i: usize| c.b.part(crate::nmshot::limb(i).p[0]).m.d;
        let along = (c.b.part(12).m.d - pelvis).normalize_or(up);
        let left = (clav(0) - clav(1)).normalize_or_zero();
        let reach = |c: &mut Ctx, i: usize, to: Vec3| {
            let l = crate::nmshot::limb(i);
            crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[delirium_data::kv!(0 int)], to, 0.0, 0.0, 1.0 - 2.0 * i as f32, 1.0, Vec3::ZERO, None);
        };
        if self.kind == 1 {
            // round his chest and arms: they're pinned to his sides
            for i in 0..2 {
                let side = if i == 0 { 1.0 } else { -1.0 };
                reach(c, i, pelvis + left * (0.22 * side) + along * 0.15);
            }
            c.ch.set_stiffness(14.0, 1.0, "ul", None);
            c.ch.set_stiffness(14.0, 1.0, "ur", None);
        } else if self.kind == 2 {
            // round his feet, dragged feet first: his arms reach out past his head, clawing at the ground in turn
            let ground = c.b.part(3).m.d.dot(up).min(c.b.part(6).m.d.dot(up));
            let head = c.b.part(12).m.d;
            for i in 0..2 {
                let side = if i == 0 { 1.0 } else { -1.0 };
                let to = head + along * (0.4 + 0.2 * (TAU * 1.2 * since + i as f32 * std::f32::consts::PI).sin()) + left * (0.22 * side);
                reach(c, i, to - up * (to.dot(up) - ground - 0.05));
            }
        }
        // round the neck: the hands claw at the rope round grabPart
        if self.kind == 0 && self.part >= 0 {
            let at = c.b.part(self.part as usize).m.d;
            // his own frame at the neck (lying, his up runs along his spine)
            let fwd = left.cross(along).normalize_or_zero();
            let claw = Vec3::new((TAU * self.claw_freq.x * since).sin(), (TAU * self.claw_freq.y * since).sin(), (TAU * self.claw_freq.z * since).sin()) * self.claw;
            for i in 0..2 {
                let g = self.grab[i] + claw;
                reach(c, i, at + fwd * g.x + left * g.y + along * g.z);
            }
        }
        // now and then the ground bumps him
        self.bump_t += dt;
        if self.bump.0 > 0.0 && self.bump_t >= 0.1 {
            self.bump_t -= 0.1;
            if c.ch.random() < self.bump.1 {
                let k = [0, 8, 10][(c.ch.random() * 2.99) as usize];
                c.out.force(k, up * self.bump.0 * w);
            }
        }
    }
}

/// RDR2's sit: riding, written from its parameters' names and how riders look (none of RDR2's code). He's swung up
/// into the seat over mountTime, then his pelvis is held there and his feet in the stirrups (hard constraints, which
/// the game moves with the horse by sending rdr2Sit again without start), his hips square to the seat, tipped by
/// hippitchstart, his back by straightnessstart, his hands forward on the reins. Thrown when his body leans past
/// failangle from the seat's up, when the seat itself tips past seatfailangle, when a hit moves his chest faster than
/// knockSpeed against his hips, or when sent throw=true: he's flung off (throwSpeed) and everything
/// lets go and he falls (catchFall), or with drag a foot stays caught and he's dragged by it for dragTime, reaching
/// back for it.
///
/// With the RDR2 core it runs RDR2's sit instead (rdr2_sit.md, `SitRdr2`).
pub struct Sit {
    seat: Vec3, up: Vec3, fwd: Vec3, stirrups: [Vec3; 2], given: [bool; 4], mount: f32, constrain: bool, square: f32,
    feet: bool, stirrup_k: f32, soft: f32, hip: (f32, f32), straight: (f32, f32), fail: f32, seat_fail: f32, slump: bool,
    drag: (bool, f32, bool, f32, f32), t: f32, from: Option<[Vec3; 3]>, thrown: Option<(f32, Option<usize>)>,
    throw: bool, knock: f32, fling: (f32, Vec3), cons: Vec<(usize, Option<Vec3>, Option<bevy_math::Quat>)>, out: Vec<String>,
    /// RDR2 core: RDR2's sit, run in this one's place
    r2: Box<SitRdr2>,
}

/// s after he's in the seat before leaning or the seat tipping can throw him.
const SETTLE: f32 = 0.6;
/// s he's flung over when thrown, and roughly what he weighs, kg.
const FLING: f32 = 0.15;
const FLUNG_MASS: f32 = 80.0;
/// His chest held upright at softweight 1, N m per radian.
const CHEST_HOLD: f32 = 900.0;
/// The pelvis and the feet.
const SIT_PARTS: [usize; 3] = [0, 3, 6];

impl Sit {
    fn new(p: &mut P) -> Result<Sit, Error> {
        // (without it RDR2's sit has no mounting, and constrain is RDR2's 0)
        let mount = p.f("mountTime", f32::NAN)?;
        let constrain = p.get("constrain").map(|v| v.i()).transpose()?;
        let r2 = Box::new(SitRdr2::new(p, if mount.is_nan() { 0.0 } else { mount.max(0.0) }, constrain.unwrap_or(0))?);
        let mut s = Sit {
            seat: Vec3::ZERO, up: Vec3::Z, fwd: Vec3::ZERO, stirrups: [Vec3::ZERO; 2], given: [false; 4],
            mount: if mount.is_nan() { 0.6 } else { mount.max(0.0) }, constrain: constrain.map_or(true, |c| c != 0),
            square: p.f("constrainstrength", 40.0)?.max(0.0),
            feet: false, stirrup_k: 0.0, soft: p.f("softweight", 0.8)?.clamp(0.0, 1.0),
            hip: (p.f("hippitchstart", -0.5)?.clamp(-1.0, 1.0), p.f("hippitchramp", 0.25)?.max(0.0)),
            straight: (p.f("straightnessstart", -0.1)?.clamp(-1.0, 1.0), p.f("straightnessramp", 0.35)?.max(0.0)),
            fail: p.f("failangle", 0.71)?, seat_fail: p.f("seatfailangle", 0.71)?, slump: p.b("Slump", false)?,
            drag: (p.b("drag", false)?, p.f("dragTime", 10.0)?.max(0.0), p.b("dragreach", true)?, p.f("dragreachdelay", 0.2)?.max(0.0),
                   p.f("dragstiffness", 9.0)?),
            t: 0.0, from: None, thrown: None, throw: false, knock: p.f("knockSpeed", 4.0)?.max(0.0), fling: (p.f("throwSpeed", 3.0)?.max(0.0), Vec3::ZERO), cons: vec![], out: vec![],
            r2,
        };
        let stirrup = p.f("constrainstrengthstirrup", 40.0)?;
        (s.feet, s.stirrup_k) = (stirrup > 0.0, stirrup * 10.0);
        for n in ["posefailangle", "standfailangle", "rolldelay", "rollspeedthreshold", "healthfaildelay"] {
            p.f(n, 0.0)?;
        }
        s.update(p)?;
        Ok(s)
    }

    fn seat_frame(&self) -> (Vec3, Vec3, Vec3) {
        let up = self.up.normalize_or(Vec3::Z);
        let fwd = (self.fwd - up * self.fwd.dot(up)).normalize_or(Vec3::X);
        (up, fwd, up.cross(fwd))
    }

    /// Where each foot goes: given, or under the seat to each side.
    fn stirrup(&self, i: usize) -> Vec3 {
        let (up, fwd, left) = self.seat_frame();
        if self.given[2 + i] { self.stirrups[i] } else { self.seat - up * 0.6 + left * (0.26 - 0.52 * i as f32) + fwd * 0.18 }
    }
}

impl Extra for Sit {
    fn update(&mut self, p: &mut P) -> Result<(), Error> {
        for (i, n) in ["seatPos", "seatForward", "stirrupL", "stirrupR"].iter().enumerate() {
            let v = p.v(n, Vec3::ZERO)?;
            if v != Vec3::ZERO {
                match i {
                    0 => self.seat = v,
                    1 => self.fwd = v,
                    _ => self.stirrups[i - 2] = v,
                }
                self.given[i] = true;
            }
        }
        self.up = p.v("seatUp", self.up)?;
        self.throw |= p.b("throw", false)?;
        self.r2.health = p.f("health", self.r2.health)?;
        Ok(())
    }

    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        if c.ch.rdr2 {
            return self.tick_rdr2(c, dt);
        }
        self.t += dt;
        let at = |k: usize| c.b.part(k).m.d;
        let (pelvis, l_foot, r_foot) = (at(0), at(3), at(6));
        let from = *self.from.get_or_insert([pelvis, l_foot, r_foot]);
        if !self.given[0] {
            self.seat = from[0];
            self.given[0] = true;
        }
        if !self.given[1] {
            let left = (at(13) - at(17)).normalize_or(Vec3::Y);
            self.fwd = left.cross(c.up);
            self.given[1] = true;
        }
        let (up, fwd, left) = self.seat_frame();
        c.out.dbm = Some(false);
        if let Some((t0, caught)) = self.thrown {
            // thrown: let go, or dragged by a caught foot
            let since = self.t - t0;
            self.cons.clear();
            // flung off: pelvis and chest pushed for FLING s, enough to move his whole body at throwSpeed
            if since < FLING && self.fling.1 != Vec3::ZERO {
                let f = self.fling.1 * (self.fling.0 * FLUNG_MASS / FLING * 0.5);
                c.out.force(0, f);
                c.out.force(10, f);
            }
            if caught.is_some() && since >= self.drag.1 && since - dt < self.drag.1 {
                self.out.push("#say rdr2Sit: the foot came free".into());
                self.out.push("catchFall start=true".into());
            }
            if let Some(i) = caught.filter(|_| since < self.drag.1) {
                self.cons.push((SIT_PARTS[1 + i], Some(self.stirrup(i)), None));
                c.ch.set_stiffness(self.drag.4, 1.0, "fb", None);
                if self.drag.2 && since > self.drag.3 {
                    let foot = at(SIT_PARTS[1 + i]);
                    for a in 0..2 {
                        let l = crate::nmshot::limb(a);
                        crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[delirium_data::kv!(0 int)], foot, 0.0, 0.0, 1.0 - 2.0 * a as f32, 1.0, Vec3::ZERO, None);
                    }
                }
            }
            return;
        }
        // into the seat: swung up and over it
        let s = if self.mount > 0.0 { (self.t / self.mount).min(1.0) } else { 1.0 };
        let ease = s * s * (3.0 - 2.0 * s);
        let arc = up * 0.7 * (std::f32::consts::PI * s).sin();
        // fairly in the seat: swung in and settled (his upper body still swings as he lands)
        let seated = s >= 1.0 && self.t > self.mount + SETTLE;
        // thrown?
        let body = (at(12) - pelvis).normalize_or(up);
        let why = if self.throw {
            Some("thrown")
        } else if seated && up.dot(Vec3::Z) < self.seat_fail {
            Some("the seat tipped")
        } else if seated && !self.slump && body.dot(up) < self.fail {
            Some("leant too far")
        } else if seated && self.knock > 0.0 && (c.b.part(10).v - c.b.part(0).v).length() > self.knock {
            Some("knocked off")
        } else {
            None
        };
        if let Some(why) = why {
            // the foot that's dragged: the lower one
            let caught = self.drag.0.then(|| if l_foot.dot(Vec3::Z) < r_foot.dot(Vec3::Z) { 0 } else { 1 });
            self.thrown = Some((self.t, caught));
            let side = if (at(12) - pelvis).dot(left) >= 0.0 { left } else { -left };
            self.fling.1 = (side + up * 0.6 - fwd * 0.2).normalize();
            self.cons.clear();
            self.out.push(format!("#say rdr2Sit: {why}{}", if caught.is_some() { ", a foot caught" } else { "" }));
            if caught.is_none() {
                self.out.push("catchFall start=true".into());
            }
            return;
        }
        self.cons.clear();
        if self.constrain {
            self.cons.push((0, Some(from[0].lerp(self.seat, ease) + arc), None));
        }
        // the feet: springs into the stirrups (held hard, his legs levered him over)
        if self.feet {
            for i in 0..2 {
                let k = SIT_PARTS[1 + i];
                let to = from[1 + i].lerp(self.stirrup(i), ease) + arc;
                let f = (to - at(k)) * self.stirrup_k - c.b.part(k).v * 2.0 * self.stirrup_k.sqrt();
                c.out.force(k, f.clamp_length_max(self.stirrup_k * 0.5));
            }
        }
        // hips square to the seat, tipped by hippitchstart
        let w = |ramp: f32| if ramp > 0.0 { (self.t / ramp).min(1.0) } else { 1.0 };
        let pitch = self.hip.0 * 0.5 * w(self.hip.1);
        // (a turn about his left tips his top forward)
        let want_up = bevy_math::Quat::from_axis_angle(left, pitch) * up;
        let hips_up = (at(7) - pelvis).normalize_or(up);
        let hips_left = (at(1) - at(4)).normalize_or(left);
        let k = self.square * 4.0 * 3.0;
        let tq = hips_up.cross(want_up) * k + hips_left.cross(left) * k - c.b.part(0).w * 2.0 * (k * 3.0).sqrt();
        c.out.torque(0, tq);
        // his chest held upright over the hips, softly
        if !self.slump && self.soft > 0.0 {
            let lean = bevy_math::Quat::from_axis_angle(left, -self.straight.0 * 0.3 * w(self.straight.1));
            let chest = (at(10) - pelvis).normalize_or(up);
            let k = self.soft * CHEST_HOLD;
            c.out.torque(10, chest.cross(lean * want_up) * k - c.b.part(10).w * 2.0 * (k * 4.0).sqrt());
        }
        // his back
        let n = SPINE().len() as f32;
        for &j in SPINE().iter() {
            c.ch.eff[j].offset[0] += self.straight.0 * 0.4 * w(self.straight.1) / n;
        }
        if self.slump {
            c.ch.set_stiffness(6.0, 0.8, "us", None);
            c.ch.set_stiffness(3.0, 0.8, "ul", None);
            c.ch.set_stiffness(3.0, 0.8, "ur", None);
            return;
        }
        c.ch.set_stiffness(11.0, 1.0, "us", None);
        c.ch.set_stiffness(9.0, 1.0, "ll", None);
        c.ch.set_stiffness(9.0, 1.0, "lr", None);
        // hands forward on the reins
        for i in 0..2 {
            let l = crate::nmshot::limb(i);
            let to = self.seat + fwd * 0.38 + up * 0.32 + left * (0.1 - 0.2 * i as f32);
            crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[delirium_data::kv!(0 int)], to, 0.0, 0.0, 1.0 - 2.0 * i as f32, 1.0, Vec3::ZERO, None);
        }
    }

    fn constraints(&self) -> Vec<(usize, Option<Vec3>, Option<bevy_math::Quat>)> {
        self.cons.clone()
    }

    fn lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.out)
    }

    fn done(&self) -> bool {
        // (flung first)
        self.thrown.is_some_and(|(t0, caught)| self.t - t0 >= if caught.is_none() { FLING } else { self.drag.1 })
    }

    fn ropes(&self) -> Vec<(usize, Vec3, Vec3, f32, f32)> {
        self.r2.ropes.clone()
    }

    fn contact_off(&self) -> u32 {
        self.r2.contact_off
    }

    fn bullet(&mut self) {
        self.r2.bullet = true;
    }

    fn probe(&mut self, c: &Ctx, world: &crate::collision::Collision, _obs: &[crate::ragdoll::Obstacle]) {
        if c.ch.rdr2 {
            self.r2.probe(c, world);
        }
    }

    fn stop(&mut self, c: &mut Ctx) {
        if c.ch.rdr2 {
            self.r2.deactivate(c, &mut self.out);
        }
    }
}

impl Sit {
    /// RDR2 core. RDR2's sit starts with him in the seat (the game's animation puts him there); Delirium's swing up
    /// over mountTime stands in for that, then RDR2's sit runs (`SitRdr2`).
    fn tick_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        self.t += dt;
        let at = |k: usize| c.b.part(k).m.d;
        let from = *self.from.get_or_insert([at(0), at(3), at(6)]);
        if !self.given[0] {
            self.seat = from[0];
            self.given[0] = true;
        }
        if !self.given[1] {
            let left = (at(13) - at(17)).normalize_or(Vec3::Y);
            self.fwd = left.cross(c.up);
            self.given[1] = true;
        }
        let (up, fwd, left) = self.seat_frame();
        self.cons.clear();
        if !self.r2.on && self.r2.mount > 0.0 && self.t < self.r2.mount + SETTLE && !self.throw {
            c.out.dbm = Some(false);
            let s = (self.t / self.r2.mount).min(1.0);
            let ease = s * s * (3.0 - 2.0 * s);
            let arc = up * 0.7 * (std::f32::consts::PI * s).sin();
            self.cons.push((0, Some(from[0].lerp(self.seat, ease) + arc), None));
            if self.feet {
                for i in 0..2 {
                    let k = SIT_PARTS[1 + i];
                    let to = from[1 + i].lerp(self.stirrup(i), ease) + arc;
                    let f = (to - at(k)) * self.stirrup_k - c.b.part(k).v * 2.0 * self.stirrup_k.sqrt();
                    c.out.force(k, f.clamp_length_max(self.stirrup_k * 0.5));
                }
            }
            if s >= 1.0 {
                // settling into the seat, hips square and chest upright (Delirium's seated posture, as without the
                // core), so RDR2's sit starts with him sitting as its animation would have him
                let pelvis = at(0);
                let want_up = bevy_math::Quat::from_axis_angle(left, self.hip.0 * 0.5) * up;
                let hips_up = (at(7) - pelvis).normalize_or(up);
                let hips_left = (at(1) - at(4)).normalize_or(left);
                let k = self.square * 4.0 * 3.0;
                c.out.torque(0, hips_up.cross(want_up) * k + hips_left.cross(left) * k - c.b.part(0).w * 2.0 * (k * 3.0).sqrt());
                let chest = (at(10) - pelvis).normalize_or(up);
                let k = CHEST_HOLD;
                c.out.torque(10, chest.cross(up) * k - c.b.part(10).w * 2.0 * (k * 4.0).sqrt());
            }
            return;
        }
        let q = bevy_math::Quat::from_mat3(&bevy_math::Mat3::from_cols(fwd, left, up));
        let throw = std::mem::take(&mut self.throw);
        self.r2.tick(c, dt, self.seat, q, throw, &mut self.out, &mut self.cons);
    }
}

/// A constraint of RDR2's sit: our part, the point on it (in its frame) and the other end, in the seat's frame
/// (carried with the seat) or the world's.
#[derive(Clone, Copy, Debug)]
struct SitCon {
    part: usize,
    local: Vec3,
    anchor: Vec3,
    carried: bool,
}

/// The seat this step, world: where it is, its turn, its velocity, and the rider's seat frame's rows (his up, his
/// left, his back: RDR2's pelvis frame at activation, carried by the seat).
#[derive(Clone, Copy, Debug)]
struct SeatNow {
    at: Vec3,
    q: bevy_math::Quat,
    up: Vec3,
    vel: Vec3,
    rows: [Vec3; 3],
}

/// The sit tests of one step (rdr2_sit.md §4.3, §6.1).
#[derive(Clone, Copy, Debug)]
struct SitTests {
    seat_ok: bool,
    trunk: f32,
    pose_ok: bool,
    in_box: bool,
    rel: f32,
    below: f32,
    health_fail: bool,
}

/// The state machine's exits (RDR2's row table 0x3cc7cf0), tried in order once a state is done.
const SIT_ROWS: [&[usize]; 8] = [&[2], &[2], &[7, 3, 5, 4], &[6, 4], &[6, 3], &[], &[3, 4], &[]];
/// RDR2's knee offset added to the activation knee angle (a tunable global, 0 in the dump).
const SIT_KNEE: f32 = 0.0;
/// The constraint slots' length rows: pelvis, right shin, left shin, the pelvis's turn.
const SIT_LEN: [usize; 4] = [0, 1, 1, 2];

#[allow(non_snake_case)]
#[inline]
fn SIT_EVENT() -> u8 { delirium_data::kv!(127 int) }
#[allow(non_snake_case)]
#[inline]
fn SIT_SUCCESS() -> u8 { delirium_data::kv!(55 int) }

fn sit_v(v: Vec3) -> String {
    format!("{},{},{}", v.x, v.y, v.z)
}

/// RDR2's sit (slot 0x108, rdr2_sit.md), with the RDR2 core. His pelvis and both shins are held to the seat by
/// distance constraints that start with 0.2 m of slack and tighten at 0.1 m/s to 0.05 m, breaking at
/// constrainstrength / constrainstrengthstirrup (unless neverreleaseconstraint), and his pelvis's turn to the seat's by
/// an angle constraint; a bullet gives them their slack back. While riding his spine leans toward the animation's
/// trunk (or the sky), his hips keep their angle and his knees their activation bend, his hands go to the reins, and
/// the balancer's hip pitch and knee bend are asked -0.5 / -0.1. RDR2's 8-state machine decides how he leaves: fall
/// (fallingReaction), roll off and slump (health), dragged by a stirrup (dragreaction, dragreach), or on his feet.
///
/// Delirium's: the horse is no body his solver holds him to, so every anchor is a world point moved with the seat
/// (RDR2's constraintoworld; RDR2's constrain 2 hangs them on the horse's body) and the horse isn't told
/// (defineAttachedObject: Delirium's horse carries no rider's mass). RDR2's distance constraints are Delirium's grip
/// ropes (a breaking strength on the solver's impulse per step); its angle constraint, which Delirium lacks, is a hard
/// hold of the pelvis's turn at the limit once he's past it. Which RDR2 pelvis axis is which is the note's guess: his
/// up, side (the seat box's 0.3 m and the hands' side) and back (the hands 0.5 m ahead); rolling off goes to his side.
/// RDR2's hip targets (the thighs kept at their angle to the trunk) are held at their activation angles here. The
/// three ground probes are ours (down from his pelvis and 0.5 m to each side). RDR2's flag F (mgr+0x5229) is taken as
/// off. Left out: the collision radius scales, the seat instance list, the keyframed state's joint limits and its
/// soft-key settings, the incoming-pose drive (no riding animation here), bodyBalance's noisy elbows and mask, the
/// shot made subordinate, the drag block override (+0x170).
pub struct SitRdr2 {
    mount: f32,
    constrain: i32,
    strength: (f32, f32),
    never_release: bool,
    /// (start, final, rate): pelvis, stirrups, the pelvis's turn
    lens: [(f32, f32, f32); 3],
    /// failangle, posefailangle, seatfailangle, standfailangle
    fail: [f32; 4],
    health_delay: f32,
    health_now: bool,
    /// rolldelay, rollspeedthreshold
    roll: (f32, f32),
    seat_box: (f32, f32),
    /// rdsforcemagstart, rdsforcemagramp, rdsForceMag
    rds: (f32, f32, f32),
    roll_dir: Vec3,
    use_roll_dir: bool,
    rds_min: f32,
    falling: i32,
    slump: bool,
    hip: (f32, f32),
    straight: (f32, f32),
    drag: bool,
    reach: bool,
    drag_time: f32,
    drag_reaction: i32,
    reach_chance: f32,
    reach_delay: f32,
    drag_slack: f32,
    drag_k: f32,
    keys: [String; 3],
    soft: (f32, f32),
    spine_anims: bool,
    pub health: f32,
    on: bool,
    t: f32,
    state: usize,
    entered: f32,
    roll_t: f32,
    seat0: (Vec3, bevy_math::Quat),
    rider: [Vec3; 3],
    q_rel: bevy_math::Quat,
    cons: [Option<SitCon>; 4],
    len: [f32; 4],
    knee: f32,
    hips: [[f32; 3]; 2],
    seeds: [f32; 3],
    fail_t: f32,
    healthy: bool,
    probes: [f32; 3],
    kids: Vec<&'static str>,
    rds_t: Option<f32>,
    prev_seat: Option<Vec3>,
    last: Option<SeatNow>,
    pub ropes: Vec<(usize, Vec3, Vec3, f32, f32)>,
    pub contact_off: u32,
    pub bullet: bool,
}

impl SitRdr2 {
    fn new(p: &mut P, mount: f32, constrain: i32) -> Result<SitRdr2, Error> {
        // taken, not used: the seat's frame is given in the world here, and there's no riding animation
        let _ = (p.v("seatuplocal", Vec3::NEG_Y)?, p.v("seatrightlocal", Vec3::X)?, p.s("activePoseMask", "fb")?);
        let _ = (p.f("softbreakthreshold", 0.15)?, p.v("softmaxaccel", Vec3::splat(50.0))?, p.v("softmaxangaccel", Vec3::splat(50.0))?);
        let _ = (p.b("stateridingtarget", false)?, p.i("animSource", 0)?, p.b("constraintsfollowanims", false)?);
        let _ = (p.f("pelvisradiusscale", 1.0)?, p.f("thighradiusscale", 1.0)?, p.b("constraintoworld", false)?);
        let slump = p.b("Slump", false)?;
        Ok(SitRdr2 {
            mount, constrain,
            strength: (p.f("constrainstrength", 40.0)?, p.f("constrainstrengthstirrup", 40.0)?),
            never_release: p.b("neverreleaseconstraint", false)?,
            lens: [(p.f("pelvisLengthStart", 0.2)?, p.f("pelvisLength", 0.05)?, p.f("pelvisRate", 0.1)?),
                   (p.f("stirrupLengthStart", 0.2)?, p.f("stirrupLength", 0.05)?, p.f("stirrupRate", 0.1)?),
                   (p.f("orientLengthStart", 0.2)?, p.f("orientLength", 0.0)?, p.f("orientRate", 0.1)?)],
            fail: [p.f("failangle", 0.71)?, p.f("posefailangle", 0.71)?, p.f("seatfailangle", 0.71)?, p.f("standfailangle", 0.71)?],
            health_delay: p.f("healthfaildelay", 0.0)?, health_now: p.b("healthFailNow", true)?,
            roll: (p.f("rolldelay", 0.3)?, p.f("rollspeedthreshold", 2.0)?),
            seat_box: (p.f("seatBoxSide", 0.3)?, p.f("seatBoxLength", 0.75)?),
            rds: (p.f("rdsforcemagstart", 0.5)?, p.f("rdsforcemagramp", 0.05)?, p.f("rdsForceMag", 2.0)?),
            roll_dir: p.v("rollDir", Vec3::Z)?, use_roll_dir: p.b("useRollDir", false)?, rds_min: p.f("rdsMinAngVel", 0.3)?,
            falling: p.i("fallingReaction", 0)?, slump: p.b("slump", slump)?,
            hip: (p.f("hippitchstart", -0.5)?, p.f("hippitchramp", 0.25)?),
            straight: (p.f("straightnessstart", -0.1)?, p.f("straightnessramp", 0.35)?),
            drag: p.b("drag", false)?, reach: p.b("dragreach", true)?, drag_time: p.f("dragTime", 10.0)?,
            drag_reaction: p.i("dragreaction", 1)?, reach_chance: p.f("dragReachChance", 0.5)?,
            reach_delay: p.f("dragreachdelay", 0.2)?, drag_slack: p.f("dragSlack", 0.0)?, drag_k: p.f("dragstiffness", 9.0)?,
            keys: [p.s("hardKeyMask", "na")?, p.s("softKeyMask", "na")?, p.s("softKeyMaskAlt", "na")?],
            soft: (p.f("softweight", 0.8)?, p.f("softweightalt", 0.8)?), spine_anims: p.b("spinefollowanims", true)?,
            health: p.f("health", 1.0)?,
            on: false, t: 0.0, state: 2, entered: 0.0, roll_t: 0.0, seat0: (Vec3::ZERO, bevy_math::Quat::IDENTITY),
            rider: [Vec3::Z, Vec3::Y, Vec3::NEG_X], q_rel: bevy_math::Quat::IDENTITY, cons: [None; 4], len: [0.0; 4],
            knee: 0.0, hips: [[0.0; 3]; 2], seeds: [0.0; 3], fail_t: 1000.0, healthy: true, probes: [f32::MAX; 3], kids: vec![],
            rds_t: None, prev_seat: None, last: None, ropes: vec![], contact_off: 0, bullet: false,
        })
    }

    fn keyed(&self) -> bool {
        self.keys.iter().any(|k| !matches!(k.as_str(), "" | "na"))
    }

    fn start(&mut self, out: &mut Vec<String>, name: &'static str, line: String) {
        if !self.kids.contains(&name) {
            self.kids.push(name);
        }
        out.push(line);
    }

    fn stop_kid(&mut self, out: &mut Vec<String>, name: &'static str) {
        if let Some(i) = self.kids.iter().position(|n| *n == name) {
            self.kids.remove(i);
            out.push(format!("{name} start=false"));
        }
    }

    /// RDR2's 0x2a800e8: every behaviour sit started is stopped.
    fn stop_kids(&mut self, out: &mut Vec<String>) {
        for n in std::mem::take(&mut self.kids) {
            out.push(format!("{n} start=false"));
        }
    }

    /// Activation (RDR2's 0x2a96df0), with him in the seat.
    fn activate(&mut self, c: &mut Ctx, seat: Vec3, q: bevy_math::Quat, out: &mut Vec<String>) {
        use crate::extras_ysk_rdr2::rp;
        self.on = true;
        self.t = 0.0;
        self.seeds = [c.ch.random() * 4000.0, c.ch.random() * 4000.0, 0.0];
        (self.fail_t, self.healthy) = (1000.0, true);
        self.knee = c.ch.actual(KNEE()[0], c.b.frames)[0];
        self.hips = [c.ch.actual(HIP()[0], c.b.frames), c.ch.actual(HIP()[1], c.b.frames)];
        // the rider's seat frame: his pelvis's frame now, in the seat's
        let pelvis = c.b.part(0).m.d;
        let left = (c.b.joint_pos(HIP()[0]) - c.b.joint_pos(HIP()[1])).normalize_or(q * Vec3::Y);
        let up = (c.b.part(rp(10)).m.d - pelvis).reject_from_normalized(left).normalize_or(q * Vec3::Z);
        let qi = q.inverse();
        self.rider = [qi * up, qi * left, qi * up.cross(left)];
        self.seat0 = (seat, q);
        self.q_rel = qi * c.b.frames[0].rotation;
        // the balancer's hip pitch and knee bend at their start values (our hip sign is RDR2's negated); every
        // riding state's enter asks RDR2's constants at once, so these hold for no step, as RDR2's code reads
        c.out.dbm_crouch.push(Crouch::Hip(11, -self.hip.0.clamp(-2.0, 2.0), 100000.0));
        c.out.dbm_crouch.push(Crouch::Knee(10, self.straight.0, 100000.0));
        self.state = if self.constrain != 0 { 0 } else if self.keyed() { 1 } else { 2 };
        self.entered = 0.0;
        self.stop_kids(out);
        let s = self.seat_now(seat, q, Vec3::ZERO);
        self.enter(c, self.state, &s, out);
    }

    fn seat_now(&self, at: Vec3, q: bevy_math::Quat, vel: Vec3) -> SeatNow {
        SeatNow { at, q, up: q * Vec3::Z, vel, rows: self.rider.map(|r| q * r) }
    }

    fn tick(&mut self, c: &mut Ctx, dt: f32, seat: Vec3, q: bevy_math::Quat, throw: bool, out: &mut Vec<String>,
            cons: &mut Vec<(usize, Option<Vec3>, Option<bevy_math::Quat>)>) {
        use crate::extras_ysk_rdr2::rp;
        let vel = self.prev_seat.map_or(Vec3::ZERO, |p| (seat - p) / dt.max(1e-6));
        self.prev_seat = Some(seat);
        if !self.on {
            self.activate(c, seat, q, out);
        }
        self.t += dt;
        let s = self.seat_now(seat, q, vel);
        // a constraint whose rope broke last step is gone
        for slot in 0..3 {
            if self.cons[slot].is_some_and(|k| c.b.doll.snapped.contains(&k.part)) {
                self.cons[slot] = None;
                out.push(format!("#say rdr2Sit: the {} constraint broke", ["pelvis", "right stirrup", "left stirrup"][slot]));
            }
        }
        // a bullet gives the seat its slack back (RDR2's 0x2a95b54, while the pelvis constraint lives)
        if std::mem::take(&mut self.bullet) && self.cons[0].is_some() {
            for slot in 0..4 {
                self.len[slot] = self.lens[SIT_LEN[slot]].0;
            }
        }
        // tightening
        for slot in 0..4 {
            if self.cons[slot].is_some() {
                let (_, to, rate) = self.lens[SIT_LEN[slot]];
                self.len[slot] = (self.len[slot] - dt * rate).max(to);
            }
        }
        // the tests
        let up = c.up;
        let pelvis = c.b.part(0).m.d;
        let top = c.b.part(rp(10)).m.d;
        let trunk = (top - pelvis).normalize_or(up);
        let pose_ok = match (c.incoming.get(0), c.incoming.get(rp(10))) {
            (Some(a), Some(b)) => trunk.dot((b.translation - a.translation).normalize_or(trunk)).clamp(-1.0, 1.0).acos() < self.fail[1],
            _ => true,
        };
        let healthy = self.health > 0.0;
        if !healthy && self.healthy {
            self.fail_t = self.t;
        }
        self.healthy = healthy;
        let d = s.at - pelvis;
        let x = SitTests {
            seat_ok: s.up.dot(up).clamp(-1.0, 1.0).acos() < self.fail[2],
            trunk: trunk.dot(up).clamp(-1.0, 1.0).acos(),
            pose_ok,
            in_box: d.dot(s.rows[1]).abs() < self.seat_box.0 && d.dot(s.rows[2]).abs() < self.seat_box.1,
            rel: (c.vel - s.vel).length(),
            below: (pelvis - s.at).dot(up),
            health_fail: !healthy && (self.health_now || self.t > self.fail_t + self.health_delay),
        };
        // Delirium's throw (the game's): out of the saddle now
        if throw && self.state <= 2 {
            out.push("#say rdr2Sit: thrown".into());
            self.exit(c, self.state, out);
            self.release(if self.drag { 0b1001 } else { 0b1111 });
            self.state = 3;
            self.entered = self.t;
            self.enter(c, 3, &s, out);
        }
        // the state machine (RDR2's 0x2aac1ac)
        if self.state <= 2 {
            // riding, the balancer stands aside [P: RDR2's shot hand-over 0x2ab9c14]
            c.out.dbm = Some(false);
        }
        self.tick_state(c, dt, &s, &x, out);
        if self.done(c, &x) {
            for &n in SIT_ROWS[self.state] {
                if self.can_enter(c, dt, n, &x) {
                    let old = self.state;
                    self.exit(c, old, out);
                    self.state = n;
                    self.entered = self.t;
                    self.enter(c, n, &s, out);
                    break;
                }
            }
        }
        // the constraints, to the solver
        self.ropes.clear();
        for slot in 0..3 {
            let Some(k) = self.cons[slot] else { continue };
            let f = c.b.frames[k.part];
            let to = if k.carried { s.at + s.q * k.anchor } else { k.anchor };
            let slack = if slot > 0 && self.state == 6 { self.drag_slack } else { 0.0 };
            let strength = if slot == 0 { self.strength.0 } else { self.strength.1 };
            let strength = if self.never_release || strength < 0.0 { -1.0 } else { strength };
            self.ropes.push((k.part, f.translation + f.rotation * k.local, to, self.len[slot] + slack, strength));
        }
        if self.cons[3].is_some() {
            // the pelvis's turn: held at the limit once past it (constrain 1: RDR2's three axes at 0.001)
            let want = s.q * self.q_rel;
            let now = c.b.frames[0].rotation;
            let dq = want.inverse() * now;
            let dq = if dq.w < 0.0 { -dq } else { dq };
            let (axis, angle) = dq.normalize().to_axis_angle();
            let lim = if self.constrain == 1 { 0.001 } else { self.len[3] };
            if angle > lim && angle.is_finite() {
                cons.push((0, None, Some(want * bevy_math::Quat::from_axis_angle(axis, lim))));
            }
        }
        self.contact_off = 0;
        if self.constrain == 1 && self.cons[0].is_some() {
            self.contact_off |= 1;
        }
        if self.state == 6 {
            for (slot, foot) in [(1, rp(7)), (2, rp(5))] {
                if self.cons[slot].is_some() {
                    self.contact_off |= 1 << foot;
                }
            }
        }
        self.last = Some(s);
    }

    fn alive(&self, mask: u32) -> bool {
        (0..4).any(|i| mask >> i & 1 != 0 && self.cons[i].is_some())
    }

    fn release(&mut self, mask: u32) {
        for i in 0..4 {
            if mask >> i & 1 != 0 {
                self.cons[i] = None;
            }
        }
    }

    /// RDR2's 0x2aa7354: every constraint let go, the caught foot collides again, and the "success".
    fn drop_all(&mut self, c: &mut Ctx) {
        self.release(0b1111);
        c.out.send(SIT_EVENT(), "sit", 0.0);
        c.out.send(SIT_SUCCESS(), "sit", 0.0);
    }

    /// RDR2's 0x2a90eb8 / 0x2a91fa4 / 0x2a91b90: the seat's constraints, at their start lengths.
    fn make(&mut self, c: &Ctx, s: &SeatNow) {
        use crate::extras_ysk_rdr2::rp;
        let qi = s.q.inverse();
        let con = |part: usize, point: Vec3, carried: bool| {
            let f = c.b.frames[part];
            SitCon { part, local: f.rotation.inverse() * (point - f.translation), anchor: if carried { qi * (point - s.at) } else { point }, carried }
        };
        let pelvis = c.b.part(0).m.d;
        if self.constrain == 2 {
            let b = self.lens[1].0;
            self.cons[1] = Some(con(rp(6), c.b.joint_pos(ANKLE()[1]) + s.up * b, true));
            self.cons[2] = Some(con(rp(4), c.b.joint_pos(ANKLE()[0]) + s.up * b, true));
            self.cons[0] = Some(con(0, pelvis + s.up * (0.5 * self.lens[0].0), true));
        } else {
            self.cons[0] = Some(con(0, pelvis, false));
        }
        self.cons[3] = Some(SitCon { part: 0, local: Vec3::ZERO, anchor: Vec3::ZERO, carried: true });
        for slot in 0..4 {
            self.len[slot] = self.lens[SIT_LEN[slot]].0;
        }
    }

    /// RDR2's 0x2aac3c8: the balancer's hip pitch -0.5 and knee bend -0.1 (owners 11 and 10), asked every riding step.
    fn pose(&self, c: &mut Ctx) {
        c.out.dbm_crouch.push(Crouch::Hip(11, 0.5, 100000.0));
        c.out.dbm_crouch.push(Crouch::Knee(10, -0.1, 100000.0));
    }

    /// RDR2's 0x2a92b1c: both knees at the left knee's activation angle.
    fn knees(&self, c: &mut Ctx) {
        for &j in KNEE().iter() {
            c.ch.eff[j].desired[0] = SIT_KNEE + self.knee;
        }
    }

    /// RDR2's 0x2aa2a48: his spine leans toward the animation's trunk (or the sky) by twice his trunk's error, split
    /// over the spine; with constrain 2 his hips are held (ours: at their activation angles) at 15, damping 1.
    fn spine(&self, c: &mut Ctx) {
        use crate::extras_ysk_rdr2::rp;
        let (pelvis, top) = (c.b.part(0).m.d, c.b.part(rp(10)).m.d);
        let u = match (c.incoming.get(0), c.incoming.get(rp(10))) {
            (Some(a), Some(b)) if self.spine_anims => (b.translation - a.translation).normalize_or(c.up),
            _ => c.up,
        };
        let trunk = (top - pelvis).normalize_or(u);
        let turn = c.b.frames[0].rotation * (self.seat0.1 * self.q_rel).inverse();
        let (pu, pl) = (turn * (self.seat0.1 * self.rider[0]), turn * (self.seat0.1 * self.rider[1]));
        let axis = pu.cross(u).normalize_or_zero();
        let amount = 2.0 * trunk.dot(u).clamp(-1.0, 1.0).acos();
        // a turn about his left leans him forward (lean1), about his front to his right (lean2 [P])
        spine_split(c, amount * axis.dot(pl), amount * axis.dot(pl.cross(pu)));
        if self.constrain == 2 {
            for i in 0..2 {
                let e = &mut c.ch.eff[HIP()[i]];
                e.desired = self.hips[i];
                e.set_stiffness(15.0, 1.0, None);
            }
        }
    }

    /// RDR2's 0x2aa6924: each hand to the reins in the rider's seat frame, unless his pelvis (0.2 s ahead, against
    /// the seat) has slid 0.1 m to that side; the elbow floor 0.28..0.6 from noise.
    fn hands(&self, c: &mut Ctx, s: &SeatNow) {
        let p = c.b.part(0).m.d + (c.b.part(0).v - s.vel) * 0.2;
        for i in 0..2 {
            let side = 1.0 - 2.0 * i as f32;
            if side * (p - s.at).dot(s.rows[1]) >= 0.1 {
                continue;
            }
            let h = s.at + s.rows[0] * 0.2 + s.rows[1] * (0.1 * side) - s.rows[2] * 0.5;
            let l = crate::nmshot::limb(i);
            crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[delirium_data::kv!(0 int)], h, 0.0, 0.0, side, 1.0, Vec3::ZERO, None);
            let floor = crate::nmfl::noise01(2.0 * self.t, self.seeds[i], 0.0) * 0.32 + 0.28;
            let e = &mut c.ch.eff[l.e[2]];
            e.desired[0] = e.desired[0].max(floor);
        }
    }

    /// Before the fall reactions: the other behaviours sit started stop (fallingReaction 0 or more), or the
    /// balancer's hip pitch and knee bend are let go at the ramps.
    fn preamble(&mut self, c: &mut Ctx, out: &mut Vec<String>) {
        if self.falling >= 0 {
            self.stop_kids(out);
        } else {
            c.out.dbm_crouch.push(Crouch::ReleaseHip(self.hip.1));
            c.out.dbm_crouch.push(Crouch::ReleaseKnee(self.straight.1));
        }
    }

    /// rollDownStairs' settings as sit sends them.
    fn rds_line(&self, force: f32, custom: bool, dir: Vec3) -> String {
        format!("rollDownStairs start=true forcemag={force} useCustomRollDir={custom} customRollDir={} minAngVel={} m_useArmToSlowDown=0 \
                 useZeroPose=false spinWhenInAir=true m_legPush=0 zAxisSpinReduction=1 targetLinearVelocity=2 targetLinearVelocityDecayTime=0.5 \
                 applyNewRollingCheatingTorques=false applyHelPerTorqueToAlign=true delayToAlignBody=0", sit_v(dir), self.rds_min)
    }

    /// RDR2's 0x2aa73dc: of two stirrups, the one further along the way he falls from the seat stays.
    fn keep_stirrup(&mut self, c: &Ctx, s: &SeatNow) {
        let (Some(r), Some(l)) = (self.cons[1], self.cons[2]) else { return };
        let d = c.b.part(0).m.d - s.at;
        let at = |k: SitCon| if k.carried { s.at + s.q * k.anchor } else { k.anchor };
        if (at(l) - s.at).dot(d) < (at(r) - s.at).dot(d) {
            self.cons[2] = None;
        } else {
            self.cons[1] = None;
        }
    }

    fn done(&self, c: &Ctx, x: &SitTests) -> bool {
        let fails = x.trunk >= self.fail[0] || !x.pose_ok || !x.seat_ok || x.health_fail;
        match self.state {
            0 => self.constrain == 0 || self.cons[0].is_none() || (fails && !self.never_release),
            1 => !(c.extras_on.contains(&"keyframe") && self.keyed()) || (fails && !self.never_release),
            2 => true,
            3 => (x.rel * x.rel < 0.25 || self.drag) && x.below < -0.02,
            4 => x.trunk >= 1.2 && x.rel >= self.roll.1 + 0.1 && !x.in_box,
            6 => self.t > self.entered + self.drag_time || !self.alive(0b1111) || !self.drag,
            _ => false,
        }
    }

    fn can_enter(&mut self, c: &Ctx, dt: f32, n: usize, x: &SitTests) -> bool {
        use crate::extras_ysk_rdr2::rp;
        match n {
            3 => !x.in_box || !x.seat_ok,
            4 => {
                if self.state == 6 || self.healthy {
                    return false;
                }
                self.roll_t += dt;
                self.roll_t > self.roll.0 && ((x.rel < self.roll.1 && x.in_box) || !x.seat_ok)
            }
            5 => self.slump && !self.healthy && self.t - self.fail_t >= self.health_delay && c.vel.length() < 0.5 && self.probes[2] < f32::MAX,
            6 => self.drag && self.alive(0b110),
            7 => {
                if x.seat_ok && x.trunk < self.fail[0] && x.pose_ok {
                    return false;
                }
                let (mut n, mut at) = (0.0, Vec3::ZERO);
                for i in 4..8 {
                    if let Some(Some((p, normal, _))) = c.b.doll.hit.get(rp(i)) {
                        if normal.dot(c.up) > 0.8 {
                            (n, at) = (n + 1.0, at + *p);
                        }
                    }
                }
                n > 0.0 && (c.com + c.vel * 0.1 - at / n).normalize_or_zero().dot(c.up) >= self.fail[3]
            }
            _ => true,
        }
    }

    fn enter(&mut self, c: &mut Ctx, n: usize, s: &SeatNow, out: &mut Vec<String>) {
        use crate::extras_ysk_rdr2::rp;
        match n {
            0 => {
                if self.constrain != 0 {
                    self.make(c, s);
                }
                self.pose(c);
            }
            1 => {
                self.pose(c);
                let [h, k, a] = self.keys.clone();
                self.start(out, "keyframe", format!("keyframe start=true hardKeyMask={h} softKeyMask={k} softKeyMaskAlt={a} softweight={} softweightalt={}",
                                                    self.soft.0, self.soft.1));
            }
            2 => self.pose(c),
            3 => {
                self.preamble(c, out);
                match self.falling {
                    0 => self.start(out, "bodyRollUp", "bodyRollUp start=true useArmToSlowDown=2 legPush=0 asymmetricalLegs=0.75".into()),
                    1 => self.start(out, "catchFall", "catchFall start=true".into()),
                    2 => {
                        let line = self.rds_line(self.rds.2, self.use_roll_dir, self.roll_dir);
                        self.start(out, "rollDownStairs", line);
                    }
                    3 => self.start(out, "smartFall", "smartFall start=true initialState=6".into()),
                    _ => {
                        c.out.dbm_crouch.push(Crouch::ReleaseHip(self.hip.1));
                        c.out.dbm_crouch.push(Crouch::ReleaseKnee(self.straight.1));
                    }
                }
                if !self.drag {
                    self.release(0b1111);
                }
                c.out.send(SIT_EVENT(), "sit", 1.0);
                out.push("#say rdr2Sit: falls off".into());
            }
            4 => {
                self.preamble(c, out);
                // the side he rolls to: his COM's, 0.2 s ahead against the seat, unless the ground decides (ours: his
                // side axis, see the struct's doc)
                let left = s.rows[1];
                let ahead = (c.vel - s.vel) * 0.2 + c.com - s.at;
                let mut side = if ahead.dot(left) >= 0.0 { 1.0 } else { -1.0 };
                let dh = self.probes[0] - self.probes[1];
                if self.probes[0] < f32::MAX && self.probes[1] < f32::MAX && dh.abs() > 0.7 {
                    side = if dh < 0.0 { 1.0 } else { -1.0 };
                }
                let r = if self.use_roll_dir { left } else { left * side };
                let line = self.rds_line(self.rds.0, true, r);
                self.start(out, "rollDownStairs", line);
                self.rds_t = Some(self.t);
                if !self.drag {
                    self.release(0b1111);
                } else {
                    // the stirrup on the far side goes
                    self.release(if r.dot(left) > 0.0 { 0b10 } else { 0b100 });
                }
                c.out.send(SIT_EVENT(), "sit", 1.0);
                out.push("#say rdr2Sit: rolls off".into());
            }
            5 => out.push("#say rdr2Sit: slumps".into()),
            6 => {
                self.keep_stirrup(c, s);
                c.ch.set_stiffness(self.drag_k, 1.0, "fb", None);
                let slot = if self.cons[2].is_some() { 2 } else { 1 };
                let foot = if slot == 2 { rp(5) } else { rp(7) };
                c.out.send(SIT_EVENT(), "sit", if slot == 2 { 1.0 } else { 2.0 });
                self.stop_kids(out);
                match self.drag_reaction {
                    0 => {
                        self.start(out, "bodyRollUp", "bodyRollUp start=true".into());
                        self.start(out, "pedalLegs", "pedalLegs start=true radius=0.6 angularSpeed=15 adaptivePedal4Dragging=true hula=true".into());
                    }
                    1 => {
                        let to = self.cons[slot].map_or(c.b.part(foot).m.d, |k| if k.carried { s.at + s.q * k.anchor } else { k.anchor });
                        self.start(out, "yanked", format!("yanked start=true ropePart={foot} ropePos={} force=0", sit_v(to)));
                    }
                    _ => self.start(out, "bodyFoetal", "bodyFoetal start=true".into()),
                }
                self.seeds[2] = c.ch.random() * 4000.0;
                out.push(format!("#say rdr2Sit: dragged by the {} foot", if slot == 2 { "left" } else { "right" }));
            }
            7 => {
                self.start(out, "bodyBalance", "bodyBalance start=true".into());
                out.push("#say rdr2Sit: lands on his feet".into());
            }
            _ => {}
        }
    }

    fn exit(&mut self, c: &mut Ctx, n: usize, out: &mut Vec<String>) {
        match n {
            0 => self.release(if self.drag && self.constrain == 2 { 0b1001 } else { 0b1111 }),
            1 => self.stop_kid(out, "keyframe"),
            4 => self.rds_t = None,
            6 => {
                self.drop_all(c);
                self.stop_kid(out, "reachForWound");
            }
            _ => {}
        }
    }

    fn tick_state(&mut self, c: &mut Ctx, dt: f32, s: &SeatNow, x: &SitTests, out: &mut Vec<String>) {
        use crate::extras_ysk_rdr2::{joints, rp};
        let _ = x;
        match self.state {
            0 | 2 => {
                self.spine(c);
                self.pose(c);
                self.knees(c);
                self.hands(c, s);
            }
            1 => {
                self.pose(c);
                self.knees(c);
            }
            3 => {
                if self.drag && self.alive(0b110) && self.cons[1].is_some() && self.cons[2].is_some() && self.t - self.entered >= 0.2 {
                    self.keep_stirrup(c, s);
                }
            }
            4 => {
                if let Some(t0) = self.rds_t {
                    let since = self.t - t0;
                    if since <= self.rds.1 + dt {
                        let k = if self.rds.1 > 0.0 { (since / self.rds.1).clamp(0.0, 1.0) } else { 1.0 };
                        out.push(format!("rollDownStairs forcemag={}", self.rds.0 + (self.rds.2 - self.rds.0) * k));
                    }
                }
            }
            5 => {
                if self.falling >= 0 {
                    self.stop_kids(out);
                }
                spine_split(c, 0.9, 0.0);
                for j in joints(0x1fff09) {
                    if j < c.ch.eff.len() {
                        c.ch.eff[j].set_stiffness(4.0, 0.5, None);
                    }
                }
            }
            6 => {
                let slot = if self.cons[2].is_some() { 2 } else if self.cons[1].is_some() { 1 } else { return };
                let foot = if slot == 2 { rp(5) } else { rp(7) };
                if self.drag_reaction == 1 {
                    // the free leg, by how face-up he lies (his front row [P])
                    let f = (c.m.c.dot(c.up) + 1.0) * 0.5;
                    let leg = if slot == 2 { 1 } else { 0 };
                    let (h, k, a) = (HIP()[leg], KNEE()[leg], ANKLE()[leg]);
                    for j in [h, k, a] {
                        c.ch.eff[j].set_stiffness(self.drag_k, 1.0, None);
                    }
                    let tw = 2.0 - 1.5 * f;
                    c.ch.eff[h].desired = [2.0 * (1.0 - f), 0.7 * (1.0 - f), tw];
                    c.ch.eff[k].desired[0] = (2.0 - f) * knee_dir(c, k);
                    c.ch.eff[a].desired = [0.5 - 1.5 * f, c.ch.eff[a].desired[1], tw];
                }
                if self.reach && self.t - self.entered > self.reach_delay {
                    let n = crate::nmfl::noise01(self.t, self.seeds[2], 0.0);
                    if n >= self.reach_chance {
                        self.stop_kid(out, "reachForWound");
                    } else if !self.kids.contains(&"reachForWound") {
                        let at = c.b.part(foot).m.d;
                        self.start(out, "reachForWound", format!("reachForWound start=true hitPart={foot} hitPoint={}", sit_v(at)));
                    }
                }
            }
            _ => {}
        }
    }

    /// The three ground samples state 2 takes (ours: down from his pelvis 0.5 m to his left, right, and under it).
    fn probe(&mut self, c: &Ctx, world: &crate::collision::Collision) {
        let Some(s) = self.last.filter(|_| self.on && self.state == 2) else { return };
        let pelvis = c.b.part(0).m.d;
        for (i, off) in [0.5, -0.5, 0.0].into_iter().enumerate() {
            let from = pelvis + s.rows[1] * off;
            self.probes[i] = world.contact(from, -c.up, 3.0).map_or(f32::MAX, |(t, _, _)| (from - c.up * t).dot(c.up));
        }
    }

    /// RDR2's deactivate and cleanup: the balancer's hip pitch and knee bend let go at the ramps, the behaviours sit
    /// started stopped, the constraints let go.
    fn deactivate(&mut self, c: &mut Ctx, out: &mut Vec<String>) {
        if !self.on {
            return;
        }
        c.out.dbm_crouch.push(Crouch::ReleaseHip(self.hip.1));
        c.out.dbm_crouch.push(Crouch::ReleaseKnee(self.straight.1));
        self.stop_kids(out);
        self.release(0b1111);
        self.ropes.clear();
        self.contact_off = 0;
        self.on = false;
    }
}

/// RDR2's bolas, written from how it looks (its own parameters aren't known, so these are Delirium's): the bolas
/// wraps his legs, holding his ankles (and less, his knees) together; after fallTime his balance gives and he
/// topples, arms thrown out; down, he struggles, both legs kicking together, his arms pushing and his trunk
/// twisting, for struggleTime, then he's free (the bolas lets go).
pub struct Bolas {
    bind: f32, fall: f32, time: f32, struggle: f32, t: f32,
}

impl Bolas {
    fn new(p: &mut P) -> Result<Bolas, Error> {
        Ok(Bolas {
            bind: p.f("bindStrength", 900.0)?.max(0.0), fall: p.f("fallTime", 0.3)?.max(0.0), time: p.f("struggleTime", 5.0)?.max(0.0),
            struggle: p.f("struggle", 0.7)?.clamp(0.0, 1.0), t: 0.0,
        })
    }
}

impl Extra for Bolas {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        use std::f32::consts::TAU;
        self.t += dt;
        // the cords: ankles held together, knees less so
        for (a, b, k) in [(3, 6, 1.0), (2, 5, 0.5)] {
            let d = c.b.part(b).m.d - c.b.part(a).m.d;
            let f = (d * self.bind * k).clamp_length_max(self.bind);
            c.out.force(a, f);
            c.out.force(b, -f);
        }
        if self.t < self.fall {
            return;
        }
        c.out.dbm = Some(false);
        let up = c.up;
        let down = c.b.part(0).m.d.dot(up) < 0.45;
        let since = self.t - self.fall;
        if !down && since < 1.5 {
            // toppling: arms thrown out
            for i in 0..2 {
                let l = crate::nmshot::limb(i);
                drive(c, l.e[1], 0, 0.7);
                drive(c, l.e[1], 1, 0.6);
                c.ch.set_stiffness(11.0, 1.0, ["ul", "ur"][i], None);
            }
            return;
        }
        // down: struggling, the bound legs kicking together, arms pushing, trunk twisting
        let s = self.struggle;
        let ph = TAU * 1.1 * since;
        for side in 0..2 {
            let (hip, knee) = (HIP()[side], KNEE()[side]);
            let kd = knee_dir(c, knee);
            c.ch.eff[hip].offset[0] += 1.3 * s * ph.sin();
            c.ch.eff[knee].offset[0] += s * (0.9 + 0.9 * (ph + 0.8).sin()) * kd;
        }
        for i in 0..2 {
            let l = crate::nmshot::limb(i);
            let ed = knee_dir(c, l.e[2]);
            drive(c, l.e[1], 0, 0.9 * s * (ph * 0.9 + i as f32).sin());
            drive(c, l.e[1], 1, 0.6 * s * (ph * 0.7 + i as f32 * 1.5).sin());
            drive(c, l.e[2], 0, (0.2 + 0.7 * s * (ph * 1.3 + i as f32 * 2.0).sin()) * ed);
        }
        let n = SPINE().len() as f32;
        for &j in SPINE().iter() {
            c.ch.eff[j].offset[2] += 0.5 * s * (ph * 0.6).sin() / n;
            c.ch.eff[j].offset[1] += 0.3 * s * (ph * 0.8 + 1.0).sin() / n;
        }
        c.ch.set_stiffness(13.0, 0.8, "fb", None);
    }

    fn done(&self) -> bool {
        self.time > 0.0 && self.t > self.fall + self.time
    }
}

// ---- RDR2's own shot hit modules (with the RDR2 core), from a study of RDR2's shot (rdr2_shot_hits.md) ----

/// RDR2's spine writer for a lean: lean1 on spine joints 0..3 in shares 1/3, 1/3, 1/6, 1/6 (lean2 likewise).
pub(crate) fn spine_split(c: &mut Ctx, lean1: f32, lean2: f32) {
    let (v, w) = (lean1 / 6.0, lean2 * 0.25);
    for (i, &j) in SPINE().iter().enumerate() {
        let k = if i < 2 { 2.0 } else { 1.0 };
        c.ch.eff[j].desired[0] = k * v;
        c.ch.eff[j].desired[1] = k * w;
    }
}

/// RDR2's shot module stiffness for the spine and for the neck and head (13, damping 0.5).
const RDR2_SHOT_SPINE: f32 = 13.0;

/// RDR2's from-behind arm pose, (clavicle, shoulder, elbow, wrist) per arm, as (lean1, lean2, twist).
const SFB_ARMS: [([f32; 3], [f32; 3], f32, [f32; 3]); 2] = [
    ([-0.4, -0.15, 0.0], [0.0, 0.0, -0.35], 1.2, [-0.4, -0.65, 0.4]),
    ([-0.5, -0.15, 0.0], [0.05, 0.0, -0.4], 1.2, [-0.25, -0.75, 0.4]),
];

impl FromBehind {
    /// RDR2: at full strength on the hit, falling linearly to nothing over sfbPeriod. The spine arches back, the neck
    /// and head go back, the balancer pitches the hips (owner 9) and, from sfbKneesOnset, gives at the knees (owner 6);
    /// the arms are thrown back into a fixed pose from sfbArmsOnset; no new steps for sfbForceBalancePeriod.
    fn tick_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        self.t += dt;
        let r = self.rdr2;
        let t = self.t.min(r.period);
        let f = if r.period < 1e-5 { 0.0 } else { 1.0 - t / r.period };
        c.ch.set_stiffness(RDR2_SHOT_SPINE, 0.5, "us", None);
        spine_split(c, -f * r.amounts[0], 0.0);
        if r.amounts[2].abs() > 0.0 {
            // the hip pitch, flexing as ours (RDR2's sign the other way)
            c.out.dbm_crouch.push(Crouch::Hip(9, -(r.amounts[2] * f).clamp(-2.0, 2.0), f32::INFINITY));
        }
        c.ch.set_stiffness(RDR2_SHOT_SPINE, 0.5, "un", None);
        for &j in NECK().iter() {
            c.ch.eff[j].desired[0] = -f * r.amounts[1];
        }
        if t >= r.arms_onset {
            for (i, (clav, shoulder, elbow, wrist)) in SFB_ARMS.into_iter().enumerate() {
                let l = crate::nmshot::limb(i);
                let noise = if r.noise != 0.0 { (c.ch.random() - 0.5) * r.noise } else { 0.0 };
                c.ch.eff[l.e[0]].desired = clav;
                c.ch.eff[l.e[1]].desired = shoulder;
                c.ch.eff[l.e[2]].desired[0] = elbow + noise;
                c.ch.eff[l.e[3]].desired = wrist;
            }
        }
        let span = r.period - r.knees_onset;
        if t >= r.knees_onset && span > 1e-5 && r.amounts[3].abs() > 0.0 {
            let k = -r.amounts[3] * (1.0 - (t - r.knees_onset).clamp(0.0, span) / span);
            c.out.dbm_crouch.push(Crouch::Knee(6, k, f32::INFINITY));
        }
        // RDR2 forces balance here only while balancerCollisionsReaction is in state 4 or 7 (rdr2_shot_hits.md §5 step
        // 8); Delirium has no such state, so under the core it never does
    }
}

impl InGuts {
    /// RDR2: at full strength on the hit, falling linearly to nothing over sigPeriod: the spine and the neck and head
    /// fold forward, the balancer pitches the hips (owner 10) and, from sigKneesOnset, gives at the knees (owner 7);
    /// no new steps for sigForceBalancePeriod.
    fn tick_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        self.t += dt;
        let r = self.rdr2;
        let t = self.t.min(r.period);
        let f = if r.period < 1e-5 { 0.0 } else { 1.0 - t / r.period };
        spine_split(c, f * r.spine, 0.0);
        if r.hip.abs() > 0.0 {
            c.out.dbm_crouch.push(Crouch::Hip(10, (r.hip * f).clamp(-2.0, 2.0), f32::INFINITY));
        }
        c.ch.set_stiffness(RDR2_SHOT_SPINE, 0.5, "un", None);
        for &j in NECK().iter() {
            c.ch.eff[j].desired[0] = f * r.neck;
        }
        let span = r.period - r.knees_onset;
        if t >= r.knees_onset && span > 1e-5 && r.knee.abs() > 0.0 {
            let k = -r.knee * (1.0 - (t - r.knees_onset).clamp(0.0, span) / span);
            c.out.dbm_crouch.push(Crouch::Knee(7, k, f32::INFINITY));
        }
        // no forced balance: as fromBehind, RDR2 gates it on balancerCollisionsReaction's states 4/7
    }
}

impl Snap {
    /// RDR2's snap body: each joint kicked about its own randomised axis (the snap axis plus U[-r, r] on each
    /// component), sized by the joint group, as a torque on its child part and the opposite on its parent.
    fn apply_rdr2(&mut self, c: &mut Ctx, m: f32) {
        let (a, b) = (c.m.a, c.m.b);
        let n = if self.use_dir && self.dir != Vec3::ZERO { self.dir.normalize() } else { -c.m.c };
        let snap = if self.use_dir && self.dir != Vec3::ZERO { n.cross(b).normalize_or(a) } else { a };
        let random = self.random;
        let mut axis = |c: &mut Ctx, s: Vec3| {
            let r = Vec3::new(c.ch.random(), c.ch.random(), c.ch.random()) * (2.0 * random) - Vec3::splat(random);
            (s + r).normalize_or(s)
        };
        let torque = |c: &mut Ctx, j: usize, t: Vec3| {
            c.out.torque(j + 1, t);
            c.out.torque(PARENT[j + 1], -t);
        };
        // his state: nothing touching (airborne), only the legs (moving or standing), more than the legs (as it is)
        let legs = c.b.doll.collided(&[1, 2, 3, 4, 5, 6]);
        let other = c.b.doll.collided(&[0, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20]);
        let m = if !legs && !other {
            m * self.mult[2]
        } else if !other {
            m * self.mult[if c.vel.length() <= self.moving { 1 } else { 0 }]
        } else {
            m
        };
        let hinge = |j: usize| (*KNEE()).contains(&j) || (*ELBOW()).contains(&j);
        if (0..21).contains(&self.part) {
            let k = self.part as usize;
            if k == 0 {
                for (j, g) in [((*HIP())[0], 0.5), ((*HIP())[1], 0.5), ((*SPINE())[0], 1.0)] {
                    let s = axis(c, snap);
                    torque(c, j, s * (m * 15.0 * g));
                }
            } else {
                for j in [k.checked_sub(1), (k < 20).then_some(k)].into_iter().flatten() {
                    if hinge(j) {
                        let h = c.b.eff1(j).m.c;
                        torque(c, j, h * (-m * 2.0));
                    } else {
                        let s = axis(c, snap);
                        torque(c, j, s * (m * 15.0));
                    }
                }
            }
        }
        let mix = (n + (b - n) * 0.5).normalize_or(b);
        for (on, sign, i) in [(self.limbs[2], 1.0, 0usize), (self.limbs[3], -1.0, 1)] {
            if on {
                let l = crate::nmshot::limb(i);
                let h = c.b.eff1(l.e[2]).m.c;
                torque(c, l.e[2], h * (2.0 * m));
                let s = axis(c, mix);
                torque(c, l.e[1], s * (4.0 * m * sign));
                let s = axis(c, b);
                torque(c, l.e[0], s * (4.0 * m * sign));
            }
        }
        if self.limbs[5] {
            for &j in NECK().iter().rev() {
                let s = axis(c, snap);
                torque(c, j, s * m);
            }
        }
        if self.limbs[4] {
            for (i, &j) in SPINE().iter().enumerate().rev() {
                let s = axis(c, snap);
                torque(c, j, s * (m * if i == 3 { 8.0 } else { 15.0 }));
            }
        }
        if self.hip >= 1 {
            let leg = if self.hip >= 2 { n } else { snap };
            let planted = [c.b.doll.collided(&[3]), c.b.doll.collided(&[6])];
            for (on, i) in [(self.limbs[0], 0usize), (self.limbs[1], 1)] {
                if !on {
                    continue;
                }
                let mut g = -45.0 * m;
                if planted[1 - i] && !planted[i] {
                    g *= -1.5;
                }
                if i == 1 && self.phased {
                    g = -g;
                }
                let s = axis(c, leg);
                torque(c, (*HIP())[i], s * g);
            }
        }
    }
}

impl Spin {
    /// RDR2's shot spin: no twisting of the body while he balances. While the spin runs (shotSpinTime, then
    /// shotSpinSettle) the balancer is turned to his facing swung by clamp(lever x shotSpinTwistMult, +-2.6) and turns
    /// by stepping (only in the spin phase; the settle counts any heading as faced). Only falling does a constant yaw
    /// torque (+-shotSpinTorque, none within shotSpinTorqueTol) push him round. All of it only while the yaw turned
    /// since the hit, the turn rate and the spin stay under shotSpinTwist, shotSpinTwistVel and shotSpinTwistMom.
    /// Then the step turn (stepTurnType 1 back along the bullet, other values to attackerPos) after stepTurnWait.
    fn tick_rdr2(&mut self, c: &mut Ctx, dt: f32) {
        let r = self.rdr2;
        self.t += dt;
        let (up, b) = (c.up, c.m.b);
        let yaw_rate = c.angvel.dot(b);
        self.turned += yaw_rate * dt;
        let lever = *self.lever.get_or_insert_with(|| {
            // the hit's lever across the bullet about up, over the leg's length
            let l = (c.b.joint_pos((*HIP())[0]) - c.b.joint_pos((*ANKLE())[0])).length().max(0.1);
            ((self.hit - c.com).cross(self.dir.normalize_or_zero())).dot(up) / l
        });
        let lever = self.spin.unwrap_or(lever);
        let facing = {
            let f = -c.m.c;
            (f - up * f.dot(up)).normalize_or(Vec3::Y)
        };
        let gate = self.turned.abs() <= r.twist && yaw_rate.abs() <= r.twist_vel
            && (c.angvel * c.b.doll.mass()).dot(b).abs() <= r.twist_mom;
        if !gate {
            c.out.dbm_turn = Some(Vec3::ZERO);
            return;
        }
        let spinning = self.t < self.time + self.settle;
        if spinning && lever.abs() > self.min {
            let angle = (lever * r.twist_mult).clamp(-2.6, 2.6);
            let heading = bevy_math::Quat::from_axis_angle(up, angle) * facing;
            if self.t < self.time && c.dbm_failed {
                let off = facing.cross(heading).dot(up).atan2(facing.dot(heading));
                if off.abs() >= r.tol {
                    let feet = [c.b.doll.collided(&[3]), c.b.doll.collided(&[6])];
                    let any = c.b.doll.collided(&(0..21).collect::<Vec<_>>());
                    let k = if !any { 0.2 } else if feet[0] != feet[1] { 0.5 } else { 1.0 };
                    let tq = (r.torque * off.signum()).clamp(-r.torque_max, r.torque_max) * k;
                    for p in [0, 7, 8] {
                        c.out.torque(p, b * tq / 3.0);
                    }
                }
            }
            c.out.dbm_turn = Some(heading * 0.5);
            c.out.dbm_turn_facing = Some(if self.t < self.time { r.align } else { -0.1 });
            return;
        }
        let from = if self.spin_on { self.time + self.settle } else { 0.0 } + self.step_wait;
        if !c.dbm_failed && self.step_type > 0 && self.t > from && self.t <= from + self.step_time {
            let to = if self.step_type == 1 { -self.dir } else { self.attacker - c.com };
            let to = bevy_math::Quat::from_axis_angle(up, r.step_offset) * (to - up * to.dot(up)).normalize_or_zero();
            if to != Vec3::ZERO {
                c.out.dbm_turn = Some(to * 0.5);
                c.out.dbm_turn_facing = Some(0.9);
                return;
            }
        }
        c.out.dbm_turn = Some(Vec3::ZERO);
    }
}

/// RDR2's NM bullet applier (applyBulletImpulse): each bullet becomes one or two records, the extra share on the trunk
/// part (RDR2's part 3, Spine0: ours 7) and the main one on the part hit, each spread evenly over 0.1 s and scaled by
/// NM's own shot modifier, which drops 0.15 after each main record (floor 0.2). Its recovery between bullets, 0.38/s,
/// is read off the recording, not the code. A new bullet while this runs adds its records to the ones still going.
pub struct BulletImpulse {
    /// part, hit point in the part's frame (None: its centre), the record's whole impulse, s left
    recs: Vec<(usize, Option<Vec3>, Vec3, f32)>,
    /// new records not yet placed in their part's frame (world hit points)
    new: Vec<(usize, Option<Vec3>, Vec3, f32)>,
    modifier: f32,
}

/// RDR2's record period (+0x34), s.
const BULLET_PERIOD: f32 = 0.1;
/// RDR2's trunk part for the extra share (part 3, SKEL_Spine0Body) on our body.
const BULLET_TRUNK: usize = 7;

/// What RDR2's shot task knows about a bullet when it builds the impulse (0x1c68ea4, 0x1c6eb20).
pub struct BulletHit {
    /// the room's force for the weapon, N s (see `bullet_impulse`)
    pub force: f32,
    /// part hit, ours
    pub part: usize,
    /// the bullet's direction and his root's horizontal velocity, world
    pub dir: Vec3,
    pub root_vel: Vec3,
    /// the task's first bullet (the builder's p5: the health factor and the root speed limit run only then)
    pub first: bool,
    /// his health after this hit over his full health (task+0x89c)
    pub health: f32,
    /// this hit kills him (+0x89c = 0), or he is dying already (+0x9d8 bit 2)
    pub kill: bool,
    pub dying: bool,
    /// standing (+0x9d8 bit 0x20): his left foot to his head is longer than 1 m and more than 0.9 up
    pub standing: bool,
    /// the bullet goes along his front axis, so it came from behind (+0x8a8 > 0; +0x9d8 bit 0x10 clear)
    pub from_behind: bool,
    /// unit z of his pelvis to neck (0x1c4a838)
    pub upright: f32,
}

/// What RDR2's shot task sends for a bullet.
pub struct BulletSend {
    /// the applyBulletImpulse's impulse, N s, and its extraShare
    pub impulse: f32,
    pub extra_share: Option<f32>,
    /// the applyImpulse on Spine0 of a shot in the leg (equalizeAmount 1, at the part's frame), N s before the equalize
    pub root: Option<f32>,
}

/// RDR2's bullet impulse from the shot task's builder 0x1c68ea4 and its caller 0x1c6eb20, for the factors whose inputs
/// a Delirium scene has. The XML's tunables except where it can't be what ran, as the recordings show: the trunk share
/// reads 0.75 (the XML says 0.5), and the leg's root impulse fires though the XML's upright limit 1.0 can't be passed.
///
/// RDR2's bullet starts from the damage event's force and two per-weapon factors, in a weapon file we don't have. So
/// ours: the room's force, scaled so the pistol's 50 gives what the recorded Cattleman gives on a standing man at
/// 85% health, 49.38 N s on the chest and the back (`BULLET_BASE`, and 58.49 in a thigh, `BULLET_LEG`). The root
/// impulse (LegShotFallRootImpulseMult 0.375 times the unfactored force) is a third recorded number: 67.5 N s in the
/// body's momentum at once, through the equalize's 1.905 on RDR2's Spine0 (`BULLET_ROOT`).
///
/// Left out, no input: armour (x1.2), the mounted and vehicle states, the bracing x0.5, the shot-against-wall factor,
/// an arm hit's overflow to the clavicle, rapid fire and the shotgun's pellets, the "hit again soon" flag (+0x9d9 bit
/// 0, which takes the extra share and the low trunk's factor from later bullets), the kill's per-weapon factor.
pub fn bullet_impulse(h: &BulletHit) -> BulletSend {
    /// 49.38 / (1.3 standing x 1.085 health) / 50
    const BULLET_BASE: f32 = 0.7002;
    /// 58.49 / 49.38, the thigh's: same chain in the code, so it is in the part's force [P]
    const BULLET_LEG: f32 = 1.1845;
    /// 67.5 / 1.905 / (50 x BULLET_BASE x BULLET_LEG)
    const BULLET_ROOT: f32 = 0.854;
    /// The upright limit that ran: 1.0 in the XML. Ours, the standing test's.
    const LEG_ROOT_UPRIGHT: f32 = 0.9;
    let leg = matches!(h.part, 1..=6);
    let mut f = h.force * BULLET_BASE * if leg { BULLET_LEG } else { 1.0 };
    let force = f;
    // 0x0D7DB754, not on someone falling or dying
    if h.standing && !h.dying {
        f *= 1.3;
    }
    let head = matches!(h.part, 11 | 12);
    if head {
        // HeadShot; scaled by how upright he is on someone dying
        f *= if h.dying { 0.25 + (1.2 - 0.25) * h.upright.max(0.0) } else { 1.2 };
    } else if h.kill {
        f *= 1.5;
    } else if h.first {
        // the damage-taken factor is 1 at both ends; armour would go here
        f *= 1.0 + 0.1 * h.health.clamp(0.0, 1.0);
    }
    // 0x1c4a5a8: running into the bullet, or with it, past the task's speed
    let speed = h.root_vel.with_z(0.0).length();
    if speed > BULLET_RUN_SPEED {
        let dot = h.dir.normalize_or_zero().dot(h.root_vel.with_z(0.0).normalize_or_zero());
        let k = dot.abs() * 0.5 + 0.1 * speed;
        f *= if dot < -0.6 { (k * 2.7 + 1.0).min(3.0) } else if dot > 0.6 { k * 0.6 } else { 1.0 };
    }
    f = f.min(500.0);
    // the low trunk from behind; the front's x1.1 is on two bones of the upper chest we don't have [P]
    if h.from_behind && h.standing && !h.dying && matches!(h.part, 0 | 7 | 8) {
        f *= 1.3;
    }
    let trunk = matches!(h.part, 0 | 7..=10);
    let extra_share = (trunk && h.standing && !h.dying && (!h.first || speed < 100.0) && f >= 0.01).then_some(-0.75);
    let root = (leg && !h.dying && h.upright > LEG_ROOT_UPRIGHT).then_some(force * BULLET_ROOT);
    BulletSend { impulse: f, extra_share, root }
}

/// CTaskNMShot::Tunables +0x68, m/s, the speed past which running against or with the bullet counts.
const BULLET_RUN_SPEED: f32 = 1.5;

impl BulletImpulse {
    fn new(p: &mut P) -> Result<BulletImpulse, Error> {
        let mut b = BulletImpulse { recs: vec![], new: vec![], modifier: 1.0 };
        b.take(p)?;
        Ok(b)
    }

    fn take(&mut self, p: &mut P) -> Result<(), Error> {
        let part = p.i("partIndex", 0)?.max(0) as usize;
        let imp = p.v("impulse", Vec3::ZERO)?;
        let at = p.v("hitPoint", Vec3::NAN)?;
        let at = (!at.is_nan()).then_some(at);
        let equalize = p.f("equalizeAmount", 0.0)?;
        let s = p.f("extraShare", 0.0)?;
        let (extra, main) = if s < 0.0 { (-s, 1.0) } else if s <= 1.0 { (s, 1.0 - s) } else { (0.0, 1.0) };
        if extra.abs() >= 1e-4 {
            self.new.push((BULLET_TRUNK, None, imp * extra * self.modifier, equalize));
        }
        if main.abs() >= 1e-4 {
            self.new.push((part, at, imp * main * self.modifier, equalize));
            self.modifier = (self.modifier - 0.15).clamp(0.2, 1.0);
        }
        Ok(())
    }
}

impl Extra for BulletImpulse {
    fn tick(&mut self, c: &mut Ctx, dt: f32) {
        let f = c.b.frames;
        for (k, at, imp, equalize) in std::mem::take(&mut self.new) {
            if k >= f.len() {
                continue;
            }
            let imp = if equalize != 0.0 {
                let mean = (0..f.len()).map(|q| c.b.doll.part_mass(q)).sum::<f32>() / f.len() as f32;
                imp * (1.0 - equalize + equalize * c.b.doll.part_mass(k) / mean)
            } else {
                imp
            };
            let local = at.map(|p| f[k].rotation.inverse() * (p - f[k].translation));
            self.recs.push((k, local, imp, BULLET_PERIOD));
            c.ch.core.bullet = Some(imp);
        }
        for (k, local, imp, left) in &mut self.recs {
            let h = dt.min(*left);
            if h <= 0.0 {
                continue;
            }
            let at = f[*k].translation + local.map_or(Vec3::ZERO, |l| f[*k].rotation * l);
            c.out.impulse.push((*k, at, *imp * (h / BULLET_PERIOD)));
            *left -= h;
        }
        self.recs.retain(|r| r.3 > 1e-6);
        self.modifier = (self.modifier + 0.38 * dt).min(1.0);
    }

    fn done(&self) -> bool {
        self.recs.is_empty() && self.new.is_empty() && self.modifier >= 1.0
    }

    fn hit(&mut self, p: &mut P) -> Option<Result<(), Error>> {
        Some(self.take(p))
    }
}
