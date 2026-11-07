//! ctx+0x2bf0, 0x2a4e1dc: the character's dimensions referred to the reference ped, by which every float a message stores is multiplied (`rdr2_params.md`: the
//! scale index of each). Time T, length L, mass M and their products and inverses. The game measures L from two parts (.5636 z), M from the two masses over 97,
//! T = 2.35 sqrt(z / g); by construction 1 on its male, with a jitter of .19%. Here each of Delirium's rigs is its own reference: its male comes out exactly 1,
//! Brawler, Random and Custom their real scale. Only under the RDR2 core; a table of 1s changes nothing.
//! `SCALED` is generated from the registered descriptors (scale index and default): parameters whose index isn't 1 on a biped.
use crate::msg::Value;
use crate::ragdoll::Build;

/// The table's entries, by index. [C] for the formulae; 23 (yaw inertia) and 25, 27 (a limb's length, a joint's) follow M L^2 and L here [P].
pub fn table(build: &Build) -> Option<[f32; 31]> {
    let chain = |p: &crate::ragdoll::PartDef| !p.reach && p.length > 0.0;
    let (scaled, plain) = build.parts.iter().filter(|p| chain(p)).fold((0.0, 0.0), |(a, b), p| (a + p.length, b + p.length / p.size.max(1e-3)));
    let l = if plain > 0.0 { scaled / plain } else { 1.0 };
    let (mass, plain_mass) = build.parts.iter().fold((0.0, 0.0), |(a, b), p| (a + p.mass * p.size.powi(3) * p.heft, b + p.mass));
    let m = if plain_mass > 0.0 { mass / plain_mass } else { 1.0 };
    if (l - 1.0).abs() < 0.002 && (m - 1.0).abs() < 0.002 {
        return None;
    }
    let t = l.sqrt();
    let v = l / t;
    let a = l / (t * t);
    let (i, f, q) = (m * l * l, m * a, m * l * l / (t * t));
    Some([1.0, t, 1.0 / t, l, 1.0 / l, 1.0, 1.0, m, 1.0 / m, v, 1.0 / v, a, 1.0 / a, 1.0 / t, t, 1.0 / (t * t), t * t, i, 1.0 / i, f, 1.0 / f, q, 1.0 / q, i, 1.0 / i, l, 1.0 / l, l, 1.0 / l, 1.0, 1.0])
}

/// A message's parameters as the game's handler stores them: each scaled, and on a start the defaults of those not sent.
pub fn scale(table: &[f32; 31], msg: &str, params: &[(&str, Value)]) -> Vec<(String, Value)> {
    let low = msg.to_ascii_lowercase();
    let row = |n: &str| SCALED.binary_search_by(|r| (r.0, r.1).cmp(&(low.as_str(), n))).ok().map(|i| SCALED[i]);
    let mut out: Vec<(String, Value)> = params.iter().map(|(n, v)| {
        let v = match (row(n), v) {
            (Some(r), Value::Float(x)) => Value::Float(x * table[r.2 as usize]),
            (Some(r), Value::Vec3(x)) => Value::Vec3(*x * table[r.2 as usize]),
            (_, v) => v.clone(),
        };
        (n.to_string(), v)
    }).collect();
    if params.iter().any(|(n, v)| *n == "start" && *v == Value::Bool(true)) {
        for r in SCALED.iter().filter(|r| r.0 == low && !params.iter().any(|(n, _)| *n == r.1)) {
            out.push((r.1.to_string(), Value::Float(r.3 * table[r.2 as usize])));
        }
    }
    out
}

pub static SCALED: [(&str, &str, u8, f32); 132] = [
    ("armswindmill", "angVelThreshold", 9, 0.1),
    ("armswindmill", "leftRadius1", 3, 0.75),
    ("armswindmill", "leftRadius2", 3, 0.75),
    ("armswindmill", "leftSpeed", 13, 1.0),
    ("armswindmill", "rightRadius1", 3, 0.75),
    ("armswindmill", "rightRadius2", 3, 0.75),
    ("armswindmill", "rightSpeed", 13, 1.0),
    ("armswindmilladaptive", "angSpeed", 13, 6.28),
    ("armswindmilladaptive", "angSpeedMultiplier4Dragging", 13, 1.0),
    ("armswindmilladaptive", "maxangspeed", 13, 4.0),
    ("balancercollisionsreaction", "braceDistance", 3, 2.0),
    ("balancercollisionsreaction", "braceReachAbsorbtionTime", 1, 0.15),
    ("balancercollisionsreaction", "braceTargetPredictionTime", 1, 0.7),
    ("balancercollisionsreaction", "braceminbracetime", 1, 0.3),
    ("balancercollisionsreaction", "sitstarttime", 1, 0.3),
    ("bodyrollup", "rollVelForSuccess", 9, 0.2),
    ("bodyrollup", "rollVelLinearContribution", 9, 1.0),
    ("bodyrollup", "velocityOffset", 9, 2.0),
    ("bodyrollup", "velocityScale", 9, 0.2),
    ("bodywrithe", "rollTorqueScale", 21, 150.0),
    ("braceforimpact", "braceDistance", 3, 0.5),
    ("braceforimpact", "dampSpinThresh", 13, 4.0),
    ("braceforimpact", "dampUpVelThresh", 9, 2.0),
    ("buoyancy", "rightingStrength", 21, 25.0),
    ("configurebalance", "avoidFeedback", 3, 0.6),
    ("configurebalance", "avoidFootWidth", 3, 0.1),
    ("configurebalance", "claviclecrouchingaccumulated", 25, -0.1),
    ("configurebalance", "claviclecrouchingaccumulatedb", 25, 0.05),
    ("configurebalance", "dontStepTime", 1, 0.0),
    ("configurebalance", "extraFeetApart", 3, 0.0),
    ("configurebalance", "feetapartmax", 3, 1.0),
    ("configurebalance", "feetapartmin", 3, 1.0),
    ("configurebalance", "feetapartvel", 9, 100.0),
    ("configurebalance", "giveUpHeight", 3, 0.5),
    ("configurebalance", "legsApartMax", 3, 2.0),
    ("configurebalance", "legsApartRestep", 3, 0.2),
    ("configurebalance", "legsTogetherRestep", 3, 1.0),
    ("configurebalance", "liftfeet", 3, 0.07),
    ("configurebalance", "liftfeetvariance", 3, 0.0),
    ("configurebalance", "noslopetime", 1, 0.0),
    ("configurebalance", "runningThresh", 25, 0.65),
    ("configurebalance", "runningthreshb", 25, 0.65),
    ("configurebalance", "sidewayscheatforce", 21, 0.0),
    ("configurebalance", "slopemovefoot", 3, 0.07),
    ("configurebalance", "stableLinSpeedThresh", 9, 0.25),
    ("configurebalance", "stableRotSpeedThresh", 13, 0.25),
    ("configurebalance", "stepDecisionThreshold", 3, 0.0),
    ("configurebalance", "stepHeight", 3, 0.1),
    ("configurebalance", "stepHeightRun", 3, 0.6),
    ("configurebalance", "stepturnwait", 1, -1.0),
    ("configurebalance", "stepwait", 1, -1.0),
    ("configurebalance", "useComDirTurnVelThresh", 9, 0.0),
    ("falloverwall", "MaxAngVel", 13, 2.0),
    ("falloverwall", "distanceToSendSuccessMessage", 3, -1.0),
    ("falloverwall", "magOfForce", 19, 0.5),
    ("falloverwall", "maxDistanceFromPelToHitPoint", 3, 0.25),
    ("falloverwall", "maxForceDist", 3, 0.8),
    ("falloverwall", "maxWallHeight", 3, -1.0),
    ("falloverwall", "minLegHeight", 3, 0.4),
    ("falloverwall", "rollingBackThr", 13, 0.5),
    ("falloverwall", "rollingPotential", 9, 0.3),
    ("forceleantoposition", "bodyPart", 7, 1.844668379813852e+19),
    ("forceleantoposition", "leanAmount", 7, 0.2),
    ("forceleantoposition", "pos", 7, 0.0),
    ("grab", "grabDistance", 3, 1.0),
    ("grab", "grabStrength", 19, -1.0),
    ("grab", "maxReachDistance", 3, 0.7),
    ("grab", "move2Radius", 3, 0.0),
    ("grab", "orientationConstraintScale", 21, 1.0),
    ("grab", "stickyHands", 19, 4.0),
    ("highfall", "legRadius", 3, 0.4),
    ("keyframe", "softbreakthreshold", 3, -1.0),
    ("meleereact", "flinchaborttime", 1, 0.5),
    ("meleereact", "leanaborttime", 1, 0.5),
    ("onfire", "predictTime", 1, 0.1),
    ("onfire", "rollOverRadius", 3, 2.0),
    ("onfire", "rollTorqueScale", 21, 25.0),
    ("pedallegs", "angularSpeed", 13, 10.0),
    ("pedallegs", "centreForwards", 3, 0.0),
    ("pedallegs", "centreSideways", 3, 0.0),
    ("pedallegs", "centreUp", 3, 0.0),
    ("pedallegs", "pedalOffset", 3, 0.0),
    ("pedallegs", "radius", 3, 0.25),
    ("pedallegs", "radiusVariance", 3, 0.4),
    ("pedallegs", "speedAsymmetry", 13, 8.0),
    ("pedallegs", "spread", 3, 0.0),
    ("pointgun", "lefthandweapondistance", 3, -1.0),
    ("pointgun", "rifleConstraintMinDistance", 3, 0.2),
    ("pointgun", "righthandweapondistance", 3, -1.0),
    ("quadrupedbase", "lookahead", 1, 0.5),
    ("quadrupedbase", "maxscrabblespeed", 1, 11.0),
    ("quadrupedbase", "maxsteplength", 3, 0.6),
    ("quadrupedbase", "minscrabblespeed", 1, 3.0),
    ("quadrupedbase", "restepdistance", 3, 0.35),
    ("quadrupedbase", "rollpredictiontime", 1, 0.0),
    ("quadrupedbase", "writhespeed", 1, 0.5),
    ("reachforwound", "headzone", 3, 0.5),
    ("rolldownstairs", "armReachLength", 3, 0.4),
    ("rolldownstairs", "forcemag", 19, 0.55),
    ("shot", "angVelScale", 13, 1.0),
    ("shot", "flingWidth", 3, 0.5),
    ("shotshockspin", "shockSpinLiftForceMult", 19, 0.0),
    ("shotshockspin", "shockSpinMax", 21, 90.0),
    ("shotshockspin", "shockSpinMin", 21, 50.0),
    ("shotspin", "shotspintorque", 21, 10.0),
    ("shotspin", "shotspintorquemax", 21, 20.0),
    ("shotspin", "shotspintwistmom", 17, 100.0),
    ("shotspin", "shotspintwistvel", 13, 20.0),
    ("sit", "dragTime", 1, 10.0),
    ("sit", "dragreachdelay", 1, 0.2),
    ("sit", "rdsForceMag", 19, 2.0),
    ("sit", "rdsMinAngVel", 13, 0.3),
    ("sit", "rdsforcemagramp", 19, 0.05),
    ("sit", "rdsforcemagstart", 19, 0.5),
    ("smartfall", "armAngSpeed", 9, 7.85),
    ("smartfall", "cfZAxisSpinReduction", 23, 0.5),
    ("smartfall", "legAngSpeed", 13, 7.85),
    ("smartfall", "legRadius", 3, 0.4),
    ("smartfall", "maxSpeedForRecoverableFall", 9, 15.0),
    ("smartfall", "minSpeedForBrace", 9, 10.0),
    ("smartfall", "orientateMax", 21, 300.0),
    ("smartfall", "pdStrength", 21, 0.0),
    ("smartfall", "rdsForceMag", 19, 0.8),
    ("smartfall", "rdsForceVelThreshold", 9, 10.0),
    ("smartfall", "rdsTargetLinearVelocity", 9, 1.0),
    ("staggerfall", "maxPushoffVel", 9, 1.2),
    ("staggerfall", "pushOffDist", 25, 0.28),
    ("staggerfall", "pushoffdistb", 25, 0.23),
    ("swim", "extendDist", 3, 0.63),
    ("swim", "extendSpeed", 3, 4.0),
    ("teeter", "preTeeterTime", 1, 0.5),
    ("teeter", "teeterTime", 1, 1.0),
];
