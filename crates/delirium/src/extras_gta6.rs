//! Behaviours named after GTA 6's reaction sets (its `gameplay@behaviors@...` animation library: bullet impacts, hit
//! reacts, recoveries, stumbles, falls, rolls, writhe, burning, electrocution, vehicle tumbles). GTA 6's own Euphoria
//! behaviours aren't in its data files, so each of these runs on Delirium's existing behaviours (the RDR2 extras and
//! the core set): sending one sends a few of those, with Delirium's own round values. None of GTA 6's code or numbers
//! are here. Like every extra they do nothing until sent with start=true; start=false stops what they started.
use crate::extras::P;
use crate::msg::{Error, Kind, Message, Param, Value};
use bevy_math::Vec3;

pub static MESSAGES: [Message; 16] = [
    Message { name: "gta6BulletImpact", params: &[
        Param { name: "bodyPart", kind: Kind::Int, used: true, doc: "the part hit, 0..20 (-1: the chest)" },
        Param { name: "hitPoint", kind: Kind::Vec3, used: true, doc: "where the bullet hit, world (zero: that part's centre)" },
        Param { name: "bulletDir", kind: Kind::Vec3, used: true, doc: "the bullet's direction (zero: no push, no spin)" },
        Param { name: "toughness", kind: Kind::Int, used: true, doc: "0 normal, 1 tough, 2 very tough: stiffer, a smaller kick, no limp or fold (0)" },
        Param { name: "fromBehind", kind: Kind::Bool, used: true, doc: "shot in the back: he arches instead of folding (false)" },
        Param { name: "reach", kind: Kind::Bool, used: true, doc: "a hand goes to the wound (true)" },
    ] },
    Message { name: "gta6ShotgunKnockdown", params: &[
        Param { name: "area", kind: Kind::Int, used: true, doc: "0 chest, 1 gut, 2 left leg, 3 right leg (0)" },
        Param { name: "bulletDir", kind: Kind::Vec3, used: true, doc: "the blast's direction (zero: no push)" },
        Param { name: "force", kind: Kind::Float, used: true, doc: "the push along bulletDir, N s (80)" },
    ] },
    Message { name: "gta6HitReact", params: &[
        Param { name: "weapon", kind: Kind::Int, used: true, doc: "what he holds: 0 nothing, 1 a pistol, 2 a rifle (0)" },
        Param { name: "style", kind: Kind::Int, used: true, doc: "0 normal, 1 tough, 2 swat: less give (0)" },
        Param { name: "hitPart", kind: Kind::Int, used: true, doc: "the part hit, 0..20 (-1: the chest)" },
        Param { name: "hitPoint", kind: Kind::Vec3, used: true, doc: "where, world (zero: that part's centre)" },
        Param { name: "hitNormal", kind: Kind::Vec3, used: true, doc: "the body's surface normal there, outwards (zero: no push or snap direction)" },
    ] },
    Message { name: "gta6RecoverBalance", params: &[
        Param { name: "armed", kind: Kind::Bool, used: true, doc: "holding a weapon: arms stay in, no windmill (false)" },
        Param { name: "windmill", kind: Kind::Bool, used: true, doc: "arms circle while he catches himself (true)" },
    ] },
    Message { name: "gta6Stumble", params: &[
        Param { name: "staggerTime", kind: Kind::Float, used: true, doc: "s he staggers (1.5)" },
        Param { name: "toKnees", kind: Kind::Bool, used: true, doc: "he ends on his knees (false)" },
    ] },
    Message { name: "gta6Trip", params: &[
        Param { name: "handsAndKnees", kind: Kind::Bool, used: true, doc: "the fall ends on all fours (true)" },
    ] },
    Message { name: "gta6LowSpeedImpact", params: &[
        Param { name: "strength", kind: Kind::Int, used: true, doc: "0 light, 1 medium, 2 heavy: heavy knocks him over (1)" },
        Param { name: "dir", kind: Kind::Vec3, used: true, doc: "the way he's pushed (zero: no push)" },
        Param { name: "at", kind: Kind::Vec3, used: true, doc: "where, world (zero: the pelvis)" },
    ] },
    Message { name: "gta6Freefall", params: &[
        Param { name: "armsUp", kind: Kind::Float, used: true, doc: "how high the arms go while falling, as highFall's (-1: highFall's own)" },
    ] },
    Message { name: "gta6StuntFall", params: &[
        Param { name: "forwardRoll", kind: Kind::Bool, used: true, doc: "roll forwards on landing (false)" },
    ] },
    Message { name: "gta6FallOverFence", params: &[
        Param { name: "fencePoint", kind: Kind::Vec3, used: true, doc: "a point on the fence's top, world (zero: fallOverWall's own)" },
        Param { name: "fenceEdge", kind: Kind::Vec3, used: true, doc: "the fence's direction along its top (zero: fallOverWall's own)" },
    ] },
    Message { name: "gta6Roll", params: &[
        Param { name: "tucked", kind: Kind::Bool, used: true, doc: "rolled into a ball (true); false: loose, tumbling" },
    ] },
    Message { name: "gta6Writhe", params: &[
        Param { name: "dying", kind: Kind::Bool, used: true, doc: "hurt and fading on the floor rather than thrashing (false)" },
    ] },
    Message { name: "gta6Burning", params: &[
        Param { name: "onGround", kind: Kind::Bool, used: true, doc: "already down: he writhes; false: staggers flailing (false)" },
    ] },
    Message { name: "gta6Electrocuted", params: &[
        Param { name: "stunMag", kind: Kind::Float, used: true, doc: "each jolt's kick, as electrocute's (0.25)" },
    ] },
    Message { name: "gta6VehicleTumble", params: &[
        Param { name: "knockedOut", kind: Kind::Bool, used: true, doc: "out cold: he goes limp instead of tumbling (false)" },
    ] },
    Message { name: "gta6ReachForInjury", params: &[
        Param { name: "hitPart", kind: Kind::Int, used: true, doc: "the part hurt, 0..20 (-1: the belly)" },
        Param { name: "hitPoint", kind: Kind::Vec3, used: true, doc: "the wound, world (zero: in front of the part)" },
        Param { name: "bothHands", kind: Kind::Bool, used: true, doc: "both hands on a trunk wound (false)" },
    ] },
];

/// One message to send: its name and parameters.
pub type Line = (&'static str, Vec<(&'static str, Value)>);

/// Messages a preset sends that are one-shots: there's nothing to stop on start=false.
const ONE_SHOT: [&str; 3] = ["applyImpulse", "bodyRelax", "stopAllBehaviours"];

pub fn is(msg: &str) -> bool {
    MESSAGES.iter().any(|m| m.name == msg)
}

/// The messages a GTA 6 behaviour stands for. `at(k)`: part k's centre, world. None: not a GTA 6 behaviour.
/// With start=true the list starts its behaviours; start=false stops them; no start: nothing.
pub fn expand(msg: &str, params: &[(&str, Value)], at: &dyn Fn(usize) -> Vec3) -> Option<Result<Vec<Line>, Error>> {
    if !is(msg) {
        return None;
    }
    let mut p = P::new(params);
    let out = (|| {
        let start = p.b("start", false)?;
        let given = params.iter().any(|(n, _)| *n == "start");
        let lines = build(msg, &mut p, at)?;
        p.done()?;
        Ok(match (given, start) {
            (true, true) => lines,
            (true, false) => lines.into_iter().filter(|(m, _)| !ONE_SHOT.contains(m))
                .map(|(m, _)| (m, vec![("start", Value::Bool(false))])).collect(),
            (false, _) => vec![],
        })
    })();
    Some(out)
}

fn on() -> (&'static str, Value) {
    ("start", Value::Bool(true))
}

fn part(k: i32, none: usize) -> usize {
    if (0..21).contains(&k) { k as usize } else { none }
}

fn or_at(v: Vec3, k: usize, at: &dyn Fn(usize) -> Vec3) -> Vec3 {
    if v == Vec3::ZERO { at(k) } else { v }
}

fn pick<T: Copy>(xs: &[T], i: i32) -> T {
    xs[(i.max(0) as usize).min(xs.len() - 1)]
}

const CHEST: usize = 10;

fn build(msg: &str, p: &mut P, at: &dyn Fn(usize) -> Vec3) -> Result<Vec<Line>, Error> {
    use Value::{Bool, Float, Int, Vec3 as V};
    let mut out: Vec<Line> = vec![];
    match msg {
        "gta6BulletImpact" => {
            let k = part(p.i("bodyPart", -1)?, CHEST);
            let hit = or_at(p.v("hitPoint", Vec3::ZERO)?, k, at);
            let dir = p.v("bulletDir", Vec3::ZERO)?.normalize_or_zero();
            let tough = p.i("toughness", 0)?.clamp(0, 2);
            let (behind, reach) = (p.b("fromBehind", false)?, p.b("reach", true)?);
            let mut shot = vec![on(), ("bodyPart", Int(k as i32)), ("hitPoint", V(hit)), ("bodyStiffness", Float(pick(&[10.0, 12.0, 14.0], tough))),
                                ("reachForWound", Bool(false))];
            if dir != Vec3::ZERO {
                shot.extend([("normal", V(-dir)), ("bulletVel", V(dir * 400.0))]);
            }
            out.push(("shot", shot));
            let mut snap = vec![on(), ("bodyPart", Int(k as i32)), ("snapMag", Float(pick(&[0.5, 0.35, 0.2], tough)))];
            if dir != Vec3::ZERO {
                snap.push(("bulletDir", V(dir)));
                out.push(("shotSpin", vec![on(), ("hitPoint", V(hit)), ("bulletDir", V(dir))]));
            }
            out.push(("shotSnap", snap));
            if tough < 2 {
                match k {
                    1..=6 => out.push(("shotInjuredLeg", vec![on(), ("bodyPart", Int(k as i32))])),
                    13..=20 => {
                        let mut arm = vec![on(), ("bodyPart", Int(k as i32))];
                        if dir != Vec3::ZERO {
                            arm.push(("bulletDir", V(dir)));
                        }
                        out.push(("shotInjuredArm", arm));
                    }
                    _ if behind => out.push(("shotFromBehind", vec![on()])),
                    0 | 7 | 8 => out.push(("shotInGuts", vec![on()])),
                    _ => {}
                }
            }
            if reach {
                out.push(("reachForWound", vec![on(), ("hitPart", Int(k as i32)), ("hitPoint", V(hit))]));
            }
        }
        "gta6ShotgunKnockdown" => {
            let area = p.i("area", 0)?.clamp(0, 3);
            let k = pick(&[CHEST, 8, 2, 5], area);
            let dir = p.v("bulletDir", Vec3::ZERO)?.normalize_or_zero();
            let force = p.f("force", 80.0)?;
            let hit = at(k);
            out.push(("shot", vec![on(), ("bodyPart", Int(k as i32)), ("hitPoint", V(hit)), ("bodyStiffness", Float(8.0)),
                                   ("useCatchFallOnFall", Bool(true)), ("reachForWound", Bool(false))]));
            let mut snap = vec![on(), ("bodyPart", Int(k as i32)), ("snapMag", Float(0.8))];
            if dir != Vec3::ZERO {
                snap.push(("bulletDir", V(dir)));
                out.push(("applyImpulse", vec![("partIndex", Int(k as i32)), ("hitPoint", V(hit)), ("impulse", V(dir * force))]));
            }
            out.push(("shotSnap", snap));
            match area {
                1 => out.push(("shotInGuts", vec![on()])),
                2 | 3 => out.push(("shotInjuredLeg", vec![on(), ("bodyPart", Int(k as i32)), ("legInjury", Float(0.8))])),
                _ => {}
            }
        }
        "gta6HitReact" => {
            let (weapon, style) = (p.i("weapon", 0)?.clamp(0, 2), p.i("style", 0)?.clamp(0, 2));
            let k = part(p.i("hitPart", -1)?, CHEST);
            let hit = or_at(p.v("hitPoint", Vec3::ZERO)?, k, at);
            let normal = p.v("hitNormal", Vec3::ZERO)?.normalize_or_zero();
            if weapon == 0 {
                out.push(("meleeReact", vec![on(), ("hitPoint", V(hit)), ("hitNormal", V(normal)), ("hitPart", Int(k as i32)),
                                             ("armsType", Int(1)), ("csLooseness", Float(pick(&[0.5, 0.3, 0.2], style)))]));
            } else {
                let mut snap = vec![on(), ("bodyPart", Int(k as i32)), ("snapMag", Float(pick(&[0.3, 0.2, 0.15], style)))];
                if normal != Vec3::ZERO {
                    snap.push(("bulletDir", V(-normal)));
                }
                out.push(("shotSnap", snap));
                out.push(("upperBodyFlinch", vec![on(), ("pos", V(hit)), ("bodyStiffness", Float(pick(&[10.0, 12.0, 13.0], style)))]));
                out.push(("bodyBalance", vec![on()]));
            }
        }
        "gta6RecoverBalance" => {
            let (armed, windmill) = (p.b("armed", false)?, p.b("windmill", true)?);
            out.push(("bodyBalance", vec![on(), ("armsOutOnPush", Bool(!armed))]));
            out.push(("rdr2Balance", vec![on()]));
            if windmill && !armed {
                out.push(("armsWindmillAdaptive", vec![on(), ("disableOnImpact", Bool(true))]));
            }
        }
        "gta6Stumble" => {
            let (time, knees) = (p.f("staggerTime", 1.5)?, p.b("toKnees", false)?);
            out.push(("stumble", vec![on(), ("staggerTime", Float(time)), ("useArmsBrace", Bool(true))]));
            if knees {
                out.push(("shotFallToKnees", vec![on()]));
            }
        }
        "gta6Trip" => {
            let hk = p.b("handsAndKnees", true)?;
            out.push(("staggerFall", vec![on(), ("alwaysBendForwards", Bool(true))]));
            out.push(("setFallingReaction", vec![on(), ("handsAndKnees", Bool(hk))]));
        }
        "gta6LowSpeedImpact" => {
            let s = p.i("strength", 1)?.clamp(0, 2);
            let dir = p.v("dir", Vec3::ZERO)?.normalize_or_zero();
            let point = or_at(p.v("at", Vec3::ZERO)?, 0, at);
            if dir != Vec3::ZERO {
                out.push(("applyImpulse", vec![("partIndex", Int(0)), ("hitPoint", V(point)), ("impulse", V(dir * pick(&[25.0, 50.0, 100.0], s)))]));
            }
            if s < 2 {
                out.push(("bodyBalance", vec![on(), ("armsOutOnPush", Bool(true))]));
            } else {
                out.push(("staggerFall", vec![on()]));
            }
        }
        "gta6Freefall" => {
            let up = p.f("armsUp", -1.0)?;
            let mut hf = vec![on()];
            if up >= 0.0 {
                hf.push(("armsUp", Float(up)));
            }
            out.push(("highFall", hf));
        }
        "gta6StuntFall" => {
            let roll = p.b("forwardRoll", false)?;
            out.push(("smartFall", vec![on(), ("forwardRoll", Bool(roll))]));
        }
        "gta6FallOverFence" => {
            let (point, edge) = (p.v("fencePoint", Vec3::ZERO)?, p.v("fenceEdge", Vec3::ZERO)?);
            let mut f = vec![on()];
            if point != Vec3::ZERO {
                f.push(("fallOverPoint", V(point)));
            }
            if edge != Vec3::ZERO {
                f.push(("fallOverEdge", V(edge)));
            }
            out.push(("fallOverWall", f));
        }
        "gta6Roll" => {
            if p.b("tucked", true)? {
                out.push(("bodyRollUp", vec![on()]));
            } else {
                out.push(("rollDownStairs", vec![on()]));
            }
        }
        "gta6Writhe" => {
            if p.b("dying", false)? {
                out.push(("injuredOnGround", vec![on(), ("numInjuries", Int(1))]));
            } else {
                out.push(("bodyWrithe", vec![on()]));
            }
        }
        "gta6Burning" => {
            if p.b("onGround", false)? {
                out.push(("bodyWrithe", vec![on()]));
            } else {
                out.push(("stumble", vec![on(), ("staggerTime", Float(3.0)), ("useArmsBrace", Bool(false))]));
                out.push(("armsWindmill", vec![on()]));
            }
        }
        "gta6Electrocuted" => {
            let mag = p.f("stunMag", 0.25)?;
            out.push(("electrocute", vec![on(), ("stunMag", Float(mag))]));
        }
        "gta6VehicleTumble" => {
            if p.b("knockedOut", false)? {
                out.push(("bodyRelax", vec![("relaxation", Float(80.0))]));
            } else {
                out.push(("rollDownStairs", vec![on(), ("spinWhenInAir", Bool(true))]));
            }
        }
        "gta6ReachForInjury" => {
            let k = p.i("hitPart", -1)?;
            let hit = p.v("hitPoint", Vec3::ZERO)?;
            let both = p.b("bothHands", false)?;
            out.push(("reachForWound", vec![on(), ("hitPart", Int(k)), ("hitPoint", V(hit)), ("reachWithOneHand", Bool(!both))]));
        }
        _ => {}
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every message and parameter a GTA 6 behaviour sends exists, with the right kind, for every choice it offers.
    #[test]
    fn presets_send_real_messages() {
        let at = |_: usize| Vec3::new(0.1, 0.2, 1.0);
        let vary: &[(&str, &[Value])] = &[
            ("bodyPart", &[Value::Int(-1), Value::Int(0), Value::Int(2), Value::Int(12), Value::Int(15)]),
            ("hitPart", &[Value::Int(-1), Value::Int(3)]),
            ("toughness", &[Value::Int(0), Value::Int(2)]), ("area", &[Value::Int(0), Value::Int(1), Value::Int(2)]),
            ("weapon", &[Value::Int(0), Value::Int(2)]), ("strength", &[Value::Int(0), Value::Int(2)]),
            ("bulletDir", &[Value::Vec3(Vec3::ZERO), Value::Vec3(Vec3::Y)]), ("hitNormal", &[Value::Vec3(Vec3::X)]),
            ("dir", &[Value::Vec3(Vec3::X)]), ("fromBehind", &[Value::Bool(true)]), ("toKnees", &[Value::Bool(true)]),
            ("tucked", &[Value::Bool(false)]), ("dying", &[Value::Bool(true)]), ("onGround", &[Value::Bool(true)]),
            ("knockedOut", &[Value::Bool(true)]), ("armed", &[Value::Bool(true)]), ("armsUp", &[Value::Float(0.5)]),
            ("fencePoint", &[Value::Vec3(Vec3::ONE)]), ("fenceEdge", &[Value::Vec3(Vec3::X)]),
        ];
        let kind_ok = |k: Kind, v: &Value| matches!((k, v), (Kind::Float, Value::Float(_)) | (Kind::Int, Value::Int(_))
            | (Kind::Bool, Value::Bool(_)) | (Kind::Vec3, Value::Vec3(_)) | (Kind::Str, Value::Str(_)));
        for m in &MESSAGES {
            let mut cases: Vec<Vec<(&str, Value)>> = vec![vec![]];
            for (n, vs) in vary {
                if m.params.iter().any(|q| q.name == *n) {
                    cases.extend(vs.iter().map(|v| vec![(*n, v.clone())]));
                }
            }
            for case in cases {
                for start in [true, false] {
                    let mut ps = case.clone();
                    ps.push(("start", Value::Bool(start)));
                    let lines = expand(m.name, &ps, &at).unwrap().unwrap_or_else(|e| panic!("{} {ps:?}: {e:?}", m.name));
                    assert!(!lines.is_empty() || !start, "{} sends nothing", m.name);
                    for (msg, lp) in lines {
                        let target = crate::msg::find(msg).unwrap_or_else(|| panic!("{}: no message {msg}", m.name));
                        assert!(!is(msg), "{} sends another GTA 6 behaviour", m.name);
                        for (n, v) in lp {
                            if n == "start" {
                                continue;
                            }
                            let q = target.params.iter().find(|q| q.name == n).unwrap_or_else(|| panic!("{}: {msg} has no {n}", m.name));
                            assert!(kind_ok(q.kind, &v), "{}: {msg}.{n} is not a {:?}", m.name, q.kind);
                        }
                    }
                }
            }
        }
        assert!(expand("bodyBalance", &[], &at).is_none());
        assert!(expand("gta6Roll", &[("tucked", Value::Bool(true))], &at).unwrap().unwrap().is_empty());
        assert!(expand("gta6Roll", &[("nope", Value::Bool(true))], &at).unwrap().is_err());
    }
}
