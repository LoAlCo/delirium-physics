//! The rig's joint values: GTA IV's (from the player's GTA IV, the default) or Red Dead Redemption 2's, read from the
//! player's own RDR2 naturalmotion XML (fred.xml for Homer's slot, wilma.xml for Marge's). RDR2's game reads only
//! these from that file: each joint's limits, motor flips, muscle stiffness, strength, damping and force caps. Its
//! joint pivots and axes, masses, shapes and self-collision come from the ped's physics fragment, so with the XML alone
//! this is RDR2's joints on Homer's (or Marge's) body. Nothing changes unless an RDR2 rig is set.
use std::sync::RwLock;

/// One joint's values from RDR2's rig.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RigJoint {
    pub limits: [(f32, f32); 3],
    pub flip: [bool; 3],
    pub muscle_stiffness: f32,
    pub damping: f32,
    pub strength: f32,
    pub max_torque: [f32; 2],
}

/// RDR2's joints for each rig slot (Homer's, Marge's), when set.
static RDR2: [RwLock<Option<Vec<RigJoint>>>; 2] = [const { RwLock::new(None) }; 2];

/// RDR2's joint names for each of Delirium's joints, in Delirium's order.
const NAMES: [&[&str]; 20] = [
    &["hip_left"], &["knee_left"], &["ankle_left"], &["hip_right"], &["knee_right"], &["ankle_right"],
    &["spine_0"], &["spine_1"], &["spine_2"], &["spine_3"], &["neck_lower"], &["neck_upper", "upper_neck"],
    &["clavicle_jnt_left", "clavicle_left"], &["shoulder_left"], &["elbow_left"], &["wrist_left"],
    &["clavicle_jnt_right", "clavicle_right"], &["shoulder_right"], &["elbow_right"], &["wrist_right"],
];

fn tag<'a>(s: &'a str, t: &str) -> Option<&'a str> {
    // (ASCII lower case keeps every byte where it was, so the offset is the original's)
    let a = s.to_ascii_lowercase().find(&format!("<{}>", t.to_ascii_lowercase()))? + t.len() + 2;
    Some(s[a..].split('<').next()?.trim())
}

fn num(s: &str, t: &str) -> Option<f32> {
    tag(s, t)?.parse().ok()
}

/// The XML's joint blocks: (name, three-dof, body).
fn blocks(xml: &str) -> Vec<(String, bool, &str)> {
    let mut out = vec![];
    for (kind, three) in [("threedof", true), ("onedof", false)] {
        let open = format!("<{kind} name=\"");
        let mut rest = xml;
        while let Some(i) = rest.find(&open) {
            let r = &rest[i + open.len()..];
            let name = r.split('"').next().unwrap_or("").to_string();
            let end = r.find(&format!("</{kind}>")).unwrap_or(r.len());
            out.push((name, three, &r[..end]));
            rest = &r[end..];
        }
    }
    out
}

/// Reads an RDR2 biped rig XML into its 20 joints, in Delirium's order.
pub fn parse(xml: &str) -> Result<Vec<RigJoint>, String> {
    if !xml.contains("<bodyType>biped</bodyType>") {
        return Err("not an RDR2 biped rig".into());
    }
    let all = blocks(xml);
    NAMES.iter().map(|names| {
        let (_, three, b) = all.iter().find(|(n, _, _)| names.contains(&n.as_str())).ok_or_else(|| format!("no RDR2 joint {}", names[0]))?;
        let pair = |lo: &str, hi: &str| (num(b, lo).unwrap_or(0.0), num(b, hi).unwrap_or(0.0));
        let limits = if *three {
            [pair("minFirstLeanAngle", "maxFirstLeanAngle"), pair("minSecondLeanAngle", "maxSecondLeanAngle"), pair("minTwistAngle", "maxTwistAngle")]
        } else {
            [pair("minAngle", "maxAngle"), (0.0, 0.0), (0.0, 0.0)]
        };
        let flag = |t: &str| tag(b, t) == Some("true");
        let need = |t: &str| num(b, t).ok_or_else(|| format!("{}: no {t}", names[0]));
        Ok(RigJoint {
            limits, flip: [flag("reverseFirstLeanMotor"), flag("reverseSecondLeanMotor"), flag("reverseTwistMotor")],
            muscle_stiffness: need("defaultMuscleStiffness")?, damping: need("defaultMuscleDamping")?, strength: need("defaultMuscleStrength")?,
            max_torque: [need("defaultLeanForceCap")?, if *three { need("defaultTwistForceCap")? } else { 0.0 }],
        })
    }).collect()
}

/// Sets (Some) or clears (None) RDR2's rig for a rig slot (crate::nmrs::HOMER or MARGE): it takes effect for every
/// character built after this.
pub fn set_rdr2(nm: usize, joints: Option<Vec<RigJoint>>) {
    if let Some(slot) = RDR2.get(nm) {
        *slot.write().unwrap_or_else(|e| e.into_inner()) = joints;
    }
}

/// Whether RDR2's rig is set for this slot.
pub fn is_rdr2(nm: usize) -> bool {
    RDR2.get(nm).is_some_and(|s| s.read().unwrap_or_else(|e| e.into_inner()).is_some())
}

/// Joint `i` of rig slot `nm` with RDR2's values, when RDR2's rig is set; otherwise as it is.
pub fn patch(nm: usize, i: usize, j: &delirium_data::nm_asset::Joint) -> delirium_data::nm_asset::Joint {
    let mut j = j.clone();
    if let Some(r) = RDR2.get(nm).and_then(|s| s.read().unwrap_or_else(|e| e.into_inner()).as_ref().and_then(|v| v.get(i).copied())) {
        (j.limits, j.flip, j.muscle_stiffness, j.damping, j.strength) = (r.limits, r.flip, r.muscle_stiffness, r.damping, r.strength);
        j.max_torque = if j.three { r.max_torque } else { [r.max_torque[0], j.max_torque[1]] };
    }
    j
}
