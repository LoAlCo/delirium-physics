//! GTA 6 bodies on the RDR2 core. GTA 6's Euphoria character files aren't in its data, but its ragdoll body is: the
//! player decodes their own copy into a "gta6-ragdoll-1" JSON (per part: the GTA 6 bone, a mass value and the
//! collision shapes; plus the skeleton's bone positions). This builds an RDR2 body (nm_character_from_rdr2: RDR2's
//! joint limits, motor flips, muscles and force caps, from the player's own fred.xml or wilma.xml) and gives it GTA 6's
//! mass distribution and proportions: the base's total mass shared out as GTA 6 shares it, and the thighs, shins, upper
//! arms, forearms and trunk sized as GTA 6's skeleton has them. Nothing from GTA 6 is stored here; the numbers come from
//! the player's file each time. Existing bodies are unchanged.
use crate::character::{BuildPart, Character};
use crate::rdr2::PARENT;
use crate::*;
use serde::Deserialize;
use std::collections::HashMap;

pub const FORMAT_GTA6: &str = "gta6-ragdoll-1";

#[derive(Deserialize)]
pub struct Gta6Part {
    pub bone: String,
    pub mass: f32,
}

#[derive(Deserialize)]
pub struct Gta6Ragdoll {
    pub format: String,
    pub parts: Vec<Gta6Part>,
    pub bones: HashMap<String, [f32; 3]>,
}

/// GTA 6's ragdoll bone for each Delirium part. GTA 6's ragdoll has Spine4 and Spine5 where RDR2's has Spine2 and Spine4
/// (matched by order up the spine), Acromion for the clavicle part and Neck1 for the neck part.
pub const BONE_OF: [&str; 21] = [
    "SKEL_Pelvis", "SKEL_L_Thigh", "SKEL_L_Calf", "SKEL_L_Foot", "SKEL_R_Thigh", "SKEL_R_Calf", "SKEL_R_Foot",
    "SKEL_Spine0", "SKEL_Spine1", "SKEL_Spine4", "SKEL_Spine5", "SKEL_Neck1", "SKEL_Head",
    "SKEL_L_Acromion", "SKEL_L_UpperArm", "SKEL_L_Forearm", "SKEL_L_Hand",
    "SKEL_R_Acromion", "SKEL_R_UpperArm", "SKEL_R_Forearm", "SKEL_R_Hand",
];

/// The other side's part (itself for the middle ones).
const MIRROR: [usize; 21] = [0, 4, 5, 6, 1, 2, 3, 7, 8, 9, 10, 11, 12, 17, 18, 19, 20, 13, 14, 15, 16];

/// Limb parts sized from GTA 6's skeleton: (part, bone at its start, bone at its end).
const LIMBS: [(usize, &str, &str); 8] = [
    (1, "SKEL_L_Thigh", "SKEL_L_Calf"), (2, "SKEL_L_Calf", "SKEL_L_Foot"),
    (4, "SKEL_R_Thigh", "SKEL_R_Calf"), (5, "SKEL_R_Calf", "SKEL_R_Foot"),
    (14, "SKEL_L_UpperArm", "SKEL_L_Forearm"), (15, "SKEL_L_Forearm", "SKEL_L_Hand"),
    (18, "SKEL_R_UpperArm", "SKEL_R_Forearm"), (19, "SKEL_R_Forearm", "SKEL_R_Hand"),
];

/// Trunk parts: scaled together so the pelvis-to-head height is GTA 6's.
const TRUNK: [usize; 8] = [0, 7, 8, 9, 10, 11, 13, 17];

fn dist(a: [f32; 3], b: [f32; 3]) -> f32 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2) + (a[2] - b[2]).powi(2)).sqrt()
}

impl Gta6Ragdoll {
    pub fn parse(t: &str) -> Result<Gta6Ragdoll, String> {
        let r: Gta6Ragdoll = serde_json::from_str(t).map_err(|e| e.to_string())?;
        if r.format != FORMAT_GTA6 {
            return Err(format!("format {:?}, not {FORMAT_GTA6:?}", r.format));
        }
        for b in BONE_OF {
            if !r.parts.iter().any(|p| p.bone == b) {
                return Err(format!("no part for {b}"));
            }
        }
        Ok(r)
    }

    /// Each Delirium part's share of the whole body's mass; `symmetric` averages left and right.
    pub fn shares(&self, symmetric: bool) -> [f32; 21] {
        let raw: [f32; 21] = std::array::from_fn(|d| self.parts.iter().find(|p| p.bone == BONE_OF[d]).map_or(0.0, |p| p.mass.max(0.0)));
        let m: [f32; 21] = std::array::from_fn(|d| if symmetric { 0.5 * (raw[d] + raw[MIRROR[d]]) } else { raw[d] });
        let total: f32 = m.iter().sum::<f32>().max(1e-6);
        m.map(|x| x / total)
    }

    fn bone(&self, n: &str) -> Option<[f32; 3]> {
        self.bones.get(n).copied()
    }
}

impl Character {
    /// An RDR2 body (`xml`) on `base`, with GTA 6's mass distribution from `rag` (`symmetric`: left and right alike).
    pub fn from_gta6(rag: &Gta6Ragdoll, xml: &str, base: &Character, symmetric: bool) -> Result<Character, String> {
        let mut c = Character::from_rdr2(xml, base)?;
        let total: f32 = c.parts.iter().map(|p| p.mass).sum();
        for (p, s) in c.parts.iter_mut().zip(rag.shares(symmetric)) {
            p.mass = total * s;
        }
        Ok(c)
    }

    /// The 21 fit points of `fit_points`, moved so the limbs and trunk have GTA 6's lengths.
    pub fn gta6_fit_points(&self, rag: &Gta6Ragdoll) -> Vec<[f32; 3]> {
        let p = self.fit_points();
        let mut scale = [1.0f32; 21];
        // a part's length: from its joint to its child's joint (fit point d is the joint into part d)
        let child = |d: usize| (1..21).find(|&k| PARENT[k] == d);
        for (d, a, b) in LIMBS {
            if let (Some(k), Some(ga), Some(gb)) = (child(d), rag.bone(a), rag.bone(b)) {
                let ours = dist(p[k], p[d]);
                if ours > 1e-4 {
                    scale[d] = dist(ga, gb) / ours;
                }
            }
        }
        if let (Some(pel), Some(head)) = (rag.bone("SKEL_Pelvis"), rag.bone("SKEL_Head")) {
            let ours = dist(p[12], p[0]);
            if ours > 1e-4 {
                let s = dist(head, pel) / ours;
                for d in TRUNK {
                    scale[d] = s;
                }
            }
        }
        let mut out = p.clone();
        for d in 1..21 {
            let up = PARENT[d];
            out[d] = [0, 1, 2].map(|i| out[up][i] + (p[d][i] - p[up][i]) * scale[up]);
        }
        out
    }
}

/// A GTA 6 body as `nm-character-1` JSON, on the RDR2 core: `ragdoll_path` the player's own decoded GTA 6 ragdoll
/// ("gta6-ragdoll-1" JSON), `xml_path` an RDR2 naturalmotion XML (fred.xml for a man, wilma.xml for a woman), `base_json`
/// a base body (from nm_character_from_xrt). With `build_path` (the base's build dump) the result carries a build fitted
/// to GTA 6's proportions, its masses shared out as GTA 6's. `symmetric` != 0 averages left and right masses (GTA 6's
/// left parts weigh more than its right ones; unexplained). Writes the JSON into `buf` like nm_character_from_rdr2.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nm_character_from_gta6(ragdoll_path: *const c_char, xml_path: *const c_char, base_json: *const c_char,
                                                build_path: *const c_char, symmetric: i32, buf: *mut c_char, len: i32) -> i32 {
    let (Some(rp), Some(xp), Some(bj)) = (unsafe { text(ragdoll_path) }, unsafe { text(xml_path) }, unsafe { text(base_json) }) else {
        return fail(NM_E_ARG, "nm_character_from_gta6: no ragdoll, xml or base");
    };
    let rag = match std::fs::read_to_string(rp).map_err(|e| e.to_string()).and_then(|t| Gta6Ragdoll::parse(&t)) {
        Ok(r) => r,
        Err(e) => return fail(NM_E_PARSE, format!("{rp}: {e}")),
    };
    let xml = match std::fs::read_to_string(xp) {
        Ok(t) => t,
        Err(e) => return fail(NM_E_IO, format!("{xp}: {e}")),
    };
    let base = match Character::parse(bj) {
        Ok(c) => c,
        Err(e) => return fail(NM_E_PARSE, format!("nm_character_from_gta6: base: {e}")),
    };
    let mut c = match Character::from_gta6(&rag, &xml, &base, symmetric != 0) {
        Ok(c) => c,
        Err(e) => return fail(NM_E_PARSE, format!("nm_character_from_gta6: {e}")),
    };
    if let Some(bp) = unsafe { text(build_path) } {
        let Ok(t) = std::fs::read_to_string(bp) else { return fail(NM_E_IO, format!("{bp}: unreadable")) };
        let Ok((b, f)) = std::panic::catch_unwind(move || Build::undump(&t)) else {
            return fail(NM_E_PARSE, "nm_character_from_gta6: not a build dump");
        };
        let pts: Vec<Vec3> = c.gta6_fit_points(&rag).iter().map(|p| Vec3::from_array(*p)).collect();
        let (fb, ff) = b.fit(&f, &pts);
        let mut parts: Vec<BuildPart> = fb.parts.iter().zip(&ff).map(|(p, fr)| BuildPart::from_def(p, fr)).collect();
        // the build's masses as the character's, its inertia scaled with them (the shapes are the fitted base's)
        let total: f32 = parts.iter().map(|p| p.mass).sum();
        for (bp, s) in parts.iter_mut().zip(rag.shares(symmetric != 0)) {
            let m = total * s;
            if bp.mass > 1e-6 {
                bp.inertia = bp.inertia.map(|x| x * m / bp.mass);
            }
            bp.mass = m;
        }
        c.build = Some(parts);
    }
    match serde_json::to_string_pretty(&c) {
        Ok(s) => unsafe { put(&s, buf, len) },
        Err(e) => fail(NM_E_PARSE, e.to_string()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rag(masses: [f32; 21]) -> Gta6Ragdoll {
        Gta6Ragdoll { format: FORMAT_GTA6.into(), bones: HashMap::new(),
                      parts: BONE_OF.iter().zip(masses).map(|(b, m)| Gta6Part { bone: b.to_string(), mass: m }).collect() }
    }

    #[test]
    fn shares_add_up_and_mirror() {
        let mut m = [1.0f32; 21];
        m[1] = 3.0; // left thigh heavier than the right
        let r = rag(m);
        let s = r.shares(false);
        assert!((s.iter().sum::<f32>() - 1.0).abs() < 1e-5);
        assert!(s[1] > s[4]);
        let s = r.shares(true);
        assert!((s.iter().sum::<f32>() - 1.0).abs() < 1e-5);
        assert!((s[1] - s[4]).abs() < 1e-6);
    }

    #[test]
    fn parse_wants_every_part() {
        let mut r = rag([1.0; 21]);
        r.parts.pop();
        let t = serde_json::json!({"format": FORMAT_GTA6, "bones": {},
            "parts": r.parts.iter().map(|p| serde_json::json!({"bone": p.bone, "mass": p.mass})).collect::<Vec<_>>()}).to_string();
        assert!(Gta6Ragdoll::parse(&t).is_err());
        assert!(Gta6Ragdoll::parse("{\"format\":\"other\",\"parts\":[],\"bones\":{}}").is_err());
    }
}
