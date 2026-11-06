//! Red Dead Redemption 2's own human body for the RDR2 rig, built from the player's own copies of two files: the
//! ped's ragdoll (meta_base_male.yft / meta_base_female.yft, exported with OpenIV's "Save content" as `.yft.sys`, read
//! by `rdr2frag`) and its Euphoria joints (fred.xml / wilma.xml, read by `delirium::rig`). From the fragment: each
//! part's mass, inertia, shape and centre, each joint's pivot and axes, and which parts collide with each other; from
//! the XML: each joint's limits, motor flips, muscle and force caps (the fragment holds no muscles). The result goes in
//! its own rig slot (`nmrs::RDR2_FRED` / `RDR2_WILMA`) in the same tables as Homer's, so the rest of Delirium runs on
//! it unchanged; Homer's and Marge's slots are never touched. Nothing of either file is kept or written anywhere.
//!
//! How it maps: RDR2's 21 links are Delirium's 21 parts, by name. RDR2 keeps each joint's frames centred in its
//! limits (RAGE's), where GTA IV's tables hold the joint's own frame; the centring is undone with the same turns
//! Delirium gives GTA IV's joints the other way (`nmrs::rage_turn`), and the zero pose (every joint at zero) is built
//! from them, pelvis out. Each part's frame for the rest of Delirium is Homer's (or Marge's) in that zero pose, the
//! body's own (RDR2's link frame) is reached through the build's links, so GTA IV's clips, get-ups and drawing keep
//! working on it. It stands in Homer's standing pose.
//!
//! Where it isn't RDR2's: a link of two capsules (the thighs, the upper chest, the head) is one part, its mass and
//! inertia both capsules' about their centre of mass, its shape the first capsule (the widest radius of the two) at
//! that centre; the feet and hands are RDR2's boxes, their inertia Delirium's for that box.
use crate::rdr2frag::Frag;
use bevy::prelude::*;
use delirium::ragdoll::{Build, Limit, PartDef};
use delirium::rig::RigJoint;
use delirium_data::nm_asset;

/// RDR2's link for each of Delirium's parts, in Delirium's order.
const LINKS: [&str; 21] = [
    "skel_pelvis", "skel_l_thigh", "skel_l_calf", "skel_l_foot", "skel_r_thigh", "skel_r_calf", "skel_r_foot",
    "skel_spine0", "skel_spine1", "skel_spine2", "skel_spine4", "skel_neck0", "skel_head", "skel_l_clavicle",
    "skel_l_upperarm", "skel_l_forearm", "skel_l_hand", "skel_r_clavicle", "skel_r_upperarm", "skel_r_forearm",
    "skel_r_hand",
];

/// GTA IV's tables keep a mirror of Delirium's frame (x and y turned about).
fn ours(v: Vec3) -> Vec3 {
    Vec3::new(-v.x, -v.y, v.z)
}

fn quat(m: Mat3) -> Quat {
    Quat::from_mat3(&m).normalize()
}

/// A part's turn and place as GTA IV's tables keep them (`nmrs`'s asset frames).
fn table_part(t: Quat, at: Vec3) -> [[f32; 3]; 4] {
    let m = Mat3::from_quat(t);
    [(-m.row(0)).to_array(), (-m.row(1)).to_array(), m.row(2).to_array(), ours(at).to_array()]
}

pub struct Body {
    pub build: Build,
    pub frames: Vec<Transform>,
    /// what it is, for the log
    pub note: String,
}

/// RDR2's body in rig slot `slot`, from its fragment `fr` and its joints `xml` (Delirium's order), on `base` (Homer's
/// or Marge's build, standing in `base_frames`), whose rig slot's tables give the zero pose's frames and the joints'
/// names and kinds.
pub fn build(fr: &Frag, xml: &[RigJoint], base: &Build, base_frames: &[Transform], slot: usize) -> Result<Body, String> {
    let (Some(asset), Some(table)) = (delirium::nmrs::ASSET[base.nm].get(), delirium::nmrs::PARTS[base.nm].get()) else {
        return Err("GTA IV's body isn't loaded".into());
    };
    let n = LINKS.len();
    if base.parts.len() != n || asset.len() != n - 1 || table.len() != n || xml.len() != n - 1 || base_frames.len() != n {
        return Err("needs GTA IV's 21-part body".into());
    }
    if fr.links.len() != n {
        return Err(format!("not RDR2's human ragdoll ({} links, not 21)", fr.links.len()));
    }
    let link_of: Vec<usize> = LINKS.iter().map(|name| fr.links.iter().position(|l| l.name == *name).ok_or_else(|| format!("not RDR2's human ragdoll (no {name})")))
        .collect::<Result<_, _>>()?;
    // each part: its link's frame (its first capsule's turn at the centre of mass of all its capsules), mass,
    // inertia about that centre along that turn, and shape
    struct P { turn: Quat, mass: f32, inertia: Vec3, radius: f32, half: f32, boxed: Option<Vec3> }
    let mut parts = vec![];
    for &l in &link_of {
        let link = &fr.links[l];
        let (turn, com) = link.frame();
        if turn.determinant() < 0.0 {
            return Err(format!("{}: a mirrored frame", link.name));
        }
        let mass: f32 = link.capsules.iter().map(|c| c.mass).sum();
        let mut tensor = Mat3::ZERO;
        for c in &link.capsules {
            let d = c.at + c.turn * c.cg - com;
            tensor += c.turn * Mat3::from_diagonal(c.inertia) * c.turn.transpose() + (Mat3::IDENTITY * d.length_squared() - Mat3::from_cols(d * d.x, d * d.y, d * d.z)) * c.mass;
        }
        let local = turn.transpose() * tensor * turn;
        let first = &link.capsules[0];
        let radius = link.capsules.iter().map(|c| c.radius).fold(first.radius, f32::max);
        let (radius, half) = match first.boxed {
            Some(e) => (e.min_element(), 0.0),
            None => (radius, (first.half - radius).max(0.0)),
        };
        parts.push(P { turn: quat(turn), mass, inertia: Vec3::new(local.x_axis.x, local.y_axis.y, local.z_axis.z), radius, half, boxed: first.boxed });
    }
    // the zero pose, in the fragment's frame (x left, y back, z up: Delirium's for GTA IV's tables), pelvis out
    let (mut turn, mut at) = (vec![Quat::IDENTITY; n], vec![Vec3::ZERO; n]);
    (turn[0], at[0]) = (parts[0].turn, fr.links[link_of[0]].frame().1);
    let mut joints = vec![];
    let mut worst = 0.0f32;
    for c in 1..n {
        let i = c - 1;
        let p = base.parts[c].parent.ok_or("GTA IV's body has a second root")?;
        let j = fr.joints.iter().find(|j| j.child == link_of[c] && j.parent == link_of[p])
            .ok_or_else(|| format!("RDR2's ragdoll doesn't join {} to {}", LINKS[c], LINKS[p]))?;
        if j.in_parent.determinant() < 0.0 || j.in_child.determinant() < 0.0 {
            return Err(format!("{}: a mirrored joint frame", LINKS[c]));
        }
        let (fp, fc) = (quat(j.in_parent), quat(j.in_child));
        let three = asset[i].three;
        let (ta, tb) = delirium::nmrs::rage_turn(three, &xml[i].limits);
        let frame = (turn[p] * fp * ta.inverse()).normalize();
        turn[c] = (frame * tb * fc.inverse()).normalize();
        let pivot = at[p] + turn[p] * j.at_parent;
        at[c] = pivot - turn[c] * j.at_child;
        // (the fragment's own pose: its pivot seen from both links, a check the reading is right)
        let ((tp, cp), (tc, cc)) = (fr.links[link_of[p]].frame(), fr.links[link_of[c]].frame());
        worst = worst.max((cp + tp * j.at_parent).distance(cc + tc * j.at_child));
        joints.push((frame, pivot, fp, fc));
    }
    if worst > 0.02 {
        return Err(format!("RDR2's ragdoll's joints don't meet ({worst:.3} m apart): not a fragment I can read"));
    }
    // put it in GTA IV's tables where Homer's pelvis is
    let shift = ours(Vec3::from(table[0][3])) - at[0];
    let zero = delirium::nmrs::zero_turns(base);
    // the tables
    let table_parts: Vec<[[f32; 3]; 4]> = (0..n).map(|k| table_part(turn[k], at[k] + shift)).collect();
    let table_joints: Vec<nm_asset::Joint> = joints.iter().enumerate().map(|(i, &(frame, pivot, _, _))| {
        let (h, x) = (&asset[i], &xml[i]);
        nm_asset::Joint {
            name: h.name.clone(), index: h.index, three: h.three, pos: ours(pivot + shift).to_array(), twist_axis: ours(frame * Vec3::Z).to_array(),
            lean_axis: if h.three { ours(frame * Vec3::X).to_array() } else { h.lean_axis }, limits: x.limits, flip: x.flip,
            muscle_stiffness: x.muscle_stiffness, damping: x.damping, strength: x.strength,
            max_torque: if h.three { x.max_torque } else { [x.max_torque[0], h.max_torque[1]] }, centred: h.centred,
        }
    }).collect();
    let masses: Vec<f32> = parts.iter().map(|p| p.mass).collect();
    let mut pairs: Vec<(usize, usize)> = fr.pairs.iter().filter_map(|&(a, b)| Some((link_of.iter().position(|&l| l == a)?, link_of.iter().position(|&l| l == b)?)))
        .map(|(a, b)| (a.min(b), a.max(b))).collect();
    pairs.sort();
    pairs.dedup();
    let npairs = pairs.len();
    // the links' own capsules, each in its link's frame, for the legs' meeting test
    *delirium::ragdoll::LINK_CAPSULES.write().unwrap() = link_of.iter().map(|&l| {
        let (turn, com) = fr.links[l].frame();
        fr.links[l].capsules.iter().map(|c| {
            let e = c.turn * Vec3::Y * (c.half - c.radius).max(0.0);
            (turn.transpose() * (c.at - e - com), turn.transpose() * (c.at + e - com), c.radius)
        }).collect()
    }).collect();
    delirium::nmrs::set(slot, table_joints, table_parts, masses.clone(), pairs);
    // each part: Homer's frame in the zero pose; its body's (RDR2's link) through the build's links
    let links: Vec<Quat> = (0..n).map(|k| (zero[k].inverse() * turn[k]).normalize()).collect();
    let defs = (0..n).map(|k| {
        let (b, p) = (&base.parts[k], &parts[k]);
        let limit = match k.checked_sub(1) {
            None => Limit::Locked,
            Some(i) => {
                let (_, _, fp, fc) = joints[i];
                let q = b.parent.unwrap_or(0);
                let (pf, cf) = ((links[q] * fp).normalize(), (links[k] * fc).normalize());
                let l = xml[i].limits;
                let half = |(lo, hi): (f32, f32)| (hi - lo) * 0.5;
                if asset[i].three {
                    Limit::Bone { pf, cf, lean: [half(l[0]), half(l[1])], twist: half(l[2]) }
                } else {
                    Limit::BoneHinge { pf, cf, lo: l[0].0, hi: l[0].1 }
                }
            }
        };
        PartDef { bone: b.bone, parent: b.parent, mass: p.mass, radius: p.radius, half: p.half, offset: b.offset, rest: Quat::IDENTITY,
                  anchor_parent: Vec3::ZERO, anchor: Vec3::ZERO, limit, lower: b.lower, reach: b.reach, name: b.name.clone(), bind: b.bind,
                  boxed: p.boxed, inertia: p.inertia, size: 1.0, heft: 1.0, grown: 1.0, length: 0.0 }
    }).collect();
    let mut build = Build { parts: defs, nm: slot, links };
    // standing as Homer stands: his parts' turns, placed on RDR2's pivots
    let mut frames = base_frames.to_vec();
    delirium::nmrs::asset_layout(&mut build, &mut frames);
    for k in 0..n {
        build.parts[k].bind = frames[k].rotation;
        build.parts[k].rest = build.parts[k].parent.map_or(Quat::IDENTITY, |q| frames[k].rotation.inverse() * frames[q].rotation);
    }
    let mass: f32 = masses.iter().sum();
    let low = build.parts.iter().zip(&frames).map(|(p, f)| f.translation.z - p.radius - p.half).fold(f32::MAX, f32::min);
    let top = build.parts.iter().zip(&frames).map(|(p, f)| f.translation.z + p.radius + p.half).fold(f32::MIN, f32::max);
    let note = format!("RDR2's body from its ragdoll: {mass:.1} kg, {:.2} m, {npairs} self-collision pairs", top - low);
    Ok(Body { build, frames, note })
}

/// The player's export of RDR2's ragdoll for Fred (or Wilma): in one of `dirs`, in OpenIV's folder for it there
/// (`meta_base_male_yft`), or in that folder in a `human` folder beside it.
pub fn find(dirs: &[&str], female: bool) -> Option<std::path::PathBuf> {
    let name = if female { "meta_base_female" } else { "meta_base_male" };
    dirs.iter().filter(|d| !d.is_empty()).flat_map(|d| {
        let d = std::path::Path::new(d);
        [d.to_path_buf(), d.join(format!("{name}_yft")), d.join("..").join("human").join(format!("{name}_yft"))]
    }).map(|d| d.join(format!("{name}.yft.sys"))).find(|p| p.is_file())
}
