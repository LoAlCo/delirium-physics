use bevy_math::Vec3;

use crate::nmrs::Character;
use crate::nmshot::{limb, Ctx};

#[derive(Clone, Copy, Debug)]
pub struct PointArmParams {
    pub target: Vec3,
    pub twist: f32,
    pub arm_straightness: f32,
    pub arm_stiffness: f32,
    pub arm_damping: f32,
    #[allow(dead_code)]
    pub instance_index: i32,
    pub point_swing_limit: f32,
    pub use_left_arm: bool,
    pub use_zero_pose_when_not_pointing: bool,
    /// RDR2's per-arm parameters (left, right); Some for a message sent under the RDR2 core
    pub rdr2: Option<[Rdr2Arm; 2]>,
}

/// RDR2's pointArm parameters for one arm (its message has a set for each arm).
#[derive(Clone, Copy, Debug)]
pub struct Rdr2Arm {
    pub target: Vec3,
    pub twist: f32,
    pub arm_straightness: f32,
    pub arm_stiffness: f32,
    pub arm_damping: f32,
    pub point_swing_limit: f32,
    pub use_arm: bool,
    pub use_zero_pose: bool,
}

impl Default for Rdr2Arm {
    fn default() -> Rdr2Arm {
        Rdr2Arm { target: Vec3::ZERO, twist: 0.3, arm_straightness: 0.8, arm_stiffness: 15.0, arm_damping: 1.0, point_swing_limit: 1.5,
                  use_arm: false, use_zero_pose: false }
    }
}

/// RDR2's reach: the target is put this far times armStraightness from the shoulder.
const RDR2_REACH: f32 = 0.7;
/// RDR2's pointing hysteresis band about pointSwingLimit.
const RDR2_BAND: f32 = 0.1;

#[derive(Clone, Debug)]
pub struct PointArm {
    pub p: PointArmParams,
    pub(crate) reach: f32,
    pub active: bool,
    pub(crate) pointing: bool,
    /// RDR2: each arm's pointing flag (left, right)
    pointing2: [bool; 2],
    /// RDR2 core: the arms (left, right) animPose holds this tick (its overidePointArm and effectorMask)
    pub held: [bool; 2],
}

impl Default for PointArmParams {
    fn default() -> PointArmParams {
        PointArmParams { target: Vec3::ZERO, twist: delirium_data::kv!(24), arm_straightness: delirium_data::kv!(20), arm_stiffness: delirium_data::kv!(30), arm_damping: 1.0, instance_index: -1,
                         point_swing_limit: delirium_data::kv!(12), use_left_arm: false, use_zero_pose_when_not_pointing: false,
                         rdr2: None }
    }
}

impl Default for PointArm {
    fn default() -> PointArm {
        PointArm { p: PointArmParams::default(), reach: delirium_data::kv!(74), active: false, pointing: false, pointing2: [true; 2], held: [false; 2] }
    }
}

fn swing(ch: &Character, j: usize) -> f32 {
    let l = ch.limits(j);
    let d = ch.eff[j].desired;
    let (m0, h0) = ((l[0] + l[1]) * 0.5, (l[1] - l[0]) * 0.5);
    let (m1, h1) = ((l[2] + l[delirium_data::kv!(0 int)]) * 0.5, (l[delirium_data::kv!(0 int)] - l[2]) * 0.5);
    (((d[0] - m0) / h0).powi(2) + ((d[1] - m1) / h1).powi(2)).sqrt()
}

pub(crate) fn share(ch: &mut Character, frames: &[crate::prelude::Transform], clav: usize, shoulder: usize) {
    let (cl, sl) = (ch.limits(clav), ch.limits(shoulder));
    let now = ch.actual(clav, frames);
    for k in 0..2 {
        let t = now[k] + ch.eff[shoulder].desired[k];
        let (lo, hi) = (2 * k, 2 * k + 1);
        let i = if t >= 0.0 { hi } else { lo };
        let s = 1.0 / (cl[i] + sl[i]);
        ch.eff[clav].desired[k] = cl[i] * s * t;
        ch.eff[shoulder].desired[k] = sl[i] * s * t;
    }
}

impl PointArm {
    pub fn activate(&mut self, p: PointArmParams) {
        self.p = p;
        if !self.active {
            (self.active, self.pointing, self.pointing2) = (true, true, [true; 2]);
        }
    }

    pub fn deactivate(&mut self) {
        self.active = false;
    }

    pub fn tick(&mut self, c: &mut Ctx) {
        if !self.active {
            return;
        }
        if let Some(arms) = self.p.rdr2.filter(|_| c.ch.rdr2) {
            // the right arm first, as RDR2 does, skipping an arm animPose (slot 0xd8) holds
            for i in [1, 0] {
                if arms[i].use_arm && !self.held[i] {
                    self.arm_rdr2(c, i, arms[i]);
                }
            }
            return;
        }
        let p = self.p;
        let (l, side, mask) = if p.use_left_arm { (limb(0), 1.0, "ul") } else { (limb(1), -1.0, "ur") };
        let at = c.b.joint_pos(l.e[1]);
        let t = (crate::rigid::nrm(p.target - at) * self.reach).mul_add(Vec3::splat(p.arm_straightness), at);
        let (clav, sh) = (c.ch.eff[l.e[0]].desired, c.ch.eff[l.e[1]].desired);
        c.ch.set_stiffness(p.arm_stiffness, p.arm_damping, mask, None);
        crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[delirium_data::kv!(0 int)], t, p.twist, 1.0, side, 1.0, Vec3::ZERO, None);
        share(c.ch, c.b.frames, l.e[0], l.e[1]);
        let s = swing(c.ch, l.e[0]);
        if !self.pointing && s < p.point_swing_limit - delirium_data::kv!(78) {
            self.pointing = true;
        }
        if self.pointing && s > p.point_swing_limit + delirium_data::kv!(78) {
            self.pointing = false;
        }
        let ch = &mut *c.ch;
        if self.pointing {
            for j in l.e {
                ch.eff[j].oppose_gravity = 1.0;
            }
            ch.eff[l.e[delirium_data::kv!(0 int)]].desired = [0.0; 3];
            ch.eff[l.e[0]].muscle_stiffness = delirium_data::kv!(12);
            ch.eff[l.e[1]].muscle_stiffness = 0.5;
            ch.eff[l.e[2]].muscle_stiffness = 0.5;
            ch.eff[l.e[delirium_data::kv!(0 int)]].desired[0] = ch.eff[l.e[2]].desired[0] * -0.5;
        } else if p.use_zero_pose_when_not_pointing {
            ch.eff[l.e[0]].desired = [0.0; 3];
            ch.eff[l.e[1]].desired = [0.0, 0.5, 0.0];
            ch.eff[l.e[2]].desired[0] = 0.5;
            for j in Character::mask(mask) {
                ch.eff[j].blend_to_zero(1.0);
            }
            ch.set_stiffness(delirium_data::kv!(65), 1.0, mask, None);
        } else {
            ch.eff[l.e[0]].desired = clav;
            ch.eff[l.e[1]].desired = sh;
        }
    }

    /// RDR2's arm (rdr2_pointarm.md section 3): the hand reaches 0.7 x armStraightness from the shoulder towards the
    /// target; the arm's stiffness; the clavicle's commanded swing (last step's, against its limits) with a 0.1
    /// band decides pointing. Pointing: oppose gravity on the arm, clavicle muscle 1.5, the wrist straight and bent
    /// back half the elbow. Not pointing: the clavicle and shoulder held as last commanded, or the zero pose at
    /// stiffness 8 (RDR2's 8 x the character's stiffness scale, 1 here). The IK request is Delirium's arm IK, with
    /// its clavicle share standing in for RDR2's limb IK service (not read); no instances, so the target is world.
    fn arm_rdr2(&mut self, c: &mut Ctx, i: usize, a: Rdr2Arm) {
        let (l, side, mask) = if i == 0 { (limb(0), 1.0, "ul") } else { (limb(1), -1.0, "ur") };
        let (clav, sh, elbow) = (c.ch.eff[l.e[0]].desired, c.ch.eff[l.e[1]].desired, c.ch.eff[l.e[2]].desired[0]);
        let g = swing(c.ch, l.e[0]);
        let at = c.b.joint_pos(l.e[1]);
        let t = crate::rigid::nrm(a.target - at) * (RDR2_REACH * a.arm_straightness) + at;
        c.ch.set_stiffness(a.arm_stiffness, a.arm_damping, mask, None);
        crate::nmdbm::limb_ik(c.b, c.ch, l.e[1], l.e[2], l.p[0], l.p[3], t, a.twist, 1.0, side, 1.0, Vec3::ZERO, None);
        share(c.ch, c.b.frames, l.e[0], l.e[1]);
        let pointing = &mut self.pointing2[i];
        if !*pointing && g < a.point_swing_limit - RDR2_BAND {
            *pointing = true;
        }
        if *pointing && g > a.point_swing_limit + RDR2_BAND {
            *pointing = false;
        }
        let ch = &mut *c.ch;
        if *pointing {
            for j in l.e {
                ch.eff[j].oppose_gravity = 1.0;
            }
            ch.eff[l.e[0]].muscle_stiffness = 1.5;
            ch.eff[l.e[1]].muscle_stiffness = 0.5;
            ch.eff[l.e[2]].muscle_stiffness = 0.5;
            ch.eff[l.e[3]].desired = [-0.5 * elbow, 0.0, 0.0];
        } else if a.use_zero_pose {
            ch.eff[l.e[0]].desired = [0.0; 3];
            ch.eff[l.e[1]].desired = [0.0, 0.5, 0.0];
            ch.eff[l.e[2]].desired[0] = 0.5;
            for j in Character::mask(mask) {
                ch.eff[j].blend_to_zero(1.0);
            }
            ch.set_stiffness(8.0, 1.0, mask, None);
        } else {
            ch.eff[l.e[0]].desired = clav;
            ch.eff[l.e[1]].desired = sh;
        }
    }
}
