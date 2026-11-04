//! RDR2 core's limb requests (experimental): Red Dead Redemption 2's behaviours don't write joints, they queue
//! requests that each limb resolves once a step, the highest-priority request first, later ones filling only what is
//! left (`acc = (1 - accW) req + accW acc`, `accW = min(accW + w, 1)`). GTA IV's (and Delirium's) behaviours write
//! the joints directly and the last one to write wins.
//!
//! Delirium's behaviours still write the joints, so this records each one's writes as its request: a snapshot of the
//! seven channels RDR2 keeps per joint (three angles, strength, damping, muscle stiffness, oppose gravity) is taken
//! when the frame starts, and after each behaviour whatever it changed becomes a request at its priority. Before the
//! muscles are driven the requests are resolved RDR2's way. A behaviour that wrote a value already there isn't seen
//! as writing it. Behaviours write at full weight, so the highest-priority writer of each channel wins.
//!
//! Fitted to Delirium's GTA IV behaviours:
//! - the shot's modules each make their own request; its reaching for the wound ranks as RDR2's separate
//!   reachForWound, and once catchFall runs the shot ranks below it;
//! - the balancer's lean of the lowest spine joint (GTA IV's aid, which bodyBalance and the shot override by design;
//!   in RDR2 that joint is the spine limb's, not the legs') and its ankles (set every step as defaults the shot and
//!   the others change: with them first, he fell after a plain shot and on ramps) rank below every behaviour;
//! - RDR2's per-joint scale layer (the shot's looseness law lowering strength, the lowest wins) is not here: the GTA
//!   IV shot writes strengths, not looseness scales, and taking the lowest undid the balancer's leg stiffness.
//!
//! Priorities: RDR2's behaviours open their requests at a priority their constructor sets, numbered in the order of
//! their behaviour slot (balancer 0x18 first, then bodyBalance 0x20, braceForImpact 0x28, shot 0x38 ... keyframe
//! 0x120); the core's own requests come after all of them. That the queue runs in that order (lowest number first)
//! is our reading: the code that sorts it hasn't been found. Here a priority is the RDR2 slot.

use crate::nmrs::Effector;

/// The channels of one joint, in RDR2's per-joint record order.
const CHANNELS: usize = 7;

fn channels(e: &Effector) -> [f32; CHANNELS] {
    [e.desired[0], e.desired[1], e.desired[2], e.oppose_gravity, e.muscle_stiffness, e.strength, e.damping]
}

fn set(e: &mut Effector, k: usize, v: f32) {
    match k {
        0..=2 => e.desired[k] = v,
        3 => e.oppose_gravity = v,
        4 => e.muscle_stiffness = v,
        5 => e.strength = v,
        _ => e.damping = v,
    }
}

/// The shot's slot.
pub const SHOT: u32 = 0x38;

/// The core's own requests (setStiffness and the like, sent by the game) come after every behaviour.
pub const CORE: u32 = 0x138;

/// RDR2's behaviour slot for a message an extra handles (the core's for those with none).
pub fn slot(message: &str) -> u32 {
    match message {
        "leanRandom" | "leanInDirection" | "leanToPosition" | "leanTowardsObject" | "hipsLeanRandom" | "hipsLeanInDirection"
        | "hipsLeanToPosition" | "hipsLeanTowardsObject" | "forceLeanRandom" | "forceLeanInDirection" | "forceLeanToPosition"
        | "forceLeanTowardsObject" => 0x18,
        "rdr2Balance" => 0x20,
        "shotSnap" | "shotSpin" | "shotFromBehind" | "shotInGuts" | "shotInjuredLeg" | "shotInjuredArm" | "shotFallToKnees" => 0x38,
        "staggerFall" => 0x48,
        "meleeReact" => 0x40,
        "armsWindmill" => 0x58,
        "handsAndKnees" => 0x70,
        "injuredOnGround" => 0x78,
        "rdr2Lasso" | "rdr2Bolas" => 0x88,
        "reachForWound" => 0xa8,
        "duck" => 0xb0,
        "rdr2Sit" => 0x108,
        "smartFall" => 0x110,
        "wallCollision" => 0x118,
        "keyframe" => 0x120,
        _ => CORE,
    }
}

#[derive(Clone, Debug, Default)]
pub struct Requests {
    last: Vec<[f32; CHANNELS]>,
    /// (priority, order written, joint, channel, value)
    list: Vec<(u32, u32, usize, usize, f32)>,
    n: u32,
}

impl Requests {
    pub fn begin(eff: &[Effector]) -> Requests {
        Requests { last: eff.iter().map(channels).collect(), list: vec![], n: 0 }
    }

    /// What changed since the last mark is a request at this priority.
    pub fn mark(&mut self, eff: &[Effector], priority: u32) {
        self.mark_some(eff, priority, None, None);
    }

    /// As `mark`, for these joints only (the rest stay for the next mark).
    pub fn mark_joints(&mut self, eff: &[Effector], priority: u32, only: &[usize]) {
        self.mark_some(eff, priority, Some(only), None);
    }

    fn mark_some(&mut self, eff: &[Effector], priority: u32, only: Option<&[usize]>, channel: Option<usize>) {
        for (j, e) in eff.iter().enumerate() {
            if only.is_some_and(|o| !o.contains(&j)) {
                continue;
            }
            let now = channels(e);
            let Some(was) = self.last.get_mut(j) else { continue };
            for k in (0..CHANNELS).filter(|k| channel.is_none_or(|c| c == *k)) {
                if now[k].to_bits() != was[k].to_bits() {
                    self.list.push((priority, self.n, j, k, now[k]));
                    self.n += 1;
                    was[k] = now[k];
                }
            }
        }
    }

    /// RDR2's resolution: per channel, the requests in priority order, each filling what is left; every request here
    /// has weight 1, so the highest-priority one fills it. Within one priority the latest write counts, as it does
    /// within one behaviour.
    /// `reversed`: the highest slot first instead (the other reading of the queue order, for comparison).
    pub fn resolve(mut self, eff: &mut [Effector], reversed: bool) {
        if reversed {
            self.list.sort_by_key(|r| (std::cmp::Reverse(r.0), std::cmp::Reverse(r.1)));
        } else {
            self.list.sort_by_key(|r| (r.0, std::cmp::Reverse(r.1)));
        }
        let mut done = vec![[false; CHANNELS]; eff.len()];
        for &(_, _, j, k, v) in &self.list {
            if !done[j][k] {
                done[j][k] = true;
                set(&mut eff[j], k, v);
            }
        }
    }
}
