# Red Dead Redemption 2 reactions: study notes

What we learned about how Red Dead Redemption 2 (RDR2) makes a character react when it is shot, falls, gets bumped
or is hit in a fight, what the game showed when we recorded it, and how Delirium's RDR2 mode does the same. The
notes are written so another game using Delirium can get the same feel.

**What's in here and what isn't.** These are notes in our own words. There is no RDR2 code here, no RDR2 data and
none of its tuning values. The numbers a reaction uses live in the player's own `physicstasks` file (in the game's
`data/tune`, exported as XML from your own copy), and Delirium reads them at run time. Times and distances quoted
below are our own measurements from recordings of the game, not values copied from it.

How sure each point is:

| Tag | Meaning |
|---|---|
| **[read]** | Read in how the game works. |
| **[seen]** | Seen in a recording of the game. |
| **[partly]** | The shape is known, a detail is inferred. |
| **[guess]** | A guess that still needs checking. |

---

## How RDR2 builds a reaction

**Two layers.** RDR2's Euphoria only runs *behaviours*: bodyBalance, shot, smartFall, catchFall, rollDownStairs and
so on. What starts them, with which settings and when, is a *game task*: CTaskNMShot, CTaskNMHighFall, CTaskNMBalance,
CTaskNMMelee and others. A task does five things. **[read]**

1. **Picks message sets by situation.** It looks at the part hit, the weapon, stairs, being on a horse, dying and the
   attacker's move. The sets are named in `physicstasks`.
2. **Merges the sets by priority.** It goes from the lowest priority to the highest, and later values win. Then it
   sends each merged message once.
3. **Edits a few messages with live values** before sending them, such as the hit point, the push or the look target.
4. **Keeps working every frame.** It sends an update set and a few live messages, and answers the behaviours'
   feedback (for example "balance failed") with event sets.
5. **Sometimes pushes the body itself.** Some tasks apply their own forces, outside Euphoria.

So feeling like Red Dead needs both layers: the behaviours (Delirium's RDR2 core) and the task logic described below.

**Mods change the sets.** All our recordings so far were made with the *Western styles* mod installed. It replaces
the sets for shots, high falls, bumps and melee with more dramatic, softer ones. Stock RDR2 and modded RDR2 feel
different, so compare against recordings made with the same `physicstasks` file. **[seen]**

## How the physics is stepped while standing

* The game runs Euphoria at the game's frame rate, not a fixed rate. **[read]**
* While a character is standing on its balancer, any frame of 1/60 s or longer is run as **two half steps**.
  This lasts from the hit until the standing phase ends. Once the character falls, it goes back to one step per frame.
  **[read, seen]**
* In 405 recorded frames, every frame under 16 ms ran one step and every frame over about 17.7 ms ran two equal
  steps. None ran three. **[seen]**
* At the 50-80 fps we recorded, that makes a standing step of about 10-12.5 ms.

This decides how fast the feet leave the ground in a stagger. Delirium does the same in RDR2 mode (`rdr2_split` in
`crates/delirium/src/nm.rs`). Against the recordings it brought the stepping foot's timing in line with the game:

| Leg shot | RDR2 | Before | After |
|---|---|---|---|
| Stepping foot's first contact gone | 0.070 s | 0.122 s | 0.076 s |
| Stepping foot clear of the ground | 0.126 s | 0.206 s | 0.126 s |
| Standing foot clear of the ground | 0.389 s | 0.383 s | 0.404 s |

The figures are averages over 3 game shots and 6 Delirium runs.

## Foot contacts

These rules set the footwork timing of a stagger, together with the step rule above. **[read, seen]**

* A contact's age resets every step it is still found.
* Once a contact stops being found and gets old enough, it is dropped, at most one per foot per step.
* A contact that has **slid** goes first. Otherwise the one furthest off goes.
* On a stepping foot the contacts went one by one: by the slide test first (2.8-25 cm of slide), then by distance. **[seen]**

---

## Reactions by situation

### Shot (CTaskNMShot)

**The sets for each bullet [read]**, merged:

* **Always:** a base set, a set for the character type, and situation sets (on a horse, seated, on stairs,
  armoured, dying).
* **The weapon:** the weapon's own set.
* **Where it hit:** head, neck, low trunk from the front, arm, hand or leg. The leg set always sends him into the stagger.
* **What he holds:** holding nothing → the unarmed sets and the stagger path; holding a gun → the one- or
  two-handed sets.
* **The shot itself:** a second bullet soon after has its own set; a shooter close by uses close-range sets.

**Then, in code:**

* Body stiffness is worked out from health and speed.
* The bullet's push is applied, with a share to the trunk.
* The shot's extra parts (snap, gut shot, shot from behind, spin, fall to knees, injured leg or arm) run only when
  their own switch is on.

**Later events:**

* Going over a wall uses the wall sets.
* Losing balance uses the bleeding-out sets.
* A wall behind him uses the "shot against wall" set.

**In the game [seen]** (Cattleman revolver, a standing townsman):

| Shot | What runs | How it plays out |
|---|---|---|
| Chest | Shot, bodyBalance with its head look, balancerCollisionsReaction | He steps back and stays up. His centre moves about 0.7 m in 1 s. |
| Back | The same, from behind | About 0.8 m in 1 s, 0.9 m in 2 s. |
| Leg | Shot, staggerFall, reachForWound | The leg gives and he hops and staggers for 1-3 s, then goes down. As he falls, smartFall, bodyFoetal, pedalLegs, armsWindmillAdaptive, catchFall and bodyRollUp take over. About 1 m at 1 s, 2.5 m at 2 s, 5 m at 3 s. |

### Pushed or bumped (CTaskNMBalance)

**Start [read]:**

* The push type's set: default, weak, aggressive, or falling off a moving car.
* Sets for moving ground, stairs or a steep slope, found by probing under him.
* A "bumped by a ped" set when the bumper is a character.
* He looks at the bumper.
* **The bump also counts as a light bullet** on the lower spine, pointing away from the bumper.

**Every frame [read]:**

* **Contact sets:** while touched, a "being pushed" set. After a short gap with no contact, a "not being pushed"
  set. Pushed on the ground for too long, a set of its own.
* **Staying upright:** the force that keeps him upright **weakens as he speeds up**.
* **Bracing:** with the bumper close and in front, the arm on that side braces and he looks at the bumper.
* **A short push:** the task gives its own small push for a moment after contact.
* **Falling:** at an edge he teeters. Moving fast on the ground uses a rolling-fall set; slower, a little later, a
  catch-fall set.

**Losing balance [read]:** with stairs under his chest he goes straight into **rollDownStairs**. Otherwise the
"balance failed" set.

**The player bumping an NPC [seen].** We recorded 11 bumps out of 11. The NPC's task had **no bumper**: no "bumped
by a ped" set, no brace and no look at the player. It only used the default start set, which runs bodyBalance,
balancerCollisionsReaction and bodyBalance's own head look. Many went down 0.5-1.4 s later into roll-up and catchFall.

### Hit in melee (CTaskNMMelee)

**Sets, in this order [read]:**

1. Start.
2. Passive (the victim doesn't fight back).
3. Armoured.
4. Bound ankles.
5. Dying.
6. Player death.
7. On stairs.
8. Seated or on a horse.
9. The **weapon's** set.
10. The **attacker's move's** set (for example a standing kick, a bear's swipe, a hammer, a knife). The mod adds bat moves.

**The blow [read]:**

* **The push** comes from the weapon. If he moves into the blow, a **counter push** at the pelvis is taken off it.
* **The bullet-push message's "impulse" holds three scales, not a direction.** One is for the whole blow, the others
  for groups of parts. Stock sets double the blow.
* **meleeReact** always gets a fresh hit on the part struck, and a light bullet is always sent too.
* **Finishers** add timed pushes across several parts along the blow: the kick, the bear's swipe (with an upward
  lift), and a blow from a rider, scaled and turned by the horse's speed.

**Every frame:** the update set. He flinches and looks at the attacker. For a passive victim, a set's lean stops
after a random time from the file. **[read]**

**Losing balance:** the "balance failed" set, plus its stairs version when on stairs. **[read]**

**In the game [seen]** (fist fights, Western styles):

* Punches stagger him on bodyBalance and balancerCollisionsReaction.
* **Knock-downs start shot, staggerFall and reachForWound, and go straight into smartFall** (curl up, pedal) within
  0.05 s.

### Flinch (CTaskNMFlinch)

Used for light hits and the water cannon. **[read]**

* **Sets:** start, water cannon, armoured, bound ankles, dying, player death, on stairs, holding a one- or two-handed
  weapon, and the weapon's set.
* **The hit:** a push on the hit part along the hit, then a light bullet.
* **Every frame:** he looks at the source and flinches from it.

Studied but **not ported yet**.

### High fall (CTaskNMHighFall)

**When the game starts it [seen].** A townsman dropped from 4, 8, 15 and 30 m, twice each:

* **4 and 8 m:** no ragdoll at all. The fall and the landing stay animated.
* **15 and 30 m:** Euphoria takes over when his centre is about **13.5 m above the ground**, at the same height for
  both drops.
* **Big landings:** all four killed him, whatever his health.

**Sets [read]:**

* **Start:** the start set plus the fall type's set (a plain fall, a vault, hit by a car, sliding down a slope, off
  an edge, exhausted, a stunt jump or a jump collision).
* **Every frame:** the update set.
* **Height bands:** with no ground close below and falling fast, a "high-high fall" set: arms windmilling overhead,
  legs spread. Falling faster still, a "super high fall" set. Leaving the bands, an end set.
* **The behaviour:** it runs **smartFall**, not the older highFall behaviour. **[read, seen]**

**The pitch push [read, seen].** For the first part of a second, the task pushes the **upper chest** along the body's
sideways (horizontal) motion. The push is stronger with speed and when he is upright. Any small drift is enough to
pick the direction. This tips him over **pelvis first, onto his front (arms waving) or onto his back (curling up)**,
nearly level by the time he hits the ground.

* **Where it pushes:** we placed the push on the top spine part from the recordings. The fall's spin grew by that
  part's height above the centre of mass.
* **Why that matters:** pushed at the pelvis, the body hardly turned.

**smartFall's states:** in the air (windmill and pedal, look down), brace or crash before the impact (curl up), land
on his feet, **hard landing (catchFall)**, forward roll, settle, and teeter at an edge.

### Down the stairs

* **Starting the roll:** losing balance with stairs under the chest starts **rollDownStairs** at once (bump task) or
  a stairs failure set (melee). While standing on stairs, the shot and melee tasks add their stairs sets.
* **While rolling:** rollDownStairs keeps the roll going and the arms slow it down. smartFall's roll and
  hard-landing states and catchFall's friction help. It stops when he is slow, then catchFall or roll-up.
* **The mod's set:** its rolling-fall set stops everything, then starts a falling reaction, rollDownStairs and
  pedalLegs.

Not recorded in the game yet.

### On the ground

| Behaviour | What it does |
|---|---|
| catchFall | Arms out to catch the ground, then the lying stiffness from the task's sets. |
| bodyRollUp | Curls and rolls while still moving. |
| bodyFoetal | Curls up while tumbling. |
| pedalLegs | Pedals the legs in the air. |
| bodyWrithe, injuredOnGround | Wounded on the ground, holding the wound. |

**Getting up** is the game's animation, not Euphoria.

### Teetering and walls

* **teeter:** near an edge the balancer leans away and the arms windmill. Past the edge, it's a fall.
* **balancerCollisionsReaction:** with a wall behind or ahead he braces, slumps down the wall, or tips over a low
  wall (**fallOverWall**).

### Dying

* **Sets:** shot, melee and flinch each have a dying set, and there are player-death sets.
* **Then:** the body goes limp.

### On fire

* **The behaviour is ported:** onFire, with flailing, patting, falling and rolling.
* **The task isn't:** which sets RDR2 sends when a character catches fire hasn't been studied yet.

### Player compared with NPCs

What's known so far:

* When the player bumps an NPC, the NPC's task gets no bumper. **[seen]**
* Push detection counts a toucher with the player flag. **[partly]**
* Melee and flinch have player-death sets. **[read]**

Arthur himself ragdolling hasn't been recorded yet, so don't assume the player reacts exactly like an NPC.

---

## What Delirium's RDR2 mode does today

On branch `rdr2-core-split`.

**Behaviours (the RDR2 core):**

| Group | Behaviours |
|---|---|
| Balance and body | bodyBalance, headLook, staggerFall, StayUpright |
| Shot | shot and its parts (snap, spin, fall to knees, gut, from behind, injured leg and arm), reachForWound |
| Falling and the ground | smartFall, catchFall, bodyRollUp, rollDownStairs, bodyWrithe, injuredOnGround, pedalLegs, bodyFoetal, armsWindmill (and its adaptive version) |
| Edges and walls | teeter, highFall, fallOverWall, balancerCollisionsReaction |
| Hits | upperBodyFlinch, meleeReact |
| Others | braceForImpact, controlledFall, onFire, buoyancy, spineTwist, animPose, pointGun, yanked, swim, keyframe, sit (riding) |
| Horse | the quadruped behaviour (in part) |

**Tasks, reading the sets from the player's `physicstasks`** (the playroom's `crates/delirium-room/src/gametask.rs`
reads, merges and selects them):

| Situation | What's done |
|---|---|
| Shots | Done. |
| High falls | Start and in-air sets, the height bands and the pitch push. |
| Bumps | The Hero running into someone. |
| Melee | Punches and kicks, with the passive, armoured, mounted and stairs sets and their rules. |
| Physics steps | The two-half-step rule while standing. |

**Still Delirium's or GTA IV's own:** flinch, the fire task, explosions, being hit by a wagon or horse, the lasso
task, getting up.

**Open gaps:**

* The height where a high fall hands over to Euphoria. The playroom goes by falling speed instead.
* smartFall's friction after landing.
* The weapon push's exact law in melee.
* One group of parts in melee's push.
* The seated, bound-ankles, player-death, steep-slope and moving-ground sets.

## Not studied yet

The sets for these exist in the file, but how the game uses them hasn't been studied:

* the fire task
* explosions
* hit by a wagon or horse (brace)
* lasso and bolas
* electrocution
* drunk
* water and river rapids
* injured on the ground
* through a windscreen
* jumping or rolling off a moving vehicle
* trapped

Each one follows the same recipe: study the task, record it in the game, port the task layer.

---

## Recipe: RDR2-like reactions in another game with Delirium

1. **Body and core.** Use the RDR2 body (`nm_character_from_rdr2` from your own fred.xml or wilma.xml) and the RDR2
   core. In the playroom that's the `rdr2 core` button. The SDK doesn't have a switch for it yet.
2. **Step rule.** While the ragdoll stands, run a frame of 1/60 s or more as two half steps. Once it falls, run
   whole frames.
3. **The player's file.** Read `physicstasks` (XML) at run time. Never ship RDR2's values. `gametask.rs` has the
   reader, the merge and the set picker.
4. **Each situation.** Do what its section above says:
   * Choose the sets, merge them, and fill in the live messages.
   * Send each merged message. Use `nm_msg_send_text` per merged line, or `nm_msg_begin` / `nm_msg_set_*` /
     `nm_msg_send`.
   * Apply the task's own pushes with `nm_agent_apply_impulse`.
   * Run the every-frame part and the feedback events.
5. **Check against the game.** Record the same situation in RDR2 and compare the posture and which behaviours run,
   not one exact run. Falls only need to land close, not step for step.

## How these notes were made

* **The game's own logic.** Each game task and behaviour was studied from the user's own copy of the game.
* **Recordings.** A logging plugin wrote what Euphoria was doing in the game, step by step: which behaviours ran,
  their settings, every body part's position, foot contacts and step times.
* **Scripted scenes.** The logger set up shots, drops from set heights, and free play for bumps and fights.
* **Comparison.** Delirium ran the same scenes headless, and the two were compared.

Neither the logger nor the game data is published.
