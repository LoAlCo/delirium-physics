# rdr2nmx: a Red Dead Redemption 2 Euphoria logger

An ASI mod for Red Dead Redemption 2 (story mode) that records what the game's Euphoria (NaturalMotion) ragdolls are
doing, step by step, to files you can study offline. It's what the recordings in
[docs/rdr2-study-notes.md](../../docs/rdr2-study-notes.md) were made with. You can use it to compare Delirium's RDR2
mode against the game.

## What it records

* **Every ragdoll the game hands to Euphoria:**
  * its body (the rig) once;
  * then each step: every body part's position and motion, the joints, which behaviours are running and their
    settings, and the limb requests.
* **During the shot scenarios only** (four small hooks, removed afterwards): each step's foot and body contacts with
  the ground, which contact the game drops and why, the legs' muscles, and the physics step times.

Nothing is sent anywhere. Files go to `<RDR2 folder>\rdr2nmx\<time>_<name>.nmx`, with a text log `rdr2nmx.log`.

## Safety

* **Story mode only.** It does nothing in Red Dead Online. Don't use it there.
* **Locked to one game build.** It checks the game's code against the build it was written for before reading
  anything. On any other build it does nothing, and with the hooks it installs none.
* **Read-only, apart from the test scenarios.** The scenarios spawn a ped, shoot or push it, then delete it.
* **To turn it off:**
  * delete `rdr2nmx.asi`;
  * or create an empty file `<RDR2 folder>\rdr2nmx\DISABLE`;
  * or create `<RDR2 folder>\rdr2nmx\NOHOOKS` to keep the logging but never install the hooks.

## Install

1. Install [Script Hook RDR2](http://www.dev-c.com/rdr2/scripthookrdr2/) and an ASI loader, as for other RDR2 script
   mods.
2. Put `rdr2nmx.asi` in the RDR2 folder.

Get `rdr2nmx.asi` from this fork's releases, or build it (below).

**Other ragdoll mods change what you'd record.** For example: Ped Damage Overhaul, Simple Ragdoll, Enhanced Brawling,
Dynamic Shove & Throw, Drag Bodies. `session.ps1` moves them aside while you log and puts them back afterwards (close
the game first):

```powershell
.\session.ps1 start -game "D:\RedDeadRedemption2"
```

```powershell
.\session.ps1 end -game "D:\RedDeadRedemption2"
```

Add your own mods with `-mods`. A mod that replaces `physicstasks` (such as Western styles) also changes the
reactions; note which one you had.

## Keys (in game)

| Key | What it does |
|---|---|
| F9 | Run the selected scenario |
| Ctrl+F9 | Next scenario |
| F10 | Stop |
| F11 | Free logging on/off: records whatever ragdolls while you play (your own bumps, fights, falls) |
| F12 | Write every current ragdoll's body and settings now |

## Scenarios

They run in front of the player.

| Name | What it does |
|---|---|
| `capture` (default) | Leg and chest shots, three each, alternating, each to its own file, with the hooks. |
| `falls` | A townsman dropped from 4, 8, 15 and 30 m, twice each. |
| `pushes` | Bumps and fist fights between townsmen. In our tests they didn't ragdoll, so use F11 and do it yourself instead. |
| `real_shot` | A clone of the player, one chest shot (60 of 400 health) from 3 m. |
| `shot_back` | The same shot, from behind. |
| `shot_leg` | The same shot, at the left thigh. |
| `shot_ragdoll` | The chest shot, ragdolled a frame before it. |
| `push` | A clone ragdolled and pushed at the chest. |
| `stand` | A clone standing, 5 s. |

## Reading the files

`nmxdump.py` summarises a recording: the records, the ragdolls, and the behaviours running each step. With hooked
recordings it also decodes the contacts and the step times.

```bash
python -I nmxdump.py --steps file.nmx
```

`--steps` prints every step; `--calls` prints every contact refresh.

The format is in `src/nmxfile.h`.

## Build

* **You need:** the Visual Studio 2022 Build Tools (C++), and the
  [Script Hook RDR 2 SDK](http://www.dev-c.com/rdr2/scripthookrdr2/) unpacked into `sdk\` here, so that `sdk\inc` and
  `sdk\lib` exist. The SDK isn't included because its licence doesn't allow redistribution.
* **Build:** run `build.bat`. The ASI lands in `out\rdr2nmx.asi`.

## What isn't here

No game files and no game code:

* **The build check** stores only hashes of the game's code.
* **The addresses** are those of the one build it supports.
