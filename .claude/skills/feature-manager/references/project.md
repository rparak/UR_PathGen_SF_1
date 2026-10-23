# feature-manager: project reference for UR_PathGen_SF_1

This project's part of the change procedure. The generic skill is personal
(`~/.claude/skills/feature-manager/`); this file overrides it where they differ. This folder has no
`SKILL.md` on purpose: a personal skill wins over a project skill of the same name (Claude Code docs,
*Resolve skills that share a name*), so a copy here would be ignored and drift. Style rules come from
`project-style-rpx` and its project reference (`.claude/skills/project-style-rpx/references/`,
including `urscript.md` and `mjcf.md`).

## What the output is

Collision-free UR10e + Robotiq 2F-85 joint-space paths for workcell SF1, verified in MuJoCo: a
`*.urscript` points file and a `_Sequence.script` motion sequence that the owner loads into the
robot program (SPEC §1, §10).

## Physical system and safety

The output drives a real UR10e. **Simulation passed ≠ safe on the robot.** Every output not yet
tested on the real cell by the owner is a proposal that is run there at reduced speed first, with
someone at the stop button. No change, report or commit message claims more. "Simulation passed ≠
safe on the robot" appears wherever a result is presented. Language: English (CLAUDE.md); the
owner's notes may be in Czech or Slovak.

## Always "large" (ask before building)

- Formats: `*.urscript`, the motion sequence `*.script`, the collision set or its manifest,
  `Configuration/Cell_SF1.yaml`, a job file.
- A change to a clearance, a margin or a self-collision exclusion.
- Typical gaps to ask about: which points (A, B, a new learned point), the unit and frame of a
  value, a configuration field versus a code constant, the margin that applies, what happens when
  the planner cannot meet the limits.

## Real data for reproducing a premise

The learned points in `Data/Programs/`, the shipped configuration, the committed collision set
(`CAD/Collision/`). Walk the chain: the parsed tokens, DH FK · TCP against `_p`, the compiled MuJoCo
model (bodies, geoms, exclusions, margins), the pair and distance that `Check_Clearance` reports, the
collision piece involved and its excess.

## Premise example

(Hypothetical) "the robot hits the table at B" could be a wrong base yaw (the TCP lands elsewhere in
the world; FK vs `_p` cannot see it), a collision hull that over-fills an open shell (excess, not
contact), a missing pad-span hull, or a real collision. Each needs a different fix; a larger margin
would hide all four.

## Typical better levers

Tightening a hull or adding a pair-specific margin instead of lowering a global clearance; reporting
a near pair instead of excluding it; failing explicitly instead of returning a sixth via point.
Results that could look right and be wrong: a clearance shown to 0.01 mm when the kinematic error is
millimetres; a path checked against a model that differs from the robot.

## Domain checklist

The references are SPEC sections and the style project pages (`urscript.md`, `mjcf.md`).

**Kinematics and TCP**

- [ ] **One kinematic truth: the official UR10e DH** (SPEC §9.2). MuJoCo FK must equal DH FK; a
      Menagerie offset is never trusted as-is (mjcf.md).
- [ ] **The TCP is configured, not derived** (SPEC §9.3). `Derive_TCP_Offset` is a diagnostic; it never
      writes the configuration or feeds a check.
- [ ] **Regression pin vs sanity bound** (SPEC §9.4). The pin holds the measured FK residual in a narrow
      band to detect change; the bound (half of `Clearance.Environment`) decides acceptance at runtime.
      Neither is widened to let a change pass.
- [ ] Orientation compared by the atan2 geodesic angle, never component-wise rotvec (urscript.md).
- [ ] Frames named for every value (world = STL, Polyscope base, flange; SPEC §9.1).

**Collision and convex decomposition**

- [ ] **Conservativeness beats tightness** (SPEC §7.2, §7.3): the collision set contains the original
      surface within `eps`; a CoACD result that is not conservative falls back to the hull.
- [ ] **Excess** of a hull is measured and reported; an open shell whose hull fills space the robot
      needs is flagged, never silently accepted (SPEC §7.4).
- [ ] Degenerate components thickened, not dropped (SPEC §7.2).
- [ ] The collision set's STL hash matches; a new STL means a rebuild (mjcf.md).

**Clearance**

- [ ] **Margins and the budget** (SPEC §9.8): `Clearance.Environment` covers kinematic error, `eps` and
      cables. A change that consumes margin updates the budget, both cases.
- [ ] **Start or goal inside the margin is an explicit error** (`Clearance_Error_Cls`, naming the point,
      every pair, its distance and the margin). No silent pass, no silent failure, no override without
      an explicit decision (SPEC §9.7).
- [ ] Segment checks sample so that no robot point moves more than `Clearance.Environment / 4` between
      samples, from joint step × lever arm, not a fixed count (SPEC §9.7).
- [ ] Static bodies are never collided by MuJoCo; a clearance involving one is checked separately
      (mjcf.md).

**Motion output**

- [ ] **Blends: `r=0` only** until blend-envelope checking exists (SPEC §8.5). A blend request is a
      new feature with its own checking, not a parameter.
- [ ] Generated moves with joint targets only, `movej(<name>_q, a=<a>, v=<v>, r=0)`, never `movej(p[...])` or
      a generated `movel`; station lines are copied verbatim from the program (SPEC §10.4).
- [ ] **At most 5 via points, or an explicit failure** - never a longer or colliding path returned
      (SPEC §1, §14 risk 8).
- [ ] Learned points written from their tokens; the round trip stays byte-identical.

**Joint limits and branches**

- [ ] **No wrapping**: a generated segment stays on the branches of its two ends; a learned point
      takes another branch or turn only through its job's `Point_Configurations`, with a source
      (SPEC §8.6, §10.5).
- [ ] **Elbow ±180° is a precondition** of the `upper_arm_link–forearm_link` exclusion; a wider limit
      is rejected at load (SPEC §8, §9.6).
- [ ] Polyscope safety limits may be tighter than ±360° (`.claude/project-docs/KNOWN_ISSUES.md`).

**Self-collision**

- [ ] **Exclusions are explicit** and come from the configuration (plus `base–env_stand` in code);
      filterparent stays disabled; a near pair is reported, not excluded (mjcf.md).
- [ ] The gripper rigid group and the pad-span hull still describe the configured opening.

**Simulation vs reality**

- [ ] **Nominal DH, no `calibration.conf`** (SPEC §3, §9.4). The FK residual is real kinematic error,
      measured at the 78 learned points (at most 4.04 mm / 0.377°); at generated via points it is
      unmeasured.
- [ ] Not modelled: the carried part beyond the pad ends, cables, dynamics. A change that relies on
      any of them says so.

**Software**

- [ ] Heavy runtimes (CoACD, trimesh) stay out of what the tests import.

## Budgets

The clearance budget, SPEC §9.8: `Clearance.Environment` covers kinematic error, `eps` and cables.
Before and after numbers for any change to kinematics, geometry or checking: FK residuals, minimum
distances per pair, violation counts, piece counts and excess, runtimes - on the same inputs and
seed. Name what holds only at the learned points (not at generated via points).

## Visual changes

For a scene change, open it in the viewer (`mjpython` on macOS) and look at what changed; a passing
distance test does not show a misplaced body.

## Commits

- **Never commit a regenerated `models/` file.**
- Commit a rebuilt `CAD/Collision/` only as its own deliberate change with its manifest and report.

## Docs

- Configuration: update the SPEC §8 (or §8.6) table in the same change.
- README: the triggers are in `.claude/skills/readme-writer/references/project.md` > Triggers.
