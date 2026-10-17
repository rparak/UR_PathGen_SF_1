# UR_PathGen_SF_1 — Specification

Status: **rev. 4, approved by the owner (2026-10-17)** with the amendments listed under "Approval"
below, which are applied during implementation.
Numbers marked *measured* come from scratch-folder experiments run on 2026-10-17 against the files
named; they are re-measured by the tests once the skeleton exists.

Approval (2026-10-17), applied during implementation:
- **Q9 → (a).** The `base – upper_arm_link` self margin is 0 mm, for that pair only. The test must
  establish the true minimum, not a grid minimum: a 1° grid moves the upper-arm end by ≈ 10 mm,
  more than the 2.3 mm found. It runs a coarse grid over `shoulder_pan` ∈ [0, 2π) (periodic) ×
  `shoulder_lift` within its limits, then local minimisation (scipy) from the 20 lowest cells.
  It asserts the refined minimum > 0 and pins it as a regression value.
- Degenerate components (§7.2) are thickened along the plane normal from the SVD of their vertices.
  A mean face normal is not used, because the STL winding is inconsistent and normals can cancel.
  Collinear components are thickened in two orthogonal directions. Both cases are tested.
- §4.5 quotes the exact MuJoCo error for the > 200 000-face STL.
- The time of one `Check_Clearance(q)` with the full environment collision set (mean and p95 over
  1000 random `q`) and the collision geom count are measured. They decide whether the §7.2 merge
  pass is needed.
- If A or B violates the margin against the full environment set: **stop and report the pairs**.
  `Hull_Excess_Max`, `eps` and the margins are never tuned to make it pass.

Rev. 4 (owner review of rev. 3):
- Elbow limit ±180° is a load-checked precondition (§8, §9.6).
- Clearance failures at A/B fail the test (§11).
- Conservativeness uses surface samples (§7.3).
- Prototype measurements were added: pair distances, margin accuracy, compile time (§4.5).
- Option (ii) pad-span hull is adopted (§9.6).
- The geodesic angle uses atan2 (§9.5).
- Static bodies (§9.7), degenerate STL components (§7.2) and loading the visual mesh (§5) are
  specified.

Rev. 3 (owner review): the TCP is configured, not derived (§9.3). The coupling was measured on the
robot (§4.4). The sanity bound and the clearance budget are made consistent (§9.4, §9.8).
`Mount_Yaw` = 0 is a documented assumption (§9.3). MuJoCo vs DH FK is checked at 1e-9 over random
joint vectors (§11). The demo is either clearance-checked or labelled visual-only (§2). Gripper
coverage of the carried part is stated as it actually is (§9.6).

## 1. Goal

Generate and verify, in simulation, a collision-free joint-space path for the UR10e of cell SF1
before it is run on the real robot.

Long-term pipeline:

1. Read learned robot positions from a Polyscope `*.urscript` variable file
   (`global <name>_p=p[...]`, `global <name>_q=[...]`).
2. Plan a collision-free path from point A to point B with **at most 5 via points**.
3. Write the via points in the same format, plus a `movej(...)` sequence, ready to paste into the
   robot program.

**Simulation passed ≠ safe on the robot.** Every output is a proposal that is run on the real cell
at reduced speed first. The tool never claims more than that.

## 2. First milestone (this skeleton) — in scope

1. MuJoCo scene: the SF1 environment from `CAD/SF1_Environment.stl` (visual: the original mesh;
   collision: a committed convex decomposition), the UR10e with the Robotiq 2F-85, base at
   world (0, 0, 0.7152) m.
2. A demo that rotates one joint (base, `shoulder_pan`, ±45° around `home_safe`) as a kinematic
   replay and shows it in the MuJoCo viewer. Before showing anything, the demo checks the whole
   motion with the segment check of §9.7 and prints the result (samples, minimum distance, every
   violating pair).
   - If the check passes, the demo runs labelled **"clearance checked"**.
   - If it fails, the demo runs labelled **"VISUAL ONLY — UNCHECKED"**: an `[WARNING]` line and the
     viewer overlay say so, and the violations are listed.

   The amplitude is never reduced to make the check pass. Which of the two labels the shipped motion
   gets is reported at implementation and pinned by a test.
3. A URScript parser and writer that read the test positions exactly and write them back
   byte-identically.
4. The kinematic checks: DH FK · configured TCP against `_p`, MuJoCo FK against DH FK, the
   derived TCP as a diagnostic, the coupling in the scene, the clearance check of A and B.
5. Installer, tests, style checker, the three project skills, docs.

## 3. Out of scope for the milestone

- The planner (RRT-Connect + shortcutting, ≤ 5 via points) — the first `/feature-manager` task.
- Modelling the carried part (accepted risk, `KNOWN_ISSUES.md`).
- Blend radius > 0 and blend-envelope checking. The writer always emits `r=0` (§8.5).
- Dynamics, payload, cycle time. The simulation is kinematic only.
- Windows. Ubuntu is a target but cannot be run on this machine: it stays **unverified**.
- Robot-specific calibration (`calibration.conf`): nominal DH only (§9.4).

## 4. Measured facts this specification rests on

### 4.1 `CAD/SF1_Environment.stl` (*measured*)

| Quantity | Value |
|---|---|
| Format, size | binary STL, 87 897 484 B |
| Triangles | 1 757 948 |
| Vertices | 5 273 844 raw, 891 504 after merging duplicates |
| Bounding box [m] | x [−1.609, 0.934], y [−1.413, 1.045], z [−0.0016, 2.023] → metres confirmed |
| Watertight | no (as a whole); winding not consistent |
| Connected components | 2 756 (1 620 closed, 1 136 open shells) |
| Objects (bbox-touching clusters) | (a) machine/rack behind the robot, x [−1.61, −0.47], up to 2.02 m, 1.15 M triangles; (b) robot stand + side table, x [−0.34, 0.93], y [−1.41, 0.51], z ≤ 0.781; (c) cabinet, x [−1.0, −0.4], y [0.56, 1.05], up to 1.48 m |
| Robot in STL | no — no vertex within r < 0.3 m of the base axis above z = 0.722 |
| Under the base | two aluminium profiles (x ∈ ±[0.0375, 0.1275]) with top face at z = 0.715 → **0.2 mm gap** to the base at 0.7152; no mounting plate |
| Side table | top plate z 0.715–0.721 at x [0.34, 0.93], y [−0.455, 0.455]; parts and a fixture on it up to z 0.781 |

### 4.2 Menagerie models (*measured*, commit `62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35`, 2026-10-14)

- `universal_robots_ur10e`: BSD-3-Clause (Copyright 2018 ROS Industrial Consortium).
  `robotiq_2f85`: BSD-2-Clause (Copyright 2013 ROS-Industrial). Both allow vendoring with the
  licence file kept.
- The MJCF root body `base` carries `quat="0 0 0 -1"` (180° about z). With it, the MJCF world frame
  **is** the Polyscope base frame: MuJoCo and DH flange positions agree up to the d6 offset below,
  with no extra rotation. The scene therefore places the model unrotated at the configured base pose; the
  test of §11 pins this.
- **Menagerie kinematics differ from the official UR10e DH**: body offsets are rounded (0.181 vs
  0.1807, 0.613 vs 0.6127, 0.571 vs 0.57155, 0.12 vs 0.11985 …) and `attachment_site` sits at
  0.100 m along the wrist-3 axis instead of d6 = 0.11655 m. The visual mesh `wrist3.obj` ends at
  0.1168 m, so the mesh is right and the **site is 16.5 mm inside the wrist**. Measured flange
  difference MuJoCo vs DH: 16.9 mm (home), 16.6 mm (drop_rotate_1).
- Joint ranges in the MJCF: ±2π, except the elbow ±π.
- Robot collision geometry in the MJCF is hand-placed capsules, which do not necessarily contain
  the visual mesh.
- 2F-85: the inner silicone-face gap is 85.16 mm at control 0 and 0.88 mm at 255. This matches the
  85 mm stroke and validates the measurement method. 50.000 mm is reached at driver angle 0.3509 rad.
  The Menagerie root `base_mount` is a Ø75 × 13.9 mm disc, i.e. a coupling-like part (§4.4).
- Rev. 1 of this file claimed "pad tip ≈ 161 mm, 11–21 mm gap to the TCP". That was a measurement
  error: it used the pad centre and a collision-box centre, not the end of the pad mesh. The
  corrected numbers are in §4.4.

### 4.3 FK vs `_p` with nominal DH (*measured*)

The test computes `FK_DH(q) · TCP_config` and compares it with `T(p)`. `TCP_config` is the
configured Polyscope TCP (0, 0, 174.0) mm, no rotation (§9.3). All values below are *measured* with
nominal DH:

| | home | drop_rotate_1 |
|---|---|---|
| position error, world frame [mm] | (−0.04, 1.11, −1.31), \|e\| = **1.72** | (0.14, 2.24, 3.34), \|e\| = **4.02** |
| same error in the flange frame [mm] | (1.11, −0.04, −1.31) | (−3.30, −0.23, −2.29) |
| orientation error (geodesic) | **0.261°** | **0.376°** |

**These deviations are kinematic error.** The TCP is known exactly (configured on the robot, §9.3).
`_p` is computed by the controller from the same `q`, but with the robot's individual calibration,
while we use nominal DH because there is no `calibration.conf`. The deviation therefore measures
the difference between nominal and calibrated kinematics at these two poses. It is not a TCP error
and not noise.

Diagnostic only (never used as configuration): the TCP that would make `_p` match exactly at one
pose, `FK_DH(q)⁻¹ · T(p)`:

| | home | drop_rotate_1 |
|---|---|---|
| derived TCP in the flange frame | (1.11, −0.04, 172.69) mm, 0.27° | (−3.30, −0.23, 171.71) mm, 0.38° |
| derived on home, applied to drop_rotate_1 | — | 4.52 mm, 0.62° |

The derived TCP differs between the two poses by 4.52 mm, and that spread is the same kinematic
error. This is why the TCP is not derived.

### 4.4 Robotiq 2F-85: coupling, TCP, fingertip (sources in §16)

**From the manual:**

- Coupling `GRP-CPL-062` / `GRP-ES-CPL-062` (ISO 9409-1-50-4-M6, one M6 dowel pin): **16.9 mm**
  thick overall, Ø75, with a **2.9 mm** deep Ø71 F8 bore that receives the gripper's Ø71 h8
  spigot. The flange face to the gripper's datum face is therefore 16.9 − 2.9 = **14.0 mm**.
  **Measured on the SF1 robot by the owner (2026-10-17): 14.0 mm.** This confirms the drawing and
  closes the question.
- Gripper datum face to the finger end: **149.3 mm** open, **162.8 mm** closed. Pad length 38 mm.
- TCP of the 2F-85, "coupling included": **(0, 0, 174.0) mm**, Rx = Ry = Rz = 0. This is identical
  in the UR manual (rev. 2020/02/11, p. 127) and in the Cobots manual (rev. 2021/05/19, p. 82, whose
  revision note says the TCP table was updated). The manual does not say which physical point the
  TCP is. **The owner confirmed that the Polyscope TCP of SF1 is exactly this value.**
- Mounting: "Secure the gripper coupling on the robot wrist, and use the coupling dowel pin to align
  parts." Fig. 3-2 shows the dowel pin next to the coupling cable and the wrist connector. The angle
  between the finger opening direction and the dowel pin is **not stated numerically**.

**Comparison 1 — diagnostic TCP vs configured TCP:** the TCP derived from home,
(1.11, −0.04, 172.69) mm, vs the configured (0, 0, 174.0) mm differs by **1.72 mm** (Δz = −1.31 mm)
and **0.26°**. Derived from drop_rotate_1, the difference is 4.02 mm and 0.38°. These are the
kinematic errors of §4.3 seen from the other side.

**Comparison 2 — fingertip distances.** All values are along the flange z axis, measured from the
**DH flange face** (frame 6, §9.1), Menagerie gripper geometry, settled in simulation:

| Point | (a) Menagerie as is (`base_mount` 13.9 mm) | (b) manual coupling (16.9 mm, gripper datum at 14.0 mm) | manual chain (14.0 + gripper dim.) |
|---|---|---|---|
| pad-face centre, 50 mm opening | 151.39 | 154.39 | — (not given) |
| pad end, 50 mm opening | 170.23 | 173.23 | — (not given) |
| pad end, fully open | 160.33 | 163.33 | 163.3 |
| pad end, closed | 173.49 | 176.49 | 176.8 |

(b) = (a) + 3.0 mm. That is exactly the sourced thickness difference 16.9 − 13.9 (manual coupling
vs Menagerie disc), not a fitted value. Against the manual's dimension chain, (a) is off by 3.0 mm
(open) and 3.3 mm (closed); **(b) is off by 0.03 mm and 0.31 mm**. Measured from the gripper datum,
the Menagerie model therefore reproduces the manual's finger dimensions. What it lacks is the
3.0 mm of coupling thickness.

**Decision: (b).** It is the sourced geometry, and the measurement on the robot confirms it:
flange face → gripper body = 14.0 mm.

### 4.5 Prototype scene measurements (*measured*, scratch, 2026-10-17)

The prototype is built as §9 specifies:
- MjSpec, DH patch, hulls of the robot visual meshes, coupling cylinder, gripper at 50 mm with the
  pad-span hull;
- elbow ±180°, other joints ±360°;
- the stand: 839 STL components inside `Stand_Region`, one hull each.

Distances are `mj_geomDistance` between convex hulls, the minimum over the geoms of each body pair.

**Kinematics and gripper**

| Quantity | Result |
|---|---|
| MuJoCo flange vs DH flange, 1000 random `q` in ±2π | position max **1.3e-15 m**; rotation max 6.3e-8 rad with the arccos formula, see below |
| Arccos vs atan2 geodesic, 1000 rotations equal up to rounding | arccos **6.0e-8 rad**, atan2 **1.6e-16 rad** → the arccos form cannot resolve 1e-9 near 0; §9.5 now uses atan2 |
| Gripper opening / equality residual / pad end from the flange face | 50.000 mm / 4.1e-6 / **173.23 mm** (as §4.4 (b)) |
| Finger opening direction in the flange frame (`Mount_Yaw` = 0) | **flange ±x** |

**Pair distances over 1000 seeded random `q` within the limits, plus A and B.** 116 non-excluded
body pairs: robot↔robot and robot↔gripper with a 5 mm margin; robot (except `base`)↔`env_stand`
with a 20 mm margin. "Fraction < margin" counts samples inside the pair's margin.

| Pair | Margin | Min [mm] | p5 [mm] | Fraction < margin | Fraction < 0 | A [mm] | B [mm] |
|---|---|---|---|---|---|---|---|
| **base – upper_arm_link** | 5 | **2.3** | 2.4 | **0.175** | **0.000** | 20.6 | 20.6 |
| wrist_1_link – wrist_3_link (asked) | 5 | 24.3 | 24.4 | 0 | 0 | 24.4 | 24.6 |
| shoulder_link – env_stand (asked) | 20 | 99.2 | 99.2 | 0 | 0 | 99.2 | 99.2 |
| wrist_2_link – gripper base (asked) | 5 | 58.4 | 58.4 | 0 | 0 | 58.4 | 58.4 |
| wrist_3_link – gripper base (rigid to each other) | 5 | 9.6 | 9.6 | 0 | 0 | 9.6 | 9.6 |
| shoulder_link – wrist_1_link | 5 | 11.1 | 11.1 | 0 | 0 | — | — |
| forearm_link – wrist_2_link | 5 | −10.9 | 1.6 | 0.076 | 0.044 | 32.7 | 32.2 |
| upper_arm_link – env_stand | 20 | −223 | −181 | 0.431 | 0.416 | 108.6 | 104.2 |
| forearm_link – env_stand | 20 | −366 | −265 | 0.417 | 0.397 | ≥ 200 | ≥ 200 |

All other pairs fall into two groups:
- Their minimum stays above the margin.
- Or their minimum is deeply negative: genuine collisions of random poses that fold the arm onto
  itself or dive into the stand. For these, fraction < margin exceeds fraction < 0 only by the
  transition band around such collisions (e.g. forearm – wrist_2 7.6 % vs 4.4 %), not by a
  persistent near-contact.

The only other pair with a positive minimum inside its margin is base – wrist_2_link: min 2.3 mm,
in 0.1 % of samples. It is noted under Q9; at that rate the 5 mm margin stays. Sampling took 39 s.

- **Structural violation: one pair, `base – upper_arm_link`.** It is inside 5 mm in 17.5 % of
  samples, yet never below 2.3 mm. The hulls pass close but never touch. Excluding the pair would
  hide it, so it is **not** excluded. Proposal (Q9): a pair-specific self margin of 0 mm (contact
  only) for exactly this pair. A test would justify it with a dense sweep of `shoulder_pan` ×
  `shoulder_lift` showing the minimum stays > 0, since only these two joints move the upper arm
  relative to the base. Alternatives: keep 5 mm and forbid ≈ 17 % of joint space, or lower
  `Clearance.Self` globally.
- The stand and self-collision fractions above are not structural: they are random poses that
  really collide, the minima are deeply negative, and the A/B values are clear.
- At A and B every pair is outside its margin (smallest: base – upper_arm 20.6 mm, forearm –
  wrist_2 32.2 mm, wrist_3 – gripper base 9.6 mm, which is constant and rigid).

**Margin accuracy, mesh hull vs mesh hull** (environment margin 20 mm, robot margin 5 mm; the
robot geom on a free joint): four cases, namely a robot link hull (`forearm_3`) at three orientations
against the face of a box hull, and box hull edge against box hull edge.

| Gap | Contacts from `mj_collision` | Reported `dist` | `mj_geomDistance` error |
|---|---|---|---|
| 10 mm | yes, in all 4 cases | 10.000 mm | 0.000 mm |
| 30 mm | none, in all 4 cases | — | 0.000 mm |

The first attempt reported no contacts at all, because both test bodies were static. **MuJoCo never
collides two static (jointless, world-welded) bodies.** That matters for the scene (§9.7).

**Scene build**

| Item | Result |
|---|---|
| Visual STL through the MuJoCo file loader | **refused**: `ValueError: Error: stl_decoder: number of faces should be between 1 and 200000 in STL file '…/CAD/SF1_Environment.stl'; perhaps this is an ASCII file?` |
| Visual STL as `uservert`/`userface` from trimesh (891 504 vertices, 1 757 948 faces), `contype=conaffinity=0`, shell inertia | compiles; **no hull stored** (`mesh_graphadr = −1`) |
| Compile time with / without the visual mesh | **1.3 s** / 0.1 s |
| Reading, splitting and hulling the STL | 5.5 s |
| Stand components inside `Stand_Region` | 839, of which **270 are degenerate** (planar: no 3-D hull, MuJoCo refuses them) |

### 4.6 Base yaw from the taught points (*measured*, 2026-10-17)

The owner's program `sf1_plc_v4.script` (sha256 `ecec8e53…8eae`; its 156 global `_p`/`_q` lines are
committed verbatim as `test/fixtures/sf1_plc_v4_positions.urscript`) contains grasp and drop points
whose targets are known. World TCP (DH FK · TCP of `_q`, base at `Base_Position`), checked by a
vertical ray through the STL triangles:

| Points | At 180° | At 0° |
|---|---|---|
| `pick_obj_1..8` | x [−0.21, 0.22], y −0.96, z 0.757–0.763: on the pallet-table plate (component 1060, x [−0.295, 0.295], y [−1.411, −0.501], top 0.721), below the part tops at 0.7705–0.7735 | x [−0.22, 0.21], y +0.96: nothing in the STL |
| `unload_obj_1..8` | x 0.42–0.85, y 0.00, z 0.756–0.761: on the side-table plate (component 1061, top 0.721), parts up to 0.7735 | x −0.42 … −0.85, y 0: nothing in the STL |
| `bench_drop_1..8` | (−1.12, −0.08, 1.08): inside machine (a) | x 1.12, beyond the STL (x max 0.934) |

At 180° every point lands where it handles a part; at 0° none does. The owner also sees the robot
at home facing the machine. With 180°, the regrasp at the rotation station collides with the model
(`KNOWN_ISSUES.md`): the STL does not match the cell there. The grasp points themselves are inside
the margin of the modelled parts they grasp (−12 to −20 mm), and the machine drops are 150 mm inside
the hull of machine (a): both are expected for points that handle parts, and neither is a learned
point of the planner.

## 5. Platform

- macOS on Apple silicon (primary, verified here) and Ubuntu Linux (unverified).
- Python **3.11**, called explicitly. On macOS it is `/opt/homebrew/bin/python3.11`, on Linux
  `python3.11` from `PATH`. A bare `python3` is never used: miniforge's numpy is broken on this
  machine.
- The viewer: `mujoco.viewer.launch_passive` runs under `mjpython` on macOS and under `python` on
  Linux. The demo detects the case and, on macOS without `mjpython`, exits with an `[ERROR]` line
  that gives the command to use. A `--headless` mode runs without a viewer (used by the tests).
- The 1.76 M-triangle visual mesh is passed to MuJoCo as `uservert`/`userface` read by trimesh,
  because the MuJoCo STL loader refuses more than 200 000 faces (§4.5). Its geom has
  `contype=conaffinity=0` and shell inertia, so MuJoCo stores no hull for it. A test pins this.
- Pinned versions (installed 2026-10-17, re-checked at implementation): `mujoco==3.16.0`,
  `numpy==2.4.6`, `scipy==1.17.1`, `trimesh==5.1.1`, `coacd==1.0.14`, plus `pyyaml`, `pytest`,
  `rtree` (exact pins in `requirements.txt`). CoACD and trimesh are used only by the decomposition
  build script, never by the tests.

## 6. Repository structure

```
README.md                 install, run, test — nothing else
SPEC.md                   this file
CLAUDE.md                 short: links to the skills, test and checker commands
KNOWN_ISSUES.md           accepted risks and known limitations, only what the skeleton decided
LICENSE                   MIT, Copyright (c) 2026 JIC Smart Factory
install.sh                macOS + Ubuntu, OS detection
requirements.txt          pinned
.gitignore                .venv, __pycache__, _source_skills/, .pytest_cache
.claude/settings.json
.claude/skills/project-style-rpx/   SKILL.md, README.md, references/, assets/, scripts/
.claude/skills/feature-manager/SKILL.md
.claude/skills/project-optimize/SKILL.md
CAD/SF1_Environment.stl                       source, unchanged (visual mesh)
CAD/Collision/SF1_Environment_Collision.npz   convex pieces (vertices, faces, piece → component, body)
CAD/Collision/SF1_Environment_Collision.json  manifest (§7.6)
Configuration/Cell_SF1.yaml                   cell configuration (§8)
Data/Positions/SF1_Positions.urscript         the learned points under the project names (§10.3)
models/MANIFEST.json                          Menagerie commit, licence, sha256 of every vendored file
models/universal_robots_ur10e/                xml, assets/, LICENSE, README.md, CHANGELOG.md (no png)
models/robotiq_2f85/                          same
src/Configuration/Core.py   Load_Configuration → Cell_Configuration_Str (the only YAML reader)
src/URScript/Core.py        Read_URScript, Write_URScript, Write_Movej_Sequence (the only URScript reader/writer)
src/Kinematics/Core.py      UR10e DH FK, rotvec ↔ matrix, Geodesic_Angle, Normalize_Rotvec, Derive_TCP_Offset
src/Collision/Core.py       Build_Collision_Set, Write_Collision_Set, Read_Collision_Set (the only npz/manifest reader/writer)
src/Scene/Core.py           Build_Scene (MjSpec), Set_Configuration, Check_Clearance
Example/build_collision.py  regenerates CAD/Collision/ from the STL (runs CoACD; not in the test suite)
Example/demo_rotate_axis.py the viewer demo
test/<Domain>/test_<module>.py mirroring src/, test/Install/test_install.py, test/Example/test_demo.py
docs/design-decisions.md
```

`src/` layering: `Configuration`, `URScript` and `Kinematics` import nothing of the project;
`Collision` → `Kinematics`; `Scene` → everything above. Scripts in `Example/` reach `src/` through a
path built from `__file__`.

## 7. Environment collision geometry (risk 1, requirement E)

MuJoCo collides meshes through their convex hull. The STL as one mesh would be one blob, so it is
never used for collision; it stays the visual geom (`contype=conaffinity=0`).

### 7.1 Crop to the reachable volume

- Centre: the shoulder point (base frame (0, 0, d1)), mapped to world.
- Radius `R_crop`: an upper bound on the distance from that point of any robot or gripper
  collision vertex in any configuration. It is the sum of the body-origin offsets along the chain
  after the shoulder, plus the largest vertex radius of the outermost bodies (gripper included),
  plus the environment clearance, plus 0.05 m. It is computed from the scene, not configured, and
  recorded in the manifest. Expected ≈ 1.7 m.
- A component is kept whole if any of its vertices lies inside the sphere. Triangles are never cut.
- Reported: components kept / dropped, triangles kept.

### 7.2 Hull first, decompose only where the hull is too coarse

1. A convex hull per kept connected component. A **degenerate** component (planar or collinear, no
   3-D hull; 270 of 839 in the stand alone, §4.5) is thickened by ±0.5 mm along the normal of its
   plane, from the SVD of its centred vertices (smallest singular vector). A mean face normal is not
   used, because the winding is inconsistent and normals can cancel. A collinear component is
   thickened by ±0.5 mm along two directions orthogonal to its line. The result contains the original, so it stays conservative and adds
   ≤ 0.5 mm of excess. Every collision mesh gets shell inertia, because the bodies are static.
2. Hull **excess** per component: the maximum distance from the hull surface (its vertices plus
   area-uniform samples, fixed seed) to the original component surface.
3. Excess ≤ `Hull_Excess_Max` (build-script parameter, default 10 mm, recorded in the manifest) →
   the hull is the collision piece.
4. Otherwise run CoACD on that component only (`preprocess_mode="auto"` for open shells, fixed
   seed, parameters recorded) and re-measure. If the CoACD pieces fail the conservativeness check
   (§7.3), the component falls back to its hull. Conservativeness beats tightness.
5. Optional merge pass (only if the piece count makes collision queries too slow): bbox-touching
   neighbours whose joint hull stays within `Hull_Excess_Max` become one piece. *Unsettled* — it is
   used only if the measured query time asks for it.

Reported: pieces by method (hull / CoACD / fallback), runtime of each stage, total runtime.

### 7.3 Verification — surface metrics, not volume

Volume is meaningless for a non-watertight mesh, so it is not used.

1. **Conservativeness**: every test point lies inside, or within `eps = 1 mm`, of the union of
   collision pieces (signed distance to the convex pieces). Reported: max violation, number of
   violating points (must be 0).
   - **Hull-only components (and thickened degenerate ones) are exempt by convexity.** A convex hull
     contains every convex combination of its vertices, so the whole original surface lies inside.
   - **Components decomposed by CoACD** are tested on their vertices **and** on area-uniform
     surface samples of the original mesh: fixed seed 0, density **1 sample per 1 cm²** of
     original surface (at least 1 per triangle). A large flat CAD triangle can leave a union of
     pieces while all three vertices stay inside, which a vertex-only test cannot see.
   - A component that fails falls back to its hull (§7.2).
2. **Excess**: the maximum distance the collision geometry extends beyond the original surface,
   overall and per piece. Reported: max, p95, and the 20 largest with their location.

### 7.4 Open shells filling space the robot needs

A hull of an open shell (sheet-metal housings, frames, especially in object (a)) fills its
interior. The build report lists every piece with excess > `Hull_Excess_Max` after fallback, with
its bbox and the object it belongs to. A test asserts that neither the TCP nor any robot body at A
or B lies inside such a piece. Whether the planner later needs to enter such a region is flagged in
`KNOWN_ISSUES.md`, not silently accepted.

### 7.5 Stand body

The pieces of components whose bbox lies fully inside `Environment.Stand_Region` go to the static
body `env_stand`; all others go to `env`. Only the pair robot `base` ↔ `env_stand` is excluded (the
base sits 0.2 mm above the profiles and never moves). `shoulder_link` and everything above are
checked against the stand.

### 7.6 Storage and manifest

`CAD/Collision/` is committed. The manifest holds: the sha256 of the source STL; the crop centre
and radius; `Hull_Excess_Max`, `eps`, CoACD version and parameters, seed; counts and runtimes; both
metrics. `Read_Collision_Set` refuses a set whose recorded STL hash differs from the file on disk
(`Collision_Error_Cls`, with the sentence "rerun Example/build_collision.py"). Tests read the set and
never run CoACD.

**Component overrides.** `Environment.Component_Overrides` lists STL components whose geometry the
real cell contradicts, each with a rigid translation (≤ 150 mm) and its source. The build translates
the component's own vertex copy before the crop and the decomposition, so a neighbour sharing a
vertex is not moved, and the conservativeness check runs against the translated surface. The
manifest records the overrides (`component_overrides`); the scene refuses a set whose record differs
from the configuration (`Scene_Error_Cls`, "Rebuild it with …"). A set built before overrides existed
records none. The visual mesh is never changed, so at an overridden component the viewer shows the
original surface and the collision set the corrected one.

## 8. Cell configuration — `Configuration/Cell_SF1.yaml`

The loader rules follow `references/yaml.md` of the style skill. A REQUIRED field that is missing,
or a value that is invalid, fails the load; all problems are collected first and named together. An
unknown key is reported as a WARNING. Every field is read by code whose behaviour it changes. No
field without a reader.

| Field | Meaning | Unit | Status |
|---|---|---|---|
| `Environment.Visual_Mesh` | the STL, repository-relative | path | REQUIRED |
| `Environment.Collision_Set` | the `.npz`, repository-relative | path | REQUIRED |
| `Environment.Stand_Region.Min/Max` | box selecting the stand components (§7.5) | m | REQUIRED |
| `Environment.Component_Overrides` | rigid translations of STL components before the build: `{Component, Translation [x, y, z] (world, ≤ 150 mm), Source}` each; may be empty (§7.6) | –, m, text | REQUIRED |
| `Robot.Model` | `models/universal_robots_ur10e/ur10e.xml` | path | REQUIRED |
| `Robot.Base_Position` | Polyscope base origin in the STL/world frame: `[0.0, 0.0, 0.7152]` | m | REQUIRED |
| `Robot.Base_Yaw` | rotation of the Polyscope base frame about world z: `180.0` (owner, 2026-10-17; confirmed by the taught points of `sf1_plc_v4.script` and photos of the cell, §4.6). *Corrected 2026-10-17: was `0.0`, which put every taught grasp point in empty space* | deg | REQUIRED |
| `Robot.Joint_Position_Limits.Min/Max` | joint position limits, 6 each. Shipped: ±360° for base, shoulder, wrists; **±180° for the elbow** (Menagerie default, owner). Load rejects: any joint beyond ±360°, and **the elbow beyond ±180°**, because that limit is the precondition for excluding `upper_arm_link–forearm_link` (§9.6). The Polyscope safety limits may be tighter (`KNOWN_ISSUES.md`) | deg | REQUIRED |
| `Tool.Model` | `models/robotiq_2f85/2f85.xml` | path | REQUIRED |
| `Tool.Coupling.Thickness` | coupling thickness: `0.0169` (manual rev. 2020/02/11 p. 119, Fig. 5-5) | m | REQUIRED |
| `Tool.Coupling.Diameter` | coupling outer diameter: `0.075` | m | REQUIRED |
| `Tool.Coupling.Spigot_Depth` | depth of the Ø71 bore receiving the gripper: `0.0029`; gripper datum = thickness − depth = 14.0 mm (measured on the robot) | m | REQUIRED |
| `Tool.Mount_Yaw` | rotation of the gripper about the flange z axis relative to the Menagerie default: `0.0`. **Documented assumption**, not measured (§9.3, `KNOWN_ISSUES.md`) | deg | REQUIRED |
| `Tool.Opening` | inner pad-face gap held during transit: `0.050` | m | REQUIRED |
| `Tool.TCP_Offset` | TCP in the flange frame `[x, y, z, rx, ry, rz]`: `[0.0, 0.0, 0.174, 0.0, 0.0, 0.0]`. Source: the Polyscope TCP of SF1 (owner, 2026-10-17), identical to the Robotiq value (manual rev. 2020/02/11 p. 127, §5.2.3) | m, rad (rotvec) | REQUIRED |
| `Clearance.Environment` | minimum robot/gripper ↔ environment distance: `0.020` | m | REQUIRED |
| `Clearance.Self` | minimum distance between non-excluded robot/gripper bodies: `0.005` | m | REQUIRED |
| `Clearance.Self_Pair_Overrides` | `[[base, upper_arm_link, 0.0]]` (Q9 (a)): a pair-specific self margin. Allowed only for a pair whose true minimum stays > 0: grid over the joints that move it, then scipy local minimisation from the 20 lowest cells; the test checks every entry and pins the refined minimum | m | REQUIRED |
| `Clearance.Station_Zone.Min/Max` | world box of the station zone: environment pieces whose bounding box meets it are the zone pieces (§9.7) | m | REQUIRED |
| `Clearance.Station_Zone.Environment` | margin of the zone pieces where the zone applies; > 0 and ≤ `Clearance.Environment` | m | REQUIRED |
| `Clearance.Station_Zone.Endpoint_Radius` | a generated sample uses the zone margin only while its TCP lies within this distance of a fixed station point's TCP (§9.7) | m | REQUIRED |
| `Self_Collision.Excluded_Pairs` | explicit body pairs never checked (§9.6) | — | REQUIRED |
| `Self_Collision.Rigid_Groups` | groups of bodies rigid to each other; every pair inside a group is excluded (the gripper at fixed opening) | — | REQUIRED |
| `Kinematics.FK_Sanity_Bound.Position` | the most a learned point's DH FK·TCP may differ from its `_p` before it is refused: `0.010` (§9.4) | m | REQUIRED |

The orientation sanity bound is derived from the position bound (§9.4), not configured. There are
no optional fields in the milestone, so there are no defaults to keep in sync. Values that have a
single legal value are deliberately **not** fields: robot type (UR10e DH is a code constant), blend
radius (always 0, §8.5), joint wrapping (never, §10.5). Speed and acceleration are not fields either:
`movej` with `r=0` follows the same joint-linear path at any speed, so they cannot change what is
checked (owner, Q4).

### 8.5 Blend radius

Every emitted `movej` has `r=0`. The collision check covers the joint-linear segment between
consecutive targets (§9.7). With `r > 0` the robot leaves that segment, so a blend is a future
feature that needs envelope checking. Until then it is not a parameter.

## 9. Kinematics, TCP and collision model

### 9.1 Frames

- World = STL frame. Polyscope base frame = world translated by `Robot.Base_Position` and rotated
  by `Robot.Base_Yaw` about z. The Menagerie root (with its built-in 180°) is placed at exactly that
  pose; §4.2 shows that the Menagerie world frame already equals the Polyscope base frame.
- `Robot.Base_Yaw` = 180°: the base frame's +x points to world −x, towards machine (a). FK vs `_p`
  cannot see the yaw; the placement of the taught points decides it (§4.6). *Corrected 2026-10-17:
  0° was configured until then.*
- Flange frame = DH frame 6 (z out of the flange).
- TCP = flange · `Tool.TCP_Offset`.

### 9.2 One kinematic truth: the official UR10e DH

d = (0.1807, 0, 0, 0.17415, 0.11985, 0.11655) m, a = (0, −0.6127, −0.57155, 0, 0, 0) m,
α = (π/2, 0, 0, π/2, −π/2, 0). The scene builder overwrites the Menagerie body offsets and the
flange site through MjSpec so that MuJoCo FK equals DH FK. Vendored files stay byte-identical to
upstream; the patch is in code and in `docs/design-decisions.md`. The gripper attaches at the
corrected flange.

### 9.3 TCP and gripper geometry (owner: configured, sourced, do not tune)

1. **The TCP is configured, not derived.** `Tool.TCP_Offset` = (0, 0, 0.174) m, no rotation: the
   Polyscope TCP of SF1, identical to the Robotiq value (§4.4, §16). The YAML comment above the field
   cites both sources. Every FK check uses `FK_DH(q) · TCP_config`.
2. The derived TCP (`FK_DH(q)⁻¹ · T(p)`) is a **diagnostic only**. `Derive_TCP_Offset` exists to
   print it (§4.3, §4.4 comparison 1). It never writes the configuration, never feeds a check, and
   is held only by the regression pin (§9.4).
3. Gripper geometry: the Menagerie `base_mount` mesh geoms (visual and collision) are removed in
   MjSpec. They are replaced by a cylinder of `Tool.Coupling.Diameter` × `Tool.Coupling.Thickness`
   (visual and collision) on the flange face. The gripper `base` body is moved so that its datum
   lies at `Thickness − Spigot_Depth` from the flange face. That is a +3.0 mm shift from the
   Menagerie position; the shift is computed from the configured coupling values, not stored.
   Vendored files stay untouched.
4. Pinned by a test against the **manual's** dimension chain: the pad end from the flange face is
   163.3 mm open and 176.8 mm closed, each within 0.5 mm (the manual gives 0.1 mm; the model
   reproduces it to 0.31 mm). The values at 50 mm opening (pad-face centre 154.39 mm, pad end
   173.23 mm, variant (b) of §4.4) are pinned as regression values (±0.05 mm).
5. If a pin fails after a change, the test reports the numbers and **nothing is adjusted to make it
   pass**.
6. **Mounting orientation, `Tool.Mount_Yaw` = 0: a documented assumption, not measured** (owner).
   The manual indexes the coupling with the dowel pin next to the wrist connector (p. 21, p. 23
   Fig. 3-2) but states no angle. The scene uses the Menagerie default orientation. At
   implementation I report in which flange axis the fingers open in the scene. Recorded in the
   YAML comment and in `KNOWN_ISSUES.md`. This relaxes only this item: every other requirement, and
   "simulation passed ≠ safe on the robot", stays as it is.

### 9.4 FK accuracy: regression pin + independent sanity bound

The residual `FK_DH(q) · TCP_config` vs `_p` is kinematic error (§4.3): nominal DH vs the
robot's calibration. Two separate checks, with separate jobs:

1. **Regression pin (narrow band, test only):** the measured residuals with the configured TCP,
   home 1.72 mm / 0.261° and drop_rotate_1 4.02 mm / 0.376°, plus the diagnostic derived-TCP values
   of §4.3.
   The band is ±0.01 mm / ±0.001°. These numbers come from this measurement on purpose: the pin
   detects any change in FK, DH constants, rotvec conversion, the parser or the TCP. It says nothing
   about whether the residual is acceptable.
2. **Sanity bound (independent of the measurement, enforced at runtime):** derived from the
   clearance margin, not from the data.
   - Position: **10 mm** = half of `Clearance.Environment`. A learned point whose `_p` and `_q`
     disagree by more than that would consume over half the margin, or it points to a typo or a
     different TCP. Either way it must be refused.
   - Orientation: θ_max = arcsin(position bound / L_tool), where L_tool is the largest distance of
     any gripper collision vertex from the flange origin, computed from the scene (≈ 0.18 m →
     θ_max ≈ 3.2°). This is the rotation at which the farthest gripper point moves by the
     position bound.
   - `Check_Learned_Point` applies the bound to every point read from a file and raises
     `Kinematics_Error_Cls` with the numbers. A point is never accepted silently.

The measured 4.02 mm / 0.38° lies at 40 % / 12 % of the bound.

**Consistency with the clearance budget.** The kinematic error is known at two poses only, so its
size along a path can only be assumed. The sanity bound allows up to 10 mm at a learned point. The
budget (§9.8) therefore shows both cases, the measured 4.0 mm and the bound's 10 mm, and
`KNOWN_ISSUES.md` states that the error was measured at two poses only and may be larger elsewhere
along the path.

### 9.5 Orientation comparison (requirement A)

- Always the geodesic angle, never component-wise rotation vectors. It is computed as
  `θ = atan2(‖vee(M − Mᵀ)‖ / 2, (tr M − 1) / 2)` with `M = R_aᵀ R_b`. The arccos form cannot resolve
  angles below ≈ 6e-8 rad (§4.5), and the 1e-9 test needs that resolution.
  `home_p` has |r| = 3.1393 rad ≈ π, where r and the equivalent −r′ differ component-wise by ≈ 2π
  while describing the same rotation. A test pins exactly this case.
- `Normalize_Rotvec` for **generated** points: θ ∈ [0, π]. θ < 1e-12 → (0, 0, 0). If π − θ < 1e-9
  the axis sign is chosen so that its first component with |·| > 1e-9 is positive. Learned points A
  and B are always written verbatim and never normalised.

### 9.6 Robot and gripper collision geometry; self-collision

- Robot: the convex hulls of the Menagerie visual meshes of each link (MuJoCo hulls every mesh
  geom). Each hull contains its visual part by construction. The Menagerie capsules are removed,
  since they do not necessarily contain the mesh.
- Coupling: the cylinder of §9.3 in body `base_mount`, which replaces the Menagerie disc.
- Gripper: the Menagerie 2F-85 collision geoms, at the joint angles that give `Tool.Opening`. These
  are solved once at scene build by bisection on the simulated equilibrium of the gripper alone; the
  check is that the equality-constraint residual is < 0.1 mm.
- **Pad-span hull (owner: option (ii)).** One extra collision geom in the gripper `base` body is
  the convex hull of the 32 corners of the four pad boxes at the fixed opening. It covers the space
  between the fingers. It belongs to the gripper rigid group. A part protruding beyond the pad ends
  is still not covered (`KNOWN_ISSUES.md`).
- Why the pad-span hull is needed: the gripper has no single convex hull. Each part (chassis,
  drivers, couplers, spring links, followers, pad boxes) is hulled separately. Without the extra
  geom the space between the fingers is free: the chassis hull ends ≈ 104 mm from the flange face,
  while the pads span ≈ 136–173 mm, ≈ 50 mm apart.
- MuJoCo's automatic parent/child filter is **disabled** (`<flag filterparent="disable"/>`). Only
  the configured exclusions apply:
  - `Excluded_Pairs` (adjacent links, whose hulls touch at the joint):
    `base–shoulder_link`, `shoulder_link–upper_arm_link`, `upper_arm_link–forearm_link`,
    `forearm_link–wrist_1_link`, `wrist_1_link–wrist_2_link`, `wrist_2_link–wrist_3_link`,
    `wrist_3_link–<gripper base_mount>`.
    **The `upper_arm_link–forearm_link` exclusion is valid only with the elbow limited to ±180°.**
    Beyond that the forearm folds onto the upper arm, and the excluded pair would hide it. The
    loader therefore rejects an elbow limit beyond ±180° (§8).
  - `Rigid_Groups`: one group with all 2F-85 bodies (base_mount, base, left/right driver, coupler,
    spring_link, follower, pad, silicone_pad). They do not move relative to each other at fixed
    opening, so checking them carries no information.
  - Environment: `base–env_stand` only (§7.5). This one is code, not configuration, because it has
    a single legal value.
- `base – upper_arm_link` is a structural near pair (min 2.3 mm, inside 5 mm in 17.5 % of samples,
  §4.5). It is **not** excluded; Q9 decides how it is handled.
- A test asserts that the compiled model's exclusion set equals exactly the expansion of the
  configuration plus `base–env_stand`, with nothing from model defaults.
- Joint ranges in the compiled model are set from `Robot.Joint_Position_Limits`, not from the
  Menagerie defaults.

### 9.7 Clearance check (risk 6, requirement D)

- Implementation: geom margins. Environment geoms get `Clearance.Environment`, robot and gripper
  geoms get `Clearance.Self`. MuJoCo generates a contact while the gap is below the **sum** of the
  two geom margins (25 mm robot↔environment, 10 mm robot↔robot; measured with mujoco 3.16,
  `test/Scene/test_margin.py`). That is a superset of the configured margins, and `Check_Clearance`
  filters every contact by the pair's configured margin itself: the larger geom margin, i.e. 20 mm
  for robot↔environment and 5 mm for robot↔robot, or the pair override. Contacts with
  `dist < margin` are violations; `dist < 0` is a collision. (Corrected 2026-10-17: an earlier
  revision said MuJoCo itself uses the larger margin.) The configuration is set with `mj_kinematics` +
  `mj_collision`, with no dynamics.
- `Check_Clearance(q)` returns every violating pair (bodies, distance, margin), never just a bool.
- **Static bodies.** MuJoCo never collides two jointless, world-welded bodies (§4.5). The robot
  `base` is one, so `mj_collision` never reports `base` ↔ `env` (`env_stand` is excluded anyway).
  The base does not move, so its clearance to `env` is a constant. It is checked once at scene
  build with `mj_geomDistance` and fails the build with the violating pieces if any is below
  `Clearance.Environment`. A test pins that this check exists and runs.
- A segment between joint targets (the planner's job, specified here so the planner inherits it):
  joint-linear interpolation, as `movej` moves, sampled so that no point of the robot moves more than
  `Clearance.Environment / 4` between samples. The bound comes from joint step × lever arm (DH),
  not from a fixed sample count.
- **Station zone (taught vs generated).** The zone pieces (§8 `Clearance.Station_Zone`) keep the
  global geom margin, so MuJoCo still generates their contacts below 25 mm; `Check_Clearance(q,
  station_zone=True)` filters a contact with a zone piece by the zone margin instead. `Check_Segment`
  takes the kind of segment:
  - **taught** (a segment of the robot program): the zone margin on every sample;
  - **generated** (a planner's segment; the default): the global margins everywhere, except on a
    sample whose TCP lies within `Endpoint_Radius` of the TCP of a fixed station point
    (`Station_Zone_Active`), where the zone margin applies. The station geometry is fitted, not
    measured, so a new path may not use it more tightly than the taught program does.

  A learned point is taught: `Check_Learned_Point` applies the zone margin to the zone pieces.
- **Start or goal inside the margin**: `Plan` (future) and `Check_Learned_Point` (milestone) raise
  `Clearance_Error_Cls`, which names the point, every violating pair, its distance and the margin.
  No silent pass, no silent failure, no override in the milestone. An override is a decision for a
  later feature.

### 9.8 Clearance budget

`Clearance.Environment` = 20 mm must cover:

| Item | Case A: measured | Case B: sanity bound | Basis |
|---|---|---|---|
| Kinematic error (nominal DH vs the calibrated robot) | 4.0 mm | 10.0 mm | A: max measured, at two poses only (§4.3). B: the most a learned point may have (§9.4) |
| Collision-set conservativeness | 1.0 mm | 1.0 mm | `eps` (§7.3) |
| Coupling | 0 mm | 0 mm | measured on the robot, 14.0 mm = drawing (§4.4) |
| **Remaining for cables** at the base and the tool (not modelled) | **15.0 mm** | **9.0 mm** | 20 − kinematic error − eps |

The error was measured at two poses only. Case A is the expected case; case B is what the checks
can guarantee for learned points. At generated via points the error cannot be measured, so neither
case is guaranteed there (`KNOWN_ISSUES.md`).

Not in the budget, because they are not small bounded errors. Each is listed in `KNOWN_ISSUES.md`:

- the carried 50 × 50 mm part (not modelled, accepted by the owner; not covered by the gripper
  geometry, §9.6);
- `Tool.Mount_Yaw` = 0, an unmeasured assumption (§9.3);
- Polyscope safety limits possibly tighter than ±360°.

Self-collision (5 mm) is not affected by cables or calibration at the gripper. Only eps applies.

## 10. Data formats

### 10.1 URScript variables (reader and writer in `src/URScript/Core.py` only)

```
global <name>_p=p[x, y, z, rx, ry, rz]       # m, rad (rotation vector) in the Polyscope base frame
global <name>_q=[j0, j1, j2, j3, j4, j5]      # rad: base, shoulder, elbow, wrist1, wrist2, wrist3
```

- Number grammar: `-?(\d+)?(\.\d*)?([eE][+-]?\d+)?` with at least one digit, so `.238…`, `-.175…`
  and `2.44E-5` are accepted. Separators are `, ` as written by Polyscope; the reader also accepts
  any whitespace.
- The reader keeps each number's **original token** next to its float. The writer writes learned
  points from the tokens, so `Write_URScript(Read_URScript(text)) == text` byte for byte.
- Strict: a non-empty line that is not one of the two forms (comments `#` excepted) → 
  `URScript_Error_Cls` with the line number. Also an error: a name with `_p` but no `_q` or the
  reverse, a duplicate name, or a vector of the wrong length.
- **Generated** points get fixed precision: `%.12f` with the leading zero dropped (`0.5` → `.500000000000`,
  `-0.5` → `-.500000000000`), `-0` written as `0`. Their `_p` comes from DH FK · TCP with
  `Normalize_Rotvec` and is informational (§10.4).

### 10.2 Encoder noise

`2.44E-5` rad in `home_q` is encoder noise from teaching. It is read and written unchanged.

### 10.3 Naming convention

| Original (robot) | Project | Role |
|---|---|---|
| `home` | `home_safe` | A |
| `drop_rotate_1` | `drop_rot_01` | B |
| — | `via_<from>_<to>_<nn>`, e.g. `via_home_safe_drop_rot_01_01` | generated via points |

`Data/Positions/SF1_Positions.urscript` holds A and B under the project names, with values
verbatim. The original export is kept as a test fixture, and a test asserts that the number tokens
of both files are identical. The mapping is recorded in that file's comment header and in
`docs/design-decisions.md`.

### 10.4 Motion output (requirement B)

```
movej(<name>_q, r=0)
```

`a` and `v` are omitted, so the robot uses the URScript defaults (a = 1.4 rad/s², v = 1.05 rad/s).
The operator sets the speed on the robot. Speed does not change the path of a `movej` with `r=0`,
so it is not the tool's to choose. `r=0` is written explicitly because it is the one argument that
does change the path. Joint targets only, never `movej(p[...])`. `_p` of via points is
informational: the robot would otherwise solve its own IK and could pick another branch.

### 10.5 Joint branches (requirement C)

Joint values are never wrapped into [−π, π]. UR joints span ±2π and `movej` goes to the exact
value. The planner and writer stay on the branch of A and B. A test feeds A and B through parse →
path (the milestone's trivial path [A, B]) → write and asserts the tokens are unchanged, plus a
synthetic point with a joint at 4.0 rad that must come out as 4.0.

## 11. Tests of the first milestone

| Test file | What it pins |
|---|---|
| `test/URScript/test_core.py` | exact float values of all 24 numbers; byte-identical round trip; `.238`, `-.175`, `E-5` tokens; strict errors (malformed, missing pair, duplicate, wrong length, line number in message); generated-point format; no wrapping (4.0 rad); `movej` line uses `_q` and `r=0`; renamed file tokens == original export; the `sf1_plc_v4` extract is read strictly with every token, and equals its source lines 2453–2608 byte for byte with the recorded sha256 when `SF1_PLC_V4_SCRIPT` names the program (otherwise skipped, never passed) |
| `test/Configuration/test_core.py` | shipped file loads; each REQUIRED field missing → named failure; invalid values (wrong length, limits outside ±360°, min ≥ max, coupling spigot depth ≥ thickness, negative clearance or sanity bound, a component override beyond 150 mm, without a source or named twice, a zone margin above `Clearance.Environment`, a zone box with Min ≥ Max, a zero endpoint radius) fail; all problems listed at once; unknown key → WARNING |
| `test/Kinematics/test_core.py` | `FK_DH(q) · TCP_config` vs `_p`: regression pin (§9.4) of 1.72 mm / 0.261° and 4.02 mm / 0.376° (±0.01 mm / ±0.001°); diagnostic derived TCP pinned the same way and never read by any check; both points within the sanity bound; `Check_Learned_Point` rejects a point beyond it (perturbed `_q`) with the numbers in the message; configured TCP == (0, 0, 0.174, 0, 0, 0); geodesic angle 0 between r and its π-equivalent; `Normalize_Rotvec` rule incl. near-π |
| `test/Scene/test_core.py` | scene compiles; base body world position == `Base_Position` exactly and yaw == `Base_Yaw`; **at `Base_Yaw` = 90° (a temporary copy of the configuration) `home_safe`'s TCP lands at `Base_Position` + (−y, x, z) of its `_p` within 3 mm, in MuJoCo and through the base frame (the file is in degrees; read as radians the test fails)**; **MuJoCo flange FK (after the MjSpec offset override) == DH flange FK within 1e-9 m / 1e-9 rad (atan2 geodesic, §9.5) for 1000 seeded random joint vectors within the limits**, plus A and B; pad-span hull present in the gripper rigid group; fingers open along flange ±x at `Mount_Yaw` = 0; visual mesh has no hull (`mesh_graphadr = −1`); base↔env build check runs; loader rejects an elbow limit beyond ±180°; gripper opening == `Tool.Opening` ± 0.5 mm, equality residual < 0.1 mm; coupling cylinder present (visual + collision), Menagerie `base_mount` mesh geoms absent, vendored files byte-identical to `models/MANIFEST.json`; pad end from the flange face 163.3 mm open and 176.8 mm closed (± 0.5 mm, manual chain) and the 50 mm values 154.39 / 173.23 mm (± 0.05 mm, regression); exclusion set == configuration expansion + `base–env_stand`, filterparent disabled; joint ranges == `Joint_Position_Limits`; **placement (§4.6): `pick_obj_1..8` on the pallet table, `unload_obj_1..8` on the side table (between the plate top and the highest part), `bench_drop_1..8` inside object (a); the same check at `Base_Yaw` = 0 refuses all 24**; A and B pass `Check_Clearance`, and **any violation fails the test, listing every pair with its distance and margin** (no report-only path); B (`drop_rot_01`) is a strict xfail while the STL does not match the cell at the rotation station (`KNOWN_ISSUES.md`), so it turns red once the geometry is fixed; `hover_unload_2` (clear) pushed into the side table is reported as a collision; a collision set whose recorded component overrides differ from the configuration is refused; the station zone applies to a generated configuration only within `Endpoint_Radius` of a station point (DH distances); a segment kind other than taught/generated is refused |
| `test/Scene/test_margin.py` | margin accuracy on **mesh hull vs mesh hull** (robot geom on a joint, so not static): a robot link hull at 3 orientations vs a box-hull face, and box-hull edge vs edge; at 10 mm a violation is reported, at 30 mm none; reported distance error < 0.5 mm (prototype: 0.000 mm, §4.5) |
| `test/Collision/test_core.py` | manifest STL hash == sha256 of the file; `R_crop` ≥ the bound recomputed from the current scene, and no robot collision vertex leaves the sphere over 1000 seeded random `q`; conservativeness violations == 0 at `eps` (vertices + 1 sample/cm² for CoACD components, hull-only exempt); degenerate components thickened and containing the original; excess metrics present and equal to a recomputation on a fixed sample; no robot body at A/B inside a flagged open-shell piece (B a strict xfail while the STL does not match the cell at the rotation station); mismatched hash → `Collision_Error_Cls` |
| `test/Example/test_demo.py` | headless demo runs N steps without exception; only the chosen joint changes, monotonically between turning points; it stays within `Joint_Position_Limits`; the pre-check runs on the dense segment samples of §9.7; the label equals the check result ("clearance checked" iff no violation, otherwise "VISUAL ONLY — UNCHECKED"); the shipped motion's label is pinned |
| `test/Install/test_install.py` | `bash -n`; sourced steps under a hermetic `PATH`: OS detection (Darwin arm64, Linux), Python 3.11 resolution (`python3` never called), hash verification fails on a tampered model file, every log line in the log format |
| `.claude/skills/project-style-rpx/scripts/test_check_style.py` | the checker itself (outside `test/`) |

Expected numbers from the tests are taken from the measurements above or from the configuration,
never from the code under test.

## 12. Installer — `install.sh`

Steps, cheapest check first:

1. detect the OS and architecture;
2. find Python 3.11 (macOS: Homebrew path; Linux: `python3.11`) or fail with the install command;
3. create `.venv` if absent; report a broken one with the command to remove it, never delete it;
4. `pip install -r requirements.txt`;
5. verify `models/MANIFEST.json` hashes (and the collision manifest's STL hash).

Re-runnable, with log lines in the project log format. `--detect-only` stops after the plan. Ubuntu
path: written, tested only against stubs, **unverified**.

## 13. Skills (`.claude/skills/`)

Written as new skills of this project from `_source_skills/` as templates:

- **project-style-rpx**: RP conventions kept (`_Cls`, `_Str`, `CONST_*`, Pascal_Snake_Case, `import
  typing as tp`, numbered `Description/Args/Returns` docstrings, `#` MIT header with JIC Smart
  Factory, `*_Error_Cls` with operator sentences, pytest). Removed: Qt, UI design, Basler, vision
  models, AnyScan decisions. Rewritten: `shell.md` (this `install.sh`, macOS + Ubuntu), `yaml.md` (the
  cell configuration). New: `urscript.md` (one reader, one writer, round trip, units, tokens) and
  `mjcf.md` (vendored models, patch in code, collision geometry, exclusions). `check_style.py`:
  Qt exemptions removed, paths `src`, `Example`, `test`; its tests pass.
- **feature-manager**: the procedure is kept (restate → verify the premise → challenge the solution →
  report → measure → tests → small commits → docs). New robotics checklist: kinematics and TCP,
  collision and decomposition, clearance, blends, joint limits and branches, sim vs reality,
  "simulation passed ≠ safe on the robot".
- **project-optimize**: the four kinds (cleanup, speed, complexity, reliability) and "behaviour does
  not change" are kept. Added: the generated path is output, so it is compared byte for byte on a
  fixed seed before and after. Examples are generic.

No invented project history and no links to documents the skeleton does not create; everything
unverified is marked *Unsettled*.

## 14. Risks and how they are handled

| # | Risk | Handling |
|---|---|---|
| 1 | Mesh collision = convex hull | crop → hull per component → CoACD only where coarse; surface metrics (§7) |
| 2 | Units / frames | STL in m (measured); base pose and yaw REQUIRED fields; Menagerie 180° verified (§4.2); base-position test; a 90° yaw test pins degrees in the file; the placement test of the taught points (§4.6) catches a wrong world yaw, which FK vs `_p` cannot (it caught the earlier 0°) |
| 3 | Models | Menagerie pinned commit, BSD-3/BSD-2, vendored, hash manifest; Menagerie offsets patched to DH (16.5 mm flange error found) |
| 4 | Sim vs real kinematics | DH truth; configured TCP (Polyscope = Robotiq manual), derived TCP diagnostic only (§9.3); coupling from the manual, measured on the robot (§9.3); regression pin + sanity bound (§9.4); no calibration file → residual in the clearance budget (§9.8) |
| 5 | A→B configuration change (base ~167°, elbow −120° → +83.8°) | `movej` only, joint targets only; `movel` is never emitted |
| 6 | Check vs real motion | joint-linear dense sampling with a displacement bound; `r=0` fixed |
| 7 | Self-collision, limits, gripper | explicit exclusions, filterparent off; elbow ±180° as the precondition of the upper_arm–forearm exclusion; others ±360° (Polyscope limits possibly tighter); structural pair base–upper_arm reported, not excluded (Q9); gripper fixed at 50 mm with a pad-span hull; part beyond the pad ends not covered |
| 8 | ≤ 5 via points | planner (next feature) must report failure, never return a longer or colliding path |
| — | Open shells over-filling space | flagged per piece (§7.4) |
| — | 88 MB STL, 1.76 M triangles as visual mesh | MuJoCo's STL loader refuses > 200 000 faces → passed as `uservert`/`userface`, no hull stored, compile 1.3 s (§4.5) |

## 15. Open questions

Resolved by the owner (2026-10-17):

- TCP configured as (0, 0, 174.0, 0, 0, 0), §9.3.
- Coupling measured, 14.0 mm, §4.4.
- Regression pin + sanity bound, §9.4.
- Budget with both cases, §9.8.
- `Mount_Yaw` = 0 as a documented assumption, §9.3.
- No speed/acceleration fields, hardware ±360° limits, §8.

Rev. 4: Q8 → option (ii), pad-span hull; Q5, Q6, Q7 → defaults accepted; elbow ±180°.

**For the owner:**

- **Q9 — `base – upper_arm_link` (§4.5).** It is within 5 mm in 17.5 % of samples and never below
  2.3 mm. (`base – wrist_2_link`: 0.1 %, min 2.3 mm. It is rare enough to keep the 5 mm margin.)
  Options:
  - (a) *recommended*: pair-specific margin 0 mm (contact only) for exactly this pair, justified
    by a test sweeping `shoulder_pan` × `shoulder_lift`;
  - (b) keep 5 mm and lose ≈ 17 % of joint space;
  - (c) lower `Clearance.Self` for all pairs.

**Note for the first planner task (owner, no change now):** learned pick/drop points near fixtures
may lie inside the 20 mm margin. Approach and retreat segments must be addressed explicitly. The
milestone keeps the hard error of §9.7.

**`KNOWN_ISSUES.md` at the skeleton** (only what the skeleton decided):

1. Kinematic error was measured at two poses only (1.72 mm, 4.02 mm). It may be larger elsewhere
   along the path; at generated via points it is unmeasured (§9.8).
2. No `calibration.conf`: nominal DH.
3. The carried 50 × 50 mm part is not modelled (accepted). The pad-span hull covers the space
   between the fingers; **a part protruding beyond the pad ends is not covered** (§9.6).
4. `Tool.Mount_Yaw` = 0 is an unmeasured assumption (standard Robotiq mounting).
5. Joint limits are ±360° (elbow ±180°, the precondition of the upper_arm–forearm exclusion). The
   Polyscope safety limits may be tighter and must be checked against the robot before a path is
   run.
6. Cables at the base and the tool are not modelled; the remaining margin is 15.0 mm (case A) or
   9.0 mm (case B) (§9.8).
7. Hulls of open shells may fill space (§7.4); the pieces are listed by the build report.
8. Ubuntu install path not verified.

## 16. Sources

- Robotiq, *2F-85 & 2F-140 for Universal Robots — Instruction Manual*, revision 2020/02/11
  ("Fused sections that were exclusive to e-Series or to CB-Series"), 153 pages,
  <https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_UR_PDF_20200211.pdf>,
  sha256 `02d95c13b336c2da7449472a27c2052cd861d7098bf6105b44783ff179917030`:
  - p. 21, §3.4.1 — the coupling is mandatory; "use the coupling dowel pin to align parts";
  - p. 23, Fig. 3-2 — e-Series installation (dowel pin, coupling, connection protector);
  - p. 116, Fig. 5-1 — 2F-85 opened: 149.3 mm, pad 38 mm, Ø71 h8 / Ø75 base, finger axis X;
  - p. 117, Fig. 5-2 — 2F-85 closed: 162.8 mm;
  - p. 119, §5.1.2, Fig. 5-5 — coupling GRP-CPL-062 / GRP-ES-CPL-062: 16.9 mm, 2.9 mm, Ø71 F8, Ø75;
  - p. 123, Table 5-1 — "Maximum height 162.8 mm", fitted with coupling GRP-CPL-062;
  - p. 127, §5.2.3 — TCP 2F-85 (0.0, 0.0, 174.0) mm, "a coupling is included".
- Robotiq, *2F-85 & 2F-140 for Cobots — Instruction Manual*, revision 2021/05/19 ("Updated Center of
  mass and TCP table (section 5.2.3)"), 99 pages,
  <https://blog.robotiq.com/hubfs/support-files/2F-85_2F-140_General_PDF_20210623.pdf>,
  sha256 `31290c7c0a5acc25c3301c824ce0089306f38a3d78b6fc44db464d10eeaa3b74`:
  p. 64 Fig. 5-1 (same 149.3 mm), p. 68 Fig. 5-6 (same coupling 16.9 / 2.9 mm), p. 82 §5.2.3
  (TCP 2F-85 (0.0, 0.0, 174.0) mm, unchanged).
- Owner, 2026-10-17: Polyscope TCP of SF1 = (0, 0, 174.0, 0, 0, 0); flange face → gripper body
  measured on the robot = 14.0 mm.
- MuJoCo Menagerie, commit `62ab00f2bc44f6f20c1c9b7a13f3fb1dd41a4e35`: `universal_robots_ur10e`
  (BSD-3-Clause), `robotiq_2f85` (BSD-2-Clause).
- Universal Robots, UR10e DH parameters (d, a, α as in §9.2), from UR's published "DH parameters for
  calculations of kinematics and dynamics" table.

## 17. Implementation record (skeleton, 2026-10-17)

Measured on the development Mac (M2 Pro), Python 3.11, mujoco 3.16.0.

- **Components are split at manifold edges** (an edge of exactly two faces), as trimesh does and as
  §4.1 was measured: 2756 components. The first implementation used shared vertices (2270), then
  any shared edge (2428). With both, the stand frame was one 21 440-face component; its CoACD parts
  missed eps by 0.12 mm, so it fell back to one hull extending 297 mm beyond the frame. The robot
  base sat 0.2 mm above that hull, and the static-base check refused the scene. This was an
  implementation mismatch with §4.1, not a parameter: `Hull_Excess_Max`, `eps` and the margins were
  not changed.
- **Conservativeness check corrected (owner review, 2026-10-17).** The first build's check stopped
  at the first part within eps for each point, so its reported maximum (0.99998 mm) was an upper
  bound capped at eps, not the true maximum. The verdicts were right. The check now computes the
  exact distance to the union: numpy point-triangle distance, matching trimesh to 4e-11 m, with
  pieces skipped only when a facet-plane lower bound proves they cannot be closer. Each component
  has its own seed. Exact recomputation on the old set gave a true maximum of 0.769 mm. Example of
  the inflation: component 646 was reported 0.943 mm and is truly 0.138 mm.
  `test/Collision/test_conservativeness.py` pins the pruned computation against the exhaustive one
  and recomputes the recorded maximum.
- **Collision set (rebuilt):**
  - crop radius 2.026 m (a conservative chain bound: all 2756 components are kept);
  - 6716 pieces: 2125 hull, 503 thickened (all planar), 4087 CoACD, 1 fallback;
  - bodies: 4575 `env`, 2141 `env_stand`;
  - conservativeness: 1 601 339 points over the accepted CoACD components; true max 0.769 mm
    (component 2132); 0 points above 0.9·eps, 0 above 0.99·eps. The one rejected component, 1638
    (an open shell in object (a)), is at 1.121 mm > eps and is kept as its hull;
  - excess: max 234.8 mm (that fallback hull), p95 11.3 mm; 396 pieces flagged above 10 mm;
  - build time 1145 s (CoACD 952 s); 3.6 MB `.npz`.
  - The piece counts differ slightly from the first build: the per-component seeds changed the
    excess samples, so a few borderline components fall on the other side of 10 mm.
- **Scene:** 6785 geoms, 6751 of them collision geoms, 6716 of those environment; build 1.75 s
  without the visual mesh.
- **`Check_Clearance(q)`** over 1000 random `q` within the limits: mean 3.10 ms, p95 13.90 ms, max
  55.4 ms. 714 of the 1000 random poses violate some margin. Whether the §7.2 merge pass is needed
  is a planner-time decision; for the skeleton it is not.
- **Closest robot ↔ environment pairs at A and B:** shoulder_link ↔ stand profiles at 99.2 mm at
  both points (the base is excluded from the stand). Against `env` without the stand: A,
  upper_arm_link ↔ a column of object (a) at 125.0 mm; B, upper_arm_link ↔ the side-table plate at
  262.7 mm.
- **MuJoCo FK vs DH** over 1000 random `q`: ≤ 1e-15 m, ≤ 1e-15 rad.
- **`base – upper_arm_link` true minimum:** 2.3237 mm (grid 2.3263 mm).
- **A and B against the full environment:** no violation. *Corrected 2026-10-17: measured at `Base_Yaw` = 0. At 180° A is clear and B penetrates the rotation-station model by 27.4 mm (`KNOWN_ISSUES.md`); the closest-pair line above is also from 0°.*
- **Demo** (base ±45° around `home_safe`): 1234 joint-linear samples, every margin kept, label
  **"clearance checked"**. At 180° (2026-10-17) the same: 1234 samples, every margin kept.
