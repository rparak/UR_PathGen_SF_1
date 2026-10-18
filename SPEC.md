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

**Current goal (owner, 2026-10-17): optimise the 90° part rotation (regrasp) sequence** of the robot
program `sf1_plc_v4.script`. The station operations stay verbatim; the transfer segments between
them (S1 home → drop, S2 retreat → approach, S3 lift → home) are generated with the planner of §10.6
and compared with the taught baseline. The A → B planner below is the tool for that. The holder at
the station exists so that the robot can grip the part from the other side.

**The only deliverable (owner, 2026-10-18) is case 17 from home to home**, this taught sequence and
its optimised version:

```
movej home (r=0.01) → Waypoint_1 (r=0.05) → drop_rotate_1 → movel Waypoint_4 → [gripper open]
→ Waypoint_5 → Waypoint_1 (r=0.05) → Waypoint_2 (r=0.01) → Waypoint_3 (r=0.05) → Waypoint_18
→ pick_rotate_1 → [gripper close] → movel Waypoint_6 → Waypoint_2 (r=0.01) → home
```

Optimised: S1 (home → drop_rotate_1), S2 (Waypoint_5 → Waypoint_18) and S3 (Waypoint_6 → home)
replaced, the station operations verbatim. *Corrected 2026-10-18: the deliverables were cases 17
and 18, each as the whole case including the loading moves into and out of machine (a).* **Case 18
and the other 19 cases of the program remain as ground truth only:** the program extract (every
learned point and the whole cases 17 and 18, §10.1) and the tests that validate the model against
real program segments (the replay of cases 17 and 18, §11; the penetration reference of all 21
cases, `test/Scene/test_program.py`; the placement of the grasp points, §4.6) are kept. Nothing is planned, written or replayed for them.

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
2. A demo that rotates one joint (base, `shoulder_pan`, ±45° around `home`) as a kinematic
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

- The planner (RRT-Connect + shortcutting, ≤ 5 via points) — implemented since, §10.6.
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
committed verbatim as `Data/Programs/sf1_plc_v4/sf1_plc_v4_positions.urscript`) contains grasp and drop points
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

### 4.7 Station correction fitted to the taught program (*measured*, 2026-10-17)

*Superseded 2026-10-18 by §4.9: fitted without the machine assembly shift. Kept as the record of the
first fit.*

No measurement of the cell is available. The ground truth is the owner's statement that every
segment of `sf1_plc_v4.script` runs on the real robot without collision. At `Base_Yaw` = 180° the
rotation sequence of cases 17 and 18 (home → … → home, movel checked joint-linear, 50 mm opening
while holding, 85 mm open — assumption: full 85 mm stroke for `rq_set_pos_spd_for(0, ...)`;
conservative; owner did not confirm (the URCap sets `set_open_mm(50.0, 1)`)) came within 5 mm of two STL
components only: 2710, the left window profile (one hull piece, −59.3 mm), and 1154, the machine
body (21 pieces; only the flagged front pieces were touched, −29.2 mm).

**Fit.** For each component, the smallest rigid translation (≤ 150 mm) that keeps every segment of
the sequence ≥ 5 mm clear: 400 directions (Fibonacci sphere), bisection on the length, then 300
seeded shrinking steps (seed 20261017); distances by `mj_geomDistance` from every robot collision
geom at the segment's opening.

| Component | Translation (world) | Worst before | Worst after |
|---|---|---|---|
| 2710 (post) | (−114.8, −6.2, −0.6) mm; only directions near −x are feasible (36 of 400) | −59.3 mm | +5.3 mm |
| 1154 (machine body, whole component) | (−32.6, −11.6, +3.6) mm | −29.2 mm | +5.2 mm |

After the shift the post spans world x [−0.817, −0.727]: nearly flush with the machine front
instead of 108 mm proud of it. The real robot sweeps the post's STL volume at three heights
(station, 1.42 m, 1.7 m), so the post cannot stand there; a y shift or a z trim does not work (the
station is on one side, the gripper sweeps the other).

**All cases.** Every unique segment of all 21 cases (121), joint-linear, both openings, before and
after: no segment gets new or deeper penetration, neither in its worst value nor in its distance to
1154 or to 2710 alone; every distance to 1154 grows, and the closest distance to 2710 outside cases
17/18 falls from 67.0 to 39.9 mm (load3 → load_hower). The whole-component shift of 1154 is
therefore used, not a split of the component (owner's condition, 2026-10-17).

**Rotation sequence after the rebuild** (minimum robot – environment distance; the station zone,
§9.7, contains the post, four 1154 front pieces and the window's top profile):

| Segment | Case 17 | Case 18 |
|---|---|---|
| home → Waypoint_1 | ≥ 20 mm | ≥ 20 mm |
| Waypoint_1 → drop | −27.4 → 6.5 mm | −28.4 → 5.8 mm |
| drop → movel target (Waypoint_4 / 7) | −28.2 → 5.7 mm | −29.2 → 5.2 mm |
| movel target → Waypoint_5 | −27.7 → 6.6 mm | −29.0 → 5.3 mm |
| Waypoint_5 → Waypoint_1 | −20.0 → 14.9 mm | −20.0 → 14.9 mm |
| Waypoint_1 → Waypoint_2 | −52.6 → 15.3 mm (top profile) | −52.6 → 15.3 mm |
| Waypoint_2 → Waypoint_3 | −54.6 → ≥ 20 mm | −54.6 → ≥ 20 mm |
| Waypoint_3 → Waypoint_18 / 11 | −8.9 → 19.2 mm | −2.1 → ≥ 20 mm |
| → pick | −18.6 → 11.3 mm | −16.3 → 13.6 mm |
| pick → movel target (Waypoint_6 / 8) | −20.1 → 5.7 mm | −19.0 → 6.8 mm |
| movel target → Waypoint_2 | −59.3 → 9.5 mm | −58.6 → 12.4 mm |
| Waypoint_2 → home | ≥ 20 mm | ≥ 20 mm |

Checked as taught, every segment keeps its margins. Checked as generated (station points as the
fixed points), only Waypoint_1 → Waypoint_2 violates: the top profile at 15.3 mm with the TCP about
300 mm from every station point.

### 4.8 Owner photos and tape readings (2026-10-18)

**Photos.** Two obstacles at the rotation station are missing from the STL, both on the outer face
of the left window profile (component 2710). They are estimated from a photo, scaled by the window
height (0.757 m sill to top profile, about 2.6 mm/px, ±30 mm):

- **the holder:** a black U-shaped holder at sill height, about 80 mm wide and 140 mm tall
  (z 0.955 – 1.095 m), under the part;
- **the stub:** a short vertical profile with an orange-framed piece on top, just above the holder,
  about 60 mm wide (z 1.093 – 1.264 m), reaching about drop height.

Their depth toward the robot is not visible.

**Tape readings** (owner, rough; the owner asked that nothing be fitted to them):

| Reading | Tape | STL | After §4.9 |
|---|---|---|---|
| base axis → machine front at window height | ~700 mm | 612 mm (window profiles) | 712 mm (right profile), 728 mm (left profile, fitted) |
| robot-table edge (x −0.340) → lower cabinet front, low | ~300 mm | 127 mm | 226 mm |

### 4.9 Machine (a) shifted, post refitted (*measured*, 2026-10-18)

*Superseded later on 2026-10-18 by §4.11: the machine a further 50 mm, the post fit removed. Kept as
the record of the −100 mm state.*

**Owner facts.**

- Machine (a) stands about 100 mm further from the robot than the STL, normal to its front (world
  −x). The machine body, the front column with the window frame and the closed lower cabinet move as
  one rigid assembly. This is an estimate, not a measurement.
- `Tool.Mount_Yaw` = 0° is confirmed on the robot. On 2026-10-18 the owner first stated 90° and then
  withdrew it.
- The release opening of 85 mm is real: the part is released into a magazine with free space below
  and above.

**Selection.** Grouping components by touching bounding boxes (1 mm) gives exactly three objects:

- (a) the machine, 1749 components, x −1.609 … −0.466 m;
- (b) the robot stand, pallet and side tables, 951 components;
- (c) the side cabinet, 56 components, y ≥ 0.555 m.

Object (a) is the assembly (`machine_a`, the object of component 1154). Its front strip is its 232
components ahead of x = −0.70 m: posts 2709/2710, top members 2711/2699, the cabinet panel 180 and
frame 2696 – 2708. The owner confirmed the selection on renders. Both fits of §4.7 are replaced.

**Post refit.** With the shift alone, the taught sequence penetrates the post 2710 by 10.4 mm. The
smallest further translation of 2710 that keeps every segment of cases 17 and 18 at least 5 mm clear
(exact distances, every sample, each segment at its opening):

- **15.7 mm along −x** (bisection, 0.15 mm), configured: it is the machine front's normal;
- 15.5 mm in the free direction, (−15.3, −2.6, +1.2) mm, among 400 directions.

The post totals 115.7 mm behind its STL position, consistent with the first fit (−114.8, −6.2,
−0.6 mm, §4.7). Its front is at x = −0.728, 74 mm ahead of the shifted body front; the STL draws it
90 mm ahead. The machine body 1154 needs nothing beyond the shift: the closest taught sample of
cases 17 and 18 is 60.3 mm from it.

**Rebuild** (`machine_a` −100 mm, 2710 a further −15.7 mm): 6716 pieces, every count, method and body
as before; conservativeness max 0.769 mm; excess max 234.8 mm, p95 11.27 mm (was 11.30 mm); 398
pieces flagged (was 396: borderline excess samples of translated components); 1109 s. The manifest
records both overrides and the 1749 members of the assembly.

**All 21 cases** (125 unique segments, joint-linear, 50 and 85 mm, against the 2026-10-17 state):

- no segment gets new or deeper penetration;
- penetrating segments: 63 before, 63 after;
- the machine-interior penetrations shrink by about 9 mm (deepest: Waypoint_19 → bench_drop_8,
  −154.1 → −145.0 mm, component 1638);
- no self-collision.

Cases 17 and 18 as taught (minimum distance, mm, its own opening per segment; all to the post 2710
or nothing within 50 mm):

| Segment | Case 17 | Case 18 |
|---|---|---|
| Waypoint_1 → drop | 6.5 | 5.7 |
| drop → Waypoint_4 / 7 | 5.7 | **5.1** |
| Waypoint_4 / 7 → Waypoint_5 (85 mm) | 7.2 | 6.0 |
| Waypoint_5 → Waypoint_1 | 21.3 | 21.3 |
| Waypoint_1 → Waypoint_2 | ≥ 50 | ≥ 50 |
| Waypoint_2 → Waypoint_3 | 30.8 | 30.8 |
| Waypoint_3 → Waypoint_18 / 11 | 36.2 | 42.1 |
| approach → pick | 15.9 | 17.0 |
| pick → Waypoint_6 / 8 | 10.0 | 10.3 |
| Waypoint_6 / 8 → Waypoint_2 | 11.5 | 13.3 |

Every segment keeps its margins as taught and keeps the generated rule too: wherever it comes within
20 mm of the station, its TCP lies within 50 mm of a station point. Before the shift, Waypoint_1 →
Waypoint_2 passed the window's top profile at 15.3 mm far from the station; the top profile moved
with the machine.

**The station zone stays** (5 mm). The drop and release points keep 5.5 – 7.4 mm, by construction
of the post fit, below the global 20 mm. The zone now holds 12 pieces (14 before the shift).

**Committed S1 – S3** (generated rule): all six pass unchanged. They are not re-planned, and
`Data/Generated/` stays as committed. They keep at least 181.2 mm to the machine-interior and pallet
pieces.

### 4.10 Station holder box (*measured*, 2026-10-18)

The holder of §4.8 is one world-axis-aligned box, `Environment.Added_Boxes` `station_bracket`
(exact distances, every taught sample of cases 17 and 18 at its opening, the release included):

- x −0.735 … −0.645 m: from the post face (x −0.728, the box starting 8 mm inside the post) to
  90 mm in front of it, an assumed depth;
- y −0.340 … −0.260 m and the bottom at 0.955 m: from the photo;
- **top 1.112 m:** the highest at which every taught sample keeps 5 mm (1.1126 m, the open finger).
  That is 17 mm above the photo's estimate of 1.095 m, where the clearance is 21.1 mm. The box keeps
  5 mm from the taught release, where the magazine has free space below and above the part.

The committed S1 – S3 keep the generated rule with the box, and within 20 mm of it no generated
sample comes closer than its segment's closer taught end.

**The stub is not modelled** (owner, 2026-10-18). At its estimated position (y −0.330 … −0.270,
z 1.093 … 1.264 m) the most a box could stand proud of the post face is 0.3 mm: flush with the
face, the taught drop's holding pad already passes at 5.3 mm. With such a plate, case 17 S2 would
come 1.2 mm closer to it than its taught end. The photo's stub therefore contradicts the taught drop
at its estimated position; its real position is unknown (`KNOWN_ISSUES.md`).

### 4.11 Machine (a) at −150 mm, side cabinet −50 mm, post fit removed (*measured*, 2026-10-18)

**Owner facts (estimates, not measured).** Machine (a) stands a further 50 mm along −x: 150 mm in
all, at the 150 mm bound of an override. **Owner's judgement (2026-10-18): the −150 mm position
matches the real cell better than −100 mm**, and generated paths may use the space it frees (the
re-planned S1 – S3 of §17 are kept). It remains an estimate, not a measurement; its accuracy is not
yet stated. The side cabinet, object (c) (56 components, named by its
body 2643), stands 50 mm along −x, a separate assembly with its own source. The robot stand and the
tables, object (b), stay. The holder stays where the robot places the part (world-fixed); it is
mounted on the post, so its back is extended to the shifted post face.

**Rebuild** (`machine_a` −150 mm, `side_cabinet` −50 mm, no component override): 6718 pieces (6716),
by method hull 2125, thickened 503, CoACD 4089, fallback 1; conservativeness max 0.769 mm
(unchanged); excess max 234.8 mm, p95 11.27 mm; 396 pieces flagged (398); 1119 s. The manifest
records both assemblies, their 1749 and 56 members, and no component override. Every number below
was first measured by translating the committed pieces in scratch and then again on the rebuilt
set; the two agree to 0.1 mm.

**Post 2710: the fit is removed.** With the machine at −150 mm the post alone keeps **≥ 39.1 mm** to
every taught sample of cases 17 and 18 (case 18 drop_rotate_2 → Waypoint_7; at −100 mm with its fit:
5.1 mm), so the smallest extra translation that keeps 5 mm is zero. The post now stands 34.3 mm
behind its earlier fit (−115.7 mm), its face at x −0.762; the program no longer constrains it, so
that position rests only on the owner's estimate.

Cases 17 / 18 as taught (minimum distance, mm, its own opening per segment; ≥ 60 not listed exactly):

| Segment | −100 mm, post fitted (§4.9) | −150 mm, no fit |
|---|---|---|
| Waypoint_1 → drop | 6.5 / 5.7 | 40.4 / 39.8 (post) |
| drop → Waypoint_4 / 7 | 5.7 / 5.1 | 39.5 / 39.1 (post) |
| Waypoint_4 / 7 → Waypoint_5 (85 mm) | 7.2 / 6.0 | 40.6 / 39.8 (post) |
| Waypoint_5 → Waypoint_1 | 21.3 / 21.3 | 50.0 / 50.0 (post) |
| Waypoint_2 → Waypoint_3 | 30.8 / 30.8 | ≥ 60 |
| Waypoint_3 → Waypoint_18 / 11 | 15.1 / 27.2 (holder) | 15.1 / 27.2 (holder) |
| approach → pick (85 mm) | 5.5 / 14.4 (holder) | 5.5 / 14.4 (holder) |
| pick → Waypoint_6 / 8 | 10.0 / 10.3 (post) | 17.8 / 27.8 (holder) |
| Waypoint_6 / 8 → Waypoint_2 | 11.5 / 13.3 (post) | 45.0 / 47.6 (post) |

Every segment keeps its margins as taught and keeps the generated rule. **Only the holder still
comes within 20 mm** of a taught sample; every STL piece keeps ≥ 39.1 mm, and every station point
keeps ≥ 39.1 mm to every STL piece. To the holder: pick_rotate_1 open 5.5 mm, pick_rotate_2 open
14.4, Waypoint_18 open 15.1, pick_rotate_1 holding 17.8 mm.

**Holder.** Its taught clearance is unchanged, 5.5 mm (case 17 Waypoint_18 → pick_rotate_1, the open
finger). With the post face at x −0.762 the earlier box (back at x −0.735, 8 mm into the fitted post)
would leave a 27 mm gap to the post; its back is extended to x −0.770 (8 mm into the shifted face),
at no clearance cost (5.5 mm either way). The highest top at which every taught sample keeps 20 mm
would be 1.0962 m; the photo's 1.095 m gives 21.1 mm (owner's choice: the fitted 1.112 m stays).
*Re-measured 2026-10-18 with the added boxes as convex meshes (§8): 5.53 mm, 21.05 mm, the 5 mm top
1.1125 m and the 20 mm top 1.0961 m - unchanged within the bisection step; the exact distances were
right, only MuJoCo's box-box contacts were missing.*

**Stub** (§4.10, not modelled). At its estimated position (y −0.330 … −0.270, z 1.093 … 1.264 m) on
the shifted post face, a box could stand up to 34.6 mm proud of it and every taught sample would
keep 5 mm (closest: case 17 pick_rotate_1 → Waypoint_6, the holding pad). The argument of §4.10 that
the stub contradicts the taught drop held only for the fitted post.

**Station zone** (box unchanged in this step): 8 pieces, components 180, 181, 1065, 2699, 2701, 2702,
2704, 2710, plus the holder; the machine body 1154 has left it (12 pieces before). Shrunk in §4.12.

**All 21 cases** (`test/Scene/test_program.py`, 125 unique segments, 50 and 85 mm): no segment gets
new or deeper penetration against the −100 mm reference; 63 penetrating segments before and after;
the machine-interior penetrations shrink by up to 25.3 mm (load2 ↔ load3 −99.5 → −74.2 mm; deepest
Waypoint_19 → bench_drop_8 −145.0 → −140.4 mm), the grasp contacts not at all; no self-collision.
No segment comes within 50 mm of the side cabinet (c), before or after: the program neither confirms
nor contradicts its shift. The reference is re-recorded at −150 mm.

**Placement** (§4.6): unchanged; the bench_drop points lie at x ≈ −1.12 m, inside machine (a) both
where the STL draws it and shifted. `load3` → 1638: −74.1 mm (−99.5 mm at −100 mm).

**Committed S1 – S3** (case 17): all three still pass `Verify_Path` on the new scene. S3's direct
segment now keeps the generated rule (it did not at −100 mm).

### 4.12 Station zone shrunk to the holder's neighbourhood (*measured*, 2026-10-18)

**Owner decision (2026-10-18), option S.** After the shift the program no longer constrains the
post (≥ 39.1 mm, §4.11), so the post's extra 34 mm beyond its earlier fit (−115.7 mm) rests only on
the owner's estimate; a generated path must keep the full 20 mm from it. The zone (§9.7) therefore
holds only the holder, the one geometry the taught program comes closer to than 20 mm (5.5 mm).

**Box** `[-0.755, -0.36, 0.94]` – `[-0.62, -0.24, 1.13]` m (was `[-0.80, -0.40, 1.00]` –
`[-0.55, -0.20, 1.85]`): the holder box and the sill-frame pieces it sits on, components 181, 1062,
2701, 2702, 2704 (5 pieces; 8 before, with the post 2710, the top profile 2699 / 1065 and the
machine front 180). No taught sample comes within 20 mm of a sill-frame piece; the post's face
(x −0.762) lies outside the box.

**Checks.** With the holder removed, every taught segment of cases 17 and 18 keeps the global
margins everywhere (the generated rule without station points): the zone is needed for the holder
only. With the shrunk zone the taught sequence keeps its margins as taught and as generated. Every
taught sample under 20 mm has its TCP within 13.5 mm of a station point (44 mm before the shift), so
`Endpoint_Radius` (50 mm) stays. The committed S1 – S3 still pass `Verify_Path`.

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
Data/Programs/sf1_plc_v4/                     extracts of the owner's program, verbatim, with its sha256:
                                              its 156 global _p/_q lines, and the whole cases 17 and 18
                                              (§10.1, ground truth); the simplified taught program of
                                              case 17 home → home, sf1_plc_v4_case_17.urscript +
                                              _sequence.script (§10.4)
models/MANIFEST.json                          Menagerie commit, licence, sha256 of every vendored file
models/universal_robots_ur10e/                xml, assets/, LICENSE, README.md, CHANGELOG.md (no png)
models/robotiq_2f85/                          same
src/Configuration/Core.py   Load_Configuration → Cell_Configuration_Str (the only YAML reader)
src/URScript/Core.py        Read_URScript, Write_URScript, Write_Movej_Sequence, Write_Motion_Sequence, Program_Entries,
                            Rotation_Sequence (the only URScript reader/writer)
src/Kinematics/Core.py      UR10e DH FK, rotvec ↔ matrix, Geodesic_Angle, Normalize_Rotvec, Derive_TCP_Offset
src/Collision/Core.py       Build_Collision_Set, Write_Collision_Set, Read_Collision_Set (the only npz/manifest reader/writer)
src/Scene/Core.py           Build_Scene (MjSpec), Set_Configuration, Check_Clearance, the taught / generated rule
src/Planner/Core.py         Planner_Cls (direct, RRT-Connect, shortcut, refinement), Verify_Path, duration estimates
Example/plan_rotation.py    plans S1 - S3 of case 17, writes Data/Generated/, prints the report
Example/replay_rotation.py  viewer replay of a numbered catalogue of taught and optimised paths (§10.7); screenshots
Data/Generated/             SF1_Rotation_17.urscript (points) and SF1_Rotation_17_Sequence.script (home → home)
Example/write_taught_programs.py  writes the simplified taught program of case 17 home → home (Data/Programs/sf1_plc_v4/)
Example/build_collision.py  regenerates CAD/Collision/ from the STL (runs CoACD; not in the test suite)
Example/demo_rotate_axis.py the viewer demo
Example/demo_visualize_position.py  the robot static at one learned position, clearance by the taught rule (§10.7)
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

**Assembly overrides.** `Environment.Assembly_Overrides` translates a whole STL object (§4.1: the
components whose bounding boxes touch within 1 mm), named by one of its components, by one rigid
translation (≤ 150 mm) with its source. A component override adds its own translation on top, each
entry bounded on its own. `Component_Translations` computes the translation of every component, for
the build and for the tests. The manifest records the assembly overrides (`assembly_overrides`) and
their members (`assembly_components`); the scene refuses a set whose record differs from the
configuration, as for component overrides.

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
| `Environment.Assembly_Overrides` | rigid translations of whole STL objects before the build: `{Name, Object_Of (a component of the object), Translation [x, y, z] (world, ≤ 150 mm), Source}` each; may be empty (§7.6). Shipped: `machine_a`, object (a), −150 mm along x, and `side_cabinet`, object (c), −50 mm along x (owner estimates, 2026-10-18, not measured; §4.11) | –, –, m, text | REQUIRED |
| `Environment.Component_Overrides` | rigid translations of STL components before the build, added to any assembly translation: `{Component, Translation [x, y, z] (world, ≤ 150 mm), Source}` each; may be empty (§7.6). Shipped: none (the post fit of §4.9 was removed, §4.11) | –, m, text | REQUIRED |
| `Environment.Added_Boxes` | obstacles missing from the STL, added to body `env` as convex meshes of their eight corners (world-axis-aligned boxes; not box primitives, whose contacts with the gripper's box pads are missed inside the margin) with the environment margin: `{Name, Min [x, y, z], Max [x, y, z] (world), Source}` each; a box meeting `Clearance.Station_Zone` is a zone piece; may be empty. Shipped: `station_bracket`, the holder (§4.10; its back at the shifted post, §4.11). The collision set is not rebuilt for them | –, m, m, text | REQUIRED |
| `Robot.Model` | `models/universal_robots_ur10e/ur10e.xml` | path | REQUIRED |
| `Robot.Base_Position` | Polyscope base origin in the STL/world frame: `[0.0, 0.0, 0.7152]` | m | REQUIRED |
| `Robot.Base_Yaw` | rotation of the Polyscope base frame about world z: `180.0` (owner, 2026-10-17; confirmed by the taught points of `sf1_plc_v4.script` and photos of the cell, §4.6). *Corrected 2026-10-17: was `0.0`, which put every taught grasp point in empty space* | deg | REQUIRED |
| `Robot.Joint_Position_Limits.Min/Max` | joint position limits, 6 each. Shipped: ±360° for base, shoulder, wrists; **±180° for the elbow** (Menagerie default, owner). Load rejects: any joint beyond ±360°, and **the elbow beyond ±180°**, because that limit is the precondition for excluding `upper_arm_link–forearm_link` (§9.6). The Polyscope safety limits may be tighter (`KNOWN_ISSUES.md`) | deg | REQUIRED |
| `Tool.Model` | `models/robotiq_2f85/2f85.xml` | path | REQUIRED |
| `Tool.Coupling.Thickness` | coupling thickness: `0.0169` (manual rev. 2020/02/11 p. 119, Fig. 5-5) | m | REQUIRED |
| `Tool.Coupling.Diameter` | coupling outer diameter: `0.075` | m | REQUIRED |
| `Tool.Coupling.Spigot_Depth` | depth of the Ø71 bore receiving the gripper: `0.0029`; gripper datum = thickness − depth = 14.0 mm (measured on the robot) | m | REQUIRED |
| `Tool.Mount_Yaw` | rotation of the gripper about the flange z axis relative to the Menagerie default: `0.0`, the fingers open along flange ±x. **Confirmed on the robot by the owner (2026-10-18)**; a statement of 90° made that day was withdrawn (§9.3) | deg | REQUIRED |
| `Tool.Opening` | inner pad-face gap of a scene that names no program case (the demo, the tests): `0.050`, the 50 × 50 mm part (owner) | m | REQUIRED |
| `Tool.Holding_Openings` | inner pad-face gap while the part of a program case is held, `{Case, Opening, Source}` each, used by planning, verification and replay of that case: case 17 `0.050` (owner, 2026-10-18: the 50 × 50 mm part), the only deliverable (§1). *Corrected 2026-10-18: case 18 `0.050` was listed, an assumption.* Load rejects: an empty list, an opening ≤ 0 or above the 85 mm stroke, an empty source, a case named twice; a script refuses a case without an entry | m | REQUIRED |
| `Tool.TCP_Offset` | TCP in the flange frame `[x, y, z, rx, ry, rz]`: `[0.0, 0.0, 0.174, 0.0, 0.0, 0.0]`. Source: the Polyscope TCP of SF1 (owner, 2026-10-17), identical to the Robotiq value (manual rev. 2020/02/11 p. 127, §5.2.3) | m, rad (rotvec) | REQUIRED |
| `Clearance.Environment` | minimum robot/gripper ↔ environment distance: `0.020` | m | REQUIRED |
| `Clearance.Self` | minimum distance between non-excluded robot/gripper bodies: `0.005` | m | REQUIRED |
| `Clearance.Self_Pair_Overrides` | `[[base, upper_arm_link, 0.0]]` (Q9 (a)): a pair-specific self margin. Allowed only for a pair whose true minimum stays > 0: grid over the joints that move it, then scipy local minimisation from the 20 lowest cells; the test checks every entry and pins the refined minimum | m | REQUIRED |
| `Clearance.Station_Zone.Min/Max` | world box of the station zone: environment pieces whose bounding box meets it are the zone pieces (§9.7). Shipped: the holder's neighbourhood, `[-0.755, -0.36, 0.94]` – `[-0.62, -0.24, 1.13]` (owner, 2026-10-18, §4.12) | m | REQUIRED |
| `Clearance.Station_Zone.Environment` | margin of the zone pieces where the zone applies; > 0 and ≤ `Clearance.Environment` | m | REQUIRED |
| `Clearance.Station_Zone.Endpoint_Radius` | a generated sample uses the zone margin only while its TCP lies within this distance of a fixed station point's TCP (§9.7) | m | REQUIRED |
| `Clearance.Workspace.Min/Max` | world box a generated segment keeps every robot collision geom inside: x/y the STL's extents, z max 2.35 m, the highest point of the taught rotation sequence (2.322 m) rounded up — space proved free by the taught program (§9.7) | m | REQUIRED |
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
6. **Mounting orientation, `Tool.Mount_Yaw` = 0: confirmed by the owner (2026-10-18).** The manual
   indexes the coupling with the dowel pin next to the wrist connector (p. 21, p. 23 Fig. 3-2) but
   states no angle; the scene uses the Menagerie default orientation, and the fingers open along
   flange ±x (§4.5). Until 2026-10-18 this was a documented assumption; a statement of 90° made that
   day was withdrawn. The taught program cannot discriminate the yaw (a consistent station fit
   exists at 0° and at 90°), so it is configured, never fitted.

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
- **Station zone (taught vs generated).** The zone pieces (§8 `Clearance.Station_Zone`: the
  collision-set pieces and the `Environment.Added_Boxes` whose bounding box meets the zone box) keep the
  global geom margin, so MuJoCo still generates their contacts below 25 mm; `Check_Clearance(q,
  station_zone=True)` filters a contact with a zone piece by the zone margin instead. `Check_Segment`
  takes the kind of segment:
  - **taught** (a segment of the robot program): the zone margin on every sample;
  - **generated** (a planner's segment; the default): the global margins everywhere, except on a
    sample whose TCP lies within `Endpoint_Radius` of the TCP of a fixed station point
    (`Station_Zone_Active`), where the zone margin applies. The station geometry is fitted, not
    measured, so a new path may not use it more tightly than the taught program does. In addition,
    every robot collision geom (mesh vertices, primitive box corners) stays inside
    `Clearance.Workspace`: the space outside the STL is not modelled, and the taught program proves
    free only what it uses.

  `Check_Configuration(q, kind, station_tcps)` is the rule of one sample; `Check_Segment` applies it
  to every sample and returns every violation; `Segment_Valid` applies it in van der Corput order
  (ends, middle, quarters, …) and stops at the first violation. Same samples, same rule: the verdict
  equals "`Check_Segment` finds nothing", pinned on seeded near-margin segments.

  **Station-box rule** (generated segments; owner, 2026-10-18). The station boxes (the
  `Environment.Added_Boxes` that are zone pieces) are fitted to the taught program, not measured, so
  a generated path may not use them more tightly than the taught program does: no sample of a
  generated segment comes closer to a station box than the closer of its two taught ends, both
  capped at `Clearance.Environment` (`Station_Box_Floors(q_a, q_b)`; `Check_Configuration`,
  `Check_Segment` and `Segment_Valid` take the floors as `box_floors`). A violation names the box
  and the floor as its margin. The distances are MuJoCo's contacts, which agree with
  `mj_geomDistance` for these meshes (`test/Scene/test_margin.py`).

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
- Polyscope safety limits possibly tighter than ±360°.

Self-collision (5 mm) is not affected by cables or calibration at the gripper. Only eps applies.

**Inside the station zone** (§9.7) the environment margin is 5 mm: eps (1 mm) + the measured
kinematic error at the drop points (4.0 mm) leaves **0 mm for cables**, and case B (10 mm) is **not
covered**. The zone holds only the holder and the sill frame it sits on (§4.12); the holder's top was
fitted to keep the taught segments 5 mm clear of the nominal-DH robot (§4.10), so the zone margin
pins consistency with the taught program, not real clearance. A generated path gets it only within
50 mm of a station point.

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

**Program excerpt** (`Data/Programs/sf1_plc_v4/sf1_plc_v4_cases_17_18.script`, read by
`Read_Program_Excerpt` only): lines of the robot program kept verbatim, indentation included: the
whole body of each case's branch `elif (n == switch_1):` (case 17 lines 3397 – 3478, case 18
3480 – 3561), out of machine (a), the rotation sequence home → … → home, back into machine (a).
*Corrected 2026-10-18: it held only the rotation sequence.* Column-0
`#` lines before the first block are the header (source, sha256); a column-0 `# case <n>:` line opens
a case. Each program line is classified as a movej to a learned point
(`movej(get_inverse_kin(<name>_p, qnear=<name>_q), …)`), a movel to a learned pose, a gripper
action (`rq_set_pos_spd_for(0 | 255, …)`: open / close) or other (URCap boilerplate, Polyscope
markers). Strict: an unknown gripper position, a repeated or empty case is an error. The extracts are
compared with the program, byte for byte, when `SF1_PLC_V4_SCRIPT` names it; otherwise that test is
skipped.

### 10.2 Encoder noise

`2.44E-5` rad in `home_q` is encoder noise from teaching. It is read and written unchanged.

### 10.3 Naming convention

Learned points keep the program's names. Their single source is the program extract
`Data/Programs/sf1_plc_v4/sf1_plc_v4_positions.urscript` (§10.1): A = `home`, B = `drop_rotate_1`.
Generated via points are `via_<from>_<to>_<nn>`, `nn` from `01`, e.g. `via_home_drop_rotate_1_01`.
The original robot export of A and B is kept as the test fixture
`test/URScript/data/SF1_Positions_original.urscript`; a test asserts that its number tokens equal
the extract's.

*Corrected 2026-10-18: the project names `home_safe` / `drop_rot_01` and the file
`Data/Positions/SF1_Positions.urscript` that held them are retired; one source of learned points,
under the robot's names.*

### 10.4 Motion output (requirement B)

```
movej(<name>_q, a=1.3962634015954636, v=1.0471975511965976, r=0)
```

*Corrected 2026-10-17 (owner):* `a` and `v` are written, as the tokens of the robot program
`sf1_plc_v4.script`, so a generated move runs at the speed the duration estimate assumes (§10.6).
Speed does not change the path of a `movej` with `r=0`. `r=0` is written explicitly because it is
the one argument that does change the path. Joint targets only, never `movej(p[...])`. `_p` of via
points is informational: the robot would otherwise solve its own IK and could pick another branch.

**Simplified taught program** (`Example/write_taught_programs.py`,
`Data/Programs/sf1_plc_v4/sf1_plc_v4_case_17.urscript` and `…_case_17_sequence.script`): the rotation
sequence of case 17, home → … → home (`Rotation_Sequence`: from the case's first movej to home to its
second, inclusive), as its motion program, written by `Write_Motion_Sequence` from `Program_Entries`:
every movej and movel in program order, verbatim without indentation (targets, a, v, r), each
gripper action as a `# Gripper open/close` comment naming the program line, nothing else (no URCap
code, no PLC I/O, no waits). The points file holds the points it moves to, with the program's
tokens. *Corrected 2026-10-18: the files held the whole case, loading moves included, for cases 17
and 18.*

**Exception: station lines copied verbatim.** In a motion sequence (`Write_Motion_Sequence`) the
station operations of the robot program are copied verbatim (without indentation), with their own
`movel(...)` and `movej(get_inverse_kin(<name>_p, qnear=<name>_q), …)` and their a, v, r. They are
the owner's tested program, not generated output; the gripper nodes are marked by comments.

### 10.5 Joint branches (requirement C)

Joint values are never wrapped into [−π, π]. UR joints span ±2π and `movej` goes to the exact
value. The planner and writer stay on the branch of A and B. A test feeds A and B through parse →
path (the milestone's trivial path [A, B]) → write and asserts the tokens are unchanged, plus a
synthetic point with a joint at 4.0 rad that must come out as 4.0.

### 10.6 Planner and the rotation-sequence output

*Since the machine shift (2026-10-18) the taught S1 – S3 all keep the generated rule; S1 – S3 are
optimised for time (home → home 23.37 → 16.66 s, estimate), not for clearance, and the paths
written on the final scene use the space the estimated shift frees (§17).*

**Segments** (case 17, §1). S1 home → drop_rotate_1 (holding: case 17's `Tool.Holding_Openings`
entry, 50 mm), S2 Waypoint_5 → Waypoint_18 (open, 85 mm: the full stroke for `rq_set_pos_spd_for(0, ...)`; the
release opening is real, owner 2026-10-18, §4.9), S3 Waypoint_6 → home (holding). The ends are the
program's `_q` tokens, never wrapped. The fixed station points of the generated rule (§9.7) are case
17's six: drop_rotate_1, Waypoint_4, Waypoint_5, Waypoint_18, pick_rotate_1, Waypoint_6. *Corrected
2026-10-18: both cases' eleven, while case 18 was a deliverable.*

**Planner** (`Planner_Cls.Plan`):
1. Both ends must keep the generated rule (§9.7), else `Planner_Error_Cls`.
2. The direct segment, if it keeps the rule.
3. Otherwise each of 20 restarts (generator `[seed, restart]`) runs RRT-Connect in joint space within
   the joint limits (elbow ±180°): step 0.35 rad (max-joint), samples half from the box around both
   ends expanded by 1 rad, half from the whole range, ≤ 3000 extensions; then 150 shortcut
   attempts.
4. The two fastest paths are refined (150 attempts each, generator `[seed, 1000 + rank]`): a via
   point is removed or moved by a shrinking random step; a change is kept only if it lowers the
   cost and both touched segments keep the rule.
5. The lowest cost wins, ties going to fewer via points, then to the earlier restart. More than 5 via
   points, or no path, raises `Planner_Error_Cls`.

Edges use `Segment_Valid` (early exit, same verdict as the full check) with the station-box floors
of the segment's two ends (§9.7), set by `Plan`; `Verify_Path` computes the floors of the path's own
ends. **Every budget is an
iteration count**, never a time: the output depends on the inputs and the seed only, and two runs
are byte-identical. Wall-clock time is reported, never used to stop.

**Objective.** Minimise the estimated duration plus `w` × the clearance penalty; tie-break on fewer
via points.
- Estimated duration: the sum over segments of the trapezoid (or triangle) profile of the leading
  joint from rest to rest, a = 1.3962634015954636 rad/s², v = 1.0471975511965976 rad/s (the
  program's), r = 0. **An estimate**: the controller's profile, blends and the gripper are not
  modelled.
- Clearance penalty: the sum over the samples of every segment of max(0, 30 mm − d), d the smallest
  robot – environment distance (`Environment_Distance`), for samples whose TCP lies outside the
  station neighbourhood. The hard margins are unchanged; the penalty only prefers clearance above
  them. `w` = 0.5 s/m (owner, 2026-10-17, after the trade-off in §17), set in `Example/plan_rotation.py`.

**Final verification.** `Verify_Path` re-checks every segment with the full generated-rule check
(`Check_Segment`, no early exit, no cache), at most 5 via points, within the joint limits; a path
that fails is never written.

**Output** (`Example/plan_rotation.py`, `Data/Generated/`):
- `SF1_Rotation_17.urscript`: the learned points the optimised sequence moves to, with the
  program's tokens, in the order of first use, then the via points `via_<from>_<to>_<nn>` (§10.1
  format, `_p` informational).
- `SF1_Rotation_17_Sequence.script`: **case 17 home → … → home** in program order, in the form of the
  simplified taught program (§10.4), so the two compare line by line: the station lines copied
  verbatim, the gripper nodes as comments, S1 – S3 as generated movej (§10.4) to their via points
  and ends (`Rotation_Entries`). The program's move to home blends into S1 (`r=0.01`) and a blend
  into a generated segment is not checked (§8.5), so the start is written as the generated
  `movej(home_q, a=…, v=…, r=0)` with a comment (owner, 2026-10-18). *Corrected 2026-10-18: the file
  held the whole case, the loading moves out of and back into machine (a) verbatim, for cases 17
  and 18.*
- The script prints, per segment, taught vs optimised: via points, estimated duration, Σ|Δq|,
  max-joint length, the smallest robot – environment distance and its robot body, the generated-rule
  violations, and the home → home totals (station operations estimated with the same model, movel
  on the TCP path at 1.2 m/s², 0.25 m/s). Every output says simulation passed ≠ safe on the robot.

### 10.7 Viewers

All viewer scripts build the scene from the collision geometry only and start with geom group 3
shown; `--with-stl` loads the cell STL as well. Learned points come from the program extract only.

**`Example/demo_visualize_position.py`** shows the robot static at one learned position: `--pos
NAME` or `NAME_q` (a `NAME_p` is refused: the viewer shows joint positions, the robot's own IK would
decide the joints of a pose; an unknown name is refused), `--list` prints every name, `--opening
{holding, open}` (default holding; holding is case 17's `Tool.Holding_Openings` width). It prints and overlays the clearance under the taught rule, listing every
violating pair.

**`Example/replay_rotation.py`** replays one path of a numbered catalogue in one viewer window,
each case 17 home → home: 1 taught (`Data/Programs/sf1_plc_v4/`), 2 optimised (`Data/Generated/`),
3 taught then optimised with the strict comparison. *Corrected 2026-10-18: 1 – 4 were the whole
cases 17 and 18, 5 and 6 their rotation sequences.* `--path N`
selects, without it the catalogue is printed and a number asked; `--list` prints it. Read through
`Read_Motion_Sequence`, the one reader of the motion-sequence format.
- **Motion.** The path starts at its first target, home. A movej is joint-linear; a movel moves the TCP on the straight line to the DH TCP of its
  target's `_q` (orientation turning about one axis), solved for joints step by step from the
  previous solution (`Linear_Path`, `Solve_IK_Near`, steps ≤ 5 mm / 1°) and required to end at that
  `_q`. Blends are not replayed: every move stops at its target.
- **Time.** Each move plays for its estimated duration / `--speed` (> 0; 1.0 = estimated real time):
  the trapezoid of §10.6 with the line's own a and v (the leading joint for a movej, the TCP path for
  a movel), position along it by `Profile_Fraction`. The gripper's time and the program's waits are
  not modelled.
- **Gripper.** The path starts at home holding the part (owner, 2026-10-18: the part is held from the
  start of the case through the loading moves to the release at the magazine, and again from the
  re-pick back home; the gripper is open only between the release and the re-pick, S2); the opening
  switches between the holding width and open (85 mm) at the gripper actions. The window shows the
  holding scene; the open state is shown by its finger joints, the pad-span hull hidden. Every check
  uses the scene of the move's own state.
- **Labels.** Each move is checked by the rule of its kind: generated movej by the generated rule,
  program lines by the taught rule; a movel along its TCP line. Entry 3 also prints the strict
  comparison: S1 – S3 of both variants judged by the generated rule. `--skip-check` replays without
  the check (labelled so). The overlay shows the path, the variant, the move and its section, the
  gripper state and width, the estimated elapsed time and the label.

**Machine (a) interior.** A configuration that comes within `Clearance.Environment` of the interior
hull of machine (a) (component 1638, `KNOWN_ISSUES.md`) is labelled *model not valid here*, never as
a collision: the hull fills the space the loading moves use (`Scene_Cls.Components_Within`). The
replayed home → home paths stay far from it; the position viewer shows the label at the loading points.

## 11. Tests of the first milestone

| Test file | What it pins |
|---|---|
| `test/URScript/test_core.py` | exact float values of all 24 numbers; byte-identical round trip; `.238`, `-.175`, `E-5` tokens; strict errors (malformed, missing pair, duplicate, wrong length, line number in message); generated-point format; no wrapping (4.0 rad); `movej` line uses `_q` and `r=0`; renamed file tokens == original export; the case extract reads in program order (the whole case), equals its source lines 3397–3478 / 3480–3561 — the whole body of each case's branch — when `SF1_PLC_V4_SCRIPT` is set (otherwise skipped), and a malformed excerpt is refused; `Program_Entries` keeps movej/movel verbatim, comments the gripper and drops every other line; `Rotation_Sequence` returns case 17 from its first to its last move to home (typed) and refuses one or three; `Read_Motion_Sequence` reads program movej/movel, generated movej, gripper comments with a, v, r and sections, the header naming no section, and refuses any other line by number; the `sf1_plc_v4` positions extract is read strictly with every token, and equals its source lines 2453–2608 byte for byte with the recorded sha256 when `SF1_PLC_V4_SCRIPT` names the program (otherwise skipped, never passed) |
| `test/Configuration/test_core.py` | shipped file loads; each REQUIRED field missing → named failure; invalid values (wrong length, limits outside ±360°, min ≥ max, coupling spigot depth ≥ thickness, negative clearance or sanity bound, an assembly or component override beyond 150 mm, without a source or named twice, an added box with Min ≥ Max, without a source or named twice, an empty list of added boxes accepted, a zone margin above `Clearance.Environment`, a zone box with Min ≥ Max, a zero endpoint radius) fail; all problems listed at once; unknown key → WARNING |
| `test/Kinematics/test_core.py` | `Solve_IK_Near` returns, from 0.05 rad away, the configuration of 20 seeded targets (1e-9 m / rad, same branch) and refuses a target 3 m away; `Linear_Path` of the drop and lift movel keeps every TCP on the straight line (1e-9 m) in order, ≤ 5 mm apart, turning evenly, from the start's `_q` to the target's `_q` exactly; `FK_DH(q) · TCP_config` vs `_p`: regression pin (§9.4) of 1.72 mm / 0.261° and 4.02 mm / 0.376° (±0.01 mm / ±0.001°); diagnostic derived TCP pinned the same way and never read by any check; both points within the sanity bound; `Check_Learned_Point` rejects a point beyond it (perturbed `_q`) with the numbers in the message; configured TCP == (0, 0, 0.174, 0, 0, 0); geodesic angle 0 between r and its π-equivalent; `Normalize_Rotvec` rule incl. near-π |
| `test/Scene/test_core.py` | `Components_Within` names component 1638 at `load3` (−74.1 mm ± 1 mm, §4.11), nothing at `home`, and refuses a threshold above `Clearance.Environment`; scene compiles; base body world position == `Base_Position` exactly and yaw == `Base_Yaw`; **at `Base_Yaw` = 90° (a temporary copy of the configuration) `home`'s TCP lands at `Base_Position` + (−y, x, z) of its `_p` within 3 mm, in MuJoCo and through the base frame (the file is in degrees; read as radians the test fails)**; **MuJoCo flange FK (after the MjSpec offset override) == DH flange FK within 1e-9 m / 1e-9 rad (atan2 geodesic, §9.5) for 1000 seeded random joint vectors within the limits**, plus A and B; pad-span hull present in the gripper rigid group; fingers open along the configured `Mount_Yaw` (flange ±x at 0); visual mesh has no hull (`mesh_graphadr = −1`); base↔env build check runs; loader rejects an elbow limit beyond ±180°; gripper opening == `Tool.Opening` ± 0.5 mm, equality residual < 0.1 mm; coupling cylinder present (visual + collision), Menagerie `base_mount` mesh geoms absent, vendored files byte-identical to `models/MANIFEST.json`; pad end from the flange face 163.3 mm open and 176.8 mm closed (± 0.5 mm, manual chain) and the 50 mm values 154.39 / 173.23 mm (± 0.05 mm, regression); exclusion set == configuration expansion + `base–env_stand`, filterparent disabled; joint ranges == `Joint_Position_Limits`; **placement (§4.6): `pick_obj_1..8` on the pallet table, `unload_obj_1..8` on the side table (between the plate top and the highest part), `bench_drop_1..8` inside object (a) shifted by the owner's 150 mm; the same check at `Base_Yaw` = 0 refuses all 24**; A and B pass `Check_Clearance`, and **any violation fails the test, listing every pair with its distance and margin** (no report-only path); `hover_unload_2` (clear) pushed into the side table is reported as a collision; a collision set whose recorded assembly or component overrides differ from the configuration is refused; an added box is a convex mesh of `env` whose vertices are its eight configured corners, with the environment margin, a zone piece only when it meets the zone box; the station zone applies to a generated configuration only within `Endpoint_Radius` of a station point (DH distances); a segment kind other than taught/generated is refused; a generated configuration with the tool pointing up above 2.35 m violates the workspace (excursion = the pad end's, from DH), the taught rule does not apply the box; the van der Corput order is a permutation of the samples |
| `test/Scene/test_station.py` | the taught rotation sequence of cases 17 and 18 (home → … → home, typed from the program, 50 mm holding / 85 mm open) checked as taught has no violation: no penetration, the zone margin at the zone pieces, the global margins elsewhere; every taught segment also keeps the generated rule (it comes within 20 mm of the station only within 50 mm of a station point), and Waypoint_18 → pick_rotate_1 (open, 5.5 mm to the holder) is clear as generated only with the station points; the holder box is a zone piece in both scenes, every taught sample keeps the zone margin to it and comes within the global margin, and MuJoCo's contact at the closest sample has the exact distance; the manifest records exactly the two assembly shifts and no component override, which are the configured ones; `Environment_Distance` equals the exact distance in the 20 – 30 mm band (start of pick_rotate_1 → Waypoint_6); the zone holds the holder and exactly the sill-frame pieces 181, 1062, 2701, 2702, 2704, none of the post, top profile, machine body, right window profile, tables or machine-interior hull; without the holder every taught segment of both cases keeps the global margins everywhere; the early-exit check agrees with the full check on 72 seeded segments from the taught points (both verdicts occur); `Station_Box_Floors` is the closer end's exact distance capped at the margin (S2: holder 15.1 mm; home -> Waypoint_1: 20 mm), and with S2's floors pick_rotate_1 violates the station-box rule naming the holder, without floors or under the taught rule it does not |
| `test/Scene/test_program.py` | **slow** (≈ 70 s; `-m "not slow"` deselects it): every unique segment of the 21 cases of the program (125, `test/Scene/data/sf1_plc_v4_segments.json`), joint-linear on the samples of §9.7, at 50 and 85 mm: no segment the reference records as clear penetrates the environment, no recorded one (63: the machine-interior hull 1638 and the grasp contacts) is more than 0.5 mm deeper than recorded, no robot body penetrates another; with `SF1_PLC_V4_SCRIPT` set the segment list equals the program's (otherwise skipped) |
| `test/Planner/test_core.py` | `Profile_Fraction` is 0 at the start, 1 at the estimated duration (triangle and trapezoid, movej and movel), 0.5 at half time, a t²/2 early, never decreasing; duration estimates against hand-computed values (triangle, trapezoid, leading joint, path sum, movel); the committed paths of case 17 pass `Verify_Path`, S2 with fewer via points than taught, S1 and S3 no more; home and the station points keep their tokens, no loading point is written; the sequence file is case 17 home → home, its motion lines in program order (the blended start at home generated with r=0; generated movej with the program's a, v and r=0; station lines verbatim; gripper comments; the safety sentence); aligned with the simplified taught program, it differs only in S1 – S3 and the start at home, every added line a generated movej, the gripper comments identical; within the global margin no generated sample comes closer to a station box than the segment's closer taught end, and every sample keeps the generated rule's margin to it (exact distances); `Plan` sets the station-box floors of its own ends for every edge (an approach to the holder valid before a plan is refused after planning S3, floor 20 mm), and `Verify_Path` refuses a path closer to a station box than its ends (Waypoint_18 -> pick_rotate_1 -> Waypoint_18); one reduced planning run (S1, whose direct segment fails) is byte-identical twice; RRT-Connect alone is byte-identical twice on a seed that draws from both sample sources (box around the ends, whole range), counted by a stand-in generator; an impossible plan raises (inflated margin, no iterations, six via points); the penalty counts only far from the station (a grasp approach of case 1 counts, the open approach to the pick, 5.5 mm from the holder, does not) |
| `test/Scene/test_margin.py` | margin accuracy on **mesh hull vs mesh hull** (robot geom on a joint, so not static): a robot link hull at 3 orientations vs a box-hull face, and box-hull edge vs edge; at 10 mm a violation is reported, at 30 mm none; on seeded configurations near the station (6 station points × 4, σ 0.05 rad, both openings) every robot – environment geom pair whose exact distance (`mj_geomDistance`) is below `Clearance.Environment` gets a contact within 0.5 mm of it, covering the gripper pads (boxes) and the meshes against the set's pieces and the added boxes; reported distance error < 0.5 mm (prototype: 0.000 mm, §4.5) |
| `test/Collision/test_core.py` | `Build_Collision_Set` on three synthetic cubes, hull only (no CoACD): an assembly override moves the touching pair, a component override adds on top, the manifest records the members `[0, 1]` and both overrides with their sources, each piece translated by exactly its sum, the third cube untouched; manifest STL hash == sha256 of the file; the STL splits into three objects of 1749 / 951 / 56 components, an assembly named by the machine body moves exactly object (a) with a component override added on top, and the manifest records the configured assemblies' members; the configured overrides move every component of (a) by −150 mm and of (c) by −50 mm along x, and nothing of (b); `R_crop` ≥ the bound recomputed from the current scene, and no robot collision vertex leaves the sphere over 1000 seeded random `q`; conservativeness violations == 0 at `eps` (vertices + 1 sample/cm² for CoACD components, hull-only exempt); degenerate components thickened and containing the original; excess metrics present and equal to a recomputation on a fixed sample; no robot body at A/B inside a flagged open-shell piece; mismatched hash → `Collision_Error_Cls` |
| `test/Example/test_demo.py` | headless demo runs N steps without exception; only the chosen joint changes, monotonically between turning points; it stays within `Joint_Position_Limits`; the pre-check runs on the dense segment samples of §9.7; the label equals the check result ("clearance checked" iff no violation, otherwise "VISUAL ONLY — UNCHECKED"); the shipped motion's label is pinned |
| `test/Example/test_write_taught_programs.py` | the simplified program is, after its header, exactly case 17 home → home as the owner states it (§1, typed in the test) and one contiguous run of the program: each movej/movel the program's line without indentation (home `r=0.01`), a gripper comment naming the program line at each gripper action, nothing else; its points file holds exactly those targets in first-use order with the extract's tokens; the script writes exactly the committed two files, byte for byte |
| `test/Example/test_plan_rotation.py` | the holding opening is case 17's configured `Tool.Holding_Openings` value (not another entry's, not `Tool.Opening`), open is 85 mm; a configuration without case 17 is refused; `Rotation_Entries` is home → … → home only: a generated start at home (the program blends, r=0.01), S1 – S3 generated, the station lines verbatim, no loading move |
| `test/Example/test_visualize_position.py` | headless: `home_q` reports "clearance kept (taught rule)" and `load3` "model not valid here", each at case 17's configured 50 mm with its source; `home_p` and an unknown name are refused (exit 1, no scene built); `--list` prints every position of the extract in order |
| `test/Example/test_replay.py` | every catalogue entry (case 17 home → home; headless, `--skip-check`) visits the moves of its files in order (patterns typed in the test), each by the rule of its kind, the gripper switching at the actions (85 mm open, 50 mm closed) and every move run in the state after them, each movel on its TCP line (≥ 9 steps), starting at home; `--speed 4` plays in a quarter of the estimated time, `1.0` in the estimated time, within a frame per move; a path outside the catalogue (given or typed), and a speed that is 0, negative or NaN, are refused before a scene is built; `--list` prints the three entries; path 2 with the dense check and screenshots, without `--with-stl`: built without the visual STL (same geoms), its 9 moves each keep the rule of their kind (generated movej the generated rule, program lines the taught rule) |
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
4. `Tool.Mount_Yaw` = 0 was an unmeasured assumption until the owner confirmed it (2026-10-18).
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
- **Collision set rebuilt with the component overrides (2026-10-17):** 2710 and 1154 translated
  (§4.7); 6716 pieces, the same counts by method and body, conservativeness max 0.769 mm, excess max
  234.8 mm / p95 11.3 mm, 396 flagged: every metric is identical to the build without overrides
  (the translation is rigid); build time 1141 s. The manifest records both overrides.
- **Planner, rotation sequence (2026-10-17, M2 Pro).** Generated rule: 0.28 ms per sample (workspace
  check 0.05 ms after precomputing bounding points; 9.8 ms before). `Example/plan_rotation.py` with
  the default budget takes about 36 min for both cases; two full runs gave byte-identical outputs.
  Every S needs one via point; S2 one instead of three. Penalty weight w, smallest clearance outside
  the station neighbourhood and estimated duration:

  | Case, S | Taught | w = 0 | w = 0.5 (chosen, written) |
  |---|---|---|---|
  | 17 S1 | 5.95 s, 26.0 mm | 4.84 s, 24.5 mm | 4.88 s, 27.6 mm |
  | 17 S2 | 8.30 s, 15.4 mm (fails the generated rule) | 4.51 s, 25.3 mm | 4.51 s, 33.8 mm |
  | 17 S3 | 6.81 s, 28.5 mm | 5.65 s, 25.6 mm | 5.71 s, 32.9 mm |
  | 18 S1 | 5.96 s, 26.5 mm | 4.89 s, 23.2 mm | 4.94 s, 26.6 mm |
  | 18 S2 | 8.29 s, 15.4 mm (fails the generated rule) | 4.48 s, 28.9 mm | 4.47 s, 34.5 mm |
  | 18 S3 | 6.84 s, 29.5 mm | 5.78 s, 20.4 mm | 5.71 s, 30.4 mm |
  | home → home 17 / 18 | 23.37 / 23.49 s | 17.32 / 17.54 s | 17.41 / 17.52 s |

  Estimates (§10.6). The optimised paths reach no deeper into the machine window than the taught
  ones (deepest gripper point x = −0.756 m in both, at the shared station ends).
- **Re-plan on the current scene (2026-10-18).** The committed S1 – S3 had been planned on the
  2026-10-17 scene and were not re-planned after the machine shift, the post refit and the station
  holder box (§4.9, §4.10). Re-planned with the same seed, budget and w = 0.5 s/m when the output
  became the whole case (31 min, M2 Pro): every S still needs one via point; the via points differ
  (the scene differs). Estimated duration and smallest clearance outside the station neighbourhood:

  | Case, S | Taught | Optimised (written) |
  |---|---|---|
  | 17 S1 | 5.95 s, 26.0 mm | 4.81 s, 35.9 mm |
  | 17 S2 | 8.30 s, 42.5 mm | 4.45 s, 30.0 mm |
  | 17 S3 | 6.81 s, 30.1 mm | 5.67 s, 32.0 mm |
  | 18 S1 | 5.96 s, 26.5 mm | 4.79 s, 46.4 mm |
  | 18 S2 | 8.29 s, 42.5 mm | 4.46 s, 29.7 mm |
  | 18 S3 | 6.84 s, 31.1 mm | 5.64 s, 30.3 mm |
  | home → home 17 / 18 | 23.37 / 23.49 s | 17.24 / 17.29 s |

  **The taught S2 now keeps the generated rule** (0 violations in both cases; on 2026-10-17 it
  came to 15.4 mm and failed): the machine shift moved the reason it was optimised. The optimised S2
  is shorter (one via point instead of three, 3.8 s less, estimate) but passes closer to the
  environment than the taught S2 outside the station neighbourhood (30.0 / 29.7 mm against 42.5 mm).
  The owner keeps the re-planned output (2026-10-18): **the optimisation of S2 is now about time,
  not safety.** The optimised S2 stays 10 mm above the 20 mm hard margin; against the 30 mm penalty
  threshold it is at it in case 17 (30.0 mm) and 0.3 mm inside in case 18 (29.7 mm, penalty 0.002 m).
- **Machine assembly and post refit (2026-10-18, §4.9).** `machine_a` −100 mm, post 2710 a further
  −15.7 mm; the 2026-10-17 overrides replaced. The collision set was rebuilt: 1109 s, the same 6716
  pieces, conservativeness 0.769 mm, 398 flagged (was 396). The committed S1 – S3 keep the generated
  rule unchanged and were not re-planned.
- **Re-plan of case 17 on the final scene (2026-10-18, §4.11, §4.12).** The deliverable narrowed to
  case 17 home → home (§1); machine (a) at −150 mm, the side cabinet −50 mm, no post fit, the zone
  shrunk to the holder. S1 – S3 re-planned once: seed 20261017, the full budget, w = 0.5 s/m, the six
  station points of case 17; 605 s (M2 Pro). Each `Verify_Path`-checked, 0 generated-rule
  violations. Estimated duration, via points, and the smallest clearance outside the station
  neighbourhood (with the piece), all on the final scene:

  | S | Taught | Committed (planned at −100 mm) | Re-planned (written) |
  |---|---|---|---|
  | S1 | 5.95 s, 1 via, 60.3 mm | 4.81 s, 1 via, 64.0 mm | 4.76 s, 1 via, 29.7 mm (top profile 2699) |
  | S2 | 8.30 s, 3 vias, 75.9 mm | 4.45 s, 1 via, 48.9 mm | 4.31 s, 1 via, 31.0 mm (top profile 2699) |
  | S3 | 6.81 s, 1 via, 61.9 mm | 5.67 s, 1 via, 65.6 mm | 5.27 s, direct, 23.3 mm (post 2710) |
  | home → home | 23.37 s | 17.24 s | 16.66 s |

  Home → home with the station operations (2.31 s), all estimates. Within the station neighbourhood
  S2 of every variant comes to 15.1 mm (the holder, at Waypoint_18). The objective trades clearance
  above 30 mm for any time (the penalty starts at 30 mm): the re-planned S1 and S2 pass the window's
  top profile at about 30 mm, and S3 runs direct, 0.40 s faster than the committed S3, 3.3 mm above
  the 20 mm margin (penalty 0.478 m, 0.24 s of cost). **The re-planned paths use the space the
  shift frees:** with the machine at the earlier −100 mm and the post fitted they would penetrate
  the top profile by 19.6 mm (S1) and 16.2 mm (S2) and the post by 9.8 mm (S3). They keep ≥ 228 mm to
  the machine-interior hull and the parts on the tables. Not repeated: a second identical run (the
  determinism is pinned by `test_a_reduced_planning_run_is_byte_identical_twice`).
- **Demo** (base ±45° around `home`): 1234 joint-linear samples, every margin kept, label
  **"clearance checked"**. At 180° (2026-10-17) the same: 1234 samples, every margin kept.
