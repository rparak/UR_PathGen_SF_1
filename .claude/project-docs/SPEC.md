# UR_PathGen_SF_1 — Specification

Status: the current specification, approved by the owner (rev. 4, 2026-10-17) and kept current
since; its history is in git. A number marked *measured* names what it was measured on; where a test
pins it, the test is named.

## 1. Goal

Generate and verify, in simulation, a collision-free joint-space path for the UR10e of cell SF1
before it is run on the real robot.

**Current goal (owner): optimise the 90° part rotation (regrasp) sequence** of the robot program
`sf1_plc_v4.script`. The station operations stay verbatim; the transfer segments between them (S1
home → drop, S2 retreat → approach, S3 lift → home) are generated with the planner of §10.6 and
compared with the taught baseline. The holder at the station exists so that the robot can grip the
part from the other side.

**The deliverable is case 17 from home to home**, this taught sequence and its optimised version:

```
movej home (r=0.01) → Waypoint_1 (r=0.05) → drop_rotate_1 → movel Waypoint_4 → [gripper open]
→ Waypoint_5 → Waypoint_1 (r=0.05) → Waypoint_2 (r=0.01) → Waypoint_3 (r=0.05) → Waypoint_18
→ pick_rotate_1 → [gripper close] → movel Waypoint_6 → Waypoint_2 (r=0.01) → home
```

Optimised: S1 (home → drop_rotate_1), S2 (Waypoint_5 → Waypoint_18) and S3 (Waypoint_6 → home)
replaced, the station operations verbatim. The delivered version is **c2**, the output of the
default job `sf1_plc_v4_case_17` in `Data/Generated/`, tested on the real cell by the owner
(2026-10-21); c1 is the job `sf1_plc_v4_case_17_c1` (§8.6, §10.6).

**Case 18 is not a deliverable.** Its job `sf1_plc_v4_case_18` exists to show that another case
plans without code changes; nothing of it is committed, and it is refused until
`Tool.Holding_Openings` has an entry for it. Cases 17, 18 and the other 19 cases of the program are
ground truth: the program extract (every learned point and cases 17 and 18, §10.1) and the tests
that validate the model against real program segments (the taught replay, §11; the penetration
reference of all 21 cases, `test/Scene/test_program.py`; the placement of the grasp points, §4.6).

Pipeline:

1. Read learned robot positions from a Polyscope `*.urscript` variable file
   (`global <name>_p=p[...]`, `global <name>_q=[...]`).
2. Plan a collision-free path from point A to point B with **at most 5 via points**.
3. Write the via points in the same format, plus a `movej(...)` sequence, ready to paste into the
   robot program.

**Simulation passed ≠ safe on the robot.** Every output not yet tested on the real cell is a
proposal that is run there at reduced speed first. The tool never claims more than that.

## 2. Scope

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

   The amplitude is never reduced to make the check pass. The shipped motion is labelled
   "clearance checked" (`test/Example/test_demo.py`).
3. A URScript parser and writer that read the learned points exactly and write them back
   byte-identically.
4. The kinematic checks: DH FK · configured TCP against `_p`, MuJoCo FK against DH FK, the
   derived TCP as a diagnostic, the coupling in the scene, the clearance check of A and B.
5. The planner and the rotation-sequence output (§10.6), the replay (§10.7) and the README figures
   (`Example/make_readme_figures.py`, §6).
6. Installer, tests, style checker, the three project skills, docs.

## 3. Out of scope

- Modelling the carried part (accepted risk, `KNOWN_ISSUES.md`).
- Blend radius > 0 and blend-envelope checking. Every emitted `movej` has `r=0` (§8.5).
- Dynamics and payload. The simulation is kinematic only; durations are estimates (§10.6).
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

**Fingertip distances.** All values are along the flange z axis, measured from the
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

The A and B columns of the `env_stand` rows were measured at `Base_Yaw` = 0, before it was set to
180° (§4.6); the robot pairs do not depend on the yaw.

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
in 0.1 % of samples; at that rate the 5 mm margin stays.

- **Structural near pair: `base – upper_arm_link`.** It is inside 5 mm in 17.5 % of samples, yet
  never below 2.3 mm: the hulls pass close but never touch. Excluding the pair would hide it, so it
  is **not** excluded; it alone has a 0 mm self margin (owner, Q9 (a); `Self_Pair_Overrides`, §8),
  justified by its true minimum, 2.3237 mm, found by a sweep of `shoulder_pan` × `shoulder_lift`
  (the only joints that move the upper arm relative to the base) and pinned (§17).
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
| Visual STL as `uservert`/`userface` (891 504 vertices, 1 757 948 faces), `contype=conaffinity=0`, shell inertia | compiles; **no hull stored** (`mesh_graphadr = −1`) |
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
at home facing the machine. The grasp points lie inside the margin of the modelled parts they grasp
(−12 to −20 mm), and the machine drops inside the filled interior of machine (a) (§4.11): both are
expected for points that handle parts, and neither is an end point of a planned segment. Where the
STL contradicted the taught rotation sequence at 180°, the machine shift and the station boxes
(§4.10 – §4.13) model the cell.

### 4.7 Station correction fitted to the taught program (removed)

The component fits of 2026-10-17 (the post 2710 and the machine body 1154) no longer exist: machine
(a) moves as one assembly (§4.11) and no component is fitted. The number is kept because it is cited.

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

| Reading | Tape | STL | Model (machine at −150 mm, §4.11) |
|---|---|---|---|
| base axis → machine front at window height | ~700 mm | 612 mm (window profiles) | 762 mm (both profiles) |
| robot-table edge (x −0.340) → lower cabinet front, low | ~300 mm | 127 mm | 276 mm |

### 4.9 Owner facts: the machine assembly, the gripper, the release (2026-10-18)

- **Machine (a) is one rigid assembly**: the machine body, the front column with the window frame
  and the closed lower cabinet. Grouping the STL components by touching bounding boxes (1 mm) gives
  exactly three objects: (a) the machine, 1749 components, x −1.609 … −0.466 m; (b) the robot stand,
  pallet and side tables, 951 components; (c) the side cabinet, 56 components, y ≥ 0.555 m.
  `machine_a` is the object of component 1154; the owner confirmed the selection on renders. Its
  placement: §4.11.
- **`Tool.Mount_Yaw` = 0° is confirmed** on the robot: the fingers open along flange ±x.
- **The release opening of 85 mm is real**: the part is released into a magazine with free space
  below and above.

### 4.10 Station holder box (*measured*, 2026-10-18)

The holder of §4.8 is one world-axis-aligned box, `Environment.Added_Boxes` `station_bracket`,
x −0.770 … −0.645 m, y −0.340 … −0.260 m, z 0.955 … 1.112 m (exact distances, every taught sample
of cases 17 and 18 at its opening, the release included, the boxes as convex meshes):

- front x −0.645 m: an assumed depth, 90 mm in front of the post face the taught program
  constrained before the machine shift; back x −0.770 m, 8 mm into the post face at x −0.762, since
  the holder is mounted on the post (at no clearance cost). It stays where the robot places the part
  (world-fixed) when the machine moves;
- y and the bottom from the photo;
- **top 1.112 m:** the highest at which every taught sample keeps 5 mm (limit 1.1125 m, the open
  finger). That is 17 mm above the photo's estimate of 1.095 m, where the clearance is 21.05 mm (the
  20 mm top would be 1.0961 m; the owner kept the fitted top). The box keeps 5 mm from the taught
  release, where the magazine has free space below and above the part.

Closest taught sample: 5.53 mm (case 17 Waypoint_18 → pick_rotate_1, the open finger).

### 4.11 Machine (a) at −150 mm, side cabinet −50 mm, no component fit (*measured*, 2026-10-18)

**Owner facts (estimates, not measured).** Machine (a) stands 150 mm further along −x than the STL
draws it, at the 150 mm bound of an override. **Owner's judgement (2026-10-18): the −150 mm position
matches the real cell better than −100 mm**, and generated paths may use the space it frees. Its
accuracy, ±10 mm (owner), is `Clearance.Machine_Extra` (§9.7). The side cabinet, object (c) (body
2643), stands 50 mm along −x, a separate assembly with its own source. The robot stand and the
tables, object (b), stay. The collision set records both assemblies, their 1749 and 56 members and
no component override (§17).

**Post 2710: no fit.** The post alone keeps **≥ 39.1 mm** to every taught sample of cases 17 and 18
(case 18 drop_rotate_2 → Waypoint_7), so the program does not constrain it: its face at x −0.762
rests only on the owner's estimate.

Cases 17 / 18 as taught (minimum distance to the STL and the holder, mm, its own opening per
segment; ≥ 60 not listed; the distances to the stub are in §4.13):

| Segment | Case 17 / 18 |
|---|---|
| Waypoint_1 → drop | 40.4 / 39.8 (post) |
| drop → Waypoint_4 / 7 | 39.5 / 39.1 (post) |
| Waypoint_4 / 7 → Waypoint_5 (85 mm) | 40.6 / 39.8 (post) |
| Waypoint_5 → Waypoint_1 | 50.0 / 50.0 (post) |
| Waypoint_2 → Waypoint_3 | ≥ 60 |
| Waypoint_3 → Waypoint_18 / 11 | 15.1 / 27.2 (holder) |
| approach → pick (85 mm) | 5.5 / 14.4 (holder) |
| pick → Waypoint_6 / 8 | 17.8 / 27.8 (holder) |
| Waypoint_6 / 8 → Waypoint_2 | 45.0 / 47.6 (post) |

Every segment keeps its margins as taught, and the generated rule without the station-box floors
(§9.7). **Only the station boxes come within 20 mm** of a taught sample (the holder 5.53 mm, §4.10;
the stub 5.03 mm, §4.13); every STL piece keeps ≥ 39.1 mm, and every station point keeps ≥ 39.1 mm
to every STL piece. To the holder: pick_rotate_1 open 5.5 mm, pick_rotate_2 open 14.4, Waypoint_18
open 15.1, pick_rotate_1 holding 17.8 mm.

**All 21 cases** (`test/Scene/test_program.py`, 125 unique segments, 50 and 85 mm): 63 segments
penetrate, recorded as the test's reference. The machine-interior penetrations reach −74.2 mm
(load2 ↔ load3) and −140.4 mm at the deepest (Waypoint_19 → bench_drop_8, component 1638); no
self-collision. No segment comes within 50 mm of the side cabinet (c): the program neither confirms
nor contradicts its shift.

**Placement** (§4.6): the bench_drop points lie at x ≈ −1.12 m, inside machine (a) both where the
STL draws it and shifted. `load3` → 1638: −74.1 mm.

### 4.12 Station zone: the station boxes' neighbourhood (*measured*, 2026-10-18)

**Owner decision (2026-10-18).** The program does not constrain the post (≥ 39.1 mm, §4.11), so its
position rests only on the owner's estimate, and a generated path keeps the full 20 mm from it. The
zone (§9.7) holds only the station boxes, the geometry the taught program comes closer to than
20 mm.

**Box** `[-0.755, -0.36, 0.94]` – `[-0.62, -0.24, 1.13]` m: the holder and stub boxes and the
sill-frame pieces they sit on, components 181, 1062, 2701, 2702, 2704 (5 pieces). No taught sample
comes within 20 mm of a sill-frame piece; the post's face (x −0.762) lies outside the box.

**Checks.** Without the station boxes, every taught segment of cases 17 and 18 keeps the global
margins everywhere (the generated rule without station points). With the zone the taught sequence
keeps its margins as taught, and as generated without the station-box floors. Every taught sample
of cases 17 and 18 under 20 mm (615 samples, at `Segment_Samples` spacing) has its TCP within
41.2 mm of a station point of its job (the farthest: case 18, the movel to Waypoint_8, 12.3 mm to
the stub; case 17 at most 38.2 mm), so `Endpoint_Radius` (50 mm) covers them (measured 2026-10-23;
13.5 mm before the stub was modelled).

### 4.13 Station stub box (*measured*, 2026-10-18)

**Owner decision (2026-10-18).** The stub of §4.8 is modelled as an `Environment.Added_Boxes` entry,
`station_stub`, source "owner photo + taught points, not measured", as large as the taught program
allows: y −0.330 … −0.270 m and z 1.093 … 1.264 m from the photo; x from 8 mm inside the shifted post
face (−0.770 m) to the farthest front at which every taught sample of cases 17 and 18 keeps 5 mm:
**−0.7274 m, 34.6 mm proud** of the face (bisection, 0.06 mm; exact distances, convex mesh boxes:
the limit −0.72737 m). Closest taught sample: 5.03 mm (case 17 pick_rotate_1 → Waypoint_6, the
holding pad). It meets the zone box, so it is a zone piece like the holder (§4.12).

Taught clearance to the stub (mm, ≥ 60 not listed): case 17 / 18 Waypoint_1 → drop 13.6 / 11.6,
drop → Waypoint_4 / 7 10.1 / 8.8, → Waypoint_5 (85 mm) 11.4 / 11.2, Waypoint_3 → Waypoint_18 / 11 8.3 /
12.6, approach → pick 10.0 / 13.2, pick → Waypoint_6 / 8 5.03 / 7.1, Waypoint_5 → Waypoint_1 42.6.

## 5. Platform

- macOS on Apple silicon (primary, verified here) and Ubuntu Linux (unverified).
- Python **3.11**, called explicitly. On macOS it is `/opt/homebrew/bin/python3.11`, on Linux
  `python3.11` from `PATH`. A bare `python3` is never used: miniforge's numpy is broken on this
  machine.
- The viewer: `mujoco.viewer.launch_passive` runs under `mjpython` on macOS and under `python` on
  Linux. The demo detects the case and, on macOS without `mjpython`, exits with an `[ERROR]` line
  that gives the command to use. A `--headless` mode runs without a viewer (used by the tests).
- The 1.76 M-triangle visual mesh is passed to MuJoCo as `uservert`/`userface`, read by the project's
  numpy STL reader (`Read_Binary_STL`, `src/Collision/Core.py`), because the MuJoCo STL loader refuses more than 200 000 faces (§4.5). Its geom has
  `contype=conaffinity=0` and shell inertia, so MuJoCo stores no hull for it. A test pins this.
- Pinned versions (installed 2026-10-17, re-checked at implementation): `mujoco==3.16.0`,
  `numpy==2.4.6`, `scipy==1.17.1`, `trimesh==5.1.1`, `coacd==1.0.14`, plus `pyyaml`, `pytest`,
  `rtree` (exact pins in `requirements.txt`); `matplotlib==3.11.2` (2026-10-20) for the README
  figures only. CoACD and trimesh are imported only by the decomposition
  build (`Build_Collision_Set`), never by the tests.

## 6. Repository structure

```
README.md                 what the project is, results, install, run, test, adapting to another cell
CLAUDE.md                 short: where the internal docs and the skills are, test and checker commands
LICENSE                   MIT, Copyright (c) 2026 JIC Smart Factory
install.sh                macOS + Ubuntu, OS detection
requirements.txt          pinned
.gitignore                .venv, __pycache__, *.pyc, .pytest_cache, _source_skills/, .DS_Store
.claude/settings.json
.claude/project-docs/SPEC.md              this file
.claude/project-docs/KNOWN_ISSUES.md      accepted risks and known limitations
.claude/project-docs/design-decisions.md  deliberate choices whose reason is not visible in the code
.claude/skills/project-style-rpx/   SKILL.md, references/, assets/, scripts/
.claude/skills/feature-manager/SKILL.md
.claude/skills/project-optimize/SKILL.md
CAD/SF1_Environment.stl                       source, unchanged (visual mesh)
CAD/Collision/SF1_Environment_Collision.npz   convex pieces (vertices, faces, piece → component, body)
CAD/Collision/SF1_Environment_Collision.json  manifest (§7.6)
Configuration/Cell_SF1.yaml                   cell configuration (§8)
Configuration/Jobs/<name>.yaml                planning jobs (§8.6): sf1_plc_v4_case_17 (the default, c2,
                                              delivered), sf1_plc_v4_case_17_c1, sf1_plc_v4_case_18
                                              (not a deliverable)
Data/Programs/sf1_plc_v4/                     extracts of the owner's program, verbatim, with its sha256:
                                              sf1_plc_v4_positions.urscript (its 156 global _p/_q lines)
                                              and sf1_plc_v4_cases_17_18.script (cases 17 and 18, §10.1);
                                              the simplified taught program of
                                              case 17 home → home, sf1_plc_v4_case_17.urscript +
                                              _sequence.script (§10.4)
models/MANIFEST.json                          Menagerie commit, licence, sha256 of every vendored file
models/universal_robots_ur10e/                xml, assets/, LICENSE, README.md, CHANGELOG.md (no png)
models/robotiq_2f85/                          same
src/Configuration/Core.py   Load_Configuration → Cell_Configuration_Str, Load_Job → Job_Configuration_Str (the only YAML reader)
src/URScript/Core.py        Read_URScript, Read_Program_Excerpt, Read_Motion_Sequence, Write_URScript, Write_Movej_Sequence,
                            Write_Motion_Sequence, Program_Entries, Resolve_Sequence, Extract_Positions, Extract_Cases
                            (the only URScript reader/writer)
src/Kinematics/Core.py      UR10e DH FK, rotvec ↔ matrix (Matrix_To_Rotvec), Geodesic_Angle, Solve_IK_Analytic, Derive_TCP_Offset
src/Collision/Core.py       Build_Collision_Set, Write_Collision_Set, Read_Collision_Set (the only npz/manifest reader/writer)
src/Scene/Core.py           Scene_Cls (built through MjSpec), Set_Joints, Check_Clearance, the taught / generated rule
src/Planner/Core.py         Planner_Cls (direct, RRT-Connect, shortcut, refinement), Verify_Path, duration estimates
Example/plan_rotation.py    plans a job (--job, default sf1_plc_v4_case_17) into --out (default
                            Data/Generated/) after its taught model check (§9.7), prints the report
Example/replay_rotation.py  viewer replay of a numbered catalogue of taught and optimised paths, or of a job's output (--job) (§10.7); screenshots
Data/Generated/             the delivered c2: SF1_Rotation_17.urscript (points) and SF1_Rotation_17_Sequence.script
                            (home → home)
Data/Generated/variants/    taught, b_optimised, c1, c2, c3 in the same two files each, and comparison.csv / .md
Example/write_variants.py   writes Data/Generated/variants/ and the comparison table (§10.6)
Example/make_readme_figures.py  writes every README image into Media/ from comparison.csv and the variant files
Media/                      the README images: graphs (light and dark) and MuJoCo renders, all generated
Example/write_taught_programs.py  writes the simplified taught program of case 17 home → home (Data/Programs/sf1_plc_v4/)
Example/extract_program.py  cuts the positions and cases extracts from a robot program (§10.1)
Example/build_collision.py  regenerates CAD/Collision/ from the STL (runs CoACD; not in the test suite)
Example/demo_rotate_axis.py the viewer demo
Example/demo_visualize_position.py  the robot static at one learned position, clearance by the taught rule (§10.7)
test/<Domain>/test_<module>.py mirroring src/, with further files where a domain needs them
                            (test/Scene/test_station.py, test_margin.py, test_program.py;
                            test/Collision/test_conservativeness.py, test_excess.py); test/Install/test_install.py;
                            test/Example/test_<script>.py for the scripts; test/Docs/test_readme.py (the README
                            table against comparison.csv, its images, the root); test/conftest.py puts src/ on the
                            path and holds shared fixtures; test/URScript/data/SF1_Positions_original.urscript
                            (the robot's export of A and B, §10.3); test/Scene/data/sf1_plc_v4_segments.json
                            (the penetration reference of all 21 cases, §4.11)
pytest.ini                  import mode and the slow marker; the one configuration file at the root
```

`src/` layering: `Configuration`, `URScript`, `Kinematics` and `Collision` import nothing of the
project; `Scene` → those four; `Planner` → `Configuration` and `Scene`. Scripts in `Example/` reach
`src/` through a path built from `__file__`.

## 7. Environment collision geometry (risk 1, requirement E)

MuJoCo collides meshes through their convex hull. The STL as one mesh would be one blob, so it is
never used for collision; it stays the visual geom (`contype=conaffinity=0`).

### 7.1 Crop to the reachable volume

- Centre: the shoulder point (base frame (0, 0, d1)), mapped to world.
- Radius `R_crop`: an upper bound on the distance from that point of any robot or gripper
  collision vertex in any configuration. It is the sum of the body-origin offsets along the chain
  after the shoulder, plus the largest vertex radius of the outermost bodies (gripper included),
  plus the environment clearance, plus 0.05 m. It is computed from the scene, not configured, and
  recorded in the manifest (SF1: 2.026 m, every component kept, §17).
- A component is kept whole if any of its vertices lies inside the sphere. Triangles are never cut.
- Reported: components kept / dropped, triangles kept.

### 7.2 Hull first, decompose only where the hull is too coarse

1. A convex hull per kept connected component. A **degenerate** component (planar or collinear, no
   3-D hull; SF1: 503 of 2756, all planar, §17) is thickened by ±0.5 mm along the normal of its
   plane, from the SVD of its centred vertices (smallest singular vector). A mean face normal is not
   used, because the winding is inconsistent and normals can cancel. A collinear component is
   thickened by ±0.5 mm along two directions orthogonal to its line. The result contains the original, so it stays conservative and adds
   ≤ 0.5 mm of excess. Every collision mesh gets shell inertia, because the bodies are static.
2. Hull **excess** per component: the maximum distance from the hull surface (its vertices plus
   area-uniform samples, fixed seed) to the original component surface.
3. Excess ≤ `CONST_HULL_EXCESS_MAX` (10 mm, `Example/build_collision.py`, recorded in the manifest) →
   the hull is the collision piece.
4. Otherwise run CoACD on that component only (`preprocess_mode="auto"` for open shells, fixed
   seed, parameters recorded) and re-measure. If the CoACD pieces fail the conservativeness check
   (§7.3), the component falls back to its hull. Conservativeness beats tightness.
5. A merge pass (bbox-touching neighbours whose joint hull stays within the excess threshold become
   one piece) is not implemented: the measured query time did not need it (§17).

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

When a configuration violates a margin against the full environment set, the pairs are reported.
`CONST_HULL_EXCESS_MAX`, `eps` and the margins are never tuned to make a check pass.

### 7.4 Open shells filling space the robot needs

A hull of an open shell (sheet-metal housings, frames, especially in object (a)) fills its
interior. The build report lists every piece with excess > `CONST_HULL_EXCESS_MAX` after fallback, with
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
and radius; the excess threshold, `eps`, the CoACD version and parameters, seed; counts and
runtimes; both metrics.

**Pending for the next rebuild** (no rebuild now): the build reads the version as
`coacd.__version__`, which the installed package does not define, so the committed manifest records
`unknown` although `requirements.txt` pins `coacd==1.0.14`. Before the set is next rebuilt, the
build reads it with `importlib.metadata.version('coacd')` (1.0.14 in `.venv`), so the manifest
records the version that made the pieces. `Read_Collision_Set` refuses a set whose recorded STL hash differs from the file on disk
(`Collision_Error_Cls`, with the sentence "rerun Example/build_collision.py"). Tests read the set and
never run CoACD.

**Component overrides.** `Environment.Component_Overrides` lists STL components whose geometry the
real cell contradicts, each with a rigid translation (≤ 150 mm) and its source. The build translates
the component's own vertex copy before the crop and the decomposition, so a neighbour sharing a
vertex is not moved, and the conservativeness check runs against the translated surface. The
manifest records the overrides (`component_overrides`); the scene refuses a set whose record differs
from the configuration (`Scene_Error_Cls`, "Rebuild it with …"). The visual mesh is never changed, so at an overridden component the viewer shows the
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
| `Environment.Component_Overrides` | rigid translations of STL components before the build, added to any assembly translation: `{Component, Translation [x, y, z] (world, ≤ 150 mm), Source}` each; may be empty (§7.6). Shipped: none (no component is fitted, §4.11) | –, m, text | REQUIRED |
| `Environment.Added_Boxes` | obstacles missing from the STL, added to body `env` as convex meshes of their eight corners (world-axis-aligned boxes; not box primitives, whose contacts with the gripper's box pads are missed inside the margin) with the environment margin: `{Name, Min [x, y, z], Max [x, y, z] (world), Source}` each; a box meeting `Clearance.Station_Zone` is a zone piece; may be empty. Shipped: `station_bracket`, the holder (§4.10), and `station_stub` (§4.13); both are zone pieces. The collision set is not rebuilt for them | –, m, m, text | REQUIRED |
| `Robot.Model` | `models/universal_robots_ur10e/ur10e.xml` | path | REQUIRED |
| `Robot.Base_Position` | Polyscope base origin in the STL/world frame: `[0.0, 0.0, 0.7152]` | m | REQUIRED |
| `Robot.Base_Yaw` | rotation of the Polyscope base frame about world z: `180.0` (owner, 2026-10-17; confirmed by the taught points of `sf1_plc_v4.script` and photos of the cell, §4.6) | deg | REQUIRED |
| `Robot.Joint_Position_Limits.Min/Max` | joint position limits, 6 each. Shipped: ±360° for base, shoulder, wrists; **±180° for the elbow** (Menagerie default, owner). Load rejects: any joint beyond ±360°, and **the elbow beyond ±180°**, because that limit is the precondition for excluding `upper_arm_link–forearm_link` (§9.6). The Polyscope safety limits may be tighter (`KNOWN_ISSUES.md`) | deg | REQUIRED |
| `Tool.Model` | `models/robotiq_2f85/2f85.xml` | path | REQUIRED |
| `Tool.Coupling.Thickness` | coupling thickness: `0.0169` (manual rev. 2020/02/11 p. 119, Fig. 5-5) | m | REQUIRED |
| `Tool.Coupling.Diameter` | coupling outer diameter: `0.075` | m | REQUIRED |
| `Tool.Coupling.Spigot_Depth` | depth of the Ø71 bore receiving the gripper: `0.0029`; gripper datum = thickness − depth = 14.0 mm (measured on the robot) | m | REQUIRED |
| `Tool.Mount_Yaw` | rotation of the gripper about the flange z axis relative to the Menagerie default: `0.0`, the fingers open along flange ±x. **Confirmed on the robot by the owner (2026-10-18)** (§4.9, §9.3) | deg | REQUIRED |
| `Tool.Opening` | inner pad-face gap of a scene that names no program case (the demo, the tests): `0.050`, the 50 × 50 mm part (owner) | m | REQUIRED |
| `Tool.Holding_Openings` | inner pad-face gap while the part of a program case is held, `{Program, Case, Opening, Source}` each, keyed by program folder (`Data/Programs/<Program>/`) and case, so a case number names a case of one program only (owner, 2026-10-21); used by planning, verification and replay of a job of that case (§8.6): sf1_plc_v4 case 17 `0.050` (owner, 2026-10-18: the 50 × 50 mm part). Case 18 has no entry: its part's size is not known. Load rejects: an empty list, an entry without its program, an opening ≤ 0 or above the 85 mm stroke, an empty source, a case of a program named twice; a script refuses a case without an entry, naming the entry to add | –, –, m, – | REQUIRED |
| `Tool.TCP_Offset` | TCP in the flange frame `[x, y, z, rx, ry, rz]`: `[0.0, 0.0, 0.174, 0.0, 0.0, 0.0]`. Source: the Polyscope TCP of SF1 (owner, 2026-10-17), identical to the Robotiq value (manual rev. 2020/02/11 p. 127, §5.2.3) | m, rad (rotvec) | REQUIRED |
| `Clearance.Environment` | minimum robot/gripper ↔ environment distance: `0.020` | m | REQUIRED |
| `Clearance.Self` | minimum distance between non-excluded robot/gripper bodies: `0.005` | m | REQUIRED |
| `Clearance.Self_Pair_Overrides` | `[[base, upper_arm_link, 0.0]]` (Q9 (a)): a pair-specific self margin. Allowed only for a pair whose true minimum stays > 0: grid over the joints that move it, then scipy local minimisation from the 20 lowest cells; the test checks every entry and pins the refined minimum | m | REQUIRED |
| `Clearance.Station_Zone.Min/Max` | world box of the station zone: environment pieces whose bounding box meets it are the zone pieces (§9.7). Shipped: the station boxes' neighbourhood, `[-0.755, -0.36, 0.94]` – `[-0.62, -0.24, 1.13]` (owner, 2026-10-18, §4.12) | m | REQUIRED |
| `Clearance.Station_Zone.Environment` | margin of the zone pieces where the zone applies; > 0 and ≤ `Clearance.Environment`. Shipped: `0.005` (§9.8) | m | REQUIRED |
| `Clearance.Station_Zone.Endpoint_Radius` | a generated sample uses the zone margin only while its TCP lies within this distance of a fixed station point's TCP (§9.7). Shipped: `0.050` (§4.12) | m | REQUIRED |
| `Clearance.Machine_Extra.Assembly/Margin/Source` | extra environment margin of **generated** segments to one assembly's pieces outside the station zone, for the accuracy of its estimated placement (§9.7). Shipped: `machine_a`, `0.010` (owner accuracy estimate of the machine position, 2026-10-18), so a generated path keeps 30 mm to machine (a) outside the zone. `plan_rotation.py`'s report and the headers of its output state the margin with its source (`Machine_Margin`). Load rejects: an assembly not in `Environment.Assembly_Overrides`, a negative margin or one above 150 mm, an empty source | –, m, text | REQUIRED |
| `Clearance.Workspace.Min/Max` | world box a generated segment keeps every robot collision geom inside: x/y the STL's extents, z max 2.35 m, the highest point of the taught rotation sequence (2.322 m) rounded up — space proved free by the taught program (§9.7) | m | REQUIRED |
| `Self_Collision.Excluded_Pairs` | explicit body pairs never checked (§9.6) | — | REQUIRED |
| `Self_Collision.Rigid_Groups` | groups of bodies rigid to each other; every pair inside a group is excluded (the gripper at fixed opening) | — | REQUIRED |
| `Kinematics.FK_Sanity_Bound.Position` | the most a learned point's DH FK·TCP may differ from its `_p` before it is refused: `0.010` (§9.4) | m | REQUIRED |

The orientation sanity bound is derived from the position bound (§9.4), not configured. There are
no optional fields, so there are no defaults to keep in sync. Values that have a
single legal value are deliberately **not** fields: robot type (UR10e DH is a code constant), blend
radius (always 0, §8.5), joint wrapping (never, §10.5). Speed and acceleration are not fields either:
`movej` with `r=0` follows the same joint-linear path at any speed, so they cannot change what is
checked (owner, Q4).

### 8.5 Blend radius

Every emitted `movej` has `r=0`. The collision check covers the joint-linear segment between
consecutive targets (§9.7). With `r > 0` the robot leaves that segment, so a blend is a future
feature that needs envelope checking. Until then it is not a parameter.

### 8.6 Planning jobs — `Configuration/Jobs/<name>.yaml`

What is planned is a job, not code (owner, 2026-10-21): `Example/plan_rotation.py --job <name>`
(default `sf1_plc_v4_case_17`) loads it with `Load_Job` (`src/Configuration/Core.py`, the only
reader, with the rules of §8: every field REQUIRED, all problems named at once, an unknown key a
WARNING). `--job` also takes a `.yaml` path.

| Field | Meaning | Unit | Status |
|---|---|---|---|
| `Program` | the program folder, repository-relative: its extracts (§10.1), `<name>_positions.urscript` and the one `<name>_cases_*.script` holding `Case`; both must carry the same sha256 | path | REQUIRED |
| `Case` | the case of the program's switch | – | REQUIRED |
| `Segments` | the segments planned, in program order, `{Name, From, To}` each; `From` and `To` are learned points the case moves to, matched in program order (§10.6) | –, point, point | REQUIRED, ≥ 1 |
| `Point_Configurations` | learned points written with another `_q`, the `_p` the program's (§10.5): `{Point, Joint (1 base … 6 wrist 3), Turns (whole, ≠ 0), Source}` or `{Point, Branch ("S± E± W±"), Source}` each; may be empty | –, –, 2π, –, text | REQUIRED |
| `Output.Points`, `Output.Sequence` | the output file names (`*.urscript`, `*.script`), written into `--out` | file | REQUIRED |
| `Output.Title`, `Output.Status` | what the sequence is ("the <Title> <from> -> <to>") and what the output is (its first status line), in both headers; one line of ASCII each | text | REQUIRED |
| `Output.Stations` | the name of each station operation between two segments (one fewer than `Segments`), in the sequence's section comments | names | REQUIRED |

Derived from the case's lines, never configured (`Resolve_Sequence`, `Load_Job_Plan`): the span
(the first segment's `From` line to the last segment's `To` line), each segment's taught via
points and **gripper state** (the result of the case's last gripper action before it, 0 open and
255 close = holding; before the first action, the opposite of it), the **station operations**
(every line between two segments, copied verbatim), the **fixed station points** of the
generated rule (every point of the span but its two ends and the taught via points), and the
**station time** (each station line from the end before it, with the line's own `a` and `v`).
Refused: a segment end that is not a learned point or not reached in program order, a segment
containing a gripper action, a station line blending (`r > 0`) into the next generated segment
(§8.5), gripper actions that do not alternate or are absent, a point of the span the positions
extract does not hold, a station line without `a` and `v`, a configured point the output does not
move to, and a taught `movej` of a segment at another `a` or `v` than the generated moves'
(`CONST_MOVEJ_A_TOKEN` / `_V_TOKEN`, `KNOWN_ISSUES.md`). The holding opening stays in
`Tool.Holding_Openings` (§8). The seed (`20261017 × 1000 + case × 10 + k`), the penalty weight
and the planner's budgets stay code constants (§10.6).

Shipped: `sf1_plc_v4_case_17`, the default, case 17 S1 – S3 as c2 (the drop on branch S− E− W−,
the pick's wrist-3 turn, §10.5); `sf1_plc_v4_case_17_c1`, c1; `sf1_plc_v4_case_18`, case 18's S1 –
S3 (home → drop_rotate_2, Waypoint_5 → Waypoint_11, Waypoint_8 → home), no point configured, kept to
show that another case plans without code changes (owner, 2026-10-21). Case 18 is not a
deliverable: no output of it is committed, and without a `Tool.Holding_Openings` entry for it
(its part's size is not known) `--job sf1_plc_v4_case_18` is refused. A slow test plans it with an
assumed 50 mm and a reduced planner budget: its taught moves pass the model check with no error and
no warning, and every segment is planned and verified. When the program's line to the first
`From` does not blend (case 18), the start's comment says so.

## 9. Kinematics, TCP and collision model

### 9.1 Frames

- World = STL frame. Polyscope base frame = world translated by `Robot.Base_Position` and rotated
  by `Robot.Base_Yaw` about z. The Menagerie root (with its built-in 180°) is placed at exactly that
  pose; §4.2 shows that the Menagerie world frame already equals the Polyscope base frame.
- `Robot.Base_Yaw` = 180°: the base frame's +x points to world −x, towards machine (a). FK vs `_p`
  cannot see the yaw; the placement of the taught points decides it (§4.6).
- Flange frame = DH frame 6 (z out of the flange).
- TCP = flange · `Tool.TCP_Offset`.

### 9.2 One kinematic truth: the official UR10e DH

d = (0.1807, 0, 0, 0.17415, 0.11985, 0.11655) m, a = (0, −0.6127, −0.57155, 0, 0, 0) m,
α = (π/2, 0, 0, π/2, −π/2, 0). The scene builder overwrites the Menagerie body offsets and the
flange site through MjSpec so that MuJoCo FK equals DH FK. Vendored files stay byte-identical to
upstream; the patch is in code and in `design-decisions.md`. The gripper attaches at the
corrected flange.

**Inverse kinematics** (`src/Kinematics/Core.py`, all on these DH parameters). `Solve_IK_Near`
(numeric, damped least squares) follows one branch, as the controller follows a movel.
`Solve_IK_Analytic` (closed form) returns every solution of a TCP pose, up to 8,
each labelled by its branch `S± E± W±`: S the sign of the shoulder solution θ1 = atan2(y5, x5) ±
acos(d4 / r5) + π/2 (wrist centre at (x5, y5), r5 from the base axis), E the sign of the elbow angle,
W the sign of sin θ5; angles in [−π, π), every joint but the elbow also reachable ± 2π
(`Nearest_Representative`); each solution is checked by DH FK (1e-9 m / rad). `IK_Branch` names the
branch of a configuration. Home and the pick's points of case 17 lie on S− E− W+, the drop's on
S+ E+ W− (`test_the_taught_points_lie_on_their_measured_branches`). The analytic IK serves the branch proposals for re-teaching (§10.6);
a solution of a learned `_p` is nominal-DH, millimetres from what the robot's calibrated IK returns.

### 9.3 TCP and gripper geometry (owner: configured, sourced, do not tune)

1. **The TCP is configured, not derived.** `Tool.TCP_Offset` = (0, 0, 0.174) m, no rotation: the
   Polyscope TCP of SF1, identical to the Robotiq value (§4.4, §16). The YAML comment above the field
   cites both sources. Every FK check uses `FK_DH(q) · TCP_config`.
2. The derived TCP (`FK_DH(q)⁻¹ · T(p)`) is a **diagnostic only**. `Derive_TCP_Offset` exists to
   print it (§4.3). It never writes the configuration, never feeds a check, and
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
   flange ±x (§4.5). The taught program cannot discriminate the yaw (a consistent station fit
   exists at 0° and at 90°), so it is configured, never fitted.

### 9.4 FK accuracy: regression pin + independent sanity bound

The residual `FK_DH(q) · TCP_config` vs `_p` is kinematic error (§4.3): nominal DH vs the
robot's calibration. Two separate checks, with separate jobs:

1. **Regression pin (narrow band, test only):** the measured residuals with the configured TCP,
   home 1.72 mm / 0.261° and drop_rotate_1 4.02 mm / 0.376°, within ±0.01 mm / ±0.001°, and the
   diagnostic derived-TCP values of §4.3, within ±0.001 mm / ±0.001°. These numbers come from this measurement on purpose: the pin
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
   - `Check_Learned_Kinematics` applies the bound to every learned point a script reads, before
     it is used: `plan_rotation.py`, `write_variants.py` and `demo_visualize_position.py` all 78
     points of the extract, `replay_rotation.py` every learned (non-via) point of each variant file
     (`plan_rotation.Check_Learned`, the orientation bound from a robot-only scene at
     `Tool.Opening`, L_tool 176.7 mm → θ_max 3.245°). A point outside stops the script with
     `Kinematics_Error_Cls` and its numbers; the script prints the largest residual otherwise. A
     point is never accepted silently. All 78 points lie within it, the largest 4.04 mm
     (Waypoint_7) and 0.377° (Waypoint_1).

The largest measured 4.04 mm / 0.377° lies at 40 % / 12 % of the bound.

**Consistency with the clearance budget.** The sanity bound allows up to 10 mm at a learned point,
so the budget (§9.8) shows both the measured 4.0 mm and the bound's 10 mm.

### 9.5 Orientation comparison (requirement A)

- Always the geodesic angle, never component-wise rotation vectors. It is computed as
  `θ = atan2(‖vee(M − Mᵀ)‖ / 2, (tr M − 1) / 2)` with `M = R_aᵀ R_b`. The arccos form cannot resolve
  angles below ≈ 6e-8 rad (§4.5), and the 1e-9 test needs that resolution.
  `home_p` has |r| = 3.1393 rad ≈ π, where r and the equivalent −r′ differ component-wise by ≈ 2π
  while describing the same rotation. A test pins exactly this case.
- `Matrix_To_Rotvec` for **generated** points: θ ∈ [0, π]. θ < 1e-12 → (0, 0, 0). If π − θ < 1e-9
  the axis sign is chosen so that its first component with |·| > 1e-9 is positive. A learned point's
  `_p` is always written verbatim and never normalised.

### 9.6 Robot and gripper collision geometry; self-collision

- Robot: the convex hulls of the Menagerie visual meshes of each link (MuJoCo hulls every mesh
  geom). Each hull contains its visual part by construction. The Menagerie capsules are removed,
  since they do not necessarily contain the mesh.
- Coupling: the cylinder of §9.3 in body `base_mount`, which replaces the Menagerie disc.
- Gripper: the Menagerie 2F-85 collision geoms, at the joint angles that give the scene's opening:
  `Tool.Opening`, or for a job's segment its holding opening (`Tool.Holding_Openings`) or the open
  85 mm (§4.9). These are solved once at scene build by bisection on the simulated equilibrium of the gripper alone; the
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
  §4.5). It is **not** excluded; its self margin is 0 mm (owner, Q9 (a); true minimum
  2.3237 mm, §17).
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
  `dist < margin` are violations; `dist < 0` is a collision. The configuration is set with `mj_kinematics` +
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
    (`Near_Station`), where the zone margin applies. The station geometry is fitted, not
    measured, so a new path may not use it more tightly than the taught program does. In addition,
    every robot collision geom (mesh vertices, primitive box corners) stays inside
    `Clearance.Workspace`: the space outside the STL is not modelled, and the taught program proves
    free only what it uses.

  `Check_Configuration(q, kind, station_tcps)` is the rule of one sample; `Check_Segment` applies it
  to every sample and returns every violation; `Segment_Valid` applies it in van der Corput order
  (ends, middle, quarters, …) and stops at the first violation. Same samples, same rule: the verdict
  equals "`Check_Segment` finds nothing", pinned on seeded near-margin segments.

  **Machine accuracy margin** (generated segments; owner, 2026-10-18). The machine's position is
  estimated to ±10 mm (`Clearance.Machine_Extra`), so a generated sample keeps
  `Clearance.Environment` + 10 mm = 30 mm to the pieces of machine (a) outside the station zone
  (the members recorded in the manifest, the zone pieces excluded). Their geoms carry the 30 mm
  margin, so MuJoCo generates their contacts up to 35 mm; `Check_Clearance(q, machine_extra=True)`
  applies it, only for generated configurations. Taught segments, the station zone and every other
  object keep their margins.

  **Station-box rule** (generated segments; owner, 2026-10-18). The station boxes (the
  `Environment.Added_Boxes` that are zone pieces) are fitted to the taught program, not measured, so
  a generated path may not use them more tightly than the taught program does: no sample of a
  generated segment comes closer to a station box than the closer of its two taught ends, both
  capped at `Clearance.Environment` (`Station_Box_Floors(q_a, q_b)`; `Check_Configuration`,
  `Check_Segment` and `Segment_Valid` take the floors as `box_floors`). A violation names the box
  and the floor as its margin. The distances are MuJoCo's contacts, which agree with
  `mj_geomDistance` for these meshes (`test/Scene/test_margin.py`).

  A learned point is taught: `Check_Learned_Point` applies the zone margin to the zone pieces.
- **Start or goal inside the margin**: `Plan` refuses an end that breaks the generated rule, naming
  every violating pair, its distance and the margin. No silent pass, no silent failure, no override.
  No script holds a learned point to the margins when it reads it: the scripts check the FK bound
  (§9.4), and the viewers label a point by its rule. The grasp, load and unload points touch the
  parts and fixtures or lie inside machine (a): `Check_Learned_Point` (the taught rule, raising
  `Clearance_Error_Cls`; used by the tests) refuses 50 of the extract's 78 points at 50 mm and 41
  at 85 mm, none of case 17's.

**The taught model check** (`Check_Taught_Model`, `Example/plan_rotation.py`, owner 2026-10-21).
The station zone, the station boxes and the workspace top are fitted to the taught samples of
cases 17 and 18 (§4.10 – §4.13). Before a job is planned, its taught moves (`Taught_Moves`: every
move of the job's sequence after its first, with the program's `_q`, in the gripper state of the
move, a movel on its TCP line) are sampled as a segment check samples (§9.7) and measured under the
taught rule (`Environment_Clearance`). A sample inside modelled geometry (distance < 0: the real
robot ran there, so the model is not valid for the case there) stops the run before planning, one
`[ERROR]` per move naming the case, the program line, the robot body, the piece (with "fitted to
the taught samples of cases 17 and 18" for the station boxes and zone pieces) and the depth;
nothing is planned or written and nothing is refitted. Less room than the taught margin and a robot
geom above `Clearance.Workspace` are `[WARNING]`s: they are conservative, not unsafe. Case 17's
15 taught moves pass with no error and no warning (`test/Example/test_plan_rotation.py`). Only the
job's span is checked: the program's loading moves pass up to 140 mm into machine (a)'s filled
interior (§4.11, `KNOWN_ISSUES.md`), where the model is known not to be valid.

### 9.8 Clearance budget

`Clearance.Environment` = 20 mm must cover:

| Item | Case A: measured | Case B: sanity bound | Basis |
|---|---|---|---|
| Kinematic error (nominal DH vs the calibrated robot) | 4.0 mm | 10.0 mm | A: max measured over all 78 learned points, 4.04 mm at Waypoint_7 (§9.4). B: the most a learned point may have (§9.4) |
| Collision-set conservativeness | 1.0 mm | 1.0 mm | `eps` (§7.3) |
| Coupling | 0 mm | 0 mm | measured on the robot, 14.0 mm = drawing (§4.4) |
| **Remaining for cables** at the base and the tool (not modelled) | **15.0 mm** | **9.0 mm** | 20 − kinematic error − eps |

The error is measured at all 78 learned points (at most 4.04 mm at Waypoint_7, 0.377° at
Waypoint_1) and every learned point is checked at load. Case A is the expected case; case B is what
the checks can guarantee for learned points. At generated via points, and between points along a
path, the error cannot be measured, so neither case is guaranteed there (`KNOWN_ISSUES.md`).

Not in the budget, because they are not small bounded errors. Each is listed in `KNOWN_ISSUES.md`:

- the carried 50 × 50 mm part (not modelled, accepted by the owner; not covered by the gripper
  geometry, §9.6);
- Polyscope safety limits possibly tighter than ±360°.

Self-collision (5 mm) is not affected by cables or calibration at the gripper. Only eps applies.

**To machine (a) outside the station zone**, a generated path keeps 30 mm: the 20 mm above plus
10 mm for the accuracy of the machine's estimated position (owner, 2026-10-18, §9.7). The 10 mm
covers the position only; the budget above is unchanged. Taught segments are not held to it.

**Inside the station zone** (§9.7) the environment margin is 5 mm: eps (1 mm) + the measured
kinematic error at the drop points (4.0 mm) leaves **0 mm for cables**, and case B (10 mm) is **not
covered**. The zone holds only the holder, the stub and the sill frame (§4.12); the holder's top
and the stub's depth were fitted to keep the taught segments 5 mm clear of the nominal-DH robot
(§4.10, §4.13), so the zone margin pins consistency with the taught program, not real clearance. A generated path gets it only within
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
  `-0.5` → `-.500000000000`); a value that rounds to zero loses its sign (`.000000000000`). Their `_p` comes from DH FK · TCP with
  `Matrix_To_Rotvec` and is informational (§10.4).

**Program excerpt** (`Data/Programs/sf1_plc_v4/sf1_plc_v4_cases_17_18.script`, read by
`Read_Program_Excerpt` only): lines of the robot program kept verbatim, indentation included: the
whole body of each case's branch `elif (<n> == switch_1):` (case 17 lines 3397 – 3478, case 18
3480 – 3561), out of machine (a), the rotation sequence home → … → home, back into machine (a). Column-0 `#` lines before the first block are the header (source, sha256); a column-0 `# case <n>:` line opens
a case. Each program line is classified as a movej to a learned point
(`movej(get_inverse_kin(<name>_p, qnear=<name>_q), …)`), a movel to a learned pose, a gripper
action (`rq_set_pos_spd_for(0 | 255, …)`: open / close) or other (URCap boilerplate, Polyscope
markers). Strict: an unknown gripper position, a repeated or empty case is an error.

**Making the extracts** (`Example/extract_program.py --program PATH --cases N [N ...] --out
Data/Programs/<name>/`, owner 2026-10-21; writers `Extract_Positions` and `Extract_Cases` in
`src/URScript/Core.py`). It writes `<name>_positions.urscript`, every `global <name>_p` / `_q` line of
the program verbatim in program order, and `<name>_cases_<n>_<m>.script`, each case the whole body
of its switch branch `(el)if (<n> == <variable>):` up to the next line indented no deeper
(`Program_Cases`), verbatim. Both headers name the program, its line ranges and its sha256
(`Read_Extract_Source` reads the name and the hash back). Refused, with nothing written: a case the
program does not have, a point with only one of `_p` and `_q`, a name defined twice, a case already
held by another cases file of the folder, a cases file of the folder from another sha256. Both
extracts it writes are the committed ones byte for byte. With
`SF1_PLC_V4_SCRIPT` naming the program, the tests compare the extracts with the program, byte for
byte, and re-extracting cases 17 and 18 reproduces both committed files; otherwise those tests are
skipped.

### 10.2 Encoder noise

`2.44E-5` rad in `home_q` is encoder noise from teaching. It is read and written unchanged.

### 10.3 Naming convention

Learned points keep the program's names. Their single source is the positions extract of the job's
program, `Data/Programs/<program>/<program>_positions.urscript` (§10.1; `sf1_plc_v4` for every
shipped job). The tests' points A = `home` and B = `drop_rotate_1`.
Generated via points are `via_<from>_<to>_<nn>`, `nn` from `01`, e.g. `via_home_drop_rotate_1_01`.
The original robot export of A and B is kept as the test fixture
`test/URScript/data/SF1_Positions_original.urscript`; a test asserts that its number tokens equal
the extract's.

### 10.4 Motion output (requirement B)

```
movej(<name>_q, a=1.3962634015954636, v=1.0471975511965976, r=0)
```

`a` and `v` are written, as the tokens of the robot program
`sf1_plc_v4.script`, so a generated move runs at the speed the duration estimate assumes (§10.6).
Speed does not change the path of a `movej` with `r=0`. `r=0` is written explicitly because it is
the one argument that does change the path. Joint targets only, never `movej(p[...])`. `_p` of via
points is informational: the robot would otherwise solve its own IK and could pick another branch.

**Simplified taught program** (`Example/write_taught_programs.py`,
`Data/Programs/sf1_plc_v4/sf1_plc_v4_case_17.urscript` and `…_case_17_sequence.script`): the rotation
sequence of case 17, home → … → home (the default job's sequence, `Resolve_Sequence`: from the
case's first movej to home to its second, inclusive), as its motion program, written by `Write_Motion_Sequence` from `Program_Entries`:
every movej and movel in program order, verbatim without indentation (targets, a, v, r), each
gripper action as a `# Gripper open/close` comment naming the program line, nothing else (no URCap
code, no PLC I/O, no waits). The points file holds the points it moves to, with the program's
tokens.

**Exception: station lines copied verbatim.** In a motion sequence (`Write_Motion_Sequence`) the
station operations of the robot program are copied verbatim (without indentation), with their own
`movel(...)` and `movej(get_inverse_kin(<name>_p, qnear=<name>_q), …)` and their a, v, r. They are
the owner's tested program, not generated output; the gripper nodes are marked by comments.

### 10.5 Joint branches (requirement C)

Joint values are never wrapped into [−π, π]. UR joints span ±2π and `movej` goes to the exact
value. A generated segment stays on the branches of its two ends; a learned point takes another `_q`
only through the job's `Point_Configurations` (§8.6), with its source. A test feeds A and B through
parse → path [A, B] → write and asserts the tokens are unchanged, plus a synthetic point with a
joint at 4.0 rad that must come out as 4.0.

**One owner-approved exception** (2026-10-19, "q6 representative chosen to avoid an unnecessary
wrist-3 turn"): in c1 and in the delivered c2 the pick's three station points, Waypoint_18,
pick_rotate_1 and Waypoint_6, use wrist 3 one turn lower than taught (4.739 − 2π = −1.544 rad;
the default job's `Point_Configurations`, §8.6, applied by `Configured_Points` of `Example/plan_rotation.py`). The arm pose and the `_p` are
the program's; only that `_q` token changes (`Replace_Joints`, a generated token), every other token
is kept. S3 then turns wrist 3 by 1.5 rad instead of 4.7. The
program's pick lines stay verbatim: `get_inverse_kin(<name>_p, qnear=<name>_q)` takes the solution
nearest the new qnear on the robot, which the first run at reduced speed confirms (`KNOWN_ISSUES.md`).
No re-teaching: the pose is unchanged. No other joint is turned.

**The delivered drop on another IK branch** (owner, 2026-10-21: c2, delivered and tested on the
real cell). In the default job the drop's three points, drop_rotate_1, Waypoint_4 and Waypoint_5,
are `Branch: S- E- W-` entries (§8.6): each the nominal-DH IK of its `_p` on that branch, turned to
the representative nearest the `_q` of the point the output moves to just before it, as configured
(home → drop_rotate_1 → Waypoint_4 → Waypoint_5), which reproduces the tested c2 to 1e-11 rad
(`design-decisions.md`). The `_p` tokens are the program's; the program's drop lines stay verbatim
and take the new qnear.

### 10.6 Planner and the rotation-sequence output

S1 – S3 are optimised for time, not for clearance: the delivered c2 takes 12.05 s home → home
against the taught 23.37 s (estimates, `Data/Generated/variants/comparison.md`), using the space
the estimated machine shift frees while keeping 30 mm to machine (a) outside the zone (§9.7). The
taught S2 does not keep the generated rule's station-box floor: its last leg, Waypoint_3 →
Waypoint_18, comes 8.4 mm to the station stub (open, 85 mm), below the floor of 10.1 mm
(Waypoint_18's own distance), on 51 samples (`Example/plan_rotation.py` report). Under the taught
rule it is clear.

**Segments** (the job, §8.6; the default `sf1_plc_v4_case_17`, case 17, §1). S1 home →
drop_rotate_1 (holding: case 17's `Tool.Holding_Openings` entry, 50 mm), S2 Waypoint_5 →
Waypoint_18 (open, 85 mm: the full stroke for `rq_set_pos_spd_for(0, ...)`; the release opening is
real, owner 2026-10-18, §4.9), S3 Waypoint_6 → home (holding); the states are derived from the
case's gripper actions (§8.6). The ends are the program's `_q` tokens, never wrapped, except the
job's `Point_Configurations` (§10.5): S2 ends and S3 starts with the pick's wrist 3 one turn lower.
The fixed station points of the generated rule (§9.7), derived from the job's sequence, are case
17's six: drop_rotate_1, Waypoint_4, Waypoint_5, Waypoint_18, pick_rotate_1, Waypoint_6. Segment k
of the job plans with the seed 20261017 × 1000 + case × 10 + k.

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

**Parallel planning** (`Plan_All`, `--workers N`, owner 2026-10-19). `Plan` runs four steps
(`Prepare`, `Restart`, `Refine_Candidate`, `Finish`); each depends only on its segment, its own
generator (`[seed, restart]`, `[seed, 1000 + rank]`) and its arguments, and the result is chosen by
the same rule (`Rank_Found`, `Choose_Refined`). With `--workers N` > 1 they run in a pool of N
spawned processes, each with its own scene (MjModel / MjData) and planner, S1 – S3 at once: all
prepares, then the 60 restarts, then the 6 refinements, then the 3 final checks. The output is
byte-identical for every N. Planning takes about 857 s with 1 worker and 463 s with 8 (M2 Pro,
2026-10-19); the critical path is the refinement, 150 dependent steps per candidate, which more
workers cannot shorten without changing the output (`design-decisions.md`).

**Objective.** Minimise the estimated duration plus `w` × the clearance penalty; tie-break on fewer
via points.
- Estimated duration: the sum over segments of the trapezoid (or triangle) profile of the leading
  joint from rest to rest, a = 1.3962634015954636 rad/s², v = 1.0471975511965976 rad/s (the
  program's), r = 0. **An estimate**: the controller's profile, blends and the gripper are not
  modelled.
- Clearance penalty: the sum over the samples of every segment of max(0, 30 mm − d), d the smallest
  robot – environment distance (`Environment_Distance`), for samples whose TCP lies outside the
  station neighbourhood. The hard margins are unchanged; the penalty only prefers clearance above
  them. `w` = 0.5 s/m (owner, 2026-10-17, after the trade-off recorded in `design-decisions.md`),
  `CONST_PENALTY_WEIGHT` in `Example/plan_rotation.py`; `--penalty-weight` overrides it.

**Final verification.** `Verify_Path` re-checks every segment with the full generated-rule check
(`Check_Segment`, no early exit, no cache), at most 5 via points, within the joint limits; a path
that fails is never written.

**Output** (`Example/plan_rotation.py`, `Data/Generated/`):
- `SF1_Rotation_17.urscript`: the learned points the optimised sequence moves to, in the order of
  first use, then the via points `via_<from>_<to>_<nn>` (§10.1 format, `_p` informational). Its
  header names the learned points written verbatim from the extract (`_p` and `_q`) and those with
  the extract's `_p` and another `_q` (`Provenance`; delivered, c2: the drop's three on branch
  S− E− W− and the pick's three with wrist 3 one turn lower, §10.5); a learned point with another `_p` is never written. The status lines say why. Both
  files' headers state the machine margin with its source (`Clearance.Machine_Extra`, §8).
- `SF1_Rotation_17_Sequence.script`: **case 17 home → … → home** in program order, in the form of the
  simplified taught program (§10.4), so the two compare line by line: the station lines copied
  verbatim, the gripper nodes as comments, S1 – S3 as generated movej (§10.4) to their via points
  and ends (`Sequence_Entries`). The program's move to home blends into S1 (`r=0.01`) and a blend
  into a generated segment is not checked (§8.5), so the start is written as the generated
  `movej(home_q, a=…, v=…, r=0)` with a comment (owner, 2026-10-18). Its header names the station
  lines whose points have another `_q` (delivered, c2: the drop's movel and retreat, the pick and the
  lift). `Data/Generated/` is the default job's output, c2, delivered and tested on the real cell
  (owner, 2026-10-21).
- The script prints the machine margin with its source and, per segment, taught vs optimised: via points, estimated duration, Σ|Δq|,
  max-joint length, the smallest robot – environment distance and its robot body, the generated-rule
  violations, and the home → home totals (station operations estimated with the same model, movel
  on the TCP path at 1.2 m/s², 0.25 m/s). Every output says simulation passed ≠ safe on the robot.

**Variants** (`Example/write_variants.py`, `Data/Generated/variants/`, owner 2026-10-19). Every
variant of case 17 home → home is kept for comparison, each in `<name>/` as `SF1_Rotation_17.urscript`
and `SF1_Rotation_17_Sequence.script` (the formats above), each file stating its status:
- `taught`: the program's sequence (the simplified taught program).
- `b_optimised`: S1 – S3 planned on the program's points: an earlier optimisation (17.50 s).
- `c1`: the previous optimisation, simulation only (§10.5), planned as the job
  `sf1_plc_v4_case_17_c1` plans it (the same points and motion lines).
- `c2`: **delivered - tested on the real cell (owner, 2026-10-21)**; no further details are
  recorded. The default job `sf1_plc_v4_case_17`: the same points and motion lines as
  `Data/Generated/`. `c3`: **proposal for re-teaching, not verified on the robot.** Both put the drop's three points on
  another IK branch (§9.2): each the nominal-DH IK of its `_p` on the branch (c2 S− E− W−, c3
  S− E− W+), turned to the representative nearest the point before it (home first); the `_p` are the
  program's, the `_q` generated; the pick as in c1. c3 keeps home's branch; c2 flips the wrist
  (θ5 passes π in S1 while the part is held) and Waypoint_5 lies 8° from the wrist singularity
  (|sin θ5| = 0.14, a movej). c3's branch change means re-teaching and re-verifying the drop on the
  robot at reduced speed; a re-taught program arrives as a new export, extracted as before.

S1 – S3 of every planned variant are planned as `plan_rotation.py` plans them (the same seeds, budget,
weight; `Plan_All`, `--workers`), from the points as written (the generated `_q` tokens, 12 decimals).
`comparison.csv` and `comparison.md`, written by the same script from the variant files
(`Measure_Variant`, `--table-only` re-measures): per section (S1, the drop, S2, the pick, S3) and in
total, motion lines, distinct points, via points, the estimated duration (§10.6 model, every move
rest to rest with its line's a and v, r = 0; the taught blends listed as not modelled), Σ|Δq| and
the max-joint length (a movel along its solved TCP line), the TCP path length over the checked samples,
the largest TCP and robot heights (world z; `Robot_Top`), the closest robot – environment pair and
the pair with the least room above its rule's margin (`Environment_Clearance`), and the samples
breaking the rule of their move. Home → home (estimates): taught 23.37 s, b_optimised 17.50,
c1 14.66, c2 12.05, c3 13.86; the largest robot height 2.32 / 2.26 / 2.26 / 1.82 / 1.78 m; 0 samples
breaking the rule in every variant.

### 10.7 Viewers

All viewer scripts build the scene from the collision geometry only and start with geom group 3
shown; `--with-stl` loads the cell STL as well. Learned points come from the program extract only.

**`Example/demo_visualize_position.py`** shows the robot static at one learned position: `--pos
NAME` or `NAME_q` (a `NAME_p` is refused: the viewer shows joint positions, the robot's own IK would
decide the joints of a pose; an unknown name is refused), `--list` prints every name, `--opening
{holding, open}` (default holding; holding is case 17's `Tool.Holding_Openings` width). It prints and overlays the clearance under the taught rule, listing every
violating pair.

**`Example/replay_rotation.py`** replays one path of a numbered catalogue in one viewer window,
each case 17 home → home, read from `Data/Generated/variants/` (§10.6): 1 taught, 2 b_optimised,
3 c1 (previous optimisation, simulation only), 4 c2 (delivered - tested on the real cell, owner,
2026-10-21) and 5 c3 (proposal – not verified on the robot), 6 all five one after
another with the strict comparison. `--path N`
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
  program lines by the taught rule; a movel along its TCP line; the generated rule with the
  variant's own station points. Entry 6 also prints the strict comparison: S1 – S3 of every variant
  judged by the generated rule. `--skip-check` replays without the check (labelled so). The overlay
  shows the path, the variant with its estimated home → home time (c3 marked "proposal - not
  verified on the robot", `Variant_Title`, also printed; c2 unmarked), the move and its section, the gripper
  state and width, the estimated elapsed time and the label.

**`replay_rotation.py --job <name> [--out DIR]`** (owner, 2026-10-21) replays a job's output instead
of a catalogue entry: the job's `Output.Points` and `Output.Sequence` in `--out` (default
`Data/Generated/`), with the same moves, labels, timing and screenshots, the gripper starting in
the state of the job's first segment (derived from the program, §8.6), the station points the job's.
`--job` with `--path` is refused. The catalogue stays case 17's five variants; its start state
`CONST_ENTRY_STATE` (holding) is the default job's first segment state, pinned by a test.

**Machine (a) interior.** A configuration that comes within `Clearance.Environment` of the interior
hull of machine (a) (component 1638, `KNOWN_ISSUES.md`) is labelled *model not valid here*, never as
a collision: the hull fills the space the loading moves use (`Scene_Cls.Components_Within`). The
replayed home → home paths stay far from it; the position viewer shows the label at the loading points.

## 11. Tests

One sentence of scope per file; each test's docstring states what it pins. *slow* tests are
deselected by `-m "not slow"`.

| Test file | What it covers |
|---|---|
| `test/URScript/test_core.py` | The variable-file reader and writer (exact values, byte-identical round trip, Polyscope tokens, strict errors, generated format, no wrapping), the program excerpt and its case extracts (against the program when `SF1_PLC_V4_SCRIPT` is set), `Program_Entries`, `Resolve_Sequence` of case 17 and its refusals, `Read_Motion_Sequence`, and `Replace_Joints`. |
| `test/Configuration/test_core.py` | `Load_Configuration` and `Load_Job`: the shipped files load typed; every missing REQUIRED field and invalid value is named, all at once; an unknown key is a WARNING; an unknown job name lists the jobs. |
| `test/Kinematics/test_core.py` | DH FK, `Solve_IK_Analytic` / `IK_Branch` / `Nearest_Representative` / `Solve_IK_Near`, `Linear_Path`, the taught points' measured branches, the regression pin of §9.4 (residuals ±0.01 mm / ±0.001°, derived TCP ±0.001 mm, never read by a check), the sanity bound (position and orientation, `Check_FK_Consistency`), the configured TCP, the geodesic angle and `Matrix_To_Rotvec` near π. |
| `test/Scene/test_core.py` | The scene: base pose and yaw (a 90° copy pins degrees), MuJoCo FK == DH FK (1e-9), the coupling, gripper and pad-span hull geometry against the manual chain, the exclusion set and joint ranges, the placement of the taught grasp points (§4.6), A and B clear, the added boxes, the taught / generated / workspace rules, `Check_Learned_Point` refusing a point with its numbers, `Environment_Clearance`, `Robot_Top`, `Components_Within`, and every `Self_Pair_Overrides` entry's true minimum (2.3237 mm). |
| `test/Scene/test_station.py` | The taught rotation sequence of cases 17 and 18 against the station model: clear as taught, and as generated without the station-box floors; the station boxes (holder and stub) are zone pieces; the zone holds the boxes and exactly the sill-frame pieces; without the boxes every taught segment keeps the global margins; the early exit agrees with the full check; the station-box floors and the machine margin. |
| `test/Scene/test_program.py` | *slow*: every unique segment of the 21 cases (125) at 50 and 85 mm against the recorded penetration reference (`test/Scene/data/sf1_plc_v4_segments.json`): nothing new or more than 0.5 mm deeper, no self-penetration. |
| `test/Scene/test_margin.py` | MuJoCo's contacts against the exact distance: mesh hull vs mesh hull at 10 and 30 mm, and every geom pair near the station (pads, meshes, set pieces, added boxes) within 0.5 mm. |
| `test/Planner/test_core.py` | The planner: `Plan_All` equal with and without a pool, the duration and profile estimates, the committed paths pass `Verify_Path`, the delivered files' tokens and sequence, the station-box rule in `Plan` and `Verify_Path`, byte-identical reduced runs, the refusals, and the penalty counting only far from the station. |
| `test/Collision/test_core.py` | `Build_Collision_Set` on synthetic cubes (assembly and component overrides, the manifest record), the STL's three objects, the configured overrides, the crop bound, conservativeness, thickening, excess, and the STL hash refusal. |
| `test/Collision/test_conservativeness.py` | The pruned conservativeness computation equals the exhaustive one, and the recorded maxima are recomputed exactly (§17). |
| `test/Collision/test_excess.py` | The pruned surface distance equals the exhaustive one, and the recorded excess and flags of the largest pieces are recomputed. |
| `test/Example/test_demo.py` | The headless demo: only the chosen joint moves, within its limits, pre-checked on the dense samples; the label follows the check, and the shipped motion's label ("clearance checked") is pinned. |
| `test/Example/test_extract_program.py` | `extract_program.py` on a synthetic program, its refusals, and (with `SF1_PLC_V4_SCRIPT`) the committed extracts reproduced byte for byte. |
| `test/Example/test_write_taught_programs.py` | The simplified taught program is case 17 home → home as the owner states it, and the script writes the committed files byte for byte. |
| `test/Example/test_plan_rotation.py` | `plan_rotation.py`: refusals before anything is written, the delivered headers' provenance and machine margin, the holding opening per program case, `Sequence_Entries`, `Configured_Points`, the station time, the taught model check, synthetic jobs refused; *slow*: the full run, the c1 and c2 jobs reproducing their committed files, and case 18 planning without code changes at an assumed 50 mm and a reduced budget. |
| `test/Example/test_write_variants.py` | The five variants' files, statuses and provenance, c2 equal to `Data/Generated/`, the proposals' drop branches, and `comparison.csv`; *slow*: the table re-measured byte for byte. |
| `test/Example/test_visualize_position.py` | The position viewer, headless: the FK refusal, the taught-rule and "model not valid here" labels, refused names, `--list`. |
| `test/Example/test_replay.py` | The replay, headless: the FK refusal, every catalogue entry's moves, gripper states and movel lines, the timing, the refusals, path 6 (c3 marked a proposal), and `--job` replaying the default job's output (12.05 s). |
| `test/Example/test_make_readme_figures.py` | The figure script names its output from any directory; the collision-model groups cover the environment once; the renders never load the visual STL; *slow*: the graphs are the committed ones byte for byte. |
| `test/Docs/test_readme.py` | The README's results tables equal `comparison.csv`, every image it shows exists (and every `Media/` file is shown), each figure has an alt text and a numbered caption, and the root holds only what the README describes. |
| `test/Install/test_install.py` | `bash -n`, the OS detection, the Python 3.11 resolution (`python3` never called), the hash verification and the log format, under a hermetic `PATH`. |
| `.claude/skills/project-style-rpx/scripts/test_check_style.py` | The style checker itself (outside `test/`). |

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

- **project-style-rpx**: the code, shell, YAML, Markdown, URScript and MJCF conventions;
  `scripts/check_style.py` checks `src`, `Example` and `test`.
- **feature-manager**: how a feature, a behaviour change or a bug fix is added, with the robotics
  review checklist.
- **project-optimize**: cleanup, speed, complexity and reliability work that does not change
  behaviour; a generated path is compared byte for byte.

## 14. Risks and how they are handled

| # | Risk | Handling |
|---|---|---|
| 1 | Mesh collision = convex hull | crop → hull per component → CoACD only where coarse; surface metrics (§7) |
| 2 | Units / frames | STL in m (measured); base pose and yaw REQUIRED fields; Menagerie 180° verified (§4.2); base-position test; a 90° yaw test pins degrees in the file; the placement test of the taught points (§4.6) catches a wrong world yaw, which FK vs `_p` cannot |
| 3 | Models | Menagerie pinned commit, BSD-3/BSD-2, vendored, hash manifest; Menagerie offsets patched to DH (16.5 mm flange error found) |
| 4 | Sim vs real kinematics | DH truth; configured TCP (Polyscope = Robotiq manual), derived TCP diagnostic only (§9.3); coupling from the manual, measured on the robot (§9.3); regression pin + sanity bound (§9.4); no calibration file → residual in the clearance budget (§9.8) |
| 5 | A→B configuration change (base ~167°, elbow −120° → +83.8°) | generated moves are `movej` to joint targets only; no generated `movel` (the station's taught `movel` lines are copied verbatim, §10.4) |
| 6 | Check vs real motion | joint-linear dense sampling with a displacement bound; `r=0` fixed |
| 7 | Self-collision, limits, gripper | explicit exclusions, filterparent off; elbow ±180° as the precondition of the upper_arm–forearm exclusion; others ±360° (Polyscope limits possibly tighter); structural pair base–upper_arm not excluded, 0 mm self margin (Q9 (a)); gripper rigid at the segment's opening (holding or open 85 mm) with a pad-span hull; part beyond the pad ends not covered |
| 8 | ≤ 5 via points | `Plan` and `Verify_Path` raise `Planner_Error_Cls` beyond 5 via points or without a path, never return a longer or colliding path |
| — | Open shells over-filling space | flagged per piece (§7.4) |
| — | 88 MB STL, 1.76 M triangles as visual mesh | MuJoCo's STL loader refuses > 200 000 faces → passed as `uservert`/`userface`, no hull stored, compile 1.3 s (§4.5) |

## 15. Open questions

None. The limitations accepted instead of resolved are in `KNOWN_ISSUES.md`.

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

## 17. Implementation measurements

Measured on the development Mac (M2 Pro), Python 3.11, mujoco 3.16.0. The collision-set values are
recorded in `CAD/Collision/SF1_Environment_Collision.json`.

- **Components are split at manifold edges** (an edge of exactly two faces), as §4.1 was measured:
  2756 components. Splitting at shared vertices or at any shared edge merged the stand frame into one
  component, whose fallback hull reached 297 mm beyond it and failed the static-base check.
- **Conservativeness is the exact distance to the union** of a component's CoACD parts (numpy
  point-triangle distance, pieces skipped only when a facet-plane lower bound proves they cannot be
  closer; a seed per component). A check that stops at the first part within eps reports a value
  capped at eps (the first build reported 0.99998 mm), not the true maximum.
  `test/Collision/test_conservativeness.py` holds the pruned computation against the exhaustive one
  and recomputes the recorded maximum.
- **Collision set:** crop radius 2.026 m (all 2756 components kept); 6718 pieces: 2125 hull,
  503 thickened (all planar), 4089 CoACD, 1 fallback; bodies 4577 `env`, 2141 `env_stand`.
  Conservativeness over 1 601 339 points: true maximum 0.769 mm (component 2132), 0 points above
  0.9·eps. The fallback is component 1638 (an open shell of machine (a)), whose CoACD parts reach
  1.121 mm > eps. Excess: maximum 234.8 mm (that fallback hull), p95 11.3 mm; 396 pieces flagged
  above 10 mm. Build 1119 s (CoACD 935 s).
- **`Check_Clearance(q)`** over 1000 random `q` within the limits: mean 3.10 ms, p95 13.90 ms,
  maximum 55.4 ms; the generated rule 0.28 ms per planner sample (measured 2026-10-17, on the set
  before the machine shift). The §7.2 merge pass was not needed.
- **MuJoCo FK vs DH** over 1000 random `q`: ≤ 1e-15 m, ≤ 1e-15 rad.
- **`base – upper_arm_link` true minimum:** 2.3237 mm (grid 2.3263 mm), pinned by
  `test/Scene/test_core.py`; the reason for its 0 mm self margin (§9.6).
