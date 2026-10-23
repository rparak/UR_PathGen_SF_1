# Known issues

**Simulation passed ≠ safe on the robot.** Every path this tool generates is a proposal. Run it on
the real SF1 cell at reduced speed first, with someone at the stop button. This file lists what the
simulation does not model or has not verified. It contains only what has been decided or
measured.

## Kinematic error measured at the learned points only, unmeasured at generated via points

The tool uses the nominal UR10e DH parameters; there is no `calibration.conf` of this robot.
Comparing `DH_FK(q) · TCP` with the robot's own `_p` gives the difference between nominal and
calibrated kinematics. It is measured at all 78 learned points of the program extract: at most
4.04 mm (`Waypoint_7`) and 0.377° (`Waypoint_1`), measured 2026-10-21; `home` (A) 1.72 mm / 0.261°,
`drop_rotate_1` (B) 4.02 mm / 0.376°. Every learned point a script reads is checked at load against
the 10 mm sanity bound (SPEC §9.4); a point outside stops the script. At the generated via points,
and between points along a path, the error is unmeasured and may be larger.

- **Impact:** of the 20 mm environment margin, 4.0 mm is taken by the measured error, and up to
  10 mm by the sanity bound; what is left is for cables (see *Cables are not modelled*, SPEC §9.8).
- **Evidence:** SPEC §4.3 (A and B), SPEC §9.4 (all 78 points); `Example/plan_rotation.py` prints
  the largest residual of the 78 points on every run.
- **Pinned by:** `test/Kinematics/test_core.py::test_the_configured_tcp_residual_is_pinned` (A and B);
  the check at load by `test/Example/test_plan_rotation.py`, `test_write_variants.py`,
  `test_visualize_position.py` and `test_replay.py` (a point outside the bound is refused).
- **What would close it:** the robot's `calibration.conf` and FK with its corrections.

## No measurement of the cell; c2 is the only generated path run on the robot

No measurement of the cell is available (owner, 2026-10-17). Every result of this tool, including
everything below, is a simulation of a model that was in part fitted to the robot program, not
measured. The one generated path run on the robot is c2, delivered and tested on the real cell
(owner, 2026-10-21); no further details are recorded.

## The rotation-station geometry is approximate: the station boxes fitted to the taught program, the post placed by an estimate

At `Base_Yaw` = 180° the taught rotation sequence (cases 17 and 18, home → … → home) collides with
the original STL although the real robot runs it. Machine (a) is shifted −150 mm along x as an
assembly (the owner's estimate, next item); with that the post (left window profile 2710) needs no
fit: the taught sequence keeps ≥ 39.1 mm to it (SPEC §4.11). **Nothing in the program constrains
where the post is; its position is only the owner's estimate.** The visual mesh is unchanged, so the
viewer shows the original post and machine front, not the collision geometry.

Inside the station zone (`Clearance.Station_Zone`: the holder, the stub and the sill frame they sit
on, SPEC §4.12) the environment margin is 5 mm instead of 20 mm: for taught segments everywhere, for
a generated path only while its TCP lies within 50 mm of a station point. Everything else, the post
included, takes the full 20 mm.

- **Impact:**
  - **0 mm is left for cables inside the zone** (5 mm = 1 mm eps + 4.0 mm measured kinematic error);
  - **case B (10 mm sanity bound) is not covered** there;
  - **the station boxes pin consistency with the taught program, not real clearance:** each is as
    large as the taught program allows at 5 mm (the holder 5.53 mm, the stub 5.03 mm), by
    construction;
  - **if the machine stands closer than estimated**, the post is closer than modelled; the taught
    program keeps ≥ 39.1 mm to the model, so a real post up to about 34 mm closer is still consistent
    with it. A generated path keeps 20 mm to the modelled post, not more.
- **The holder is an estimate** (`station_bracket`, SPEC §4.10): its footprint from a photo, its
  front assumed, its top (1.112 m) the highest the taught sequence allows. Its arms beside the part
  are not modelled.
- **The stub is an estimate** (`station_stub`, SPEC §4.13): y and z from the photo, its depth the
  largest the taught program allows (34.6 mm proud of the post). The real stub may be smaller; it
  cannot be larger without contradicting the taught program. Run the robot near the drop at reduced
  speed first.
- **The open gripper is 85 mm**, the real release opening (owner, 2026-10-18).
- **Evidence:** SPEC §4.10 (holder), §4.11 (−150 mm, no fit, the per-segment table), §4.12 (the
  zone), §4.13 (stub).
- **Pinned by:** `test/Scene/test_station.py` (the replay of both cases as taught and as generated,
  the recorded overrides, the zone pieces, the station boxes) and
  `test/Planner/test_core.py::test_no_generated_sample_comes_closer_to_a_station_box_than_its_taught_end`.
- **Another case or program is guarded, not fitted:** `Check_Taught_Model` (SPEC §9.7) stops a job
  whose taught line passes through modelled geometry, but sees only where the taught program ran:
  geometry fitted to cases 17 and 18 that a new case does not touch (a station box too large for its
  path, an endpoint radius) is not checked, and nothing is refit.
- **What would close it:** measuring the machine's position, the post, the holder and the stub on
  the real cell, then correcting the STL and removing the overrides and the boxes.

## The positions of machine (a) and the side cabinet are the owner's estimates, not measurements

Machine (a), all 1749 components, is shifted −150 mm along x as one rigid assembly, and the side
cabinet (c), 56 components, −50 mm along x as another (`Environment.Assembly_Overrides`): estimates
of the owner (2026-10-18), normal to the machine front. The robot stand and the tables stay. The owner's tape readings were rough, and nothing is
fitted to them (SPEC §4.8). After the shift, the robot-table edge → lower cabinet front is 276 mm
(tape about 300 mm) and the base axis → right window profile 762 mm (tape about 700 mm).

- **Impact:**
  - if either stands closer than estimated, every clearance to it is smaller than reported. The
    owner estimates the machine's position to ±10 mm: generated paths keep 30 mm to machine (a)
    outside the station zone (`Clearance.Machine_Extra`); taught segments and the station zone are
    not covered by it, and a larger error is not;
  - **the space the shift frees exists only in the estimate:** the taught program does not reach it
    (it keeps ≥ 39.1 mm to the post, and no segment comes within 50 mm of the side cabinet, SPEC
    §4.11). **A generated path that uses it must first run on the robot at reduced speed**, with
    someone at the stop button.
- **Pinned by:** `test/Scene/test_station.py::test_the_corrections_are_recorded_and_configured`,
  `test/Collision/test_core.py::test_the_configured_shift_moves_the_machine_and_the_side_cabinet_only`.
- **What would close it:** measuring the machine front (cabinet and window profiles) and the side
  cabinet relative to the robot base, then correcting the STL.

## Machine (a) interior is filled by open-shell hulls; real loading paths penetrate it; the model is not usable inside the machine

The grinder-loading segments of `sf1_plc_v4.script` reach into machine (a), and the model there is a
single convex hull: component 1638, the set's only fallback piece (its CoACD parts reached 1.121 mm,
above eps 1 mm, so its hull was kept), flagged as an open shell, 234.8 mm excess. Real segments penetrate it by up to
140 mm: load2 ↔ load3 (−74.2 mm, in every case 1 – 21), load_hower / rect_pick_hower → load3,
and the bench segments through Waypoint_9/10/12/13/14/15/16/17/19 to bench_drop_* / bench_sqr_pick
(down to −140.4 mm; cases 1 – 8, 17, 18, 20, 21). Not corrected (out of scope, owner 2026-10-17).

Two other kinds of penetration in the program are expected and not errors of the machine model:
the grasp and unload points reach into the modelled parts on the pallet and side tables
(components 191 – 206, one per case, down to −19.9 mm), and the fixtures there (components 2712 –
2718, 236, 237, 243, 244) are touched by up to 3.3 mm.

- **Impact:** no path inside machine (a) can be checked or planned; a result there means nothing.
  The delivered S1 – S3 (c2, home ↔ station) keep 227.9 mm or more to component 1638 (closest: S2
  at Waypoint_5, measured 2026-10-23 on every sample), and component 1638 is outside the station
  zone.
- **Evidence:** replay of all 125 unique segments of the 21 cases (the 4 home → first target entries
  included), joint-linear, 50 and 85 mm openings, on the current set (SPEC §4.11).
- **Pinned by:** `test/Scene/test_program.py::test_no_program_segment_penetrates_more_than_recorded`
  (the 63 penetrating segments and their depths are its reference: none may be added or deepen).
- **What would close it:** a decomposition of component 1638 that passes the conservativeness check
  (or a measured, simplified machine interior), then a replay of the loading segments.

## Generated transfer segments: estimates, margins without reserve, unproven space

The optimised segments S1 - S3 of case 17's rotation sequence (`Data/Generated/`, c2, the
deliverable) pass the full generated-rule check in simulation. What that does not cover:

- **Durations are estimates:** a trapezoid on the leading joint with the program's a and v, a stop
  at every via point; the controller's profile, blends and the gripper's time are not modelled. The
  cycle-time gain (23.37 -> 12.05 s home -> home, estimate) is not measured.
- **Clearance has no reserve beyond the margins:** the hard limits are 20 mm, 30 mm to machine (a)
  outside the station zone (±10 mm position accuracy), and 5 mm to the zone's pieces within 50 mm
  of a station point. The clearance penalty (weight 0.5 s/m, the owner's choice) only prefers
  clearance below 30 mm. With its TCP outside the station neighbourhood c2 keeps 52.9 mm (S1),
  34.7 mm (S2) and 59.4 mm (S3); its least room above a margin is 10.9 mm, to machine (a) in S1
  (40.9 mm), and S2 passes the stub at 10.1 mm, as close as its taught end (measured 2026-10-23;
  `Data/Generated/variants/comparison.md`).
- **The optimised paths depend on the estimated machine position:** they keep 30 mm to it, which
  covers the stated ±10 mm, not the whole shift: with the machine at the earlier estimate (−100 mm,
  post fitted) the paths planned 2026-10-18 would have penetrated the top profile by up to 18.5 mm
  (measured before c2; not measured for c2). The space they use is freed only by the owner's
  estimate of a further 50 mm, which nothing in the program confirms; the owner judges it closer to the real cell
  (2026-10-18), still unmeasured. **Run them on the robot at reduced speed first** (see "The
  positions of machine (a) and the side cabinet").
- **Space:** a generated segment stays inside `Clearance.Workspace` (the STL's x/y extents, up to
  2.35 m, the height the taught program reaches); what lies outside the STL is unknown.
- **The carried part, cables, Polyscope safety limits** as elsewhere in this file.
- **Pinned by:** `test/Planner/test_core.py` (the committed paths pass `Verify_Path`).

## The pick's wrist 3 is one turn lower than taught: unconfirmed on the robot

The delivered output gives Waypoint_18, pick_rotate_1 and Waypoint_6 wrist 3 = −1.544 rad, one turn
below the taught 4.739 rad (owner, 2026-10-19, SPEC §10.5). The pose and `_p` are unchanged; the
program's pick lines solve `get_inverse_kin(<name>_p, qnear=<name>_q)` on the robot.

- **Not verified:** that the controller's IK, given the new qnear, returns the configuration one turn
  lower (the simulation drives `_q` directly), and that the gripper cable and any wrist-3 safety
  limit allow it. c2 was tested on the real cell, but whether that test ran the pick lines is not
  recorded (owner, 2026-10-23). The tool cable is not modelled; over the cycle wrist 3 now spans
  −1.544 … 1.636 rad at c2's points, against 0 … 4.74 rad taught, so it winds less, not more.
- **Impact:** the first run at reduced speed, with someone at the stop button, watches wrist 3 at
  the pick (Waypoint_18 and pick_rotate_1) and on S3.
- **Pinned by:** `test/Example/test_plan_rotation.py::test_the_delivered_points_turn_only_the_picks_wrist_3`,
  `test/Planner/test_core.py::test_the_station_points_keep_their_tokens`.
- **What would close it:** the first run on the robot.

## The proposal c3 is a nominal-DH branch of the taught drop, never taught

`Data/Generated/variants/c3` puts the drop's three points on another IK branch (S− E− W+, home's;
SPEC §10.6): the nominal-DH IK of the program's `_p`, 13.86 s home -> home (estimate). c2, built
the same way on S− E− W−, ran on the real cell (previous items).

- **Not verified:** the robot's calibrated IK gives other joints (the drop's FK residual is 4.0 mm,
  SPEC §9.4), so its station clearances (12.0 mm; the taught branch 10.6 mm) are model values;
  cables and the carried part are not modelled.
- **Impact:** c3 may not run as written. Using it means re-teaching the drop on that branch and
  re-verifying it on the robot at reduced speed; the re-taught program then replaces the proposal.
- **Pinned by:** `test/Example/test_write_variants.py::test_a_proposal_moves_the_drop_to_its_branch_and_nothing_else`.
- **What would close it:** re-teaching the drop on that branch on the robot (owner, 2026-10-19:
  not now).

## Generated moves run at sf1_plc_v4's movej a and v only; a job at another speed is refused

The generated `movej` lines are written with the program's `a` = 1.3962634015954636 rad/s² and
`v` = 1.0471975511965976 rad/s (`CONST_MOVEJ_A_TOKEN` / `_V_TOKEN`, `src/Planner/Core.py`), and the
planner's duration objective uses the same values. A job whose taught segment lines move at another
`a` or `v` is refused before planning (`Load_Job_Plan`, SPEC §8.6), rather than written at a speed
its estimate did not assume.

- **Impact:** a program taught at other joint speeds cannot be planned until the tokens become a
  per-job value; the station lines, copied verbatim, keep their own speeds.
- **Evidence:** every taught `movej` inside S1 – S3 of cases 17 and 18 uses these tokens (the
  extract, 2026-10-21).
- **Pinned by:** the check in `Load_Job_Plan`, `Example/plan_rotation.py`.
- **What would close it:** the segment's taught `a` and `v` carried into its planning request and
  its written moves.

## The replay's timing is an estimate

`Example/replay_rotation.py` plays each move for its estimated duration (the trapezoid of SPEC
§10.6 with the line's a and v). Blends are not replayed (every move stops), and the gripper's time
and the program's waits are left out, so the elapsed time shown is not the robot's.

- **Impact:** the replay shows the order, the shape and the relative timing of the moves; real cycle
  times must be measured on the robot.
- **Pinned by:** `test/Example/test_replay.py::test_the_speed_scales_the_playback`.
- **What would close it:** a cycle-time measurement on the robot.

## movel is checked joint-linear by the station tests, on its TCP line by the replay

The robot moves the TCP of a movel on a straight line; the station tests (`test/Scene/test_station.py`)
check the taught movel joint-linear, the replay checks it on the TCP line (`Linear_Path`). Measured
2026-10-18, the two differ by at most 0.0014 / 0.0009 rad per joint on the drops and 0.025 / 0.030 rad
on the lifts (case 17 / 18); both keep their margins.

- **Impact:** a margin decided at the lift could differ between the two checks; none does today.
- **What would close it:** checking movel on the TCP line in the station tests as well.

## The carried part is not modelled

The gripper holds a 50 × 50 mm part of negligible mass during transit (owner); it is not in the
scene. The pad-span hull (one convex hull over both pads at the 50 mm opening) covers the space
between the fingers. **A part protruding beyond the pad ends, or beyond the pads sideways, is not
covered.**

- **Impact:** check on the robot that the part does not stick out of the fingers' envelope.
- **Pinned by:** `test/Scene/test_core.py::test_the_coupling_replaces_the_menagerie_disc` (pad-span hull).
- **What would close it:** a part model attached to the gripper.

## Joint limits are the hardware range, not the cell's safety configuration

Configured: ±360° for base, shoulder and wrists, ±180° for the elbow. The elbow limit is the
precondition for excluding `upper_arm_link – forearm_link` from self-collision. The Polyscope safety
joint limits of SF1 may be tighter.

- **Impact:** a path may use joint ranges the robot's safety configuration forbids. Check every
  target against Installation → Safety → Joints before running.
- **Pinned by:** `test/Configuration/test_core.py` (elbow beyond ±180° refused).

## Cables are not modelled

The cables at the base and at the tool are not in the scene. They are covered only by what the
environment margin leaves after the kinematic error and the collision-set tolerance: 15.0 mm in
the measured case, 9.0 mm at the sanity bound, and 0 mm inside the station zone (SPEC §9.8).

## Open shells filled by their hulls

A convex hull of an open shell (sheet-metal housing, frame) fills its inside. The build lists every
piece whose excess exceeds 10 mm in `CAD/Collision/SF1_Environment_Collision.json`
(`flagged_open_shells`). The robot cannot pass through those pieces in the simulation, even where
the real cell has free space.

- **Impact:** a path that would need such space is not found. No such path is ever reported as
  collision-free.
- **Pinned by:** `test/Collision/test_core.py::test_no_robot_body_at_a_or_b_is_inside_a_flagged_piece`.

## `base – upper_arm_link` has a 0 mm self margin

The hulls of these two bodies pass within 2.3237 mm of each other and never touch (owner, Q9 (a)).
This one pair is checked for contact only, not for the 5 mm self margin.

- **Pinned by:** `test/Scene/test_core.py::test_the_base_upper_arm_override_has_a_true_minimum_above_zero`.

## Ubuntu install path not verified

`install.sh` detects Linux and looks up `python3.11` on PATH. This has been tested only against
stubs, never on an Ubuntu machine.
