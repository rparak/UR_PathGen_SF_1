# Known issues

**Simulation passed ≠ safe on the robot.** Every path this tool generates is a proposal. Run it on
the real SF1 cell at reduced speed first, with someone at the stop button. This file lists what the
simulation does not model or has not verified. It contains only what the skeleton decided or
measured.

## Kinematic error measured at two poses only

The tool uses the nominal UR10e DH parameters; there is no `calibration.conf` of this robot.
Comparing `DH_FK(q) · TCP` with the robot's own `_p` gives 1.72 mm / 0.261° at `home` (A) and
4.02 mm / 0.376° at `drop_rotate_1` (B). That is the difference between nominal and calibrated
kinematics. Along a path, and at generated via points, the error is unmeasured and may be larger.

- **Impact:** of the 20 mm environment margin, 4.0 mm is taken by the measured error, which leaves
  15.0 mm for cables. If the error reached the 10 mm sanity bound, 9.0 mm would be left (SPEC §9.8).
- **Evidence:** SPEC §4.3.
- **Pinned by:** `test/Kinematics/test_core.py::test_the_configured_tcp_residual_is_pinned`.
- **What would close it:** the robot's `calibration.conf` and FK with its corrections.

## Simulation only: no real robot is available

No real robot and no measurement of the cell are available (owner, 2026-10-17). Every result of this
tool, including everything below, is a simulation of a model that was in part fitted to the robot
program, not measured.

- **Impact:** any new path through the station region must first run on the robot at reduced
  speed, with someone at the stop button, before it is used.

## The rotation-station geometry is approximate: the holder fitted to the taught program, the post placed by an estimate

At `Base_Yaw` = 180° the taught rotation sequence (cases 17 and 18, home → … → home) collides with
the original STL although the real robot runs it. Machine (a) is shifted −150 mm along x as an
assembly (the owner's estimate, below). With that the left window profile 2710 needs no fit: the
taught sequence keeps ≥ 39.1 mm to it (SPEC §4.11). *Until 2026-10-18 (machine at −100 mm) the
post was translated a further −15.7 mm, fitted so that the sequence kept 5 mm (SPEC §4.9).* The post
now stands 34.3 mm behind that fit: **nothing in the program constrains where it is; its position is
only the owner's estimate.** The visual mesh is unchanged, so the viewer shows the original post and
machine front, not the collision geometry.

Inside the station zone (`Clearance.Station_Zone`: the post, the machine-front pieces, the window's
top profile, the holder) the environment margin is 5 mm instead of 20 mm: for taught segments
everywhere, for a generated path only while its TCP lies within 50 mm of a station point.

- **Impact:**
  - **0 mm is left for cables inside the zone** (5 mm = 1 mm eps + 4.0 mm measured kinematic error);
  - **case B (10 mm sanity bound) is not covered** there;
  - **the holder pins consistency with the taught program, not real clearance:** its top is the
    highest at which the taught pick keeps 5 mm (5.5 mm, the open finger), by construction;
  - **if the machine stands closer than estimated**, the post is closer than modelled; the taught
    program keeps ≥ 39.1 mm to the model, so a real post up to about 34 mm closer is still consistent
    with it.
- **Also assumed:** movel segments checked joint-linear (the robot moves the TCP on a line); blended
  corners (r > 0) checked through the corner, not along the blend; nominal DH (4.0 mm error at the
  drop); the open gripper at 85 mm — the release opening is real (owner, 2026-10-18).
- **The station holder is an estimate** (`Environment.Added_Boxes`, `station_bracket`, source "owner
  photo + taught points, back to the shifted post (owner, 2026-10-18), not measured", SPEC §4.10,
  §4.11): one box under the part, world-fixed where the robot places the part. Its footprint comes
  from the photo, its front (90 mm in front of the earlier post face) is assumed, its back reaches
  into the shifted post (it is mounted on it), and its top (1.112 m) is the highest the taught
  sequence allows. The holder's arms beside the part are not modelled.
- **The stub is not modelled.** With the post fitted (−100 mm) the photo's stub contradicted the
  taught drop at its estimated position. With the post at −150 mm it no longer does: a stub on the
  post face could stand up to 34.6 mm proud and every taught sample would keep 5 mm (SPEC §4.11).
  It is still not modelled (not decided); a generated path may therefore pass where the real stub
  stands. Run the robot near the drop at reduced speed first.
- **Evidence:** SPEC §4.7 (first fit), §4.9 (−100 mm and the post fit), §4.10 (holder, stub), §4.11
  (−150 mm, the fit removed, the per-segment table).
- **Pinned by:** `test/Scene/test_station.py` (the replay of both cases as taught and as generated,
  the recorded overrides, the zone pieces, the holder) and
  `test/Planner/test_core.py::test_no_generated_sample_comes_closer_to_a_station_box_than_its_taught_end`.
- **What would close it:** measuring the machine's position, the post, the holder and the stub on
  the real cell, then correcting the STL and removing the overrides and the box.

## The positions of machine (a) and the side cabinet are the owner's estimates, not measurements

Machine (a), all 1749 components, is shifted −150 mm along x as one rigid assembly, and the side
cabinet (c), 56 components, −50 mm along x as another (`Environment.Assembly_Overrides`): estimates
of the owner (2026-10-18; the machine first 100 mm, then a further 50 mm), normal to the machine
front. The robot stand and the tables stay. The owner's tape readings were rough, and nothing is
fitted to them (SPEC §4.8). After the shift, the robot-table edge → lower cabinet front is 276 mm
(tape about 300 mm) and the base axis → right window profile 762 mm (tape about 700 mm).

- **Impact:**
  - if either stands closer than estimated, every clearance to it is smaller than reported;
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
single convex hull: component 1638, the set's only fallback piece (CoACD missed eps by 1.12 mm, so
its hull was kept), flagged as an open shell, 234.8 mm excess. Real segments penetrate it by up to
140 mm: load2 ↔ load3 (−74.2 mm, in every case 1 – 21), load_hower / rect_pick_hower → load3,
and the bench segments through Waypoint_9/10/12/13/14/15/16/17/19 to bench_drop_* / bench_sqr_pick
(down to −140.4 mm; cases 1 – 8, 17, 18, 20, 21). Not corrected (out of scope, owner 2026-10-17).
*Updated 2026-10-18: with the machine shifted 150 mm (100 mm: −99.5 and −145.0 mm; unshifted: −108.6
and −154.1 mm).*

Two other kinds of penetration in the program are expected and not errors of the machine model:
the grasp and unload points reach into the modelled parts on the pallet and side tables
(components 191 – 206, one per case, down to −19.9 mm), and the fixtures there (components 2712 –
2718, 236, 237, 243, 244) are touched by up to 3.3 mm.

- **Impact:** no path inside machine (a) can be checked or planned; a result there means nothing.
  The rotation-sequence segments S1 – S3 (home ↔ station) keep at least 181.2 mm to every one of
  these pieces (closest: case 17 S2 to component 1638), and component 1638 is outside the station
  zone.
- **Evidence:** replay of all 125 unique segments of the 21 cases (the 4 home → first target entries
  included), joint-linear, 50 and 85 mm openings, on the set of each date (SPEC §4.9, §4.11).
- **Pinned by:** `test/Scene/test_program.py::test_no_program_segment_penetrates_more_than_recorded`
  (the 63 penetrating segments and their depths are its reference: none may be added or deepen).
- **What would close it:** a decomposition of component 1638 that passes the conservativeness check
  (or a measured, simplified machine interior), then a replay of the loading segments.

## Generated transfer segments: estimates, margins without reserve, unproven space

The optimised segments S1 - S3 of case 17's rotation sequence (`Data/Generated/`, the only
deliverable) pass the full generated-rule check in simulation. What that does not cover:

- **Durations are estimates:** a trapezoid on the leading joint with the program's a and v, a stop
  at every via point; the controller's profile, blends and the gripper's time are not modelled. The
  cycle-time gain (about 6 s home -> home) is not measured.
- **Clearance has no reserve beyond the margins:** the hard limits are 20 mm, and 5 mm within
  50 mm of a station point. The clearance penalty (weight 0.5 s/m, the owner's choice)
  keeps the optimised paths at 29.7 mm or more outside the station neighbourhood (re-planned
  2026-10-18 on the shifted machine; 26.6 mm on 2026-10-17); without it they went down to 20.4 mm
  (2026-10-17).
- **Space:** a generated segment stays inside `Clearance.Workspace` (the STL's x/y extents, up to
  2.35 m, the height the taught program reaches); what lies outside the STL is unknown.
- **The carried part, cables, Polyscope safety limits** as elsewhere in this file.
- **Impact:** run every generated path on the robot at reduced speed first, with someone at the
  stop button; no real robot is available.
- **Pinned by:** `test/Planner/test_core.py` (the committed paths pass `Verify_Path`).

## The replay's timing is an estimate

`Example/replay_rotation.py` plays each move for its estimated duration (the trapezoid of SPEC
§10.6 with the line's a and v). Blends are not replayed (every move stops), and the gripper's time
and the program's waits are left out, so the elapsed time shown is not the robot's. The gripper
state is not an estimate: the robot holds the part from the start of the case, places it into the
magazine and releases it, picks it up again and returns home holding it; the gripper is open only
between the release and the re-pick (S2) (owner fact, 2026-10-18).

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
the measured case, 9.0 mm at the sanity bound (SPEC §9.8).

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
