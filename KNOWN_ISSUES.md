# Known issues

**Simulation passed ≠ safe on the robot.** Every path this tool generates is a proposal. Run it on
the real SF1 cell at reduced speed first, with someone at the stop button. This file lists what the
simulation does not model or has not verified. It contains only what the skeleton decided or
measured.

## Kinematic error measured at two poses only

The tool uses the nominal UR10e DH parameters; there is no `calibration.conf` of this robot.
Comparing `DH_FK(q) · TCP` with the robot's own `_p` gives 1.72 mm / 0.261° at `home_safe` and
4.02 mm / 0.376° at `drop_rot_01`. That is the difference between nominal and calibrated
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

## The rotation-station geometry is approximate: fitted to the taught program, not measured

At `Base_Yaw` = 180° the taught rotation sequence (cases 17 and 18, home → … → home) collided with
the original STL although the real robot runs it. Two components are translated before the collision
set is built (`Environment.Component_Overrides`, source "derived from sf1_plc_v4.script cases 17/18,
not measured"): the left window profile 2710 by (−114.8, −6.2, −0.6) mm, which puts it nearly flush
with the machine front, and the machine body 1154 by (−32.6, −11.6, +3.6) mm. Each is the smallest
translation that keeps every segment of the sequence 5 mm clear; no segment of the other cases got
new or deeper penetration (SPEC §4.7). The visual mesh is unchanged, so the viewer shows the
original post and machine front, not the collision geometry.

Inside the station zone (`Clearance.Station_Zone`: the post, the machine-front pieces, the window's
top profile) the environment margin is 5 mm instead of 20 mm: for taught segments everywhere, for a
generated path only while its TCP lies within 50 mm of a station point.

- **Impact:**
  - **0 mm is left for cables inside the zone** (5 mm = 1 mm eps + 4.0 mm measured kinematic error);
  - **case B (10 mm sanity bound) is not covered** there;
  - **the fitted geometry pins consistency with the taught program, not real clearance:** the
    taught segments keep ≥ 5.2 mm by construction;
  - what the translations hide: a real protrusion of the post up to about 110 mm, at every height,
    and a 35 mm placement error of the machine body.
- **Also assumed in the fit:** movel segments checked joint-linear (the robot moves the TCP on a
  line); blended corners (r > 0) checked through the corner, not along the blend; nominal DH (4.0 mm
  error at the drop); the open gripper at 85 mm — assumption: full 85 mm stroke for
  `rq_set_pos_spd_for(0, ...)`; conservative; owner did not confirm (the URCap sets
  `set_open_mm(50.0, 1)`).
- **Evidence:** SPEC §4.7 (fit, all-cases check, per-segment table).
- **Pinned by:** `test/Scene/test_station.py` (the replay of both cases as taught, the generated
  rule, the recorded overrides, the zone pieces).
- **What would close it:** measuring the post, the machine front and the station bracket on the
  real cell, then correcting the STL and removing the overrides.

## Machine (a) interior is filled by open-shell hulls; real loading paths penetrate it; the model is not usable inside the machine

The grinder-loading segments of `sf1_plc_v4.script` reach into machine (a), and the model there is a
single convex hull: component 1638, the set's only fallback piece (CoACD missed eps by 1.12 mm, so
its hull was kept), flagged as an open shell, 234.8 mm excess. Real segments penetrate it by up to
154 mm: load2 ↔ load3 (−108.6 mm, in every case 1 – 21), load_hower / rect_pick_hower → load3,
and the bench segments through Waypoint_9/10/12/13/14/15/16/17/19 to bench_drop_* / bench_sqr_pick
(down to −154.1 mm; cases 1 – 8, 17, 18, 20, 21). Not corrected (out of scope, owner 2026-10-17).

Two other kinds of penetration in the program are expected and not errors of the machine model:
the grasp and unload points reach into the modelled parts on the pallet and side tables
(components 191 – 206, one per case, down to −19.9 mm), and the fixtures there (components 2712 –
2718, 236, 237, 243, 244) are touched by up to 3.3 mm.

- **Impact:** no path inside machine (a) can be checked or planned; a result there means nothing.
  The rotation-sequence segments S1 – S3 (home ↔ station) keep at least 90.2 mm to every one of these
  pieces (closest: Waypoint_5 → Waypoint_1 to component 1638), and component 1638 is outside the
  station zone.
- **Evidence:** scratch replay of all 121 unique segments of the 21 cases on the rebuilt set,
  2026-10-17 (joint-linear, 50 and 85 mm openings).
- **What would close it:** a decomposition of component 1638 that passes the conservativeness check
  (or a measured, simplified machine interior), then a replay of the loading segments.

## Generated transfer segments: estimates, margins without reserve, unproven space

The optimised segments S1 - S3 of the rotation sequence (`Data/Generated/`) pass the full
generated-rule check in simulation. What that does not cover:

- **Durations are estimates:** a trapezoid on the leading joint with the program's a and v, a stop
  at every via point; the controller's profile, blends and the gripper's time are not modelled. The
  cycle-time gain (about 6 s home -> home per case) is not measured.
- **Clearance has no reserve beyond the margins:** the hard limits are 20 mm, and 5 mm within
  50 mm of a station point. The clearance penalty (weight 0.5 s/m, the owner's choice)
  keeps the optimised paths at 26.6 mm or more outside the station neighbourhood; without it they
  went down to 20.4 mm.
- **Space:** a generated segment stays inside `Clearance.Workspace` (the STL's x/y extents, up to
  2.35 m, the height the taught program reaches); what lies outside the STL is unknown.
- **The carried part, cables, Polyscope safety limits** as elsewhere in this file.
- **Impact:** run every generated path on the robot at reduced speed first, with someone at the
  stop button; no real robot is available.
- **Pinned by:** `test/Planner/test_core.py` (the committed paths pass `Verify_Path`).

## The carried part is not modelled

The gripper holds a 50 × 50 mm part of negligible mass during transit (owner); it is not in the
scene. The pad-span hull (one convex hull over both pads at the 50 mm opening) covers the space
between the fingers. **A part protruding beyond the pad ends, or beyond the pads sideways, is not
covered.**

- **Impact:** check on the robot that the part does not stick out of the fingers' envelope.
- **Pinned by:** `test/Scene/test_core.py::test_the_coupling_replaces_the_menagerie_disc` (pad-span hull).
- **What would close it:** a part model attached to the gripper.

## Gripper mounting orientation is assumed

`Tool.Mount_Yaw` = 0 is the standard dowel-pin-indexed Robotiq mounting. In the scene the fingers
open along the flange ±x axis. This was not measured on the robot (owner decision), and the manual
gives no angle.

- **Impact:** if the real gripper is turned, the fingers sweep other space (up to ≈ 75 mm for 90°).
- **Pinned by:** `test/Scene/test_core.py::test_the_fingers_open_along_flange_x`.
- **What would close it:** one look at the robot: do the fingers close along tool x?

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
