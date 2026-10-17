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

## The STL does not match the cell at the rotation station

With `Base_Yaw` = 180°, the regrasp sequence of `sf1_plc_v4.script` (cases 17 and 18) collides with
the environment model. The real robot runs it, so the model is wrong there, or the base position
is. The station is a small bracket on the left vertical profile of the machine-front window of
machine (a), lower part (owner). In the model the drop TCP lies on the machine-front plane
(world x = −0.703 m against −0.702 m), next to a vertical post (STL component 2710, 90 × 45 ×
1937 mm, world x [−0.702, −0.612], y [−0.290, −0.245]).

Joint-linear replay of the program's segments (movel approximated as joint-linear, 50 mm opening
while holding, 85 mm open; distances are the worst over the segment):

| Segment | Case 17 | Case 18 | Piece (flagged open shell, excess) |
|---|---|---|---|
| home → Waypoint_1 | clear | clear | |
| Waypoint_1 → drop | −27.4 mm | −28.4 mm | 3236 (yes, 37.9 mm), 3238 (yes, 44.0 mm) |
| drop → movel target (Waypoint_4 / 7) | −28.2 mm | −29.2 mm | 3236 |
| movel target → Waypoint_5 | −27.3 mm | −29.0 mm | 3236 |
| Waypoint_5 → Waypoint_1 | −19.1 mm | −19.1 mm | 3236 |
| Waypoint_1 → Waypoint_2 | −46.0 mm | −46.0 mm | **6524 (no, 4.6 mm)** |
| Waypoint_2 → Waypoint_3 | −51.0 mm | −51.0 mm | **6524** |
| Waypoint_3 → Waypoint_18 / 11 | −2.3 mm | +5.2 mm (margin only) | 3238 |
| → pick | −15.4 mm | −13.2 mm | 3238 |
| pick → movel target (Waypoint_6 / 8) | −20.1 mm | −19.0 mm | 3238 |
| movel target → Waypoint_2 | −59.3 mm | −58.6 mm | **6524** |
| Waypoint_2 → home | clear | clear | |

Piece 6524 is the post itself, an unflagged hull 4.6 mm larger than the post; the original surface
passes through the gripper hulls (0.0 mm from their vertices). This is not hull over-fill. Pieces
3236 and 3238 (the machine front) are mixed: at the drop the original surface is 0.5 – 3 mm from the
contact point, near the pick 9 – 27 mm (partly over-fill). No base yaw between 170° and 190° clears
both drop (≤ 172°) and pick (≥ 182°), and Waypoint_6 → Waypoint_2 still penetrates at 190°.

- **Impact:** nothing near the rotation station can be checked or planned: the model reports
  collisions the real robot does not have. A path there that the simulation called clear would mean
  nothing either, until the geometry is fixed.
- **Evidence:** scratch replay 2026-10-17 (yaw from the configuration, `Clearance.Environment` 20 mm).
- **Pinned by:** strict xfails `test/Scene/test_core.py::test_a_and_b_keep_every_clearance_margin[drop_rot_01]`
  and `test/Collision/test_core.py::test_no_robot_body_at_a_or_b_is_inside_a_flagged_piece[drop_rot_01]`.
- **What would close it:** touch-probing the post, the machine-front plane and the station bracket
  on the real cell, then correcting the STL (or the base position) to match.

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
