# Case 17 home -> home: the variants compared

Written by `Example/write_variants.py` from the files in this directory; `comparison.csv` holds the same
numbers. **Simulation passed ≠ safe on the robot.**

| Variant | Status |
|---|---|
| taught | the taught program (sf1_plc_v4, case 17) |
| b_optimised | previous optimisation (2026-10-18, 17.50 s), superseded by c1 |
| c1 | previous optimisation (simulation only), = Data/Generated/: the pick's wrist 3 one turn lower, same poses |
| c2 | delivered - tested on the real cell (owner, 2026-10-21): the drop on branch S- E- W- (wrist flipped in S1) |
| c3 | proposal for re-teaching, not verified on the robot: the drop on branch S- E- W+ (home's branch) |

How the numbers are made:

- **Estimated duration**: an estimate. Every move from rest to rest (r = 0 everywhere): the trapezoid of the
  leading joint for a movej, of the TCP path for a movel, with the line's own a and v; gripper time and waits
  excluded. The taught program's blends are **not modelled**: home (r=0.01), Waypoint_1 (r=0.05), Waypoint_1 (r=0.05), Waypoint_2 (r=0.01), Waypoint_3 (r=0.05), Waypoint_2 (r=0.01).
- **Motion lines** include the start at home; **distinct points** are the targets moved to; **via points** are a
  transfer section's moves before its end.
- **Lengths**: Σ|Δq| over all joints and the max-joint length per move, a movel along its solved TCP line; the
  TCP path length over the checked samples (no robot point moves more than 5 mm between two).
- **Heights**: world z (the robot base stands at 0.7152 m); the robot height is its highest collision vertex,
  the gripper included.
- **Clearance**: robot - environment, capped at 200 mm. Each move by the rule of its kind: generated movej by
  the generated rule, program lines by the taught rule; the least room is the distance minus that pair's
  margin. The station-box floors and the workspace count in the samples breaking the rule only.
- c2 and c3 drop points are the nominal-DH IK of the program's `_p`: millimetres from what the robot's
  calibrated IK returns. c2 was delivered and tested on the real cell (owner, 2026-10-21); c3 must be
  re-taught and re-verified on the robot at reduced speed.

## Home -> home

| | taught | b_optimised | c1 | c2 | c3 |
|---|---|---|---|---|---|
| Motion lines | 13 | 12 | 12 | 10 | 12 |
| Distinct points | 10 | 11 | 11 | 9 | 11 |
| Via points | 5 | 4 | 4 | 2 | 4 |
| Estimated duration [s] | 23.37 | 17.50 | 14.66 | 12.05 | 13.86 |
| Joint path Σ\|Δq\| [rad] | 45.440 | 38.642 | 35.395 | 23.462 | 24.792 |
| Max-joint length [rad] | 16.467 | 12.098 | 8.774 | 7.121 | 8.122 |
| TCP path length [m] | 7.062 | 6.604 | 6.446 | 3.237 | 3.372 |
| Max TCP height [m] | 2.294 | 2.224 | 2.224 | 1.624 | 1.710 |
| Max robot height [m] | 2.322 | 2.263 | 2.263 | 1.818 | 1.780 |
| Closest robot - environment | 5.5 mm, gripper_right_follower - station_bracket | 5.5 mm, gripper_right_follower - station_bracket | 5.5 mm, gripper_right_follower - station_bracket | 5.5 mm, gripper_right_follower - station_bracket | 5.5 mm, gripper_right_follower - station_bracket |
| Least room above the rule's margin | 0.5 mm, gripper_right_follower - station_bracket (margin 5 mm) | 0.4 mm, gripper_base - machine_a piece 6515 (margin 30 mm) | 0.4 mm, gripper_base - machine_a piece 6515 (margin 30 mm) | 0.5 mm, gripper_right_follower - station_bracket (margin 5 mm) | 0.5 mm, gripper_right_follower - station_bracket (margin 5 mm) |
| Samples breaking the rule | 0 | 0 | 0 | 0 | 0 |

## S1

| | taught | b_optimised | c1 | c2 | c3 |
|---|---|---|---|---|---|
| Motion lines | 2 | 2 | 2 | 1 | 2 |
| Distinct points | 2 | 2 | 2 | 1 | 2 |
| Via points | 1 | 1 | 1 | 0 | 1 |
| Estimated duration [s] | 5.95 | 4.76 | 4.76 | 2.48 | 3.43 |
| Joint path Σ\|Δq\| [rad] | 15.199 | 13.901 | 13.901 | 6.420 | 7.164 |
| Max-joint length [rad] | 4.661 | 3.557 | 3.557 | 1.810 | 2.390 |
| TCP path length [m] | 2.782 | 2.641 | 2.641 | 0.761 | 0.880 |
| Max TCP height [m] | 2.294 | 2.224 | 2.224 | 1.363 | 1.388 |
| Max robot height [m] | 2.322 | 2.263 | 2.263 | 1.591 | 1.578 |
| Closest robot - environment | 16.3 mm, gripper_left_coupler - station_stub | 16.3 mm, gripper_left_coupler - station_stub | 16.3 mm, gripper_left_coupler - station_stub | 16.4 mm, gripper_left_follower - station_stub | 16.4 mm, gripper_left_follower - station_stub |
| Least room above the rule's margin | 11.3 mm, gripper_left_coupler - station_stub (margin 5 mm) | 0.4 mm, gripper_base - machine_a piece 6515 (margin 30 mm) | 0.4 mm, gripper_base - machine_a piece 6515 (margin 30 mm) | 10.9 mm, gripper_left_spring_link - machine_a piece 6526 (margin 30 mm) | 10.9 mm, gripper_left_spring_link - machine_a piece 6526 (margin 30 mm) |
| Samples breaking the rule | 0 | 0 | 0 | 0 | 0 |

## drop

| | taught | b_optimised | c1 | c2 | c3 |
|---|---|---|---|---|---|
| Motion lines | 2 | 2 | 2 | 2 | 2 |
| Distinct points | 2 | 2 | 2 | 2 | 2 |
| Estimated duration [s] | 0.97 | 0.97 | 0.97 | 0.98 | 0.98 |
| Joint path Σ\|Δq\| [rad] | 0.484 | 0.484 | 0.484 | 0.486 | 0.489 |
| Max-joint length [rad] | 0.198 | 0.198 | 0.198 | 0.201 | 0.198 |
| TCP path length [m] | 0.133 | 0.133 | 0.133 | 0.133 | 0.133 |
| Max TCP height [m] | 1.320 | 1.320 | 1.320 | 1.317 | 1.317 |
| Max robot height [m] | 1.554 | 1.554 | 1.554 | 1.557 | 1.536 |
| Closest robot - environment | 10.6 mm, gripper_left_spring_link - station_stub | 10.6 mm, gripper_left_spring_link - station_stub | 10.6 mm, gripper_left_spring_link - station_stub | 12.0 mm, gripper_base - station_stub | 12.0 mm, gripper_base - station_stub |
| Least room above the rule's margin | 5.6 mm, gripper_left_spring_link - station_stub (margin 5 mm) | 5.6 mm, gripper_left_spring_link - station_stub (margin 5 mm) | 5.6 mm, gripper_left_spring_link - station_stub (margin 5 mm) | 7.0 mm, gripper_base - station_stub (margin 5 mm) | 7.0 mm, gripper_base - station_stub (margin 5 mm) |
| Samples breaking the rule | 0 | 0 | 0 | 0 | 0 |

## S2

| | taught | b_optimised | c1 | c2 | c3 |
|---|---|---|---|---|---|
| Motion lines | 4 | 3 | 3 | 2 | 3 |
| Distinct points | 4 | 3 | 3 | 2 | 3 |
| Via points | 3 | 2 | 2 | 1 | 2 |
| Estimated duration [s] | 8.30 | 4.81 | 4.85 | 4.52 | 5.37 |
| Joint path Σ\|Δq\| [rad] | 18.275 | 12.797 | 12.941 | 8.488 | 9.070 |
| Max-joint length [rad] | 5.659 | 3.220 | 3.088 | 3.180 | 3.603 |
| TCP path length [m] | 3.138 | 2.791 | 2.702 | 1.373 | 1.389 |
| Max TCP height [m] | 2.208 | 2.189 | 2.196 | 1.624 | 1.710 |
| Max robot height [m] | 2.237 | 2.233 | 2.244 | 1.818 | 1.780 |
| Closest robot - environment | 8.4 mm, gripper_right_pad - station_stub | 10.1 mm, gripper_base - station_stub | 10.1 mm, gripper_base - station_stub | 10.1 mm, gripper_base - station_stub | 10.1 mm, gripper_base - station_stub |
| Least room above the rule's margin | 3.4 mm, gripper_right_pad - station_stub (margin 5 mm) | 5.1 mm, gripper_base - station_stub (margin 5 mm) | 5.1 mm, gripper_base - station_stub (margin 5 mm) | 5.1 mm, gripper_base - station_stub (margin 5 mm) | 5.1 mm, gripper_base - station_stub (margin 5 mm) |
| Samples breaking the rule | 0 | 0 | 0 | 0 | 0 |

## pick

| | taught | b_optimised | c1 | c2 | c3 |
|---|---|---|---|---|---|
| Motion lines | 2 | 2 | 2 | 2 | 2 |
| Distinct points | 2 | 2 | 2 | 2 | 2 |
| Estimated duration [s] | 1.34 | 1.34 | 1.34 | 1.34 | 1.34 |
| Joint path Σ\|Δq\| [rad] | 0.835 | 0.835 | 0.835 | 0.835 | 0.835 |
| Max-joint length [rad] | 0.384 | 0.384 | 0.384 | 0.384 | 0.384 |
| TCP path length [m] | 0.218 | 0.218 | 0.218 | 0.218 | 0.218 |
| Max TCP height [m] | 1.381 | 1.381 | 1.381 | 1.381 | 1.381 |
| Max robot height [m] | 1.562 | 1.562 | 1.562 | 1.562 | 1.562 |
| Closest robot - environment | 5.5 mm, gripper_right_follower - station_bracket | 5.5 mm, gripper_right_follower - station_bracket | 5.5 mm, gripper_right_follower - station_bracket | 5.5 mm, gripper_right_follower - station_bracket | 5.5 mm, gripper_right_follower - station_bracket |
| Least room above the rule's margin | 0.5 mm, gripper_right_follower - station_bracket (margin 5 mm) | 0.5 mm, gripper_right_follower - station_bracket (margin 5 mm) | 0.5 mm, gripper_right_follower - station_bracket (margin 5 mm) | 0.5 mm, gripper_right_follower - station_bracket (margin 5 mm) | 0.5 mm, gripper_right_follower - station_bracket (margin 5 mm) |
| Samples breaking the rule | 0 | 0 | 0 | 0 | 0 |

## S3

| | taught | b_optimised | c1 | c2 | c3 |
|---|---|---|---|---|---|
| Motion lines | 2 | 2 | 2 | 2 | 2 |
| Distinct points | 2 | 2 | 2 | 2 | 2 |
| Via points | 1 | 1 | 1 | 1 | 1 |
| Estimated duration [s] | 6.81 | 5.62 | 2.74 | 2.74 | 2.74 |
| Joint path Σ\|Δq\| [rad] | 10.648 | 10.626 | 7.234 | 7.234 | 7.234 |
| Max-joint length [rad] | 5.565 | 4.739 | 1.546 | 1.546 | 1.546 |
| TCP path length [m] | 0.791 | 0.821 | 0.752 | 0.752 | 0.752 |
| Max TCP height [m] | 1.410 | 1.435 | 1.386 | 1.386 | 1.386 |
| Max robot height [m] | 1.594 | 1.724 | 1.688 | 1.688 | 1.688 |
| Closest robot - environment | 45.0 mm, gripper_base - machine_a piece 6526 | 45.0 mm, gripper_base - machine_a piece 6526 | 45.0 mm, gripper_base - machine_a piece 6526 | 45.0 mm, gripper_base - machine_a piece 6526 | 45.0 mm, gripper_base - machine_a piece 6526 |
| Least room above the rule's margin | 25.0 mm, gripper_base - machine_a piece 6526 (margin 20 mm) | 15.0 mm, gripper_base - machine_a piece 6526 (margin 30 mm) | 15.0 mm, gripper_base - machine_a piece 6526 (margin 30 mm) | 15.0 mm, gripper_base - machine_a piece 6526 (margin 30 mm) | 15.0 mm, gripper_base - machine_a piece 6526 (margin 30 mm) |
| Samples breaking the rule | 0 | 0 | 0 | 0 | 0 |
