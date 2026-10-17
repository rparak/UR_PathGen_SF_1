# Design decisions

Deliberate choices whose reason is not visible in the code. Each one says what would reopen it.

## The official UR10e DH parameters are the one kinematic truth; Menagerie is patched in code

**Decision.** `Kinematics.Core` uses the official UR10e DH parameters. `Scene.Core` overwrites the
Menagerie body offsets and moves the flange site from 0.100 m to d6 = 0.11655 m through MjSpec.
The vendored files stay byte-identical to upstream (`models/MANIFEST.json`).

**Why.** The Menagerie offsets are rounded (0.181 vs 0.1807, 0.613 vs 0.6127 …), and its
`attachment_site` sits 16.5 mm inside the wrist; the visual mesh `wrist3.obj` ends at 0.1168 m.
Measured flange difference before the patch: 16.9 mm (home), 16.6 mm (drop_rotate_1). After it,
MuJoCo and DH agree to 1e-15 m over 1000 random joint vectors.

**Alternatives.** Editing the vendored XML was rejected: it breaks the hash manifest and hides the
patch. Correcting the TCP instead was rejected: the gripper geometry would sit 16.5 mm too close to
the wrist.

**Revisit if** Menagerie fixes the UR10e offsets, or the robot's `calibration.conf` becomes available.

Pinned by `test/Scene/test_core.py::test_mujoco_flange_equals_dh_flange_for_random_joints`.

## The TCP is configured, not derived from `_p`

**Decision.** `Tool.TCP_Offset` = (0, 0, 0.174 m, 0, 0, 0): the Polyscope TCP of SF1 (owner), identical
to the Robotiq value (2F-85 & 2F-140 for Universal Robots, Instruction Manual rev. 2020/02/11,
p. 127). `Derive_TCP_Offset` is a diagnostic only.

**Why.** The TCP derived from one pose differs from the one derived from the other by 4.52 mm /
0.62°: that spread is the kinematic error of nominal DH, not a property of the tool.

**Alternatives.** Deriving the TCP on one pose and validating on the other (an earlier draft) was
rejected for the reason above.

**Revisit if** the Polyscope TCP changes.

Pinned by `test/Kinematics/test_core.py::test_the_configured_tcp_is_the_robotiq_value`.

## The Robotiq coupling replaces the Menagerie mount disc

**Decision.** The Menagerie `base_mount` mesh (Ø75 × 13.9 mm) is replaced by a cylinder of the
configured coupling (Ø75 × 16.9 mm, manual p. 119). The gripper is shifted so that its datum sits
16.9 − 2.9 = 14.0 mm from the flange face: +3.0 mm against Menagerie.

**Why.** Flange face → gripper body was measured on the robot by the owner: 14.0 mm. With the shift,
the model's pad ends match the manual's dimension chain to 0.03 mm (open, 163.3 mm) and 0.31 mm
(closed, 176.8 mm).

**Alternatives.** Menagerie as is was rejected: it is 3.0–3.3 mm short. Fitting the shift to the
derived TCP was rejected as tuning.

**Revisit if** the coupling on the robot changes.

Pinned by `test/Scene/test_core.py::test_the_fingers_match_the_manual_dimension_chain`.

## Robot collision geometry is the hull of each visual mesh part

**Decision.** Every Menagerie visual mesh geom gets a collision twin (MuJoCo hulls it). The
Menagerie capsules are deleted.

**Why.** A hull contains its mesh by construction. The hand-placed capsules need not.

**Alternatives.** Keeping the capsules was rejected: they are not conservative.

**Revisit if** query time becomes the bottleneck of the planner.

## Only configured exclusions apply; one pair has a 0 mm self margin

**Decision.** MuJoCo's parent/child filter is disabled. The exclusions are exactly the adjacent
pairs and the gripper rigid group from the configuration, plus `base – env_stand`.
`base – upper_arm_link` is not excluded but has a self margin of 0 mm, so it is checked for contact
only.

**Why.** Prototype measurement over 1000 random joint vectors: that pair is within 5 mm in 17.5 %
of samples. Its true minimum is 2.3237 mm: a pan × lift grid followed by Nelder–Mead from the 20
lowest cells. A 5 mm margin would forbid a sixth of the joint space for a pair that never touches.
The elbow limit of ±180° is the precondition of the `upper_arm_link – forearm_link` exclusion and is
enforced at load.

**Alternatives.** Excluding the pair was rejected: it would hide a real collision if the geometry
changed. Lowering `Clearance.Self` for all pairs was rejected as too broad.

**Revisit if** the robot or gripper geometry changes; the test then recomputes the minimum.

Pinned by `test/Scene/test_core.py::test_only_the_configured_exclusions_apply` and
`::test_the_base_upper_arm_override_has_a_true_minimum_above_zero`.

## A pad-span hull covers the space between the fingers

**Decision.** One extra collision geom in the gripper base: the hull of the 32 corners of the four
pad boxes at the configured opening.

**Why.** Each gripper part is hulled separately, so without it the space between the fingers (where
the 50 × 50 mm part is held) would be free (owner, Q8 (ii)).

**Revisit if** the carried part is modelled.

## The environment is a hull per component, CoACD only where the hull is too coarse

**Decision.** The whole STL is never a collision mesh. Each connected component becomes its hull.
A component whose hull extends more than 10 mm beyond it is decomposed with CoACD, and is kept only
if the parts pass the conservativeness check (vertices plus 1 sample per cm², eps 1 mm); otherwise
its hull is kept. A degenerate (planar or collinear) component is thickened by ±0.5 mm along SVD
directions, because the STL winding is inconsistent and face normals can cancel.

**Why.** MuJoCo collides a mesh through its convex hull, so the STL as one mesh would be one blob.
A hull is conservative by convexity. CoACD parts are not conservative by construction, so they are
checked.

**Alternatives.** CoACD on the whole mesh (owner: no). A volume comparison: meaningless for a
non-watertight mesh.

**Revisit if** the STL changes (the manifest hash forces a rebuild) or query time requires merging
pieces.

Pinned by `test/Collision/test_core.py`.

## STL components are split at manifold edges

**Decision.** `Connected_Components` connects two faces only through an edge shared by exactly two
faces.

**Why.** The stand profiles touch along non-manifold edges. Splitting by shared vertex (2270
components) or by any shared edge (2428) made the whole stand frame one component; its CoACD parts
missed eps by 0.12 mm, and the fallback hull extended 297 mm beyond the frame, 0.2 mm under the robot
base. Manifold-edge splitting gives the 2756 components of SPEC 4.1 (measured with trimesh), and each
profile is hulled on its own.

**Alternatives.** Raising eps or `Hull_Excess_Max` until the frame passed was rejected: tuning to
make a check pass.

**Revisit if** a CAD export joins real parts by manifold edges (then they hull together).

Pinned by `test/Collision/test_core.py::test_components_are_split_at_non_manifold_edges`.

## CoACD parts are checked with the exact distance to their union

**Decision.** `Union_Distances` computes, for each check point, the exact distance to the union of a
component's CoACD parts: numpy point-triangle distance (Ericson), with a part skipped only when its
largest facet-plane distance proves it cannot be closer. There is no early exit, so the recorded
maximum is the true one. Every component has its own seed.

**Why.** Corrected 2026-10-17: the first check stopped at the first part within eps for each point,
which capped the reported maximum just below eps (0.99998 mm) while the true maximum was 0.769 mm.
The verdicts were unaffected.

**Alternatives.** trimesh closest-point per point: exact but slow, and it would bring trimesh into
the tests.

Pinned by `test/Collision/test_conservativeness.py`.

## The crop sphere is a conservative chain bound

**Decision.** The crop radius is the sum of the body offsets from the shoulder plus each body's
vertex radius, plus the clearance, plus 0.05 m: 2.026 m.

**Why.** It holds for every configuration without sampling. It is loose: it keeps every STL
component, so the crop removes nothing in SF1. Removing too little costs only build time.

**Revisit if** the build time matters; a tighter bound must stay a proven upper bound.

Pinned by `test/Collision/test_core.py::test_the_crop_sphere_contains_the_robot_everywhere`.

## The STL is read by the project's own numpy reader

**Decision.** `Collision.Core.Read_Binary_STL` reads the STL for the build and for the visual mesh.
The visual mesh is passed to MuJoCo as `uservert`/`userface`.

**Why.** MuJoCo's STL loader refuses more than 200 000 faces (the STL has 1 757 948). trimesh is kept
out of what the tests import (owner: trimesh and CoACD only in the build).

**Revisit if** MuJoCo lifts the limit.

Pinned by `test/Scene/test_core.py::test_the_visual_mesh_is_not_hulled`.

## The static base is checked once at build

**Decision.** `Scene_Cls` checks the base against the `env` body with `mj_geomDistance` when it is
built, and fails if any piece is within the environment margin.

**Why.** MuJoCo never collides two static bodies, so `mj_collision` cannot report `base – env`.

Pinned by `test/Scene/test_core.py::test_the_static_base_clearance_is_checked_at_build`.

## Orientations are compared by the atan2 geodesic angle

**Decision.** `Geodesic_Angle` = atan2(‖vee(M − Mᵀ)‖ / 2, (tr M − 1) / 2), never rotation-vector
components.

**Why.** `home_p` has |r| ≈ π, where r and its equivalent differ by ≈ 2π component-wise. The arccos
form cannot resolve below ≈ 6e-8 rad; atan2 resolves 1e-16 (measured).

Pinned by `test/Kinematics/test_core.py`.

## Every emitted movej has r=0 and joint targets only

**Decision.** `Write_Movej_Sequence` writes `movej(<name>_q, r=0)`. `a` and `v` are omitted (URScript
defaults), and no pose target is ever written.

**Why.** The check covers the joint-linear segment. A blend leaves it, and a pose target lets the
robot choose its own IK branch. Speed does not change the path of an unblended `movej`.

**Revisit if** blend-envelope checking exists.

Pinned by `test/URScript/test_core.py::test_movej_uses_joint_targets_and_no_blend`.

## Learned points are stored under project names, values verbatim

**Decision.** `home` → `home_safe` (A), `drop_rotate_1` → `drop_rot_01` (B); generated via points are
`via_<from>_<to>_<nn>`. The robot export is kept as `test/URScript/data/SF1_Positions_original.urscript`.

**Why.** A consistent convention for generated output; the original tokens (encoder noise
included) are what the robot knows.

Pinned by `test/URScript/test_core.py::test_the_renamed_points_keep_every_token`.

## `Robot.Base_Yaw` is 180°, decided by where the taught points land

**Decision.** `Robot.Base_Yaw` = 180.0° (owner, 2026-10-17; confirmed by the taught points of
`sf1_plc_v4.script` and photos of the real cell). A placement test checks that the grasp and drop
points land on what they handle; at 0° it refuses all of them.

*Corrected 2026-10-17: 0° was configured and recorded as confirmed.* That value was an assumption.
The test meant to catch a wrong yaw (`drop_rot_01` above the side table) encoded the same
assumption, so it passed. FK vs `_p` is blind to the yaw, and clearance does not decide it either:
at 0° both learned points and the ±45° demo were clear, because the robot then swept empty space.

**Why.** At 180° `pick_obj_1..8` land on the pallet-table plate below the tops of the parts,
`unload_obj_1..8` on the side-table plate, and `bench_drop_1..8` inside machine (a) (grinder
loading). At 0° the first two groups land in empty space and the third beyond the STL (SPEC §4.6).
The owner sees the robot at home facing the machine.

**Alternatives.** Keeping 0° and testing clearance only: rejected, clearance cannot tell an empty
cell from a correct one. Deriving the yaw from a clearance optimum: rejected, it would fit the model
instead of measuring the cell. Tests that now fail only because the STL does not match the cell at
the rotation station are strict xfails (`KNOWN_ISSUES.md`): they turn red as soon as the geometry
is fixed, so the failure stays visible.

**Revisit if** the touch-probed cell disagrees with the placement, or `Robot.Base_Position` x/y
turns out to be off (it is not verified against the STL).

Pinned by `test/Scene/test_core.py::test_the_taught_points_land_where_they_handle_parts`,
`test/Scene/test_core.py::test_the_placement_check_refuses_a_base_yaw_of_0_deg` and
`test/Scene/test_core.py::test_a_base_yaw_of_90_deg_turns_a_known_point_by_90_deg`.
