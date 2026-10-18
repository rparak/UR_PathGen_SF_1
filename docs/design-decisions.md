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

**Decision.** `Write_Movej_Sequence` writes `movej(<name>_q, a=<a>, v=<v>, r=0)` and no pose target is
ever written. *Corrected 2026-10-17 (owner):* `a` and `v` were omitted (URScript defaults); they are
now the robot program's tokens (1.3962634015954636, 1.0471975511965976), so a generated move runs
at the speed the duration estimate assumes. Station lines copied verbatim from the program are the
one exception to "movej with joint targets only" (SPEC §10.4).

**Why.** The check covers the joint-linear segment. A blend leaves it, and a pose target lets the
robot choose its own IK branch. Speed does not change the path of an unblended `movej`.

**Revisit if** blend-envelope checking exists.

Pinned by `test/URScript/test_core.py::test_movej_uses_joint_targets_and_no_blend`.

## Learned points keep the program's names; the program extract is their single source

**Decision.** Learned points are read only from `Data/Programs/sf1_plc_v4/sf1_plc_v4_positions.urscript`,
under the robot's names (A = `home`, B = `drop_rotate_1`); generated via points are
`via_<from>_<to>_<nn>`. The robot export of A and B is kept as
`test/URScript/data/SF1_Positions_original.urscript`.

*Corrected 2026-10-18 (owner): A and B were first stored under project names (`home_safe`,
`drop_rot_01`) in `Data/Positions/SF1_Positions.urscript`. That file is retired.*

**Why.** Two files held the same points under two names, and the program, the generated output
and the viewers all use the robot's names. One source cannot drift; the original tokens (encoder
noise included) are what the robot knows.

**Alternatives.** Keeping the renamed file next to the extract, with a test that their tokens agree:
it doubles the names an operator has to map.

**Revisit if** a second robot program is added and its point names clash with this one's.

Pinned by `test/URScript/test_core.py::test_the_plc_extract_is_read_with_every_token` (A and B in the
extract carry the export's tokens) and `::test_the_fixture_is_the_robot_export`.

## `Robot.Base_Yaw` is 180°, decided by where the taught points land

**Decision.** `Robot.Base_Yaw` = 180.0° (owner, 2026-10-17; confirmed by the taught points of
`sf1_plc_v4.script` and photos of the real cell). A placement test checks that the grasp and drop
points land on what they handle; at 0° it refuses all of them.

*Corrected 2026-10-17: 0° was configured and recorded as confirmed.* That value was an assumption.
The test meant to catch a wrong yaw (`drop_rotate_1` above the side table) encoded the same
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

## Contradicted geometry is corrected by a recorded component translation, not by editing the STL

**Decision.** `Environment.Component_Overrides` translates whole STL components before the collision
set is built (component id, translation ≤ 150 mm, source). The manifest records the overrides; the
scene refuses a set whose record differs from the configuration. The visual mesh stays unchanged.

**Why.** The STL is the owner's CAD and is hashed; editing it would hide which numbers were fitted.
A translation applied to the component's own vertex copy keeps the hull conservative (the check runs
against the translated surface) and the component ids stable, because the components are found
before the translation.

**Alternatives.** Moving pieces of the built set: rejected, CoACD piece ids change with every rebuild
and a moved piece no longer contains its surface. Deleting or trimming a component: rejected, it
removes geometry where nothing contradicts it. A larger margin elsewhere: rejected, it does not
remove a contradiction.

**Revisit if** the cell is measured (then the STL itself is corrected and the overrides removed), or
a correction needs a rotation.

Pinned by `test/Scene/test_core.py::test_a_set_built_with_other_overrides_is_refused` and
`test/Configuration/test_core.py::test_an_invalid_value_fails_and_never_falls_back`.

## A whole object's placement is an assembly override, separate from a component fit

**Decision.** `Environment.Assembly_Overrides` translates every component of one STL object before
the build. An object is the set of components whose bounding boxes touch within 1 mm (SPEC §4.1),
named by one of its components. A component override adds its own translation on top, and each
entry is bounded at 150 mm on its own. The manifest records both kinds and the members of each
assembly; the scene refuses a set whose record differs from the configuration. One function,
`Component_Translations`, computes the result for the build and for the tests.

**Why.** A misplaced machine is one rigid assembly: one claim with one source. A fit of a single
component on top of it is a different claim, often from a different source. Keeping the two apart
keeps each source visible and each bound meaningful. The object rule is reproducible from the STL;
the objects stand more than 50 mm apart, so the 1 mm tolerance is not critical.

**Alternatives.**

- One component override per member with a shared source: rejected, unreadable (object (a) has 1749
  components), and a typo would move a single part silently.
- A selection box: rejected, it catches parts by position, not by what they belong to.

**Revisit if** the STL changes (the object split is pinned at 1749 / 951 / 56 components), or an
assembly needs a rotation.

Pinned by `test/Collision/test_core.py::test_the_stl_has_three_objects_and_an_assembly_moves_exactly_one`,
`test/Collision/test_core.py::test_the_configured_shift_moves_the_machine_and_the_side_cabinet_only`
and `test/Scene/test_core.py::test_a_set_built_with_other_overrides_is_refused`.

## Obstacles missing from the STL are configured box primitives

**Decision.** `Environment.Added_Boxes` lists world-axis-aligned boxes (name, corners, source). The
scene adds each as a box primitive to body `env` with the environment margin; a box that meets the
station-zone box is a zone piece.

**Why.** Some obstacles are seen on the cell (photos) but not in the CAD. A primitive needs no
collision-set rebuild for a guessed geometry, keeps its source next to its numbers, and gives exact
distances.

**Alternatives.** A box piece in the collision set: rejected, it needs a rebuild and a manifest
change for every estimate. Editing the STL: rejected, the STL is the owner's hashed CAD.

**Revisit if** an obstacle needs another shape than a box, or the STL is corrected.

Pinned by `test/Scene/test_core.py::test_an_added_box_is_an_env_box_and_a_zone_piece_only_inside_the_zone`
and `test/Configuration/test_core.py::test_an_empty_list_of_added_boxes_is_accepted`.

## The station zone is a region of the environment, applied fully to taught and near the station points to generated segments

**Decision.** Environment pieces whose bounding box meets `Clearance.Station_Zone` get the zone
margin. A taught segment uses it on every sample; a generated segment only on samples whose TCP lies
within `Endpoint_Radius` of a fixed station point; elsewhere the global margin applies.

**Why.** The uncertainty is in the geometry of one region, not in where the TCP is, so the reduction
belongs to that region's pieces. The taught program is the evidence that the robot passes there; a
generated path has no such evidence, so it may use the reduced margin only where it has to reach the
same station points (owner, 2026-10-17).

**Alternatives.** A TCP sphere for every check: rejected, it reduces the margin of every pair while
the TCP is near (the elbow included) and does not cover a near region the TCP is far from (the
window's top profile at 15.3 mm with the TCP 300 mm away). A lower global margin: rejected, it would
consume the budget everywhere.

**Revisit if** the station is measured, or the planner needs station points other than the taught ones.

Pinned by `test/Scene/test_core.py::test_the_station_zone_applies_within_the_endpoint_radius`.

## The station zone holds the holder only; the post takes the full margin

**Decision.** `Clearance.Station_Zone` is the holder's neighbourhood, `[-0.755, -0.36, 0.94]` –
`[-0.62, -0.24, 1.13]` m: the holder box and the sill-frame pieces it sits on (owner, 2026-10-18,
option S of three). The post, the window's top profile and the machine front return to the 20 mm
margin.

**Why.** After the machine shift (SPEC §4.11) the taught program keeps ≥ 39.1 mm to the post, so it
no longer constrains where the post is: its extra 34 mm beyond the earlier fit (−115.7 mm) rests
only on the owner's estimate, and a generated path must keep 20 mm from it. The holder is the only
geometry the program still comes closer to than 20 mm (5.5 mm, its top fitted to that); without it
every taught segment keeps the global margins everywhere (SPEC §4.12).

**Alternatives.** Keeping the zone (K): a generated path could then pass the post at 5 mm near a
station point while the program keeps 39 mm, using space only the estimate provides. Removing the
zone and lowering the holder's top to the photo's 1.095 m (R; 21.1 mm to every taught sample): it
gives up the 17 mm of fitted, conservative height, and the zone's endpoint radius also defines the
penalty's neighbourhood, so the planner's objective would change.

**Revisit if** the machine or the post is measured, the holder is measured (its top may then keep
20 mm), or the program comes closer to the post again.

Pinned by `test/Scene/test_station.py::test_the_zone_holds_the_holders_neighbourhood_only` and
`test/Scene/test_station.py::test_without_the_holder_the_taught_sequence_keeps_the_global_margin`.

## The rotation-station geometry is fitted to the taught program only where the program constrains it

**Decision.** Machine (a) is shifted −150 mm along x as one assembly and the side cabinet (c) −50 mm
as another (`machine_a`, `side_cabinet`, the owner's estimates). No component is fitted on top: the
taught rotation sequence keeps ≥ 39.1 mm to the post 2710 without one (SPEC §4.11). A component is
fitted only where the taught program contradicts the estimated placement, by the smallest
translation that keeps every taught segment 5 mm clear. The holder box is fitted that way (its top,
§4.10). *Corrected 2026-10-18: with the machine at −100 mm, component 2710 (the left window profile)
was translated a further −15.7 mm along x: the smallest translation along the machine front's normal
that keeps every segment of the taught rotation sequence (cases 17 and 18) at least 5 mm clear (the
free-direction optimum was 15.5 mm). The owner's further 50 mm made it unnecessary, and it was
removed rather than kept on top (−165.7 mm).* The station zone holds the holder's
neighbourhood only (entry "The station zone holds the holder only"; until 2026-10-18 its box
started at world x = −0.80 m, so the machine-interior fallback hull, component 1638, stayed
outside it).
*Corrected 2026-10-18: before the owner's assembly estimate, the fit was 2710 by (−114.8, −6.2,
−0.6) mm and 1154 by (−32.6, −11.6, +3.6) mm (SPEC §4.7); the post now totals −115.7 mm along x,
consistent with it (SPEC §4.9).*

**Why.** No measurement is available, and the real robot runs the whole program without collision
(owner): the program is the only evidence of where the cell is not. A rigid translation per
component changes the least geometry for that evidence. The machine's placement is the owner's
estimate, kept apart from the fit as an assembly override; only the post, which the taught drop
contradicts, is fitted on top, along −x, which matches the owner's description and is only 0.2 mm
longer than the free direction. No segment of any of the 21 cases gets new or deeper penetration
(SPEC §4.9). The approved box (x from −0.85 m) also caught
the 235 mm-excess interior hull and about 30 small parts behind the machine front; at −0.80 m the
zone holds the station frame only and the taught replay still keeps every margin.

**Alternatives.** Trimming the post in z: rejected, it removes the post exactly where the robot
works. Translating only the touched 1154 pieces: rejected, piece ids change with every rebuild and
a moved piece leaves its surface. A lower global margin: rejected, it hides every contradiction at
once.

Removing the post fit at −150 mm: a fit that the program no longer requires would move the post on
no evidence. Kept on top it would put the post 165.7 mm behind the STL, beyond what either the
program or the owner states.

**Revisit if** the station is measured, the machine is placed closer again, or a taught segment changes.

Pinned by `test/Scene/test_station.py::test_the_taught_rotation_sequence_keeps_its_margins`,
`test/Scene/test_station.py::test_the_corrections_are_recorded_and_configured` and
`test/Scene/test_station.py::test_the_zone_holds_the_holders_neighbourhood_only`.

## The station holder is a box sized by a photo and capped by the taught sweep; the stub is not modelled

**Decision.** The U-shaped holder under the part, missing from the STL, is one box in
`Environment.Added_Boxes` (`station_bracket`). Its footprint and bottom come from an owner photo,
and its x range runs from the post face to an assumed 90 mm in front. *Updated 2026-10-18 (SPEC
§4.11): the box stays where the robot places the part (front at x −0.645 m); with the machine at
−150 mm and the post fit removed, its back is extended to 8 mm inside the shifted post face
(x −0.770 m), because the holder is mounted on the post (owner). The extension costs no taught
clearance (5.5 mm either way); without it the model would show a 27 mm gap between post and holder.*
Its top is the highest at
which every taught sample of cases 17 and 18 keeps 5 mm: 1.112 m, 17 mm above the photo's estimate
(SPEC §4.10). The stub seen above the holder is not modelled (owner, 2026-10-18).

**Why.** The optimised paths reach the station points from directions the taught program never
used, and the model had nothing under the part. The photo cannot resolve the height to better than
about ±30 mm; the taught program bounds it from above, because the real robot runs it, release
included. The largest box that bound allows is the conservative end. The stub cannot stand where
the photo puts it: the post fit leaves the taught drop 5 mm from the post face there, so a box could
be at most 0.3 mm proud. Such a plate would claim a geometry nothing supports and would still steer
the planner. *Corrected 2026-10-18: that held for the fitted post at −100 mm. With the machine at
−150 mm a stub on the post face could stand up to 34.6 mm proud (SPEC §4.11); whether to model it is
open.*

**Alternatives.**

- The photo's height for the holder (1.095 m): rejected, less conservative.
- The stub as a 0.3 mm plate beside the post: rejected by the owner; its real position is unknown,
  and a plate there would only move case 17 S2.
- A U with arms: rejected, the arms' position relative to the fingers is not visible.

**Revisit if** the holder or the stub is measured, the drop or the pick is re-taught, or the owner
decides on the stub at the shifted post.

Pinned by `test/Scene/test_station.py::test_the_taught_station_clears_the_station_boxes_by_the_zone_margin`
and `test/Planner/test_core.py::test_no_generated_sample_comes_closer_to_a_station_box_than_its_taught_end`.

## The gripper is mounted at 0°, confirmed by the owner; the yaw is never fitted

**Decision.** `Tool.Mount_Yaw` = 0°: the standard dowel-pin-indexed Robotiq mounting, with the
fingers opening along flange ±x. The owner confirmed it on the robot (2026-10-18). Earlier that day
the owner stated 90°; the statement was withdrawn the same day, and nothing derived from it was
committed.

**Why.** The taught program cannot discriminate the yaw: at 0° and at 90° alike, a station geometry
exists that keeps cases 17 and 18 at least 5 mm clear (at 90° the post would have needed 172.4 mm).
So the yaw must come from the robot, not from a fit, and the station geometry is fitted to the
configured yaw, never the other way round.

**Alternatives.** Choosing the yaw that needs the smaller station correction: rejected, it would turn
an unmeasured geometry error into a claim about the gripper.

**Revisit if** the gripper is remounted.

Pinned by `test/Scene/test_core.py::test_the_fingers_open_along_the_configured_mount_yaw` and
`test/Configuration/test_core.py::test_the_shipped_configuration_loads_in_si_units`.

## A generated segment stays inside the space the taught program proved free

**Decision.** `Clearance.Workspace` (x/y: the STL's extents; z: up to 2.35 m) bounds every robot
collision geom of a generated segment; taught segments are not held to it.

**Why.** The space outside the STL is not modelled, and an optimiser treats it as free: 24 % of random
configurations within the joint limits leave the STL's box, by up to 585 mm. The taught rotation
sequence reaches 2.322 m, 300 mm above the STL's top, so that height is proved free by the robot
itself; nothing higher or outside the STL's x/y extents is (owner, 2026-10-17).

**Alternatives.** The STL's box alone: rejected, it forbids the taught path. No box: rejected, a path
through the cable trays above the cell would be reported clear.

**Revisit if** the space around the cell is modelled, or a taught path reaches higher.

Pinned by `test/Scene/test_core.py::test_a_generated_configuration_above_the_workspace_is_refused`.

## The planner's edge check stops at the first violation, with the full check's verdict

**Decision.** `Segment_Valid` checks the same samples with the same rule as `Check_Segment`, in van der
Corput order, and stops at the first violation. The written path is re-checked with the full check.

**Why.** One generated check of a 2306-sample segment costs about 0.65 s; RRT-Connect and shortcutting
need thousands. A colliding edge usually fails in its middle, which the order visits third.

**Alternatives.** Coarser sampling for edges: rejected, it changes the verdict. A distance-field
approximation: rejected, it is a second geometry to keep consistent.

**Revisit if** the per-sample rule changes in a way that makes order matter.

Pinned by `test/Scene/test_station.py::test_the_early_exit_check_agrees_with_the_full_check` and
`test/Scene/test_core.py::test_the_van_der_corput_order_visits_every_sample_once`.

## Generated segments minimise the estimated duration, with a soft clearance penalty

**Decision.** The planner minimises the estimated duration (trapezoid on the leading joint with the
program's a and v, a stop at every via point) plus w times the sum over samples of
max(0, 30 mm − d) outside the station neighbourhood, w = 0.5 s/m; ties go to fewer via points.

**Why.** Cycle time is what the owner optimises; with r = 0 every via point is a stop, so the same
estimate also rewards fewer via points. An optimiser moves a path up to the hard margin, which has
no reserve beyond the budget; the penalty prefers clearance above the margin where the geometry is
not fitted (owner, 2026-10-17). The owner chose w = 0.5 s/m after this trade-off (estimated
duration; smallest clearance outside the station neighbourhood):

| Case, S | w = 0 | w = 0.5 |
|---|---|---|
| 17 S1 | 4.84 s, 24.5 mm | 4.88 s, 27.6 mm |
| 17 S2 | 4.51 s, 25.3 mm | 4.51 s, 33.8 mm |
| 17 S3 | 5.65 s, 25.6 mm | 5.71 s, 32.9 mm |
| 18 S1 | 4.89 s, 23.2 mm | 4.94 s, 26.6 mm |
| 18 S2 | 4.48 s, 28.9 mm | 4.47 s, 34.5 mm |
| 18 S3 | 5.78 s, 20.4 mm | 5.71 s, 30.4 mm |
| home → home 17 / 18 | 17.32 / 17.54 s | 17.41 / 17.52 s |

At most 0.09 s for about 6 mm more of the smallest clearance.

**Alternatives.** Joint-path length: rejected, it ignores the stops and the leading joint. A larger
hard margin: rejected, it would fail the station operations the owner tested.

**Revisit if** blends become allowed (then the estimate needs the blend), or real cycle times are
measured.

Pinned by `test/Planner/test_core.py::test_the_duration_estimates_match_hand_computed_values` and
`test/Planner/test_core.py::test_the_penalty_counts_only_far_from_the_station`.

## The planner's budgets are iteration counts, so its output is reproducible

**Decision.** Restarts, RRT extensions, shortcut and refinement attempts are counted; every random
draw comes from a generator seeded with the recorded seed, the segment and the restart. Wall-clock
time is reported, never used to stop.

**Why.** A time budget makes the output depend on the machine and its load, and two runs would
differ (owner, 2026-10-17).

**Alternatives.** A time budget with a fixed seed: rejected for that reason.

**Revisit if** planning takes too long on the owner's machine (then the counts change, recorded).

Pinned by `test/Planner/test_core.py::test_a_reduced_planning_run_is_byte_identical_twice`.

## The holding opening is configured per program case

**Decision.** `Tool.Holding_Openings` gives the gripper opening while the part of each program
case is held, with its source: case 17 = 0.050 m (owner). Planning, the verification of the
committed output and the replay take case 17's entry; open is the full 85 mm stroke (a gripper property, a code constant). `Tool.Opening` stays
the opening of a scene that names no case (the demo, the tests).

**Why.** Cases 17 and 18 are independent and probably handle parts of different size (owner,
2026-10-18). One global opening would silently apply case 17's part to case 18. The owner chose a
configuration field over a code constant: it is a cell fact with a source, like the other
`Tool` values.

**Alternatives.** A code constant next to the case table in `Example/plan_rotation.py`: rejected by
the owner.

**Revisit if** a case handles more than one part, or the URCap's position for a part is measured.

*Corrected 2026-10-18: case 18 = 0.050 m (an assumption, its part's size unknown) was listed while
case 18 was a deliverable; it is removed with it. The ground-truth check of case 18 types its own
assumed opening (`test/Scene/test_station.py`).*

Pinned by `test/Configuration/test_core.py::test_the_shipped_configuration_loads_in_si_units`,
`test/Example/test_plan_rotation.py` and the scenes of `test/Scene/test_station.py` and
`test/Planner/test_core.py`.

## Inside machine (a) the label is "model not valid here", not a collision

**Decision.** A configuration whose robot or gripper geometry comes within `Clearance.Environment`
of the interior hull of machine (a) (component 1638) is labelled *model not valid here* by the
viewers, whatever the clearance check reports there. The violating pairs are still listed.

**Why.** That hull is an open shell filled by its hull (235 mm excess, `KNOWN_ISSUES.md`); every
taught loading point inside the machine penetrates it by 99 – 144 mm (measured 2026-10-18 at 50 mm:
load3 −99.5, Waypoint_13 −137.4, bench_drop_11 −140.8 mm), although the program runs there on the
real robot. Calling those moves collisions would be false; calling them clear would be worse.

**Alternatives.** Excluding the piece from the check: rejected, it would hide a real contact with
the machine's outer wall. Rebuilding the piece: open, it needs a decomposition that passes the
conservativeness check (`KNOWN_ISSUES.md`).

**Revisit if** component 1638 is decomposed or the machine interior is modelled.

Pinned by `test/Scene/test_core.py::test_components_within_names_the_machine_interior_hull` and
`test/Example/test_visualize_position.py::test_a_known_position_reports_its_clearance`.

## The replay moves a movel on its TCP line to the DH TCP of its target's _q

**Decision.** `Example/replay_rotation.py` replays a movel as the controller runs it: the TCP on the
straight line, the orientation turning about one axis, the joints solved step by step from the
previous solution (`Kinematics.Core.Linear_Path`, `Solve_IK_Near`). The line ends at the DH TCP of
the target's `_q`, not at its `_p`, and must reach that `_q` itself; otherwise the replay refuses.

**Why.** The robot reaches `_p` with its calibrated kinematics, which corresponds to `_q`; with
nominal DH, `_p` and `DH_FK(_q)` differ by the measured residual (up to 4 mm, `KNOWN_ISSUES.md`).
Ending at `DH_FK(_q)` keeps the line consistent with the one kinematic truth and with the next move,
which starts at `_q`. Replaying it joint-linear (as before) shows a path the robot does not take.

**Alternatives.** The line to `_p` with nominal DH: it ends up to 4 mm off `_q` and the next movej
starts elsewhere. An analytic UR inverse kinematics: one more implementation of the kinematics to
keep equal to the DH; the numeric one uses the DH directly.

**Revisit if** a calibrated kinematic model (`calibration.conf`) is used.

Pinned by `test/Kinematics/test_core.py::test_a_movel_follows_the_straight_tcp_line_to_its_target` and
`test/Example/test_replay.py::test_every_path_visits_its_moves_in_program_order`.

## After the machine shift, S2 is optimised for time, not safety; the re-planned paths are kept

**Decision.** `Data/Generated/` keeps the S1 – S3 re-planned on the current scene (2026-10-18: the
machine shifted 100 mm, the post refitted, the station holder box added; same seed, budget and
w = 0.5 s/m), not the ones planned on the 2026-10-17 scene (owner, 2026-10-18).

**Why.** A path planned on a scene that is no longer the model is not what the checks describe. On
the current scene the taught S2 also keeps the generated rule (0 violations in both cases; on
2026-10-17 it came to 15.4 mm and failed), so S2's optimisation is now about time: one via point
instead of three, 8.30 / 8.29 s -> 4.45 / 4.46 s (estimate, case 17 / 18). The price is clearance:
outside the station neighbourhood the optimised S2 passes at 30.0 / 29.7 mm against the taught
42.5 mm, 10 mm above the 20 mm hard margin, at the 30 mm penalty threshold in case 17 and 0.3 mm
inside it in case 18 (penalty 0.002 m).

**Alternatives.** Keeping the 2026-10-17 paths: they keep the generated rule on the current scene
(SPEC 4.9), but they were planned for another model. Keeping the taught S2: 3.8 s slower per case
(estimate), 12.5 mm more clearance.

*Updated 2026-10-18: with the deliverable narrowed to case 17 and the machine at −150 mm, S1 – S3
were re-planned once on the final scene (SPEC §17): 4.76 / 4.31 / 5.27 s (S3 direct), home -> home
16.66 s against the committed 17.24 s; outside the station neighbourhood they pass at 29.7 / 31.0 /
23.3 mm (top profile, top profile, post) against the committed 64.0 / 48.9 / 65.6 mm. The same
objective, now with more free space: time is bought down to the 30 mm penalty threshold and, for
S3, 6.7 mm inside it. All three use space only the estimated shift frees (`KNOWN_ISSUES.md`).*

**Revisit if** the cell is measured (the machine position is an estimate, `KNOWN_ISSUES.md`), the
penalty weight changes, or the owner prefers the clearance of the taught or committed paths to the
time.

Pinned by `test/Planner/test_core.py::test_the_committed_paths_pass_the_independent_verification`
and `test/Example/test_replay.py::test_every_path_visits_its_moves_in_program_order` (entry 3 prints
the strict comparison).

## The deliverable is case 17 home -> home; case 18 and the whole cases are ground truth only

**Decision.** The planner, the taught-program writer, the replay and `Data/` deliver one sequence:
case 17 of `sf1_plc_v4.script` from home to home, taught (`Data/Programs/sf1_plc_v4/sf1_plc_v4_case_17*`)
and optimised (`Data/Generated/SF1_Rotation_17*`: S1 home -> drop_rotate_1, S2 Waypoint_5 ->
Waypoint_18 and S3 Waypoint_6 -> home generated, the station lines verbatim). The replay catalogue
is 1 taught, 2 optimised, 3 both with the strict comparison. The program extract (every learned
point, the whole cases 17 and 18) and the tests that check the model against real program segments
stay: the taught replay of cases 17 and 18 and the placement of the grasp points. The generated rule's station points are case 17's six.

**Why.** Owner, 2026-10-18: this exact taught sequence and its optimised version are the only
deliverable; the holder exists so that the robot can grip the part from the other side. The loading
moves run inside machine (a), where the model is not valid (`KNOWN_ISSUES.md`), so a whole-case
deliverable carried lines the tool cannot check. The other cases still run on the real robot, so
they remain the evidence the fitted geometry is checked against.

**Alternatives.** Keeping case 18 as a second deliverable: rejected by the owner; its part's size
was an assumption. Removing case 18 from the tests as well: rejected, it is ground truth that
constrains the station geometry as much as case 17.

**Revisit if** the owner asks for another case or for the whole case again.

Pinned by `test/Example/test_write_taught_programs.py`, `test/Planner/test_core.py::test_the_sequence_file_has_the_program_structure`,
`test/Example/test_replay.py::test_list_prints_the_catalogue` and, for the ground truth,
`test/Scene/test_station.py::test_the_taught_rotation_sequence_keeps_its_margins`.

## The program's penetrations are a recorded reference, not required to be zero

**Decision.** `test/Scene/test_program.py` replays every unique segment of the 21 cases of
`sf1_plc_v4.script` (125, both openings) and compares it with a recorded reference
(`test/Scene/data/sf1_plc_v4_segments.json`): the 63 segments that penetrate the environment, each
with its depth. A segment that penetrates without being recorded, a recorded one more than 0.5 mm
deeper, or any self-collision fails. The reference is re-recorded only with a deliberate change of
the scene, its before / after comparison in the commit.

**Why.** Every segment runs on the real robot (owner), so a change of the model that makes the
program penetrate more is a change away from the cell. The program cannot be required to be clear:
the machine-interior hull 1638 fills the space the loading moves use (down to -145 mm), and the grasp
points reach into the modelled parts (KNOWN_ISSUES). 0.5 mm is far above the sampling and solver
noise (the reference was reproduced to 1e-5 m by an independent scratch replay) and far below a
real geometry change.

**Alternatives.** Zero penetration everywhere: impossible with the interior hull. A scratch replay
before each change (as on 2026-10-17 and 2026-10-18): not run by default, so a change could skip it.
Checking cases 17 and 18 only: the post fit and the machine shift move geometry the other cases use.

**Revisit if** component 1638 is decomposed, the parts on the tables are removed from the model, or
the program changes (its sha256 is recorded).

Pinned by `test/Scene/test_program.py`.
