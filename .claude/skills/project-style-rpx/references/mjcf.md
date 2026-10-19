# MJCF - vendored models, patches in code, collision geometry

The scene is the UR10e and the Robotiq 2F-85 from MuJoCo Menagerie plus the SF1 environment, built
by `src/Scene/Core.py` (`Scene_Cls`) through `mujoco.MjSpec` (SPEC §4.2, §4.5, §7, §9). ✅ =
enforced by a test; 📄 = documentation only. The tests are in `test/Scene/test_core.py`,
`test/Scene/test_margin.py`, `test/Collision/test_core.py` and
`test/Collision/test_conservativeness.py`.

## Vendored models are never edited ✅

- `models/universal_robots_ur10e/` and `models/robotiq_2f85/` are byte-identical to upstream at the
  pinned Menagerie commit, licence files kept (BSD-3-Clause, BSD-2-Clause).
- `models/MANIFEST.json` records the commit, the licences and the sha256 of every vendored file.
  `install.sh` verifies it; a scene test checks the files against it. A changed file is a failure,
  not a new baseline.
- Updating a model is a deliberate change: new commit, new manifest, every pin re-measured.

## Every patch is in code, through MjSpec ✅

What the scene needs that the vendored files do not say is applied in `Scene_Cls` and recorded in
`docs/design-decisions.md`, never written into `models/`:

- **DH offsets.** The Menagerie body offsets are rounded and its flange site sits 16.5 mm inside the
  wrist; the builder overwrites them with the official UR10e DH so MuJoCo FK equals DH FK (within
  1e-9 m / 1e-9 rad, SPEC §9.2, §11).
- **Coupling.** The Menagerie `base_mount` mesh geoms are removed and replaced by a cylinder of
  `Tool.Coupling.Diameter` × `Tool.Coupling.Thickness` (visual and collision) on the flange face.
- **Gripper shift.** The gripper `base` body is moved so its datum lies `Thickness − Spigot_Depth`
  from the flange face; the shift is computed from the configured values, not stored (SPEC §9.3).
- **Pad-span hull.** One extra collision geom in the gripper `base`: the convex hull of the 32
  corners of the four pad boxes at the configured opening, in the gripper rigid group (SPEC §9.6).
- **Joint ranges** come from `Robot.Joint_Position_Limits`, not the Menagerie defaults.
- **Base pose** from `Robot.Base_Position` / `Base_Yaw`; the Menagerie root keeps its built-in 180°,
  which already makes its world frame the Polyscope base frame (SPEC §4.2, §9.1).
- A patch value comes from the configuration or a sourced constant; nothing is tuned to make a pin
  pass (SPEC §9.3 item 5).

## Self-collision exclusions are explicit ✅

- MuJoCo's automatic parent/child filter is **disabled** (`<flag filterparent="disable"/>`).
- Only the configured exclusions apply: `Self_Collision.Excluded_Pairs` and every pair inside a
  `Self_Collision.Rigid_Groups` group, plus `base–env_stand` in code (single legal value).
- The compiled model's exclusion set equals exactly that expansion, with nothing from model
  defaults (SPEC §9.6).
- An exclusion has a precondition when it has one: `upper_arm_link–forearm_link` is valid only with
  the elbow limited to ±180°, so the loader rejects a wider limit.
- A near pair is reported, never excluded to make the check pass (`base – upper_arm_link`, SPEC §4.5).

## Robot collision geometry 📄

- The robot collides through the **convex hulls of its visual meshes** (MuJoCo hulls every mesh
  geom); each hull contains its visual part. The Menagerie capsules are removed: they need not
  contain the mesh.
- The gripper keeps its Menagerie collision geoms at the joint angles that give `Tool.Opening`, plus
  the pad-span hull.
- Clearance is implemented with geom margins (`Clearance.Environment` on environment geoms,
  `Clearance.Self` on robot and gripper geoms). MuJoCo generates a contact below the **sum** of the
  two geom margins; `Check_Clearance` filters each contact by the pair's configured margin (the
  larger geom margin, or a pair override). `dist < margin` is a violation, `dist < 0` a collision
  (SPEC §9.7).

## Static bodies never collide with each other ✅

MuJoCo never collides two jointless, world-welded bodies (SPEC §4.5). The robot `base` is one, so
`mj_collision` never reports `base` ↔ `env`. Its clearance is a constant, checked once at scene
build with `mj_geomDistance`; the build fails with the violating pieces if any is below
`Clearance.Environment`. A test of a margin between two geoms puts at least one on a joint.

## The environment ✅

- **Visual:** `CAD/SF1_Environment.stl` unchanged. MuJoCo's STL loader refuses more than 200 000
  faces (the file has 1 757 948), so the mesh is passed as `uservert`/`userface`, with
  `contype=conaffinity=0` and shell inertia, so no hull is stored (`mesh_graphadr = −1`, SPEC §5).
  It is never used for collision: as one mesh it would collide as one hull.
- **Collision:** the committed set in `CAD/Collision/`, read through `Read_Collision_Set`, which
  refuses a set whose recorded STL sha256 differs from the file on disk (`Collision_Error_Cls`:
  "rerun Example/build_collision.py"). Tests never run CoACD.
- **Built by** `Example/build_collision.py`: crop to the reachable sphere, a hull per connected
  component, CoACD only where a hull's excess exceeds `Hull_Excess_Max`, fallback to the hull when the
  CoACD pieces are not conservative (SPEC §7.2). **Conservativeness beats tightness.**
- **Degenerate components** (planar or collinear, no 3-D hull, which MuJoCo refuses) are thickened
  by ±0.5 mm along their mean face normal before hulling; the result contains the original.
- Every collision mesh gets shell inertia (the bodies are static). Stand components go to body
  `env_stand`, all others to `env` (SPEC §7.5).
- Open shells whose hull fills space are listed by the build report and in `KNOWN_ISSUES.md`, never
  silently accepted (SPEC §7.4).
