# project-style-rpx: project reference for UR_PathGen_SF_1

The conventions of this repository that the generic skill (`~/.claude/skills/project-style-rpx/`)
leaves to the project: what `.claude/project-docs/SPEC.md` decided, as the code follows it. It
overrides the generic skill where they differ. This folder has no `SKILL.md` on purpose: a personal
skill wins over a project skill of the same name (Claude Code docs, *Resolve skills that share a
name*), so a copy here would be ignored and drift. ✅ = enforced (checker, test, loader); 📄 =
documentation only.

Project-only pages in this folder: **urscript.md** (the Polyscope `*.urscript` variable files) and
**mjcf.md** (the vendored Menagerie models, patches in code, collision geometry).

## Header and checker

- Holder: **JIC Smart Factory** - `Copyright (c) 2026 JIC Smart Factory`, `Author   : JIC Smart
  Factory`; licence MIT (SPEC §6). The `#`-comment header form; the `"""` form is not used here.
  `File Name:` examples: `../URScript/Core.py` for `src/URScript/Core.py`,
  `../Example/build_collision.py` for a script, `../test/URScript/test_core.py` for a test.
- `check_style.json` (this folder): class roots `src`, `Example`; test root `test`; check roots
  `src`, `Example`, `test`; no exclusions. Pinned by `test/Docs/test_skills.py`.
- Command (CLAUDE.md): `.venv/bin/python ~/.claude/skills/project-style-rpx/scripts/check_style.py src Example test`.

## Layout and layers (SPEC §6)

```
src/<Domain>/Core.py        Configuration, URScript, Kinematics, Collision, Scene, Planner
Example/<script>.py         build_collision.py, demo_rotate_axis.py, demo_visualize_position.py, extract_program.py,
                            make_readme_figures.py, plan_rotation.py, replay_rotation.py, write_taught_programs.py,
                            write_variants.py
test/<Domain>/test_<module>.py   mirroring src/; test/Install/, test/Example/, test/Docs/
Configuration/Cell_SF1.yaml  the cell configuration
Configuration/Jobs/           the planning jobs (what plan_rotation.py plans)
Data/Programs/<program>/     extracts of the owner's robot program: the learned points and case lines,
                            cut by Example/extract_program.py
models/                      vendored Menagerie models + MANIFEST.json
Data/Generated/              the delivered output (c2) and variants/ with the comparison table
Media/                       the README images, written by Example/make_readme_figures.py
CAD/                         the environment STL and CAD/Collision/ (the committed collision set)
.claude/project-docs/        SPEC.md, design-decisions.md, KNOWN_ISSUES.md
install.sh
```

The scripts folder is `Example/`. Scripts import other scripts as modules (`plan_rotation`,
`replay_rotation`, `write_variants`).

Layer order 📄 (no test):

```
Configuration, URScript, Kinematics, Collision   import nothing of the project
Scene                               → Configuration, URScript, Kinematics, Collision
Planner                             → Configuration, Scene
Example/                            → any of src/, and plan_rotation, replay_rotation, write_variants
```

`Kinematics` is pure DH maths and rotation helpers: it knows nothing of MuJoCo, files or the
configuration file.

## Root ✅

The root holds only `README.md`, `LICENSE`, `CLAUDE.md`, `src`, `test`, `Example`, `Data`, `CAD`,
`Configuration`, `models`, `install.sh`, `requirements.txt`, `.gitignore`, `Media`, `pytest.ini` and
`.claude`. Exception: **`pytest.ini`** (the test runner needs it at the root). Pinned by
`test/Docs/test_readme.py::test_the_root_holds_only_what_the_readme_describes`, which also checks
the three docs in `.claude/project-docs/`.

## Formats - one reader and writer each 📄 (SPEC §6)

| Format | Read and written by |
|---|---|
| Cell configuration `Configuration/Cell_SF1.yaml` | `Configuration.Core` (`Load_Configuration`); read only, written by hand |
| Planning jobs `Configuration/Jobs/<name>.yaml` | `Configuration.Core` (`Load_Job`); read only, written by hand. What a job implies in the program is resolved by `URScript.Core.Resolve_Sequence` |
| Polyscope variables `*.urscript` (`Data/Programs/`, `Data/Generated/`) | `URScript.Core` (`Read_URScript`, `Write_URScript`, `Write_Movej_Sequence`) - urscript.md |
| Program extracts: positions `*_positions.urscript` and cases `*_cases_*.script` (`Data/Programs/`) | `URScript.Core` (`Extract_Positions`, `Extract_Cases`, `Read_Extract_Source`; the positions read by `Read_URScript`, the cases by `Read_Program_Excerpt`); written only through `Example/extract_program.py` |
| Motion sequences `*.script` (`Data/Generated/`, the simplified taught program in `Data/Programs/`) | `URScript.Core` (`Read_Motion_Sequence`, `Write_Motion_Sequence`) |
| Collision set `CAD/Collision/SF1_Environment_Collision.npz` + `.json` manifest | `Collision.Core` (`Build_Collision_Set`, `Write_Collision_Set`, `Read_Collision_Set`) |
| Vendored MJCF `models/*/*.xml` | `Scene.Core` (`Scene_Cls`, through MjSpec); never written (mjcf.md) |
| Variant comparison `Data/Generated/variants/comparison.csv` / `.md` | `Example/write_variants.py` (`Write_Table`, `Read_Table_CSV`); read by `make_readme_figures.py` through `Read_Table_CSV` |
| README images `Media/*.png` | written only by `Example/make_readme_figures.py`; never read by code |
| `models/MANIFEST.json` | `install.sh` verifies it; `test/Scene/test_core.py` checks the vendored files against it (SPEC §11). No `src/` module reads it |

Tools in `Example/` go through these modules too: `build_collision.py` writes through
`Write_Collision_Set`, the demo reads the configuration through `Load_Configuration`.

The environment STL `CAD/SF1_Environment.stl` is read by one reader, `Collision.Core.Read_Binary_STL`
(numpy only): the collision build and the scene's visual mesh both go through it, and the mesh is
handed to MuJoCo as `uservert`/`userface` because MuJoCo's own STL loader refuses more than 200 000
faces (SPEC §4.5). Its hash is checked by `Read_Collision_Set` and `install.sh`.

Enforced rows (✅): `*.urscript` byte-identical round trip, generated moves
`movej(<name>_q, a=<a>, v=<v>, r=0)` only, station lines verbatim, no joint wrapping -
`test/URScript/test_core.py` (urscript.md). Vendored `models/` byte-identical to
`models/MANIFEST.json`, filterparent disabled, exclusion set == configuration expansion +
`base–env_stand` - `test/Scene/test_core.py`, `install.sh` (mjcf.md).

## Heavy runtimes

CoACD and trimesh are used by the decomposition build script (`Example/build_collision.py`, through
`Build_Collision_Set`), never by the tests (SPEC §5): the tests read the committed collision set, and
**tests never run CoACD** (SPEC §7.6). MuJoCo, numpy and scipy are the normal module-level imports.

## Configuration - `Configuration/Cell_SF1.yaml` (SPEC §8)

One file describes the cell: the environment meshes, its assembly overrides and added boxes, the
robot and its base pose, the tool and its holding openings, the clearances (the station zone, the
machine margin, the workspace), the self-collision exclusions and the FK sanity bound. It is read
only by `src/Configuration/Core.py` (`Load_Configuration` → `Cell_Configuration_Str`); the planning
jobs `Configuration/Jobs/<name>.yaml` follow the same rules through `Load_Job` →
`Job_Configuration_Str` (SPEC §8.6). The loader's rules are tested in
`test/Configuration/test_core.py`. SPEC §8: "No field without a reader."

- Header sections name the frames: world = STL frame, Polyscope base frame, flange frame (SPEC §9.1).
- Sections: `Environment.*`, `Robot.*`, `Tool.*`, `Clearance.*`, `Self_Collision.*`, `Kinematics.*`
  (`Tool.Coupling.Spigot_Depth`, `Robot.Joint_Position_Limits.Min`). Units include
  `[m, rad (rotvec)]`.
- **No optional fields** in this milestone (SPEC §8). Example of a field with its source:

```yaml
Tool:
  # Coupling between the flange and the gripper (Robotiq GRP-ES-CPL-062).
  Coupling:
    # Overall coupling thickness along the flange z axis. Source: Robotiq 2F-85 manual
    # rev. 2020/02/11, p. 119, Fig. 5-5. [m] REQUIRED.
    Thickness: 0.0169
```

- Invalid values the loader refuses include: a wrong vector length, `Min ≥ Max`, a joint limit
  beyond ±360°, the elbow beyond ±180° (the precondition of the `upper_arm_link–forearm_link`
  exclusion, SPEC §9.6), a spigot depth ≥ the coupling thickness, a negative clearance or sanity
  bound (SPEC §11). Degrees are converted to radians in the loader.
- A documented estimate: `Environment.Assembly_Overrides` is the owner's estimate, *not measured*,
  listed in `.claude/project-docs/KNOWN_ISSUES.md` (SPEC §4.11).
- **Single legal value, so a code constant, not a field** (SPEC §8): the **robot type** (the UR10e DH
  table is a code constant); the **blend radius** (always `r=0` until blend-envelope checking
  exists, SPEC §8.5); **joint wrapping** (never, SPEC §10.5); **speed and acceleration** (`movej` with
  `r=0` follows the same joint-linear path at any speed, so they cannot change what is checked); the
  `base–env_stand` exclusion (code, SPEC §9.6). In code: no `blend_radius` parameter on the writer, no
  `robot_type`, no `wrap_joints` flag (SPEC §8, §10.4). *Corrected 2026-10-19: this listed no
  `a`/`v` arguments on the `movej` line; they are the program's tokens since 2026-10-17
  (urscript.md).*
- **Derived, not stored**: the orientation sanity bound (from the position bound and the tool
  length, SPEC §9.4), the gripper's +3.0 mm shift (from the coupling values, SPEC §9.3), `R_crop`
  (SPEC §7.1).
- Machine-specific values: the project has none; no override convention is defined (*Unsettled*).

## Units

SI in code: lengths [m], angles [rad] (orientations as rotation vectors [rad] or matrices), times
[s]. Degrees appear only in `Cell_SF1.yaml` fields marked `[deg]` and are converted once by
`Load_Configuration`. The Polyscope files carry m and rad already (urscript.md).

## Errors

`Configuration_Error_Cls`, `URScript_Error_Cls`, `Kinematics_Error_Cls`, `Collision_Error_Cls`,
`Scene_Error_Cls`, `Clearance_Error_Cls`, `Planner_Error_Cls` (SPEC §7.6, §9.4, §9.7, §10.1, §10.6).
Examples of the operator sentence and of naming every violation:

- *"The collision set CAD/Collision/SF1_Environment_Collision.npz was built from a different STL
  than CAD/SF1_Environment.stl. Rerun Example/build_collision.py."* (SPEC §7.6)
- `Check_Clearance` returns every violating pair with distance and margin; `Clearance_Error_Cls`
  names the point, each pair, its distance and the margin (SPEC §9.7).
- `Load_Configuration` → `Configuration_Error_Cls` listing every problem (SPEC §8).

Constants example: `CONST_DH_D` - the official UR10e DH link offsets d1..d6, the one kinematic
truth; Menagerie offsets are patched to these (SPEC §9.2).

## Installer - `install.sh` (SPEC §5, §12)

`install.sh` is the only shell file. Tested by `test/Install/test_install.py` (sources the script,
stubs `uname`, `python3.11` and a `python3` that fails the test if called).

- **Targets**: macOS on Apple silicon - primary, verified on the development machine. Ubuntu Linux -
  written and tested only against stubs: **unverified** (SPEC §3, §12). No `.bat`: Windows is not a
  target (SPEC §3).
- **Interpreter**: Python 3.11, macOS `/opt/homebrew/bin/python3.11`, Linux `python3.11` from
  `PATH`. A bare `python3` is never called: on the development machine it is miniforge's, whose
  numpy is broken (SPEC §5).
- **Verification step**: every sha256 in `models/MANIFEST.json` against the vendored files, and the
  collision manifest's recorded STL hash against `CAD/SF1_Environment.stl`. A mismatch fails naming
  the file. **Models are never downloaded**: they are vendored and committed (mjcf.md).
- Option: `--detect-only`.
- Prerequisite: Python 3.11 (`brew install python@3.11`; Ubuntu `python3.11` and `python3.11-venv`),
  checked by the installer's presence check.
- `requirements.txt`: direct dependencies pinned `==`, grouped under comments naming what needs
  them; no lock file.

## Tests

- Layout: `test/<Domain>/test_<module>.py`, topic files where a domain needs them
  (`test/Scene/test_station.py`, `test_margin.py`, `test_program.py`;
  `test/Collision/test_conservativeness.py`, `test_excess.py`); `test/Install/test_install.py`;
  `test/Example/test_<script>.py` per script; `test/Docs/test_readme.py` (README) and
  `test/Docs/test_skills.py` (the skills layout and this folder's `check_style.json`) (SPEC §6, §11).
  `pytest.ini`: `--import-mode=importlib`, the `slow` marker.
- Everyday `-m "not slow"` (about 9 min), full suite before a commit (about 20 min) (CLAUDE.md).
- **Expected numbers come from the measurements in SPEC §4 or from the configuration, never from the
  code under test** (SPEC §11).
- Equivalence outputs: the written `.urscript` and `_Sequence.script` on a fixed seed, the
  `Check_Clearance` report, the collision-set manifest.
- Pin vs bound: the FK residual pin (SPEC §9.3 item 5) vs the sanity bound of half the environment
  clearance (SPEC §9.4).
- Script tests run `--headless` for the demo (SPEC §5). The demo runs under `mjpython` on macOS and
  exits with an `[ERROR]` line naming the command when it is not.

## Records - `.claude/project-docs/`

- Every implemented decision SPEC records (DH truth and the MjSpec patch, the configured TCP, the
  coupling geometry, the pad-span hull, …) has a design-decisions entry.
- `KNOWN_ISSUES.md` opens with **"Simulation passed ≠ safe on the robot"**: every generated path is
  a proposal, run first at reduced speed on the real cell (SPEC §1).
- `README.md`: written with readme-writer; it never mentions the internal docs or the skills
  (SPEC §6). `test/Docs/test_readme.py` pins its two results tables to `comparison.csv`, every
  `Media/` image shown and every shown image present, alt text and captions, and the root.

## Departures from the generic skill

None. (`Example/` is this project's name for the scripts folder.)
