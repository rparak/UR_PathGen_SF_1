# UR_PathGen_SF_1

Generating and verifying collision-free joint-space paths for the UR10e (Robotiq 2F-85) of workcell
SF1 in MuJoCo, before they run on the real robot. Specification: [SPEC.md](SPEC.md). Limitations:
[KNOWN_ISSUES.md](KNOWN_ISSUES.md). **Simulation passed ≠ safe on the robot.**

## Install

macOS on Apple silicon (verified) or Ubuntu Linux (not verified), with Python 3.11
(`brew install python@3.11`, or `sudo apt install python3.11 python3.11-venv`):

```bash
./install.sh
```

It creates `.venv`, installs the pinned `requirements.txt`, and verifies the vendored models and the
collision set against their hashes. It is safe to re-run; `./install.sh --detect-only` only reports.

## Run

The demo rotates the base joint ±45° around `home`. It checks the motion first and labels it
"clearance checked" or "VISUAL ONLY - UNCHECKED".

macOS (the MuJoCo viewer needs `mjpython`):

```bash
.venv/bin/mjpython Example/demo_rotate_axis.py
```

Linux:

```bash
.venv/bin/python Example/demo_rotate_axis.py
```

Geom groups in the viewer (toggle with the number keys): `1` the visual STL of the cell, `2` the
robot's visual meshes, `3` the collision geometry (environment pieces and robot). Every viewer script
builds the scene without the visual STL by default (faster, lighter) and starts with group 3 shown;
`--with-stl` loads the STL as well.

Show the robot static at one learned position of the program, with its clearance (taught rule):

```bash
.venv/bin/mjpython Example/demo_visualize_position.py --pos home_q
.venv/bin/python Example/demo_visualize_position.py --list
```

`--opening open` shows the gripper open (85 mm), `--case 18` uses case 18's holding width. A
position inside machine (a) is labelled "model not valid here" (see `KNOWN_ISSUES.md`).

Plan the transfer segments of the rotation sequence (cases 17 and 18; about 35 minutes, writes
`Data/Generated/` and prints the taught vs optimised report). `Data/Generated/` holds, per case, the
optimised whole case in the same form as the simplified taught program in `Data/Programs/sf1_plc_v4/`:
`SF1_Rotation_<case>.urscript` (the points, including the generated via points) and
`SF1_Rotation_<case>_Sequence.script` (the taught lines unchanged except S1 – S3, which are generated
movej with `r=0`). Both can be pasted into the robot program; run them at reduced speed first:

```bash
.venv/bin/python Example/plan_rotation.py
```

Replay taught vs optimised (macOS: `mjpython`), or write screenshots instead of opening the viewer:

```bash
.venv/bin/mjpython Example/replay_rotation.py --case 17 --segment S2
.venv/bin/python Example/replay_rotation.py --case 17 --segment S2 --screenshots <directory>
```

`Data/Programs/sf1_plc_v4/` holds verbatim extracts of the owner's robot program (its learned points
and the whole cases 17 and 18) and, written from them, the simplified taught program of each case:
`sf1_plc_v4_case_<n>.urscript` (the points) and `sf1_plc_v4_case_<n>_sequence.script` (every movej and
movel of the case in program order with the program's a, v, r; gripper actions as comments; no URCap
code, no PLC I/O). Rewrite them after the extracts change:

```bash
.venv/bin/python Example/write_taught_programs.py
```

Rebuild the environment collision set after the STL changes (takes several minutes, runs CoACD):

```bash
.venv/bin/python Example/build_collision.py
```

## Test

```bash
.venv/bin/python -m pytest
```

Style checker and its own tests:

```bash
.venv/bin/python .claude/skills/project-style-rpx/scripts/check_style.py src Example test
```

```bash
.venv/bin/python -m pytest .claude/skills/project-style-rpx/scripts/test_check_style.py
```
