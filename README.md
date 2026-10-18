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

`--opening open` shows the gripper open (85 mm). A position inside machine (a) is labelled "model
not valid here" (see `KNOWN_ISSUES.md`).

The deliverable is case 17 of the program from home to home: the taught sequence and its optimised
version. Plan its transfer segments S1 – S3 (about 10 minutes, writes `Data/Generated/` and prints
the taught vs optimised report). `Data/Generated/` holds the optimised sequence in the same form as
the simplified taught program in `Data/Programs/sf1_plc_v4/`: `SF1_Rotation_17.urscript` (the
points, including the generated via points) and `SF1_Rotation_17_Sequence.script` (the taught lines
unchanged except S1 – S3, which are generated movej with `r=0`). Both can be pasted into the robot
program; run them at reduced speed first:

```bash
.venv/bin/python Example/plan_rotation.py
```

Replay a path in the viewer (macOS: `mjpython`), or write screenshots instead:

```bash
.venv/bin/mjpython Example/replay_rotation.py --path 2
.venv/bin/mjpython Example/replay_rotation.py --path 3 --speed 0.5
.venv/bin/python Example/replay_rotation.py --path 2 --screenshots <directory>
```

| `--path` | Path (case 17, home → home) |
|---|---|
| 1 | taught |
| 2 | optimised |
| 3 | taught, then optimised, with the strict comparison |

Without `--path` the catalogue is printed and a number asked; `--list` prints it. `--speed` is
relative to the estimated real time (1.0). movel moves the TCP on a straight line; the gripper
switches between the holding width and open at the gripper actions. Each move is labelled by its
clearance (taught lines by the taught rule, generated moves by the generated rule). Entry 3 also
prints S1 – S3 of both variants judged by the generated rule. `--headless` prints every move instead
of opening the viewer.

`Data/Programs/sf1_plc_v4/` holds verbatim extracts of the owner's robot program (its learned points
and the whole cases 17 and 18, kept as ground truth for the model) and, written from them, the
simplified taught program of case 17 home → home: `sf1_plc_v4_case_17.urscript` (the points) and
`sf1_plc_v4_case_17_sequence.script` (every movej and movel in program order with the program's a,
v, r; gripper actions as comments; no URCap code, no PLC I/O). Rewrite them after the extracts
change:

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

`-m "not slow"` leaves out the tests marked slow (the replay of every segment of the program, about
70 s).

Style checker and its own tests:

```bash
.venv/bin/python .claude/skills/project-style-rpx/scripts/check_style.py src Example test
```

```bash
.venv/bin/python -m pytest .claude/skills/project-style-rpx/scripts/test_check_style.py
```
