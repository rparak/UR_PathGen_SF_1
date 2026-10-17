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

The demo rotates the base joint ±45° around `home_safe`. It checks the motion first and labels it
"clearance checked" or "VISUAL ONLY - UNCHECKED".

macOS (the MuJoCo viewer needs `mjpython`):

```bash
.venv/bin/mjpython Example/demo_rotate_axis.py
```

Linux:

```bash
.venv/bin/python Example/demo_rotate_axis.py
```

In the viewer, collision geometry is in geom group 3 (toggle with the `3` key).

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
