# UR_PathGen_SF_1

Collision-free UR10e path generation for workcell SF1, verified in MuJoCo. The specification is
`SPEC.md`; accepted limitations are in `KNOWN_ISSUES.md`; reasons behind choices are in
`docs/design-decisions.md`.

Talk to the owner in Czech; write every file of the project in English.

## Skills

- `.claude/skills/project-style-rpx/` — code, shell, YAML, Markdown, URScript and MJCF conventions.
  Use it for every change.
- `.claude/skills/feature-manager/` — how a feature is added (the next one: the A → B planner).
- `.claude/skills/project-optimize/` — cleanup, speed, complexity and reliability work. Behaviour
  does not change, and a generated path is compared byte for byte.

## Commands

Never use a bare `python3` on this machine: miniforge's numpy is broken. Use `.venv/bin/python`.

```bash
./install.sh
.venv/bin/python -m pytest
.venv/bin/python .claude/skills/project-style-rpx/scripts/check_style.py src Example test
.venv/bin/python -m pytest .claude/skills/project-style-rpx/scripts/test_check_style.py
.venv/bin/mjpython Example/demo_rotate_axis.py
```
