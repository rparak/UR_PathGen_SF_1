# UR_PathGen_SF_1

Collision-free UR10e path generation for workcell SF1, verified in MuJoCo. The internal docs are
in `.claude/project-docs/`: the specification `SPEC.md`, the accepted limitations
`KNOWN_ISSUES.md` and the reasons behind choices `design-decisions.md`. `README.md` is the public
page; it does not mention the internal docs or the skills.

Communicate with the owner in English only: reports, questions, summaries, commit messages and
every file. The owner's notes may be in Czech or Slovak; answer in English anyway.

## Skills

The skills are optional and private (`rparak/rp-claude-skills`, cloned to `~/.claude/skills/`); this
repository keeps only their project layer, `.claude/skills/<skill>/references/` (`project.md`, plus
the pages named there). The project installs, runs and passes its tests without them; only the
style checker and the README checker commands need them.

- `project-style-rpx` — code, shell, YAML and Markdown conventions; this repository's in
  `references/project.md`, `urscript.md`, `mjcf.md` and `check_style.json`. Use it for every change.
- `feature-manager` — how a feature, a behaviour change or a bug fix is added; the robotics
  checklist is in `references/project.md`.
- `project-optimize` — cleanup, speed, complexity and reliability work. Behaviour does not change,
  and a generated path is compared byte for byte (`references/project.md`).
- `readme-writer` — every README change (`references/project.md`).

## Commands

Never use a bare `python3` on this machine: miniforge's numpy is broken. Use `.venv/bin/python`.

```bash
./install.sh
.venv/bin/python -m pytest -m "not slow"     # everyday, about 9 min
.venv/bin/python -m pytest                    # before a commit, about 20 min
.venv/bin/python ~/.claude/skills/project-style-rpx/scripts/check_style.py src Example test   # needs the optional private skills
.venv/bin/python -m pytest ~/.claude/skills/project-style-rpx/scripts/test_check_style.py       # needs the optional private skills
.venv/bin/python Example/plan_rotation.py --job <name> --workers 8 --out /tmp/<name>   # Configuration/Jobs/<name>.yaml; without --out it overwrites Data/Generated/
.venv/bin/mjpython Example/replay_rotation.py --job <name> --out /tmp/<name>
.venv/bin/python Example/extract_program.py --program PATH --cases N [N ...] --out Data/Programs/<name>/
.venv/bin/mjpython Example/demo_rotate_axis.py
.venv/bin/python Example/make_readme_figures.py   # after the variants or a figure's input change
```
