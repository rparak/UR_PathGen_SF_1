# project-optimize: project reference for UR_PathGen_SF_1

This project's part of optimization work. The generic skill is personal
(`~/.claude/skills/project-optimize/`); this file overrides it where they differ. This folder has no
`SKILL.md` on purpose: a personal skill wins over a project skill of the same name (Claude Code docs,
*Resolve skills that share a name*), so a copy here would be ignored and drift.

## Outputs compared byte for byte

**The generated path is output.** Any step that touches code on the path from the learned points to
the written file - parsing, kinematics, the scene, the clearance check, sampling, the planner, the
writer - is proven by writing the output on a fixed seed before and after and comparing both files
(`.urscript` and `_Sequence.script`) byte for byte, together with the clearance report. A path that
is still collision-free but different is a behaviour change: a faster sampler that picks other via
points is for `feature-manager`.

| Output | Produced by | Deterministic inputs (seed if any) |
|---|---|---|
| `<job>.urscript` points and `_Sequence.script` motion lines, the printed report (clearance verdicts and every violation) | `.venv/bin/python Example/plan_rotation.py --job <name> --workers 8 --out "$SCRATCH/<before or after>"` (CLAUDE.md; without `--out` it overwrites `Data/Generated/`) | the job file, `Cell_SF1.yaml`, `Data/Programs/`, the committed collision set; seed `CONST_SEED` = 20261017 (`Example/plan_rotation.py`; segment k: seed × 1000 + case × 10 + k); budgets are iteration counts, so the worker count does not change the output |
| The variants and `comparison.csv` / `.md` | `Example/write_variants.py` | the same, per variant |
| The collision set and its manifest, where the build is touched | `Example/build_collision.py` (about 19 min, runs CoACD) | `CAD/SF1_Environment.stl`, the configuration |
| Every `[INFO]`/`[WARNING]`/`[ERROR]` line and operator sentence | the scripts above and the tests | as above |

Two runs of the default job (`sf1_plc_v4_case_17`, 8 workers) on 2026-10-23: both files
byte-identical to each other and to the committed `Data/Generated/` files; the printed output
identical after the filter below.

## Normalisation filter for printed output

What varies between runs and is replaced (nothing else): the taught model check's time, the two
wall-clock times of the `Wall time:` line, and the `--out` path of the `Written to` line.

```bash
F='s/(samples, )[0-9]+ s\)/\1<T> s)/; s/(Wall time: planning )[0-9]+ s/\1<T> s/; s/(, report )[0-9]+ s\./\1<T> s./; s/^(\[INFO\] Written to ).*(\. Simulation passed)/\1<OUT>\2/'
sed -E "$F" "$SCRATCH/before.stdout" > "$SCRATCH/before.normalised"
```

The estimated durations in the report (`taught %.2f s, optimised %.2f s`) are computed, not
measured, and are **not** filtered.

## Never touched

- **`models/`**: vendored, byte-identical to `models/MANIFEST.json`. A changed file fails the
  installer and the scene tests (style project page `mjcf.md`).
- **`CAD/`**: the source STL and the committed collision set. Rebuilding the set is a deliberate
  change with its own report, not a cleanup.
- **`Data/Programs/`** and test fixtures (the original robot export): their tokens are the pins.

## Reached without a grep hit

- **Entry points**: every script in `Example/`, and `install.sh` (which names `requirements.txt`,
  `models/MANIFEST.json` and the collision manifest). Scripts import other scripts as modules
  (`plan_rotation`, `replay_rotation`, `write_variants`), and tests import them too.
- **Tests**: `test/conftest.py` puts `src/` on the path; script tests run `Example/` scripts in a
  subprocess.
- **Configuration keys**: a `Cell_Configuration_Str` field is read through `Load_Configuration` and
  the YAML, a `Job_Configuration_Str` field through `Load_Job` and a job file.
- **MuJoCo names by string**: bodies, geoms and sites are looked up by name (`base`, `env_stand`,
  `upper_arm_link`, exclusion pairs in the configuration). Search for the name as a string literal
  and in `Cell_SF1.yaml`.
- **Files by path string**: `models/`, `CAD/`, `Data/Programs/`, test fixtures, and
  `.claude/skills/project-style-rpx/references/check_style.json`.
- **Docs**: `SPEC.md`, `design-decisions.md`, `KNOWN_ISSUES.md` (in `.claude/project-docs/`) and
  the README.

## Slow paths and timings

- Candidates: the scene build, segment sampling, distance queries, the collision build.
- Suite: `-m "not slow"` about 9 min, full about 20 min (CLAUDE.md; 9 min 12 s and 19 min 53 s on an
  M2 Pro, 2026-10-23). Planning `--workers 8` 2 min 43 s, 1 worker 10 min 27 s (M2 Pro, 2026-10-21).
- Non-determinism shows as a `.urscript` that differs between two runs on the same seed.

## Interpreter and tools

- Project python: `.venv/bin/python` (never a bare `python3`: miniforge's numpy is broken, CLAUDE.md).
- Explicit python for throwaway environments: `/opt/homebrew/bin/python3.11` (on Linux,
  `python3.11` from `PATH`).
- Code folders for radon: `src Example`.
