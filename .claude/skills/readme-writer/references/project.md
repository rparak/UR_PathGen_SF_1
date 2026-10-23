# readme-writer: project reference for UR_PathGen_SF_1

The README decisions of this repository. The generic skill is user level
(`~/.claude/skills/readme-writer/`); this file overrides it where they differ. This folder has no
`SKILL.md` on purpose: a project skill of the same name would shadow the user-level one.

Conventions for code, Markdown and commits come from `project-style-rpx` and `feature-manager`.
Run every Python command with `.venv/bin/python` (never a bare `python3`, `CLAUDE.md`).

## Readers

- Main reader: an engineer or researcher who wants to see the result (the variants of case 17),
  run the planner and replay a path.
- Others: an integrator adapting it to another program, case or cell (the collapsed "Another
  program, case or path" block); a maintainer (tests, regeneration, layout).

## Sources of numbers

| What the README shows | Source | Written by | Pinned by |
|---|---|---|---|
| Both results tables (times, change vs taught, via points, joint travel, TCP path, max robot height, closest, least room and margin) and the times and heights in the captions of Figures 3, 7-10 | `Data/Generated/variants/comparison.csv` (`.md` alongside) | `Example/write_variants.py` | `test/Docs/test_readme.py::test_the_results_table_is_comparison_csv` (tables only) |
| Margins (20 mm environment, 5 mm self, 5 mm station zone, 30 mm machine (a), 50 mm zone radius, 2.35 m workspace top), the TCP (0, 0, 174) mm, holding openings | `Configuration/Cell_SF1.yaml` | the owner | `test/Configuration/test_core.py::test_the_shipped_configuration_loads_in_si_units` |
| The movej a = 1.396 rad/s² (80 °/s²), v = 1.047 rad/s (60 °/s) of generated moves | `src/Planner/Core.py` `CONST_MOVEJ_A_TOKEN`, `CONST_MOVEJ_V_TOKEN` (SF1's values) | code | `test/Example/test_plan_rotation.py::test_a_program_line_the_job_cannot_time_is_refused` (the refusal) |
| 6718 collision pieces, the 10 mm excess threshold, the 0.5 mm thickening | `CAD/Collision/SF1_Environment_Collision.json` (manifest) | `Example/build_collision.py` | not pinned in the README's form (the collision tests check the set, not the count) |
| 78 learned points, largest residual 4.04 mm / 0.377° | `Data/Programs/sf1_plc_v4/sf1_plc_v4_positions.urscript`, measured | `test/Kinematics/test_core.py` | `test_the_configured_tcp_residual_is_pinned` (per point, in a band) |
| Planning time 2 min 43 s (8 workers), 10 min 27 s (1 worker) | measured run, M2 Pro, 2026-10-21 | — | — |
| Suite times 9 min 12 s (`not slow`), 19 min 53 s (full); slow test times | measured runs, M2 Pro, 2026-10-23 (the four slow-test times 2026-10-20) | — | — |
| Variant generation about 6 minutes, collision build about 19 minutes, figures under a minute | measured runs (figures: 6.9 s, M2 Pro, 2026-10-23) | — | — |
| Python 3.11, MuJoCo 3.16.0 (badges) | `install.sh`, `requirements.txt` | — | — |

A changed `comparison.csv` updates both tables, every caption that quotes a time or height, and
the figures (`make_readme_figures.py`). A re-measured runtime gets its date and machine.

## Statuses

The owner's statuses (2026-10-21), the same words in the tables, prose, figure labels and captions:

| Variant | Status words |
|---|---|
| `taught` | the taught program (reference) |
| `b_optimised` | previous optimisation, superseded by c1 |
| `c1` | previous optimisation (simulation only) |
| `c2` | **delivered — tested on the real cell (owner, 2026-10-21)**; never called unverified |
| `c3` | proposal for re-teaching, not verified on the robot |

## Figures

- **Script**: `Example/make_readme_figures.py` writes every image in `Media/` from
  `comparison.csv`, the variant files and the configuration. Options: `--config FILE`,
  `--out DIR` (default `Media/`), `--graphs-only` (skip the MuJoCo renders). Full run 6.9 s on an
  M2 Pro (2026-10-23), 30 images, byte-identical to the committed ones. Run it into a temporary
  `--out` first and compare, then into `Media/`.
- **Folder**: `Media/`, 15 figures × light and dark = 30 PNG, 5.1 MB. Budget: at most 400 KB per
  PNG, 6 MB in all (largest now: `station_collision_light.png`, 386 KB).
- **Renders show the environment as the collision model only** (the convex pieces with the
  configured corrections), never the visual STL; the captions say so.
- **Status labels per variant** in every graph (CONST_SHORT, CONST_STATUS_NOTE in the script):
  c2 delivered †, c1 previous optimisation (simulation only), c3 proposal *, with the footnote
  spelling out † and *.
- **Estimate footnote** on every graph that shows time: "Simulation estimate — blends and gripper
  time not modelled."
- **Numbering**: Figures 1-3 visible (the cell, the cycle-time graph, the c2 path), Figures 4-15 in
  the "All figures" gallery, numbered in document order. Captions: `**Figure N:**` in Markdown,
  `<p><b>Figure N:</b> ...</p>` in the gallery's table cells.
- `<img>` width: Figure 1 `85%` (centred); every other figure `100%`, of the page or, in the
  gallery's two-column tables, of its cell.

## Rules

- **Root layout**: the root holds only `README.md`, `LICENSE`, `CLAUDE.md`, `src`, `test`,
  `Example`, `Data`, `CAD`, `Configuration`, `models`, `install.sh`, `requirements.txt`,
  `.gitignore`, `Media`, `pytest.ini` and `.claude`. `pytest.ini` is the one exception to "only
  what the README describes" (the test runner needs it at the root).
- **Never mentioned in the README**: the internal docs (`.claude/project-docs/`), the skills,
  `CLAUDE.md`'s content. User-facing output names no internal document either.
- **Safety alert** directly after Figure 1, as a GitHub alert:
  `> [!WARNING]` then "**Simulation passed ≠ safe on the robot.** ...", naming c2 as the only
  path tested on the real cell (owner, 2026-10-21). "Simulation passed ≠ safe on the robot" also
  opens Limitations and safety.
- **Visible / collapsed split** (216 visible lines of 714, 2026-10-23):
  - visible: title, value sentence, badges, Figure 1, the alert, Contents, About, Results (compact
    table, Figures 2-3), Quick start, How it works (Mermaid + bullets), Inputs and outputs (file
    table), Limitations and safety, Repository layout, License, Acknowledgement;
  - collapsed: All metrics, All figures (4-15), All usage, The full method, Every file, What a run
    needs, Another program, case or path.
- The compact results table keeps 5 columns; the 10-column table lives in "All metrics".
- The `--path` table in Quick start keeps its numbers left-aligned (an option value, not a
  measurement).

## README tests

| Test | Pins |
|---|---|
| `test/Docs/test_readme.py::test_the_results_table_is_comparison_csv` | two results tables, the five variants in order, every number from `comparison.csv`, the three status words |
| `test/Docs/test_readme.py::test_every_image_the_readme_shows_exists` | every image shown is in `Media/`, every `Media/` file is shown |
| `test/Docs/test_readme.py::test_every_figure_has_an_alt_text_and_a_numbered_caption_below_it` | alt text ≥ 12 words, a caption `Figure N:` after each image, numbered in order |
| `test/Docs/test_readme.py::test_the_root_holds_only_what_the_readme_describes` | the root entries above; the internal docs in `.claude/project-docs/` |
| `test/Example/test_make_readme_figures.py::test_the_script_names_its_output_from_any_directory` | `--help` from any directory names `Media/` and `--graphs-only` |
| `test/Example/test_make_readme_figures.py::test_the_graphs_are_the_committed_ones` (slow) | the graphs regenerate byte for byte, both themes; the estimate warning is printed |
| `test/Example/test_make_readme_figures.py::test_the_collision_model_groups_cover_the_environment_once` | each environment geom in exactly one group of Figure 6, with its margin |
| `test/Example/test_make_readme_figures.py::test_the_renders_never_load_the_visual_stl` | the renders use the collision model, never the STL; both themes |

Commands:

```bash
.venv/bin/python -m pytest test/Docs/test_readme.py test/Example/test_make_readme_figures.py
.venv/bin/python ~/.claude/skills/readme-writer/scripts/check_readme.py README.md
```

Known checker results that are decisions, not defects: the left-aligned `--path` column, the two static badges (checked against `install.sh` and
`requirements.txt`), the Mermaid block (verify on GitHub after push; no `mmdc` installed).

## Pointers

- `CLAUDE.md`, Skills: one line naming this skill.
- `feature-manager` §9, README bullet: one line naming this skill.
