# URScript - the Polyscope variable files

Learned robot positions come from, and generated via points go back to, a Polyscope `*.urscript`
variable file (SPEC §1, §10). ✅ = enforced by a test; 📄 = documentation only. The tests are in
`test/URScript/test_core.py` (and `test/Kinematics/test_core.py` for orientations).

## The format 📄

```
global <name>_p=p[x, y, z, rx, ry, rz]
global <name>_q=[j0, j1, j2, j3, j4, j5]
```

| Line | Values | Unit, frame |
|---|---|---|
| `<name>_p` | TCP position `x, y, z` and rotation vector `rx, ry, rz` | m; rad (rotation vector, axis × angle); the **Polyscope base frame** |
| `<name>_q` | joints base, shoulder, elbow, wrist 1, wrist 2, wrist 3 | rad |

- Number grammar `-?(\d+)?(\.\d*)?([eE][+-]?\d+)?` with at least one digit: `.238…`, `-.175…` and
  `2.44E-5` are valid (SPEC §10.1). Separators are `, ` as Polyscope writes them; the reader also
  accepts any whitespace.
- Every point has both lines. `_q` is what the robot moves to; `_p` of a learned point is checked
  against DH FK · configured TCP (SPEC §9.4); `_p` of a generated point is informational.

## One reader, one writer 📄

`src/URScript/Core.py` holds the only readers (`Read_URScript`; for the `*.script` files
`Read_Program_Excerpt` and `Read_Motion_Sequence`) and the only writers (`Write_URScript`,
`Write_Movej_Sequence`, `Write_Motion_Sequence`). No other module parses or formats these lines - not a
script, not a test helper (a test compares against literal text; generic skill references/structure.md > One reader and writer per file format). A
program line's `a`, `v` and `r` reach a script as `Program_Line_Str.A_Token` / `V_Token` / `R_Token`.

## The reader is strict ✅

`URScript_Error_Cls`, with the line number in the message, for:

- a non-empty line that is neither form (`#` comments excepted);
- a name with `_p` but no `_q`, or the reverse;
- a duplicate name;
- a vector of the wrong length (6 for both).

Never a silent skip, never a best-effort parse.

## Tokens are kept, the round trip is byte-identical ✅

- The reader keeps each number's **original token** beside its float. The writer writes learned
  points from the tokens, so `Write_URScript(Read_URScript(text)) == text`, byte for byte.
- A learned point is never re-formatted, re-normalised or rounded. Encoder noise such as `2.44E-5`
  rad is read and written unchanged (SPEC §10.2). The one exception: a point a job configures with
  another `_q` (SPEC §8.6) gets its changed joints as generated tokens (`Replace_Joints`); its `_p`
  and every unchanged token stay verbatim.
- **Generated points** get fixed precision: `%.12f` with the leading zero dropped
  (`0.5` → `.500000000000`, `-0.5` → `-.500000000000`); a value that rounds to zero loses its sign
  (`.000000000000`). Their `_p` comes from DH
  FK · TCP with `Matrix_To_Rotvec` (SPEC §9.5).

## Joints are never wrapped ✅

UR joints span ±2π and `movej` goes to the exact value. A joint is never wrapped into [−π, π]; a
generated segment stays on the branches of its two ends, and a learned point takes another `_q` only
through its job's `Point_Configurations`, with a source. A synthetic joint at 4.0 rad comes out as 4.0
(SPEC §10.5).

## Motion output: generated `movej(<name>_q, a=<a>, v=<v>, r=0)`, station lines verbatim ✅

```
movej(<name>_q, a=1.3962634015954636, v=1.0471975511965976, r=0)
```

- **A generated move has a joint target only, never `movej(p[...])`**: with a pose target the robot
  solves its own IK and may pick another branch. No generated `movel` is written (SPEC §14 risk 5).
- **Station lines are the one exception**: in a motion sequence (`Write_Motion_Sequence`) the
  station operations of the robot program are copied verbatim, with their own `movel(...)` and
  `movej(get_inverse_kin(<name>_p, qnear=<name>_q), ...)` and their a, v, r (SPEC §10.4).
- `r=0` is written explicitly: it is the one argument that changes the path. A blend radius > 0 is a
  future feature that needs envelope checking (SPEC §8.5).
- `a` and `v` are the robot program's tokens, passed to `Write_Movej_Sequence`, so a generated move
  runs at the speed the duration estimate assumes. Speed does not change the path of a `movej` with
  `r=0`.

## Naming 📄

| Point | Name | Role |
|---|---|---|
| learned | the robot program's name, e.g. `home`, `Waypoint_18` | a segment's ends come from the job (`From`, `To`); the tests' A = `home`, B = `drop_rotate_1` |
| generated | `via_<from>_<to>_<nn>`, e.g. `via_Waypoint_5_Waypoint_18_01` | via points, `nn` from `01` |

Learned points keep the robot's names; their single source is the positions extract of the job's
program, `Data/Programs/<program>/<program>_positions.urscript` (`sf1_plc_v4` for SF1). The original export is kept as a test
fixture, and its number tokens equal the extract's (SPEC §10.3). At most **5 via points** per path; more is a failure, never a longer output
(SPEC §1, §14 risk 8).

## Comparing orientations ✅

- Always the **geodesic angle**, never component-wise rotation vectors. A rotation vector with
  |r| ≈ π and its equivalent −r′ differ component-wise by ≈ 2π while describing the same rotation
  (`home_p` has |r| = 3.1393 rad).
- Computed with **atan2**, `θ = atan2(‖vee(M − Mᵀ)‖ / 2, (tr M − 1) / 2)`, `M = R_aᵀ R_b`. The
  arccos form cannot resolve angles below ≈ 6e-8 rad, and the 1e-9 checks need that (SPEC §4.5, §9.5).
- Of the written points, `Matrix_To_Rotvec` applies to generated points only: θ ∈ [0, π]; θ < 1e-12 → (0, 0, 0); if
  π − θ < 1e-9, the axis sign makes the first component with |·| > 1e-9 positive.
