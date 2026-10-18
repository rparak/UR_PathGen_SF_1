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

`src/URScript/Core.py` holds the only reader (`Read_URScript`) and the only writers
(`Write_URScript`, `Write_Movej_Sequence`). No other module parses or formats these lines - not a
script, not a test helper (a test compares against literal text, references/python.md > Tests).

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
  rad is read and written unchanged (SPEC §10.2).
- **Generated points** get fixed precision: `%.12f` with the leading zero dropped
  (`0.5` → `.500000000000`, `-0.5` → `-.500000000000`), `-0` written as `0`. Their `_p` comes from DH
  FK · TCP with `Normalize_Rotvec` (SPEC §9.5).

## Joints are never wrapped ✅

UR joints span ±2π and `movej` goes to the exact value. A joint is never wrapped into [−π, π]; the
planner and writer stay on the branch of A and B. A synthetic joint at 4.0 rad comes out as 4.0
(SPEC §10.5).

## Motion output: `movej(<name>_q, r=0)` only ✅

```
movej(<name>_q, r=0)
```

- **Joint targets only, never `movej(p[...])`**: with a pose target the robot solves its own IK and
  may pick another branch. `movel` is never emitted (SPEC §14 risk 5).
- `r=0` is written explicitly: it is the one argument that changes the path. A blend radius > 0 is a
  future feature that needs envelope checking (SPEC §8.5).
- `a` and `v` are omitted (URScript defaults); the operator sets the speed on the robot. Speed does
  not change the path of a `movej` with `r=0`.

## Naming 📄

| Point | Name | Role |
|---|---|---|
| robot `home` | `home` | A |
| robot `drop_rotate_1` | `drop_rotate_1` | B |
| generated | `via_<from>_<to>_<nn>`, e.g. `via_home_drop_rotate_1_01` | via points, `nn` from `01` |

Learned points keep the robot's names; their single source is the program extract
`Data/Programs/sf1_plc_v4/sf1_plc_v4_positions.urscript`. The original export is kept as a test
fixture, and its number tokens equal the extract's (SPEC §10.3). At most **5 via points** per path; more is a failure, never a longer output
(SPEC §1, §14 risk 8).

## Comparing orientations ✅

- Always the **geodesic angle**, never component-wise rotation vectors. A rotation vector with
  |r| ≈ π and its equivalent −r′ differ component-wise by ≈ 2π while describing the same rotation
  (`home_p` has |r| = 3.1393 rad).
- Computed with **atan2**, `θ = atan2(‖vee(M − Mᵀ)‖ / 2, (tr M − 1) / 2)`, `M = R_aᵀ R_b`. The
  arccos form cannot resolve angles below ≈ 6e-8 rad, and the 1e-9 checks need that (SPEC §4.5, §9.5).
- `Normalize_Rotvec` applies to generated points only: θ ∈ [0, π]; θ < 1e-12 → (0, 0, 0); if
  π − θ < 1e-9, the axis sign makes the first component with |·| > 1e-9 positive.
