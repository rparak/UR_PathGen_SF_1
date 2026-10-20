## =========================================================================== ##
# MIT License
# Copyright (c) 2026 JIC Smart Factory
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
## =========================================================================== ##
# Author   : JIC Smart Factory
# File Name: ../test/Example/test_write_variants.py
## =========================================================================== ##

# System (Default)
import os
import sys
# Subprocess (Running the writer as a person does)
import subprocess
# CSV (Comma-separated values)
import csv
# Dataclasses (Data Classes)
import dataclasses
# Importlib (Import machinery)
import importlib.util
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.:
#   ../src/URScript/Core — the variant files
from URScript.Core import Read_URScript
#   ../src/Kinematics/Core — the proposals' branches
from Kinematics.Core import IK_Branch, TCP_Pose, Pose_To_Matrix, Geodesic_Angle
#   ../src/Scene/Core — the scene of each gripper state
from Scene.Core import Scene_Cls

"""
Description:
    Initialization of constants.

    The variants of case 17 home -> home (Data/Generated/variants/) and their comparison table. The
    expected totals are typed here from the planner's own report (taught, b_optimised, c1; SPEC 10.6,
    17) and the writer's run of 2026-10-19 (c2, c3).
"""
# The variants in order, and the estimated home -> home duration of each (s, as written: 0.01 s).
#   Unit [Dict<string, string>]
CONST_TOTALS = {'taught': '23.37', 'b_optimised': '17.50', 'c1': '14.66', 'c2': '12.05', 'c3': '13.86'}
# The proposals' drop branches (SPEC 10.6) and the drop's points.
#   Unit [Dict<string, string>], [Tuple<string>]
CONST_BRANCHES = {'c2': 'S- E- W-', 'c3': 'S- E- W+'}
CONST_DROP = ('drop_rotate_1', 'Waypoint_4', 'Waypoint_5')
# The status words each variant's files must carry.
#   Unit [Dict<string, string>]
CONST_STATUS = {'taught': 'taught program', 'b_optimised': 'previous optimisation', 'c1': 'delivered',
                'c2': 'proposal for re-teaching, not verified on the robot', 'c3': 'proposal for re-teaching, not verified on the robot'}


def _Writer(root):
    for folder in ('Example', 'src'):
        if os.path.join(root, folder) not in sys.path:
            sys.path.insert(0, os.path.join(root, folder))
    spec = importlib.util.spec_from_file_location('write_variants', os.path.join(root, 'Example', 'write_variants.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _Folder(root, *parts):
    return os.path.join(root, 'Data', 'Generated', 'variants', *parts)


def _Visible(folder):
    # The names in a folder without hidden files (.DS_Store, which Finder writes; git ignores it).
    return sorted(name for name in os.listdir(folder) if not name.startswith('.'))


def _Points(root, variant):
    with open(_Folder(root, variant, 'SF1_Rotation_17.urscript'), encoding='utf-8') as handle:
        return Read_URScript(handle.read()).Points


def _Rows(root):
    with open(_Folder(root, 'comparison.csv'), encoding='utf-8', newline='') as handle:
        return list(csv.DictReader(handle))


def test_every_variant_has_its_files_and_its_status(root):
    """
    Description:
        Each of the five variants has its points and its sequence, each file naming the variant's
        status (taught program, previous optimisation, delivered, proposal for re-teaching not
        verified on the robot) and saying simulation is not safety. Hidden files (a .DS_Store that
        Finder writes, ignored by git) are not part of the output.
    """

    assert _Visible(_Folder(root)) == sorted(list(CONST_TOTALS) + ['comparison.csv', 'comparison.md'])
    for variant, status in CONST_STATUS.items():
        assert _Visible(_Folder(root, variant)) == ['SF1_Rotation_17.urscript', 'SF1_Rotation_17_Sequence.script']
        for name in _Visible(_Folder(root, variant)):
            with open(_Folder(root, variant, name), encoding='utf-8') as handle:
                text = handle.read()
            assert '# Variant %s: ' % variant in text and status in text, (variant, name)
            assert 'Simulation passed != safe on the robot' in text


def test_c1_is_the_delivered_output(root):
    """
    Description:
        The c1 variant is the delivered output of Data/Generated/: the same points token for token
        and the same motion lines; only the status comments differ.
    """

    with open(os.path.join(root, 'Data', 'Generated', 'SF1_Rotation_17.urscript'), encoding='utf-8') as handle:
        delivered = Read_URScript(handle.read()).Points
    c1 = _Points(root, 'c1')
    assert list(c1) == list(delivered)
    assert all((c1[n].P_Tokens, c1[n].Q_Tokens) == (delivered[n].P_Tokens, delivered[n].Q_Tokens) for n in c1)
    texts = []
    for path in (os.path.join(root, 'Data', 'Generated', 'SF1_Rotation_17_Sequence.script'), _Folder(root, 'c1', 'SF1_Rotation_17_Sequence.script')):
        with open(path, encoding='utf-8') as handle:
            texts.append([line for line in handle.read().splitlines() if not line.startswith('#') or line.startswith('# Gripper')])
    assert texts[0] == texts[1]


def test_the_taught_variant_is_the_taught_program_apart_from_its_header(root):
    """
    Description:
        The taught program is kept twice on purpose: Data/Programs/sf1_plc_v4/ holds it as input,
        variants/taught as the comparison. Each pair of files (points, sequence) is the same after
        its header, the leading comment lines, line for line; only the headers differ.
    """

    pairs = (('sf1_plc_v4_case_17.urscript', 'SF1_Rotation_17.urscript'),
             ('sf1_plc_v4_case_17_sequence.script', 'SF1_Rotation_17_Sequence.script'))
    for program, variant in pairs:
        texts = []
        for path in (os.path.join(root, 'Data', 'Programs', 'sf1_plc_v4', program), _Folder(root, 'taught', variant)):
            with open(path, encoding='utf-8') as handle:
                lines = handle.read().splitlines()
            header = next(i for i, line in enumerate(lines) if not line.startswith('#'))
            assert header > 0, path
            texts.append((lines[:header], lines[header:]))
        assert texts[0][1] == texts[1][1], program
        assert texts[0][0] != texts[1][0], program

@pytest.mark.parametrize('variant', sorted(CONST_BRANCHES))
def test_a_proposal_moves_the_drop_to_its_branch_and_nothing_else(root, points, cfg, variant):
    """
    Description:
        A proposal's drop points keep the program's _p tokens; their joints lie on the proposal's
        branch and reach that pose with the configured TCP (nominal DH, 1e-9 m / rad); every other
        learned point is c1's, token for token.
    """

    proposal, c1 = _Points(root, variant), _Points(root, 'c1')
    for name, point in proposal.items():
        if name.startswith('via_'):
            continue
        if name in CONST_DROP:
            assert point.P_Tokens == points[name].P_Tokens and IK_Branch(point.Q) == CONST_BRANCHES[variant]
            reached, target = TCP_Pose(point.Q, cfg.TCP_Offset), Pose_To_Matrix(points[name].P)
            assert np.linalg.norm(reached[:3, 3] - target[:3, 3]) < 1e-9 and Geodesic_Angle(reached[:3, :3], target[:3, :3]) < 1e-9
        else:
            assert (point.P_Tokens, point.Q_Tokens) == (c1[name].P_Tokens, c1[name].Q_Tokens), name


def test_the_table_totals_are_the_reported_durations_with_no_rule_broken(root):
    """
    Description:
        comparison.csv holds every variant's sections and total in order; the home -> home estimates
        are the ones the planner reported (taught 23.37, b_optimised 17.50, c1 14.66 s) and the
        writer's run measured (c2 12.05, c3 13.86 s, 2026-10-19; the branch study's 12.06 s for c2
        planned from the unrounded IK values, its S2 differs); no sample of any variant breaks the
        rule of its move.
    """

    rows = _Rows(root)
    assert [(r['variant'], r['section']) for r in rows] == [(v, s) for v in CONST_TOTALS for s in ('S1', 'drop', 'S2', 'pick', 'S3', 'total')]
    assert {r['variant']: r['t_est_s'] for r in rows if r['section'] == 'total'} == CONST_TOTALS
    assert all(r['violating_samples'] == '0' for r in rows)


def test_a_learned_point_outside_the_sanity_bound_is_refused(perturbed_copy, fk_refusal, tmp_path, cfg):
    """
    Description:
        Every learned point of the extract is checked against the FK sanity bound before planning
        (SPEC 9.4): with drop_rotate_1's base joint turned by 0.05 rad (about 25 mm), the writer stops
        with exit 1 and the point's numbers, and nothing is written.
    """

    copy = perturbed_copy(os.path.join('Data', 'Programs', 'sf1_plc_v4', 'sf1_plc_v4_positions.urscript'), 'drop_rotate_1')
    result = subprocess.run([sys.executable, os.path.join(copy, 'Example', 'write_variants.py'), '--out', str(tmp_path / 'out')],
                            capture_output=True, text=True, cwd=str(tmp_path), timeout=600)
    assert result.returncode == 1, result.stdout + result.stderr
    refusal = fk_refusal.search(result.stdout)
    assert refusal and refusal.group(1) == 'drop_rotate_1', result.stdout
    assert float(refusal.group(2)) > cfg.FK_Sanity_Bound_Position * 1e3
    assert not (tmp_path / 'out').exists()


def test_every_variant_header_states_what_is_verbatim(root, points):
    """
    Description:
        In every planned variant (b_optimised, c1, c2, c3) the points file's header names the
        learned points written verbatim from the extract and those with its _p and another _q, and
        both claims hold token for token for every learned point; b_optimised has none of the latter,
        c1 the pick's three, c2 and c3 the drop's three as well. Every file of every variant says
        once that simulation is not safety.
    """

    expected = {'b_optimised': (), 'c1': ('Waypoint_18', 'pick_rotate_1', 'Waypoint_6'),
                'c2': ('drop_rotate_1', 'Waypoint_4', 'Waypoint_5', 'Waypoint_18', 'pick_rotate_1', 'Waypoint_6'),
                'c3': ('drop_rotate_1', 'Waypoint_4', 'Waypoint_5', 'Waypoint_18', 'pick_rotate_1', 'Waypoint_6')}
    for variant, other in expected.items():
        with open(_Folder(root, variant, 'SF1_Rotation_17.urscript'), encoding='utf-8') as handle:
            header = [line for line in handle.read().splitlines() if line.startswith('#')]
        learned = [name for name in _Points(root, variant) if not name.startswith('via_')]
        verbatim = [name for name in learned if name not in other]
        assert ('# Verbatim from Data/Programs/sf1_plc_v4/sf1_plc_v4_positions.urscript (_p and _q): %s.' % ', '.join(verbatim)
                in header), variant
        assert [line for line in header if line.startswith("# The program's _p, another _q:")] == (
            ["# The program's _p, another _q: %s." % ', '.join(n for n in learned if n in other)] if other else []), variant
        written = _Points(root, variant)
        for name in learned:
            assert written[name].P_Tokens == points[name].P_Tokens, (variant, name)
            assert (written[name].Q_Tokens == points[name].Q_Tokens) == (name not in other), (variant, name)
    for variant in CONST_TOTALS:
        for name in _Visible(_Folder(root, variant)):
            with open(_Folder(root, variant, name), encoding='utf-8') as handle:
                assert handle.read().count('# Simulation passed != safe on the robot') == 1, (variant, name)


@pytest.mark.slow
def test_the_table_is_what_the_variant_files_measure(root, cfg):
    """
    Description:
        Measured again from the five variants' files (Measure_Variant, the scenes of both gripper
        states), the table written from them (Table_CSV, Table_Markdown) is comparison.csv and
        comparison.md byte for byte.
    """

    writer = _Writer(root)
    scenes = {state: Scene_Cls(dataclasses.replace(cfg, Opening=writer.Opening(cfg, state))) for state in ('holding', 'open')}
    table = {variant: writer.Measure_Variant(_Folder(root, variant), cfg, scenes) for variant in CONST_TOTALS}
    with open(_Folder(root, 'comparison.csv'), encoding='utf-8') as handle:
        assert writer.Table_CSV(table) == handle.read()
    with open(_Folder(root, 'comparison.md'), encoding='utf-8') as handle:
        assert writer.Table_Markdown(table, writer.Blends(_Folder(root, 'taught')), cfg.Base_Position[2]) == handle.read()


def test_the_table_is_read_back_as_written(root, tmp_path):
    """
    Description:
        Read_Table_CSV returns comparison.csv as written, every value as its string, per variant and
        section (compared with the standard csv reader); a file with other columns is refused.
    """

    writer = _Writer(root)
    table = writer.Read_Table_CSV(_Folder(root, 'comparison.csv'))
    rows = _Rows(root)
    assert sum(len(sections) for sections in table.values()) == len(rows)
    for row in rows:
        assert table[row['variant']][row['section']] == row
    other = tmp_path / 'comparison.csv'
    other.write_text('variant,section,t_est_s\ntaught,total,23.37\n', encoding='utf-8')
    with pytest.raises(ValueError, match='columns of comparison.csv'):
        writer.Read_Table_CSV(str(other))
