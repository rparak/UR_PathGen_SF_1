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
# File Name: ../test/Example/test_plan_rotation.py

# System (Default)
import os
import sys
# Subprocess (Subprocess management)
import subprocess
# Dataclasses (Data Classes)
import dataclasses
# Importlib (Import machinery)
import importlib.util
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Math (Mathematical functions)
import math
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.:
#   ../Kinematics/Core — the pose of a point
from Kinematics.Core import TCP_Pose
#   ../Configuration/Core — the error a missing case raises
from Configuration.Core import Configuration_Error_Cls
#   ../URScript/Core — the case extract
from URScript.Core import Read_Program_Excerpt, Read_URScript

"""
Description:
    Initialization of constants.

    The gripper opening of the deliverable's path comes from the configuration (case 17's
    Tool.Holding_Openings entry), and the open state is the full 2F-85 stroke.
"""
# The delivered output's learned points by provenance (owner, 2026-10-19, SPEC 10.5): written
# verbatim from the extract, or the extract's _p with wrist 3 one turn lower (the pick operation);
# and the station lines among the latter.
#   Unit [Tuple<string>]
CONST_VERBATIM = ('home', 'drop_rotate_1', 'Waypoint_4', 'Waypoint_5')
CONST_OTHER_Q = ('Waypoint_18', 'pick_rotate_1', 'Waypoint_6')
CONST_STATION_OTHER_Q = ('pick_rotate_1', 'Waypoint_6')


def _Plan_Module(root):
    spec = importlib.util.spec_from_file_location('plan_rotation', os.path.join(root, 'Example', 'plan_rotation.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_holding_opening_is_the_cases_configured_value(root, cfg):
    """
    Description:
        While the part is held, the path uses case 17's configured opening (another entry's opening
        and Tool.Opening are not used); open is 85 mm.
    """

    plan = _Plan_Module(root)
    other = dataclasses.replace(cfg, Opening=0.030, Holding_Openings={17: (0.045, 'a'), 18: (0.040, 'b')})
    assert plan.Opening(other, 'holding') == 0.045
    assert plan.Opening(other, 'open') == 0.085


def test_a_configuration_without_case_17_is_refused(root, cfg):
    """
    Description:
        A configuration whose Tool.Holding_Openings does not list case 17 is refused, naming the case
        and the field, never given a default.
    """

    plan = _Plan_Module(root)
    with pytest.raises(Configuration_Error_Cls, match=r'Tool\.Holding_Openings has no entry for case 17'):
        plan.Opening(dataclasses.replace(cfg, Holding_Openings={18: (0.050, 'a')}), 'holding')


def test_the_optimised_sequence_starts_at_home_with_a_generated_move(root):
    """
    Description:
        Rotation_Entries is home -> ... -> home and nothing else: the program's move to home blends
        (r=0.01, typed from the program) into S1, so the start is a generated movej to home (r=0); S1
        - S3 are generated movej to their via points and ends; the station lines are the program's,
        verbatim; no line of the loading moves before or after appears.
    """

    plan = _Plan_Module(root)
    with open(os.path.join(root, 'Data', 'Programs', 'sf1_plc_v4', 'sf1_plc_v4_cases_17_18.script'), encoding='utf-8') as handle:
        lines = Read_Program_Excerpt(handle.read())[17]
    homes = [k for k, line in enumerate(lines) if line.Kind == 'movej' and line.Target == 'home']
    assert lines[homes[0]].Raw.rstrip().endswith(', r=0.01)')
    entries = plan.Rotation_Entries(lines[homes[0]:homes[1] + 1], {'S1': ['v1'], 'S2': ['v2'], 'S3': ['v3']})
    moves = [e for e in entries if e[0] != 'comment']
    raw = {(line.Kind, line.Target): line.Raw for line in lines[homes[0]:homes[1] + 1] if line.Kind in ('movel', 'movej')}
    assert moves == [('movej', 'home'), ('movej', 'v1'), ('movej', 'drop_rotate_1'), ('program', raw['movel', 'Waypoint_4']),
                     ('program', raw['movej', 'Waypoint_5']), ('movej', 'v2'), ('movej', 'Waypoint_18'),
                     ('program', raw['movej', 'pick_rotate_1']), ('program', raw['movel', 'Waypoint_6']), ('movej', 'v3'),
                     ('movej', 'home')]


def _Run(root, tmp_path, *args):
    return subprocess.run([sys.executable, os.path.join(root, 'Example', 'plan_rotation.py')] + list(args),
                          capture_output=True, text=True, cwd=str(tmp_path), timeout=3600)


def test_a_worker_count_below_one_is_refused(root, tmp_path):
    """
    Description:
        --workers 0 is refused with exit 1 and a sentence naming the flag, before anything is
        planned or written.
    """

    result = _Run(root, tmp_path, '--workers', '0', '--out', str(tmp_path / 'out'))
    assert result.returncode == 1
    assert '[ERROR] --workers must be 1 or more' in result.stdout
    assert not (tmp_path / 'out').exists()


def test_a_learned_point_outside_the_sanity_bound_is_refused(perturbed_copy, fk_refusal, tmp_path, cfg):
    """
    Description:
        Every learned point of the extract is checked against the FK sanity bound before planning
        (SPEC 9.4): with drop_rotate_1's base joint turned by 0.05 rad (about 25 mm), the run stops with
        exit 1 and the point's numbers, and nothing is written.
    """

    copy = perturbed_copy(os.path.join('Data', 'Programs', 'sf1_plc_v4', 'sf1_plc_v4_positions.urscript'), 'drop_rotate_1')
    result = _Run(copy, tmp_path, '--out', str(tmp_path / 'out'))
    assert result.returncode == 1, result.stdout + result.stderr
    refusal = fk_refusal.search(result.stdout)
    assert refusal and refusal.group(1) == 'drop_rotate_1', result.stdout
    assert float(refusal.group(2)) > cfg.FK_Sanity_Bound_Position * 1e3
    assert not (tmp_path / 'out').exists()


@pytest.mark.slow
def test_a_parallel_run_writes_the_committed_output(root, tmp_path):
    """
    Description:
        The full planning run with 4 spawned workers (S1 - S3 at once, the recorded seed and budget),
        started in a fresh interpreter from an unrelated directory, writes both output files byte for
        byte as committed in Data/Generated/ (written by the sequential run), and reports its wall time.
    """

    result = _Run(root, tmp_path, '--workers', '4', '--out', str(tmp_path / 'out'))
    assert result.returncode == 0, result.stdout + result.stderr
    assert '[INFO] Wall time: planning' in result.stdout and '(4 workers)' in result.stdout
    for name in ('SF1_Rotation_17.urscript', 'SF1_Rotation_17_Sequence.script'):
        with open(os.path.join(root, 'Data', 'Generated', name), 'rb') as handle:
            committed = handle.read()
        assert (tmp_path / 'out' / name).read_bytes() == committed, name


def test_the_delivered_points_turn_only_the_picks_wrist_3(root, points, cfg):
    """
    Description:
        Delivered_Points (owner, 2026-10-19, SPEC 10.5): wrist 3 of Waypoint_18, pick_rotate_1 and
        Waypoint_6 one turn lower (taught 4.739 rad), every other joint and every _p token the
        program's, every other point unchanged; the same arm pose (DH TCP equal to 1e-12 m and
        rad); the given points are not changed.
    """

    plan = _Plan_Module(root)
    before = {name: list(point.Q_Tokens) for name, point in points.items()}
    delivered = plan.Delivered_Points(points)
    turned = ('Waypoint_18', 'pick_rotate_1', 'Waypoint_6')
    for name, point in points.items():
        d = delivered[name]
        assert d.P_Tokens == point.P_Tokens
        if name not in turned:
            assert d is point
            continue
        assert d.Q_Tokens[:5] == point.Q_Tokens[:5] and point.Q[5] == pytest.approx(4.739, abs=0.001)
        assert d.Q[5] == pytest.approx(point.Q[5] - 2.0 * math.pi, abs=1e-11)
        a, b = TCP_Pose(d.Q, cfg.TCP_Offset), TCP_Pose(point.Q, cfg.TCP_Offset)
        assert np.max(np.abs(a - b)) < 1e-12
    assert {name: list(point.Q_Tokens) for name, point in points.items()} == before


def test_the_delivered_headers_state_what_is_verbatim(root, points):
    """
    Description:
        The delivered points file names the learned points written verbatim from the extract (home,
        the drop's three) and those with the extract's _p and another _q (the pick's three, wrist 3
        one turn lower), and both claims hold token for token; it claims nothing else verbatim. The
        sequence file names the station lines whose points have another _q (the pick and the lift).
    """

    folder = os.path.join(root, 'Data', 'Generated')
    with open(os.path.join(folder, 'SF1_Rotation_17.urscript'), encoding='utf-8') as handle:
        text = handle.read()
    header = [line for line in text.splitlines() if line.startswith('#')]
    assert ('# Verbatim from Data/Programs/sf1_plc_v4/sf1_plc_v4_positions.urscript (_p and _q): %s.' % ', '.join(CONST_VERBATIM)
            in header)
    assert "# The program's _p, another _q: %s." % ', '.join(CONST_OTHER_Q) in header
    assert [line for line in header if 'verbatim' in line.lower()] == [header[3]]
    delivered = Read_URScript(text).Points
    assert [n for n in delivered if not n.startswith('via_')] == ['home', 'drop_rotate_1', 'Waypoint_4', 'Waypoint_5',
                                                                    'Waypoint_18', 'pick_rotate_1', 'Waypoint_6']
    for name in CONST_VERBATIM:
        assert (delivered[name].P_Tokens, delivered[name].Q_Tokens) == (points[name].P_Tokens, points[name].Q_Tokens)
    for name in CONST_OTHER_Q:
        assert delivered[name].P_Tokens == points[name].P_Tokens and delivered[name].Q_Tokens != points[name].Q_Tokens
    with open(os.path.join(folder, 'SF1_Rotation_17_Sequence.script'), encoding='utf-8') as handle:
        sequence = [line for line in handle.read().splitlines() if line.startswith('#')]
    assert ("# The station lines to %s are the program's text; their points have the program's _p and" % ', '.join(CONST_STATION_OTHER_Q)
            in sequence)


def test_a_written_point_with_another_pose_is_refused(root, points):
    """
    Description:
        Provenance sorts the learned points written into verbatim and another _q, and refuses one
        whose _p is not the extract's (it would be another pose under a learned name).
    """

    plan = _Plan_Module(root)
    delivered = plan.Delivered_Points(points)
    assert plan.Provenance(['home', 'pick_rotate_1'], delivered, points) == (['home'], ['pick_rotate_1'])
    moved = dict(delivered, home=points['drop_rotate_1'])
    with pytest.raises(plan.Planner_Error_Cls, match='The point home to be written has another _p'):
        plan.Provenance(['home'], moved, points)
