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
# Re (Regular expressions)
import re
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# PyYAML (YAML parser and emitter) [pip3 install pyyaml]
import yaml
# Custom Lib.:
#   ../Kinematics/Core — the pose of a point
from Kinematics.Core import TCP_Pose
#   ../Planner/Core — the duration estimates, the station time typed from them
from Planner.Core import Estimate_Duration, Linear_Duration
#   ../Configuration/Core — the error a missing case raises
from Configuration.Core import Configuration_Error_Cls
#   ../Scene/Core — the scene of each gripper state
from Scene.Core import Scene_Cls
#   ../URScript/Core — the case extract
from URScript.Core import Read_Program_Excerpt, Read_URScript

"""
Description:
    Initialization of constants.

    The gripper opening of the deliverable's path comes from the configuration (case 17's
    Tool.Holding_Openings entry), and the open state is the full 2F-85 stroke.
"""
# The delivered output's (c2, owner 2026-10-21) learned points by provenance (SPEC 10.5): written
# verbatim from the extract, or the extract's _p with another _q (the drop on branch S- E- W-, the
# pick's wrist 3 one turn lower); and the station lines among the latter.
#   Unit [Tuple<string>]
CONST_VERBATIM = ('home',)
CONST_OTHER_Q = ('drop_rotate_1', 'Waypoint_4', 'Waypoint_5', 'Waypoint_18', 'pick_rotate_1', 'Waypoint_6')
CONST_STATION_OTHER_Q = ('Waypoint_4', 'Waypoint_5', 'pick_rotate_1', 'Waypoint_6')
# The variant files the two case-17 jobs reproduce, point and motion lines (owner, 2026-10-21).
#   Unit [Dict<string, string>]
CONST_JOB_VARIANTS = {'sf1_plc_v4_case_17': 'c2', 'sf1_plc_v4_case_17_c1': 'c1'}


def _Plan_Module(root):
    spec = importlib.util.spec_from_file_location('plan_rotation', os.path.join(root, 'Example', 'plan_rotation.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _Job(plan, name='sf1_plc_v4_case_17'):
    return plan.Load_Job_Plan(name)


def test_the_holding_opening_is_the_cases_configured_value(root, cfg):
    """
    Description:
        While the part is held, the path uses the configured opening of case 17 of sf1_plc_v4
        (another case's, another program's case 17 and Tool.Opening are not used); open is 85 mm.
    """

    plan = _Plan_Module(root)
    other = dataclasses.replace(cfg, Opening=0.030, Holding_Openings={('sf1_plc_v4', 17): (0.045, 'a'), ('sf1_plc_v4', 18): (0.040, 'b'),
                                                                      ('other', 17): (0.035, 'c')})
    assert plan.Opening(other, 'holding', _Job(plan)) == 0.045
    assert plan.Opening(other, 'open', _Job(plan)) == 0.085


def test_a_configuration_without_case_17_is_refused(root, cfg):
    """
    Description:
        A configuration whose Tool.Holding_Openings does not list case 17 of sf1_plc_v4 is refused,
        naming the case, the program and the field, never given a default (case 17 of another
        program does not count).
    """

    plan = _Plan_Module(root)
    with pytest.raises(Configuration_Error_Cls, match=r'Tool\.Holding_Openings has no entry for case 17 of sf1_plc_v4'):
        plan.Opening(dataclasses.replace(cfg, Holding_Openings={('sf1_plc_v4', 18): (0.050, 'a'), ('other', 17): (0.05, 'b')}),
                     'holding', _Job(plan))


def test_the_variant_file_names_are_the_case_17_jobs_output(root):
    """
    Description:
        The variants' file names (CONST_FILE_NAMES) are the default job's Output names: one name for
        the delivered files in Data/Generated/ and in Data/Generated/variants/<name>/.
    """

    plan = _Plan_Module(root)
    job = _Job(plan).Job
    assert plan.CONST_FILE_NAMES == (job.Output_Points, job.Output_Sequence) == ('SF1_Rotation_17.urscript',
                                                                              'SF1_Rotation_17_Sequence.script')


def test_the_station_time_is_estimated_from_the_station_lines(root, cfg):
    """
    Description:
        The station operations' time is the sum over their lines, each from the end before it with
        its own a and v: the drop's movel (1.2 m/s^2, 0.25 m/s, SPEC 10.6) and retreat movej, the
        pick's movej and the lift's movel, typed here from the program's sequence; 2.31 s (the
        report, 2026-10-19).
    """

    plan = _Plan_Module(root)
    job = _Job(plan)
    p = plan.Configured_Points(job, cfg)
    a, v = 1.3962634015954636, 1.0471975511965976
    expected = (Linear_Duration(p['drop_rotate_1'].P, p['Waypoint_4'].P, 1.2, 0.25)
                + Estimate_Duration([p['Waypoint_4'].Q, p['Waypoint_5'].Q], a, v)
                + Estimate_Duration([p['Waypoint_18'].Q, p['pick_rotate_1'].Q], a, v)
                + Linear_Duration(p['pick_rotate_1'].P, p['Waypoint_6'].P, 1.2, 0.25))
    assert plan.Station_Duration(job, p) == expected
    assert round(expected, 2) == 2.31


def test_an_unknown_job_is_refused_before_anything_is_written(root, tmp_path):
    """
    Description:
        --job naming no file in Configuration/Jobs/ is refused with exit 1, naming the job and the
        jobs there; nothing is written.
    """

    result = _Run(root, tmp_path, '--job', 'sf1_plc_v4_case_99', '--out', str(tmp_path / 'out'))
    assert result.returncode == 1
    assert '[ERROR] The job "sf1_plc_v4_case_99" cannot be read' in result.stdout and 'sf1_plc_v4_case_17' in result.stdout
    assert not (tmp_path / 'out').exists()


def test_the_optimised_sequence_starts_at_home_with_a_generated_move(root):
    """
    Description:
        Sequence_Entries of the default job is home -> ... -> home and nothing else: the program's move to home blends
        (r=0.01, typed from the program) into S1, so the start is a generated movej to home (r=0); S1
        - S3 are generated movej to their via points and ends; the station lines are the program's,
        verbatim; no line of the loading moves before or after appears.
    """

    plan = _Plan_Module(root)
    with open(os.path.join(root, 'Data', 'Programs', 'sf1_plc_v4', 'sf1_plc_v4_cases_17_18.script'), encoding='utf-8') as handle:
        lines = Read_Program_Excerpt(handle.read())[17]
    homes = [k for k, line in enumerate(lines) if line.Kind == 'movej' and line.Target == 'home']
    assert lines[homes[0]].Raw.rstrip().endswith(', r=0.01)')
    entries = plan.Sequence_Entries(_Job(plan), {'S1': ['v1'], 'S2': ['v2'], 'S3': ['v3']})
    moves = [e for e in entries if e[0] != 'comment']
    raw = {(line.Kind, line.Target): line.Raw for line in lines[homes[0]:homes[1] + 1] if line.Kind in ('movel', 'movej')}
    assert moves == [('movej', 'home'), ('movej', 'v1'), ('movej', 'drop_rotate_1'), ('program', raw['movel', 'Waypoint_4']),
                     ('program', raw['movej', 'Waypoint_5']), ('movej', 'v2'), ('movej', 'Waypoint_18'),
                     ('program', raw['movej', 'pick_rotate_1']), ('program', raw['movel', 'Waypoint_6']), ('movej', 'v3'),
                     ('movej', 'home')]


def _Margin_Lines(cfg):
    # The machine margin with its source as the report and the headers state it, typed here from the
    # configuration's Clearance.Environment and Clearance.Machine_Extra.
    return ['Machine margin: generated segments keep %.1f mm (Clearance.Environment %.1f + Machine_Extra %.1f mm) to %s'
            % ((cfg.Clearance_Environment + cfg.Machine_Extra_Margin) * 1e3, cfg.Clearance_Environment * 1e3,
               cfg.Machine_Extra_Margin * 1e3, cfg.Machine_Extra_Assembly),
            'outside the station zone; source: %s.' % cfg.Machine_Extra_Source]


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
def test_a_parallel_run_writes_the_committed_output(root, tmp_path, cfg):
    """
    Description:
        The full planning run with 4 spawned workers (S1 - S3 at once, the recorded seed and budget),
        started in a fresh interpreter from an unrelated directory, writes both output files byte for
        byte as committed in Data/Generated/ (written by the sequential run), reports the machine
        margin with its source and its wall time.
    """

    result = _Run(root, tmp_path, '--workers', '4', '--out', str(tmp_path / 'out'))
    assert result.returncode == 0, result.stdout + result.stderr
    assert '\n'.join('[INFO] ' + line for line in _Margin_Lines(cfg)) in result.stdout
    assert '[INFO] Wall time: planning' in result.stdout and '(4 workers)' in result.stdout
    for name in ('SF1_Rotation_17.urscript', 'SF1_Rotation_17_Sequence.script'):
        with open(os.path.join(root, 'Data', 'Generated', name), 'rb') as handle:
            committed = handle.read()
        assert (tmp_path / 'out' / name).read_bytes() == committed, name


def test_the_delivered_points_turn_only_the_picks_wrist_3(root, points, cfg):
    """
    Description:
        Configured_Points of the c1 job (owner, 2026-10-19, SPEC 10.5): wrist 3 of Waypoint_18, pick_rotate_1 and
        Waypoint_6 one turn lower (taught 4.739 rad), every other joint and every _p token the
        program's, every other point unchanged; the same arm pose (DH TCP equal to 1e-12 m and
        rad); the given points are not changed.
    """

    plan = _Plan_Module(root)
    job = _Job(plan, 'sf1_plc_v4_case_17_c1')
    before = {name: list(point.Q_Tokens) for name, point in job.Points.items()}
    delivered = plan.Configured_Points(job, cfg)
    turned = ('Waypoint_18', 'pick_rotate_1', 'Waypoint_6')
    for name, point in points.items():
        d = delivered[name]
        assert d.P_Tokens == point.P_Tokens
        if name not in turned:
            assert d.Q_Tokens == point.Q_Tokens
            continue
        assert d.Q_Tokens[:5] == point.Q_Tokens[:5] and point.Q[5] == pytest.approx(4.739, abs=0.001)
        assert d.Q[5] == pytest.approx(point.Q[5] - 2.0 * math.pi, abs=1e-11)
        a, b = TCP_Pose(d.Q, cfg.TCP_Offset), TCP_Pose(point.Q, cfg.TCP_Offset)
        assert np.max(np.abs(a - b)) < 1e-12
    assert {name: list(point.Q_Tokens) for name, point in job.Points.items()} == before


def test_the_delivered_headers_state_what_is_verbatim(root, points):
    """
    Description:
        The delivered points file (c2) names the learned points written verbatim from the extract
        (home) and those with the extract's _p and another _q (the drop's three on branch S- E- W-,
        the pick's three with wrist 3 one turn lower), and both claims hold token for token; it claims
        nothing else verbatim. The sequence file names the station lines whose points have another _q
        (the drop's movel and retreat, the pick and the lift).
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


def test_a_written_point_with_another_pose_is_refused(root, points, cfg):
    """
    Description:
        Provenance sorts the learned points written into verbatim and another _q, and refuses one
        whose _p is not the extract's (it would be another pose under a learned name).
    """

    plan = _Plan_Module(root)
    delivered = plan.Configured_Points(_Job(plan), cfg)
    assert plan.Provenance(['home', 'pick_rotate_1'], delivered, points) == (['home'], ['pick_rotate_1'])
    moved = dict(delivered, home=points['drop_rotate_1'])
    with pytest.raises(plan.Planner_Error_Cls, match='The point home to be written has another _p'):
        plan.Provenance(['home'], moved, points)


def test_the_delivered_headers_state_the_machine_margin_and_its_source(root, cfg):
    """
    Description:
        Both delivered files state, in their header, the margin generated segments keep to machine (a)
        outside the station zone (Clearance.Environment + Clearance.Machine_Extra.Margin, 30 mm) and
        the source of the extra margin, as configured.
    """

    for name in ('SF1_Rotation_17.urscript', 'SF1_Rotation_17_Sequence.script'):
        with open(os.path.join(root, 'Data', 'Generated', name), encoding='utf-8') as handle:
            text = handle.read()
        assert '\n'.join('# ' + line for line in _Margin_Lines(cfg)) + '\n' in text, name
    assert cfg.Clearance_Environment + cfg.Machine_Extra_Margin == pytest.approx(0.030)


def test_a_branch_point_is_the_ik_nearest_the_point_before_it(root, cfg):
    """
    Description:
        The default job's Branch configurations (owner, 2026-10-21, the chain rule) give the drop's
        three points exactly the joints of c2 in Data/Generated/variants/c2/ (made 2026-10-19 by the
        earlier Proposal_Points, before the rule was a job field), token for token, with the
        program's _p; the IK solution nearest each point's own taught _q would differ by one turn of
        wrist 2.
    """

    plan = _Plan_Module(root)
    job = _Job(plan)
    points = plan.Configured_Points(job, cfg)
    with open(os.path.join(root, 'Data', 'Generated', 'variants', 'c2', 'SF1_Rotation_17.urscript'), encoding='utf-8') as handle:
        c2 = Read_URScript(handle.read()).Points
    for name in ('drop_rotate_1', 'Waypoint_4', 'Waypoint_5', 'Waypoint_18', 'pick_rotate_1', 'Waypoint_6'):
        assert (points[name].P_Tokens, points[name].Q_Tokens) == (c2[name].P_Tokens, c2[name].Q_Tokens), name
    assert points['drop_rotate_1'].Q[4] - job.Points['drop_rotate_1'].Q[4] == pytest.approx(3.635, abs=0.001)


def _Body(path):
    # A file's lines but its header's status lines: those between the machine margin's second line
    # and the safety line (the job's Output.Status or the variant's status).
    with open(path, encoding='utf-8') as handle:
        lines = handle.read().splitlines()
    first = next(k for k, line in enumerate(lines) if line.startswith('# outside the station zone; source:'))
    last = next(k for k, line in enumerate(lines) if line.startswith('# Simulation passed != safe'))
    return lines[:first + 1] + lines[last:]


@pytest.mark.slow
@pytest.mark.parametrize('job', sorted(CONST_JOB_VARIANTS))
def test_each_case_17_job_reproduces_its_committed_variant(root, tmp_path, job):
    """
    Description:
        The c2 job (the default) and the c1 job, each planned in full with 4 workers from an
        unrelated directory, write their variant's points and motion lines byte for byte as
        committed in Data/Generated/variants/<c2 | c1>/ (owner, 2026-10-21); the headers differ only
        in their status lines.
    """

    result = _Run(root, tmp_path, '--job', job, '--workers', '4', '--out', str(tmp_path / 'out'))
    assert result.returncode == 0, result.stdout + result.stderr
    for name in ('SF1_Rotation_17.urscript', 'SF1_Rotation_17_Sequence.script'):
        assert _Body(str(tmp_path / 'out' / name)) == _Body(os.path.join(root, 'Data', 'Generated', 'variants', CONST_JOB_VARIANTS[job], name))


def test_the_taught_moves_of_case_17_pass_the_model(root, cfg):
    """
    Description:
        Replayed against the model under the taught rule, the 15 moves of case 17 home -> home (the
        program's joints, each in its gripper state) pass through no modelled geometry, keep their
        taught margins and stay below the workspace top (measured 2026-10-21: 8866 samples, no error,
        no warning): the fitted geometry is valid where case 17 ran.
    """

    plan = _Plan_Module(root)
    job = _Job(plan)
    scenes = {state: Scene_Cls(dataclasses.replace(cfg, Opening=plan.Opening(cfg, state, job))) for state in ('holding', 'open')}
    moves = plan.Taught_Moves(job, cfg)
    assert [(line.Kind, line.Target, state) for line, state, _ in moves][:4] == [
        ('movej', 'Waypoint_1', 'holding'), ('movej', 'drop_rotate_1', 'holding'), ('movel', 'Waypoint_4', 'holding'),
        ('movej', 'Waypoint_5', 'open')]
    errors, warnings, samples = plan.Check_Taught_Model(job, cfg, scenes)
    assert (errors, warnings) == ([], []) and samples > 0


def test_a_taught_line_inside_the_model_stops_the_run(root, cfg, tmp_path):
    """
    Description:
        With a test-only box of 60 mm around the TCP of Waypoint_2 (world frame, from the configured
        base pose and TCP), the taught move to Waypoint_2 (among others) passes into it: the run stops with exit 1
        before planning, each error naming the case, the program line of the move, the box and the
        depth, and saying nothing is refitted; nothing is written.
    """

    with open(os.path.join(root, 'Configuration', 'Cell_SF1.yaml'), encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    with open(os.path.join(root, 'Data', 'Programs', 'sf1_plc_v4', 'sf1_plc_v4_positions.urscript'), encoding='utf-8') as handle:
        q = Read_URScript(handle.read()).Points['Waypoint_2'].Q
    c, s = math.cos(cfg.Base_Yaw), math.sin(cfg.Base_Yaw)
    base = np.array([[c, -s, 0.0, cfg.Base_Position[0]], [s, c, 0.0, cfg.Base_Position[1]], [0.0, 0.0, 1.0, cfg.Base_Position[2]],
                     [0.0, 0.0, 0.0, 1.0]])
    tcp = (base @ TCP_Pose(q, cfg.TCP_Offset))[:3, 3]
    data['Environment']['Added_Boxes'].append({'Name': 'guard_test', 'Min': [float(v) - 0.03 for v in tcp],
                                               'Max': [float(v) + 0.03 for v in tcp], 'Source': 'test only'})
    config = tmp_path / 'cell.yaml'
    config.write_text(yaml.safe_dump(data), encoding='utf-8')
    with open(os.path.join(root, 'Data', 'Programs', 'sf1_plc_v4', 'sf1_plc_v4_cases_17_18.script'), encoding='utf-8') as handle:
        lines = Read_Program_Excerpt(handle.read())[17]
    first = next(line.Line for line in lines if line.Kind == 'movej' and line.Target == 'Waypoint_2' and line.Line > 3407)
    result = _Run(root, tmp_path, '--config', str(config), '--out', str(tmp_path / 'out'))
    assert result.returncode == 1, result.stdout + result.stderr
    errors = [line for line in result.stdout.splitlines() if line.startswith('[ERROR] The model is not valid for case 17 of sf1_plc_v4')]
    assert any('the taught movej to Waypoint_2 at program line %d' % first in line and 'mm into' in line for line in errors), errors
    assert all('guard_test' in line for line in errors)
    assert 'Nothing is planned, and nothing is refitted' in result.stdout
    assert not (tmp_path / 'out').exists()


def test_a_taught_move_above_the_workspace_top_is_a_warning_only(root, cfg):
    """
    Description:
        With Clearance.Workspace's top lowered to 2.0 m, below the 2.322 m case 17's taught sequence
        reaches (SPEC 8), the taught moves that rise above it are warnings naming the excursion, never
        errors: a top that is too low is conservative (a generated path may not use the space), not
        unsafe.
    """

    plan = _Plan_Module(root)
    job = _Job(plan)
    low = dataclasses.replace(cfg, Workspace_Max=[cfg.Workspace_Max[0], cfg.Workspace_Max[1], 2.0])
    scenes = {state: Scene_Cls(dataclasses.replace(low, Opening=plan.Opening(low, state, job))) for state in ('holding', 'open')}
    errors, warnings, _ = plan.Check_Taught_Model(job, low, scenes)
    assert errors == [] and warnings
    assert all('beyond Clearance.Workspace' in line for line in warnings)


# Changes to the default job that it cannot hold, each with the refusal's words: a segment end that
# is not a learned point, a segment crossing the release (program line 3425), a case no extract
# holds, and a configured point the output does not move to (load3: a loading move).
#   Unit [List<Tuple<string, Any, string>>]
CONST_BAD_JOBS = [('Segments', [{'Name': 'S1', 'From': 'home', 'To': 'nowhere'}, {'Name': 'S2', 'From': 'Waypoint_5', 'To': 'Waypoint_18'},
                                {'Name': 'S3', 'From': 'Waypoint_6', 'To': 'home'}], 'nowhere is not a learned point of the program'),
                  ('Segments', [{'Name': 'S1', 'From': 'drop_rotate_1', 'To': 'Waypoint_5'}, {'Name': 'S2', 'From': 'Waypoint_18', 'To': 'home'}],
                   'contains the gripper action at program line 3425'),
                  ('Case', 99, 'plans case 99, which is in no cases extract of Data/Programs/sf1_plc_v4 \\(its cases: 17, 18\\)'),
                  ('Point_Configurations', [{'Point': 'load3', 'Joint': 6, 'Turns': 1, 'Source': 'x'}],
                   'configures load3, which the output does not move to')]


@pytest.mark.parametrize('field, value, fragment', CONST_BAD_JOBS)
def test_a_job_the_program_cannot_hold_is_refused(root, tmp_path, field, value, fragment):
    """
    Description:
        A synthetic job (the default job with one field changed, as a file outside Configuration/Jobs/)
        is refused by plan_rotation.py --job <file> with exit 1 and a sentence naming the segment, the
        case or the point, before anything is planned or written.
    """

    with open(os.path.join(root, 'Configuration', 'Jobs', 'sf1_plc_v4_case_17.yaml'), encoding='utf-8') as handle:
        data = yaml.safe_load(handle)
    data[field] = value
    if field == 'Segments':
        data['Output']['Stations'] = ['drop', 'pick'][:len(value) - 1]
    job = tmp_path / 'synthetic.yaml'
    job.write_text(yaml.safe_dump(data), encoding='utf-8')
    result = _Run(root, tmp_path, '--job', str(job), '--out', str(tmp_path / 'out'))
    assert result.returncode == 1
    assert re.search(r'^\[ERROR\] .*' + fragment, result.stdout, re.M), result.stdout
    assert not (tmp_path / 'out').exists()


def test_the_case_18_job_is_refused_without_its_holding_opening(root, tmp_path):
    """
    Description:
        The case-18 job is shipped, but Tool.Holding_Openings has no entry for case 18 (its part's size
        is not known, SPEC 8): --job sf1_plc_v4_case_18 is refused with exit 1, naming the entry to
        add, and nothing is written; no opening is assumed.
    """

    result = _Run(root, tmp_path, '--job', 'sf1_plc_v4_case_18', '--out', str(tmp_path / 'out'))
    assert result.returncode == 1
    assert ('[ERROR] Tool.Holding_Openings has no entry for case 18 of sf1_plc_v4; add {Program: sf1_plc_v4, Case: 18, '
            'Opening: <m>, Source: <text>}') in result.stdout
    assert not (tmp_path / 'out').exists()


@pytest.mark.slow
def test_case_18_plans_without_code_changes(root, cfg, tmp_path):
    """
    Description:
        The case-18 job plans with the code that plans case 17 (owner, 2026-10-21): with an assumed
        holding opening of 50 mm (test only; no shipped entry) and a reduced budget (2 restarts), its
        taught moves pass the model check, S1 home -> drop_rotate_2, S2 Waypoint_5 -> Waypoint_11
        and S3 Waypoint_8 -> home are planned and verified (at most 5 via points each, every
        generated segment keeping the generated rule), and both files are written: case 18's header,
        its station lines verbatim from the program (typed from the extract), no point of case 17's
        drop or pick, and a start that does not claim a blend (case 18's line to home has none).
    """

    plan = _Plan_Module(root)
    job = _Job(plan, 'sf1_plc_v4_case_18')
    assumed = dataclasses.replace(cfg, Holding_Openings={**cfg.Holding_Openings, ('sf1_plc_v4', 18): (0.050, 'test assumption')})
    scenes = {state: Scene_Cls(dataclasses.replace(assumed, Opening=plan.Opening(assumed, state, job))) for state in ('holding', 'open')}
    assert plan.Check_Taught_Model(job, assumed, scenes)[0] == []
    budget = plan.Planner_Budget_Str(Restarts=2, RRT_Iterations=1000, Shortcut_Iterations=20, Refine_Candidates=1, Refine_Iterations=10)
    plan.Plan_Job(job, assumed, str(tmp_path / 'out'), plan.CONST_PENALTY_WEIGHT, 1, budget)
    points = Read_URScript((tmp_path / 'out' / 'SF1_Rotation_18.urscript').read_text(encoding='utf-8')).Points
    sequence = (tmp_path / 'out' / 'SF1_Rotation_18_Sequence.script').read_text(encoding='utf-8')
    assert sequence.startswith('# Case 18 of sf1_plc_v4.script, the rotation sequence home -> home, optimised')
    for line in ('movel(Waypoint_7_p, a=1.2, v=0.25)', 'movel(Waypoint_8_p, a=1.2, v=0.25)',
                 'movej(get_inverse_kin(Waypoint_5_p, qnear=Waypoint_5_q), a=1.3962634015954636, v=1.0471975511965976)',
                 'movej(get_inverse_kin(pick_rotate_2_p, qnear=pick_rotate_2_q), a=1.3962634015954636, v=1.0471975511965976)',
                 '# Start at home (generated, r=0), as the program\'s line (no blend)'):
        assert line + '\n' in sequence, line
    assert not {'drop_rotate_1', 'Waypoint_4', 'Waypoint_18', 'pick_rotate_1', 'Waypoint_6'} & set(points)
    ends = {'S1': ('home', 'drop_rotate_2'), 'S2': ('Waypoint_5', 'Waypoint_11'), 'S3': ('Waypoint_8', 'home')}
    station_q = [points[n].Q for n in ('drop_rotate_2', 'Waypoint_7', 'Waypoint_5', 'Waypoint_11', 'pick_rotate_2', 'Waypoint_8')]
    for segment, (start, goal) in ends.items():
        vias = sorted(n for n in points if n.startswith('via_%s_%s_' % (start, goal)))
        assert len(vias) <= 5
        path = [np.asarray(points[n].Q) for n in [start] + vias + [goal]]
        scene = scenes['open' if segment == 'S2' else 'holding']
        floors = scene.Station_Box_Floors(path[0], path[-1])
        for a, b in zip(path[:-1], path[1:]):
            assert scene.Check_Segment(a, b, kind='generated', station_q=station_q, box_floors=floors)[1] == [], segment
