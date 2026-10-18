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
# File Name: ../test/Planner/test_core.py
## =========================================================================== ##

# System (Default)
import os
import re
# Difflib (Sequence comparison)
import difflib
# Dataclasses (Data Classes)
import dataclasses
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# MuJoCo (Physics simulator) [pip3 install mujoco]
import mujoco
# Custom Lib.:
#   ../src/Planner/Core — the planner and the estimates under test
from Planner.Core import (Planner_Cls, Planner_Budget_Str, Planner_Error_Cls, Segment_Duration, Estimate_Duration,
                          Linear_Duration, Profile_Fraction, CONST_MOVEJ_A_TOKEN, CONST_MOVEJ_V_TOKEN)
#   ../src/Scene/Core — the scene at each opening
from Scene.Core import Scene_Cls
#   ../src/URScript/Core — the committed outputs and the program extracts
from URScript.Core import Read_URScript, Read_Program_Excerpt, Make_Generated_Point, Make_Document, Write_URScript
#   ../src/Kinematics/Core — the informational _p of a via point
from Kinematics.Core import TCP_Pose, Matrix_To_Pose

"""
Description:
    Initialization of constants.

    The committed outputs in Data/Generated/ are verified here independently of the planner; the
    full planning runs only in Example/plan_rotation.py. One reduced planning run is repeated to
    pin determinism.
"""
# The program's movej acceleration and speed, typed from sf1_plc_v4.script.
#   Unit [float] (rad/s^2, rad/s)
CONST_A, CONST_V = 1.3962634015954636, 1.0471975511965976
# The deliverable, case 17 home -> home: its station points and the three generated segments
# (owner, 2026-10-18).
#   Unit [Dict<string, string>]
CONST_CASE = {'drop': 'drop_rotate_1', 'below': 'Waypoint_4', 'retreat': 'Waypoint_5', 'approach': 'Waypoint_18',
              'pick': 'pick_rotate_1', 'above': 'Waypoint_6'}
# The gripper opening of each segment, typed here: S1 and S3 hold the 50 x 50 mm part (case 17's
# Tool.Holding_Openings, owner), S2 runs open at the full 85 mm stroke (owner, 2026-10-18, SPEC 4.9).
#   Unit [Dict<string, float>] (m)
CONST_OPENING = {'S1': 0.050, 'S2': 0.085, 'S3': 0.050}
CONST_STATION_POINTS = ('drop_rotate_1', 'Waypoint_4', 'Waypoint_5', 'Waypoint_18', 'pick_rotate_1', 'Waypoint_6')
# Via points the taught program uses per segment.
#   Unit [Dict<string, int>]
CONST_TAUGHT_VIAS = {'S1': 1, 'S2': 3, 'S3': 1}
# The reduced planning run repeated for determinism: S1 of case 17 (its direct segment fails; S3's
# no longer does since the machine shift, SPEC 4.11).
#   Unit [Planner_Budget_Str], [int]
CONST_REDUCED = Planner_Budget_Str(Restarts=2, RRT_Iterations=600, Shortcut_Iterations=40, Refine_Candidates=1,
                                   Refine_Iterations=20)
CONST_REDUCED_SEED = 7
# The station boxes (Environment.Added_Boxes, owner photo: the holder under the part), and the cap
# of their distance search: far beyond every margin, so that a generated sample is compared with
# its taught end exactly unless both are farther than the cap.
#   Unit [Tuple<string>], [float] (m)
CONST_STATION_BOXES, CONST_BOX_DISTMAX = ('station_bracket',), 0.200


def _Segments():
    c = CONST_CASE
    return {'S1': ('home', c['drop']), 'S2': (c['retreat'], c['approach']), 'S3': (c['above'], 'home')}


@pytest.fixture(scope='module')
def program(root):
    """
    Description:
        The program's points and the case lines (Data/Programs/sf1_plc_v4/).
    """

    folder = os.path.join(root, 'Data', 'Programs', 'sf1_plc_v4')
    with open(os.path.join(folder, 'sf1_plc_v4_positions.urscript'), encoding='utf-8') as handle:
        points = Read_URScript(handle.read()).Points
    with open(os.path.join(folder, 'sf1_plc_v4_cases_17_18.script'), encoding='utf-8') as handle:
        lines = Read_Program_Excerpt(handle.read())
    return points, lines


@pytest.fixture(scope='module')
def scenes(cfg):
    """
    Description:
        The scene with the environment at each segment opening. The configured holding opening must
        be the one typed here.
    """

    assert cfg.Holding_Openings[17][0] == CONST_OPENING['S1']
    return {o: Scene_Cls(dataclasses.replace(cfg, Opening=o)) for o in sorted(set(CONST_OPENING.values()))}


@pytest.fixture(scope='module')
def planners(program, scenes):
    """
    Description:
        A planner per segment, with the fixed station points, penalty weight 0.
    """

    points, _ = program
    station_q = [points[name].Q for name in CONST_STATION_POINTS]
    return {segment: Planner_Cls(scenes[o], station_q, 0.0, CONST_REDUCED) for segment, o in CONST_OPENING.items()}


def _Output(root):
    folder = os.path.join(root, 'Data', 'Generated')
    with open(os.path.join(folder, 'SF1_Rotation_17.urscript'), encoding='utf-8') as handle:
        document = Read_URScript(handle.read())
    with open(os.path.join(folder, 'SF1_Rotation_17_Sequence.script'), encoding='utf-8') as handle:
        sequence = handle.read()
    return document, sequence


def _Path(document, points, start, goal):
    prefix = 'via_%s_%s_' % (start, goal)
    vias = sorted(name for name in document.Points if name.startswith(prefix) and re.fullmatch(r'\d{2}', name[len(prefix):]))
    return [np.array(points[start].Q)] + [np.array(document.Points[name].Q) for name in vias] + [np.array(points[goal].Q)], vias


def test_the_duration_estimates_match_hand_computed_values():
    """
    Description:
        Trapezoid on the leading joint (a = 1.3963 rad/s^2, v = 1.0472 rad/s; v^2/a = 0.7854 rad):
        0.5 rad -> triangle 2 sqrt(0.5 / a) = 1.19683 s; 2.0 rad -> 2.0 / v + v / a = 1.909859 + 0.75 s; the
        leading joint decides, a path sums its segments. movel on the TCP path (1.2 m/s^2, 0.25 m/s):
        0.06 m -> 0.44833 s, 0.04 m -> triangle 0.36515 s.
    """

    assert Segment_Duration([0.0] * 6, [0.5, 0.1, 0, 0, 0, 0], CONST_A, CONST_V) == pytest.approx(1.19683, abs=1e-5)
    assert Segment_Duration([0.0] * 6, [0.3, -2.0, 0, 0, 0, 1.0], CONST_A, CONST_V) == pytest.approx(2.659859, abs=1e-6)
    assert Estimate_Duration([[0.0] * 6, [0.5, 0, 0, 0, 0, 0], [0.5, 0, 0, 0, 0, 2.5]], CONST_A, CONST_V) == pytest.approx(
        1.19683 + 2.5 / CONST_V + CONST_V / CONST_A, abs=1e-5)
    assert Linear_Duration([0.0] * 6, [0.06, 0, 0, 1, 1, 1], 1.2, 0.25) == pytest.approx(0.44833, abs=1e-5)
    assert Linear_Duration([0.0] * 6, [0.0, 0.04, 0, 0, 0, 0], 1.2, 0.25) == pytest.approx(0.36515, abs=1e-5)


def test_the_committed_paths_pass_the_independent_verification(root, program, planners):
    """
    Description:
        Every generated segment of the committed output, rebuilt from its via points, passes
        Verify_Path (the full generated-rule check, no early exit, at most 5 via points, within the
        joint limits); S2 needs fewer via points than the taught path, S1 and S3 no more.
    """

    points, _ = program
    document, _ = _Output(root)
    for segment, (start, goal) in _Segments().items():
        path, vias = _Path(document, points, start, goal)
        planners[segment].Verify_Path(path)
        assert len(vias) <= CONST_TAUGHT_VIAS[segment] - (1 if segment == 'S2' else 0)


def _Box_Distance(scene, robot, box, q):
    m, d = scene.Model, scene.Data
    scene.Set_Joints(q)
    distance = CONST_BOX_DISTMAX
    for g in robot:
        # Bounding spheres farther apart than the cap cannot come closer.
        if np.linalg.norm(d.geom_xpos[g] - d.geom_xpos[box]) - m.geom_rbound[g] - m.geom_rbound[box] > CONST_BOX_DISTMAX:
            continue
        distance = min(distance, mujoco.mj_geomDistance(m, d, g, box, CONST_BOX_DISTMAX, None))
    return distance


def test_no_generated_sample_comes_closer_to_a_station_box_than_its_taught_end(root, cfg, program, scenes):
    """
    Description:
        The station boxes (Environment.Added_Boxes, not in the STL; fitted, not measured) are in the
        scenes the paths are verified in. For every committed generated segment and every station
        box, measured exactly (mj_geomDistance): within the global margin, no sample comes closer to
        the box than the closer of the segment's two taught ends, so a generated path never uses the
        boxes' space more tightly than the taught program does (farther away, where the rule does
        not look, a fraction of a millimetre either way means nothing); and every sample keeps the
        generated rule's margin to the box (the zone margin within the endpoint radius of a station
        point, the global margin elsewhere).
    """

    points, _ = program
    document, _ = _Output(root)
    station_q = [points[name].Q for name in CONST_STATION_POINTS]
    for segment, (start, goal) in _Segments().items():
        scene = scenes[CONST_OPENING[segment]]
        m = scene.Model
        robot = [g for g in range(m.ngeom) if m.geom_contype[g] and m.geom_bodyid[g] != 0
                 and not m.body(m.geom_bodyid[g]).name.startswith('env') and m.body(m.geom_bodyid[g]).name != 'base']
        tcps = scene.Station_TCPs(station_q)
        path, _ = _Path(document, points, start, goal)
        for name in CONST_STATION_BOXES:
            box = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'env_box_' + name)
            assert box >= 0 and name in scene.Zone_Boxes
            taught = min(_Box_Distance(scene, robot, box, path[0]), _Box_Distance(scene, robot, box, path[-1]))
            for leg, (q_a, q_b) in enumerate(zip(path[:-1], path[1:])):
                for k, q in enumerate(scene.Segment_Samples(q_a, q_b)):
                    distance = _Box_Distance(scene, robot, box, q)
                    margin = cfg.Station_Zone_Environment if scene.Near_Station(q, tcps) else cfg.Clearance_Environment
                    assert distance >= margin, (segment, name, leg, k, distance, margin)
                    assert min(distance, cfg.Clearance_Environment) >= min(taught, cfg.Clearance_Environment) - 1e-9, (
                        segment, name, leg, k, distance, taught)


def test_the_station_points_keep_their_tokens(root, program):
    """
    Description:
        The output holds home and the station points, each the program's point token for token, and
        nothing of the loading moves; every other point is a generated via point.
    """

    points, lines = program
    document, _ = _Output(root)
    fixed = {'home'} | set(CONST_CASE.values())
    for name, point in document.Points.items():
        if name in fixed:
            assert (point.P_Tokens, point.Q_Tokens) == (points[name].P_Tokens, points[name].Q_Tokens)
        else:
            assert name.startswith('via_')
    assert fixed <= set(document.Points)


def test_the_sequence_file_has_the_program_structure(root, program):
    """
    Description:
        The sequence file is case 17 home -> home. Its motion lines are, in order: the start at home
        (generated with r=0: the program blends into S1); S1's via points and the drop as generated
        movej (joint targets, the program's a and v, r=0); the drop's movel, the retreat and the pick
        verbatim; S2 and S3 as generated movej ending at home. Nothing of the loading moves. The
        gripper actions appear as comments; the header says simulation is not safety.
    """

    points, lines = program
    document, sequence = _Output(root)
    c = CONST_CASE
    def Generated_Lines(names):
        return ['movej(%s_q, a=%s, v=%s, r=0)' % (n, CONST_MOVEJ_A_TOKEN, CONST_MOVEJ_V_TOKEN) for n in names]
    def Verbatim_Lines(kind, target):
        return [[line.Raw.strip() for line in lines[17] if line.Kind == kind and line.Target == target][0]]
    vias = {s: _Path(document, points, a, b)[1] for s, (a, b) in _Segments().items()}
    expected = (Generated_Lines(['home'] + vias['S1'] + [c['drop']]) + Verbatim_Lines('movel', c['below'])
                + Verbatim_Lines('movej', c['retreat']) + Generated_Lines(vias['S2'] + [c['approach']])
                + Verbatim_Lines('movej', c['pick']) + Verbatim_Lines('movel', c['above']) + Generated_Lines(vias['S3'] + ['home']))
    motion = [line for line in sequence.splitlines() if line and not line.startswith('#')]
    assert motion == expected
    comments = [line for line in sequence.splitlines() if line.startswith('# Gripper')]
    assert [line.split(':')[0] for line in comments] == ['# Gripper open', '# Gripper close']
    assert 'Simulation passed != safe on the robot' in sequence


def test_the_optimised_case_is_the_taught_case_with_s1_to_s3_replaced(root):
    """
    Description:
        The optimised home -> home (Data/Generated/) and the simplified taught program
        (Data/Programs/sf1_plc_v4/) differ only in S1 - S3: aligned line by line, every taught line
        that is not kept is a move of S1 - S3 (to Waypoint_1/2/3, the drop, the approach or the
        closing home) or the blended start at home; every optimised line that is not taught is a
        generated movej (joint target, r=0). The gripper comments are identical, so the two files
        can be compared and pasted side by side.
    """

    c = CONST_CASE
    with open(os.path.join(root, 'Data', 'Programs', 'sf1_plc_v4', 'sf1_plc_v4_case_17_sequence.script'), encoding='utf-8') as handle:
        taught = [line for line in handle.read().splitlines() if not line.startswith('#') or line.startswith('# Gripper')]
    optimised = [line for line in _Output(root)[1].splitlines() if not line.startswith('#') or line.startswith('# Gripper')]
    generated = re.compile(r'^movej\(\w+_q, a=%s, v=%s, r=0\)$' % (re.escape(CONST_MOVEJ_A_TOKEN), re.escape(CONST_MOVEJ_V_TOKEN)))
    replaced_targets = {'Waypoint_1', 'Waypoint_2', 'Waypoint_3', c['drop'], c['approach'], 'home'}
    removed, added = [], []
    for tag, i1, i2, j1, j2 in difflib.SequenceMatcher(None, taught, optimised, autojunk=False).get_opcodes():
        if tag != 'equal':
            removed += taught[i1:i2]
            added += optimised[j1:j2]
    targets = [re.match(r'^movej\(get_inverse_kin\((\w+)_p, qnear=\1_q\)', line).group(1) for line in removed]
    assert set(targets) <= replaced_targets, targets
    assert targets.count('home') == 2
    assert all(generated.match(line) for line in added), added
    assert [line for line in taught if line.startswith('# Gripper')] == [line for line in optimised if line.startswith('# Gripper')]


def test_a_reduced_planning_run_is_byte_identical_twice(cfg, program):
    """
    Description:
        The same segment planned twice with the same seed and iteration budget (fresh planners, so
        nothing is shared) gives the same path, byte for byte, and the same written points.
    """

    points, _ = program
    station_q = [points[name].Q for name in CONST_STATION_POINTS]
    scene = Scene_Cls(dataclasses.replace(cfg, Opening=CONST_OPENING['S1']))
    texts = []
    for _ in range(2):
        result = Planner_Cls(scene, station_q, 0.0, CONST_REDUCED).Plan(points['home'].Q, points['drop_rotate_1'].Q, CONST_REDUCED_SEED)
        assert result.Origin.startswith('rrt')
        vias = [Make_Generated_Point('via_%02d' % k, Matrix_To_Pose(TCP_Pose(q, cfg.TCP_Offset)), list(q))
                for k, q in enumerate(result.Path[1:-1], start=1)]
        texts.append(Write_URScript(Make_Document(vias)))
    assert texts[0] == texts[1]


@pytest.mark.parametrize('kind', ['inflated margin', 'no iterations', 'six via points'])
def test_an_impossible_plan_raises(cfg, program, kind):
    """
    Description:
        The planner never returns a colliding or too long path: an inflated environment margin makes
        the start break the rule, a budget of one RRT iteration finds nothing, and a path with six
        via points is refused by the verification - each with Planner_Error_Cls naming the reason.
    """

    points, _ = program
    station_q = [points[name].Q for name in CONST_STATION_POINTS]
    start, goal = points['Waypoint_5'].Q, points['Waypoint_18'].Q
    if kind == 'inflated margin':
        scene = Scene_Cls(dataclasses.replace(cfg, Opening=0.085, Clearance_Environment=0.150))
        with pytest.raises(Planner_Error_Cls, match='breaks the generated rule'):
            Planner_Cls(scene, station_q, 0.0, CONST_REDUCED).Plan(start, goal, CONST_REDUCED_SEED)
        return
    scene = Scene_Cls(dataclasses.replace(cfg, Opening=0.085))
    if kind == 'no iterations':
        budget = dataclasses.replace(CONST_REDUCED, Restarts=1, RRT_Iterations=1)
        with pytest.raises(Planner_Error_Cls, match='found no path'):
            Planner_Cls(scene, station_q, 0.0, budget).Plan(start, goal, CONST_REDUCED_SEED)
        return
    path = [np.asarray(start) + (np.asarray(goal) - np.asarray(start)) * t for t in np.linspace(0.0, 1.0, 8)]
    with pytest.raises(Planner_Error_Cls, match='at most 5'):
        Planner_Cls(scene, station_q, 0.0, CONST_REDUCED).Verify_Path(path)


def test_the_penalty_counts_only_far_from_the_station(planners, program):
    """
    Description:
        The clearance penalty sums max(0, 30 mm - d) over samples outside the station
        neighbourhood. The grasp approach hover_obj_1 -> pick_obj_1 (case 1, far from the station)
        reaches 2.8 mm into a fixture at the open 85 mm (SPEC 4.9), so it is at least 32.7 mm. The
        open approach to the pick, Waypoint_18 -> pick_rotate_1, comes within 5.5 mm of the holder
        (SPEC 4.11), but inside the neighbourhood, so it is 0.
    """

    points, _ = program
    far = planners['S2'].Segment_Penalty(np.array(points['hover_obj_1'].Q), np.array(points['pick_obj_1'].Q))
    near = planners['S2'].Segment_Penalty(np.array(points['Waypoint_18'].Q), np.array(points['pick_rotate_1'].Q))
    assert far >= 0.030 + 0.0027
    assert near == 0.0


def test_rrt_connect_is_reproducible_with_both_sample_sources(planners, program):
    """
    Description:
        RRT-Connect alone, on S2 of case 17 with generator [8, 0]: that run draws 23 samples from the
        box around both ends and 25 from the whole joint range (measured 2026-10-17), so both sample
        sources take part. Two runs with the same seed give the same tree path, byte for byte. (The
        reduced planning run of the test above connects after one sample from the whole range, so it
        cannot see the box sampler.)
    """

    points, _ = program
    start, goal = np.array(points['Waypoint_5'].Q), np.array(points['Waypoint_18'].Q)
    paths = [planners['S2'].RRT_Connect(start, goal, np.random.default_rng([8, 0])) for _ in range(2)]
    assert paths[0] is not None and len(paths[0]) > 2
    assert np.array(paths[0]).tobytes() == np.array(paths[1]).tobytes()


@pytest.mark.parametrize('distance, a, v', [(0.5, CONST_A, CONST_V), (2.0, CONST_A, CONST_V), (0.06, 1.2, 0.25), (0.04, 1.2, 0.25)])
def test_the_profile_ends_with_the_duration_estimate(distance, a, v):
    """
    Description:
        Profile_Fraction, the position profile the replay plays, is 0 at the start, 1 exactly at the
        estimated duration of the same move (Segment_Duration on the leading joint, Linear_Duration
        on the TCP path; triangle and trapezoid cases), below 1 just before, 0.5 at half the time
        (the profile is symmetric), a t^2 / 2 of the distance early in the acceleration, and never
        decreasing.
    """

    duration = (Segment_Duration([0.0] * 6, [distance, 0, 0, 0, 0, 0], a, v) if a == CONST_A
                else Linear_Duration([0.0] * 6, [distance, 0, 0, 0, 0, 0], a, v))
    assert Profile_Fraction(0.0, distance, a, v) == 0.0
    assert Profile_Fraction(duration, distance, a, v) == pytest.approx(1.0, abs=1e-12)
    assert Profile_Fraction(duration * (1.0 - 1e-3), distance, a, v) < 1.0
    assert Profile_Fraction(duration / 2.0, distance, a, v) == pytest.approx(0.5, abs=1e-12)
    # The acceleration phase from rest: s = a t^2 / 2 (before the peak speed, in both shapes).
    t = 0.1 * min(v / a, duration / 2.0)
    assert Profile_Fraction(t, distance, a, v) == pytest.approx(0.5 * a * t * t / distance, rel=1e-9)
    samples = [Profile_Fraction(duration * k / 200.0, distance, a, v) for k in range(201)]
    assert all(b >= a_ for a_, b in zip(samples[:-1], samples[1:]))
