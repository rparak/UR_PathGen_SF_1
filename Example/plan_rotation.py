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
# File Name: ../Example/plan_rotation.py
## =========================================================================== ##

# System (Default)
import os
import sys
#   Add access if it is not in the system path.
CONST_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src')
if CONST_SRC not in sys.path:
    sys.path.append(CONST_SRC)
# Argparse (Command-line parsing)
import argparse
# Re (Regular expressions)
import re
# Dataclasses (Data Classes)
import dataclasses
# Time (Time access)
import time
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.:
#   ../Configuration/Core — the cell configuration
from Configuration.Core import (Load_Configuration, Cell_Configuration_Str, Configuration_Error_Cls, CONST_DEFAULT_CONFIG_FILE,
                                CONST_PROJECT_ROOT)
#   ../URScript/Core — the program extracts and the output files
from URScript.Core import (Read_URScript, Read_Program_Excerpt, Make_Generated_Point, Make_Document, Write_URScript,
                           Write_Motion_Sequence, Program_Entries, Gripper_Comment, Program_Line_Str)
#   ../Kinematics/Core — the informational _p of a via point
from Kinematics.Core import TCP_Pose, Matrix_To_Pose
#   ../Scene/Core — the scene at each gripper opening
from Scene.Core import Scene_Cls
#   ../Planner/Core — the planner and the estimates
from Planner.Core import (Planner_Cls, Planner_Budget_Str, Planner_Error_Cls, Estimate_Duration, Linear_Duration,
                          Path_Length, CONST_MOVEJ_A_TOKEN, CONST_MOVEJ_V_TOKEN, CONST_PENALTY_DISTANCE)

"""
Description:
    Initialization of constants.
"""
# The owner's program extracts (SPEC 10.1) and the default output directory (SPEC 10.6).
#   Unit [string]
CONST_PROGRAM_DIR = os.path.join(CONST_PROJECT_ROOT, 'Data', 'Programs', 'sf1_plc_v4')
CONST_POSITIONS = os.path.join(CONST_PROGRAM_DIR, 'sf1_plc_v4_positions.urscript')
CONST_CASES_FILE = os.path.join(CONST_PROGRAM_DIR, 'sf1_plc_v4_cases_17_18.script')
CONST_OUTPUT_DIR = os.path.join(CONST_PROJECT_ROOT, 'Data', 'Generated')
# The rotation sequence of each case: the station points (fixed, taught) and the taught via points
# of the three generated segments S1 home -> drop, S2 retreat -> approach, S3 lift -> home (owner).
#   Unit [Dict<int, Dict<string, string>>]
CONST_CASES = {
    17: {'home': 'home', 'drop': 'drop_rotate_1', 'below': 'Waypoint_4', 'retreat': 'Waypoint_5',
         'approach': 'Waypoint_18', 'pick': 'pick_rotate_1', 'above': 'Waypoint_6'},
    18: {'home': 'home', 'drop': 'drop_rotate_2', 'below': 'Waypoint_7', 'retreat': 'Waypoint_5',
         'approach': 'Waypoint_11', 'pick': 'pick_rotate_2', 'above': 'Waypoint_8'},
}
CONST_TAUGHT_VIAS = {'S1': ['Waypoint_1'], 'S2': ['Waypoint_1', 'Waypoint_2', 'Waypoint_3'], 'S3': ['Waypoint_2']}
# The fixed station points of both cases (the endpoint neighbourhood of the generated rule).
#   Unit [Tuple<string>]
CONST_STATION_POINTS = ('drop_rotate_1', 'drop_rotate_2', 'Waypoint_4', 'Waypoint_7', 'Waypoint_5', 'Waypoint_18',
                        'Waypoint_11', 'pick_rotate_1', 'pick_rotate_2', 'Waypoint_6', 'Waypoint_8')
# Gripper opening after rq_set_pos_spd_for(0, ...): the full 85 mm stroke; the release opening of
# 85 mm is real, the part is released into a magazine with free space (owner, 2026-10-18, SPEC 4.9).
# While the part is held, the case's Tool.Holding_Openings applies (Segment_Opening).
#   Unit [float] (m)
CONST_RELEASE_OPENING = 0.085
# The gripper state of each generated segment: S1 and S3 carry the part, S2 runs open.
#   Unit [Dict<string, string>]
CONST_SEGMENT_STATE = {'S1': 'holding', 'S2': 'open', 'S3': 'holding'}
# The recorded seed of every planning run; segment k (0 = S1) of a case plans with
# seed * 1000 + case * 10 + k.
#   Unit [int]
CONST_SEED = 20261017
# Weight of the clearance penalty in the objective (SPEC 10.6), chosen by the owner (2026-10-17):
# against w = 0 it raised the smallest clearance outside the station
# neighbourhood from 20.4 - 28.9 mm to 26.6 - 34.5 mm for +0.09 / -0.02 s home -> home (cases 17 / 18).
#   Unit [float] (s/m)
CONST_PENALTY_WEIGHT = 0.5
# The program's movel acceleration and speed (sf1_plc_v4.script), for the station operations'
# duration estimate only; their lines are copied verbatim.
#   Unit [float] (m/s^2, m/s)
CONST_MOVEL_A, CONST_MOVEL_V = 1.2, 0.25
# The STL component whose hull fills the interior of machine (a): the set's only fallback piece, an
# open shell with 235 mm excess; the loading moves of the program penetrate it by up to 145 mm, so
# the model is not valid there (KNOWN_ISSUES, SPEC 4.9). A configuration within the environment
# margin of it is labelled CONST_LABEL_INVALID, never as a collision.
#   Unit [Tuple<int>]
CONST_MACHINE_INTERIOR = (1638,)
# The clearance labels of a configuration or a segment shown in a viewer.
#   Unit [string]
CONST_LABEL_CLEAR = 'clearance kept (%s rule)'
CONST_LABEL_VIOLATED = 'CLEARANCE VIOLATED (%s rule)'
CONST_LABEL_INVALID = 'model not valid here (machine (a) interior, KNOWN_ISSUES)'
# The blend radius token of a program movej (absent = no blend).
#   Unit [Pattern]
CONST_BLEND = re.compile(r', r=([^,()]+)\)\s*$')
# Cap of the reported robot - environment distance.
#   Unit [float] (m)
CONST_REPORT_DISTMAX = 0.200


def Segments(case: int) -> tp.Dict[str, tp.Tuple[str, str]]:
    """
    Description:
        The ends of the three generated segments of a case.

    Args:
        (1) case [int]: 17 or 18.

    Returns:
        (1) parameter [Dict<string, Tuple<string, string>>]: S1, S2, S3 -> (from, to).
    """

    c = CONST_CASES[case]
    return {'S1': (c['home'], c['drop']), 'S2': (c['retreat'], c['approach']), 'S3': (c['above'], c['home'])}


def Opening(cfg: Cell_Configuration_Str, case: int, state: str) -> float:
    """
    Description:
        The gripper opening of a case in a gripper state: the case's Tool.Holding_Openings while the
        part is held, CONST_RELEASE_OPENING when open.

    Args:
        (1) cfg [Cell_Configuration_Str(object)]: The cell configuration.
        (2) case [int]: The program case.
        (3) state [string]: 'holding' or 'open'.

    Returns:
        (1) parameter [float]: The inner pad-face gap (m).
    """

    if state == 'open':
        return CONST_RELEASE_OPENING
    if case not in cfg.Holding_Openings:
        raise Configuration_Error_Cls('Tool.Holding_Openings has no entry for case %d; add {Case: %d, Opening: <m>, '
                                      'Source: <text>} to the configuration.' % (case, case))
    return cfg.Holding_Openings[case][0]


def Configuration_Label(scene: Scene_Cls, cfg: Cell_Configuration_Str, q: tp.Sequence[float], kind: str,
                        station_tcps: tp.Optional[np.ndarray] = None) -> tp.Tuple[str, tp.List, tp.List[tp.Tuple[int, float]]]:
    """
    Description:
        The clearance label of one configuration under the rule of its kind: CONST_LABEL_INVALID when
        a robot geom comes within Clearance.Environment of the machine (a) interior (the model is not
        valid there, whatever the check says), else clear or violated.

    Args:
        (1) scene [Scene_Cls(object)]: The scene with the environment.
        (2) cfg [Cell_Configuration_Str(object)]: The cell configuration (Clearance.Environment).
        (3) q [Vector<float> 1x6]: Joint angles (rad).
        (4) kind [string]: 'taught' or 'generated'.
        (5) station_tcps [Matrix<float> Nx3]: The station points' TCPs (generated rule only).

    Returns:
        (1) parameter [Tuple<string, List<Violation_Str>, List<Tuple<int, float>>>]: The label, the
                                                                                   violations and the
                                                                                   machine-interior contacts.
    """

    violations = scene.Check_Configuration(q, kind, station_tcps)
    interior = scene.Components_Within(q, CONST_MACHINE_INTERIOR, cfg.Clearance_Environment)
    if interior:
        return CONST_LABEL_INVALID, violations, interior
    return (CONST_LABEL_VIOLATED if violations else CONST_LABEL_CLEAR) % kind, violations, interior


def Measure(scene: Scene_Cls, path: tp.Sequence[np.ndarray], station_q: tp.Sequence[tp.Sequence[float]]) -> tp.Dict:
    """
    Description:
        The report measures of a path: via points, estimated duration, joint-path lengths, the
        smallest robot - environment distance with its robot body (everywhere, and outside the
        station neighbourhood), the clearance penalty and the generated-rule violations.

    Args:
        (1) scene [Scene_Cls(object)]: The scene at the path's opening.
        (2) path [List<Vector<float> 1x6>]: The configurations, ends included (rad).
        (3) station_q [List<Vector<float> 1x6>]: The fixed station points (rad).

    Returns:
        (1) parameter [Dict]: The measures.
    """

    a, v = float(CONST_MOVEJ_A_TOKEN), float(CONST_MOVEJ_V_TOKEN)
    total, max_joint = Path_Length(path)
    tcps = scene.Station_TCPs(station_q)
    closest, body, far, far_body, penalty, violations = CONST_REPORT_DISTMAX, '', CONST_REPORT_DISTMAX, '', 0.0, 0
    for q_a, q_b in zip(path[:-1], path[1:]):
        violations += len(scene.Check_Segment(q_a, q_b, kind='generated', station_q=station_q)[1])
        for q in scene.Segment_Samples(q_a, q_b):
            distance, robot = scene.Environment_Distance(q, CONST_REPORT_DISTMAX)
            if distance < closest:
                closest, body = distance, robot
            if not scene.Near_Station(q, tcps):
                penalty += max(0.0, CONST_PENALTY_DISTANCE - distance)
                if distance < far:
                    far, far_body = distance, robot
    return {'vias': len(path) - 2, 'duration': Estimate_Duration(path, a, v), 'sum': total, 'max': max_joint,
            'closest': closest, 'body': body, 'far': far, 'far_body': far_body, 'penalty': penalty,
            'violations': violations}


def Station_Duration(points: tp.Dict, c: tp.Dict[str, str]) -> float:
    """
    Description:
        Estimated duration of the station operations (unchanged): the drop's movel, the retreat
        movej, the pick movej and the lift movel; gripper time excluded.

    Args:
        (1) points [Dict<string, Robot_Point_Str>]: The program's points.
        (2) c [Dict<string, string>]: The case's point names.

    Returns:
        (1) parameter [float]: The duration (s).
    """

    a, v = float(CONST_MOVEJ_A_TOKEN), float(CONST_MOVEJ_V_TOKEN)
    return (Linear_Duration(points[c['drop']].P, points[c['below']].P, CONST_MOVEL_A, CONST_MOVEL_V)
            + Estimate_Duration([points[c['below']].Q, points[c['retreat']].Q], a, v)
            + Estimate_Duration([points[c['approach']].Q, points[c['pick']].Q], a, v)
            + Linear_Duration(points[c['pick']].P, points[c['above']].P, CONST_MOVEL_A, CONST_MOVEL_V))


def Program_Item(lines: tp.Sequence[Program_Line_Str], kind: str, target: str) -> Program_Line_Str:
    """
    Description:
        The one program line of a case with the given kind and target; refuses none or several.

    Args:
        (1) lines [List<Program_Line_Str>]: The case's program lines.
        (2 - 3) kind, target [string]: 'movej' / 'movel' / 'gripper' and the target.

    Returns:
        (1) parameter [Program_Line_Str(object)]: The line.
    """

    found = [line for line in lines if line.Kind == kind and line.Target == target]
    if len(found) != 1:
        raise Planner_Error_Cls('The program excerpt has %d %s lines to %s; exactly one is expected.' % (len(found), kind, target))
    return found[0]


def Program_Line(lines: tp.Sequence[Program_Line_Str], kind: str, target: str) -> str:
    """
    Description:
        The one program line of a case with the given kind and target; refuses none or several.

    Args:
        (1) lines [List<Program_Line_Str>]: The case's program lines.
        (2 - 3) kind, target [string]: 'movej' / 'movel' / 'gripper' and the target.

    Returns:
        (1) parameter [string]: The line as in the program.
    """

    return Program_Item(lines, kind, target).Raw


def Rotation_Bounds(lines: tp.Sequence[Program_Line_Str], home: str) -> tp.Tuple[int, int]:
    """
    Description:
        The rotation sequence inside a whole case: the indices of its first and its last movej to
        home (home -> ... -> home); refuses a case that does not move to home exactly twice.

    Args:
        (1) lines [List<Program_Line_Str>]: The case's program lines.
        (2) home [string]: The home point's name.

    Returns:
        (1) parameter [Tuple<int, int>]: The indices of the two movej to home.
    """

    homes = [k for k, line in enumerate(lines) if line.Kind == 'movej' and line.Target == home]
    if len(homes) != 2:
        raise Planner_Error_Cls('The case moves to %s %d times; the rotation sequence needs exactly two (home -> ... -> home).'
                                % (home, len(homes)))
    return homes[0], homes[1]


def Whole_Case_Entries(lines: tp.Sequence[Program_Line_Str], c: tp.Dict[str, str],
                       vias: tp.Dict[str, tp.List[str]]) -> tp.List[tp.Tuple[str, str]]:
    """
    Description:
        The optimised whole case as motion-sequence entries: the program's lines before and after
        the rotation sequence verbatim (Program_Entries), the station operations verbatim, S1 - S3
        as generated movej to their via points and ends. The arrival at home that opens the
        rotation sequence stays the program's line unless that line blends (r > 0): a blend into a
        generated segment is not checked (SPEC 8.5), so the arrival is then a generated movej with
        r=0 (owner, 2026-10-18).

    Args:
        (1) lines [List<Program_Line_Str>]: The case's program lines.
        (2) c [Dict<string, string>]: The case's point names (CONST_CASES).
        (3) vias [Dict<string, List<string>>]: The via point names of S1, S2 and S3.

    Returns:
        (1) parameter [List<Tuple<string, string>>]: The entries in order.
    """

    first, last = Rotation_Bounds(lines, c['home'])
    blend = CONST_BLEND.search(lines[first].Raw)
    if blend is not None and float(blend.group(1)) > 0.0:
        arrival = [('comment', 'Arrival at %s (generated, r=0): the program\'s line blends (r=%s) into S1, and a blend into a '
                               'generated segment is not checked' % (c['home'], blend.group(1))), ('movej', c['home'])]
    else:
        arrival = [('comment', 'Arrival at %s (verbatim from the program)' % c['home']), ('program', lines[first].Raw)]
    return ([('comment', 'Before the rotation sequence (verbatim from the program)')] + Program_Entries(lines[:first]) + arrival
            + [('comment', 'S1 (generated): %s -> %s' % (c['home'], c['drop']))]
            + [('movej', name) for name in vias['S1']] + [('movej', c['drop'])]
            + [('comment', 'Station: drop (verbatim from the program)'),
               ('program', Program_Line(lines, 'movel', c['below'])),
               ('comment', Gripper_Comment(Program_Item(lines, 'gripper', 'open'))),
               ('program', Program_Line(lines, 'movej', c['retreat'])),
               ('comment', 'S2 (generated): %s -> %s' % (c['retreat'], c['approach']))]
            + [('movej', name) for name in vias['S2']] + [('movej', c['approach'])]
            + [('comment', 'Station: pick (verbatim from the program)'), ('program', Program_Line(lines, 'movej', c['pick'])),
               ('comment', Gripper_Comment(Program_Item(lines, 'gripper', 'close'))),
               ('program', Program_Line(lines, 'movel', c['above'])),
               ('comment', 'S3 (generated): %s -> %s' % (c['above'], c['home']))]
            + [('movej', name) for name in vias['S3']] + [('movej', c['home'])]
            + [('comment', 'After the rotation sequence (verbatim from the program)')] + Program_Entries(lines[last + 1:]))


def Learned_Order(entries: tp.Sequence[tp.Tuple[str, str]], lines: tp.Sequence[Program_Line_Str]) -> tp.List[str]:
    """
    Description:
        The learned points a motion sequence moves to, in the order of first use: the targets of its
        program lines and of its generated movej, the generated via points (via_*) excluded.

    Args:
        (1) entries [List<Tuple<string, string>>]: The motion-sequence entries.
        (2) lines [List<Program_Line_Str>]: The case's program lines (the targets of the program entries).

    Returns:
        (1) parameter [List<string>]: The point names.
    """

    target = {line.Raw: line.Target for line in lines if line.Kind in ('movej', 'movel')}
    names: tp.List[str] = []
    for kind, value in entries:
        name = target[value] if kind == 'program' else value if kind == 'movej' and not value.startswith('via_') else None
        if name is not None and name not in names:
            names.append(name)
    return names


def main():
    """
    Description:
        Plan the generated segments S1-S3 of the rotation sequence of cases 17 and 18, verify each
        independently, write Data/Generated/SF1_Rotation_<case>.urscript (the points) and
        SF1_Rotation_<case>_Sequence.script (home -> ... -> home in program order), and print the
        report: taught vs optimised per segment and the home -> home totals.

            .venv/bin/python Example/plan_rotation.py [--penalty-weight W] [--out DIR]
    """

    parser = argparse.ArgumentParser(description='Plan the transfer segments of the rotation sequence.')
    parser.add_argument('--config', default=CONST_DEFAULT_CONFIG_FILE, help='the cell configuration (default: %(default)s)')
    parser.add_argument('--penalty-weight', type=float, default=CONST_PENALTY_WEIGHT, help='clearance penalty weight [s/m]')
    parser.add_argument('--out', default=CONST_OUTPUT_DIR, help='output directory (default: %(default)s)')
    parser.add_argument('--case', type=int, choices=sorted(CONST_CASES), action='append', help='one case (default: both)')
    args = parser.parse_args()
    print('[WARNING] Simulation passed != safe on the robot. Run every generated path on the real cell at reduced speed first.')
    cases = args.case or sorted(CONST_CASES)
    try:
        cfg = Load_Configuration(args.config)
        openings = {(case, segment): Opening(cfg, case, state) for case in cases for segment, state in CONST_SEGMENT_STATE.items()}
    except Configuration_Error_Cls as error:
        print('[ERROR] %s' % error)
        return 1
    for case in cases:
        print('[INFO] Case %d: holding opening %.1f mm (%s), open %.1f mm.'
              % (case, cfg.Holding_Openings[case][0] * 1e3, cfg.Holding_Openings[case][1], CONST_RELEASE_OPENING * 1e3))
    with open(CONST_POSITIONS, encoding='utf-8') as handle:
        points = Read_URScript(handle.read()).Points
    with open(CONST_CASES_FILE, encoding='utf-8') as handle:
        program = Read_Program_Excerpt(handle.read())
    station_q = [points[name].Q for name in CONST_STATION_POINTS]
    scenes = {opening: Scene_Cls(dataclasses.replace(cfg, Opening=opening)) for opening in sorted(set(openings.values()))}
    budget = Planner_Budget_Str()
    os.makedirs(args.out, exist_ok=True)
    for case in cases:
        c, rows, generated = CONST_CASES[case], [], {}
        for index, (segment, (start, goal)) in enumerate(Segments(case).items()):
            scene = scenes[openings[(case, segment)]]
            planner = Planner_Cls(scene, station_q, args.penalty_weight, budget)
            started = time.time()
            try:
                result = planner.Plan(points[start].Q, points[goal].Q, CONST_SEED * 1000 + case * 10 + index)
            except Planner_Error_Cls as error:
                print('[ERROR] Case %d %s (%s -> %s): %s Nothing is written for this case.' % (case, segment, start, goal, error))
                return 1
            taught = [np.asarray(points[n].Q) for n in [start] + CONST_TAUGHT_VIAS[segment] + [goal]]
            generated[segment] = result.Path
            rows.append((segment, start, goal, Measure(scene, taught, station_q), Measure(scene, result.Path, station_q),
                         result, time.time() - started))
        # Outputs: the points and the sequence, written only after every segment is verified.
        vias = {segment: ['via_%s_%s_%02d' % (Segments(case)[segment][0], Segments(case)[segment][1], k + 1)
                          for k in range(len(path) - 2)] for segment, path in generated.items()}
        via_points = [Make_Generated_Point(name, Matrix_To_Pose(TCP_Pose(q, cfg.TCP_Offset)), list(q))
                      for segment in ('S1', 'S2', 'S3') for name, q in zip(vias[segment], generated[segment][1:-1])]
        lines = program[case]
        entries = Whole_Case_Entries(lines, c, vias)
        # The learned points of the optimised case, in the order of first use (the taught via points
        # of S1 - S3 are no longer used), then the generated via points.
        learned = Learned_Order(entries, lines)
        header = ['# Case %d of sf1_plc_v4.script, the whole case, optimised: the learned points it moves to (verbatim, in' % case,
                  '# the order of first use) and the generated via points of S1-S3 (_p informational, the robot is driven',
                  '# by _q). Generated by Example/plan_rotation.py, seed %d, penalty weight %g s/m. Simulation passed !='
                  % (CONST_SEED, args.penalty_weight),
                  '# safe on the robot: run it on the real cell at reduced speed first.']
        with open(os.path.join(args.out, 'SF1_Rotation_%d.urscript' % case), 'w', encoding='utf-8') as handle:
            handle.write(Write_URScript(Make_Document([points[name] for name in learned] + via_points, header)))
        sequence_header = ['# Case %d of sf1_plc_v4.script, the whole case, optimised: the program\'s lines verbatim except the' % case,
                           '# transfer segments S1-S3 of the rotation sequence, which are generated movej (the program\'s a and v,',
                           '# r=0). The robot enters the case where the previous case left it. Points: SF1_Rotation_%d.urscript.' % case,
                           '# Simulation passed != safe on the robot: run it at reduced speed first.']
        with open(os.path.join(args.out, 'SF1_Rotation_%d_Sequence.script' % case), 'w', encoding='utf-8') as handle:
            handle.write(Write_Motion_Sequence(entries, sequence_header, CONST_MOVEJ_A_TOKEN, CONST_MOVEJ_V_TOKEN))
        # Report.
        print('[INFO] Case %d (estimated durations: trapezoid on the leading joint, a=%s, v=%s, blends and gripper excluded;'
              % (case, CONST_MOVEJ_A_TOKEN, CONST_MOVEJ_V_TOKEN))
        print('[INFO]   "far": outside the 50 mm station neighbourhood, where the penalty counts):')
        print('[INFO]   %-3s %-9s %4s %8s %9s %9s %9s %-25s %10s %-25s %9s %4s %s'
              % ('S', 'path', 'vias', 'T_est/s', 'sum|dq|', 'max-jnt', 'min d/mm', 'robot body', 'far d/mm', 'robot body (far)',
                 'penalty/m', 'viol', 'wall/s'))
        totals = {'taught': 0.0, 'optimised': 0.0}
        for segment, start, goal, before, after, result, wall in rows:
            for label, m in (('taught', before), ('optimised', after)):
                totals[label] += m['duration']
                print('[INFO]   %-3s %-9s %4d %8.2f %9.3f %9.3f %9.1f %-25s %10.1f %-25s %9.3f %4d %s'
                      % (segment, label, m['vias'], m['duration'], m['sum'], m['max'], m['closest'] * 1e3, m['body'] or '-',
                         m['far'] * 1e3, m['far_body'] or '-', m['penalty'], m['violations'],
                         '%.0f (%s)' % (wall, result.Origin) if label == 'optimised' else ''))
        fixed = Station_Duration(points, c)
        print('[INFO]   home -> home: taught %.2f s, optimised %.2f s (S1-S3 + station operations %.2f s; estimate)'
              % (totals['taught'] + fixed, totals['optimised'] + fixed, fixed))
    print('[INFO] Written to %s. Simulation passed != safe on the robot.' % os.path.relpath(args.out, CONST_PROJECT_ROOT))
    return 0


if __name__ == '__main__':
    sys.exit(main())
