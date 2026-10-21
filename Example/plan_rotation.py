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
# Math (Mathematical functions)
import math
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
from URScript.Core import (Read_URScript, Read_Program_Excerpt, Make_Generated_Point, Make_Document, Write_URScript, Replace_Joints,
                           Write_Motion_Sequence, Rotation_Sequence, Gripper_Comment, Program_Line_Str, URScript_Error_Cls)
#   ../Kinematics/Core — the informational _p of a via point
from Kinematics.Core import TCP_Pose, Matrix_To_Pose, Kinematics_Error_Cls
#   ../Scene/Core — the scene at each gripper opening and the check of the learned points
from Scene.Core import Scene_Cls, Check_Learned_Kinematics
#   ../Planner/Core — the planner and the estimates
from Planner.Core import (Plan_All, Plan_Request_Str, Make_Pool, Planner_Budget_Str, Planner_Error_Cls, Estimate_Duration,
                          Linear_Duration, Path_Length, CONST_MOVEJ_A_TOKEN, CONST_MOVEJ_V_TOKEN, CONST_PENALTY_DISTANCE)

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
# The file names of a rotation sequence: its points and its motion sequence (the delivered output
# and every variant).
#   Unit [Tuple<string, string>]
CONST_FILE_NAMES = ('SF1_Rotation_17.urscript', 'SF1_Rotation_17_Sequence.script')
# The variants kept for comparison (owner, 2026-10-19; Example/write_variants.py), each in its own
# directory, with its status: what it is and whether it may run on the robot. c2 was delivered and
# tested on the real cell, c1 is the previous optimisation (owner, 2026-10-21). ASCII only: the
# status is written into the variant files' headers.
#   Unit [string], [Dict<string, string>]
CONST_VARIANT_DIR = os.path.join(CONST_OUTPUT_DIR, 'variants')
CONST_VARIANTS = {'taught': 'the taught program (sf1_plc_v4, case 17)',
                  'b_optimised': 'previous optimisation (2026-10-18, 17.50 s), superseded by c1',
                  'c1': 'previous optimisation (simulation only), = Data/Generated/: the pick\'s wrist 3 one turn lower, same poses',
                  'c2': 'delivered - tested on the real cell (owner, 2026-10-21): the drop on branch S- E- W- (wrist flipped in S1)',
                  'c3': 'proposal for re-teaching, not verified on the robot: the drop on branch S- E- W+ (home\'s branch)'}
# The deliverable (owner, 2026-10-18): case 17 of the program, home -> ... -> home. The other cases
# of the extract are ground truth for the model only (SPEC 1).
#   Unit [int]
CONST_CASE = 17
# Its points: home, the station points (fixed, taught) and the taught via points of the three
# generated segments S1 home -> drop, S2 retreat -> approach, S3 lift -> home (owner).
#   Unit [Dict<string, string>], [Dict<string, List<string>>]
CONST_POINTS = {'home': 'home', 'drop': 'drop_rotate_1', 'below': 'Waypoint_4', 'retreat': 'Waypoint_5',
                'approach': 'Waypoint_18', 'pick': 'pick_rotate_1', 'above': 'Waypoint_6'}
CONST_TAUGHT_VIAS = {'S1': ['Waypoint_1'], 'S2': ['Waypoint_1', 'Waypoint_2', 'Waypoint_3'], 'S3': ['Waypoint_2']}
# The fixed station points (the endpoint neighbourhood of the generated rule).
#   Unit [Tuple<string>]
CONST_STATION_POINTS = ('drop_rotate_1', 'Waypoint_4', 'Waypoint_5', 'Waypoint_18', 'pick_rotate_1', 'Waypoint_6')
# The delivered station joints (owner, 2026-10-19: "q6 representative chosen to avoid an unnecessary
# wrist-3 turn"): the pick's three points use wrist 3 one turn lower, 4.739 - 2 pi = -1.544 rad. The
# same arm pose and the same _p; S3 turns wrist 3 by 1.5 instead of 4.7 rad (home -> home 17.50 ->
# 14.66 s, estimate). The one exception to never turning a learned joint (SPEC 10.5): the robot's
# get_inverse_kin(<name>_p, qnear=<name>_q) of the pick's lines takes the solution nearest the new
# qnear, which the first run at reduced speed confirms (KNOWN_ISSUES). Point -> (joint, turns).
#   Unit [Dict<string, Tuple<int, int>>] (index, 2 pi)
CONST_Q_TURNS = {'Waypoint_18': (5, -1), 'pick_rotate_1': (5, -1), 'Waypoint_6': (5, -1)}
# Gripper opening after rq_set_pos_spd_for(0, ...): the full 85 mm stroke; the release opening of
# 85 mm is real, the part is released into a magazine with free space (owner, 2026-10-18, SPEC 4.9).
# While the part is held, case 17's Tool.Holding_Openings entry applies (Opening).
#   Unit [float] (m)
CONST_RELEASE_OPENING = 0.085
# The gripper state of each generated segment: S1 and S3 carry the part, S2 runs open.
#   Unit [Dict<string, string>]
CONST_SEGMENT_STATE = {'S1': 'holding', 'S2': 'open', 'S3': 'holding'}
# The recorded seed of every planning run; segment k (0 = S1) plans with seed * 1000 + case * 10 + k
# (case 17).
#   Unit [int]
CONST_SEED = 20261017
# Weight of the clearance penalty in the objective (SPEC 10.6), chosen by the owner (2026-10-17):
# against w = 0 it raised the smallest clearance outside the station
# neighbourhood from 20.4 - 28.9 mm to 26.6 - 34.5 mm for +0.09 / -0.02 s home -> home (cases 17 / 18;
# measured 2026-10-17, when both cases were planned).
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


def Segments() -> tp.Dict[str, tp.Tuple[str, str]]:
    """
    Description:
        The ends of the three generated segments.

    Returns:
        (1) parameter [Dict<string, Tuple<string, string>>]: S1, S2, S3 -> (from, to).
    """

    c = CONST_POINTS
    return {'S1': (c['home'], c['drop']), 'S2': (c['retreat'], c['approach']), 'S3': (c['above'], c['home'])}


def Opening(cfg: Cell_Configuration_Str, state: str) -> float:
    """
    Description:
        The gripper opening of case 17 in a gripper state: its Tool.Holding_Openings entry while the
        part is held, CONST_RELEASE_OPENING when open.

    Args:
        (1) cfg [Cell_Configuration_Str(object)]: The cell configuration.
        (2) state [string]: 'holding' or 'open'.

    Returns:
        (1) parameter [float]: The inner pad-face gap (m).
    """

    if state == 'open':
        return CONST_RELEASE_OPENING
    if CONST_CASE not in cfg.Holding_Openings:
        raise Configuration_Error_Cls('Tool.Holding_Openings has no entry for case %d; add {Case: %d, Opening: <m>, '
                                      'Source: <text>} to the configuration.' % (CONST_CASE, CONST_CASE))
    return cfg.Holding_Openings[CONST_CASE][0]


def Read_Program() -> tp.Tuple[tp.Dict, tp.List[Program_Line_Str]]:
    """
    Description:
        The program's learned points (the positions extract) and the program lines of case 17's
        rotation sequence, home -> ... -> home (Rotation_Sequence), from the extracts (SPEC 10.1).

    Returns:
        (1) parameter [Tuple<Dict<string, Robot_Point_Str>, List<Program_Line_Str>>]: The points and the lines.
    """

    with open(CONST_POSITIONS, encoding='utf-8') as handle:
        points = Read_URScript(handle.read()).Points
    with open(CONST_CASES_FILE, encoding='utf-8') as handle:
        lines = Rotation_Sequence(Read_Program_Excerpt(handle.read())[CONST_CASE], CONST_POINTS['home'])
    return points, lines


def Check_Learned(cfg: Cell_Configuration_Str, points: tp.Dict, source: str) -> None:
    """
    Description:
        Check every learned point read against the FK sanity bound before it is used (SPEC 9.4,
        Check_Learned_Kinematics), the orientation bound from the tool length of a robot-only scene at
        Tool.Opening, and print the largest residual. Generated via points (via_*) are not learned and
        are not checked. Raises Kinematics_Error_Cls naming the first point outside, with its numbers.

    Args:
        (1) cfg [Cell_Configuration_Str(object)]: The cell configuration.
        (2) points [Dict<string, Robot_Point_Str>]: The points read.
        (3) source [string]: What they were read from, for the printed line.
    """

    learned = [point for name, point in points.items() if not name.startswith('via_')]
    residuals = Check_Learned_Kinematics(Scene_Cls(cfg, with_environment=False), cfg, learned)
    print('[INFO] FK sanity bound (%.1f mm): the %d learned points of %s within it, the largest residual %.2f mm / %.3f deg.'
          % (cfg.FK_Sanity_Bound_Position * 1e3, len(residuals), source, max(r[0] for r in residuals.values()) * 1e3,
             math.degrees(max(r[1] for r in residuals.values()))))


def Plan_Requests(cfg: Cell_Configuration_Str, points: tp.Dict, station_q: tp.Sequence[tp.Sequence[float]],
                  penalty_weight: float) -> tp.List[Plan_Request_Str]:
    """
    Description:
        The planning requests of S1, S2 and S3: each at its gripper opening (CONST_SEGMENT_STATE),
        between the _q of its ends, with the full budget and its own seed, seed * 1000 + case * 10 + k
        (k = 0 for S1).

    Args:
        (1) cfg [Cell_Configuration_Str(object)]: The cell configuration.
        (2) points [Dict<string, Robot_Point_Str>]: The points the ends are taken from.
        (3) station_q [List<Vector<float> 1x6>]: The fixed station points (rad).
        (4) penalty_weight [float]: The clearance penalty weight (s/m).

    Returns:
        (1) parameter [List<Plan_Request_Str>]: S1, S2, S3.
    """

    return [Plan_Request_Str('%s (%s -> %s)' % (segment, start, goal),
                             dataclasses.replace(cfg, Opening=Opening(cfg, CONST_SEGMENT_STATE[segment])),
                             [np.asarray(q, dtype=float) for q in station_q], penalty_weight, Planner_Budget_Str(),
                             np.asarray(points[start].Q, dtype=float), np.asarray(points[goal].Q, dtype=float),
                             CONST_SEED * 1000 + CONST_CASE * 10 + index)
            for index, (segment, (start, goal)) in enumerate(Segments().items())]


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
        station neighbourhood), the clearance penalty and the generated-rule violations (the
        station-box rule with the floors of the path's ends included).

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
    floors = scene.Station_Box_Floors(path[0], path[-1])
    for q_a, q_b in zip(path[:-1], path[1:]):
        violations += len(scene.Check_Segment(q_a, q_b, kind='generated', station_q=station_q, box_floors=floors)[1])
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


def Station_Duration(points: tp.Dict) -> float:
    """
    Description:
        Estimated duration of the station operations (unchanged): the drop's movel, the retreat
        movej, the pick movej and the lift movel; gripper time excluded.

    Args:
        (1) points [Dict<string, Robot_Point_Str>]: The program's points.

    Returns:
        (1) parameter [float]: The duration (s).
    """

    c = CONST_POINTS
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


def Rotation_Entries(lines: tp.Sequence[Program_Line_Str], vias: tp.Dict[str, tp.List[str]]) -> tp.List[tp.Tuple[str, str]]:
    """
    Description:
        The optimised rotation sequence, home -> ... -> home, as motion-sequence entries: the start at
        home and S1 - S3 as generated movej to their via points and ends, the station operations
        verbatim from the program. The program's own move to home blends (case 17: r=0.01) into S1,
        and a blend into a generated segment is not checked (SPEC 8.5), so the start is the
        generated movej to home with r=0 (owner, 2026-10-18).

    Args:
        (1) lines [List<Program_Line_Str>]: The program lines of the rotation sequence (Rotation_Sequence).
        (2) vias [Dict<string, List<string>>]: The via point names of S1, S2 and S3.

    Returns:
        (1) parameter [List<Tuple<string, string>>]: The entries in order.
    """

    c = CONST_POINTS
    blend = CONST_BLEND.search(lines[0].Raw)
    return ([('comment', 'Start at %s (generated, r=0): the program\'s line blends (r=%s) into S1, and a blend into a '
                         'generated segment is not checked' % (c['home'], blend.group(1) if blend else '0')), ('movej', c['home']),
             ('comment', 'S1 (generated): %s -> %s' % (c['home'], c['drop']))]
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
            + [('movej', name) for name in vias['S3']] + [('movej', c['home'])])


def Learned_Order(entries: tp.Sequence[tp.Tuple[str, str]], lines: tp.Sequence[Program_Line_Str]) -> tp.List[str]:
    """
    Description:
        The learned points a motion sequence moves to, in the order of first use: the targets of its
        program lines and of its generated movej, the generated via points (via_*) excluded.

    Args:
        (1) entries [List<Tuple<string, string>>]: The motion-sequence entries.
        (2) lines [List<Program_Line_Str>]: The program lines (the targets of the program entries).

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


def Delivered_Points(points: tp.Dict) -> tp.Dict:
    """
    Description:
        The program's points with the delivered station joints (CONST_Q_TURNS): each listed joint
        turned by its whole turns, every other token the program's (Replace_Joints).

    Args:
        (1) points [Dict<string, Robot_Point_Str>]: The program's points.

    Returns:
        (1) parameter [Dict<string, Robot_Point_Str>]: A new dictionary; the given points are not changed.
    """

    out = dict(points)
    for name, (joint, turns) in CONST_Q_TURNS.items():
        q = list(points[name].Q)
        q[joint] += turns * 2.0 * math.pi
        out[name] = Replace_Joints(points[name], q)
    return out


def Delivered_Status() -> tp.List[str]:
    """
    Description:
        The status header lines of the delivered output: which station joints differ from the program.

    Returns:
        (1) parameter [List<string>]: Comment lines.
    """

    return ['# Delivered. %s use wrist 3 one turn lower than taught (owner, 2026-10-19: q6 representative chosen'
            % ', '.join(CONST_Q_TURNS), '# to avoid an unnecessary wrist-3 turn); same pose, same _p (SPEC 10.5).']


def Machine_Margin(cfg: Cell_Configuration_Str) -> tp.List[str]:
    """
    Description:
        The machine margin the generated segments keep, with its source (Clearance.Machine_Extra,
        SPEC 9.7): Clearance.Environment plus the extra margin to the pieces of the assembly outside
        the station zone. Two lines of text, for the report and the output headers.

    Args:
        (1) cfg [Cell_Configuration_Str(object)]: The cell configuration.

    Returns:
        (1) parameter [List<string>]: The text, in two lines.
    """

    return ['Machine margin: generated segments keep %.1f mm (Clearance.Environment %.1f + Machine_Extra %.1f mm) to %s'
            % ((cfg.Clearance_Environment + cfg.Machine_Extra_Margin) * 1e3, cfg.Clearance_Environment * 1e3,
               cfg.Machine_Extra_Margin * 1e3, cfg.Machine_Extra_Assembly),
            'outside the station zone; source: %s.' % cfg.Machine_Extra_Source]


def Provenance(names: tp.Sequence[str], points: tp.Dict, program_points: tp.Dict) -> tp.Tuple[tp.List[str], tp.List[str]]:
    """
    Description:
        Which learned points are written verbatim from the program extract (_p and _q tokens) and
        which keep the program's _p with another _q (Delivered_Points, a proposal's drop). A learned
        point whose _p differs from the program's is refused: it would be another pose.

    Args:
        (1) names [List<string>]: The learned points written, in order.
        (2) points [Dict<string, Robot_Point_Str>]: The points written.
        (3) program_points [Dict<string, Robot_Point_Str>]: The program's points (the extract).

    Returns:
        (1) parameter [Tuple<List<string>, List<string>>]: The verbatim points and those with another _q.
    """

    verbatim, other_q = [], []
    for name in names:
        if points[name].P_Tokens != program_points[name].P_Tokens:
            raise Planner_Error_Cls('The point %s to be written has another _p than the program\'s; a learned point keeps its pose.'
                                    % name)
        (verbatim if points[name].Q_Tokens == program_points[name].Q_Tokens else other_q).append(name)
    return verbatim, other_q


def Write_Outputs(out: str, cfg: Cell_Configuration_Str, points: tp.Dict, program_points: tp.Dict, lines: tp.Sequence[Program_Line_Str],
                  generated: tp.Dict[str, tp.List[np.ndarray]], penalty_weight: float, status: tp.Sequence[str]) -> None:
    """
    Description:
        Write the optimised rotation sequence: SF1_Rotation_17.urscript (the learned points it moves
        to, in the order of first use, then the generated via points via_<from>_<to>_<nn>) and
        SF1_Rotation_17_Sequence.script (Rotation_Entries). Each header names the learned points written
        verbatim from the extract and those with the program's _p and another _q (Provenance); the
        status lines say why.

    Args:
        (1) out [string]: The output directory.
        (2) cfg [Cell_Configuration_Str(object)]: The cell configuration (the TCP of the informational _p).
        (3) points [Dict<string, Robot_Point_Str>]: The points the paths were planned between (learned).
        (4) program_points [Dict<string, Robot_Point_Str>]: The program's points, as read from the extract.
        (5) lines [List<Program_Line_Str>]: The program lines of the rotation sequence (Rotation_Sequence).
        (6) generated [Dict<string, List<Vector<float> 1x6>>]: The verified path of S1, S2 and S3, ends included (rad).
        (7) penalty_weight [float]: The clearance penalty weight the paths were planned with (s/m).
        (8) status [List<string>]: Comment lines saying what the output is and why points differ, after the description.
    """

    vias = {segment: ['via_%s_%s_%02d' % (Segments()[segment][0], Segments()[segment][1], k + 1)
                      for k in range(len(path) - 2)] for segment, path in generated.items()}
    via_points = [Make_Generated_Point(name, Matrix_To_Pose(TCP_Pose(q, cfg.TCP_Offset)), list(q))
                  for segment in ('S1', 'S2', 'S3') for name, q in zip(vias[segment], generated[segment][1:-1])]
    entries = Rotation_Entries(lines, vias)
    # The learned points of the optimised sequence, in the order of first use (the taught via points
    # of S1 - S3 are no longer used), then the generated via points.
    learned = Learned_Order(entries, lines)
    verbatim, other_q = Provenance(learned, points, program_points)
    source = os.path.relpath(CONST_POSITIONS, CONST_PROJECT_ROOT)
    header = (['# Case %d of sf1_plc_v4.script, the rotation sequence home -> home, optimised: the learned points it' % CONST_CASE,
               '# moves to, in the order of first use, then the generated via points of S1-S3 (_p informational, the',
               '# robot is driven by _q). Generated by Example/plan_rotation.py, seed %d, penalty weight %g s/m.'
               % (CONST_SEED, penalty_weight),
               '# Verbatim from %s (_p and _q): %s.' % (source, ', '.join(verbatim))]
              + (['# The program\'s _p, another _q: %s.' % ', '.join(other_q)] if other_q else [])
              + ['# ' + line for line in Machine_Margin(cfg)] + list(status)
              + ['# Simulation passed != safe on the robot: run it on the real cell at reduced speed first.'])
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, CONST_FILE_NAMES[0]), 'w', encoding='utf-8') as handle:
        handle.write(Write_URScript(Make_Document([points[name] for name in learned] + via_points, header)))
    sequence_header = ['# Case %d of sf1_plc_v4.script, the rotation sequence home -> home, optimised: the station lines' % CONST_CASE,
                       '# verbatim from the program, the transfer segments S1-S3 generated movej (the program\'s a and v,',
                       '# r=0). Points: SF1_Rotation_%d.urscript.' % CONST_CASE]
    # The points of the station lines copied verbatim whose _q is not the program's.
    target = {line.Raw: line.Target for line in lines if line.Kind in ('movej', 'movel')}
    station = {target[value] for kind, value in entries if kind == 'program'}
    station_other_q = [name for name in other_q if name in station]
    if station_other_q:
        sequence_header += ['# The station lines to %s are the program\'s text; their points have the program\'s _p and'
                            % ', '.join(station_other_q), '# another _q (SF1_Rotation_%d.urscript).' % CONST_CASE]
    sequence_header += ['# ' + line for line in Machine_Margin(cfg)] + list(status) + [
        '# Simulation passed != safe on the robot: run it at reduced speed first.']
    with open(os.path.join(out, CONST_FILE_NAMES[1]), 'w', encoding='utf-8') as handle:
        handle.write(Write_Motion_Sequence(entries, sequence_header, CONST_MOVEJ_A_TOKEN, CONST_MOVEJ_V_TOKEN))


def main():
    """
    Description:
        Plan the generated segments S1-S3 of the rotation sequence of case 17, verify each
        independently, write Data/Generated/SF1_Rotation_17.urscript (the points) and
        SF1_Rotation_17_Sequence.script (home -> ... -> home in program order), and print the report:
        taught vs optimised per segment and the home -> home totals.

            .venv/bin/python Example/plan_rotation.py [--workers N] [--penalty-weight W] [--out DIR]

        --workers N plans S1-S3 at once in N spawned processes (each with its own scene); the output
        is byte-identical for every N (Plan_All). 1 (default) plans in this process.
    """

    parser = argparse.ArgumentParser(description='Plan the transfer segments of the rotation sequence of case 17.')
    parser.add_argument('--config', default=CONST_DEFAULT_CONFIG_FILE, help='the cell configuration (default: %(default)s)')
    parser.add_argument('--penalty-weight', type=float, default=CONST_PENALTY_WEIGHT, help='clearance penalty weight [s/m]')
    parser.add_argument('--out', default=CONST_OUTPUT_DIR, help='output directory (default: %(default)s)')
    parser.add_argument('--workers', type=int, default=1, help='planning processes; the output does not depend on it (default: 1)')
    args = parser.parse_args()
    if args.workers < 1:
        print('[ERROR] --workers must be 1 or more (1 = plan in this process), not %d.' % args.workers)
        return 1
    print('[WARNING] Simulation passed != safe on the robot. Run every generated path on the real cell at reduced speed first.')
    try:
        cfg = Load_Configuration(args.config)
        openings = {segment: Opening(cfg, state) for segment, state in CONST_SEGMENT_STATE.items()}
        program_points, lines = Read_Program()
        Check_Learned(cfg, program_points, os.path.basename(CONST_POSITIONS))
    except (Configuration_Error_Cls, URScript_Error_Cls, Kinematics_Error_Cls, OSError) as error:
        print('[ERROR] %s' % error)
        return 1
    # The delivered station joints (CONST_Q_TURNS); the taught rows of the report keep the program's.
    points = Delivered_Points(program_points)
    print('[INFO] Case %d: holding opening %.1f mm (%s), open %.1f mm.'
          % (CONST_CASE, cfg.Holding_Openings[CONST_CASE][0] * 1e3, cfg.Holding_Openings[CONST_CASE][1], CONST_RELEASE_OPENING * 1e3))
    for line in Machine_Margin(cfg):
        print('[INFO] %s' % line)
    station_q = [points[name].Q for name in CONST_STATION_POINTS]
    started = time.time()
    try:
        requests = Plan_Requests(cfg, points, station_q, args.penalty_weight)
        if args.workers > 1:
            with Make_Pool(args.workers) as pool:
                results = Plan_All(requests, pool)
        else:
            results = Plan_All(requests)
    except Planner_Error_Cls as error:
        print('[ERROR] %s Nothing is written.' % error)
        return 1
    planned = time.time() - started
    scenes = {opening: Scene_Cls(dataclasses.replace(cfg, Opening=opening)) for opening in sorted(set(openings.values()))}
    rows, generated = [], {}
    for (segment, (start, goal)), result in zip(Segments().items(), results):
        scene = scenes[openings[segment]]
        taught = [np.asarray(program_points[n].Q) for n in [start] + CONST_TAUGHT_VIAS[segment] + [goal]]
        generated[segment] = result.Path
        rows.append((segment, Measure(scene, taught, station_q), Measure(scene, result.Path, station_q), result))
    # Outputs: the points and the sequence, written only after every segment is verified.
    Write_Outputs(args.out, cfg, points, program_points, lines, generated, args.penalty_weight, Delivered_Status())
    # Report.
    print('[INFO] Case %d (estimated durations: trapezoid on the leading joint, a=%s, v=%s, blends and gripper excluded;'
          % (CONST_CASE, CONST_MOVEJ_A_TOKEN, CONST_MOVEJ_V_TOKEN))
    print('[INFO]   "far": outside the 50 mm station neighbourhood, where the penalty counts):')
    print('[INFO]   %-3s %-9s %4s %8s %9s %9s %9s %-25s %10s %-25s %9s %4s %s'
          % ('S', 'path', 'vias', 'T_est/s', 'sum|dq|', 'max-jnt', 'min d/mm', 'robot body', 'far d/mm', 'robot body (far)',
             'penalty/m', 'viol', 'origin'))
    totals = {'taught': 0.0, 'optimised': 0.0}
    for segment, before, after, result in rows:
        for label, m in (('taught', before), ('optimised', after)):
            totals[label] += m['duration']
            print('[INFO]   %-3s %-9s %4d %8.2f %9.3f %9.3f %9.1f %-25s %10.1f %-25s %9.3f %4d %s'
                  % (segment, label, m['vias'], m['duration'], m['sum'], m['max'], m['closest'] * 1e3, m['body'] or '-',
                     m['far'] * 1e3, m['far_body'] or '-', m['penalty'], m['violations'],
                     result.Origin if label == 'optimised' else ''))
    fixed = Station_Duration(points)
    print('[INFO]   home -> home: taught %.2f s, optimised %.2f s (S1-S3 + station operations %.2f s; estimate)'
          % (totals['taught'] + fixed, totals['optimised'] + fixed, fixed))
    print('[INFO] Wall time: planning %.0f s (%d worker%s), report %.0f s.'
          % (planned, args.workers, '' if args.workers == 1 else 's', time.time() - started - planned))
    print('[INFO] Written to %s. Simulation passed != safe on the robot.' % os.path.relpath(args.out, CONST_PROJECT_ROOT))
    return 0


if __name__ == '__main__':
    sys.exit(main())
