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
# Glob (Unix style pathname pattern expansion)
import glob
# Textwrap (Text wrapping)
import textwrap
# Dataclasses (Data Classes)
import dataclasses
# Time (Time access)
import time
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.:
#   ../Configuration/Core — the cell configuration and the jobs
from Configuration.Core import (Load_Configuration, Load_Job, Cell_Configuration_Str, Job_Configuration_Str, Configuration_Error_Cls,
                                CONST_DEFAULT_CONFIG_FILE, CONST_PROJECT_ROOT, CONST_JOINT_NAMES)
#   ../URScript/Core — the program extracts, the job's sequence and the output files
from URScript.Core import (Read_URScript, Read_Program_Excerpt, Read_Extract_Source, Resolve_Sequence, Make_Generated_Point, Make_Document,
                           Write_URScript, Replace_Joints, Write_Motion_Sequence, Program_Entries, Program_Line_Str, Sequence_Plan_Str,
                           URScript_Error_Cls)
#   ../Kinematics/Core — the informational _p of a via point, the IK branch of a configured point and a taught movel's line
from Kinematics.Core import (TCP_Pose, Matrix_To_Pose, Pose_To_Matrix, Solve_IK_Analytic, Nearest_Representative, Linear_Path,
                             Kinematics_Error_Cls)
#   ../Scene/Core — the scene at each gripper opening and the check of the learned points
from Scene.Core import Scene_Cls, Check_Learned_Kinematics
#   ../Planner/Core — the planner and the estimates
from Planner.Core import (Plan_All, Plan_Request_Str, Make_Pool, Planner_Budget_Str, Planner_Error_Cls, Estimate_Duration,
                          Linear_Duration, Path_Length, CONST_MOVEJ_A_TOKEN, CONST_MOVEJ_V_TOKEN, CONST_PENALTY_DISTANCE)

"""
Description:
    Initialization of constants.

    What is planned - the program, the case, the segments, the configured points and the output
    names - is a job, Configuration/Jobs/<name>.yaml (SPEC 8.6); what the job implies in the
    program (the gripper states, the station lines, the station points) is derived from the case's
    lines (Resolve_Sequence).
"""
# The job planned without --job (owner, 2026-10-18: case 17 of sf1_plc_v4.script, home -> home; since
# 2026-10-21 as c2, delivered and tested on the real cell), the job of the previous optimisation c1,
# and the default output directory (SPEC 10.6).
#   Unit [string]
CONST_DEFAULT_JOB = 'sf1_plc_v4_case_17'
CONST_C1_JOB = 'sf1_plc_v4_case_17_c1'
CONST_OUTPUT_DIR = os.path.join(CONST_PROJECT_ROOT, 'Data', 'Generated')
# The extract file names in a program folder (Example/extract_program.py, SPEC 10.1).
#   Unit [string]
CONST_POSITIONS_NAME = '%s_positions.urscript'
CONST_CASES_NAME = '%s_cases_*.script'
# The file names of a variant of case 17: its points and its motion sequence, the case-17 jobs'
# Output.Points and Output.Sequence (test/Example/test_plan_rotation.py pins them equal).
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
                  'c1': 'previous optimisation (simulation only): the pick\'s wrist 3 one turn lower, same poses',
                  'c2': ('delivered - tested on the real cell (owner, 2026-10-21), = Data/Generated/: the drop on branch S- E- W- '
                         '(wrist flipped in S1)'),
                  'c3': 'proposal for re-teaching, not verified on the robot: the drop on branch S- E- W+ (home\'s branch)'}
# Gripper opening after rq_set_pos_spd_for(0, ...): the full 85 mm stroke; the release opening of
# 85 mm is real, the part is released into a magazine with free space (owner, 2026-10-18, SPEC 4.9).
# While the part is held, the case's Tool.Holding_Openings entry applies (Opening).
#   Unit [float] (m)
CONST_RELEASE_OPENING = 0.085
# The recorded seed of every planning run; segment k (0 = the first) plans with
# seed * 1000 + case * 10 + k.
#   Unit [int]
CONST_SEED = 20261017
# Weight of the clearance penalty in the objective (SPEC 10.6), chosen by the owner (2026-10-17):
# against w = 0 it raised the smallest clearance outside the station
# neighbourhood from 20.4 - 28.9 mm to 26.6 - 34.5 mm for +0.09 / -0.02 s home -> home (cases 17 / 18;
# measured 2026-10-17, when both cases were planned).
#   Unit [float] (s/m)
CONST_PENALTY_WEIGHT = 0.5
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
# The blend radius token, and the acceleration and speed tokens, of a program move (absent = no
# blend; a and v are required of every station line, whose duration is estimated from them).
#   Unit [Pattern]
CONST_BLEND = re.compile(r', r=([^,()]+)\)\s*$')
CONST_SPEED = re.compile(r', a=([^,()]+), v=([^,()]+)')
# Cap of the reported robot - environment distance.
#   Unit [float] (m)
CONST_REPORT_DISTMAX = 0.200
# The width the status lines of an output header are wrapped to: the delivered header's first line
# (2026-10-19), so the delivered output is written byte for byte.
#   Unit [int] (characters)
CONST_STATUS_WIDTH = 135
# The environment geometry fitted to the taught samples of cases 17 and 18, not measured
# (KNOWN_ISSUES; SPEC 4.10, 4.12, 4.13): the added station boxes and the station-zone pieces, as
# Scene_Cls.Environment_Label names them. A taught line passing into one is reported with this note.
#   Unit [Tuple<string>]
CONST_FITTED = ('station_bracket', 'station_stub', 'station-zone piece')
# A whole number of turns in words, for the status lines.
#   Unit [Dict<int, string>]
CONST_TURN_WORDS = {1: 'one', 2: 'two', 3: 'three'}


@dataclasses.dataclass
class Job_Plan_Str:
    """
    Description:
        A job with what it plans: the program's points, the case's lines and the sequence the job
        resolves in them (Load_Job_Plan).
    """

    # The job as loaded.
    #   Unit [Job_Configuration_Str]
    Job: Job_Configuration_Str = dataclasses.field(default_factory=Job_Configuration_Str)
    # The program folder's name (the key of Tool.Holding_Openings) and the program's file name.
    #   Unit [string]
    Program_Name: str = ''
    Program_File: str = ''
    # The extracts read.
    #   Unit [string]
    Positions: str = ''
    Cases_File: str = ''
    # The program's learned points (the positions extract, the program's tokens).
    #   Unit [Dict<string, Robot_Point_Str>]
    Points: tp.Dict = dataclasses.field(default_factory=dict)
    # The job's sequence in the case's lines.
    #   Unit [Sequence_Plan_Str]
    Sequence: Sequence_Plan_Str = dataclasses.field(default_factory=Sequence_Plan_Str)


def Job_Extracts(job: Job_Configuration_Str) -> tp.Tuple[str, str]:
    """
    Description:
        The extracts of a job's program folder: <folder>/<name>_positions.urscript and the one
        <name>_cases_*.script that holds the job's case.

    Args:
        (1) job [Job_Configuration_Str(object)]: The job.

    Returns:
        (1) parameter [Tuple<string, string>]: The positions and the cases file.
    """

    name = os.path.basename(job.Program)
    positions = os.path.join(job.Program, CONST_POSITIONS_NAME % name)
    if not os.path.isfile(positions):
        raise Configuration_Error_Cls('The job %s: %s has no %s. Make the extracts with Example/extract_program.py.'
                                      % (job.Name, os.path.relpath(job.Program, CONST_PROJECT_ROOT), CONST_POSITIONS_NAME % name))
    holding, cases = [], []
    for path in sorted(glob.glob(os.path.join(job.Program, CONST_CASES_NAME % name))):
        with open(path, encoding='utf-8') as handle:
            found = Read_Program_Excerpt(handle.read())
        cases += found
        if job.Case in found:
            holding.append(path)
    if len(holding) != 1:
        raise Configuration_Error_Cls('The job %s plans case %d, which %s. Extract it with Example/extract_program.py --program <the '
                                      'program> --cases %d --out %s.'
                                      % (job.Name, job.Case, 'is in no cases extract of %s (its cases: %s)'
                                         % (os.path.relpath(job.Program, CONST_PROJECT_ROOT), ', '.join(str(c) for c in sorted(cases)) or 'none')
                                         if not holding else 'is in %d cases extracts' % len(holding),
                                         job.Case, os.path.relpath(job.Program, CONST_PROJECT_ROOT)))
    return positions, holding[0]


def Default_Positions() -> str:
    """
    Description:
        The positions extract of the default job's program (the learned points the viewers show).

    Returns:
        (1) parameter [string]: The file.
    """

    return Job_Extracts(Load_Job(CONST_DEFAULT_JOB))[0]


def Load_Job_Plan(job: str) -> Job_Plan_Str:
    """
    Description:
        Load a job and resolve it in its program: the extracts (one program version, the same
        sha256), the learned points, the case's lines and the job's sequence (Resolve_Sequence).
        Also refused: a point the sequence moves to that the positions extract does not hold, a
        configured point the output does not move to, a station line without its a
        and v (its duration is estimated from them), and a taught movej of a segment at another a
        or v than the generated moves' (CONST_MOVEJ_A_TOKEN / _V_TOKEN, KNOWN_ISSUES).

    Args:
        (1) job [string]: The job's name or file (Load_Job).

    Returns:
        (1) parameter [Job_Plan_Str(object)]: The job and its sequence.
    """

    loaded = Load_Job(job)
    positions, cases_file = Job_Extracts(loaded)
    with open(positions, encoding='utf-8') as handle:
        text = handle.read()
    program_file, sha256 = Read_Extract_Source(text)
    points = Read_URScript(text).Points
    with open(cases_file, encoding='utf-8') as handle:
        text = handle.read()
    if Read_Extract_Source(text) != (program_file, sha256):
        raise Configuration_Error_Cls('The job %s: %s and %s come from different programs or versions (their sha256 differ). '
                                      'Re-extract both from one program.' % (loaded.Name, os.path.basename(positions),
                                                                             os.path.basename(cases_file)))
    lines = Read_Program_Excerpt(text)[loaded.Case]
    sequence = Resolve_Sequence(lines, loaded.Segments, points)
    missing = sorted({line.Target for line in sequence.Lines if line.Kind in ('movej', 'movel') and line.Target not in points})
    if missing:
        raise Configuration_Error_Cls('The job %s: case %d moves to %s, which the positions extract does not hold.'
                                      % (loaded.Name, loaded.Case, ', '.join(missing)))
    plan = Job_Plan_Str(loaded, os.path.basename(loaded.Program), program_file, positions, cases_file, points, sequence)
    order = Learned_Order(Sequence_Entries(plan, {name: [] for name, _, _ in loaded.Segments}), sequence.Lines)
    for point, _, _, _, _ in loaded.Point_Configurations:
        if point not in order:
            raise Configuration_Error_Cls('The job %s configures %s, which the output does not move to (it moves to %s).'
                                          % (loaded.Name, point, ', '.join(order)))
    for station in sequence.Stations:
        for line in station:
            if line.Kind in ('movej', 'movel') and CONST_SPEED.search(line.Raw) is None:
                raise Configuration_Error_Cls('The station line at program line %d has no a and v, so its duration cannot be '
                                              'estimated: %s' % (line.Line, line.Raw.strip()))
    for segment in sequence.Segments:
        for line in segment.Lines:
            speed = CONST_SPEED.search(line.Raw) if line.Kind == 'movej' else None
            if line.Kind == 'movej' and (speed is None or (float(speed.group(1)), float(speed.group(2)))
                                         != (float(CONST_MOVEJ_A_TOKEN), float(CONST_MOVEJ_V_TOKEN))):
                raise Configuration_Error_Cls('Segment %s: the taught line at program line %d moves at another a and v than the '
                                              'generated moves are written with (a=%s, v=%s, src/Planner/Core.py): %s. A job of '
                                              'another speed is not supported (KNOWN_ISSUES).'
                                              % (segment.Name, line.Line, CONST_MOVEJ_A_TOKEN, CONST_MOVEJ_V_TOKEN, line.Raw.strip()))
    return plan


def Segments(plan: Job_Plan_Str) -> tp.Dict[str, tp.Tuple[str, str]]:
    """
    Description:
        The ends of the job's generated segments.

    Args:
        (1) plan [Job_Plan_Str(object)]: The job.

    Returns:
        (1) parameter [Dict<string, Tuple<string, string>>]: Segment -> (from, to), in order.
    """

    return {segment.Name: (segment.From, segment.To) for segment in plan.Sequence.Segments}


def Opening(cfg: Cell_Configuration_Str, state: str, plan: Job_Plan_Str) -> float:
    """
    Description:
        The gripper opening of a job's case in a gripper state: the Tool.Holding_Openings entry of
        its program and case while the part is held, CONST_RELEASE_OPENING when open.

    Args:
        (1) cfg [Cell_Configuration_Str(object)]: The cell configuration.
        (2) state [string]: 'holding' or 'open'.
        (3) plan [Job_Plan_Str(object)]: The job.

    Returns:
        (1) parameter [float]: The inner pad-face gap (m).
    """

    if state == 'open':
        return CONST_RELEASE_OPENING
    key = (plan.Program_Name, plan.Job.Case)
    if key not in cfg.Holding_Openings:
        raise Configuration_Error_Cls('Tool.Holding_Openings has no entry for case %d of %s; add {Program: %s, Case: %d, Opening: <m>, '
                                      'Source: <text>} to the configuration.' % (key[1], key[0], key[0], key[1]))
    return cfg.Holding_Openings[key][0]


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


def Plan_Requests(cfg: Cell_Configuration_Str, plan: Job_Plan_Str, points: tp.Dict, station_q: tp.Sequence[tp.Sequence[float]],
                  penalty_weight: float, budget: tp.Optional[Planner_Budget_Str] = None) -> tp.List[Plan_Request_Str]:
    """
    Description:
        The planning requests of the job's segments: each at the opening of its gripper state,
        between the _q of its ends, with its own seed, seed * 1000 + case * 10 + k (k = 0 for the
        first).

    Args:
        (1) cfg [Cell_Configuration_Str(object)]: The cell configuration.
        (2) plan [Job_Plan_Str(object)]: The job.
        (3) points [Dict<string, Robot_Point_Str>]: The points the ends are taken from.
        (4) station_q [List<Vector<float> 1x6>]: The fixed station points (rad).
        (5) penalty_weight [float]: The clearance penalty weight (s/m).
        (6) budget [Planner_Budget_Str(object)]: The iteration budgets (none = the full budget).

    Returns:
        (1) parameter [List<Plan_Request_Str>]: One per segment, in order.
    """

    return [Plan_Request_Str('%s (%s -> %s)' % (s.Name, s.From, s.To), dataclasses.replace(cfg, Opening=Opening(cfg, s.State, plan)),
                             [np.asarray(q, dtype=float) for q in station_q], penalty_weight, budget or Planner_Budget_Str(),
                             np.asarray(points[s.From].Q, dtype=float), np.asarray(points[s.To].Q, dtype=float),
                             CONST_SEED * 1000 + plan.Job.Case * 10 + index)
            for index, s in enumerate(plan.Sequence.Segments)]


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


def Station_Duration(plan: Job_Plan_Str, points: tp.Dict) -> float:
    """
    Description:
        Estimated duration of the station operations (copied verbatim): every move of each station,
        from the end of the segment before it, with the line's own a and v - a movej by the trapezoid
        of its leading joint, a movel on the TCP line; gripper time excluded.

    Args:
        (1) plan [Job_Plan_Str(object)]: The job.
        (2) points [Dict<string, Robot_Point_Str>]: The points as written.

    Returns:
        (1) parameter [float]: The duration (s).
    """

    total = 0.0
    for segment, station in zip(plan.Sequence.Segments, plan.Sequence.Stations):
        previous = segment.To
        for line in station:
            if line.Kind not in ('movej', 'movel'):
                continue
            a, v = (float(token) for token in CONST_SPEED.search(line.Raw).groups())
            if line.Kind == 'movej':
                total += Estimate_Duration([points[previous].Q, points[line.Target].Q], a, v)
            else:
                total += Linear_Duration(points[previous].P, points[line.Target].P, a, v)
            previous = line.Target
    return total


def Sequence_Entries(plan: Job_Plan_Str, vias: tp.Dict[str, tp.List[str]]) -> tp.List[tp.Tuple[str, str]]:
    """
    Description:
        The optimised sequence as motion-sequence entries: the start at the first segment's From,
        each segment as generated movej to its via points and its end, and between two segments its
        station operation verbatim from the program (Program_Entries: moves as program lines, the
        gripper actions as comments). The program's own move to the start may blend (case 17:
        r=0.01) into the first segment, and a blend into a generated segment is not checked (SPEC
        8.5), so the start is the generated movej with r=0 (owner, 2026-10-18).

    Args:
        (1) plan [Job_Plan_Str(object)]: The job.
        (2) vias [Dict<string, List<string>>]: The via point names of each segment.

    Returns:
        (1) parameter [List<Tuple<string, string>>]: The entries in order.
    """

    sequence, first = plan.Sequence, plan.Sequence.Segments[0]
    blend = CONST_BLEND.search(sequence.Lines[0].Raw)
    start = ('Start at %s (generated, r=0): the program\'s line blends (r=%s) into %s, and a blend into a generated segment is '
             'not checked' % (first.From, blend.group(1), first.Name) if blend else
             'Start at %s (generated, r=0), as the program\'s line (no blend)' % first.From)
    entries = [('comment', start), ('movej', first.From)]
    for k, segment in enumerate(sequence.Segments):
        entries += ([('comment', '%s (generated): %s -> %s' % (segment.Name, segment.From, segment.To))]
                    + [('movej', name) for name in vias[segment.Name]] + [('movej', segment.To)])
        if k < len(sequence.Stations):
            entries += [('comment', 'Station: %s (verbatim from the program)' % plan.Job.Stations[k])] + Program_Entries(sequence.Stations[k])
    return entries


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


def Configured_Points(plan: Job_Plan_Str, cfg: Cell_Configuration_Str,
                      configurations: tp.Optional[tp.Sequence[tp.Tuple[str, int, int, str, str]]] = None) -> tp.Dict:
    """
    Description:
        The program's points with the job's Point_Configurations applied, in the order the output
        moves to them: a joint turn adds its whole turns to the taught joint; an IK branch takes the
        nominal-DH IK of the point's _p on that branch (Solve_IK_Analytic), turned to the
        representative nearest the _q of the point the output moves to just before it, as already
        configured (owner, 2026-10-21: the rule of the delivered c2's drop, SPEC 10.5). The _p tokens
        stay the program's (Replace_Joints). Refused: a branch the pose has no solution on, a branch
        at the output's first point, a configured joint outside the joint limits.

    Args:
        (1) plan [Job_Plan_Str(object)]: The job.
        (2) cfg [Cell_Configuration_Str(object)]: The cell configuration (the TCP and the joint limits).
        (3) configurations [List<Tuple<string, int, int, string, string>>]: Instead of the job's (a
                                                                           variant's), the same form.

    Returns:
        (1) parameter [Dict<string, Robot_Point_Str>]: A new dictionary; the program's points are not changed.
    """

    chosen = {c[0]: c for c in (plan.Job.Point_Configurations if configurations is None else configurations)}
    out = dict(plan.Points)
    previous = None
    for name in Learned_Order(Sequence_Entries(plan, {s.Name: [] for s in plan.Sequence.Segments}), plan.Sequence.Lines):
        if name in chosen:
            _, joint, turns, branch, _ = chosen[name]
            if branch:
                solutions = {b: q for b, q in Solve_IK_Analytic(Pose_To_Matrix(out[name].P), cfg.TCP_Offset)}
                if branch not in solutions:
                    raise Kinematics_Error_Cls('%s has no solution on branch %s (its branches: %s).' % (name, branch, ', '.join(sorted(solutions))))
                if previous is None:
                    raise Kinematics_Error_Cls('%s is the first point of the output; a branch is chosen nearest the point before it.' % name)
                q = Nearest_Representative(solutions[branch], np.asarray(out[previous].Q, dtype=float))
            else:
                q = list(out[name].Q)
                q[joint] += turns * 2.0 * np.pi
            for i, value in enumerate(q):
                if not cfg.Joint_Limits_Min[i] <= value <= cfg.Joint_Limits_Max[i]:
                    raise Kinematics_Error_Cls('%s configured has %s at %.3f rad, outside the joint limits [%.3f, %.3f].'
                                               % (name, CONST_JOINT_NAMES[i], value, cfg.Joint_Limits_Min[i], cfg.Joint_Limits_Max[i]))
            out[name] = Replace_Joints(out[name], q)
        previous = name
    return out


def Status_Lines(plan: Job_Plan_Str) -> tp.List[str]:
    """
    Description:
        The status header lines of the job's output: its Output.Status, then which learned points
        have another _q than taught and why (Point_Configurations, grouped by kind and source),
        wrapped at CONST_STATUS_WIDTH.

    Args:
        (1) plan [Job_Plan_Str(object)]: The job.

    Returns:
        (1) parameter [List<string>]: Comment lines.
    """

    groups: tp.Dict[tp.Tuple[int, int, str, str], tp.List[str]] = {}
    for point, joint, turns, branch, source in plan.Job.Point_Configurations:
        groups.setdefault((joint, turns, branch, source), []).append(point)
    parts = []
    for (joint, turns, branch, source), names in groups.items():
        if branch:
            parts.append('%s take IK branch %s, nearest the point before them (%s)' % (', '.join(names), branch, source))
        else:
            parts.append('%s use %s %s turn%s %s than taught (%s)' % (', '.join(names), CONST_JOINT_NAMES[joint],
                                                                      CONST_TURN_WORDS.get(abs(turns), str(abs(turns))),
                                                                      '' if abs(turns) == 1 else 's', 'lower' if turns < 0 else 'higher', source))
    text = plan.Job.Status + (' ' + '; '.join(parts) + '; same pose, same _p (SPEC 10.5).' if parts else '')
    return textwrap.wrap(text, width=CONST_STATUS_WIDTH, initial_indent='# ', subsequent_indent='# ', break_on_hyphens=False)


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


def Write_Outputs(out: str, cfg: Cell_Configuration_Str, plan: Job_Plan_Str, points: tp.Dict, generated: tp.Dict[str, tp.List[np.ndarray]],
                  penalty_weight: float, status: tp.Sequence[str], names: tp.Optional[tp.Tuple[str, str]] = None) -> None:
    """
    Description:
        Write the optimised sequence: the points file (the learned points it moves to, in the order
        of first use, then the generated via points via_<from>_<to>_<nn>) and the motion sequence
        (Sequence_Entries), named as the job's Output. Each header names the learned points written
        verbatim from the extract and those with the program's _p and another _q (Provenance); the
        status lines say why.

    Args:
        (1) out [string]: The output directory.
        (2) cfg [Cell_Configuration_Str(object)]: The cell configuration (the TCP of the informational _p).
        (3) plan [Job_Plan_Str(object)]: The job (the program's points and lines).
        (4) points [Dict<string, Robot_Point_Str>]: The points the paths were planned between (learned).
        (5) generated [Dict<string, List<Vector<float> 1x6>>]: The verified path of each segment, ends included (rad).
        (6) penalty_weight [float]: The clearance penalty weight the paths were planned with (s/m).
        (7) status [List<string>]: Comment lines saying what the output is and why points differ, after the description.
        (8) names [Tuple<string, string>]: The points and the sequence file names instead of the job's (a variant's).
    """

    segments = plan.Sequence.Segments
    points_name, sequence_name = names or (plan.Job.Output_Points, plan.Job.Output_Sequence)
    vias = {s.Name: ['via_%s_%s_%02d' % (s.From, s.To, k + 1) for k in range(len(generated[s.Name]) - 2)] for s in segments}
    via_points = [Make_Generated_Point(name, Matrix_To_Pose(TCP_Pose(q, cfg.TCP_Offset)), list(q))
                  for s in segments for name, q in zip(vias[s.Name], generated[s.Name][1:-1])]
    entries = Sequence_Entries(plan, vias)
    # The learned points of the optimised sequence, in the order of first use (the taught via points
    # of the segments are no longer used), then the generated via points.
    lines = plan.Sequence.Lines
    learned = Learned_Order(entries, lines)
    verbatim, other_q = Provenance(learned, points, plan.Points)
    source = os.path.relpath(plan.Positions, CONST_PROJECT_ROOT)
    span = '%s %s -> %s' % (plan.Job.Title, segments[0].From, segments[-1].To)
    names_text = segments[0].Name if len(segments) == 1 else '%s-%s' % (segments[0].Name, segments[-1].Name)
    header = (['# Case %d of %s, the %s, optimised: the learned points it' % (plan.Job.Case, plan.Program_File, span),
               '# moves to, in the order of first use, then the generated via points of %s (_p informational, the' % names_text,
               '# robot is driven by _q). Generated by Example/plan_rotation.py, seed %d, penalty weight %g s/m.'
               % (CONST_SEED, penalty_weight),
               '# Verbatim from %s (_p and _q): %s.' % (source, ', '.join(verbatim))]
              + (['# The program\'s _p, another _q: %s.' % ', '.join(other_q)] if other_q else [])
              + ['# ' + line for line in Machine_Margin(cfg)] + list(status)
              + ['# Simulation passed != safe on the robot: run it on the real cell at reduced speed first.'])
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, points_name), 'w', encoding='utf-8') as handle:
        handle.write(Write_URScript(Make_Document([points[name] for name in learned] + via_points, header)))
    sequence_header = ['# Case %d of %s, the %s, optimised: the station lines' % (plan.Job.Case, plan.Program_File, span),
                       '# verbatim from the program, the transfer segments %s generated movej (the program\'s a and v,' % names_text,
                       '# r=0). Points: %s.' % points_name]
    # The points of the station lines copied verbatim whose _q is not the program's.
    target = {line.Raw: line.Target for line in lines if line.Kind in ('movej', 'movel')}
    station = {target[value] for kind, value in entries if kind == 'program'}
    station_other_q = [name for name in other_q if name in station]
    if station_other_q:
        sequence_header += ['# The station lines to %s are the program\'s text; their points have the program\'s _p and'
                            % ', '.join(station_other_q), '# another _q (%s).' % points_name]
    sequence_header += ['# ' + line for line in Machine_Margin(cfg)] + list(status) + [
        '# Simulation passed != safe on the robot: run it at reduced speed first.']
    with open(os.path.join(out, sequence_name), 'w', encoding='utf-8') as handle:
        handle.write(Write_Motion_Sequence(entries, sequence_header, CONST_MOVEJ_A_TOKEN, CONST_MOVEJ_V_TOKEN))


def Taught_Moves(plan: Job_Plan_Str, cfg: Cell_Configuration_Str) -> tp.List[tp.Tuple[Program_Line_Str, str, tp.List[np.ndarray]]]:
    """
    Description:
        The taught moves of the job's sequence as the robot runs them, from its first segment's From
        (each line after the first): a movej joint-linear, a movel on its TCP line (Linear_Path), with
        the program's _q (the taught joints, no point configuration) and the gripper state of the move
        (the first segment's, switched by each gripper line).

    Args:
        (1) plan [Job_Plan_Str(object)]: The job.
        (2) cfg [Cell_Configuration_Str(object)]: The cell configuration (the TCP).

    Returns:
        (1) parameter [List<Tuple<Program_Line_Str, string, List<Vector<float> 1x6>>>]: Per move: its line,
                                                                                        its gripper state and
                                                                                        its configurations (rad).
    """

    state = plan.Sequence.Segments[0].State
    previous = np.asarray(plan.Points[plan.Sequence.Lines[0].Target].Q, dtype=float)
    moves = []
    for line in plan.Sequence.Lines[1:]:
        if line.Kind == 'gripper':
            state = 'open' if line.Target == 'open' else 'holding'
        elif line.Kind in ('movej', 'movel'):
            target = np.asarray(plan.Points[line.Target].Q, dtype=float)
            moves.append((line, state, [previous, target] if line.Kind == 'movej' else Linear_Path(previous, target, cfg.TCP_Offset)))
            previous = target
    return moves


def Check_Taught_Model(plan: Job_Plan_Str, cfg: Cell_Configuration_Str, scenes: tp.Dict[str, Scene_Cls]) -> tp.Tuple[tp.List[str], tp.List[str], int]:
    """
    Description:
        Replay the job's taught moves against the model under the taught rule (owner, 2026-10-21):
        the station zone, the station boxes and the workspace top are fitted to cases 17 and 18, so a
        case they were not fitted to is checked before it is planned. A sample inside modelled
        geometry (distance < 0: the real robot ran there) means the model is not valid for the case
        there: an error naming the line, the robot body, the piece and the depth. Less room than the
        taught margin, and a robot geom above the workspace top, are warnings. Nothing is refitted.

    Args:
        (1) plan [Job_Plan_Str(object)]: The job.
        (2) cfg [Cell_Configuration_Str(object)]: The cell configuration.
        (3) scenes [Dict<string, Scene_Cls>]: The scene of each gripper state the moves use.

    Returns:
        (1) parameter [Tuple<List<string>, List<string>, int>]: The errors, the warnings (one sentence
                                                                 each, per move) and the samples checked.
    """

    errors, warnings, count = [], [], 0
    for line, state, configurations in Taught_Moves(plan, cfg):
        scene = scenes[state]
        samples: tp.List[np.ndarray] = []
        for a, b in zip(configurations[:-1], configurations[1:]):
            found = list(scene.Segment_Samples(a, b))
            samples += found if not samples else found[1:]
        count += len(samples)
        closest, closest_pair, room, room_pair, margin = CONST_REPORT_DISTMAX, '', CONST_REPORT_DISTMAX, '', 0.0
        above = (0.0, '')
        for q in samples:
            c, c_pair, r, r_pair, r_margin = scene.Environment_Clearance(q, 'taught', None, CONST_REPORT_DISTMAX)
            if c < closest:
                closest, closest_pair = c, c_pair
            if r < room:
                room, room_pair, margin = r, r_pair, r_margin
            for v in scene.Workspace_Violations(q):
                if -v.Distance > above[0]:
                    above = (-v.Distance, v.Body_1)
        where = 'the taught %s to %s at program line %d (gripper %s)' % (line.Kind, line.Target, line.Line, state)
        if closest < 0.0:
            fitted = ' - geometry fitted to the taught samples of cases 17 and 18, not measured' if any(
                f in closest_pair for f in CONST_FITTED) else ''
            errors.append('The model is not valid for case %d of %s: %s passes %.1f mm into %s%s.'
                          % (plan.Job.Case, plan.Program_Name, where, -closest * 1e3, closest_pair, fitted))
        elif room < 0.0:
            warnings.append('%s keeps %.1f mm to %s, below its taught margin of %.1f mm.'
                            % (where[0].upper() + where[1:], (room + margin) * 1e3, room_pair, margin * 1e3))
        if above[0] > 0.0:
            warnings.append('%s reaches %.1f mm beyond Clearance.Workspace (%s): the workspace was fitted to case 17\'s taught '
                            'sequence, so a generated path may not use that space.' % (where[0].upper() + where[1:], above[0] * 1e3, above[1]))
    return errors, warnings, count


def Plan_Job(plan: Job_Plan_Str, cfg: Cell_Configuration_Str, out: str, penalty_weight: float, workers: int,
             budget: tp.Optional[Planner_Budget_Str] = None) -> tp.List[str]:
    """
    Description:
        Plan the job's segments, verify each independently, write the outputs (Write_Outputs) and
        return the report: taught vs optimised per segment and the totals over the sequence.

    Args:
        (1) plan [Job_Plan_Str(object)]: The job.
        (2) cfg [Cell_Configuration_Str(object)]: The cell configuration.
        (3) out [string]: The output directory.
        (4) penalty_weight [float]: The clearance penalty weight (s/m).
        (5) workers [int]: Planning processes (1 = this process); the output does not depend on it.
        (6) budget [Planner_Budget_Str(object)]: The iteration budgets (none = the full budget).

    Returns:
        (1) parameter [List<string>]: The report lines, without their [INFO] prefix.
    """

    points = Configured_Points(plan, cfg)
    station_q = [points[name].Q for name in plan.Sequence.Station_Points]
    started = time.time()
    requests = Plan_Requests(cfg, plan, points, station_q, penalty_weight, budget)
    if workers > 1:
        with Make_Pool(workers) as pool:
            results = Plan_All(requests, pool)
    else:
        results = Plan_All(requests)
    planned = time.time() - started
    openings = {s.Name: Opening(cfg, s.State, plan) for s in plan.Sequence.Segments}
    scenes = {opening: Scene_Cls(dataclasses.replace(cfg, Opening=opening)) for opening in sorted(set(openings.values()))}
    rows, generated = [], {}
    for segment, result in zip(plan.Sequence.Segments, results):
        scene = scenes[openings[segment.Name]]
        taught = [np.asarray(plan.Points[n].Q) for n in [segment.From] + segment.Taught_Vias + [segment.To]]
        generated[segment.Name] = result.Path
        rows.append((segment.Name, Measure(scene, taught, station_q), Measure(scene, result.Path, station_q), result))
    # Outputs: the points and the sequence, written only after every segment is verified.
    Write_Outputs(out, cfg, plan, points, generated, penalty_weight, Status_Lines(plan))
    # Report.
    report = ['Case %d (estimated durations: trapezoid on the leading joint, a=%s, v=%s, blends and gripper excluded;'
              % (plan.Job.Case, CONST_MOVEJ_A_TOKEN, CONST_MOVEJ_V_TOKEN),
              '  "far": outside the 50 mm station neighbourhood, where the penalty counts):',
              '  %-3s %-9s %4s %8s %9s %9s %9s %-25s %10s %-25s %9s %4s %s'
              % ('S', 'path', 'vias', 'T_est/s', 'sum|dq|', 'max-jnt', 'min d/mm', 'robot body', 'far d/mm', 'robot body (far)',
                 'penalty/m', 'viol', 'origin')]
    totals = {'taught': 0.0, 'optimised': 0.0}
    for segment, before, after, result in rows:
        for label, m in (('taught', before), ('optimised', after)):
            totals[label] += m['duration']
            report.append('  %-3s %-9s %4d %8.2f %9.3f %9.3f %9.1f %-25s %10.1f %-25s %9.3f %4d %s'
                          % (segment, label, m['vias'], m['duration'], m['sum'], m['max'], m['closest'] * 1e3, m['body'] or '-',
                             m['far'] * 1e3, m['far_body'] or '-', m['penalty'], m['violations'],
                             result.Origin if label == 'optimised' else ''))
    fixed = Station_Duration(plan, points)
    segments = plan.Sequence.Segments
    report.append('  %s -> %s: taught %.2f s, optimised %.2f s (%s + station operations %.2f s; estimate)'
                  % (segments[0].From, segments[-1].To, totals['taught'] + fixed, totals['optimised'] + fixed,
                     segments[0].Name if len(segments) == 1 else '%s-%s' % (segments[0].Name, segments[-1].Name), fixed))
    report.append('Wall time: planning %.0f s (%d worker%s), report %.0f s.'
                  % (planned, workers, '' if workers == 1 else 's', time.time() - started - planned))
    return report


def main():
    """
    Description:
        Plan the generated segments of a job (Configuration/Jobs/<name>.yaml; the default: case 17
        of sf1_plc_v4.script, home -> home), verify each independently, write its points file and
        its motion sequence (the job's Output names) into Data/Generated/, and print the report:
        taught vs optimised per segment and the totals over the sequence.

            .venv/bin/python Example/plan_rotation.py [--job NAME] [--workers N] [--penalty-weight W] [--out DIR]

        --workers N plans the segments at once in N spawned processes (each with its own scene); the
        output is byte-identical for every N (Plan_All). 1 (default) plans in this process.
    """

    parser = argparse.ArgumentParser(description='Plan the transfer segments of a job (default: the rotation sequence of case 17).')
    parser.add_argument('--job', default=CONST_DEFAULT_JOB, help='the job, Configuration/Jobs/<NAME>.yaml or a .yaml file (default: %(default)s)')
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
        plan = Load_Job_Plan(args.job)
        for level, message in plan.Job.Notes:
            print('[%s] %s' % (level, message))
        for segment in plan.Sequence.Segments:
            Opening(cfg, segment.State, plan)
        Check_Learned(cfg, plan.Points, os.path.basename(plan.Positions))
        Configured_Points(plan, cfg)
    except (Configuration_Error_Cls, URScript_Error_Cls, Kinematics_Error_Cls, OSError) as error:
        print('[ERROR] %s' % error)
        return 1
    states = {s.State for s in plan.Sequence.Segments}
    print('[INFO] Case %d: %s.' % (plan.Job.Case, ', '.join(
        (['holding opening %.1f mm (%s)' % (cfg.Holding_Openings[(plan.Program_Name, plan.Job.Case)][0] * 1e3,
                                             cfg.Holding_Openings[(plan.Program_Name, plan.Job.Case)][1])] if 'holding' in states else [])
        + (['open %.1f mm' % (CONST_RELEASE_OPENING * 1e3)] if 'open' in states else []))))
    for line in Machine_Margin(cfg):
        print('[INFO] %s' % line)
    # The model against the case's taught moves before anything is planned (Check_Taught_Model).
    started = time.time()
    try:
        scenes = {state: Scene_Cls(dataclasses.replace(cfg, Opening=Opening(cfg, state, plan)))
                  for state in sorted({state for _, state, _ in Taught_Moves(plan, cfg)})}
        errors, warnings, samples = Check_Taught_Model(plan, cfg, scenes)
    except (Configuration_Error_Cls, Kinematics_Error_Cls) as error:
        print('[ERROR] %s' % error)
        return 1
    for line in warnings:
        print('[WARNING] %s' % line)
    if errors:
        for line in errors:
            print('[ERROR] %s' % line)
        print('[ERROR] The model does not describe case %d of %s where the taught program ran. Nothing is planned, and nothing '
              'is refitted: check the cell geometry there (KNOWN_ISSUES, SPEC 9.7).' % (plan.Job.Case, plan.Program_Name))
        return 1
    print('[INFO] Taught model check: %d taught moves of case %d (%d samples, %.0f s) pass through no modelled geometry; %d warning(s).'
          % (len(Taught_Moves(plan, cfg)), plan.Job.Case, samples, time.time() - started, len(warnings)))
    try:
        report = Plan_Job(plan, cfg, args.out, args.penalty_weight, args.workers)
    except Planner_Error_Cls as error:
        print('[ERROR] %s Nothing is written.' % error)
        return 1
    for line in report:
        print('[INFO] %s' % line)
    print('[INFO] Written to %s. Simulation passed != safe on the robot.' % os.path.relpath(args.out, CONST_PROJECT_ROOT))
    return 0


if __name__ == '__main__':
    sys.exit(main())
