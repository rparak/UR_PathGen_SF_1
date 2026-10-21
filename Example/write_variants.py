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
# File Name: ../Example/write_variants.py
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
# CSV (CSV file reading), for reading comparison.csv back
import csv
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
from Configuration.Core import Load_Configuration, Cell_Configuration_Str, Configuration_Error_Cls, CONST_DEFAULT_CONFIG_FILE, CONST_PROJECT_ROOT
#   ../URScript/Core — the program extracts and the writers
from URScript.Core import (Read_Motion_Sequence, Program_Entries, Make_Document, Write_URScript, Write_Motion_Sequence, Motion_Str,
                           URScript_Error_Cls)
#   ../Kinematics/Core — the TCP
from Kinematics.Core import TCP_Pose, Kinematics_Error_Cls
#   ../Scene/Core — the scene of each gripper state
from Scene.Core import Scene_Cls
#   ../Planner/Core — the planner
from Planner.Core import Plan_All, Make_Pool, Planner_Error_Cls
#   ../Example/plan_rotation — the default job (case 17), its configured points and the writer
from plan_rotation import (CONST_DEFAULT_JOB, CONST_C1_JOB, CONST_PENALTY_WEIGHT, CONST_VARIANT_DIR, CONST_VARIANTS, CONST_FILE_NAMES,
                           CONST_REPORT_DISTMAX, Job_Plan_Str, Load_Job_Plan, Opening, Check_Learned, Plan_Requests, Configured_Points, Write_Outputs)
#   ../Example/replay_rotation — the variant files and their moves, as replayed
from replay_rotation import Load_Variant, Build_Moves, Run_Floors, Move_Samples, Move_Str
#   ../Example/write_taught_programs — the points a case moves to
from write_taught_programs import Case_Targets

"""
Description:
    Initialization of constants.
"""
# The drop branch of the proposal for re-teaching (Solve_IK_Analytic's labels, measured 2026-10-19):
# c3 keeps home's shoulder, elbow and wrist. Its pick is c1's. c2, which keeps home's shoulder and
# elbow and flips the wrist (S- E- W-), is the default job since it was delivered (owner, 2026-10-21).
#   Unit [Dict<string, string>]
CONST_PROPOSAL_BRANCHES = {'c3': 'S- E- W+'}
# The drop operation's points in order: each proposal point is the IK solution of its _p on the
# branch nearest the point before it (drop_rotate_1: nearest home; Configured_Points).
#   Unit [Tuple<string>]
CONST_DROP_POINTS = ('drop_rotate_1', 'Waypoint_4', 'Waypoint_5')
# The sections of home -> home in order, and the target each one ends with.
#   Unit [Tuple<Tuple<string, string>>]
CONST_SECTIONS = (('S1', 'drop_rotate_1'), ('drop', 'Waypoint_5'), ('S2', 'Waypoint_18'), ('pick', 'Waypoint_6'), ('S3', 'home'))
# The safety line of the taught variant's files (Write_Outputs adds its own to the planned variants').
#   Unit [string]
CONST_SAFETY_LINE = '# Simulation passed != safe on the robot: run it on the real cell at reduced speed first.'
# The comparison table's files.
#   Unit [string]
CONST_TABLE_CSV, CONST_TABLE_MD = 'comparison.csv', 'comparison.md'
# The columns of comparison.csv, in order.
#   Unit [Tuple<string>]
CONST_COLUMNS = ('variant', 'section', 'motion_lines', 'distinct_points', 'vias', 't_est_s', 'sum_abs_dq_rad', 'max_joint_rad',
                 'tcp_length_m', 'max_tcp_z_m', 'max_robot_z_m', 'closest_mm', 'closest_pair', 'least_room_mm', 'least_room_pair',
                 'least_room_margin_mm', 'violating_samples')


def Proposal_Configurations(plan: Job_Plan_Str, branch: str) -> tp.List[tp.Tuple[str, int, int, str, str]]:
    """
    Description:
        A proposal's point configurations: the c1 job's (the pick's wrist 3 one turn lower) and the
        drop's three on another IK branch, each nearest the point before it (Configured_Points).
        The _p tokens stay the program's; the _q are generated.

    Args:
        (1) plan [Job_Plan_Str(object)]: The c1 job.
        (2) branch [string]: The drop's branch, e.g. 'S- E- W-'.

    Returns:
        (1) parameter [List<Tuple<string, int, int, string, string>>]: The configurations (Job_Configuration_Str form).
    """

    return list(plan.Job.Point_Configurations) + [(name, -1, 0, branch, 'proposal for re-teaching (owner, 2026-10-19)')
                                                  for name in CONST_DROP_POINTS]


def Variant_Points(plan: Job_Plan_Str, c1: Job_Plan_Str, cfg: Cell_Configuration_Str) -> tp.Dict[str, tp.Dict]:
    """
    Description:
        The points of every planned variant: b_optimised the program's (as before 2026-10-19), c1
        the c1 job's (the pick's wrist 3 one turn lower), c2 the default job's (the drop on S- E- W-,
        delivered), c3 the drop on another branch with c1's pick.

    Args:
        (1) plan [Job_Plan_Str(object)]: The default job (c2).
        (2) c1 [Job_Plan_Str(object)]: The c1 job.
        (3) cfg [Cell_Configuration_Str(object)]: The cell configuration.

    Returns:
        (1) parameter [Dict<string, Dict<string, Robot_Point_Str>>]: Variant -> its points.
    """

    out = {'b_optimised': Configured_Points(plan, cfg, []), 'c1': Configured_Points(c1, cfg), 'c2': Configured_Points(plan, cfg)}
    for variant, branch in CONST_PROPOSAL_BRANCHES.items():
        out[variant] = Configured_Points(c1, cfg, Proposal_Configurations(c1, branch))
    return out


def Write_Taught(folder: str, plan: Job_Plan_Str, status: tp.Sequence[str]) -> None:
    """
    Description:
        The taught variant: the points the taught sequence moves to (the program's tokens, in the
        order of first use) and every movej and movel of the sequence verbatim, the gripper actions
        as comments - the simplified taught program in the variants' file names, with its status.

    Args:
        (1) folder [string]: The variant's directory.
        (2) plan [Job_Plan_Str(object)]: The job (the program's points and the sequence's lines).
        (3) status [List<string>]: The status header lines.
    """

    points, lines, case = plan.Points, plan.Sequence.Lines, plan.Job.Case
    names = Case_Targets(lines)
    os.makedirs(folder, exist_ok=True)
    header = ['# Case %d of %s as taught, home -> home: the points it moves to, verbatim from' % (case, plan.Program_File),
              '# %s. Written by Example/write_variants.py.' % os.path.relpath(plan.Positions, CONST_PROJECT_ROOT)] + list(status) + [
              CONST_SAFETY_LINE]
    with open(os.path.join(folder, CONST_FILE_NAMES[0]), 'w', encoding='utf-8') as handle:
        handle.write(Write_URScript(Make_Document([points[name] for name in names], header)))
    header = ['# Case %d of %s as taught, home -> home: every movej and movel in program order, verbatim' % (case, plan.Program_File),
              '# (targets, a, v, r), the gripper actions as comments. Points: %s.' % CONST_FILE_NAMES[0]] + list(status) + [
              CONST_SAFETY_LINE]
    with open(os.path.join(folder, CONST_FILE_NAMES[1]), 'w', encoding='utf-8') as handle:
        handle.write(Write_Motion_Sequence(Program_Entries(lines), header, '', ''))


def Variant_Moves(folder: str, cfg: Cell_Configuration_Str) -> tp.Tuple[tp.List[Move_Str], tp.Dict, tp.List]:
    """
    Description:
        The moves of a written variant, as the replay builds them (Build_Moves): movej joint-linear,
        movel on its TCP line, every move timed by its estimate with the line's own a and v.

    Args:
        (1) folder [string]: The variant's directory.
        (2) cfg [Cell_Configuration_Str(object)]: The cell configuration.

    Returns:
        (1) parameter [Tuple<List<Move_Str>, Dict<string, Robot_Point_Str>, List<Motion_Str>>]: The
                                                                                               moves (the
                                                                                               start
                                                                                               excluded),
                                                                                               the points
                                                                                               and the
                                                                                               operations.
    """

    operations, points = Load_Variant(folder)
    return Build_Moves(operations, points, cfg), points, operations


def Section_Of(moves: tp.Sequence[Move_Str]) -> tp.List[str]:
    """
    Description:
        The section of each move: S1 up to the move to drop_rotate_1, the drop up to Waypoint_5, S2
        up to Waypoint_18, the pick up to Waypoint_6, S3 to home.

    Args:
        (1) moves [List<Move_Str>]: The moves in order.

    Returns:
        (1) parameter [List<string>]: The section of each.
    """

    out, k = [], 0
    for move in moves:
        out.append(CONST_SECTIONS[k][0])
        if move.Kind != 'gripper' and move.Target == CONST_SECTIONS[k][1] and k < len(CONST_SECTIONS) - 1:
            k += 1
    return out


def Measure_Variant(folder: str, cfg: Cell_Configuration_Str, scenes: tp.Dict[str, Scene_Cls], plan: Job_Plan_Str) -> tp.List[tp.Dict]:
    """
    Description:
        The comparison rows of a written variant, per section and in total. Per move: the motion
        lines, the distinct points it moves to, the via points (a transfer section's moves before its
        end), the estimated duration (Build_Moves: rest to rest, the line's a and v, r = 0; blends not
        modelled), the joint path length sum |dq| and the max-joint length (a movel along its solved
        line), and over its checked samples (Move_Samples, as the replay checks them): the TCP path
        length, the largest TCP and robot heights (world z), the closest robot - environment pair and
        the pair with the least room above the margin of the move's rule (Environment_Clearance), and
        the samples that break the move's rule (Check_Configuration, the station-box floors of its
        generated run included).

    Args:
        (1) folder [string]: The variant's directory.
        (2) cfg [Cell_Configuration_Str(object)]: The cell configuration.
        (3) scenes [Dict<string, Scene_Cls>]: The scene of each gripper state ('holding', 'open').
        (4) plan [Job_Plan_Str(object)]: The job (its station points).

    Returns:
        (1) parameter [List<Dict>]: One row per section, then the total.
    """

    moves, points, operations = Variant_Moves(folder, cfg)
    station_q = [points[name].Q for name in plan.Sequence.Station_Points]
    tcps = scenes['holding'].Station_TCPs(station_q)
    floors = Run_Floors(moves, scenes)
    sections = Section_Of(moves)
    rows = [Section_Row(section, [(index, move) for index, move in enumerate(moves) if sections[index] == section], cfg, scenes,
                        tcps, floors)
            for section, _ in CONST_SECTIONS]
    return rows + [Total_Row(rows, operations)]


def Section_Row(section: str, moves: tp.Sequence[tp.Tuple[int, Move_Str]], cfg: Cell_Configuration_Str,
                scenes: tp.Dict[str, Scene_Cls], tcps: np.ndarray, floors: tp.Dict[int, tp.Dict[str, float]]) -> tp.Dict:
    """
    Description:
        The comparison row of one section (Measure_Variant): its moves' counts, estimates and lengths,
        and the measures over their checked samples.

    Args:
        (1) section [string]: The section (CONST_SECTIONS).
        (2) moves [List<Tuple<int, Move_Str>>]: The section's moves with their index among all moves, in order.
        (3) cfg [Cell_Configuration_Str(object)]: The cell configuration (the TCP).
        (4) scenes [Dict<string, Scene_Cls>]: The scene of each gripper state.
        (5) tcps [Matrix<float> Nx3]: The station points' TCPs (Station_TCPs).
        (6) floors [Dict<int, Dict<string, float>>]: The station-box floors of each generated move (Run_Floors).

    Returns:
        (1) parameter [Dict]: The row.
    """

    frame = scenes['holding'].Base_Frame()
    row = {'section': section, 'motion_lines': 0, 'targets': [], 't_est_s': 0.0, 'sum_abs_dq_rad': 0.0, 'max_joint_rad': 0.0,
           'tcp_length_m': 0.0, 'max_tcp_z_m': -np.inf, 'max_robot_z_m': -np.inf, 'closest': (CONST_REPORT_DISTMAX, ''),
           'room': (CONST_REPORT_DISTMAX, '', 0.0), 'violating_samples': 0}
    for index, move in moves:
        if move.Kind == 'gripper':
            continue
        scene = scenes[move.State]
        row['motion_lines'] += 1
        row['targets'].append(move.Target)
        row['t_est_s'] += move.Duration
        steps = np.abs(np.diff(np.asarray(move.Configurations), axis=0))
        row['sum_abs_dq_rad'] += float(steps.sum())
        row['max_joint_rad'] += float(steps.sum(axis=0).max())
        last = None
        for q in Move_Samples(scene, move):
            tcp = (frame @ TCP_Pose(q, cfg.TCP_Offset))[:3, 3]
            if last is not None:
                row['tcp_length_m'] += float(np.linalg.norm(tcp - last))
            last = tcp
            row['max_tcp_z_m'] = max(row['max_tcp_z_m'], float(tcp[2]))
            row['max_robot_z_m'] = max(row['max_robot_z_m'], scene.Robot_Top(q))
            closest, pair, room, room_pair, margin = scene.Environment_Clearance(q, move.Rule, tcps, CONST_REPORT_DISTMAX)
            if closest < row['closest'][0]:
                row['closest'] = (closest, pair)
            if room < row['room'][0]:
                row['room'] = (room, room_pair, margin)
            if scene.Check_Configuration(q, move.Rule, tcps if move.Rule == 'generated' else None, floors.get(index)):
                row['violating_samples'] += 1
    row['distinct_points'] = len(set(row['targets']))
    row['vias'] = row['motion_lines'] - 1 if section.startswith('S') else ''
    return row


def Total_Row(rows: tp.Sequence[tp.Dict], operations: tp.Sequence[Motion_Str]) -> tp.Dict:
    """
    Description:
        The total row of a variant (Measure_Variant): sums and extremes of its section rows; the
        motion lines include the start, the distinct points are those of the whole sequence.

    Args:
        (1) rows [List<Dict>]: The section rows (Section_Row).
        (2) operations [List<Motion_Str>]: The variant's operations.

    Returns:
        (1) parameter [Dict]: The row.
    """

    total = {'section': 'total', 'motion_lines': sum(r['motion_lines'] for r in rows) + 1,
             'distinct_points': len({o.Target for o in operations if o.Kind != 'gripper'}),
             'vias': sum(r['vias'] for r in rows if r['vias'] != ''),
             'closest': min((r['closest'] for r in rows), key=lambda c: c[0]),
             'room': min((r['room'] for r in rows), key=lambda c: c[0])}
    for key in ('t_est_s', 'sum_abs_dq_rad', 'max_joint_rad', 'tcp_length_m', 'violating_samples'):
        total[key] = sum(r[key] for r in rows)
    for key in ('max_tcp_z_m', 'max_robot_z_m'):
        total[key] = max(r[key] for r in rows)
    return total


def Format_Row(variant: str, row: tp.Dict) -> tp.Dict[str, str]:
    """
    Description:
        A comparison row as written: durations to 0.01 s, joint lengths to 0.001 rad, lengths and
        heights to 0.001 m, distances to 0.1 mm.

    Args:
        (1) variant [string]: The variant.
        (2) row [Dict]: A row of Measure_Variant.

    Returns:
        (1) parameter [Dict<string, string>]: Column -> text (CONST_COLUMNS).
    """

    return {'variant': variant, 'section': row['section'], 'motion_lines': str(row['motion_lines']),
            'distinct_points': str(row['distinct_points']), 'vias': str(row['vias']), 't_est_s': '%.2f' % row['t_est_s'],
            'sum_abs_dq_rad': '%.3f' % row['sum_abs_dq_rad'], 'max_joint_rad': '%.3f' % row['max_joint_rad'],
            'tcp_length_m': '%.3f' % row['tcp_length_m'], 'max_tcp_z_m': '%.3f' % row['max_tcp_z_m'],
            'max_robot_z_m': '%.3f' % row['max_robot_z_m'], 'closest_mm': '%.1f' % (row['closest'][0] * 1e3),
            'closest_pair': row['closest'][1], 'least_room_mm': '%.1f' % (row['room'][0] * 1e3), 'least_room_pair': row['room'][1],
            'least_room_margin_mm': '%.0f' % (row['room'][2] * 1e3), 'violating_samples': str(row['violating_samples'])}


def Table_CSV(table: tp.Dict[str, tp.List[tp.Dict]]) -> str:
    """
    Description:
        comparison.csv: one line per variant and section (CONST_COLUMNS); no field holds a comma.

    Args:
        (1) table [Dict<string, List<Dict>>]: Variant -> its rows (Measure_Variant).

    Returns:
        (1) parameter [string]: The file content.
    """

    out = [','.join(CONST_COLUMNS)]
    for variant, rows in table.items():
        for row in rows:
            values = Format_Row(variant, row)
            if any(',' in values[c] for c in CONST_COLUMNS):
                raise ValueError('A comparison value holds a comma: %s.' % values)
            out.append(','.join(values[c] for c in CONST_COLUMNS))
    return '\n'.join(out) + '\n'


def Read_Table_CSV(path: str) -> tp.Dict[str, tp.Dict[str, tp.Dict[str, str]]]:
    """
    Description:
        Read comparison.csv as Table_CSV writes it: the values as written (strings), per variant and
        section. A file whose header is not CONST_COLUMNS is refused.

    Args:
        (1) path [string]: The file.

    Returns:
        (1) parameter [Dict<string, Dict<string, Dict<string, string>>>]: Variant -> section -> column -> value.
    """

    with open(path, encoding='utf-8', newline='') as handle:
        rows = list(csv.reader(handle))
    if not rows or tuple(rows[0]) != CONST_COLUMNS:
        raise ValueError('%s does not have the columns of comparison.csv; rewrite it with Example/write_variants.py --table-only.' % path)
    table: tp.Dict[str, tp.Dict[str, tp.Dict[str, str]]] = {}
    for row in rows[1:]:
        values = dict(zip(CONST_COLUMNS, row))
        table.setdefault(values['variant'], {})[values['section']] = values
    return table


def Table_Markdown(table: tp.Dict[str, tp.List[tp.Dict]], blends: tp.Sequence[tp.Tuple[str, float]], base_z: float, case: int) -> str:
    """
    Description:
        comparison.md: the variants and their status, the home -> home totals, then one table per
        section, the variants as columns; what is estimated and what is not modelled is said first.

    Args:
        (1) table [Dict<string, List<Dict>>]: Variant -> its rows (Measure_Variant).
        (2) blends [List<Tuple<string, float>>]: The taught program's blended lines (target, r).
        (3) base_z [float]: The robot base height (Robot.Base_Position z) (m).
        (4) case [int]: The case.

    Returns:
        (1) parameter [string]: The file content.
    """

    variants = list(table)
    formatted = {v: {row['section']: Format_Row(v, row) for row in rows} for v, rows in table.items()}
    metrics = (('motion_lines', 'Motion lines'), ('distinct_points', 'Distinct points'), ('vias', 'Via points'),
               ('t_est_s', 'Estimated duration [s]'), ('sum_abs_dq_rad', 'Joint path Σ\\|Δq\\| [rad]'), ('max_joint_rad', 'Max-joint length [rad]'),
               ('tcp_length_m', 'TCP path length [m]'), ('max_tcp_z_m', 'Max TCP height [m]'), ('max_robot_z_m', 'Max robot height [m]'),
               ('closest', 'Closest robot - environment'), ('room', 'Least room above the rule\'s margin'),
               ('violating_samples', 'Samples breaking the rule'))
    def Cell(v: str, section: str, key: str) -> str:
        """
        Description:
            One cell of a section table.

        Args:
            (1) v [string]: The variant.
            (2) section [string]: The section.
            (3) key [string]: The metric.

        Returns:
            (1) parameter [string]: The text.
        """

        f = formatted[v][section]
        if key == 'closest':
            return '%s mm, %s' % (f['closest_mm'], f['closest_pair'] or '-')
        if key == 'room':
            return '%s mm, %s (margin %s mm)' % (f['least_room_mm'], f['least_room_pair'] or '-', f['least_room_margin_mm'])
        return f[key] or '-'
    out = ['# Case %d home -> home: the variants compared' % case, '',
           'Written by `Example/write_variants.py` from the files in this directory; `comparison.csv` holds the same',
           'numbers. **Simulation passed ≠ safe on the robot.**', '',
           '| Variant | Status |', '|---|---|']
    out += ['| %s | %s |' % (v, CONST_VARIANTS[v]) for v in variants]
    out += ['', 'How the numbers are made:', '',
            '- **Estimated duration**: an estimate. Every move from rest to rest (r = 0 everywhere): the trapezoid of the',
            '  leading joint for a movej, of the TCP path for a movel, with the line\'s own a and v; gripper time and waits',
            '  excluded. The taught program\'s blends are **not modelled**: %s.'
            % ', '.join('%s (r=%g)' % (t, r) for t, r in blends),
            '- **Motion lines** include the start at home; **distinct points** are the targets moved to; **via points** are a',
            '  transfer section\'s moves before its end.',
            '- **Lengths**: Σ|Δq| over all joints and the max-joint length per move, a movel along its solved TCP line; the',
            '  TCP path length over the checked samples (no robot point moves more than 5 mm between two).',
            '- **Heights**: world z (the robot base stands at %.4f m); the robot height is its highest collision vertex,' % base_z,
            '  the gripper included.',
            '- **Clearance**: robot - environment, capped at %.0f mm. Each move by the rule of its kind: generated movej by'
            % (CONST_REPORT_DISTMAX * 1e3),
            '  the generated rule, program lines by the taught rule; the least room is the distance minus that pair\'s',
            '  margin. The station-box floors and the workspace count in the samples breaking the rule only.',
            '- c2 and c3 drop points are the nominal-DH IK of the program\'s `_p`: millimetres from what the robot\'s',
            '  calibrated IK returns. c2 was delivered and tested on the real cell (owner, 2026-10-21); c3 must be',
            '  re-taught and re-verified on the robot at reduced speed.', '',
            '## Home -> home', '']
    sections = [s for s, _ in CONST_SECTIONS] + ['total']
    for section in ['total'] + sections[:-1]:
        if section != 'total':
            out += ['## %s' % section, '']
        out += ['| | %s |' % ' | '.join(variants), '|---|%s' % ('---|' * len(variants))]
        for key, title in metrics:
            if key == 'vias' and section in ('drop', 'pick'):
                continue
            out.append('| %s | %s |' % (title, ' | '.join(Cell(v, section, key) for v in variants)))
        out.append('')
    return '\n'.join(out)



def Blends(folder: str) -> tp.List[tp.Tuple[str, float]]:
    """
    Description:
        The blended lines of a written variant's sequence: (target, r) of every move with r > 0.

    Args:
        (1) folder [string]: The variant's directory.

    Returns:
        (1) parameter [List<Tuple<string, float>>]: The lines in order.
    """

    with open(os.path.join(folder, CONST_FILE_NAMES[1]), encoding='utf-8') as handle:
        return [(o.Target, o.R) for o in Read_Motion_Sequence(handle.read()) if o.Kind != 'gripper' and o.R > 0.0]


def Write_Table(out: str, cfg: Cell_Configuration_Str, scenes: tp.Dict[str, Scene_Cls], plan: Job_Plan_Str) -> tp.Dict[str, tp.List[tp.Dict]]:
    """
    Description:
        Measure every written variant (Measure_Variant) and write comparison.csv and comparison.md.

    Args:
        (1) out [string]: The variants directory.
        (2) cfg [Cell_Configuration_Str(object)]: The cell configuration.
        (3) scenes [Dict<string, Scene_Cls>]: The scene of each gripper state.
        (4) plan [Job_Plan_Str(object)]: The job (its case and station points).

    Returns:
        (1) parameter [Dict<string, List<Dict>>]: Variant -> its rows.
    """

    table = {variant: Measure_Variant(os.path.join(out, variant), cfg, scenes, plan) for variant in CONST_VARIANTS}
    with open(os.path.join(out, CONST_TABLE_CSV), 'w', encoding='utf-8') as handle:
        handle.write(Table_CSV(table))
    with open(os.path.join(out, CONST_TABLE_MD), 'w', encoding='utf-8') as handle:
        handle.write(Table_Markdown(table, Blends(os.path.join(out, 'taught')), cfg.Base_Position[2], plan.Job.Case))
    return table


def main():
    """
    Description:
        Write every variant of case 17 home -> home into Data/Generated/variants/<name>/ (the points
        SF1_Rotation_17.urscript and the sequence SF1_Rotation_17_Sequence.script, each with its
        status), then comparison.csv and comparison.md. taught: the program's sequence. b_optimised,
        c1, c2, c3: S1 - S3 planned as Example/plan_rotation.py plans them (the same seeds, budget and
        penalty weight), on the program's points (b_optimised, as before 2026-10-19), the points with
        the pick's wrist 3 one turn lower (c1, the job sf1_plc_v4_case_17_c1) and the drop on another IK
        branch (c2, the default job, = Data/Generated/, delivered and tested on the real cell; c3, a
        proposal for re-teaching). Nothing is written
        unless every segment is planned and verified.

            .venv/bin/python Example/write_variants.py [--workers N] [--out DIR]
    """

    parser = argparse.ArgumentParser(description='Write the variants of case 17 home -> home and their comparison table.')
    parser.add_argument('--config', default=CONST_DEFAULT_CONFIG_FILE, help='the cell configuration (default: %(default)s)')
    parser.add_argument('--out', default=CONST_VARIANT_DIR, help='output directory (default: %(default)s)')
    parser.add_argument('--workers', type=int, default=1, help='planning processes; the output does not depend on it (default: 1)')
    parser.add_argument('--table-only', action='store_true', help='measure the written variants and rewrite the table only')
    args = parser.parse_args()
    if args.workers < 1:
        print('[ERROR] --workers must be 1 or more (1 = plan in this process), not %d.' % args.workers)
        return 1
    print('[WARNING] Simulation passed != safe on the robot. c3 is a proposal for re-teaching, not verified on the robot.')
    try:
        cfg = Load_Configuration(args.config)
        plan = Load_Job_Plan(CONST_DEFAULT_JOB)
        openings = {state: Opening(cfg, state, plan) for state in ('holding', 'open')}
        Check_Learned(cfg, plan.Points, os.path.basename(plan.Positions))
        variant_points = Variant_Points(plan, Load_Job_Plan(CONST_C1_JOB), cfg)
    except (Configuration_Error_Cls, URScript_Error_Cls, Kinematics_Error_Cls, OSError) as error:
        print('[ERROR] %s' % error)
        return 1
    started = time.time()
    if not args.table_only:
        requests = []
        for variant, vp in variant_points.items():
            requests += [dataclasses.replace(r, Label='%s %s' % (variant, r.Label))
                         for r in Plan_Requests(cfg, plan, vp, [vp[n].Q for n in plan.Sequence.Station_Points], CONST_PENALTY_WEIGHT)]
        try:
            if args.workers > 1:
                with Make_Pool(args.workers) as pool:
                    results = Plan_All(requests, pool)
            else:
                results = Plan_All(requests)
        except Planner_Error_Cls as error:
            print('[ERROR] %s Nothing is written.' % error)
            return 1
        print('[INFO] Planned %d segments of %d variants in %.0f s (%d worker%s).'
              % (len(requests), len(variant_points), time.time() - started, args.workers, '' if args.workers == 1 else 's'))
        Write_Taught(os.path.join(args.out, 'taught'), plan, Status_Lines('taught'))
        segments = [s.Name for s in plan.Sequence.Segments]
        for k, (variant, vp) in enumerate(variant_points.items()):
            generated = {s: r.Path for s, r in zip(segments, results[len(segments) * k:len(segments) * (k + 1)])}
            Write_Outputs(os.path.join(args.out, variant), cfg, plan, vp, generated, CONST_PENALTY_WEIGHT, Status_Lines(variant),
                          CONST_FILE_NAMES)
    scenes = {state: Scene_Cls(dataclasses.replace(cfg, Opening=o)) for state, o in openings.items()}
    measured = time.time()
    table = Write_Table(args.out, cfg, scenes, plan)
    print('[INFO] Variants (case %d home -> home; estimated durations, r = 0, blends and gripper excluded):' % plan.Job.Case)
    for variant, rows in table.items():
        total = rows[-1]
        print('[INFO]   %-12s %6.2f s  %2d lines  max robot z %.3f m  closest %.1f mm (%s)  %d samples breaking the rule  [%s]'
              % (variant, total['t_est_s'], total['motion_lines'], total['max_robot_z_m'], total['closest'][0] * 1e3,
                 total['closest'][1], total['violating_samples'], CONST_VARIANTS[variant]))
    print('[INFO] Measured in %.0f s. Written to %s. Simulation passed != safe on the robot.'
          % (time.time() - measured, os.path.relpath(args.out, CONST_PROJECT_ROOT)))
    return 0


def Status_Lines(variant: str) -> tp.List[str]:
    """
    Description:
        The status header line of a variant's files (the safety line follows from the writer).

    Args:
        (1) variant [string]: The variant.

    Returns:
        (1) parameter [List<string>]: Comment lines.
    """

    return ['# Variant %s: %s.' % (variant, CONST_VARIANTS[variant])]


if __name__ == '__main__':
    sys.exit(main())
