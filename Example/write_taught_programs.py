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
# File Name: ../Example/write_taught_programs.py

# System (Default)
import os
import sys
#   Add access if it is not in the system path.
CONST_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src')
if CONST_SRC not in sys.path:
    sys.path.append(CONST_SRC)
# Argparse (Command-line parsing)
import argparse
# Typing (Support for type hints)
import typing as tp
# Custom Lib.:
#   ../Configuration/Core — the project root
from Configuration.Core import CONST_PROJECT_ROOT
#   ../URScript/Core — the writers
from URScript.Core import Program_Entries, Program_Line_Str, Make_Document, Write_URScript, Write_Motion_Sequence, URScript_Error_Cls
#   ../Example/plan_rotation — the program extracts (SPEC 10.1) and the case (case 17, home -> ... -> home)
from plan_rotation import CONST_PROGRAM_DIR, CONST_CASE, Read_Program

"""
Description:
    Initialization of constants.
"""
# The file names of the simplified program: its points and its motion sequence.
#   Unit [string]
CONST_POINTS_NAME = 'sf1_plc_v4_case_%d.urscript' % CONST_CASE
CONST_SEQUENCE_NAME = 'sf1_plc_v4_case_%d_sequence.script' % CONST_CASE


def Case_Targets(lines: tp.Sequence[Program_Line_Str]) -> tp.List[str]:
    """
    Description:
        The learned points a case moves to, in the order of their first use.

    Args:
        (1) lines [List<Program_Line_Str>]: The case's program lines.

    Returns:
        (1) parameter [List<string>]: The point names.
    """

    names: tp.List[str] = []
    for line in lines:
        if line.Kind in ('movej', 'movel') and line.Target not in names:
            names.append(line.Target)
    return names


def main():
    """
    Description:
        Write the simplified taught program of case 17's rotation sequence, home -> ... -> home: its
        points with the program's tokens (sf1_plc_v4_case_17.urscript) and its motion program
        (sf1_plc_v4_case_17_sequence.script): every movej and movel in program order, verbatim with
        their targets, a, v and r, the gripper actions as comments, nothing else.

            .venv/bin/python Example/write_taught_programs.py [--out DIR]
    """

    parser = argparse.ArgumentParser(description='Write the simplified taught program of case 17 (home -> home).')
    parser.add_argument('--out', default=CONST_PROGRAM_DIR, help='output directory (default: %(default)s)')
    args = parser.parse_args()
    try:
        points, lines = Read_Program()
    except (OSError, URScript_Error_Cls) as error:
        print('[ERROR] %s' % error)
        return 1
    targets = Case_Targets(lines)
    missing = [name for name in targets if name not in points]
    if missing:
        print('[ERROR] Case %d moves to %s, which the positions extract does not hold.' % (CONST_CASE, ', '.join(missing)))
        return 1
    os.makedirs(args.out, exist_ok=True)
    points_header = ['# Case %d of sf1_plc_v4.script as taught, home -> home: the points it moves to, in the order of' % CONST_CASE,
                     '# first use, verbatim from Data/Programs/sf1_plc_v4/sf1_plc_v4_positions.urscript. Written by',
                     '# Example/write_taught_programs.py.',
                     '# _p: TCP pose [m, rad rotation vector] in the Polyscope base frame; _q: joints [rad], base to wrist 3.']
    with open(os.path.join(args.out, CONST_POINTS_NAME), 'w', encoding='utf-8') as handle:
        handle.write(Write_URScript(Make_Document([points[name] for name in targets], points_header)))
    sequence_header = ['# Case %d of sf1_plc_v4.script as taught, the rotation sequence home -> home: every movej and movel' % CONST_CASE,
                       '# in program order, verbatim (targets, a, v, r), the gripper actions as comments. Left out: the',
                       '# Robotiq URCap code, the PLC I/O and the waits.',
                       '# Points: %s. Written by Example/write_taught_programs.py.' % CONST_POINTS_NAME]
    with open(os.path.join(args.out, CONST_SEQUENCE_NAME), 'w', encoding='utf-8') as handle:
        # No generated movej here: the a and v tokens are not used.
        handle.write(Write_Motion_Sequence(Program_Entries(lines), sequence_header, '', ''))
    print('[INFO] Case %d (home -> home): %d points, %d motion lines, %d gripper actions written to %s.'
          % (CONST_CASE, len(targets), sum(1 for line in lines if line.Kind in ('movej', 'movel')),
             sum(1 for line in lines if line.Kind == 'gripper'), os.path.relpath(args.out, CONST_PROJECT_ROOT)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
