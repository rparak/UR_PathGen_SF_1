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
# File Name: ../Example/extract_program.py
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
# Glob (Unix style pathname pattern expansion)
import glob
# Hashlib (Secure hashes)
import hashlib
# Typing (Support for type hints)
import typing as tp
# Custom Lib.:
#   ../Configuration/Core — the project root
from Configuration.Core import CONST_PROJECT_ROOT
#   ../URScript/Core — the extract writers and the case-extract reader
from URScript.Core import Extract_Positions, Extract_Cases, Read_Program_Excerpt, Read_Extract_Source, URScript_Error_Cls

"""
Description:
    Initialization of constants.
"""
# The extract file names in Data/Programs/<name>/ (SPEC 10.1): the positions and one cases file per
# extraction, named after its cases.
#   Unit [string]
CONST_POSITIONS_NAME = '%s_positions.urscript'
CONST_CASES_NAME = '%s_cases_%s.script'


def Check_Other_Extracts(out: str, name: str, cases_name: str, cases: tp.Sequence[int], sha256: str) -> None:
    """
    Description:
        Refuse an extraction into a folder whose other cases files hold one of the cases (a case
        would be in two files) or come from another version of the program (another sha256).

    Args:
        (1) out [string]: The program folder.
        (2) name [string]: The program folder's name.
        (3) cases_name [string]: The file about to be written (it may be replaced).
        (4) cases [List<int>]: The cases about to be written.
        (5) sha256 [string]: The sha256 of the program being extracted.
    """

    for path in sorted(glob.glob(os.path.join(out, CONST_CASES_NAME % (name, '*')))):
        if os.path.basename(path) == cases_name:
            continue
        with open(path, encoding='utf-8') as handle:
            text = handle.read()
        both = sorted(set(cases) & set(Read_Program_Excerpt(text)))
        if both:
            raise URScript_Error_Cls('%s already holds case %s. Remove it, or extract the other cases only.'
                                     % (os.path.basename(path), ', '.join(str(c) for c in both)))
        if Read_Extract_Source(text)[1] != sha256:
            raise URScript_Error_Cls('%s comes from another version of the program (another sha256). Re-extract its cases '
                                     'from this program, or remove it.' % os.path.basename(path))


def main():
    """
    Description:
        Cut the two extracts of a robot program (SPEC 10.1) into a program folder: the positions
        <name>_positions.urscript (every global _p / _q line) and the cases
        <name>_cases_<n>_<m>.script (the whole body of each case's branch), verbatim, each with the
        program's line ranges and sha256 in its header. <name> is the folder's name.

            .venv/bin/python Example/extract_program.py --program PATH --cases N [N ...] --out Data/Programs/<name>/
    """

    parser = argparse.ArgumentParser(description='Cut the positions and cases extracts of a robot program.')
    parser.add_argument('--program', required=True, help='the robot program (*.script, as the controller writes it)')
    parser.add_argument('--cases', type=int, nargs='+', required=True, help='the cases of the program\'s switch to extract')
    parser.add_argument('--out', required=True, help='the program folder, Data/Programs/<name>/')
    args = parser.parse_args()
    out = os.path.abspath(args.out)
    name = os.path.basename(os.path.normpath(out))
    cases_name = CONST_CASES_NAME % (name, '_'.join(str(c) for c in sorted(args.cases)))
    try:
        with open(args.program, 'rb') as handle:
            content = handle.read()
        sha256 = hashlib.sha256(content).hexdigest()
        text = content.decode('utf-8')
        lines = text.split('\n')[:-1] if text.endswith('\n') else text.split('\n')
        program = os.path.basename(args.program)
        positions = Extract_Positions(lines, program, sha256)
        cases = Extract_Cases(lines, args.cases, program, sha256)
        os.makedirs(out, exist_ok=True)
        Check_Other_Extracts(out, name, cases_name, args.cases, sha256)
    except (OSError, UnicodeDecodeError, URScript_Error_Cls) as error:
        print('[ERROR] %s Nothing is written.' % error)
        return 1
    for file_name, content_out in ((CONST_POSITIONS_NAME % name, positions), (cases_name, cases)):
        with open(os.path.join(out, file_name), 'w', encoding='utf-8') as handle:
            handle.write(content_out)
    shown = os.path.relpath(out, CONST_PROJECT_ROOT) if out.startswith(CONST_PROJECT_ROOT) else out
    print('[INFO] %s (sha256 %s): %d point lines and the cases %s written to %s (%s, %s).'
          % (program, sha256, sum(1 for line in positions.split('\n') if line.strip().startswith('global ')),
             ', '.join(str(c) for c in sorted(args.cases)), shown, CONST_POSITIONS_NAME % name, cases_name))
    return 0


if __name__ == '__main__':
    sys.exit(main())
