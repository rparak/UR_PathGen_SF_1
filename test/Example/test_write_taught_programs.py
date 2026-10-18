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
# File Name: ../test/Example/test_write_taught_programs.py

# System (Default)
import os
import sys
# Subprocess (Subprocess management)
import subprocess
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.:
#   ../URScript/Core — the extracts the simplified programs are checked against
from URScript.Core import Read_URScript, Read_Program_Excerpt

"""
Description:
    Initialization of constants.

    The simplified taught programs in Data/Programs/sf1_plc_v4/ are checked against the extract
    lines they come from, not against the writer.
"""
# The extracts and the simplified programs.
#   Unit [Tuple<string>]
CONST_FOLDER = ('Data', 'Programs', 'sf1_plc_v4')
CONST_FILES = ('sf1_plc_v4_case_%d.urscript', 'sf1_plc_v4_case_%d_sequence.script')


def _Read(root, name):
    with open(os.path.join(root, *CONST_FOLDER, name), encoding='utf-8') as handle:
        return handle.read()


@pytest.mark.parametrize('case', [17, 18])
def test_every_motion_line_is_the_programs_line_in_order(root, case):
    """
    Description:
        After its comment header, the simplified program of a case is exactly: each movej and movel
        line of the case, in program order, equal to the program's line without its indentation
        (target, a, v and r unchanged), and a '# Gripper open/close' comment naming the program's
        rq_set_pos_spd_for line at the place of each gripper action. Nothing else: no URCap code, no
        PLC I/O, no waits.
    """

    lines = Read_Program_Excerpt(_Read(root, 'sf1_plc_v4_cases_17_18.script'))[case]
    body = _Read(root, CONST_FILES[1] % case).splitlines()
    while body and body[0].startswith('#') and not body[0].startswith('# Gripper'):
        body.pop(0)
    source = [line for line in lines if line.Kind in ('movej', 'movel', 'gripper')]
    assert len(body) == len(source)
    assert sum(1 for line in source if line.Kind == 'gripper') == 2
    for written, line in zip(body, source):
        if line.Kind == 'gripper':
            assert written.startswith('# Gripper %s: ' % line.Target) and line.Raw.strip() in written, written
        else:
            assert written == line.Raw.lstrip(), written


@pytest.mark.parametrize('case', [17, 18])
def test_the_points_are_the_cases_targets_with_the_programs_tokens(root, case):
    """
    Description:
        The points file of a case holds exactly the points the case moves to, in the order of first
        use, each with the positions extract's tokens.
    """

    lines = Read_Program_Excerpt(_Read(root, 'sf1_plc_v4_cases_17_18.script'))[case]
    targets = list(dict.fromkeys(line.Target for line in lines if line.Kind in ('movej', 'movel')))
    extract = Read_URScript(_Read(root, 'sf1_plc_v4_positions.urscript')).Points
    written = Read_URScript(_Read(root, CONST_FILES[0] % case)).Points
    assert list(written) == targets
    for name in targets:
        assert written[name].P_Tokens == extract[name].P_Tokens and written[name].Q_Tokens == extract[name].Q_Tokens


def test_the_writer_reproduces_the_committed_programs(root, tmp_path):
    """
    Description:
        Example/write_taught_programs.py, run as a person runs it (a fresh interpreter, an unrelated
        directory), writes the committed files byte for byte: they are its output, not edited by
        hand.
    """

    out = tmp_path / 'out'
    result = subprocess.run([sys.executable, os.path.join(root, 'Example', 'write_taught_programs.py'), '--out', str(out)],
                            capture_output=True, text=True, cwd=str(tmp_path), timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr
    for case in (17, 18):
        for name in CONST_FILES:
            assert (out / (name % case)).read_text(encoding='utf-8') == _Read(root, name % case)
