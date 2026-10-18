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
# Custom Lib.:
#   ../URScript/Core — the extracts the simplified programs are checked against
from URScript.Core import Read_URScript, Read_Program_Excerpt

"""
Description:
    Initialization of constants.

    The simplified taught program in Data/Programs/sf1_plc_v4/ is checked against the extract lines
    it comes from and against the owner's statement of the sequence, not against the writer.
"""
# The extracts and the simplified program (case 17, home -> home).
#   Unit [Tuple<string>]
CONST_FOLDER = ('Data', 'Programs', 'sf1_plc_v4')
CONST_FILES = ('sf1_plc_v4_case_17.urscript', 'sf1_plc_v4_case_17_sequence.script')
# The deliverable as the owner states it (2026-10-18): case 17 from home to home, each line (kind,
# target), the gripper actions as ('gripper', 'open' / 'close').
#   Unit [List<Tuple<string, string>>]
CONST_SEQUENCE = [('movej', 'home'), ('movej', 'Waypoint_1'), ('movej', 'drop_rotate_1'), ('movel', 'Waypoint_4'),
                  ('gripper', 'open'), ('movej', 'Waypoint_5'), ('movej', 'Waypoint_1'), ('movej', 'Waypoint_2'),
                  ('movej', 'Waypoint_3'), ('movej', 'Waypoint_18'), ('movej', 'pick_rotate_1'), ('gripper', 'close'),
                  ('movel', 'Waypoint_6'), ('movej', 'Waypoint_2'), ('movej', 'home')]


def _Read(root, name):
    with open(os.path.join(root, *CONST_FOLDER, name), encoding='utf-8') as handle:
        return handle.read()


def test_every_motion_line_is_the_programs_line_in_order(root):
    """
    Description:
        After its comment header, the simplified program is exactly case 17 from home to home as the
        owner states it, and it is one contiguous run of the program: each movej and movel line equal
        to the program's line without its indentation (target, a, v and r unchanged, home r=0.01),
        and a '# Gripper open/close' comment naming the program's rq_set_pos_spd_for line at the place
        of each gripper action. Nothing else: no loading moves, no URCap code, no PLC I/O, no waits.
    """

    source = [line for line in Read_Program_Excerpt(_Read(root, 'sf1_plc_v4_cases_17_18.script'))[17]
              if line.Kind in ('movej', 'movel', 'gripper')]
    body = _Read(root, CONST_FILES[1]).splitlines()
    while body and body[0].startswith('#') and not body[0].startswith('# Gripper'):
        body.pop(0)
    assert len(body) == len(CONST_SEQUENCE)
    starts = [k for k in range(len(source) - len(body) + 1)
              if [(line.Kind, line.Target) for line in source[k:k + len(body)]] == CONST_SEQUENCE]
    assert len(starts) == 1, starts
    for written, line in zip(body, source[starts[0]:]):
        if line.Kind == 'gripper':
            assert written.startswith('# Gripper %s: ' % line.Target) and line.Raw.strip() in written, written
        else:
            assert written == line.Raw.lstrip(), written
    assert body[0].endswith(', r=0.01)') and body[-1].endswith('v=1.0471975511965976)')


def test_the_points_are_the_cases_targets_with_the_programs_tokens(root):
    """
    Description:
        The points file holds exactly the points of case 17 home -> home, in the order of first use,
        each with the positions extract's tokens.
    """

    targets = list(dict.fromkeys(target for kind, target in CONST_SEQUENCE if kind != 'gripper'))
    extract = Read_URScript(_Read(root, 'sf1_plc_v4_positions.urscript')).Points
    written = Read_URScript(_Read(root, CONST_FILES[0])).Points
    assert list(written) == targets
    for name in targets:
        assert written[name].P_Tokens == extract[name].P_Tokens and written[name].Q_Tokens == extract[name].Q_Tokens


def test_the_writer_reproduces_the_committed_programs(root, tmp_path):
    """
    Description:
        Example/write_taught_programs.py, run as a person runs it (a fresh interpreter, an unrelated
        directory), writes the committed files byte for byte and nothing else: they are its output,
        not edited by hand.
    """

    out = tmp_path / 'out'
    result = subprocess.run([sys.executable, os.path.join(root, 'Example', 'write_taught_programs.py'), '--out', str(out)],
                            capture_output=True, text=True, cwd=str(tmp_path), timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr
    assert sorted(path.name for path in out.iterdir()) == sorted(CONST_FILES)
    for name in CONST_FILES:
        assert (out / name).read_text(encoding='utf-8') == _Read(root, name)
