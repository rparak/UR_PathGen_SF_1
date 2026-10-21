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
# File Name: ../test/Example/test_extract_program.py
## =========================================================================== ##

# System (Default)
import os
import sys
# Hashlib (Secure hashes)
import hashlib
# Regular expressions (Default)
import re
# Subprocess (Subprocess management)
import subprocess
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.:
#   ../URScript/Core — the readers the extracts must satisfy
from URScript.Core import Read_URScript, Read_Program_Excerpt

"""
Description:
    Initialization of constants.

    The extracts of a small synthetic program are typed here line by line from the program below,
    not produced by the writer.
"""
# A synthetic robot program: two learned points split by another global (a non-contiguous range),
# globals that are not _p / _q, and a switch of three cases, case 2 with a nested if.
#   Unit [List<string>]
CONST_PROGRAM = ['def synth():',
                 '  global b_dest_pose=[1, 2, 3, 4, 5, 6]',
                 '  global home_p=p[.1, -.2, .3, 1.0, 2.0, 3.0]',
                 '  global home_q=[0.0, -1.5, 1.5, 0.0, 1.57, 2.4E-5]',
                 '  global count=0',
                 '  global pick_p=p[.4, .5, .6, 0.1, 0.2, 0.3]',
                 '  global pick_q=[1.0, -1.0, 1.0, -1.0, 1.0, -1.0]',
                 '  while (True):',
                 '    switch_1 = plc_int',
                 '    if (1 == switch_1):',
                 '      movej(get_inverse_kin(home_p, qnear=home_q), a=1.4, v=1.05)',
                 '    elif (2 == switch_1):',
                 '      $ 5 "Case 2"',
                 '      movej(get_inverse_kin(pick_p, qnear=pick_q), a=1.4, v=1.05)',
                 '      if (count > 3):',
                 '        rq_set_pos_spd_for(0, 255, 255, "1")',
                 '      end',
                 '      movel(home_p, a=1.2, v=0.25)',
                 '    elif (3 == switch_1):',
                 '      sleep(0.5)',
                 '    end',
                 '  end',
                 'end',
                 '']
# Its extracts into the folder synth/, typed from CONST_PROGRAM: points on lines 3-4 and 6-7, case 1
# line 11, case 2 lines 13-18.
#   Unit [string]
CONST_SHA = hashlib.sha256('\n'.join(CONST_PROGRAM).encode('utf-8')).hexdigest()
CONST_POSITIONS = '\n'.join([
    '# Global _p/_q lines of the owner\'s robot program synth.script, verbatim, lines 3-4, 6-7.',
    '# Source sha256: %s' % CONST_SHA,
    '# An extract only: the program itself (Robotiq URCap code, network settings) is not committed.',
    '# _p: TCP pose [m, rad rotation vector] in the Polyscope base frame; _q: joints [rad], base to wrist 3.',
    '  global home_p=p[.1, -.2, .3, 1.0, 2.0, 3.0]',
    '  global home_q=[0.0, -1.5, 1.5, 0.0, 1.57, 2.4E-5]',
    '  global pick_p=p[.4, .5, .6, 0.1, 0.2, 0.3]',
    '  global pick_q=[1.0, -1.0, 1.0, -1.0, 1.0, -1.0]']) + '\n'
CONST_CASES = '\n'.join([
    '# Lines of the owner\'s robot program synth.script, verbatim: cases 1, 2.',
    '# Each case is the whole body of its branch \'(<n> == switch_1):\' of the program\'s switch.',
    '# Source sha256: %s' % CONST_SHA,
    '# An extract only: the program itself (Robotiq URCap code, network settings) is not committed.',
    '# A line starting with \'# case <n>:\' at column 0 opens the lines of that case; program lines keep their indentation.',
    '# case 1: lines 11-11',
    '      movej(get_inverse_kin(home_p, qnear=home_q), a=1.4, v=1.05)',
    '# case 2: lines 13-18',
    '      $ 5 "Case 2"',
    '      movej(get_inverse_kin(pick_p, qnear=pick_q), a=1.4, v=1.05)',
    '      if (count > 3):',
    '        rq_set_pos_spd_for(0, 255, 255, "1")',
    '      end',
    '      movel(home_p, a=1.2, v=0.25)']) + '\n'
# The owner's program, when available (the extract headers' sha256), and the committed extracts it
# must reproduce.
#   Unit [string], [Tuple<string>]
CONST_PLC_ENV = 'SF1_PLC_V4_SCRIPT'
CONST_PLC_SHA256 = 'ecec8e53007d93acd5cf3da5b545f588a02dbc1b9c26497c82282c37bcfb8eae'
CONST_PLC_FILES = ('sf1_plc_v4_positions.urscript', 'sf1_plc_v4_cases_17_18.script')


def _Run(root, tmp_path, program, *args):
    path = tmp_path / 'synth.script'
    path.write_text('\n'.join(program), encoding='utf-8')
    return subprocess.run([sys.executable, os.path.join(root, 'Example', 'extract_program.py'), '--program', str(path)] + list(args),
                          capture_output=True, text=True, cwd=str(tmp_path), timeout=120)


def test_a_synthetic_program_gives_the_expected_extracts(root, tmp_path):
    """
    Description:
        Run as a person runs it (a fresh interpreter, an unrelated directory), the script writes
        exactly the two extracts typed above into synth/, named after the folder: the _p / _q lines
        verbatim with their two line ranges and the sha256, each case the whole body of its branch
        (case 2 with its nested if), and nothing else. The readers take both as they are.
    """

    result = _Run(root, tmp_path, CONST_PROGRAM, '--cases', '2', '1', '--out', 'synth')
    assert result.returncode == 0, result.stdout + result.stderr
    out = tmp_path / 'synth'
    assert sorted(path.name for path in out.iterdir()) == ['synth_cases_1_2.script', 'synth_positions.urscript']
    assert (out / 'synth_positions.urscript').read_text(encoding='utf-8') == CONST_POSITIONS
    assert (out / 'synth_cases_1_2.script').read_text(encoding='utf-8') == CONST_CASES
    assert list(Read_URScript(CONST_POSITIONS).Points) == ['home', 'pick']
    assert [(line.Kind, line.Target) for line in Read_Program_Excerpt(CONST_CASES)[2] if line.Kind != 'other'] == [
        ('movej', 'pick'), ('gripper', 'open'), ('movel', 'home')]


@pytest.mark.parametrize('program, cases, fragment', [
    (CONST_PROGRAM, ['4'], 'synth\\.script has no case 4 \\(its cases: 1, 2, 3\\)'),
    ([line for line in CONST_PROGRAM if not line.strip().startswith('global pick_q')], ['1'],
     'defines pick_p, but the program has no pick_q'),
    (CONST_PROGRAM[:4] + ['  global home_p=p[.1, .2, .3, 1, 2, 3]'] + CONST_PROGRAM[4:], ['1'],
     'Program line 5 defines home_p again \\(first on line 3\\)'),
])
def test_an_invalid_extraction_is_refused_and_nothing_is_written(root, tmp_path, program, cases, fragment):
    """
    Description:
        A case the program does not have, a point with only its _p, and a name defined twice are each
        refused with exit 1 and a sentence naming the case or the program line; no file is written.
    """

    result = _Run(root, tmp_path, program, '--cases', *cases, '--out', 'synth')
    assert result.returncode == 1
    assert '[ERROR]' in result.stdout
    assert re.search(fragment, result.stdout), result.stdout
    assert not (tmp_path / 'synth').exists()


def test_a_case_already_in_another_cases_file_is_refused(root, tmp_path):
    """
    Description:
        Extracting case 2 into a folder whose synth_cases_1_2.script already holds it is refused,
        naming that file: a case lives in one cases file of a program folder only.
    """

    assert _Run(root, tmp_path, CONST_PROGRAM, '--cases', '1', '2', '--out', 'synth').returncode == 0
    result = _Run(root, tmp_path, CONST_PROGRAM, '--cases', '2', '3', '--out', 'synth')
    assert result.returncode == 1
    assert 'synth_cases_1_2.script already holds case 2' in result.stdout
    assert not (tmp_path / 'synth' / 'synth_cases_2_3.script').exists()


def test_re_extracting_the_owners_program_reproduces_the_committed_extracts(root, tmp_path):
    """
    Description:
        With SF1_PLC_V4_SCRIPT naming the owner's program (the recorded sha256), extracting cases 17
        and 18 into sf1_plc_v4/ writes both committed extracts byte for byte. Without the program the
        test is skipped, never passed.
    """

    source = os.environ.get(CONST_PLC_ENV, '')
    if not source:
        pytest.skip('%s is not set: the extracts cannot be re-made from sf1_plc_v4.script.' % CONST_PLC_ENV)
    with open(source, 'rb') as handle:
        assert hashlib.sha256(handle.read()).hexdigest() == CONST_PLC_SHA256
    result = subprocess.run([sys.executable, os.path.join(root, 'Example', 'extract_program.py'), '--program', source,
                             '--cases', '17', '18', '--out', 'sf1_plc_v4'], capture_output=True, text=True, cwd=str(tmp_path), timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr
    for name in CONST_PLC_FILES:
        with open(os.path.join(root, 'Data', 'Programs', 'sf1_plc_v4', name), 'rb') as handle:
            assert (tmp_path / 'sf1_plc_v4' / name).read_bytes() == handle.read(), name
