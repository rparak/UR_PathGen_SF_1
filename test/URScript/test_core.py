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
# File Name: ../test/URScript/test_core.py
## =========================================================================== ##

# System (Default)
import os
# Hashlib (Secure hashes)
import hashlib
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.:
#   ../src/URScript/Core — the reader and writer under test
from URScript.Core import (Read_URScript, Write_URScript, Write_Movej_Sequence, Make_Generated_Point, Make_Document,
                           Format_Generated, URScript_Error_Cls)

"""
Description:
    Initialization of constants.

    The expected values are typed from the robot export (the owner's message), not read from the
    files under test.
"""
# The robot export, verbatim.
#   Unit [string]
CONST_EXPORT = (
    'global home_p=p[.238739153619, -.175248184859, .472678665244, 2.218804439569, -2.220766336909, .005958014905]\n'
    'global home_q=[2.4439792468911037E-5, -0.785396472816803, -2.094330072402954, -1.8325640163817347, '
    '1.5708248615264893, 1.098817665479146E-5]\n'
    'global drop_rotate_1_p=p[.702750032831, .304908453934, .601746000573, -1.178088968154, -1.218990569788, '
    '-1.196227307052]\n'
    'global drop_rotate_1_q=[2.917997121810913, -1.3577392858317872, 1.4622281233416956, -0.03949482858691411, '
    '-0.2542222181903284, 1.5013701915740967]\n')
# The 24 values, typed as float literals.
#   Unit [Dict<string, List<float>>]
CONST_VALUES = {
    'home': ([0.238739153619, -0.175248184859, 0.472678665244, 2.218804439569, -2.220766336909, 0.005958014905],
             [2.4439792468911037e-05, -0.785396472816803, -2.094330072402954, -1.8325640163817347, 1.5708248615264893,
              1.098817665479146e-05]),
    'drop_rotate_1': ([0.702750032831, 0.304908453934, 0.601746000573, -1.178088968154, -1.218990569788, -1.196227307052],
                      [2.917997121810913, -1.3577392858317872, 1.4622281233416956, -0.03949482858691411,
                       -0.2542222181903284, 1.5013701915740967]),
}

# The owner's robot program the extract test/fixtures/sf1_plc_v4_positions.urscript comes from: its
# sha256 and the 1-based line range of its global _p/_q lines (measured 2026-10-17). The program is
# not committed; the comparison runs only when SF1_PLC_V4_SCRIPT names it.
#   Unit [string], [Tuple<int>]
CONST_PLC_SHA256 = 'ecec8e53007d93acd5cf3da5b545f588a02dbc1b9c26497c82282c37bcfb8eae'
CONST_PLC_LINES = (2453, 2608)
CONST_PLC_ENV = 'SF1_PLC_V4_SCRIPT'
# Header lines of the extract before the verbatim lines.
#   Unit [int]
CONST_PLC_HEADER = 4

def _Read_File(root, *parts):
    with open(os.path.join(root, *parts), encoding='utf-8') as handle:
        return handle.read()


def test_the_export_is_read_exactly():
    """
    Description:
        All 24 numbers are read to the exact float of their literal, including '.238...' without a
        leading zero and the encoder noise 2.44E-5, which is not rounded or cleaned.
    """

    points = Read_URScript(CONST_EXPORT).Points
    assert list(points) == ['home', 'drop_rotate_1']
    for name, (p, q) in CONST_VALUES.items():
        assert points[name].P == p
        assert points[name].Q == q
    assert points['home'].Q_Tokens[0] == '2.4439792468911037E-5'
    assert points['home'].P_Tokens[0] == '.238739153619'
    assert points['home'].P_Tokens[1] == '-.175248184859'


def test_the_export_round_trips_byte_for_byte():
    """
    Description:
        Write_URScript(Read_URScript(text)) == text for the robot export and for both shipped files.
    """

    assert Write_URScript(Read_URScript(CONST_EXPORT)) == CONST_EXPORT


def test_the_shipped_files_round_trip_byte_for_byte(root):
    """
    Description:
        The positions file (with its comment header) and the export fixture round-trip exactly.
    """

    for parts in (('Data', 'Positions', 'SF1_Positions.urscript'), ('test', 'URScript', 'data', 'SF1_Positions_original.urscript')):
        text = _Read_File(root, *parts)
        assert Write_URScript(Read_URScript(text)) == text


def test_the_fixture_is_the_robot_export(root):
    """
    Description:
        The kept fixture is byte-identical to the export typed here.
    """

    assert _Read_File(root, 'test', 'URScript', 'data', 'SF1_Positions_original.urscript') == CONST_EXPORT


def test_the_plc_extract_is_its_source_verbatim(root):
    """
    Description:
        With SF1_PLC_V4_SCRIPT naming the owner's program, the program has the recorded sha256 and the
        extract's lines after its header are the program's lines 2453-2608, byte for byte. Without
        the program the test is skipped, never passed.
    """

    source = os.environ.get(CONST_PLC_ENV, '')
    if not source:
        pytest.skip('%s is not set: the extract cannot be compared with sf1_plc_v4.script.' % CONST_PLC_ENV)
    with open(source, 'rb') as handle:
        program = handle.read()
    assert hashlib.sha256(program).hexdigest() == CONST_PLC_SHA256
    with open(os.path.join(root, 'test', 'fixtures', 'sf1_plc_v4_positions.urscript'), 'rb') as handle:
        extract = handle.read().split(b'\n')[CONST_PLC_HEADER:]
    assert extract == program.split(b'\n')[CONST_PLC_LINES[0] - 1:CONST_PLC_LINES[1]] + [b'']


def test_the_plc_extract_is_read_with_every_token(root):
    """
    Description:
        The strict reader takes the extract as it is: 78 points, each with _p and _q, and
        drop_rotate_1 keeps the export's tokens (the same point as the robot export typed here).
    """

    document = Read_URScript(_Read_File(root, 'test', 'fixtures', 'sf1_plc_v4_positions.urscript'))
    assert len(document.Points) == (CONST_PLC_LINES[1] - CONST_PLC_LINES[0] + 1) // 2
    export = Read_URScript(CONST_EXPORT).Points
    for name in ('home', 'drop_rotate_1'):
        assert document.Points[name].P_Tokens == export[name].P_Tokens
        assert document.Points[name].Q_Tokens == export[name].Q_Tokens


def test_the_renamed_points_keep_every_token(root):
    """
    Description:
        home -> home_safe and drop_rotate_1 -> drop_rot_01: renamed, values verbatim (tokens equal).
    """

    original = Read_URScript(CONST_EXPORT).Points
    renamed = Read_URScript(_Read_File(root, 'Data', 'Positions', 'SF1_Positions.urscript')).Points
    assert list(renamed) == ['home_safe', 'drop_rot_01']
    for old, new in (('home', 'home_safe'), ('drop_rotate_1', 'drop_rot_01')):
        assert renamed[new].P_Tokens == original[old].P_Tokens
        assert renamed[new].Q_Tokens == original[old].Q_Tokens


@pytest.mark.parametrize('text, fragment', [
    ('global a_p=p[1, 2, 3, 4, 5, 6]\nglobal a_q=[1, 2, 3, 4, 5]\n', 'has 5 values'),
    ('global a_p=p[1, 2, 3, 4, 5, 6]\n', 'no a_q'),
    ('global a_q=[1, 2, 3, 4, 5, 6]\n', 'no a_p'),
    ('global a_p=p[1, 2, 3, 4, 5, 6]\nglobal a_q=[1, 2, 3, 4, 5, 6]\nglobal a_q=[1, 2, 3, 4, 5, 6]\n', 'Line 3'),
    ('global a_p=p[1, 2, 3, 4, 5, 6]\nmovej(a_q)\n', 'Line 2'),
    ('global a_p=p[1, 2, x, 4, 5, 6]\n', 'Line 1: "x" is not a number'),
])
def test_a_malformed_file_is_refused_naming_the_problem(text, fragment):
    """
    Description:
        Strict reading: wrong length, missing pair, duplicate, an unknown line or a non-number
        raise URScript_Error_Cls whose message names the problem (and the line).
    """

    with pytest.raises(URScript_Error_Cls, match=fragment):
        Read_URScript(text)


def test_comments_and_blank_lines_are_kept():
    """
    Description:
        A '#' comment and a blank line survive the round trip.
    """

    text = '# header\n\n' + CONST_EXPORT
    assert Write_URScript(Read_URScript(text)) == text


@pytest.mark.parametrize('value, token', [(0.5, '.500000000000'), (-0.5, '-.500000000000'), (-0.0, '.000000000000'),
                                          (-1e-14, '.000000000000'), (4.0, '4.000000000000'),
                                          (-2.094330072402954, '-2.094330072403')])
def test_a_generated_value_has_fixed_precision_in_polyscope_style(value, token):
    """
    Description:
        Generated values: 12 decimals, leading zero dropped, negative zero written as zero.
    """

    assert Format_Generated(value) == token


def test_joint_values_are_never_wrapped():
    """
    Description:
        A joint at 4.0 rad (beyond pi) and the learned A/B values pass parse -> path -> write
        unchanged: UR joints span +-2*pi and movej goes to the exact value.
    """

    learned = Read_URScript(CONST_EXPORT).Points
    synthetic = Make_Generated_Point('via_home_drop_rotate_1_01', [0.5, 0.0, 0.5, 0.0, 3.14, 0.0],
                                     [4.0, -4.0, 1.0, 6.0, -6.2, 0.0])
    path = [learned['home'], synthetic, learned['drop_rotate_1']]
    text = Write_URScript(Make_Document(path))
    again = Read_URScript(text).Points
    assert again['via_home_drop_rotate_1_01'].Q == [4.0, -4.0, 1.0, 6.0, -6.2, 0.0]
    for name in ('home', 'drop_rotate_1'):
        assert again[name].Q_Tokens == learned[name].Q_Tokens
        assert again[name].P_Tokens == learned[name].P_Tokens


def test_movej_uses_joint_targets_and_no_blend():
    """
    Description:
        Each line is movej(<name>_q, r=0): joint targets only, never a pose, and no blend.
    """

    assert Write_Movej_Sequence(['home_safe', 'drop_rot_01']) == 'movej(home_safe_q, r=0)\nmovej(drop_rot_01_q, r=0)\n'
