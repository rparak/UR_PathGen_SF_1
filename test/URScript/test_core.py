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
# Math (Mathematical functions)
import math
# Hashlib (Secure hashes)
import hashlib
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.:
#   ../src/URScript/Core — the reader and writer under test
from URScript.Core import (Read_URScript, Write_URScript, Write_Movej_Sequence, Make_Generated_Point, Make_Document,
                           Format_Generated, URScript_Error_Cls, Read_Program_Excerpt, Write_Motion_Sequence, Program_Entries,
                           Gripper_Comment, Read_Motion_Sequence, Rotation_Sequence, Replace_Joints)

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

# The owner's robot program the extracts in Data/Programs/sf1_plc_v4/ come from: its
# sha256 and the 1-based line range of its global _p/_q lines (measured 2026-10-17). The program is
# not committed; the comparison runs only when SF1_PLC_V4_SCRIPT names it.
#   Unit [string], [Tuple<int>]
CONST_PLC_SHA256 = 'ecec8e53007d93acd5cf3da5b545f588a02dbc1b9c26497c82282c37bcfb8eae'
CONST_PLC_LINES = (2453, 2608)
CONST_PLC_ENV = 'SF1_PLC_V4_SCRIPT'
# Header lines of the extract before the verbatim lines.
#   Unit [int]
CONST_PLC_HEADER = 4
# The program lines of the whole cases 17 and 18 (the body of 'elif (n == switch_1):'), 1-based,
# inclusive.
#   Unit [Dict<int, Tuple<int>>]
CONST_PLC_CASE_LINES = {17: (3397, 3478), 18: (3480, 3561)}
# The whole case in program order, typed from the Polyscope program tree (moves and gripper): out of
# machine (a), the rotation sequence home -> ... -> home, back into machine (a).
#   Unit [Dict<int, List<Tuple<string, string>>>]
CONST_PLC_CASE_ORDER = {
    case: [('movej', n) for n in before]
          + [('movej', 'home'), ('movej', 'Waypoint_1'), ('movej', drop), ('movel', below), ('gripper', 'open'),
             ('movej', 'Waypoint_5'), ('movej', 'Waypoint_1'), ('movej', 'Waypoint_2'), ('movej', 'Waypoint_3'),
             ('movej', approach), ('movej', pick), ('gripper', 'close'), ('movel', above), ('movej', 'Waypoint_2'),
             ('movej', 'home')]
          + [('movej', n) for n in after]
    for case, before, drop, below, approach, pick, above, after in (
        (17, ('load_hower', 'load3', 'load2'), 'drop_rotate_1', 'Waypoint_4', 'Waypoint_18', 'pick_rotate_1', 'Waypoint_6',
         ('load2', 'load3', 'Waypoint_10', 'Waypoint_13', 'bench_drop_11')),
        (18, ('Waypoint_16', 'load3', 'load2'), 'drop_rotate_2', 'Waypoint_7', 'Waypoint_11', 'pick_rotate_2', 'Waypoint_8',
         ('load2', 'load3', 'Waypoint_12', 'bench_rect_drop', 'bench_drop_12')))}

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
        A written file with its comment header (the generated points of case 17) and the export
        fixture round-trip exactly.
    """

    for parts in (('Data', 'Generated', 'SF1_Rotation_17.urscript'), ('test', 'URScript', 'data', 'SF1_Positions_original.urscript')):
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
    with open(os.path.join(root, 'Data', 'Programs', 'sf1_plc_v4', 'sf1_plc_v4_positions.urscript'), 'rb') as handle:
        extract = handle.read().split(b'\n')[CONST_PLC_HEADER:]
    assert extract == program.split(b'\n')[CONST_PLC_LINES[0] - 1:CONST_PLC_LINES[1]] + [b'']


def test_the_plc_extract_is_read_with_every_token(root):
    """
    Description:
        The strict reader takes the extract as it is: 78 points, each with _p and _q, and
        drop_rotate_1 keeps the export's tokens (the same point as the robot export typed here).
    """

    document = Read_URScript(_Read_File(root, 'Data', 'Programs', 'sf1_plc_v4', 'sf1_plc_v4_positions.urscript'))
    assert len(document.Points) == (CONST_PLC_LINES[1] - CONST_PLC_LINES[0] + 1) // 2
    export = Read_URScript(CONST_EXPORT).Points
    for name in ('home', 'drop_rotate_1'):
        assert document.Points[name].P_Tokens == export[name].P_Tokens
        assert document.Points[name].Q_Tokens == export[name].Q_Tokens


def test_the_case_extract_is_its_source_verbatim(root):
    """
    Description:
        With SF1_PLC_V4_SCRIPT naming the owner's program, every line of each case in the case
        extract is the program's line of the recorded range, byte for byte, the range is the whole
        body of the case's branch, and the program has the recorded sha256. Without the program the test is skipped, never passed.
    """

    source = os.environ.get(CONST_PLC_ENV, '')
    if not source:
        pytest.skip('%s is not set: the extract cannot be compared with sf1_plc_v4.script.' % CONST_PLC_ENV)
    with open(source, 'rb') as handle:
        program = handle.read()
    assert hashlib.sha256(program).hexdigest() == CONST_PLC_SHA256
    lines = program.decode('utf-8').split('\n')
    cases = Read_Program_Excerpt(_Read_File(root, 'Data', 'Programs', 'sf1_plc_v4', 'sf1_plc_v4_cases_17_18.script'))
    assert sorted(cases) == sorted(CONST_PLC_CASE_LINES)
    for case, (first, last) in CONST_PLC_CASE_LINES.items():
        assert [line.Raw for line in cases[case]] == lines[first - 1:last]
        # The whole case: the range is the body of its branch, from after its 'elif' to the next one.
        assert lines[first - 2].strip() == 'elif (%d == switch_1):' % case
        assert lines[last].strip().startswith('elif (')


def test_the_case_extract_reads_in_program_order(root):
    """
    Description:
        The reader returns, for each case, the moves to learned points and the gripper actions in the
        order of the program tree.
    """

    cases = Read_Program_Excerpt(_Read_File(root, 'Data', 'Programs', 'sf1_plc_v4', 'sf1_plc_v4_cases_17_18.script'))
    for case, order in CONST_PLC_CASE_ORDER.items():
        assert [(line.Kind, line.Target) for line in cases[case] if line.Kind != 'other'] == order


def test_the_rotation_sequence_runs_from_the_first_home_to_the_last(root):
    """
    Description:
        Rotation_Sequence returns case 17's lines from its first movej to home to its second,
        inclusive: the moves typed from the program (between load2 before it and load2 after it),
        the two gripper actions inside; a case that moves to home once or three times is refused.
    """

    lines = Read_Program_Excerpt(_Read_File(root, 'Data', 'Programs', 'sf1_plc_v4', 'sf1_plc_v4_cases_17_18.script'))[17]
    rotation = Rotation_Sequence(lines, 'home')
    moves = [(line.Kind, line.Target) for line in rotation if line.Kind != 'other']
    assert moves == [('movej', 'home'), ('movej', 'Waypoint_1'), ('movej', 'drop_rotate_1'), ('movel', 'Waypoint_4'),
                     ('gripper', 'open'), ('movej', 'Waypoint_5'), ('movej', 'Waypoint_1'), ('movej', 'Waypoint_2'),
                     ('movej', 'Waypoint_3'), ('movej', 'Waypoint_18'), ('movej', 'pick_rotate_1'), ('gripper', 'close'),
                     ('movel', 'Waypoint_6'), ('movej', 'Waypoint_2'), ('movej', 'home')]
    homes = [k for k, line in enumerate(lines) if line.Kind == 'movej' and line.Target == 'home']
    with pytest.raises(URScript_Error_Cls, match='moves to home 1 time'):
        Rotation_Sequence(lines[:homes[1]], 'home')
    with pytest.raises(URScript_Error_Cls, match='moves to home 3 time'):
        Rotation_Sequence(list(lines) + [lines[homes[0]]], 'home')


@pytest.mark.parametrize('text, fragment', [
    ('movej(get_inverse_kin(home_p, qnear=home_q))\n', 'before the first'),
    ('# case 17: x\n  rq_set_pos_spd_for(128, 255, 255, "1")\n', 'position 128'),
    ('# case 17: x\n  sleep(1)\n# case 17: y\n  sleep(1)\n', 'second time'),
    ('# case 17: x\n', 'no lines'),
])
def test_a_malformed_program_excerpt_is_refused(text, fragment):
    """
    Description:
        The excerpt reader is strict: a line before the first case, an unknown gripper position, a
        repeated case and an empty case are refused, naming the problem.
    """

    with pytest.raises(URScript_Error_Cls, match=fragment):
        Read_Program_Excerpt(text)


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


def test_replacing_joints_keeps_the_pose_and_every_unchanged_token():
    """
    Description:
        Replace_Joints of a learned point with wrist 3 turned by -2 pi (the owner's choice for the
        pick, SPEC 10.5): the _p tokens and the five unchanged _q tokens are the file's own, the
        changed one is the generated token of the new value; written and read back it is that value;
        the given point is not changed. Six values are required.
    """

    learned = Read_URScript(CONST_EXPORT).Points['drop_rotate_1']
    before = (list(learned.P_Tokens), list(learned.Q_Tokens))
    turned = list(learned.Q)
    turned[5] -= 2.0 * math.pi
    point = Replace_Joints(learned, turned)
    assert point.P_Tokens == learned.P_Tokens and point.P == learned.P
    assert point.Q_Tokens[:5] == learned.Q_Tokens[:5]
    assert point.Q_Tokens[5] == Format_Generated(learned.Q[5] - 2.0 * math.pi)
    again = Read_URScript(Write_URScript(Make_Document([point]))).Points['drop_rotate_1']
    assert again.Q[5] == float(Format_Generated(learned.Q[5] - 2.0 * math.pi)) and again.Q[:5] == learned.Q[:5]
    assert (learned.P_Tokens, learned.Q_Tokens) == before
    with pytest.raises(URScript_Error_Cls, match='needs 6 joint values'):
        Replace_Joints(learned, turned[:5])


def test_movej_uses_joint_targets_and_no_blend():
    """
    Description:
        Each line is movej(<name>_q, a=<a>, v=<v>, r=0): joint targets only, never a pose, the
        given a and v tokens as written, and no blend.
    """

    assert (Write_Movej_Sequence(['home', 'drop_rotate_1'], '1.3962634015954636', '1.0471975511965976')
            == 'movej(home_q, a=1.3962634015954636, v=1.0471975511965976, r=0)\n'
               'movej(drop_rotate_1_q, a=1.3962634015954636, v=1.0471975511965976, r=0)\n')


def test_a_motion_sequence_writes_generated_moves_program_lines_and_comments():
    """
    Description:
        A generated entry becomes movej(<name>_q, a, v, r=0), a program line is copied without its
        indentation, a comment gets '# '; the header comes first and an unknown entry is refused.
    """

    text = Write_Motion_Sequence([('movej', 'via_a_b_01'), ('program', '        movel(Waypoint_4_p, a=1.2, v=0.25)'),
                                  ('comment', 'Gripper open')], ['# head'], '1.4', '1.05')
    assert text == ('# head\nmovej(via_a_b_01_q, a=1.4, v=1.05, r=0)\nmovel(Waypoint_4_p, a=1.2, v=0.25)\n'
                    '# Gripper open\n')
    with pytest.raises(URScript_Error_Cls, match='movej, program or comment'):
        Write_Motion_Sequence([('movel', 'x')], [], '1', '1')


def test_program_entries_keep_the_moves_and_comment_the_gripper():
    """
    Description:
        Program_Entries keeps every movej and movel line verbatim (indentation is removed only by
        the writer), turns a gripper action into its comment naming the program line, and leaves
        out everything else (Polyscope markers, PLC I/O, waits); a non-gripper line has no gripper
        comment.
    """

    text = ('# case 17: test\n'
            '        $ 290 "load_hower" "breakAfter"\n'
            '        movej(get_inverse_kin(a_p, qnear=a_q), a=0.17453292519943295, v=0.17453292519943295)\n'
            '        rq_set_pos_spd_for(0, 255, 255, "1")\n'
            '        movel(b_p, a=1.2, v=0.25)\n'
            '        write_output_boolean_register(0,True)\n'
            '        sleep(0.5)\n')
    lines = Read_Program_Excerpt(text)[17]
    assert Program_Entries(lines) == [
        ('program', '        movej(get_inverse_kin(a_p, qnear=a_q), a=0.17453292519943295, v=0.17453292519943295)'),
        ('comment', 'Gripper open: the program\'s Robotiq node goes here (rq_set_pos_spd_for(0, 255, 255, "1"))'),
        ('program', '        movel(b_p, a=1.2, v=0.25)')]
    with pytest.raises(URScript_Error_Cls, match='Only a gripper line'):
        Gripper_Comment(lines[1])


def test_a_motion_sequence_is_read_with_its_tokens_and_sections():
    """
    Description:
        Read_Motion_Sequence returns the moves and gripper actions in order: a program movej with
        and without r, a generated movej to a joint target, a movel, the gripper comment; a, v and r
        as written (r = 0 when absent); the header comments name no section, a later comment does.
        Any other line is refused, naming its number.
    """

    text = ('# header 1\n# header 2\n'
            'movej(get_inverse_kin(a_p, qnear=a_q), a=0.17453292519943295, v=0.17453292519943295)\n'
            '# S1 (generated): a -> b\n'
            'movej(via_a_b_01_q, a=1.3962634015954636, v=1.0471975511965976, r=0)\n'
            'movel(b_p, a=1.2, v=0.25)\n'
            '# Gripper open: the program\'s Robotiq node goes here (rq_set_pos_spd_for(0, 255, 255, "1"))\n'
            'movej(get_inverse_kin(c_p, qnear=c_q), a=1.3962634015954636, v=1.0471975511965976, r=0.01)\n')
    found = [(m.Kind, m.Target, m.A, m.V, m.R, m.Generated, m.Section, m.Line) for m in Read_Motion_Sequence(text)]
    assert found == [('movej', 'a', 0.17453292519943295, 0.17453292519943295, 0.0, False, '', 3),
                     ('movej', 'via_a_b_01', 1.3962634015954636, 1.0471975511965976, 0.0, True, 'S1 (generated): a -> b', 5),
                     ('movel', 'b', 1.2, 0.25, 0.0, False, 'S1 (generated): a -> b', 6),
                     ('gripper', 'open', 0.0, 0.0, 0.0, False, 'S1 (generated): a -> b', 7),
                     ('movej', 'c', 1.3962634015954636, 1.0471975511965976, 0.01, False, 'S1 (generated): a -> b', 8)]
    with pytest.raises(URScript_Error_Cls, match='Line 2 is not a move'):
        Read_Motion_Sequence('movel(b_p, a=1.2, v=0.25)\nsleep(0.5)\n')
