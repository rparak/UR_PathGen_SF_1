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
# File Name: ../URScript/Core.py
## =========================================================================== ##

# Regular expressions (Default)
import re
# Typing (Support for type hints)
import typing as tp
# Dataclasses (Data Classes)
from dataclasses import dataclass, field

"""
Description:
    Initialization of constants.

    The only reader and writer of Polyscope variable files (*.urscript):

        global <name>_p=p[x, y, z, rx, ry, rz]    m, rad (rotation vector), Polyscope base frame
        global <name>_q=[j0, j1, j2, j3, j4, j5]   rad, base to wrist 3

    Every number keeps its original token, so a learned point is written back exactly as it was
    read (encoder noise such as 2.44E-5 included). Joint values are never wrapped: UR joints span
    +-2*pi and movej goes to the exact value. Generated points get a fixed precision.

    Also the only reader and writer of a program excerpt (lines of a robot program kept verbatim,
    opened per case by a '# case <n>:' line at column 0) and of the positions extract (the program's
    global _p / _q lines verbatim), and the only writer of a motion sequence (movej lines of
    generated points, program lines copied verbatim, comments).
"""
# A number as Polyscope writes it: '.238...', '-.175...', '2.44E-5', '-0.785...'.
#   Unit [string]
CONST_NUMBER = r'-?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?'
# A pose and a joint line. Any whitespace is accepted around the separators.
#   Unit [Pattern]
CONST_POSE_LINE = re.compile(r'^global\s+([A-Za-z_][A-Za-z0-9_]*)_p\s*=\s*p\[(.*)\]\s*$')
CONST_JOINT_LINE = re.compile(r'^global\s+([A-Za-z_][A-Za-z0-9_]*)_q\s*=\s*\[(.*)\]\s*$')
# Program-excerpt lines: a block marker, a movej to a learned point by its pose with qnear (as
# Polyscope writes it), a movel to a learned pose, and a Robotiq gripper position command. A move
# captures its target, then its a, v and (movej) r tokens when the line has them.
#   Unit [Pattern]
CONST_CASE_MARKER = re.compile(r'^# case (\d+):(?: lines (\d+)-\d+)?')
CONST_PROGRAM_MOVEJ = re.compile(r'^\s*movej\(get_inverse_kin\(([A-Za-z_]\w*)_p, qnear=\1_q\)(?:, a=([^,()]+), v=([^,()]+)(?:, r=([^,()]+))?)?\)\s*$')
CONST_PROGRAM_MOVEL = re.compile(r'^\s*movel\(([A-Za-z_]\w*)_p(?:, a=([^,()]+), v=([^,()]+))?\)\s*$')
CONST_PROGRAM_GRIPPER = re.compile(r'^\s*rq_set_pos_spd_for\((\d+), \d+, \d+, "\d+"\)\s*$')
# Robotiq position commands of the program: 0 opens fully, 255 closes (on the part).
#   Unit [Dict<string, string>]
CONST_GRIPPER_ACTIONS = {'0': 'open', '255': 'close'}
# The lines of a motion sequence (Write_Motion_Sequence): a program movej or movel with its tokens,
# a generated movej to a joint target, and a gripper comment (Gripper_Comment).
#   Unit [Pattern]
CONST_SEQUENCE_MOVEJ = re.compile(r'^movej\(get_inverse_kin\(([A-Za-z_]\w*)_p, qnear=\1_q\), a=([^,()]+), v=([^,()]+)(?:, r=([^,()]+))?\)$')
CONST_SEQUENCE_JOINT = re.compile(r'^movej\(([A-Za-z_]\w*)_q, a=([^,()]+), v=([^,()]+), r=([^,()]+)\)$')
CONST_SEQUENCE_MOVEL = re.compile(r'^movel\(([A-Za-z_]\w*)_p, a=([^,()]+), v=([^,()]+)(?:, r=([^,()]+))?\)$')
CONST_SEQUENCE_GRIPPER = re.compile(r'^# Gripper (open|close): ')
# Lines of a robot program the extracts are cut from: a global _p / _q variable (Polyscope writes
# the learned points so), and a branch of a Polyscope switch ('if (17 == switch_1):', 'elif (...)'),
# whose body is the case.
#   Unit [Pattern]
CONST_PROGRAM_GLOBAL = re.compile(r'^\s*global\s+([A-Za-z_][A-Za-z0-9_]*)_(p|q)\s*=')
CONST_PROGRAM_BRANCH = re.compile(r'^(\s*)(?:if|elif) \((\d+) == ([A-Za-z_]\w*)\):\s*$')
# The header lines of an extract naming its source (Extract_Positions, Extract_Cases).
#   Unit [Pattern]
CONST_EXTRACT_PROGRAM = re.compile(r"^# .* of the owner's robot program (\S+), verbatim")
CONST_EXTRACT_SHA256 = re.compile(r'^# Source sha256: ([0-9a-f]{64})$')
# What an extract leaves out, in its header.
#   Unit [string]
CONST_EXTRACT_NOTE = '# An extract only: the program itself (Robotiq URCap code, network settings) is not committed.'
# Decimal places of a generated value: 1e-12 m / rad, far below anything the robot resolves.
#   Unit [int]
CONST_GENERATED_DECIMALS = 12


class URScript_Error_Cls(ValueError):
    """
    Description:
        A *.urscript file cannot be read. The message names the line and what is wrong.
    """


@dataclass
class Robot_Point_Str:
    """
    Description:
        One robot point: its TCP pose and its joint vector, with the original number tokens.
    """

    # The variable name without the _p / _q suffix.
    #   Unit [string]
    Name: str = ''
    # TCP pose [x, y, z, rx, ry, rz] in the Polyscope base frame.
    #   Unit [Vector<float> 1x6] (m, rad)
    P: tp.List[float] = field(default_factory=list)
    # Joint vector, base to wrist 3.
    #   Unit [Vector<float> 1x6] (rad)
    Q: tp.List[float] = field(default_factory=list)
    # The tokens P and Q are written from: the file's own text for a learned point, fixed precision
    # for a generated one.
    #   Unit [List<string> 1x6]
    P_Tokens: tp.List[str] = field(default_factory=list)
    Q_Tokens: tp.List[str] = field(default_factory=list)


@dataclass
class Program_Line_Str:
    """
    Description:
        One line of a program excerpt: what it does and its text as in the program.
    """

    # 'movej', 'movel', 'gripper' or 'other' (URCap boilerplate, Polyscope markers, comments).
    #   Unit [string]
    Kind: str = ''
    # The target point of a move, 'open' / 'close' for the gripper, '' otherwise.
    #   Unit [string]
    Target: str = ''
    # The line exactly as in the program, indentation included.
    #   Unit [string]
    Raw: str = ''
    # Its 1-based line number in the program (from the case marker's range; 0 when the excerpt
    # gives none).
    #   Unit [int]
    Line: int = 0
    # The acceleration, speed and blend radius tokens of a move as written ('' when the line has
    # none; a movel has no blend).
    #   Unit [string] (rad/s^2, rad/s, m for movej; m/s^2, m/s for movel)
    A_Token: str = ''
    V_Token: str = ''
    R_Token: str = ''


@dataclass
class Motion_Str:
    """
    Description:
        One operation of a motion sequence file: a move or a gripper action.
    """

    # 'movej', 'movel' or 'gripper'.
    #   Unit [string]
    Kind: str = ''
    # The target point of a move; 'open' / 'close' for the gripper.
    #   Unit [string]
    Target: str = ''
    # Acceleration, speed and blend radius as written (r = 0 when the line has none).
    #   Unit [float] (rad/s^2, rad/s, m for movej; m/s^2, m/s, m for movel)
    A: float = 0.0
    V: float = 0.0
    R: float = 0.0
    # True for a generated movej to a joint target, False for a line of the robot program.
    #   Unit [bool]
    Generated: bool = False
    # The last comment line before the operation after the file's header (its section; '' if none).
    #   Unit [string]
    Section: str = ''
    # The 1-based line number in the file.
    #   Unit [int]
    Line: int = 0


@dataclass
class URScript_Document_Str:
    """
    Description:
        A parsed *.urscript file: its lines in order (comments and blanks kept verbatim) and its points.
    """

    # Each line: ('text', raw) for a comment or blank line, ('p', name) or ('q', name) for a variable.
    #   Unit [List<Tuple<string, string>>]
    Lines: tp.List[tp.Tuple[str, str]] = field(default_factory=list)
    # The points by name, in the order of their first line.
    #   Unit [Dict<string, Robot_Point_Str>]
    Points: tp.Dict[str, Robot_Point_Str] = field(default_factory=dict)
    # Whether the text ended with a newline.
    #   Unit [bool]
    Final_Newline: bool = True


@dataclass
class Sequence_Segment_Str:
    """
    Description:
        One segment of a job's sequence (Resolve_Sequence): the program lines it replaces by a
        generated path.
    """

    # The segment's name and its ends (learned points).
    #   Unit [string]
    Name: str = ''
    From: str = ''
    To: str = ''
    # The gripper state while it runs: 'holding' or 'open'.
    #   Unit [string]
    State: str = ''
    # The targets of the taught moves strictly between its ends.
    #   Unit [List<string>]
    Taught_Vias: tp.List[str] = field(default_factory=list)
    # The program lines after the From line up to the To line, inclusive.
    #   Unit [List<Program_Line_Str>]
    Lines: tp.List[Program_Line_Str] = field(default_factory=list)


@dataclass
class Sequence_Plan_Str:
    """
    Description:
        A job's sequence in a case's program lines (Resolve_Sequence).
    """

    # The program lines from the first segment's From line to the last segment's To line.
    #   Unit [List<Program_Line_Str>]
    Lines: tp.List[Program_Line_Str] = field(default_factory=list)
    # The segments in program order.
    #   Unit [List<Sequence_Segment_Str>]
    Segments: tp.List[Sequence_Segment_Str] = field(default_factory=list)
    # Between each two segments, the station operation: the lines after the first's To line up to
    # the second's From line, inclusive, copied verbatim.
    #   Unit [List<List<Program_Line_Str>>]
    Stations: tp.List[tp.List[Program_Line_Str]] = field(default_factory=list)
    # The fixed station points of the generated rule (SPEC 9.7): every point the sequence moves to
    # but its first and last end and the taught via points, in program order.
    #   Unit [List<string>]
    Station_Points: tp.List[str] = field(default_factory=list)


def Parse_Numbers(text: str, line_number: int) -> tp.List[str]:
    """
    Description:
        Split a comma-separated list into number tokens, refusing anything that is not a number.

    Args:
        (1) text [string]: The text between the brackets.
        (2) line_number [int]: The line, for the error message.

    Returns:
        (1) parameter [List<string>]: The tokens, stripped of whitespace.
    """

    tokens = [t.strip() for t in text.split(',')]
    for token in tokens:
        if re.fullmatch(CONST_NUMBER, token) is None:
            raise URScript_Error_Cls('Line %d: "%s" is not a number. Each value must look like 0.5, .5, -.5 or 2.4E-5.'
                                     % (line_number, token))
    return tokens


def Read_URScript(text: str) -> URScript_Document_Str:
    """
    Description:
        Parse the text of a *.urscript variable file. Strict: a line that is neither a variable, a
        '#' comment nor blank is an error, and so is a vector of the wrong length, a duplicate
        variable, or a name with only one of _p and _q.

    Args:
        (1) text [string]: The file content.

    Returns:
        (1) parameter [URScript_Document_Str(object)]: The parsed document.
    """

    document = URScript_Document_Str(Final_Newline=text.endswith('\n'))
    seen: tp.Dict[tp.Tuple[str, str], int] = {}
    for number, raw in enumerate(text.split('\n')[:-1] if document.Final_Newline else text.split('\n'), start=1):
        line = raw.strip()
        if line == '' or line.startswith('#'):
            document.Lines.append(('text', raw))
            continue
        pose, joints = CONST_POSE_LINE.match(line), CONST_JOINT_LINE.match(line)
        if pose is None and joints is None:
            raise URScript_Error_Cls('Line %d is neither "global <name>_p=p[...]" nor "global <name>_q=[...]": %s'
                                     % (number, raw))
        kind, match = ('p', pose) if pose is not None else ('q', joints)
        name, tokens = match.group(1), Parse_Numbers(match.group(2), number)
        if len(tokens) != 6:
            raise URScript_Error_Cls('Line %d: %s_%s has %d values; it must have 6.' % (number, name, kind, len(tokens)))
        if (name, kind) in seen:
            raise URScript_Error_Cls('Line %d: %s_%s is defined again (first on line %d). Remove the duplicate.'
                                     % (number, name, kind, seen[(name, kind)]))
        seen[(name, kind)] = number
        point = document.Points.setdefault(name, Robot_Point_Str(Name=name))
        if kind == 'p':
            point.P, point.P_Tokens = [float(t) for t in tokens], tokens
        else:
            point.Q, point.Q_Tokens = [float(t) for t in tokens], tokens
        document.Lines.append((kind, name))
    for name, point in document.Points.items():
        if not point.P or not point.Q:
            missing = '_p' if not point.P else '_q'
            raise URScript_Error_Cls('%s has no %s%s. Every point needs both its pose (_p) and its joints (_q).'
                                     % (name, name, missing))
    return document


def Read_Program_Excerpt(text: str) -> tp.Dict[int, tp.List[Program_Line_Str]]:
    """
    Description:
        Parse a program excerpt into its cases. Before the first '# case <n>:' line only column-0
        comments are allowed; inside a case every line is a program line, classified as a movej or
        movel to a learned point (with its a, v and r tokens as written: the one reader of them), a
        gripper open/close or other. Strict: an unknown gripper position,
        a repeated case or a case without lines is an error.

    Args:
        (1) text [string]: The file content.

    Returns:
        (1) parameter [Dict<int, List<Program_Line_Str>>]: The lines of each case, in program order.
    """

    cases: tp.Dict[int, tp.List[Program_Line_Str]] = {}
    current, first = None, 0
    for number, raw in enumerate(text.split('\n')[:-1] if text.endswith('\n') else text.split('\n'), start=1):
        marker = CONST_CASE_MARKER.match(raw)
        if marker is not None:
            current, first = int(marker.group(1)), int(marker.group(2) or 0)
            if current in cases:
                raise URScript_Error_Cls('Line %d opens case %d a second time.' % (number, current))
            cases[current] = []
            continue
        if current is None:
            if not raw.startswith('#'):
                raise URScript_Error_Cls('Line %d comes before the first "# case <n>:" line and is not a comment: %s'
                                         % (number, raw))
            continue
        movej, movel, gripper = CONST_PROGRAM_MOVEJ.match(raw), CONST_PROGRAM_MOVEL.match(raw), CONST_PROGRAM_GRIPPER.match(raw)
        line = first + len(cases[current]) if first else 0
        if movej is not None:
            cases[current].append(Program_Line_Str('movej', movej.group(1), raw, line, movej.group(2) or '', movej.group(3) or '',
                                                   movej.group(4) or ''))
        elif movel is not None:
            cases[current].append(Program_Line_Str('movel', movel.group(1), raw, line, movel.group(2) or '', movel.group(3) or ''))
        elif gripper is not None:
            if gripper.group(1) not in CONST_GRIPPER_ACTIONS:
                raise URScript_Error_Cls('Line %d sets the gripper to position %s; only 0 (open) and 255 (close) are '
                                         'known.' % (number, gripper.group(1)))
            cases[current].append(Program_Line_Str('gripper', CONST_GRIPPER_ACTIONS[gripper.group(1)], raw, line))
        else:
            cases[current].append(Program_Line_Str('other', '', raw, line))
    for case, lines in cases.items():
        if not lines:
            raise URScript_Error_Cls('Case %d has no lines.' % case)
    return cases


def Line_Ranges(numbers: tp.Sequence[int]) -> str:
    """
    Description:
        Line numbers as ranges of consecutive lines: [1, 2, 3, 7] -> '1-3, 7'.

    Args:
        (1) numbers [List<int>]: Line numbers, ascending.

    Returns:
        (1) parameter [string]: The ranges.
    """

    ranges: tp.List[tp.List[int]] = []
    for number in numbers:
        if ranges and number == ranges[-1][1] + 1:
            ranges[-1][1] = number
        else:
            ranges.append([number, number])
    return ', '.join('%d-%d' % (a, b) if a != b else '%d' % a for a, b in ranges)


def Extract_Positions(lines: tp.Sequence[str], program: str, sha256: str) -> str:
    """
    Description:
        The positions extract of a robot program: every global <name>_p / <name>_q line, verbatim and
        in program order, after a header naming the program, its line ranges and its sha256. Strict:
        a malformed vector, a duplicate name or a name with only one of _p and _q is an error naming
        the program line.

    Args:
        (1) lines [List<string>]: The program's lines, without their newline.
        (2) program [string]: The program's file name.
        (3) sha256 [string]: The sha256 of the program file (hex).

    Returns:
        (1) parameter [string]: The extract, newline-terminated.
    """

    seen: tp.Dict[tp.Tuple[str, str], int] = {}
    numbers = []
    for number, raw in enumerate(lines, start=1):
        found = CONST_PROGRAM_GLOBAL.match(raw)
        if found is None:
            continue
        name, kind = found.group(1), found.group(2)
        match = (CONST_POSE_LINE if kind == 'p' else CONST_JOINT_LINE).match(raw.strip())
        if match is None or len(Parse_Numbers(match.group(2), number)) != 6:
            raise URScript_Error_Cls('Program line %d is not a 6-value %s_%s line: %s' % (number, name, kind, raw.strip()))
        if (name, kind) in seen:
            raise URScript_Error_Cls('Program line %d defines %s_%s again (first on line %d); the program has a duplicate name.'
                                     % (number, name, kind, seen[(name, kind)]))
        seen[(name, kind)] = number
        numbers.append(number)
    if not numbers:
        raise URScript_Error_Cls('%s has no global <name>_p / <name>_q lines.' % program)
    for name, kind in seen:
        other = 'q' if kind == 'p' else 'p'
        if (name, other) not in seen:
            raise URScript_Error_Cls('Program line %d defines %s_%s, but the program has no %s_%s. Every point needs both.'
                                     % (seen[(name, kind)], name, kind, name, other))
    header = ['# Global _p/_q lines of the owner\'s robot program %s, verbatim, lines %s.' % (program, Line_Ranges(numbers)),
              '# Source sha256: %s' % sha256, CONST_EXTRACT_NOTE,
              '# _p: TCP pose [m, rad rotation vector] in the Polyscope base frame; _q: joints [rad], base to wrist 3.']
    return '\n'.join(header + [lines[n - 1] for n in numbers]) + '\n'


def Program_Cases(lines: tp.Sequence[str]) -> tp.Dict[int, tp.Tuple[int, int, str]]:
    """
    Description:
        The cases of a robot program's switches: each the whole body of its branch
        '(el)if (<n> == <variable>):', up to the next line indented no deeper than the branch (the
        next branch or the switch's end). A case number in two branches is an error.

    Args:
        (1) lines [List<string>]: The program's lines, without their newline.

    Returns:
        (1) parameter [Dict<int, Tuple<int, int, string>>]: Case -> its first and last line (1-based,
                                                            inclusive) and the branch line, stripped.
    """

    cases: tp.Dict[int, tp.Tuple[int, int, str]] = {}
    for number, raw in enumerate(lines, start=1):
        branch = CONST_PROGRAM_BRANCH.match(raw)
        if branch is None:
            continue
        case, depth = int(branch.group(2)), len(branch.group(1))
        if case in cases:
            raise URScript_Error_Cls('Program lines %d and %d both open case %d; the case is ambiguous.'
                                     % (cases[case][0] - 1, number, case))
        last = number
        while last < len(lines) and (not lines[last].strip() or len(lines[last]) - len(lines[last].lstrip()) > depth):
            last += 1
        if last == number:
            raise URScript_Error_Cls('Program line %d opens case %d, which has no lines.' % (number, case))
        cases[case] = (number + 1, last, raw.strip())
    return cases


def Extract_Cases(lines: tp.Sequence[str], cases: tp.Sequence[int], program: str, sha256: str) -> str:
    """
    Description:
        The case extract of a robot program (read by Read_Program_Excerpt): a header naming the
        program and its sha256, then per case, ascending, a '# case <n>: lines <a>-<b>' line and the
        whole body of its branch (Program_Cases), verbatim. A case the program does not have, or one
        asked for twice, is an error.

    Args:
        (1) lines [List<string>]: The program's lines, without their newline.
        (2) cases [List<int>]: The cases.
        (3) program [string]: The program's file name.
        (4) sha256 [string]: The sha256 of the program file (hex).

    Returns:
        (1) parameter [string]: The extract, newline-terminated.
    """

    found = Program_Cases(lines)
    if len(set(cases)) != len(cases):
        raise URScript_Error_Cls('A case is asked for twice: %s.' % ', '.join(str(c) for c in cases))
    missing = [case for case in cases if case not in found]
    if missing:
        raise URScript_Error_Cls('%s has no case %s (its cases: %s).'
                                 % (program, ', '.join(str(c) for c in missing), ', '.join(str(c) for c in sorted(found)) or 'none'))
    variables = sorted({CONST_PROGRAM_BRANCH.match(found[case][2]).group(3) for case in cases})
    out = ['# Lines of the owner\'s robot program %s, verbatim: cases %s.' % (program, ', '.join(str(c) for c in sorted(cases))),
           '# Each case is the whole body of its branch \'(<n> == %s):\' of the program\'s switch.' % ' / '.join(variables),
           '# Source sha256: %s' % sha256, CONST_EXTRACT_NOTE,
           '# A line starting with \'# case <n>:\' at column 0 opens the lines of that case; program lines keep their indentation.']
    for case in sorted(cases):
        first, last, _ = found[case]
        out += ['# case %d: lines %d-%d' % (case, first, last)] + list(lines[first - 1:last])
    return '\n'.join(out) + '\n'


def Read_Extract_Source(text: str) -> tp.Tuple[str, str]:
    """
    Description:
        The source an extract names in its header (Extract_Positions, Extract_Cases): the program's
        file name and its sha256.

    Args:
        (1) text [string]: The extract's content.

    Returns:
        (1) parameter [Tuple<string, string>]: The file name and the sha256 (hex).
    """

    program = sha256 = ''
    for raw in text.split('\n'):
        if not raw.startswith('#'):
            break
        name, digest = CONST_EXTRACT_PROGRAM.match(raw), CONST_EXTRACT_SHA256.match(raw)
        program = name.group(1) if name is not None else program
        sha256 = digest.group(1) if digest is not None else sha256
    if not program or not sha256:
        raise URScript_Error_Cls('The extract\'s header does not name its program and sha256; write it with Example/extract_program.py.')
    return program, sha256


def Format_Generated(value: float) -> str:
    """
    Description:
        Format a generated value with fixed precision in Polyscope's style: the leading zero is
        dropped ('0.5' -> '.500000000000') and negative zero is written as zero.

    Args:
        (1) value [float]: The value.

    Returns:
        (1) parameter [string]: The token.
    """

    token = '%.*f' % (CONST_GENERATED_DECIMALS, value)
    if float(token) == 0.0:
        token = token.lstrip('-')
    if token.startswith('0.'):
        token = token[1:]
    elif token.startswith('-0.'):
        token = '-' + token[2:]
    return token


def Make_Generated_Point(name: str, P: tp.Sequence[float], Q: tp.Sequence[float]) -> Robot_Point_Str:
    """
    Description:
        A generated point with fixed-precision tokens. Its _p is informational: the robot is driven
        by _q only.

    Args:
        (1) name [string]: The variable name without suffix.
        (2) P [Vector<float> 1x6]: TCP pose (m, rad); its rotation vector must already be normalised.
        (3) Q [Vector<float> 1x6]: Joint vector (rad); never wrapped.

    Returns:
        (1) parameter [Robot_Point_Str(object)]: The point.
    """

    if len(P) != 6 or len(Q) != 6:
        raise URScript_Error_Cls('A generated point needs 6 pose and 6 joint values; %s has %d and %d.'
                                 % (name, len(P), len(Q)))
    p_tokens = [Format_Generated(v) for v in P]
    q_tokens = [Format_Generated(v) for v in Q]
    return Robot_Point_Str(Name=name, P=[float(t) for t in p_tokens], Q=[float(t) for t in q_tokens],
                           P_Tokens=p_tokens, Q_Tokens=q_tokens)


def Replace_Joints(point: Robot_Point_Str, Q: tp.Sequence[float]) -> Robot_Point_Str:
    """
    Description:
        The point with other joint values and its pose unchanged: the _p tokens are kept, a joint
        whose value does not change keeps its token, a changed joint gets a generated token
        (Format_Generated). For a learned point whose joints an owner decision changes, e.g. a wrist
        turned by a whole turn; the values are never wrapped here.

    Args:
        (1) point [Robot_Point_Str(object)]: The point.
        (2) Q [Vector<float> 1x6]: The new joint vector (rad).

    Returns:
        (1) parameter [Robot_Point_Str(object)]: A new point; the given one is not changed.
    """

    if len(Q) != 6:
        raise URScript_Error_Cls('%s needs 6 joint values, not %d.' % (point.Name, len(Q)))
    tokens = [token if float(token) == float(value) else Format_Generated(float(value)) for token, value in zip(point.Q_Tokens, Q)]
    return Robot_Point_Str(Name=point.Name, P=list(point.P), Q=[float(t) for t in tokens], P_Tokens=list(point.P_Tokens),
                           Q_Tokens=tokens)


def Write_URScript(document: URScript_Document_Str) -> str:
    """
    Description:
        Write a document back to text from its tokens, in Polyscope's form ('global x_p=p[a, b]').
        For a file in that form, Write_URScript(Read_URScript(text)) == text, byte for byte.

    Args:
        (1) document [URScript_Document_Str(object)]: The document.

    Returns:
        (1) parameter [string]: The file content.
    """

    out = []
    for kind, value in document.Lines:
        if kind == 'text':
            out.append(value)
        elif kind == 'p':
            out.append('global %s_p=p[%s]' % (value, ', '.join(document.Points[value].P_Tokens)))
        else:
            out.append('global %s_q=[%s]' % (value, ', '.join(document.Points[value].Q_Tokens)))
    return '\n'.join(out) + ('\n' if document.Final_Newline else '')


def Make_Document(points: tp.Sequence[Robot_Point_Str], header: tp.Sequence[str] = ()) -> URScript_Document_Str:
    """
    Description:
        A document holding the given points, each as its _p line then its _q line, after header
        comment lines.

    Args:
        (1) points [List<Robot_Point_Str>]: The points, learned or generated.
        (2) header [List<string>]: Comment lines, each starting with '#'.

    Returns:
        (1) parameter [URScript_Document_Str(object)]: The document.
    """

    document = URScript_Document_Str()
    for line in header:
        if not line.startswith('#'):
            raise URScript_Error_Cls('A header line must start with "#": %s' % line)
        document.Lines.append(('text', line))
    for point in points:
        if point.Name in document.Points:
            raise URScript_Error_Cls('%s appears twice. Every point name must be unique.' % point.Name)
        document.Points[point.Name] = point
        document.Lines += [('p', point.Name), ('q', point.Name)]
    return document


def Write_Movej_Sequence(names: tp.Sequence[str], a_token: str, v_token: str) -> str:
    """
    Description:
        One 'movej(<name>_q, a=<a>, v=<v>, r=0)' line per point. Joint targets only, never
        movej(p[...]): with a pose the robot solves its own inverse kinematics and may pick another
        branch. a and v are the program's tokens, so a generated move runs at the speed the
        duration estimate assumes; r=0 is explicit because a blend leaves the checked joint-linear
        segment (SPEC 10.4).

    Args:
        (1) names [List<string>]: Point names in motion order.
        (2 - 3) a_token, v_token [string]: Joint acceleration (rad/s^2) and speed (rad/s), as written.

    Returns:
        (1) parameter [string]: The lines, newline-terminated.
    """

    return ''.join('movej(%s_q, a=%s, v=%s, r=0)\n' % (name, a_token, v_token) for name in names)


def Write_Motion_Sequence(entries: tp.Sequence[tp.Tuple[str, str]], header: tp.Sequence[str], a_token: str,
                          v_token: str) -> str:
    """
    Description:
        A motion sequence in program order: ('movej', name) becomes a generated movej to <name>_q
        (Write_Movej_Sequence), ('program', line) a program line copied verbatim without its
        indentation (the robot program's own a, v and r included), ('comment', text) a '# ' line.

    Args:
        (1) entries [List<Tuple<string, string>>]: The entries in order.
        (2) header [List<string>]: Comment lines, each starting with '#'.
        (3 - 4) a_token, v_token [string]: The generated movej's acceleration and speed, as written.

    Returns:
        (1) parameter [string]: The file content, newline-terminated.
    """

    out = []
    for line in header:
        if not line.startswith('#'):
            raise URScript_Error_Cls('A header line must start with "#": %s' % line)
        out.append(line + '\n')
    for kind, value in entries:
        if kind == 'movej':
            out.append(Write_Movej_Sequence([value], a_token, v_token))
        elif kind == 'program':
            out.append(value.lstrip() + '\n')
        elif kind == 'comment':
            out.append('# %s\n' % value)
        else:
            raise URScript_Error_Cls('A motion-sequence entry is movej, program or comment, not %r.' % (kind,))
    return ''.join(out)


def Gripper_Comment(line: Program_Line_Str) -> str:
    """
    Description:
        The comment that stands for a gripper action of the program in a motion sequence.

    Args:
        (1) line [Program_Line_Str(object)]: A gripper line of the program.

    Returns:
        (1) parameter [string]: The comment text (without '# ').
    """

    if line.Kind != 'gripper':
        raise URScript_Error_Cls('Only a gripper line has a gripper comment, not a %s line: %s' % (line.Kind, line.Raw.strip()))
    return 'Gripper %s: the program\'s Robotiq node goes here (%s)' % (line.Target, line.Raw.strip())


def Program_Entries(lines: tp.Sequence[Program_Line_Str]) -> tp.List[tp.Tuple[str, str]]:
    """
    Description:
        The motion-sequence entries (Write_Motion_Sequence) of program lines as taught: every movej
        and movel as a program line, verbatim with its target, a, v and r; every gripper action as
        its comment (Gripper_Comment). Every other line (URCap code, Polyscope markers, PLC I/O,
        waits) is left out.

    Args:
        (1) lines [List<Program_Line_Str>]: Program lines in order (Read_Program_Excerpt).

    Returns:
        (1) parameter [List<Tuple<string, string>>]: The entries in order.
    """

    entries = []
    for line in lines:
        if line.Kind in ('movej', 'movel'):
            entries.append(('program', line.Raw))
        elif line.Kind == 'gripper':
            entries.append(('comment', Gripper_Comment(line)))
    return entries


def Resolve_Sequence(lines: tp.Sequence[Program_Line_Str], segments: tp.Sequence[tp.Tuple[str, str, str]],
                     learned: tp.Collection[str]) -> Sequence_Plan_Str:
    """
    Description:
        The sequence a job plans in a case's program lines (SPEC 10.6). Each segment's ends are
        matched in program order: From at the next move to it after the previous segment's end, To at
        the next move to it after From. The sequence runs from the first segment's From line to the
        last segment's To line; the lines between two segments are their station operation, copied
        verbatim. A segment's gripper state is the result of the case's last gripper action before
        it (0 open, 255 close = holding), or, before the first, the opposite of that first action.
        Refused, each with a sentence naming the segment and the line: an end that is not a learned
        point, an end the case does not move to in that order, a segment containing a gripper action,
        a station line blending (r > 0) into the generated segment after it, a case without gripper
        actions, and two consecutive gripper actions of the same kind.

    Args:
        (1) lines [List<Program_Line_Str>]: The case's program lines in order (Read_Program_Excerpt).
        (2) segments [List<Tuple<string, string, string>>]: The segments (name, from, to), in program order.
        (3) learned [Collection<string>]: The program's learned point names.

    Returns:
        (1) parameter [Sequence_Plan_Str(object)]: The sequence.
    """

    def Where(index: int) -> str:
        """
        Description:
            Where a line of the case is, for a message: its program line, else its place in the case.

        Args:
            (1) index [int]: The line's index in the case.

        Returns:
            (1) parameter [string]: The place.
        """

        return 'program line %d' % lines[index].Line if lines[index].Line else 'case line %d' % (index + 1)

    def Next_Move(target: str, after: int) -> int:
        """
        Description:
            The index of the case's next move to a point after a line.

        Args:
            (1) target [string]: The point.
            (2) after [int]: The index after which to look.

        Returns:
            (1) parameter [int]: The index, -1 when there is none.
        """

        for index in range(after + 1, len(lines)):
            if lines[index].Kind in ('movej', 'movel') and lines[index].Target == target:
                return index
        return -1

    # 1\ The gripper actions of the whole case: alternating, at least one.
    actions = [(index, line.Target) for index, line in enumerate(lines) if line.Kind == 'gripper']
    if not actions:
        raise URScript_Error_Cls('The case has no gripper action, so the gripper state of its segments (holding or open) '
                                 'cannot be derived.')
    for (_, first), (index, second) in zip(actions[:-1], actions[1:]):
        if first == second:
            raise URScript_Error_Cls('The case\'s gripper actions must alternate; %s is a second %s in a row.' % (Where(index), second))

    # 2\ Match the segments in order.
    out, after = Sequence_Plan_Str(), -1
    bounds = []
    for name, start, goal in segments:
        for point in (start, goal):
            if point not in learned:
                raise URScript_Error_Cls('Segment %s (%s -> %s): %s is not a learned point of the program (it is not in the '
                                         'positions extract).' % (name, start, goal, point))
        i_from = Next_Move(start, after)
        i_to = Next_Move(goal, i_from) if i_from >= 0 else -1
        if i_from < 0 or i_to < 0:
            missing, since = (start, after) if i_from < 0 else (goal, i_from)
            raise URScript_Error_Cls('Segment %s (%s -> %s): the case does not move to %s after %s. A segment\'s ends are '
                                     'matched in program order.' % (name, start, goal, missing,
                                                                    Where(since) if since >= 0 else 'its start'))
        crossed = [index for index, _ in actions if i_from < index < i_to]
        if crossed:
            raise URScript_Error_Cls('Segment %s (%s -> %s) contains the gripper action at %s (%s). A generated segment runs at '
                                     'one gripper state; end it before the action.' % (name, start, goal, Where(crossed[0]),
                                                                                        lines[crossed[0]].Raw.strip()))
        if bounds and lines[i_from].Kind == 'movej' and lines[i_from].R_Token:
            raise URScript_Error_Cls('Segment %s (%s -> %s): the station line at %s blends into it (%s); a blend into a generated '
                                     'segment is not checked (SPEC 8.5).' % (name, start, goal, Where(i_from), lines[i_from].Raw.strip()))
        before = [target for index, target in actions if index < i_from]
        state = ('open' if before[-1] == 'open' else 'holding') if before else ('holding' if actions[0][1] == 'open' else 'open')
        out.Segments.append(Sequence_Segment_Str(name, start, goal, state,
                                                 [line.Target for line in lines[i_from + 1:i_to] if line.Kind in ('movej', 'movel')],
                                                 list(lines[i_from + 1:i_to + 1])))
        if bounds:
            out.Stations.append(list(lines[bounds[-1][1] + 1:i_from + 1]))
        bounds.append((i_from, i_to))
        after = i_to

    # 3\ The sequence and its station points: every point it moves to, but its two ends and the taught via points.
    out.Lines = list(lines[bounds[0][0]:bounds[-1][1] + 1])
    for k, segment in enumerate(out.Segments):
        names = ([segment.To] if k < len(out.Segments) - 1 else []) + (
            [line.Target for line in out.Stations[k] if line.Kind in ('movej', 'movel')] if k < len(out.Stations) else [])
        out.Station_Points += [n for n in names if n not in out.Station_Points]
    return out


def Read_Motion_Sequence(text: str) -> tp.List[Motion_Str]:
    """
    Description:
        Parse a motion sequence file (Write_Motion_Sequence): program movej and movel lines, generated
        movej to joint targets and gripper comments, in order. The comment lines before the first
        operation are the file's header; every later comment line names the section of the
        operations after it. Strict: any other line is an error naming its number.

    Args:
        (1) text [string]: The file content.

    Returns:
        (1) parameter [List<Motion_Str>]: The operations in order.
    """

    out: tp.List[Motion_Str] = []
    section = ''
    for number, raw in enumerate(text.splitlines(), start=1):
        if not raw.strip():
            continue
        gripper = CONST_SEQUENCE_GRIPPER.match(raw)
        if gripper is not None:
            out.append(Motion_Str('gripper', gripper.group(1), Section=section, Line=number))
            continue
        if raw.startswith('#'):
            section = raw[1:].strip() if out else ''
            continue
        for pattern, kind, generated in ((CONST_SEQUENCE_MOVEJ, 'movej', False), (CONST_SEQUENCE_JOINT, 'movej', True),
                                         (CONST_SEQUENCE_MOVEL, 'movel', False)):
            match = pattern.match(raw)
            if match is not None:
                try:
                    a, v, r = float(match.group(2)), float(match.group(3)), float(match.group(4) or 0.0)
                except ValueError as error:
                    raise URScript_Error_Cls('Line %d has a non-numeric a, v or r: %s' % (number, raw)) from error
                out.append(Motion_Str(kind, match.group(1), a, v, r, generated, section, number))
                break
        else:
            raise URScript_Error_Cls('Line %d is not a move, a gripper comment or a comment of a motion sequence: %s'
                                     % (number, raw))
    return out
