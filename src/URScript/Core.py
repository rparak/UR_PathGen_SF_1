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
"""
# A number as Polyscope writes it: '.238...', '-.175...', '2.44E-5', '-0.785...'.
#   Unit [string]
CONST_NUMBER = r'-?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?'
# A pose and a joint line. Any whitespace is accepted around the separators.
#   Unit [Pattern]
CONST_POSE_LINE = re.compile(r'^global\s+([A-Za-z_][A-Za-z0-9_]*)_p\s*=\s*p\[(.*)\]\s*$')
CONST_JOINT_LINE = re.compile(r'^global\s+([A-Za-z_][A-Za-z0-9_]*)_q\s*=\s*\[(.*)\]\s*$')
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


def Write_Movej_Sequence(names: tp.Sequence[str]) -> str:
    """
    Description:
        One 'movej(<name>_q, r=0)' line per point. Joint targets only, never movej(p[...]): with a
        pose the robot solves its own inverse kinematics and may pick another branch. a and v are
        omitted (URScript defaults; the operator sets the speed), and r=0 is explicit because a
        blend leaves the checked joint-linear segment.

    Args:
        (1) names [List<string>]: Point names in motion order.

    Returns:
        (1) parameter [string]: The lines, newline-terminated.
    """

    return ''.join('movej(%s_q, r=0)\n' % name for name in names)
