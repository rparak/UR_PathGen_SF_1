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
# File Name: ../test/Scene/test_program.py
## =========================================================================== ##

# System (Default)
import os
import re
import json
import hashlib
# Dataclasses (Data Classes)
import dataclasses
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.:
#   ../src/Scene/Core — the scene the program's segments are replayed in
from Scene.Core import Scene_Cls

"""
Description:
    Initialization of constants.

    Every segment of the owner's program runs on the real robot without collision (owner), so the
    program is ground truth for the model even where the model is known to be wrong (the
    machine-interior hull, the grasp contacts; KNOWN_ISSUES). The reference
    (test/Scene/data/sf1_plc_v4_segments.json) records which segments penetrate the environment and
    how deep; a change to the scene may make none of them worse.
"""
# The reference: every unique segment of the 21 cases and the penetrating ones with their depth.
#   Unit [Tuple<string>]
CONST_REFERENCE = ('test', 'Scene', 'data', 'sf1_plc_v4_segments.json')
# A penetration may be at most this much deeper than recorded (sampling and the convex distance
# solver differ by far less; a real change of the geometry by more).
#   Unit [float] (m)
CONST_TOLERANCE = 0.0005
# The two gripper openings of the program: the 50 mm holding width and the full 85 mm stroke; each
# segment is checked at both (the gripper state of every case is not modelled here).
#   Unit [Tuple<float>] (m)
CONST_OPENINGS = (0.050, 0.085)
# The cap of the distance search: just above the self margin; only a penetration matters here.
#   Unit [float] (m)
CONST_DISTMAX = 0.006
# The owner's program, when available: its environment variable, sha256 (the extract headers'), the
# case branch and a move to a learned point (movej through get_inverse_kin, or movel).
#   Unit [string], [Pattern]
CONST_PLC_ENV = 'SF1_PLC_V4_SCRIPT'
CONST_PLC_SHA256 = 'ecec8e53007d93acd5cf3da5b545f588a02dbc1b9c26497c82282c37bcfb8eae'
CONST_CASE_BRANCH = re.compile(r'^(\s*)(?:el)?if \((\d+) == switch_1\):\s*$')
CONST_MOVE = re.compile(r'^\s*move[jl]\((?:get_inverse_kin\()?(\w+)_p\b')


@pytest.fixture(scope='module')
def reference(root):
    """
    Description:
        The recorded segments and penetrations.
    """

    with open(os.path.join(root, *CONST_REFERENCE), encoding='utf-8') as handle:
        return json.load(handle)


@pytest.mark.slow
def test_no_program_segment_penetrates_more_than_recorded(cfg, points, reference):
    """
    Description:
        Every unique segment of the 21 cases, joint-linear on the samples of SPEC 9.7, at both
        openings: no segment that the reference records as clear penetrates the environment, no
        recorded one is deeper than recorded (+ CONST_TOLERANCE), and no robot or gripper body
        penetrates another. Every offending segment is listed with its depth.
    """

    scenes = [Scene_Cls(dataclasses.replace(cfg, Opening=opening)) for opening in CONST_OPENINGS]
    recorded = reference['penetrating']
    assert set(recorded) <= {'%s|%s' % (a, b) for a, b in reference['segments']}
    problems = []
    for a, b in reference['segments']:
        deepest, own = CONST_DISTMAX, []
        for scene in scenes:
            for q in scene.Segment_Samples(points[a].Q, points[b].Q):
                deepest = min(deepest, scene.Environment_Distance(q, CONST_DISTMAX)[0])
                own += [v for v in scene.Check_Clearance(q) if v.Distance < 0.0
                        and not (v.Body_1.startswith('env') or v.Body_2.startswith('env'))]
        key = '%s|%s' % (a, b)
        if deepest < 0.0 and key not in recorded:
            problems.append('%s -> %s: new penetration, %.1f mm' % (a, b, deepest * 1e3))
        elif key in recorded and deepest < recorded[key] - CONST_TOLERANCE:
            problems.append('%s -> %s: %.1f mm, deeper than the recorded %.1f mm' % (a, b, deepest * 1e3, recorded[key] * 1e3))
        problems += ['%s -> %s: self-collision %s - %s, %.1f mm' % (a, b, v.Body_1, v.Body_2, v.Distance * 1e3) for v in own[:1]]
    assert problems == []


def test_the_segment_list_is_the_programs(reference):
    """
    Description:
        With SF1_PLC_V4_SCRIPT naming the owner's program (the recorded sha256), the reference's
        segments are exactly the program's: in every case branch, each move to a learned point
        from the previous target, a case entered from home. Without the program the test is
        skipped, never passed.
    """

    source = os.environ.get(CONST_PLC_ENV, '')
    if not source:
        pytest.skip('%s is not set: the segment list cannot be compared with sf1_plc_v4.script.' % CONST_PLC_ENV)
    with open(source, 'rb') as handle:
        program = handle.read()
    assert hashlib.sha256(program).hexdigest() == CONST_PLC_SHA256 == reference['source_sha256']
    segments, cases, indent, previous = set(), [], None, None
    for line in program.decode('utf-8').split('\n'):
        branch = CONST_CASE_BRANCH.match(line)
        if branch:
            indent, previous = branch.group(1), 'home'
            cases.append(int(branch.group(2)))
            continue
        if indent is not None and line.strip() and len(line) - len(line.lstrip()) <= len(indent):
            indent = None
        move = CONST_MOVE.match(line) if indent is not None else None
        if move and move.group(1) != previous:
            segments.add((previous, move.group(1)))
            previous = move.group(1)
    assert cases == list(range(1, 22))
    assert sorted(segments) == sorted(tuple(s) for s in reference['segments'])
