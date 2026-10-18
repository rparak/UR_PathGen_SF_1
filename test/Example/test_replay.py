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
# File Name: ../test/Example/test_replay.py

# System (Default)
import os
import re
import sys
# Subprocess (Subprocess management)
import subprocess
# Pytest (Testing framework) [pip3 install pytest]
import pytest

"""
Description:
    Initialization of constants.

    The replay is run the way a person runs it: a fresh interpreter, from an unrelated directory,
    headless. The expected order of the moves is read from the files with patterns typed here, not
    with the replay's reader.
"""
# The catalogue: number -> (case, the variants in order, whole case or rotation sequence only).
#   Unit [Dict<int, Tuple<int, Tuple<string>, string>>]
CONST_CATALOGUE = {1: (17, ('taught',), 'whole'), 2: (17, ('optimised',), 'whole'), 3: (18, ('taught',), 'whole'),
                   4: (18, ('optimised',), 'whole'), 5: (17, ('taught', 'optimised'), 'rotation'),
                   6: (18, ('taught', 'optimised'), 'rotation')}
# The sequence file of each variant.
#   Unit [Dict<string, Tuple<string>>]
CONST_FILES = {'taught': ('Data', 'Programs', 'sf1_plc_v4', 'sf1_plc_v4_case_%d_sequence.script'),
               'optimised': ('Data', 'Generated', 'SF1_Rotation_%d_Sequence.script')}
# A move or a gripper action in a sequence file, and in the replay's step lines.
#   Unit [Pattern]
CONST_FILE_MOVE = re.compile(r'^(movej|movel)\((?:get_inverse_kin\()?(\w+?)_[pq][,)]')
CONST_FILE_GRIPPER = re.compile(r'^# Gripper (open|close):')
CONST_STEP = re.compile(r'^\[INFO\] Step \d+: (?:(movej|movel) -> (\w+) \((taught|generated).*?\)(?:, TCP line in (\d+) steps)?, '
                        r'gripper (holding|open), '
                        r'|gripper (open|close), ([\d.]+) mm\.)')
CONST_PLAYBACK = re.compile(r'Playback (\w+): estimated ([\d.]+) s at speed ([\d.]+) -> (\d+) frames, ([\d.]+) s\.')
# The openings of case 17 and 18: holding 50 mm (Tool.Holding_Openings), open 85 mm; frames per second.
#   Unit [Dict<string, float>] (mm), [float] (1/s)
CONST_WIDTH = {'open': 85.0, 'close': 50.0}
CONST_FPS = 30.0


def _Run(root, tmp_path, *args, stdin=''):
    return subprocess.run([sys.executable, os.path.join(root, 'Example', 'replay_rotation.py')] + list(args),
                          capture_output=True, text=True, cwd=str(tmp_path), timeout=900, input=stdin)


def _Expected(root, case, variant, scope):
    """
    The operations of a variant in file order, ('move', kind, target, rule) or ('gripper', action);
    a movej to a joint target is generated, a program line taught. For the rotation scope from the
    first move to home to the last; the start (first move) dropped.
    """

    with open(os.path.join(root, *CONST_FILES[variant]).replace('%d', str(case)), encoding='utf-8') as handle:
        lines = handle.read().splitlines()
    operations = []
    for line in lines:
        move, gripper = CONST_FILE_MOVE.match(line), CONST_FILE_GRIPPER.match(line)
        if move:
            operations.append(('move', move.group(1), move.group(2), 'taught' if 'get_inverse_kin' in line or
                               move.group(1) == 'movel' else 'generated'))
        elif gripper:
            operations.append(('gripper', gripper.group(1)))
    if scope == 'rotation':
        homes = [k for k, o in enumerate(operations) if o[0] == 'move' and o[2] == 'home']
        operations = operations[homes[0]:homes[-1] + 1]
    return operations[1:]


@pytest.mark.parametrize('number', sorted(CONST_CATALOGUE))
def test_every_path_visits_its_moves_in_program_order(root, tmp_path, number):
    """
    Description:
        Each catalogue entry replays (headless, without the dense clearance check) every move of its
        files in order, each variant after the other; generated movej are replayed by the generated
        rule and program lines by the taught rule; the gripper switches at the gripper actions (85
        mm open, the case's 50 mm when closed) and the moves after an action run in its state; a movel
        is replayed on its TCP line.
    """

    case, variants, scope = CONST_CATALOGUE[number]
    result = _Run(root, tmp_path, '--path', str(number), '--headless', '--skip-check')
    assert result.returncode == 0, result.stdout + result.stderr
    steps = [m for m in (CONST_STEP.match(line) for line in result.stdout.splitlines()) if m]
    expected = [o for variant in variants for o in _Expected(root, case, variant, scope)]
    observed = [('move', m.group(1), m.group(2), m.group(3)) if m.group(1) else ('gripper', m.group(6)) for m in steps]
    assert observed == expected
    assert sum(1 for o in expected if o[0] == 'gripper') == 2 * len(variants)
    state = 'holding'
    for m, o in zip(steps, expected):
        if o[0] == 'gripper':
            assert float(m.group(7)) == CONST_WIDTH[o[1]]
            state = 'open' if o[1] == 'open' else 'holding'
        else:
            assert m.group(5) == state, m.group(0)
            # A movel runs on its straight TCP line, solved in steps of at most 5 mm (the shortest,
            # case 18's drop, is 49 mm: at least 9 steps); a movej is joint-linear.
            assert (int(m.group(4)) >= 9) if o[1] == 'movel' else m.group(4) is None, m.group(0)
    assert re.findall(r'\[INFO\] (\w+): start at (\w+) \(gripper holding\)\.', result.stdout) == [
        (variant, 'home' if scope == 'rotation' else ('load_hower' if case == 17 else 'Waypoint_16')) for variant in variants]


def test_the_speed_scales_the_playback(root, tmp_path):
    """
    Description:
        --speed 1.0 plays a path in its estimated real time and --speed 4 in a quarter of it, each to
        within one frame per move (every move is rounded up to whole frames); the estimate does not
        depend on the speed.
    """

    runs = {}
    for speed in ('1.0', '4'):
        result = _Run(root, tmp_path, '--path', '1', '--headless', '--skip-check', '--speed', speed)
        assert result.returncode == 0, result.stdout + result.stderr
        runs[float(speed)] = CONST_PLAYBACK.search(result.stdout).groups()
    moves = 22
    assert runs[1.0][1] == runs[4.0][1]
    estimate = float(runs[1.0][1])
    for speed, (_, _, _, frames, playback) in runs.items():
        assert estimate / speed <= float(playback) <= estimate / speed + moves / CONST_FPS
        assert float(playback) == pytest.approx(int(frames) / CONST_FPS, abs=1e-3)


@pytest.mark.parametrize('args, stdin, fragment', [
    (('--path', '7'), '', 'There is no path 7'),
    (('--speed', '0'), '', '--speed must be a finite number greater than 0'),
    (('--path', '1', '--speed', '-1'), '', '--speed must be a finite number greater than 0'),
    (('--path', '1', '--speed', 'nan'), '', '--speed must be a finite number greater than 0'),
    ((), '9\n', 'There is no path 9'),
])
def test_an_invalid_number_or_speed_is_refused(root, tmp_path, args, stdin, fragment):
    """
    Description:
        A path number outside the catalogue (given or typed at the prompt) and a speed that is not a
        finite number above 0 are refused with exit 1 and a sentence naming the problem, before any
        scene is built.
    """

    result = _Run(root, tmp_path, *args, '--headless', '--skip-check', stdin=stdin)
    assert result.returncode == 1
    assert '[ERROR] ' + fragment in result.stdout
    assert 'Scene built' not in result.stdout


def test_list_prints_the_catalogue(root, tmp_path):
    """
    Description:
        --list prints the six numbered paths and exits without building a scene.
    """

    result = _Run(root, tmp_path, '--list')
    assert result.returncode == 0
    assert [int(n) for n in re.findall(r'^  (\d)  case 1[78] ', result.stdout, re.M)] == [1, 2, 3, 4, 5, 6]
    assert 'Scene built' not in result.stdout


def test_one_case_is_checked_densely_and_labelled_by_kind(root, scene, tmp_path):
    """
    Description:
        Path 2 (case 17 optimised, whole case) with the dense clearance check and screenshots,
        without --with-stl: the scene is built without the visual STL (exactly the geoms of the scene
        without it); every generated movej keeps the generated rule; every move with an end inside
        machine (a) (load_hower, load3, Waypoint_10, Waypoint_13, bench_drop_11: inside the interior
        hull, KNOWN_ISSUES) is labelled "model not valid here", not as a collision; every other
        program line keeps the taught rule; one screenshot is written.
    """

    inside = {'load_hower', 'load3', 'Waypoint_10', 'Waypoint_13', 'bench_drop_11'}
    result = _Run(root, tmp_path, '--path', '2', '--screenshots', str(tmp_path / 'shots'))
    assert result.returncode == 0, result.stdout + result.stderr
    built = re.search(r'Scene built in [\d.]+ s: (\d+) geoms; visual mesh (not loaded|loaded)\.', result.stdout)
    assert built is not None and built.group(2) == 'not loaded' and int(built.group(1)) == scene.Model.ngeom
    previous = 'load_hower'
    moves = re.findall(r'^\[INFO\] Step \d+: (?:movej|movel) -> (\w+) \((taught|generated).*: (.+)\.$', result.stdout, re.M)
    assert len(moves) == 18
    for target, rule, label in moves:
        if previous in inside or target in inside:
            assert label == 'model not valid here (machine (a) interior, KNOWN_ISSUES)', (previous, target, label)
        else:
            assert label == 'clearance kept (%s rule)' % rule, (previous, target, label)
        previous = target
    assert os.listdir(tmp_path / 'shots') == ['path2_optimised.png']
