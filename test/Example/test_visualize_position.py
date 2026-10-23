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
# File Name: ../test/Example/test_visualize_position.py

# System (Default)
import os
import sys
# Subprocess (Subprocess management)
import subprocess
# Pytest (Testing framework) [pip3 install pytest]
import pytest

"""
Description:
    Initialization of constants.

    The viewer is run the way a person runs it: a fresh interpreter, from an unrelated directory,
    headless.
"""


def _Run(root, tmp_path, *args):
    return subprocess.run([sys.executable, os.path.join(root, 'Example', 'demo_visualize_position.py')] + list(args),
                          capture_output=True, text=True, cwd=str(tmp_path), timeout=300)


@pytest.mark.parametrize('argument, name, label', [
    ('home_q', 'home', 'clearance kept (taught rule)'),
    ('load3', 'load3', 'model not valid here (machine (a) interior)'),
])
def test_a_known_position_reports_its_clearance(root, tmp_path, argument, name, label):
    """
    Description:
        A learned position, named with or without _q, is shown at case 17's configured holding
        opening (50 mm, owner) and reported under the taught rule: home keeps every margin; load3,
        inside machine (a), lies in the interior hull and is labelled "model not valid here", not a
        collision.
    """

    result = _Run(root, tmp_path, '--pos', argument, '--headless')
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'Position %s_q, gripper holding at 50.0 mm (case 17: owner, 2026-10-18: the 50 x 50 mm part).' % name in result.stdout
    assert '[INFO] LABEL: %s' % label in result.stdout


@pytest.mark.parametrize('argument, fragment', [
    ('home_p', 'home_p is a TCP pose (_p); this viewer shows joint positions. Use --pos home or --pos home_q.'),
    ('home_x', 'There is no learned position "home_x"'),
])
def test_an_unknown_or_pose_name_is_refused(root, tmp_path, argument, fragment):
    """
    Description:
        A _p name and an unknown name are refused with exit 1 and a sentence naming the problem,
        before any scene is built.
    """

    result = _Run(root, tmp_path, '--pos', argument, '--headless')
    assert result.returncode == 1
    assert '[ERROR] ' + fragment in result.stdout
    assert 'Scene built' not in result.stdout


def test_list_prints_every_position_of_the_extract(root, tmp_path, points):
    """
    Description:
        --list prints every learned position of the program extract (the single source), in order.
    """

    result = _Run(root, tmp_path, '--list')
    assert result.returncode == 0
    assert [line.strip() for line in result.stdout.splitlines()[1:]] == list(points)


def test_a_learned_point_outside_the_sanity_bound_is_refused(perturbed_copy, fk_refusal, tmp_path, cfg):
    """
    Description:
        Every learned point of the extract is checked against the FK sanity bound before one is
        shown (SPEC 9.4): with drop_rotate_1's base joint turned by 0.05 rad (about 25 mm), showing home
        stops with exit 1 and drop_rotate_1's numbers, before the scene is built.
    """

    copy = perturbed_copy(os.path.join('Data', 'Programs', 'sf1_plc_v4', 'sf1_plc_v4_positions.urscript'), 'drop_rotate_1')
    result = _Run(copy, tmp_path, '--pos', 'home', '--headless')
    assert result.returncode == 1, result.stdout + result.stderr
    refusal = fk_refusal.search(result.stdout)
    assert refusal and refusal.group(1) == 'drop_rotate_1', result.stdout
    assert float(refusal.group(2)) > cfg.FK_Sanity_Bound_Position * 1e3
    assert 'Scene built' not in result.stdout


def test_with_stl_warns_that_the_stl_is_not_the_collision_model(root, tmp_path, stl_warning):
    """
    Description:
        --with-stl prints one [WARNING] that the visual STL is the original CAD without the
        collision model's corrections, naming each of them (headless: the STL itself is not loaded);
        without the option there is no such warning.
    """

    result = _Run(root, tmp_path, '--pos', 'home_q', '--headless', '--with-stl')
    assert result.returncode == 0, result.stdout + result.stderr
    stl_warning(result.stdout, True)
    result = _Run(root, tmp_path, '--pos', 'home_q', '--headless')
    assert result.returncode == 0, result.stdout + result.stderr
    stl_warning(result.stdout, False)
