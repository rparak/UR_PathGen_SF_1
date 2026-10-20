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
# File Name: ../test/conftest.py
## =========================================================================== ##

# System (Default)
import os
import sys
# Regular expressions (Default)
import re
# Shell utilities (Default)
import shutil
# Pytest (Testing framework) [pip3 install pytest]
import pytest

"""
Description:
    Initialization of constants.

    Shared fixtures. src/ is put on sys.path here (there are no __init__.py files).
"""
# The repository root.
#   Unit [string]
CONST_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if os.path.join(CONST_ROOT, 'src') not in sys.path:
    sys.path.insert(0, os.path.join(CONST_ROOT, 'src'))


@pytest.fixture(scope='session')
def root():
    """
    Description:
        The repository root.
    """

    return CONST_ROOT


@pytest.fixture(scope='session')
def cfg():
    """
    Description:
        The shipped cell configuration.
    """

    from Configuration.Core import Load_Configuration
    return Load_Configuration()


@pytest.fixture(scope='session')
def points():
    """
    Description:
        Every learned point of the owner's program sf1_plc_v4.script (the committed extract, the single
        source of learned points; A = home, B = drop_rotate_1).
    """

    from URScript.Core import Read_URScript
    with open(os.path.join(CONST_ROOT, 'Data', 'Programs', 'sf1_plc_v4', 'sf1_plc_v4_positions.urscript'), encoding='utf-8') as handle:
        return Read_URScript(handle.read()).Points


@pytest.fixture(scope='session')
def robot_scene(cfg):
    """
    Description:
        The scene without the environment (robot, coupling, gripper).
    """

    from Scene.Core import Scene_Cls
    return Scene_Cls(cfg, with_environment=False)


@pytest.fixture(scope='session')
def scene(cfg):
    """
    Description:
        The scene with the environment collision set (no visual mesh).
    """

    from Scene.Core import Scene_Cls
    return Scene_Cls(cfg, with_environment=True)


@pytest.fixture
def perturbed_copy(root, tmp_path):
    """
    Description:
        A copy of the repository in which one learned point does not agree with its pose: src,
        Example, Configuration and Data copied under tmp_path (models and CAD linked, never
        changed), and in one points file the base joint of one point's _q turned by 0.05 rad (about
        25 mm at the TCP, beyond the 10 mm sanity bound). The scripts find the project root from
        their own path, so a script run from the copy reads the perturbed file.
    """

    def Make(relative: str, name: str) -> str:
        copy = str(tmp_path / 'repository')
        for folder in ('src', 'Example', 'Configuration', 'Data'):
            shutil.copytree(os.path.join(root, folder), os.path.join(copy, folder), ignore=shutil.ignore_patterns('__pycache__'))
        for folder in ('models', 'CAD'):
            os.symlink(os.path.join(root, folder), os.path.join(copy, folder))
        path = os.path.join(copy, relative)
        with open(path, encoding='utf-8') as handle:
            lines = handle.read().split('\n')
        prefix = 'global %s_q=[' % name
        index = next(i for i, line in enumerate(lines) if line.lstrip().startswith(prefix))
        head, tail = lines[index].split(prefix, 1)
        first, rest = tail.split(',', 1)
        lines[index] = '%s%s%r,%s' % (head, prefix, float(first) + 0.05, rest)
        with open(path, 'w', encoding='utf-8') as handle:
            handle.write('\n'.join(lines))
        return copy

    return Make


@pytest.fixture(scope='session')
def fk_refusal():
    """
    Description:
        The [ERROR] line of a script that refuses a learned point outside the FK sanity bound
        (Check_FK_Consistency's sentence, the 10 mm bound typed here): the point and its position
        residual in mm.
    """

    return re.compile(r'^\[ERROR\] Point (\w+): DH FK of its _q with the configured TCP is ([0-9.]+) mm / [0-9.]+ deg from '
                      r'its _p; the sanity bound is 10\.00 mm', re.M)


@pytest.fixture(scope='session')
def stl_warning(cfg):
    """
    Description:
        A check of a viewer's output for the visual-STL warning: the lines starting with
        '[WARNING] The visual STL'. With --with-stl there is exactly one, saying the STL is the
        original CAD and naming every correction of the shipped configuration (each assembly
        override with its x translation in mm, each added box); without it there is none.
    """

    def Check(stdout: str, expected: bool) -> None:
        lines = [line for line in stdout.splitlines() if line.startswith('[WARNING] The visual STL')]
        if not expected:
            assert lines == [], lines
            return
        assert len(lines) == 1, stdout
        assert 'original CAD' in lines[0] and 'does not match the collision geometry' in lines[0]
        for name, _, translation, _ in cfg.Assembly_Overrides:
            assert '%s moved by [%.0f,' % (name, translation[0] * 1e3) in lines[0], name
        for name, _, _, _ in cfg.Added_Boxes:
            assert 'the box %s added' % name in lines[0], name

    return Check
