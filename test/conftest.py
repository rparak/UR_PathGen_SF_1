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
        The learned SF1 points (home_safe = A, drop_rot_01 = B).
    """

    from URScript.Core import Read_URScript
    with open(os.path.join(CONST_ROOT, 'Data', 'Positions', 'SF1_Positions.urscript'), encoding='utf-8') as handle:
        return Read_URScript(handle.read()).Points


@pytest.fixture(scope='session')
def plc_points():
    """
    Description:
        Every learned point of the owner's program sf1_plc_v4.script (the committed extract).
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
