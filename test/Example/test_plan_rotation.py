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
# File Name: ../test/Example/test_plan_rotation.py

# System (Default)
import os
# Dataclasses (Data Classes)
import dataclasses
# Importlib (Import machinery)
import importlib.util
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.:
#   ../Configuration/Core — the error a missing case raises
from Configuration.Core import Configuration_Error_Cls

"""
Description:
    Initialization of constants.

    The gripper opening of a case's path comes from the configuration, per case (Tool.Holding_Openings),
    and the open state is the full 2F-85 stroke.
"""


def _Plan_Module(root):
    spec = importlib.util.spec_from_file_location('plan_rotation', os.path.join(root, 'Example', 'plan_rotation.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_holding_opening_is_the_cases_configured_value(root, cfg):
    """
    Description:
        While the part is held, each case uses its own configured opening (two cases with different
        openings give different values; Tool.Opening is not used); open is 85 mm in every case.
    """

    plan = _Plan_Module(root)
    two = dataclasses.replace(cfg, Opening=0.030, Holding_Openings={17: (0.050, 'a'), 18: (0.040, 'b')})
    assert plan.Opening(two, 17, 'holding') == 0.050
    assert plan.Opening(two, 18, 'holding') == 0.040
    assert plan.Opening(two, 17, 'open') == plan.Opening(two, 18, 'open') == 0.085


def test_a_case_without_a_holding_opening_is_refused(root, cfg):
    """
    Description:
        A case that Tool.Holding_Openings does not list is refused, naming the case and the field,
        never given a default.
    """

    plan = _Plan_Module(root)
    with pytest.raises(Configuration_Error_Cls, match=r'Tool\.Holding_Openings has no entry for case 18'):
        plan.Opening(dataclasses.replace(cfg, Holding_Openings={17: (0.050, 'a')}), 18, 'holding')
