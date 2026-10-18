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
#   ../URScript/Core — the case extract
from URScript.Core import Read_Program_Excerpt

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


@pytest.mark.parametrize('case, generated_arrival', [(17, True), (18, False)])
def test_the_arrival_at_home_is_generated_only_where_the_program_blends_into_s1(root, case, generated_arrival):
    """
    Description:
        Whole_Case_Entries keeps the program's arrival at home verbatim unless that line blends
        (case 17: r=0.01, typed from the program); then the arrival is a generated movej to home
        (r=0), because a blend into a generated segment is not checked. Every program line before
        the first and after the last home is kept, in order.
    """

    plan = _Plan_Module(root)
    with open(os.path.join(root, 'Data', 'Programs', 'sf1_plc_v4', 'sf1_plc_v4_cases_17_18.script'), encoding='utf-8') as handle:
        lines = Read_Program_Excerpt(handle.read())[case]
    entries = plan.Whole_Case_Entries(lines, plan.CONST_CASES[case], {'S1': ['v1'], 'S2': ['v2'], 'S3': ['v3']})
    homes = [k for k, line in enumerate(lines) if line.Kind == 'movej' and line.Target == 'home']
    first = [e for e in entries if e[0] != 'comment'][len([1 for line in lines[:homes[0]] if line.Kind in ('movej', 'movel')])]
    assert first == (('movej', 'home') if generated_arrival else ('program', lines[homes[0]].Raw))
    programs = [value for kind, value in entries if kind == 'program']
    before = [line.Raw for line in lines[:homes[0]] if line.Kind in ('movej', 'movel')]
    after = [line.Raw for line in lines[homes[1] + 1:] if line.Kind in ('movej', 'movel')]
    assert programs[:len(before)] == before and programs[len(programs) - len(after):] == after
