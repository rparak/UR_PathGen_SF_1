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

    The gripper opening of the deliverable's path comes from the configuration (case 17's
    Tool.Holding_Openings entry), and the open state is the full 2F-85 stroke.
"""


def _Plan_Module(root):
    spec = importlib.util.spec_from_file_location('plan_rotation', os.path.join(root, 'Example', 'plan_rotation.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_holding_opening_is_the_cases_configured_value(root, cfg):
    """
    Description:
        While the part is held, the path uses case 17's configured opening (another entry's opening
        and Tool.Opening are not used); open is 85 mm.
    """

    plan = _Plan_Module(root)
    other = dataclasses.replace(cfg, Opening=0.030, Holding_Openings={17: (0.045, 'a'), 18: (0.040, 'b')})
    assert plan.Opening(other, 'holding') == 0.045
    assert plan.Opening(other, 'open') == 0.085


def test_a_configuration_without_case_17_is_refused(root, cfg):
    """
    Description:
        A configuration whose Tool.Holding_Openings does not list case 17 is refused, naming the case
        and the field, never given a default.
    """

    plan = _Plan_Module(root)
    with pytest.raises(Configuration_Error_Cls, match=r'Tool\.Holding_Openings has no entry for case 17'):
        plan.Opening(dataclasses.replace(cfg, Holding_Openings={18: (0.050, 'a')}), 'holding')


def test_the_optimised_sequence_starts_at_home_with_a_generated_move(root):
    """
    Description:
        Rotation_Entries is home -> ... -> home and nothing else: the program's move to home blends
        (r=0.01, typed from the program) into S1, so the start is a generated movej to home (r=0); S1
        - S3 are generated movej to their via points and ends; the station lines are the program's,
        verbatim; no line of the loading moves before or after appears.
    """

    plan = _Plan_Module(root)
    with open(os.path.join(root, 'Data', 'Programs', 'sf1_plc_v4', 'sf1_plc_v4_cases_17_18.script'), encoding='utf-8') as handle:
        lines = Read_Program_Excerpt(handle.read())[17]
    homes = [k for k, line in enumerate(lines) if line.Kind == 'movej' and line.Target == 'home']
    assert lines[homes[0]].Raw.rstrip().endswith(', r=0.01)')
    entries = plan.Rotation_Entries(lines[homes[0]:homes[1] + 1], {'S1': ['v1'], 'S2': ['v2'], 'S3': ['v3']})
    moves = [e for e in entries if e[0] != 'comment']
    raw = {(line.Kind, line.Target): line.Raw for line in lines[homes[0]:homes[1] + 1] if line.Kind in ('movel', 'movej')}
    assert moves == [('movej', 'home'), ('movej', 'v1'), ('movej', 'drop_rotate_1'), ('program', raw['movel', 'Waypoint_4']),
                     ('program', raw['movej', 'Waypoint_5']), ('movej', 'v2'), ('movej', 'Waypoint_18'),
                     ('program', raw['movej', 'pick_rotate_1']), ('program', raw['movel', 'Waypoint_6']), ('movej', 'v3'),
                     ('movej', 'home')]
