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
# File Name: ../test/Example/test_demo.py
## =========================================================================== ##

# System (Default)
import os
import sys
# Subprocess (Subprocess management)
import subprocess
# Importlib (Import machinery)
import importlib.util
# Math (Mathematical functions)
import math
# Numpy (Array computing) [pip3 install numpy]
import numpy as np

"""
Description:
    Initialization of constants.

    The demo is run the way a person runs it: a fresh interpreter, from an unrelated directory.
"""
# The label the shipped motion gets from its clearance pre-check, pinned at implementation
# (2026-10-17). A change of the cell, the margins or the motion that flips it fails here.
#   Unit [string]
CONST_SHIPPED_LABEL = 'clearance checked'


def _Demo_Module(root):
    spec = importlib.util.spec_from_file_location('demo_rotate_axis', os.path.join(root, 'Example', 'demo_rotate_axis.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_demo_runs_headless_and_reports_its_label(root, tmp_path):
    """
    Description:
        The headless demo exits 0, prints the label decided by the clearance pre-check of the
        joint-linear samples, and replays every frame.
    """

    result = subprocess.run([sys.executable, os.path.join(root, 'Example', 'demo_rotate_axis.py'), '--headless'],
                            cwd=str(tmp_path), capture_output=True, text=True, timeout=600)
    assert result.returncode == 0, result.stdout + result.stderr
    labels = [line.split('LABEL: ', 1)[1] for line in result.stdout.splitlines() if 'LABEL: ' in line]
    assert len(labels) == 1 and labels[0] in ('clearance checked', 'VISUAL ONLY - UNCHECKED')
    checked = 'keep every margin' in result.stdout
    assert (labels[0] == 'clearance checked') == checked
    assert 'Replayed' in result.stdout
    print(result.stdout)
    assert labels[0] == CONST_SHIPPED_LABEL


def test_only_the_base_joint_moves_within_its_limits(root, cfg, points):
    """
    Description:
        Every frame differs from home only in joint 0, which moves monotonically between the
        turning points +45 deg and -45 deg and stays within the configured limits.
    """

    demo = _Demo_Module(root)
    start = np.array(points['home'].Q)
    waypoints = demo.Motion_Waypoints(start, 0, math.radians(45.0))
    frames = demo.Replay_Frames(waypoints, 0, math.radians(30.0), 60.0)
    values = np.array([f[0] for f in frames])
    for f in frames:
        assert np.array_equal(np.delete(f, 0), np.delete(start, 0))
    turn_1, turn_2 = int(np.argmax(values)), int(np.argmin(values))
    assert abs(values[turn_1] - start[0] - math.radians(45.0)) < 1e-12
    assert abs(values[turn_2] - start[0] + math.radians(45.0)) < 1e-12
    assert np.all(np.diff(values[:turn_1 + 1]) > 0) and np.all(np.diff(values[turn_1:turn_2 + 1]) < 0)
    assert np.all(np.diff(values[turn_2:]) > 0)
    assert cfg.Joint_Limits_Min[0] <= values.min() and values.max() <= cfg.Joint_Limits_Max[0]


def test_the_segment_check_samples_densely_enough(scene, cfg, points):
    """
    Description:
        The demo's check uses Segment_Samples: consecutive samples are close enough that no robot
        point moves more than Clearance.Environment / 4 (bounded by sum |dq_i| * lever_i).
    """

    start = np.array(points['home'].Q)
    end = start.copy()
    end[0] += math.radians(45.0)
    samples = scene.Segment_Samples(start, end)
    step = np.abs(np.diff(samples, axis=0)).max(axis=0)
    assert float(np.dot(step, scene.Joint_Levers())) <= cfg.Clearance_Environment / 4.0 + 1e-12


def test_a_motion_into_the_side_table_is_labelled_unchecked(root, scene, points):
    """
    Description:
        The label follows the check, not the motion: hover_unload_2 (over the side table, clear)
        lowered by the shoulder until the gripper is in the side table (-0.3 rad: TCP at z = 0.678 m,
        below the 0.715 m plate) gets "VISUAL ONLY - UNCHECKED" with its violations, while the
        shipped motion gets "clearance checked".
    """

    demo = _Demo_Module(root)
    start = np.array(points['hover_unload_2'].Q)
    assert scene.Check_Clearance(start) == []
    into_table = start + np.array([0.0, -0.3, 0.0, 0.0, 0.0, 0.0])
    label, samples, violations = demo.Check_Motion(scene, [start, into_table])
    assert label == 'VISUAL ONLY - UNCHECKED'
    assert violations and samples > 1
    assert any(v.Body_1.startswith('env') or v.Body_2.startswith('env') for _, v in violations)
    shipped = demo.Motion_Waypoints(np.array(points['home'].Q), 0, math.radians(45.0))
    assert demo.Check_Motion(scene, shipped)[0] == 'clearance checked'
