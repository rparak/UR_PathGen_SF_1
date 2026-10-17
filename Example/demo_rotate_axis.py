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
# File Name: ../Example/demo_rotate_axis.py
## =========================================================================== ##

# System (Default)
import os
import sys
#   Add access if it is not in the system path.
CONST_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src')
if CONST_SRC not in sys.path:
    sys.path.append(CONST_SRC)
# Argparse (Command-line parsing)
import argparse
# Math (Mathematical functions)
import math
# Time (Time access)
import time
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# MuJoCo (Physics simulator) [pip3 install mujoco]
import mujoco
# Custom Lib.:
#   ../Configuration/Core — the cell configuration
from Configuration.Core import Load_Configuration, Configuration_Error_Cls, CONST_DEFAULT_CONFIG_FILE, CONST_PROJECT_ROOT
#   ../URScript/Core — the learned points
from URScript.Core import Read_URScript
#   ../Scene/Core — the scene and its clearance check
from Scene.Core import Scene_Cls

"""
Description:
    Initialization of constants.
"""
# The learned points and the one the motion starts from.
#   Unit [string]
CONST_POSITIONS = os.path.join(CONST_PROJECT_ROOT, 'Data', 'Positions', 'SF1_Positions.urscript')
CONST_START = 'home_safe'
# The rotated joint (0 = base, shoulder_pan) and the amplitude around the start (owner, Q5).
#   Unit [int], [float] (deg)
CONST_JOINT = 0
CONST_AMPLITUDE = 45.0
# Replay rate of the rotated joint in the viewer: for watching only; with movej r=0 the speed does
# not change the path that is checked.
#   Unit [float] (deg/s)
CONST_REPLAY_RATE = 30.0
# Frames per second of the replay.
#   Unit [float] (1/s)
CONST_FPS = 60.0
# The two labels; which one is shown is decided by the clearance pre-check, never by hand.
#   Unit [string]
CONST_LABEL_CHECKED = 'clearance checked'
CONST_LABEL_UNCHECKED = 'VISUAL ONLY - UNCHECKED'


def Motion_Waypoints(q_start: tp.Sequence[float], joint: int, amplitude: float) -> tp.List[np.ndarray]:
    """
    Description:
        Start -> +amplitude -> -amplitude -> start, on one joint.

    Args:
        (1) q_start [Vector<float> 1x6]: The start (rad).
        (2) joint [int]: The rotated joint.
        (3) amplitude [float]: The amplitude (rad).

    Returns:
        (1) parameter [List<Vector<float> 1x6>]: The waypoints.
    """

    waypoints = []
    for offset in (0.0, amplitude, -amplitude, 0.0):
        q = np.array(q_start, dtype=float)
        q[joint] += offset
        waypoints.append(q)
    return waypoints


def Replay_Frames(waypoints: tp.List[np.ndarray], joint: int, rate: float, fps: float) -> tp.List[np.ndarray]:
    """
    Description:
        Joint-linear frames between consecutive waypoints at the replay rate.

    Args:
        (1) waypoints [List<Vector<float> 1x6>]: The waypoints (rad).
        (2) joint [int]: The rotated joint.
        (3) rate [float]: Joint rate (rad/s).
        (4) fps [float]: Frames per second.

    Returns:
        (1) parameter [List<Vector<float> 1x6>]: The frames.
    """

    frames = [waypoints[0]]
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        count = max(1, int(math.ceil(abs(b[joint] - a[joint]) / rate * fps)))
        frames += [a + (b - a) * k / count for k in range(1, count + 1)]
    return frames


def Check_Motion(scene: Scene_Cls, waypoints: tp.List[np.ndarray]) -> tp.Tuple[str, int, tp.List[tp.Tuple[int, tp.Any]]]:
    """
    Description:
        Check every joint-linear segment between consecutive waypoints, as movej moves, and choose
        the label from the result: CONST_LABEL_CHECKED only when no sample violates a margin.

    Args:
        (1) scene [Scene_Cls(object)]: The scene with the environment.
        (2) waypoints [List<Vector<float> 1x6>]: The waypoints (rad).

    Returns:
        (1) parameter [Tuple<string, int, List<Tuple<int, Violation_Str>>>]: The label, the number of
            samples, and every (global sample index, violation).
    """

    samples, violations = 0, []
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        count, found = scene.Check_Segment(a, b)
        violations += [(samples + k, v) for k, v in found]
        samples += count
    label = CONST_LABEL_CHECKED if not violations else CONST_LABEL_UNCHECKED
    return label, samples, violations


def main():
    """
    Description:
        Rotate the base joint +-45 deg around home_safe as a kinematic replay. Before anything is
        shown, the whole motion is checked with the segment check (joint-linear samples, as movej
        moves); the label says whether it passed. The amplitude is never reduced to make it pass.

            macOS:  .venv/bin/mjpython Example/demo_rotate_axis.py
            Linux:  .venv/bin/python Example/demo_rotate_axis.py
            Tests:  .venv/bin/python Example/demo_rotate_axis.py --headless
    """

    parser = argparse.ArgumentParser(description='Rotate one joint of the UR10e in the SF1 scene.')
    parser.add_argument('--config', default=CONST_DEFAULT_CONFIG_FILE, help='the cell configuration (default: %(default)s)')
    parser.add_argument('--headless', action='store_true', help='check and replay without a viewer')
    args = parser.parse_args()
    try:
        cfg = Load_Configuration(args.config)
    except Configuration_Error_Cls as error:
        print('[ERROR] %s' % error)
        return 1
    for level, message in cfg.Notes:
        print('[%s] %s' % (level, message))
    with open(CONST_POSITIONS, encoding='utf-8') as handle:
        start = Read_URScript(handle.read()).Points[CONST_START].Q

    # 1\ The scene (the visual mesh only when it is shown).
    t0 = time.time()
    scene = Scene_Cls(cfg, with_environment=True, with_visual=not args.headless)
    print('[INFO] Scene built in %.1f s: %d geoms.' % (time.time() - t0, scene.Model.ngeom))

    # 2\ Check the whole motion first; the label follows from the result.
    waypoints = Motion_Waypoints(start, CONST_JOINT, math.radians(CONST_AMPLITUDE))
    label, samples, violations = Check_Motion(scene, waypoints)
    if violations:
        worst = {}
        for _, v in violations:
            key = (v.Body_1, v.Body_2)
            if key not in worst or v.Distance < worst[key].Distance:
                worst[key] = v
        print('[WARNING] %s: the motion violates %d pair(s) in %d of %d samples: %s'
              % (label, len(worst), len(set(k for k, _ in violations)), samples,
                 '; '.join('%s - %s %.2f mm (margin %.1f mm)' % (v.Body_1, v.Body_2, v.Distance * 1e3, v.Margin * 1e3)
                           for v in worst.values())))
    else:
        print('[INFO] %s: %d samples of the joint-linear motion keep every margin.' % (label, samples))
    print('[INFO] LABEL: %s' % label)

    # 3\ Replay.
    frames = Replay_Frames(waypoints, CONST_JOINT, math.radians(CONST_REPLAY_RATE), CONST_FPS)
    if args.headless:
        for q in frames:
            scene.Set_Joints(q)
        print('[INFO] Replayed %d frames headless.' % len(frames))
        return 0
    import mujoco.viewer
    try:
        viewer = mujoco.viewer.launch_passive(scene.Model, scene.Data)
    except RuntimeError as error:
        # On macOS launch_passive refuses to run outside mjpython.
        print('[ERROR] The MuJoCo viewer cannot start (%s). On macOS run: .venv/bin/mjpython '
              'Example/demo_rotate_axis.py' % error)
        return 2
    with viewer:
        viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                          'SF1 demo: base +-%.0f deg' % CONST_AMPLITUDE, label))
        while viewer.is_running():
            for q in frames:
                if not viewer.is_running():
                    break
                scene.Set_Joints(q)
                viewer.sync()
                time.sleep(1.0 / CONST_FPS)
    return 0


if __name__ == '__main__':
    sys.exit(main())
