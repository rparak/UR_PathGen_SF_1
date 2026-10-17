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
# File Name: ../Example/replay_rotation.py
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
# Dataclasses (Data Classes)
import dataclasses
# Struct (Binary data) and Zlib (Compression), for writing PNG files
import struct
import zlib
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
#   ../URScript/Core — the program's points and the generated points
from URScript.Core import Read_URScript
#   ../Scene/Core — the scene and the generated rule
from Scene.Core import Scene_Cls
#   ../Example/plan_rotation — the cases, segments and openings the outputs were planned with
from plan_rotation import CONST_CASES, CONST_TAUGHT_VIAS, CONST_STATION_POINTS, CONST_OPENING, CONST_POSITIONS, Segments

"""
Description:
    Initialization of constants.
"""
# The generated outputs (Example/plan_rotation.py).
#   Unit [string]
CONST_OUTPUT_DIR = os.path.join(CONST_PROJECT_ROOT, 'Data', 'Generated')
# Labels: the generated-rule check passed, or not (then the motion is shown for looking only).
#   Unit [string]
CONST_LABEL_CHECKED = 'clearance checked (generated rule)'
CONST_LABEL_FAILED = 'VISUAL ONLY - fails the generated rule'
# Replay: frames per second and the leading joint's speed (for watching only; movej with r=0 follows
# the same path at any speed).
#   Unit [float] (1/s), [float] (rad/s)
CONST_FPS, CONST_REPLAY_RATE = 30.0, 0.8
# Screenshot: size and the camera looking at the station from the robot side (world frame).
#   Unit [int] (px), [Vector<float> 1x3] (m), [float] (deg)
CONST_SHOT_SIZE = (960, 720)
CONST_SHOT_EYE, CONST_SHOT_LOOK, CONST_SHOT_FOVY = (0.55, -1.10, 1.95), (-0.45, -0.20, 1.35), 50.0


def Paths(case: int, points: tp.Dict, generated: tp.Dict, segment: str) -> tp.Dict[str, tp.List[np.ndarray]]:
    """
    Description:
        The taught and the optimised path of one segment.

    Args:
        (1) case [int]: 17 or 18.
        (2) points [Dict<string, Robot_Point_Str>]: The program's points.
        (3) generated [Dict<string, Robot_Point_Str>]: The points of the generated output.
        (4) segment [string]: 'S1', 'S2' or 'S3'.

    Returns:
        (1) parameter [Dict<string, List<Vector<float> 1x6>>]: 'taught' and 'optimised'.
    """

    start, goal = Segments(case)[segment]
    prefix = 'via_%s_%s_' % (start, goal)
    vias = sorted(name for name in generated if name.startswith(prefix))
    return {'taught': [np.asarray(points[n].Q) for n in [start] + CONST_TAUGHT_VIAS[segment] + [goal]],
            'optimised': [np.asarray(points[start].Q)] + [np.asarray(generated[n].Q) for n in vias] + [np.asarray(points[goal].Q)]}


def Label(scene: Scene_Cls, path: tp.List[np.ndarray], station_q: tp.List) -> tp.Tuple[str, int, int]:
    """
    Description:
        Check every segment of a path with the full generated-rule check and choose its label.

    Args:
        (1) scene [Scene_Cls(object)]: The scene at the segment's opening.
        (2) path [List<Vector<float> 1x6>]: The configurations (rad).
        (3) station_q [List<Vector<float> 1x6>]: The fixed station points (rad).

    Returns:
        (1) parameter [Tuple<string, int, int>]: The label, the samples and the violations.
    """

    samples = violations = 0
    for a, b in zip(path[:-1], path[1:]):
        count, found = scene.Check_Segment(a, b, kind='generated', station_q=station_q)
        samples, violations = samples + count, violations + len(found)
    return (CONST_LABEL_CHECKED if not violations else CONST_LABEL_FAILED), samples, violations


def Frames(path: tp.List[np.ndarray]) -> tp.List[np.ndarray]:
    """
    Description:
        Joint-linear frames between the configurations at the replay rate of the leading joint.

    Args:
        (1) path [List<Vector<float> 1x6>]: The configurations (rad).

    Returns:
        (1) parameter [List<Vector<float> 1x6>]: The frames.
    """

    frames = [path[0]]
    for a, b in zip(path[:-1], path[1:]):
        count = max(1, int(np.ceil(np.max(np.abs(b - a)) / CONST_REPLAY_RATE * CONST_FPS)))
        frames += [a + (b - a) * k / count for k in range(1, count + 1)]
    return frames


def Write_PNG(image: np.ndarray, path: str) -> None:
    """
    Description:
        Write an RGB image as a PNG file (no image library needed).

    Args:
        (1) image [Matrix<uint8> HxWx3]: The pixels.
        (2) path [string]: The file.
    """

    raw = b''.join(b'\x00' + image[i].tobytes() for i in range(image.shape[0]))
    def Chunk(tag: bytes, data: bytes) -> bytes:
        """
        Description:
            One PNG chunk with its length and CRC.

        Args:
            (1) tag [bytes]: The chunk type.
            (2) data [bytes]: The chunk data.

        Returns:
            (1) parameter [bytes]: The chunk.
        """

        return struct.pack('>I', len(data)) + tag + data + struct.pack('>I', zlib.crc32(tag + data) & 0xffffffff)
    with open(path, 'wb') as handle:
        handle.write(b'\x89PNG\r\n\x1a\n' + Chunk(b'IHDR', struct.pack('>IIBBBBB', image.shape[1], image.shape[0], 8, 2, 0, 0, 0))
                     + Chunk(b'IDAT', zlib.compress(raw, 6)) + Chunk(b'IEND', b''))


def Screenshot(scene: Scene_Cls, q: np.ndarray, path: str) -> None:
    """
    Description:
        Render the scene offscreen at q with the visual environment, the robot and the collision
        geometry (viewer key 3), from the station camera.

    Args:
        (1) scene [Scene_Cls(object)]: The scene (built with the visual mesh).
        (2) q [Vector<float> 1x6]: The configuration (rad).
        (3) path [string]: The PNG file.
    """

    m = scene.Model
    width, height = CONST_SHOT_SIZE
    m.vis.global_.offwidth, m.vis.global_.offheight = max(m.vis.global_.offwidth, width), max(m.vis.global_.offheight, height)
    m.vis.global_.fovy = CONST_SHOT_FOVY
    renderer = mujoco.Renderer(m, height, width)
    scene.Set_Joints(q)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    view = np.asarray(CONST_SHOT_LOOK) - np.asarray(CONST_SHOT_EYE)
    camera.lookat[:] = CONST_SHOT_LOOK
    camera.distance = float(np.linalg.norm(view))
    camera.azimuth = float(np.degrees(np.arctan2(view[1], view[0])))
    camera.elevation = float(np.degrees(np.arcsin(view[2] / np.linalg.norm(view))))
    option = mujoco.MjvOption()
    for group in range(6):
        option.geomgroup[group] = 1 if group in (1, 2, 3) else 0
    renderer.update_scene(scene.Data, camera=camera, scene_option=option)
    Write_PNG(renderer.render(), path)
    renderer.close()


def main():
    """
    Description:
        Replay the taught and the optimised path of one generated segment, each labelled by the
        full generated-rule check; with --screenshots, render each at its closest approach to the
        environment instead of opening the viewer.

            macOS:  .venv/bin/mjpython Example/replay_rotation.py --case 17 --segment S2
            Images: .venv/bin/python Example/replay_rotation.py --case 17 --segment S2 --screenshots DIR
    """

    parser = argparse.ArgumentParser(description='Replay taught vs optimised segments of the rotation sequence.')
    parser.add_argument('--config', default=CONST_DEFAULT_CONFIG_FILE, help='the cell configuration (default: %(default)s)')
    parser.add_argument('--case', type=int, choices=sorted(CONST_CASES), default=17)
    parser.add_argument('--segment', choices=('S1', 'S2', 'S3'), default='S2')
    parser.add_argument('--screenshots', default='', help='write PNG files to this directory instead of the viewer')
    args = parser.parse_args()
    print('[WARNING] Simulation passed != safe on the robot.')
    try:
        cfg = Load_Configuration(args.config)
    except Configuration_Error_Cls as error:
        print('[ERROR] %s' % error)
        return 1
    with open(CONST_POSITIONS, encoding='utf-8') as handle:
        points = Read_URScript(handle.read()).Points
    with open(os.path.join(CONST_OUTPUT_DIR, 'SF1_Rotation_%d.urscript' % args.case), encoding='utf-8') as handle:
        generated = Read_URScript(handle.read()).Points
    station_q = [points[name].Q for name in CONST_STATION_POINTS]
    scene = Scene_Cls(dataclasses.replace(cfg, Opening=CONST_OPENING[args.segment]), with_environment=True, with_visual=True)
    paths = Paths(args.case, points, generated, args.segment)
    labels = {}
    for kind, path in paths.items():
        labels[kind], samples, violations = Label(scene, path, station_q)
        print('[INFO] Case %d %s %s (%d via points): %s - %d samples, %d violations.'
              % (args.case, args.segment, kind, len(path) - 2, labels[kind], samples, violations))
    if args.screenshots:
        os.makedirs(args.screenshots, exist_ok=True)
        tcps = scene.Station_TCPs(station_q)
        for kind, path in paths.items():
            # The closest approach where the paths differ: outside the station neighbourhood (both
            # share their ends, which lie inside it).
            samples = [q for a, b in zip(path[:-1], path[1:]) for q in scene.Segment_Samples(a, b)
                       if not scene.Near_Station(q, tcps)]
            worst = min(samples, key=lambda q: scene.Environment_Distance(q, 0.2)[0])
            distance, body = scene.Environment_Distance(worst, 0.2)
            name = os.path.join(args.screenshots, 'case%d_%s_%s.png' % (args.case, args.segment, kind))
            Screenshot(scene, worst, name)
            print('[INFO] %s: %s at its closest approach outside the station neighbourhood (%.1f mm, %s).'
                  % (name, kind, distance * 1e3, body))
        return 0
    import mujoco.viewer
    try:
        viewer = mujoco.viewer.launch_passive(scene.Model, scene.Data)
    except RuntimeError as error:
        print('[ERROR] The MuJoCo viewer cannot start (%s). On macOS run: .venv/bin/mjpython Example/replay_rotation.py' % error)
        return 2
    with viewer:
        while viewer.is_running():
            for kind, path in paths.items():
                viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                                  'Case %d %s %s (%d via points)' % (args.case, args.segment, kind, len(path) - 2), labels[kind]))
                for q in Frames(path):
                    if not viewer.is_running():
                        break
                    scene.Set_Joints(q)
                    viewer.sync()
                    time.sleep(1.0 / CONST_FPS)
    return 0


if __name__ == '__main__':
    sys.exit(main())
