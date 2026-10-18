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
# Math (Mathematical functions)
import math
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
from Configuration.Core import Load_Configuration, Cell_Configuration_Str, Configuration_Error_Cls, CONST_DEFAULT_CONFIG_FILE
#   ../URScript/Core — the motion sequences and their points
from URScript.Core import Read_URScript, Read_Motion_Sequence, Motion_Str, URScript_Error_Cls
#   ../Kinematics/Core — the TCP path of a movel
from Kinematics.Core import TCP_Pose, Matrix_To_Pose, Linear_Path, Kinematics_Error_Cls
#   ../Scene/Core — the scenes and the collision geometry group
from Scene.Core import Scene_Cls, CONST_COLLISION_GROUP, CONST_GRIPPER_PREFIX, CONST_PAD_SPAN_RGBA
#   ../Planner/Core — the duration estimates and their profile
from Planner.Core import Segment_Duration, Linear_Duration, Profile_Fraction
#   ../Example/plan_rotation — the cases, the openings, the outputs and the clearance labels
from plan_rotation import (CONST_CASES, CONST_TAUGHT_VIAS, CONST_STATION_POINTS, CONST_PROGRAM_DIR, CONST_OUTPUT_DIR, CONST_POSITIONS,
                           CONST_MACHINE_INTERIOR, CONST_LABEL_CLEAR, CONST_LABEL_VIOLATED, CONST_LABEL_INVALID, Segments,
                           Opening)

"""
Description:
    Initialization of constants.
"""
# The paths that can be replayed: number -> (case, the variants shown one after the other, scope,
# name). 'whole': the whole case as written (Data/Programs/sf1_plc_v4/ taught, Data/Generated/
# optimised); 'rotation': its rotation sequence only, home -> ... -> home.
#   Unit [Dict<int, Tuple<int, Tuple<string>, string, string>>]
CONST_CATALOGUE = {
    1: (17, ('taught',), 'whole', 'case 17 taught (whole case)'),
    2: (17, ('optimised',), 'whole', 'case 17 optimised (whole case)'),
    3: (18, ('taught',), 'whole', 'case 18 taught (whole case)'),
    4: (18, ('optimised',), 'whole', 'case 18 optimised (whole case)'),
    5: (17, ('taught', 'optimised'), 'rotation', 'case 17 rotation sequence only (home -> home), taught then optimised'),
    6: (18, ('taught', 'optimised'), 'rotation', 'case 18 rotation sequence only (home -> home), taught then optimised'),
}
# The files of each variant: (points, motion sequence).
#   Unit [Dict<string, Tuple<string, string>>]
CONST_FILES = {'taught': (os.path.join(CONST_PROGRAM_DIR, 'sf1_plc_v4_case_%d.urscript'),
                          os.path.join(CONST_PROGRAM_DIR, 'sf1_plc_v4_case_%d_sequence.script')),
               'optimised': (os.path.join(CONST_OUTPUT_DIR, 'SF1_Rotation_%d.urscript'),
                             os.path.join(CONST_OUTPUT_DIR, 'SF1_Rotation_%d_Sequence.script'))}
# The gripper state on entering a case: holding its part. Inferred: the case drops a part at the
# station first and picks it up again, and no gripper action precedes the drop or follows the last
# pick (assumption; the owner did not confirm it, 2026-10-18).
#   Unit [string]
CONST_ENTRY_STATE = 'holding'
# Frames per second of the replay; the playback time of a move is its estimated duration / speed.
#   Unit [float] (1/s)
CONST_FPS = 30.0
# The strict comparison of entries 5 and 6: S1 - S3 of both variants judged by the generated rule.
#   Unit [string]
CONST_STRICT_CHECKED = 'keeps the generated rule'
CONST_STRICT_FAILED = 'fails the generated rule'
# Screenshot: size and the camera looking at the station from the robot side (world frame).
#   Unit [int] (px), [Vector<float> 1x3] (m), [float] (deg)
CONST_SHOT_SIZE = (960, 720)
CONST_SHOT_EYE, CONST_SHOT_LOOK, CONST_SHOT_FOVY = (0.55, -1.10, 1.95), (-0.45, -0.20, 1.35), 50.0


@dataclasses.dataclass
class Move_Str:
    """
    Description:
        One move of a replayed path, or a gripper action (Kind 'gripper', no configurations).
    """

    # 'movej', 'movel' or 'gripper'; the target point, or 'open' / 'close'.
    #   Unit [string]
    Kind: str = ''
    Target: str = ''
    # The clearance rule of the move: 'generated' for a generated movej, 'taught' for a program line.
    #   Unit [string]
    Rule: str = ''
    # The gripper state during the move: 'holding' or 'open'.
    #   Unit [string]
    State: str = ''
    # The configurations of the move: its two ends (movej), or the solved TCP line (movel).
    #   Unit [List<Vector<float> 1x6>] (rad)
    Configurations: tp.List[np.ndarray] = dataclasses.field(default_factory=list)
    # Its estimated duration, and the distance, acceleration and speed of its profile.
    #   Unit [float] (s), [float] (rad or m), [float] (rad/s^2 or m/s^2), [float] (rad/s or m/s)
    Duration: float = 0.0
    Distance: float = 0.0
    A: float = 0.0
    V: float = 0.0
    # The section comment of the line in the file.
    #   Unit [string]
    Section: str = ''
    # The clearance label (Check_Moves); '' until checked.
    #   Unit [string]
    Label: str = ''


def Catalogue_Text() -> str:
    """
    Description:
        The numbered catalogue of the paths, one line each.

    Returns:
        (1) parameter [string]: The lines.
    """

    return '\n'.join('  %d  %s' % (number, entry[3]) for number, entry in sorted(CONST_CATALOGUE.items()))


def Load_Variant(case: int, variant: str) -> tp.Tuple[tp.List[Motion_Str], tp.Dict]:
    """
    Description:
        The operations of one variant of a case and its points (the variant's own points file).

    Args:
        (1) case [int]: 17 or 18.
        (2) variant [string]: 'taught' or 'optimised'.

    Returns:
        (1) parameter [Tuple<List<Motion_Str>, Dict<string, Robot_Point_Str>>]: Operations and points.
    """

    points_file, sequence_file = (name % case for name in CONST_FILES[variant])
    with open(points_file, encoding='utf-8') as handle:
        points = Read_URScript(handle.read()).Points
    with open(sequence_file, encoding='utf-8') as handle:
        return Read_Motion_Sequence(handle.read()), points


def Rotation_Only(operations: tp.List[Motion_Str], home: str) -> tp.List[Motion_Str]:
    """
    Description:
        The rotation sequence of a whole case: from its first move to home to its last, inclusive.

    Args:
        (1) operations [List<Motion_Str>]: The whole case.
        (2) home [string]: The home point's name.

    Returns:
        (1) parameter [List<Motion_Str>]: The operations of home -> ... -> home.
    """

    homes = [k for k, o in enumerate(operations) if o.Kind == 'movej' and o.Target == home]
    if len(homes) < 2:
        raise URScript_Error_Cls('The sequence moves to %s %d time(s); a rotation sequence needs two.' % (home, len(homes)))
    return operations[homes[0]:homes[-1] + 1]


def Build_Moves(operations: tp.List[Motion_Str], points: tp.Dict, cfg: Cell_Configuration_Str) -> tp.List[Move_Str]:
    """
    Description:
        The moves of a path from its operations. The path starts at its first target (the robot
        enters a case where the previous case left it). A movej is joint-linear, timed by the
        trapezoid of its leading joint with the line's a and v; a movel is the straight TCP line to
        the DH TCP of its target's _q, solved for joints step by step (Linear_Path), timed by the
        trapezoid on the TCP path. Blends are not replayed: every move stops at its target. A gripper
        action switches the state; its own duration is not modelled.

    Args:
        (1) operations [List<Motion_Str>]: The operations in order.
        (2) points [Dict<string, Robot_Point_Str>]: The points of the variant.
        (3) cfg [Cell_Configuration_Str(object)]: The cell configuration (the TCP).

    Returns:
        (1) parameter [List<Move_Str>]: The moves and gripper actions, the start excluded.
    """

    q = np.asarray(points[operations[0].Target].Q, dtype=float)
    state, moves = CONST_ENTRY_STATE, []
    for o in operations[1:]:
        if o.Kind == 'gripper':
            state = 'open' if o.Target == 'open' else 'holding'
            moves.append(Move_Str('gripper', o.Target, State=state, Section=o.Section))
            continue
        target = np.asarray(points[o.Target].Q, dtype=float)
        if o.Kind == 'movej':
            configurations = [q, target]
            distance = float(np.max(np.abs(target - q)))
            duration = Segment_Duration(q, target, o.A, o.V)
        else:
            configurations = Linear_Path(q, target, cfg.TCP_Offset)
            p_a, p_b = Matrix_To_Pose(TCP_Pose(q, cfg.TCP_Offset)), Matrix_To_Pose(TCP_Pose(target, cfg.TCP_Offset))
            distance = float(np.linalg.norm(np.asarray(p_b[:3]) - np.asarray(p_a[:3])))
            duration = Linear_Duration(p_a, p_b, o.A, o.V)
        moves.append(Move_Str(o.Kind, o.Target, 'generated' if o.Generated else 'taught', state, configurations,
                              duration, distance, o.A, o.V, o.Section))
        q = target
    return moves


def Move_Samples(scene: Scene_Cls, move: Move_Str) -> tp.List[np.ndarray]:
    """
    Description:
        The checked samples of a move: the segment samples (SPEC 9.7) between consecutive
        configurations, so a movel is sampled along its TCP line.

    Args:
        (1) scene [Scene_Cls(object)]: Any scene of the cell (the sampling depends on the robot only).
        (2) move [Move_Str(object)]: The move.

    Returns:
        (1) parameter [List<Vector<float> 1x6>]: The samples (rad).
    """

    samples: tp.List[np.ndarray] = []
    for a, b in zip(move.Configurations[:-1], move.Configurations[1:]):
        found = list(scene.Segment_Samples(a, b))
        samples += found if not samples else found[1:]
    return samples


def Check_Moves(moves: tp.List[Move_Str], scenes: tp.Dict[str, Scene_Cls], cfg: Cell_Configuration_Str,
                station_q: tp.List) -> tp.Dict[str, int]:
    """
    Description:
        Label every move by the rule of its kind, in the scene of its gripper state:
        CONST_LABEL_INVALID when a sample comes within Clearance.Environment of the machine (a)
        interior (the model is not valid there), else violated or clear.

    Args:
        (1) moves [List<Move_Str>]: The moves; their Label is set.
        (2) scenes [Dict<string, Scene_Cls>]: The scene of each gripper state.
        (3) cfg [Cell_Configuration_Str(object)]: The cell configuration.
        (4) station_q [List<Vector<float> 1x6>]: The fixed station points (generated rule).

    Returns:
        (1) parameter [Dict<string, int>]: The number of moves per label.
    """

    counts: tp.Dict[str, int] = {}
    for move in moves:
        if move.Kind == 'gripper':
            continue
        scene = scenes[move.State]
        tcps = scene.Station_TCPs(station_q) if move.Rule == 'generated' else None
        violated = interior = False
        for q in Move_Samples(scene, move):
            violations = scene.Check_Configuration(q, move.Rule, tcps)
            violated = violated or bool(violations)
            if any('env' in (v.Body_1, v.Body_2) for v in violations):
                interior = interior or bool(scene.Components_Within(q, CONST_MACHINE_INTERIOR, cfg.Clearance_Environment))
        move.Label = CONST_LABEL_INVALID if interior else (CONST_LABEL_VIOLATED if violated else CONST_LABEL_CLEAR) % move.Rule
        counts[move.Label] = counts.get(move.Label, 0) + 1
    return counts


def Frames(moves: tp.List[Move_Str], speed: float) -> tp.List[tp.Tuple[int, float]]:
    """
    Description:
        The replay frames: per move, enough frames at CONST_FPS that it plays for its estimated
        duration / speed, each frame (move index, time into the move (s, estimated real time)).

    Args:
        (1) moves [List<Move_Str>]: The moves.
        (2) speed [float]: Playback speed relative to the estimated real time (> 0).

    Returns:
        (1) parameter [List<Tuple<int, float>>]: The frames.
    """

    frames = []
    for k, move in enumerate(moves):
        if move.Kind == 'gripper':
            frames.append((k, 0.0))
            continue
        count = max(1, int(math.ceil(move.Duration * CONST_FPS / speed)))
        frames += [(k, move.Duration * i / count) for i in range(1, count + 1)]
    return frames


def Frame_Configuration(move: Move_Str, t: float) -> np.ndarray:
    """
    Description:
        The configuration of a move at time t under its trapezoid profile: joint-linear for a movej,
        along the solved TCP line for a movel (its configurations are evenly spaced on the line).

    Args:
        (1) move [Move_Str(object)]: The move.
        (2) t [float]: Time into the move (s).

    Returns:
        (1) parameter [Vector<float> 1x6]: The configuration (rad).
    """

    position = Profile_Fraction(t, move.Distance, move.A, move.V) * (len(move.Configurations) - 1)
    k = min(int(position), len(move.Configurations) - 2)
    a, b = move.Configurations[k], move.Configurations[k + 1]
    return a + (b - a) * (position - k)


def Move_Text(move: Move_Str) -> str:
    """
    Description:
        One line naming a move or a gripper action.

    Args:
        (1) move [Move_Str(object)]: The move.

    Returns:
        (1) parameter [string]: The text.
    """

    if move.Kind == 'gripper':
        return 'gripper %s' % move.Target
    section = move.Section.split(':')[0]
    return '%s -> %s (%s%s)' % (move.Kind, move.Target, move.Rule, ', ' + section if section else '')


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
    return (CONST_STRICT_CHECKED if not violations else CONST_STRICT_FAILED), samples, violations


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
        (1) scene [Scene_Cls(object)]: The scene (with or without the visual mesh).
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


def Show_Gripper(view: Scene_Cls, state_scene: Scene_Cls) -> None:
    """
    Description:
        Show the gripper of state_scene in the viewer's scene (display only): its finger joints, and
        the pad-span hull only when both scenes have the same opening. Every check uses the scene of
        the move's own state.

    Args:
        (1) view [Scene_Cls(object)]: The scene shown in the viewer.
        (2) state_scene [Scene_Cls(object)]: The scene of the gripper state to show.
    """

    m, d, source = view.Model, view.Data, state_scene.Data
    for j in range(m.njnt):
        if m.joint(j).name.startswith(CONST_GRIPPER_PREFIX):
            d.qpos[m.jnt_qposadr[j]] = source.qpos[m.jnt_qposadr[j]]
    pad = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'pad_span')
    m.geom_rgba[pad, 3] = CONST_PAD_SPAN_RGBA[3] if state_scene is view else 0.0


def Print_Steps(playlist: tp.List, openings: tp.Dict[str, float], speed: float) -> None:
    """
    Description:
        Print every step of the replayed variants (headless and screenshot runs): each move with
        its rule, gripper state, estimated duration and label, each gripper action with its
        opening, and the playback time at the speed.

    Args:
        (1) playlist [List<Tuple<string, string, List<Move_Str>, List<Tuple<int, float>>>]: Per variant: its
                                                                                            name, start, moves, frames.
        (2) openings [Dict<string, float>]: The opening of each gripper state (m).
        (3) speed [float]: Playback speed relative to the estimated real time.
    """

    for variant, start, moves, frames in playlist:
        print('[INFO] %s: start at %s (gripper %s).' % (variant, start, CONST_ENTRY_STATE))
        elapsed = 0.0
        for k, move in enumerate(moves):
            if move.Kind == 'gripper':
                print('[INFO] Step %d: gripper %s, %.1f mm.' % (k + 1, move.Target, openings[move.State] * 1e3))
                continue
            elapsed += move.Duration
            line = ', TCP line in %d steps' % (len(move.Configurations) - 1) if move.Kind == 'movel' else ''
            print('[INFO] Step %d: %s%s, gripper %s, %.2f s (t = %.2f s): %s.'
                  % (k + 1, Move_Text(move), line, move.State, move.Duration, elapsed, move.Label))
        moving = sum(1 for k, _ in frames if moves[k].Kind != 'gripper')
        print('[INFO] Playback %s: estimated %.3f s at speed %g -> %d frames, %.3f s.'
              % (variant, sum(m.Duration for m in moves), speed, moving, moving / CONST_FPS))


def Write_Screenshots(folder: str, number: int, paths: tp.Dict, scenes: tp.Dict[str, Scene_Cls], station_q: tp.List) -> None:
    """
    Description:
        Render each variant at its closest approach to the environment outside the station
        neighbourhood and outside machine (a) (where the model is not valid), in the scene of the
        move's gripper state.

    Args:
        (1) folder [string]: The output directory.
        (2) number [int]: The catalogue number (file names).
        (3) paths [Dict<string, Tuple<string, List<Move_Str>>>]: Per variant: start and moves.
        (4) scenes [Dict<string, Scene_Cls>]: The scene of each gripper state.
        (5) station_q [List<Vector<float> 1x6>]: The fixed station points (rad).
    """

    os.makedirs(folder, exist_ok=True)
    tcps = scenes['holding'].Station_TCPs(station_q)
    for variant, (_, moves) in paths.items():
        candidates = [(q, move.State) for move in moves if move.Kind != 'gripper' and move.Label != CONST_LABEL_INVALID
                      for q in Move_Samples(scenes[move.State], move) if not scenes[move.State].Near_Station(q, tcps)]
        worst, state = min(candidates, key=lambda c: scenes[c[1]].Environment_Distance(c[0], 0.2)[0])
        distance, body = scenes[state].Environment_Distance(worst, 0.2)
        file = os.path.join(folder, 'path%d_%s.png' % (number, variant))
        Screenshot(scenes[state], worst, file)
        print('[INFO] %s: %s at its closest approach outside the station neighbourhood and machine (a) (%.1f mm, %s).'
              % (file, variant, distance * 1e3, body))


def Play(viewer: tp.Any, playlist: tp.List, scenes: tp.Dict[str, Scene_Cls], openings: tp.Dict[str, float], title: str,
         speed: float) -> None:
    """
    Description:
        Play every variant once in the viewer at CONST_FPS, the gripper shown in each move's state,
        with the overlay: the path, the variant, the current move, the gripper, the estimated
        elapsed time and the move's clearance label.

    Args:
        (1) viewer [mujoco.viewer.Handle]: The passive viewer of scenes['holding'].
        (2) playlist [List<Tuple<string, string, List<Move_Str>, List<Tuple<int, float>>>]: As Print_Steps.
        (3) scenes [Dict<string, Scene_Cls>]: The scene of each gripper state.
        (4) openings [Dict<string, float>]: The opening of each gripper state (m).
        (5) title [string]: The path's number and name.
        (6) speed [float]: Playback speed relative to the estimated real time.
    """

    view = scenes['holding']
    for variant, _, moves, frames in playlist:
        elapsed, current = 0.0, -1
        Show_Gripper(view, scenes[CONST_ENTRY_STATE])
        for k, t in frames:
            if not viewer.is_running():
                return
            move = moves[k]
            if k != current:
                elapsed += moves[current].Duration if current >= 0 and moves[current].Kind != 'gripper' else 0.0
                current = k
                Show_Gripper(view, scenes[move.State])
            if move.Kind != 'gripper':
                view.Set_Joints(Frame_Configuration(move, t))
            viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                              'Path %s\n%s\n%s\ngripper %s %.1f mm\nt = %.2f s (estimated, x%g)'
                              % (title, variant, Move_Text(move), move.State, openings[move.State] * 1e3, elapsed + t, speed),
                              move.Label or '-'))
            viewer.sync()
            time.sleep(1.0 / CONST_FPS)


def Choose_Path(path: int) -> int:
    """
    Description:
        The catalogue number: the --path value, or asked for after printing the catalogue.

    Args:
        (1) path [int]: --path, 0 when not given.

    Returns:
        (1) parameter [int]: The number, 0 when none valid was given.
    """

    if not path:
        print('[INFO] Paths:\n%s' % Catalogue_Text())
        try:
            answer = input('[INFO] Path number: ').strip()
        except EOFError:
            answer = ''
        path = int(answer) if answer.isdigit() else -1
    if path not in CONST_CATALOGUE:
        print('[ERROR] There is no path %s; choose one of %s (--list prints the catalogue).'
              % (path if path != -1 else 'with that number', ', '.join(str(n) for n in sorted(CONST_CATALOGUE))))
        return 0
    return path


def main():
    """
    Description:
        Replay a path of the catalogue in one viewer window: movej joint-linear, movel on its
        straight TCP line, at the estimated real time times --speed, the gripper switching between
        the case's holding opening and open at the gripper actions. Every move is labelled by the
        rule of its kind (taught lines: taught rule; generated movej: generated rule); a move inside
        machine (a) is labelled "model not valid here" (KNOWN_ISSUES). Entries 5 and 6 also print
        the strict comparison: S1 - S3 of both variants judged by the generated rule.

            macOS:   .venv/bin/mjpython Example/replay_rotation.py --path 2
            List:    .venv/bin/python Example/replay_rotation.py --list
            Images:  .venv/bin/python Example/replay_rotation.py --path 2 --screenshots DIR
            Tests:   .venv/bin/python Example/replay_rotation.py --path 2 --headless [--skip-check]

        The scene is built without the visual STL and the viewer starts with the collision geometry
        (geom group 3) shown; --with-stl loads the STL (geom group 1) as well.
    """

    parser = argparse.ArgumentParser(description='Replay a taught or optimised path of the rotation cases.')
    parser.add_argument('--config', default=CONST_DEFAULT_CONFIG_FILE, help='the cell configuration (default: %(default)s)')
    parser.add_argument('--path', type=int, default=0, help='the catalogue number (without it the catalogue is printed and asked)')
    parser.add_argument('--list', action='store_true', help='print the catalogue and exit')
    parser.add_argument('--speed', type=float, default=1.0, help='playback relative to the estimated real time (default: 1.0)')
    parser.add_argument('--with-stl', action='store_true', help='load the visual STL of the cell (default: collision geometry only)')
    parser.add_argument('--screenshots', default='', help='write PNG files to this directory instead of the viewer')
    parser.add_argument('--headless', action='store_true', help='replay without a viewer, printing every move')
    parser.add_argument('--skip-check', action='store_true', help='do not check clearance (moves are labelled "not checked")')
    args = parser.parse_args()
    if args.list:
        print('[INFO] Paths:\n%s' % Catalogue_Text())
        return 0
    if not (math.isfinite(args.speed) and args.speed > 0.0):
        print('[ERROR] --speed must be a finite number greater than 0 (1.0 = estimated real time), not %r.' % args.speed)
        return 1
    number = Choose_Path(args.path)
    if not number:
        return 1
    case, variants, scope, name = CONST_CATALOGUE[number]
    print('[WARNING] Simulation passed != safe on the robot.')
    try:
        cfg = Load_Configuration(args.config)
        openings = {state: Opening(cfg, case, state) for state in ('holding', 'open')}
        loaded = {variant: Load_Variant(case, variant) for variant in variants}
        paths = {}
        for variant, (operations, points) in loaded.items():
            chosen = Rotation_Only(operations, CONST_CASES[case]['home']) if scope == 'rotation' else operations
            paths[variant] = (chosen[0].Target, Build_Moves(chosen, points, cfg))
    except (Configuration_Error_Cls, URScript_Error_Cls, Kinematics_Error_Cls, OSError) as error:
        print('[ERROR] %s' % error)
        return 1
    print('[INFO] Path %d: %s. Gripper: holding %.1f mm (%s), open %.1f mm.'
          % (number, name, openings['holding'] * 1e3, cfg.Holding_Openings[case][1], openings['open'] * 1e3))
    t0 = time.time()
    with_visual = args.with_stl and not (args.headless or args.screenshots)
    scenes = {'holding': Scene_Cls(dataclasses.replace(cfg, Opening=openings['holding']), with_environment=True,
                                   with_visual=with_visual)}
    scenes['open'] = (scenes['holding'] if openings['open'] == openings['holding']
                      else Scene_Cls(dataclasses.replace(cfg, Opening=openings['open']), with_environment=True))
    print('[INFO] Scene built in %.1f s: %d geoms; visual mesh %s.'
          % (time.time() - t0, scenes['holding'].Model.ngeom, 'loaded' if with_visual else 'not loaded'))
    with open(CONST_POSITIONS, encoding='utf-8') as handle:
        program_points = Read_URScript(handle.read()).Points
    station_q = [program_points[n].Q for n in CONST_STATION_POINTS]
    for variant, (start, moves) in paths.items():
        if args.skip_check:
            for move in moves:
                move.Label = '' if move.Kind == 'gripper' else 'clearance not checked (--skip-check)'
        else:
            counts = Check_Moves(moves, scenes, cfg, station_q)
            print('[INFO] %s: %s.' % (variant, '; '.join('%d move(s) %s' % (n, label) for label, n in sorted(counts.items()))))
    if scope == 'rotation' and not args.skip_check:
        # The strict comparison: S1 - S3 of both variants by the generated rule (why S2 was optimised).
        for segment in ('S1', 'S2', 'S3'):
            scene = scenes['open' if segment == 'S2' else 'holding']
            for variant, path in Paths(case, program_points, loaded['optimised'][1], segment).items():
                label, samples, violations = Label(scene, path, station_q)
                print('[INFO] Strict comparison (generated rule), case %d %s %s (%d via points): %s - %d samples, %d violations.'
                      % (case, segment, variant, len(path) - 2, label, samples, violations))
    playlist = [(variant, start, moves, Frames(moves, args.speed)) for variant, (start, moves) in paths.items()]
    if args.headless or args.screenshots:
        Print_Steps(playlist, openings, args.speed)
        if args.screenshots:
            Write_Screenshots(args.screenshots, number, paths, scenes, station_q)
        return 0
    import mujoco.viewer
    view = scenes['holding']
    try:
        viewer = mujoco.viewer.launch_passive(view.Model, view.Data)
    except RuntimeError as error:
        print('[ERROR] The MuJoCo viewer cannot start (%s). On macOS run: .venv/bin/mjpython Example/replay_rotation.py' % error)
        return 2
    with viewer:
        if not with_visual:
            with viewer.lock():
                viewer.opt.geomgroup[CONST_COLLISION_GROUP] = 1
        while viewer.is_running():
            Play(viewer, playlist, scenes, openings, '%d: %s' % (number, name), args.speed)
    return 0


if __name__ == '__main__':
    sys.exit(main())
