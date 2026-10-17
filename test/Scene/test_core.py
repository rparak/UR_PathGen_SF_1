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
# File Name: ../test/Scene/test_core.py
## =========================================================================== ##

# System (Default)
import os
# Hashlib (Secure hashes)
import hashlib
# JSON (JSON encoder and decoder)
import json
# Math (Mathematical functions)
import math
# Regular expressions (Default)
import re
# Dataclasses (Data Classes)
import dataclasses
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# MuJoCo (Physics simulator) [pip3 install mujoco]
import mujoco
# SciPy (Scientific computing) [pip3 install scipy]
from scipy.optimize import minimize
# Custom Lib.:
#   ../src/Scene/Core — the scene under test
from Scene.Core import Scene_Cls, Scene_Error_Cls, Clearance_Error_Cls, Check_Learned_Point
#   ../src/Kinematics/Core — the kinematic refusal
from Kinematics.Core import Kinematics_Error_Cls
#   ../src/Kinematics/Core — the DH truth
from Kinematics.Core import DH_FK, TCP_Pose, Geodesic_Angle, Matrix_To_Pose

"""
Description:
    Initialization of constants.

    Expected values come from the measurements in SPEC 4 (STL, Robotiq manual, prototype), the
    configuration, or the DH truth; never from the scene under test.
"""
# Side table (SPEC 4.1): x/y extent and the highest part on it, world frame.
#   Unit [float] (m)
CONST_SIDE_TABLE_X = (0.34, 0.93)
CONST_SIDE_TABLE_Y = (-0.455, 0.455)
CONST_SIDE_TABLE_TOP = 0.781
# Object (a), the machine behind the robot (SPEC 4.1): bounding box, world frame.
#   Unit [Vector<float> 1x3] (m)
CONST_OBJECT_A_MIN = (-1.609, -0.422, -0.002)
CONST_OBJECT_A_MAX = (-0.466, 0.458, 2.023)
# The manual's dimension chain (SPEC 4.4): pad end from the flange face, fully open and closed,
# = 14.0 mm coupling datum + 149.3 / 162.8 mm (Robotiq manual rev. 2020/02/11, p. 116-117), and the
# tolerance (the manual gives 0.1 mm; the model reproduced it to 0.31 mm).
#   Unit [float] (mm)
CONST_PAD_END_OPEN = 163.3
CONST_PAD_END_CLOSED = 176.8
CONST_PAD_END_TOLERANCE = 0.5
# Regression values at the 50 mm opening (SPEC 4.4 (b)): pad-face centre and pad end.
#   Unit [float] (mm)
CONST_PAD_FACE_50 = 154.39
CONST_PAD_END_50 = 173.23
CONST_REGRESSION_MM = 0.05
# The only pair-specific margin, the body pair and the joints that move one relative to the other.
#   Unit [Tuple]
CONST_OVERRIDE_PAIR = ('base', 'upper_arm_link')
# True minimum of base - upper_arm_link (refined), measured 2026-10-17 (grid minimum 2.3263 mm); +-0.01 mm.
#   Unit [float] (mm)
CONST_BASE_UPPER_ARM_MIN = 2.3237
# Adjacent exclusions expected from the configuration (typed here, not read from it).
#   Unit [List<Tuple<string>>]
CONST_ADJACENT = [('base', 'shoulder_link'), ('shoulder_link', 'upper_arm_link'), ('upper_arm_link', 'forearm_link'),
                  ('forearm_link', 'wrist_1_link'), ('wrist_1_link', 'wrist_2_link'), ('wrist_2_link', 'wrist_3_link'),
                  ('wrist_3_link', 'gripper_base_mount')]


def _Body(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)


def _Pad_Points(scene):
    """
    Description:
        Pad-end height and pad-face-centre height along the flange z axis, from the flange face (mm).
    """

    m, d = scene.Model, scene.Data
    F = scene.Flange_Pose()
    ends, faces = [], []
    for side in ('left', 'right'):
        b = _Body(m, 'gripper_%s_pad' % side)
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == b and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH and m.geom_contype[g] == 0:
                mesh = m.geom_dataid[g]
                local = m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh] + m.mesh_vertnum[mesh]]
                world = local @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
                ends.append(np.max((world - F[:3, 3]) @ F[:3, 2]))
        centre = d.xmat[b].reshape(3, 3) @ np.array([0.0, -0.0066, 0.01875]) + d.xpos[b]
        faces.append((centre - F[:3, 3]) @ F[:3, 2])
    return 1e3 * float(np.max(ends)), 1e3 * float(np.mean(faces))


def test_the_base_sits_at_the_configured_pose(robot_scene, cfg):
    """
    Description:
        The base body is exactly at Base_Position, and the flange equals Base_Frame * DH_FK, which
        also pins the Menagerie 180 deg and the configured yaw.
    """

    base = _Body(robot_scene.Model, 'base')
    assert list(robot_scene.Data.xpos[base]) == cfg.Base_Position
    q = [0.3, -1.0, 1.2, -0.5, 0.7, 0.2]
    robot_scene.Set_Joints(q)
    expected = robot_scene.Base_Frame() @ DH_FK(q)
    assert np.allclose(robot_scene.Flange_Pose(), expected, atol=1e-12)
    assert expected[2, 3] > 0.0 and cfg.Base_Position[2] == 0.7152


def test_mujoco_flange_equals_dh_flange_for_random_joints(robot_scene, cfg, points):
    """
    Description:
        After the MjSpec offset override, MuJoCo FK equals DH FK within 1e-9 m / 1e-9 rad (atan2
        geodesic) for 1000 seeded random joint vectors within the limits, and at A and B.
    """

    rng = np.random.default_rng(0)
    qs = list(rng.uniform(cfg.Joint_Limits_Min, cfg.Joint_Limits_Max, size=(1000, 6)))
    qs += [points['home_safe'].Q, points['drop_rot_01'].Q]
    worst_p = worst_r = 0.0
    for q in qs:
        robot_scene.Set_Joints(q)
        F, T = robot_scene.Flange_Pose(), robot_scene.Base_Frame() @ DH_FK(q)
        worst_p = max(worst_p, float(np.linalg.norm(F[:3, 3] - T[:3, 3])))
        worst_r = max(worst_r, Geodesic_Angle(F[:3, :3], T[:3, :3]))
    assert worst_p < 1e-9 and worst_r < 1e-9


def test_the_gripper_holds_the_configured_opening(robot_scene, cfg):
    """
    Description:
        The solved opening is Tool.Opening +-0.5 mm with an equality residual < 0.1 mm.
    """

    gap, residual = robot_scene.Gripper_Gap
    assert abs(gap - cfg.Opening) < 0.0005
    assert residual < 0.0001


def test_the_fingers_match_the_manual_dimension_chain(cfg):
    """
    Description:
        With the coupling (datum at 14.0 mm, measured on the robot) the pad end lies 163.3 mm (open)
        and 176.8 mm (closed) from the flange face, within 0.5 mm.
    """

    for opening, expected in ((0.0850, CONST_PAD_END_OPEN), (0.0010, CONST_PAD_END_CLOSED)):
        scene = Scene_Cls(dataclasses.replace(cfg, Opening=opening), with_environment=False)
        scene.Set_Joints(np.zeros(6))
        end, _ = _Pad_Points(scene)
        assert abs(end - expected) < CONST_PAD_END_TOLERANCE, (opening, end)


def test_the_fingers_at_50_mm_are_pinned(robot_scene):
    """
    Description:
        Regression: pad-face centre 154.39 mm, pad end 173.23 mm from the flange face (SPEC 4.4).
    """

    robot_scene.Set_Joints(np.zeros(6))
    end, face = _Pad_Points(robot_scene)
    assert end == pytest.approx(CONST_PAD_END_50, abs=CONST_REGRESSION_MM)
    assert face == pytest.approx(CONST_PAD_FACE_50, abs=CONST_REGRESSION_MM)


def test_the_fingers_open_along_flange_x(robot_scene):
    """
    Description:
        At Mount_Yaw = 0 the pads separate along the flange x axis (the documented assumption).
    """

    m, d = robot_scene.Model, robot_scene.Data
    robot_scene.Set_Joints(np.zeros(6))
    F = robot_scene.Flange_Pose()
    axis = F[:3, :3].T @ (d.xpos[_Body(m, 'gripper_left_pad')] - d.xpos[_Body(m, 'gripper_right_pad')])
    axis /= np.linalg.norm(axis)
    assert abs(axis[0]) > 0.999


def test_the_coupling_replaces_the_menagerie_disc(robot_scene, cfg):
    """
    Description:
        The coupling cylinder exists as visual and collision geom with the configured size; the
        Menagerie base_mount mesh geoms are gone; the pad-span hull is a collision geom of the
        gripper base.
    """

    m = robot_scene.Model
    mount = _Body(m, 'gripper_base_mount')
    geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] == mount]
    assert sorted(m.geom(g).name for g in geoms) == ['gripper_coupling', 'gripper_coupling_visual']
    for g in geoms:
        assert m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER
        assert list(m.geom_size[g][:2]) == pytest.approx([cfg.Coupling_Diameter / 2.0, cfg.Coupling_Thickness / 2.0])
    span = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'gripper_pad_span')
    assert m.body(m.geom_bodyid[span]).name == 'gripper_base' and m.geom_contype[span] == 1


def test_the_vendored_models_are_untouched(root):
    """
    Description:
        Every vendored file matches models/MANIFEST.json: all patches live in code.
    """

    with open(os.path.join(root, 'models', 'MANIFEST.json'), encoding='utf-8') as handle:
        manifest = json.load(handle)
    for path, digest in manifest['sha256'].items():
        with open(os.path.join(root, path), 'rb') as handle:
            assert hashlib.sha256(handle.read()).hexdigest() == digest, path


def test_only_the_configured_exclusions_apply(scene, cfg):
    """
    Description:
        The parent/child filter is disabled and the compiled exclusions are exactly the adjacent
        pairs, every pair of the gripper rigid group, and base - env_stand.
    """

    m = scene.Model
    assert m.opt.disableflags & mujoco.mjtDisableBit.mjDSBL_FILTERPARENT
    compiled = set()
    for i in range(m.nexclude):
        signature = int(m.exclude_signature[i])
        compiled.add(frozenset((m.body(signature >> 16).name, m.body(signature & 0xFFFF).name)))
    group = cfg.Rigid_Groups[0]
    expected = {frozenset(p) for p in CONST_ADJACENT} | {frozenset(('base', 'env_stand'))}
    expected |= {frozenset((a, b)) for i, a in enumerate(group) for b in group[i + 1:]}
    assert len(group) == 14 and compiled == expected


def test_joint_ranges_are_the_configured_limits(robot_scene, cfg):
    """
    Description:
        The compiled joint ranges are the configuration's (elbow +-pi, others +-2*pi), not Menagerie's.
    """

    m = robot_scene.Model
    for i, name in enumerate(('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
                              'wrist_2_joint', 'wrist_3_joint')):
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        assert list(m.jnt_range[j]) == [cfg.Joint_Limits_Min[i], cfg.Joint_Limits_Max[i]]
    assert cfg.Joint_Limits_Max[2] == pytest.approx(math.pi)


def test_drop_point_lies_above_the_side_table_and_outside_object_a(robot_scene, cfg, points):
    """
    Description:
        The world-yaw check that FK vs _p cannot do: drop_rot_01's TCP in the world frame is over
        the side table, above its highest part, and outside the machine behind the robot.
    """

    tcp = (robot_scene.Base_Frame() @ TCP_Pose(points['drop_rot_01'].Q, cfg.TCP_Offset))[:3, 3]
    assert CONST_SIDE_TABLE_X[0] < tcp[0] < CONST_SIDE_TABLE_X[1]
    assert CONST_SIDE_TABLE_Y[0] < tcp[1] < CONST_SIDE_TABLE_Y[1]
    assert tcp[2] > CONST_SIDE_TABLE_TOP
    assert not all(CONST_OBJECT_A_MIN[k] <= tcp[k] <= CONST_OBJECT_A_MAX[k] for k in range(3))


def test_a_and_b_keep_every_clearance_margin(scene, cfg, points):
    """
    Description:
        A and B against the full environment and self-collision. Any violation fails, listing
        every pair with its distance and margin.
    """

    for name in ('home_safe', 'drop_rot_01'):
        violations = scene.Check_Clearance(points[name].Q)
        assert not violations, '%s: %s' % (name, '; '.join('%s - %s %.2f mm (margin %.1f mm)' % (
            v.Body_1, v.Body_2, v.Distance * 1e3, v.Margin * 1e3) for v in violations))
        Check_Learned_Point(scene, cfg, points[name])


def test_a_pose_pushed_into_the_side_table_is_a_collision(scene, cfg, points):
    """
    Description:
        Lowering drop_rot_01 by the shoulder until the TCP is below the side table top inside its
        extent is reported as a collision with the environment, and Check_Learned_Point refuses it.
    """

    q = np.array(points['drop_rot_01'].Q)
    for delta in np.linspace(0.0, 1.0, 201):
        trial = q + np.array([0.0, delta, 0.0, 0.0, 0.0, 0.0])
        tcp = (scene.Base_Frame() @ TCP_Pose(trial, cfg.TCP_Offset))[:3, 3]
        if tcp[2] < 0.70 and CONST_SIDE_TABLE_X[0] < tcp[0] < CONST_SIDE_TABLE_X[1]:
            break
    else:
        pytest.fail('No shoulder offset lowers the TCP into the side table; the test setup is wrong.')
    violations = scene.Check_Clearance(trial)
    assert any(v.Body_2.startswith('env') or v.Body_1.startswith('env') for v in violations)
    assert min(v.Distance for v in violations) < 0.0
    # A consistent point (its _p is the FK of its _q), so it is the clearance check that refuses it.
    point = dataclasses.replace(points['drop_rot_01'], Q=list(trial), P=Matrix_To_Pose(TCP_Pose(trial, cfg.TCP_Offset)))
    with pytest.raises(Clearance_Error_Cls, match='drop_rot_01 violates'):
        Check_Learned_Point(scene, cfg, point)


def test_the_static_base_clearance_is_checked_at_build(scene, cfg):
    """
    Description:
        MuJoCo never collides two static bodies, so base - env is checked once at build. Here it is
        clear; a base placed into the side table fails the build naming the pieces.
    """

    assert scene.Static_Base_Clearance() == []
    with pytest.raises(Scene_Error_Cls, match='robot base is within'):
        Scene_Cls(dataclasses.replace(cfg, Base_Position=[0.6, 0.0, 0.72]), with_environment=True)


def test_the_base_upper_arm_override_has_a_true_minimum_above_zero(robot_scene, cfg):
    """
    Description:
        base - upper_arm_link has a 0 mm self margin (owner, Q9 (a)) only because it never touches.
        Only shoulder_pan (periodic) and shoulder_lift move the upper arm relative to the base, so
        the true minimum is found by a coarse grid over pan in [0, 2*pi) x lift within its limits,
        then local minimisation from the 20 lowest cells. It must be > 0, and it is pinned.
    """

    assert cfg.Self_Pair_Overrides == [(CONST_OVERRIDE_PAIR[0], CONST_OVERRIDE_PAIR[1], 0.0)]

    def Distance(x):
        q = [x[0], float(np.clip(x[1], cfg.Joint_Limits_Min[1], cfg.Joint_Limits_Max[1])), 0.0, 0.0, 0.0, 0.0]
        return robot_scene.Minimum_Distance(CONST_OVERRIDE_PAIR[0], CONST_OVERRIDE_PAIR[1], q, distmax=0.5)

    pans = np.linspace(0.0, 2.0 * math.pi, 72, endpoint=False)
    lifts = np.linspace(cfg.Joint_Limits_Min[1], cfg.Joint_Limits_Max[1], 145)
    grid = sorted((Distance([p, l]), p, l) for p in pans for l in lifts)
    refined = [minimize(Distance, [p, l], method='Nelder-Mead',
                        options={'xatol': 1e-6, 'fatol': 1e-8, 'maxiter': 400}).fun for _, p, l in grid[:20]]
    true_minimum = min(min(refined), grid[0][0])
    assert true_minimum > 0.0
    print('base - upper_arm_link true minimum: %.4f mm (grid minimum %.4f mm)' % (true_minimum * 1e3, grid[0][0] * 1e3))
    if CONST_BASE_UPPER_ARM_MIN is not None:
        assert true_minimum * 1e3 == pytest.approx(CONST_BASE_UPPER_ARM_MIN, abs=0.01)


def test_the_visual_mesh_is_not_hulled(cfg):
    """
    Description:
        The 1.76 M-triangle visual mesh is passed as uservert/userface (MuJoCo's STL loader refuses
        > 200 000 faces), never collides and has no stored hull.
    """

    scene = Scene_Cls(cfg, with_environment=False, with_visual=True)
    m = scene.Model
    mesh = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_MESH, 'env_visual')
    geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'env_visual')
    assert m.mesh_facenum[mesh] == 1757948
    assert m.mesh_graphadr[mesh] == -1
    assert m.geom_contype[geom] == 0 and m.geom_conaffinity[geom] == 0


def test_check_learned_point_refuses_a_point_whose_joints_disagree_with_its_pose(scene, cfg, points):
    """
    Description:
        The kinematic branch of Check_Learned_Point: drop_rot_01 with its base joint turned by
        0.05 rad but its _p unchanged is refused with Kinematics_Error_Cls before any clearance check.
        Expected numbers, independent of the FK code: the TCP moves along a chord 2 r sin(0.025)
        with r = |(x, y)| of _p = 0.7660 m, i.e. 38.30 mm, give or take the measured kinematic
        error (<= 4.1 mm); the orientation by 0.05 rad = 2.865 deg, give or take 0.4 deg.
    """

    p = points['drop_rot_01'].P
    chord = 2.0 * math.hypot(p[0], p[1]) * math.sin(0.025) * 1e3
    q = list(points['drop_rot_01'].Q)
    q[0] += 0.05
    with pytest.raises(Kinematics_Error_Cls) as error:
        Check_Learned_Point(scene, cfg, dataclasses.replace(points['drop_rot_01'], Q=q))
    match = re.search(r'Point drop_rot_01: .* is ([0-9.]+) mm / ([0-9.]+) deg from its _p; the sanity bound is '
                      r'10\.00 mm / ([0-9.]+) deg', str(error.value))
    assert match, str(error.value)
    assert abs(float(match.group(1)) - chord) <= 4.1
    assert abs(float(match.group(2)) - math.degrees(0.05)) <= 0.4


def test_a_self_pair_inside_5_mm_is_reported(robot_scene, cfg):
    """
    Description:
        A pose (found by seeded random search) where forearm_link and wrist_3_link are 1-5 mm apart,
        neither excluded nor overridden: Check_Clearance reports exactly that pair with the 5 mm
        self margin and the distance mj_geomDistance gives.
    """

    q = [2.3617, -0.3394, -1.1313, 1.0575, 4.4924, -4.9803]
    distance = robot_scene.Minimum_Distance('forearm_link', 'wrist_3_link', q)
    assert 0.001 < distance < cfg.Clearance_Self
    violations = robot_scene.Check_Clearance(q)
    assert [(v.Body_1, v.Body_2) for v in violations] == [('forearm_link', 'wrist_3_link')]
    assert violations[0].Margin == cfg.Clearance_Self
    assert abs(violations[0].Distance - distance) < 1e-4


def test_contacts_beyond_the_configured_margin_are_not_violations(scene, cfg):
    """
    Description:
        MuJoCo generates contacts up to the sum of both geom margins (25 / 10 mm); Check_Clearance
        must filter by the configured margin (20 / 5 mm). Two poses found by seeded random search:
        a robot - environment contact at 21-24 mm and a robot - robot contact at 6-9 mm are present
        in MuJoCo's contact list, and neither is a violation.
    """

    m, d = scene.Model, scene.Data
    for q, pair, band in (([0.9263, -2.1894, -2.0539, -1.2618, 5.6555, -5.5791], ('env', 'wrist_2_link'), (0.021, 0.024)),
                          ([-2.4298, -1.9392, 1.2995, -4.1812, -0.7126, 1.3425], ('forearm_link', 'wrist_2_link'),
                           (0.006, 0.009))):
        assert scene.Check_Clearance(q) == []
        contacts = [float(d.contact[i].dist) for i in range(d.ncon)
                    if tuple(sorted((m.body(m.geom_bodyid[d.contact[i].geom1]).name,
                                     m.body(m.geom_bodyid[d.contact[i].geom2]).name))) == pair]
        assert contacts and band[0] < min(contacts) < band[1], (pair, contacts)
