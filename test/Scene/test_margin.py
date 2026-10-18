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
# File Name: ../test/Scene/test_margin.py
## =========================================================================== ##

# System (Default)
import os
# Math (Mathematical functions)
import math
# Collections (Container datatypes)
import collections
# Dataclasses (Data Classes)
import dataclasses
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# MuJoCo (Physics simulator) [pip3 install mujoco]
import mujoco
# Custom Lib.:
#   ../src/Scene/Core — the cell scene whose contacts are compared with exact distances
from Scene.Core import Scene_Cls

"""
Description:
    Initialization of constants.

    Margin accuracy of the mechanism Check_Clearance relies on: MuJoCo reports a contact for a geom
    pair closer than the sum of their margins, with its distance (measured, mujoco 3.16). Tested on mesh hull vs mesh
    hull (what the real check uses), with the robot geom on a free joint: MuJoCo never collides two
    static bodies (SPEC 4.5). The true distance is known by construction.
"""
# Environment and robot geom margins, as configured (Clearance.Environment, Clearance.Self).
#   Unit [float] (m)
CONST_ENV_MARGIN = 0.020
CONST_ROBOT_MARGIN = 0.005
# Allowed error of the reported distance.
#   Unit [float] (m)
CONST_DISTANCE_TOLERANCE = 0.0005
# Seeded configurations near the station for the agreement of contacts with exact distances: each
# fixed station point perturbed (normal, sigma per joint) at both openings (50 mm holding, 85 mm open).
#   Unit [Tuple<string>], [int], [float] (rad), [Tuple<float>] (m), [List<int>]
CONST_AGREEMENT_POINTS = ('drop_rotate_1', 'Waypoint_4', 'Waypoint_5', 'Waypoint_18', 'pick_rotate_1', 'Waypoint_6')
CONST_AGREEMENT_PER_POINT, CONST_AGREEMENT_SIGMA = 4, 0.05
CONST_AGREEMENT_OPENINGS, CONST_AGREEMENT_SEED = (0.050, 0.085), [20261018]
# The robot - environment geom-type pairs the agreement must have seen within the margin: the gripper
# pads (boxes) and the meshes, each against the collision set's pieces and the added boxes.
#   Unit [Set<Tuple<string, string>>]
CONST_AGREEMENT_KINDS = {('box', 'piece'), ('box', 'added box'), ('mesh', 'piece'), ('mesh', 'added box')}
# The robot link mesh used (a vendored Menagerie forearm part).
#   Unit [string]
CONST_LINK_MESH = os.path.join('models', 'universal_robots_ur10e', 'assets', 'forearm_3.obj')


def _Box(hx, hy, hz):
    return np.array([[sx * hx, sy * hy, sz * hz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])


def _Contacts(root, mode, quat, gap):
    """
    Description:
        Build a two-body scene with the given true gap; return (contact distances, geomDistance).
        'face': a robot link hull above the top face of a large box hull; the true distance is the
        height of the link's lowest hull vertex above the face. 'edge': a box hull rotated 45 deg
        about x (bottom ridge along x) above a box hull rotated 45 deg about y (top ridge along y);
        the true distance is the vertical gap between the crossing ridges.
    """

    spec = mujoco.MjSpec()
    if mode == 'face':
        spec.add_mesh(name='robot', file=os.path.join(root, CONST_LINK_MESH))
    else:
        spec.add_mesh(name='robot', uservert=_Box(0.05, 0.05, 0.05).flatten().tolist())
    body = spec.worldbody.add_body(name='robot')
    body.add_freejoint()
    body.add_geom(name='robot', type=mujoco.mjtGeom.mjGEOM_MESH, meshname='robot', margin=CONST_ROBOT_MARGIN)
    model = spec.compile()
    data = mujoco.MjData(model)
    data.qpos[:3], data.qpos[3:7] = [0.0, 0.0, 0.5], quat
    mujoco.mj_kinematics(model, data)
    g = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'robot')
    mesh = model.geom_dataid[g]
    local = model.mesh_vert[model.mesh_vertadr[mesh]:model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
    world = local @ data.geom_xmat[g].reshape(3, 3).T + data.geom_xpos[g]
    lowest = world[np.argmin(world[:, 2])]
    if mode == 'face':
        spec.add_mesh(name='env', uservert=_Box(0.5, 0.5, 0.02).flatten().tolist())
        pos, env_quat = [lowest[0], lowest[1], lowest[2] - gap - 0.02], [1.0, 0.0, 0.0, 0.0]
    else:
        spec.add_mesh(name='env', uservert=_Box(0.05, 0.05, 0.05).flatten().tolist())
        pos = [0.0, 0.0, lowest[2] - gap - 0.05 * math.sqrt(2.0)]
        env_quat = [math.cos(math.pi / 8.0), 0.0, math.sin(math.pi / 8.0), 0.0]
    spec.worldbody.add_body(name='env', pos=pos, quat=env_quat).add_geom(
        name='env', type=mujoco.mjtGeom.mjGEOM_MESH, meshname='env', margin=CONST_ENV_MARGIN)
    model = spec.compile()
    data = mujoco.MjData(model)
    data.qpos[:3], data.qpos[3:7] = [0.0, 0.0, 0.5], quat
    mujoco.mj_kinematics(model, data)
    mujoco.mj_collision(model, data)
    distances = [float(data.contact[i].dist) for i in range(data.ncon)]
    g1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'robot')
    g2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'env')
    return distances, mujoco.mj_geomDistance(model, data, g1, g2, 0.1, None)


@pytest.mark.parametrize('mode, quat', [
    ('face', [1.0, 0.0, 0.0, 0.0]),
    ('face', list(np.array([0.9, 0.3, 0.2, 0.1]) / np.linalg.norm([0.9, 0.3, 0.2, 0.1]))),
    ('face', [math.cos(math.pi / 8.0), math.sin(math.pi / 8.0), 0.0, 0.0]),
    ('edge', [math.cos(math.pi / 8.0), math.sin(math.pi / 8.0), 0.0, 0.0]),
])
def test_a_10_mm_gap_violates_the_20_mm_margin_and_30_mm_does_not(root, mode, quat):
    """
    Description:
        At 10 mm a contact is reported with dist within 0.5 mm of 10 mm; at 30 mm none is. The
        geom distance agrees with the true gap within 0.5 mm in both cases.
    """

    near, near_distance = _Contacts(root, mode, quat, 0.010)
    far, far_distance = _Contacts(root, mode, quat, 0.030)
    assert near and all(abs(d - 0.010) < CONST_DISTANCE_TOLERANCE for d in near)
    assert far == []
    assert abs(near_distance - 0.010) < CONST_DISTANCE_TOLERANCE
    assert abs(far_distance - 0.030) < CONST_DISTANCE_TOLERANCE


def test_mujoco_generates_contacts_up_to_the_sum_of_both_margins():
    """
    Description:
        Measured with mujoco 3.16: a contact is generated while the gap is below the SUM of the two
        geom margins (contact.includemargin = m1 + m2), not the larger one. Check_Clearance relies on
        this being a superset of its configured margin (20 mm environment, 5 mm self) and filters by
        the configured margin itself. Two box hulls on free joints: margins 5 + 5 mm give a contact
        at 9 mm and none at 11 mm; 20 + 5 mm at 24 mm and none at 26 mm.
    """

    for margins, inside, outside in (((CONST_ROBOT_MARGIN, CONST_ROBOT_MARGIN), 0.009, 0.011),
                                     ((CONST_ENV_MARGIN, CONST_ROBOT_MARGIN), 0.024, 0.026)):
        for gap, expected in ((inside, True), (outside, False)):
            spec = mujoco.MjSpec()
            spec.add_mesh(name='box', uservert=_Box(0.05, 0.05, 0.05).flatten().tolist())
            for name, margin in zip(('lower', 'upper'), margins):
                body = spec.worldbody.add_body(name=name)
                body.add_freejoint()
                body.add_geom(name=name, type=mujoco.mjtGeom.mjGEOM_MESH, meshname='box', margin=margin)
            model = spec.compile()
            data = mujoco.MjData(model)
            data.qpos[:] = [0.0, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.6 + gap, 1.0, 0.0, 0.0, 0.0]
            mujoco.mj_kinematics(model, data)
            mujoco.mj_collision(model, data)
            distances = [float(data.contact[i].dist) for i in range(data.ncon)]
            assert bool(distances) == expected, (margins, gap, distances)
            assert all(abs(d - gap) < CONST_DISTANCE_TOLERANCE for d in distances)
            assert all(abs(data.contact[i].includemargin - sum(margins)) < 1e-12 for i in range(data.ncon))


def test_every_pair_within_the_margin_gets_a_contact_with_its_exact_distance(cfg, points):
    """
    Description:
        On seeded configurations near the station, at both openings, every robot - environment geom
        pair whose exact distance (mj_geomDistance) is below Clearance.Environment gets a contact from
        mj_collision, at that distance within CONST_DISTANCE_TOLERANCE; this covers every geom-type
        pair the scene has there (the gripper pads, boxes, and the meshes, against the collision
        set's pieces and the added boxes). MuJoCo's box-box collision missed such pairs (gripper pad
        vs a box primitive, KNOWN_ISSUES), so the added boxes are meshes.
    """

    rng = np.random.default_rng(CONST_AGREEMENT_SEED)
    seen, problems = collections.Counter(), []
    for opening in CONST_AGREEMENT_OPENINGS:
        scene = Scene_Cls(dataclasses.replace(cfg, Opening=opening))
        m, d = scene.Model, scene.Data
        env = [g for g in range(m.ngeom) if m.geom_contype[g] and m.body(m.geom_bodyid[g]).name.startswith('env')]
        robot = [g for g in range(m.ngeom) if m.geom_contype[g] and m.geom_bodyid[g] != 0
                 and not m.body(m.geom_bodyid[g]).name.startswith('env') and m.body(m.geom_bodyid[g]).name != 'base']
        for name in CONST_AGREEMENT_POINTS:
            for _ in range(CONST_AGREEMENT_PER_POINT):
                q = np.array(points[name].Q) + rng.normal(0.0, CONST_AGREEMENT_SIGMA, 6)
                scene.Set_Joints(q)
                mujoco.mj_collision(m, d)
                contacts = {}
                for c in d.contact[:d.ncon]:
                    pair = (min(c.geom1, c.geom2), max(c.geom1, c.geom2))
                    contacts[pair] = min(contacts.get(pair, np.inf), c.dist)
                for g in robot:
                    for e in env:
                        # Bounding spheres farther apart than the margin cannot come within it.
                        if (np.linalg.norm(d.geom_xpos[g] - d.geom_xpos[e]) - m.geom_rbound[g] - m.geom_rbound[e]
                                > cfg.Clearance_Environment):
                            continue
                        exact = mujoco.mj_geomDistance(m, d, g, e, 2.0 * cfg.Clearance_Environment, None)
                        if exact >= cfg.Clearance_Environment:
                            continue
                        robot_kind = 'box' if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX else 'mesh'
                        seen[robot_kind, 'added box' if m.geom(e).name.startswith('env_box_') else 'piece'] += 1
                        reported = contacts.get((min(g, e), max(g, e)))
                        if reported is None or abs(reported - exact) > CONST_DISTANCE_TOLERANCE:
                            problems.append('%s near %s: %s - %s exact %.2f mm, contact %s'
                                            % (name, opening, m.geom(g).name or m.body(m.geom_bodyid[g]).name, m.geom(e).name,
                                               exact * 1e3, 'none' if reported is None else '%.2f mm' % (reported * 1e3)))
    assert CONST_AGREEMENT_KINDS <= set(seen), seen
    assert problems == [], '\n'.join(problems[:20])
