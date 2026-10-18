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
# File Name: ../test/Collision/test_core.py
## =========================================================================== ##

# System (Default)
import os
# Shutil (High-level file operations)
import shutil
# JSON (JSON encoder and decoder)
import json
# Hashlib (Secure hashes)
import hashlib
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# MuJoCo (Physics simulator) [pip3 install mujoco]
import mujoco
# SciPy (Scientific computing) [pip3 install scipy]
from scipy.spatial import ConvexHull
# Custom Lib.:
#   ../src/Collision/Core — the collision set under test
from Collision.Core import (Read_Collision_Set, Read_Binary_STL, Thicken_Degenerate, Connected_Components,
                            Collision_Error_Cls, CONST_THICKEN, Component_Objects, Component_Translations)

"""
Description:
    Initialization of constants.

    The tests read the committed set; they never run CoACD (SPEC 7.6).
"""
# The STL's measured size (SPEC 4.1): vertices after merging, faces.
#   Unit [int]
CONST_STL_VERTICES = 891504
CONST_STL_FACES = 1757948
# Connected components by manifold edge, as measured independently with trimesh (SPEC 4.1).
#   Unit [int]
CONST_STL_COMPONENTS = 2756
# Components of the objects (SPEC 4.1): the machine body, the left window post at the station
# and the lower cabinet panel (object (a)); the stand profile, the pallet and side tables and the
# side cabinet (objects (b) and (c)).
#   Unit [int], [Tuple<int>]
CONST_MACHINE_BODY, CONST_STATION_POST, CONST_CABINET_PANEL = 1154, 2710, 180
CONST_NOT_MACHINE = (2681, 1060, 1061, 2643)


@pytest.fixture(scope='module')
def collision(root, cfg):
    """
    Description:
        The committed collision set.
    """

    return Read_Collision_Set(cfg.Collision_Set, root)


@pytest.fixture(scope='module')
def stl(cfg):
    """
    Description:
        The environment STL, merged.
    """

    return Read_Binary_STL(cfg.Visual_Mesh)


def test_the_stl_is_read_with_its_measured_size(stl):
    """
    Description:
        The numpy reader gives the counts measured independently with trimesh (SPEC 4.1).
    """

    vertices, faces = stl
    assert len(vertices) == CONST_STL_VERTICES and len(faces) == CONST_STL_FACES


def test_components_are_split_at_non_manifold_edges(stl):
    """
    Description:
        Parts that only touch (the stand profiles, along non-manifold edges) are separate
        components: the count is trimesh's 2756 (SPEC 4.1), and two triangles sharing one edge with
        a third are three components while a manifold pair is one.
    """

    vertices, faces = stl
    count, _ = Connected_Components(faces)
    assert count == CONST_STL_COMPONENTS
    fan = np.array([[0, 1, 2], [1, 0, 3], [0, 1, 4]])
    assert Connected_Components(fan)[0] == 3
    assert Connected_Components(np.array([[0, 1, 2], [1, 0, 3]]))[0] == 1


def test_the_stl_has_three_objects_and_an_assembly_moves_exactly_one(stl, collision, cfg):
    """
    Description:
        Components grouped by touching bounding boxes form the three objects of SPEC 4.1: the machine
        (a) with 1749 components, the robot side (b) with 951, the side cabinet (c) with 56. The
        machine holds the body, the station post and the lower cabinet panel, and none of the stand,
        the side tables or the side cabinet. An assembly named by the body moves exactly that
        object, and a component override adds its own translation on top. The manifest records the
        members of the configured assemblies.
    """

    vertices, faces = stl
    _, labels = Connected_Components(faces)
    objects = Component_Objects(vertices, faces, labels)
    assert sorted(np.bincount(objects).tolist()) == [56, 951, 1749]
    machine = set(np.flatnonzero(objects == objects[CONST_MACHINE_BODY]).tolist())
    assert len(machine) == 1749 and {CONST_STATION_POST, CONST_CABINET_PANEL} <= machine
    assert not machine & set(CONST_NOT_MACHINE)
    shift, members = Component_Translations(vertices, faces, labels, [('test', CONST_MACHINE_BODY, [-0.1, 0.0, 0.0], 'test')],
                                            [(CONST_STATION_POST, [-0.01, 0.0, 0.0], 'test')])
    assert set(members['test']) == machine and set(shift) == machine
    assert np.allclose(shift[CONST_MACHINE_BODY], [-0.1, 0.0, 0.0]) and np.allclose(shift[CONST_STATION_POST], [-0.11, 0.0, 0.0])
    _, configured = Component_Translations(vertices, faces, labels, cfg.Assembly_Overrides, cfg.Component_Overrides)
    assert collision.Manifest.get('assembly_components', {}) == {name: sorted(c) for name, c in configured.items()}


def test_the_manifest_names_the_stl_it_was_built_from(root, cfg, collision):
    """
    Description:
        The recorded source hash is the sha256 of CAD/SF1_Environment.stl.
    """

    with open(cfg.Visual_Mesh, 'rb') as handle:
        digest = hashlib.sha256(handle.read()).hexdigest()
    assert collision.Manifest['source_stl'] == os.path.join('CAD', 'SF1_Environment.stl')
    assert collision.Manifest['source_sha256'] == digest


def test_a_set_built_from_another_stl_is_refused(root, cfg, tmp_path):
    """
    Description:
        A manifest whose source hash differs from the file on disk is refused with the rebuild command.
    """

    npz = tmp_path / 'set.npz'
    shutil.copy(cfg.Collision_Set, npz)
    with open(os.path.splitext(cfg.Collision_Set)[0] + '.json', encoding='utf-8') as handle:
        manifest = json.load(handle)
    manifest['source_sha256'] = '0' * 64
    (tmp_path / 'set.json').write_text(json.dumps(manifest), encoding='utf-8')
    with pytest.raises(Collision_Error_Cls, match='build_collision.py'):
        Read_Collision_Set(str(npz), root)
    with pytest.raises(Collision_Error_Cls, match='missing'):
        Read_Collision_Set(str(tmp_path / 'none.npz'), root)


def test_the_crop_sphere_contains_the_robot_everywhere(robot_scene, collision, cfg):
    """
    Description:
        The recorded crop radius is at least the bound recomputed from the current scene, and over
        1000 seeded random joint vectors no robot or gripper collision vertex leaves the reach bound.
    """

    assert collision.Manifest['crop_radius'] >= robot_scene.Crop_Radius() - 1e-12
    centre, bound = robot_scene.Shoulder_Point(), robot_scene.Reach_Bound()
    m, d = robot_scene.Model, robot_scene.Data
    rng = np.random.default_rng(0)
    worst = 0.0
    for q in rng.uniform(cfg.Joint_Limits_Min, cfg.Joint_Limits_Max, size=(1000, 6)):
        robot_scene.Set_Joints(q)
        for g in range(m.ngeom):
            if not m.geom_contype[g] or m.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH:
                continue
            mesh = m.geom_dataid[g]
            local = m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh] + m.mesh_vertnum[mesh]]
            world = local @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
            worst = max(worst, float(np.max(np.linalg.norm(world - centre, axis=1))))
    assert worst <= bound


def test_every_hull_piece_contains_its_component(stl, collision, cfg):
    """
    Description:
        Hull and thickened pieces are conservative by convexity: every vertex of the original
        component, translated by its configured assembly and component overrides, lies inside the
        piece's hull (checked here, independently of the build).
    """

    vertices, faces = stl
    _, labels = Connected_Components(faces)
    shift, _ = Component_Translations(vertices, faces, labels, cfg.Assembly_Overrides, cfg.Component_Overrides)
    checked = 0
    for piece, component, method in zip(collision.Pieces, collision.Components, collision.Methods):
        if method not in ('hull', 'thickened', 'fallback'):
            continue
        original = vertices[np.unique(faces[labels == component])] + shift.get(component, 0.0)
        hull = ConvexHull(piece)
        signed = original @ hull.equations[:, :3].T + hull.equations[:, 3]
        assert np.max(signed) <= 1e-7, (component, float(np.max(signed)))
        checked += 1
    assert checked > 0


def test_conservativeness_and_excess_are_recorded(collision):
    """
    Description:
        The manifest records 0 violations above eps (CoACD components were checked on vertices and
        1 sample per cm^2), the excess metrics, and the flagged open-shell pieces.
    """

    manifest = collision.Manifest
    assert manifest['conservativeness']['violations_above_eps'] == 0
    assert manifest['conservativeness']['max_violation_m'] <= manifest['eps']
    assert manifest['excess']['max_m'] == pytest.approx(max(collision.Excess))
    flagged = set(manifest['flagged_open_shells']['pieces'])
    assert flagged == {i for i, e in enumerate(collision.Excess) if e > manifest['hull_excess_max']}


def test_a_planar_component_with_inconsistent_winding_is_thickened_along_its_plane_normal():
    """
    Description:
        Two triangles of a square in a tilted plane, wound oppositely (their face normals cancel):
        the SVD normal still gives a hull of thickness 2 * CONST_THICKEN containing the square.
    """

    normal = np.array([1.0, 2.0, 2.0]) / 3.0
    u = np.cross(normal, [1.0, 0.0, 0.0])
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    square = np.array([0.5 * a * u + 0.5 * b * v + [0.1, 0.2, 0.3] for a, b in ((0, 0), (1, 0), (1, 1), (0, 1))])
    points, kind = Thicken_Degenerate(square)
    assert kind == 'planar'
    hull = ConvexHull(points)
    heights = (points - square[0]) @ normal
    assert heights.max() - heights.min() == pytest.approx(2.0 * CONST_THICKEN, rel=1e-6)
    assert np.max(square @ hull.equations[:, :3].T + hull.equations[:, 3]) <= 1e-12


def test_a_collinear_component_is_thickened_in_two_directions():
    """
    Description:
        Points on a line get a box cross-section of 2 * CONST_THICKEN in both orthogonal directions.
    """

    line = np.array([[0.0, 0.0, 0.0], [0.1, 0.1, 0.0], [0.3, 0.3, 0.0]])
    points, kind = Thicken_Degenerate(line)
    assert kind == 'collinear'
    hull = ConvexHull(points)
    assert hull.volume > 0.0
    direction = np.array([1.0, 1.0, 0.0]) / np.sqrt(2.0)
    for axis in (np.array([0.0, 0.0, 1.0]), np.array([1.0, -1.0, 0.0]) / np.sqrt(2.0)):
        extent = points @ axis
        assert extent.max() - extent.min() == pytest.approx(2.0 * CONST_THICKEN, rel=1e-6)
    assert abs(np.dot(direction, np.array([0.0, 0.0, 1.0]))) < 1e-12
    assert np.max(line @ hull.equations[:, :3].T + hull.equations[:, 3]) <= 1e-12


@pytest.mark.parametrize('name', ['home', 'drop_rotate_1'])
def test_no_robot_body_at_a_or_b_is_inside_a_flagged_piece(scene, collision, points, name):
    """
    Description:
        At A and B no robot or gripper collision geom penetrates a flagged open-shell piece.
    """

    m, d = scene.Model, scene.Data
    flagged = ['env_piece_%d' % i for i in collision.Manifest['flagged_open_shells']['pieces']]
    robot = [g for g in range(m.ngeom) if m.geom_contype[g] and not m.body(m.geom_bodyid[g]).name.startswith('env')
             and m.geom_bodyid[g] != 0]
    scene.Set_Joints(points[name].Q)
    for piece in flagged:
        e = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, piece)
        for g in robot:
            assert mujoco.mj_geomDistance(m, d, g, e, 0.05, None) > 0.0, (name, piece, m.geom(g).name)
