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
# File Name: ../test/Kinematics/test_core.py
## =========================================================================== ##

# Math (Mathematical functions)
import math
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.:
#   ../src/Kinematics/Core — the kinematics under test
from Kinematics.Core import (DH_FK, TCP_Residual, Derive_TCP_Offset, Geodesic_Angle, Rotvec_To_Matrix, Matrix_To_Rotvec,
                             Check_FK_Consistency, Kinematics_Error_Cls, Orientation_Sanity_Bound, TCP_Pose,
                             Solve_IK_Near, Linear_Path, Solve_IK_Analytic, IK_Branch, Nearest_Representative, Wrap_Angle)

"""
Description:
    Initialization of constants.

    Regression pins (SPEC 9.4): the residuals of DH_FK(q) * TCP_config vs _p measured on
    2026-10-17 (SPEC 4.3). They detect any change in FK, DH constants, rotation conversion, the
    parser or the TCP; they say nothing about whether the residual is acceptable (the sanity bound
    does that).
"""
# (point, position residual [mm], orientation residual [deg]).
#   Unit [Tuple]
CONST_RESIDUAL_PINS = (('home', 1.7177033, 0.2608401), ('drop_rotate_1', 4.0229419, 0.3763663))
# Diagnostic derived TCP per point: position in the flange frame [mm], rotation angle [deg].
#   Unit [Tuple]
CONST_DERIVED_PINS = (('home', (1.112, -0.036, 172.692), 0.2608401),
                      ('drop_rotate_1', (-3.299, -0.229, 171.709), 0.3763663))
# Pin bands: +-0.01 mm and +-0.001 deg (residuals); +-0.001 mm for the derived TCP position.
#   Unit [float]
CONST_PIN_MM = 0.01
CONST_PIN_DEG = 0.001
CONST_PIN_DERIVED_MM = 0.001


def test_dh_fk_at_zero_is_the_ur10e_stretched_pose():
    """
    Description:
        At q = 0 the UR flange lies at x = a2 + a3, y = -(d4 + d6), z = d1 - d5 (official UR10e
        values typed here), pointing along -y.
    """

    T = DH_FK([0.0] * 6)
    assert T[:3, 3] == pytest.approx([-0.6127 - 0.57155, -(0.17415 + 0.11655), 0.1807 - 0.11985], abs=1e-12)
    assert T[:3, 2] == pytest.approx([0.0, -1.0, 0.0], abs=1e-12)


@pytest.mark.parametrize('name, mm, deg', CONST_RESIDUAL_PINS)
def test_the_configured_tcp_residual_is_pinned(points, cfg, name, mm, deg):
    """
    Description:
        DH_FK(q) * TCP_config vs _p: the kinematic error of nominal DH (no calibration.conf).
    """

    position, orientation = TCP_Residual(points[name].Q, points[name].P, cfg.TCP_Offset)
    assert position * 1e3 == pytest.approx(mm, abs=CONST_PIN_MM)
    assert math.degrees(orientation) == pytest.approx(deg, abs=CONST_PIN_DEG)


@pytest.mark.parametrize('name, xyz, deg', CONST_DERIVED_PINS)
def test_the_diagnostic_derived_tcp_is_pinned(points, name, xyz, deg):
    """
    Description:
        The TCP that would make _p exact at one pose, a diagnostic only: it differs between the
        two poses by the kinematic error, which is why the TCP is configured.
    """

    T = Derive_TCP_Offset(points[name].Q, points[name].P)
    assert list(T[:3, 3] * 1e3) == pytest.approx(list(xyz), abs=CONST_PIN_DERIVED_MM)
    assert math.degrees(Geodesic_Angle(np.eye(3), T[:3, :3])) == pytest.approx(deg, abs=CONST_PIN_DEG)


def test_the_configured_tcp_is_the_robotiq_value(cfg):
    """
    Description:
        The configured TCP is (0, 0, 174 mm, 0, 0, 0): the Polyscope TCP, equal to the Robotiq
        manual value; it is never derived.
    """

    assert cfg.TCP_Offset == [0.0, 0.0, 0.174, 0.0, 0.0, 0.0]


def test_both_learned_points_are_within_the_sanity_bound(points, cfg):
    """
    Description:
        A and B pass the runtime check with the bound of 10 mm and the derived orientation bound.
    """

    bound = Orientation_Sanity_Bound(cfg.FK_Sanity_Bound_Position, 0.18)
    for name in ('home', 'drop_rotate_1'):
        Check_FK_Consistency(name, points[name].Q, points[name].P, cfg.TCP_Offset, cfg.FK_Sanity_Bound_Position, bound)


def test_a_point_whose_joints_disagree_with_its_pose_is_refused(points, cfg):
    """
    Description:
        Shifting the base joint by 0.05 rad moves the TCP ~25 mm: refused with both numbers named.
    """

    q = list(points['drop_rotate_1'].Q)
    q[0] += 0.05
    with pytest.raises(Kinematics_Error_Cls, match=r'drop_rotate_1: .* mm .* deg from its _p'):
        Check_FK_Consistency('drop_rotate_1', q, points['drop_rotate_1'].P, cfg.TCP_Offset, cfg.FK_Sanity_Bound_Position,
                             Orientation_Sanity_Bound(cfg.FK_Sanity_Bound_Position, 0.18))


def test_the_geodesic_angle_ignores_the_rotation_vector_sign_flip_near_pi(points):
    """
    Description:
        home_p has |r| ~ 3.139 rad. r and the equivalent -(2*pi - |r|) * r/|r| differ component-wise
        by ~2*pi but are the same rotation: the geodesic angle between them is 0.
    """

    r = np.array(points['home'].P[3:])
    theta = np.linalg.norm(r)
    assert theta == pytest.approx(3.1393, abs=1e-4)
    equivalent = -(2.0 * math.pi - theta) * r / theta
    assert np.max(np.abs(r - equivalent)) > 4.0
    assert Geodesic_Angle(Rotvec_To_Matrix(r), Rotvec_To_Matrix(equivalent)) < 1e-12


def test_the_geodesic_angle_resolves_tiny_angles():
    """
    Description:
        atan2 resolves 1e-10 rad; the arccos form cannot go below ~6e-8 (SPEC 4.5).
    """

    assert Geodesic_Angle(np.eye(3), Rotvec_To_Matrix([1e-10, 0.0, 0.0])) == pytest.approx(1e-10, rel=1e-6)


@pytest.mark.parametrize('r', [[0.3, -0.2, 0.1], [0.0, 0.0, 0.0], [1.5, 1.5, -1.0], [-2.0, 0.5, 1.0]])
def test_rotation_vectors_round_trip(r):
    """
    Description:
        Below pi the normalised rotation vector of a matrix is the vector it came from.
    """

    assert list(Matrix_To_Rotvec(Rotvec_To_Matrix(r))) == pytest.approx(r, abs=1e-12)


@pytest.mark.parametrize('axis', [[-1.0, 0.0, 0.0], [0.0, -1.0, 1.0], [-1.0, 1.0, 0.0], [0.0, 0.0, -1.0]])
def test_at_pi_the_first_nonzero_component_is_positive(axis):
    """
    Description:
        At exactly pi, r and -r are the same rotation; the rule picks the one whose first
        component with magnitude above 1e-9 is positive, with angle pi.
    """

    axis = np.array(axis) / np.linalg.norm(axis)
    r = Matrix_To_Rotvec(Rotvec_To_Matrix(math.pi * axis))
    assert np.linalg.norm(r) == pytest.approx(math.pi, abs=1e-9)
    first = next(c for c in r if abs(c) > 1e-9)
    assert first > 0.0
    assert Geodesic_Angle(Rotvec_To_Matrix(r), Rotvec_To_Matrix(math.pi * axis)) < 1e-9


def test_rotations_on_either_side_of_pi_are_close():
    """
    Description:
        Rotations by pi - 1e-3 and pi + 1e-3 about the same axis differ by 2e-3 rad, but their
        normalised rotation vectors point in opposite directions (component-wise ~2*pi apart). The
        geodesic angle must give 2e-3 rad; a component-wise comparison gives ~6.28.
    """

    axis = np.array([0.6, -0.48, 0.64])
    axis /= np.linalg.norm(axis)
    below, above = Rotvec_To_Matrix((math.pi - 1e-3) * axis), Rotvec_To_Matrix((math.pi + 1e-3) * axis)
    assert np.linalg.norm(Matrix_To_Rotvec(below) - Matrix_To_Rotvec(above)) > 6.0
    assert Geodesic_Angle(below, above) == pytest.approx(2e-3, abs=1e-12)


def test_the_ik_finds_the_configuration_on_the_branch_it_starts_from(cfg):
    """
    Description:
        Solve_IK_Near, started 0.05 rad (per joint, seeded) from 20 seeded configurations within
        the joint limits away from the elbow singularity, returns a configuration whose DH TCP is
        the target within 1e-9 m / 1e-9 rad, and it is the configuration itself (the same branch,
        1e-6 rad). A target 3 m away is refused, never approximated.
    """

    rng = np.random.default_rng(20261018)
    for _ in range(20):
        q = rng.uniform(-math.pi, math.pi, 6)
        q[2] = rng.uniform(0.5, 2.5) * rng.choice([-1.0, 1.0])
        q[4] = rng.uniform(0.5, 2.5)
        target = TCP_Pose(q, cfg.TCP_Offset)
        found = Solve_IK_Near(target, q + rng.uniform(-0.05, 0.05, 6), cfg.TCP_Offset)
        reached = TCP_Pose(found, cfg.TCP_Offset)
        assert np.linalg.norm(reached[:3, 3] - target[:3, 3]) < 1e-9
        assert Geodesic_Angle(reached[:3, :3], target[:3, :3]) < 1e-9
        assert np.max(np.abs(found - q)) < 1e-6
    far = TCP_Pose(np.zeros(6), cfg.TCP_Offset)
    far[:3, 3] += [3.0, 0.0, 0.0]
    with pytest.raises(Kinematics_Error_Cls, match='out of reach or at a singularity'):
        Solve_IK_Near(far, np.zeros(6), cfg.TCP_Offset)


def test_the_analytic_ik_gives_all_eight_branches_and_the_configuration_itself(cfg):
    """
    Description:
        For 50 seeded configurations within the joint limits away from the singularities (elbow
        and wrist 2 at least 0.3 rad from 0 and pi), Solve_IK_Analytic of their DH TCP returns at
        most 8 solutions, each on its own branch (a pose may be out of reach on one shoulder or
        elbow branch), each reaching the TCP within 1e-9 m / 1e-9 rad (checked here with TCP_Pose),
        and one of them is the configuration itself modulo 2 pi (1e-8 rad), on the branch IK_Branch
        names. Some pose has all 8.
    """

    rng = np.random.default_rng(20261019)
    counts = []
    for _ in range(50):
        q = rng.uniform(-2.0 * math.pi, 2.0 * math.pi, 6)
        q[2] = rng.uniform(0.3, math.pi - 0.3) * rng.choice([-1.0, 1.0])
        q[4] = rng.uniform(0.3, math.pi - 0.3) * rng.choice([-1.0, 1.0])
        target = TCP_Pose(q, cfg.TCP_Offset)
        solutions = Solve_IK_Analytic(target, cfg.TCP_Offset)
        counts.append(len(solutions))
        assert 1 <= len(solutions) <= 8 and len({branch for branch, _ in solutions}) == len(solutions)
        for _, s in solutions:
            reached = TCP_Pose(s, cfg.TCP_Offset)
            assert np.linalg.norm(reached[:3, 3] - target[:3, 3]) < 1e-9
            assert Geodesic_Angle(reached[:3, :3], target[:3, :3]) < 1e-9
        same = [branch for branch, s in solutions if max(abs(Wrap_Angle(a - b)) for a, b in zip(s, q)) < 1e-8]
        assert same == [IK_Branch(q)]
    assert max(counts) == 8


@pytest.mark.parametrize('name, branch', [('home', 'S- E- W+'), ('drop_rotate_1', 'S+ E+ W-'), ('Waypoint_4', 'S+ E+ W-'),
                                          ('Waypoint_5', 'S+ E+ W-'), ('Waypoint_18', 'S- E- W+'), ('pick_rotate_1', 'S- E- W+'),
                                          ('Waypoint_6', 'S- E- W+')])
def test_the_taught_points_lie_on_their_measured_branches(points, name, branch):
    """
    Description:
        The branches of case 17's taught home and station points (measured 2026-10-19): the drop's
        three points are on the branch opposite home in shoulder, elbow and wrist; the pick's are on
        home's branch. The pose of each (its _q's DH TCP) has all 8 solutions.
    """

    assert IK_Branch(points[name].Q) == branch
    assert len(Solve_IK_Analytic(DH_FK(points[name].Q), [0.0] * 6)) == 8


def test_the_nearest_representative_turns_every_joint_but_the_elbow():
    """
    Description:
        Nearest_Representative moves each joint but the elbow by whole turns to within pi of the
        reference: wrist 3 of the taught pick (4.739 rad) next to home (0) becomes 4.739 - 2 pi; a
        joint already within pi is kept exactly; the elbow is never turned, even 3 rad away.
    """

    q = Nearest_Representative([0.5, -7.0, 3.0, 2.0, -4.0, 4.739], [0.0, 0.0, -0.1, 0.0, 0.0, 0.0])
    assert q[0] == 0.5 and q[3] == 2.0 and q[2] == 3.0
    assert q[5] == pytest.approx(4.739 - 2.0 * math.pi, abs=1e-12)
    assert q[1] == pytest.approx(-7.0 + 2.0 * math.pi, abs=1e-12)
    assert q[4] == pytest.approx(-4.0 + 2.0 * math.pi, abs=1e-12)


def test_a_movel_follows_the_straight_tcp_line_to_its_target(cfg, points):
    """
    Description:
        Linear_Path of the station movel drop_rotate_1 -> Waypoint_4 (60 mm down) and of the lift
        pick_rotate_1 -> Waypoint_6 (194 mm up): every configuration's DH TCP lies on the straight
        line between the two TCPs (1e-9 m), in order, at most 5 mm apart; the orientation turns
        evenly about one axis; the path starts at the start's _q and ends at the target's _q
        exactly.
    """

    for start, goal in (('drop_rotate_1', 'Waypoint_4'), ('pick_rotate_1', 'Waypoint_6')):
        q_a, q_b = points[start].Q, points[goal].Q
        path = Linear_Path(q_a, q_b, cfg.TCP_Offset)
        T_a, T_b = TCP_Pose(q_a, cfg.TCP_Offset), TCP_Pose(q_b, cfg.TCP_Offset)
        line = T_b[:3, 3] - T_a[:3, 3]
        assert list(path[0]) == list(q_a) and list(path[-1]) == list(q_b)
        turn = Geodesic_Angle(T_a[:3, :3], T_b[:3, :3])
        for k, q in enumerate(path):
            T = TCP_Pose(q, cfg.TCP_Offset)
            along = float(np.dot(T[:3, 3] - T_a[:3, 3], line) / np.dot(line, line))
            assert along == pytest.approx(k / (len(path) - 1), abs=1e-6)
            assert np.linalg.norm(T_a[:3, 3] + along * line - T[:3, 3]) < 1e-9
            assert Geodesic_Angle(T_a[:3, :3], T[:3, :3]) == pytest.approx(turn * k / (len(path) - 1), abs=1e-6)
        assert np.linalg.norm(line) / (len(path) - 1) <= 0.005 + 1e-12
