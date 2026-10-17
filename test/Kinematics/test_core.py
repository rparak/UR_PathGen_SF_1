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
                             Check_FK_Consistency, Kinematics_Error_Cls, Orientation_Sanity_Bound)

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
CONST_RESIDUAL_PINS = (('home_safe', 1.7177033, 0.2608401), ('drop_rot_01', 4.0229419, 0.3763663))
# Diagnostic derived TCP per point: position in the flange frame [mm], rotation angle [deg].
#   Unit [Tuple]
CONST_DERIVED_PINS = (('home_safe', (1.112, -0.036, 172.692), 0.2608401),
                      ('drop_rot_01', (-3.299, -0.229, 171.709), 0.3763663))
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
    for name in ('home_safe', 'drop_rot_01'):
        Check_FK_Consistency(name, points[name].Q, points[name].P, cfg.TCP_Offset, cfg.FK_Sanity_Bound_Position, bound)


def test_a_point_whose_joints_disagree_with_its_pose_is_refused(points, cfg):
    """
    Description:
        Shifting the base joint by 0.05 rad moves the TCP ~25 mm: refused with both numbers named.
    """

    q = list(points['drop_rot_01'].Q)
    q[0] += 0.05
    with pytest.raises(Kinematics_Error_Cls, match=r'drop_rot_01: .* mm .* deg from its _p'):
        Check_FK_Consistency('drop_rot_01', q, points['drop_rot_01'].P, cfg.TCP_Offset, cfg.FK_Sanity_Bound_Position,
                             Orientation_Sanity_Bound(cfg.FK_Sanity_Bound_Position, 0.18))


def test_the_geodesic_angle_ignores_the_rotation_vector_sign_flip_near_pi(points):
    """
    Description:
        home_p has |r| ~ 3.139 rad. r and the equivalent -(2*pi - |r|) * r/|r| differ component-wise
        by ~2*pi but are the same rotation: the geodesic angle between them is 0.
    """

    r = np.array(points['home_safe'].P[3:])
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
