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
# File Name: ../Kinematics/Core.py
## =========================================================================== ##

# Math (Mathematical functions)
import math
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np

"""
Description:
    Initialization of constants.

    The one kinematic truth of the project: the official UR10e DH parameters (Universal Robots,
    "DH parameters for calculations of kinematics and dynamics"). Frames: FK returns the flange
    (DH frame 6) in the Polyscope base frame; a TCP is the flange times the configured TCP offset.
    Orientations are compared by the geodesic angle, never by rotation-vector components.
"""
# UR10e DH parameters: d and a in metres, alpha in radians, joints base to wrist 3.
#   Unit [Vector<float> 1x6]
CONST_DH_D = (0.1807, 0.0, 0.0, 0.17415, 0.11985, 0.11655)
CONST_DH_A = (0.0, -0.6127, -0.57155, 0.0, 0.0, 0.0)
CONST_DH_ALPHA = (math.pi / 2.0, 0.0, 0.0, math.pi / 2.0, -math.pi / 2.0, 0.0)
# Below this angle a rotation is the identity, and its rotation vector is written as zeros.
#   Unit [float] (rad)
CONST_ZERO_ANGLE = 1e-12
# Within this distance of pi the axis sign of a rotation vector is ambiguous (r and -r describe the
# same rotation); the first component with magnitude above CONST_AXIS_EPS is then made positive.
#   Unit [float] (rad)
CONST_PI_BAND = 1e-9
CONST_AXIS_EPS = 1e-9
# Closer to pi than this, the axis is taken from the symmetric part of R: sin(theta) is too small
# for the skew part to give it accurately.
#   Unit [float] (rad)
CONST_SYMMETRIC_BAND = 1e-6


class Kinematics_Error_Cls(ValueError):
    """
    Description:
        A learned point is kinematically inconsistent: its _q and _p disagree beyond the sanity
        bound. The message gives both differences and what to check.
    """


def DH_FK(q: tp.Sequence[float]) -> np.ndarray:
    """
    Description:
        Forward kinematics of the UR10e with the official DH parameters.

    Args:
        (1) q [Vector<float> 1x6]: Joint angles (rad), base to wrist 3.

    Returns:
        (1) parameter [Matrix<float> 4x4]: The flange in the Polyscope base frame.
    """

    T = np.eye(4)
    for i in range(6):
        ct, st = math.cos(q[i]), math.sin(q[i])
        ca, sa = math.cos(CONST_DH_ALPHA[i]), math.sin(CONST_DH_ALPHA[i])
        T = T @ np.array([[ct, -st * ca, st * sa, CONST_DH_A[i] * ct],
                          [st, ct * ca, -ct * sa, CONST_DH_A[i] * st],
                          [0.0, sa, ca, CONST_DH_D[i]],
                          [0.0, 0.0, 0.0, 1.0]])
    return T


def Rotvec_To_Matrix(r: tp.Sequence[float]) -> np.ndarray:
    """
    Description:
        Rodrigues' formula: a rotation vector (axis * angle) to a rotation matrix.

    Args:
        (1) r [Vector<float> 1x3]: The rotation vector (rad).

    Returns:
        (1) parameter [Matrix<float> 3x3]: The rotation matrix.
    """

    r = np.asarray(r, dtype=float)
    theta = float(np.linalg.norm(r))
    if theta < CONST_ZERO_ANGLE:
        return np.eye(3)
    k = r / theta
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(theta) * K + (1.0 - math.cos(theta)) * (K @ K)


def Geodesic_Angle(R_a: np.ndarray, R_b: np.ndarray) -> float:
    """
    Description:
        The angle of the rotation between two orientations, atan2(|vee(M - M^T)| / 2, (tr M - 1) / 2)
        with M = R_a^T R_b.

        Note:
            The arccos form, arccos((tr M - 1) / 2), cannot resolve angles below ~6e-8 rad near 0
            (measured, SPEC 4.5); atan2 resolves ~1e-16.

    Args:
        (1 - 2) R_a, R_b [Matrix<float> 3x3]: Rotation matrices.

    Returns:
        (1) parameter [float]: The angle in [0, pi] (rad).
    """

    M = np.asarray(R_a).T @ np.asarray(R_b)
    v = np.array([M[2, 1] - M[1, 2], M[0, 2] - M[2, 0], M[1, 0] - M[0, 1]])
    return math.atan2(float(np.linalg.norm(v)) / 2.0, (float(np.trace(M)) - 1.0) / 2.0)


def Matrix_To_Rotvec(R: np.ndarray) -> np.ndarray:
    """
    Description:
        A rotation matrix to its normalised rotation vector, the rule for generated points: the
        angle is in [0, pi]; below CONST_ZERO_ANGLE the vector is zero; within CONST_PI_BAND of pi
        the first component with magnitude above CONST_AXIS_EPS is positive. Learned points are
        never passed through this: they are written verbatim.

    Args:
        (1) R [Matrix<float> 3x3]: The rotation matrix.

    Returns:
        (1) parameter [Vector<float> 1x3]: The rotation vector (rad).
    """

    R = np.asarray(R, dtype=float)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    theta = math.atan2(float(np.linalg.norm(v)) / 2.0, (float(np.trace(R)) - 1.0) / 2.0)
    if theta < CONST_ZERO_ANGLE:
        return np.zeros(3)
    if math.pi - theta > CONST_SYMMETRIC_BAND:
        axis = v / np.linalg.norm(v)
    else:
        # Near pi: R + I = 2 k k^T (approximately); its largest column gives the axis.
        B = (R + np.eye(3)) / 2.0
        k = int(np.argmax(np.diag(B)))
        axis = B[:, k] / math.sqrt(max(B[k, k], 1e-300))
        axis = axis / np.linalg.norm(axis)
        if float(np.dot(axis, v)) < 0.0:
            axis = -axis
    if math.pi - theta < CONST_PI_BAND:
        for component in axis:
            if abs(component) > CONST_AXIS_EPS:
                if component < 0.0:
                    axis = -axis
                break
    return axis * theta


def Pose_To_Matrix(p: tp.Sequence[float]) -> np.ndarray:
    """
    Description:
        A Polyscope pose [x, y, z, rx, ry, rz] to a homogeneous matrix.

    Args:
        (1) p [Vector<float> 1x6]: Position (m) and rotation vector (rad).

    Returns:
        (1) parameter [Matrix<float> 4x4]: The pose.
    """

    T = np.eye(4)
    T[:3, :3] = Rotvec_To_Matrix(p[3:6])
    T[:3, 3] = p[0:3]
    return T


def Matrix_To_Pose(T: np.ndarray) -> tp.List[float]:
    """
    Description:
        A homogeneous matrix to a Polyscope pose with a normalised rotation vector.

    Args:
        (1) T [Matrix<float> 4x4]: The pose.

    Returns:
        (1) parameter [Vector<float> 1x6]: Position (m) and rotation vector (rad).
    """

    return [float(x) for x in T[:3, 3]] + [float(x) for x in Matrix_To_Rotvec(T[:3, :3])]


def TCP_Pose(q: tp.Sequence[float], tcp_offset: tp.Sequence[float]) -> np.ndarray:
    """
    Description:
        The TCP in the Polyscope base frame: DH_FK(q) * TCP offset.

    Args:
        (1) q [Vector<float> 1x6]: Joint angles (rad).
        (2) tcp_offset [Vector<float> 1x6]: The configured TCP in the flange frame (m, rad).

    Returns:
        (1) parameter [Matrix<float> 4x4]: The TCP pose.
    """

    return DH_FK(q) @ Pose_To_Matrix(tcp_offset)


def TCP_Residual(q: tp.Sequence[float], p: tp.Sequence[float], tcp_offset: tp.Sequence[float]) -> tp.Tuple[float, float]:
    """
    Description:
        How far DH_FK(q) * TCP is from the pose p. With the configured TCP this is kinematic error:
        nominal DH against the robot's own calibration, which computed p from the same q.

    Args:
        (1) q [Vector<float> 1x6]: Joint angles (rad).
        (2) p [Vector<float> 1x6]: The pose (m, rad).
        (3) tcp_offset [Vector<float> 1x6]: The configured TCP (m, rad).

    Returns:
        (1) parameter [Tuple<float> 1x2]: (position error (m), geodesic orientation error (rad)).
    """

    T, P = TCP_Pose(q, tcp_offset), Pose_To_Matrix(p)
    return float(np.linalg.norm(T[:3, 3] - P[:3, 3])), Geodesic_Angle(T[:3, :3], P[:3, :3])


def Derive_TCP_Offset(q: tp.Sequence[float], p: tp.Sequence[float]) -> np.ndarray:
    """
    Description:
        Diagnostic only: the TCP that would make p match exactly at this one pose,
        DH_FK(q)^-1 * T(p). It differs between poses by the kinematic error, which is why the TCP
        is configured and never derived. Nothing reads this to configure or check anything.

    Args:
        (1) q [Vector<float> 1x6]: Joint angles (rad).
        (2) p [Vector<float> 1x6]: The pose (m, rad).

    Returns:
        (1) parameter [Matrix<float> 4x4]: The TCP in the flange frame.
    """

    return np.linalg.inv(DH_FK(q)) @ Pose_To_Matrix(p)


def Orientation_Sanity_Bound(position_bound: float, tool_length: float) -> float:
    """
    Description:
        The orientation sanity bound derived from the position bound: the rotation at which the
        farthest gripper point, tool_length from the flange origin, moves by the position bound.

    Args:
        (1) position_bound [float]: The position sanity bound (m).
        (2) tool_length [float]: Largest distance of a gripper collision vertex from the flange origin (m).

    Returns:
        (1) parameter [float]: The bound (rad).
    """

    return math.asin(min(1.0, position_bound / tool_length))


def Check_FK_Consistency(name: str, q: tp.Sequence[float], p: tp.Sequence[float], tcp_offset: tp.Sequence[float],
                         position_bound: float, orientation_bound: float) -> tp.Tuple[float, float]:
    """
    Description:
        Refuse a learned point whose _q and _p disagree beyond the sanity bound (a typo, another
        TCP, another robot). Never accepted silently.

    Args:
        (1) name [string]: The point name, for the message.
        (2 - 3) q, p [Vector<float> 1x6]: Its joints (rad) and pose (m, rad).
        (4) tcp_offset [Vector<float> 1x6]: The configured TCP.
        (5 - 6) position_bound, orientation_bound [float]: The sanity bounds (m, rad).

    Returns:
        (1) parameter [Tuple<float> 1x2]: The residual (m, rad) when within the bounds.
    """

    position, orientation = TCP_Residual(q, p, tcp_offset)
    if position > position_bound or orientation > orientation_bound:
        raise Kinematics_Error_Cls(
            'Point %s: DH FK of its _q with the configured TCP is %.2f mm / %.3f deg from its _p; the sanity bound is '
            '%.2f mm / %.3f deg. Check that _p and _q were taught together, with the same TCP, on this robot.'
            % (name, position * 1e3, math.degrees(orientation), position_bound * 1e3, math.degrees(orientation_bound)))
    return position, orientation
