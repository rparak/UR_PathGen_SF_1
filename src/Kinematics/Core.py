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
# Numeric inverse kinematics (Solve_IK_Near): damped least squares on the DH Jacobian, taken by
# central differences. Converged when the position error is below 1e-9 m and the geodesic angle
# below 1e-9 rad; at most 100 iterations.
#   Unit [float] (rad), [float] (-), [float] (m), [float] (rad), [int]
CONST_IK_DELTA, CONST_IK_DAMPING, CONST_IK_POSITION_TOL, CONST_IK_ANGLE_TOL, CONST_IK_ITERATIONS = 1e-7, 1e-6, 1e-9, 1e-9, 100
# Largest TCP step between two configurations of a straight-line motion (Linear_Path): 5 mm and
# 1 deg, so consecutive solutions stay on one branch; the clearance check samples between them.
#   Unit [float] (m), [float] (rad)
CONST_LINE_STEP, CONST_LINE_ANGLE_STEP = 0.005, math.radians(1.0)


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
        T = T @ DH_Transform(i, q[i])
    return T


def DH_Transform(i: int, theta: float) -> np.ndarray:
    """
    Description:
        The DH transform of one joint: frame i to frame i + 1 at the joint angle theta.

    Args:
        (1) i [int]: The joint, 0 (base) to 5 (wrist 3).
        (2) theta [float]: Its angle (rad).

    Returns:
        (1) parameter [Matrix<float> 4x4]: The transform.
    """

    ct, st = math.cos(theta), math.sin(theta)
    ca, sa = math.cos(CONST_DH_ALPHA[i]), math.sin(CONST_DH_ALPHA[i])
    return np.array([[ct, -st * ca, st * sa, CONST_DH_A[i] * ct],
                     [st, ct * ca, -ct * sa, CONST_DH_A[i] * st],
                     [0.0, sa, ca, CONST_DH_D[i]],
                     [0.0, 0.0, 0.0, 1.0]])


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


def Pose_Error(T: np.ndarray, T_goal: np.ndarray) -> np.ndarray:
    """
    Description:
        The 6-vector from T to T_goal: the position difference and the rotation vector of
        R_target * R^T (both in the base frame).

    Args:
        (1 - 2) T, T_goal [Matrix<float> 4x4]: The poses.

    Returns:
        (1) parameter [Vector<float> 1x6]: (m, rad).
    """

    return np.concatenate([T_goal[:3, 3] - T[:3, 3], Matrix_To_Rotvec(T_goal[:3, :3] @ T[:3, :3].T)])


def Solve_IK_Near(T_goal: np.ndarray, q_near: tp.Sequence[float], tcp_offset: tp.Sequence[float]) -> np.ndarray:
    """
    Description:
        The joint angles whose DH TCP is T_goal, found from q_near by damped least squares: the
        solution on q_near's branch, as the robot's controller follows a movel. Joints are never
        wrapped. A target it cannot reach is an error, never an approximate answer.

    Args:
        (1) T_goal [Matrix<float> 4x4]: The TCP pose in the Polyscope base frame.
        (2) q_near [Vector<float> 1x6]: The start (rad), close to the solution.
        (3) tcp_offset [Vector<float> 1x6]: The configured TCP (m, rad).

    Returns:
        (1) parameter [Vector<float> 1x6]: The joint angles (rad).
    """

    q = np.array(q_near, dtype=float)
    for _ in range(CONST_IK_ITERATIONS):
        T = TCP_Pose(q, tcp_offset)
        error = Pose_Error(T, T_goal)
        if np.linalg.norm(error[:3]) < CONST_IK_POSITION_TOL and Geodesic_Angle(T[:3, :3], T_goal[:3, :3]) < CONST_IK_ANGLE_TOL:
            return q
        J = np.zeros((6, 6))
        for k in range(6):
            dq = np.zeros(6)
            dq[k] = CONST_IK_DELTA
            J[:, k] = (Pose_Error(TCP_Pose(q - dq, tcp_offset), TCP_Pose(q + dq, tcp_offset))) / (2.0 * CONST_IK_DELTA)
        q = q + J.T @ np.linalg.solve(J @ J.T + CONST_IK_DAMPING ** 2 * np.eye(6), error)
    raise Kinematics_Error_Cls('No joint solution near %s reaches the TCP pose within %d iterations (left %.3f mm). The pose '
                               'is out of reach or at a singularity.'
                               % (np.round(np.asarray(q_near), 4).tolist(), CONST_IK_ITERATIONS, np.linalg.norm(error[:3]) * 1e3))


def Linear_Path(q_a: tp.Sequence[float], q_b: tp.Sequence[float], tcp_offset: tp.Sequence[float]) -> tp.List[np.ndarray]:
    """
    Description:
        The joint configurations of a movel from q_a to q_b: the TCP moves on the straight line
        between their DH TCPs, the orientation turns about one axis at a constant rate, steps of at
        most CONST_LINE_STEP and CONST_LINE_ANGLE_STEP, each solved from the previous one. The line
        must end at q_b itself (the movel's target _q), else the controller would reach another
        branch: an error.

    Args:
        (1 - 2) q_a, q_b [Vector<float> 1x6]: Start and target (rad).
        (3) tcp_offset [Vector<float> 1x6]: The configured TCP (m, rad).

    Returns:
        (1) parameter [List<Vector<float> 1x6>]: The configurations, q_a first and q_b last (rad).
    """

    T_a, T_b = TCP_Pose(q_a, tcp_offset), TCP_Pose(q_b, tcp_offset)
    turn = Matrix_To_Rotvec(T_b[:3, :3] @ T_a[:3, :3].T)
    count = max(1, int(math.ceil(max(np.linalg.norm(T_b[:3, 3] - T_a[:3, 3]) / CONST_LINE_STEP,
                                     np.linalg.norm(turn) / CONST_LINE_ANGLE_STEP))))
    path = [np.array(q_a, dtype=float)]
    for k in range(1, count + 1):
        T = np.eye(4)
        T[:3, 3] = T_a[:3, 3] + (T_b[:3, 3] - T_a[:3, 3]) * k / count
        T[:3, :3] = Rotvec_To_Matrix(turn * k / count) @ T_a[:3, :3]
        path.append(Solve_IK_Near(T, path[-1], tcp_offset))
    if np.max(np.abs(path[-1] - np.asarray(q_b, dtype=float))) > 1e-6:
        raise Kinematics_Error_Cls('The straight line from %s ends at %s, not at its target %s: another branch.'
                                   % (np.round(np.asarray(q_a), 4).tolist(), np.round(path[-1], 4).tolist(),
                                      np.round(np.asarray(q_b), 4).tolist()))
    path[-1] = np.array(q_b, dtype=float)
    return path


def Wrap_Angle(x: float) -> float:
    """
    Description:
        An angle in [-pi, pi).

    Args:
        (1) x [float]: The angle (rad).

    Returns:
        (1) parameter [float]: The wrapped angle (rad).
    """

    return (x + math.pi) % (2.0 * math.pi) - math.pi


def Solve_IK_Analytic(T_goal: np.ndarray, tcp_offset: tp.Sequence[float]) -> tp.List[tp.Tuple[str, np.ndarray]]:
    """
    Description:
        Every closed-form solution of the UR10e (official DH) for a TCP pose: up to 8, one per
        branch 'S<s> E<e> W<w>'. S: the sign of the shoulder solution, theta1 = atan2(y5, x5)
        + s * acos(d4 / r5) + pi / 2 (wrist centre (x5, y5), r5 its distance from the base axis);
        E: the sign of the elbow angle; W: the sign of sin(theta5). Angles in [-pi, pi), the elbow
        in [-pi, pi] (its limit); every other joint may also take its value +- 2 pi
        (Nearest_Representative). A branch at a singularity (wrist sin(theta5) = 0, elbow
        stretched, wrist centre on the base axis) is left out. Each solution is checked by DH_FK
        to 1e-9 m and rad: a wrong one is an error, never returned.

    Args:
        (1) T_goal [Matrix<float> 4x4]: The TCP pose in the Polyscope base frame.
        (2) tcp_offset [Vector<float> 1x6]: The configured TCP (m, rad).

    Returns:
        (1) parameter [List<Tuple<string, Vector<float> 1x6>>]: (branch, joint angles (rad)).
    """

    d1, _, _, d4, _, d6 = CONST_DH_D
    a2, a3 = CONST_DH_A[1], CONST_DH_A[2]
    T = np.asarray(T_goal, dtype=float) @ np.linalg.inv(Pose_To_Matrix(tcp_offset))
    wrist = T[:3, 3] - d6 * T[:3, 2]
    r5 = math.hypot(wrist[0], wrist[1])
    out: tp.List[tp.Tuple[str, np.ndarray]] = []
    if r5 <= d4:
        return out
    for s in (1.0, -1.0):
        t1 = math.atan2(wrist[1], wrist[0]) + s * math.acos(d4 / r5) + math.pi / 2.0
        s1, c1 = math.sin(t1), math.cos(t1)
        c5 = (T[0, 3] * s1 - T[1, 3] * c1 - d4) / d6
        if abs(c5) >= 1.0:
            continue
        for w in (1.0, -1.0):
            t5 = w * math.acos(c5)
            s5 = math.sin(t5)
            t6 = math.atan2((-T[0, 1] * s1 + T[1, 1] * c1) / s5, (T[0, 0] * s1 - T[1, 0] * c1) / s5)
            T14 = np.linalg.inv(DH_Transform(0, t1)) @ T @ np.linalg.inv(DH_Transform(4, t5) @ DH_Transform(5, t6))
            x, y = (T14 @ np.array([0.0, -d4, 0.0, 1.0]))[:2]
            c3 = (x * x + y * y - a2 * a2 - a3 * a3) / (2.0 * a2 * a3)
            if abs(c3) >= 1.0:
                continue
            for e in (1.0, -1.0):
                t3 = e * math.acos(c3)
                t2 = -math.atan2(y, -x) + math.asin(a3 * math.sin(t3) / math.hypot(x, y))
                T34 = np.linalg.inv(DH_Transform(1, t2) @ DH_Transform(2, t3)) @ T14
                t4 = math.atan2(T34[1, 0], T34[0, 0])
                q = np.array([Wrap_Angle(t1), Wrap_Angle(t2), t3, Wrap_Angle(t4), Wrap_Angle(t5), Wrap_Angle(t6)])
                found = DH_FK(q)
                if np.linalg.norm(found[:3, 3] - T[:3, 3]) > 1e-9 or Geodesic_Angle(found[:3, :3], T[:3, :3]) > 1e-9:
                    raise Kinematics_Error_Cls('The analytic solution %s of branch S%s E%s W%s misses its pose; the closed-form '
                                               'IK is wrong for this pose.' % (np.round(q, 6).tolist(), '+-'[s < 0],
                                                                                '+-'[e < 0], '+-'[w < 0]))
                out.append(('S%s E%s W%s' % ('+-'[s < 0], '+-'[e < 0], '+-'[w < 0]), q))
    return out


def IK_Branch(q: tp.Sequence[float]) -> str:
    """
    Description:
        The branch (Solve_IK_Analytic) of a configuration: that of the analytic solution of its own
        flange pose that equals it, joint by joint modulo 2 pi.

    Args:
        (1) q [Vector<float> 1x6]: Joint angles (rad).

    Returns:
        (1) parameter [string]: 'S<s> E<e> W<w>'.
    """

    solutions = Solve_IK_Analytic(DH_FK(q), [0.0] * 6)
    if not solutions:
        raise Kinematics_Error_Cls('The configuration %s is at a singularity; it has no branch.' % np.round(np.asarray(q), 6).tolist())
    def Distance(solution: np.ndarray) -> float:
        """
        Description:
            The largest joint difference modulo 2 pi.

        Args:
            (1) solution [Vector<float> 1x6]: An analytic solution (rad).

        Returns:
            (1) parameter [float]: The difference (rad).
        """

        return max(abs(Wrap_Angle(a - b)) for a, b in zip(solution, q))
    return min(solutions, key=lambda item: Distance(item[1]))[0]


def Nearest_Representative(q: tp.Sequence[float], q_near: tp.Sequence[float]) -> np.ndarray:
    """
    Description:
        The same configuration with every joint but the elbow moved by whole turns (2 pi) to lie
        within pi of q_near: the representative a joint-linear move from q_near reaches with the
        least rotation. The elbow is kept (its limit is +-pi, SPEC 8).

    Args:
        (1) q [Vector<float> 1x6]: Joint angles (rad).
        (2) q_near [Vector<float> 1x6]: The reference (rad).

    Returns:
        (1) parameter [Vector<float> 1x6]: The representative (rad).
    """

    out = np.array(q, dtype=float)
    for i in (0, 1, 3, 4, 5):
        out[i] = q_near[i] + Wrap_Angle(out[i] - q_near[i])
    return out
