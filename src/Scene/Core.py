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
# File Name: ../Scene/Core.py
## =========================================================================== ##

# Math (Mathematical functions)
import math
# Typing (Support for type hints)
import typing as tp
# Dataclasses (Data Classes)
from dataclasses import dataclass
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# MuJoCo (Physics simulator) [pip3 install mujoco]
import mujoco
# Custom Lib.:
#   ../Configuration/Core — the cell configuration
from Configuration.Core import Cell_Configuration_Str, CONST_PROJECT_ROOT
#   ../Kinematics/Core — the DH truth and the learned-point check
from Kinematics.Core import CONST_DH_D, Check_FK_Consistency, Orientation_Sanity_Bound, TCP_Pose
#   ../Collision/Core — the environment collision set and the STL reader
from Collision.Core import Read_Collision_Set, Read_Binary_STL, Component_Overrides_Record, Assembly_Overrides_Record
#   ../URScript/Core — the robot point structure
from URScript.Core import Robot_Point_Str

"""
Description:
    Initialization of constants.

    Builds the MuJoCo scene through MjSpec (SPEC 9): the vendored Menagerie files are never
    edited; every patch is applied here. The scene is kinematic only: configurations are set and
    checked, nothing is simulated.
"""
# Menagerie UR10e body offsets replaced by the DH values (SPEC 4.2, 9.2): (body, position in its
# parent). The y offsets 0.176 - 0.137 + 0.13515 sum to d4 = 0.17415. The flange site moves from
# 0.100 to d6 = 0.11655 m along the wrist-3 axis: Menagerie had it 16.5 mm inside the wrist.
#   Unit [Tuple<Tuple<string, Vector<float> 1x3>>] (m)
CONST_DH_BODY_OFFSETS = (('shoulder_link', (0.0, 0.0, CONST_DH_D[0])),
                         ('forearm_link', (0.0, -0.137, 0.6127)),
                         ('wrist_1_link', (0.0, 0.0, 0.57155)),
                         ('wrist_2_link', (0.0, CONST_DH_D[3] - 0.176 + 0.137, 0.0)),
                         ('wrist_3_link', (0.0, 0.0, CONST_DH_D[4])))
CONST_FLANGE_OFFSET = (0.0, CONST_DH_D[5], 0.0)
# The Menagerie flange site and the robot's joints, base to wrist 3.
#   Unit [string], [Tuple<string>]
CONST_FLANGE_SITE = 'attachment_site'
CONST_ROBOT_JOINTS = ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint',
                      'wrist_2_joint', 'wrist_3_joint')
# Robot bodies, base to wrist 3. The base is static (no joint).
#   Unit [Tuple<string>]
CONST_ROBOT_BODIES = ('base', 'shoulder_link', 'upper_arm_link', 'forearm_link', 'wrist_1_link', 'wrist_2_link',
                      'wrist_3_link')
# Prefix of every gripper element in the scene.
#   Unit [string]
CONST_GRIPPER_PREFIX = 'gripper_'
# The gripper datum (the face its Ø71 spigot seats on) in the Menagerie model: the 13.9 mm
# base_mount disc minus the 2.9 mm bore, i.e. 11.0 mm from the mount face. Inferred from the
# Menagerie finger ends matching the manual's dimension chain to 0.03 / 0.31 mm with this datum
# (SPEC 4.4). The gripper is shifted so its datum lies at Thickness - Spigot_Depth.
#   Unit [float] (m)
CONST_MENAGERIE_GRIPPER_DATUM = 0.0110
# Position of the Menagerie base_mount body in the gripper root frame.
#   Unit [float] (m)
CONST_MENAGERIE_MOUNT_Z = 0.007
# Gripper control value range and the settle steps of the opening solve.
#   Unit [float], [int]
CONST_GRIPPER_CTRL_MAX = 255.0
CONST_GRIPPER_SETTLE_STEPS = 5000
CONST_GRIPPER_BISECTIONS = 30
# The pad face: the silicone pad's inner face centre in the pad body frame (Menagerie pad mesh:
# silicone y in [-6.6, -5.0] mm, length 37.5 mm).
#   Unit [Vector<float> 1x3] (m)
CONST_PAD_FACE_CENTRE = (0.0, -0.0066, 0.01875)
# Pad collision boxes of the Menagerie gripper, used for the pad-span hull.
#   Unit [Tuple<string>]
CONST_PAD_BOXES = ('left_pad1', 'left_pad2', 'right_pad1', 'right_pad2')
# Collision geom group (hidden in the viewer by default) and visual environment group.
#   Unit [int]
CONST_COLLISION_GROUP = 3
# Viewer colours of the collision geometry (no effect on collision): every robot and gripper
# collision geom in one translucent red, the pad-span hull orange so it stays distinguishable.
#   Unit [Vector<float> 1x4] (RGBA)
CONST_ROBOT_COLLISION_RGBA = [0.9, 0.3, 0.3, 0.3]
CONST_PAD_SPAN_RGBA = [1.0, 0.5, 0.0, 0.3]
CONST_ENV_VISUAL_GROUP = 1
# Name prefix of the geom of an Environment.Added_Boxes entry (env_box_<Name>).
#   Unit [string]
CONST_ADDED_BOX_PREFIX = 'env_box_'
# Extra radius of the crop sphere beyond the reach bound and the clearance (SPEC 7.1).
#   Unit [float] (m)
CONST_CROP_PAD = 0.05


class Scene_Error_Cls(ValueError):
    """
    Description:
        The scene cannot be built as configured (an unknown body in an exclusion, the static base
        inside the environment margin). The message says what to change.
    """


class Clearance_Error_Cls(ValueError):
    """
    Description:
        A learned point violates a clearance margin. The message names the point, every violating
        pair, its distance and the margin. There is no override.
    """


@dataclass
class Violation_Str:
    """
    Description:
        One body pair closer than its margin.
    """

    # The two bodies.
    #   Unit [string]
    Body_1: str = ''
    Body_2: str = ''
    # Their smallest signed distance (negative = penetration).
    #   Unit [float] (m)
    Distance: float = 0.0
    # The margin that applies to the pair.
    #   Unit [float] (m)
    Margin: float = 0.0


__gripper_cache: tp.Dict[tp.Tuple[str, float], tp.Tuple[tp.Dict[str, float], float, float, np.ndarray]] = {}


def Solve_Gripper_Opening(tool_model: str, opening: float) -> tp.Tuple[tp.Dict[str, float], float, float, np.ndarray]:
    """
    Description:
        The gripper joint angles that give an inner pad-face gap of 'opening', by bisection on the
        control value of the gripper alone, settled without gravity. Also returns the pad-span
        points: the corners of the four pad boxes in the gripper 'base' body frame. Cached.

    Args:
        (1) tool_model [string]: The 2F-85 MJCF.
        (2) opening [float]: The inner pad-face gap (m).

    Returns:
        (1) parameter [Tuple<Dict<string, float>, float, float, Matrix<float> 32x3>]:
            joint angles by (unprefixed) name, the achieved gap (m), the equality residual (m),
            the pad-span points (m).
    """

    key = (tool_model, opening)
    if key in __gripper_cache:
        return __gripper_cache[key]
    m = mujoco.MjModel.from_xml_path(tool_model)
    m.opt.gravity[:] = 0.0
    d = mujoco.MjData(m)
    pads = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, s + '_pad') for s in ('left', 'right')]

    def Settle(ctrl: float) -> float:
        """
        Description:
            Settle the gripper at a control value and measure the inner pad-face gap.

        Args:
            (1) ctrl [float]: The control value, 0 (open) to 255 (closed).

        Returns:
            (1) parameter [float]: The gap (m).
        """

        mujoco.mj_resetData(m, d)
        d.ctrl[0] = ctrl
        for _ in range(CONST_GRIPPER_SETTLE_STEPS):
            mujoco.mj_step(m, d)
        face = [d.xmat[b].reshape(3, 3) @ np.asarray(CONST_PAD_FACE_CENTRE) + d.xpos[b] for b in pads]
        return float(np.linalg.norm(face[0] - face[1]))

    lo, hi = 0.0, CONST_GRIPPER_CTRL_MAX
    for _ in range(CONST_GRIPPER_BISECTIONS):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if Settle(mid) > opening else (lo, mid)
    gap = Settle(0.5 * (lo + hi))
    joints = {m.joint(j).name: float(d.qpos[m.jnt_qposadr[j]]) for j in range(m.njnt)}
    residual = float(np.max(np.abs(d.efc_pos[:d.ne]))) if d.ne else 0.0
    base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'base')
    R_b, p_b = d.xmat[base].reshape(3, 3), d.xpos[base]
    corners = []
    for name in CONST_PAD_BOXES:
        g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, name)
        R, p, size = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g], m.geom_size[g]
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                for sz in (-1.0, 1.0):
                    corners.append(R_b.T @ (p + R @ (np.array([sx, sy, sz]) * size) - p_b))
    __gripper_cache[key] = (joints, gap, residual, np.array(corners))
    return __gripper_cache[key]


def Van_Der_Corput_Order(n: int) -> tp.List[int]:
    """
    Description:
        The indices 0 .. n-1, both ends first, then the middle, then the quarters and so on (the
        base-2 van der Corput sequence), each exactly once.

    Args:
        (1) n [int]: The number of samples.

    Returns:
        (1) parameter [List<int>]: A permutation of range(n).
    """

    if n <= 2:
        return list(range(n))
    order, seen = [0, n - 1], {0, n - 1}
    denominator = 2
    # Terminates: once denominator >= 2 (n - 1) the grid spacing is at most 0.5 sample, so every
    # index is the rounding of a grid point.
    while len(order) < n:
        for numerator in range(1, denominator, 2):
            k = int(round(numerator * (n - 1) / denominator))
            if k not in seen:
                seen.add(k)
                order.append(k)
        denominator *= 2
    return order


def Yaw_Quat(angle: float) -> tp.List[float]:
    """
    Description:
        The quaternion (w, x, y, z) of a rotation about z.

    Args:
        (1) angle [float]: The angle (rad).

    Returns:
        (1) parameter [Vector<float> 1x4]: The quaternion.
    """

    return [math.cos(angle / 2.0), 0.0, 0.0, math.sin(angle / 2.0)]


class Scene_Cls(object):
    """
    Description:
        The SF1 scene: the UR10e (DH-patched) with coupling and 2F-85 at a fixed opening, the
        environment collision set and its visual mesh, and the clearance check.

    Initialization of the Class:
        Args:
            (1) Cell_Configuration_Str [Cell_Configuration_Str(object)]: The cell configuration.
            (2) with_environment [bool]: Add the environment collision set (needs the built set).
            (3) with_visual [bool]: Add the 1.76 M-triangle visual mesh (needed only for viewing).

        Example:
            Initialization:
                # Initialization of the class.
                Cls = Scene_Cls(Load_Configuration(), with_environment=True, with_visual=False)

            Features:
                # Properties of the class.
                Cls.Model, Cls.Data

                # Functions of the class.
                Cls.Check_Clearance(q)
    """

    def __init__(self, Cell_Configuration_Str: Cell_Configuration_Str, with_environment: bool = True,
                 with_visual: bool = False) -> None:
        # << PRIVATE >> #
        self.__cfg = Cell_Configuration_Str
        self.__gripper_joints, self.__gap, self.__residual, pad_span = Solve_Gripper_Opening(
            self.__cfg.Tool_Model, self.__cfg.Opening)
        spec = self.__Build_Spec(pad_span, with_environment, with_visual)
        self.__model = spec.compile()
        self.__data = mujoco.MjData(self.__model)
        self.__robot_qpos = np.array([self.__model.jnt_qposadr[mujoco.mj_name2id(self.__model, mujoco.mjtObj.mjOBJ_JOINT, j)]
                                      for j in CONST_ROBOT_JOINTS])
        self.__flange = mujoco.mj_name2id(self.__model, mujoco.mjtObj.mjOBJ_SITE, CONST_FLANGE_SITE)
        self.__overrides = {frozenset((a, b)): margin for a, b, margin in self.__cfg.Self_Pair_Overrides}
        self.__zone_geoms = ({mujoco.mj_name2id(self.__model, mujoco.mjtObj.mjOBJ_GEOM, 'env_piece_%d' % i)
                              for i in self.__zone_pieces}
                             | {mujoco.mj_name2id(self.__model, mujoco.mjtObj.mjOBJ_GEOM, CONST_ADDED_BOX_PREFIX + name)
                                for name in self.__zone_boxes})
        # The geom of each station box (an added box that is a zone piece), for the station-box rule.
        self.__station_box_geoms = {mujoco.mj_name2id(self.__model, mujoco.mjtObj.mjOBJ_GEOM, CONST_ADDED_BOX_PREFIX + name): name
                                    for name in self.__zone_boxes}
        # Robot collision geoms (the static base included) for the workspace check of generated segments.
        m = self.__model
        self.__robot_geoms = np.array([g for g in range(m.ngeom) if m.geom_contype[g] and m.geom_bodyid[g] != 0
                                       and not m.body(m.geom_bodyid[g]).name.startswith('env')])
        self.__robot_local = [self.__Geom_Local(g) for g in self.__robot_geoms]
        self.__robot_rbound = np.array([float(np.max(np.linalg.norm(local, axis=1))) for local in self.__robot_local])
        # Environment geoms, whose margins Environment_Distance raises for one collision pass.
        self.__env_geoms = np.array([g for g in range(m.ngeom) if m.geom_contype[g]
                                     and m.body(m.geom_bodyid[g]).name.startswith('env')], dtype=int)
        # The STL component of each environment piece's geom (Components_Within).
        self.__geom_component = {mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'env_piece_%d' % i): int(c)
                                 for i, c in enumerate(self.__piece_components)}
        self.__Set_Gripper()
        self.Set_Joints(np.zeros(6))
        if with_environment:
            self.__Check_Static_Base()

    def __Build_Spec(self, pad_span: np.ndarray, with_environment: bool, with_visual: bool) -> mujoco.MjSpec:
        """
        Description:
            Assemble the MjSpec: DH patch, robot collision hulls, coupling, gripper, exclusions,
            environment.

        Args:
            (1) pad_span [Matrix<float> 32x3]: Pad-box corners in the gripper base frame (m).
            (2 - 3) with_environment, with_visual [bool]: As in the constructor.

        Returns:
            (1) parameter [MjSpec]: The specification.
        """

        cfg = self.__cfg
        spec = mujoco.MjSpec.from_file(cfg.Robot_Model)
        bodies = {b.name: b for b in spec.bodies}
        # 1\ Placement and DH patch. The Menagerie root carries 180 deg about z, which makes its
        #    world frame the Polyscope base frame (SPEC 4.2); the base yaw is applied on top.
        base = bodies['base']
        base.pos = list(cfg.Base_Position)
        q_menagerie = np.array(base.quat, dtype=float)
        q_out = np.zeros(4)
        mujoco.mju_mulQuat(q_out, np.array(Yaw_Quat(cfg.Base_Yaw)), q_menagerie)
        base.quat = list(q_out)
        for name, offset in CONST_DH_BODY_OFFSETS:
            bodies[name].pos = list(offset)
        flange = [s for s in spec.sites if s.name == CONST_FLANGE_SITE][0]
        flange.pos = list(CONST_FLANGE_OFFSET)
        # 2\ Joint ranges from the configuration.
        joints = {j.name: j for j in spec.joints}
        for i, name in enumerate(CONST_ROBOT_JOINTS):
            joints[name].range = [cfg.Joint_Limits_Min[i], cfg.Joint_Limits_Max[i]]
        # 3\ Robot collision: the hull of every visual mesh (it contains the mesh); the Menagerie
        #    capsules are removed (they need not contain it).
        for geom in list(spec.geoms):
            if geom.type == mujoco.mjtGeom.mjGEOM_MESH:
                geom.parent.add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname=geom.meshname, pos=list(geom.pos),
                                     quat=list(geom.quat), contype=1, conaffinity=1, group=CONST_COLLISION_GROUP,
                                     margin=cfg.Clearance_Self, gap=0.0, rgba=CONST_ROBOT_COLLISION_RGBA)
                geom.contype, geom.conaffinity = 0, 0
            else:
                spec.delete(geom)
        # 4\ Gripper: coupling cylinder instead of the Menagerie disc, shifted to the measured datum,
        #    fixed joints, pad-span hull; attached at the flange with the mounting yaw.
        tool = mujoco.MjSpec.from_file(cfg.Tool_Model)
        tool_bodies = {b.name: b for b in tool.bodies}
        mount = tool_bodies['base_mount']
        for geom in list(mount.geoms):
            tool.delete(geom)
        half = cfg.Coupling_Thickness / 2.0
        for contype, group, rgba in ((0, 2, [0.15, 0.15, 0.15, 1.0]), (1, CONST_COLLISION_GROUP, CONST_ROBOT_COLLISION_RGBA)):
            mount.add_geom(name='coupling' if contype else 'coupling_visual', type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                           size=[cfg.Coupling_Diameter / 2.0, half, 0.0], pos=[0.0, 0.0, half - CONST_MENAGERIE_MOUNT_Z],
                           contype=contype, conaffinity=contype, group=group, rgba=rgba, margin=cfg.Clearance_Self)
        gripper_base = tool_bodies['base']
        shift = (cfg.Coupling_Thickness - cfg.Coupling_Spigot_Depth) - CONST_MENAGERIE_GRIPPER_DATUM
        gripper_base.pos = [gripper_base.pos[0], gripper_base.pos[1], gripper_base.pos[2] + shift]
        tool.add_mesh(name='pad_span', uservert=pad_span.flatten().tolist())
        gripper_base.add_geom(name='pad_span', type=mujoco.mjtGeom.mjGEOM_MESH, meshname='pad_span', contype=1,
                              conaffinity=1, group=CONST_COLLISION_GROUP, margin=cfg.Clearance_Self,
                              rgba=CONST_PAD_SPAN_RGBA)
        for geom in tool.geoms:
            if geom.contype != 0 or geom.conaffinity != 0:
                geom.margin, geom.gap = cfg.Clearance_Self, 0.0
                if geom.name != 'pad_span':
                    geom.rgba = CONST_ROBOT_COLLISION_RGBA
        for actuator in list(tool.actuators):
            tool.delete(actuator)
        # The gripper's contact options matter only for simulated grasping, which this kinematic
        # scene never does; aligning them with the robot's avoids MuJoCo's attach-conflict warning.
        tool.option.impratio, tool.option.cone = spec.option.impratio, spec.option.cone
        mount_frame = flange.parent.add_frame(pos=list(flange.pos), quat=list(flange.quat))
        yaw_frame = mount_frame.add_frame(quat=Yaw_Quat(cfg.Mount_Yaw))
        yaw_frame.attach_body(mount, CONST_GRIPPER_PREFIX, '')
        # 5\ Exclusions: only the configured ones (MuJoCo's parent/child filter is disabled).
        spec.option.disableflags |= mujoco.mjtDisableBit.mjDSBL_FILTERPARENT
        spec.option.disableflags |= mujoco.mjtDisableBit.mjDSBL_GRAVITY
        known = set(b.name for b in spec.bodies)
        pairs = list(cfg.Excluded_Pairs)
        for group in cfg.Rigid_Groups:
            pairs += [(a, b) for i, a in enumerate(group) for b in group[i + 1:]]
        for a, b in pairs:
            for name in (a, b):
                if name not in known:
                    raise Scene_Error_Cls('Self_Collision names the body "%s", which the scene does not have. Use one of: %s'
                                          % (name, ', '.join(sorted(known))))
            spec.add_exclude(bodyname1=a, bodyname2=b)
        for a, b, _ in cfg.Self_Pair_Overrides:
            for name in (a, b):
                if name not in known:
                    raise Scene_Error_Cls('Clearance.Self_Pair_Overrides names the body "%s", which the scene does not have.'
                                          % name)
        # 6\ Environment: static bodies env and env_stand; only base - env_stand is excluded, because
        #    the base sits 0.2 mm above the stand profiles and never moves.
        if with_environment:
            collision_set = Read_Collision_Set(cfg.Collision_Set, CONST_PROJECT_ROOT)
            # A set built before overrides existed records none.
            recorded = collision_set.Manifest.get('assembly_overrides', [])
            if recorded != Assembly_Overrides_Record(cfg.Assembly_Overrides):
                raise Scene_Error_Cls('The collision set %s was built with the assembly overrides %s, but the '
                                      'configuration has %s. Rebuild it with: .venv/bin/python Example/build_collision.py'
                                      % (cfg.Collision_Set, recorded, Assembly_Overrides_Record(cfg.Assembly_Overrides)))
            recorded = collision_set.Manifest.get('component_overrides', [])
            if recorded != Component_Overrides_Record(cfg.Component_Overrides):
                raise Scene_Error_Cls('The collision set %s was built with the component overrides %s, but the '
                                      'configuration has %s. Rebuild it with: .venv/bin/python Example/build_collision.py'
                                      % (cfg.Collision_Set, recorded, Component_Overrides_Record(cfg.Component_Overrides)))
            # Station zone: every piece whose bounding box meets the configured box.
            zone_lo, zone_hi = np.asarray(cfg.Station_Zone_Min), np.asarray(cfg.Station_Zone_Max)
            self.__zone_pieces = [i for i, v in enumerate(collision_set.Pieces)
                                  if np.all(v.max(axis=0) >= zone_lo) and np.all(v.min(axis=0) <= zone_hi)]
            env_bodies = {name: spec.worldbody.add_body(name=name) for name in ('env', 'env_stand')}
            for i, (vertices, body) in enumerate(zip(collision_set.Pieces, collision_set.Bodies)):
                spec.add_mesh(name='env_piece_%d' % i, uservert=vertices.flatten().tolist(),
                              inertia=mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL)
                env_bodies[body].add_geom(name='env_piece_%d' % i, type=mujoco.mjtGeom.mjGEOM_MESH,
                                          meshname='env_piece_%d' % i, contype=1, conaffinity=1,
                                          group=CONST_COLLISION_GROUP, margin=cfg.Clearance_Environment, gap=0.0,
                                          rgba=[0.3, 0.6, 0.9, 0.3])
            # Obstacles missing from the STL (Environment.Added_Boxes): convex meshes of their eight
            # corners in env, like the set's pieces, zone pieces by the same rule. Not box primitives:
            # MuJoCo's box-box collision misses pairs inside the margin (the gripper pads are boxes;
            # KNOWN_ISSUES, 2026-10-18), mesh pairs go through the convex collider.
            self.__zone_boxes = []
            for name, lo, hi, _ in cfg.Added_Boxes:
                lo_v, hi_v = np.asarray(lo), np.asarray(hi)
                corners = np.array([[x, y, z] for x in (lo_v[0], hi_v[0]) for y in (lo_v[1], hi_v[1]) for z in (lo_v[2], hi_v[2])])
                spec.add_mesh(name=CONST_ADDED_BOX_PREFIX + name, uservert=corners.flatten().tolist(),
                              inertia=mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL)
                env_bodies['env'].add_geom(name=CONST_ADDED_BOX_PREFIX + name, type=mujoco.mjtGeom.mjGEOM_MESH,
                                           meshname=CONST_ADDED_BOX_PREFIX + name, contype=1, conaffinity=1,
                                           group=CONST_COLLISION_GROUP, margin=cfg.Clearance_Environment, gap=0.0,
                                           rgba=[0.3, 0.6, 0.9, 0.3])
                if np.all(hi_v >= zone_lo) and np.all(lo_v <= zone_hi):
                    self.__zone_boxes.append(name)
            spec.add_exclude(bodyname1='base', bodyname2='env_stand')
            self.__collision_manifest = collision_set.Manifest
            self.__piece_components = list(collision_set.Components)
        else:
            self.__collision_manifest = {}
            self.__zone_pieces, self.__zone_boxes, self.__piece_components = [], [], []
        if with_visual:
            vertices, faces = Read_Binary_STL(cfg.Visual_Mesh)
            spec.add_mesh(name='env_visual', uservert=vertices.flatten().tolist(), userface=faces.flatten().tolist(),
                          inertia=mujoco.mjtMeshInertia.mjMESH_INERTIA_SHELL)
            spec.worldbody.add_geom(name='env_visual', type=mujoco.mjtGeom.mjGEOM_MESH, meshname='env_visual',
                                    contype=0, conaffinity=0, group=CONST_ENV_VISUAL_GROUP, rgba=[0.7, 0.7, 0.72, 1.0])
        return spec

    def __Set_Gripper(self) -> None:
        """
        Description:
            Write the solved gripper joint angles into qpos; they never change afterwards.
        """

        for j in range(self.__model.njnt):
            name = self.__model.joint(j).name
            if name.startswith(CONST_GRIPPER_PREFIX):
                self.__data.qpos[self.__model.jnt_qposadr[j]] = self.__gripper_joints[name[len(CONST_GRIPPER_PREFIX):]]

    def __Check_Static_Base(self) -> None:
        """
        Description:
            MuJoCo never collides two static bodies, so mj_collision cannot report base - env. The
            base does not move: its clearance to env is checked once here, with mj_geomDistance.
        """

        violations = self.Static_Base_Clearance()
        if violations:
            raise Scene_Error_Cls('The robot base is within %.1f mm of the environment: %s. Check Robot.Base_Position '
                                  'and Environment.Stand_Region.' % (self.__cfg.Clearance_Environment * 1e3,
                                  '; '.join('%s %.2f mm' % (v.Body_2, v.Distance * 1e3) for v in violations)))

    def Static_Base_Clearance(self) -> tp.List[Violation_Str]:
        """
        Description:
            The base's clearance to the env body (not env_stand, which it sits on), by mj_geomDistance.

        Returns:
            (1) parameter [List<Violation_Str>]: The env pieces within Clearance.Environment.
        """

        m, d = self.__model, self.__data
        mujoco.mj_kinematics(m, d)
        base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'base')
        env = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'env')
        margin = self.__cfg.Clearance_Environment
        base_geoms = [g for g in range(m.ngeom) if m.geom_bodyid[g] == base and m.geom_contype[g]]
        violations = []
        for g in range(m.ngeom):
            if m.geom_bodyid[g] != env:
                continue
            for b in base_geoms:
                gap = np.linalg.norm(d.geom_xpos[g] - d.geom_xpos[b]) - m.geom_rbound[g] - m.geom_rbound[b]
                if gap > margin:
                    continue
                distance = mujoco.mj_geomDistance(m, d, b, g, margin, None)
                if distance < margin:
                    violations.append(Violation_Str('base', m.geom(g).name, float(distance), margin))
        return violations

    @property
    def Model(self) -> mujoco.MjModel:
        """
        Description:
            The compiled model.

        Returns:
            (1) parameter [MjModel]: The model.
        """

        return self.__model

    @property
    def Data(self) -> mujoco.MjData:
        """
        Description:
            The data the scene's configurations are written to.

        Returns:
            (1) parameter [MjData]: The data.
        """

        return self.__data

    @property
    def Gripper_Gap(self) -> tp.Tuple[float, float]:
        """
        Description:
            The achieved pad-face gap and the equality residual of the opening solve.

        Returns:
            (1) parameter [Tuple<float> 1x2]: (gap (m), residual (m)).
        """

        return self.__gap, self.__residual

    @property
    def Collision_Manifest(self) -> tp.Dict:
        """
        Description:
            The manifest of the environment collision set ({} without environment).

        Returns:
            (1) parameter [Dict]: The manifest.
        """

        return self.__collision_manifest

    def Set_Joints(self, q: tp.Sequence[float]) -> None:
        """
        Description:
            Set the robot joints and update the kinematics (no dynamics).

        Args:
            (1) q [Vector<float> 1x6]: Joint angles (rad), base to wrist 3; never wrapped.
        """

        self.__data.qpos[self.__robot_qpos] = np.asarray(q, dtype=float)
        mujoco.mj_kinematics(self.__model, self.__data)

    def Flange_Pose(self) -> np.ndarray:
        """
        Description:
            The flange in the world frame, for the current joints.

        Returns:
            (1) parameter [Matrix<float> 4x4]: The pose.
        """

        T = np.eye(4)
        T[:3, :3] = self.__data.site_xmat[self.__flange].reshape(3, 3)
        T[:3, 3] = self.__data.site_xpos[self.__flange]
        return T

    def Base_Frame(self) -> np.ndarray:
        """
        Description:
            The Polyscope base frame in the world frame (configured position and yaw).

        Returns:
            (1) parameter [Matrix<float> 4x4]: The pose.
        """

        T = np.eye(4)
        c, s = math.cos(self.__cfg.Base_Yaw), math.sin(self.__cfg.Base_Yaw)
        T[:3, :3] = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        T[:3, 3] = self.__cfg.Base_Position
        return T

    def Pair_Margin(self, body_1: str, body_2: str, geom_margin: float) -> float:
        """
        Description:
            The configured margin of a body pair: the pair override if any, else the larger of the two
            geom margins (Clearance.Environment for a robot - environment pair, Clearance.Self for a
            robot - robot pair).

        Args:
            (1 - 2) body_1, body_2 [string]: The bodies.
            (3) geom_margin [float]: The contact's margin (m).

        Returns:
            (1) parameter [float]: The margin (m).
        """

        return self.__overrides.get(frozenset((body_1, body_2)), geom_margin)

    def Check_Clearance(self, q: tp.Sequence[float], station_zone: bool = False) -> tp.List[Violation_Str]:
        """
        Description:
            Every body pair closer than its configured margin at q, with the smallest distance per
            pair. MuJoCo generates a contact while the gap is below the sum of the two geom margins
            (measured, mujoco 3.16), a superset of the configured margin, so each contact is filtered
            here by the pair's configured margin. With station_zone, a contact with a station-zone
            piece is filtered by Clearance.Station_Zone.Environment instead (its geom keeps the
            global margin, so the contact is still generated).

        Args:
            (1) q [Vector<float> 1x6]: Joint angles (rad).
            (2) station_zone [bool]: Apply the zone margin to the zone pieces.

        Returns:
            (1) parameter [List<Violation_Str>]: The violations, sorted by distance (empty = clear).
        """

        m, d = self.__model, self.__data
        self.Set_Joints(q)
        mujoco.mj_collision(m, d)
        worst: tp.Dict[tp.Tuple[str, str], Violation_Str] = {}
        for i in range(d.ncon):
            contact = d.contact[i]
            b1, b2 = m.body(m.geom_bodyid[contact.geom1]).name, m.body(m.geom_bodyid[contact.geom2]).name
            margin = self.Pair_Margin(b1, b2, float(max(m.geom_margin[contact.geom1], m.geom_margin[contact.geom2])))
            if station_zone and (contact.geom1 in self.__zone_geoms or contact.geom2 in self.__zone_geoms):
                margin = self.__cfg.Station_Zone_Environment
            # MuJoCo generated this contact below the sum of the margins; only the configured one counts.
            if contact.dist >= margin:
                continue
            key = tuple(sorted((b1, b2)))
            if key not in worst or contact.dist < worst[key].Distance:
                worst[key] = Violation_Str(key[0], key[1], float(contact.dist), margin)
        return sorted(worst.values(), key=lambda v: v.Distance)

    def Tool_Length(self) -> float:
        """
        Description:
            The largest distance of a gripper collision vertex from the flange origin (bounding
            hull vertices of mesh geoms, corners of primitives).

        Returns:
            (1) parameter [float]: The length (m).
        """

        m, d = self.__model, self.__data
        flange = self.Flange_Pose()
        best = 0.0
        for g in range(m.ngeom):
            if not m.geom_contype[g] or not m.body(m.geom_bodyid[g]).name.startswith(CONST_GRIPPER_PREFIX):
                continue
            for point in self.__Geom_Points(g):
                best = max(best, float(np.linalg.norm(point - flange[:3, 3])))
        return best

    def __Geom_Points(self, g: int) -> np.ndarray:
        """
        Description:
            World points bounding a collision geom: its mesh vertices, or its primitive's box corners.

        Args:
            (1) g [int]: The geom id.

        Returns:
            (1) parameter [Matrix<float> Nx3]: The points (m).
        """

        d = self.__data
        return self.__Geom_Local(g) @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]

    def __Geom_Local(self, g: int) -> np.ndarray:
        """
        Description:
            Points bounding a collision geom in its own frame: its mesh vertices, or its primitive's
            box corners.

        Args:
            (1) g [int]: The geom id.

        Returns:
            (1) parameter [Matrix<float> Nx3]: The points (m).
        """

        m = self.__model
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = m.geom_dataid[g]
            local = m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh] + m.mesh_vertnum[mesh]]
        else:
            size = m.geom_size[g]
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_CYLINDER:
                size = np.array([size[0], size[0], size[1]])
            local = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * size
        return np.array(local, dtype=float)

    def Reach_Bound(self) -> float:
        """
        Description:
            An upper bound on the distance of any robot or gripper collision vertex from the
            shoulder point, in any configuration (triangle inequality along the chain): the sum of
            the body offsets from the shoulder to each body, plus that body's largest vertex
            radius. Used for the crop sphere (SPEC 7.1).

        Returns:
            (1) parameter [float]: The bound (m).
        """

        m = self.__model
        shoulder = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'shoulder_link')
        base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, 'base')
        best = 0.0
        for g in range(m.ngeom):
            b = int(m.geom_bodyid[g])
            if not m.geom_contype[g] or b == 0 or m.body(b).name.startswith('env'):
                continue
            # Chain length from the shoulder to body b.
            chain, node = 0.0, b
            while node not in (shoulder, base, 0):
                chain += float(np.linalg.norm(m.body_pos[node]))
                node = int(m.body_parentid[node])
            if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
                mesh = m.geom_dataid[g]
                local = m.mesh_vert[m.mesh_vertadr[mesh]:m.mesh_vertadr[mesh] + m.mesh_vertnum[mesh]]
                R = np.zeros(9)
                mujoco.mju_quat2Mat(R, m.geom_quat[g])
                radius = float(np.max(np.linalg.norm(local @ R.reshape(3, 3).T + m.geom_pos[g], axis=1)))
            else:
                radius = float(np.linalg.norm(m.geom_pos[g])) + float(m.geom_rbound[g])
            if b == base:
                # The base is fixed: its distance from the shoulder point is bounded directly.
                radius += CONST_DH_D[0]
            best = max(best, chain + radius)
        return best

    def Shoulder_Point(self) -> np.ndarray:
        """
        Description:
            The shoulder point (base frame (0, 0, d1)) in the world frame: the crop-sphere centre.

        Returns:
            (1) parameter [Vector<float> 1x3]: The point (m).
        """

        return (self.Base_Frame() @ np.array([0.0, 0.0, CONST_DH_D[0], 1.0]))[:3]

    def Crop_Radius(self) -> float:
        """
        Description:
            The crop-sphere radius: reach bound + environment clearance + CONST_CROP_PAD (SPEC 7.1).

        Returns:
            (1) parameter [float]: The radius (m).
        """

        return self.Reach_Bound() + self.__cfg.Clearance_Environment + CONST_CROP_PAD

    def Segment_Samples(self, q_a: tp.Sequence[float], q_b: tp.Sequence[float]) -> np.ndarray:
        """
        Description:
            Joint-linear samples of the segment q_a -> q_b, as movej moves, dense enough that no
            point of the robot moves more than Clearance.Environment / 4 between samples. The step
            bound is sum_i |dq_i| * lever_i, lever_i = the largest distance of any collision vertex
            downstream of joint i from that joint's origin (chain sum + radius).

        Args:
            (1 - 2) q_a, q_b [Vector<float> 1x6]: Segment ends (rad).

        Returns:
            (1) parameter [Matrix<float> Kx6]: The samples including both ends.
        """

        dq = np.abs(np.asarray(q_b, dtype=float) - np.asarray(q_a, dtype=float))
        sweep = float(np.dot(dq, self.Joint_Levers()))
        count = max(1, int(math.ceil(sweep / (self.__cfg.Clearance_Environment / 4.0))))
        t = np.linspace(0.0, 1.0, count + 1)[:, None]
        return np.asarray(q_a, dtype=float) + t * (np.asarray(q_b, dtype=float) - np.asarray(q_a, dtype=float))

    def Joint_Levers(self) -> np.ndarray:
        """
        Description:
            For each robot joint, an upper bound on the distance of any downstream collision vertex
            from the joint's body origin.

        Returns:
            (1) parameter [Vector<float> 1x6]: The levers (m).
        """

        m = self.__model
        levers = np.zeros(6)
        for i, name in enumerate(CONST_ROBOT_JOINTS):
            body = int(m.jnt_bodyid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)])
            for g in range(m.ngeom):
                b = int(m.geom_bodyid[g])
                if not m.geom_contype[g] or b == 0:
                    continue
                chain, node = 0.0, b
                while node not in (body, 0):
                    chain += float(np.linalg.norm(m.body_pos[node]))
                    node = int(m.body_parentid[node])
                if node != body:
                    continue
                levers[i] = max(levers[i], chain + float(np.linalg.norm(m.geom_pos[g])) + float(m.geom_rbound[g]))
        return levers

    @property
    def Zone_Pieces(self) -> tp.List[int]:
        """
        Description:
            The collision-set pieces whose bounding box meets the station-zone box.

        Returns:
            (1) parameter [List<int>]: The piece indices.
        """

        return list(self.__zone_pieces)

    @property
    def Zone_Boxes(self) -> tp.List[str]:
        """
        Description:
            The names of the Environment.Added_Boxes entries that meet the station-zone box.

        Returns:
            (1) parameter [List<string>]: The names.
        """

        return list(self.__zone_boxes)

    def Station_TCPs(self, station_q: tp.Sequence[tp.Sequence[float]]) -> np.ndarray:
        """
        Description:
            The world TCP positions of the fixed station points, computed once per check.

        Args:
            (1) station_q [List<Vector<float> 1x6>]: The joints of the fixed station points (rad).

        Returns:
            (1) parameter [Matrix<float> Nx3]: The positions (m).
        """

        frame = self.Base_Frame()
        return np.array([(frame @ TCP_Pose(p, self.__cfg.TCP_Offset))[:3, 3] for p in station_q]).reshape(-1, 3)

    def Workspace_Violations(self, q: tp.Sequence[float]) -> tp.List[Violation_Str]:
        """
        Description:
            Every robot body with a collision geom outside Clearance.Workspace at q: the space outside
            the STL is not modelled, and a generated segment may use only space the taught program
            proved free. The distance is minus the largest excursion, the margin 0.

        Args:
            (1) q [Vector<float> 1x6]: Joint angles (rad).

        Returns:
            (1) parameter [List<Violation_Str>]: The violations, one per body (empty = inside).
        """

        self.Set_Joints(q)
        return self.__Workspace_Current()

    def __Workspace_Current(self) -> tp.List[Violation_Str]:
        """
        Description:
            Workspace_Violations for the joints already set. A geom whose bounding sphere lies inside
            the box is inside; only the others are checked point by point.

        Returns:
            (1) parameter [List<Violation_Str>]: The violations, one per body.
        """

        m, d = self.__model, self.__data
        lo, hi = np.asarray(self.__cfg.Workspace_Min), np.asarray(self.__cfg.Workspace_Max)
        centres = d.geom_xpos[self.__robot_geoms]
        outside = np.any((centres - self.__robot_rbound[:, None] < lo) | (centres + self.__robot_rbound[:, None] > hi), axis=1)
        worst: tp.Dict[str, float] = {}
        for i in np.flatnonzero(outside):
            g = int(self.__robot_geoms[i])
            points = self.__robot_local[i] @ d.geom_xmat[g].reshape(3, 3).T + d.geom_xpos[g]
            excursion = float(np.max(np.maximum(points - hi, lo - points)))
            if excursion > 0.0:
                body = m.body(m.geom_bodyid[g]).name
                worst[body] = max(worst.get(body, 0.0), excursion)
        return [Violation_Str(body, 'workspace', -e, 0.0) for body, e in sorted(worst.items(), key=lambda x: -x[1])]

    def Components_Within(self, q: tp.Sequence[float], components: tp.Sequence[int], distance: float) -> tp.List[tp.Tuple[int, float]]:
        """
        Description:
            The given STL components that a robot or gripper geom comes closer to than distance at q,
            each with its smallest signed distance. MuJoCo generates every contact below the sum of
            the geom margins (at least Clearance.Environment), so distance may not exceed it.

        Args:
            (1) q [Vector<float> 1x6]: Joint angles (rad).
            (2) components [List<int>]: STL component ids (the collision set's piece -> component map).
            (3) distance [float]: The threshold (m), at most Clearance.Environment.

        Returns:
            (1) parameter [List<Tuple<int, float>>]: (component, distance (m)), sorted by distance.
        """

        if distance > self.__cfg.Clearance_Environment:
            raise Scene_Error_Cls('Components_Within sees contacts only below Clearance.Environment (%.3f m), not %.3f m.'
                                  % (self.__cfg.Clearance_Environment, distance))
        m, d = self.__model, self.__data
        wanted = {int(c) for c in components}
        self.Set_Joints(q)
        mujoco.mj_collision(m, d)
        found: tp.Dict[int, float] = {}
        for i in range(d.ncon):
            contact = d.contact[i]
            for env, other in ((contact.geom1, contact.geom2), (contact.geom2, contact.geom1)):
                component = self.__geom_component.get(env)
                if component in wanted and other not in self.__geom_component and contact.dist < distance:
                    found[component] = min(found.get(component, np.inf), float(contact.dist))
        return sorted(found.items(), key=lambda item: item[1])

    def Environment_Distance(self, q: tp.Sequence[float], distmax: float) -> tp.Tuple[float, str]:
        """
        Description:
            The smallest robot - environment distance at q, capped at distmax. MuJoCo generates a
            contact below the sum of the two geom margins (SPEC 9.7), so the environment margins are
            raised for one collision pass to distmax - Clearance.Self and restored; the exclusions
            apply as in Check_Clearance (base - env_stand is never reported).

        Args:
            (1) q [Vector<float> 1x6]: Joint angles (rad).
            (2) distmax [float]: The cap (m); larger than Clearance.Self.

        Returns:
            (1) parameter [Tuple<float, string>]: The distance (m), at most distmax (negative when
                                                  penetrating), and the robot body ('' at the cap).
        """

        m, d = self.__model, self.__data
        saved = m.geom_margin[self.__env_geoms].copy()
        try:
            m.geom_margin[self.__env_geoms] = distmax - self.__cfg.Clearance_Self
            self.Set_Joints(q)
            mujoco.mj_collision(m, d)
            best, body = distmax, ''
            env = set(self.__env_geoms.tolist())
            for i in range(d.ncon):
                contact = d.contact[i]
                if (contact.geom1 in env) != (contact.geom2 in env) and contact.dist < best:
                    robot = contact.geom2 if contact.geom1 in env else contact.geom1
                    best, body = float(contact.dist), m.body(m.geom_bodyid[robot]).name
        finally:
            m.geom_margin[self.__env_geoms] = saved
        return best, body

    def Check_Configuration(self, q: tp.Sequence[float], kind: str = 'generated',
                            station_tcps: tp.Optional[np.ndarray] = None,
                            box_floors: tp.Optional[tp.Dict[str, float]] = None) -> tp.List[Violation_Str]:
        """
        Description:
            The clearance rule of one configuration of a segment (SPEC 9.7). Taught: the zone margin
            at the zone pieces. Generated: the global margins, the zone margin only while the TCP
            lies within Endpoint_Radius of a station point's TCP, every robot geom inside
            Clearance.Workspace, and, with box_floors, the station-box rule: no station box closer
            than its floor (Station_Box_Floors of the generated segment's ends).

        Args:
            (1) q [Vector<float> 1x6]: Joint angles (rad).
            (2) kind [string]: 'taught' or 'generated'.
            (3) station_tcps [Matrix<float> Nx3]: The station points' TCPs (Station_TCPs); none = no
                                                  neighbourhood.
            (4) box_floors [Dict<string, float>]: The floor of each station box (m); none = no rule.

        Returns:
            (1) parameter [List<Violation_Str>]: The violations (empty = the configuration is valid).
        """

        if kind not in ('taught', 'generated'):
            raise Scene_Error_Cls('A segment is either "taught" or "generated", not %r.' % (kind,))
        if kind == 'taught':
            return self.Check_Clearance(q, station_zone=True)
        near = self.Near_Station(q, station_tcps)
        # Check_Clearance has set the joints and run the collision pass the box rule reads.
        violations = self.Check_Clearance(q, station_zone=near)
        if box_floors:
            for name, (distance, body) in self.__Station_Box_Current().items():
                if distance < box_floors[name]:
                    violations.append(Violation_Str(body, CONST_ADDED_BOX_PREFIX + name, distance, box_floors[name]))
        return violations + self.__Workspace_Current()

    def __Station_Box_Current(self) -> tp.Dict[str, tp.Tuple[float, str]]:
        """
        Description:
            The smallest robot distance to each station box in the current collision pass, capped at
            Clearance.Environment: MuJoCo reports every pair within the margin with its exact
            distance for these meshes (test/Scene/test_margin.py).

        Returns:
            (1) parameter [Dict<string, Tuple<float, string>>]: Box name -> (distance (m), robot body).
        """

        m, d = self.__model, self.__data
        found = {name: (self.__cfg.Clearance_Environment, '') for name in self.__station_box_geoms.values()}
        for i in range(d.ncon):
            contact = d.contact[i]
            for box, robot in ((contact.geom1, contact.geom2), (contact.geom2, contact.geom1)):
                name = self.__station_box_geoms.get(box)
                if name is not None and contact.dist < found[name][0]:
                    found[name] = (float(contact.dist), m.body(m.geom_bodyid[robot]).name)
        return found

    def Station_Box_Floors(self, q_a: tp.Sequence[float], q_b: tp.Sequence[float]) -> tp.Dict[str, float]:
        """
        Description:
            The station-box rule's floor of each station box for a generated segment between two
            taught ends: the closer end's distance to the box, capped at Clearance.Environment. A
            generated sample may come no closer to a station box (the geometry is fitted to the
            taught program, not measured) than the taught program does at the segment's ends.

        Args:
            (1 - 2) q_a, q_b [Vector<float> 1x6]: The taught ends (rad).

        Returns:
            (1) parameter [Dict<string, float>]: Box name -> floor (m).
        """

        floors = {}
        for q in (q_a, q_b):
            self.Set_Joints(q)
            mujoco.mj_collision(self.__model, self.__data)
            for name, (distance, _) in self.__Station_Box_Current().items():
                floors[name] = min(floors.get(name, distance), distance)
        return floors

    def Near_Station(self, q: tp.Sequence[float], station_tcps: tp.Optional[np.ndarray]) -> bool:
        """
        Description:
            Whether the TCP at q lies within Clearance.Station_Zone.Endpoint_Radius of a station
            point's TCP.

        Args:
            (1) q [Vector<float> 1x6]: Joint angles (rad).
            (2) station_tcps [Matrix<float> Nx3]: The station points' TCPs (Station_TCPs); none = no
                                                  neighbourhood.

        Returns:
            (1) parameter [bool]: True inside the neighbourhood.
        """

        if station_tcps is None or not len(station_tcps):
            return False
        tcp = (self.Base_Frame() @ TCP_Pose(q, self.__cfg.TCP_Offset))[:3, 3]
        return bool(np.min(np.linalg.norm(station_tcps - tcp, axis=1)) <= self.__cfg.Station_Zone_Endpoint_Radius)

    def Station_Zone_Active(self, q: tp.Sequence[float], station_q: tp.Sequence[tp.Sequence[float]]) -> bool:
        """
        Description:
            Whether a generated configuration may use the zone margin: its TCP lies within
            Clearance.Station_Zone.Endpoint_Radius of the TCP of a fixed station point (SPEC 9.7).

        Args:
            (1) q [Vector<float> 1x6]: Joint angles (rad).
            (2) station_q [List<Vector<float> 1x6>]: The joints of the fixed station points (rad).

        Returns:
            (1) parameter [bool]: True inside the neighbourhood of a station point.
        """

        frame = self.Base_Frame()
        tcp = (frame @ TCP_Pose(q, self.__cfg.TCP_Offset))[:3, 3]
        return any(np.linalg.norm(tcp - (frame @ TCP_Pose(p, self.__cfg.TCP_Offset))[:3, 3])
                   <= self.__cfg.Station_Zone_Endpoint_Radius for p in station_q)

    def Check_Segment(self, q_a: tp.Sequence[float], q_b: tp.Sequence[float], kind: str = 'generated',
                      station_q: tp.Sequence[tp.Sequence[float]] = (),
                      box_floors: tp.Optional[tp.Dict[str, float]] = None) -> tp.Tuple[int, tp.List[tp.Tuple[int, Violation_Str]]]:
        """
        Description:
            Check every sample of a joint-linear segment. A taught segment (from the robot program)
            uses the zone margin for the zone pieces on every sample. A generated segment keeps the
            global margins, except on samples whose TCP lies within the endpoint radius of a fixed
            station point: the station geometry is fitted, not measured, and a new path must not use
            it more tightly than the taught program does.

        Args:
            (1 - 2) q_a, q_b [Vector<float> 1x6]: Segment ends (rad).
            (3) kind [string]: 'taught' or 'generated'.
            (4) station_q [List<Vector<float> 1x6>]: The joints of the fixed station points (rad);
                                                      used by a generated segment only.
            (5) box_floors [Dict<string, float>]: The station-box floors (Station_Box_Floors of the
                                                  generated path's ends); none = no station-box rule.

        Returns:
            (1) parameter [Tuple<int, List<Tuple<int, Violation_Str>>>]: The number of samples and
                                                                          every (sample, violation).
        """

        if kind not in ('taught', 'generated'):
            raise Scene_Error_Cls('A segment is either "taught" or "generated", not %r.' % (kind,))
        samples = self.Segment_Samples(q_a, q_b)
        tcps = self.Station_TCPs(station_q)
        found = []
        for k, q in enumerate(samples):
            found += [(k, v) for v in self.Check_Configuration(q, kind, tcps, box_floors)]
        return len(samples), found

    def Segment_Valid(self, q_a: tp.Sequence[float], q_b: tp.Sequence[float], kind: str = 'generated',
                      station_q: tp.Sequence[tp.Sequence[float]] = (), box_floors: tp.Optional[tp.Dict[str, float]] = None) -> bool:
        """
        Description:
            Whether Check_Segment would find no violation, stopping at the first violating sample.
            The same samples and the same rule as Check_Segment, visited in van der Corput order
            (ends, middle, quarters, ...) so that a colliding segment is usually rejected early; the
            verdict is identical by construction and pinned by a test.

        Args:
            (1 - 2) q_a, q_b [Vector<float> 1x6]: Segment ends (rad).
            (3) kind [string]: 'taught' or 'generated'.
            (4) station_q [List<Vector<float> 1x6>]: The joints of the fixed station points (rad).
            (5) box_floors [Dict<string, float>]: The station-box floors; none = no station-box rule.

        Returns:
            (1) parameter [bool]: True when every sample is valid.
        """

        if kind not in ('taught', 'generated'):
            raise Scene_Error_Cls('A segment is either "taught" or "generated", not %r.' % (kind,))
        samples = self.Segment_Samples(q_a, q_b)
        tcps = self.Station_TCPs(station_q)
        for k in Van_Der_Corput_Order(len(samples)):
            if self.Check_Configuration(samples[k], kind, tcps, box_floors):
                return False
        return True

    def Minimum_Distance(self, body_1: str, body_2: str, q: tp.Sequence[float], distmax: float = 1.0) -> float:
        """
        Description:
            The smallest signed distance between two bodies' collision geoms at q (mj_geomDistance;
            capped at distmax).

        Args:
            (1 - 2) body_1, body_2 [string]: The bodies.
            (3) q [Vector<float> 1x6]: Joint angles (rad).
            (4) distmax [float]: The cap (m).

        Returns:
            (1) parameter [float]: The distance (m).
        """

        m, d = self.__model, self.__data
        self.Set_Joints(q)
        a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_1)
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body_2)
        best = distmax
        for g1 in range(m.ngeom):
            if m.geom_bodyid[g1] != a or not m.geom_contype[g1]:
                continue
            for g2 in range(m.ngeom):
                if m.geom_bodyid[g2] != b or not m.geom_contype[g2]:
                    continue
                best = min(best, mujoco.mj_geomDistance(m, d, g1, g2, best, None))
        return best


def Check_Learned_Point(scene: Scene_Cls, cfg: Cell_Configuration_Str, point: Robot_Point_Str) -> tp.Tuple[float, float]:
    """
    Description:
        Accept a learned point only if its _q and _p agree within the sanity bound and it keeps
        every clearance margin, the zone margin for the station-zone pieces (a learned point is
        taught). Otherwise raise, naming everything found (SPEC 9.4, 9.7).

    Args:
        (1) scene [Scene_Cls(object)]: The scene with the environment.
        (2) cfg [Cell_Configuration_Str(object)]: The configuration.
        (3) point [Robot_Point_Str(object)]: The point.

    Returns:
        (1) parameter [Tuple<float> 1x2]: Its kinematic residual (m, rad).
    """

    scene.Set_Joints(np.zeros(6))
    orientation_bound = Orientation_Sanity_Bound(cfg.FK_Sanity_Bound_Position, scene.Tool_Length())
    residual = Check_FK_Consistency(point.Name, point.Q, point.P, cfg.TCP_Offset, cfg.FK_Sanity_Bound_Position,
                                    orientation_bound)
    # A learned point is taught: the zone margin applies to the zone pieces.
    violations = scene.Check_Clearance(point.Q, station_zone=True)
    if violations:
        raise Clearance_Error_Cls('Point %s violates %d clearance margin(s): %s. Learned points are never accepted '
                                  'inside a margin; re-teach the point or review the cell model.'
                                  % (point.Name, len(violations), '; '.join(
                                      '%s - %s %.2f mm (margin %.1f mm)' % (v.Body_1, v.Body_2, v.Distance * 1e3,
                                                                            v.Margin * 1e3) for v in violations)))
    return residual
