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
# File Name: ../Configuration/Core.py
## =========================================================================== ##

# System (Default)
import os
# Math (Mathematical functions)
import math
# Re (Regular expressions)
import re
# Typing (Support for type hints)
import typing as tp
# Dataclasses (Data Classes)
from dataclasses import dataclass, field
# PyYAML (YAML parser and emitter) [pip3 install pyyaml]
import yaml

"""
Description:
    Initialization of constants.

    The only reader of the cell configuration (Configuration/Cell_SF1.yaml). Every field is
    REQUIRED; every problem is collected and raised once as Configuration_Error_Cls. Angles in the
    file are degrees, and are converted to radians here so that no other module converts units.
"""
# The repository root (two levels above this file: src/Configuration/).
#   Unit [string]
CONST_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
# The shipped cell configuration.
#   Unit [string]
CONST_DEFAULT_CONFIG_FILE = os.path.join(CONST_PROJECT_ROOT, 'Configuration', 'Cell_SF1.yaml')
# Hardware joint range of the UR10e (every joint).
#   Unit [deg]
CONST_HARDWARE_LIMIT = 360.0
# The elbow (joint index 2) must stay within +-180 deg: that limit is the precondition for excluding
# upper_arm_link - forearm_link from self-collision (SPEC 9.6). Beyond it the forearm folds onto
# the upper arm and the excluded pair would hide the collision.
#   Unit [deg]
CONST_ELBOW_LIMIT = 180.0
# The index of the elbow joint in a joint vector.
#   Unit [int]
CONST_ELBOW_INDEX = 2
# The largest translation one assembly or component override may apply: corrections are bounded
# (owner, 2026-10-17), so a large one is a configuration error, not a fit. Each entry is bounded on
# its own: an assembly shift and a component fit on top of it are separate claims (owner, 2026-10-18).
#   Unit [float] (m)
CONST_OVERRIDE_TRANSLATION_MAX = 0.150
# The largest inner pad-face gap of the Robotiq 2F-85: its 85 mm stroke (Instruction Manual
# rev. 2020/02/11, section 1). A holding opening above it cannot be configured.
#   Unit [float] (m)
CONST_GRIPPER_STROKE = 0.085
# The known keys, as nested dictionaries; a leaf is None. Used to report unknown keys.
#   Unit [Dict]
CONST_SCHEMA = {
    'Environment': {'Visual_Mesh': None, 'Collision_Set': None, 'Stand_Region': {'Min': None, 'Max': None},
                    'Assembly_Overrides': None, 'Component_Overrides': None, 'Added_Boxes': None},
    'Robot': {'Model': None, 'Base_Position': None, 'Base_Yaw': None, 'Joint_Position_Limits': {'Min': None, 'Max': None}},
    'Tool': {'Model': None, 'Coupling': {'Thickness': None, 'Diameter': None, 'Spigot_Depth': None},
             'Mount_Yaw': None, 'Opening': None, 'Holding_Openings': None, 'TCP_Offset': None},
    'Clearance': {'Environment': None, 'Self': None, 'Self_Pair_Overrides': None,
                  'Station_Zone': {'Min': None, 'Max': None, 'Environment': None, 'Endpoint_Radius': None},
                  'Machine_Extra': {'Assembly': None, 'Margin': None, 'Source': None},
                  'Workspace': {'Min': None, 'Max': None}},
    'Self_Collision': {'Excluded_Pairs': None, 'Rigid_Groups': None},
    'Kinematics': {'FK_Sanity_Bound': {'Position': None}},
}


class Configuration_Error_Cls(ValueError):
    """
    Description:
        The cell configuration cannot be used. The message lists every problem found, each as a
        sentence naming the field and what to change.
    """


@dataclass
class Cell_Configuration_Str:
    """
    Description:
        The loaded cell configuration, in SI units (metres, radians). Paths are absolute.
    """

    # The environment visual mesh (STL).
    #   Unit [string]
    Visual_Mesh: str = ''
    # The environment collision set (.npz).
    #   Unit [string]
    Collision_Set: str = ''
    # Lower and upper corner of the stand-selection box, world frame.
    #   Unit [Vector<float> 1x3] (m)
    Stand_Region_Min: tp.List[float] = field(default_factory=list)
    Stand_Region_Max: tp.List[float] = field(default_factory=list)
    # Rigid translations of whole STL objects (SPEC 4.1) before the collision set is built
    # (name, a component of the object, translation, source).
    #   Unit [List<Tuple<string, int, List<float> 1x3, string>>] (-, -, m, -)
    Assembly_Overrides: tp.List[tp.Tuple[str, int, tp.List[float], str]] = field(default_factory=list)
    # Rigid translations applied to STL components before the collision set is built, added to any
    # assembly translation (component, translation, source).
    #   Unit [List<Tuple<int, List<float> 1x3, string>>] (-, m, -)
    Component_Overrides: tp.List[tp.Tuple[int, tp.List[float], str]] = field(default_factory=list)
    # Obstacles missing from the STL, added to the environment as world-axis-aligned boxes
    # (name, lower corner, upper corner, source).
    #   Unit [List<Tuple<string, List<float> 1x3, List<float> 1x3, string>>] (-, m, m, -)
    Added_Boxes: tp.List[tp.Tuple[str, tp.List[float], tp.List[float], str]] = field(default_factory=list)
    # The robot MJCF.
    #   Unit [string]
    Robot_Model: str = ''
    # Origin of the Polyscope base frame in the world frame.
    #   Unit [Vector<float> 1x3] (m)
    Base_Position: tp.List[float] = field(default_factory=list)
    # Rotation of the Polyscope base frame about world z.
    #   Unit [float] (rad)
    Base_Yaw: float = 0.0
    # Joint position limits, base to wrist 3.
    #   Unit [Vector<float> 1x6] (rad)
    Joint_Limits_Min: tp.List[float] = field(default_factory=list)
    Joint_Limits_Max: tp.List[float] = field(default_factory=list)
    # The gripper MJCF.
    #   Unit [string]
    Tool_Model: str = ''
    # Coupling thickness, outer diameter and the depth of the bore receiving the gripper.
    #   Unit [float] (m)
    Coupling_Thickness: float = 0.0
    Coupling_Diameter: float = 0.0
    Coupling_Spigot_Depth: float = 0.0
    # Rotation of the gripper about flange z relative to the Menagerie default.
    #   Unit [float] (rad)
    Mount_Yaw: float = 0.0
    # Inner pad-face gap of a scene that names no case (the demo, the tests).
    #   Unit [float] (m)
    Opening: float = 0.0
    # Inner pad-face gap while a case's part is held, per case of the robot program (opening, source).
    #   Unit [Dict<int, Tuple<float, string>>] (-, m, -)
    Holding_Openings: tp.Dict[int, tp.Tuple[float, str]] = field(default_factory=dict)
    # TCP in the flange frame [x, y, z, rx, ry, rz].
    #   Unit [Vector<float> 1x6] (m, rad)
    TCP_Offset: tp.List[float] = field(default_factory=list)
    # Minimum robot/gripper to environment distance.
    #   Unit [float] (m)
    Clearance_Environment: float = 0.0
    # Minimum distance between non-excluded robot/gripper bodies.
    #   Unit [float] (m)
    Clearance_Self: float = 0.0
    # Pair-specific self margins (body, body, margin).
    #   Unit [List<Tuple<string, string, float>>] (m)
    Self_Pair_Overrides: tp.List[tp.Tuple[str, str, float]] = field(default_factory=list)
    # Station zone: lower and upper corner of the box, world frame; environment pieces whose
    # bounding box meets it get the zone margin where the zone applies.
    #   Unit [Vector<float> 1x3] (m)
    Station_Zone_Min: tp.List[float] = field(default_factory=list)
    Station_Zone_Max: tp.List[float] = field(default_factory=list)
    # The environment margin of the zone pieces where the zone applies.
    #   Unit [float] (m)
    Station_Zone_Environment: float = 0.0
    # A generated sample gets the zone margin only while its TCP lies within this distance of a
    # fixed station point.
    #   Unit [float] (m)
    Station_Zone_Endpoint_Radius: float = 0.0
    # The extra environment margin of a generated segment to one assembly's pieces outside the
    # station zone (the accuracy of its estimated position): the assembly's name, the margin and
    # its source.
    #   Unit [string], [float] (m), [string]
    Machine_Extra_Assembly: str = ''
    Machine_Extra_Margin: float = 0.0
    Machine_Extra_Source: str = ''
    # Lower and upper corner of the box a generated segment must keep every robot collision geom
    # inside, world frame.
    #   Unit [Vector<float> 1x3] (m)
    Workspace_Min: tp.List[float] = field(default_factory=list)
    Workspace_Max: tp.List[float] = field(default_factory=list)
    # Body pairs never checked.
    #   Unit [List<Tuple<string, string>>]
    Excluded_Pairs: tp.List[tp.Tuple[str, str]] = field(default_factory=list)
    # Groups of mutually rigid bodies; every pair inside a group is excluded.
    #   Unit [List<List<string>>]
    Rigid_Groups: tp.List[tp.List[str]] = field(default_factory=list)
    # Sanity bound on |DH FK * TCP - _p| of a learned point.
    #   Unit [float] (m)
    FK_Sanity_Bound_Position: float = 0.0
    # (level, message) notes from the load, e.g. unknown keys.
    #   Unit [List<Tuple<string, string>>]
    Notes: tp.List[tp.Tuple[str, str]] = field(default_factory=list)


def Unknown_Keys(data: tp.Any, schema: tp.Dict, prefix: str) -> tp.List[str]:
    """
    Description:
        List the keys of a mapping that the schema does not know, as dotted paths.

    Args:
        (1) data [Any]: The (sub)mapping read from the file.
        (2) schema [Dict]: The matching part of CONST_SCHEMA.
        (3) prefix [string]: The dotted path of data.

    Returns:
        (1) parameter [List<string>]: Dotted paths of unknown keys.
    """

    unknown = []
    if not isinstance(data, dict):
        return unknown
    for key, value in data.items():
        path = prefix + str(key)
        if key not in schema:
            unknown.append(path)
        elif isinstance(schema[key], dict):
            unknown += Unknown_Keys(value, schema[key], path + '.')
    return unknown


def Valid_Component_Override(item: tp.Any) -> bool:
    """
    Description:
        Whether one entry of Environment.Component_Overrides has exactly the keys Component (an id
        >= 0), Translation (three finite numbers) and Source (non-empty text).

    Args:
        (1) item [Any]: The entry as read from YAML.

    Returns:
        (1) parameter [bool]: True when the entry is well formed.
    """

    return (isinstance(item, dict) and set(item) == {'Component', 'Translation', 'Source'}
            and isinstance(item['Component'], int) and not isinstance(item['Component'], bool) and item['Component'] >= 0
            and isinstance(item['Translation'], list) and len(item['Translation']) == 3
            and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in item['Translation'])
            and isinstance(item['Source'], str) and item['Source'].strip() != '')


def Valid_Point(value: tp.Any) -> bool:
    """
    Description:
        Whether a value read from YAML is a list of three finite numbers (booleans excluded).

    Args:
        (1) value [Any]: The value.

    Returns:
        (1) parameter [bool]: True for [x, y, z].
    """

    return (isinstance(value, list) and len(value) == 3
            and all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in value))


def Valid_Assembly_Override(item: tp.Any) -> bool:
    """
    Description:
        Whether one entry of Environment.Assembly_Overrides has exactly the keys Name (letters, digits
        and underscores), Object_Of (a component id >= 0), Translation (three finite numbers) and
        Source (non-empty text).

    Args:
        (1) item [Any]: The entry as read from YAML.

    Returns:
        (1) parameter [bool]: True when the entry is well formed.
    """

    return (isinstance(item, dict) and set(item) == {'Name', 'Object_Of', 'Translation', 'Source'}
            and isinstance(item['Name'], str) and re.fullmatch(r'[A-Za-z0-9_]+', item['Name']) is not None
            and isinstance(item['Object_Of'], int) and not isinstance(item['Object_Of'], bool) and item['Object_Of'] >= 0
            and Valid_Point(item['Translation'])
            and isinstance(item['Source'], str) and item['Source'].strip() != '')


def Valid_Holding_Opening(item: tp.Any) -> bool:
    """
    Description:
        Whether one entry of Tool.Holding_Openings has exactly the keys Case (a case number >= 1),
        Opening (a finite number) and Source (non-empty text).

    Args:
        (1) item [Any]: The entry as read from YAML.

    Returns:
        (1) parameter [bool]: True when the entry is well formed.
    """

    return (isinstance(item, dict) and set(item) == {'Case', 'Opening', 'Source'}
            and isinstance(item['Case'], int) and not isinstance(item['Case'], bool) and item['Case'] >= 1
            and isinstance(item['Opening'], (int, float)) and not isinstance(item['Opening'], bool)
            and math.isfinite(item['Opening'])
            and isinstance(item['Source'], str) and item['Source'].strip() != '')


def Valid_Added_Box(item: tp.Any) -> bool:
    """
    Description:
        Whether one entry of Environment.Added_Boxes has exactly the keys Name (letters, digits and
        underscores), Min and Max (three finite numbers each) and Source (non-empty text).

    Args:
        (1) item [Any]: The entry as read from YAML.

    Returns:
        (1) parameter [bool]: True when the entry is well formed.
    """

    return (isinstance(item, dict) and set(item) == {'Name', 'Min', 'Max', 'Source'}
            and isinstance(item['Name'], str) and re.fullmatch(r'[A-Za-z0-9_]+', item['Name']) is not None
            and Valid_Point(item['Min']) and Valid_Point(item['Max'])
            and isinstance(item['Source'], str) and item['Source'].strip() != '')


class Field_Reader_Cls(object):
    """
    Description:
        Reads typed values from the loaded mapping, collecting every problem instead of stopping
        at the first.

    Initialization of the Class:
        Args:
            (1) data [Dict]: The mapping loaded from the YAML file.

        Example:
            Initialization:
                # Initialization of the class.
                Cls = Field_Reader_Cls(data)

            Features:
                # Properties of the class.
                Cls.Problems

                # Functions of the class.
                Cls.Number('Tool.Opening', positive=True)
    """

    def __init__(self, data: tp.Dict) -> None:
        # << PRIVATE >> #
        self.__data = data
        self.__problems: tp.List[str] = []

    @property
    def Problems(self) -> tp.List[str]:
        """
        Description:
            Every problem found so far.

        Returns:
            (1) parameter [List<string>]: One sentence per problem.
        """

        return self.__problems

    def Fail(self, message: str) -> None:
        """
        Description:
            Record a problem.

        Args:
            (1) message [string]: The sentence naming the field and what to change.
        """

        self.__problems.append(message)

    def Raw(self, path: str) -> tp.Any:
        """
        Description:
            The value at a dotted path, or None (with a problem recorded) if it is missing.

        Args:
            (1) path [string]: The dotted key path, e.g. 'Tool.Coupling.Thickness'.

        Returns:
            (1) parameter [Any]: The value, or None.
        """

        node = self.__data
        for key in path.split('.'):
            if not isinstance(node, dict) or key not in node or node[key] is None:
                self.Fail('%s is missing; it is REQUIRED. Add it to the configuration file.' % path)
                return None
            node = node[key]
        return node

    def Number(self, path: str, positive: bool = False, non_negative: bool = False) -> float:
        """
        Description:
            A real number, optionally > 0 or >= 0.

        Args:
            (1) path [string]: The dotted key path.
            (2) positive [bool]: Require > 0.
            (3) non_negative [bool]: Require >= 0.

        Returns:
            (1) parameter [float]: The value (0.0 when it is missing or invalid; a problem is recorded).
        """

        value = self.Raw(path)
        if value is None:
            return 0.0
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            self.Fail('%s must be a finite number, not %r.' % (path, value))
            return 0.0
        if positive and value <= 0.0:
            self.Fail('%s must be greater than 0, not %r.' % (path, value))
        if non_negative and value < 0.0:
            self.Fail('%s must not be negative, not %r.' % (path, value))
        return float(value)

    def Vector(self, path: str, length: int) -> tp.List[float]:
        """
        Description:
            A list of finite numbers of a given length.

        Args:
            (1) path [string]: The dotted key path.
            (2) length [int]: The required length.

        Returns:
            (1) parameter [List<float>]: The values (zeros when missing or invalid; a problem is recorded).
        """

        value = self.Raw(path)
        if value is None:
            return [0.0] * length
        if (not isinstance(value, list) or len(value) != length
                or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in value)):
            self.Fail('%s must be a list of %d finite numbers, not %r.' % (path, length, value))
            return [0.0] * length
        return [float(v) for v in value]

    def Path(self, path: str, must_exist: bool = True) -> str:
        """
        Description:
            A repository-relative path, returned absolute. With must_exist, the file must exist.

        Args:
            (1) path [string]: The dotted key path.
            (2) must_exist [bool]: Require the file to exist now. The collision set does not: it is
                                   built from this configuration, and its reader reports it missing.

        Returns:
            (1) parameter [string]: The absolute path ('' when missing; a problem is recorded).
        """

        value = self.Raw(path)
        if value is None:
            return ''
        if not isinstance(value, str) or os.path.isabs(value):
            self.Fail('%s must be a path relative to the repository root, not %r.' % (path, value))
            return ''
        full = os.path.join(CONST_PROJECT_ROOT, value)
        if must_exist and not os.path.isfile(full):
            self.Fail('%s names %s, which does not exist. Fix the path or create the file.' % (path, value))
        return full

    def Names(self, path: str, size: tp.Optional[int]) -> tp.List[tp.List[str]]:
        """
        Description:
            A list of lists of body names; each inner list has the given size (any size >= 2 if None).

        Args:
            (1) path [string]: The dotted key path.
            (2) size [int]: The required inner length, or None.

        Returns:
            (1) parameter [List<List<string>>]: The names ([] when invalid; a problem is recorded).
        """

        value = self.Raw(path)
        if value is None:
            return []
        ok = isinstance(value, list) and all(
            isinstance(item, list) and all(isinstance(n, str) and n for n in item)
            and (len(item) == size if size is not None else len(item) >= 2) for item in value)
        if not ok:
            self.Fail('%s must be a list of %s of body names, not %r.'
                      % (path, 'pairs' if size == 2 else 'groups (at least two names each)', value))
            return []
        return [list(item) for item in value]


def Load_Configuration(config_path: str = CONST_DEFAULT_CONFIG_FILE) -> Cell_Configuration_Str:
    """
    Description:
        Load and validate the cell configuration. Every problem is collected, then the load fails
        once with all of them. An unknown key is a WARNING note, not a failure.

    Args:
        (1) config_path [string]: The YAML file.

    Returns:
        (1) parameter [Cell_Configuration_Str(object)]: The configuration in SI units.
    """

    # 1\ Read the file.
    try:
        with open(config_path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
    except OSError as error:
        raise Configuration_Error_Cls('The configuration file "%s" cannot be read (%s). Check the path.'
                                      % (config_path, error)) from error
    except yaml.YAMLError as error:
        raise Configuration_Error_Cls('The configuration file "%s" is not valid YAML: %s' % (config_path, error)) from error
    if not isinstance(data, dict):
        raise Configuration_Error_Cls('The configuration file "%s" must be a mapping of sections.' % config_path)

    # 2\ Read every field, collecting problems.
    reader = Field_Reader_Cls(data)
    cfg = Cell_Configuration_Str()
    cfg.Visual_Mesh = reader.Path('Environment.Visual_Mesh')
    cfg.Collision_Set = reader.Path('Environment.Collision_Set', must_exist=False)
    cfg.Stand_Region_Min = reader.Vector('Environment.Stand_Region.Min', 3)
    cfg.Stand_Region_Max = reader.Vector('Environment.Stand_Region.Max', 3)
    if any(lo >= hi for lo, hi in zip(cfg.Stand_Region_Min, cfg.Stand_Region_Max)):
        reader.Fail('Environment.Stand_Region.Min must be below Max on every axis.')
    assembly_overrides = reader.Raw('Environment.Assembly_Overrides')
    if assembly_overrides is not None:
        if not isinstance(assembly_overrides, list) or not all(Valid_Assembly_Override(a) for a in assembly_overrides):
            reader.Fail('Environment.Assembly_Overrides must be a list of {Name: letters, digits, _, Object_Of: component '
                        'id >= 0, Translation: [x, y, z], Source: text}, not %r.' % (assembly_overrides,))
        else:
            for a in assembly_overrides:
                if math.sqrt(sum(v * v for v in a['Translation'])) > CONST_OVERRIDE_TRANSLATION_MAX:
                    reader.Fail('Environment.Assembly_Overrides: the assembly %s moves by more than %.0f mm; a correction '
                                'is bounded.' % (a['Name'], CONST_OVERRIDE_TRANSLATION_MAX * 1e3))
            names = [a['Name'] for a in assembly_overrides]
            if len(set(names)) != len(names):
                reader.Fail('Environment.Assembly_Overrides names an assembly more than once: %r.' % (names,))
            cfg.Assembly_Overrides = [(a['Name'], a['Object_Of'], [float(v) for v in a['Translation']], a['Source'])
                                      for a in assembly_overrides]
    component_overrides = reader.Raw('Environment.Component_Overrides')
    if component_overrides is not None:
        if not isinstance(component_overrides, list) or not all(Valid_Component_Override(o) for o in component_overrides):
            reader.Fail('Environment.Component_Overrides must be a list of {Component: id >= 0, Translation: [x, y, z], '
                        'Source: text}, not %r.' % (component_overrides,))
        else:
            for o in component_overrides:
                if math.sqrt(sum(v * v for v in o['Translation'])) > CONST_OVERRIDE_TRANSLATION_MAX:
                    reader.Fail('Environment.Component_Overrides: component %d moves by more than %.0f mm; a correction '
                                'derived from the taught program is bounded.' % (o['Component'], CONST_OVERRIDE_TRANSLATION_MAX * 1e3))
            ids = [o['Component'] for o in component_overrides]
            if len(set(ids)) != len(ids):
                reader.Fail('Environment.Component_Overrides names a component more than once: %r.' % (ids,))
            cfg.Component_Overrides = [(o['Component'], [float(v) for v in o['Translation']], o['Source'])
                                       for o in component_overrides]
    added_boxes = reader.Raw('Environment.Added_Boxes')
    if added_boxes is not None:
        if not isinstance(added_boxes, list) or not all(Valid_Added_Box(b) for b in added_boxes):
            reader.Fail('Environment.Added_Boxes must be a list of {Name: letters, digits, _, Min: [x, y, z], '
                        'Max: [x, y, z], Source: text}, not %r.' % (added_boxes,))
        else:
            for b in added_boxes:
                if any(lo >= hi for lo, hi in zip(b['Min'], b['Max'])):
                    reader.Fail('Environment.Added_Boxes: the box %s has Min %r not below Max %r on every axis.'
                                % (b['Name'], b['Min'], b['Max']))
            names = [b['Name'] for b in added_boxes]
            if len(set(names)) != len(names):
                reader.Fail('Environment.Added_Boxes names a box more than once: %r.' % (names,))
            cfg.Added_Boxes = [(b['Name'], [float(v) for v in b['Min']], [float(v) for v in b['Max']], b['Source'])
                               for b in added_boxes]
    cfg.Robot_Model = reader.Path('Robot.Model')
    cfg.Base_Position = reader.Vector('Robot.Base_Position', 3)
    cfg.Base_Yaw = math.radians(reader.Number('Robot.Base_Yaw'))
    limits_min = reader.Vector('Robot.Joint_Position_Limits.Min', 6)
    limits_max = reader.Vector('Robot.Joint_Position_Limits.Max', 6)
    for i, (lo, hi) in enumerate(zip(limits_min, limits_max)):
        bound = CONST_ELBOW_LIMIT if i == CONST_ELBOW_INDEX else CONST_HARDWARE_LIMIT
        if lo >= hi:
            reader.Fail('Robot.Joint_Position_Limits: joint %d has Min %r not below Max %r.' % (i, lo, hi))
        if lo < -bound or hi > bound:
            if i == CONST_ELBOW_INDEX:
                reader.Fail('Robot.Joint_Position_Limits: the elbow (joint 2) must stay within +-%.0f deg, not [%r, %r]. '
                            'That limit is the precondition for excluding upper_arm_link - forearm_link from '
                            'self-collision.' % (bound, lo, hi))
            else:
                reader.Fail('Robot.Joint_Position_Limits: joint %d must stay within the hardware range +-%.0f deg, '
                            'not [%r, %r].' % (i, bound, lo, hi))
    cfg.Joint_Limits_Min = [math.radians(v) for v in limits_min]
    cfg.Joint_Limits_Max = [math.radians(v) for v in limits_max]
    cfg.Tool_Model = reader.Path('Tool.Model')
    cfg.Coupling_Thickness = reader.Number('Tool.Coupling.Thickness', positive=True)
    cfg.Coupling_Diameter = reader.Number('Tool.Coupling.Diameter', positive=True)
    cfg.Coupling_Spigot_Depth = reader.Number('Tool.Coupling.Spigot_Depth', non_negative=True)
    if cfg.Coupling_Spigot_Depth >= cfg.Coupling_Thickness > 0.0:
        reader.Fail('Tool.Coupling.Spigot_Depth (%r) must be smaller than Tool.Coupling.Thickness (%r).'
                    % (cfg.Coupling_Spigot_Depth, cfg.Coupling_Thickness))
    cfg.Mount_Yaw = math.radians(reader.Number('Tool.Mount_Yaw'))
    cfg.Opening = reader.Number('Tool.Opening', positive=True)
    holding = reader.Raw('Tool.Holding_Openings')
    if holding is not None:
        if not isinstance(holding, list) or not holding or not all(Valid_Holding_Opening(h) for h in holding):
            reader.Fail('Tool.Holding_Openings must be a non-empty list of {Case: case number >= 1, Opening: m, '
                        'Source: text}, not %r.' % (holding,))
        else:
            for h in holding:
                if not 0.0 < h['Opening'] <= CONST_GRIPPER_STROKE:
                    reader.Fail('Tool.Holding_Openings: case %d holds at %r m; the opening must be greater than 0 and '
                                'at most the 2F-85 stroke of %.3f m.' % (h['Case'], h['Opening'], CONST_GRIPPER_STROKE))
            cases = [h['Case'] for h in holding]
            if len(set(cases)) != len(cases):
                reader.Fail('Tool.Holding_Openings names a case more than once: %r.' % (cases,))
            cfg.Holding_Openings = {h['Case']: (float(h['Opening']), h['Source']) for h in holding}
    cfg.TCP_Offset = reader.Vector('Tool.TCP_Offset', 6)
    cfg.Clearance_Environment = reader.Number('Clearance.Environment', positive=True)
    cfg.Clearance_Self = reader.Number('Clearance.Self', positive=True)
    overrides = reader.Raw('Clearance.Self_Pair_Overrides')
    if overrides is not None:
        if isinstance(overrides, list) and all(
                isinstance(o, list) and len(o) == 3 and isinstance(o[0], str) and isinstance(o[1], str)
                and isinstance(o[2], (int, float)) and not isinstance(o[2], bool) and 0.0 <= o[2] for o in overrides):
            cfg.Self_Pair_Overrides = [(o[0], o[1], float(o[2])) for o in overrides]
        else:
            reader.Fail('Clearance.Self_Pair_Overrides must be a list of [body, body, margin >= 0], not %r.' % (overrides,))
    cfg.Station_Zone_Min = reader.Vector('Clearance.Station_Zone.Min', 3)
    cfg.Station_Zone_Max = reader.Vector('Clearance.Station_Zone.Max', 3)
    if any(lo >= hi for lo, hi in zip(cfg.Station_Zone_Min, cfg.Station_Zone_Max)):
        reader.Fail('Clearance.Station_Zone.Min must be below Max on every axis.')
    cfg.Station_Zone_Environment = reader.Number('Clearance.Station_Zone.Environment', positive=True)
    if cfg.Station_Zone_Environment > cfg.Clearance_Environment > 0.0:
        reader.Fail('Clearance.Station_Zone.Environment (%r) must not exceed Clearance.Environment (%r): the zone only '
                    'reduces the margin.' % (cfg.Station_Zone_Environment, cfg.Clearance_Environment))
    cfg.Station_Zone_Endpoint_Radius = reader.Number('Clearance.Station_Zone.Endpoint_Radius', positive=True)
    assembly = reader.Raw('Clearance.Machine_Extra.Assembly')
    if assembly is not None:
        if not isinstance(assembly, str) or assembly not in [a[0] for a in cfg.Assembly_Overrides]:
            reader.Fail('Clearance.Machine_Extra.Assembly must name an Environment.Assembly_Overrides entry (%s), not %r.'
                        % (', '.join(a[0] for a in cfg.Assembly_Overrides) or 'none configured', assembly))
        else:
            cfg.Machine_Extra_Assembly = assembly
    cfg.Machine_Extra_Margin = reader.Number('Clearance.Machine_Extra.Margin', non_negative=True)
    if cfg.Machine_Extra_Margin > CONST_OVERRIDE_TRANSLATION_MAX:
        reader.Fail('Clearance.Machine_Extra.Margin (%r m) exceeds %.0f mm, the largest placement correction an assembly '
                    'may have; it stands for the accuracy of that placement.' % (cfg.Machine_Extra_Margin,
                                                                                  CONST_OVERRIDE_TRANSLATION_MAX * 1e3))
    source = reader.Raw('Clearance.Machine_Extra.Source')
    if source is not None:
        if not isinstance(source, str) or not source.strip():
            reader.Fail('Clearance.Machine_Extra.Source must be text naming where the margin comes from, not %r.' % (source,))
        else:
            cfg.Machine_Extra_Source = source
    cfg.Workspace_Min = reader.Vector('Clearance.Workspace.Min', 3)
    cfg.Workspace_Max = reader.Vector('Clearance.Workspace.Max', 3)
    if any(lo >= hi for lo, hi in zip(cfg.Workspace_Min, cfg.Workspace_Max)):
        reader.Fail('Clearance.Workspace.Min must be below Max on every axis.')
    cfg.Excluded_Pairs = [(a, b) for a, b in reader.Names('Self_Collision.Excluded_Pairs', 2)]
    cfg.Rigid_Groups = reader.Names('Self_Collision.Rigid_Groups', None)
    cfg.FK_Sanity_Bound_Position = reader.Number('Kinematics.FK_Sanity_Bound.Position', positive=True)

    # 3\ Fail once with every problem; report unknown keys as notes.
    if reader.Problems:
        raise Configuration_Error_Cls('The configuration "%s" has %d problem(s):\n  - %s'
                                      % (config_path, len(reader.Problems), '\n  - '.join(reader.Problems)))
    for key in Unknown_Keys(data, CONST_SCHEMA, ''):
        cfg.Notes.append(('WARNING', 'Unrecognised key %s is ignored (misspelt?).' % key))
    return cfg
