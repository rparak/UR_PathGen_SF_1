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

    The only reader of the cell configuration (Configuration/Cell_SF1.yaml) and of the planning
    jobs (Configuration/Jobs/<name>.yaml). Every field is REQUIRED; every problem is collected and
    raised once as Configuration_Error_Cls. Angles in the file are degrees, and are converted to
    radians here so that no other module converts units.
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
# The planning jobs, Configuration/Jobs/<name>.yaml (SPEC 8.6).
#   Unit [string]
CONST_JOB_DIR = os.path.join(CONST_PROJECT_ROOT, 'Configuration', 'Jobs')
# The joints of a joint vector, base to wrist 3, as a job's Point_Configurations numbers them (1-6)
# and as messages and headers name them.
#   Unit [Tuple<string>]
CONST_JOINT_NAMES = ('base', 'shoulder', 'elbow', 'wrist 1', 'wrist 2', 'wrist 3')
# An IK branch as Solve_IK_Analytic labels it: shoulder, elbow and wrist sign.
#   Unit [Pattern]
CONST_BRANCH = re.compile(r'S[+-] E[+-] W[+-]')
# A learned point, segment or station name, and an output file name of a job.
#   Unit [Pattern]
CONST_JOB_NAME = re.compile(r'[A-Za-z_][A-Za-z0-9_]*')
CONST_OUTPUT_FILE = {'Points': re.compile(r'[A-Za-z0-9_.-]+\.urscript'), 'Sequence': re.compile(r'[A-Za-z0-9_.-]+\.script')}
# The known keys of a job, as CONST_SCHEMA.
#   Unit [Dict]
CONST_JOB_SCHEMA = {'Program': None, 'Case': None, 'Segments': None, 'Point_Configurations': None,
                    'Output': {'Points': None, 'Sequence': None, 'Title': None, 'Status': None, 'Stations': None}}
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
    # Inner pad-face gap while a case's part is held, per program folder and case (opening, source).
    #   Unit [Dict<Tuple<string, int>, Tuple<float, string>>] (-, -, m, -)
    Holding_Openings: tp.Dict[tp.Tuple[str, int], tp.Tuple[float, str]] = field(default_factory=dict)
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
        Whether one entry of Tool.Holding_Openings has exactly the keys Program (a program folder
        name under Data/Programs/), Case (a case number >= 1), Opening (a finite number) and Source
        (non-empty text).

    Args:
        (1) item [Any]: The entry as read from YAML.

    Returns:
        (1) parameter [bool]: True when the entry is well formed.
    """

    return (isinstance(item, dict) and set(item) == {'Program', 'Case', 'Opening', 'Source'}
            and isinstance(item['Program'], str) and re.fullmatch(r'[A-Za-z0-9_.-]+', item['Program']) is not None
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
            reader.Fail('Tool.Holding_Openings must be a non-empty list of {Program: program folder, Case: case number '
                        '>= 1, Opening: m, Source: text}, not %r.' % (holding,))
        else:
            for h in holding:
                if not 0.0 < h['Opening'] <= CONST_GRIPPER_STROKE:
                    reader.Fail('Tool.Holding_Openings: case %d of %s holds at %r m; the opening must be greater than 0 '
                                'and at most the 2F-85 stroke of %.3f m.' % (h['Case'], h['Program'], h['Opening'], CONST_GRIPPER_STROKE))
            cases = [(h['Program'], h['Case']) for h in holding]
            if len(set(cases)) != len(cases):
                reader.Fail('Tool.Holding_Openings names a case of a program more than once: %r.' % (cases,))
            cfg.Holding_Openings = {(h['Program'], h['Case']): (float(h['Opening']), h['Source']) for h in holding}
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


@dataclass
class Job_Configuration_Str:
    """
    Description:
        A loaded planning job (Configuration/Jobs/<name>.yaml): which program, case and segments are
        planned, how learned points are configured, and what the output is called. Paths are
        absolute. What the job implies in the program (the gripper states, the station lines) is
        resolved from the program's lines, not stored here.
    """

    # The job's name (the file name without .yaml) and the file.
    #   Unit [string]
    Name: str = ''
    File: str = ''
    # The program folder holding the extracts (Example/extract_program.py).
    #   Unit [string]
    Program: str = ''
    # The case of the program's switch.
    #   Unit [int]
    Case: int = 0
    # The segments to optimise, in program order (name, from, to).
    #   Unit [List<Tuple<string, string, string>>]
    Segments: tp.List[tp.Tuple[str, str, str]] = field(default_factory=list)
    # Learned points written with another _q: (point, joint index 0-5, whole turns, '', source) for a
    # joint turn, (point, -1, 0, branch, source) for an IK branch.
    #   Unit [List<Tuple<string, int, int, string, string>>] (-, -, 2 pi, -, -)
    Point_Configurations: tp.List[tp.Tuple[str, int, int, str, str]] = field(default_factory=list)
    # The output file names, the sequence's title and the output's status in the headers, and the
    # name of each verbatim block between two segments.
    #   Unit [string], [List<string>]
    Output_Points: str = ''
    Output_Sequence: str = ''
    Title: str = ''
    Status: str = ''
    Stations: tp.List[str] = field(default_factory=list)
    # (level, message) notes from the load, e.g. unknown keys.
    #   Unit [List<Tuple<string, string>>]
    Notes: tp.List[tp.Tuple[str, str]] = field(default_factory=list)


def Job_File(job: str) -> str:
    """
    Description:
        The file of a job: Configuration/Jobs/<job>.yaml for a name, the path itself for a path
        ending in .yaml.

    Args:
        (1) job [string]: The job's name or file.

    Returns:
        (1) parameter [string]: The file.
    """

    return os.path.abspath(job) if job.endswith('.yaml') else os.path.join(CONST_JOB_DIR, job + '.yaml')


def Header_Text(reader: Field_Reader_Cls, path: str) -> str:
    """
    Description:
        A job's text that goes into an output header: non-empty, one line, ASCII only (the output
        is pasted into a Polyscope program).

    Args:
        (1) reader [Field_Reader_Cls(object)]: The job's reader.
        (2) path [string]: The dotted key path.

    Returns:
        (1) parameter [string]: The text ('' when missing or invalid; a problem is recorded).
    """

    value = reader.Raw(path)
    if value is None:
        return ''
    if not isinstance(value, str) or not value.strip() or '\n' in value or not value.isascii():
        reader.Fail('%s must be one line of ASCII text (it goes into the output headers), not %r.' % (path, value))
        return ''
    return value


def Load_Job(job: str) -> Job_Configuration_Str:
    """
    Description:
        Load and validate a planning job (SPEC 8.6), as Load_Configuration: every problem is
        collected, then the load fails once with all of them; an unknown key is a WARNING note.

    Args:
        (1) job [string]: The job's name (Configuration/Jobs/<name>.yaml) or its file (*.yaml).

    Returns:
        (1) parameter [Job_Configuration_Str(object)]: The job.
    """

    # 1\ Read the file.
    path = Job_File(job)
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle)
    except OSError as error:
        known = sorted(name[:-5] for name in os.listdir(CONST_JOB_DIR) if name.endswith('.yaml')) if os.path.isdir(CONST_JOB_DIR) else []
        raise Configuration_Error_Cls('The job "%s" cannot be read (%s). The jobs in Configuration/Jobs/: %s.'
                                      % (job, error, ', '.join(known) or 'none')) from error
    except yaml.YAMLError as error:
        raise Configuration_Error_Cls('The job file "%s" is not valid YAML: %s' % (path, error)) from error
    if not isinstance(data, dict):
        raise Configuration_Error_Cls('The job file "%s" must be a mapping of fields.' % path)

    # 2\ Read every field, collecting problems.
    reader = Field_Reader_Cls(data)
    out = Job_Configuration_Str(Name=os.path.basename(path)[:-5], File=path)
    program = reader.Raw('Program')
    if program is not None:
        if not isinstance(program, str) or os.path.isabs(program) or not os.path.isdir(os.path.join(CONST_PROJECT_ROOT, program)):
            reader.Fail('Program must name a program folder relative to the repository root (Data/Programs/<name>), not %r. '
                        'Make it with Example/extract_program.py.' % (program,))
        else:
            out.Program = os.path.normpath(os.path.join(CONST_PROJECT_ROOT, program))
    case = reader.Raw('Case')
    if case is not None:
        if not isinstance(case, int) or isinstance(case, bool) or case < 1:
            reader.Fail('Case must be a case number >= 1, not %r.' % (case,))
        else:
            out.Case = case
    segments = reader.Raw('Segments')
    if segments is not None:
        if not isinstance(segments, list) or not segments or not all(
                isinstance(s, dict) and set(s) == {'Name', 'From', 'To'}
                and all(isinstance(s[k], str) and CONST_JOB_NAME.fullmatch(s[k]) for k in s) for s in segments):
            reader.Fail('Segments must be a non-empty list of {Name: name, From: learned point, To: learned point}, not %r.'
                        % (segments,))
        else:
            out.Segments = [(s['Name'], s['From'], s['To']) for s in segments]
            names = [s[0] for s in out.Segments]
            if len(set(names)) != len(names):
                reader.Fail('Segments names a segment more than once: %r.' % (names,))
            for name, start, goal in out.Segments:
                if start == goal:
                    reader.Fail('Segments: %s starts and ends at %s; a segment joins two points.' % (name, start))
    configurations = reader.Raw('Point_Configurations')
    if configurations is not None:
        if not isinstance(configurations, list):
            reader.Fail('Point_Configurations must be a list (may be empty), not %r.' % (configurations,))
            configurations = []
        for item in configurations:
            turn = isinstance(item, dict) and set(item) == {'Point', 'Joint', 'Turns', 'Source'}
            branch = isinstance(item, dict) and set(item) == {'Point', 'Branch', 'Source'}
            if not (turn or branch) or not isinstance(item['Point'], str) or not CONST_JOB_NAME.fullmatch(item['Point']) \
                    or not isinstance(item['Source'], str) or not item['Source'].strip() or not item['Source'].isascii():
                reader.Fail('Point_Configurations: each entry is {Point, Joint: 1-6, Turns: whole turns != 0, Source} or '
                            '{Point, Branch: e.g. "S- E- W-", Source}, the source ASCII text, not %r.' % (item,))
            elif turn and not (isinstance(item['Joint'], int) and not isinstance(item['Joint'], bool) and 1 <= item['Joint'] <= 6
                               and isinstance(item['Turns'], int) and not isinstance(item['Turns'], bool) and item['Turns'] != 0):
                reader.Fail('Point_Configurations: %s needs Joint 1 (base) to 6 (wrist 3) and Turns a whole number other '
                            'than 0, not %r.' % (item['Point'], item))
            elif branch and not (isinstance(item['Branch'], str) and CONST_BRANCH.fullmatch(item['Branch'])):
                reader.Fail('Point_Configurations: %s needs Branch as "S<+|-> E<+|-> W<+|->", not %r.' % (item['Point'], item['Branch']))
            elif turn:
                out.Point_Configurations.append((item['Point'], item['Joint'] - 1, item['Turns'], '', item['Source']))
            else:
                out.Point_Configurations.append((item['Point'], -1, 0, item['Branch'], item['Source']))
        points = [c[0] for c in out.Point_Configurations]
        if len(set(points)) != len(points):
            reader.Fail('Point_Configurations configures a point more than once: %r. One entry per point.' % (points,))
    for key in ('Points', 'Sequence'):
        value = reader.Raw('Output.' + key)
        if value is not None and not (isinstance(value, str) and CONST_OUTPUT_FILE[key].fullmatch(value)):
            reader.Fail('Output.%s must be a file name ending in %s, not %r.'
                        % (key, '.urscript' if key == 'Points' else '.script', value))
        elif value is not None:
            setattr(out, 'Output_' + key, value)
    if out.Output_Points and out.Output_Points == out.Output_Sequence:
        reader.Fail('Output.Points and Output.Sequence must be different files.')
    out.Title = Header_Text(reader, 'Output.Title')
    out.Status = Header_Text(reader, 'Output.Status')
    stations = reader.Raw('Output.Stations')
    if stations is not None:
        if not isinstance(stations, list) or not all(isinstance(s, str) and CONST_JOB_NAME.fullmatch(s) for s in stations):
            reader.Fail('Output.Stations must be a list of names, not %r.' % (stations,))
        elif out.Segments and len(stations) != len(out.Segments) - 1:
            reader.Fail('Output.Stations names %d station(s); the %d segments have %d between them.'
                        % (len(stations), len(out.Segments), len(out.Segments) - 1))
        else:
            out.Stations = list(stations)

    # 3\ Fail once with every problem; report unknown keys as notes.
    if reader.Problems:
        raise Configuration_Error_Cls('The job "%s" has %d problem(s):\n  - %s' % (path, len(reader.Problems), '\n  - '.join(reader.Problems)))
    for key in Unknown_Keys(data, CONST_JOB_SCHEMA, ''):
        out.Notes.append(('WARNING', 'Unrecognised key %s of the job is ignored (misspelt?).' % key))
    return out
