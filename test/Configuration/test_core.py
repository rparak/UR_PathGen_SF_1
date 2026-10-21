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
# File Name: ../test/Configuration/test_core.py
## =========================================================================== ##

# System (Default)
import os
# Math (Mathematical functions)
import math
# Copy (Shallow and deep copy)
import copy
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# PyYAML (YAML parser and emitter) [pip3 install pyyaml]
import yaml
# Custom Lib.:
#   ../src/Configuration/Core — the loader under test
from Configuration.Core import Load_Configuration, Load_Job, Configuration_Error_Cls, CONST_DEFAULT_CONFIG_FILE, CONST_PROJECT_ROOT

"""
Description:
    Initialization of constants.

    Every REQUIRED field, named here independently of the loader's schema, so making a field
    optional in the loader cannot remove it from this test.
"""
# Every field of the shipped file, as a dotted path.
#   Unit [List<string>]
CONST_REQUIRED = ['Environment.Visual_Mesh', 'Environment.Collision_Set', 'Environment.Stand_Region.Min',
                  'Environment.Stand_Region.Max', 'Environment.Assembly_Overrides', 'Environment.Component_Overrides',
                  'Environment.Added_Boxes',
                  'Robot.Model', 'Robot.Base_Position', 'Robot.Base_Yaw',
                  'Robot.Joint_Position_Limits.Min', 'Robot.Joint_Position_Limits.Max', 'Tool.Model',
                  'Tool.Coupling.Thickness', 'Tool.Coupling.Diameter', 'Tool.Coupling.Spigot_Depth', 'Tool.Mount_Yaw',
                  'Tool.Opening', 'Tool.Holding_Openings', 'Tool.TCP_Offset', 'Clearance.Environment', 'Clearance.Self',
                  'Clearance.Self_Pair_Overrides', 'Clearance.Station_Zone.Min', 'Clearance.Station_Zone.Max',
                  'Clearance.Station_Zone.Environment', 'Clearance.Station_Zone.Endpoint_Radius',
                  'Clearance.Machine_Extra.Assembly', 'Clearance.Machine_Extra.Margin', 'Clearance.Machine_Extra.Source',
                  'Clearance.Workspace.Min', 'Clearance.Workspace.Max',
                  'Self_Collision.Excluded_Pairs', 'Self_Collision.Rigid_Groups',
                  'Kinematics.FK_Sanity_Bound.Position']


def _Shipped():
    with open(CONST_DEFAULT_CONFIG_FILE, encoding='utf-8') as handle:
        return yaml.safe_load(handle)


def _Set(data, path, value):
    node = data
    keys = path.split('.')
    for key in keys[:-1]:
        node = node[key]
    if value is _Set:
        del node[keys[-1]]
    else:
        node[keys[-1]] = value


def _Load(tmp_path, data):
    path = tmp_path / 'cell.yaml'
    path.write_text(yaml.safe_dump(data), encoding='utf-8')
    return Load_Configuration(str(path))


def test_the_shipped_configuration_loads_in_si_units():
    """
    Description:
        The shipped file loads without notes; degrees become radians; the TCP is the configured
        Polyscope/Robotiq value (0, 0, 174 mm).
    """

    cfg = Load_Configuration()
    assert cfg.Notes == []
    assert cfg.Station_Zone_Environment == 0.005 and cfg.Station_Zone_Endpoint_Radius == 0.050
    assert (cfg.Machine_Extra_Assembly, cfg.Machine_Extra_Margin, cfg.Machine_Extra_Source) == (
        'machine_a', 0.010, 'owner accuracy estimate of the machine position, 2026-10-18')
    assert cfg.Base_Position == [0.0, 0.0, 0.7152]
    assert cfg.TCP_Offset == [0.0, 0.0, 0.174, 0.0, 0.0, 0.0]
    assert cfg.Joint_Limits_Max[2] == pytest.approx(math.pi)
    assert cfg.Joint_Limits_Max[0] == pytest.approx(2.0 * math.pi)
    assert cfg.Clearance_Environment == 0.020 and cfg.Clearance_Self == 0.005
    assert cfg.Self_Pair_Overrides == [('base', 'upper_arm_link', 0.0)]
    assert cfg.Mount_Yaw == 0.0
    assert cfg.Holding_Openings == {('sf1_plc_v4', 17): (0.050, 'owner, 2026-10-18: the 50 x 50 mm part')}
    assert cfg.Assembly_Overrides == [
        ('machine_a', 1154, [-0.150, 0.0, 0.0], 'owner estimate, 2026-10-18 (100 mm, then a further 50 mm), not measured'),
        ('side_cabinet', 2643, [-0.050, 0.0, 0.0], 'owner estimate, 2026-10-18, not measured')]
    assert cfg.Component_Overrides == []
    assert cfg.Added_Boxes == [('station_bracket', [-0.770, -0.340, 0.955], [-0.645, -0.260, 1.112],
                                'owner photo + taught points, back to the shifted post (owner, 2026-10-18), not measured'),
                               ('station_stub', [-0.770, -0.330, 1.093], [-0.7274, -0.270, 1.264],
                                'owner photo + taught points, not measured')]


@pytest.mark.parametrize('field', CONST_REQUIRED)
def test_a_missing_required_field_fails_naming_it(tmp_path, field):
    """
    Description:
        Removing any REQUIRED field fails the load with a message naming it.
    """

    data = _Shipped()
    _Set(data, field, _Set)
    with pytest.raises(Configuration_Error_Cls, match=field.replace('.', r'\.') + ' is missing'):
        _Load(tmp_path, data)


@pytest.mark.parametrize('field, value, fragment', [
    ('Robot.Base_Position', [0.0, 0.7152], 'list of 3'),
    ('Robot.Joint_Position_Limits.Max', [360.0, 360.0, 181.0, 360.0, 360.0, 360.0], 'elbow'),
    ('Robot.Joint_Position_Limits.Min', [-361.0, -360.0, -180.0, -360.0, -360.0, -360.0], 'hardware range'),
    ('Robot.Joint_Position_Limits.Min', [-360.0, -360.0, 180.0, -360.0, -360.0, -360.0], 'not below Max'),
    ('Tool.Coupling.Spigot_Depth', 0.0169, 'smaller than'),
    ('Clearance.Environment', -0.02, 'greater than 0'),
    ('Kinematics.FK_Sanity_Bound.Position', 0.0, 'greater than 0'),
    ('Tool.Opening', 'wide', 'finite number'),
    ('Tool.Holding_Openings', [{'Program': 'p', 'Case': 17, 'Opening': 0.086, 'Source': 'x'}], 'at most the 2F-85 stroke'),
    ('Tool.Holding_Openings', [{'Program': 'p', 'Case': 17, 'Opening': 0.0, 'Source': 'x'}], 'greater than 0'),
    ('Tool.Holding_Openings', [{'Program': 'p', 'Case': 17, 'Opening': 0.05, 'Source': ' '}], 'Source: text'),
    ('Tool.Holding_Openings', [{'Program': 'p', 'Case': 17, 'Opening': 0.05}], 'Source: text'),
    ('Tool.Holding_Openings', [{'Case': 17, 'Opening': 0.05, 'Source': 'a'}], 'Program: program folder'),
    ('Tool.Holding_Openings', [], 'non-empty list'),
    ('Tool.Holding_Openings', [{'Program': 'p', 'Case': 17, 'Opening': 0.05, 'Source': 'a'},
                               {'Program': 'p', 'Case': 17, 'Opening': 0.04, 'Source': 'b'}], 'more than once'),
    ('Self_Collision.Excluded_Pairs', [['base']], 'pairs'),
    ('Clearance.Self_Pair_Overrides', [['base', 'upper_arm_link', -1.0]], 'margin >= 0'),
    ('Robot.Model', 'models/none.xml', 'does not exist'),
    ('Environment.Component_Overrides', [{'Component': 2710, 'Translation': [-0.151, 0.0, 0.0], 'Source': 'fit'}],
     'more than 150 mm'),
    ('Environment.Component_Overrides', [{'Component': 2710, 'Translation': [0.0, 0.0, 0.0], 'Source': ' '}],
     'Source: text'),
    ('Environment.Component_Overrides', [{'Component': 1, 'Translation': [0.0, 0.0, 0.0], 'Source': 'a'},
                                         {'Component': 1, 'Translation': [0.0, 0.0, 0.0], 'Source': 'b'}],
     'more than once'),
    ('Environment.Assembly_Overrides', [{'Name': 'a', 'Object_Of': 1154, 'Translation': [-0.151, 0.0, 0.0], 'Source': 'x'}],
     'more than 150 mm'),
    ('Environment.Assembly_Overrides', [{'Name': 'a', 'Object_Of': 1154, 'Translation': [0.0, 0.0, 0.0], 'Source': ''}],
     'Source: text'),
    ('Environment.Assembly_Overrides', [{'Name': 'a', 'Object_Of': 1, 'Translation': [0.0, 0.0, 0.0], 'Source': 'x'},
                                        {'Name': 'a', 'Object_Of': 2, 'Translation': [0.0, 0.0, 0.0], 'Source': 'y'}],
     'more than once'),
    ('Environment.Added_Boxes', [{'Name': 'b', 'Min': [0.0, 0.0, 0.1], 'Max': [0.1, 0.1, 0.1], 'Source': 'photo'}],
     'not below Max'),
    ('Environment.Added_Boxes', [{'Name': 'b', 'Min': [0.0, 0.0, 0.0], 'Max': [0.1, 0.1, 0.1], 'Source': ''}],
     'Source: text'),
    ('Environment.Added_Boxes', [{'Name': 'b', 'Min': [0.0, 0.0, 0.0], 'Max': [0.1, 0.1, 0.1], 'Source': 'a'},
                                 {'Name': 'b', 'Min': [0.0, 0.0, 0.0], 'Max': [0.1, 0.1, 0.1], 'Source': 'b'}],
     'more than once'),
    ('Clearance.Station_Zone.Environment', 0.021, 'must not exceed Clearance.Environment'),
    ('Clearance.Machine_Extra.Assembly', 'machine_b', 'must name an Environment.Assembly_Overrides entry'),
    ('Clearance.Machine_Extra.Margin', -0.001, 'must not be negative'),
    ('Clearance.Machine_Extra.Margin', 0.151, 'exceeds 150 mm'),
    ('Clearance.Machine_Extra.Source', ' ', 'must be text'),
    ('Clearance.Station_Zone.Max', [-0.85, -0.20, 1.85], 'below Max'),
    ('Clearance.Station_Zone.Endpoint_Radius', 0.0, 'greater than 0'),
    ('Clearance.Workspace.Max', [0.934, 1.045, -0.002], 'Workspace.Min must be below Max'),
])
def test_an_invalid_value_fails_and_never_falls_back(tmp_path, field, value, fragment):
    """
    Description:
        An invalid value fails the load naming the problem. In particular the elbow limit beyond
        +-180 deg is refused: it is the precondition of the upper_arm - forearm exclusion.
    """

    data = _Shipped()
    _Set(data, field, value)
    with pytest.raises(Configuration_Error_Cls, match=fragment):
        _Load(tmp_path, data)


def test_an_empty_list_of_added_boxes_is_accepted(tmp_path):
    """
    Description:
        Environment.Added_Boxes is REQUIRED but may be empty: a cell without obstacles missing from
        the STL loads with no box and no note.
    """

    data = _Shipped()
    _Set(data, 'Environment.Added_Boxes', [])
    cfg = _Load(tmp_path, data)
    assert cfg.Added_Boxes == [] and cfg.Notes == []


def test_every_problem_is_listed_at_once(tmp_path):
    """
    Description:
        Two problems are reported together, not one at a time.
    """

    data = _Shipped()
    _Set(data, 'Tool.Opening', _Set)
    _Set(data, 'Clearance.Self', -1.0)
    with pytest.raises(Configuration_Error_Cls) as error:
        _Load(tmp_path, data)
    assert '2 problem(s)' in str(error.value)
    assert 'Tool.Opening' in str(error.value) and 'Clearance.Self' in str(error.value)


def test_an_unknown_key_is_a_warning_not_a_failure(tmp_path):
    """
    Description:
        A misspelt key is reported as a WARNING note; the load succeeds.
    """

    data = copy.deepcopy(_Shipped())
    data['Tool']['Opennig'] = 0.05
    cfg = _Load(tmp_path, data)
    assert cfg.Notes == [('WARNING', 'Unrecognised key Tool.Opennig is ignored (misspelt?).')]


def test_the_same_case_number_of_two_programs_is_two_entries(tmp_path):
    """
    Description:
        Tool.Holding_Openings is keyed by program and case: case 17 of another program is its own
        entry, never sf1_plc_v4's (owner, 2026-10-21).
    """

    data = _Shipped()
    _Set(data, 'Tool.Holding_Openings', [{'Program': 'sf1_plc_v4', 'Case': 17, 'Opening': 0.05, 'Source': 'a'},
                                         {'Program': 'other', 'Case': 17, 'Opening': 0.03, 'Source': 'b'}])
    assert _Load(tmp_path, data).Holding_Openings == {('sf1_plc_v4', 17): (0.05, 'a'), ('other', 17): (0.03, 'b')}


# Every field of a job, as a dotted path, named here independently of the loader's schema.
#   Unit [List<string>]
CONST_JOB_REQUIRED = ['Program', 'Case', 'Segments', 'Point_Configurations', 'Output.Points', 'Output.Sequence', 'Output.Title',
                      'Output.Status', 'Output.Stations']
# The shipped default job, typed from the owner's statements (2026-10-18, 2026-10-19).
#   Unit [string], [List<Tuple>]
CONST_JOB_TURN = 'owner, 2026-10-19: q6 representative chosen to avoid an unnecessary wrist-3 turn'
CONST_JOB_SEGMENTS = [('S1', 'home', 'drop_rotate_1'), ('S2', 'Waypoint_5', 'Waypoint_18'), ('S3', 'Waypoint_6', 'home')]


def _Shipped_Job():
    with open(os.path.join(CONST_PROJECT_ROOT, 'Configuration', 'Jobs', 'sf1_plc_v4_case_17.yaml'), encoding='utf-8') as handle:
        return yaml.safe_load(handle)


def _Load_Job(tmp_path, data):
    path = tmp_path / 'job.yaml'
    path.write_text(yaml.safe_dump(data), encoding='utf-8')
    return Load_Job(str(path))


def test_the_shipped_case_17_jobs_load():
    """
    Description:
        The default job loads by its name without notes: program sf1_plc_v4, case 17, S1 - S3 as the
        owner set them, as c2 (owner, 2026-10-21: delivered and tested on the real cell): the drop's
        three points on branch S- E- W-, wrist 3 of the pick's three points one turn lower; its
        output names. The c1 job is the same without the branch, with its own status.
    """

    job = Load_Job('sf1_plc_v4_case_17')
    assert job.Notes == []
    assert job.Program == os.path.join(CONST_PROJECT_ROOT, 'Data', 'Programs', 'sf1_plc_v4')
    assert job.Case == 17 and job.Segments == CONST_JOB_SEGMENTS
    turns = [(name, 5, -1, '', CONST_JOB_TURN) for name in ('Waypoint_18', 'pick_rotate_1', 'Waypoint_6')]
    assert job.Point_Configurations == [(name, -1, 0, 'S- E- W-', 'owner, 2026-10-21: c2, delivered and tested on the real cell')
                                        for name in ('drop_rotate_1', 'Waypoint_4', 'Waypoint_5')] + turns
    assert (job.Output_Points, job.Output_Sequence, job.Title, job.Status, job.Stations) == (
        'SF1_Rotation_17.urscript', 'SF1_Rotation_17_Sequence.script', 'rotation sequence',
        'Delivered - tested on the real cell (owner, 2026-10-21).', ['drop', 'pick'])
    c1 = Load_Job('sf1_plc_v4_case_17_c1')
    assert c1.Notes == [] and c1.Point_Configurations == turns and (c1.Case, c1.Segments) == (17, CONST_JOB_SEGMENTS)
    assert c1.Status == 'Previous optimisation, simulation only (owner, 2026-10-21).'


@pytest.mark.parametrize('field', CONST_JOB_REQUIRED)
def test_a_missing_job_field_fails_naming_it(tmp_path, field):
    """
    Description:
        Removing any REQUIRED field of a job fails the load with a message naming it.
    """

    data = _Shipped_Job()
    _Set(data, field, _Set)
    with pytest.raises(Configuration_Error_Cls, match=field.replace('.', r'\.') + ' is missing'):
        _Load_Job(tmp_path, data)


@pytest.mark.parametrize('field, value, fragment', [
    ('Program', 'Data/Programs/none', 'must name a program folder'),
    ('Case', 0, 'case number >= 1'),
    ('Segments', [], 'non-empty list'),
    ('Segments', [{'Name': 'S1', 'From': 'home', 'To': 'home'}], 'starts and ends at home'),
    ('Segments', [{'Name': 'S1', 'From': 'home', 'To': 'a'}, {'Name': 'S1', 'From': 'b', 'To': 'home'}], 'more than once'),
    ('Point_Configurations', [{'Point': 'home', 'Joint': 7, 'Turns': 1, 'Source': 'x'}], 'Joint 1 \\(base\\) to 6'),
    ('Point_Configurations', [{'Point': 'home', 'Joint': 6, 'Turns': 0, 'Source': 'x'}], 'other than 0'),
    ('Point_Configurations', [{'Point': 'home', 'Branch': 'S- E-', 'Source': 'x'}], 'Branch as'),
    ('Point_Configurations', [{'Point': 'home', 'Branch': 'S- E- W-', 'Source': ' '}], 'ASCII text'),
    ('Point_Configurations', [{'Point': 'home', 'Branch': 'S- E- W-', 'Source': 'x'},
                              {'Point': 'home', 'Joint': 6, 'Turns': 1, 'Source': 'y'}], 'more than once'),
    ('Output.Points', 'SF1.script', 'ending in .urscript'),
    ('Output.Sequence', 'SF1_Rotation_17.urscript', 'ending in .script'),
    ('Output.Status', 'Delivered \u2014 tested', 'ASCII text'),
    ('Output.Stations', ['drop'], 'the 3 segments have 2 between them'),
])
def test_an_invalid_job_value_fails(tmp_path, field, value, fragment):
    """
    Description:
        An invalid job value fails the load naming the problem; a status with a non-ASCII dash is
        refused (it goes into a header pasted into Polyscope).
    """

    data = _Shipped_Job()
    _Set(data, field, value)
    with pytest.raises(Configuration_Error_Cls, match=fragment):
        _Load_Job(tmp_path, data)


def test_every_job_problem_is_listed_at_once_and_an_unknown_key_is_a_warning(tmp_path):
    """
    Description:
        Two problems of a job are reported together; a misspelt key alone is a WARNING note and the
        job loads.
    """

    data = _Shipped_Job()
    _Set(data, 'Case', _Set)
    _Set(data, 'Output.Title', '')
    with pytest.raises(Configuration_Error_Cls) as error:
        _Load_Job(tmp_path, data)
    assert '2 problem(s)' in str(error.value) and 'Case is missing' in str(error.value) and 'Output.Title' in str(error.value)
    data = _Shipped_Job()
    data['Output']['Titel'] = 'x'
    assert _Load_Job(tmp_path, data).Notes == [('WARNING', 'Unrecognised key Output.Titel of the job is ignored (misspelt?).')]


def test_an_unknown_job_name_is_refused_naming_the_jobs():
    """
    Description:
        A job name with no file in Configuration/Jobs/ is refused, listing the jobs there.
    """

    with pytest.raises(Configuration_Error_Cls, match=r'The job "sf1_plc_v4_case_99" cannot be read .*sf1_plc_v4_case_17'):
        Load_Job('sf1_plc_v4_case_99')
