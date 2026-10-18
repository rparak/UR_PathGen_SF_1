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
# File Name: ../test/Scene/test_station.py
## =========================================================================== ##

# Dataclasses (Data Classes)
import dataclasses
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# MuJoCo (Physics simulator) [pip3 install mujoco]
import mujoco
# Custom Lib.:
#   ../src/Scene/Core — the scene and the taught / generated rule under test
from Scene.Core import Scene_Cls
#   ../src/Collision/Core — the set's components and the manifest form of the overrides
from Collision.Core import Read_Collision_Set, Component_Overrides_Record, Assembly_Overrides_Record

"""
Description:
    Initialization of constants.

    The rotation (regrasp) sequence of the owner's program sf1_plc_v4.script, cases 17 and 18, from
    home to home (owner-confirmed boundaries), typed from the program lines 3407-3463 and
    3490-3546. Every segment runs on the real robot without collision (owner). Case 17 is the
    deliverable; both cases are ground truth for the model here (SPEC 1). movel segments
    are checked joint-linear here (an approximation; the robot moves the TCP on a line), and blended
    corners (r > 0) are checked through the corner, not along the blend.
"""
# Gripper opening per case and state, typed here: holding the case's part (case 17: the 50 x 50 mm
# part, owner, its Tool.Holding_Openings entry; case 18 is not a deliverable and has no entry: its
# part's size is unknown and 50 mm is assumed for this check) and open after rq_set_pos_spd_for(0, ...),
# the full 85 mm stroke (owner, 2026-10-18: the release opening is real, SPEC 4.9).
#   Unit [Dict<Tuple<int, string>, float>] (m)
CONST_OPENING = {(17, 'holding'): 0.050, (17, 'open'): 0.085, (18, 'holding'): 0.050, (18, 'open'): 0.085}
# The segments (from, to, gripper state) of each case.
#   Unit [Dict<int, List<Tuple<string, string, string>>>]
CONST_SEQUENCES = {
    17: [('home', 'Waypoint_1', 'holding'), ('Waypoint_1', 'drop_rotate_1', 'holding'),
         ('drop_rotate_1', 'Waypoint_4', 'holding'), ('Waypoint_4', 'Waypoint_5', 'open'),
         ('Waypoint_5', 'Waypoint_1', 'open'), ('Waypoint_1', 'Waypoint_2', 'open'), ('Waypoint_2', 'Waypoint_3', 'open'),
         ('Waypoint_3', 'Waypoint_18', 'open'), ('Waypoint_18', 'pick_rotate_1', 'open'),
         ('pick_rotate_1', 'Waypoint_6', 'holding'), ('Waypoint_6', 'Waypoint_2', 'holding'), ('Waypoint_2', 'home', 'holding')],
    18: [('home', 'Waypoint_1', 'holding'), ('Waypoint_1', 'drop_rotate_2', 'holding'),
         ('drop_rotate_2', 'Waypoint_7', 'holding'), ('Waypoint_7', 'Waypoint_5', 'open'),
         ('Waypoint_5', 'Waypoint_1', 'open'), ('Waypoint_1', 'Waypoint_2', 'open'), ('Waypoint_2', 'Waypoint_3', 'open'),
         ('Waypoint_3', 'Waypoint_11', 'open'), ('Waypoint_11', 'pick_rotate_2', 'open'),
         ('pick_rotate_2', 'Waypoint_8', 'holding'), ('Waypoint_8', 'Waypoint_2', 'holding'), ('Waypoint_2', 'home', 'holding')],
}
# The fixed station points, which stay verbatim (owner): drop, the drop's movel target, the retreat
# after the drop, the approach to the pick, the pick and the pick's movel target, both cases.
#   Unit [Tuple<string>]
CONST_STATION_POINTS = ('drop_rotate_1', 'drop_rotate_2', 'Waypoint_4', 'Waypoint_7', 'Waypoint_5', 'Waypoint_18',
                        'Waypoint_11', 'pick_rotate_1', 'pick_rotate_2', 'Waypoint_6', 'Waypoint_8')
# The corrections and their sources (SPEC 4.11): the machine and the side cabinet as the owner
# estimates them; no component fit (the post's earlier fit is no longer needed).
#   Unit [List<Tuple<string, int, List<float> 1x3, string>>] (-, -, m, -),
#        [List<Tuple<int, List<float> 1x3, string>>] (-, m, -)
CONST_ASSEMBLIES = [('machine_a', 1154, [-0.150, 0.0, 0.0], 'owner estimate, 2026-10-18 (100 mm, then a further 50 mm), not measured'),
                    ('side_cabinet', 2643, [-0.050, 0.0, 0.0], 'owner estimate, 2026-10-18, not measured')]
CONST_OVERRIDES = []
# STL components: the left window profile (the post), the window's top profile, the machine body,
# the side table and the pallet table, the machine-interior fallback hull (235 mm excess) and the
# right window profile - none in the zone since it holds the holder's neighbourhood only (SPEC
# 4.12) - and the sill-frame pieces under the holder, which are. SPEC 4.1, 4.6, 4.11.
#   Unit [int], [Tuple<int>]
CONST_POST, CONST_TOP_PROFILE, CONST_SIDE_TABLE, CONST_PALLET_TABLE = 2710, 2699, 1061, 1060
CONST_MACHINE_INTERIOR, CONST_RIGHT_PROFILE, CONST_MACHINE_BODY = 1638, 2709, 1154
CONST_SILL_FRAME = (181, 1062, 2701, 2702, 2704)
# The station boxes (Environment.Added_Boxes, owner photo: the holder under the part and the stub
# above it on the post), and the cap
# of their distance search: above the global margin (20 mm), so that every distance inside a
# margin is exact.
#   Unit [Tuple<string>], [float] (m)
CONST_STATION_BOXES, CONST_BOX_DISTMAX = ('station_bracket', 'station_stub'), 0.030
# Near-margin segments for the early-exit equivalence: from each taught point of the sequence to a
# seeded perturbation of it (normal, sigma per joint), at the segment's opening.
#   Unit [int], [float] (rad), [int]
CONST_EQUIVALENCE_SEED, CONST_EQUIVALENCE_SIGMA, CONST_EQUIVALENCE_PER_POINT = 20261017, 0.15, 3


@pytest.fixture(scope='module')
def scenes(cfg):
    """
    Description:
        The scene with the environment at the opening of each case and gripper state (one scene per
        distinct opening). Case 17's configured holding opening must be the one typed here.
    """

    assert cfg.Holding_Openings[17][0] == CONST_OPENING[17, 'holding']
    built = {opening: Scene_Cls(dataclasses.replace(cfg, Opening=opening)) for opening in sorted(set(CONST_OPENING.values()))}
    return {key: built[opening] for key, opening in CONST_OPENING.items()}


def _Station_Q(points):
    return [points[name].Q for name in CONST_STATION_POINTS]


@pytest.mark.parametrize('case', sorted(CONST_SEQUENCES))
def test_the_taught_rotation_sequence_keeps_its_margins(scenes, points, case):
    """
    Description:
        Every segment of the taught rotation sequence, checked as taught, has no penetration and
        keeps the zone margin at the station-zone pieces and the global margins elsewhere. Any
        violation fails, naming the segment, the sample, the pair and the distance.
    """

    found = []
    for a, b, state in CONST_SEQUENCES[case]:
        samples, violations = scenes[case, state].Check_Segment(points[a].Q, points[b].Q, kind='taught')
        assert samples > 1
        found += ['%s -> %s sample %d: %s - %s %.2f mm (margin %.1f mm)' % (a, b, k, v.Body_1, v.Body_2, v.Distance * 1e3,
                                                                            v.Margin * 1e3) for k, v in violations]
    assert not found, '\n'.join(found)


@pytest.mark.parametrize('name', CONST_STATION_BOXES)
def test_the_taught_station_clears_the_station_boxes_by_the_zone_margin(scenes, cfg, points, name):
    """
    Description:
        Each station box (Environment.Added_Boxes, not in the STL) is in both scenes as a zone
        piece, so the replay above checks it. Its exact distance (mj_geomDistance) to every robot
        geom on every taught sample stays at or above the zone margin; it comes within the global
        margin, so the zone margin decides; and the contact MuJoCo generates at the closest sample
        has that same distance, so Check_Clearance sees the box.
    """

    closest = (np.inf, None, None, None)
    for case in sorted(CONST_SEQUENCES):
        for a, b, state in CONST_SEQUENCES[case]:
            scene = scenes[case, state]
            m, d = scene.Model, scene.Data
            box = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'env_box_' + name)
            assert box >= 0 and name in scene.Zone_Boxes
            robot = [g for g in range(m.ngeom) if m.geom_contype[g] and m.geom_bodyid[g] != 0
                     and not m.body(m.geom_bodyid[g]).name.startswith('env') and m.body(m.geom_bodyid[g]).name != 'base']
            for q in scene.Segment_Samples(points[a].Q, points[b].Q):
                scene.Set_Joints(q)
                for g in robot:
                    # Bounding spheres farther apart than the cap cannot come closer.
                    if (np.linalg.norm(d.geom_xpos[g] - d.geom_xpos[box]) - m.geom_rbound[g] - m.geom_rbound[box]
                            > CONST_BOX_DISTMAX):
                        continue
                    distance = mujoco.mj_geomDistance(m, d, g, box, CONST_BOX_DISTMAX, None)
                    if distance < closest[0]:
                        closest = (distance, '%d %s -> %s' % (case, a, b), (case, state), q)
    distance, where, key, q = closest
    assert cfg.Station_Zone_Environment <= distance < cfg.Clearance_Environment, (distance, where)
    scene = scenes[key]
    scene.Set_Joints(q)
    mujoco.mj_collision(scene.Model, scene.Data)
    box = mujoco.mj_name2id(scene.Model, mujoco.mjtObj.mjOBJ_GEOM, 'env_box_' + name)
    contacts = [c.dist for c in scene.Data.contact[:scene.Data.ncon] if box in (c.geom1, c.geom2)]
    assert contacts and min(contacts) == pytest.approx(distance, abs=1e-6)


@pytest.mark.parametrize('case', sorted(CONST_SEQUENCES))
def test_every_taught_segment_keeps_the_generated_rule(scenes, points, case):
    """
    Description:
        With the machine assembly shifted (SPEC 4.11), the taught rotation
        sequence keeps even the generated rule: wherever it comes closer than the global margin to
        the station, its TCP lies within the endpoint radius of a station point. The zone margin is
        therefore used only near the station points, never along the way.
    """

    found = []
    for a, b, state in CONST_SEQUENCES[case]:
        _, violations = scenes[case, state].Check_Segment(points[a].Q, points[b].Q, kind='generated',
                                                    station_q=_Station_Q(points))
        found += ['%s -> %s sample %d: %s - %s %.2f mm (margin %.1f mm)' % (a, b, k, v.Body_1, v.Body_2, v.Distance * 1e3,
                                                                            v.Margin * 1e3) for k, v in violations]
    assert not found, '\n'.join(found)


def test_a_generated_segment_uses_the_zone_margin_near_a_station_point(scenes, cfg, points):
    """
    Description:
        The open approach to the pick, Waypoint_18 -> pick_rotate_1, keeps less than 20 mm to the
        holder (5.5 mm, SPEC 4.11): as a generated segment it is clear only because every sample
        within 20 mm has its TCP within 50 mm of a station point; without station points the global
        margin applies and it violates.
    """

    scene, a, b = scenes[17, 'open'], points['Waypoint_18'].Q, points['pick_rotate_1'].Q
    assert scene.Check_Segment(a, b, kind='generated', station_q=_Station_Q(points))[1] == []
    violations = scene.Check_Segment(a, b, kind='generated')[1]
    assert violations and all(v.Margin == cfg.Clearance_Environment for _, v in violations)


def test_the_station_box_floor_is_the_closer_taught_end_capped_at_the_margin(scenes, cfg, points):
    """
    Description:
        Station_Box_Floors gives each station box the closer end's exact distance (mj_geomDistance,
        computed here), capped at Clearance.Environment: S2 (Waypoint_5 -> Waypoint_18, open) has
        the holder's floor at Waypoint_18 (15.1 mm, SPEC 4.11), and home -> Waypoint_1 is farther
        than the margin from every station box, so its floors are the margin.
    """

    scene = scenes[17, 'open']
    m, d = scene.Model, scene.Data
    robot = [g for g in range(m.ngeom) if m.geom_contype[g] and m.geom_bodyid[g] != 0
             and not m.body(m.geom_bodyid[g]).name.startswith('env') and m.body(m.geom_bodyid[g]).name != 'base']
    def Exact(q, name):
        box = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'env_box_' + name)
        scene.Set_Joints(q)
        return min(mujoco.mj_geomDistance(m, d, g, box, CONST_BOX_DISTMAX, None) for g in robot)
    a, b = points['Waypoint_5'].Q, points['Waypoint_18'].Q
    floors = scene.Station_Box_Floors(a, b)
    assert set(floors) == set(CONST_STATION_BOXES)
    for name in CONST_STATION_BOXES:
        assert floors[name] == pytest.approx(min(Exact(a, name), Exact(b, name), cfg.Clearance_Environment), abs=1e-6)
    assert floors['station_bracket'] == pytest.approx(0.0151, abs=0.0001)
    assert scene.Station_Box_Floors(points['home'].Q, points['Waypoint_1'].Q) == {name: cfg.Clearance_Environment
                                                                                   for name in CONST_STATION_BOXES}


def test_a_generated_sample_closer_to_a_station_box_than_its_floor_violates(scenes, cfg, points):
    """
    Description:
        The station-box rule: pick_rotate_1 (open) passes the holder at 5.5 mm, within the zone
        margin near the station, so it keeps the generated rule without floors; with S2's floors
        (15.1 mm, its closer taught end) it violates, naming the holder and the floor as the margin
        (and every other station box it is closer to than its floor);
        a floor at or below its distance does not. The taught rule ignores the floors.
    """

    scene, q = scenes[17, 'open'], points['pick_rotate_1'].Q
    tcps = scene.Station_TCPs(_Station_Q(points))
    floors = scene.Station_Box_Floors(points['Waypoint_5'].Q, points['Waypoint_18'].Q)
    assert scene.Check_Configuration(q, 'generated', tcps) == []
    violations = {v.Body_2: v for v in scene.Check_Configuration(q, 'generated', tcps, floors)}
    assert 'env_box_station_bracket' in violations
    assert violations['env_box_station_bracket'].Margin == floors['station_bracket']
    assert violations['env_box_station_bracket'].Distance == pytest.approx(0.0055, abs=0.0001)
    # Any other violation is a station box below its own floor (the stub: 10.0 mm at the pick, 10.1 mm at Waypoint_18).
    assert all(name[len('env_box_'):] in floors and v.Distance < v.Margin == floors[name[len('env_box_'):]]
               for name, v in violations.items())
    assert scene.Check_Configuration(q, 'generated', tcps, {name: 0.005 for name in floors}) == []
    assert scene.Check_Configuration(q, 'taught', None) == []


def test_generated_segments_keep_the_extra_margin_to_the_machine_outside_the_zone(scenes, cfg, points, root):
    """
    Description:
        Clearance.Machine_Extra (machine (a), +10 mm, the owner's accuracy estimate): on the direct
        segment Waypoint_6 -> home (holding), a sample 25 mm (exact mj_geomDistance, +-0.5 mm) from a
        machine-(a) piece outside the zone, and farther than 30 mm from everything else, violates
        the generated rule with the 30 mm margin; a sample 35 mm away does not; the taught rule
        keeps 20 mm to it, so the taught replay is unaffected.
    """

    scene = scenes[17, 'holding']
    m, d = scene.Model, scene.Data
    collision = Read_Collision_Set(cfg.Collision_Set, root)
    members, zone = set(collision.Manifest['assembly_components']['machine_a']), set(scene.Zone_Pieces)
    machine = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, 'env_piece_%d' % i) for i, c in enumerate(collision.Components)
               if c in members and i not in zone]
    others = [g for g in range(m.ngeom) if m.geom_contype[g] and m.body(m.geom_bodyid[g]).name.startswith('env')
              and g not in set(machine)]
    robot = [g for g in range(m.ngeom) if m.geom_contype[g] and m.geom_bodyid[g] != 0
             and not m.body(m.geom_bodyid[g]).name.startswith('env') and m.body(m.geom_bodyid[g]).name != 'base']
    def Exact(q, geoms):
        scene.Set_Joints(q)
        best = 0.040
        for g in robot:
            for e in geoms:
                if np.linalg.norm(d.geom_xpos[g] - d.geom_xpos[e]) - m.geom_rbound[g] - m.geom_rbound[e] > best:
                    continue
                best = min(best, mujoco.mj_geomDistance(m, d, g, e, best, None))
        return best
    assert (cfg.Machine_Extra_Assembly, cfg.Machine_Extra_Margin) == ('machine_a', 0.010)
    tcps = scene.Station_TCPs(_Station_Q(points))
    found = {}
    for q in scene.Segment_Samples(points['Waypoint_6'].Q, points['home'].Q)[::3]:
        # A cheap pre-selection; the exact distances below decide.
        if not 0.023 < scene.Environment_Distance(q, 0.040)[0] < 0.037:
            continue
        distance = Exact(q, machine)
        for target in (0.025, 0.035):
            if target not in found and abs(distance - target) <= 0.0005 and Exact(q, others) > 0.030:
                found[target] = (q, distance)
    assert set(found) == {0.025, 0.035}, found
    q, distance = found[0.025]
    violations = scene.Check_Configuration(q, 'generated', tcps)
    assert [round(v.Margin, 6) for v in violations] == [0.030] and violations[0].Distance == pytest.approx(distance, abs=1e-6)
    assert scene.Check_Configuration(q, 'taught') == []
    assert scene.Check_Configuration(found[0.035][0], 'generated', tcps) == []


def test_the_corrections_are_recorded_and_configured(scenes, cfg):
    """
    Description:
        The collision set's manifest records exactly the two assembly shifts and no component
        correction, which are also the configured ones (the scene would refuse a mismatch).
    """

    manifest = scenes[17, 'holding'].Collision_Manifest
    assert manifest['assembly_overrides'] == Assembly_Overrides_Record(CONST_ASSEMBLIES)
    assert manifest['assembly_overrides'] == Assembly_Overrides_Record(cfg.Assembly_Overrides)
    assert manifest['component_overrides'] == Component_Overrides_Record(CONST_OVERRIDES)
    assert manifest['component_overrides'] == Component_Overrides_Record(cfg.Component_Overrides)


def test_the_zone_holds_the_holders_neighbourhood_only(scenes, cfg, root):
    """
    Description:
        The station zone is the holder, the stub and the sill frame (owner, 2026-10-18, SPEC 4.12,
        4.13): the two station boxes and exactly the sill-frame components are zone pieces; the post, the window's
        top profile, the machine body, the right window profile, the tables and the machine-interior
        hull are not, so they keep the global margin.
    """

    components = Read_Collision_Set(cfg.Collision_Set, root).Components
    zone = {components[i] for i in scenes[17, 'holding'].Zone_Pieces}
    assert scenes[17, 'holding'].Zone_Boxes == list(CONST_STATION_BOXES)
    assert zone == set(CONST_SILL_FRAME)
    assert not zone & {CONST_POST, CONST_TOP_PROFILE, CONST_MACHINE_BODY, CONST_RIGHT_PROFILE, CONST_SIDE_TABLE,
                       CONST_PALLET_TABLE, CONST_MACHINE_INTERIOR}


@pytest.mark.parametrize('case', sorted(CONST_SEQUENCES))
def test_without_the_holder_the_taught_sequence_keeps_the_global_margin(cfg, points, case):
    """
    Description:
        The zone margin is needed for the station boxes only: with them removed, every taught
        segment keeps the global margins everywhere (the generated rule without station points),
        the post included (>= 39.1 mm, SPEC 4.11).
    """

    no_holder = {opening: Scene_Cls(dataclasses.replace(cfg, Opening=opening, Added_Boxes=[]))
                 for opening in sorted(set(CONST_OPENING.values()))}
    found = []
    for a, b, state in CONST_SEQUENCES[case]:
        _, violations = no_holder[CONST_OPENING[case, state]].Check_Segment(points[a].Q, points[b].Q, kind='generated')
        found += ['%s -> %s sample %d: %s - %s %.2f mm' % (a, b, k, v.Body_1, v.Body_2, v.Distance * 1e3) for k, v in violations]
    assert not found, '\n'.join(found)


def test_the_early_exit_check_agrees_with_the_full_check(scenes, points):
    """
    Description:
        Segment_Valid stops at the first violating sample; its verdict equals "Check_Segment finds no
        violation" on seeded segments from every taught point of the sequence to perturbations of it
        (generated rule, station points as the fixed points), and both verdicts occur.
    """

    rng = np.random.default_rng(CONST_EQUIVALENCE_SEED)
    verdicts = []
    for case in sorted(CONST_SEQUENCES):
        for a, _, state in CONST_SEQUENCES[case]:
            q = np.array(points[a].Q)
            for _ in range(CONST_EQUIVALENCE_PER_POINT):
                target = q + rng.normal(0.0, CONST_EQUIVALENCE_SIGMA, 6)
                full = not scenes[case, state].Check_Segment(q, target, kind='generated', station_q=_Station_Q(points))[1]
                fast = scenes[case, state].Segment_Valid(q, target, kind='generated', station_q=_Station_Q(points))
                assert fast == full, (case, a, list(target))
                verdicts.append(full)
    assert any(verdicts) and not all(verdicts)


def test_the_environment_distance_equals_the_exact_distance(scenes, points):
    """
    Description:
        Environment_Distance (raised margins, one collision pass) equals the smallest
        mj_geomDistance between every robot geom (the static base excluded, as in the scene) and
        every environment geom, capped at 30 mm, on samples 12 - 27 of Waypoint_3 -> Waypoint_18 (open,
        the approach to the station boxes), which include distances in the 20 - 30 mm band that
        contacts at the normal margins never show.
    """

    scene = scenes[17, 'open']
    m, d = scene.Model, scene.Data
    env = [g for g in range(m.ngeom) if m.geom_contype[g] and m.body(m.geom_bodyid[g]).name.startswith('env')]
    robot = [g for g in range(m.ngeom) if m.geom_contype[g] and m.geom_bodyid[g] != 0
             and not m.body(m.geom_bodyid[g]).name.startswith('env') and m.body(m.geom_bodyid[g]).name != 'base']
    samples = scene.Segment_Samples(points['Waypoint_3'].Q, points['Waypoint_18'].Q)
    band = 0
    for q in samples[12:28]:
        measured = scene.Environment_Distance(q, 0.030)[0]
        scene.Set_Joints(q)
        exact = 0.030
        for g in robot:
            for e in env:
                exact = min(exact, mujoco.mj_geomDistance(m, d, g, e, exact, None))
        assert abs(measured - exact) < 1e-6, (measured, exact)
        band += int(0.020 < exact < 0.030)
    assert band > 0
