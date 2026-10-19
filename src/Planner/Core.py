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
# File Name: ../src/Planner/Core.py
## =========================================================================== ##

# Math (Mathematical functions)
import math
# Multiprocessing (Process-based parallelism)
import multiprocessing
import multiprocessing.pool
# Typing (Support for type hints)
import typing as tp
# Dataclasses (Data Classes)
from dataclasses import dataclass, field
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.:
#   ../Configuration/Core — the configuration a worker builds its scene from
from Configuration.Core import Cell_Configuration_Str
#   ../Scene/Core — the generated-rule checks
from Scene.Core import Scene_Cls

"""
Description:
    Initialization of constants.

    Joint-space planning of a generated segment between two fixed configurations (SPEC 10.6): the
    direct segment first, otherwise RRT-Connect, shortcutting and a local refinement of the via
    points, restarted with seeds derived from one recorded seed. Every budget is an iteration count,
    never a time, so the output depends on the inputs and the seed only.
"""
# The program's movej acceleration and speed, as its tokens (sf1_plc_v4.script), used for the
# duration estimate and written into every generated movej.
#   Unit [string] (rad/s^2, rad/s)
CONST_MOVEJ_A_TOKEN = '1.3962634015954636'
CONST_MOVEJ_V_TOKEN = '1.0471975511965976'
# At most this many via points per segment (SPEC 1); a longer path is a failure, never an output.
#   Unit [int]
CONST_MAX_VIAS = 5
# The penalty counts robot - environment distances below this, outside the station neighbourhood
# (owner, 2026-10-17); the hard margins are unchanged.
#   Unit [float] (m)
CONST_PENALTY_DISTANCE = 0.030
# RRT-Connect: the largest joint step of one extension, and the share of samples drawn from the box
# around both ends (expanded by CONST_LOCAL_EXPANSION), the rest from the whole joint range.
#   Unit [float] (rad), [float] (-), [float] (rad)
CONST_RRT_STEP = 0.35
CONST_LOCAL_SHARE = 0.5
CONST_LOCAL_EXPANSION = 1.0
# Refinement: the perturbation of a via point shrinks from the first to the last value.
#   Unit [float] (rad)
CONST_REFINE_SIGMA = (0.20, 0.01)
# A worker process's own scenes and planners (each Scene_Cls holds one MjModel / MjData), built on
# its first task for a configuration and kept for its later tasks (Request_Planner). Empty in the
# main process unless Plan_All runs without a pool.
#   Unit [Dict<string, Scene_Cls>], [Dict<string, Planner_Cls>]
CONST_WORKER_SCENES: tp.Dict[str, Scene_Cls] = {}
CONST_WORKER_PLANNERS: tp.Dict[str, tp.Any] = {}


class Planner_Error_Cls(ValueError):
    """
    Description:
        No generated path meets the rules within the limits. The message says which segment and why.
    """


@dataclass
class Planner_Budget_Str:
    """
    Description:
        The planner's budgets, all iteration counts.
    """

    # Independent RRT-Connect runs, each with its own derived seed.
    #   Unit [int]
    Restarts: int = 20
    # Extensions of one RRT-Connect run before it gives up.
    #   Unit [int]
    RRT_Iterations: int = 3000
    # Shortcut attempts on each found path.
    #   Unit [int]
    Shortcut_Iterations: int = 150
    # Paths (the fastest after shortcutting) that are refined, and the attempts on each.
    #   Unit [int]
    Refine_Candidates: int = 2
    Refine_Iterations: int = 150


@dataclass
class Plan_Result_Str:
    """
    Description:
        A planned and verified path with its measures.
    """

    # The configurations from start to goal (ends included).
    #   Unit [List<Vector<float> 1x6>] (rad)
    Path: tp.List[np.ndarray] = field(default_factory=list)
    # Estimated duration (trapezoid on the leading joint, stops at every via point).
    #   Unit [float] (s)
    Duration: float = 0.0
    # The clearance penalty: sum over samples of max(0, 30 mm - d) outside the station neighbourhood.
    #   Unit [float] (m)
    Penalty: float = 0.0
    # How the path was found: 'direct' or 'rrt' with the restart it came from.
    #   Unit [string]
    Origin: str = ''
    # Restarts that found a path (of those tried).
    #   Unit [int]
    Found: int = 0


@dataclass
class Plan_Request_Str:
    """
    Description:
        One generated segment to plan (Plan_All): everything a worker process needs to build its own
        scene and planner and to plan the segment as Planner_Cls.Plan does.
    """

    # The segment's name in messages, e.g. 'S1 (home -> drop_rotate_1)'.
    #   Unit [string]
    Label: str = ''
    # The cell configuration at the segment's gripper opening (Opening set).
    #   Unit [Cell_Configuration_Str]
    Configuration: tp.Optional[Cell_Configuration_Str] = None
    # The joints of the fixed station points.
    #   Unit [List<Vector<float> 1x6>] (rad)
    Station_Q: tp.List[np.ndarray] = field(default_factory=list)
    # Weight of the clearance penalty in the cost.
    #   Unit [float] (s/m)
    Penalty_Weight: float = 0.0
    # The iteration budgets.
    #   Unit [Planner_Budget_Str]
    Budget: Planner_Budget_Str = field(default_factory=Planner_Budget_Str)
    # The segment's ends, never wrapped.
    #   Unit [Vector<float> 1x6] (rad)
    Start: np.ndarray = field(default_factory=lambda: np.zeros(6))
    Goal: np.ndarray = field(default_factory=lambda: np.zeros(6))
    # The recorded seed of the segment.
    #   Unit [int]
    Seed: int = 0


def Segment_Duration(q_a: tp.Sequence[float], q_b: tp.Sequence[float], a: float, v: float) -> float:
    """
    Description:
        Estimated duration of one movej with r=0: the trapezoid (or triangle) profile of the joint
        that moves farthest, from rest to rest. An estimate: the controller's actual profile is
        not modelled.

    Args:
        (1 - 2) q_a, q_b [Vector<float> 1x6]: Segment ends (rad).
        (3 - 4) a, v [float]: Joint acceleration (rad/s^2) and speed (rad/s).

    Returns:
        (1) parameter [float]: The duration (s).
    """

    distance = float(np.max(np.abs(np.asarray(q_b, dtype=float) - np.asarray(q_a, dtype=float))))
    if distance >= v * v / a:
        return distance / v + v / a
    return 2.0 * math.sqrt(distance / a)


def Profile_Fraction(t: float, distance: float, a: float, v: float) -> float:
    """
    Description:
        The fraction of a rest-to-rest move done at time t under the profile the duration estimates
        assume: accelerate at a, cruise at v, decelerate at a (a triangle when the distance is too
        short to reach v). 0 at t <= 0, 1 at the end of the move.

    Args:
        (1) t [float]: Time since the start of the move (s).
        (2) distance [float]: The length of the move (rad for a joint, m for the TCP).
        (3 - 4) a, v [float]: Acceleration and speed (rad/s^2, rad/s or m/s^2, m/s).

    Returns:
        (1) parameter [float]: The fraction, 0 - 1.
    """

    if distance <= 0.0:
        return 1.0
    peak = min(v, math.sqrt(distance * a))
    t_ramp = peak / a
    total = 2.0 * t_ramp + (distance - peak * t_ramp) / peak
    t = min(max(t, 0.0), total)
    if t < t_ramp:
        done = 0.5 * a * t * t
    elif t <= total - t_ramp:
        done = 0.5 * peak * t_ramp + peak * (t - t_ramp)
    else:
        done = distance - 0.5 * a * (total - t) ** 2
    return min(1.0, max(0.0, done / distance))


def Estimate_Duration(path: tp.Sequence[tp.Sequence[float]], a: float, v: float) -> float:
    """
    Description:
        Estimated duration of a path of movej with r=0 (the robot stops at every via point).

    Args:
        (1) path [List<Vector<float> 1x6>]: The configurations, ends included (rad).
        (2 - 3) a, v [float]: Joint acceleration (rad/s^2) and speed (rad/s).

    Returns:
        (1) parameter [float]: The duration (s).
    """

    return sum(Segment_Duration(q_a, q_b, a, v) for q_a, q_b in zip(path[:-1], path[1:]))


def Linear_Duration(p_a: tp.Sequence[float], p_b: tp.Sequence[float], a: float, v: float) -> float:
    """
    Description:
        Estimated duration of one movel: the trapezoid (or triangle) profile on the TCP's straight
        path, from rest to rest. An estimate: the tool's rotation speed limit is not modelled.

    Args:
        (1 - 2) p_a, p_b [Vector<float> 1x6]: The poses (m, rad); only the positions are used.
        (3 - 4) a, v [float]: Tool acceleration (m/s^2) and speed (m/s).

    Returns:
        (1) parameter [float]: The duration (s).
    """

    distance = float(np.linalg.norm(np.asarray(p_b[:3], dtype=float) - np.asarray(p_a[:3], dtype=float)))
    if distance >= v * v / a:
        return distance / v + v / a
    return 2.0 * math.sqrt(distance / a)


def Path_Length(path: tp.Sequence[tp.Sequence[float]]) -> tp.Tuple[float, float]:
    """
    Description:
        Joint-path length: the sum over segments of sum_i |dq_i|, and of max_i |dq_i|.

    Args:
        (1) path [List<Vector<float> 1x6>]: The configurations, ends included (rad).

    Returns:
        (1) parameter [Tuple<float> 1x2]: Both lengths (rad).
    """

    steps = [np.abs(np.asarray(b, dtype=float) - np.asarray(a, dtype=float)) for a, b in zip(path[:-1], path[1:])]
    return float(sum(s.sum() for s in steps)), float(sum(s.max() for s in steps))


class Planner_Cls(object):
    """
    Description:
        Plans generated segments in one scene (one gripper opening) under the generated rule, with
        the fixed station points as the endpoint neighbourhood.

    Initialization of the Class:
        Args:
            (1) scene [Scene_Cls(object)]: The scene with the environment, at the segment's opening.
            (2) station_q [List<Vector<float> 1x6>]: The joints of the fixed station points (rad).
            (3) penalty_weight [float]: Weight of the clearance penalty in the cost (s/m).
            (4) Planner_Budget_Str [Planner_Budget_Str(object)]: The iteration budgets.

        Example:
            Initialization:
                # Initialization of the class.
                Cls = Planner_Cls(scene, station_q, 0.5, Planner_Budget_Str())

            Features:
                # Functions of the class.
                Cls.Plan(q_start, q_goal, seed)
    """

    def __init__(self, scene: Scene_Cls, station_q: tp.Sequence[tp.Sequence[float]], penalty_weight: float,
                 Planner_Budget_Str: Planner_Budget_Str) -> None:
        # << PRIVATE >> #
        self.__scene = scene
        self.__station_q = [np.asarray(q, dtype=float) for q in station_q]
        self.__station_tcps = scene.Station_TCPs(self.__station_q)
        self.__weight = float(penalty_weight)
        self.__budget = Planner_Budget_Str
        self.__a, self.__v = float(CONST_MOVEJ_A_TOKEN), float(CONST_MOVEJ_V_TOKEN)
        limits = scene.Model.jnt_range[:6]
        self.__lo, self.__hi = np.array(limits[:, 0]), np.array(limits[:, 1])
        self.__valid_cache: tp.Dict[bytes, bool] = {}
        self.__penalty_cache: tp.Dict[bytes, float] = {}
        # The station-box floors of the segment being planned (Plan); none outside a plan.
        self.__box_floors: tp.Optional[tp.Dict[str, float]] = None

    def __Key(self, q_a: np.ndarray, q_b: np.ndarray) -> bytes:
        """
        Description:
            The cache key of a segment: the bytes of both ends.

        Args:
            (1 - 2) q_a, q_b [Vector<float> 1x6]: Segment ends (rad).

        Returns:
            (1) parameter [bytes]: The key.
        """

        return np.concatenate([q_a, q_b]).tobytes()

    def Edge_Valid(self, q_a: np.ndarray, q_b: np.ndarray) -> bool:
        """
        Description:
            Whether a generated segment keeps the generated rule (early exit; same verdict as the
            full check), with the station-box floors of the segment being planned. Remembered per
            segment.

        Args:
            (1 - 2) q_a, q_b [Vector<float> 1x6]: Segment ends (rad).

        Returns:
            (1) parameter [bool]: True when valid.
        """

        key = self.__Key(q_a, q_b)
        if key not in self.__valid_cache:
            self.__valid_cache[key] = self.__scene.Segment_Valid(q_a, q_b, kind='generated', station_q=self.__station_q,
                                                                 box_floors=self.__box_floors)
        return self.__valid_cache[key]

    def Segment_Penalty(self, q_a: np.ndarray, q_b: np.ndarray) -> float:
        """
        Description:
            Sum over the segment's samples of max(0, 30 mm - d), d the robot - environment distance,
            for samples whose TCP lies outside the station neighbourhood. Remembered per segment.

        Args:
            (1 - 2) q_a, q_b [Vector<float> 1x6]: Segment ends (rad).

        Returns:
            (1) parameter [float]: The penalty (m).
        """

        key = self.__Key(q_a, q_b)
        if key not in self.__penalty_cache:
            total = 0.0
            for q in self.__scene.Segment_Samples(q_a, q_b):
                if not self.__scene.Near_Station(q, self.__station_tcps):
                    total += max(0.0, CONST_PENALTY_DISTANCE - self.__scene.Environment_Distance(q, CONST_PENALTY_DISTANCE)[0])
            self.__penalty_cache[key] = total
        return self.__penalty_cache[key]

    def Cost(self, path: tp.Sequence[np.ndarray]) -> float:
        """
        Description:
            The objective: estimated duration plus penalty_weight times the clearance penalty
            (SPEC 10.6); with weight 0 the penalty is not computed.

        Args:
            (1) path [List<Vector<float> 1x6>]: The configurations, ends included (rad).

        Returns:
            (1) parameter [float]: The cost (s).
        """

        cost = Estimate_Duration(path, self.__a, self.__v)
        if self.__weight > 0.0:
            cost += self.__weight * sum(self.Segment_Penalty(a, b) for a, b in zip(path[:-1], path[1:]))
        return cost

    def __Sample(self, rng: np.random.Generator, local_lo: np.ndarray, local_hi: np.ndarray) -> np.ndarray:
        """
        Description:
            One RRT sample: from the box around both ends with probability CONST_LOCAL_SHARE,
            otherwise from the whole joint range.

        Args:
            (1) rng [np.random.Generator]: The run's generator.
            (2 - 3) local_lo, local_hi [Vector<float> 1x6]: The box around both ends (rad).

        Returns:
            (1) parameter [Vector<float> 1x6]: The sample (rad).
        """

        if rng.random() < CONST_LOCAL_SHARE:
            return rng.uniform(local_lo, local_hi)
        return rng.uniform(self.__lo, self.__hi)

    def __Extend(self, tree: tp.List[np.ndarray], parents: tp.List[int], target: np.ndarray) -> tp.Tuple[str, int]:
        """
        Description:
            Extend a tree from its node nearest to the target (max-joint distance) by at most one
            step; the new edge must keep the generated rule.

        Args:
            (1) tree [List<Vector<float> 1x6>]: The tree's nodes (rad).
            (2) parents [List<int>]: Each node's parent (-1 for the root).
            (3) target [Vector<float> 1x6]: The target (rad).

        Returns:
            (1) parameter [Tuple<string, int>]: 'reached', 'advanced' or 'trapped', and the new (or
                                                 nearest) node.
        """

        nodes = np.array(tree)
        near = int(np.argmin(np.max(np.abs(nodes - target), axis=1)))
        step = target - tree[near]
        length = float(np.max(np.abs(step)))
        reached = length <= CONST_RRT_STEP
        new = target.copy() if reached else tree[near] + step * (CONST_RRT_STEP / length)
        if not self.Edge_Valid(tree[near], new):
            return 'trapped', near
        tree.append(new)
        parents.append(near)
        return ('reached' if reached else 'advanced'), len(tree) - 1

    def RRT_Connect(self, q_start: np.ndarray, q_goal: np.ndarray, rng: np.random.Generator) -> tp.Optional[tp.List[np.ndarray]]:
        """
        Description:
            Bidirectional RRT-Connect in joint space within the joint limits; samples are drawn half
            from the box around both ends (expanded by 1 rad, clipped to the limits), half from the
            whole range. Edges are checked with the generated rule.

        Args:
            (1 - 2) q_start, q_goal [Vector<float> 1x6]: The ends (rad).
            (3) rng [np.random.Generator]: The run's generator.

        Returns:
            (1) parameter [List<Vector<float> 1x6> or None]: The path, or None within the budget.
        """

        local_lo = np.maximum(np.minimum(q_start, q_goal) - CONST_LOCAL_EXPANSION, self.__lo)
        local_hi = np.minimum(np.maximum(q_start, q_goal) + CONST_LOCAL_EXPANSION, self.__hi)
        trees = ([q_start.copy()], [q_goal.copy()])
        parents = ([-1], [-1])
        for iteration in range(self.__budget.RRT_Iterations):
            a, b = (0, 1) if iteration % 2 == 0 else (1, 0)
            status, new = self.__Extend(trees[a], parents[a], self.__Sample(rng, local_lo, local_hi))
            if status == 'trapped':
                continue
            while True:
                status_b, reach = self.__Extend(trees[b], parents[b], trees[a][new])
                if status_b != 'advanced':
                    break
            if status_b == 'reached':
                branch_a = self.__Branch(trees[a], parents[a], new)
                branch_b = self.__Branch(trees[b], parents[b], reach)
                path = branch_a[::-1] + branch_b[1:]
                return path if a == 0 else path[::-1]
        return None

    def __Branch(self, tree: tp.List[np.ndarray], parents: tp.List[int], node: int) -> tp.List[np.ndarray]:
        """
        Description:
            The nodes from a node back to the tree's root.

        Args:
            (1) tree [List<Vector<float> 1x6>]: The tree's nodes (rad).
            (2) parents [List<int>]: Each node's parent (-1 for the root).
            (3) node [int]: The start node.

        Returns:
            (1) parameter [List<Vector<float> 1x6>]: The branch, node first.
        """

        out = []
        while node != -1:
            out.append(tree[node])
            node = parents[node]
        return out

    def Shortcut(self, path: tp.List[np.ndarray], rng: np.random.Generator) -> tp.List[np.ndarray]:
        """
        Description:
            Random shortcutting: join two vertices directly when the segment keeps the rule. Each
            accepted shortcut removes vertices, so the duration never grows.

        Args:
            (1) path [List<Vector<float> 1x6>]: The path (rad).
            (2) rng [np.random.Generator]: The run's generator.

        Returns:
            (1) parameter [List<Vector<float> 1x6>]: The shortened path.
        """

        path = list(path)
        for _ in range(self.__budget.Shortcut_Iterations):
            if len(path) <= 2:
                break
            i, j = sorted(rng.choice(len(path), size=2, replace=False))
            if j - i < 2:
                continue
            if self.Edge_Valid(path[i], path[j]):
                path = path[:i + 1] + path[j:]
        return path

    def Refine(self, path: tp.List[np.ndarray], rng: np.random.Generator) -> tp.List[np.ndarray]:
        """
        Description:
            Local refinement of the via points: remove one when its neighbours join validly, or move
            one by a shrinking random step; a change is kept only if it lowers the cost and both
            touched segments keep the rule.

        Args:
            (1) path [List<Vector<float> 1x6>]: The path (rad).
            (2) rng [np.random.Generator]: The run's generator.

        Returns:
            (1) parameter [List<Vector<float> 1x6>]: The refined path.
        """

        path = list(path)
        cost = self.Cost(path)
        iterations = self.__budget.Refine_Iterations
        for it in range(iterations):
            if len(path) <= 2:
                break
            k = int(rng.integers(1, len(path) - 1))
            if rng.random() < 0.25:
                trial = path[:k] + path[k + 1:]
                if self.Cost(trial) < cost and self.Edge_Valid(path[k - 1], path[k + 1]):
                    path, cost = trial, self.Cost(trial)
                continue
            sigma = CONST_REFINE_SIGMA[0] * (CONST_REFINE_SIGMA[1] / CONST_REFINE_SIGMA[0]) ** (it / max(1, iterations - 1))
            moved = np.clip(path[k] + rng.normal(0.0, sigma, 6), self.__lo, self.__hi)
            trial = path[:k] + [moved] + path[k + 1:]
            trial_cost = self.Cost(trial)
            if trial_cost < cost and self.Edge_Valid(path[k - 1], moved) and self.Edge_Valid(moved, path[k + 1]):
                path, cost = trial, trial_cost
        return path

    def __Use_Ends(self, start: np.ndarray, goal: np.ndarray) -> None:
        """
        Description:
            Set the station-box floors of a segment's two ends (Scene_Cls.Station_Box_Floors) for
            every edge checked after it. The edge verdicts depend on the floors, so those of another
            segment are forgotten.

        Args:
            (1 - 2) start, goal [Vector<float> 1x6]: The segment's ends (rad).
        """

        floors = self.__scene.Station_Box_Floors(start, goal)
        if floors != self.__box_floors:
            self.__box_floors, self.__valid_cache = floors, {}

    def Prepare(self, q_start: tp.Sequence[float], q_goal: tp.Sequence[float]) -> bool:
        """
        Description:
            Step 1 of a plan: set the station-box floors of the two ends, refuse an end that breaks
            the generated rule, and say whether the direct segment keeps it.

        Args:
            (1 - 2) q_start, q_goal [Vector<float> 1x6]: The ends (rad); never wrapped.

        Returns:
            (1) parameter [bool]: True when the direct segment keeps the rule.
        """

        start, goal = np.asarray(q_start, dtype=float), np.asarray(q_goal, dtype=float)
        self.__Use_Ends(start, goal)
        for name, q in (('start', start), ('goal', goal)):
            violations = self.__scene.Check_Configuration(q, 'generated', self.__station_tcps)
            if violations:
                raise Planner_Error_Cls('The %s configuration breaks the generated rule: %s.' % (name, '; '.join(
                    '%s - %s %.1f mm (margin %.1f mm)' % (v.Body_1, v.Body_2, v.Distance * 1e3, v.Margin * 1e3)
                    for v in violations)))
        return self.Edge_Valid(start, goal)

    def Restart(self, q_start: tp.Sequence[float], q_goal: tp.Sequence[float], seed: int,
                restart: int) -> tp.Optional[tp.Tuple[tp.List[np.ndarray], int]]:
        """
        Description:
            Step 2 of a plan, one restart: RRT-Connect and shortcutting with generator [seed, restart],
            under the station-box floors of the two ends. Depends on its arguments only, so the
            restarts can run in any order and in any process.

        Args:
            (1 - 2) q_start, q_goal [Vector<float> 1x6]: The ends (rad).
            (3) seed [int]: The recorded seed.
            (4) restart [int]: The restart's number.

        Returns:
            (1) parameter [Tuple<List<Vector<float> 1x6>, int> or None]: The shortcut path and the
                                                                         restart, or None.
        """

        start, goal = np.asarray(q_start, dtype=float), np.asarray(q_goal, dtype=float)
        self.__Use_Ends(start, goal)
        rng = np.random.default_rng([seed, restart])
        path = self.RRT_Connect(start, goal, rng)
        return None if path is None else (self.Shortcut(path, rng), restart)

    def Refine_Candidate(self, path: tp.List[np.ndarray], seed: int, rank: int,
                         restart: int) -> tp.Tuple[float, int, int, tp.List[np.ndarray]]:
        """
        Description:
            Step 3 of a plan: refine one of the fastest paths with generator [seed, 1000 + rank],
            under the station-box floors of its ends.

        Args:
            (1) path [List<Vector<float> 1x6>]: The path (rad).
            (2) seed [int]: The recorded seed.
            (3) rank [int]: Its rank among the found paths (Rank_Found).
            (4) restart [int]: The restart it came from.

        Returns:
            (1) parameter [Tuple<float, int, int, List<Vector<float> 1x6>>]: Cost, length, restart and
                                                                              the refined path.
        """

        self.__Use_Ends(np.asarray(path[0], dtype=float), np.asarray(path[-1], dtype=float))
        refined = self.Refine(path, np.random.default_rng([seed, 1000 + rank]))
        return self.Cost(refined), len(refined), restart, refined

    def Finish(self, path: tp.List[np.ndarray], origin: str, found: int) -> Plan_Result_Str:
        """
        Description:
            Step 4 of a plan: the independent final check (Verify_Path) and the measures.

        Args:
            (1) path [List<Vector<float> 1x6>]: The chosen path (rad).
            (2) origin [string]: 'direct' or 'rrt restart <n>'.
            (3) found [int]: Restarts that found a path.

        Returns:
            (1) parameter [Plan_Result_Str(object)]: The path and its measures.
        """

        self.Verify_Path(path)
        return Plan_Result_Str(Path=path, Duration=Estimate_Duration(path, self.__a, self.__v),
                               Penalty=sum(self.Segment_Penalty(a, b) for a, b in zip(path[:-1], path[1:])),
                               Origin=origin, Found=found)

    def Plan(self, q_start: tp.Sequence[float], q_goal: tp.Sequence[float], seed: int) -> Plan_Result_Str:
        """
        Description:
            Plan one generated segment in this process: the direct segment if it keeps the rule;
            otherwise every restart (generator [seed, restart]) runs RRT-Connect and shortcutting,
            the fastest paths are refined (generator [seed, 1000 + rank]), and the lowest cost wins,
            ties going to fewer via points (Rank_Found, Choose_Refined). Every edge keeps the
            station-box rule with the floors of these two ends (Scene_Cls.Station_Box_Floors). The
            result is verified with the full check (Verify_Path). Raises when an end breaks the
            rule, or no path with at most 5 via points is found within the budget. Plan_All runs
            the same steps in a process pool.

        Args:
            (1 - 2) q_start, q_goal [Vector<float> 1x6]: The ends (rad); never wrapped.
            (3) seed [int]: The recorded seed.

        Returns:
            (1) parameter [Plan_Result_Str(object)]: The path and its measures.
        """

        start, goal = np.asarray(q_start, dtype=float), np.asarray(q_goal, dtype=float)
        if self.Prepare(start, goal):
            return self.Finish([start, goal], 'direct', 0)
        found = [r for r in (self.Restart(start, goal, seed, k) for k in range(self.__budget.Restarts)) if r is not None]
        ranked = Rank_Found(found, self.__budget)
        refined = [self.Refine_Candidate(path, seed, rank, restart) for rank, (path, restart) in enumerate(ranked)]
        path, restart = Choose_Refined(refined)
        return self.Finish(path, 'rrt restart %d' % restart, len(found))

    def Verify_Path(self, path: tp.Sequence[np.ndarray]) -> None:
        """
        Description:
            The independent final check: every segment with the full generated-rule check
            (Check_Segment, no early exit, no cache), the station-box rule with the floors of the
            path's own ends, at most 5 via points, every configuration within the joint limits.
            Raises naming the first problem.

        Args:
            (1) path [List<Vector<float> 1x6>]: The configurations, ends included (rad).
        """

        if len(path) - 2 > CONST_MAX_VIAS:
            raise Planner_Error_Cls('The path has %d via points; at most %d are allowed.' % (len(path) - 2, CONST_MAX_VIAS))
        for q in path:
            if np.any(np.asarray(q) < self.__lo) or np.any(np.asarray(q) > self.__hi):
                raise Planner_Error_Cls('A configuration of the path lies outside the joint limits: %s.' % (list(q),))
        floors = self.__scene.Station_Box_Floors(path[0], path[-1])
        for index, (q_a, q_b) in enumerate(zip(path[:-1], path[1:])):
            samples, violations = self.__scene.Check_Segment(q_a, q_b, kind='generated', station_q=self.__station_q,
                                                             box_floors=floors)
            if violations:
                k, v = violations[0]
                raise Planner_Error_Cls('Segment %d of the path breaks the generated rule at sample %d of %d: %s - %s '
                                        '%.1f mm (margin %.1f mm).' % (index, k, samples, v.Body_1, v.Body_2,
                                                                       v.Distance * 1e3, v.Margin * 1e3))


def Rank_Found(found: tp.List[tp.Tuple[tp.List[np.ndarray], int]],
               Planner_Budget_Str: Planner_Budget_Str) -> tp.List[tp.Tuple[tp.List[np.ndarray], int]]:
    """
    Description:
        The paths the restarts found, fastest first (estimated duration, then fewer configurations,
        then the earlier restart), cut to the Refine_Candidates that are refined. Raises when no
        restart found a path.

    Args:
        (1) found [List<Tuple<List<Vector<float> 1x6>, int>>]: (shortcut path, restart), in restart order.
        (2) Planner_Budget_Str [Planner_Budget_Str(object)]: The iteration budgets.

    Returns:
        (1) parameter [List<Tuple<List<Vector<float> 1x6>, int>>]: The candidates in rank order.
    """

    if not found:
        raise Planner_Error_Cls('RRT-Connect found no path in %d restarts of %d iterations each.'
                                % (Planner_Budget_Str.Restarts, Planner_Budget_Str.RRT_Iterations))
    a, v = float(CONST_MOVEJ_A_TOKEN), float(CONST_MOVEJ_V_TOKEN)
    ranked = sorted(found, key=lambda item: (Estimate_Duration(item[0], a, v), len(item[0]), item[1]))
    return ranked[:Planner_Budget_Str.Refine_Candidates]


def Choose_Refined(refined: tp.List[tp.Tuple[float, int, int, tp.List[np.ndarray]]]) -> tp.Tuple[tp.List[np.ndarray], int]:
    """
    Description:
        The plan's result among the refined candidates: the lowest cost, ties going to fewer via
        points, then to the earlier restart; a path with more than 5 via points is never chosen.

    Args:
        (1) refined [List<Tuple<float, int, int, List<Vector<float> 1x6>>>]: (cost, length, restart, path).

    Returns:
        (1) parameter [Tuple<List<Vector<float> 1x6>, int>]: The path and its restart.
    """

    ordered = sorted(refined, key=lambda item: (item[0], item[1], item[2]))
    short = [r for r in ordered if len(r[3]) - 2 <= CONST_MAX_VIAS]
    if not short:
        raise Planner_Error_Cls('Every path found needs more than %d via points (fewest: %d).'
                                % (CONST_MAX_VIAS, min(len(r[3]) for r in ordered) - 2))
    return short[0][3], short[0][2]


def Request_Planner(Plan_Request_Str: Plan_Request_Str) -> Planner_Cls:
    """
    Description:
        This process's planner for a request: built on first use, one scene per configuration
        (CONST_WORKER_SCENES) and one planner per configuration, station points, weight and budget
        (CONST_WORKER_PLANNERS).

    Args:
        (1) Plan_Request_Str [Plan_Request_Str(object)]: The request.

    Returns:
        (1) parameter [Planner_Cls(object)]: The planner.
    """

    r = Plan_Request_Str
    scene_key = repr(r.Configuration)
    planner_key = repr((scene_key, [np.asarray(q, dtype=float).tolist() for q in r.Station_Q], r.Penalty_Weight, r.Budget))
    if scene_key not in CONST_WORKER_SCENES:
        CONST_WORKER_SCENES[scene_key] = Scene_Cls(r.Configuration)
    if planner_key not in CONST_WORKER_PLANNERS:
        CONST_WORKER_PLANNERS[planner_key] = Planner_Cls(CONST_WORKER_SCENES[scene_key], r.Station_Q, r.Penalty_Weight, r.Budget)
    return CONST_WORKER_PLANNERS[planner_key]


def Run_Plan_Task(task: tp.Tuple[str, Plan_Request_Str, tp.Tuple]) -> tp.Tuple[str, tp.Any]:
    """
    Description:
        One step of Plan_All in a worker process: 'prepare' (Planner_Cls.Prepare), 'restart'
        (Restart), 'refine' (Refine_Candidate) or 'finish' (Finish), on the process's own planner.
        A Planner_Error_Cls comes back as its message, so Plan_All can name the segment.

    Args:
        (1) task [Tuple<string, Plan_Request_Str, Tuple>]: The step, the request and the step's arguments.

    Returns:
        (1) parameter [Tuple<string, Any>]: ('ok', the step's result) or ('error', the message).
    """

    kind, request, args = task
    planner = Request_Planner(request)
    try:
        if kind == 'prepare':
            return 'ok', planner.Prepare(request.Start, request.Goal)
        if kind == 'restart':
            return 'ok', planner.Restart(request.Start, request.Goal, request.Seed, *args)
        if kind == 'refine':
            return 'ok', planner.Refine_Candidate(args[0], request.Seed, args[1], args[2])
        if kind == 'finish':
            return 'ok', planner.Finish(*args)
    except Planner_Error_Cls as error:
        return 'error', str(error)
    raise ValueError('A plan task is prepare, restart, refine or finish, not %r.' % (kind,))


def Make_Pool(workers: int) -> multiprocessing.pool.Pool:
    """
    Description:
        A pool of worker processes for Plan_All, started with 'spawn' (a fresh interpreter each, safe
        with MuJoCo on every platform). Each worker builds its own scenes and planners.

    Args:
        (1) workers [int]: The number of processes (>= 1).

    Returns:
        (1) parameter [multiprocessing.pool.Pool]: The pool; the caller closes it.
    """

    if workers < 1:
        raise ValueError('A pool needs at least one worker, not %d.' % workers)
    return multiprocessing.get_context('spawn').Pool(workers)


def Plan_All(requests: tp.Sequence[Plan_Request_Str], pool: tp.Optional[multiprocessing.pool.Pool] = None) -> tp.List[Plan_Result_Str]:
    """
    Description:
        Plan several segments. Without a pool each is planned in this process by Planner_Cls.Plan.
        With a pool the same steps run in its workers, every segment at once: all prepares, then
        every restart of every segment that needs a search, then their refinements, then the final
        checks, each step's results in request and restart order. Each step depends only on its
        request, its own generator ([seed, restart], [seed, 1000 + rank]) and its arguments, and the
        result is chosen by the same rule (Rank_Found, Choose_Refined), so the paths are identical
        to Plan's, byte for byte, for any number of workers. Raises Planner_Error_Cls naming the
        first segment that fails.

    Args:
        (1) requests [List<Plan_Request_Str>]: The segments.
        (2) pool [multiprocessing.pool.Pool]: Make_Pool's workers; none = this process.

    Returns:
        (1) parameter [List<Plan_Result_Str>]: The results, in request order.
    """

    def Run(tasks: tp.List[tp.Tuple[str, Plan_Request_Str, tp.Tuple]]) -> tp.List[tp.Any]:
        """
        Description:
            Run the tasks in the pool, in order; raise on the first error, naming its segment.

        Args:
            (1) tasks [List<Tuple<string, Plan_Request_Str, Tuple>>]: The tasks.

        Returns:
            (1) parameter [List<Any>]: Their results.
        """

        out = []
        for (_, request, _), (status, value) in zip(tasks, pool.map(Run_Plan_Task, tasks, chunksize=1)):
            if status == 'error':
                raise Planner_Error_Cls('%s: %s' % (request.Label, value))
            out.append(value)
        return out

    if pool is None:
        results = []
        for r in requests:
            try:
                results.append(Request_Planner(r).Plan(r.Start, r.Goal, r.Seed))
            except Planner_Error_Cls as error:
                raise Planner_Error_Cls('%s: %s' % (r.Label, error)) from error
        return results
    direct = Run([('prepare', r, ()) for r in requests])
    search = [i for i, d in enumerate(direct) if not d]
    restarts = [(i, k) for i in search for k in range(requests[i].Budget.Restarts)]
    found = Run([('restart', requests[i], (k,)) for i, k in restarts])
    ranked = {}
    for i in search:
        try:
            ranked[i] = Rank_Found([f for (j, _), f in zip(restarts, found) if j == i and f is not None], requests[i].Budget)
        except Planner_Error_Cls as error:
            raise Planner_Error_Cls('%s: %s' % (requests[i].Label, error)) from error
    candidates = [(i, rank, path, restart) for i in search for rank, (path, restart) in enumerate(ranked[i])]
    refined = Run([('refine', requests[i], (path, rank, restart)) for i, rank, path, restart in candidates])
    chosen = {}
    for i in search:
        try:
            chosen[i] = Choose_Refined([r for (j, _, _, _), r in zip(candidates, refined) if j == i])
        except Planner_Error_Cls as error:
            raise Planner_Error_Cls('%s: %s' % (requests[i].Label, error)) from error
    finish = []
    for i, r in enumerate(requests):
        if direct[i]:
            finish.append(('finish', r, ([np.asarray(r.Start, dtype=float), np.asarray(r.Goal, dtype=float)], 'direct', 0)))
        else:
            count = sum(1 for (j, _), f in zip(restarts, found) if j == i and f is not None)
            finish.append(('finish', r, (chosen[i][0], 'rrt restart %d' % chosen[i][1], count)))
    return Run(finish)
