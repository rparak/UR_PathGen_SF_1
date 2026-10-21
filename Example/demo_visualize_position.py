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
# File Name: ../Example/demo_visualize_position.py

# System (Default)
import os
import sys
#   Add access if it is not in the system path.
CONST_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src')
if CONST_SRC not in sys.path:
    sys.path.append(CONST_SRC)
# Argparse (Command-line parsing)
import argparse
# Dataclasses (Data Classes)
import dataclasses
# Time (Time access)
import time
# Typing (Support for type hints)
import typing as tp
# Custom Lib.:
#   ../Configuration/Core — the cell configuration
from Configuration.Core import Load_Configuration, Configuration_Error_Cls, CONST_DEFAULT_CONFIG_FILE
#   ../URScript/Core — the learned points
from URScript.Core import Robot_Point_Str, URScript_Error_Cls
#   ../Kinematics/Core — the refusal of a point outside the FK sanity bound
from Kinematics.Core import Kinematics_Error_Cls
#   ../Scene/Core — the scene
from Scene.Core import Scene_Cls, CONST_COLLISION_GROUP, Visual_Mesh_Warning
#   ../Example/plan_rotation — the default job (the program extract, the case), the openings and the clearance label
from plan_rotation import CONST_DEFAULT_JOB, Load_Job_Plan, Opening, Configuration_Label, Check_Learned


def Resolve_Name(name: str, points: tp.Dict[str, Robot_Point_Str], source: str) -> tp.Tuple[str, str]:
    """
    Description:
        The learned point a --pos argument names: a joint position, with or without its _q suffix.
        A _p name (a TCP pose) is refused: the viewer shows joint positions, and the robot's own IK
        would decide the joints of a pose.

    Args:
        (1) name [string]: The argument.
        (2) points [Dict<string, Robot_Point_Str>]: The learned points.
        (3) source [string]: The file they come from, for the message.

    Returns:
        (1) parameter [Tuple<string, string>]: The point name and '' - or '' and the refusal sentence.
    """

    if name.endswith('_p') and name[:-2] in points:
        return '', ('%s is a TCP pose (_p); this viewer shows joint positions. Use --pos %s or --pos %s_q.'
                    % (name, name[:-2], name[:-2]))
    base = name[:-2] if name.endswith('_q') else name
    if base not in points:
        return '', ('There is no learned position "%s" in %s. --list prints the %d names.'
                    % (name, source, len(points)))
    return base, ''


def main():
    """
    Description:
        Show the robot static at one learned position of the default job's program extract, with the
        gripper at its case's (17) holding opening or open, and report its clearance under the taught rule.

            macOS:  .venv/bin/mjpython Example/demo_visualize_position.py --pos home_q
            List:   .venv/bin/python Example/demo_visualize_position.py --list
            Tests:  .venv/bin/python Example/demo_visualize_position.py --pos home --headless

        The scene is built without the visual STL and the viewer starts with the collision geometry
        (geom group 3) shown; --with-stl loads the STL (geom group 1) as well, with a [WARNING] that it is
        the original CAD without the collision model's corrections (Visual_Mesh_Warning).
    """

    parser = argparse.ArgumentParser(description='Show the UR10e static at one learned position.')
    parser.add_argument('--config', default=CONST_DEFAULT_CONFIG_FILE, help='the cell configuration (default: %(default)s)')
    parser.add_argument('--pos', default='', help='a learned position: NAME or NAME_q (not NAME_p)')
    parser.add_argument('--list', action='store_true', help='print every learned position and exit')
    parser.add_argument('--opening', choices=('holding', 'open'), default='holding', help='gripper state (default: %(default)s)')
    parser.add_argument('--with-stl', action='store_true', help='load the visual STL of the cell (default: collision geometry only)')
    parser.add_argument('--headless', action='store_true', help='report without a viewer')
    args = parser.parse_args()
    try:
        plan = Load_Job_Plan(CONST_DEFAULT_JOB)
    except (Configuration_Error_Cls, URScript_Error_Cls, OSError) as error:
        print('[ERROR] %s' % error)
        return 1
    points, source, case = plan.Points, os.path.basename(plan.Positions), plan.Job.Case
    if args.list:
        print('[INFO] %d learned positions in %s:' % (len(points), source))
        for name in points:
            print('  %s' % name)
        return 0
    if not args.pos:
        print('[ERROR] Name a position with --pos NAME (--list prints them).')
        return 1
    name, refusal = Resolve_Name(args.pos, points, source)
    if refusal:
        print('[ERROR] %s' % refusal)
        return 1
    print('[WARNING] Simulation passed != safe on the robot.')
    try:
        cfg = Load_Configuration(args.config)
        opening = Opening(cfg, args.opening, plan)
        Check_Learned(cfg, points, source)
    except (Configuration_Error_Cls, Kinematics_Error_Cls) as error:
        print('[ERROR] %s' % error)
        return 1
    opening_source = cfg.Holding_Openings[(plan.Program_Name, case)][1] if args.opening == 'holding' else 'the full 2F-85 stroke'
    # The visual STL is the original CAD without the collision model's corrections: say so when it is asked for.
    if args.with_stl:
        print('[WARNING] %s' % Visual_Mesh_Warning(cfg))
    t0 = time.time()
    with_visual = args.with_stl and not args.headless
    scene = Scene_Cls(dataclasses.replace(cfg, Opening=opening), with_environment=True, with_visual=with_visual)
    print('[INFO] Scene built in %.1f s: %d geoms; visual mesh %s.'
          % (time.time() - t0, scene.Model.ngeom, 'loaded' if with_visual else 'not loaded'))
    q = points[name].Q
    label, violations, interior = Configuration_Label(scene, cfg, q, 'taught')
    print('[INFO] Position %s_q, gripper %s at %.1f mm (case %d: %s).' % (name, args.opening, opening * 1e3, case, opening_source))
    for component, distance in interior:
        print('[WARNING] Within the margin of the machine (a) interior hull (component %d): %.1f mm.' % (component, distance * 1e3))
    for v in violations:
        print('[%s] %s - %s: %.1f mm (margin %.1f mm)' % ('INFO' if interior else 'WARNING', v.Body_1, v.Body_2,
                                                          v.Distance * 1e3, v.Margin * 1e3))
    print('[INFO] LABEL: %s' % label)
    scene.Set_Joints(q)
    if args.headless:
        return 0
    import mujoco.viewer
    try:
        viewer = mujoco.viewer.launch_passive(scene.Model, scene.Data)
    except RuntimeError as error:
        print('[ERROR] The MuJoCo viewer cannot start (%s). On macOS run: .venv/bin/mjpython '
              'Example/demo_visualize_position.py' % error)
        return 2
    pairs = '\n'.join('%s - %s %.1f mm' % (v.Body_1, v.Body_2, v.Distance * 1e3) for v in violations[:8])
    with viewer:
        with viewer.lock():
            if not with_visual:
                viewer.opt.geomgroup[CONST_COLLISION_GROUP] = 1
        viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                          '%s_q  gripper %s %.1f mm (case %d)' % (name, args.opening, opening * 1e3, case),
                          label + ('\n' + pairs if pairs else '')))
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.05)
    return 0


if __name__ == '__main__':
    sys.exit(main())
