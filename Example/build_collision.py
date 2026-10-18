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
# File Name: ../Example/build_collision.py
## =========================================================================== ##

# System (Default)
import os
import sys
#   Add access if it is not in the system path.
CONST_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src')
if CONST_SRC not in sys.path:
    sys.path.append(CONST_SRC)
# Argparse (Command-line parsing)
import argparse
# Time (Time access)
import time
# Custom Lib.:
#   ../Configuration/Core — the cell configuration
from Configuration.Core import Load_Configuration, Configuration_Error_Cls, CONST_DEFAULT_CONFIG_FILE, CONST_PROJECT_ROOT
#   ../Collision/Core — the build, the STL reader and the writer
from Collision.Core import Build_Collision_Set, Read_Binary_STL, Write_Collision_Set
#   ../Scene/Core — the robot-only scene that bounds the reach
from Scene.Core import Scene_Cls

"""
Description:
    Initialization of constants.
"""
# Hull excess above which a component is decomposed with CoACD (SPEC 7.2; owner accepted the
# default, Q7).
#   Unit [float] (m)
CONST_HULL_EXCESS_MAX = 0.010
# Conservativeness tolerance (SPEC 7.3; Q7).
#   Unit [float] (m)
CONST_EPS = 0.001
# Seed of every sample and of CoACD, so the set is reproducible.
#   Unit [int]
CONST_SEED = 0


def main():
    """
    Description:
        Build CAD/Collision/ from the environment STL: crop to the robot's reach (bounded by the
        configured robot-only scene), hull per component, CoACD only where the hull is too coarse.
        Prints the metrics; the manifest records them.

            .venv/bin/python Example/build_collision.py
    """

    parser = argparse.ArgumentParser(description='Build the environment collision set.')
    parser.add_argument('--config', default=CONST_DEFAULT_CONFIG_FILE, help='the cell configuration (default: %(default)s)')
    args = parser.parse_args()
    try:
        cfg = Load_Configuration(args.config)
    except Configuration_Error_Cls as error:
        print('[ERROR] %s' % error)
        return 1
    for level, message in cfg.Notes:
        print('[%s] %s' % (level, message))

    started = time.time()
    scene = Scene_Cls(cfg, with_environment=False)
    centre, radius = scene.Shoulder_Point(), scene.Crop_Radius()
    print('[INFO] Crop sphere: centre %s m, radius %.3f m (reach bound %.3f m).'
          % ([round(float(x), 4) for x in centre], radius, scene.Reach_Bound()))
    vertices, faces = Read_Binary_STL(cfg.Visual_Mesh)
    for name, component, translation, source in cfg.Assembly_Overrides:
        print('[INFO] Assembly %s (the object of component %d) is translated by %s mm before the build (%s).'
              % (name, component, [round(v * 1e3, 1) for v in translation], source))
    for component, translation, source in cfg.Component_Overrides:
        print('[INFO] Component %d is translated by %s mm before the build (%s).'
              % (component, [round(v * 1e3, 1) for v in translation], source))
    print('[INFO] %s: %d vertices, %d faces.' % (os.path.relpath(cfg.Visual_Mesh, CONST_PROJECT_ROOT), len(vertices), len(faces)))
    result = Build_Collision_Set(vertices, faces, centre, radius, cfg.Stand_Region_Min, cfg.Stand_Region_Max,
                                 CONST_HULL_EXCESS_MAX, CONST_EPS, CONST_SEED, cfg.Assembly_Overrides, cfg.Component_Overrides,
                                 progress_cb=lambda message: print('[INFO] %s' % message, flush=True))
    result.Manifest['runtime_s']['total'] = round(time.time() - started, 2)
    os.makedirs(os.path.dirname(cfg.Collision_Set), exist_ok=True)
    manifest_path = Write_Collision_Set(result, cfg.Collision_Set, cfg.Visual_Mesh, CONST_PROJECT_ROOT)
    manifest = result.Manifest
    print('[INFO] %d pieces %s, bodies %s.' % (manifest['pieces'], manifest['pieces_by_method'], manifest['pieces_by_body']))
    print('[INFO] Conservativeness: %d points checked, max violation %.3f mm.'
          % (manifest['conservativeness']['points_checked'], manifest['conservativeness']['max_violation_m'] * 1e3))
    print('[INFO] Excess: max %.1f mm, p95 %.1f mm; %d pieces flagged (> %.0f mm).'
          % (manifest['excess']['max_m'] * 1e3, manifest['excess']['p95_m'] * 1e3,
             len(manifest['flagged_open_shells']['pieces']), CONST_HULL_EXCESS_MAX * 1e3))
    print('[INFO] Runtime %s s. Written %s.' % (manifest['runtime_s'], os.path.relpath(manifest_path, CONST_PROJECT_ROOT)))
    return 0


if __name__ == '__main__':
    sys.exit(main())
