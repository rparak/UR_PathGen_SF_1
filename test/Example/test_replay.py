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
# File Name: ../test/Example/test_replay.py
## =========================================================================== ##

# System (Default)
import os
import re
import sys
# Subprocess (Subprocess management)
import subprocess

"""
Description:
    Initialization of constants.

    The replay is run the way a person runs it: a fresh interpreter, from an unrelated directory.
"""


def test_the_replay_builds_without_the_visual_mesh_by_default(root, scene, tmp_path):
    """
    Description:
        Without --with-stl the replay builds its scene without the visual STL: it says so, and it
        has exactly the geoms of the scene without the visual mesh (--with-stl adds one, the
        1.76 M-triangle STL). Headless: the screenshots replace the viewer.
    """

    result = subprocess.run([sys.executable, os.path.join(root, 'Example', 'replay_rotation.py'), '--case', '17',
                             '--segment', 'S2', '--screenshots', str(tmp_path)],
                            capture_output=True, text=True, cwd=str(tmp_path), timeout=600)
    assert result.returncode == 0, result.stdout + result.stderr
    built = re.search(r'Scene built in [\d.]+ s: (\d+) geoms; visual mesh (not loaded|loaded)\.', result.stdout)
    assert built is not None, result.stdout
    assert built.group(2) == 'not loaded'
    assert int(built.group(1)) == scene.Model.ngeom
    assert sorted(os.listdir(tmp_path)) == ['case17_S2_optimised.png', 'case17_S2_taught.png']
