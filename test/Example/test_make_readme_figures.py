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
# File Name: ../test/Example/test_make_readme_figures.py
## =========================================================================== ##

# System (Default)
import os
import sys
# Subprocess (Running the script as a person does)
import subprocess
# Importlib (Import machinery), for running the script in this process with the STL reader replaced
import importlib.util
# Pytest (Testing framework) [pip3 install pytest]
import pytest

"""
Description:
    Initialization of constants.

    The README figure script, run the way a person runs it: a fresh interpreter, from an unrelated
    directory. The graphs are compared byte for byte with the committed ones in Media/; the MuJoCo
    renders are not, because their pixels depend on the OpenGL driver of the machine.
"""
# The graphs, each written for the light and the dark page theme.
#   Unit [Tuple<string>]
CONST_GRAPHS = ('cycle_time', 'path_length', 'max_height', 'clearance', 'joints', 'tcp_paths')
# The MuJoCo renders, each written for the light and the dark page theme.
#   Unit [Tuple<string>]
CONST_RENDERS = ('cell_overview', 'robot_home', 'station_collision', 'path_taught_vs_c1', 'path_c3_proposal')


def _Run(root, cwd, *args):
    return subprocess.run([sys.executable, os.path.join(root, 'Example', 'make_readme_figures.py')] + list(args),
                          cwd=str(cwd), capture_output=True, text=True, timeout=600)


def test_the_script_names_its_output_from_any_directory(root, tmp_path):
    """
    Description:
        --help runs in a fresh interpreter from an unrelated directory and names Media/ (built from
        the script's own location) as the default output, and the --graphs-only option.
    """

    result = _Run(root, tmp_path, '--help')
    assert result.returncode == 0, result.stdout + result.stderr
    assert os.path.join(root, 'Media') in ''.join(result.stdout.split())
    assert '--graphs-only' in result.stdout


@pytest.mark.slow
def test_the_graphs_are_the_committed_ones(root, tmp_path):
    """
    Description:
        Written again from the repository's data (--graphs-only, into an empty directory), every graph
        is the committed Media/ file byte for byte, in both themes, and nothing else is written; the
        script states that the figures are simulation estimates.
    """

    out = tmp_path / 'media'
    result = _Run(root, tmp_path, '--graphs-only', '--out', str(out))
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'Simulation passed != safe on the robot' in result.stdout
    expected = sorted('%s_%s.png' % (name, mode) for name in CONST_GRAPHS for mode in ('light', 'dark'))
    assert sorted(os.listdir(out)) == expected
    for name in expected:
        with open(os.path.join(root, 'Media', name), 'rb') as committed, open(os.path.join(out, name), 'rb') as written:
            assert written.read() == committed.read(), name


def test_the_renders_never_load_the_visual_stl(root, tmp_path, monkeypatch):
    """
    Description:
        The whole script (graphs and renders) runs with the project's STL reader replaced by one that
        fails: the renders show the environment as its collision model, never the visual STL, which
        is the original CAD without the corrections. Every render is written in both themes.
    """

    import Scene.Core

    def Refuse(path):
        raise AssertionError('the figure script loaded the visual STL %s' % path)

    for folder in ('Example', 'src'):
        if os.path.join(root, folder) not in sys.path:
            monkeypatch.syspath_prepend(os.path.join(root, folder))
    spec = importlib.util.spec_from_file_location('make_readme_figures', os.path.join(root, 'Example', 'make_readme_figures.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(Scene.Core, 'Read_Binary_STL', Refuse)
    monkeypatch.setattr(sys, 'argv', ['make_readme_figures.py', '--out', str(tmp_path)])
    assert module.main() == 0
    written = set(os.listdir(tmp_path))
    for name in CONST_RENDERS:
        assert {'%s_light.png' % name, '%s_dark.png' % name} <= written, name
