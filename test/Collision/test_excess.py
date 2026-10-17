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
# File Name: ../test/Collision/test_excess.py
## =========================================================================== ##

# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.:
#   ../src/Collision/Core — the set and the distance functions
from Collision.Core import (Read_Collision_Set, Read_Binary_STL, Connected_Components, Hull_Of, Sample_Surface,
                            Surface_Distance, Point_Triangle_Distance)

"""
Description:
    Initialization of constants.

    The recorded excess of a piece (the build: trimesh closest point, hull vertices plus area
    samples) is recomputed independently: numpy exact distance from the piece's hull vertices plus
    an independent area sample (another seed) to the original component surface. Both are maxima
    over samples, so they differ by sampling only. The distance to the surface is 1-Lipschitz along
    the hull surface, so a sample with mean spacing s misses the true maximum by about s / 2; the
    tolerance is half the sum of both spacings: the build's (1 per cm^2, count clamped to
    [64, 20000]) and this test's (4000 samples).
"""
# Seed and the number of independent surface samples per piece.
#   Unit [int]
CONST_SEED = 7
CONST_SAMPLES = 4000
# Pieces of component 1154 checked besides the selected ones (it has the most CoACD pieces).
#   Unit [int]
CONST_COMPONENT = 1154
CONST_COMPONENT_PIECES = 3


@pytest.fixture(scope='module')
def setup(root, cfg):
    """
    Description:
        The committed set, the STL by component, and the configured component translations (an
        overridden component's excess is measured against its translated surface).
    """

    collision = Read_Collision_Set(cfg.Collision_Set, root)
    vertices, faces = Read_Binary_STL(cfg.Visual_Mesh)
    _, labels = Connected_Components(faces)
    shift = {c: np.asarray(t) for c, t, _ in cfg.Component_Overrides}
    return collision, vertices, faces, labels, shift


def _Selected(collision):
    """
    Description:
        The largest-excess piece (the fallback hull), the largest CoACD, hull and thickened pieces,
        and the first pieces of component 1154.
    """

    excess, methods = np.array(collision.Excess), np.array(collision.Methods)
    chosen = [int(np.argmax(excess))]
    for method in ('coacd', 'hull', 'thickened'):
        chosen.append(int(np.argmax(np.where(methods == method, excess, -1.0))))
    chosen += [i for i, c in enumerate(collision.Components) if c == CONST_COMPONENT][:CONST_COMPONENT_PIECES]
    return chosen


def test_the_pruned_surface_distance_is_exact(setup):
    """
    Description:
        Surface_Distance (pruned by a k-d tree) equals the distance to every triangle, also when the
        vertex array holds the whole STL (a first version took its bound from any vertex of it).
    """

    collision, vertices, faces, labels, shift = setup
    piece = int(np.argmax(collision.Excess))
    f = faces[labels == collision.Components[piece]]
    points = Hull_Of(collision.Pieces[piece])[0].points[:200]
    # vertices is the whole STL: the bound must come from this surface's vertices only.
    assert np.max(np.abs(Surface_Distance(points, vertices, f) - Point_Triangle_Distance(points, vertices[f]))) < 1e-12


def test_the_recorded_excess_matches_an_independent_recomputation(setup):
    """
    Description:
        For the selected pieces, incl. the 235 mm fallback hull, the recomputed excess equals the
        recorded one within half the sum of both sample spacings; the largest piece stays flagged above 10 mm.
    """

    collision, vertices, faces, labels, shift = setup
    rng = np.random.default_rng(CONST_SEED)
    flagged = set(collision.Manifest['flagged_open_shells']['pieces'])
    for piece in _Selected(collision):
        hull = Hull_Of(collision.Pieces[piece])[0]
        samples = Sample_Surface(hull.points, hull.simplices, rng, 1e9, CONST_SAMPLES, CONST_SAMPLES, per_triangle=False)
        points = np.concatenate([hull.points[hull.vertices], samples])
        c = collision.Components[piece]
        recomputed = float(np.max(Surface_Distance(points, vertices + shift.get(c, 0.0), faces[labels == c])))
        build_count = min(20000, max(64, int(np.ceil(hull.area * 1e4))))
        tolerance = 0.5 * (np.sqrt(hull.area / build_count) + np.sqrt(hull.area / CONST_SAMPLES))
        recorded = collision.Excess[piece]
        assert abs(recomputed - recorded) <= tolerance, (piece, collision.Methods[piece], recorded, recomputed, tolerance)
        assert (piece in flagged) == (recorded > collision.Manifest['hull_excess_max'])
    assert collision.Excess[int(np.argmax(collision.Excess))] > 0.2
