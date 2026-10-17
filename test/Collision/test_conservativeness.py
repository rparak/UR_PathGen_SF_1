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
# File Name: ../test/Collision/test_conservativeness.py
## =========================================================================== ##

# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.:
#   ../src/Collision/Core — the conservativeness check under test
from Collision.Core import (Read_Collision_Set, Read_Binary_STL, Connected_Components, Conservativeness_Points, Hull_Of,
                            Union_Distances, Point_Triangle_Distance)

"""
Description:
    Initialization of constants.

    Evidence for the conservativeness numbers of the committed set. The build checks CoACD parts with
    Union_Distances, which skips a piece when a facet-plane lower bound proves it cannot be closer.
    These tests hold it against the exhaustive computation (every piece, every triangle), whose
    point-triangle distance is itself checked against an independent formula; and they recompute the
    recorded maximum, so it cannot be a value capped by an early exit (the first build's check had
    one: SPEC 17).
"""
# Components with the largest true distances (measured 2026-10-17 with the exhaustive computation),
# plus three ordinary CoACD components.
#   Unit [Tuple<int>]
CONST_COMPONENTS = (2132, 1322, 2643, 1154, 1856, 539, 646, 2269, 1219, 1207)
CONST_ORDINARY = 3
# Agreement required between the pruned and the exhaustive computation: 1e-6 mm.
#   Unit [float] (m)
CONST_AGREEMENT = 1e-9


def _Independent_Distance(point, triangle):
    """
    Description:
        Distance to a triangle the plain way: the plane foot if it falls inside (barycentric test),
        otherwise the nearest of the three edges as clamped segments.
    """

    a, b, c = triangle
    n = np.cross(b - a, c - a)
    n = n / np.linalg.norm(n)
    foot = point - np.dot(point - a, n) * n
    v0, v1, v2 = b - a, c - a, foot - a
    d00, d01, d11, d20, d21 = v0 @ v0, v0 @ v1, v1 @ v1, v2 @ v0, v2 @ v1
    den = d00 * d11 - d01 * d01
    v, w = (d11 * d20 - d01 * d21) / den, (d00 * d21 - d01 * d20) / den
    if v >= 0.0 and w >= 0.0 and v + w <= 1.0:
        return float(np.linalg.norm(point - foot))
    best = np.inf
    for p, q in ((a, b), (b, c), (c, a)):
        t = np.clip(np.dot(point - p, q - p) / np.dot(q - p, q - p), 0.0, 1.0)
        best = min(best, float(np.linalg.norm(point - (p + t * (q - p)))))
    return best


@pytest.fixture(scope='module')
def setup(root, cfg):
    """
    Description:
        The committed set, the STL by component, the pieces of each CoACD component, and the
        configured component translations (the build checks an overridden component against its
        translated surface, so the recomputation does too).
    """

    collision = Read_Collision_Set(cfg.Collision_Set, root)
    vertices, faces = Read_Binary_STL(cfg.Visual_Mesh)
    _, labels = Connected_Components(faces)
    pieces = {}
    for i, (c, m) in enumerate(zip(collision.Components, collision.Methods)):
        if m == 'coacd':
            pieces.setdefault(c, []).append(i)
    shift = {c: np.asarray(t) for c, t, _ in cfg.Component_Overrides}
    return collision, (vertices, shift), faces, labels, pieces


def _Component(mesh, faces, labels, c):
    vertices, shift = mesh
    f = faces[labels == c]
    used = np.unique(f)
    remap = np.full(len(vertices), -1)
    remap[used] = np.arange(len(used))
    return vertices[used] + shift.get(c, 0.0), remap[f]


def test_the_point_triangle_distance_matches_an_independent_formula():
    """
    Description:
        Random points against random (incl. thin) triangles: every Voronoi region agrees within 1e-12 m.
    """

    rng = np.random.default_rng(3)
    triangles = rng.normal(size=(40, 3, 3))
    triangles[0] = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.5, 1e-4, 0.0]]
    points = rng.normal(size=(300, 3)) * 2.0
    for triangle in triangles:
        fast = Point_Triangle_Distance(points, triangle[None])
        slow = np.array([_Independent_Distance(p, triangle) for p in points])
        assert np.max(np.abs(fast - slow)) < 1e-12


def test_the_pruned_check_equals_the_exhaustive_one(setup, cfg):
    """
    Description:
        On the ten components with the largest distances and three ordinary CoACD components, the
        pruned distances equal the exhaustive ones point by point (1e-6 mm), so the verdict and the
        maximum are the same.
    """

    collision, vertices, faces, labels, pieces = setup
    eps = collision.Manifest['eps']
    ordinary = sorted(c for c in pieces if c not in CONST_COMPONENTS)[:CONST_ORDINARY]
    for c in list(CONST_COMPONENTS) + ordinary:
        assert c in pieces, 'component %d is not a CoACD component of the committed set' % c
        V, F = _Component(vertices, faces, labels, c)
        points = Conservativeness_Points(V, F, collision.Manifest['seed'], c)
        hulls = [Hull_Of(collision.Pieces[i])[0] for i in pieces[c]]
        pruned, full = Union_Distances(points, hulls, pruned=True), Union_Distances(points, hulls, pruned=False)
        assert np.max(np.abs(pruned - full)) < CONST_AGREEMENT, c
        assert (pruned.max() > eps) == (full.max() > eps)


def test_a_union_missing_a_part_is_a_violation_for_both(setup):
    """
    Description:
        Removing the largest part of a component leaves original surface far outside: both
        computations report the same violation (> eps) and the same maximum.
    """

    collision, vertices, faces, labels, pieces = setup
    c = CONST_COMPONENTS[3]
    V, F = _Component(vertices, faces, labels, c)
    points = Conservativeness_Points(V, F, collision.Manifest['seed'], c)
    hulls = [Hull_Of(collision.Pieces[i])[0] for i in pieces[c]]
    hulls.pop(int(np.argmax([h.volume for h in hulls])))
    pruned, full = Union_Distances(points, hulls, pruned=True), Union_Distances(points, hulls, pruned=False)
    assert full.max() > collision.Manifest['eps']
    assert abs(pruned.max() - full.max()) < CONST_AGREEMENT


def test_the_recorded_maximum_is_the_true_maximum(setup):
    """
    Description:
        Recomputing every accepted CoACD component with its recorded seed gives exactly the recorded
        global maximum, per-component maxima and threshold counts: nothing is capped.
    """

    collision, vertices, faces, labels, pieces = setup
    record = collision.Manifest['conservativeness']
    eps = collision.Manifest['eps']
    worst, above_09, above_099, points_total = 0.0, 0, 0, 0
    largest = {r['component']: r['max_m'] for r in record['largest']}
    for c, ids in pieces.items():
        V, F = _Component(vertices, faces, labels, c)
        points = Conservativeness_Points(V, F, collision.Manifest['seed'], c)
        d = Union_Distances(points, [Hull_Of(collision.Pieces[i])[0] for i in ids])
        worst = max(worst, float(d.max()))
        above_09 += int(np.sum(d > 0.9 * eps))
        above_099 += int(np.sum(d > 0.99 * eps))
        points_total += len(points)
        if c in largest:
            assert abs(float(d.max()) - largest[c]) < CONST_AGREEMENT
    assert abs(worst - record['max_violation_m']) < CONST_AGREEMENT
    assert worst <= eps
    assert above_09 == record['points_above_0.9_eps'] and above_099 == record['points_above_0.99_eps']
    assert points_total == record['points_checked']
