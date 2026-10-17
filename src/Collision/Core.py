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
# File Name: ../Collision/Core.py
## =========================================================================== ##

# System (Default)
import os
# Hashlib (Secure hashes)
import hashlib
# JSON (JSON encoder and decoder)
import json
# Time (Time access)
import time
# Typing (Support for type hints)
import typing as tp
# Dataclasses (Data Classes)
from dataclasses import dataclass, field
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# SciPy (Scientific computing) [pip3 install scipy]
from scipy.spatial import ConvexHull, QhullError, cKDTree
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components

"""
Description:
    Initialization of constants.

    The environment collision set: the only reader and writer of CAD/Collision/*.npz and its JSON
    manifest, and the only STL reader of the project. MuJoCo collides a mesh through its convex
    hull, so the workcell STL is cut into convex pieces: a hull per connected component, and a
    CoACD decomposition only where the hull is too coarse (SPEC 7). trimesh and coacd are imported
    only inside Build_Collision_Set: reading a committed set needs neither.
"""
# Thickening of a degenerate (planar or collinear) component on each side of it. The result
# contains the original, so the piece stays conservative and adds at most this much excess.
#   Unit [float] (m)
CONST_THICKEN = 0.0005
# A hull with a smaller volume is degenerate (float32 STL coordinates make a planar part slightly
# non-planar, so qhull may succeed with a sliver).
#   Unit [float] (m^3)
CONST_DEGENERATE_VOLUME = 1e-12
# A singular value below this (relative to the largest) marks a missing dimension of a component.
#   Unit [float] (-)
CONST_RANK_TOLERANCE = 1e-6
# Surface sampling density for the excess and conservativeness measurements (SPEC 7.3): 1 sample
# per cm^2. Excess sampling of one piece is clamped to [CONST_MIN_SAMPLES, CONST_MAX_SAMPLES].
#   Unit [float] (1/m^2), [int]
CONST_SAMPLE_DENSITY = 1.0e4
CONST_MIN_SAMPLES = 64
CONST_MAX_SAMPLES = 20000
# Body codes stored per piece.
#   Unit [Tuple<string>]
CONST_BODIES = ('env', 'env_stand')
# Piece methods: the component's hull; a thickened degenerate component's hull; a CoACD part; the
# hull of a component whose CoACD parts failed the conservativeness check.
#   Unit [Tuple<string>]
CONST_METHODS = ('hull', 'thickened', 'coacd', 'fallback')
# Point-triangle pairs evaluated per chunk by Point_Triangle_Distance (memory bound).
#   Unit [int]
CONST_DISTANCE_CHUNK = 2000000
# Number of largest-excess pieces listed in the manifest.
#   Unit [int]
CONST_TOP_EXCESS = 20


class Collision_Error_Cls(ValueError):
    """
    Description:
        The collision set is missing, unreadable or built from another STL. The message says to
        rebuild it with Example/build_collision.py.
    """


@dataclass
class Collision_Set_Str:
    """
    Description:
        A committed environment collision set: convex pieces in world coordinates.
    """

    # Vertices of each piece (MuJoCo hulls them).
    #   Unit [List<Matrix<float> Nx3>] (m)
    Pieces: tp.List[np.ndarray] = field(default_factory=list)
    # Body of each piece, one of CONST_BODIES.
    #   Unit [List<string>]
    Bodies: tp.List[str] = field(default_factory=list)
    # Source STL component of each piece.
    #   Unit [List<int>]
    Components: tp.List[int] = field(default_factory=list)
    # Method of each piece, one of CONST_METHODS.
    #   Unit [List<string>]
    Methods: tp.List[str] = field(default_factory=list)
    # Excess of each piece over the original surface.
    #   Unit [List<float>] (m)
    Excess: tp.List[float] = field(default_factory=list)
    # The manifest (build parameters, hashes, metrics).
    #   Unit [Dict]
    Manifest: tp.Dict = field(default_factory=dict)


def Sha256_File(path: str) -> str:
    """
    Description:
        The sha256 of a file, read in chunks.

    Args:
        (1) path [string]: The file.

    Returns:
        (1) parameter [string]: The hex digest.
    """

    digest = hashlib.sha256()
    with open(path, 'rb') as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()


def Read_Binary_STL(path: str) -> tp.Tuple[np.ndarray, np.ndarray]:
    """
    Description:
        Read a binary STL with shared vertices merged (exact float32 equality).

        Note:
            MuJoCo's own STL loader refuses more than 200 000 faces (SPEC 4.5); this reader has no
            such limit, and its result is passed to MuJoCo as uservert / userface.

    Args:
        (1) path [string]: The STL file.

    Returns:
        (1) parameter [Tuple<Matrix<float> Nx3, Matrix<int> Mx3>]: Vertices (m) and faces.
    """

    size = os.path.getsize(path)
    with open(path, 'rb') as handle:
        handle.read(80)
        count = int(np.frombuffer(handle.read(4), dtype='<u4')[0])
        if size != 84 + 50 * count:
            raise Collision_Error_Cls('%s is not a binary STL (%d bytes for %d triangles). Export it as binary STL.'
                                      % (path, size, count))
        record = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
        triangles = np.fromfile(handle, dtype=record, count=count)
    corners = np.ascontiguousarray(triangles['vertices'].reshape(-1, 3))
    keys = corners.view(np.dtype((np.void, corners.dtype.itemsize * 3))).ravel()
    _, first, inverse = np.unique(keys, return_index=True, return_inverse=True)
    return corners[first].astype(np.float64), inverse.reshape(-1, 3).astype(np.int64)


def Connected_Components(faces: np.ndarray) -> tp.Tuple[int, np.ndarray]:
    """
    Description:
        Label faces by connected component, where two faces are connected when they share a
        manifold edge (an edge of exactly two faces).

        Note:
            Not by a shared vertex, and not through a non-manifold edge (three or more faces): CAD
            parts that merely touch (the profiles of the robot stand meet along such edges) would
            merge into one component whose hull fills the whole frame. Measured: by shared vertex
            2270 components, by any shared edge 2428, by manifold edge 2756 (as trimesh splits, the
            count of SPEC 4.1). With the first two, the stand frame was one 21 440-face component
            whose fallback hull extended 297 mm beyond it.

    Args:
        (1) faces [Matrix<int> Mx3]: The faces.

    Returns:
        (1) parameter [Tuple<int, Vector<int> 1xM>]: Number of components and the label of each face.
    """

    n_faces = len(faces)
    edges = np.sort(np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]]), axis=1)
    _, edge_id, edge_count = np.unique(edges, axis=0, return_inverse=True, return_counts=True)
    edge_id = edge_id.ravel()
    face_id = np.tile(np.arange(n_faces), 3)
    manifold = edge_count[edge_id] == 2
    # Bipartite graph: faces 0..M-1, edges M..M+E-1; a face is linked to its manifold edges only.
    size = n_faces + len(edge_count)
    graph = coo_matrix((np.ones(int(manifold.sum()), dtype=np.int8), (face_id[manifold], n_faces + edge_id[manifold])),
                       shape=(size, size))
    _, labels = connected_components(graph, directed=False)
    _, compact = np.unique(labels[:n_faces], return_inverse=True)
    return int(compact.max()) + 1, compact


def Thicken_Degenerate(vertices: np.ndarray) -> tp.Tuple[np.ndarray, str]:
    """
    Description:
        Thicken a component that has no 3-D hull. The directions come from the SVD of its centred
        vertices, not from face normals: the STL winding is inconsistent, so normals can cancel.
        Planar: +-CONST_THICKEN along the plane normal. Collinear: +-CONST_THICKEN along two
        directions orthogonal to the line. A single point: along all three axes.

    Args:
        (1) vertices [Matrix<float> Nx3]: The component's vertices (m).

    Returns:
        (1) parameter [Tuple<Matrix<float> Kx3, string>]: The thickened point set and its kind
                                                           ('planar', 'collinear' or 'point').
    """

    centred = vertices - vertices.mean(axis=0)
    _, s, Vt = np.linalg.svd(centred, full_matrices=True)
    scale = s[0] if s.size and s[0] > 0.0 else 1.0
    rank = int(np.sum(s > CONST_RANK_TOLERANCE * scale)) if s.size and s[0] > 0.0 else 0
    if rank >= 2:
        directions, kind = [Vt[2]], 'planar'
    elif rank == 1:
        directions, kind = [Vt[1], Vt[2]], 'collinear'
    else:
        directions, kind = [np.eye(3)[0], np.eye(3)[1], np.eye(3)[2]], 'point'
    points = [vertices]
    for d in directions:
        points += [vertices + CONST_THICKEN * d, vertices - CONST_THICKEN * d]
    if kind != 'planar':
        # Corners of the thickened cross-section, so the hull is a box around the line or point.
        for a in (-1.0, 1.0):
            for b in (-1.0, 1.0):
                points.append(vertices + CONST_THICKEN * (a * directions[0] + b * directions[1]))
    return np.concatenate(points, axis=0), kind


def Hull_Of(points: np.ndarray) -> tp.Tuple[tp.Optional[ConvexHull], float]:
    """
    Description:
        The convex hull of a point set, or None when qhull cannot build a 3-D hull.

    Args:
        (1) points [Matrix<float> Nx3]: The points.

    Returns:
        (1) parameter [Tuple<ConvexHull or None, float>]: The hull and its volume (m^3).
    """

    if len(points) < 4:
        return None, 0.0
    try:
        hull = ConvexHull(points)
    except QhullError:
        return None, 0.0
    return hull, float(hull.volume)


def Sample_Surface(vertices: np.ndarray, faces: np.ndarray, rng: np.random.Generator, density: float,
                   minimum: int, maximum: tp.Optional[int], per_triangle: bool) -> np.ndarray:
    """
    Description:
        Area-uniform samples on a triangle surface.

    Args:
        (1 - 2) vertices, faces [Matrix<float> Nx3, Matrix<int> Mx3]: The surface.
        (3) rng [Generator]: Seeded random generator.
        (4) density [float]: Samples per m^2.
        (5 - 6) minimum, maximum [int]: Clamp of the count (maximum None = no clamp).
        (7) per_triangle [bool]: Put at least one sample on every triangle (its centroid).

    Returns:
        (1) parameter [Matrix<float> Kx3]: The samples (m).
    """

    tri = vertices[faces]
    area = 0.5 * np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1)
    total = float(area.sum())
    count = max(minimum, int(np.ceil(total * density)))
    if maximum is not None:
        count = min(count, maximum)
    samples = []
    if total > 0.0 and count > 0:
        index = rng.choice(len(faces), size=count, p=area / total)
        u, v = rng.random(count), rng.random(count)
        flip = u + v > 1.0
        u[flip], v[flip] = 1.0 - u[flip], 1.0 - v[flip]
        t = tri[index]
        samples.append(t[:, 0] + u[:, None] * (t[:, 1] - t[:, 0]) + v[:, None] * (t[:, 2] - t[:, 0]))
    if per_triangle:
        samples.append(tri.mean(axis=1))
    return np.concatenate(samples, axis=0) if samples else np.zeros((0, 3))


def Point_Triangle_Distance(points: np.ndarray, triangles: np.ndarray) -> np.ndarray:
    """
    Description:
        Exact Euclidean distance from each point to the nearest of a set of triangles (closest
        point on a triangle by its Voronoi regions, Ericson, Real-Time Collision Detection, 5.1.5).

    Args:
        (1) points [Matrix<float> Nx3]: The points (m).
        (2) triangles [Tensor<float> Mx3x3]: The triangles (m).

    Returns:
        (1) parameter [Vector<float> 1xN]: The distances (m).
    """

    result = np.full(len(points), np.inf)
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    ab, ac = b - a, c - a
    chunk = max(1, CONST_DISTANCE_CHUNK // max(1, len(triangles)))
    for start in range(0, len(points), chunk):
        p = points[start:start + chunk, None, :]
        ap, bp, cp = p - a, p - b, p - c
        d1, d2 = np.einsum('ijk,jk->ij', ap, ab), np.einsum('ijk,jk->ij', ap, ac)
        d3, d4 = np.einsum('ijk,jk->ij', bp, ab), np.einsum('ijk,jk->ij', bp, ac)
        d5, d6 = np.einsum('ijk,jk->ij', cp, ab), np.einsum('ijk,jk->ij', cp, ac)
        va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2
        with np.errstate(divide='ignore', invalid='ignore'):
            # Face region (default), then the edge and vertex regions override it.
            denom = va + vb + vc
            v, w = vb / denom, vc / denom
            closest = a + v[..., None] * ab + w[..., None] * ac
            # Edge BC.
            mask = (va <= 0.0) & ((d4 - d3) >= 0.0) & ((d5 - d6) >= 0.0)
            t = (d4 - d3) / ((d4 - d3) + (d5 - d6))
            closest = np.where(mask[..., None], b + t[..., None] * (c - b), closest)
            # Edge AC.
            mask = (vb <= 0.0) & (d2 >= 0.0) & (d6 <= 0.0)
            t = d2 / (d2 - d6)
            closest = np.where(mask[..., None], a + t[..., None] * ac, closest)
            # Edge AB.
            mask = (vc <= 0.0) & (d1 >= 0.0) & (d3 <= 0.0)
            t = d1 / (d1 - d3)
            closest = np.where(mask[..., None], a + t[..., None] * ab, closest)
        # Vertex regions.
        closest = np.where(((d6 >= 0.0) & (d5 <= d6))[..., None], c, closest)
        closest = np.where(((d3 >= 0.0) & (d4 <= d3))[..., None], b, closest)
        closest = np.where(((d1 <= 0.0) & (d2 <= 0.0))[..., None], a, closest)
        result[start:start + chunk] = np.min(np.linalg.norm(p - closest, axis=2), axis=1)
    return result


def Surface_Distance(points: np.ndarray, vertices: np.ndarray, faces: np.ndarray) -> np.ndarray:
    """
    Description:
        Exact distance from each point to a triangle surface, without testing every triangle: the
        nearest surface vertex gives an upper bound u; a triangle can be closer only if its centroid
        lies within u + its own circumradius (about the centroid), so only those are evaluated
        exactly with Point_Triangle_Distance.

    Args:
        (1) points [Matrix<float> Nx3]: The points (m).
        (2 - 3) vertices, faces [Matrix<float> Kx3, Matrix<int> Mx3]: The surface; vertices may be a
                                                            larger array that faces index into.

    Returns:
        (1) parameter [Vector<float> 1xN]: The distances (m).
    """

    triangles = vertices[faces]
    centroid = triangles.mean(axis=1)
    radius = np.max(np.linalg.norm(triangles - centroid[:, None, :], axis=2), axis=1)
    # Only the surface's own vertices bound the distance: the array may hold other surfaces too.
    upper, _ = cKDTree(vertices[np.unique(faces)]).query(points)
    tree = cKDTree(centroid)
    result = np.empty(len(points))
    for i, (point, bound) in enumerate(zip(points, upper)):
        near = np.asarray(tree.query_ball_point(point, bound + float(radius.max())), dtype=np.int64)
        near = near[np.linalg.norm(centroid[near] - point, axis=1) <= bound + radius[near]]
        result[i] = min(bound, float(Point_Triangle_Distance(point[None], triangles[near])[0])) if near.size else bound
    return result


def Union_Distances(points: np.ndarray, hulls: tp.List[ConvexHull], pruned: bool = True) -> np.ndarray:
    """
    Description:
        The exact distance of each point outside the union of convex pieces (0 inside). A point is
        inside a piece when no facet plane has it on the outer side. Outside every piece, its
        distance to the union is the minimum over the pieces of the exact distance to each piece's
        boundary triangles.

        With pruned=True a piece is skipped for a point when its largest facet distance (a lower
        bound of the Euclidean distance to that piece) is already >= the best distance found; this
        never changes the result. With pruned=False every piece is evaluated (the reference).

    Args:
        (1) points [Matrix<float> Nx3]: The points (m).
        (2) hulls [List<ConvexHull>]: The pieces.
        (3) pruned [bool]: Skip pieces that provably cannot be closer.

    Returns:
        (1) parameter [Vector<float> 1xN]: Outside distance per point (m); inf when there are no pieces.
    """

    if not hulls:
        return np.full(len(points), np.inf)
    bound = np.stack([np.max(points @ h.equations[:, :3].T + h.equations[:, 3], axis=1) for h in hulls], axis=1)
    distance = np.where(np.min(bound, axis=1) <= 0.0, 0.0, np.inf)
    outside = np.nonzero(distance > 0.0)[0]
    if outside.size == 0:
        return distance
    best = np.full(outside.size, np.inf)
    for k in np.argsort(np.min(bound[outside], axis=0)):
        todo = np.nonzero(bound[outside, k] < best)[0] if pruned else np.arange(outside.size)
        if todo.size == 0:
            continue
        triangles = hulls[k].points[hulls[k].simplices]
        best[todo] = np.minimum(best[todo], Point_Triangle_Distance(points[outside[todo]], triangles))
    distance[outside] = best
    return distance


def Conservativeness_Points(vertices: np.ndarray, faces: np.ndarray, seed: int, component: int) -> np.ndarray:
    """
    Description:
        The points a CoACD component is checked on: its vertices, plus area-uniform samples of its
        surface at CONST_SAMPLE_DENSITY with at least one per triangle (its centroid), from the
        generator seeded [seed, component]. A large flat triangle can leave a union of convex
        parts while its three vertices stay inside; the samples see that.

    Args:
        (1 - 2) vertices, faces [Matrix<float> Nx3, Matrix<int> Mx3]: The component.
        (3 - 4) seed, component [int]: The build seed and the component id.

    Returns:
        (1) parameter [Matrix<float> Kx3]: The points (m).
    """

    rng = np.random.default_rng([seed, component, 1])
    return np.concatenate([vertices, Sample_Surface(vertices, faces, rng, CONST_SAMPLE_DENSITY, 0, None,
                                                    per_triangle=True)], axis=0)


def Piece_Excess(hull: ConvexHull, original: tp.Any, trimesh: tp.Any, rng: np.random.Generator) -> float:
    """
    Description:
        Excess of one convex piece: the largest distance from its surface (vertices plus
        area-uniform samples) to the original component surface.

    Args:
        (1) hull [ConvexHull]: The piece.
        (2) original [trimesh.Trimesh]: The original component.
        (3) trimesh [module]: The trimesh module (injected).
        (4) rng [Generator]: Seeded random generator.

    Returns:
        (1) parameter [float]: The excess (m).
    """

    samples = Sample_Surface(hull.points, hull.simplices, rng, CONST_SAMPLE_DENSITY, CONST_MIN_SAMPLES,
                             CONST_MAX_SAMPLES, per_triangle=False)
    points = np.concatenate([hull.points[hull.vertices], samples], axis=0)
    _, d, _ = trimesh.proximity.closest_point(original, points)
    return float(np.max(d))


def Component_Overrides_Record(overrides: tp.Sequence[tp.Tuple[int, tp.Sequence[float], str]]) -> tp.List[tp.Dict]:
    """
    Description:
        The manifest form of the component overrides, sorted by component, so that the build and
        the check against the configuration compare the same thing.

    Args:
        (1) overrides [List<Tuple<int, Vector<float> 1x3, string>>]: (component, translation (m), source).

    Returns:
        (1) parameter [List<Dict>]: One record per override.
    """

    return [{'component': int(c), 'translation_m': [float(v) for v in t], 'source': str(src)}
            for c, t, src in sorted(overrides, key=lambda o: o[0])]


def Build_Collision_Set(vertices: np.ndarray, faces: np.ndarray, centre: tp.Sequence[float], radius: float,
                        stand_min: tp.Sequence[float], stand_max: tp.Sequence[float], hull_excess_max: float,
                        eps: float, seed: int,
                        overrides: tp.Sequence[tp.Tuple[int, tp.Sequence[float], str]],
                        progress_cb: tp.Optional[tp.Callable[[str], None]] = None) -> Collision_Set_Str:
    """
    Description:
        Build the collision set (SPEC 7): crop the components to the reach sphere, hull each one
        (thickening degenerate ones), and decompose with CoACD only a component whose hull excess
        exceeds hull_excess_max; CoACD parts that fail the conservativeness check fall back to
        the hull. An overridden component is translated rigidly before all of it, and checked
        against its translated surface. Every measurement is recorded in the manifest.

    Args:
        (1 - 2) vertices, faces [Matrix<float> Nx3, Matrix<int> Mx3]: The environment STL (world frame).
        (3 - 4) centre, radius [Vector<float> 1x3, float]: The crop sphere (m).
        (5 - 6) stand_min, stand_max [Vector<float> 1x3]: The stand-selection box (m).
        (7) hull_excess_max [float]: Excess above which a hull is decomposed (m).
        (8) eps [float]: Conservativeness tolerance (m).
        (9) seed [int]: Seed of every random sample and of CoACD.
        (10) overrides [List<Tuple<int, Vector<float> 1x3, string>>]: (component, translation (m), source).
        (11) progress_cb [function]: Optional sink for progress lines.

    Returns:
        (1) parameter [Collision_Set_Str(object)]: The set (Manifest without the source fields,
                                                   which the writer adds).
    """

    # Heavy runtimes, needed only here.
    import trimesh
    import coacd
    report = progress_cb if progress_cb is not None else (lambda message: None)
    timing: tp.Dict[str, float] = {}

    # 1\ Components and the crop.
    t0 = time.time()
    n_components, face_label = Connected_Components(faces)
    order = np.argsort(face_label, kind='stable')
    bounds = np.searchsorted(face_label[order], np.arange(n_components + 1))
    centre_v = np.asarray(centre, dtype=float)
    shift = {int(c): np.asarray(t, dtype=float) for c, t, _ in overrides}
    unknown = sorted(c for c in shift if not 0 <= c < n_components)
    if unknown:
        raise Collision_Error_Cls('Environment.Component_Overrides names component(s) %s, but the STL has components '
                                  '0 - %d. Fix the configuration.' % (unknown, n_components - 1))
    kept = []
    for c in range(n_components):
        f = faces[order[bounds[c]:bounds[c + 1]]]
        used = np.unique(f)
        if np.min(np.linalg.norm(vertices[used] + shift.get(c, 0.0) - centre_v, axis=1)) <= radius:
            kept.append((c, used, f))
    missing = sorted(c for c in shift if c not in {k[0] for k in kept})
    if missing:
        raise Collision_Error_Cls('Overridden component(s) %s lie outside the crop sphere; an override there has no '
                                  'effect. Remove it from Environment.Component_Overrides.' % missing)
    timing['components_and_crop'] = time.time() - t0
    report('%d components, %d kept inside the %.3f m crop sphere (%.1f s).'
           % (n_components, len(kept), radius, timing['components_and_crop']))

    # 2\ Hull, measure, decompose where needed.
    out = Collision_Set_Str()
    t_hull = t_coacd = t_check = 0.0
    coacd.set_log_level('error')
    stand_lo, stand_hi = np.asarray(stand_min), np.asarray(stand_max)
    checked: tp.List[tp.Dict] = []
    degenerate_kinds = {'planar': 0, 'collinear': 0, 'point': 0}
    for done, (c, used, f) in enumerate(kept):
        # Each component has its own vertex copy, so a translation never moves a neighbour.
        V = vertices[used] + shift.get(c, 0.0)
        remap = np.full(len(vertices), -1, dtype=np.int64)
        remap[used] = np.arange(len(used))
        F = remap[f]
        body = 'env_stand' if np.all(V.min(axis=0) >= stand_lo) and np.all(V.max(axis=0) <= stand_hi) else 'env'
        original = trimesh.Trimesh(V, F, process=False)
        t_component = t = time.time()
        hull, volume = Hull_Of(V)
        method = 'hull'
        if hull is None or volume < CONST_DEGENERATE_VOLUME:
            thick, kind = Thicken_Degenerate(V)
            degenerate_kinds[kind] += 1
            hull, volume = Hull_Of(thick)
            method = 'thickened'
        # One generator per component, so every component's samples are reproducible on their own.
        rng = np.random.default_rng([seed, int(c)])
        excess = Piece_Excess(hull, original, trimesh, rng)
        t_hull += time.time() - t
        pieces = [(hull, method, excess)]
        if method == 'hull' and excess > hull_excess_max:
            t = time.time()
            parts = coacd.run_coacd(coacd.Mesh(V, F), preprocess_mode='auto', seed=seed)
            hulls = [h for h in (Hull_Of(np.asarray(p[0], dtype=float))[0] for p in parts) if h is not None]
            t_coacd += time.time() - t
            t = time.time()
            test = Conservativeness_Points(V, F, seed, int(c))
            distance = Union_Distances(test, hulls)
            worst = float(np.max(distance))
            checked.append({'component': int(c), 'points': int(len(test)), 'max_m': worst,
                            'above_0.9_eps': int(np.sum(distance > 0.9 * eps)),
                            'above_0.99_eps': int(np.sum(distance > 0.99 * eps)),
                            'outside_points': int(np.sum(distance > 0.0)),
                            'p99_outside_m': float(np.percentile(distance[distance > 0.0], 99)) if np.any(distance > 0.0) else 0.0,
                            'accepted': bool(hulls and worst <= eps)})
            t_check += time.time() - t
            if hulls and worst <= eps:
                pieces = [(h, 'coacd', Piece_Excess(h, original, trimesh, rng)) for h in hulls]
            else:
                pieces = [(hull, 'fallback', excess)]
                report('Component %d: CoACD parts leave the original by %.2f mm (> eps); its hull is kept.'
                       % (c, worst * 1e3))
        for h, m, e in pieces:
            out.Pieces.append(np.asarray(h.points[h.vertices], dtype=float))
            out.Bodies.append(body)
            out.Components.append(int(c))
            out.Methods.append(m)
            out.Excess.append(float(e))
        spent = time.time() - t_component
        if spent > 10.0:
            report('Component %d (%d faces): %s, %d piece(s), %.1f s.' % (c, len(F), pieces[0][1], len(pieces), spent))
        if (done + 1) % 250 == 0:
            report('%d / %d components processed.' % (done + 1, len(kept)))
    timing.update({'hull_and_excess': t_hull, 'coacd': t_coacd, 'conservativeness_check': t_check})

    # 3\ Metrics.
    accepted = [r for r in checked if r['accepted']]
    excess = np.asarray(out.Excess)
    top = np.argsort(-excess)[:CONST_TOP_EXCESS]
    flagged = [i for i in range(len(excess)) if excess[i] > hull_excess_max]
    out.Manifest = {
        'crop_centre': [float(x) for x in centre_v], 'crop_radius': float(radius),
        'hull_excess_max': float(hull_excess_max), 'eps': float(eps), 'seed': int(seed),
        'coacd': {'version': getattr(coacd, '__version__', 'unknown'), 'preprocess_mode': 'auto', 'other': 'defaults'},
        'sample_density_per_m2': CONST_SAMPLE_DENSITY, 'thicken': CONST_THICKEN,
        'components_total': int(n_components), 'components_kept': len(kept),
        'degenerate_components': degenerate_kinds,
        'pieces': len(out.Pieces),
        'pieces_by_method': {m: out.Methods.count(m) for m in CONST_METHODS},
        'pieces_by_body': {b: out.Bodies.count(b) for b in CONST_BODIES},
        'conservativeness': {'note': 'hull and thickened pieces are conservative by convexity; CoACD components '
                                     'are checked on their vertices and 1 sample/cm^2 (>= 1 per triangle, seed '
                                     '[seed, component]) with the exact distance to the union of their parts',
                             'points_checked': int(sum(r['points'] for r in accepted)),
                             'max_violation_m': float(max((r['max_m'] for r in accepted), default=0.0)),
                             'points_above_0.9_eps': int(sum(r['above_0.9_eps'] for r in accepted)),
                             'points_above_0.99_eps': int(sum(r['above_0.99_eps'] for r in accepted)),
                             'violations_above_eps': 0,
                             'largest': sorted(accepted, key=lambda r: -r['max_m'])[:CONST_TOP_EXCESS // 2],
                             'rejected': [r for r in checked if not r['accepted']]},
        'excess': {'max_m': float(excess.max()) if excess.size else 0.0,
                   'p95_m': float(np.percentile(excess, 95)) if excess.size else 0.0,
                   'largest': [{'piece': int(i), 'component': out.Components[i], 'method': out.Methods[i],
                                'body': out.Bodies[i], 'excess_m': float(excess[i]),
                                'bbox_min': [float(x) for x in out.Pieces[i].min(axis=0)],
                                'bbox_max': [float(x) for x in out.Pieces[i].max(axis=0)]} for i in top]},
        'flagged_open_shells': {'note': 'pieces whose excess exceeds hull_excess_max after fallback: their hull may '
                                        'fill space the robot could need (SPEC 7.4)',
                                'pieces': flagged},
        'component_overrides': Component_Overrides_Record(overrides),
        'runtime_s': {k: round(v, 2) for k, v in timing.items()},
    }
    return out


def Write_Collision_Set(collision_set: Collision_Set_Str, npz_path: str, source_stl: str, project_root: str) -> str:
    """
    Description:
        Write the set (.npz) and its manifest (.json beside it), recording the source STL's path
        (repository-relative) and sha256.

    Args:
        (1) collision_set [Collision_Set_Str(object)]: The set.
        (2) npz_path [string]: The .npz path.
        (3) source_stl [string]: The STL it was built from.
        (4) project_root [string]: The repository root, for the relative path.

    Returns:
        (1) parameter [string]: The manifest path.
    """

    counts = np.array([len(p) for p in collision_set.Pieces], dtype=np.int64)
    np.savez_compressed(npz_path, vertices=np.concatenate(collision_set.Pieces, axis=0), counts=counts,
                        bodies=np.array([CONST_BODIES.index(b) for b in collision_set.Bodies], dtype=np.int8),
                        components=np.array(collision_set.Components, dtype=np.int64),
                        methods=np.array([CONST_METHODS.index(m) for m in collision_set.Methods], dtype=np.int8),
                        excess=np.array(collision_set.Excess, dtype=np.float64))
    manifest = dict(collision_set.Manifest)
    manifest['source_stl'] = os.path.relpath(source_stl, project_root)
    manifest['source_sha256'] = Sha256_File(source_stl)
    manifest_path = os.path.splitext(npz_path)[0] + '.json'
    with open(manifest_path, 'w', encoding='utf-8') as handle:
        json.dump(manifest, handle, indent=2)
        handle.write('\n')
    return manifest_path


def Read_Collision_Set(npz_path: str, project_root: str) -> Collision_Set_Str:
    """
    Description:
        Read a committed set and refuse it if its source STL has changed since it was built.

    Args:
        (1) npz_path [string]: The .npz path.
        (2) project_root [string]: The repository root (the manifest's paths are relative to it).

    Returns:
        (1) parameter [Collision_Set_Str(object)]: The set.
    """

    manifest_path = os.path.splitext(npz_path)[0] + '.json'
    if not os.path.isfile(npz_path) or not os.path.isfile(manifest_path):
        raise Collision_Error_Cls('The collision set %s (or its manifest) is missing. Build it with: '
                                  '.venv/bin/python Example/build_collision.py' % npz_path)
    with open(manifest_path, 'r', encoding='utf-8') as handle:
        manifest = json.load(handle)
    stl = os.path.join(project_root, manifest['source_stl'])
    if not os.path.isfile(stl) or Sha256_File(stl) != manifest['source_sha256']:
        raise Collision_Error_Cls('The collision set %s was built from another %s. Rebuild it with: '
                                  '.venv/bin/python Example/build_collision.py' % (npz_path, manifest['source_stl']))
    data = np.load(npz_path)
    split = np.cumsum(data['counts'])[:-1]
    return Collision_Set_Str(Pieces=np.split(data['vertices'], split),
                             Bodies=[CONST_BODIES[i] for i in data['bodies']],
                             Components=[int(c) for c in data['components']],
                             Methods=[CONST_METHODS[i] for i in data['methods']],
                             Excess=[float(e) for e in data['excess']], Manifest=manifest)
