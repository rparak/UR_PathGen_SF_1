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
from scipy.spatial import ConvexHull, QhullError
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


def Union_Violation(points: np.ndarray, hulls: tp.List[ConvexHull], eps: float, trimesh: tp.Any) -> float:
    """
    Description:
        How far the worst point lies outside the union of convex pieces, decided exactly where it
        matters. A point is inside a piece when every facet plane has it on the inner side. For a
        point outside, the Euclidean distance to a piece is at least its largest facet distance, so
        only pieces whose largest facet distance is <= eps can hold it within eps; the exact
        distance is computed for those pairs only. Stops at the first point beyond eps.

    Args:
        (1) points [Matrix<float> Nx3]: The points (m).
        (2) hulls [List<ConvexHull>]: The pieces.
        (3) eps [float]: The tolerance (m).
        (4) trimesh [module]: The trimesh module (injected; imported only by the build).

    Returns:
        (1) parameter [float]: The largest outside distance found (m); a value > eps is a lower
                               bound of the first violating point's distance.
    """

    if not hulls:
        return float('inf')
    facet = np.stack([np.max(points @ h.equations[:, :3].T + h.equations[:, 3], axis=1) for h in hulls], axis=1)
    best = np.min(facet, axis=1)
    worst = 0.0
    meshes: tp.Dict[int, tp.Any] = {}
    for i in np.nonzero(best > 0.0)[0]:
        candidates = np.nonzero(facet[i] <= eps)[0]
        distance = float('inf')
        for k in candidates:
            if k not in meshes:
                meshes[k] = trimesh.Trimesh(hulls[k].points, hulls[k].simplices, process=False)
            _, d, _ = trimesh.proximity.closest_point(meshes[k], points[i:i + 1])
            distance = min(distance, float(d[0]))
            if distance <= eps:
                break
        if distance > eps:
            return max(distance if np.isfinite(distance) else float(best[i]), float(best[i]))
        worst = max(worst, distance)
    return worst


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


def Build_Collision_Set(vertices: np.ndarray, faces: np.ndarray, centre: tp.Sequence[float], radius: float,
                        stand_min: tp.Sequence[float], stand_max: tp.Sequence[float], hull_excess_max: float,
                        eps: float, seed: int,
                        progress_cb: tp.Optional[tp.Callable[[str], None]] = None) -> Collision_Set_Str:
    """
    Description:
        Build the collision set (SPEC 7): crop the components to the reach sphere, hull each one
        (thickening degenerate ones), and decompose with CoACD only a component whose hull excess
        exceeds hull_excess_max; CoACD parts that fail the conservativeness check fall back to
        the hull. Every measurement is recorded in the manifest.

    Args:
        (1 - 2) vertices, faces [Matrix<float> Nx3, Matrix<int> Mx3]: The environment STL (world frame).
        (3 - 4) centre, radius [Vector<float> 1x3, float]: The crop sphere (m).
        (5 - 6) stand_min, stand_max [Vector<float> 1x3]: The stand-selection box (m).
        (7) hull_excess_max [float]: Excess above which a hull is decomposed (m).
        (8) eps [float]: Conservativeness tolerance (m).
        (9) seed [int]: Seed of every random sample and of CoACD.
        (10) progress_cb [function]: Optional sink for progress lines.

    Returns:
        (1) parameter [Collision_Set_Str(object)]: The set (Manifest without the source fields,
                                                   which the writer adds).
    """

    # Heavy runtimes, needed only here.
    import trimesh
    import coacd
    report = progress_cb if progress_cb is not None else (lambda message: None)
    rng = np.random.default_rng(seed)
    timing: tp.Dict[str, float] = {}

    # 1\ Components and the crop.
    t0 = time.time()
    n_components, face_label = Connected_Components(faces)
    order = np.argsort(face_label, kind='stable')
    bounds = np.searchsorted(face_label[order], np.arange(n_components + 1))
    centre_v = np.asarray(centre, dtype=float)
    kept = []
    for c in range(n_components):
        f = faces[order[bounds[c]:bounds[c + 1]]]
        used = np.unique(f)
        if np.min(np.linalg.norm(vertices[used] - centre_v, axis=1)) <= radius:
            kept.append((c, used, f))
    timing['components_and_crop'] = time.time() - t0
    report('%d components, %d kept inside the %.3f m crop sphere (%.1f s).'
           % (n_components, len(kept), radius, timing['components_and_crop']))

    # 2\ Hull, measure, decompose where needed.
    out = Collision_Set_Str()
    t_hull = t_coacd = t_check = 0.0
    coacd.set_log_level('error')
    stand_lo, stand_hi = np.asarray(stand_min), np.asarray(stand_max)
    conservative_points = 0
    conservative_worst = 0.0
    degenerate_kinds = {'planar': 0, 'collinear': 0, 'point': 0}
    for done, (c, used, f) in enumerate(kept):
        V = vertices[used]
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
        excess = Piece_Excess(hull, original, trimesh, rng)
        t_hull += time.time() - t
        pieces = [(hull, method, excess)]
        if method == 'hull' and excess > hull_excess_max:
            t = time.time()
            parts = coacd.run_coacd(coacd.Mesh(V, F), preprocess_mode='auto', seed=seed)
            hulls = [h for h in (Hull_Of(np.asarray(p[0], dtype=float))[0] for p in parts) if h is not None]
            t_coacd += time.time() - t
            t = time.time()
            test = np.concatenate([V, Sample_Surface(V, F, rng, CONST_SAMPLE_DENSITY, 0, None, per_triangle=True)], axis=0)
            worst = Union_Violation(test, hulls, eps, trimesh)
            conservative_points += len(test)
            t_check += time.time() - t
            if hulls and worst <= eps:
                conservative_worst = max(conservative_worst, worst)
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
                                     'are checked on their vertices and 1 sample/cm^2 (>= 1 per triangle)',
                             'points_checked': int(conservative_points),
                             'max_violation_m': float(conservative_worst),
                             'violations_above_eps': 0},
        'excess': {'max_m': float(excess.max()) if excess.size else 0.0,
                   'p95_m': float(np.percentile(excess, 95)) if excess.size else 0.0,
                   'largest': [{'piece': int(i), 'component': out.Components[i], 'method': out.Methods[i],
                                'body': out.Bodies[i], 'excess_m': float(excess[i]),
                                'bbox_min': [float(x) for x in out.Pieces[i].min(axis=0)],
                                'bbox_max': [float(x) for x in out.Pieces[i].max(axis=0)]} for i in top]},
        'flagged_open_shells': {'note': 'pieces whose excess exceeds hull_excess_max after fallback: their hull may '
                                        'fill space the robot could need (SPEC 7.4)',
                                'pieces': flagged},
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
