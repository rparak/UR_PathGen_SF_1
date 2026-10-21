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
# File Name: ../Example/make_readme_figures.py
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
# Dataclasses (Data Classes)
import dataclasses
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# MuJoCo (Physics simulator) [pip3 install mujoco]
import mujoco
# Matplotlib (Visualization) [pip3 install matplotlib], drawn without a display
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.lines
import matplotlib.patches
# Custom Lib.:
#   ../Configuration/Core — the cell configuration
from Configuration.Core import Load_Configuration, Cell_Configuration_Str, Configuration_Error_Cls, CONST_DEFAULT_CONFIG_FILE, CONST_PROJECT_ROOT
#   ../URScript/Core — a variant file that cannot be read
from URScript.Core import URScript_Error_Cls
#   ../Kinematics/Core — the TCP of a configuration
from Kinematics.Core import TCP_Pose, Kinematics_Error_Cls
#   ../Scene/Core — the scene, its station-zone pieces and the collision geometry group
from Scene.Core import Scene_Cls, CONST_ADDED_BOX_PREFIX, CONST_COLLISION_GROUP
#   ../Example/plan_rotation — the variants and the gripper openings of the case
from plan_rotation import CONST_VARIANT_DIR, CONST_VARIANTS, Opening
#   ../Example/replay_rotation — the moves of a variant, their samples and their timing, as replayed
from replay_rotation import Load_Variant, Build_Moves, Move_Samples, Frames, Frame_Configuration, Write_PNG
#   ../Example/write_variants — the comparison table
from write_variants import CONST_TABLE_CSV, CONST_SECTIONS, Read_Table_CSV, Section_Of

"""
Description:
    Initialization of constants.
"""
# The README's image folder.
#   Unit [string]
CONST_MEDIA_DIR = os.path.join(CONST_PROJECT_ROOT, 'Media')
# The variants in the order of the comparison table, and the short status shown under each name.
#   Unit [Tuple<string>], [Dict<string, string>]
CONST_ORDER = tuple(CONST_VARIANTS)
CONST_SHORT = {'taught': 'reference', 'b_optimised': 'previous optimisation', 'c1': 'delivered',
               'c2': 'proposal *', 'c3': 'proposal *'}
CONST_PROPOSAL_NOTE = '* proposal for re-teaching, not verified on the robot'
CONST_ESTIMATE_NOTE = 'Simulation estimate (MuJoCo, nominal DH; r = 0, gripper time not modelled). Simulation passed ≠ safe on the robot.'
# The page themes a graph is written for (GitHub's light and dark page colours) and their ink.
#   Unit [Dict<string, Dict<string, string>>] (hex colour)
CONST_THEMES = {
    'light': {'surface': '#ffffff', 'text': '#1f2328', 'muted': '#59636e', 'grid': '#d1d9e0', 'axis': '#9198a1'},
    'dark': {'surface': '#0d1117', 'text': '#f0f6fc', 'muted': '#9198a1', 'grid': '#30363d', 'axis': '#656c76'},
}
# One colour per variant in fixed order: slots 1 - 5 of the dataviz reference palette, light and
# dark steps (an adjacent-pair colour-vision check passes in both modes, per that palette's record).
#   Unit [Dict<string, Dict<string, string>>] (hex colour)
CONST_VARIANT_COLORS = {
    'light': dict(zip(CONST_ORDER, ('#2a78d6', '#eb6834', '#1baf7a', '#eda100', '#e87ba4'))),
    'dark': dict(zip(CONST_ORDER, ('#3987e5', '#d95926', '#199e70', '#c98500', '#d55181'))),
}
# The stacked sections of the cycle-time graph: S1 - S3 on a one-hue ordinal ramp, the taught
# station operations (drop + pick) in grey.
#   Unit [Tuple<string>], [Dict<string, Tuple<string>>] (hex colour)
CONST_STACK = ('S1', 'S2', 'S3', 'station')
CONST_STACK_COLORS = {'light': ('#86b6ef', '#3987e5', '#1c5cab', '#b8b7b0'),
                      'dark': ('#1c5cab', '#3987e5', '#86b6ef', '#5f5e5a')}
# The joints, in order.
#   Unit [Tuple<string>]
CONST_JOINTS = ('base', 'shoulder', 'elbow', 'wrist 1', 'wrist 2', 'wrist 3')
# Graph size and resolution.
#   Unit [float] (dots per inch)
CONST_DPI = 150.0
# The renders show the environment as its collision model only (the convex pieces at their corrected
# positions and the added boxes), never the visual STL: the STL is the original CAD without the
# corrections (Visual_Mesh_Warning) and would show the robot apparently inside the machine. Size,
# backdrop per page theme, headlight (the only light; no specular, so every surface is matte), the
# radius of a variant's path and of the taught path drawn thin for reference, and the cameras
# (look-at point in the world frame, distance, azimuth, elevation) shared by several renders.
#   Unit [int] (px), [Vector<float> 1x3] (0 - 1), [float] (m), [Tuple] (m, m, deg, deg)
CONST_RENDER_SIZE = (1280, 800)
CONST_RENDER_BACKGROUND = {'light': (0.965, 0.965, 0.957), 'dark': (0.086, 0.106, 0.133)}
CONST_HEADLIGHT = ((0.40, 0.40, 0.40), (0.75, 0.75, 0.75), (0.0, 0.0, 0.0))
CONST_PATH_RADIUS, CONST_REFERENCE_RADIUS = 0.008, 0.004
CONST_CAMERA_CELL = ((-0.55, 0.0, 0.85), 3.8, -135.0, -22.0)
CONST_CAMERA_PATH = ((-0.40, 0.0, 1.55), 3.1, -150.0, -6.0)
# The colours of the collision model: the environment one neutral grey; the station-zone pieces (the
# holder boxes and the CAD pieces that meet the zone) orange where a render highlights them; the
# robot's and gripper's collision hulls red where shown; in the collision-model figure each group in
# its own colour and the zone box translucent orange.
#   Unit [Vector<float> 1x3] (0 - 1), [Vector<float> 1x4] (0 - 1)
CONST_ENV_RGB = (0.66, 0.68, 0.71)
CONST_ZONE_RGBA = (0.92, 0.41, 0.20, 1.0)
CONST_ZONE_BOX_RGBA = (0.92, 0.41, 0.20, 0.18)
CONST_ROBOT_COLLISION_RGBA = (0.85, 0.22, 0.22, 0.45)
CONST_ROBOT_GROUP_RGBA = (0.85, 0.22, 0.22, 1.0)
CONST_STAND_RGB = (0.42, 0.45, 0.50)
CONST_OTHER_RGB = (0.78, 0.79, 0.80)
# The collision-model figure's name of each Environment.Assembly_Overrides object, and its colour;
# an object not listed is named by its configuration name, in CONST_ENV_RGB.
#   Unit [Dict<string, Tuple<string, Vector<float> 1x3>>] (-, 0 - 1)
CONST_ASSEMBLY_GROUPS = {'machine_a': ('machine (a)', (0.52, 0.64, 0.80)),
                         'side_cabinet': ('side cabinet (c)', (0.66, 0.56, 0.80))}
# The status in the title of a path render.
#   Unit [Dict<string, string>]
CONST_RENDER_STATUS = {'taught': 'the taught program (reference)', 'b_optimised': 'previous optimisation, superseded by c1',
                       'c1': 'delivered', 'c2': 'proposal — not verified on the robot',
                       'c3': 'proposal — not verified on the robot'}
# At most this many capsules per drawn path (the path is thinned evenly).
#   Unit [int]
CONST_PATH_CAPSULES = 600


@dataclasses.dataclass
class Render_Str:
    """
    Description:
        One MuJoCo render of the README, written for the light and the dark page theme.
    """

    # The point of c1 the robot stands at.
    #   Unit [string]
    Point: str = 'home'
    # The camera: look-at point (world frame), distance, azimuth, elevation.
    #   Unit [Tuple<Vector<float> 1x3, float, float, float>] (m, m, deg, deg)
    Camera: tp.Tuple = CONST_CAMERA_CELL
    # The environment's opacity.
    #   Unit [float] (0 - 1)
    Opacity: float = 1.0
    # How the environment and the robot are coloured: 'zone' (grey, the station-zone pieces orange,
    # the robot by its visual meshes), 'neutral' (all grey), 'groups' (the collision-model figure:
    # each group in its colour, the zone box, the robot by its collision hulls only).
    #   Unit [string]
    Colouring: str = 'zone'
    # Whether the robot's collision hulls are drawn over its visual meshes (translucent red).
    #   Unit [bool]
    Robot_Collision: bool = False
    # The TCP paths drawn: (variant, radius), each in the variant's graph colour.
    #   Unit [Tuple<Tuple<string, float>>] (-, m)
    Paths: tp.Tuple = ()
    # The variant a path render is about ('' for none); a framed render gets a title, a legend and
    # the estimate note, the others are the bare image.
    #   Unit [string], [bool]
    Variant: str = ''
    Framed: bool = False


# The renders, by file name.
#   Unit [Dict<string, Render_Str>]
CONST_RENDERS = {
    'cell_overview': Render_Str(),
    'robot_home': Render_Str(Camera=((-0.10, 0.0, 1.25), 2.3, -150.0, -15.0)),
    'station_collision': Render_Str(Point='Waypoint_5', Camera=((-0.69, -0.27, 1.20), 0.8, 160.0, -25.0), Opacity=0.35,
                                    Robot_Collision=True),
    'collision_model': Render_Str(Colouring='groups', Framed=True),
    'path_taught': Render_Str(Camera=CONST_CAMERA_PATH, Colouring='neutral', Paths=(('taught', CONST_PATH_RADIUS),),
                              Variant='taught', Framed=True),
}
CONST_RENDERS.update({'path_%s' % v: Render_Str(Camera=CONST_CAMERA_PATH, Colouring='neutral',
                                                Paths=(('taught', CONST_REFERENCE_RADIUS), (v, CONST_PATH_RADIUS)),
                                                Variant=v, Framed=True)
                      for v in CONST_ORDER[1:]})


@dataclasses.dataclass
class Variant_Data_Str:
    """
    Description:
        What the graphs and renders draw of one variant, from its files as the replay builds them.
    """

    # The variant's points.
    #   Unit [Dict<string, Robot_Point_Str>]
    Points: tp.Dict = dataclasses.field(default_factory=dict)
    # The TCP over the checked samples of every move, in the world frame.
    #   Unit [Matrix<float> Nx3] (m)
    TCP: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros((0, 3)))
    # The joints over the estimated time (Frames, Frame_Configuration).
    #   Unit [Vector<float> 1xN] (s), [Matrix<float> Nx6] (rad)
    Time: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(0))
    Joints: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros((0, 6)))
    # Each section of the sequence (CONST_SECTIONS: S1, drop, S2, pick, S3) with its start and end
    # on the estimated time.
    #   Unit [List<Tuple<string, float, float>>] (-, s, s)
    Sections: tp.List[tp.Tuple[str, float, float]] = dataclasses.field(default_factory=list)


def Load_Variant_Data(variant: str, cfg: Cell_Configuration_Str, scenes: tp.Dict[str, Scene_Cls]) -> Variant_Data_Str:
    """
    Description:
        The TCP path and the joint timeline of a written variant: its moves as the replay builds them
        (Build_Moves), the TCP of every checked sample (Move_Samples, in the scene of the move's
        gripper state, as comparison.csv measures it), the joints of every replay frame at the
        estimated real time (Frames at speed 1, Frame_Configuration) and the time span of each
        section (Section_Of, as comparison.csv divides the sequence).

    Args:
        (1) variant [string]: The variant (CONST_VARIANTS).
        (2) cfg [Cell_Configuration_Str(object)]: The cell configuration (the TCP).
        (3) scenes [Dict<string, Scene_Cls>]: The scene of each gripper state.

    Returns:
        (1) parameter [Variant_Data_Str(object)]: The data.
    """

    operations, points = Load_Variant(os.path.join(CONST_VARIANT_DIR, variant))
    moves = Build_Moves(operations, points, cfg)
    frame = scenes['holding'].Base_Frame()
    tcp = [(frame @ TCP_Pose(q, cfg.TCP_Offset))[:3, 3] for m in moves if m.Kind != 'gripper' for q in Move_Samples(scenes[m.State], m)]
    starts = np.cumsum([0.0] + [m.Duration for m in moves])
    timeline = [(starts[k] + t, Frame_Configuration(moves[k], t)) for k, t in Frames(moves, 1.0) if moves[k].Kind != 'gripper']
    q_0 = np.asarray(points[operations[0].Target].Q, dtype=float)
    of = Section_Of(moves)
    sections = [(name, float(min(starts[k] for k in range(len(moves)) if of[k] == name)),
                 float(max(starts[k + 1] for k in range(len(moves)) if of[k] == name))) for name, _ in CONST_SECTIONS]
    return Variant_Data_Str(points, np.asarray(tcp), np.array([0.0] + [t for t, _ in timeline]),
                            np.array([q_0] + [q for _, q in timeline]), sections)


def Style_Axes(ax: tp.Any, theme: tp.Dict[str, str], grid_axis: str) -> None:
    """
    Description:
        The common axes style: the theme's surface and ink, a hairline grid on one axis, no top or
        right spine.

    Args:
        (1) ax [Axes(object)]: The axes.
        (2) theme [Dict<string, string>]: The theme (CONST_THEMES).
        (3) grid_axis [string]: 'x', 'y', 'both' or '' (no grid).
    """

    ax.set_facecolor(theme['surface'])
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(theme['axis'])
    ax.tick_params(colors=theme['muted'], labelcolor=theme['text'], labelsize=9)
    ax.xaxis.label.set_color(theme['muted'])
    ax.yaxis.label.set_color(theme['muted'])
    if grid_axis:
        ax.grid(True, axis=grid_axis, color=theme['grid'], linewidth=0.8)
        ax.set_axisbelow(True)


def New_Figure(size: tp.Tuple[float, float], theme: tp.Dict[str, str], title: str, subtitle: str,
               rows: int = 1, columns: int = 1, **kwargs: tp.Any) -> tp.Tuple[tp.Any, tp.Any]:
    """
    Description:
        A figure on the theme's surface with a left-aligned title and subtitle.

    Args:
        (1) size [Tuple<float, float>]: Width and height (inch).
        (2) theme [Dict<string, string>]: The theme (CONST_THEMES).
        (3) title [string]: The title.
        (4) subtitle [string]: The line under it.
        (5) rows, columns [int]: The subplot grid.
        (6) kwargs [Dict]: Passed to plt.subplots.

    Returns:
        (1) parameter [Tuple<Figure, Axes or Array<Axes>>]: The figure and its axes.
    """

    fig, axes = plt.subplots(rows, columns, figsize=size, dpi=CONST_DPI, facecolor=theme['surface'], **kwargs)
    fig.text(0.015, 0.985, title, ha='left', va='top', fontsize=13, fontweight='bold', color=theme['text'])
    fig.text(0.015, 0.985 - 0.32 / size[1], subtitle, ha='left', va='top', fontsize=9, color=theme['muted'])
    return fig, axes


def Footnote(fig: tp.Any, theme: tp.Dict[str, str], lines: tp.Sequence[str]) -> None:
    """
    Description:
        Muted footnote lines at the bottom left of a figure.

    Args:
        (1) fig [Figure(object)]: The figure.
        (2) theme [Dict<string, string>]: The theme (CONST_THEMES).
        (3) lines [List<string>]: The lines, top to bottom.
    """

    fig.text(0.015, 0.012, '\n'.join(lines), ha='left', va='bottom', fontsize=7.5, color=theme['muted'], linespacing=1.5)


def Save_Figure(fig: tp.Any, path: str) -> None:
    """
    Description:
        Write a figure as PNG without the Software metadata, so the bytes depend on the data only,
        and close it.

    Args:
        (1) fig [Figure(object)]: The figure.
        (2) path [string]: The file.
    """

    fig.savefig(path, dpi=CONST_DPI, facecolor=fig.get_facecolor(), metadata={'Software': None})
    plt.close(fig)


def Value(table: tp.Dict, variant: str, section: str, column: str) -> float:
    """
    Description:
        One number of comparison.csv.

    Args:
        (1) table [Dict<string, Dict<string, Dict<string, string>>>]: The table (Read_Table_CSV).
        (2) variant, section, column [string]: Its row and column.

    Returns:
        (1) parameter [float]: The value.
    """

    return float(table[variant][section][column])


def Variant_Labels() -> tp.List[str]:
    """
    Description:
        The tick label of each variant: its name and short status.

    Returns:
        (1) parameter [List<string>]: The labels in CONST_ORDER.
    """

    return ['%s\n%s' % (v, CONST_SHORT[v]) for v in CONST_ORDER]


def Graph_Cycle_Time(table: tp.Dict, mode: str) -> tp.Any:
    """
    Description:
        The estimated home -> home time per variant, stacked by section: S1, S2, S3 and the taught
        station operations (drop + pick), with the total and its change against the taught program.

    Args:
        (1) table [Dict]: comparison.csv (Read_Table_CSV).
        (2) mode [string]: 'light' or 'dark'.

    Returns:
        (1) parameter [Figure(object)]: The figure.
    """

    theme = CONST_THEMES[mode]
    fig, ax = New_Figure((8.0, 4.8), theme, 'Estimated home → home time per variant',
                         'Case 17 of the SF1 program, stacked by section')
    x = np.arange(len(CONST_ORDER))
    bottom = np.zeros(len(CONST_ORDER))
    for section, color in zip(CONST_STACK, CONST_STACK_COLORS[mode]):
        parts = ('drop', 'pick') if section == 'station' else (section,)
        values = np.array([sum(Value(table, v, p, 't_est_s') for p in parts) for v in CONST_ORDER])
        ax.bar(x, values, 0.62, bottom=bottom, color=color, edgecolor=theme['surface'], linewidth=1.5,
               label='station (drop + pick, taught)' if section == 'station' else section)
        bottom += values
    taught = Value(table, 'taught', 'total', 't_est_s')
    for k, v in enumerate(CONST_ORDER):
        total = Value(table, v, 'total', 't_est_s')
        ax.text(k, total + 0.5, '%.2f s' % total, ha='center', va='bottom', fontsize=10, fontweight='bold', color=theme['text'])
        if v != 'taught':
            change = 100.0 * (total - taught) / taught
            ax.text(k, total + 1.9, '%s%.1f %% vs taught' % ('−' if change < 0 else '+', abs(change)), ha='center', va='bottom',
                    fontsize=8.5, color=theme['muted'])
    Style_Axes(ax, theme, 'y')
    ax.set_xticks(x, Variant_Labels())
    ax.set_ylabel('estimated time [s]')
    ax.set_ylim(0, taught * 1.22)
    legend = ax.legend(loc='upper right', ncol=2, fontsize=8.5, frameon=False, labelcolor=theme['text'])
    legend.set_title(None)
    fig.subplots_adjust(left=0.08, right=0.98, top=0.84, bottom=0.2)
    Footnote(fig, theme, [CONST_PROPOSAL_NOTE, CONST_ESTIMATE_NOTE])
    return fig


def Graph_Path_Length(table: tp.Dict, mode: str) -> tp.Any:
    """
    Description:
        The joint travel (sum |dq| over the six joints) and the TCP path length per variant, home -> home,
        side by side on their own axes.

    Args:
        (1) table [Dict]: comparison.csv (Read_Table_CSV).
        (2) mode [string]: 'light' or 'dark'.

    Returns:
        (1) parameter [Figure(object)]: The figure.
    """

    theme = CONST_THEMES[mode]
    fig, axes = New_Figure((8.0, 4.0), theme, 'Joint travel and TCP path length per variant',
                           'Home → home; the TCP path over the checked samples', 1, 2)
    colors = [CONST_VARIANT_COLORS[mode][v] for v in CONST_ORDER]
    y = np.arange(len(CONST_ORDER))[::-1]
    for ax, column, label, unit in ((axes[0], 'sum_abs_dq_rad', 'joint travel Σ|Δq| [rad]', '%.1f'),
                                    (axes[1], 'tcp_length_m', 'TCP path length [m]', '%.2f')):
        values = [Value(table, v, 'total', column) for v in CONST_ORDER]
        ax.barh(y, values, 0.62, color=colors)
        for yy, value in zip(y, values):
            ax.text(value + max(values) * 0.02, yy, unit % value, va='center', fontsize=9, color=theme['text'])
        Style_Axes(ax, theme, 'x')
        ax.set_xlim(0, max(values) * 1.18)
        ax.set_xlabel(label)
        ax.set_yticks(y, Variant_Labels() if ax is axes[0] else [''] * len(y))
    fig.subplots_adjust(left=0.2, right=0.97, top=0.82, bottom=0.2, wspace=0.12)
    Footnote(fig, theme, [CONST_PROPOSAL_NOTE, CONST_ESTIMATE_NOTE])
    return fig


def Graph_Max_Height(table: tp.Dict, mode: str, cfg: Cell_Configuration_Str) -> tp.Any:
    """
    Description:
        The highest robot point (world z, the gripper included) per variant, home -> home, against the
        robot base height and the top of the workspace box that generated segments keep.

    Args:
        (1) table [Dict]: comparison.csv (Read_Table_CSV).
        (2) mode [string]: 'light' or 'dark'.
        (3) cfg [Cell_Configuration_Str(object)]: The cell configuration (base height, workspace).

    Returns:
        (1) parameter [Figure(object)]: The figure.
    """

    theme = CONST_THEMES[mode]
    fig, ax = New_Figure((8.0, 3.9), theme, 'Highest robot point per variant',
                         'World z of the highest collision vertex of the robot and gripper, home → home')
    y = np.arange(len(CONST_ORDER))[::-1]
    values = [Value(table, v, 'total', 'max_robot_z_m') for v in CONST_ORDER]
    ax.barh(y, values, 0.62, color=[CONST_VARIANT_COLORS[mode][v] for v in CONST_ORDER])
    column = cfg.Workspace_Max[2] + 0.08
    for yy, value in zip(y, values):
        ax.text(column, yy, '%.3f m' % value, va='center', fontsize=9, color=theme['text'])
    for z, text, align in ((cfg.Base_Position[2], 'robot base %.3f m' % cfg.Base_Position[2], 'center'),
                           (cfg.Workspace_Max[2], 'workspace top %.2f m (generated segments)' % cfg.Workspace_Max[2], 'right')):
        ax.axvline(z, color=theme['muted'], linewidth=1.0, linestyle=(0, (4, 3)))
        ax.text(z, len(y) - 0.5, text, ha=align, va='bottom', fontsize=8, color=theme['muted'])
    Style_Axes(ax, theme, 'x')
    ax.set_yticks(y, Variant_Labels())
    ax.set_xlim(0, column + 0.4)
    ax.set_ylim(-0.6, len(y) + 0.1)
    ax.set_xlabel('world z [m]')
    fig.subplots_adjust(left=0.2, right=0.97, top=0.8, bottom=0.21)
    Footnote(fig, theme, [CONST_PROPOSAL_NOTE, CONST_ESTIMATE_NOTE])
    return fig


def Graph_Clearance(table: tp.Dict, mode: str, cfg: Cell_Configuration_Str) -> tp.Any:
    """
    Description:
        Per transfer segment (S1, S2, S3) and variant: the distance of the robot - environment pair with
        the least room above its margin (bar), that pair's margin (tick) and the room (label).

    Args:
        (1) table [Dict]: comparison.csv (Read_Table_CSV).
        (2) mode [string]: 'light' or 'dark'.
        (3) cfg [Cell_Configuration_Str(object)]: The cell configuration (the margins named in the footnote).

    Returns:
        (1) parameter [Figure(object)]: The figure.
    """

    theme = CONST_THEMES[mode]
    fig, axes = New_Figure((8.0, 4.4), theme, 'Tightest pair per transfer segment against its margin',
                           'Bar: distance of the pair with the least room; tick: that pair\'s margin; label: room above it',
                           1, 3, sharey=True)
    y = np.arange(len(CONST_ORDER))[::-1]
    for ax, section in zip(axes, ('S1', 'S2', 'S3')):
        for yy, v in zip(y, CONST_ORDER):
            room, margin = Value(table, v, section, 'least_room_mm'), Value(table, v, section, 'least_room_margin_mm')
            ax.barh(yy, room + margin, 0.62, color=CONST_VARIANT_COLORS[mode][v])
            ax.plot([margin, margin], [yy - 0.42, yy + 0.42], color=theme['text'], linewidth=2.0, solid_capstyle='butt')
            ax.text(room + margin + 1.5, yy, '+%.1f' % room, va='center', fontsize=8.5, color=theme['text'])
        Style_Axes(ax, theme, 'x')
        ax.set_title(section, fontsize=10, color=theme['text'], loc='left')
        ax.set_xlim(0, 62)
        ax.set_xlabel('distance [mm]')
    axes[0].set_yticks(y, Variant_Labels())
    fig.subplots_adjust(left=0.2, right=0.98, top=0.8, bottom=0.27, wspace=0.08)
    margins = (cfg.Clearance_Environment * 1e3, (cfg.Clearance_Environment + cfg.Machine_Extra_Margin) * 1e3,
               cfg.Station_Zone_Environment * 1e3)
    Footnote(fig, theme, ['Margins: %.0f mm environment, %.0f mm to machine (a) for generated segments outside the station zone, '
                          '%.0f mm in the station zone;' % margins,
                          'taught segments by the taught rule, generated ones by the generated rule. ' + CONST_PROPOSAL_NOTE,
                          CONST_ESTIMATE_NOTE])
    return fig


def Graph_Joints(data: tp.Dict[str, Variant_Data_Str], mode: str) -> tp.Any:
    """
    Description:
        The six joint angles of all five variants over the estimated time, one panel per joint on a
        shared time axis, the proposals dashed. A timeline strip above them shows each variant's
        sections (the taught station operations shaded) and where its sequence ends; a dotted line
        in the variant's colour carries that end through the joint panels.

    Args:
        (1) data [Dict<string, Variant_Data_Str>]: The variants' data.
        (2) mode [string]: 'light' or 'dark'.

    Returns:
        (1) parameter [Figure(object)]: The figure.
    """

    theme = CONST_THEMES[mode]
    colors = CONST_VARIANT_COLORS[mode]
    fig, axes = New_Figure((8.0, 11.0), theme, 'Joint positions over time, all five variants',
                           'Home → home at the estimated timing: every move from rest to rest (r = 0); blends and gripper time not modelled',
                           7, 1, sharex=True, gridspec_kw={'height_ratios': [1.25] + [1.0] * 6})
    timeline, joints = axes[0], axes[1:]
    end = max(data[v].Time[-1] for v in CONST_ORDER)
    for row, v in enumerate(CONST_ORDER):
        y = len(CONST_ORDER) - 1 - row
        for section, t_0, t_1 in data[v].Sections:
            station = section in ('drop', 'pick')
            timeline.barh(y, t_1 - t_0, 0.62, left=t_0, color=theme['grid'] if station else colors[v],
                          edgecolor=theme['surface'], linewidth=1.0, hatch='////' if station else None)
            if not station:
                timeline.text((t_0 + t_1) / 2.0, y, section, ha='center', va='center', fontsize=6.5, color=CONST_THEMES['light']['text'])
        timeline.text(data[v].Time[-1] + 0.25, y, '%.2f s' % data[v].Time[-1], va='center', fontsize=8, color=theme['text'])
    timeline.set_yticks(range(len(CONST_ORDER)), list(reversed(CONST_ORDER)))
    timeline.set_ylim(-0.6, len(CONST_ORDER) - 0.4)
    timeline.set_title('sections (hatched: taught station operations, drop and re-grip) and the end of each sequence',
                       fontsize=8.5, color=theme['muted'], loc='left')
    Style_Axes(timeline, theme, 'x')
    for j, ax in enumerate(joints):
        for v in CONST_ORDER:
            proposal = v in ('c2', 'c3')
            ax.plot(data[v].Time, np.degrees(data[v].Joints[:, j]), color=colors[v], linewidth=1.5,
                    linestyle=(0, (4, 2)) if proposal else '-',
                    label='%s%s (%.2f s)' % (v, ', proposal' if proposal else '', data[v].Time[-1]))
            ax.axvline(data[v].Time[-1], color=colors[v], linewidth=0.8, linestyle=(0, (1, 2)))
        Style_Axes(ax, theme, 'both')
        ax.set_ylabel('%s [°]' % CONST_JOINTS[j])
    joints[-1].set_xlabel('estimated time [s]')
    joints[-1].set_xlim(-0.3, end + 2.2)
    fig.legend(*joints[0].get_legend_handles_labels(), loc='upper left', bbox_to_anchor=(0.09, 0.935), ncol=3, fontsize=8.5,
               frameon=False, labelcolor=theme['text'])
    fig.subplots_adjust(left=0.13, right=0.98, top=0.86, bottom=0.075, hspace=0.14)
    Footnote(fig, theme, [CONST_PROPOSAL_NOTE + ' (dashed).', CONST_ESTIMATE_NOTE])
    return fig


def Graph_TCP_Paths(data: tp.Dict[str, Variant_Data_Str], mode: str, cfg: Cell_Configuration_Str) -> tp.Any:
    """
    Description:
        The TCP paths home -> home, one column per optimised variant against the taught path (grey):
        from above (x - y, top row) and from the side (x - z, bottom row), with the robot base, home
        and the station zone. Every panel of a row has the same limits.

    Args:
        (1) data [Dict<string, Variant_Data_Str>]: The variants' data.
        (2) mode [string]: 'light' or 'dark'.
        (3) cfg [Cell_Configuration_Str(object)]: The cell configuration (base, station zone).

    Returns:
        (1) parameter [Figure(object)]: The figure.
    """

    theme = CONST_THEMES[mode]
    shown = CONST_ORDER[1:]
    fig, axes = New_Figure((10.0, 6.4), theme, 'TCP paths, home → home',
                           'World frame; each variant (colour) against the taught path (grey); top row from above, bottom row from the side',
                           2, len(shown), sharex=True, sharey='row')
    zone_lo, zone_hi = np.asarray(cfg.Station_Zone_Min), np.asarray(cfg.Station_Zone_Max)
    home = data['taught'].TCP[0]
    for column, v in enumerate(shown):
        for row, (i, j) in enumerate(((0, 1), (0, 2))):
            ax = axes[row, column]
            ax.add_patch(plt.Rectangle((zone_lo[i], zone_lo[j]), zone_hi[i] - zone_lo[i], zone_hi[j] - zone_lo[j],
                                       facecolor='none', edgecolor=theme['muted'], linewidth=1.0, linestyle=(0, (3, 2))))
            ax.plot(data['taught'].TCP[:, i], data['taught'].TCP[:, j], color=theme['axis'], linewidth=1.2)
            ax.plot(data[v].TCP[:, i], data[v].TCP[:, j], color=CONST_VARIANT_COLORS[mode][v], linewidth=1.8)
            ax.plot(cfg.Base_Position[i], cfg.Base_Position[j], marker='s', markersize=7, color=theme['text'], linestyle='none')
            ax.plot(home[i], home[j], marker='o', markersize=7, color=theme['text'], markerfacecolor=theme['surface'],
                    linestyle='none')
            Style_Axes(ax, theme, 'both')
            if row == 0:
                ax.set_title('%s\n%s' % (v, CONST_SHORT[v]), fontsize=10, color=theme['text'], loc='left')
            else:
                ax.set_xlabel('x [m]')
            if column == 0:
                ax.set_ylabel('%s [m]' % 'xyz'[j])
                ax.text(zone_lo[i], zone_lo[j] - 0.04, 'station\nzone', ha='left', va='top', fontsize=7.5, color=theme['muted'])
                ax.text(cfg.Base_Position[i], cfg.Base_Position[j] - 0.06, 'base', ha='center', va='top', fontsize=7.5,
                        color=theme['muted'])
                ax.text(home[i] + 0.05, home[j], 'home', va='center', fontsize=7.5, color=theme['muted'])
    axes[0, 0].set_xlim(-0.95, 0.2)
    axes[0, 0].set_ylim(-0.75, 0.5)
    axes[1, 0].set_ylim(0.5, 2.4)
    fig.subplots_adjust(left=0.07, right=0.99, top=0.83, bottom=0.12, wspace=0.08, hspace=0.12)
    Footnote(fig, theme, [CONST_PROPOSAL_NOTE + '. ' + CONST_ESTIMATE_NOTE])
    return fig


def Draw_Path(scene: tp.Any, points: np.ndarray, rgba: tp.Sequence[float], radius: float) -> None:
    """
    Description:
        Add a TCP path to a render's scene as connected capsules, thinned to at most
        CONST_PATH_CAPSULES (display only).

    Args:
        (1) scene [MjvScene(object)]: The renderer's scene, after update_scene.
        (2) points [Matrix<float> Nx3]: The path (world frame, m).
        (3) rgba [Vector<float> 1x4]: The colour (0 - 1).
        (4) radius [float]: The capsule radius (m).
    """

    thinned = points[::max(1, len(points) // CONST_PATH_CAPSULES)]
    for a, b in zip(thinned[:-1], thinned[1:]):
        if scene.ngeom >= scene.maxgeom:
            raise RuntimeError('The render scene is full; raise max_geom in Render_Images.')
        g = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.zeros(9), np.asarray(rgba, dtype=np.float32))
        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_CAPSULE, radius, a, b)
        scene.ngeom += 1


def Draw_Backdrop(scene: tp.Any, camera: tp.Any, background: tp.Sequence[float]) -> None:
    """
    Description:
        A large box facing the camera far behind the look-at point, in the theme's background: the
        offscreen renderer clears to black, and translucent collision geometry has to blend with the
        page colour (display only). Its colour is divided by the headlight's ambient + diffuse, the
        only light of the render (Render_Images), which lights it face-on.

    Args:
        (1) scene [MjvScene(object)]: The renderer's scene, after update_scene.
        (2) camera [MjvCamera(object)]: The free camera.
        (3) background [Vector<float> 1x3]: The colour (0 - 1, CONST_RENDER_BACKGROUND).
    """

    azimuth, elevation = np.radians(camera.azimuth), np.radians(camera.elevation)
    forward = np.array([np.cos(elevation) * np.cos(azimuth), np.cos(elevation) * np.sin(azimuth), np.sin(elevation)])
    z = -forward
    x = np.cross([0.0, 0.0, 1.0], z)
    x /= np.linalg.norm(x)
    light = CONST_HEADLIGHT[0][0] + CONST_HEADLIGHT[1][0]
    g = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_BOX, np.array([200.0, 200.0, 0.1]), np.asarray(camera.lookat) + 40.0 * forward,
                        np.column_stack([x, np.cross(z, x), z]).flatten(),
                        np.array(list(np.asarray(background) / light) + [1.0], dtype=np.float32))
    g.specular = 0.0
    scene.ngeom += 1


def Environment_Groups(scene: Scene_Cls, cfg: Cell_Configuration_Str) -> tp.List[tp.Tuple[str, tp.Tuple, tp.List[int]]]:
    """
    Description:
        The environment geoms of the collision-model figure in groups, each with its legend label
        (naming the margin the planner keeps to it) and colour: the station zone (the zone pieces and
        boxes, Scene_Cls.Zone_Pieces / Zone_Boxes), the robot stand (body env_stand), each
        Environment.Assembly_Overrides object (its members in the manifest, CONST_ASSEMBLY_GROUPS)
        and every other piece. A geom belongs to the first group that claims it, in that order.

    Args:
        (1) scene [Scene_Cls(object)]: The scene with the environment.
        (2) cfg [Cell_Configuration_Str(object)]: The cell configuration (the margins).

    Returns:
        (1) parameter [List<Tuple<string, Vector<float> 1x4, List<int>>>]: Label, colour and geoms of
                                                                            each group, in legend order.
    """

    m = scene.Model
    environment = cfg.Clearance_Environment * 1e3
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or '' for g in range(m.ngeom)]
    geom_of = {name: g for g, name in enumerate(names)}
    left = {g for g in range(m.ngeom) if names[g].startswith('env_')}
    zone = {geom_of['env_piece_%d' % i] for i in scene.Zone_Pieces} | {geom_of[CONST_ADDED_BOX_PREFIX + n] for n in scene.Zone_Boxes}
    groups = [('station zone: holder and stub boxes, CAD pieces in the zone (%g mm)' % (cfg.Station_Zone_Environment * 1e3),
               CONST_ZONE_RGBA, sorted(zone))]
    left -= zone
    stand = {g for g in left if m.body(m.geom_bodyid[g]).name == 'env_stand'}
    groups.append(('robot stand (%g mm; the robot base stands on it)' % environment, tuple(CONST_STAND_RGB) + (1.0,), sorted(stand)))
    left -= stand
    components = scene.Piece_Components
    for name, _, _, _ in cfg.Assembly_Overrides:
        members = set(scene.Collision_Manifest.get('assembly_components', {}).get(name, []))
        geoms = {g for g in left if names[g].startswith('env_piece_') and components[int(names[g][len('env_piece_'):])] in members}
        label, rgb = CONST_ASSEMBLY_GROUPS.get(name, (name, CONST_ENV_RGB))
        if name == cfg.Machine_Extra_Assembly:
            label += ' (%g mm; %g mm for a generated path)' % (environment, environment + cfg.Machine_Extra_Margin * 1e3)
        else:
            label += ' (%g mm)' % environment
        groups.append((label, tuple(rgb) + (1.0,), sorted(geoms)))
        left -= geoms
    groups.append(('other environment: the tables (b) (%g mm)' % environment, tuple(CONST_OTHER_RGB) + (1.0,), sorted(left)))
    return groups


def Frame_Render(image: np.ndarray, mode: str, title: str, subtitle: str, legend: tp.Sequence[tp.Tuple[str, tp.Any, str]],
                 notes: tp.Sequence[str]) -> tp.Any:
    """
    Description:
        A render with a title, a subtitle, a legend over its top left corner and footnotes, on the
        render's own backdrop colour, so image and frame read as one panel.

    Args:
        (1) image [Matrix<uint8> HxWx3]: The render.
        (2) mode [string]: 'light' or 'dark'.
        (3 - 4) title, subtitle [string]: The heading.
        (5) legend [List<Tuple<string, Vector<float> 1x3 or 1x4, string>>]: Label, colour and kind
                                                                           ('line', 'thin' or 'patch').
        (6) notes [List<string>]: The footnote lines.

    Returns:
        (1) parameter [Figure(object)]: The figure.
    """

    theme = dict(CONST_THEMES[mode], surface='#%02x%02x%02x' % tuple(int(round(c * 255)) for c in CONST_RENDER_BACKGROUND[mode]))
    width, height = image.shape[1] / CONST_DPI, image.shape[0] / CONST_DPI
    top, bottom = 0.62, 0.18 + 0.13 * len(notes)
    fig, ax = New_Figure((width, height + top + bottom), theme, title, subtitle)
    ax.set_position([0.0, bottom / (height + top + bottom), 1.0, height / (height + top + bottom)])
    ax.imshow(image, interpolation='none')
    ax.set_axis_off()
    handles = []
    for label, color, kind in legend:
        if kind == 'patch':
            handles.append(matplotlib.patches.Patch(facecolor=color, edgecolor='none', label=label))
        else:
            handles.append(matplotlib.lines.Line2D([], [], color=color, linewidth=3.0 if kind == 'line' else 1.5, label=label))
    ax.legend(handles=handles, loc='upper left', fontsize=8.5, frameon=False, labelcolor=theme['text'], handlelength=1.8)
    Footnote(fig, theme, notes)
    return fig


def Render_Images(cfg: Cell_Configuration_Str, data: tp.Dict[str, Variant_Data_Str], table: tp.Dict, out: str) -> tp.List[str]:
    """
    Description:
        The MuJoCo renders (CONST_RENDERS), each for the light and the dark page theme: the scene at
        the holding opening built without the visual STL, the environment drawn as its collision
        model (Render_Str.Colouring), the robot by its visual meshes or, in the collision-model
        figure, by its collision hulls only, the robot at a point of c1, offscreen, lit by the
        headlight only (the model's own light is switched off, so the surfaces are matte and the
        backdrop has its exact colour), the TCP paths drawn as capsules in the graphs' colours. A
        framed render gets its title, legend and notes (Frame_Render); a path render's subtitle gives
        the variant's estimated time and highest robot point from comparison.csv.

    Args:
        (1) cfg [Cell_Configuration_Str(object)]: The cell configuration.
        (2) data [Dict<string, Variant_Data_Str>]: The variants' data (points and TCP paths).
        (3) table [Dict<string, Dict<string, Dict<string, string>>>]: The comparison table (Read_Table_CSV).
        (4) out [string]: The output directory.

    Returns:
        (1) parameter [List<string>]: The files written.
    """

    scene = Scene_Cls(dataclasses.replace(cfg, Opening=Opening(cfg, 'holding')), with_environment=True, with_visual=False)
    m = scene.Model
    width, height = CONST_RENDER_SIZE
    m.vis.global_.offwidth, m.vis.global_.offheight = max(m.vis.global_.offwidth, width), max(m.vis.global_.offheight, height)
    m.vis.headlight.ambient[:], m.vis.headlight.diffuse[:], m.vis.headlight.specular[:] = CONST_HEADLIGHT
    m.light_active[:] = 0
    renderer = mujoco.Renderer(m, height, width, max_geom=20000)
    groups = Environment_Groups(scene, cfg)
    zone = set(groups[0][2])
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or '' for g in range(m.ngeom)]
    environment = [g for g in range(m.ngeom) if names[g].startswith('env_')]
    robot_collision = [g for g in range(m.ngeom) if m.geom_group[g] == CONST_COLLISION_GROUP and not names[g].startswith('env_')]
    zone_lo, zone_hi = np.asarray(cfg.Station_Zone_Min), np.asarray(cfg.Station_Zone_Max)
    written = []
    for name, render in CONST_RENDERS.items():
        option = mujoco.MjvOption()
        for group in range(6):
            option.geomgroup[group] = 1 if group in ((CONST_COLLISION_GROUP,) if render.Colouring == 'groups' else (2, CONST_COLLISION_GROUP)) else 0
        for g in environment:
            m.geom_rgba[g] = CONST_ZONE_RGBA if render.Colouring == 'zone' and g in zone else list(CONST_ENV_RGB) + [render.Opacity]
        if render.Colouring == 'groups':
            for _, rgba, geoms in groups:
                for g in geoms:
                    m.geom_rgba[g] = rgba
        robot_rgba = CONST_ROBOT_GROUP_RGBA if render.Colouring == 'groups' else CONST_ROBOT_COLLISION_RGBA
        for g in robot_collision:
            m.geom_rgba[g] = robot_rgba if render.Robot_Collision or render.Colouring == 'groups' else (0.0, 0.0, 0.0, 0.0)
        scene.Set_Joints(data['c1'].Points[render.Point].Q)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:], camera.distance, camera.azimuth, camera.elevation = render.Camera
        for mode, background in CONST_RENDER_BACKGROUND.items():
            renderer.update_scene(scene.Data, camera=camera, scene_option=option)
            legend = []
            for variant, radius in render.Paths:
                hex_color = CONST_VARIANT_COLORS[mode][variant]
                rgb = [int(hex_color[k:k + 2], 16) / 255.0 for k in (1, 3, 5)]
                Draw_Path(renderer.scene, data[variant].TCP, rgb + [1.0], radius)
                legend.append(('%s: %s' % (variant, 'taught path, reference' if radius == CONST_REFERENCE_RADIUS else CONST_RENDER_STATUS[variant]),
                               hex_color, 'thin' if radius == CONST_REFERENCE_RADIUS else 'line'))
            if render.Colouring == 'groups':
                g = renderer.scene.geoms[renderer.scene.ngeom]
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_BOX, (zone_hi - zone_lo) / 2.0, (zone_hi + zone_lo) / 2.0, np.eye(3).flatten(),
                                    np.asarray(CONST_ZONE_BOX_RGBA, dtype=np.float32))
                renderer.scene.ngeom += 1
            Draw_Backdrop(renderer.scene, camera, background)
            image = renderer.render()
            path = os.path.join(out, '%s_%s.png' % (name, mode))
            if not render.Framed:
                Write_PNG(image, path)
            elif render.Colouring == 'groups':
                legend = [('robot and gripper collision hulls (%g mm between them)' % (cfg.Clearance_Self * 1e3), CONST_ROBOT_GROUP_RGBA, 'patch')]
                legend += [(label, rgba, 'patch') for label, rgba, _ in groups]
                legend.append(('station-zone box: inside it the zone pieces keep %g mm' % (cfg.Station_Zone_Environment * 1e3),
                               CONST_ZONE_RGBA[:3] + (0.35,), 'patch'))
                Save_Figure(Frame_Render(image, mode, 'The collision model: the cell as the planner sees it',
                                         'Convex pieces of the cell CAD and the added station boxes; the robot at home by its '
                                         'collision hulls; each group with its margin',
                                         legend, ['Not the CAD: machine (a) moved 150 mm away from the robot, the side cabinet 50 mm '
                                                  'along −x, the station boxes fitted to the taught program.',
                                                  'The planner checks every sample of a path against these pieces with the '
                                                  'margins shown. Simulation passed ≠ safe on the robot.']), path)
            else:
                v = render.Variant
                Save_Figure(Frame_Render(image, mode, '%s — %s' % (v, CONST_RENDER_STATUS[v]),
                                         'TCP path home → home in the collision model; estimated %.2f s, highest robot point %.3f m'
                                         % (Value(table, v, 'total', 't_est_s'), Value(table, v, 'total', 'max_robot_z_m')),
                                         legend, [CONST_ESTIMATE_NOTE]), path)
            written.append(path)
    renderer.close()
    return written


def main():
    """
    Description:
        Write every README image into Media/ from the repository's data: the graphs from
        Data/Generated/variants/comparison.csv and from the variant files (as the replay builds and
        times their moves), each for GitHub's light and dark theme, and the MuJoCo renders of the
        cell drawn from its collision model, never the visual STL (offscreen; no viewer). Nothing is
        random: the same data gives the same graphs.

            .venv/bin/python Example/make_readme_figures.py [--out DIR] [--graphs-only]
    """

    parser = argparse.ArgumentParser(description='Write the README figures (graphs and MuJoCo renders) into Media/.')
    parser.add_argument('--config', default=CONST_DEFAULT_CONFIG_FILE, help='the cell configuration (default: %(default)s)')
    parser.add_argument('--out', default=CONST_MEDIA_DIR, help='output directory (default: %(default)s)')
    parser.add_argument('--graphs-only', action='store_true', help='write the graphs only, not the MuJoCo renders')
    args = parser.parse_args()
    print('[WARNING] Every figure is a simulation estimate. Simulation passed != safe on the robot.')
    try:
        cfg = Load_Configuration(args.config)
        table = Read_Table_CSV(os.path.join(CONST_VARIANT_DIR, CONST_TABLE_CSV))
        scenes = {state: Scene_Cls(dataclasses.replace(cfg, Opening=Opening(cfg, state)), with_environment=False)
                  for state in ('holding', 'open')}
        data = {v: Load_Variant_Data(v, cfg, scenes) for v in CONST_ORDER}
    except (Configuration_Error_Cls, URScript_Error_Cls, Kinematics_Error_Cls, ValueError, OSError) as error:
        print('[ERROR] %s' % error)
        return 1
    for level, message in cfg.Notes:
        print('[%s] %s' % (level, message))
    for v in CONST_ORDER:
        length = float(np.sum(np.linalg.norm(np.diff(data[v].TCP, axis=0), axis=1)))
        # The table holds 0.001 m and 0.01 s: a difference beyond the rounding means the files changed.
        if abs(length - Value(table, v, 'total', 'tcp_length_m')) > 0.0015 or abs(data[v].Time[-1] - Value(table, v, 'total', 't_est_s')) > 0.006:
            print('[ERROR] %s: the variant files (TCP path %.3f m, %.2f s) do not match comparison.csv; rewrite it with '
                  'Example/write_variants.py --table-only.' % (v, length, data[v].Time[-1]))
            return 1
    os.makedirs(args.out, exist_ok=True)
    graphs = {'cycle_time': lambda mode: Graph_Cycle_Time(table, mode),
              'path_length': lambda mode: Graph_Path_Length(table, mode),
              'max_height': lambda mode: Graph_Max_Height(table, mode, cfg),
              'clearance': lambda mode: Graph_Clearance(table, mode, cfg),
              'joints': lambda mode: Graph_Joints(data, mode),
              'tcp_paths': lambda mode: Graph_TCP_Paths(data, mode, cfg)}
    written = []
    for name, draw in graphs.items():
        for mode in CONST_THEMES:
            path = os.path.join(args.out, '%s_%s.png' % (name, mode))
            Save_Figure(draw(mode), path)
            written.append(path)
    if not args.graphs_only:
        written += Render_Images(cfg, data, table, args.out)
    for path in written:
        print('[INFO] Wrote %s.' % os.path.relpath(path, CONST_PROJECT_ROOT))
    print('[INFO] %d images. Simulation passed != safe on the robot.' % len(written))
    return 0


if __name__ == '__main__':
    sys.exit(main())
