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
# File Name: ../test/Docs/test_readme.py
## =========================================================================== ##

# System (Default)
import os
# Regular expressions (Default)
import re
# Subprocess (Listing the tracked files with git)
import subprocess
# CSV (Comma-separated values), read here on its own, not through the project's reader
import csv

"""
Description:
    Initialization of constants.

    The public README against the repository: its results table against comparison.csv, its images
    against Media/, and the root it describes against the files git tracks.
"""
# The variants in the order of the table.
#   Unit [Tuple<string>]
CONST_VARIANTS = ('taught', 'b_optimised', 'c1', 'c2', 'c3')
# The transfer segments the clearance columns cover.
#   Unit [Tuple<string>]
CONST_TRANSFERS = ('S1', 'S2', 'S3')
# The fewest words of a figure's alt text: enough to say what it shows, not just name it.
#   Unit [int]
CONST_ALT_WORDS = 12
# What the root may hold (owner, 2026-10-20): the public files, the code and data folders, the
# README images, pytest's configuration and the hidden .claude/.
#   Unit [Set<string>]
CONST_ROOT_ENTRIES = {'README.md', 'LICENSE', 'CLAUDE.md', 'src', 'test', 'Example', 'Data', 'CAD', 'Configuration', 'models',
                      'install.sh', 'requirements.txt', '.gitignore', 'Media', 'pytest.ini', '.claude'}


def _Readme(root):
    with open(os.path.join(root, 'README.md'), encoding='utf-8') as handle:
        return handle.read()


def _Table_Rows(text):
    # The rows of the results table: the lines starting with a variant in backticks, split into cells.
    rows = {}
    for line in text.splitlines():
        match = re.match(r'\| `(\w+)` \|', line)
        if match:
            rows[match.group(1)] = [cell.strip() for cell in line.strip().strip('|').split(' | ')]
    return rows


def _Csv(root):
    table = {}
    with open(os.path.join(root, 'Data', 'Generated', 'variants', 'comparison.csv'), encoding='utf-8', newline='') as handle:
        for row in csv.DictReader(handle):
            table.setdefault(row['variant'], {})[row['section']] = row
    return table


def _Expected(sections, taught_total):
    # A table row from the csv: the total row's values as written, the change against the taught
    # estimate, and the closest distance and the least room (with its margin) over S1 - S3.
    total = sections['total']
    change = 100.0 * (float(total['t_est_s']) - taught_total) / taught_total
    transfers = [sections[s] for s in CONST_TRANSFERS]
    least = min(transfers, key=lambda r: float(r['least_room_mm']))
    return [total['t_est_s'], '—' if total['t_est_s'] == '%.2f' % taught_total else '%s%.1f %%' % ('−' if change < 0 else '+', abs(change)),
            total['vias'], total['sum_abs_dq_rad'], total['tcp_length_m'], total['max_robot_z_m'],
            '%.1f' % min(float(r['closest_mm']) for r in transfers), '%s (margin %s)' % (least['least_room_mm'], least['least_room_margin_mm'])]


def test_the_results_table_is_comparison_csv(root):
    """
    Description:
        Every number of the README's results table is comparison.csv's: per variant the estimated
        home -> home time, its change against the taught one, the via points, the joint travel, the
        TCP path, the highest robot point (the total row, as written), the closest distance and the
        least room above its margin over the transfer segments S1 - S3. The five variants appear in
        order, the proposals marked as not verified on the robot.
    """

    rows, table = _Table_Rows(_Readme(root)), _Csv(root)
    assert list(rows) == list(CONST_VARIANTS)
    taught_total = float(table['taught']['total']['t_est_s'])
    for variant in CONST_VARIANTS:
        assert rows[variant][2:] == _Expected(table[variant], taught_total), variant
    for variant in ('c2', 'c3'):
        assert 'not verified on the robot' in rows[variant][1]
    assert '**delivered**' in rows['c1'][1]


def test_every_image_the_readme_shows_exists(root):
    """
    Description:
        Every image the README shows (Markdown images, img src and picture srcset) is a file in Media/,
        and every file in Media/ is shown.
    """

    text = _Readme(root)
    shown = set(re.findall(r'\]\((Media/[^)\s]+)\)', text)) | set(re.findall(r'(?:src|srcset)="(Media/[^"]+)"', text))
    assert shown
    for path in shown:
        assert os.path.isfile(os.path.join(root, path)), path
    media = {'Media/' + name for name in os.listdir(os.path.join(root, 'Media')) if not name.startswith('.')}
    assert media == shown


def test_every_figure_has_an_alt_text_and_a_numbered_caption_below_it(root):
    """
    Description:
        Every image the README shows from Media/ has a descriptive alt text (at least
        CONST_ALT_WORDS words) and, after it and before the next image, a caption "Figure N:", the
        figures numbered 1, 2, ... in their order in the README.
    """

    text = _Readme(root)
    images = list(re.finditer(r'<img src="Media/[^"]+" alt="([^"]*)"', text))
    assert images
    for k, image in enumerate(images):
        assert len(image.group(1).split()) >= CONST_ALT_WORDS, image.group(0)
        following = text[image.end():images[k + 1].start() if k + 1 < len(images) else len(text)]
        caption = re.search(r'(?:\*\*|<b>)Figure (\d+):(?:\*\*|</b>) \S', following)
        assert caption and int(caption.group(1)) == k + 1, image.group(0)


def test_the_root_holds_only_what_the_readme_describes(root):
    """
    Description:
        The files git tracks at the root are the README, the licence, CLAUDE.md, the code and data
        folders, the installer, the requirements, .gitignore, Media/, pytest.ini and .claude/ -
        nothing else (no SPEC.md, KNOWN_ISSUES.md or docs/, which live in .claude/project-docs/).
    """

    result = subprocess.run(['git', 'ls-files'], cwd=root, capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stderr
    entries = {path.split('/', 1)[0] for path in result.stdout.splitlines()}
    assert entries == CONST_ROOT_ENTRIES
    for name in ('SPEC.md', 'KNOWN_ISSUES.md', 'design-decisions.md'):
        assert os.path.isfile(os.path.join(root, '.claude', 'project-docs', name)), name
