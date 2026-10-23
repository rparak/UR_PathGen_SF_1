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
# File Name: ../test/Docs/test_skills.py
## =========================================================================== ##

# System (Default)
import os
# Subprocess (Subprocess management)
import subprocess
# JSON (JSON encoder and decoder)
import json

"""
Description:
    Initialization of constants.

    The skills are personal (~/.claude/skills/); this repository keeps only their project layer
    (SPEC §6, §13). These tests pin that layer, not the personal skills, which live outside git.
"""
# The skills whose project layer this repository keeps, and the files each holds.
#   Unit [Dict<string, List<string>>]
CONST_SKILL_FILES = {'feature-manager': ['project.md'],
                     'project-optimize': ['project.md'],
                     'project-style-rpx': ['check_style.json', 'mjcf.md', 'project.md', 'urscript.md'],
                     'readme-writer': ['project.md']}
# The style checker's configuration for this repository (SPEC §6: the header holder of LICENSE; the
# code in src/ and Example/, the tests in test/, every file checked).
#   Unit [Dict<string, Any>]
CONST_CHECK_STYLE = {'licence': 'MIT License', 'holder': 'JIC Smart Factory', 'class_roots': ['src', 'Example'],
                     'test_roots': ['test'], 'check_roots': ['src', 'Example', 'test'], 'exclude': []}


def test_the_repository_skill_folders_hold_only_references(root):
    """
    Description:
        The files git tracks in .claude/skills/ are <skill>/references/<file>, exactly the listed
        ones: a SKILL.md here would be ignored in favour of the personal skill of the same name and
        drift unseen.
    """

    result = subprocess.run(['git', 'ls-files', '.claude/skills'], cwd=root, capture_output=True, text=True,
                            timeout=60)
    assert result.returncode == 0, result.stderr
    expected = {'.claude/skills/%s/references/%s' % (skill, name)
                for skill, files in CONST_SKILL_FILES.items() for name in files}
    assert set(result.stdout.splitlines()) == expected


def test_the_style_checker_configuration_checks_every_file(root):
    """
    Description:
        The checker's configuration names the holder, checks src, Example and test, follows
        exceptions through src and Example, and excludes nothing.
    """

    path = os.path.join(root, '.claude', 'skills', 'project-style-rpx', 'references', 'check_style.json')
    with open(path, encoding='utf-8') as handle:
        assert json.load(handle) == CONST_CHECK_STYLE
