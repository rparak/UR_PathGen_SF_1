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
# File Name: ../test/Install/test_install.py
## =========================================================================== ##

# System (Default)
import os
import sys
# Subprocess (Subprocess management)
import subprocess
# Regular expressions (Default)
import re
# JSON (JSON encoder and decoder)
import json
# Hashlib (Secure hashes)
import hashlib
# Pytest (Testing framework) [pip3 install pytest]
import pytest

"""
Description:
    Initialization of constants.

    install.sh is sourced in a fresh /bin/bash with a hermetic PATH: stubs for the commands that
    describe the machine (uname, sw_vers, python3.11, python3) ahead of the system directories.
    What ran for real: the script's own functions, date, shasum/sha256sum, the venv's Python (for
    the JSON of the manifest). What ran only against stubs: platform detection (Darwin arm64,
    Linux, an unsupported OS) and the interpreter lookup. pip and venv creation are not run here;
    install.sh itself was run for real on the development Mac.
"""
# One log line: 'YYYY-MM-DD HH:MM:SS [LEVEL  ] message', the level padded to 7.
#   Unit [Pattern]
CONST_LOG_LINE = re.compile(r'^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2} \[(INFO   |WARNING|ERROR  )\] \S.*$')


def _Stub(directory, name, body):
    path = directory / name
    path.write_text('#!/bin/sh\n' + body + '\n', encoding='utf-8')
    path.chmod(0o755)
    return path


def _Run(root, stubs, script):
    """
    Description:
        Source install.sh and run a snippet; returns (exit code, stdout lines).
    """

    command = 'source "%s"; %s' % (os.path.join(root, 'install.sh'), script)
    result = subprocess.run(['/bin/bash', '-c', command], capture_output=True, text=True, timeout=60,
                            env={'PATH': '%s:/usr/bin:/bin' % stubs, 'HOME': str(stubs)})
    lines = [line for line in result.stdout.splitlines() if line]
    return result.returncode, lines, result.stderr


@pytest.fixture
def stubs(tmp_path):
    """
    Description:
        A stub directory; python3 records that it was called (it must never be).
    """

    directory = tmp_path / 'bin'
    directory.mkdir()
    _Stub(directory, 'python3', 'touch "%s/python3_was_called"; exit 1' % tmp_path)
    _Stub(directory, 'sw_vers', 'echo 26.6.2')
    return directory


def test_the_script_parses(root):
    """
    Description:
        bash -n accepts the script.
    """

    assert subprocess.run(['/bin/bash', '-n', os.path.join(root, 'install.sh')]).returncode == 0


@pytest.mark.parametrize('system, arch, platform', [('Darwin', 'arm64', 'macos'), ('Linux', 'x86_64', 'linux')])
def test_the_platform_is_detected(root, stubs, system, arch, platform):
    """
    Description:
        uname decides the platform, and the INFO line naming it is printed in the log format.
    """

    _Stub(stubs, 'uname', '[ "$1" = "-s" ] && echo %s || echo %s' % (system, arch))
    code, lines, _ = _Run(root, stubs, 'Detect_Platform; echo "PLATFORM=$PLATFORM"')
    assert code == 0 and lines[-1] == 'PLATFORM=%s' % platform
    assert any('[INFO   ] Detected %s' % {'macos': 'macOS', 'linux': 'Linux'}[platform] in line for line in lines[:-1])
    assert all(CONST_LOG_LINE.match(line) for line in lines[:-1])


def test_an_unsupported_system_fails_with_error_lines(root, stubs):
    """
    Description:
        Anything but Darwin or Linux is refused with ERROR lines and a non-zero exit.
    """

    _Stub(stubs, 'uname', '[ "$1" = "-s" ] && echo FreeBSD || echo amd64')
    code, lines, _ = _Run(root, stubs, 'Detect_Platform')
    assert code == 1 and all(CONST_LOG_LINE.match(line) for line in lines)
    assert '[ERROR  ]' in lines[-1] and 'macOS' in lines[-1]


def test_python_3_11_is_found_by_its_explicit_name_never_python3(root, stubs, tmp_path):
    """
    Description:
        On Linux python3.11 is looked up on PATH, on macOS the configured absolute path is used;
        the bare python3 (broken numpy on the development Mac) is never called.
    """

    _Stub(stubs, 'python3.11', 'echo 3.11')
    code, lines, _ = _Run(root, stubs, 'PLATFORM=linux; Find_Python; echo "PYTHON=$PYTHON"')
    assert code == 0 and lines[-1] == 'PYTHON=%s' % (stubs / 'python3.11')
    mac = _Stub(stubs, 'brew_python', 'echo 3.11')
    code, lines, _ = _Run(root, stubs, 'PLATFORM=macos; CONST_PYTHON_MACOS="%s"; Find_Python; echo "PYTHON=$PYTHON"' % mac)
    assert code == 0 and lines[-1] == 'PYTHON=%s' % mac
    assert all(CONST_LOG_LINE.match(line) for line in lines[:-1])
    assert not (tmp_path / 'python3_was_called').exists()


def test_a_wrong_python_version_is_refused(root, stubs):
    """
    Description:
        An interpreter that is not 3.11 fails with the install command.
    """

    _Stub(stubs, 'python3.11', 'echo 3.10')
    code, lines, _ = _Run(root, stubs, 'PLATFORM=linux; Find_Python')
    assert code == 1 and 'Install Python 3.11' in lines[-1]


def _Fake_Repository(tmp_path):
    repo = tmp_path / 'repo'
    (repo / 'models').mkdir(parents=True)
    files = {'models/a.xml': b'<mujoco/>', 'models/b.obj': b'v 0 0 0'}
    for path, content in files.items():
        (repo / path).write_bytes(content)
    manifest = {'sha256': {p: hashlib.sha256(c).hexdigest() for p, c in files.items()}}
    (repo / 'models' / 'MANIFEST.json').write_text(json.dumps(manifest), encoding='utf-8')
    return repo


def test_matching_model_files_are_verified(root, stubs, tmp_path):
    """
    Description:
        Every file of the manifest is hashed and matches; the absent collision manifest is a WARNING.
    """

    repo = _Fake_Repository(tmp_path)
    code, lines, _ = _Run(root, stubs, 'CONST_ROOT="%s"; PYTHON="%s"; Verify_Files' % (repo, sys.executable))
    assert code == 0 and all(CONST_LOG_LINE.match(line) for line in lines)
    assert 'All 2 model files match' in lines[0] and '[WARNING]' in lines[1]


def test_a_tampered_model_file_fails_the_install(root, stubs, tmp_path):
    """
    Description:
        A changed vendored file fails with ERROR lines naming it and the command to restore it.
    """

    repo = _Fake_Repository(tmp_path)
    (repo / 'models' / 'b.obj').write_bytes(b'v 1 0 0')
    code, lines, _ = _Run(root, stubs, 'CONST_ROOT="%s"; PYTHON="%s"; Verify_Files' % (repo, sys.executable))
    assert code == 1 and all(CONST_LOG_LINE.match(line) for line in lines)
    assert any('Hash mismatch: models/b.obj' in line for line in lines)
    assert 'git checkout -- models' in lines[-1]


def test_the_shipped_models_verify(root, stubs):
    """
    Description:
        The repository's own models and collision set pass the check.
    """

    code, lines, _ = _Run(root, stubs, 'PYTHON="%s"; Verify_Files' % sys.executable)
    assert code == 0, lines
    assert all(CONST_LOG_LINE.match(line) for line in lines)
