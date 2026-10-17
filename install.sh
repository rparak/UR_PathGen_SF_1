#!/usr/bin/env bash
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
# File Name: ../install.sh
## =========================================================================== ##

# Description:
#   Install the UR_PathGen_SF_1 environment: a .venv with the pinned requirements, and a check
#   that the vendored models and the committed collision set are the files they claim to be.
#
#   Steps, cheapest check first:
#     1. Detect the platform (macOS on Apple silicon is primary; Ubuntu Linux is supported but
#        not verified on real hardware).
#     2. Find Python 3.11. It is always called by its explicit name, never as a bare python3: on
#        the development Mac, python3 resolves to a miniforge interpreter whose numpy is broken.
#     3. Create .venv if it is absent. A broken .venv is reported with the command to remove it;
#        it is never deleted by this script.
#     4. pip install -r requirements.txt.
#     5. Verify the sha256 of every file in models/MANIFEST.json, and the source STL hash recorded
#        in the collision-set manifest. Nothing is downloaded: the models are vendored.
#
#   Usage:
#     ./install.sh                  run all steps
#     ./install.sh --detect-only    report the platform, the interpreter and the plan, then stop
#
#   Output: every line is 'YYYY-MM-DD HH:MM:SS [LEVEL  ] message', the level padded to 7.

# Description:
#   Initialization of constants.

# The repository root: the directory holding this script, so it works from any directory.
#   Unit [string]
CONST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# The Python version every pinned requirement was verified with.
#   Unit [string]
CONST_PYTHON_VERSION="3.11"
# The interpreter on macOS: Homebrew's, by absolute path, so PATH order cannot pick another one.
#   Unit [string]
CONST_PYTHON_MACOS="/opt/homebrew/bin/python3.11"
# The interpreter on Linux, looked up on PATH (deadsnakes or the distribution package).
#   Unit [string]
CONST_PYTHON_LINUX="python3.11"
# Width the level is padded to, so the messages line up.
#   Unit [int]
CONST_LEVEL_WIDTH=7
# Manifests that are verified.
#   Unit [string]
CONST_MODELS_MANIFEST="models/MANIFEST.json"
CONST_COLLISION_MANIFEST="CAD/Collision/SF1_Environment_Collision.json"

# Description:
#   Print one log line in the project's format.
# Args:
#   (1) level [string]: INFO, WARNING or ERROR.
#   (2) message [string]: The message.
# Returns:
#   (1) parameter [string]: The line, on stdout.
Log() {
    local level="$1"; shift
    printf '%s [%-*s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$CONST_LEVEL_WIDTH" "$level" "$*"
}

# Description:
#   Report one or more ERROR lines and exit non-zero. The last line says what to do.
# Args:
#   (1 - n) message [string]: One ERROR line each.
# Returns:
#   (1) parameter [int]: Exit status 1.
Die() {
    local line
    for line in "$@"; do Log ERROR "$line"; done
    exit 1
}

# Description:
#   Name the line and command of anything unanticipated, instead of a bare trace.
# Args:
#   (1) line [int]: The line number.
#   (2) command [string]: The failing command.
# Returns:
#   None
On_Unexpected_Error() {
    Log ERROR "Unexpected failure at install.sh line $1: $2"
    Log ERROR "Nothing was deleted. Fix the cause above and run ./install.sh again."
    exit 1
}

# Description:
#   Interrupted: say that re-running is safe.
# Args:
#   None
# Returns:
#   None
On_Interrupt() {
    Log WARNING "Interrupted. Every step is re-runnable; run ./install.sh again."
    exit 130
}

# Description:
#   Detect the operating system and architecture.
# Args:
#   None
# Returns:
#   (1) parameter [string]: Sets PLATFORM to 'macos' or 'linux'; fails on anything else.
Detect_Platform() {
    local os arch
    os="$(uname -s)"; arch="$(uname -m)"
    case "$os" in
        Darwin)
            PLATFORM="macos"
            if [ "$arch" = "arm64" ]; then
                Log INFO "Detected macOS $(sw_vers -productVersion 2>/dev/null) on Apple silicon ($arch)."
            else
                Log WARNING "Detected macOS on $arch; only Apple silicon has been verified."
            fi ;;
        Linux)
            PLATFORM="linux"
            Log INFO "Detected Linux ($arch). The Ubuntu path is supported but not verified on hardware." ;;
        *)
            Die "Unsupported operating system '$os'." \
                "This project installs on macOS (Apple silicon) or Ubuntu Linux only." ;;
    esac
}

# Description:
#   Find the Python 3.11 interpreter by its explicit name and check its version.
# Args:
#   None
# Returns:
#   (1) parameter [string]: Sets PYTHON to the interpreter path; fails with the install command.
Find_Python() {
    local candidate version
    if [ "$PLATFORM" = "macos" ]; then candidate="$CONST_PYTHON_MACOS"; else candidate="$(command -v "$CONST_PYTHON_LINUX" || true)"; fi
    if [ -z "$candidate" ] || [ ! -x "$candidate" ]; then
        if [ "$PLATFORM" = "macos" ]; then
            Die "Python $CONST_PYTHON_VERSION was not found at $CONST_PYTHON_MACOS." \
                "Install it with: brew install python@$CONST_PYTHON_VERSION"
        else
            Die "Python $CONST_PYTHON_VERSION ($CONST_PYTHON_LINUX) was not found on PATH." \
                "Install it with: sudo apt install python$CONST_PYTHON_VERSION python$CONST_PYTHON_VERSION-venv"
        fi
    fi
    version="$("$candidate" -c 'import sys; print("%d.%d" % sys.version_info[:2])')"
    if [ "$version" != "$CONST_PYTHON_VERSION" ]; then
        Die "$candidate is Python $version, not $CONST_PYTHON_VERSION." \
            "Install Python $CONST_PYTHON_VERSION; the pinned requirements were verified with it only."
    fi
    PYTHON="$candidate"
    Log INFO "Using Python $version at $PYTHON."
}

# Description:
#   Create .venv if absent; report a broken one with the command to remove it.
# Args:
#   None
# Returns:
#   None
Ensure_Venv() {
    local venv="$CONST_ROOT/.venv"
    if [ -d "$venv" ]; then
        if "$venv/bin/python" -c 'import sys; sys.exit(0 if sys.version_info[:2] == (3, 11) else 1)' 2>/dev/null; then
            Log INFO ".venv already present and working (Python $CONST_PYTHON_VERSION)."
            return 0
        fi
        Die ".venv exists but its interpreter is missing or not Python $CONST_PYTHON_VERSION." \
            "Remove it with: rm -rf \"$venv\" and run ./install.sh again."
    fi
    "$PYTHON" -m venv "$venv" || Die "Creating .venv failed." "Check that the venv module is installed for $PYTHON."
    Log INFO "Created .venv with $PYTHON."
}

# Description:
#   Install the pinned requirements into .venv; pip's output is kept and only its tail is shown
#   on failure.
# Args:
#   None
# Returns:
#   None
Install_Requirements() {
    local out line
    out="$(mktemp)"
    if "$CONST_ROOT/.venv/bin/python" -m pip install --disable-pip-version-check -r "$CONST_ROOT/requirements.txt" > "$out" 2>&1; then
        if grep -q '^Successfully installed' "$out"; then
            Log INFO "Installed: $(grep '^Successfully installed' "$out" | sed 's/^Successfully installed //')"
        else
            Log INFO "pip changed nothing; every pinned requirement is already installed."
        fi
        rm -f "$out"
    else
        tail -n 15 "$out" | while IFS= read -r line; do Log ERROR "pip: $line"; done
        rm -f "$out"
        Die "pip install -r requirements.txt failed (output above)." "Fix the cause and run ./install.sh again."
    fi
}

# Description:
#   Compute the sha256 of a file with the tool the platform has.
# Args:
#   (1) path [string]: The file.
# Returns:
#   (1) parameter [string]: The hex digest on stdout.
Sha256() {
    if command -v shasum > /dev/null 2>&1; then shasum -a 256 "$1" | cut -d' ' -f1; else sha256sum "$1" | cut -d' ' -f1; fi
}

# Description:
#   Verify every model file against models/MANIFEST.json, and the STL hash recorded in the
#   collision-set manifest. The JSON is read with the found interpreter's standard library.
# Args:
#   None
# Returns:
#   None
Verify_Files() {
    local listing path expected actual bad=0 count=0
    listing="$(cd "$CONST_ROOT" && "$PYTHON" -c 'import json,sys; [print(p, h) for p, h in json.load(open(sys.argv[1]))["sha256"].items()]' "$CONST_MODELS_MANIFEST")" \
        || Die "$CONST_MODELS_MANIFEST cannot be read." "Restore it from git: git checkout -- $CONST_MODELS_MANIFEST"
    while read -r path expected; do
        [ -z "$path" ] && continue
        count=$((count + 1))
        if [ ! -f "$CONST_ROOT/$path" ]; then Log ERROR "Missing model file: $path"; bad=$((bad + 1)); continue; fi
        actual="$(Sha256 "$CONST_ROOT/$path")"
        if [ "$actual" != "$expected" ]; then Log ERROR "Hash mismatch: $path"; bad=$((bad + 1)); fi
    done <<EOF
$listing
EOF
    if [ "$bad" -ne 0 ]; then
        Die "$bad of $count model files differ from $CONST_MODELS_MANIFEST." \
            "The vendored models must be byte-identical to upstream; restore them with: git checkout -- models"
    fi
    Log INFO "All $count model files match $CONST_MODELS_MANIFEST."
    if [ -f "$CONST_ROOT/$CONST_COLLISION_MANIFEST" ]; then
        read -r path expected <<EOF
$(cd "$CONST_ROOT" && "$PYTHON" -c 'import json,sys; m=json.load(open(sys.argv[1])); print(m["source_stl"], m["source_sha256"])' "$CONST_COLLISION_MANIFEST")
EOF
        actual="$(Sha256 "$CONST_ROOT/$path")"
        if [ "$actual" != "$expected" ]; then
            Die "$path differs from the STL the collision set was built from." \
                "Rebuild it with: .venv/bin/python Example/build_collision.py"
        fi
        Log INFO "The collision set matches $path."
    else
        Log WARNING "$CONST_COLLISION_MANIFEST is absent; build it with: .venv/bin/python Example/build_collision.py"
    fi
}

# Description:
#   Run the steps in order.
# Args:
#   (1) option [string]: Optional --detect-only.
# Returns:
#   None
Main() {
    trap 'On_Unexpected_Error $LINENO "$BASH_COMMAND"' ERR
    trap On_Interrupt INT TERM
    case "${1:-}" in
        ""|--detect-only) ;;
        *) Die "Unknown option '$1'." "Usage: ./install.sh [--detect-only]" ;;
    esac
    Detect_Platform
    Find_Python
    Log INFO "Plan: .venv with requirements.txt, then verify $CONST_MODELS_MANIFEST and $CONST_COLLISION_MANIFEST."
    if [ "${1:-}" = "--detect-only" ]; then Log INFO "Detect only: stopping before any change."; return 0; fi
    Ensure_Venv
    Install_Requirements
    Verify_Files
    Log INFO "Done. Run the tests with: .venv/bin/python -m pytest"
}

if [ "${BASH_SOURCE[0]}" = "$0" ]; then Main "$@"; fi
