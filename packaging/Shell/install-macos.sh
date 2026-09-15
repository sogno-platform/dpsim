#!/usr/bin/env bash
#
# SPDX-FileCopyrightText: 2026 The DPsim Authors
# SPDX-License-Identifier: MPL-2.0
#
# Native DPsim development setup for macOS.
#
# This script reproduces the tested native macOS development environment:
#   - Homebrew build dependencies
#   - Eigen 3 from Homebrew
#   - Graphviz from Homebrew
#   - OpenMP runtime (libomp) from Homebrew
#   - SuiteSparse and spdlog fetched by DPsim
#   - repository-local Python 3.14 virtual environment: dpsim-python
#   - pybind11 Python bindings
#   - NumPy / pandas / SciPy / matplotlib
#   - JupyterLab / ipykernel / notebook support
#   - VILLAS dataprocessing Python package
#   - pytest tooling
#   - pre-commit tooling
#   - modern Homebrew Ruby for the markdownlint pre-commit hook
#   - modern Homebrew Node.js for the commitlint pre-commit hook
#   - native CMake/Ninja build including C++ examples
#   - independent `pip install .` Python package build
#   - Jupyter kernel registration
#
# Usage:
#   chmod +x packaging/Shell/install-macos.sh
#   ./packaging/Shell/install-macos.sh
#
# Clean project-local rebuild/retest:
#   CLEAN=1 ./packaging/Shell/install-macos.sh
#
# After setup, normal native rebuilds require no configure flags:
#   source dpsim-python/bin/activate
#   cmake --build build --parallel "$(sysctl -n hw.ncpu)"
#
# Optional environment variables:
#   BUILD_DIR=build
#   BUILD_TYPE=Release
#   JOBS=<number of parallel build jobs>
#   VENV_DIR=dpsim-python
#   CLEAN=0|1
#
set -euo pipefail

log() {
    printf '\n==> %s\n' "$*"
}

die() {
    printf '\nERROR: %s\n' "$*" >&2
    exit 1
}

# ---------------------------------------------------------------------------
# Platform checks
# ---------------------------------------------------------------------------

[[ "$(uname -s)" == "Darwin" ]] || die "This setup script is intended for macOS."

ARCH="$(uname -m)"
case "${ARCH}" in
    arm64|x86_64)
        ;;
    *)
        die "Unsupported macOS architecture: ${ARCH}"
        ;;
esac

log "Detected macOS architecture: ${ARCH}"

if ! xcode-select -p >/dev/null 2>&1; then
    cat >&2 <<'MSG'
Xcode Command Line Tools are required.

Install them with:
  xcode-select --install

After installation has completed, run this script again.
MSG
    exit 1
fi

# ---------------------------------------------------------------------------
# Homebrew
# ---------------------------------------------------------------------------

if ! command -v brew >/dev/null 2>&1; then
    log "Homebrew not found; installing Homebrew"
    command -v curl >/dev/null 2>&1 || die "curl is required to install Homebrew."
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

    # Homebrew's installer does not necessarily update the current shell.
    if [[ -x /opt/homebrew/bin/brew ]]; then
        eval "$(/opt/homebrew/bin/brew shellenv)"
    elif [[ -x /usr/local/bin/brew ]]; then
        eval "$(/usr/local/bin/brew shellenv)"
    else
        die "Homebrew installation completed, but brew could not be found."
    fi
fi

BREW_PREFIX="$(brew --prefix)"
log "Using Homebrew at ${BREW_PREFIX}"

# ---------------------------------------------------------------------------
# Homebrew dependencies
# ---------------------------------------------------------------------------

BREW_PACKAGES=(
    cmake
    ninja
    eigen@3
    graphviz
    libomp
    python@3.14
    ruby
    node
    pkg-config
)

log "Installing required Homebrew packages"
brew install "${BREW_PACKAGES[@]}"

EIGEN_PREFIX="$(brew --prefix eigen@3)"
GRAPHVIZ_PREFIX="$(brew --prefix graphviz)"
OPENMP_PREFIX="$(brew --prefix libomp)"
PYTHON_PREFIX="$(brew --prefix python@3.14)"
RUBY_PREFIX="$(brew --prefix ruby)"
NODE_PREFIX="$(brew --prefix node)"
PYTHON_BASE="${PYTHON_PREFIX}/bin/python3.14"

[[ -x "${PYTHON_BASE}" ]] \
    || die "Homebrew Python 3.14 executable not found at ${PYTHON_BASE}."

# Homebrew Ruby is keg-only. Put it ahead of Apple's system Ruby while this
# setup script runs so pre-commit creates markdownlint with a modern Ruby.
export PATH="${RUBY_PREFIX}/bin:${NODE_PREFIX}/bin:${BREW_PREFIX}/bin:${PATH}"
hash -r

# AppleClang needs the Homebrew libomp include/library locations. These are
# also inherited by the setuptools/pip CMake build below.
export CPPFLAGS="-I${OPENMP_PREFIX}/include${CPPFLAGS:+ ${CPPFLAGS}}"
export LDFLAGS="-L${OPENMP_PREFIX}/lib${LDFLAGS:+ ${LDFLAGS}}"

command -v ruby >/dev/null 2>&1 || die "Ruby could not be found after installing Homebrew Ruby."
RUBY_VERSION="$(ruby -e 'print RUBY_VERSION')"
RUBY_MAJOR="${RUBY_VERSION%%.*}"
RUBY_REST="${RUBY_VERSION#*.}"
RUBY_MINOR="${RUBY_REST%%.*}"
if (( RUBY_MAJOR < 3 || (RUBY_MAJOR == 3 && RUBY_MINOR < 1) )); then
    die "Ruby >= 3.1 is required for the markdownlint pre-commit hook; found ${RUBY_VERSION}."
fi

command -v node >/dev/null 2>&1 || die "Node.js could not be found after installing Homebrew Node.js."
NODE_VERSION="$(node --version)"
NODE_VERSION_NUMBER="${NODE_VERSION#v}"
NODE_MAJOR="${NODE_VERSION_NUMBER%%.*}"
if (( NODE_MAJOR < 20 )); then
    die "Node.js >= 20 is required for the commitlint pre-commit hook; found ${NODE_VERSION}."
fi

[[ -d "${OPENMP_PREFIX}/include" ]] || die "libomp include directory not found below ${OPENMP_PREFIX}."
[[ -d "${OPENMP_PREFIX}/lib" ]] || die "libomp library directory not found below ${OPENMP_PREFIX}."

log "Dependency versions"
printf '  CMake:    %s\n' "$(cmake --version | head -n1)"
printf '  Ninja:    %s\n' "$(ninja --version)"
printf '  Eigen 3:  %s\n' "$(brew list --versions eigen@3)"
printf '  Graphviz: %s\n' "$(dot -V 2>&1)"
printf '  libomp:   %s\n' "$(brew list --versions libomp)"
printf '  OpenMP:   %s\n' "${OPENMP_PREFIX}"
printf '  Python:   %s\n' "$("${PYTHON_BASE}" --version 2>&1)"
printf '  Ruby:     %s\n' "$(ruby --version)"
printf '  Node.js:  %s\n' "$(node --version)"

# ---------------------------------------------------------------------------
# Repository
# ---------------------------------------------------------------------------

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if REPO_ROOT="$(git -C "${SCRIPT_DIR}" rev-parse --show-toplevel 2>/dev/null)"; then
    :
else
    die "Could not determine the DPsim repository root. Run this script from a cloned DPsim repository."
fi

cd "${REPO_ROOT}"
log "DPsim repository: ${REPO_ROOT}"

# ---------------------------------------------------------------------------
# Verify required portability fixes
# ---------------------------------------------------------------------------

grep -q 'GRAPHVIZ_RENDERDATA_USES_SIZE_T' cmake/FindGraphviz.cmake \
    || die "Graphviz gvRenderData API compatibility detection is missing from cmake/FindGraphviz.cmake."
grep -q 'cgraph.h' cmake/FindGraphviz.cmake \
    || die "Graphviz-specific header discovery is missing from cmake/FindGraphviz.cmake."
grep -q 'FILESYSTEM_HAS_NATIVE_SUPPORT' cmake/FindFilesystem.cmake \
    || die "Native std::filesystem detection is missing from cmake/FindFilesystem.cmake."
grep -q 'template lpNorm<Eigen::Infinity>' dpsim/src/MNASolver.cpp \
    || die "The dependent-template lpNorm fix is missing from dpsim/src/MNASolver.cpp."
grep -q 'DPSIM_PYTHON_WITH_VILLAS' setup.py \
    || die "The platform-neutral optional VILLAS Python packaging logic is missing from setup.py."
grep -q 'FETCH_SPDLOG=ON' setup.py \
    || die "The reproducible fetched-spdlog Python packaging configuration is missing from setup.py."
grep -q 'shlex.split' setup.py \
    || die "Robust CMAKE_ARGS/CMAKE_OPTS parsing is missing from setup.py."

# ---------------------------------------------------------------------------
# Build / virtual-environment settings
# ---------------------------------------------------------------------------

BUILD_DIR="${BUILD_DIR:-build}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
JOBS="${JOBS:-$(sysctl -n hw.ncpu)}"
VENV_DIR="${VENV_DIR:-dpsim-python}"
CLEAN="${CLEAN:-0}"

case "${CLEAN}" in
    0|1) ;;
    *) die "CLEAN must be 0 or 1; got '${CLEAN}'." ;;
esac

if [[ "${VENV_DIR}" = /* ]]; then
    VENV_PATH="${VENV_DIR}"
else
    VENV_PATH="${REPO_ROOT}/${VENV_DIR}"
fi

if [[ "${BUILD_DIR}" = /* ]]; then
    BUILD_PATH="${BUILD_DIR}"
else
    BUILD_PATH="${REPO_ROOT}/${BUILD_DIR}"
fi

KERNEL_DIR="${HOME}/Library/Jupyter/kernels/dpsim-python"

# ---------------------------------------------------------------------------
# Optional clean project-local retest
# ---------------------------------------------------------------------------

if [[ "${CLEAN}" == "1" ]]; then
    [[ "${BUILD_PATH}" != "/" && "${BUILD_PATH}" != "${REPO_ROOT}" ]] \
        || die "Refusing to remove unsafe build path: ${BUILD_PATH}"
    [[ "${VENV_PATH}" != "/" && "${VENV_PATH}" != "${REPO_ROOT}" ]] \
        || die "Refusing to remove unsafe virtual-environment path: ${VENV_PATH}"

    log "CLEAN=1: removing project-local generated state"
    printf '  Build tree:         %s\n' "${BUILD_PATH}"
    printf '  Virtual environment:%s\n' " ${VENV_PATH}"
    printf '  Jupyter kernel:     %s\n' "${KERNEL_DIR}"

    rm -rf "${BUILD_PATH}" "${VENV_PATH}" "${KERNEL_DIR}"
    rm -rf python/src/dpsim.egg-info
fi

# ---------------------------------------------------------------------------
# Python virtual environment
# ---------------------------------------------------------------------------

if [[ ! -x "${VENV_PATH}/bin/python" ]]; then
    log "Creating Python virtual environment: ${VENV_PATH}"
    "${PYTHON_BASE}" -m venv "${VENV_PATH}"
else
    log "Using existing Python virtual environment: ${VENV_PATH}"
fi

PYTHON="${VENV_PATH}/bin/python"

PYTHON_MM="$("${PYTHON}" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
[[ "${PYTHON_MM}" == "3.14" ]] \
    || die "${VENV_PATH} uses Python ${PYTHON_MM}; Python 3.14 is required. Remove the environment and rerun this script."

log "Upgrading Python packaging tools"
"${PYTHON}" -m pip install --upgrade pip setuptools wheel

log "Installing pybind11 build dependencies"
"${PYTHON}" -m pip install \
    "pybind11>=3.0.0" \
    "pybind11-stubgen>=2.5"

log "Installing scientific Python dependencies"
"${PYTHON}" -m pip install \
    "numpy>=2.0.0" \
    "pandas>=2.0.0" \
    "scipy>=1.10.0" \
    matplotlib

log "Installing Jupyter notebook dependencies"
"${PYTHON}" -m pip install \
    jupyterlab \
    ipykernel \
    ipywidgets \
    nbformat \
    "nbconvert==7.17.1"

log "Installing DPsim notebook/data-processing dependencies"
"${PYTHON}" -m pip install "villas-dataprocessing>=0.2.6"

log "Installing Python test dependencies"
"${PYTHON}" -m pip install pytest pytest-xdist pytest-cov pyyaml

log "Installing pre-commit"
"${PYTHON}" -m pip install pre-commit

PYBIND11_DIR="$("${PYTHON}" -m pybind11 --cmakedir)"
[[ -d "${PYBIND11_DIR}" ]] || die "pybind11 CMake directory not found: ${PYBIND11_DIR}"

log "Python development environment"
printf '  Virtual env: %s\n' "${VENV_PATH}"
printf '  Python:      %s\n' "$("${PYTHON}" --version)"
printf '  pybind11:    %s\n' "$("${PYTHON}" -c 'import pybind11; print(pybind11.__version__)')"
printf '  pybind dir:  %s\n' "${PYBIND11_DIR}"
printf '  matplotlib:  %s\n' "$("${PYTHON}" -c 'import matplotlib; print(matplotlib.__version__)')"
printf '  JupyterLab:  %s\n' "$("${PYTHON}" -c 'import jupyterlab; print(jupyterlab.__version__)')"

# ---------------------------------------------------------------------------
# CMake dependency paths and shared macOS configuration
# ---------------------------------------------------------------------------

# Put Eigen 3 first deliberately. Do not allow another Eigen installation to
# win package discovery.
CMAKE_PREFIX_PATH_VALUE="${EIGEN_PREFIX};${GRAPHVIZ_PREFIX};${OPENMP_PREFIX};${BREW_PREFIX}"
EIGEN3_DIR="${EIGEN_PREFIX}/share/eigen3/cmake"
if [[ ! -d "${EIGEN3_DIR}" ]]; then
    EIGEN3_DIR="$(find "${EIGEN_PREFIX}" -type d -path '*/eigen3/cmake' -print -quit 2>/dev/null || true)"
fi
[[ -n "${EIGEN3_DIR}" && -d "${EIGEN3_DIR}" ]] \
    || die "Could not locate Eigen3Config.cmake below ${EIGEN_PREFIX}."

OPENMP_INCLUDE_DIR="${OPENMP_PREFIX}/include"
OPENMP_LIBRARY="${OPENMP_PREFIX}/lib/libomp.dylib"
[[ -f "${OPENMP_LIBRARY}" ]] || die "OpenMP library not found at ${OPENMP_LIBRARY}."

# Keep all dependency-discovery and macOS-specific CMake settings in one list.
# The native developer build and the independent pip package build both derive
# their configuration from this list so they cannot silently diverge.
CMAKE_SHARED_ARGS=(
    "-DCMAKE_BUILD_TYPE=${BUILD_TYPE}"
    "-DCMAKE_OSX_ARCHITECTURES=${ARCH}"
    "-DCMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH_VALUE}"
    "-DEigen3_DIR=${EIGEN3_DIR}"
    "-DGraphviz_ROOT=${GRAPHVIZ_PREFIX}"
    "-DOpenMP_ROOT=${OPENMP_PREFIX}"
    "-DOpenMP_CXX_FLAG=-Xclang -fopenmp"
    "-DOpenMP_CXX_INCLUDE_DIR=${OPENMP_INCLUDE_DIR}"
    "-DOpenMP_CXX_LIB_NAMES=libomp"
    "-DOpenMP_libomp_LIBRARY=${OPENMP_LIBRARY}"
    "-DPython3_EXECUTABLE=${PYTHON}"
    "-Dpybind11_DIR=${PYBIND11_DIR}"
    "-DFETCH_EIGEN=OFF"
    "-DFETCH_SUITESPARSE=ON"
    "-DFETCH_SPDLOG=ON"
    "-DFETCH_PYBIND=OFF"
    "-DWITH_PYBIND=ON"
    "-DWITH_GRAPHVIZ=ON"
    "-DWITH_OPENMP=ON"
    "-DWITH_CIM=OFF"
    "-DWITH_VILLAS=OFF"
)

# ---------------------------------------------------------------------------
# Configure native developer build
# ---------------------------------------------------------------------------

log "Configuring native DPsim build"
printf '  Build directory: %s\n' "${BUILD_PATH}"
printf '  Build type:      %s\n' "${BUILD_TYPE}"
printf '  Architecture:    %s\n' "${ARCH}"
printf '  Parallel jobs:   %s\n' "${JOBS}"
printf '  Eigen prefix:    %s\n' "${EIGEN_PREFIX}"
printf '  Graphviz prefix: %s\n' "${GRAPHVIZ_PREFIX}"
printf '  OpenMP prefix:   %s\n' "${OPENMP_PREFIX}"
printf '  Python:          %s\n' "${PYTHON}"

cmake \
    -S . \
    -B "${BUILD_DIR}" \
    -G Ninja \
    "${CMAKE_SHARED_ARGS[@]}" \
    -DDPSIM_BUILD_EXAMPLES=ON \
    -DDPSIM_BUILD_DOC=OFF

# Verify Graphviz did not regress to the broad Homebrew include directory.
if [[ -f "${BUILD_PATH}/CMakeCache.txt" ]]; then
    GRAPHVIZ_INCLUDE_CACHE="$(grep '^GRAPHVIZ_INCLUDE_DIR:PATH=' "${BUILD_PATH}/CMakeCache.txt" | cut -d= -f2- || true)"
    if [[ -n "${GRAPHVIZ_INCLUDE_CACHE}" ]]; then
        case "${GRAPHVIZ_INCLUDE_CACHE}" in
            */include/graphviz)
                ;;
            *)
                die "Graphviz include directory is unexpectedly broad: ${GRAPHVIZ_INCLUDE_CACHE}. Expected a graphviz-specific include directory."
                ;;
        esac
    fi
fi

if [[ -f "${BUILD_PATH}/CMakeCache.txt" ]]; then
    grep -q '^WITH_OPENMP:BOOL=ON$' "${BUILD_PATH}/CMakeCache.txt" \
        || die "WITH_OPENMP is not enabled in ${BUILD_PATH}/CMakeCache.txt."

    CONFIG_HEADER="${BUILD_PATH}/dpsim/include/dpsim/Config.h"
    [[ -f "${CONFIG_HEADER}" ]] || die "Generated DPsim configuration header not found: ${CONFIG_HEADER}"
    grep -q '^#define WITH_OPENMP' "${CONFIG_HEADER}" \
        || die "DPsim was configured without OpenMP support."

    log "Native CMake configuration includes OpenMP"
fi

# ---------------------------------------------------------------------------
# Build native developer tree
# ---------------------------------------------------------------------------

log "Building DPsim, Python bindings and C++ examples"
cmake --build "${BUILD_DIR}" --parallel "${JOBS}"

if ! find "${BUILD_PATH}" -maxdepth 2 -type f -name 'dpsimpy*.so' -print -quit | grep -q .; then
    die "The native CMake build completed but dpsimpy was not found below ${BUILD_PATH}."
fi

# ---------------------------------------------------------------------------
# Independent pip package build/install
# ---------------------------------------------------------------------------

# `pip install .` invokes its own CMake build through setup.py. Convert the
# exact same CMake settings above to one shell-parseable string. setup.py uses
# shlex.split(), so the escaped OpenMP compiler flag remains one CMake argument.
printf -v PIP_CMAKE_ARGS '%q ' "${CMAKE_SHARED_ARGS[@]}"
PIP_CMAKE_ARGS="${PIP_CMAKE_ARGS% }"

log "Preparing independent pip package build"
rm -rf \
    "${BUILD_PATH}"/temp.macosx-* \
    "${BUILD_PATH}"/lib.macosx-* \
    "${BUILD_PATH}"/bdist.macosx-*

log "Installing DPsim into ${VENV_PATH} with pip"
CMAKE_GENERATOR=Ninja \
ARCHFLAGS="-arch ${ARCH}" \
CPPFLAGS="${CPPFLAGS}" \
LDFLAGS="${LDFLAGS}" \
CMAKE_ARGS="${PIP_CMAKE_ARGS}" \
DPSIM_PYTHON_WITH_VILLAS=0 \
"${PYTHON}" -m pip install . -v

# If setup.py preserved its CMake build tree, verify the independent package
# build also had OpenMP enabled. This specifically guards against the macOS
# libomp discovery regression seen when plain `pip install .` is run without
# the required CMake arguments.
PIP_CMAKE_CACHE="$(find "${BUILD_PATH}" -maxdepth 2 -type f -path '*/temp.macosx-*/CMakeCache.txt' -print -quit 2>/dev/null || true)"
if [[ -n "${PIP_CMAKE_CACHE}" ]]; then
    grep -q '^WITH_OPENMP:BOOL=ON$' "${PIP_CMAKE_CACHE}" \
        || die "The pip/setuptools CMake build did not enable WITH_OPENMP: ${PIP_CMAKE_CACHE}"
    log "pip/setuptools CMake configuration includes OpenMP"
else
    log "pip temporary CMake cache was not retained; import verification will continue"
fi

# ---------------------------------------------------------------------------
# Python import verification
# ---------------------------------------------------------------------------

log "Verifying installed Python environment"
"${PYTHON}" - <<'PY'
import sys

import dpsim
import dpsimpy
import ipykernel
import jupyterlab
import matplotlib
import matplotlib.pyplot as plt
import nbconvert
import nbformat
import numpy
import pandas
import scipy
from villas.dataprocessing.readtools import read_timeseries_csv

print("Python:", sys.executable)
print("dpsimpy:", dpsimpy.__file__)
print("dpsim:", dpsim.__file__)
print("NumPy:", numpy.__version__)
print("pandas:", pandas.__version__)
print("SciPy:", scipy.__version__)
print("matplotlib:", matplotlib.__version__)
print("JupyterLab:", jupyterlab.__version__)
print("ipykernel:", ipykernel.__version__)
print("nbformat:", nbformat.__version__)
print("nbconvert:", nbconvert.__version__)
print("VILLAS dataprocessing import: OK")
print("DPsim Python installation: OK")
PY

"${PYTHON}" -m pip check

# ---------------------------------------------------------------------------
# Jupyter kernel
# ---------------------------------------------------------------------------

log "Registering Jupyter kernel: DPsim Python"
rm -rf "${KERNEL_DIR}"
"${PYTHON}" -m ipykernel install \
    --user \
    --name dpsim-python \
    --display-name "DPsim Python"

KERNEL_JSON="${KERNEL_DIR}/kernel.json"
[[ -f "${KERNEL_JSON}" ]] || die "Jupyter kernelspec was not created at ${KERNEL_JSON}."
grep -q "${VENV_PATH}/bin/python" "${KERNEL_JSON}" \
    || die "The DPsim Jupyter kernelspec does not point to ${VENV_PATH}/bin/python."

# ---------------------------------------------------------------------------
# Pre-commit
# ---------------------------------------------------------------------------

log "Preparing pre-commit environments"
printf '  Ruby used for hook installation:    %s\n' "$(command -v ruby)"
printf '  Ruby version:                       %s\n' "$(ruby --version)"
printf '  Node.js used for hook installation: %s\n' "$(command -v node)"
printf '  Node.js version:                    %s\n' "$(node --version)"

"${VENV_PATH}/bin/pre-commit" clean
"${VENV_PATH}/bin/pre-commit" gc || true

log "Installing pre-commit hooks"
"${VENV_PATH}/bin/pre-commit" validate-config
"${VENV_PATH}/bin/pre-commit" install --install-hooks

# Smoke-test commitlint without creating a real Git commit. commitlint itself
# uses `commitlint --edit` and therefore reads .git/COMMIT_EDITMSG. pre-commit
# additionally requires --commit-msg-filename for a manual commit-msg run.
GIT_DIR="$(git rev-parse --absolute-git-dir)"
COMMITLINT_TEST_FILE="${GIT_DIR}/COMMIT_EDITMSG"
COMMITLINT_BACKUP=""

if [[ -f "${COMMITLINT_TEST_FILE}" ]]; then
    COMMITLINT_BACKUP="$(mktemp)"
    cp "${COMMITLINT_TEST_FILE}" "${COMMITLINT_BACKUP}"
fi

restore_commitlint_test_file() {
    if [[ -n "${COMMITLINT_BACKUP}" ]]; then
        mv "${COMMITLINT_BACKUP}" "${COMMITLINT_TEST_FILE}"
    else
        rm -f "${COMMITLINT_TEST_FILE}"
    fi
}

trap restore_commitlint_test_file EXIT
printf 'build: test macOS development setup\n' > "${COMMITLINT_TEST_FILE}"
"${VENV_PATH}/bin/pre-commit" run commitlint \
    --hook-stage commit-msg \
    --commit-msg-filename "${COMMITLINT_TEST_FILE}"
restore_commitlint_test_file
trap - EXIT

# ---------------------------------------------------------------------------
# Final summary
# ---------------------------------------------------------------------------

log "DPsim macOS setup completed successfully"
printf '\nRepository:\n  %s\n' "${REPO_ROOT}"
printf '\nBuild directory:\n  %s\n' "${BUILD_PATH}"
printf '\nVirtual environment:\n  %s\n' "${VENV_PATH}"
printf '\nJupyter kernel:\n  DPsim Python\n'
printf '\nOpenMP:\n  %s\n' "${OPENMP_PREFIX}"
printf '\nExample executables are located below:\n  %s/dpsim/examples/cxx\n' "${BUILD_PATH}"

cat <<EOF2

Normal development rebuild (no CMake configure flags required):

  source "${VENV_PATH}/bin/activate"
  cmake --build "${BUILD_DIR}" --parallel "\$(sysctl -n hw.ncpu)"

The macOS configure flags are stored in:

  ${BUILD_PATH}/CMakeCache.txt

For a completely clean project-local setup/build retest:

  CLEAN=1 ./packaging/Shell/install-macos.sh

If Python bindings/package files change, the installer is the reproducible way
of rebuilding and reinstalling both the native tree and the Python package:

  ./packaging/Shell/install-macos.sh

Run all repository pre-commit checks manually with:

  source "${VENV_PATH}/bin/activate"
  pre-commit run --all-files

In VS Code/Jupyter select the kernel:

  DPsim Python
EOF2
