# SPDX-FileCopyrightText: 2026 The DPsim Authors
# SPDX-License-Identifier: MPL-2.0

import os
import platform
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import pybind11
from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext


def env_flag(name: str, default: bool = False) -> bool:
    """Read a boolean environment variable."""
    value = os.environ.get(name)

    if value is None:
        return default

    value = value.strip().lower()

    if value in ("1", "true", "yes", "on"):
        return True

    if value in ("0", "false", "no", "off"):
        return False

    raise RuntimeError(
        f"Invalid value for {name}: {value!r}. "
        "Use one of: 1/0, true/false, yes/no, on/off."
    )


# VILLAS support is optional for the Python package.
#
# Core DPsim:
#   pip install .
#
# DPsim + VILLAS:
#   DPSIM_PYTHON_WITH_VILLAS=1 pip install .
#
# This avoids assuming that VILLASnode exists simply because the platform
# is Linux or macOS.
WITH_VILLAS = env_flag("DPSIM_PYTHON_WITH_VILLAS", default=False)


class CMakeExtension(Extension):
    def __init__(self, name: str, cmake_target: str | None = None):
        super().__init__(name, sources=[])
        self.cmake_target = cmake_target or name


class CMakeBuild(build_ext):
    def run(self):
        self._cmake_configured = False
        self._cmake_extdir = None
        super().run()

    def build_extension(self, ext):
        extdir = Path(self.get_ext_fullpath(ext.name)).parent.resolve()

        cfg = "Debug" if self.debug else "Release"

        print(f"Building CMake extension {ext.name} in {cfg} configuration")

        if not self._cmake_configured:
            self._configure_cmake(extdir, cfg)
            self._cmake_configured = True
            self._cmake_extdir = extdir

        elif extdir != self._cmake_extdir:
            raise RuntimeError(
                "All DPsim Python extensions must use the same output directory."
            )

        self._build_target(ext.cmake_target, cfg)

        self._generate_stubs(extdir, ext.name)

    def _configure_cmake(self, extdir: Path, cfg: str):
        build_temp = Path(self.build_temp).resolve()
        build_temp.mkdir(parents=True, exist_ok=True)

        sourcedir = Path(__file__).resolve().parent

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}",
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG={extdir}",
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE={extdir}",
            f"-DPython3_EXECUTABLE={sys.executable}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-Dpybind11_DIR={pybind11.get_cmake_dir()}",
            f"-DCMAKE_BUILD_TYPE={cfg}",
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
            # Python bindings are the purpose of this build.
            "-DWITH_PYBIND=ON",
            "-DFETCH_PYBIND=OFF",
            # Keep Python package builds small.
            "-DDPSIM_BUILD_EXAMPLES=OFF",
            "-DDPSIM_BUILD_DOC=OFF",
            # Reproducible dependencies for package builds.
            "-DFETCH_SPDLOG=ON",
            "-DFETCH_SUITESPARSE=ON",
            # Eigen is supplied by the platform/development environment.
            # Windows already fetches Eigen in the top-level CMake logic.
            "-DFETCH_EIGEN=OFF",
            # VILLAS is explicitly optional.
            f"-DWITH_VILLAS={'ON' if WITH_VILLAS else 'OFF'}",
        ]

        # ------------------------------------------------------------------
        # Windows
        # ------------------------------------------------------------------
        #
        # Do not hard-code x64. Support Win32, x64 and ARM64 Python builds.
        if platform.system() == "Windows":
            generator = os.environ.get("CMAKE_GENERATOR", "")

            single_config = any(name in generator for name in ("NMake", "Ninja"))

            if not single_config:
                platform_map = {
                    "win32": "Win32",
                    "win-amd64": "x64",
                    "win-arm64": "ARM64",
                }

                architecture = platform_map.get(self.plat_name)

                if architecture is not None:
                    cmake_args += ["-A", architecture]

        # ------------------------------------------------------------------
        # macOS
        # ------------------------------------------------------------------
        #
        # Respect ARCHFLAGS when pip/cibuildwheel requests a specific
        # architecture or universal build.
        if platform.system() == "Darwin":
            archflags = os.environ.get("ARCHFLAGS", "")
            architectures = re.findall(r"-arch\s+(\S+)", archflags)

            if architectures:
                cmake_args.append(
                    "-DCMAKE_OSX_ARCHITECTURES=" + ";".join(architectures)
                )

        # ------------------------------------------------------------------
        # User supplied CMake options
        # ------------------------------------------------------------------
        #
        # CMAKE_ARGS is the preferred interface.
        #
        # CMAKE_OPTS is retained for compatibility with the existing DPsim
        # setup.py behavior.
        #
        # User arguments are intentionally appended last so that explicit
        # options can override the defaults above.
        env = os.environ.copy()

        for variable in ("CMAKE_ARGS", "CMAKE_OPTS"):
            value = env.get(variable)

            if value:
                cmake_args.extend(shlex.split(value))

        command = [
            "cmake",
            str(sourcedir),
            *cmake_args,
        ]

        print(" ".join(command))

        subprocess.check_call(
            command,
            cwd=build_temp,
            env=env,
        )

    def _build_target(self, target: str, cfg: str):
        build_temp = Path(self.build_temp).resolve()

        jobs = self.parallel or os.cpu_count() or 4

        # This form works with Ninja, Makefiles and Visual Studio generators.
        command = [
            "cmake",
            "--build",
            ".",
            "--target",
            target,
            "--config",
            cfg,
            "--parallel",
            str(jobs),
        ]

        print(" ".join(command))

        subprocess.check_call(
            command,
            cwd=build_temp,
        )

    def _generate_stubs(self, extdir: Path, module: str):
        stub_env = os.environ.copy()

        stub_env["PYTHONPATH"] = (
            str(extdir) + os.pathsep + stub_env.get("PYTHONPATH", "")
        )

        with tempfile.TemporaryDirectory() as stub_tmp:
            stub_tmp_path = Path(stub_tmp)

            print(f"Generating stubs for {module}")

            result = subprocess.run(
                [
                    sys.executable,
                    "-m",
                    "pybind11_stubgen",
                    module,
                    "--output-dir",
                    str(stub_tmp_path),
                ],
                env=stub_env,
            )

            if result.returncode != 0:
                raise RuntimeError(
                    f"Stub generation for {module} failed "
                    f"(exit {result.returncode})"
                )

            src_dir = stub_tmp_path / module
            dst_dir = extdir / module

            if src_dir.is_dir():
                if dst_dir.exists():
                    shutil.rmtree(dst_dir)

                shutil.copytree(src_dir, dst_dir)

                # PEP 561 marker so type checkers recognise the package
                # as typed.
                (dst_dir / "py.typed").touch()

                return

            src_file = stub_tmp_path / f"{module}.pyi"

            if src_file.is_file():
                shutil.copy2(src_file, extdir)
                return

            raise RuntimeError(
                f"Stub generation for {module} succeeded, "
                "but no generated stub was found."
            )


ext_modules = [
    CMakeExtension("dpsimpy"),
]

if WITH_VILLAS:
    ext_modules.append(CMakeExtension("dpsimpyvillas"))


setup(
    ext_modules=ext_modules,
    cmdclass={
        "build_ext": CMakeBuild,
    },
    zip_safe=False,
)
