import os
import sys
import platform
import subprocess

import pybind11
from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    def __init__(self, name):
        super().__init__(name, sources=[])


class CMakeBuild(build_ext):
    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        cfg = "Debug" if self.debug else "Release"
        print("building CMake extension in %s configuration" % cfg)

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}",
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG={extdir}",
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE={extdir}",
            f"-DPython3_EXECUTABLE={sys.executable}",
            f"-Dpybind11_DIR={pybind11.get_cmake_dir()}",
            f"-DCMAKE_BUILD_TYPE={cfg}",
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5",
        ]

        if platform.system() == "Windows":
            cmake_args += ["-A", "x64"]
            targets = ["dpsimpy"]
            if self.parallel:
                build_args = ["--config", cfg, "--parallel", str(self.parallel)]
            else:
                build_args = ["--config", cfg, "--parallel", "4"]
        else:
            targets = ["dpsimpy", "dpsimpyvillas"]
            if self.parallel:
                build_args = ["--", "-j" + str(self.parallel)]
            else:
                build_args = ["--", "-j4"]

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        env = os.environ.copy()
        if env.get("CMAKE_OPTS"):
            cmake_args += env.get("CMAKE_OPTS").split()

        sourcedir = os.path.abspath(os.path.dirname(__file__))
        print(" ".join(["cmake", sourcedir] + cmake_args))
        subprocess.check_call(
            ["cmake", sourcedir] + cmake_args,
            cwd=self.build_temp,
            env=env,
        )

        for target in targets:
            print(" ".join(["cmake", "--build", ".", "--target", target] + build_args))
            subprocess.check_call(
                ["cmake", "--build", ".", "--target", target] + build_args,
                cwd=self.build_temp,
            )


ext_modules_list = [CMakeExtension("dpsimpy")]
if platform.system() != "Windows":
    ext_modules_list.append(CMakeExtension("dpsimpyvillas"))


setup(
    ext_modules=ext_modules_list,
    cmdclass={"build_ext": CMakeBuild},
    zip_safe=False,
)
