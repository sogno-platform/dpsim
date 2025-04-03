import os
import shlex
import sys
import platform
import subprocess

from setuptools import setup, find_packages, Extension
from setuptools.command.build_ext import build_ext


class CMakeExtension(Extension):
    def __init__(self, name):
        Extension.__init__(self, name, sources=[])


class CMakeBuild(build_ext):
    def build_extension(self, ext):

        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        cfg = 'Debug' if self.debug else 'Release'
        print('building CMake extension in %s configuration' % cfg)

        cmake_args = [
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY={extdir}",
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_DEBUG={extdir}",
            f"-DCMAKE_LIBRARY_OUTPUT_DIRECTORY_RELEASE={extdir}",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DCMAKE_BUILD_TYPE={cfg}",  # Not used on MSVC, but no harm
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
        ]

        if platform.system() == 'Windows':
            cmake_args += ['-A', 'x64']
            build_args = ['--', '/m']
        else:
            if self.parallel:
                build_args = ['--', '-j' + str(self.parallel)]
            else:
                build_args = ['--', '-j4']

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        env = os.environ.copy()
        if env.get('CMAKE_OPTS'):
            cmake_args += shlex.split(env.get('CMAKE_OPTS'))

        # CMakeLists.txt is in the same directory as this setup.py file
        source_dir = os.path.abspath(os.path.dirname(__file__))

        cmake_cmd = ['cmake', source_dir] + cmake_args
        build_cmd = ['cmake', '--build', '.', '--target', 'pybind'] + build_args

        print(shlex.join(cmake_cmd))
        subprocess.check_call(cmake_cmd, cwd=self.build_temp, env=env)

        print(shlex.join(build_cmd))
        subprocess.check_call(build_cmd, cwd=self.build_temp)


setup(
    packages=find_packages('python/src'),
    package_dir={"dpsim": "python/src/dpsim"},
    python_requires=">=3.8",
    setup_requires=[
        'pytest-runner',
        'wheel'
    ],
    tests_require=[
        'pytest',
        'pyyaml',
        'nbformat',
        'nbconvert'
    ],
    ext_modules=[
        CMakeExtension('dpsimpy')
    ],
    cmdclass={
        'build_ext': CMakeBuild
    },
    zip_safe=False
)
