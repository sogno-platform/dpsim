import os
import re
import sys
import platform
import subprocess

from setuptools import setup, find_packages, Extension
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion


class CMakeExtension(Extension):
    def __init__(self, name):
        Extension.__init__(self, name, sources=[])

class CMakeBuild(build_ext):
    def build_extension(self, ext):

        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))

        debug = int(os.environ.get("DEBUG", 0)) if self.debug is None else self.debug
        cfg = 'Debug' if self.debug else 'Release'
        print('building CMake extension in %s configuration' % cfg)

        cmake_args = [
            #f"-DCMAKE_BUILD_WITH_INSTALL_RPATH=TRUE",
            f"-DPYTHON_EXECUTABLE={sys.executable}",
            f"-DCMAKE_BUILD_TYPE={cfg}",  # not used on MSVC, but no harm
            f"-DBUILD_EXAMPLES=OFF"
        ]

        if platform.system() == 'Windows':
            cmake_args += ['-DCMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG=' + extdir]
            cmake_args += ['-DCMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE=' + extdir]
            if sys.maxsize > 2**32:
                cmake_args += ['-A', 'x64']
            build_args = ['--', '/m']
        else:
            cmake_args += ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir]
            if self.parallel:
                build_args = ['--', '-j' + str(self.parallel)]
            else:
                build_args = ['--', '-j4']

        env = os.environ.copy()
        cmake_args.append('-DCMAKE_CXX_FLAGS={} -DVERSION_INFO=\'{}\''
                .format(env.get('CXXFLAGS', ''), self.distribution.get_version()))

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        if env.get('CMAKE_OPTS'):
            cmake_args += env.get('CMAKE_OPTS').split(' ')

        # CMakeLists.txt is in the same directory as this setup.py file
        sourcedir = os.path.abspath(os.path.dirname(__file__))
        print(' '.join(['cmake', sourcedir] + cmake_args))
        subprocess.check_call(
            ['cmake', sourcedir] + cmake_args, cwd=self.build_temp, env=env
        )

        print(' '.join(['cmake', '--build', '.', '--target', 'dpsimpy'] + build_args))
        subprocess.check_call(
            ['cmake', '--build', '.', '--target', 'dpsimpy'] + build_args, cwd=self.build_temp
        )

setup(
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
