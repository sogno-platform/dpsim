import os
import re
import sys
import platform
import subprocess

from setuptools import setup, find_packages, Extension
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion

class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources = [])
        self.sourcedir = os.path.abspath(sourcedir)

class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        if platform.system() == "Windows":
            cmake_version = LooseVersion(re.search(r'version\s*([\d.]+)', out.decode()).group(1))
            if cmake_version < '3.1.0':
                raise RuntimeError("CMake >= 3.1.0 is required on Windows")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
        cmake_args = ['-DCMAKE_BUILD_WITH_INSTALL_RPATH=TRUE',
                      '-DPYTHON_EXECUTABLE=' + sys.executable,
                      '-DBUILD_EXAMPLES=OFF']

        cfg = 'Debug' if self.debug else 'Release'
        build_args = ['--config', cfg]

        if platform.system() == "Windows":
            cmake_args += ['-DCMAKE_RUNTIME_OUTPUT_DIRECTORY=' + extdir]
            if sys.maxsize > 2**32:
                cmake_args += ['-A', 'x64']
            build_args += ['--', '/m']
        else:
            cmake_args += ['-DCMAKE_LIBRARY_OUTPUT_DIRECTORY=' + extdir]
            build_args += ['--', '-j4']

        env = os.environ.copy()
        env['CXXFLAGS'] = '{} -DVERSION_INFO=\\"{}\\"'.format(env.get('CXXFLAGS', ''), self.distribution.get_version())

        if not os.path.exists(self.build_temp):
            os.makedirs(self.build_temp)

        subprocess.check_call(['cmake', ext.sourcedir] + cmake_args, cwd = self.build_temp, env = env)
        subprocess.check_call(['cmake', '--build', '.'] + build_args, cwd = self.build_temp)

def read(fname):
    dname = os.path.dirname(__file__)
    fname = os.path.join(dname, fname)

    try:
        import m2r
        return m2r.parse_from_file(fname)
    except ImportError:
        with open(fname) as f:
            return f.read()

setup(
    name = "dpsim",
    version = "0.1.1",
    author = "Steffen Vogel",
    author_email = "stvogel@eonerc.rwth-aachen.de",
    description = "DPsim is a real-time power system simulator that operates in the dynamic phasor as well as electromagentic transient domain.",
    license = "GPL-3.0",
    keywords = "simulation power system real-time",
    url = "https://dpsim.fein-aachen.org",
    packages = find_packages('Source/Python'),
    package_dir = {
        'dpsim': 'Source/Python/dpsim'
    },
    long_description = read('README.md'),
    classifiers = [
        "Development Status :: 4 - Beta",
        "Topic :: Scientific/Engineering",
        "License :: OSI Approved :: GNU General Public License v3 or later (GPLv3+)",
        "Operating System :: MacOS :: MacOS X",
        "Operating System :: Microsoft :: Windows",
        "Operating System :: POSIX :: Linux",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: Implementation :: CPython"
    ],
    install_requires = [
        'acs-dataprocessing>=0.1.2',
    ],
    setup_requires = [
        'm2r',
        'breathe',
        'pytest-runner',
        'wheel'
    ],
    tests_require=[
        'pytest',
    ],
    ext_modules = [
        CMakeExtension('_dpsim')
    ],
    cmdclass = {
        'build_ext': CMakeBuild
    },
    zip_safe = False
)
