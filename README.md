# DPsim

Dynamic Phasor simulation library in C++

## Build

### Windows (Visual Studio)

1. Install Visual Studio 2017 C++ Desktop development package plus Windows 8.1 SDK
2. Install [Python 3](https://www.python.org/downloads/) using the normal installer, and add Python to your PATH.
2. Create a new folder name `Libraries` in the repository root, download [Eigen](http://eigen.tuxfamily.org) and copy it to `Libraries/eigen`.
3. Install [CMake](https://cmake.org/download/), use its GUI (with `Source` as the source path) to create project files for Visual Studio. If you installed the 64-bit version of Python, make sure to use the "Win64" version of the Visual Studio generator.
4. Use Visual Studio and the Release configuration to build the dpsim Python module. To install it, build the INSTALL project.

### Linux (CMake)

1. Make sure that Eigen, libxml2 (including development headers) and CMake are installed, e.g. using your distro's package manager.
2. Build and install libvillas-ext from [VILLASnode](https://git.rwth-aachen.de/VILLASframework/VILLASnode):
```
$ make
# make install-libvillas-ext
```
There are also RPMs available for Fedora:
```
$ wget https://villas.fein-aachen.org/packages/villas.repo
# mv villas.repo /etc/yum.repos.d
# dnf -y install villas-node-devel
```
3. Generate a makefile with CMake and use it to build the project:
```
$ mkdir build
$ cd build
$ cmake ..
$ make
```
4. Install the generated module:
```
# make install
```
Another option is to manually rename the generated module (from build/Source/libdpsim.so to dpsim.so) and ensure that dpsim.so is in somewhere in your `PYTHONPATH`.

## Documentation

1. Install [Sphinx](http://www.sphinx-doc.org/en/stable/index.html)
  - either from your Linux distribution's repo
  - or [manually](http://www.sphinx-doc.org/en/stable/install.html#windows-install-python-and-sphinx) on Windows
  - if you used the installer which already adds Python to your path and installs pip, you basically only need to run `pip install sphinx`
2. Generate the documentation by running Sphinx via CMake:
```
$ mkdir -p build
$ cd build
$ cmake ..
$ make docs
```
4. The resulting documentation will be generated in `Documentation/html/`

## Copyright

2017, Institute for Automation of Complex Power Systems, EONERC

## License

This project is released under the terms of the [GPL version 3](COPYING.md).

```
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
```

For other licensing options please consult [Prof. Antonello Monti](mailto:amonti@eonerc.rwth-aachen.de).

## Contact

[![EONERC ACS Logo](Documentation/eonerc_logo.png)](http://www.acs.eonerc.rwth-aachen.de)

- Markus Mirz <mmirz@eonerc.rwth-aachen.de>

[Institute for Automation of Complex Power Systems (ACS)](http://www.acs.eonerc.rwth-aachen.de)
[EON Energy Research Center (EONERC)](http://www.eonerc.rwth-aachen.de)
[RWTH University Aachen, Germany](http://www.rwth-aachen.de)
