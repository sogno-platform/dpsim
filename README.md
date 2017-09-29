## Build

Make sure that all subrepos (CIM-XML-Parser and arabica) are checked out.

### Windows (Visual Studio)

1. Install Visual Studio 2017 C++ Desktop development package plus Windows 8.1 SDK
2. Install [Python 3](https://www.python.org/downloads/) using the normal installer, and add Python to your PATH.
2. Create a new folder name `Libraries` in the repository root, download [Eigen](http://eigen.tuxfamily.org) and copy it to `Libraries/eigen`.
3. Install [CMake](https://cmake.org/download/), use its GUI (with `Source` as the source path) to create project files for Visual Studio. If you installed the 64-bit version of Python, make sure to use the "Win64" version of the Visual Studio generator.
4. Use Visual Studio and the Release configuration to build the dpsim Python module. To install it, build the INSTALL project.

### Linux

1. Make sure that Eigen, libxml2 (including development headers) and cmake are installed, e.g. using your distro's package manager.
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
$ mkdir Source/build
$ cd Source/build
$ cmake ..
$ make
```
4. Install the generated module:
```
# make install
```
Another option is to manually rename the generated module (from Source/build/libdpsim.so
to dpsim.so) and ensure that dpsim.so is in somewhere in your PYTHONPATH.

## Documentation

Install [sphinx](http://www.sphinx-doc.org/en/stable/index.html), either from your
Linux distribution's repo or [manually](http://www.sphinx-doc.org/en/stable/install.html#windows-install-python-and-sphinx)
on Windows (if you used the installer which already adds Python to your path and
installs pip, you basically only need to run `pip install sphinx` at the
command prompt). Then, execute `make html` in the Documentation folder to generate
the Python API documentation. Since sphinx generates the documentation by
importing the `dpsim` module, make sure it is properly installed or adjust your
`PYTHONPATH` accordingly. The resulting documentation will be generated in
`_build/html`.
