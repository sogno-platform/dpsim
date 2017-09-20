## Build

Make sure that all subrepos (CIM-XML-Parser and arabica) are checked out.

### Windows

TODO: build with Python under Windows?

1. Install Visual Studio 2017 C++ Desktop development package plus Windows 8.1 SDK
2. Create a new folder name `Libraries` in the repository root, download [Eigen](http://eigen.tuxfamily.org) and copy it to `Libraries/eigen`.
3. Install [CMake](https://cmake.org/download/), use its GUI (with `Source` as the source path) to create project files for Visual Studio.
4. Use Visual Studio to build the project.

#### Linux

1. Make sure that Eigen, libxml2 (including development headers) and cmake are installed, e.g. using your distro's package manager.
2. Build and install libvillas-ext from [VILLASnode](https://git.rwth-aachen.de/VILLASframework/VILLASnode):
```
$ make
# make install
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

## Basic git commands
* initial download: git clone [url]
* download newest version: git pull
* see changed files: git status
* add modification or new file to commit: git add --all OR git add [filename]
* create commit: git commit -m 'your comment'
* push commits from local repo to server: git push
