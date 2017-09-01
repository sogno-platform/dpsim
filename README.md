## Tasks
- Sychronous generator model interfaced through current source
- connect with CIM parser
- include boost library for additional math functions and general tasks

## Setup

Make sure that all subrepos (CIM-XML-Parser and arabica) are checked out.

### Windows

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

## Basic git commands
* initial download: git clone [url]
* download newest version: git pull
* see changed files: git status
* add modification or new file to commit: git add --all OR git add [filename]
* create commit: git commit -m 'your comment'
* push commits from local repo to server: git push

## Netlist structure
* **separate items with comma and end line with comma**
* see examples and store new netlist files in DPSolver/netlists
* first line: time step, final time e.g. 0.001,0.1,
* following lines: class name, component name, node1, node2, paramter1, parameter2... e.g. Resistor,R1,0,1,10
