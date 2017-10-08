# DPsim

Dynamic Phasor simulation library in C++

## Installation

Take a look at the [build instructions](Documentation/build.rst).

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
