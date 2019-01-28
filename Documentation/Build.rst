Build
========

Development Version
-------------------

In the repository, there is a Docker file with all required dependencies::

    $ cd dpsim
    $ docker build -t rwthacs/dpsim-dev -f Packaging/Docker/Dockerfile.dev .

Alternatively, the image can be pulled from DockerHub like so::

    $ docker pull rwthacs/dpsim-dev

To run Jupyter lab notebooks from the dpsim repository, call::

    $ git clone --recurse-submodules git@git.rwth-aachen.de:acs/core/simulation/dpsim.git
    $ cd dpsim
    $ docker run -it -p 8888:8888 -v $(pwd):/dpsim --privileged rwthacs/dpsim-dev bash

For Windows, you might need to specify the current directory like this::

    $ docker run -it -p 8888:8888 -v ${pwd}:/dpsim --privileged rwthacs/dpsim-dev bash

The DPsim C++ and DPsim Python library can be built as follows::

    $ cd dpsim
    $ mkdir build
    $ cd build
    $ cmake ..
    $ cmake --build . --target dpsim_python

To build everything run:::

    $ cmake --build .

Finally, the Python package is added to the path and Jupyter started::

    $ export PYTHONPATH=$(pwd)/Source/Python:$(pwd)/../Source/Python
    $ cd /dpsim
    $ jupyter lab --ip="0.0.0.0" --allow-root --no-browser
    

Documentation
-------------

Python
^^^^^^

- Install `Sphinx`_ or use the Docker image.
- Generate the Python documentation by running Sphinx via CMake::

    $ mkdir -p build
    $ cd build
    $ cmake ..
    $ make docs

- The resulting documentation will be generated in ``Documentation/html/``

C++
^^^

- Install `Doxygen`_ or use the Docker image.
- Generate the C++ documentation by running Doxygen via CMake:::

      $ mkdir -p build
      $ cd build
      $ cmake ..
      $ make docs_cxx

- The resulting documentation will be generated in ``Documentation/html/Cxx``

.. _Sphinx: http://www.sphinx-doc.org/en/master/index.html
.. _Doxygen: http://www.doxygen.org/


Python Package
--------------

Docker
^^^^^^

1. Follow the steps in for the development version to set up the Docker container.

2. To build the Python package run::

    $ python3 setup.py bdist


CMake for Linux
^^^^^^^^^^^^^^^

The most recent list of requirements can be found in the Dockerfiles. 

1. Make sure that the required dependencies are installed.    

   **Note:** There are currently no Debian packages for `villas-node` and `libcimpp16v29a`.
   If you want to use these optional feature, you have to build them.

2. Fetch sources::

    $ git clone --recurse-submodules git@git.rwth-aachen.de:acs/core/simulation/dpsim.git
    $ cd dpsim

3. Building the libraries and the Python package works like in Docker.

4. To install the generated Python module to your system::

    $ make install

CMake for Windows
^^^^^^^^^^^^^^^^^

1. Make sure that the required dependecies are installed::

   - Visual Studio 2017 and the C++ Desktop development package
   - `CMake`_ for Windows
   - `Git for Windows`_
   - For Python support, install `Python 3`_ using the normal installer or a distribution like   Anaconda, and add Python to your PATH.
   
2. Fetch sources::

      $ git clone --recursive git@git.rwth-aachen.de:acs/core/simulation/dpsim.git
      $ cd dpsim

3. Open a windows command prompt and navigate into the new DPsim folder.

4. Generate a Visual Studio project with CMake and use it to build the project::

      $ mkdir build
      $ cd build
      $ cmake -G "Visual Studio 15 2017 Win64" ..

5. Open Visual Studio and load the Visual Studio project from the build directory within the DPsim folder.

6. You can either build the project from within Visual Studio or from the command line by running the following command in the windows command prompt::

    $ cmake --build .

7. To build the Python package run::

    $ python3 setup.py bdist
 
8. To install the Python package use Visual Studio and the Release configuration to build the DPsim Python module and then build the INSTALL project.

.. _`Python 3`: https://www.python.org/downloads/
.. _CMake: https://cmake.org/download/
.. _`Git for Windows`: https://git-scm.com/download/win
.. _VILLASnode: https://git.rwth-aachen.de/VILLASframework/VILLASnode
.. _DPsim: https://git.rwth-aachen.de/acs/core/simulation/dpsim

Operating System Optimizations
------------------------------

The operating system configuration has a large impact on the real-time performance of DPsim.
For optimal results, follow the suggestions collected in the `VILLASnode documentation`_. 

.. _`VILLASnode documentation`: https://villas.fein-aachen.org/doc/node-tuning.html