Building
========

DPsim
^^^^^

Docker
------

There is a Docker image available with all required dependencies::

    $ cd dpsim
    $ docker build -t rwthacs/dpsim -f Packaging/Docker/Dockerfile .

Alternatively, the image can be pulled from DockerHub like so::

    $ docker pull rwthacs/dpsim

After running the container::

    $ docker run -it -v $(pwd):/dpsim rwthacs/dpsim bash


Setuptools
----------

Using setuptools is most likely the easiest way to install DPsim on your system.::

    $ git clone --recurse-submodules git@git.rwth-aachen.de:acs/core/simulation/dpsim.git
    $ cd dpsim
    $ python3 ./setup.py install

To build the Python package::

    $ python3 setup.py bdist

CMake
-----

Linux
*****

The most recent list of requirements can be found in the Dockerfiles. 

1. Make sure that the required dependencies are installed.

   Ubuntu/Debian:::
   
      $ apt-get install \
          build-essentials \
          make cmake \
          doxygen \
          graphviz \
          python3-pip \
          python3-dev \
          libeigen3-dev

   **Node:** There are currently no Debian packages for `villas-node` and `libcimpp16v29a`.
   If you want to use these optional feature, you are required to build them by hand.

   Redhat/Fedora/CentOS:::
   
      # wget https://packages.fein-aachen.org/redhat/fein.repo /etc/yum.repos.d/
      # dnf -y install \
	    git clang gdb ccache \
	    redhat-rpm-config \
	    rpmdevtools \
	    make cmake ninja-build \
	    doxygen \
	    graphviz \
	    pandoc \
	    python3-pip \
	    pkg-config

      # dnf -y install \
	    python3-devel \
	    eigen3-devel \
	    expat-devel \
	    graphviz-devel \
	    sundials-devel \
	    libcimpp16v29a \
	    libvillas-devel \
	    spdlog

      # dnf -y debuginfo-install \
	    python3

2. Fetch sources::

      $ git clone --recurse-submodules git@git.rwth-aachen.de:acs/core/simulation/dpsim.git
      $ cd dpsim

3. Generate a Makefile with CMake and use it to build the project::

      $ mkdir build
      $ cd build
      $ cmake ..
      $ make

4. Test your build by running the Jupyter notebooks:::

      $ make jupyter

5. Install the generated Python module to your system:::

      # make install

Windows
*******

1. Make sure that the required dependecies are installed::

   - Visual Studio 2017 and the C++ Desktop development package
     - ACS students and assistants can install Visual Studio via the ICT software center.
   - `CMake`_ for Windows
   - `Git for Windows`_
   
2. Open a Git Bash shell in a directory where you want to checkout the DPsim files::

2. Fetch sources::

      $ git clone --recursive git@git.rwth-aachen.de:acs/core/simulation/dpsim.git
      $ cd dpsim

3. Open a windows command prompt and navigate into the newly created DPsim folder.

4. Generate a Visual Studio project with CMake and use it to build the project::

      $ mkdir build
      $ cd build
      $ cmake -G "Visual Studio 15 2017 Win64" ..

5. Open Visual Studio and load the Visual Studio project from the build directory within the DPsim folder.

6. Alternatively, you can build the project from command line by running the following command in the windows command prompt::

    cmake --build .


Python support for Windows
~~~~~~~~~~~~~~~~~~~~~~~~~~

- Install `Python 3`_ using the normal installer or a distribution like Anaconda, and add Python to your PATH.
- Use Visual Studio and the Release configuration to build the dpsim Python module. To install it, build the INSTALL project.

.. _`Python 3`: https://www.python.org/downloads/
.. _Eigen: http://eigen.tuxfamily.org
.. _CMake: https://cmake.org/download/
.. _`Git for Windows`: https://git-scm.com/download/win
.. _VILLASnode: https://git.rwth-aachen.de/VILLASframework/VILLASnode
.. _DPsim: https://git.rwth-aachen.de/acs/core/simulation/dpsim
.. _`DPsim Libraries`: https://git.rwth-aachen.de/acs/core/simulation/dpsim-libraries


DPsim for Development
^^^^^^^^^^^^^^^^^^^^^

Docker
------

There is a Docker image available with all required dependencies::

    $ cd dpsim
    $ docker build -t rwthacs/dpsim-dev -f Packaging/Docker/Dockerfile.dev .

Alternatively, the image can be pulled from DockerHub like so::

    $ docker pull rwthacs/dpsim-dev

To run Jupyter lab notebooks from the dpsim-validation repository, call::

    $ git clone --recurse-submodules git@git.rwth-aachen.de:acs/core/simulation/dpsim-validation.git
    $ docker run -it -p 8888:8888 -v $(pwd):/dpsim-validation --privileged rwthacs/dpsim-dev bash

The DPsim C++ and DPsim Python library can be build as follows::

    $ cd dpsim-validation/dpsim
    $ mkdir build
    $ cd build
    $ cmake ..
    $ cmake --build . --target dpsim_python

To build everything run:::

    $ cmake --build .

Finally, the Python package is added to the path and Jupyter started::

    $ export PYTHONPATH=$(pwd)/Source/Python:$(pwd)/../Source/Python
    $ cd /dpsim-validation
    $ jupyter lab --ip="0.0.0.0" --allow-root
    
Documentation
^^^^^^^^^^^^^

Python
------

1. Install Sphinx_
   - either from your Linux distribution's repo
   - or manually_ on Windows
   - if you used the installer which already adds Python to your path and installs pip, you basically only need to run ``pip install sphinx``

2. Generate the Python documentation by running Sphinx via CMake:::

      $ mkdir -p build
      $ cd build
      $ cmake ..
      $ make docs

4. The resulting documentation will be generated in ``Documentation/html/``

C++
---

1. Install Doxygen
2. Generate the C++ documentation by running Doxygen via CMake:::

      $ mkdir -p build
      $ cd build
      $ cmake ..
      $ make docs_cxx

4. The resulting documentation will be generated in ``Documentation/html/Cxx``

:: _sphinx: http://www.sphinx-doc.org/en/stable/index.html
:: _manually: http://www.sphinx-doc.org/en/stable/install.html#windows-install-python-and-sphinx
