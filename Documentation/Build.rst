Building
========

DPsim
^^^^^

Docker
------

There is a Docker image available with all required dependecies:

::
    $ cd DPsim
    $ docker build -t dpsim .
    $ docker run -tiv$(pwd):/dpsim dpsim bash

Setuptools
----------

Using setuptools is most likely the easiest way to install DPsim on your system.

::
    $ cd DPsim
    # python3 ./setup.py install

CMake
-----

1. Make sure that the required dependecies are installed.

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
   
      # wget https://villas.fein-aachen.org/packages/fein.repo -O /etc/yum.repos.d/fein.repo
      # yum install \
          gcc-c++ \
          redhat-rpm-config \
          rpmdevtools \
          make cmake \
          doxygen \
          graphviz \
          python3-pip \
          python3-devel \
          eigen3-devel \
          villas-node-devel \
          libcimpp16v29a

2. Generate a makefile with CMake and use it to build the project::

      $ mkdir build
      $ cd build
      $ cmake ..
      $ make

3. Test your build by running the Jupyter notebooks:

      $ make jupyter

4. Install the generated Python module to your system:::

      # make install

Visual Studio
-------------

**This method is not recommended. Please use CMake or Python setuptools instead.**

Prerequisites
~~~~~~~~~~~~~~~~~~~~~~
First, install Visual Studio 2017 and the C++ Desktop development package.
Then, you can choose to either use the prepared Visual Studio project or generate the project files on your own using CMake.

Installation with Prepared Visual Studio Project
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- Clone the repositories DPsim_ and `DPsim Libraries`_ to the same directory.
- Open the Visual Studio solution in dpsim-libraries/VisualStudio/DPsimVS2017 to get started.

Installation with CMake
~~~~~~~~~~~~~~~~~~~~~~~
You need to install CMake_, use its GUI (with ``Source`` as the source path) to create project files for Visual Studio. 
If you installed the 64-bit version of Python, make sure to use the "Win64" version of the Visual Studio generator.

Python support for Windows
~~~~~~~~~~~~~~~~~~~~~~~~~~

- Install `Python 3`_ using the normal installer or a distribution like Anaconda, and add Python to your PATH.
- Use Visual Studio and the Release configuration to build the dpsim Python module. To install it, build the INSTALL project.

.. _`Python 3`: https://www.python.org/downloads/
.. _Eigen: http://eigen.tuxfamily.org
.. _CMake: https://cmake.org/download/
.. _VILLASnode: https://git.rwth-aachen.de/VILLASframework/VILLASnode
.. _DPsim: https://git.rwth-aachen.de/acs/core/simulation/dpsim
.. _`DPsim Libraries`: https://git.rwth-aachen.de/acs/core/simulation/dpsim-libraries

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
