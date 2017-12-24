Build DPsim
===========

Windows (Visual Studio)
-----------------------

Simple Installation
~~~~~~~~~~~~~~~~~~~~~~
1. Install Visual Studio 2017 C++ Desktop development package
2. Clone the repositories DPsim_ and `DPsim Libraries`_ to the same directory.
3. Open the Visual Studio solution in dpsim-libraries/VisualStudio/DPsimVS2017 to get started.

Installation with CMake
~~~~~~~~~~~~~~~~~~~~~~~
After step two of the previous instructions you need to install CMake_, use its GUI (with ``Source`` as the source path) to create project files for Visual Studio. If you installed the 64-bit version of Python, make sure to use the "Win64" version of the Visual Studio generator.

Python support for Windows
~~~~~~~~~~~~~~~~~~~~~~~~~~
1. Install `Python 3`_ using the normal installer, and add Python to your PATH.
2. Use Visual Studio and the Release configuration to build the dpsim Python module. To install it, build the INSTALL project.

Linux (CMake)
-------------

1. Make sure that Eigen, libxml2 (including development headers) and CMake are installed, e.g. using your distro's package manager.
2. Build and install libvillas-ext from VILLASnode_:
   ::
      $ make
      # make install-libvillas-ext

   There are also RPMs available for Fedora:
   ::
      $ wget https://villas.fein-aachen.org/packages/villas.repo
      # mv villas.repo /etc/yum.repos.d
      # dnf -y install villas-node-devel

3. Generate a makefile with CMake and use it to build the project:
   ::
      $ mkdir build
      $ cd build
      $ cmake ..
      $ make

4. Install the generated module:
   ::
      # make install

   Another option is to manually rename the generated module:
   ::
      # cp build/Source/libdpsim.so your_python_path/dpsim.so

   and ensure that ``your_python_path`` is in somewhere in your ``PYTHONPATH``.

.. _`Python 3`: https://www.python.org/downloads/
.. _Eigen: http://eigen.tuxfamily.org
.. _CMake: https://cmake.org/download/
.. _VILLASnode: https://git.rwth-aachen.de/VILLASframework/VILLASnode
.. _DPsim: https://git.rwth-aachen.de/acs/core/simulation/dpsim
.. _`DPsim Libraries`: https://git.rwth-aachen.de/acs/core/simulation/dpsim-libraries

Build Documentation
===================

Python
------

1. Install Sphinx_
   - either from your Linux distribution's repo
   - or manually_ on Windows
   - if you used the installer which already adds Python to your path and installs pip, you basically only need to run ``pip install sphinx```

2. Generate the Python documentation by running Sphinx via CMake:
   ::
      $ mkdir -p build
      $ cd build
      $ cmake ..
      $ make docs

4. The resulting documentation will be generated in ``Documentation/html/``

C++
---

1. Install Doxygen
2. Generate the C++ documentation by running Doxygen via CMake:
   ::
      $ mkdir -p build
      $ cd build
      $ cmake ..
      $ make docs_cxx

4. The resulting documentation will be generated in ``Documentation/html/Cxx``

:: _Sphinx: http://www.sphinx-doc.org/en/stable/index.html
:: _manually: http://www.sphinx-doc.org/en/stable/install.html#windows-install-python-and-sphinx
