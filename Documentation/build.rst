Build
=====

Windows (Visual Studio)
-----------------------

1. Install Visual Studio 2017 C++ Desktop development package plus Windows 8.1 SDK
2. Install `Python 3`_ using the normal installer, and add Python to your PATH.
3. Create a new folder name ``Libraries`` in the repository root, download Eigen_ and copy it to ``Libraries/eigen``.
4. Install CMake_, use its GUI (with `Source` as the source path) to create project files for Visual Studio. If you installed the 64-bit version of Python, make sure to use the "Win64" version of the Visual Studio generator.
5. Use Visual Studio and the Release configuration to build the dpsim Python module. To install it, build the INSTALL project.

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

   and ensure that ``your_python_path`` is in somewhere in your `PYTHONPATH`.

.. _`Python 3`: https://www.python.org/downloads/
.. _Eigen: http://eigen.tuxfamily.org
.. _CMake: https://cmake.org/download/
.. _VILLASnode: https://git.rwth-aachen.de/VILLASframework/VILLASnode
