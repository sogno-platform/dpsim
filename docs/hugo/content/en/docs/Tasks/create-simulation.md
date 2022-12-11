---
title: "Create New Simulation"
linkTitle: "Create New Simulation"
date: 2020-03-25
description: >
  Using DPsim for a new simulation scenario.
---

Here, we will show the implementation of a new simulation scenario defined in C++, which is using DPsim as a library.

## Directory Structure

In the end, your directory structure should look like as follows:

    my-project
      |- CMakeLists.txt
      |- source
          |- my-scenario.cpp
      |- dpsim (as submodule)

## CMake File

Your CMakeLists could look like this:

    cmake_minimum_required(VERSION 3.5)
    project(my-project CXX)

    add_subdirectory(dpsim)

    add_executable(my-scenario source/my-scenario.cpp)
	  target_link_libraries(my-scenario dpsim)

## Build the Project

The build process is similar to the one of DPsim:

    $ cd my-project
    $ mkdir build && cd build
    $ cmake ..
    $ make my-scenario