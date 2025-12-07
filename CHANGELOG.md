# Changelog

All notable changes to this project will be documented in this file.
The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and this project adheres to [Semantic Versioning](https://semver.org/).

## [v1.2.0] - 2025-12-15

### Added

- Added this CHANGELOG.md file to track changes in the project.
- Adapt DPsim to deploy it in Mybinder (#323), moving Dockerfile to new .binder folder and adding job for building the binder Dockerfile.
- Add Code Coverage Report (#395), adding codecove badge and updating yaml file for code coverage.
- Add CODEOWNERS (#290)
- Add pre-commit (#352, #361, #363, #377, #382, #383, #385, #386) to harmonize code
- feat(cmake): Add install target (#381)
- New queueless VILLAS interface / improve real-time performance (#316), creating DCGenerator in VoltageSource only if special setParameters is used and not always when the frequency is 0 (avoiding regressions), and adding DP_PH1_ProfileVoltageSource used for the villas / fpga interface.
- Reuse conductance stamp code (#306), adding functions in MNAStampUtils for stamping value as a scalar matrix.
- Supress a warning treated as an error in MSVC (#280)
- update contact details (#254), updating contact details.
- Add support for CMake `find_package` and rename "KLU" dependency to name of actual library "SuiteSparse" (#380)
- Addition of EMT::Ph1::Switch component (#312), adding test notebook for VS_SW_RL1 circuit in EMT, DP and SP and fixing initialization.
- Linear SSN (#175), making EMT::Ph3 CurrentSource and SSN Full_Serial_RLC usable in pybind with implementation and declaration adjustments for EMT Ph3 CurrentSource.
- Implement decoupling line emt ph3 (#422), improving nine bus decoupling example with 3ph decoupling line and adding test for decoupling line Ph3 to Line.ipynb example.
- VILLASfpga cosimulation development (#325), adding n-eiling as codeowner of /dpsim-villas and fixing MNASolver to start loggers for vectors.
- Villas Interface: Improve FpgaExample (#299), allowing different topologies to be tested and updating VILLAS_VERSION in Dockerfile.
- Nix packaging (#357), adding GitHub CI workflow for building DPsim with Nix and Nix packaging.
- chore: update docs on real-time and VILLASnode interfaces (#335)
- Add dpsimpyvillas module to the CMake targets and include it in the Python wheel, using the correct Python install dependencies (#449)

### Changed

- Change python to python3 to run the building for pypi (#272)
- Disable code coverage in linux-fedora-examples on workflow (#412)
- editorconfig: set C++ indentation size to two (#295), setting it twice.
- Move pytest.ini to pyproject.toml (#348)
- Reduce attributes number (#388), declaring nominal/base voltages as Real instead of Attribute<Real>.
- Replace CentOS badge with RockyLinux badge (#282)
- villas: Use Python dictionaries for defining VILLASconfig files (#343)
- Change default linear solver to KLU (#250), fixing logging statements by using macros and removing explicit choice of SparseLU from cxx examples.
- Update WSCC_9bus_mult examples and implement splitting examples in DP and EMT (#322), removing SP_Ph1_CurrentSource component and fixing path to CIM files in decoupling line diakoptics example.
- Upgrade Fedora dockerfiles to v42 (#387), fixing KLU Adapter varying entry index check and adding libre to Dockerfile to fix one of the real-time errors using villas.
- Add new action using the build wrapper from sonar directly (#296)
- Allow builds without nlohmann/json library (#362)
- feat(ci): Use trusted / OIDC publishing to PyPi.org (#375), only attempting upload to PyPi for pushes to master or tags.
- Fixes to documentation and README, adding extra details about installation methods, contribution guidelines and how to try DPsim (#457).
- Update documentation and packaging metadata contact information, add CONTRIBUTORS file and GitHub Discussions link, and improve contributor/community guidance (#429, #413).
- Extend the changelog to include entries for older release versions and previously unreleased changes (#430).
- Use the DPsim manylinux container for building the Python package and skip RC versions when publishing to PyPI.org, while still publishing RC versions to TestPyPI (#451).

### Removed

- Remove commented out code (#390)
- Remove further RWTH gitlab dependencies (#261)
- Remove Python versions 3.6 and 3.7 for PyPi upload (#229)
- Disable test actions for PRs (#237)
- chore(cmake): Remove deprecated options (#342)
- Remove outdated and broken GitLab CI pipeline (#347)

### Fixed

- 5th Order Synchronous Generator (#230), fixing some minor details and adding SG5Order to CIM Reader in DP and EMT domain.
- Adding correct logger usage (#411)
- dpsim-villas: fix hardcoded paths (#327)
- Enable compilation in gcc 14 and clang 18 (code fixes) (#294)
- feat(cmake): Allow disable LTO builds (#341), adding WITH_MARCH_NATIVE option to enable native host arch builds.
- Fix consistency of calculations at first time step (#210), using Ph3 RMS quantities for initialSingleVoltage in EMT domain and adding macro DOUBLE_EPSILON.
- Fix docu issues (#310), substituting duplicate ODESolver with DAESolver in dpsim_classes_simulation.svg.
- Fix docu pipeline (#333)
- Fix initialization rxload (#241), fixing current initialization of EMT RXLoad.
- Fix module dpsimvillas (#401), fixing ambiguous use of VoltageNorton->SetParameters and fixing examples for realtime and CIM usage.
- Fix power flow initialisation of synchronous generators (#238), fixing pybind double definition of components_at_node and enforcing domain argument for initWithPowerflow.
- Fix rocky workflow due to changes in the profiling triggering (#305)
- Fix some capitalization (#351)
- Fix sonar scanner errors due to Java deprecated version (#265)
- Fix submodule initialization for CIMpp in Dockerfiles (#331), fixing it twice.
- Fix the profiling workflow (#399)
- Fix VILLASnode in Dockerfiles (#292), updating VILLASnode version in Dockerfile.manylinux and in other Dockerfiles.
- Fix workflow: option for parallel in make and error in publish to PyPI (#303)
- fix(cmake): Show feature summary even if not Git info is available (#379)
- fix: MNASolverPlugins examples (#346), fixing include paths of MNASolverPlugin examples and wrapping extern c in preprocessor condition.
- Fixes to the doxygen documentation INPUT line (#264)
- Fixing inconsistent switch attribute names in Base::Ph1::Switch and Base::Ph3::Switch (#418)
- Fixup whitespaces (#350)
- Hotfix: missing simulation stop (#247), adding missing stop function in the Simulation using pybind.
- pybind: Errors when compiling with clang (#334), fixing rocky container and updating villas version, fixing fedora container error in mosquito install and updating villas version.
- Reuse code for MNA matrix stamp operations (#297), logging stamping in MNAStampUtils and reusing admittance stamping logic for SP R,L,C components.
- Reuse stamp code for EMT synchron. generators (#315), adding matrix stamp functions with optimized logic in MNAStampUtils and fixing stamping of full matrix.
- Revision of power flow solver (#284), fixing PFSolver and minimizing file changes.
- Trigger actions for PRs and pin the version of libcimpp (#329), pinning libcimpp version in containers and cmake and triggering relevant workflows on pull_request event.
- Update actions in workflow and improve parallelisation and cache handling (#298), changing the profiling execution to manually triggered and enabling parallelisation with reorganized cache.
- Bump braces from 3.0.2 to 3.0.3 in /docs/hugo (#304)
- Bump postcss from 8.4.20 to 8.4.31 in /docs/hugo (#281)
- Bump version of black & black jupyter to 25.11.0 (#421)
- Bump yaml from 2.1.3 to 2.2.2 in /docs/hugo (#215)
- fix clang compilation and FPGA integration (#293), fixing clang compiler errors and adding clang compilation to CI.
- Fix CMake typos and style (#376)
- Fix GitHub actions workflow for publishing to PyPi (#355), building with newer CMake versions and simplifying triggers for workflows.
- Fix realtime datalogger (#400), adding example notebook for tests and pybind bindings.
- fix std::max behaviour in simulation.cpp for windows build using CMake (#408), fixing issue where windows.h breaks standard C++ behaviour of std::max.
- Fix the villas examples workflow (#319), adding supplementary cache and testing cache deletion.
- Fix Windows and Rocky/Clang builds (#360), fixing Windows builds with newer CMake versions.
- fix(ci): Attempt to fix PyPi package publish (#353)
- fix(ci): Fix tag of pypa/gh-action-pypi-publish action (#354), fixing the tag twice.
- fix(cmake): Fix code-style (#356)
- fix(deps): Add support for newer Spdlog versions (#340), adding missing #pragma once.
- fix(docs): Fix typo and wrong diff (#337)
- fix(examples): Only build Fpga9BusHil example if build with CIMpp (#338), applying it twice.
- fix(style): Remove duplicated typedef (#339)
- python: Harmonize comment style (#349)
- chore(deps): Bump pypa/gh-action-pypi-publish from 1.12.4 to 1.13.0 in /.github/workflows (#405)
- Update villas version (#245), updating python command to python3 in test-villas-examples actions and removing libwebsockets installation from source.
- Use clang-format to format the whole codebase (#278), fixing missing includes and moving development scripts from configs/ to scripts./
- chore (deps): update to CIMpp rpm/deb install + bump VILLAS (#404), updating cmake fetch and finding of libcimpp and fixing dependent files and CIMReader.
- fix: Cleanup code-style of setup.py (#336)
- Reactivate VILLAS support in the MyBinder Python packaging workflow and fix related compilation issues when using dpsimpy with VILLAS-enabled containers (#450, #434, resolves #445).
- Create a symlink in the manylinux image to the install path of the OpenDSS C library to fix Python packaging and runtime loading issues (#448).
- Link DPsim to VILLAS libraries via CMake to fix missing linkage in VILLAS-enabled builds and examples (#447).
- Make the Fedora release workflow wait for the dpsim-dev container build to avoid race conditions in the release pipeline (#446).
- Security fix for the profiling workflow, avoiding passing commands as inputs to scripts in the profiling job (#455).
- Security fix for the VILLAS workflow, preventing commands from being passed as raw inputs into the shell (#454).
- Adapt the Sonar workflow to avoid deprecated secret checks so that scans still run for pull requests from forks (#456).
- Fix the Sonar workflow by bumping the action to the supported v6 version and adjusting configuration (#439).

## [v1.1.1] - 2023-07-13

### Added

- Add latest publications to website (#161)
- Introduce a common subclass for composite components (#177), adding comments for empty method implementations and renaming systemMatrixStamp methods for sparse matrices.
- Build and push RockyLinux image (#148)
- Update and expand documentation (#159), adding documentation on MNASimPowerComp and subcomponent handling.
- Introduce common base class MNASimPowerComp (#178), fixing merge errors and addressing SonarCloud issues.

### Changed

- Unify subcomponent handling and call MNA methods through the parent (#141)
- Move documentation to main repo (#154), adding hugo website files and moving content from dpsim-docs repo.
- Refactoring of direct linear solvers in MNA solver (#171), addressing further pull request comments and adding fetching of minimized suitesparse.

### Fixed

- Configurable linear solver (#199), addressing further pull request comments and processing default configuration options.
- Fix EMT_Ph1_Inductor initialization (#134), using Modelica results for validation and adding current validation in example VS_RL1.
- fix hugo website deployment (#158)
- Fix missing 'this' in logger macros (#217)
- Fix network frequency modulations and vsi snubber comps (#218), fixing some logging issues and trafo instantiation in dp and sp vsi for proper snubber comps.
- Fix various issues related to attributes (#140), addressing SonarCloud code smells and moving method implementations into source files.
- Profiling based optimisation (#194), addressing pull request comments and applying switched component stamps.
- Resolve FIXME comments (#142), addressing SonarCloud issues and renaming mMechPower attribute.
- Cleanup of CMake and Docker files (#168), adding missing libcimpp dependency for cimviz and comment explaining CMAKE_POLICY_DEFAULT_CMP0077.
- Deploy docs to FEIN website (#160), fixing repo url and subdir.
- fix and update documentation (#165), updating Contribution Guidelines and Roadmap.
- Fix implementation of exciter, new turbine governor model and refactor VBR models (#120), applying minor fixes and replacing validation notebook of reduced order SG models.
- Reduced-order syngen models (#213), removing unused variable in MNASolverDirect and unused methods/variables of MNASyncGenInterface.

## [v1.1.0] - 2022-12-09

### Added

- Add a basic VS Code devcontainer configuration
- add cppcheck for use in github actions, updating Dockerfile and install shell scripts and adding docker-compose for dpsim-mqtt examples.
- Add cps files to dpsim repo
- add deployment to gitlab pages
- add missing file headers, updating copyright year to 2021.
- Add new working mqtt example
- add notebook checking powerflow of ieee lv, adding notebook checking powerflow for cigre mv.
- add pillow dependencies to centos dockerfile
- add powerflow cim example
- add villas import example, using PQ attributes in SP load.
- add vsi model and examples
- allow vd, pv and pq comp at the same bus, setting bus type to vd, updating submodule grid-data and updating syngen model in sp1ph.
- avoid deadlock, adding log prefix and console logger for simulation and shmem interface.
- docker: add entry page to docker image, copying examples into image and starting Jupyter Lab by default.
- made all power components set parametersSet flag, adding setParameters flag and function with new constructor.
- python: add missing Compontents (closes #118)
- require only cmake 3.11 and make policies requiring a newer version optional
- shmem: update Python examples, allowing exporting of attributes with name & unit.
- sphinx: add sections in reference
- update cps, only declaring node templates in cpp for win and adding description type to setup.py.
- update steady state init and add logging

### Changed

- cmake dependent opt causes problem with cim
- cmake: use cmake_dependent_option() to avoid caching issues
- Create vnodes in pcomps instead of MNA solver and differentiate between 1ph and 3ph for the simNode assignment.
- flush spdlog after sim initialization, switching case for python log level and adding function to get current voltage.
- force gcc 11 in manylinux image
- initialize frequencies in SystemTopology
- Merge branch 'development' into 'master'
- Merge branch 'master' into multisampling, updating notebooks and gitignore.
- Merge branch 'patch-1' into 'master'
- Merge pull request #60 from JTS22/python-spdlog
- MNASolver: Increase maximum switch number to theoretical limit
- move libcps to models
- notebooks: clear results
- run autodoc on python submodules
- run page deploy on dedicated branch
- silence experimental fs visual studio warning
- started working on developer documentation
- Trigger PyPi-Workflow for tags (#151)
- update 9bus parallel notebook
- update cps, updating shmem distributed direct nb and shmem direct examples.
- update file headers and replace gpl with mpl in most files
- update inverter notebooks
- update shmem example of cigre mv
- update submodules path, updating dpsim results path in notebooks.
- rename Terminal, renaming TopologicalComponent and Node class.
- disable cim and render if deps not available, removing obsolete cim reader includes and working around github permission issue.
- update ci, restructuring sphinx doxygen docs and removing notebook related sphinx cmake.
- update cps, merging master and restructuring notebooks.
- update docs url
- update global scope of SynGenTrStab examples, adding Events category to examples and splitting examples in two categories: components & grids.
- update install script and new dockerfile, updating docker dev and using libxml instead of libexpat in cimpp.
- update shmem examples, using new cpp export methods in python interface and adding jupyter requirements file to docker fs.
- updated installation instructions, showing cmake invocation during setup.py and allowing setting CMake options via envvar.
- use correct minimum cmake version (3.13 instead of 3.12) because Policy CMP0076 requires 3.13, using different cache key for build with and without cuda and making CI build with CUDA support with CUDA dependencies in Dockerfile.dev.
- use updated dpsim-villas and villas-node versions, removing pipeline build caches and updating fedora and pybind versions.

### Deprecated

- moving old examples into villas-deprecated folder.

### Removed

- disable macos build
- harmonize namespace aliases of std::filesystem, removing useless printing of working directory in examples.

### Fixed

- add sst model and fix powerflow
- append .git to all module urls, fixing path to cim grid data and updating README.
- examples: fix notebook bin path, considering load "P" attribute in PF during simulation.
- fix brief class descriptions of reduced-order vbr models, fixing sp shift and adding grid voltage evaluations in validation notebooks, fixing cmd options after rebase.
- fix cim path for shmem example of cigre mv
- fix cmake policy warning
- fix dp ph3 voltagesource stamp
- Fix dpsim-villas version tag, using new villas version.
- fix GPLv3 logo (closes #125)
- fix load profile generation for different time steps in cigre shmem example, fixing cim files path and extending CLI options.
- fix minimal dockerfile
- fix powerflow with load profiles and use csvreader instead of loadprofilereader
- fix rxload initialization
- fix sourcevec writing and update villas-dataprocessing
- fixing mechanical torque as attribute after rebasing, adding logging of indices of varying matrix entries for reduced order VBR models in DP and EMT and adding smib reduced order load step examples.
- Merge branch 'debug-msp-example', updating cps and adding notebook with CS RL1 circuit.
- merge master, updating cps and using cmake_dependent_option() to avoid caching issues.
- MNASolverFactory.h: Only use Plugin Solver if activated, adding *.so to .gitignore and making MNASolverPlugin optional and deactivated on windows.
- python: fix invalid return value for Node::setInitialVoltage()
- spdlog macros, fix simNode assignment of vnodes, set links to master branch
- switch shared library off to fix dpsim_python, updating editorconfig and adding numpy cmake option again.
- update cps, ignoring attributes if index is larger than length and updating 9bus ctrl shmem example.
- update cps, fixing error in WSCC 9bus notebook and fixing feature notebooks.
- update readme, fixing shm python example and adding removed FindSphinx cmake file.
- update tlm comparison notebook, adding correct phase shift to transmission line and adding emt decoupling wave line model.
- use correct attribute dependency, making the phase changeable through mVoltageRef and using member variables for attribute access.
- use image for devcontainer, setting manual trigger for container workflow and fixing devcontainer.json.
- use initialization list for mZigZag, adding comments on voltage source behaviour and removing constructor for removing the SignalGenerator.
- add comments, adding another export to Simulation::sync and increasing timeout to allow dpsim to catch all mqtt messages.
- add link to build notebooks
- enable tests for pull requests, updating sonar settings and adding sonar cache and multithreading.
- examples: reduce sim time because of CI duration, fixing find pybind python in cmake.
- fix .gitlab-ci.yml to include correct path to rocky Dockerfile, fixing docker labels and removing duplicate needs statement.
- fix build badge
- fix doc gh-pages deployment
- fix emt ode syngen model, using systemtopolgy initFromPowerflow in examples and setting initFromNodes for MNA solver.
- fix file paths in notebook, updating notebook to use the new attribute system and exposing the entire attribute system to python.
- fix gitlab ci using old docker syntax (#147)
- Fix PyPi-Upload CI workflow (#149), finding Python3 instead of Python and updating container workflow actions.
- fix set pattern in scheduler, setting log pattern in scheduler and merging master.
- fix submodule build: throwing only warning when git version unavailable
- include Magma in Dockerfile, cleaning up gitlab-ci.yml, fixing GPUMagma inverse permutation bug and adding Magma based MNASolver.
- keep governor and exciter params separate from machine params, adding governor and exciter test examples for 9th order syngens and fixing exciter of dcim syngen.
- merge 1ph-refactoring, updating cps and fixing cpp 9 bus nb.
- Merge branch 'test-villas' into 'master', updating repo link in build docs and shmem example nb.
- Merge pull request #39 from stv0g/ci-minimal-build
- Merge pull request #46 from stv0g/ci-minimal-build
- minor tweaks to gitlab-ci
- move imports into cpp files, removing binary dir include for dpsimpy and clarifying Cpp version change.
- output dpsimpyvillas into top-level build directory, exporting complex attributes instead of matrices
- pybind: remove superfluous includes, removing unused VILLAS_VERSION variable from docker and allowing building examples without CIM support.
- re-enable notebook test, re-enabling example and adding missing test examples.
- reactivate continue_on_error parameter, adapting cigre mqtt example to run in pipeline and adding test for dpsim-mqtt-cigre example.
- refactor to store cli options as string
- remove cmake copy image, updating grid-data and examples with new grid data path.
- remove debug info from workflow, removing redundant jobs from gitlab and cleaning up dockerfiles.
- remove obsolete gitlab CI jobs, generating sphinx docs for new python interface.
- Revert :Fix compilation without libcimpp and minor tweaks to CMake code-style
- skip tests for old notebooks, adapting quickstart guide to dpsimpy and triggering actions on push to any branch.
- Use read-the-docs sphinx theme

## [v1.0.0] - 2019-04-15

### Changed

- Merge branch 'dae-solver-test' into 'development', adding documentation on real-time execution of DPsim (closes #108) and merging development into dae-solver-test.
- Merge branch 'gen-arkode' into 'development', updating Simulation.cpp and DataLogger.cpp.

### Fixed

- Continued to fix odeint example problem, adding example program for odeint based on DP_SynGen_dq_ThreePhFault of Arkode and first implementation of odeint solver class.
- Fixed initial value setting of odeint solver
- adaptations for DQ SynGen class split, merging branch 'development' into parallel and updating cps submodule.
- fixes for clang, adding parallel multimachine benchmark and fixing memleak in ODESolver.
- ifdef for sim_ode, updating .gitmodules and cps.
- include nbs in docs
- merge powerflow, applying minor logging fix in cigre mv powerflow test and writing config of data sent via villas.
- update cps, merging branch 'development' into powerflow-integration and updating dockerfile.dev.

## [v0.1.6] - 2018-11-04

## [v0.1.5] - 2018-11-03

### Added

- add missing _dpsim module to binary Python packages
- cmake: only integrate interface code if the platform supports it, adding more parameters to DPsim::RealTimeSimulation::run() and moving timer creation into {create,destroy}Timer().
- docker: enable jupyter widgets
- python: add missing invocation of loggers to python simulation loop, adding proper reference counting for logged Components/Nodes.
- python: add missing progressbar package as dependency, pausing and resuming simulation asynchronously and adding more async events.
- update CPS submodule, providing synchronization settings per interface and making start script executable.
- updated cmake files for cimpp, including cimpp submodule in make files and adding libcimpp as submodule.
- updating cps, updating freq and load step example and removing NZ publication example.
- updating emt example, merging branch 'development' into msv-pra and updating examples.

### Changed

- flush data logs on simulation end, updating notebooks and raising error when dpsim.Simulation.run() is used with an already running event loop.
- Install expat dependency and execute setup
- Merge branch 'redesign_cimreader' into 'development'
- merge updated circuit examples, updating examples.
- python: do not fail load _dpsim.Interface (closes #93), reverting "comment out interface python class".
- python: separate EventChannel in header/source
- updated copyright year in C++ example
- merge changes into development, updating .gitlab-ci.yml and deactivating windows build until we have a runner again.
- merging changes in mna solver, using new DataLogger and making MNASolver not rely on CIM::Reader.
- merging changes into development, merging branch 'refactor-move-cps' into 'development' and not crashing if there are no CIM tests.
- merging new commits from development and node-terminal update, updating CPowerSystems submodule and refactoring Add{Excitor,Goveronor} -> add{Excitor,Goveronor}.
- Update Build.rst
- updating libcps, merging attribute test and merging redesign-simulation into development.
- shmem: libvillas-ext has been obsoleted by libvillas, simplifying dpsim.Simulation.run() and fixing error messages in CPS::Python::Component:getattro().

### Fixed

- Adapted DAEsolver to new node/component structure and removed IDA dependency of Simulation file, making Residual function static and integrating DAE into Solver structure.
- docker: add Dockerfile for building a ready-to-run version of DPsim, fixing RPM dependencies and building RPM packages.
- docker: use fedora 29, fixing compilation without VILLASnode in python.
- fix DPsim logo in README.md
- fixing namespaces in simulation.cpp, updating CPS submodule for fixing linking errors and moving Python bindings for components to CPS.
- last buf fixes before merging, improving usage infos for command line args and adding missing header file.
- Merge branch 'development', changing shmem ids and increasing simulation time in wscc.
- python: fix logging of node voltages (closes #97)
- update CPS submodule, fixing attribute names in examples and python tests.
- fix windows build
- fixing CIM test yml, xfail for larger CIM example and removing TopologicalIsland from examples.
- Fixing parallel support of node indices and CIM, removing deprecated line load test and fixing network description without node objects.
- fixing segfault in mna solver when getting nodes, adding new examples and updating cps.
- Merge branch 'development', merging generator examples and updating cps.
- python: add some documentation to factory functions, fixing unit tests and changing NULL -> nullptr.
- tests: renamed python script in order to be selected by pytest, updating Python version of ShmemDistributedDirect test and refactoring examples for new lambda interface.

## [v0.1.3] - 2018-02-21

### Changed

- Update CMakeLists.txt
- refactor: DPsim::Components::Base -> DPsim::Component (closes #44), renaming "EMT_VoltageSourceNorton.{h, cpp}" to "EMT_VoltageSource_Norton.{h,cpp}" (see #43).

### Fixed

- fixed mistakes after merging, using SynGenSimulation for VBR simulation and integrating new DP VBR model with nodal analysis.
- Merge branch 'fix-transformer' into development merge, fixing CIM reader and example and applying minor changes in logging.
- examples: disable another failing CIM test, disabling broken IEEE-9-bus CIM test and refactoring mSeq -> mSequence.
- version bump, fixing url and image links in readme.

## [v0.1.1] - 2018-01-12

### Added

- added missing license and copyright headers (closes #23)
- TurbineGovernor: Added init function to initialize governor variables
- Update DP_ResVS_RXLine1.cpp, merging branch 'development' into 'shared-factory' and applying smaller cleanups.

### Changed

- do not use shared_ptrs for simulation Loggers
- Update .gitlab-ci.yml
- Update FaultSimulation.cpp, updating Simulation.cpp and merging branch 'development' into 'refactor-component-element-naming'.
- updated logdataline, fixing capitalization of filenames and excluding certain parts from Doxygen as they break Breathe.

### Removed

- refactor: rename namespace "DPsim::Component" -> "DPsim::Components", removing default constructor from components and naming base class files "Base_*.h".
- simulation: remove obsolete Generator test classes, simplifying expression and fixing real time simulation support in python.

### Fixed

- examples: fix hardcoded path (this is still ugly), installing libraries to correct location on Fedora and adding setup.py for installing / packaging with setuptools.
- fixed test builds
- Merge branch 'python-log' into 'development', fixing Python CI test and python ctor of Simulation class.

## [v0.1.0] - 2017-12-26

### Added

- changed newNode to node3, adding comments to header files of capacitor, ideal voltage source, inductor, inductor2 and voltSourceRes and adding documentation for ideal and real voltage source.
- created new components classes
- loglevel filter, updating study scripts and adding svg figures.
- Merge branch 'dev-vsa' into 'master', adding figures for synchronous machine and synchronous generator figure.
- Merge branch 'dev-vsa' into 'master', changing capacitor and inductor in simulation models and updating figure of inductor model.
- merge dev-mmi into master, documenting EMT DP comparison and first version of comparison between EMT and DP.
- moved Simulink models to other repo, adding folder DPsimReferenceExamples and deleting folder SimulinkExamples.
- Update LICENSE, adding license.
- VBR DP, merging branch 'development' and adding logo.
- Added omega_base to the equations of trapezoidal rule and implemented trapezoidal rule with current as state variable, implementing trapezoidal rule with flux as state for EMT and correcting mistake in equation of ifd.
- Created voltage behind reactance model - in construction, adding trapezoidal rule to DP synchronous generator and merging branch 'rt-exceptions' into development.

### Changed

- DP VBR working
- Merge branch 'development', merging master into dev-mmi merge and updating matlab scripts.
- Merge branch 'master' of git.rwth-aachen.de:PowerSystemSimulation/DPsim merge
- Update README.md
- VBR euler, adding matlab script to compare plecs with c++ results for synchronous generator and only compiling shmem/RT parts on Linux.
- VBR model - simulation of steady state
- adjust CMakeLists.txt, merging branch 'development' and merging branch 'cim-xml' into development.
- Merge branch 'dev-vsa' into 'master', updating LinearResistor.cpp and BaseComponent.h.

### Removed

- deleted vs project files, cmake works now in vs, adding vs folder to gitignore and deleting vs installation md file, updating build.rst.

### Fixed

- added GPLv3 headers to source files (closes #9), moving Source/Examples to Examples/ and treating unused variables as errors in Clang.
- Created Ideal Voltage Source EMT and fixed "virtual node", simplifying models with operational and fundamental parameters.
- fix compiler warnings (closes #15), creating simplified model of synchronous machine and making all generator models use SynchGenBase.
- implemented exciter and turbine to VBR DP model, fixing mistake in fault clearing function and adding Turbine Governor model.
- pass matrices via reference to silence compiler errors, adding Dockerfile and GitLab CI configuration and searching in standard location for Eigen.
