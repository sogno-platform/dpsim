---
title: "Roadmap"
linkTitle: "Roadmap"
weight: 8
---

Short-term planning for new features is done on the GitHub [Project board](https://github.com/orgs/sogno-platform/projects/1).

### Under Development

- Solver
  - [ ] CUDA sparse implementation
  - [ ] improve online system matrix computation and refactorization to support nonlinear elements in network solution (NICSLU integration)
  - [x] merge DAE solver branch
- Interfaces
  - [x] reimplement python interface using pybind and expose more models / functionalities
  - [x] add python based examples using the VILLASnode interface
  - [x] support matpower / pypower format for static simulation
- Tests, Examples, CI
  - [x] convert most of the examples to Python and test them against reference results in CI
  - [x] convert more gitlab CI jobs to github actions
  - [ ] add IEEE39 system to examples
- Models
  - [x] VBR generator model
  - [ ] SVC
  - [ ] add tap-change to transfomer

### Ideas

- Solver
  - [ ] improve integration of diakoptics solver
- Interfaces
  - [ ] implement CIM reader in Python using new pybind interface and cimpy library

