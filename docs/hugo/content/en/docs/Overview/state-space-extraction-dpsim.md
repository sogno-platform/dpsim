---
title: "State-Space Extraction"
linkTitle: "State-Space Extraction"
date: 2026-05-28
---

State-space extraction is optional and can be enabled through the `Simulation` API. During simulation setup, the MNA solver creates an `MNAStateSpaceExtractor`. During the solver task flow, a state-space extraction task uses the active direct linear solver to update the extracted discrete-time state matrix.

# Main classes

The implementation is organized around three main parts:

- `MNAStateSpaceExtractor` assembles and stores the extracted discrete-time state matrix.
- `MNAStateSpaceContributor` represents the state-space contribution of one supported component.
- `MNAStateSpaceContributorFactory` creates contributors for supported EMT components.

The extractor is owned by the MNA solver. Component contributors are created during solver initialization and are used to stamp the local matrices needed for the MNA-coupled state-space formulation.

# Supported scope

State-space extraction is available for real-valued EMT simulations with the direct MNA solver.

Supported components with extraction states are:

- EMT 3-phase inductors,
- EMT 3-phase capacitors,
- EMT 3-phase two-terminal V-type SSN components.

Supported algebraic components without extraction states are:

- EMT 3-phase resistors,
- EMT 3-phase voltage sources.

Other components are rejected explicitly when state-space extraction is enabled.

# Usage

In C++, state-space extraction can be enabled on a simulation as follows:

```cpp
Simulation sim("Example");
sim.setDomain(Domain::EMT);
sim.setSolverType(Solver::Type::MNA);
sim.doStateSpaceExtraction(true);
sim.run();

const auto &extractor = sim.getStateSpaceExtractor();
const Matrix &Ad = extractor.getDiscreteStateMatrix();
```

In Python, the corresponding API is:

```python
sim = dpsimpy.Simulation("Example")
sim.set_domain(dpsimpy.Domain.EMT)
sim.set_solver(dpsimpy.Solver.MNA)
sim.do_state_space_extraction(True)
sim.run()

extractor = sim.get_state_space_extractor()
Ad = extractor.get_discrete_state_matrix()
```

# Examples

The feature is demonstrated in:

- [C++ example](https://github.com/sogno-platform/dpsim/blob/master/dpsim/examples/cxx/StateSpace/EMT_Ph3_RLC_StateSpaceExtraction.cpp)
- [Python notebook](https://github.com/sogno-platform/dpsim/blob/master/examples/Notebooks/StateSpace/EMT_Ph3_RLC_StateSpaceExtraction.ipynb)

The examples compare equivalent EMT 3-phase RLC systems built from native MNA components and generic two-terminal V-type SSN components.
