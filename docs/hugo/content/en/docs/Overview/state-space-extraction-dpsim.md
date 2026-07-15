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
- `MNAStateSpaceContributorFactory` creates contributors for supported MNA components.

The extractor is owned by the MNA solver. Component contributors are created during solver initialization and are used to stamp the local matrices needed for the MNA-coupled state-space formulation.

# Supported scope

State-space extraction is available for EMT Ph3 and DP Ph1 simulations
using the direct MNA solver.

## EMT Ph3

Supported components with extraction states are:

- `EMT::Ph3::Inductor`,
- `EMT::Ph3::Capacitor`,
- `EMT::Ph3::TwoTerminalVTypeSSNComp`,
- `EMT::Ph3::TwoTerminalVTypeVariableSSNComp`.

Supported algebraic components without extraction states are:

- `EMT::Ph3::Resistor`,
- `EMT::Ph3::VoltageSource`.

The following composite components are supported through their immediate
MNA subcomponents:

- `EMT::Ph3::NetworkInjection`,
- `EMT::Ph3::PiLine`,
- `EMT::Ph3::RXLoad`,
- `EMT::Ph3::RxLine`,
- `EMT::Ph3::Shunt`,
- `EMT::Ph3::Transformer`.

## DP Ph1

Supported components with extraction states are:

- `DP::Ph1::Inductor`,
- `DP::Ph1::Capacitor`,
- `DP::Ph1::TwoTerminalVTypeSSNComp`,
- `DP::Ph1::MixedVTypeVariableSSNComp`.

Supported algebraic components without extraction states are:

- `DP::Ph1::Resistor`,
- `DP::Ph1::VoltageSource`.

The following composite components are supported through their immediate
MNA subcomponents:

- `DP::Ph1::NetworkInjection`,
- `DP::Ph1::PiLine`,
- `DP::Ph1::RXLoad`,
- `DP::Ph1::RxLine`,
- `DP::Ph1::Shunt`,
- `DP::Ph1::Transformer`.

Supported composite components are expanded by one level during contributor
discovery. Their immediate MNA subcomponents provide the state-space
contributions, while the composite parent remains part of the simulation and
retains its normal MNA stamping. Nested composites are currently unsupported.

Other component types are rejected explicitly when state-space extraction is
enabled.

# Usage

In C++, state-space extraction can be enabled as follows. The example below
uses EMT Ph3; replace `Domain::EMT` with `Domain::DP` for DP Ph1:

```cpp
Simulation sim("Example");
sim.setDomain(Domain::EMT);
sim.setSolverType(Solver::Type::MNA);
sim.doStateSpaceExtraction(true);
sim.run();

const auto &extractor = sim.getStateSpaceExtractor();
const Matrix &Ad = extractor.getDiscreteStateMatrix();
```

In Python, the corresponding API is shown below. Replace
`dpsimpy.Domain.EMT` with `dpsimpy.Domain.DP` for DP Ph1:

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

- [EMT Ph3 RLC extraction](https://github.com/sogno-platform/dpsim/blob/master/dpsim/examples/cxx/StateSpace/EMT_Ph3_RLC_StateSpaceExtraction.cpp)
- [EMT Ph3 composite extraction](https://github.com/sogno-platform/dpsim/blob/master/dpsim/examples/cxx/StateSpace/EMT_Ph3_Composite_StateSpaceExtraction.cpp)
- [DP Ph1 RLC extraction](https://github.com/sogno-platform/dpsim/blob/master/dpsim/examples/cxx/StateSpace/DP_Ph1_RLC_StateSpaceExtraction.cpp)
- [DP Ph1 composite extraction](https://github.com/sogno-platform/dpsim/blob/master/dpsim/examples/cxx/StateSpace/DP_Ph1_Composite_StateSpaceExtraction.cpp)

Equivalent Python notebooks are available in
[`examples/Notebooks/StateSpace`](https://github.com/sogno-platform/dpsim/tree/master/examples/Notebooks/StateSpace).
