---
title: "State-Space Extraction"
aliases: ["/docs/overview/state-space-extraction-dpsim/"]
linkTitle: "State-Space Extraction"
date: 2026-05-28
description: >
  Enabling extraction, and running modal analysis on what it produces.
weight: 5
---

The method itself, what the extracted model means and where it is valid, is derived under
[state-space extraction]({{< ref "/docs/Concepts/state-space-extraction-theory.md" >}}).
This page covers enabling it and reading the result.

State-space extraction is optional and can be enabled through the `Simulation` API. During simulation setup, the MNA solver creates an `MNAStateSpaceExtractor`. During the solver task flow, a state-space extraction task uses the active direct linear solver to update the extracted discrete-time state matrix.

## Main classes

The implementation is organized around three main parts:

- `MNAStateSpaceExtractor` assembles and stores the extracted discrete-time state matrix.
- `MNAStateSpaceContributor` represents the state-space contribution of one supported component.
- `MNAStateSpaceContributorFactory` creates contributors for supported MNA components.

The extractor is owned by the MNA solver. Component contributors are created during solver initialization and are used to stamp the local matrices needed for the MNA-coupled state-space formulation.

For the components that support extraction in each domain, see
[state-space extraction support]({{< ref "/docs/Reference/state-space-extraction-support.md" >}}).

## Usage

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

## Modal analysis of the extracted model

`StateSpaceModalAnalysis` is constructed from an `MNAStateSpaceExtractor` and computes the modes of
whatever the extractor last produced. The method is described under
[modal analysis]({{< ref "/docs/Concepts/modal-analysis.md" >}}).

`update()` runs `Eigen::EigenSolver` on the discrete state matrix and throws if it does not converge.
It then maps each discrete eigenvalue to the continuous plane with `2 / dt * (z - 1) / (z + 1)` and
keeps both sets, retrievable through `getDiscreteEigenvalues` and `getContinuousEigenvalues`.

Participation factors are the elementwise product of the right eigenvectors with the transpose of the
left ones. They require inverting the right eigenvector matrix, so `update()` throws with an explicit
message when that matrix is singular. That happens for a defective state matrix, which is a property
of the system rather than a numerical problem; the eigenvalues are still valid in that case, only the
participation factors are unavailable.

`setAnalysisFrame` selects between `StateSpaceAnalysisFrame::Native`, which analyses the states as the
components hold them, and `GlobalDQ0`, which transforms into one common frame first. The second needs
`setGlobalDq0Frame(omega, theta0)`. `getStateNames` returns names matching the frame in use, so a
participation factor can be attributed to a named state rather than to an index.

## Examples

The feature is demonstrated in:

- [EMT Ph3 RLC extraction](https://github.com/sogno-platform/dpsim/blob/master/dpsim/examples/cxx/StateSpace/EMT_Ph3_RLC_StateSpaceExtraction.cpp)
- [EMT Ph3 composite extraction](https://github.com/sogno-platform/dpsim/blob/master/dpsim/examples/cxx/StateSpace/EMT_Ph3_Composite_StateSpaceExtraction.cpp)
- [DP Ph1 RLC extraction](https://github.com/sogno-platform/dpsim/blob/master/dpsim/examples/cxx/StateSpace/DP_Ph1_RLC_StateSpaceExtraction.cpp)
- [DP Ph1 composite extraction](https://github.com/sogno-platform/dpsim/blob/master/dpsim/examples/cxx/StateSpace/DP_Ph1_Composite_StateSpaceExtraction.cpp)

Equivalent Python notebooks are available in
[`examples/Notebooks/StateSpace`](https://github.com/sogno-platform/dpsim/tree/master/examples/Notebooks/StateSpace).
