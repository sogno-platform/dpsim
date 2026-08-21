---
title: "SSN Component Implementation"
linkTitle: "SSN Components"
date: 2026-07-31
description: >
  The SSN base class hierarchy and what a new SSN component has to provide.
weight: 24
---

The method is derived under
[state-space nodal components]({{< ref "/docs/Concepts/Models/ssn-components.md" >}}) and
[state-space nodal]({{< ref "/docs/Concepts/state-space-nodal.md" >}}). This page covers only the
code.

## Base hierarchy

`SSNComp` holds the continuous matrices, the discrete pair, the equivalent admittance `mW`, the
history vector `mYHist` and the state attribute `x`. Two branches specialise it by which quantity is
the input:

- `VTypeSSNComp` takes voltage in and gives current out, so it stamps as an admittance
- `ITypeSSNComp` is the dual

Terminal-count layers sit on top: `TwoTerminalVTypeSSNComp`, `TwoTerminalITypeSSNComp` and
`FourTerminalVTypeSSNComp` handle the mapping from terminal quantities to the model input, and the
`Variable` layers add re-forming of the model between steps. Every EMT::Ph3 base sets
`PhaseType::ABC` in its constructor, which the concrete components rely on.

## What a component provides

A fixed-model component only calls `SSNComp::setParameters(A, B, C, D)` with its chosen state,
input and output. `EMT::Ph3::SSN::Inductor` is the whole pattern:

```cpp
Matrix aMatrix = Matrix::Zero(3, 3);   // x = i_abc
Matrix bMatrix = inductance.inverse(); // u = v_abc
Matrix cMatrix = Matrix::Identity(3, 3);
Matrix dMatrix = Matrix::Zero(3, 3);
SSNComp::setParameters(aMatrix, bMatrix, cMatrix, dMatrix);
```

The base does the rest: `recomputeDiscreteModel` calls
`Math::calculateStateSpaceTrapezoidalMatrices` and sets `mW = mC * mdB + mD`,
`calculateHistoryVector` returns `mC * (mdA * x + mdB * u)`, and the post step updates the state
from the old and new input.

A varying component additionally overrides `updateStateSpaceModel` (a no-op for linear components)
and, for the variable bases, `updateComponentParameters` to report whether the model changed. Only
when it reports a change is the system matrix refactorised.

## Domain differences

The formulation differs by domain, and so does the code path. The theory is under
[SSN across domains]({{< ref "/docs/Concepts/ssn-domain-formulation.md" >}}).

A component supplies the same **real** `(A, B, C, D)` in either domain. `EMT::SSNComp` discretises
them directly. `DP::SSNComp` does not: `buildAugmentedA(omega)` assembles the real-augmented
`2n x 2n` matrix with `A` on both diagonal blocks and `+wI` / `-wI` off-diagonal,
`buildAugmentedB` places `B` on both diagonal blocks, and the result goes through the *same*
`Math::calculateStateSpaceTrapezoidalMatrices` helper as EMT. The discrete blocks are then folded
back into complex form as `topLeft + j * bottomLeft`, which is the inverse of the
`[[P, -Q], [Q, P]]` representation. `mW` and the history vector are complex as a result.

`recomputeDiscreteModel` therefore takes `omega` in DP and takes no argument in EMT. A component
that hardcodes a frequency here rather than using the value handed to `mnaCompInitialize` is wrong
at any other system frequency.

{{% alert title="Watch out: the mixed SSN base needs a pre-shifted matrix" color="warning" %}}
One base does not follow this pattern. `MixedVTypeVariableSSNComp` does **not** augment internally:
it requires the derived component to hand it a state matrix that is already carrier shifted, because
its steady-state solve assumes so. Supplying an unshifted matrix there initializes to the wrong
operating point rather than failing, and it is the single easiest mistake to make when porting a
component from EMT to DP. Its discrete Norton admittance is also retained as the full packed-real
`2 x 2` block `C * Bd + D`. Mixed real/envelope controls do not generally have the
`[[P, -Q], [Q, P]]` structure required to fold that block into one complex scalar.
{{% /alert %}}

## Frame metadata

`getLocalAbcStateBlocks` returns nothing by default and should be overridden **only** for states
that genuinely form physical abc triples. It is consumed by tooling that reasons about the state
vector in the phase frame, and declaring a block that is not one produces wrong groupings rather
than an error.

## Initialization

`calculateSteadyStateStateFromInput` evaluates `(jωI − A)⁻¹ B u`, which requires the continuous
model to be set first. Components with real control states cannot use the default
`initializeFromNodesAndTerminals` on the mixed base; see
[DP Ph1 averaged VSI implementation]({{< ref "dp-ph1-averaged-vsi-implementation.md" >}}) for that
case and for the requirement that the state matrix be handed over already carrier shifted.

## The components

Fixed models: `SSN_Full_Serial_RLC`, `SSN_Capacitor`, `SSN_Inductor`, `SSNTypeV2T`, `SSNTypeI2T`.
Varying models: `SSN_Variable_Serial_RLC`, `PiecewiseLinearInductor`, and the inverter models under
[power electronics]({{< ref "/docs/Concepts/Models/Power Electronics" >}}). The `Generic` two- and
four-terminal classes take the matrices from the caller instead of forming them, so they are the
route to an SSN component without writing C++. Availability per domain is in
[model availability]({{< ref "/docs/Reference/model-availability.md" >}}).
