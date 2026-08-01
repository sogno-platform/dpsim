---
title: "State-Space Extraction Support"
linkTitle: "State-Space Extraction"
weight: 5
description: >
  Components that contribute states to the extracted state-space model.
---

For how extraction works and how to enable it, see
[state-space extraction]({{< ref "/docs/Developer Guide/Solvers/state-space-extraction.md" >}}).

State-space extraction is available for EMT Ph3 and DP Ph1 simulations
using the direct MNA solver. For models containing switches, the extracted matrix represents the currently
active switch configuration. The matrix is recomputed when the switch status
changes.

## EMT Ph3

Supported components with extraction states are:

- `EMT::Ph3::Inductor`,
- `EMT::Ph3::Capacitor`,
- `EMT::Ph3::TwoTerminalVTypeSSNComp`,
- `EMT::Ph3::TwoTerminalVTypeVariableSSNComp`.

Supported algebraic components without extraction states are:

- `EMT::Ph3::Resistor`,
- `EMT::Ph3::Switch`,
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
- `DP::Ph1::Switch`,
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
