---
title: "Models"
linkTitle: "Models"
weight: 10
description: >
  Mathematical description of the models implemented in DPsim.
---

Each page in this section derives one component family: what it represents, the equations that
describe it, and what those equations assume. None of them names a class or a file. How a model is
arranged in code is in the [Developer Guide]({{< ref "/docs/Developer Guide" >}}), and which domains
implement which model is in
[model availability]({{< ref "/docs/Reference/model-availability.md" >}}), which is generated from
the source and is therefore the authoritative answer to that question.

## The domains

A model is written per simulation domain and per phase count, so the same component may exist as a
single-phase dynamic phasor model, a three-phase electromagnetic transient model, or both.

`EMT` carries instantaneous waveforms, `DP` carries complex envelopes of those waveforms around a
carrier frequency, and `SP` carries steady-state phasors. `Ph1` is single phase, usually a positive
sequence representation, and `Ph3` is three phase. The three domains are not separate methods but
cases of one envelope description, which is the subject of
[dynamic phasors]({{< ref "/docs/Concepts/dyn-phasors.md" >}}).

The practical consequence is that the domain determines what a model can represent, not only how
fast it runs. An envelope domain cannot represent content outside the band it retains, whatever step
size is used.

## How the models group

**Passive elements and branches.** [RLC elements]({{< ref "RLC-Elements" >}}) covers the elements
every other model is built from. [Branches]({{< ref "branches.md" >}}) covers the lines connecting
two nodes, including the travelling-wave line. [Transformer]({{< ref "Transformer" >}}) covers the
two-winding transformer and the ideal transformer within it.

**Sources, switches and loads.** [Sources]({{< ref "sources.md" >}}) explains why a current source
costs nothing while a voltage source extends the system matrix.
[Switches]({{< ref "switches.md" >}}) and [loads]({{< ref "loads.md" >}}) cover the two-resistance
switch and the impedance and current representations of demand. These three share one theme: a
choice that looks physical is usually a numerical trade-off.

**Machines.** [Synchronous generator]({{< ref "Synchronous Generator" >}}) covers the full-order and
transient-stability machines, and
[reduced order]({{< ref "Synchronous Generator/reduced-order.md" >}}) the voltage-behind-reactance
family from third to sixth order. The
[regulators]({{< ref "Synchronous Generator Regulators" >}}) that drive them, exciters, turbines and
governors, sit alongside.

**Converters.** [Power electronics]({{< ref "Power Electronics" >}}) covers the averaged inverter
models, and [converter control]({{< ref "converter-control.md" >}}) the phase-locked loop, the
oscillator and the cascaded control that distinguish grid-following from grid-forming behaviour.

**Interfaces and compensation.** [SSN components]({{< ref "ssn-components.md" >}}) covers components
solved simultaneously with the network, the
[ideal transformer model]({{< ref "Ideal Transformer Model" >}}) the general way of splitting a
network, and [network injection and compensation]({{< ref "network-injection-and-compensation.md" >}})
the representation of the grid beyond the boundary.

**Signal blocks.** [Signal processing]({{< ref "signal-processing.md" >}}) covers the integrators,
filters and generators the other models are assembled from. These carry no current and connect to no
node.
