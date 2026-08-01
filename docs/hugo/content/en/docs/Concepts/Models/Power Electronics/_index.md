---
title: "Power Electronics"
linkTitle: "Power Electronics"
weight: 11
description: >
  Averaged voltage source inverter models and their control.
aliases: ["/docs/models/power-electronics/","/docs/concepts/models/power-electronics/"]
---

Every inverter model here is averaged: the switching is not represented, and the converter is
treated as a controllable voltage behind its filter. Averaging removes the switching frequency from
the problem, which is what allows a step size set by the control bandwidth rather than by the
carrier. It also means these models say nothing about switching losses, harmonic injection or any
behaviour that depends on the modulation itself.

The control that sits on top of each is derived separately under
[converter control]({{< ref "../converter-control.md" >}}), because the same cascade appears in more
than one of these models.

## Choosing among them

The models differ along two axes: which domain they are written in, and whether the converter
follows the grid or forms it.

[EMT Ph3 averaged VSI]({{< ref "emt-ph3-averaged-vsi.md" >}}) is the reference formulation. All
fourteen states are real, the three filter phases are represented individually, and there is no
carrier, so nothing is assumed about the bandwidth of what it carries.

[DP Ph1 averaged VSI]({{< ref "dp-ph1-averaged-vsi.md" >}}) is the same converter as a single
positive-sequence envelope. Its six real filter states become two complex envelopes, which is the
saving the envelope description buys, at the cost of being unable to represent an unbalance.

[DP Ph3 averaged VSI]({{< ref "dp-ph3-averaged-vsi.md" >}}) restores per-phase representation in the
envelope domain, with one complex envelope per phase and a controller that keeps a single
positive-sequence frame. Because three independent phase envelopes admit a negative-sequence
component, it carries negative-sequence current control that the single-phase model has no need for.

[EMT Ph3 grid-forming VSI]({{< ref "emt-ph3-grid-forming-vsi.md" >}}) is the one that differs in
kind rather than in representation. It carries its own frequency and angle as states instead of
tracking the grid's, so it can energise a network with no other source. Its control is nonlinear
enough that the model is linearized numerically at each operating point rather than written in
closed form.

## What they share

All four are solved simultaneously with the network rather than through a delayed injection, using
the state-space nodal method described under
[SSN components]({{< ref "../ssn-components.md" >}}). All four are therefore re-formed as the
operating point moves, and all four make the system matrix change at every step, which is the cost
of the approach.
