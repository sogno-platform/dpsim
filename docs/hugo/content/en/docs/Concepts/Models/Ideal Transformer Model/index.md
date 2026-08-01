---
title: "Ideal Transformer Model"
aliases: ["/docs/models/ideal-transformer-model/"]
linkTitle: "Ideal Transformer Model"
date: 2026-06-26
author: Andres Acosta <andres.acosta@eonerc.rwth-aachen.de>
description: >
  Splitting a circuit at any point with a controlled source pair, and what the delay costs.
weight: 4
---

The Ideal Transformer Model (ITM) is a signal component that splits a circuit into two subcircuits, using a common node as a Point of Common Coupling (PCC), in such a way that a copy of this node is found in the two subcircuits, as shown in Fig. 1, where the copies of the node are denoted as $n$ and $m$. Moreover, the circuits are coupled using a controlled voltage source and a controlled current source, which exchange their interface  currents and voltages, respectively, namely the interface signals. This exchange takes place using a ring buffer, on top of which a second ring buffer has been implemented to emulate a co-simualtion using a macro-step, which means that the exchange of interface signals can be made at an interval larger than the simulation's step size. This second ring buffer is used to implement Zero- and First-Order hold extrapolation methods, while the first ring buffer allows to linearly interpolate the value of the signal at the current time step, in case the delay between both subcircuits is not an integer multiple of the step size.

<center>
<figure margin=30%>
    <img src="./images/ITM.svg" alt="ITM">
    <figcaption>Fig. 1: Ideal Transformer Model Circuit diagram.
    </figcaption>
</figure>
</center>

To add an ITM, users must split the cirtuit and create the copies of the PCC node. An example of this process can be found in the Notebook `ITM.ipynb`.

To avoid connections of the controlled voltage source with a capacitor, or the controlled current source with an inductor, the resistors $R_{\mathrm{series}}$ and $R_{\mathrm{parallel}}$ are included.

## Why the resistors are necessary

The two failure cases they prevent are the same one seen twice. A voltage source directly across a
capacitor over-determines that node: both impose a voltage, and the capacitor's companion model and
the source's constraint row describe the same quantity. A current source in series with an inductor
under-determines the branch in the dual way, since both impose a current. In each case the system
matrix becomes singular rather than merely ill-conditioned, so the resistors are a condition for the
method to work at all and not a refinement of it.

Their values are a compromise of the kind described under [switches]({{< ref "../switches.md" >}}).
Small enough to be electrically negligible, large enough not to dominate the condition number.

## What the delay costs

The exchanged signals are always at least one step old, because each side computes from what the
other produced previously. That delay is the reason the two subcircuits can be solved separately at
all, and it is also the entire error of the method: the coupled system is not the original circuit
but the original circuit with a transport delay inserted at the point of common coupling.

The consequence is that accuracy is governed by how much the interface signals change within one
exchange interval, not by how accurately either side is solved internally. Refining the step inside
a subcircuit while holding the macro-step fixed improves nothing at the interface.

A macro-step larger than the simulation step makes this explicit, which is the point of the second
ring buffer: it is the co-simulation case, where the two sides may be different tools exchanging at
a rate neither controls. Zero-order hold holds the last received value for the whole interval;
first-order hold extrapolates linearly from the last two. The first is safe and lags; the second
tracks a smoothly varying signal better and overshoots at a discontinuity, which is exactly what a
fault produces.

## Contrast with the alternatives

Three ways of splitting a network appear in this documentation and they differ in what they cost.

Tearing, described under
[alternative solution methods]({{< ref "/docs/Concepts/alternative-solvers.md" >}}), is exact: the
removed branches are restored within the same step, so the answer matches the intact network. It
does not allow the parts to be advanced independently.

The travelling-wave line under [branches]({{< ref "../branches.md" >}}) is exact for the lossless
line it derives from, and its delay is physical rather than introduced. It requires that a real line
with a travel time longer than the step exists at the splitting point.

The ideal transformer model requires no such line and can split anywhere, and pays for that with a
delay that has no physical counterpart. It is the general method and the least accurate of the
three.
