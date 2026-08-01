---
title: "Python Tutorials"
linkTitle: "Python"
weight: 1
description: >
  The ladder, worked in Python.
---

Each tutorial starts from the one before and adds exactly one new thing. Work through them in order;
each is a complete runnable script rather than a fragment.

1. [Your first simulation]({{< ref "first-simulation" >}}). A source and a resistor. The shape of
   a script, and how to read a result back.
1. [Adding dynamics]({{< ref "adding-dynamics" >}}). An inductor, the transient it produces, and
   how to choose a time step.
1. [A network, and where it starts from]({{< ref "two-bus-network" >}}). A line between two
   buses, and initializing the dynamic run from a powerflow.
1. [Applying a fault]({{< ref "applying-a-fault" >}}). Switching during a run, and why clearing
   a fault needs more care than applying one.
1. [The same circuit in two domains]({{< ref "comparing-domains" >}}). Waveforms against envelopes,
   and what the envelope buys.
1. [Adding a machine]({{< ref "a-machine" >}}). A synchronous generator, initializing it correctly,
   and what the model order changes.
1. [Exchanging data with another tool]({{< ref "exchanging-data" >}}). Handing a value out of a
   running simulation, and where the boundary lies.

A rung on converters and their control belongs between the last two and is not written yet.

## What you need

DPsim importable from Python. If it is not, see
[install]({{< ref "/docs/User Guide/install.md" >}}) or
[build]({{< ref "/docs/Developer Guide/Architecture and Conventions/build.md" >}}).

Reading results back and plotting them uses the data processing package the example notebooks also
use. It is separate from DPsim and is imported as `villas.dataprocessing`.
