---
title: "Co-simulation and Interfaces"
linkTitle: "Co-simulation"
weight: 6
description: >
  Exchanging signals with other simulators, services or hardware during a run.
aliases: ["/docs/overview/interfaces/"]
date: 2025-02-13
---

Interfaces can be used to exchange simulation signals between a DPsim simulation and other soft- or hardware, for example an MQTT-broker or an FPGA.
Simulation signals in the form of [Attributes]({{< ref "/docs/Developer Guide/Attributes and Scheduling/Attributes/index.md" >}}) can be **imported** or **exported** once per simulation time step.
Interfaces are subclasses of `Interface` and implement the methods `addExport` and `addImport`, which add dependencies to the passed attribute that forward the attribute value from or to the interface.
This way, attributes that are imported are read from the interface before they are used in any DPsim component.
Attributes that are exported are written to the interface after they are set by a DPsim component.

## Where the boundary is, and where to read further

This page documents only DPsim's side of the interface: which attributes are exchanged, when they
are read and written relative to the time step, and how the simulation is synchronized. Everything
on the other side of the JSON configuration belongs to VILLASnode and is documented there rather
than here, so the two should be read together.

The parts most often needed are these:

- [Node types](https://villas.fein-aachen.org/doc/node-node-types.html) is the reference for the
  `type` key and its per-type options. Which protocols are available, and what each one requires,
  is decided here rather than in DPsim.
- [Nodes](https://villas.fein-aachen.org/doc/node.html) covers the surrounding configuration
  structure that the node object sits in.
- [Hooks](https://villas.fein-aachen.org/doc/node-hooks.html) describe the processing that can be
  applied to samples in transit, such as scaling, limiting or statistics. Anything that can be done
  with a hook does not need to be done in the simulation, which is usually the better place for it.

{{% alert title="Watch out: the signal mapping is positional, not by name" color="warning" %}}
Two things about the split are worth knowing before configuring anything. The signal ordering in
the VILLASnode configuration must match the order in which attributes are exported and imported,
because the mapping is positional rather than by name; a mismatch produces a running simulation
exchanging the wrong quantities. And the queueless interface additionally reserves the first input
signal for a sequence number, so its signal list is offset by one relative to a queued
configuration carrying otherwise identical data. That is described with the rest of the
configuration under
[interfaces]({{< ref "/docs/Developer Guide/Attributes and Scheduling/interface-tasks.md" >}}).
{{% /alert %}}
