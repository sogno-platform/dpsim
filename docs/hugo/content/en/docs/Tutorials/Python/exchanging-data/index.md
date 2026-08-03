---
title: "Exchanging Data With Another Tool"
linkTitle: "Exchanging Data"
weight: 8
date: 2026-07-31
description: >
  Handing a value out of a running simulation, and where the boundary lies.
---

Everything so far has ended with a CSV read after the run. This tutorial hands a value out **while**
the simulation runs, which is what co-simulation, hardware in the loop and any live coupling are
built on.

The far side here is a file, so nothing external has to be running. A file is a poor co-simulation
partner and an excellent first one: the mechanism is identical to an MQTT broker or an FPGA, and
only the configuration changes.

## What is different about this run

```python
sim = dpsimpy.RealTimeSimulation(name)
...
sim.run(1)
```

It is a `RealTimeSimulation`, not a `Simulation`, and `run` takes a start delay in seconds. An
exchange is paced by the wall clock rather than by how fast the solver can go, because the other
side is a real thing running in real time. A one-second simulation takes one second.

That also means the results carry wall-clock timestamps rather than simulation time, which is
visible in the output below.

## Configuring the far side

```python
interface_config = {
    "type": "file",
    "format": "csv",
    "uri": "logs/exchanged.csv",
    "out": {"flush": True},
}

interface = dpsimpyvillas.InterfaceVillas(
    name="dpsim-file", config=json.dumps(interface_config)
)
```

This dictionary is **not** DPsim configuration. It is a VILLASnode node description, passed through
as JSON, and its keys are documented by VILLASnode rather than here. Changing `type` from `file` to
`mqtt` and giving a broker address is the entire difference between writing to disk and publishing
to a broker; nothing in the simulation changes.

{{% alert title="Requires a build with VILLASnode" color="info" %}}
This tutorial needs `dpsimpyvillas`, a separate extension module from `dpsimpy`, which only exists
in a build configured with VILLASnode available. The rest of the ladder needs only the installed
Python package.
{{% /alert %}}

## Choosing what crosses the boundary

```python
interface.export_attribute(boundary.attr("i_intf").derive_coeff(0, 0), 0)
sim.add_interface(interface)
```

Two things are worth reading slowly.

The exported quantity is an **attribute**, the same unit the logger works in. `attr("i_intf")` takes
the whole interface current, which is a matrix, and `derive_coeff(0, 0)` selects one element of it.
Without that you would be handing across a matrix where the far side expects a number.

{{% alert title="Watch out: the mapping is positional, not by name" color="warning" %}}
The second argument is a **position**, not a name. It is the index in the signal list on the
VILLASnode side, and the mapping between the two is entirely positional. Exporting two attributes in
one order and describing them in another produces a run that exchanges the wrong quantities without
any error at all, which is the failure to watch for.
{{% /alert %}}

## What comes out

The file is written in VILLASnode's sample format, not as a DPsim result CSV:

```text
# secs,nsecs,offset,sequence,signal0
1785540392,258011384,nan,0,5.00000000000000000+0.00000000000000000i
```

A wall-clock timestamp in seconds and nanoseconds, an offset, a sequence number, then one column per
exported signal. Over one second at a 10 ms step this run wrote 101 rows.

The `offset` column is `nan` here, and that is correct rather than a misconfiguration. It reports the
delay between when a sample was created and when it was received, so it only has a value on an
incoming path, where those two instants genuinely differ and the difference is the transport latency.
These samples originate in the simulation and go straight out, so there is no receive event to
measure against and the column is empty by construction.

The sequence number is what the far side uses to detect a missed sample. The queueless interface
relies on it directly, which is why it reserves the first imported signal for a sequence counter.

## Reading in the other direction

`import_attribute` is the mirror of `export_attribute` and makes an incoming value drive something
in the simulation, typically the reference of a controlled source. Two options change the timing:
`blockOnRead` halts the simulation at the start of every step until a new value arrives, and
`syncOnSimulationStart` holds the whole run until the far side has produced its first value. Both
are described under [co-simulation]({{< ref "/docs/User Guide/co-simulation.md" >}}).

Without either, the simulation reads whatever arrived most recently and carries on, which is the
right behaviour when the far side is slower and the wrong one when the exchange must be lock-step.

## The script

The complete script for this page is [`08_exchanging_data.py`](https://github.com/sogno-platform/dpsim/blob/master/examples/Python/Tutorials/08_exchanging_data.py) under `examples/Python/Tutorials`. It needs a build with VILLASnode available.

## Where to read further

DPsim documents its own side of the boundary: which attributes cross, and when they are read and
written relative to the step. Everything on the other side of that JSON belongs to VILLASnode and is
documented there. The [co-simulation]({{< ref "/docs/User Guide/co-simulation.md" >}}) page collects
the links.

The theory of what a delay across a coupling costs is under
[the ideal transformer model]({{< ref "/docs/Concepts/Models/Ideal Transformer Model" >}}), which is
the same argument whether the two sides are two solvers or two machines.
