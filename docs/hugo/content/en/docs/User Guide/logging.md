---
title: "Logging Results"
linkTitle: "Logging"
weight: 4
date: 2026-07-31
description: >
  Choosing what a simulation records, where it goes, and what it costs.
---

A simulation records nothing unless asked. Every quantity you want afterwards has to be named before
the run, because the solver keeps only what the current step needs and discards the rest.

`dpsimpy.Logger` is the one to reach for. It is the CSV data logger and it is what nearly every
example and notebook uses. There is also a real-time data logger, which is the right choice under a
real-time timer for the reason given below, and an interface for sending results somewhere other
than a file.

## Only attributes can be logged

{{% alert title="Watch out: only attributes can be logged" color="warning" %}}
A logger records an **attribute** and nothing else. It cannot record an arbitrary expression, a
plain member variable, or a quantity a component computes internally without publishing.

The same constraint governs co-simulation and task scheduling, so a value that is not an attribute
cannot be logged, exchanged with another tool, or depended on by another task. Making it one is a
change to the model, not to the call site.
{{% /alert %}}

So the question is never "how do I log this value" but "is this value an attribute". If it is, one
line records it. If it is not, no logger option will reach it, and the answer is to expose it as an
attribute in the model, which is a code change described under
[attributes]({{< ref "/docs/Developer Guide/Attributes and Scheduling/Attributes" >}}).

`print_attribute_list()` on any object prints what it publishes, which is the reliable way to find
out. Anything absent from that list cannot be logged.

The same fact explains a convenience: because attributes are the unit, a derived quantity that is
itself an attribute is logged the same way as a terminal voltage, with no special handling.

## Registering an attribute

```python
logger = dpsimpy.Logger("my_simulation")
logger.log_attribute("n1.v", "v", n1)
logger.log_attribute("load.i_intf", "i_intf", load)

sim.add_logger(logger)
```

The three arguments are the column name you want in the output, the name of the attribute on the
object, and the object itself. The first is yours to choose and is what you will key on when reading
the file back; the second must match an attribute the object actually publishes.

Common attribute names are `v` on a node, and `i_intf` and `v_intf` on a component for the current
through it and the voltage across it. Components publish their own states as well: a machine offers
`w_r`, `delta` and `Te`, a converter its control states.

Naming an attribute that does not exist fails when the logger is set up, not at the end of the run,
so a typo costs a second rather than a simulation.

## Where the file goes

By default the file is `logs/<logger name>.csv` under the working directory, where the name is the
one given to the `Logger` constructor.

`Logger.set_log_dir` changes that directory and `Logger.get_log_dir` reports it. Calling it is what
produces the nested `logs/<something>/<name>.csv` layout the example notebooks use, so a script that
does not call it gets the flat form. Setting it is worth doing when one script runs several
simulations, since two loggers with the same name otherwise write to the same file.

The same setting governs the diagnostic text log, which is why the two land side by side.

The file is plain CSV with a `time` column first and one column per logged attribute, in the order
they were registered. A three-phase quantity becomes three columns suffixed `_0`, `_1`, `_2`, and a
complex quantity in an envelope domain is written as a complex value that the reading side parses
back.

## Reading it back

```python
import villas.dataprocessing.readtools as rt

results = rt.read_timeseries_dpsim("logs/my_simulation.csv")
voltage = results["n1.v"]
```

The keys are the column names you chose. Each value carries `time` and `values` arrays. In an
envelope domain the values are complex, so `abs()` gives the magnitude, and
`frequency_shift_list` recovers the waveform as shown in
[comparing domains]({{< ref "/docs/Tutorials/Python/comparing-domains" >}}).

The first row is written before the first solve, so it holds the state the simulation started from
rather than a result. A plot that appears to begin at zero usually begins at that row.

## What it costs

Logging is opt-in and costs nothing for what you do not ask for, but what you do ask for is written
every step. A run of 100 000 steps logging 50 attributes writes five million values, and on a large
network the file, not the solve, becomes the slow part.

Three things help. Log the attributes you will actually look at rather than everything available.
Prefer a specific attribute over a whole matrix; the `rows_max` and `cols_max` arguments to
`log_attribute` cap how much of a matrix quantity is written. And note that a large time step
reduces the file in exact proportion, so a study that only needs the envelope does not need the
step of one that needs the waveform.

{{% alert title="Note: down-sampling is not reachable from Python" color="primary" %}}
The C++ `DataLogger` additionally accepts a down-sampling factor, writing every n-th step. That
argument is not exposed on the Python `Logger`, which takes only a name, so from Python the step
size is the only control over how many rows you get.
{{% /alert %}}

## Data logging against diagnostic logging

The word covers two unrelated things and they are easy to confuse.

What this page describes is the **data logger**: numerical results, CSV, opt-in per attribute. The
other is the **diagnostic log**, the text file recording what the solver did, controlled by
`dpsimpy.LogLevel` and passed to component constructors. Raising a component's log level makes it
describe its own initialization and stamping in prose; it has no effect on the CSV.

They land in the same `logs/` directory side by side, one as `.csv` and the other as `.log`, which
is why they get mistaken for each other. A component constructed with `LogLevel.debug` produces a
great deal of text and no additional results.

The real-time data logger mentioned at the top is a data logger like the first, not a third kind of
log. It records the same results and differs only in buffering them in memory and writing at the
end, because a disk write inside a real-time step has no bound on how long it takes. See
[real-time]({{< ref "real-time.md" >}}).
