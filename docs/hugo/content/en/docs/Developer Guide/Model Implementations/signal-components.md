---
title: "Signal Component Implementation"
linkTitle: "Signal Components"
date: 2026-07-31
description: >
  How control and signal blocks are written, stepped and scheduled.
weight: 25
---

The models are derived under
[converter control]({{< ref "/docs/Concepts/Models/converter-control.md" >}}) and
[signal processing blocks]({{< ref "/docs/Concepts/Models/signal-processing.md" >}}). This page
covers only the code.

## Base and scheduling

Signal blocks derive from `SimSignalComp` and take no part in the nodal solve. They contribute tasks
through `getTasks()` rather than through the MNA hooks, and the scheduler orders them from the
attribute dependencies those tasks declare. A block that reads an attribute without declaring it may
still produce the right answer, by luck of ordering, and then change behaviour when an unrelated
component is added; see
[adding tasks to a component]({{< ref "adding-tasks.md" >}}).

Most blocks follow a two-task shape: a `PreStep` that copies the current values into the previous
ones, and a `Step` that computes the new state and output. The split exists so that a value consumed
by another block within the same step is unambiguous about which timestep it belongs to.

## The `mInputPrev` / `mInputCurr` pattern

Blocks that integrate with the trapezoidal rule need both the present and the previous input, so
they carry `mInputPrev`, `mInputCurr`, `mStatePrev`, `mStateCurr` and the matching output pair. The
`PreStep` task performs the shift. `Integrator::signalStep` is the whole pattern:

```cpp
**mStateCurr = **mStatePrev + mTimeStep / 2.0 * **mInputCurr
                            + mTimeStep / 2.0 * **mInputPrev;
**mOutputCurr = **mStateCurr;
```

`VCO::signalStep` deliberately does not, using `state + dt * input`, because it accumulates an angle
rather than integrating a control signal.

Every one of these blocks needs `setSimulationParameters(timestep)` before the run, since the step
size appears directly in the update. Blocks that expose `setInitialValues` must also have it called,
or they start from zero rather than from the operating point.

## State-space blocks

`PLL` is written as an explicit state-space block rather than as arithmetic, setting

```cpp
mA << 0, mKi, 0, 0;
mB << 1, mKp, 0, 1;
mC << 1, 0, 0, 1;
mD << 0, 0, 0, 0;
```

The first input is the nominal frequency and is held constant, which is how the feed-forward term
enters. Writing it this way means the block can be discretised by the same helpers as anything else
rather than by hand.

## `FIRFilter`

`FIRFilter` keeps a circular buffer and a write index, and `step` sums `mFilter[i] * mSignal[...]`
over the filter length before advancing the index. It contributes a single `Step` task. Filter
coefficients are supplied by the caller; nothing validates their length against the buffer or checks
that they sum to a sensible gain.

## Generators

`SignalGenerator` is the abstract base; `SineWaveGenerator`, `DCGenerator`, `CosineFMGenerator` and
`FrequencyRampGenerator` are the concrete ones, and all expose their value through a `sigOut`
attribute that a source component references.

{{% alert title="Watch out: the default ramp depends on step history" color="warning" %}}
`FrequencyRampGenerator` has two modes. The default accumulates phase incrementally, deriving its
timestep as `time - mOldTime` rather than from a configured step. The `mUseAbsoluteCalc` path
computes the phase in closed form from the ramp parameters instead. The incremental path makes the
waveform depend on the step history; the absolute path does not. Prefer the absolute path when
comparing runs at different step sizes.
{{% /alert %}}

## Source

Under `dpsim-models/src/Signal/`. Availability is in
[model availability]({{< ref "/docs/Reference/model-availability.md" >}}); these blocks are domain
independent and appear there as a list rather than a matrix.
