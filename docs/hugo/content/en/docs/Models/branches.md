---
title: "Branches"
linkTitle: "Branches"
date: 2026-07-31
description: >
  Line models connecting two nodes of the network.
---

Both line models below are composite components: they do not stamp the system matrix directly
but are built from resistor, inductor and capacitor subcomponents, each of which contributes its
own stamp. See [subcomponents]({{< ref "/docs/Overview/subcomponents.md" >}}) for how that
composition works, and [RLC elements]({{< ref "RLC-Elements" >}}) for the stamps of the
individual elements.

The transformer is documented separately under [transformer]({{< ref "Transformer" >}}).

## RX-Line

The RX line represents a line by its series resistance and series inductance only, ignoring the
shunt admittance. It is the appropriate choice for short lines, where the charging current is
negligible, and it is what the CIM reader produces for an `ACLineSegment` when no shunt data is
present.

The model is composed of a series resistor and a series inductor between the two terminals:

```math
\underline{Z} = R + j \omega L
```

An additional resistor from the inductor terminal to ground is present to make initialisation
well posed. It is not part of the physical model.

`RxLine` exists in `DP::Ph1`, `EMT::Ph3`, `SP::Ph1` and `SP::Ph3`.

## PI-Line

The PI line adds the shunt admittance of the line, split evenly between the two terminals, which
matters once the line is long enough for the charging current to affect the result. The name
comes from the shape of the equivalent circuit: a series branch with one shunt branch at each
end.

The series branch carries the resistance and inductance as above. Each terminal additionally
carries half of the total shunt capacitance and half of the total shunt conductance:

```math
\underline{Y}_{shunt} = \frac{G + j \omega C}{2}
```

Parameters are set through `setParameters(seriesResistance, seriesInductance, parallelCapacitance,
parallelConductance)`, where the capacitance and conductance given are the totals for the line;
the model performs the halving itself. The corresponding attributes are `mSeriesRes`,
`mSeriesInd`, `mParallelCap` and `mParallelCond`.

`PiLine` exists in `DP::Ph1`, `DP::Ph3`, `EMT::Ph1`, `EMT::Ph3` and `SP::Ph1`.

## Choosing between them

Use the RX line when the shunt admittance can be neglected and you want the smaller system
matrix, since the PI line introduces additional nodes for its shunt branches. Use the PI line
when the line is long enough that its charging current matters, or when you are comparing
against a reference tool that models the shunt branch.

Note that both are lumped parameter models and therefore do not reproduce travelling wave
behaviour. For splitting a network across solvers or simulators, see the decoupling components
listed under [models]({{< ref "_index.md" >}}).
