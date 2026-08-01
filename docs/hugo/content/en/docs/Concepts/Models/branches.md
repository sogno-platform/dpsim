---
title: "Branches"
aliases: ["/docs/models/branches/"]
linkTitle: "Branches"
date: 2026-07-31
description: >
  Line models connecting two nodes of the network.
weight: 2
---

Both line models below are composite components: they do not stamp the system matrix directly
but are built from resistor, inductor and capacitor subcomponents, each of which contributes its
own stamp. See [subcomponents]({{< ref "/docs/Developer Guide/Writing a Model/subcomponents.md" >}}) for how that
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

The shunt capacitance and conductance are specified as totals for the line, and the halving between
the two ends is part of the model rather than something the user does.

## Decoupling Line

The decoupling line is a distributed parameter line based on the Bergeron travelling wave
method. Unlike the two models above it is not primarily a fidelity improvement: its purpose is
to remove the direct coupling between the two terminals so that the network on either side can
be solved as an independent system, which is what makes splitting a network across solvers or
across simulators possible.

The method rests on the behaviour of a lossless line. For a line with distributed inductance and
capacitance, the quantity $v + Z_c\, i$ observed at one end reappears unchanged at the other end
one travel time later, and likewise in the opposite direction. Nothing propagates faster than
that travel time, so the two ends cannot influence each other within it. The surge impedance and
the travel time follow from the line's total inductance and capacitance,

```math
Z_c = \sqrt{\frac{L}{C}}, \qquad \tau = \sqrt{L C}.
```

Each terminal is then represented by a resistance to ground in parallel with a current source.
The resistance is $Z_c + R/4$, and the current source carries the history term, whose value
depends on the voltage and current recorded at the *other* terminal one travel time ago. Because
that value is already known when the step begins, it enters the system as a constant injection
rather than as a coupling into the admittance matrix, and the matrix separates into two blocks
that can be factorised and solved independently.

The series resistance is not distributed along the line. It is lumped, with $R/4$ placed at each
end and the remainder in the middle of the equivalent, which is why the terminating resistance
and the history coefficients carry $R/4$ terms rather than the full $R$.

The travel time is not required to be a whole number of time steps. The recorded quantities are
held in a buffer of $\lceil \tau / \Delta t \rceil$ samples and the value one travel time ago is
recovered by linear interpolation between the two nearest entries. The one hard requirement is
that the travel time exceed the time step; a line whose $\tau$ is shorter than $\Delta t$ cannot
decouple anything, and setting one up is rejected rather than silently approximated.

In the dynamic phasor domain the history terms carry an additional rotation $e^{-j \omega_s
\tau}$. This is a direct consequence of working with envelopes: a delay of $\tau$ applied to an
instantaneous waveform becomes, for the envelope, the same delay together with a phase rotation
of the carrier over that interval, as described under
[dynamic phasors]({{< ref "/docs/Concepts/dyn-phasors.md" >}}). Note that this rotation is
currently evaluated at a fixed 50 Hz rather than at the system frequency in use.

The decoupling is exact for the lossless travelling wave line it is derived from. The error
introduced in practice comes from the lumped treatment of the series resistance and from the
interpolation of the delayed quantities, and it grows as the time step approaches the travel
time.

## Choosing between them

Use the RX line when the shunt admittance can be neglected and you want the smaller system
matrix, since the PI line introduces additional nodes for its shunt branches. Use the PI line
when the line is long enough that its charging current matters, or when you are comparing
against a reference tool that models the shunt branch.

Both are lumped parameter models and therefore do not reproduce travelling wave behaviour. Use
the decoupling line when you need that behaviour, or when the reason for reaching for a line
model is to split the network in the first place. For the domains each model is available in,
see [model availability]({{< ref "/docs/Reference/model-availability.md" >}}).
