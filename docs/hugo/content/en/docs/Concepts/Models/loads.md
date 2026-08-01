---
title: "Loads"
linkTitle: "Loads"
date: 2026-07-31
description: >
  Constant impedance and constant current representations of a load, and what each assumes.
weight: 6
---

A load is specified as an active and a reactive power at a nominal voltage, but a nodal solver needs
either an admittance or a current. The two ways of making that conversion behave differently as the
terminal voltage moves away from nominal, and the difference matters more than the model's
simplicity suggests.

## Constant impedance

The powers are converted once, at the nominal voltage, into a resistance and a reactance,

```math
R = \frac{V_{nom}^2}{P}, \qquad X = \frac{V_{nom}^2}{Q},
```

and the reactance becomes an inductance or a capacitance according to its sign,

```math
L = \frac{X}{\omega} \quad (X > 0), \qquad C = -\frac{1}{\omega X} \quad (X < 0).
```

The load is then an ordinary passive branch to ground, and it is stamped exactly as the elements it
is built from.

Both conversions divide by a power, so a load with zero active power has no defined resistance and
one with zero reactive power has no defined reactance. Such a branch is simply absent rather than
infinite, which is the correct behaviour but means a load specified with one of the two set to zero
is not the load a reader might expect.

The assumption is that consumption follows the square of the voltage. At nominal voltage the load
draws exactly $P$ and $Q$; at 0.9 per unit it draws 81 percent of them. For a genuinely impedance
like load this is right, and for anything regulated it understates the demand during a depression.

## Constant current

The alternative injects a current derived from the specified power,

```math
\underline{I} = \left( \frac{S}{V_{nom}} \right)^{*},
```

held fixed as the terminal voltage varies. Consumption then falls linearly with voltage rather than
quadratically, which is closer to the behaviour of many aggregated loads.

Note what this is not. Because the current is computed from the nominal voltage and not from the
present terminal voltage, this is a constant current model and not a constant power one. A true
constant power load would require the current to be recomputed from the solved voltage at every
step, making the component nonlinear and the nodal solve iterative. The linear model is used
because it keeps the system matrix constant.

## Which to use

The three canonical load characteristics are constant impedance, constant current and constant
power, differing in whether demand follows the square of voltage, the voltage, or neither. Only the
first two are available as linear models. For a voltage excursion of a few percent the choice
changes little; for a deep depression during a fault it changes the answer materially, and the
constant impedance model is the optimistic one because it sheds load exactly when the network is
weakest.

## Shunts

A shunt is specified directly as a conductance and a susceptance rather than as a power, so no
conversion is involved. It is the natural representation for a capacitor bank or a reactor, where
the rating is an admittance and the consumed power is a consequence of the voltage rather than the
specification.
