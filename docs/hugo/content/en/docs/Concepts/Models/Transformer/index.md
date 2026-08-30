---
title: "Transformer"
aliases: ["/docs/models/transformer/"]
linkTitle: "Transformer"
date: 2021-07-22
description: >
  Two-winding transformer as a T-equivalent with a magnetizing branch, and the ideal part that extends the matrix.
weight: 3
---

## 2-Winding Transformer

The transformer is a T-equivalent: the winding resistance and leakage inductance are split evenly
between the two windings, a magnetizing branch is connected from the midpoint of that split to
ground, and an ideal transformer carries the ratio. The single line diagram is depicted in the
figure below.

![Transformer](electrical_transformer.svg)

Writing the total leakage impedance referred to the primary as $z = R + j \omega L$, the two series
segments are $z/2$ each and the magnetizing branch is a resistance in parallel with an inductance,

```math
y_m = \frac{1}{R_m} + \frac{1}{j \omega L_m} .
```

The model creates virtual nodes for the midpoint and for the ideal part, and two more when the
winding resistance is included, so a transformer with resistive losses occupies five virtual nodes
and one without occupies three.

## Magnetizing parameters

The branch is sized from the two no-load quantities a nameplate carries under IEC 60076, both given
per unit of the rated power $S_r$: the no-load current $i_0$ and the no-load loss $P_0$. With the
nominal voltage of the winding the impedance is referred to, $V_p$,

```math
R_m = \frac{V_p^2}{P_0 S_r} , \qquad
B_m = \frac{\sqrt{i_0^2 - P_0^2} \, S_r}{V_p^2} , \qquad
L_m = \frac{1}{\omega B_m} .
```

The defaults are $i_0 = 0.01$ and $P_0 = 0.001$, and `setMagnetizingBranch(i0, P0)` overrides them.
Two conditions are checked rather than assumed. The rated power must be positive, because it is the
only scale the branch can be sized against; a transformer without one is built with no magnetizing
branch and says so in the log. And $i_0$ must exceed $P_0$, because otherwise the square root above
has no real value and the branch has no reactive part at all, which is not a transformer. That case
raises rather than silently producing a purely resistive shunt.

This is why the rating is a required parameter rather than documentation, and the same is true of
the nominal voltages: the branch is referred to one winding, and which one is decided by the
reference winding.

## The ideal part and its extra equation

The ideal transformer imposes two constraints at once: the voltages are in a fixed ratio and the
powers on the two sides are equal, which makes the currents inversely proportional to the same
ratio,

```math
\frac{v_j}{v_k} = T, \qquad i_k = -T \, i_j .
```

Neither is a current balance at a node, so neither can be written as an admittance. This is the same
situation as an ideal voltage source described under [sources]({{< ref "../sources.md" >}}): the
system is extended with the branch current as an unknown, the constraint occupies the added row, and
the added diagonal entry is zero. The complete matrix stamp is

```math
\begin{array}{c|c c c}
  ~ & j & k & l \cr
  \hline
  j &  &  & -1 \cr
  k &  &  & \overline{T} \cr
  l & 1 & -T & 0
\end{array}
\begin{pmatrix}
v_j \cr
v_k \cr
i_{l} \cr
\end{pmatrix}
=
\begin{pmatrix}
  \cr
  \cr
  0\cr
\end{pmatrix}
```

The variable $j$ denotes the high voltage node while $k$ is the low voltage node, and $l$ indicates
the inserted row and column. The transformer ratio is $T = V_{j} / V_{k}$. The asymmetry of the
stamp, $-1$ against $\overline{T}$ in the added column and $1$ against $-T$ in the added row, is
exactly the statement that voltage scales by $T$ while current scales by $1/T$ with opposite sign.

A complex $T$ adds a phase shift, which is how a delta-wye connection is represented without
modelling the windings: the magnitude carries the tap ratio and the angle carries the vector group.
The conjugate on the low voltage row is what makes the phase shift preserve power rather than create
it. The three-phase representation in `SP::Ph3` and `DP::Ph3` carries this, and `EMT::Ph3` cannot,
because a phase shift is a rotation in the complex plane and the EMT state is real.

## The direction of the ratio

The reference winding decides which side the impedance is referred to. Referring an impedance across
an ideal transformer scales it by $T^2$, so the choice of side is a choice of reference, not an
approximation, and the parameters have to be given consistently with it.

By default the reference is resolved from the nominal voltages, taking the higher one, and the ratio
is then greater than one, from high voltage to low. Passing a `WindingReference` explicitly overrides
that: `Primary`, `Secondary` or `Tertiary` name the winding, and `Auto` restores the resolution from
nominal voltages. If both windings are nominally equal the resolution is ambiguous, the primary is
assumed and the model warns; an explicit reference removes the ambiguity.

Three-winding transformers are not implemented, so `Tertiary` raises rather than silently modelling
something else.

## Power flow and the behaviour gate

For power flow the T is reduced to a two-port, which is a Kron reduction eliminating the midpoint
node. With the half-leakage $z_h = z/2$ and $\Delta = z + z_h^2 y_m$,

```math
y_{\text{shunt}} = \frac{1 + z_h y_m}{\Delta} , \qquad
y_{\text{series}} = \frac{1}{\Delta} ,
```

```math
Y =
\begin{pmatrix}
y_{\text{shunt}} & -y_{\text{series}} T \cr
-y_{\text{series}} \overline{T} & y_{\text{shunt}} |T|^2
\end{pmatrix} .
```

At $y_m = 0$ this degenerates exactly to the series admittance $1/z$, so the reduction is the
textbook pi-with-tap plus a correction, not a different model.

Whether $y_m$ is present at all is decided by the component behaviour. Under
`Behaviour::Simulation` the magnetizing branch is left out, so a plain load flow matches the
textbook pi-with-tap that a user comparing against hand calculations expects. Under
`Behaviour::Initialization` and `Behaviour::MNASimulation` it is included, so that the power flow
solution seeds a time-domain run of the same circuit and the admittance the power flow sees is the
limit of the MNA conductance as the step grows.

## Scaling conventions across the domains

The domains do not agree on what a voltage or a current means, and the transformer touches all of
them, so the conventions are collected here.

Power flow node voltages are line-line RMS. The `DP::Ph3` and `EMT::Ph3` states are phase-peak
envelopes instead, so seeding one from the other needs the factor `RMS3PH_TO_PEAK1PH`, which is
$\sqrt{2/3}$. Omitting it initializes a state $\sqrt{3/2}$ too small and produces a startup
transient after an otherwise correct power flow initialization, which reads as a model defect and is
not one. `SP::Ph3` keeps the line-line RMS convention in every phase, so each phase entry carries
the whole-transformer quantity rather than a third of it.

The sources are not uniform either. `EMT::Ph3::VoltageSource` is the only source class that scales
its own reference, so it takes line-line RMS directly, while every other source takes the working
unit of its own domain. Driving a `DP::Ph3` circuit with a raw line-line value therefore places the
operating point $\sqrt{3/2}$ above what the initialization assumed, and the model rings.

The power formula follows from the same split. In `DP::Ph3` and `EMT::Ph3`, where the quantities are
phase-peak,

```math
P = \tfrac{3}{2} \, \mathrm{Re}\{ V \overline{I} \} ,
```

while in `DP::Ph1`, `SP::Ph1` and `SP::Ph3` it is $\mathrm{Re}\{ V \overline{I} \}$ from a single
entry. Applying the second form to a phase-peak domain gives exactly two thirds of the right answer.

## Verification

`dpsim/examples/cxx/Circuits/DP_Ph1_Transformer_OpenShortCircuit.cpp` runs three units through the
open-circuit and short-circuit tests in all five domains. Every unit recovers its nameplate: the
no-load current and loss from the open-circuit test, and the short-circuit voltage $u_k$ from the
short-circuit test, each to within the discretization error of the step used.

One reading of the open-circuit test needs care in EMT. The magnetizing loop has no resistive path
that damps a DC component, so an offset left by energization decays with a time constant of the
order of $L_m$ over the winding resistance, which is minutes rather than cycles. An RMS taken over a
window at one second still contains it. Extracting the fundamental with a one-cycle Fourier
component rejects it and returns the nameplate value; the phasor domains do not show the effect at
all, because they do not represent DC.
