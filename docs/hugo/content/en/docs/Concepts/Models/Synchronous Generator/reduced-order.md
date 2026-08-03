---
title: "Reduced Order Machine Models"
linkTitle: "Reduced Order"
date: 2026-07-31
description: >
  Third to sixth order machine equations and the voltage-behind-reactance form they are solved in.
weight: 1
---

The full dq0 machine keeps every rotor winding as a state. Reduced order models keep the rotor
flux linkages that matter on the timescale of interest and represent the rest algebraically, which
removes the fastest states and lets the machine be stepped at the same rate as the network.

This page derives the equations. Nothing here depends on how they are arranged in software.

## Per unit system and operational parameters

All quantities are in the reciprocal per unit system referred to the direct axis mutual
inductance [[Kundur1994](#Kundur1994)]. With rated apparent power $S_n$ and rated line to line
voltage $V_n$ as the stator base, the base peak phase voltage, base current and base impedance are

```math
V_{base} = \frac{\sqrt{2}}{\sqrt{3}} V_n, \qquad
I_{base} = \frac{S_n}{\frac{3}{2} V_{base}}, \qquad
Z_{base} = \frac{V_n}{S_n / V_n}.
```

The machine is described by operational parameters rather than by winding data: the synchronous
inductances $L_d$ and $L_q$, the transient inductances $L_d'$ and $L_q'$, the subtransient
inductances $L_d''$ and $L_q''$, the corresponding open circuit time constants $T_{d0}'$,
$T_{q0}'$, $T_{d0}''$, $T_{q0}''$, and the inertia constant $H$. A sixth parameter $T_{aa}$, the
armature to field coupling time constant, distinguishes the two sixth order variants.

## Which states each order retains

The state variables are the voltages behind the transient and subtransient reactances,
$E_d'$, $E_q'$, $E_d''$ and $E_q''$, together with the two mechanical states. They are defined
from the terminal quantities by

```math
E_d' = V_d - L_q' I_q, \qquad E_q' = V_q + L_d' I_d,
```

and analogously for the subtransient pair with $L_d''$ and $L_q''$.

| Order | Electrical states | Physical content |
| --- | --- | --- |
| 3 | $E_q'$ | Field winding only; the q axis rotor is neglected. |
| 4 | $E_d'$, $E_q'$ | Field winding and one q axis damper. |
| 5 | $E_q'$, $E_d''$, $E_q''$ | Adds both subtransient windings, no q axis transient state. |
| 6a | $E_d'$, $E_q'$, $E_d''$, $E_q''$ | Full transient and subtransient set, with $T_{aa} \neq 0$. |
| 6b | $E_d'$, $E_q'$, $E_d''$, $E_q''$ | Same states with $T_{aa} = 0$. |

Every order carries the same two mechanical states, so the third order model has five states in
total and the sixth order models have eight.

## Voltage behind reactance form

Written directly, the stator equations couple the machine currents to the network currents, and the
machine inductances appear in the axis frame while the network is solved in the phase frame. The
voltage behind reactance form removes that coupling: the machine is expressed as an internal
voltage in series with a reactance that is constant in the axis frame, so the only quantity that
changes between steps is the internal voltage.

The internal voltage is not a free variable. Applying the trapezoidal rule to the rotor flux
equations over one step $\Delta t$ gives it as a recursion in quantities already known at the start
of the step. For the transient states,

```math
E_{h,d} = A_d' \, I_q + B_d' \, E_d', \qquad
E_{h,q} = A_q' \, I_d + B_q' \, E_q' + D_q' \left( E_{f}[k-1] + E_{f}[k] \right),
```

where $E_f$ is the field voltage supplied by the excitation system. The coefficients follow from
the trapezoidal integration and depend only on the parameters and the step size:

```math
A_d' = \frac{\Delta t \, Z_d'}{2 T_{q0}' + \Delta t}, \qquad
B_d' = \frac{2 T_{q0}' - \Delta t}{2 T_{q0}' + \Delta t},
```

```math
A_q' = \frac{-\Delta t \, Z_q'}{2 T_{d0}' + \Delta t}, \qquad
B_q' = \frac{2 T_{d0}' - \Delta t}{2 T_{d0}' + \Delta t}, \qquad
D_q' = \frac{\Delta t \left( 1 - T_f \right)}{2 T_{d0}' + \Delta t}.
```

The reactance differences are $Z_q' = L_d - L_d' - Y_d$ and $Z_d' = L_q - L_q' - Y_q$. For the
orders without subtransient states $Y_d = Y_q = 0$ and $T_f = 0$. For the sixth order variant with
armature coupling,

```math
Y_d = \frac{T_{d0}''}{T_{d0}'} \frac{L_d''}{L_d'} \left( L_d - L_d' \right), \qquad
Y_q = \frac{T_{q0}''}{T_{q0}'} \frac{L_q''}{L_q'} \left( L_q - L_q' \right), \qquad
T_f = \frac{T_{aa}}{T_{d0}'}.
```

The subtransient states obey a recursion of the same shape with $T_{d0}''$ and $T_{q0}''$ in place
of the transient time constants, and with the transient history entering as a forcing term.

Two properties of this form matter. The coefficients are computed once for a fixed step size, since
they contain no state. And because $B_d'$ and $B_q'$ are the trapezoidal amplification factors
$(2T - \Delta t)/(2T + \Delta t)$, they lie strictly inside the unit interval for any positive time
constant, so the flux recursion is unconditionally stable regardless of step size.

## Mechanical equations

The rotor obeys the swing equation in per unit,

```math
2H \frac{d \omega_r}{dt} = T_m - T_e, \qquad \frac{d \theta}{dt} = \omega_{base} \, \omega_r,
```

with the electrical torque taken from the axis frame quantities as
$T_e = V_d I_d + V_q I_q$. The load angle follows from the initial operating point as the phase of
$V + j L_q I$, which is the standard construction of the q axis position from terminal conditions.

## Solution schemes

The recursion above evaluates the internal voltage from quantities at the previous step, so the
machine and the network are solved once per step in sequence. Two refinements exist for cases where
that single pass is not accurate enough.

The predictor corrector method takes the single pass result as a prediction, re-evaluates the flux
recursion using the corrected terminal quantities, and repeats until the change between successive
passes falls below a tolerance. It converges to the solution of the implicit trapezoidal step
rather than to its explicit approximation, at the cost of repeated network solutions.

The two stage predictor method splits the step differently: it advances the machine state on a
predicted terminal voltage, then applies a single correction derived from the network solution,
without iterating to convergence. It costs one extra network solve per step and removes most of the
one step delay error.

Both are schemes for solving the same equations. They do not change the model order or the retained
states.

## Validity

The reduced order models assume the stator transients are fast enough to be neglected, so the
stator is treated as algebraic. This is the assumption that makes the model valid for
electromechanical studies and invalid where stator dynamics matter, such as during the first cycles
of a close-in fault or for subsynchronous interaction. Neglecting the q axis rotor entirely, as the
third order model does, additionally removes damping that is physically present, so a third order
machine oscillates more than the same machine represented at fourth order.

## References

- <a name="Kundur1994"></a>[Kundur1994] P. Kundur, *Power System Stability and Control*. New York: McGraw-Hill, 1994. Source of the reciprocal per unit system and of the operational parameter definitions used throughout this page.
