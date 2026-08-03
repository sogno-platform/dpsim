---
title: "DP Ph1 Averaged Voltage Source Inverter"
linkTitle: "DP Ph1 Averaged Voltage Source Inverter"
weight: 2
description: >
  Single-Phase Averaged Voltage Source Inverter with State-Space Nodal Interface (Dynamic Phasor)
---

This model ports the same grid-following averaged inverter into the dynamic-phasor (DP) domain, as a single positive-sequence complex envelope rather than three abc waveforms.
The PLL, power filter, outer power control, and inner current control are baseband and stay real; only the LC filter's two states are genuine carrier-band envelopes and carry the $-j\omega_n$ shift described in [State-Space Nodal]({{< ref "state-space-nodal.md" >}}).

The terminal input is the PCC voltage envelope

```math
u = U ,
```

and the state vector is the mixed real/complex-envelope form

```math
\mathbf{x} =
\begin{bmatrix}
\psi &
\phi_{\mathrm{PLL}} &
P &
Q &
\phi_d &
\phi_q &
\gamma_d &
\gamma_q &
\operatorname{Re}\{V_c\} &
\operatorname{Im}\{V_c\} &
\operatorname{Re}\{I_f\} &
\operatorname{Im}\{I_f\}
\end{bmatrix}^\top ,
```

where $\psi := \theta_{\mathrm{PLL}} - \omega_n t$ is the PLL angle's deviation from the nominal carrier phase, tracked instead of the raw, unboundedly growing $\theta_{\mathrm{PLL}}$ for relinearization accuracy, and $V_c$, $I_f$ are complex envelopes replacing EMT's six abc filter states.

The model output is the interface current injected into the MNA system,

```math
y = \frac{U - V_c}{R_c}.
```

## Model equations

The controller uses the opposite current direction, i.e. positive current denotes inverter injection into the grid,

```math
I_{rc} = \frac{V_c - U}{R_c}.
```

Because the DP envelope already demodulates the carrier, the dq quantities are obtained by rotating the envelope by $\psi$ alone, not by the full absolute angle $\theta_{\mathrm{PLL}}$,

```math
V_{c,dq} = V_c\, e^{-j\psi}, \qquad I_{rc,dq} = I_{rc}\, e^{-j\psi},
```

with $v_{c,d} = \operatorname{Re}\{V_{c,dq}\}$, $v_{c,q} = \operatorname{Im}\{V_{c,dq}\}$, and likewise for $i_{rc,d}$, $i_{rc,q}$.

The instantaneous active and reactive powers are calculated as

```math
p = v_{c,d} i_{rc,d} + v_{c,q} i_{rc,q},
```

```math
q = -v_{c,d} i_{rc,q} + v_{c,q} i_{rc,d},
```

identical in form to EMT's; `DP::Ph1`'s own voltage/current scale already represents total power directly, with no three-phase multiplier.

The PLL and power-filter dynamics are

```math
\dot{\psi}
=
K_{p,\mathrm{PLL}} v_{c,q} +
K_{i,\mathrm{PLL}} \phi_{\mathrm{PLL}},
```

```math
\dot{\phi}_{\mathrm{PLL}} = v_{c,q},
```

```math
\dot{P} = \omega_c(p - P),
\qquad
\dot{Q} = \omega_c(q - Q).
```

The outer power-control integrators and current references are

```math
\dot{\phi}_d = P_{\mathrm{ref}} - P,
\qquad
\dot{\phi}_q = Q - Q_{\mathrm{ref}},
```

```math
i_{d,\mathrm{ref}}
=
K_{p,P}(P_{\mathrm{ref}} - P) + K_{i,P}\phi_d,
```

```math
i_{q,\mathrm{ref}}
=
K_{p,P}(Q - Q_{\mathrm{ref}}) + K_{i,P}\phi_q.
```

The inner current-control integrators and voltage references are

```math
\dot{\gamma}_d = i_{d,\mathrm{ref}} - i_{rc,d},
\qquad
\dot{\gamma}_q = i_{q,\mathrm{ref}} - i_{rc,q},
```

```math
v_{d,\mathrm{ref}}
=
K_{p,I}(i_{d,\mathrm{ref}} - i_{rc,d}) +
K_{i,I}\gamma_d,
```

```math
v_{q,\mathrm{ref}}
=
K_{p,I}(i_{q,\mathrm{ref}} - i_{rc,q}) +
K_{i,I}\gamma_q.
```

The reference voltage is transformed back to a complex envelope, rotating by $\psi$,

```math
V_{\mathrm{ref}} = (v_{d,\mathrm{ref}} + j v_{q,\mathrm{ref}})\, e^{j\psi}.
```

The LC filter dynamics carry the envelope's carrier shift explicitly,

```math
\dot{V}_c
=
\frac{1}{C_f} I_f
+
\frac{1}{C_f R_c}(U - V_c)
- j\omega_n V_c,
```

```math
\dot{I}_f
=
\frac{1}{L_f}
\left(
V_{\mathrm{ref}}
-
V_c
-
R_f I_f
\right)
- j\omega_n I_f.
```

At each simulation step, the nonlinear model is locally linearized into the affine state-space form, packing the 8 real states and the real/imaginary parts of the 2 complex states into one real 12-vector,

```math
\dot{\mathbf{x}}
\approx
\mathbf{A}\mathbf{x}
+
\mathbf{B}\mathbf{u}
+
\mathbf{E},
```

```math
\mathbf{y}
\approx
\mathbf{C}\mathbf{x}
+
\mathbf{D}\mathbf{u}
+
\mathbf{F},
```

which is then discretized and stamped into the network equations.

How this is arranged in code, together with the source and the runnable examples, is covered under
[DP Ph1 averaged VSI implementation]({{< ref "/docs/Developer Guide/Model Implementations/dp-ph1-averaged-vsi-implementation.md" >}}).
