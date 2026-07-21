---
title: "Power Electronics"
linkTitle: "Power Electronics"
date: 2026-05-20
---

DPsim provides several averaged power-electronic inverter models for simulation using EMT, DP, and SP network modeling domains.

# Three-Phase Averaged Voltage Source Inverter with State-Space Nodal Interface

The `EMT::Ph3::AvVoltSourceInverterStateSpace` model represents a grid-following averaged voltage source inverter in the EMT domain.
It is implemented as a variable state-space nodal component and can therefore be directly stamped into the MNA system.
The model includes a PLL, filtered active/reactive power measurement, outer power control, inner current control, and an LC filter with coupling resistance to the grid node.

The terminal input is the PCC voltage vector

```math
\mathbf{u} =
\begin{bmatrix}
u_a & u_b & u_c
\end{bmatrix}^\top ,
```

and the state vector is

```math
\mathbf{x} =
\begin{bmatrix}
\theta_{\mathrm{PLL}} &
\phi_{\mathrm{PLL}} &
P &
Q &
\phi_d &
\phi_q &
\gamma_d &
\gamma_q &
v_{c,a} &
v_{c,b} &
v_{c,c} &
i_{f,a} &
i_{f,b} &
i_{f,c}
\end{bmatrix}^\top .
```

The model output is the interface current injected into the MNA system,

```math
\mathbf{y} =
\frac{\mathbf{u} - \mathbf{v}_c}{R_c}.
```

## Model equations

The controller uses the opposite current direction, i.e. positive current denotes inverter injection into the grid,

```math
\mathbf{i}_{rc} =
\frac{\mathbf{v}_c - \mathbf{u}}{R_c}.
```

The Park transformation with PLL angle $\theta_{\mathrm{PLL}}$ is used to obtain dq quantities,

```math
\begin{bmatrix}
v_{c,d} \\
v_{c,q}
\end{bmatrix}
=
\mathbf{T}(\theta_{\mathrm{PLL}})\mathbf{v}_c,
\qquad
\begin{bmatrix}
i_{rc,d} \\
i_{rc,q}
\end{bmatrix}
=
\mathbf{T}(\theta_{\mathrm{PLL}})\mathbf{i}_{rc}.
```

The instantaneous active and reactive powers are calculated as

```math
p = v_{c,d} i_{rc,d} + v_{c,q} i_{rc,q},
```

```math
q = -v_{c,d} i_{rc,q} + v_{c,q} i_{rc,d}.
```

The PLL and power-filter dynamics are

```math
\dot{\theta}_{\mathrm{PLL}}
=
\omega_n + K_{p,\mathrm{PLL}} v_{c,q} +
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

The reference voltage is transformed back to abc coordinates,

```math
\mathbf{v}_{\mathrm{ref}}
=
\mathbf{T}^{-1}(\theta_{\mathrm{PLL}})
\begin{bmatrix}
v_{d,\mathrm{ref}} \\
v_{q,\mathrm{ref}}
\end{bmatrix}.
```

The LC filter dynamics are

```math
\dot{\mathbf{v}}_c
=
\frac{1}{C_f}\mathbf{i}_f
+
\frac{1}{C_f R_c}(\mathbf{u} - \mathbf{v}_c),
```

```math
\dot{\mathbf{i}}_f
=
\frac{1}{L_f}
\left(
\mathbf{v}_{\mathrm{ref}}
-
\mathbf{v}_c
-
R_f \mathbf{i}_f
\right).
```

At each simulation step, the nonlinear model is locally linearized into the affine state-space form

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

which is then discretized and stamped into the EMT MNA system.

## Source code and examples

- Source code: [header](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/include/dpsim-models/EMT/EMT_Ph3_AvVoltSourceInverterStateSpace.h), [implementation](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/src/EMT/EMT_Ph3_AvVoltSourceInverterStateSpace.cpp)
- [C++ example](https://github.com/sogno-platform/dpsim/blob/master/dpsim/examples/cxx/Components/EMT_Ph3_AvVoltSourceInverterStateSpace.cpp)
- [Python notebook](https://github.com/sogno-platform/dpsim/blob/master/examples/Notebooks/Components/EMT_Ph3_AvVoltSourceInverterStateSpace.ipynb)

# Single-Phase Averaged Voltage Source Inverter with State-Space Nodal Interface (Dynamic Phasor)

The `DP::Ph1::AvVoltSourceInverterStateSpace` model ports the same grid-following averaged inverter into the dynamic-phasor (DP) domain, as a single positive-sequence complex envelope rather than three abc waveforms.
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

which is then discretized and stamped into the DP MNA system.

## Source code and examples

- Source code: [header](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/include/dpsim-models/DP/DP_Ph1_AvVoltSourceInverterStateSpace.h), [implementation](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/src/DP/DP_Ph1_AvVoltSourceInverterStateSpace.cpp)
- [C++ example](https://github.com/sogno-platform/dpsim/blob/master/dpsim/examples/cxx/Components/DP_Ph1_AvVoltSourceInverterStateSpace.cpp)
- [Python notebook](https://github.com/sogno-platform/dpsim/blob/master/examples/Notebooks/Components/DP_Ph1_AvVoltSourceInverterStateSpace.ipynb)

# Three-Phase Averaged Voltage Source Inverter with State-Space Nodal Interface (Dynamic Phasor)

The `DP::Ph3::AvVoltSourceInverterStateSpace` model ports the same grid-following averaged inverter into the three-phase dynamic-phasor (DP) domain.
It carries an independent complex envelope per phase for the LC filter, $V_{c,a/b/c}$ and $I_{f,a/b/c}$, instead of the single positive-sequence envelope used by `DP::Ph1`, while the controller keeps a single positive-sequence dq frame shared by the PLL, power filter, and outer/inner control.
As in `DP::Ph1`, the control states are baseband and stay real; only the six per-phase filter envelopes are genuine carrier-band quantities and each carries the $-j\omega_n$ shift described in [State-Space Nodal]({{< ref "state-space-nodal.md" >}}).

The terminal input is the PCC voltage envelope of the three phases,

```math
u = \begin{bmatrix} U_a & U_b & U_c \end{bmatrix}^\top ,
```

and the state vector packs the 8 real control states ahead of the 6 complex per-phase envelopes,

```math
\mathbf{x} =
\big[\,
\psi \;\;
\phi_{\mathrm{PLL}} \;\;
P \;\;
Q \;\;
\phi_d \;\;
\phi_q \;\;
\gamma_d \;\;
\gamma_q \;\;
V_{c,a} \;\;
V_{c,b} \;\;
V_{c,c} \;\;
I_{f,a} \;\;
I_{f,b} \;\;
I_{f,c}
\,\big]^\top ,
```

where $\psi := \theta_{\mathrm{PLL}} - \omega_n t$ is again the PLL angle's deviation from the nominal carrier phase, tracked for relinearization accuracy, and each per-phase envelope contributes its real and imaginary parts to the packed real vector, giving 20 real states in total.

The model output is the per-phase interface current injected into the MNA system,

```math
y_p = \frac{U_p - V_{c,p}}{R_c}, \qquad p \in \{a, b, c\}.
```

## Model equations

The genuinely new algebra relative to `DP::Ph1` is the per-phase projection onto, and redistribution from, the single positive-sequence dq control frame.
The three capacitor-voltage envelopes are collapsed to one positive-sequence phasor,

```math
\underline{V}_c = V_{c,a} + a\, V_{c,b} + a^2 V_{c,c},
\qquad a = e^{\,j 2\pi/3},
```

and the PCC input $\underline{U}$ is projected the same way, so the controller's coupling current envelope is $\underline{I}_{rc} = (\underline{V}_c - \underline{U})/R_c$, again with positive current denoting inverter injection into the grid.
The dq quantities follow by rotating the projected envelopes by $\psi$ alone,

```math
V_{c,dq} = \tfrac{1}{2}\sqrt{\tfrac{2}{3}}\, e^{-j\psi}\, \underline{V}_c,
\qquad
I_{rc,dq} = \tfrac{1}{2}\sqrt{\tfrac{2}{3}}\, e^{-j\psi}\, \underline{I}_{rc},
```

with $v_{c,d} = \operatorname{Re}\{V_{c,dq}\}$, $v_{c,q} = \operatorname{Im}\{V_{c,dq}\}$, and likewise for $i_{rc,d}$, $i_{rc,q}$.
Together, the $1\times 3$ projection, the scalar dq rotation, and the $3\times 1$ redistribution below form a rank-one $3\times 3$ Park map on the envelope triple; under balanced operation it reduces to the `DP::Ph1` single-envelope relation.

The instantaneous active and reactive powers are

```math
p = v_{c,d} i_{rc,d} + v_{c,q} i_{rc,q},
\qquad
q = -v_{c,d} i_{rc,q} + v_{c,q} i_{rc,d},
```

with the projection scaling chosen so that $p$ and $q$ are the total three-phase active and reactive powers seen by the single-frame controller.

Everything from the PLL through the inner current control is identical in form to `DP::Ph1`, operating on the single positive-sequence dq pair. The PLL and power-filter dynamics are

```math
\dot{\psi}
=
K_{p,\mathrm{PLL}} v_{c,q} +
K_{i,\mathrm{PLL}} \phi_{\mathrm{PLL}},
\qquad
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
\qquad
i_{q,\mathrm{ref}}
=
K_{p,P}(Q - Q_{\mathrm{ref}}) + K_{i,P}\phi_q,
```

and the inner current-control integrators and voltage references are

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
\qquad
v_{q,\mathrm{ref}}
=
K_{p,I}(i_{q,\mathrm{ref}} - i_{rc,q}) +
K_{i,I}\gamma_q.
```

The single dq voltage reference $V_{\mathrm{ref},dq} = v_{d,\mathrm{ref}} + j v_{q,\mathrm{ref}}$ is redistributed back to a per-phase bridge-voltage envelope by the inverse projection,

```math
V_{\mathrm{ref},p} = \bar{a}_p \sqrt{\tfrac{2}{3}}\, V_{\mathrm{ref},dq}\, e^{j\psi},
\qquad
\bar{a}_{a/b/c} = \{1,\; a^2,\; a\},
```

so all three phases are driven from the same positive-sequence command.

The LC filter dynamics are decoupled per phase in the plant and carry the envelope's carrier shift explicitly,

```math
\dot{V}_{c,p}
=
\frac{1}{C_f} I_{f,p}
+
\frac{1}{C_f R_c}(U_p - V_{c,p})
- j\omega_n V_{c,p},
```

```math
\dot{I}_{f,p}
=
\frac{1}{L_f}
\left(
V_{\mathrm{ref},p}
-
V_{c,p}
-
R_f I_{f,p}
\right)
- j\omega_n I_{f,p},
```

with the phases coupled only through the shared control chain, via $V_{\mathrm{ref},p}$.

At each simulation step, the nonlinear model is locally linearized into the affine state-space form, packing the 8 real states and the real/imaginary parts of the 6 complex per-phase envelopes into one real 20-vector,

```math
\dot{\mathbf{x}}
\approx
\mathbf{A}\mathbf{x}
+
\mathbf{B}\mathbf{u}
+
\mathbf{E},
\qquad
\mathbf{y}
\approx
\mathbf{C}\mathbf{x}
+
\mathbf{D}\mathbf{u}
+
\mathbf{F},
```

which is then discretized and stamped into the DP MNA system.

Because the controller uses a single positive-sequence dq frame, only the positive-sequence component of an unbalanced terminal is regulated; the negative-sequence response appears in the per-phase filter envelopes but is not itself a control state, and the associated $2\omega_n$ dq ripple it would produce is not represented. A dual-sequence controller with a dedicated negative-sequence frame is future work.

## Source code and examples

- Source code: [header](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/include/dpsim-models/DP/DP_Ph3_AvVoltSourceInverterStateSpace.h), [implementation](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/src/DP/DP_Ph3_AvVoltSourceInverterStateSpace.cpp)
- [C++ example](https://github.com/sogno-platform/dpsim/blob/master/dpsim/examples/cxx/Components/DP_Ph3_AvVoltSourceInverterStateSpace.cpp)
- [Python notebook](https://github.com/sogno-platform/dpsim/blob/master/examples/Notebooks/Components/DP_Ph3_AvVoltSourceInverterStateSpace.ipynb)
