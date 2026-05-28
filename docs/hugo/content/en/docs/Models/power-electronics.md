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
