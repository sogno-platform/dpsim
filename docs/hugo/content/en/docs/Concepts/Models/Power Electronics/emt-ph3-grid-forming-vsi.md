---
title: "EMT Ph3 Grid-Forming Inverter"
linkTitle: "EMT Ph3 Grid-Forming Inverter"
weight: 4
description: >
  Three-Phase Averaged Grid-Forming Inverter with State-Space Nodal Interface
---

This model represents a grid-forming averaged voltage source inverter in the EMT domain.
The control structure follows the state-space grid-forming converter of [Gao2022](https://ieeexplore.ieee.org/document/9806927) (VSG algorithm loop, voltage loop, current loop with active damping), whose grid-following counterpart in the same paper is the basis for the averaged inverter above; the inner voltage/current control and LC filter modeling follow [Yazdani2010](https://ieeexplore.ieee.org/book/5237659).
Like the grid-following inverter above it is a variable state-space nodal component stamped directly into the MNA system, but instead of a PLL that locks to the grid it carries its own virtual synchronous machine (VSG): the internal angle and voltage magnitude are states driven by active- and reactive-power balance, so the inverter imposes a voltage and can run islanded.
The model includes the VSG swing dynamics, a reactive-power/voltage excitation loop, filtered active/reactive power measurement, a cascaded voltage and current controller, a first-order converter/digital-delay approximation, and an LC filter with coupling resistance to the grid node.

The terminal input is the PCC voltage vector

```math
\mathbf{u} =
\begin{bmatrix}
u_a & u_b & u_c
\end{bmatrix}^\top ,
```

and the 17-element state vector is

```math
\mathbf{x} =
\begin{bmatrix}
P & Q & \omega & \theta & E &
\xi_{v,d} & \xi_{v,q} &
\xi_{i,d} & \xi_{i,q} &
v_{\mathrm{del},d} & v_{\mathrm{del},q} &
v_{c,a} & v_{c,b} & v_{c,c} &
i_{f,a} & i_{f,b} & i_{f,c}
\end{bmatrix}^\top ,
```

where $\theta$ is the VSG angle (there is no PLL), $E$ is the excitation-controlled voltage magnitude, $\xi_{v}$, $\xi_{i}$ are the voltage- and current-loop integrators, and $v_{\mathrm{del}}$ are the two delay states.

The model output is the interface current injected into the MNA system,

```math
\mathbf{y} =
\frac{\mathbf{u} - \mathbf{v}_c}{R_c}.
```

## Control structure

{{< mermaid >}}
graph LR
  U["PCC voltage u"] --> FILT["LC filter<br/>vc, if"]
  FILT --> MEAS["Power measurement<br/>p, q"]
  MEAS --> PF["Measurement filters<br/>P, Q"]
  PF -->|P| SWING["VSG swing<br/>omega, theta"]
  PF -->|Q| EXC["Excitation /<br/>Q-V droop -> E"]
  EXC --> VZ["Virtual impedance<br/>E - Zv*if"]
  VZ --> VCTRL["Voltage controller<br/>-> i_ref"]
  VCTRL --> ICTRL["Current controller<br/>-> v_conv"]
  ICTRL --> DELAY["Converter delay"]
  DELAY --> FILT
  SWING -->|theta| VCTRL
  FILT --> Y["Interface current y"]
{{< /mermaid >}}

The virtual synchronous machine sets the internal angle from the active-power balance and the internal magnitude from the reactive-power/voltage loop; the cascaded voltage and current controllers then track that internal reference through the LC filter. The dashed grid-connected extensions (virtual impedance, feed-forward scaling, Q-V droop) are described below.

## Model equations

The physical grid current, positive for injection into the grid, is

```math
\mathbf{i}_g = \frac{\mathbf{v}_c - \mathbf{u}}{R_c}.
```

All dq quantities use the VSG angle $\theta$ (amplitude-invariant Park transform $\mathbf{T}(\theta)$),

```math
\mathbf{v}_{c,dq} = \mathbf{T}(\theta)\mathbf{v}_c, \qquad
\mathbf{i}_{f,dq} = \mathbf{T}(\theta)\mathbf{i}_f, \qquad
\mathbf{i}_{g,dq} = \mathbf{T}(\theta)\mathbf{i}_g,
```

and the capacitor current is $\mathbf{i}_{\mathrm{cap},dq} = \mathbf{i}_{f,dq} - \mathbf{i}_{g,dq}$.
Because the Park transform is amplitude invariant, three-phase instantaneous power carries the factor $3/2$,

```math
p = \tfrac{3}{2}\,(v_{c,d} i_{g,d} + v_{c,q} i_{g,q}),
\qquad
q = \tfrac{3}{2}\,(v_{c,q} i_{g,d} - v_{c,d} i_{g,q}),
```

and the PCC voltage magnitude is $U_{\mathrm{pcc}} = \sqrt{v_{c,d}^2 + v_{c,q}^2}$.

The measurement filters are first-order lags,

```math
\dot{P} = \omega_c(p - P),
\qquad
\dot{Q} = \omega_c(q - Q).
```

The VSG swing equation sets the angle from the active-power balance,

```math
J\dot{\omega} = \frac{P_{\mathrm{ref}} - P}{\omega} - D(\omega - \omega_n),
\qquad
\dot{\theta} = \omega,
```

with virtual inertia $J$ and damping $D$.
The reactive-power/voltage excitation controller sets the internal magnitude,

```math
\dot{E} = K_q(Q_{\mathrm{ref}} - Q) + K_u(U_n - U_{\mathrm{pcc}}),
```

an integral law on the reactive error with a voltage-droop term.
The excitation defines the dq voltage reference; in the islanded model it is aligned with the d-axis,

```math
v_{d,\mathrm{ref}} = E, \qquad v_{q,\mathrm{ref}} = 0 .
```

The voltage controller integrates the voltage error and forms the current reference with the capacitor-current feed-forward and dq decoupling,

```math
\dot{\xi}_{v,d} = v_{d,\mathrm{ref}} - v_{c,d},
\qquad
\dot{\xi}_{v,q} = v_{q,\mathrm{ref}} - v_{c,q},
```

```math
i_{d,\mathrm{ref}} = i_{g,d} - \omega C_f v_{c,q}
+ K_{p,V}(v_{d,\mathrm{ref}} - v_{c,d}) + K_{i,V}\xi_{v,d},
```

```math
i_{q,\mathrm{ref}} = i_{g,q} + \omega C_f v_{c,d}
+ K_{p,V}(v_{q,\mathrm{ref}} - v_{c,q}) + K_{i,V}\xi_{v,q}.
```

The current controller integrates the current error and forms the converter voltage reference, with inductor decoupling and optional active damping on the capacitor current,

```math
\dot{\xi}_{i,d} = i_{d,\mathrm{ref}} - i_{f,d},
\qquad
\dot{\xi}_{i,q} = i_{q,\mathrm{ref}} - i_{f,q},
```

```math
v_{d,\mathrm{conv}} = v_{c,d} - \omega L_f i_{f,q}
+ K_{p,I}(i_{d,\mathrm{ref}} - i_{f,d}) + K_{i,I}\xi_{i,d} - K_{ad} i_{\mathrm{cap},d},
```

```math
v_{q,\mathrm{conv}} = v_{c,q} + \omega L_f i_{f,d}
+ K_{p,I}(i_{q,\mathrm{ref}} - i_{f,q}) + K_{i,I}\xi_{i,q} - K_{ad} i_{\mathrm{cap},q}.
```

A first-order lag approximates the converter/digital delay,

```math
\dot{v}_{\mathrm{del},d} = \omega_d(v_{d,\mathrm{conv}} - v_{\mathrm{del},d}),
\qquad
\dot{v}_{\mathrm{del},q} = \omega_d(v_{q,\mathrm{conv}} - v_{\mathrm{del},q}),
```

and its output, transformed back to abc as $\mathbf{v}_{\mathrm{inv}} = \mathbf{T}^{-1}(\theta)\,[v_{\mathrm{del},d}\ v_{\mathrm{del},q}]^\top$, drives the LC filter,

```math
\dot{\mathbf{v}}_c = \frac{1}{C_f}\left(\mathbf{i}_f + \frac{\mathbf{u} - \mathbf{v}_c}{R_c}\right),
\qquad
\dot{\mathbf{i}}_f = \frac{1}{L_f}\left(\mathbf{v}_{\mathrm{inv}} - \mathbf{v}_c - R_f\mathbf{i}_f\right).
```

## Grid-connected control extensions

The equations above describe the islanded inverter. Three opt-in extensions adapt it to a stiff grid; each defaults to the value that recovers the islanded model exactly, so the eigenstructure is unchanged unless a setter is called.

**Virtual output impedance.** A virtual impedance $Z_v = R_v + jX_v$ is subtracted from the excitation to form the voltage reference, using the filter current $\mathbf{i}_{f,dq}$,

```math
v_{d,\mathrm{ref}} + j v_{q,\mathrm{ref}}
= E - Z_v\,(i_{f,d} + j i_{f,q}),
```

i.e.

```math
v_{d,\mathrm{ref}} = E - (R_v i_{f,d} - X_v i_{f,q}),
\qquad
v_{q,\mathrm{ref}} = -(R_v i_{f,q} + X_v i_{f,d}).
```

$Z_v = 0$ recovers $v_{d,\mathrm{ref}} = E,\ v_{q,\mathrm{ref}} = 0$.
A finite $R_v$ adds a current-proportional term opposing motion, damping the power-synchronization loop on a stiff grid at the electrical timescale, an alternative to raising the mechanical damping $D$.
The drop is taken off the filter-current *state* $\mathbf{i}_f$ rather than the algebraically reconstructed grid current $\mathbf{i}_g = (\mathbf{v}_c-\mathbf{u})/R_c$; the latter would multiply the reference by a factor $\propto 1/R_c$, amplifying state and linearization error.

**Grid-current feed-forward scale.** A scalar $\kappa$ scales the grid-current feed-forward in the current reference,

```math
i_{d,\mathrm{ref}} = \kappa\, i_{g,d} - \omega C_f v_{c,q} + \dots,
\qquad
i_{q,\mathrm{ref}} = \kappa\, i_{g,q} + \omega C_f v_{c,d} + \dots,
```

with $\kappa = 1$ the default full feed-forward.

**Proportional reactive-power droop.** When a cutoff $\omega_q > 0$ is set, the integral excitation is replaced by a proportional Q-V droop,

```math
\dot{E} = \omega_q\big(E_{\mathrm{set}} + D_q(Q_{\mathrm{ref}} - Q) - E\big),
```

a first-order lag with a stable fixed point $E^* = E_{\mathrm{set}} + D_q(Q_{\mathrm{ref}} - Q)$ and pole at $-\omega_q$.
On a stiff grid the network fixes $U_{\mathrm{pcc}}$, so the reactive error $Q_{\mathrm{ref}} - Q$ cannot be driven to zero and the integral law $\dot E = K_q(Q_{\mathrm{ref}} - Q) + K_u(U_n - U_{\mathrm{pcc}})$ has no reachable equilibrium (reactive windup); the proportional droop always has one.
The setpoint $E_{\mathrm{set}}$ is captured at initialization as the operating magnitude, so $\dot{E} = 0$ when $Q = Q_{\mathrm{ref}}$ at $t = 0$.

## Linearization and stamping

The model is nonlinear (Park transforms with the moving angle $\theta$, the $1/\omega$ swing term, the power products). It is not linearized by hand; at each simulation step the state and output Jacobians are computed by central finite differences of the nonlinear functions $\mathbf{f}(\mathbf{x},\mathbf{u}) = \dot{\mathbf{x}}$ and $\mathbf{g}(\mathbf{x},\mathbf{u}) = \mathbf{y}$,

```math
\mathbf{A} = \frac{\partial \mathbf{f}}{\partial \mathbf{x}},\quad
\mathbf{B} = \frac{\partial \mathbf{f}}{\partial \mathbf{u}},\quad
\mathbf{C} = \frac{\partial \mathbf{g}}{\partial \mathbf{x}},\quad
\mathbf{D} = \frac{\partial \mathbf{g}}{\partial \mathbf{u}},
```

each column $j$ evaluated as $[\mathbf{f}(\mathbf{x}+\delta_j\mathbf{e}_j,\mathbf{u}) - \mathbf{f}(\mathbf{x}-\delta_j\mathbf{e}_j,\mathbf{u})]/(2\delta_j)$ with a mixed relative/absolute step $\delta_j$.
Because the grid-connected extensions above all enter through $\mathbf{f}$, they are captured in $\mathbf{A}$, $\mathbf{B}$, $\mathbf{C}$ and $\mathbf{D}$ automatically.
The affine offsets fix the model to the current operating point,

```math
\mathbf{E} = \mathbf{f}(\mathbf{x}_0,\mathbf{u}_0) - \mathbf{A}\mathbf{x}_0 - \mathbf{B}\mathbf{u}_0,
\qquad
\mathbf{F} = \mathbf{g}(\mathbf{x}_0,\mathbf{u}_0) - \mathbf{C}\mathbf{x}_0 - \mathbf{D}\mathbf{u}_0,
```

giving the affine state-space form

```math
\dot{\mathbf{x}} \approx \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u} + \mathbf{E},
\qquad
\mathbf{y} \approx \mathbf{C}\mathbf{x} + \mathbf{D}\mathbf{u} + \mathbf{F},
```

The dq/abc transformations and the nonlinear controls make this local model time varying, so it holds only in a neighbourhood of the operating point it was formed at.

How the linearization is carried out and stamped, together with the source and the runnable examples, is covered under
[EMT Ph3 grid-forming VSI implementation]({{< ref "/docs/Developer Guide/Model Implementations/emt-ph3-grid-forming-vsi-implementation.md" >}}).

## References

- <a name="Gao2022"></a>[Gao2022] X. Gao, D. Zhou, A. Anvari-Moghaddam, and F. Blaabjerg, "Stability Analysis of Grid-Following and Grid-Forming Converters Based on State-Space Model," in *2022 International Power Electronics Conference (IPEC-Himeji 2022 - ECCE Asia)*, 2022, pp. 422-428. Source of both the grid-following and grid-forming state-space control structures. Its eigenvalue analysis finds grid-following control better suited to a stiff grid and grid-forming control to a weak grid; the grid-connected extensions above (virtual impedance, Q-V droop) are what let the grid-forming model stay stable when connected to a stiff grid.
- <a name="Yazdani2010"></a>[Yazdani2010] A. Yazdani and R. Iravani, *Voltage-Sourced Converters in Power Systems: Modeling, Control, and Applications*. Hoboken, NJ: Wiley-IEEE Press, 2010. Basis for the inner voltage/current control and LC-filter modeling of both inverters.
