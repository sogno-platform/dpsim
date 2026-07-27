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

The `DP::Ph3::AvVoltSourceInverterStateSpace` model extends the grid-following averaged inverter of `DP::Ph1` to the three-phase dynamic-phasor (DP) domain.
Each phase of the LC filter is represented by an independent complex envelope, $V_{c,a/b/c}$ and $I_{f,a/b/c}$, in contrast to the single positive-sequence envelope employed by `DP::Ph1`, whereas the controller retains a single positive-sequence $dq$ frame shared by the PLL, the power filter, and the outer and inner control loops.
As in `DP::Ph1`, the control states are baseband quantities and remain real-valued; only the six per-phase filter envelopes are carrier-band quantities, and each carries the $-j\omega_n$ frequency shift introduced in [State-Space Nodal]({{< ref "state-space-nodal.md" >}}).

The terminal input is the PCC voltage envelope of the three phases,

```math
u = \begin{bmatrix} U_a & U_b & U_c \end{bmatrix}^\top ,
```

and the state vector concatenates the 8 real control states ahead of the 6 complex per-phase envelopes,

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

where $\psi := \theta_{\mathrm{PLL}} - \omega_n t$ again denotes the deviation of the PLL angle from the nominal carrier phase, retained as a state to preserve relinearization accuracy. Each per-phase envelope contributes its real and imaginary parts to the packed real vector, yielding 20 real states in total.

The model output is the per-phase interface current injected into the MNA system,

```math
y_p = \frac{U_p - V_{c,p}}{R_c}, \qquad p \in \{a, b, c\}.
```

## Model equations

The main extension relative to `DP::Ph1` is the per-phase projection onto, and redistribution from, the single positive-sequence $dq$ control frame.
The three capacitor-voltage envelopes are projected onto a single positive-sequence phasor,

```math
\underline{V}_c = V_{c,a} + a\, V_{c,b} + a^2 V_{c,c},
\qquad a = e^{\,j 2\pi/3},
```

and the PCC input $\underline{U}$ is projected identically, so that the coupling-current envelope seen by the controller is $\underline{I}_{rc} = (\underline{V}_c - \underline{U})/R_c$, with positive current again denoting injection from the inverter into the grid.
The $dq$ quantities are obtained by rotating the projected envelopes by $\psi$,

```math
V_{c,dq} = \tfrac{1}{2}\sqrt{\tfrac{2}{3}}\, e^{-j\psi}\, \underline{V}_c,
\qquad
I_{rc,dq} = \tfrac{1}{2}\sqrt{\tfrac{2}{3}}\, e^{-j\psi}\, \underline{I}_{rc},
```

with $v_{c,d} = \operatorname{Re}\{V_{c,dq}\}$, $v_{c,q} = \operatorname{Im}\{V_{c,dq}\}$, and analogously for $i_{rc,d}$ and $i_{rc,q}$.
Taken together, the $1\times 3$ projection, the scalar $dq$ rotation, and the $3\times 1$ redistribution defined below constitute a rank-one $3\times 3$ Park mapping on the envelope triple, which reduces to the single-envelope relation of `DP::Ph1` under balanced operation.

The positive-sequence active and reactive power measurements used by the controller are

```math
p = v_{c,d} i_{rc,d} + v_{c,q} i_{rc,q},
\qquad
q = -v_{c,d} i_{rc,q} + v_{c,q} i_{rc,d},
```

with the projection scaling chosen so that $p$ and $q$ match the total three-phase active and reactive powers under balanced operation; under unbalanced operation they are the positive-sequence components seen by the single-frame controller.

The control chain from the PLL through the inner current loop is identical in form to that of `DP::Ph1` and operates on the single positive-sequence $dq$ pair. The PLL and power-filter dynamics read

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

The single $dq$ voltage reference $V_{\mathrm{ref},dq} = v_{d,\mathrm{ref}} + j v_{q,\mathrm{ref}}$ is redistributed to the per-phase bridge-voltage envelopes through the inverse projection,

```math
V_{\mathrm{ref},p} = \bar{a}_p \sqrt{\tfrac{2}{3}}\, V_{\mathrm{ref},dq}\, e^{j\psi},
\qquad
\bar{a}_{a/b/c} = \{1,\; a^2,\; a\},
```

so that all three phases are driven by the same positive-sequence command.

The LC-filter dynamics are decoupled per phase within the plant and carry the carrier shift of the envelope explicitly,

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

the phases being coupled only through the shared control chain, that is, through $V_{\mathrm{ref},p}$.

At each simulation step the nonlinear model is linearized about the current operating point into the affine state-space form, with the 8 real control states and the real and imaginary parts of the 6 complex per-phase envelopes packed into a single real 20-vector,

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

which is subsequently discretized and stamped into the DP MNA system.

Because the controller operates in a single positive-sequence $dq$ frame, only the positive-sequence component of an unbalanced terminal is regulated. The negative-sequence response is present in the per-phase filter envelopes but is not itself a control state, and the $2\omega_n$ ripple it would otherwise induce in the $dq$ frame is therefore not represented. A dual-sequence controller with a dedicated negative-sequence frame remains the subject of future work.

## References

- M. Mirz, S. Vogel, G. Reinke, and A. Monti, “DPsim: A dynamic phasor real-time simulator for power systems,” *SoftwareX*, vol. 10, art. 100253, 2019. <https://doi.org/10.1016/j.softx.2019.100253>
- A. Yazdani and R. Iravani, *Voltage-Sourced Converters in Power Systems: Modeling, Control, and Applications*. Hoboken, NJ: Wiley-IEEE Press, 2010. <https://ieeexplore.ieee.org/book/5237659>
- X. Gao, D. Zhou, A. Anvari-Moghaddam, and F. Blaabjerg, “Stability Analysis of Grid-Following and Grid-Forming Converters Based on State-Space Model,” in *Proc. 2022 International Power Electronics Conference (IPEC-Himeji 2022, ECCE Asia)*, pp. 422–428. <https://ieeexplore.ieee.org/document/9806927>

## Source code and examples

- Source code: [header](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/include/dpsim-models/DP/DP_Ph3_AvVoltSourceInverterStateSpace.h), [implementation](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/src/DP/DP_Ph3_AvVoltSourceInverterStateSpace.cpp)
- [C++ example](https://github.com/sogno-platform/dpsim/blob/master/dpsim/examples/cxx/Components/DP_Ph3_AvVoltSourceInverterStateSpace.cpp)
- [Python notebook](https://github.com/sogno-platform/dpsim/blob/master/examples/Notebooks/Components/DP_Ph3_AvVoltSourceInverterStateSpace.ipynb)

# Three-Phase Averaged Grid-Forming Inverter with State-Space Nodal Interface

The `EMT::Ph3::SSN_GFM` model represents a grid-forming averaged voltage source inverter in the EMT domain.
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
Because the grid-connected extensions above all enter through $\mathbf{f}$, they are captured in $\mathbf{A}$, $\mathbf{B}$, $\mathbf{C}$, $\mathbf{D}$ automatically, with no separate matrix code.
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

which is discretized and stamped into the EMT MNA system.
The dq/abc transformations and the nonlinear controls make the local model time varying, so the SSN equivalent is recomputed every simulation step.

## Source code and examples

- Source code: [header](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/include/dpsim-models/EMT/EMT_Ph3_SSN_GFM.h), [implementation](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/src/EMT/EMT_Ph3_SSN_GFM.cpp)
- [C++ example (IEEE 9-bus mixed SG/GFM/GFL)](https://github.com/sogno-platform/dpsim/blob/master/dpsim/examples/cxx/Circuits/EMT_Ph3_IEEE9_SSN_InverterMix.cpp)
- [Python notebook](https://github.com/sogno-platform/dpsim/blob/master/examples/Notebooks/Circuits/EMT_Ph3_IEEE9_SSN_InverterMix.ipynb)

## References

- <a name="Gao2022"></a>[Gao2022] X. Gao, D. Zhou, A. Anvari-Moghaddam, and F. Blaabjerg, "Stability Analysis of Grid-Following and Grid-Forming Converters Based on State-Space Model," in *2022 International Power Electronics Conference (IPEC-Himeji 2022 - ECCE Asia)*, 2022, pp. 422-428. Source of both the grid-following and grid-forming state-space control structures. Its eigenvalue analysis finds grid-following control better suited to a stiff grid and grid-forming control to a weak grid; the grid-connected extensions above (virtual impedance, Q-V droop) are what let the grid-forming model stay stable when connected to a stiff grid.
- <a name="Yazdani2010"></a>[Yazdani2010] A. Yazdani and R. Iravani, *Voltage-Sourced Converters in Power Systems: Modeling, Control, and Applications*. Hoboken, NJ: Wiley-IEEE Press, 2010. Basis for the inner voltage/current control and LC-filter modeling of both inverters.
