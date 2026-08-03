---
title: "DP Ph3 Averaged Voltage Source Inverter"
linkTitle: "DP Ph3 Averaged Voltage Source Inverter"
weight: 3
description: >
  Three-Phase Averaged Voltage Source Inverter with State-Space Nodal Interface (Dynamic Phasor)
---

This model extends the single-phase grid-following averaged inverter to the three-phase dynamic-phasor (DP) domain.
Each phase of the LC filter is represented by an independent complex envelope, $V_{c,a/b/c}$ and $I_{f,a/b/c}$, in contrast to the single positive-sequence envelope of the single-phase model, whereas the controller retains a single positive-sequence $dq$ frame shared by the PLL, the power filter, and the outer and inner control loops.
As in the single-phase case, the control states are baseband quantities and remain real-valued; only the six per-phase filter envelopes are carrier-band quantities, and each carries the $-j\omega_n$ frequency shift introduced in [State-Space Nodal]({{< ref "state-space-nodal.md" >}}).

The terminal input is the PCC voltage envelope of the three phases,

```math
u = \begin{bmatrix} U_a & U_b & U_c \end{bmatrix}^\top ,
```

and the state vector concatenates the 6 complex per-phase envelopes ahead of the 8 real control states, keeping the carrier-band and baseband blocks separate,

```math
\mathbf{x} =
\big[\,
V_{c,a} \;\;
V_{c,b} \;\;
V_{c,c} \;\;
I_{f,a} \;\;
I_{f,b} \;\;
I_{f,c} \;\;
\psi \;\;
\phi_{\mathrm{PLL}} \;\;
P \;\;
Q \;\;
\phi_d \;\;
\phi_q \;\;
\gamma_d \;\;
\gamma_q
\,\big]^\top ,
```

where $\psi := \theta_{\mathrm{PLL}} - \omega_n t$ again denotes the deviation of the PLL angle from the nominal carrier phase, retained as a state to preserve relinearization accuracy. Each per-phase envelope contributes its real and imaginary parts to the packed real vector, yielding 20 real states in total, or 22 with the optional negative-sequence loop described below.

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

At each simulation step the nonlinear model is linearized about the current operating point into the affine state-space form, with the real and imaginary parts of the 6 complex per-phase envelopes and the 8 real control states packed into a single real 20-vector,

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

In this default configuration the controller operates in a single positive-sequence $dq$ frame, so only the positive-sequence component of an unbalanced terminal is regulated. The negative-sequence response is present in the per-phase filter envelopes but is not itself a control state, and the $2\omega_n$ ripple it would otherwise induce in the $dq$ frame is therefore not represented.

## Optional negative-sequence current control

A second, negative-sequence current-control loop can be added alongside the positive-sequence one, giving the dual-sequence structure of Yazdani and Iravani, chapter 8. The two configurations answer different questions: without the loop the model has the same 20 states and the same eigenvalue count as its `EMT::Ph3` counterpart, which is what a cross-domain comparison requires, while with it the model gains 2 states and can regulate an unbalanced terminal.

The negative-sequence quantities are obtained by projecting the same three envelopes onto the conjugate sequence set,

```math
\underline{V}_c^- = V_{c,a} + a^2 V_{c,b} + a\, V_{c,c},
\qquad
\underline{I}_{rc}^- = \frac{\underline{V}_c^- - \underline{U}^-}{R_c}.
```

A negative-sequence component rotates backwards relative to the PLL frame, so in envelope terms its $dq$ image follows from conjugating the projected phasor and rotating by $+\psi$ rather than $-\psi$,

```math
I_{rc,dq}^- = \tfrac{1}{2}\sqrt{\tfrac{2}{3}}\, e^{\,j\psi}\, \overline{\underline{I}_{rc}^-} .
```

{{% alert title="Both sequence images are baseband" color="info" %}}
This is what keeps the extension cheap. The negative-sequence loop costs only the two real integrator states $\gamma_{nd}$ and $\gamma_{nq}$, with no second carrier and no $2\omega_n$ term anywhere in the model.
{{% /alert %}}

The loop itself is the same PI structure as the positive-sequence inner loop,

```math
\dot{\gamma}_{nd} = i_{nd,\mathrm{ref}} - i_{rc,nd},
\qquad
\dot{\gamma}_{nq} = i_{nq,\mathrm{ref}} - i_{rc,nq},
```

```math
v_{nd,\mathrm{ref}}
=
K_{p,I}(i_{nd,\mathrm{ref}} - i_{rc,nd}) +
K_{i,I}\gamma_{nd},
\qquad
v_{nq,\mathrm{ref}}
=
K_{p,I}(i_{nq,\mathrm{ref}} - i_{rc,nq}) +
K_{i,I}\gamma_{nq},
```

reusing the inner-loop gains $K_{p,I}$ and $K_{i,I}$. Its output is redistributed to the per-phase bridge voltages through the sequence-orthogonal set, and adds to the positive-sequence command of the previous section,

```math
V_{\mathrm{ref},p}
=
\bar{a}_p \sqrt{\tfrac{2}{3}}\, V_{\mathrm{ref},dq}\, e^{j\psi}
+
a_p \sqrt{\tfrac{2}{3}}\, \overline{V_{\mathrm{ref},dq}^-}\, e^{j\psi},
\qquad
a_{a/b/c} = \{1,\; a,\; a^2\}.
```

The two references $i_{nd,\mathrm{ref}}$ and $i_{nq,\mathrm{ref}}$ default to zero, which makes the loop a negative-sequence suppressor. A non-zero pair commands a deliberate negative-sequence injection instead, as required by some unbalanced fault ride-through grid codes.

The state vector grows to 22 by appending the two integrators after the control block, so that the envelope and positive-sequence control indices are unaffected. Under a single-line-to-ground fault, enabling the loop suppresses the negative-sequence component of the injected current by about 40 percent while moving the positive-sequence component by less than 0.1 percent.

## References

- M. Mirz, S. Vogel, G. Reinke, and A. Monti, “DPsim: A dynamic phasor real-time simulator for power systems,” *SoftwareX*, vol. 10, art. 100253, 2019. <https://doi.org/10.1016/j.softx.2019.100253>
- A. Yazdani and R. Iravani, *Voltage-Sourced Converters in Power Systems: Modeling, Control, and Applications*. Hoboken, NJ: Wiley-IEEE Press, 2010. <https://ieeexplore.ieee.org/book/5237659>
- X. Gao, D. Zhou, A. Anvari-Moghaddam, and F. Blaabjerg, “Stability Analysis of Grid-Following and Grid-Forming Converters Based on State-Space Model,” in *Proc. 2022 International Power Electronics Conference (IPEC-Himeji 2022, ECCE Asia)*, pp. 422–428. <https://ieeexplore.ieee.org/document/9806927>

How this is arranged in code, together with the source and the runnable examples, is covered under
[DP Ph3 averaged VSI implementation]({{< ref "/docs/Developer Guide/Model Implementations/dp-ph3-averaged-vsi-implementation.md" >}}).
