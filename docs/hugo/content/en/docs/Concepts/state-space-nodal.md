---
title: "State-Space Nodal"
linkTitle: "State-Space Nodal"
date: 2026-06-23
---

The state-space nodal (SSN) method represents a component by its own continuous
state-space model and couples it to the network through the nodal admittance
matrix. A component is described by

```math
\frac{d\mathbf{x}}{dt} = \mathbf{A}\mathbf{x} + \mathbf{B}\mathbf{u},
\qquad
\mathbf{y} = \mathbf{C}\mathbf{x} + \mathbf{D}\mathbf{u},
```

where $\mathbf{x}$ is the internal state, and the input $\mathbf{u}$ and output
$\mathbf{y}$ are the terminal quantities exchanged with the network (a voltage and
the corresponding current). Trapezoidal discretisation of $(\mathbf{A}, \mathbf{B})$
yields a discrete model $(\mathbf{A}_d, \mathbf{B}_d)$ and a Norton equivalent: a
constant conductance $\mathbf{W}$ stamped into the system matrix plus a history
current source recomputed each step from the previous state and input. Because the
component is solved simultaneously with the network in the same nodal system, SSN
is numerically robust without the parasitic snubbers that delayed
current-injection schemes require.

This builds directly on [Nodal Analysis]({{< ref "nodal-analysis.md" >}}) and is
the companion of
[State-Space Extraction]({{< ref "state-space-extraction-theory.md" >}}), which
recovers a state-space model from an MNA simulation rather than starting from one.

# Shift to the Dynamic-Phasor Envelope

In the [Dynamic Phasors]({{< ref "dyn-phasors.md" >}}) (shifted-frequency)
domain a physical quantity is written as a slowly varying complex envelope on a
carrier $\omega_s$,

```math
x(t) = \operatorname{Re}\{\, X(t)\, e^{j \omega_s t} \,\},
```

so that differentiation maps to
$\frac{dx}{dt} \rightarrow \frac{dX}{dt} + j \omega_s X$. The physical
state-space model therefore becomes, for the envelope,

```math
\frac{d\mathbf{X}}{dt} = (\mathbf{A} - j \omega_s \mathbf{I})\,\mathbf{X}
+ \mathbf{B}\,\mathbf{U}.
```

DPsim does not solve the dynamic-phasor system as a complex matrix: every
complex admittance $g = g_r + j g_i$ is stamped as a real $2 \times 2$ block
$\left[\begin{smallmatrix} g_r & -g_i \\ g_i & g_r \end{smallmatrix}\right]$, with the real
and imaginary node parts living in separate halves of a real-valued system. The
DP-SSN model matches this by carrying the envelope in real/imaginary-split form
$[\mathbf{X}_r; \mathbf{X}_i]$ and writing the carrier shift explicitly as a real
augmented model,

```math
\mathbf{A}_\text{aug} =
\begin{bmatrix} \mathbf{A} & \omega_s \mathbf{I} \\ -\omega_s \mathbf{I} & \mathbf{A} \end{bmatrix},
\qquad
\mathbf{B}_\text{aug} = \begin{bmatrix} \mathbf{B} & \mathbf{0} \\ \mathbf{0} & \mathbf{B} \end{bmatrix}.
```

The same real trapezoidal discretisation used by the EMT SSN models then applies
unchanged, the Norton conductance comes out as a real block that stamps directly
into the augmented network, and the interface reads and writes the real and
imaginary node halves directly with no complex assembly. The $\pm \omega_s$
coupling block is algebraically exact; placing a rotating-frame term inside the
real state matrix follows the established SSN practice for rotating-machine
models.

# Components

The single-phase dynamic-phasor SSN models are:

* `Full_Serial_RLC`, a series resistor-inductor-capacitor one-port with a
  hand-derived state-space model, used as the reference component.
* `GenericTwoTerminalVTypeSSN` and `GenericTwoTerminalITypeSSN`, which accept a
  user-supplied $(\mathbf{A}, \mathbf{B}, \mathbf{C}, \mathbf{D})$ and build the
  V-type (voltage input, current output) or I-type (current input, voltage
  output) stamping accordingly.

All three reproduce the classical dynamic-phasor stamping of the same circuit,
and the reconstructed time-domain waveform matches the EMT and EMT-SSN results
within discretisation error.

# Three-Phase Components

The same real-augmented model extends per phase to `DP::Ph3`. The
$3 \times 3$ $\mathbf{A}$, $\mathbf{B}$, $\mathbf{C}$, $\mathbf{D}$ matrices
are general, so off-diagonal entries can couple the phases together:

* `Full_Serial_RLC`, the three-phase series RLC one-port.
* `GenericTwoTerminalVTypeSSN` and `GenericTwoTerminalITypeSSN`, the
  three-phase generic V-type and I-type components.

As in the single-phase case, all three reproduce the classical three-phase
dynamic-phasor stamping exactly, and the reconstructed time-domain waveform
matches the EMT and EMT-SSN results within discretisation error once
corrected for the RMS-to-peak scaling that `EMT::Ph3` sources apply and
`DP::Ph3` sources do not, since the DP envelope already carries the complex
amplitude directly. The notebooks below only exercise the symmetrical,
diagonal case; coupling between phases is not covered by existing tests.

# Variable Components

The components above are fixed-parameter LTI systems: $(\mathbf{A}, \mathbf{B},
\mathbf{C}, \mathbf{D})$ are built once from the component's parameters and never
change. Some SSN components instead depend on the operating point and are
relinearized and re-stamped every step (via `MNAVariableCompInterface`), the same
simultaneous nodal solve but around a Jacobian frozen at the previous step's
converged state rather than a constant matrix. `DP::Ph1::AvVoltSourceInverterStateSpace`,
an averaged grid-following inverter, is one such component; see
[Power Electronics]({{< ref "power-electronics.md" >}}) for its state vector and
model equations.

# Validation and Examples

Two notebooks accompany the single-phase models, both on a single-carrier
series RLC one-port. `examples/Notebooks/Circuits/DP_generalizedSSN_RLC.ipynb`
validates the DP-SSN models against the classical dynamic-phasor stamping and
the EMT and EMT-SSN waveforms. `examples/Notebooks/Circuits/DP_SSN_RLC_accuracy.ipynb`
studies the time-step and frequency-dependent accuracy against a small-step
EMT reference.

The three-phase analogues,
`examples/Notebooks/Circuits/DP_Ph3_generalizedSSN_RLC.ipynb` and
`examples/Notebooks/Circuits/DP_Ph3_SSN_RLC_accuracy.ipynb`, repeat both
studies on `DP::Ph3` circuits, including a current-driven network with generic
V-type and I-type components and a three-phase fault transient.

# Further Reading

* C. Dufour, J. Mahseredjian, and J. Bélanger, *A Combined State-Space Nodal Method for the Simulation of Power System Transients*, *IEEE Transactions on Power Delivery*, vol. 26, no. 2, pp. 928–935, 2011. <https://doi.org/10.1109/TPWRD.2010.2090364>
* C. Dufour and D. S. Nasrallah, *State-space-nodal rotating machine models with improved numerical stability*, *IECON 2016 – 42nd Annual Conference of the IEEE Industrial Electronics Society*, 2016. <https://doi.org/10.1109/IECON.2016.7793690>
* A. A. Kida, A. C. S. Lima, F. A. Moreira, J. R. Martí, and J. Tarazona, *Inaccuracies due to the frequency warping in simulation of electrical systems using combined state–space nodal analysis*, *Electric Power Systems Research*, vol. 223, art. 109657, 2023. <https://doi.org/10.1016/j.epsr.2023.109657>
