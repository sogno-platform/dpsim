---
title: "State-Space Extraction"
linkTitle: "State-Space Extraction"
date: 2026-05-28
description: >
  Recovering a state-space model of the whole network from the solver.
weight: 6
---

A nodal simulation computes a trajectory: given a set of sources and initial conditions it
produces the node voltages step by step. It does not, by itself, say anything about the system's
modes, its damping, or how close it is to instability. Those questions are answered by the
state-space description, which the extraction recovers from the discretised network that the
simulation is already solving. The result is the discrete-time model of exactly what is being
simulated, including the effect of the discretisation itself, rather than of an idealised
continuous system that the simulation approximates.

## Where the Discrete Model Comes From

The starting point is the companion form described under
[nodal analysis]({{< ref "nodal-analysis.md" >}}). Each dynamic element has already been turned
into a conductance in parallel with a history current source, and its history term is a
difference equation in the element's own quantity: the current for an inductor, the voltage for a
capacitor. Those quantities are the states. The network solve then supplies the terminal voltages
that drive them.

This gives a system in two parts rather than one. The states advance using their own local
recurrence together with the network solution at the new instant, and the network solution at
that instant is itself driven by the states carried over from the previous one.

## Assembled Form

The extracted model is assembled in the form:

```math
\mathbf{x}[k+1]
=
\mathbf{A}_{d,\mathrm{local}} \mathbf{x}[k]
+
\mathbf{B}_{d,\mathrm{MNA}} \mathbf{x}_{\mathrm{MNA}}[k+1]
```

```math
\mathbf{Y} \mathbf{x}_{\mathrm{MNA}}[k+1]
=
\mathbf{C}_{d,\mathrm{MNA}} \mathbf{x}[k]
```

where:

- $\mathbf{x}$ is the extraction-state vector,
- $\mathbf{x}_{\mathrm{MNA}}$ is the full MNA unknown vector,
- $\mathbf{Y}$ is the active MNA system matrix,
- $\mathbf{A}_{d,\mathrm{local}}$ contains local component state-transition contributions,
- $\mathbf{B}_{d,\mathrm{MNA}}$ maps MNA unknowns to the state update,
- $\mathbf{C}_{d,\mathrm{MNA}}$ maps extraction states to MNA current injections.

Eliminating the MNA unknown vector gives the global discrete-time state matrix

```math
\mathbf{A}_{d}
=
\mathbf{A}_{d,\mathrm{local}}
+
\mathbf{B}_{d,\mathrm{MNA}}
\operatorname{solve}
\left(
\mathbf{Y},
\mathbf{C}_{d,\mathrm{MNA}}
\right)
```

The resulting matrix $\mathbf{A}_{d}$ describes the homogeneous discrete-time dynamics of the EMT MNA simulation model at the current operating point and system-matrix configuration.

The elimination is written as a solve rather than as an inverse for a reason. $\mathbf{Y}$ is
sparse and already factorised for the time stepping, so the term is obtained by one forward and
backward substitution per column of $\mathbf{C}_{d,\mathrm{MNA}}$, that is, one per extracted
state. Forming $\mathbf{Y}^{-1}$ explicitly would discard the sparsity and cost far more than the
extraction itself.

## Interpreting the Result

The eigenvalues of $\mathbf{A}_{d}$ are discrete-time modes, so they are read against the unit
circle rather than against the imaginary axis. A mode is stable when $|\lambda_d| < 1$, and the
closer it sits to the unit circle the more lightly damped it is. A corresponding continuous-time
eigenvalue follows from

```math
\lambda_c = \frac{\ln \lambda_d}{\Delta t},
```

whose real part gives the damping and whose imaginary part gives the oscillation frequency, up to
the ambiguity that any frequency above the Nyquist rate of the time step is indistinguishable
from one below it.

It is worth being clear about what this eigenvalue belongs to. It is a mode of the discretised
system, not of the underlying continuous one. The two differ by the distortion the integration
rule introduces, and for the trapezoidal rule that distortion grows with frequency, so the fastest
modes are the least faithful. Reducing the time step reduces the discrepancy; comparing extraction
results across two time steps is a practical way to see which modes are trustworthy.

## Validity

The extracted model is linear and time invariant, and it describes the system only for the
configuration it was taken from. Any event that changes $\mathbf{Y}$, a switch opening or closing
above all, produces a different $\mathbf{A}_{d}$, so a switching study means one extraction per
configuration rather than one for the simulation. The same holds for non-linear elements, whose
contributions are those at the operating point reached when the extraction was performed; moving
the operating point requires extracting again.

## References

- J. A. Hollman and J. R. Marti, *Step-by-step eigenvalue analysis with EMTP discrete-time solutions*, *IEEE Transactions on Power Systems*, 2010. <https://doi.org/10.1109/TPWRS.2009.2039810>
- Y. Han, H. Sun, B. Huang, S. Qin, M. Mu, and Y. Yu, “Discrete-Time State-Space Construction Method for SSO Analysis of Renewable Power Generation Integrated AC/DC Hybrid System,” *IEEE Transactions on Power Systems*, 2022. <https://doi.org/10.1109/TPWRS.2021.3115248>
