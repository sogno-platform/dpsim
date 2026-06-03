---
title: "State-Space Extraction"
linkTitle: "State-Space Extraction"
date: 2026-05-28
---

Discrete-time state-space model is extracted from the EMT MNA simulation model. The extracted model is assembled in the form:

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

# References

- J. A. Hollman and J. R. Marti, *Step-by-step eigenvalue analysis with EMTP discrete-time solutions*, *IEEE Transactions on Power Systems*, 2010. <https://doi.org/10.1109/TPWRS.2009.2039810>
- Y. Han, H. Sun, B. Huang, S. Qin, M. Mu, and Y. Yu, “Discrete-Time State-Space Construction Method for SSO Analysis of Renewable Power Generation Integrated AC/DC Hybrid System,” *IEEE Transactions on Power Systems*, 2022. <https://doi.org/10.1109/TPWRS.2021.3115248>
