---
title: "Eigenvalues"
linkTitle: "Eigenvalues"
date: 2024-03-20
---

In parallel to simulating a power system, DPsim allows to extract eigenvalues of the power system state-space model's state matrix for each simulation time step. Eigenvalue extraction can be enabled for a ``Simulation``.



## Equations
``discreteEigenvalues`` $z$ are computed from discretized state matrix $A_{d}$, that in EMT domain is calculated from:

$$
A_{d}=S-G_{bb}A_{bn}G_{nn}^{-1} {A_{nb}}
$$

where  $S$ is a sign matrix, $G_{bb}$ is a discretization matrix, $G_{nn}$ is a Modified Nodal Analysis (MNA) power system conductance matrix,  $A_{bn}$ and $A_{nb}$ are <i>branch x node</i> and <i>node x branch</i> incidence matrices respectively. 

The matrix $G_{nn}$ is available from power system MNA model. To prepare the rest of the matrices, each power system component needs to be stamped into $A_{bn}$ and $A_{nb}$, while dynamic components also need to be stamped into $S$ and $G_{bb}$ matrices.


``eigenvalues`` $\lambda$ of the time-continuous state-space model matrix $A$ can then be recovered from ``discreteEigenvalues`` $z$. Assuming the Trapezoidal rule of discretization in EMT domain, the equation is:

$$
\lambda=\frac{2}{\Delta t} \frac{z - 1}{z + 1}
$$


## Implementation

The ``MNAEigenvalueExtractor`` class is a template class responsible for extracting eigenvalues.
The ``EigenvalueCompInterface`` and ``EigenvalueDynamicCompInterface`` are interfaces to be implemented by all components participating in eigenvalue extraction. These interfaces provide the signatures of the functions that are used to stamp a component into the matrices $A_{bn}$, $A_{nb}$, $S$ and $G_{bb}$. 
