---
title: "Eigenvalues"
linkTitle: "Eigenvalues"
date: 2024-03-20
---

In parallel to simulating a power system, DPsim allows to extract eigenvalues of the power system state-space model's state matrix for each simulation time step. Eigenvalue extraction can be enabled for a ``Simulation``.



## Equations
``discreteEigenvalues`` $\mathbf{z}$ are computed from discretized state matrix $A_{d}$, that in EMT domain is calculated from:

$$
\mathbf{A}_{d}=\mathbf{S}-\mathbf{G}_{b}\mathbf{A}_{bn}\mathbf{G}^{-1} \mathbf{A}_{nb}
$$

where $\mathbf{S}$ is a sign matrix, $\mathbf{G}_{b}$ is a discretization matrix, $\mathbf{G}$ is a Modified Nodal Analysis (MNA) power system conductance matrix,  $\mathbf{A}_{bn}$ and $\mathbf{A}_{nb}$ are <i>branch x node</i> and <i>node x branch</i> incidence matrices respectively. 

The MNA power system conductance matrix $\mathbf{G}$ is available from power system MNA model. To prepare the rest of the matrices, each power system component needs to be stamped into $\mathbf{A}_{bn}$ and $\mathbf{A}_{nb}$, while dynamic components also need to be stamped into $\mathbf{S}$ and $\mathbf{G}_{b}$ matrices.


``eigenvalues`` $\mathbf{\lambda}$ of the time-continuous state-space model matrix $A$ can then be calculated from ``discreteEigenvalues`` $\mathbf{z}$. Assuming the Trapezoidal rule of discretization in EMT domain, the equation is:

$$
\mathbf{\lambda}=\frac{2}{\Delta t} \frac{\mathbf{z} - 1}{\mathbf{z} + 1}
$$


## Implementation

The ``MNAEigenvalueExtractor`` class is a template class responsible for extracting eigenvalues.
The ``EigenvalueCompInterface`` is interface to be implemented by all components participating in eigenvalue extraction.  The ``EigenvalueDynamicCompInterface`` is interface to be implemented by all _dynamic_ components participating in eigenvalue extraction.  These interfaces provide the signatures of the functions that are used to stamp a component into the matrices $\mathbf{A}_{bn}$, $\mathbf{A}_{nb}$, $\mathbf{S}$ and $\mathbf{G}_{b}$.

The table below provides an overview of the components, that support eigenvalue extraction in EMT and in DP domains:

| Component |Is dynamic?| EMT domain, Ph1 | DP domain, Ph1 |
| -------- |-------- | -------- | -------- |
| ``Resistor``  |&mdash;| &#x2714;  | &#x2714;  |
| ``Inductor``  |&#x2714;| &#x2714;  | &#x2714;  |
| ``Capacitor``  |&#x2714;| &#x2714;  | &#x2714;  |
| ``Switch``  |&mdash;| &mdash;  | &#x2714;  |
| ``VoltageSource``  |&mdash;| &#x2714;  | &#x2714;  |
