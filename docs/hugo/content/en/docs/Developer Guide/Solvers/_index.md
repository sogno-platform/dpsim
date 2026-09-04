---
title: "Solvers"
linkTitle: "Solvers"
weight: 15
description: >
  The solvers below the nodal one, the linear backends, and state-space extraction.
---

The [MNA solver]({{< ref "/docs/Developer Guide/Solvers/mna-solver.md" >}}) is the default and the one almost every simulation
uses; a component's side of it is under
[interfacing with the MNA solver]({{< ref "/docs/Developer Guide/Writing a Model/mnainterface.md" >}}).
Changing its time step during a run is an extension of it rather than a solver of its own.
The remaining pages cover the powerflow solver, the alternatives to nodal analysis, the linear
algebra backends, and the state-space model that can be recovered from a running simulation.

The methods themselves are derived under
[alternative solution methods]({{< ref "/docs/Concepts/alternative-solvers.md" >}}).
