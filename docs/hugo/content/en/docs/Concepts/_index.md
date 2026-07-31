---
title: "Concepts"
linkTitle: "Concepts"
weight: 4
description: >
  The theory behind the solvers and modelling approaches used in DPsim.
---

This section covers the general concepts implemented in DPsim and the physical models of the
power system components used in simulations.

Dynamic phasors and nodal analysis are the two pillars of the main solver, and are covered
first. The remaining pages describe the physical equations behind each model and how they are
transformed for dynamic phasor simulation and for the other supported domains.

DPsim also includes a load flow solver, used either on its own or to compute the initial state
of a network when that state is not part of the network data. Alongside the dynamic phasor
models there are electromagnetic transient models, which serve both as a simulation domain in
their own right and as the reference against which the dynamic phasor models are tested.
