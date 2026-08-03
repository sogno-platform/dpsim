---
title: "Developer Guide"
linkTitle: "Developer Guide"
weight: 3
menu:
  main:
    weight: 23
description: >
  How the simulator is built, for readers extending or debugging it.
aliases: ["/docs/overview/","/docs/tasks/"]
---

These pages describe how DPsim is built rather than what it computes. They are the background
for adding a component, changing a solver, or working out why a simulation behaves as it does.
For the physics and the numerical methods, see [concepts]({{< ref "/docs/Concepts" >}}).

Two ideas run through the codebase and are worth reading first. Attributes expose component
parameters and state to the logger, the Python bindings and the scheduler. Tasks carry declared
attribute dependencies, and those declarations are what the scheduler uses to order and
parallelise a timestep.
