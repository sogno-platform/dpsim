---
title: "Attributes and Scheduling"
linkTitle: "Attributes and Scheduling"
weight: 2
description: >
  The attribute system, and how it decides the order everything runs in.
---

Attributes are the unit of state in DPsim, and they are not only a way to expose a value: the
scheduler builds the execution order from the dependencies that components declare over them. The
two subjects are one subject, which is why they sit together.

This is also what [logging]({{< ref "/docs/User Guide/logging.md" >}}) and the co-simulation
interfaces operate on, so a quantity that is not an attribute cannot be recorded or exchanged.
