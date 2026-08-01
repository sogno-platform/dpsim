---
title: "Synchronous Generator Regulators"
linkTitle: "Regulators"
weight: 10
description: >
  Excitation and speed control models attached to the synchronous machine.
aliases: ["/docs/models/synchronous-generator-regulators/"]
---

In DPSim, synchronous generator control systems are solved separately from the electric network. The outputs of the electric network (active and reactive power, node voltages, branch currents and rotor speed of synchronous generators) at time $k- \Delta t$ are used as the input of the controllers to calculate their states at time $k$. Because of the relatively slow response of the controllers, the error in the network solution due to the time delay $\Delta t$ introduced by this approach is negligible.

## References

- [1] “IEEE Recommended Practice for Excitation System Models for Power System Stability Studies,” in IEEE Std 421.5-2016 (Revision of IEEE Std 421.5-2005) , vol., no., pp.1-207, 26 Aug. 2016, doi: 10.1109/IEEESTD.2016.7553421.
- [2] F. Milano, “Power system modelling and scripting,” in Power System Modelling and Scripting. London: Springer-Verlag, 2010, ISBN: 978-3-642-13669-6. doi: 10.1007/978-3-642-13669-6.
- [3] F. Milano, A. Manjavacas, “Frequency Variations in Power Systems: Modeling, State Estimation, and Control”. ISBN: 978-1-119-55184-3.
- [4] F. Milano, “Power System Analysis Toolbox: Documentation for PSAT”, ISBN: 979-8573500560.
- [5] M. Eremia; M. Shahidehpour, “Handbook of Electrical Power System Dynamics: Modeling, Stability, and Control”, <https://ieeexplore.ieee.org/book/6480471>
- [6] A. Roehder, B. Fuchs, J. Massman, M. Quester, A. Schnettler, “Transmission system stability assessment within an integrated grid development process”.
