---
title: "Ideal Transformer Model"
linkTitle: "Ideal Transformer Model"
date: 2026-06-26
author: Andres Acosta <andres.acosta@eonerc.rwth-aachen.de>
---

The Ideal Transformer Model (ITM) is a signal component that splits a circuit into two subcircuits, using a common node as a Point of Common Coupling (PCC), in such a way that a copy of this node is found in the two subcircuits, as shown in Fig. 1, where the copies of the node are denoted as $n$ and $m$. Moreover, the circuits are coupled using a controlled voltage source and a controlled current source, which exchange their interface  currents and voltages, respectively, namely the interface signals. This exchange takes place using a ring buffer, on top of which a second ring buffer has been implemented to emulate a co-simualtion using a macro-step, which means that the exchange of interface signals can be made at an interval larger than the simulation's step size. This second ring buffer is used to implement Zero- and First-Order hold extrapolation methods, while the first ring buffer allows to linearly interpolate the value of the signal at the current time step, in case the delay between both subcircuits is not an integer multiple of the step size.

<center>
<figure margin=30%>
    <img src="./images/ITM.svg" width=100% alt="ITM">
    <figcaption>Fig. 1: Ideal Transformer Model Circuit diagram.
    </figcaption>
</figure>
</center>

To add an ITM, users must split the cirtuit and create the copies of the PCC node. An example of this process can be found in the Notebook `ITM.ipynb`.

To avoid connections of the controlled voltage source with a capacitor, or the controlled current source with an inductor, the resistors $R_{\mathrm{series}}$ and $R_{\mathrm{parallel}}$ are included.
