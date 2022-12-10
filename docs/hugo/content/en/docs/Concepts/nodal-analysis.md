---
title: "Nodal Analysis"
linkTitle: "Nodal Analysis"
date: 2020-03-18
---

<script src="https://polyfill.io/v3/polyfill.min.js?features=es6"></script>
<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>

A circuit with $b$ branches has $2b$ unknowns since there are $b$ voltages and $b$ currents.
Hence, $2b$ linear independent equations are required to solve the circuit.
If the circuit has $n$ nodes and $b$ branches, it has

* Kirchoff's current law (KCL) equations
* Kirchoff's voltage law (KVL) equations
* Characteristic equations (Ohm's Law)

There are only $n-1$ KCLs since the nth equation is a linear combination of the remaining $n-1$.
At the same time, it can be demonstrated that if we can imagine a very high number of closed paths in the network, only $b-n+1$ are able to provide independent KVLs.
Finally there are $b$ characteristic equations, describing the behavior of the branch, making a total of $2b$ linear independent equations.

The nodal analysis method reduces the number of equations that need to be solved simultaneously.
$n-1$ voltage variables are defined and solved, writing $n-1$ KCL based equations.
A circuit can be solved using Nodal Analysis as follows

* Select a reference node (mathematical ground) and number the remaining $n-1$ nodes, that are the independent voltage variables
* Represent every branch current $i$ as a function of node voltage variables $v$ with the general expression $i = g(v)$
* Write $n-1$ KCL based equations in terms of node voltage variable.

The resulting equations can be written in matrix form and have to be solved for $v$.
$$\boldsymbol{Y} \boldsymbol{v} = \boldsymbol{i}$$
