---
title: "Transformer"
linkTitle: "Transformer"
date: 2023-06-09
---

## 2-Winding Power Transformer
The power transformer in DPsim based on the RL-element which is depicted in the Fig. 1.

<p align="center">
  <img src="./RL-Element.svg" width="400" alt="RL Element">
  <figcaption>Fig.1 - RL Element</figcaption>
</p>

An RL-element in the time domain is described by

```math
v = L \frac{\mathrm{d} i(t)}{dt} + Ri(t)
```

```math
i(t) = i(t-\Delta t) + \frac{1}{L} \int_{t - \Delta t}^{t} v(\tau) - R i(\tau)\ \mathrm{d} \tau
```

Applying the trapezoidal rule leads to the following algebraic equation:

```math
i(k) = G_{eq} v(k) + i_{eq}(k)
```

with

```math
a = \frac{\Delta t R}{2L}, \quad b=\frac{\Delta t}{2L} \quad \textrm{and} \quad G_{eq} = \frac{b}{1+a}
```

and

```math
i_{eq}(k) = \frac{1-a}{1+a} i(k-1) + \frac{b}{1+a} v (k-1)
```

<p align="center">
  <img src="./Transformer.svg" width="550" alt="Power Transformer">
  <figcaption>Fig.2 - Equivalent circuit Power transformer</figcaption>
</p>

To apply this to the power transformer depicted in Fig. 2, the equations of the ideal transformer have to be considered: 

```math
v(t) = v_{n0}(t) - v_{n1}^{'}(t) = v_{n0}(t) - nv_{n1}(t) 
```

```math
i(t) = \frac{i_{sec}(t)}{n}
```

so that the discretized equation for the RL-element becomes: 

```math
i(k) = G_{eq} v_{n0}(k) - n G_{eq} v_{n1}(k) + i_{eq}(k)
```

with 

```math
i_{eq}(k) = \frac{1-a}{1+a} i(k-1) + \frac{b}{1+a} (v_{n0}(k-1) - n v_{n1}(k-1))
```

Extending to 3 phase with the following assumptions:
1. Symmetrical components 
2. No mutual impedances between the phases

the equation system of the power transformer becomes: 

```math
\begin{pmatrix}
i_{a} (k) \cr
i_{b} (k) \cr
i_{c} (k) \cr
i_{a, sec} (k) \cr
i_{b, sec} (k) \cr
i_{c, sec} (k) \cr
\end{pmatrix}
=
\begin{pmatrix}
  G_{eq} &  0  & 0  &  -n G_{eq} & 0 & 0 \cr
  0 &  G_{eq}  & 0  &  0 & -n G_{eq} & 0 \cr
  0 &  0  & G_{eq}  &  0 & 0 & -n G_{eq} \cr
  n G_{eq} &  0  &  0  & -n^{2} G_{eq} & 0 & 0 \cr
  0 &  n G_{eq}  &  0  & 0 & -n^{2} G_{eq} & 0 \cr
  0 &  0  &  n G_{eq}  & 0 & 0 & -n^{2} G_{eq} \cr
\end{pmatrix} 
* 
\begin{pmatrix}
v_{n0a} (k) \cr
v_{n0b} (k) \cr
v_{n0c} (k) \cr
v_{n1a} (k) \cr
v_{n1b} (k) \cr
v_{n1c} (k) \cr
\end{pmatrix} + 
\begin{pmatrix}
i_{eq,a} (k) \cr
i_{eq,b} (k) \cr
i_{eq,c} (k) \cr
n i_{eq,a} (k) \cr
n i_{eq,b} (k) \cr
n i_{eq,c} (k) \cr
\end{pmatrix}
```


## Extension with Dynamic Phasors
The RL-element in the  DP domain is described by

$$
  \frac{\mathrm{d} \langle i \rangle (t)}{\mathrm{d}t} + j\omega \cdot \langle i \rangle (t) = \frac{\langle v \rangle (t) - R \cdot \langle i \rangle (t)}{L}
$$

Applying the trapezoidal method leads to the following algebraic equations
```math
\langle i \rangle (k)= \frac{(1-b^2 - (a*R)^2 -j*2b) }{(1+a*R)^2+b^2} \cdot \langle i \rangle(k-1) + \frac{a+a^2*R - j *ab}{(1+aR)^2 + b^2} \cdot ( \langle v \rangle (k) + \langle v \rangle(k-1))
```
with
```math
a = \frac{\Delta t}{2L}, \qquad b = \frac{\Delta t \omega}{2}
```

Analog to the EMT model to apply this to the power transformer depicted in Fig. 2, the equations of the ideal transformer have to be considered: 

```math
\langle v \rangle (k) = \langle v_{n0} \rangle (k) - \langle v_{n1}^{'} \rangle (k), \qquad \langle v_{n1}^{'} \rangle (k) = n \langle v_{n1} \rangle (k)
```

```math
\langle i \rangle (k) = n  \langle i_{sec} \rangle (k) 
```
so that the discretized equation for the RL-element becomes:
```math
\langle i \rangle (k) = Y_{eq} (\langle v_{n0} \rangle (k) - n \langle v_{n1} \rangle (k)) + \langle i_{eq} \rangle (k)
```

where:

```math
Y_{eq} = \frac{a+a^{2}R - jab}{(1+aR)^{2} + b^{2}}
```
and

```math
\langle i_{eq} \rangle (k) = \frac{(1-b^{2} - (aR)^{2} -j2b) }{(1+aR)^{2}+b^{2}} \langle i \rangle(k-1) + \frac{a+a^{2}R - jab}{(1+aR)^{2} + b^{2}} \langle v \rangle(k-1)
```

Finally, the equation systems for the power transformer in the DP model is given by:

```math
\begin{pmatrix}
\langle i \rangle (k) \cr
\langle i_{sec} \rangle (k) \cr
\end{pmatrix}
=
\begin{pmatrix}
  Y_{eq} & -nY_{eq}\cr
  nY_{eq} & -n^{2} Y_{eq} \cr
\end{pmatrix} 
* 
\begin{pmatrix}
\langle v_{n0} \rangle (k) \cr
\langle v_{n1} \rangle (k) \cr
\end{pmatrix} + 
\begin{pmatrix}
\langle i_{eq} \rangle (k) \cr
\langle n i_{eq} \rangle (k) \cr
\end{pmatrix}
```
