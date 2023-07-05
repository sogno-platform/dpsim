---
title: "Transformer_EMT_Domain"
linkTitle: "Transformer_EMT_Domain"
date: 2023-06-30
---

## 2-Winding Power Transformer
The transformer model is composed of an RL-segment and an ideal transformer.
The single line diagram is depicted in the figure below.

![RL_element](R-L_element.png)

An RL- segment in the time domain is described by

```math
\frac{\mathrm{d} i_L(t)}{\mathrm{d}t} = \frac{v_j(t) - v_k(t) - R \cdot i_L(t)}{L}
```

Transforming it into EMT domain, inductor is described as 

Diagram

Applying the trapezoidal method for inductor , it leads to a finite difference equation:
```math
i_L(t+1) = \frac{\Delta t}{2L} * v_L(t+1) + i_L(t) + \frac{\Delta t}{2L} * v_L(t)
```
with
```math
I_{L} = i_L(t) + \frac{\Delta t}{2L} * v_L(t) , \qquad R_{1} = \frac{2L}{\Delta t}
```

Applying this to the transformer model 

![Power_transformer](Transformer_EMT.png) 

Applying nodal analysis at e1

```math
-i_{0}(t) + \frac{e_{1}(t) - V_{1}'(t)}{R_{1}} + I_{L} = 0
```

Also, 
```math
e_{1}(t) = V_{0}(t) - R \cdot i_{0}(t)
```
Substituting e1 in the nodal analysis equation:
```math
i_{0}(t) = \frac{1}{1 + \frac{R}{R_{1}}} *(  \frac{V_{0}(t) - V_{1}'(t)}{R_{1}} + \frac {I_{L}}{1}  )
```

with
```math
i_{n1}(t) = - n * i_{n0}(t) ,\qquad V_{1}'(t) = n * V_{1}(t) , \qquad A = \frac{1}{1 + \frac{R}{R_{1}}}
```

So the admittance matrix becomes 

```math
\begin{pmatrix}
i_{n0} \cr
i_{n1} \cr
\end{pmatrix}
=
\begin{pmatrix}
  \frac{A}{R_{1}} & -n * \frac{A}{R_{1}}\cr
  -n * \frac{A}{R_{1}} & n^{2} * \frac{A}{R_{1}} \cr
\end{pmatrix} 
* 
\begin{pmatrix}
v_{n0} \cr
v_{n1} \cr
\end{pmatrix} + 
\begin{pmatrix}
I_{L} \cr
- n * I_{L} \cr
\end{pmatrix}
```

Extending to 3 phase with the following assumptions :
1. No mutual impedances between the phases  
2. R_phaseA = R_phaseB = R_phaseC = R 
3. L_phaseA = L_phaseB = L_phaseC = L 



```math
\begin{pmatrix}
i_{nA} \cr
i_{nB} \cr
i_{nC} \cr
i_{nA}' \cr
i_{nB}' \cr
i_{nC}' \cr
\end{pmatrix}
=
\begin{pmatrix}
  \frac{A}{R_{1}}_{PhaseA} &  0  & 0  &  -n * \frac{A}{R_{1}} & 0 & 0\cr
  0 &  \frac{A}{R_{1}}_{PhaseA}  & 0  &  0 & -n * \frac{A}{R_{1}} & 0\cr
  0 &  0  & \frac{A}{R_{1}}_{PhaseA}  &  0 & 0 & -n * \frac{A}{R_{1}}\cr
  -n * \frac{A}{R_{1}} &  0  &  0  & n^{2} * \frac{A}{R_{1}} & 0 & 0\cr
  0 &  -n * \frac{A}{R_{1}}  &  0  & 0 & n^{2} * \frac{A}{R_{1}} & 0\cr
  0 &  0  &  -n * \frac{A}{R_{1}}  & 0 & 0 & n^{2} * \frac{A}{R_{1}}\cr
\end{pmatrix} 
* 
\begin{pmatrix}
v_{A} \cr
v_{B} \cr
v_{C} \cr
v_{A}' \cr
v_{B}' \cr
v_{C}' \cr
\end{pmatrix} + 
\begin{pmatrix}
I_{La} \cr
I_{Lb} \cr
I_{Lc} \cr
- n * I_{La}  \cr
- n * I_{Lb} \cr
- n * I_{Lc} \cr
\end{pmatrix}
```
