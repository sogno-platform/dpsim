---
title: "Transformer"
linkTitle: "Transformer"
date: 2021-07-22
markup: pandoc
---

## 2-Winding Transformer
The transformer model is composed of an RL-segment and an ideal transformer.
The single line diagram is depicted in the figure below.

![Transformer](electrical_transformer.svg)

If node reduction is not applied, two virtual nodes are created to stamp this model into the system matrix.

Furthermore, the ideal transformer has an additional equation, which requires an extension of the system matrix.
The complete matrix stamp for the ideal transformer is 

$$
  \begin{array}{c|c c c}
  ~ & j & k & l \\
  \hline
  j &  &  & -1 \\ 
  k &  &  & T \\ 
  l & 1 & -T & 0
  \end{array}
  \begin{pmatrix}
    v_j\\
    v_k\\
    i_{l}\\
  \end{pmatrix}
  =
  \begin{pmatrix}
    \\
    \\
    0\\
  \end{pmatrix}
$$

The variable $j$ denotes the high voltage node while $k$ is the low voltage node.
$l$ indicates the inserted row and column to accommodate the relation between the two voltages at the ends of the transformer.
The transformer ratio is defined as $T = V_{j} / V_{k}$.
A phase shift can be introduced if $T$ is considered as a complex number.