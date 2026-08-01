---
title: "Modal Analysis"
linkTitle: "Modal Analysis"
date: 2026-07-31
description: >
  Reading the modes of an extracted state-space model, and what participation factors say about them.
weight: 7
---

Extracting a state-space model, as described under
[state-space extraction]({{< ref "state-space-extraction-theory.md" >}}), produces a discrete state
matrix. Its eigenvalues describe how the system behaves without simulating it: which oscillations
exist, how fast each decays, and which states are involved in each one.

## From discrete to continuous eigenvalues

The extracted model is discrete, so its eigenvalues $z$ live in the complex plane where stability
means $|z| < 1$. That is awkward to read, because the quantities of interest are a frequency in
hertz and a damping ratio, both of which are natural in the continuous plane.

Because the model was discretised with the trapezoidal rule, the mapping back is its inverse, the
bilinear transform

```math
\lambda = \frac{2}{\Delta t} \, \frac{z - 1}{z + 1}.
```

This maps the interior of the unit disc onto the left half plane exactly, so a mode that is stable
in one description is stable in the other, with no threshold effects at the boundary. From
$\lambda = \sigma + j\omega$ the damped frequency is $\omega / 2\pi$ and the damping ratio is
$-\sigma / |\lambda|$.

The mapping is exact for the discretisation used, not an approximation of it. What it cannot undo is
the frequency warping the trapezoidal rule introduced in the first place: a continuous mode at a
frequency approaching the Nyquist rate is represented at a shifted frequency in the discrete model,
and mapping back returns the shifted value rather than the original. Modes well below Nyquist are
unaffected; modes near it should not be read literally.

## What the eigenvectors say

The eigenvalues say which modes exist but not which parts of the system take part in them. The right
eigenvectors describe how each mode appears in the states, the left eigenvectors describe how
strongly each state excites each mode, and the product of the two, element by element,

```math
p_{ki} = \phi_{ki} \, \psi_{ik},
```

is the participation factor of state $k$ in mode $i$.

Participation factors are the useful output. A poorly damped oscillation is a number; knowing that
two particular machine rotor states dominate it is actionable. They are also dimensionless and
normalised in a way that makes them comparable across states with different units, which raw
eigenvector entries are not.

The computation requires the eigenvector matrix to be invertible. It is not, when the state matrix
is defective, meaning it has a repeated eigenvalue without a full set of independent eigenvectors.
This is not a numerical failure but a property of the system, and it is the one case where
participation factors are not defined at all.

## Choice of frame

The states of the extracted model are in whatever frame each component works in, which for a network
containing machines means several rotating frames turning at different speeds plus the network's own.
Eigenvalues of that system are still correct, but the modes mix frames, and a mode's frequency is
then relative to whichever frame its dominant states live in.

Transforming everything into one common frame before the analysis removes that ambiguity, at the
price of choosing the frame and its initial angle. The two choices are therefore: analyse in the
native frames and read each mode relative to its own states, or transform to a single frame and read
every frequency against the same reference. The second is what makes modes from different machines
directly comparable.

## Limits

The analysis is linear and local. It describes the system as it is at the operating point where the
model was extracted, and says nothing about behaviour after a large disturbance moves it elsewhere.
A system can be comfortably damped at its nominal point and not at another, so a single modal
analysis is evidence about one condition rather than about the system.
