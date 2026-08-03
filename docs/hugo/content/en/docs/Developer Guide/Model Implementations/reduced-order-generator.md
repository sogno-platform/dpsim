---
title: "Reduced Order Generator Implementation"
linkTitle: "Reduced Order Generator"
date: 2026-07-31
description: >
  How the reduced order machine equations are arranged in code and stamped into the solver.
weight: 20
---

The equations are derived under
[reduced order machine models]({{< ref "/docs/Concepts/Models/Synchronous Generator/reduced-order.md" >}}).
This page covers only their arrangement in code.

## Class hierarchy

`Base::ReducedOrderSynchronGenerator<VarType>` holds everything independent of domain and of order:
the per unit base values, the operational parameters, the mechanical states, the controller
attachments and the discretisation coefficients. It is templated on `Real` for EMT and `Complex`
for DP and SP, which is why the axis frame quantities appear twice, as `mVdq0`/`mIdq0` in the real
specialisation and `mVdq`/`mIdq` in the complex one.

Each domain then provides a `ReducedOrderSynchronGeneratorVBR` layer holding the frame transform,
and each order a concrete class. The order is recorded in `mSGOrder`, which selects which
coefficients are computed.

## Network interface

`setModelAsNortonSource` chooses between the two interface forms. The default is the Norton
equivalent, in which the machine contributes only to the right hand side vector and requests no
virtual nodes. The Thevenin form requests two virtual nodes instead. Both represent the same model;
the Norton form is cheaper because it leaves the system matrix untouched between steps
[[Wang2010](#Wang2010)].

{{% alert title="Watch out: call setModelAsNortonSource before connecting" color="warning" %}}
Note that `setModelAsNortonSource` calls `setVirtualNodeNumber`, so it must be called before the
component is connected.
{{% /alert %}}

## Coefficients

`calculateAuxiliarConstants` computes the discretisation coefficients once, since they depend only
on the parameters and the step size. The member names map to the symbols on the theory page as
follows.

| Member | Symbol |
| --- | --- |
| `mAd_t`, `mBd_t` | $A_d'$, $B_d'$ |
| `mAq_t`, `mBq_t`, `mDq_t` | $A_q'$, $B_q'$, $D_q'$ |
| `mAd_s`, `mBq_s`, `mCd_s`, `mCq_s`, `mAq_s` | subtransient coefficients |
| `mYd`, `mYq` | $Y_d$, $Y_q$, non-zero only for the 6a variant |

The naming looks wrong at first and is not. `Zd_t` is built from $L_q - L_q'$ and `Zq_t` from
$L_d - L_d'$, because each is named for the axis whose coefficient it feeds rather than for the
parameters it is assembled from. That follows the physics: the d-axis internal voltage arises from
q-axis rotor flux and decays with $T_{q0}'$, so `mAd_t` correctly combines $L_q - L_q'$ with
$T_{q0}'$ and multiplies the q-axis current.

Read a coefficient's use rather than its assignment line before concluding an axis is swapped.

## Step sequence

`mnaCompPreStep` runs before the network solve and does three things in order. It advances the
controllers, saving `mEf_prev` and `mMechTorque_prev` first because the trapezoidal history terms
need the previous values. It calls `stepInPerUnit`, which updates the frame transforms from
`mThetaMech`, recomputes the axis frame state from the terminal quantities, and evaluates the
history voltage into `mEh_vbr`. It then stamps the result into the right hand side vector.

Each concrete order implements only `specificInitialization` and `stepInPerUnit`. Everything else is
inherited.

## Initialization

Initialization runs from the powerflow solution, not from user supplied states. The base class
computes the load angle as the phase of $V + j L_q I$, projects the terminal voltage and current
onto the axis frame, and derives the field voltage from the no-load relation. Only then does
`specificInitialization` set the order specific states, which is why a concrete class can assume
`mVdq` and `mIdq` are already populated.

Attached controllers are initialized afterwards from the machine's own initial values, so an
exciter or governor never needs its own operating point.

## Controllers

Excitation, governor, turbine and power system stabilizer attach through the base class and are
optional, guarded by `mHasExciter`, `mHasGovernorAndTurbine`, `mHasTurbineGovernor` and `mHasPSS`.
The stabilizer output feeds the exciter within the same step, and the governor output feeds the
turbine, so the order of the calls in `mnaCompPreStep` is load bearing.

## Source code

- [Base class header](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/include/dpsim-models/Base/Base_ReducedOrderSynchronGenerator.h), [implementation](https://github.com/sogno-platform/dpsim/blob/master/dpsim-models/src/Base/Base_ReducedOrderSynchronGenerator.cpp)
- Concrete orders under `dpsim-models/src/{SP,DP,EMT}/` named `SynchronGenerator<N>OrderVBR`
- Availability per domain is listed under [model availability]({{< ref "/docs/Reference/model-availability.md" >}})

## References

- <a name="Wang2010"></a>[Wang2010] [IEEE Xplore document 5411963](https://ieeexplore.ieee.org/document/5411963). Cited in the machine model pages as the basis for interfacing a machine to a nodal solver through a current source that leaves the system matrix unchanged.
