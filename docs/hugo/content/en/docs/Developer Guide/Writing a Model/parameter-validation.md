---
title: "Parameter Validation"
linkTitle: "Parameter Validation"
date: 2026-08-07
description: >
  Declaring the constraints a component's parameters must satisfy, so bad data fails with a name instead of a NaN.
weight: 10
---

A component that stamps `1/R` or `M.inverse()` produces `inf` or `NaN` when the parameter is zero.
Nothing in the solver rejects that value: it enters the system matrix, the factorization propagates
it to every node, and the simulation finishes with a result that is entirely `NaN` and no error
message. The originating component is not recoverable from the output.

`ParameterCheck` moves the rejection to a single point during initialization, where the component
that owns the bad value is still known.

## Declaring constraints

Override `validateParameters` and name the constraints each parameter has to satisfy:

```cpp
void EMT::Ph3::PiLine::validateParameters(ParameterCheck &check) {
  check(mSeriesRes, {Constraint::Finite, Constraint::Invertible});
  check(mSeriesInd, {Constraint::Finite, Constraint::Invertible});
  check(mParallelCap, {Constraint::Finite});
  check(mParallelCond, {Constraint::Finite});
}
```

The parameter name in the error message is resolved from the component's own attribute map, so it
always matches the attribute the model registered and cannot drift out of sync with a hand-written
string.

Constraints that relate two parameters have no enum form. Use `require` for those:

```cpp
check.require(**mSeriesRes != 0. || **mSeriesInd != 0., "series impedance is zero");
```

## Available constraints

| Constraint | Scalar | Matrix |
| --- | --- | --- |
| `Finite` | no `inf` or `NaN` | no `inf` or `NaN` in any entry |
| `NonNegative` | `>= 0` | every entry `>= 0` |
| `Positive` | `> DOUBLE_EPSILON` | every entry `> DOUBLE_EPSILON` |
| `NonZero` | magnitude above `DOUBLE_EPSILON` | not the zero matrix |
| `DiagonalPositive` | same as `Positive` | square, every diagonal entry `> DOUBLE_EPSILON` |
| `Invertible` | same as `NonZero` | square, full rank |

A non-finite value fails every constraint, not only `Finite`.

`NonNegative`, `Positive` and `DiagonalPositive` have no meaning for a complex parameter. Applying
them to one is reported as a violation describing the mismatch, rather than passing silently.

## Choosing between Positive and Invertible for a matrix

`Positive` on a matrix is elementwise, and three-phase parameters built by
`Math::singlePhaseParameterToThreePhase` carry values on the diagonal only. Every off-diagonal entry
is a legitimate zero, so `Positive` rejects them all. A matrix with mutual coupling can also carry
negative off-diagonal terms.

Use `Invertible` where the model inverts the matrix, which is what the stamp actually requires, and
`DiagonalPositive` where only the self terms are constrained. `Invertible` is decided by the rank of
a full-pivoting LU decomposition, whose threshold is relative to the largest pivot, so it does not
reject a well-conditioned matrix merely because its entries are small.

## When the check runs

`Simulation::initialize` calls `SystemTopology::checkParameters`, which visits every component
before any solver is created. Violations from the whole topology are collected and reported together
in one `std::invalid_argument`, so a grid with several bad components is diagnosed in a single run
rather than one exception at a time:

```
invalid component parameters (2 violations):
  line1.R_series: must be square and invertible
  line4.L_series: must be square and invertible
```

## Constraints that depend on the solver

Some parameters are valid for one solver and not another. A power flow line with zero series
inductance is well defined, because the model builds an admittance from `R + jX` and never
discretizes the inductor; the same line in an MNA simulation creates an inductor subcomponent whose
stamp divides by that inductance.

`validateParameters` is not told which solver will consume the component, so a constraint may only
express what holds for every solver the component supports. Where the two differ, declare the weaker
constraint.

Only the `SP::Ph1` models are affected, because the power flow solver handles no other domain. The
DP and EMT lines have no power flow path and constrain their series inductance strictly. Before
weakening a constraint for this reason, check whether the component can actually reach the power
flow solver at all.
