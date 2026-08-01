---
title: "Signal Models"
linkTitle: "Signal Models"
weight: 4
description: >
  Controllers, regulators, generators and decoupling elements in the Signal namespace.
---

Signal models live in `CPS::Signal` and are domain independent: the same controller drives a
dynamic phasor or an electromagnetic transient machine model, because it operates on scalar
signals rather than on network quantities. The exception is the decoupling group, which exists
per domain since it inserts real components into the network.

## Excitation systems

Regulate generator terminal voltage by acting on field voltage. Equations and block diagrams are on the
[regulators]({{< ref "/docs/Concepts/Models/Synchronous Generator Regulators" >}}) page.

| Model | Description |
| --- | --- |
| `ExciterDC1` | Standard IEEE type DC1 exciter |
| `ExciterDC1Simp` | Simplified version of the IEEE type DC1 exciter |
| `ExciterST1Simp` | Simplified static exciter |
| `ExciterStatic` | Static exciter, with an anti-windup strategy for the integral component |

## Power system stabiliser

| Model | Description |
| --- | --- |
| `PSS1A` | Simplified IEEE PSS1A. Enhances damping of electromechanical oscillations, accepting rotor speed, active power and terminal voltage magnitude as optional inputs. Its output feeds the exciter |

## Turbines and governors

Governors set mechanical power from speed deviation; turbine models convert that into the torque
applied to the machine.

| Model | Description |
| --- | --- |
| `SteamTurbine` | Steam turbine, used in series with its governor |
| `SteamTurbineGovernor` | Governor for the steam turbine, instantiated separately from it |
| `HydroTurbine` | Hydro turbine, used in series with its governor |
| `HydroTurbineGovernor` | Governor for the hydro turbine, instantiated separately from it |
| `TurbineGovernorType1` | Turbine and governor combined in one component |
| `TurbineGovernor` | Turbine and governor combined in one component |

## Converter control

Control loops for the averaged inverter models. See
[power electronics]({{< ref "/docs/Concepts/Models/Power Electronics" >}}) for how these
attach to the converter.

| Model | Description |
| --- | --- |
| `PowerControllerVSI` | Power control loop used by the averaged grid-following inverter models |
| `VoltageControllerVSI` | Voltage control loop used by the grid-forming inverter models |
| `PLL` | Phase-locked loop |
| `VCO` | Voltage-controlled oscillator |

## Signal generators

Drive sources and setpoints from a prescribed waveform rather than a constant.

| Model | Description |
| --- | --- |
| `SignalGenerator` | Base class for the generators below |
| `SineWaveGenerator` | Sine wave |
| `CosineFMGenerator` | Frequency-modulated cosine |
| `FrequencyRampGenerator` | Frequency ramp |
| `DCGenerator` | Constant value |

## Filters and maths

| Model | Description |
| --- | --- |
| `FIRFilter` | Finite impulse response filter |
| `Integrator` | Integrator block used inside the control models |

## Decoupling

Split a network into parts that can be solved separately, either across solvers or across
simulators in a co-simulation. See
[co-simulation]({{< ref "/docs/User Guide/co-simulation.md" >}}).

| Model | Domains |
| --- | --- |
| `DecouplingLine` | `DP::Ph1` |
| `DecouplingLineEMT` | `EMT::Ph1` |
| `DecouplingLineEMT_Ph3` | `EMT::Ph3` |
| `DecouplingIdealTransformer` | `DP::Ph1`, `EMT::Ph1`, `EMT::Ph3`, `SP::Ph1` |

## References

- IEEE Std 421.5-2016, "IEEE Recommended Practice for Excitation System Models for Power System
  Stability Studies". <https://doi.org/10.1109/IEEESTD.2016.7553421>
- F. Milano, *Power System Modelling and Scripting*. London: Springer-Verlag, 2010.
  <https://doi.org/10.1007/978-3-642-13669-6>
- M. Eremia and M. Shahidehpour, *Handbook of Electrical Power System Dynamics: Modeling,
  Stability, and Control*. <https://ieeexplore.ieee.org/book/6480471>
