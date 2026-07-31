---
title: "Models"
linkTitle: "Models"
weight: 5
description: >
  Mathematical description of the models implemented in DPsim.
---

Models are implemented per simulation domain and per phase count, so the same component may
exist as a single phase dynamic phasor model, a three phase electromagnetic transient model, or
both. The tables below list what exists today, which is the question this page is most often
asked.

`DP` is the dynamic phasor domain, `EMT` the electromagnetic transient domain and `SP` the
static phasor domain, each with `Ph1` for single phase and `Ph3` for three phase. The generated
[reference]({{< ref "/docs/Reference" >}}) is authoritative for exact class names and signatures.

## Passive elements and sources

| Model | Domains |
| --- | --- |
| Resistor, Inductor, Capacitor | DP::Ph1, DP::Ph3, EMT::Ph1, EMT::Ph3, SP::Ph1, SP::Ph3 |
| VoltageSource | DP::Ph1, DP::Ph3, EMT::Ph1, EMT::Ph3, SP::Ph1, SP::Ph3 |
| CurrentSource | DP::Ph1, DP::Ph3, EMT::Ph1, EMT::Ph3 |
| VoltageSourceNorton | DP::Ph1, EMT::Ph1, EMT::Ph3 |
| VoltageSourceRamp | DP::Ph1, EMT::Ph1 |
| ProfileVoltageSource | DP::Ph1 |
| ControlledVoltageSource, ControlledCurrentSource | DP::Ph1, EMT::Ph3, SP::Ph1 |
| NetworkInjection | DP::Ph1, DP::Ph3, EMT::Ph3, SP::Ph1 |

## Branches

Covered in more detail under [branches]({{< ref "branches.md" >}}),
[RLC elements]({{< ref "RLC-Elements" >}}) and [transformer]({{< ref "Transformer" >}}).

| Model | Domains |
| --- | --- |
| PiLine | DP::Ph1, DP::Ph3, EMT::Ph1, EMT::Ph3, SP::Ph1 |
| RxLine | DP::Ph1, EMT::Ph3, SP::Ph1, SP::Ph3 |
| SeriesResistor | DP::Ph3, EMT::Ph3 |
| Transformer | DP::Ph1, EMT::Ph3, SP::Ph1 |
| SolidStateTransformer | SP::Ph1 |

## Switches and loads

| Model | Domains |
| --- | --- |
| Switch | DP::Ph1, DP::Ph3, EMT::Ph1, EMT::Ph3, SP::Ph1 |
| SeriesSwitch | DP::Ph3, EMT::Ph3 |
| varResSwitch | DP::Ph1, SP::Ph1 |
| RXLoad | DP::Ph1, EMT::Ph3 |
| RXLoadSwitch, PQLoadCS | DP::Ph1 |
| Load | SP::Ph1 |
| Shunt | DP::Ph1, EMT::Ph3, SP::Ph1 |
| SVC | DP::Ph1 |

## Synchronous generators

See [synchronous generator]({{< ref "Synchronous Generator" >}}) for the model equations. VBR
denotes the voltage-behind-reactance formulation, PCM the predictor-corrector method and TPM the
two-stage predictor method.

| Model | Domains |
| --- | --- |
| SynchronGenerator3OrderVBR | DP::Ph1, EMT::Ph3, SP::Ph1 |
| SynchronGenerator4OrderVBR | DP::Ph1, EMT::Ph3, SP::Ph1 |
| SynchronGenerator5OrderVBR | DP::Ph1, EMT::Ph3, SP::Ph1 |
| SynchronGenerator6aOrderVBR, SynchronGenerator6bOrderVBR | DP::Ph1, EMT::Ph3, SP::Ph1 |
| SynchronGenerator4OrderPCM | DP::Ph1, EMT::Ph3 |
| SynchronGenerator6OrderPCM, SynchronGenerator4OrderTPM | DP::Ph1 |
| SynchronGeneratorDQ, DQODE, DQTrapez (full order) | DP::Ph3, EMT::Ph3 |
| SynchronGeneratorTrStab | DP::Ph1, EMT::Ph3, SP::Ph1 |
| SynchronGeneratorIdeal | DP::Ph1, EMT::Ph3 |
| SynchronGeneratorIter | DP::Ph1 |

## Power electronics

See [power electronics]({{< ref "power-electronics.md" >}}).

| Model | Domains |
| --- | --- |
| AvVoltageSourceInverterDQ | DP::Ph1, EMT::Ph3, SP::Ph1 |
| AvVoltSourceInverterStateSpace | DP::Ph1, DP::Ph3, EMT::Ph3 |
| Inverter (with harmonics) | DP::Ph1 |
| VoltageSourceInverter | SP::Ph1 |
| VSIVoltageControlVCO | EMT::Ph3 |
| SSN_GFM (grid forming) | EMT::Ph3 |

## State-space nodal components

Components solved simultaneously with the network rather than through a delayed current
injection. The method is described under
[state-space nodal]({{< ref "/docs/Concepts/state-space-nodal.md" >}}).

| Model | Domains |
| --- | --- |
| SSN_Full_Serial_RLC | DP::Ph1, DP::Ph3, EMT::Ph1, EMT::Ph3 |
| SSN_Variable_Serial_RLC | DP::Ph1 |
| SSN_Capacitor, SSN_Inductor | EMT::Ph3 |
| GenericTwoTerminalVTypeSSN, GenericTwoTerminalITypeSSN | DP::Ph1, DP::Ph3, EMT::Ph3 |
| GenericFourTerminalVTypeSSN | EMT::Ph3 |
| MixedVTypeVariableSSNComp | DP::Ph1, DP::Ph3 |

## Signal and control models

These are domain independent and live in the `Signal` namespace.

Excitation and voltage control: ExciterDC1, ExciterDC1Simp, ExciterST1Simp, ExciterStatic and
PSS1A, described under
[synchronous generator regulators]({{< ref "Synchronous Generator Regulators" >}}).

Turbines and governors: SteamTurbine, SteamTurbineGovernor, HydroTurbine, HydroTurbineGovernor,
TurbineGovernor and TurbineGovernorType1.

Converter control: PowerControllerVSI, VoltageControllerVSI, PLL and VCO.

Signal generators and filters: SineWaveGenerator, CosineFMGenerator, DCGenerator,
FrequencyRampGenerator, FIRFilter and Integrator.

Decoupling components, used to split a network across solvers or simulators: DecouplingLine,
DecouplingLineEMT, DecouplingLineEMT_Ph3, and DecouplingIdealTransformer for DP::Ph1, EMT::Ph1,
EMT::Ph3 and SP::Ph1. The underlying method is described under
[ideal transformer model]({{< ref "Ideal Transformer Model" >}}).
