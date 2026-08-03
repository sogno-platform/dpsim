---
title: "Model Availability"
aliases: ["/docs/models/"]
linkTitle: "Model Availability"
weight: 3
description: >
  Which simulation domain implements which model.
---

Which model exists in which domain. A tick means the domain has an implementation, a dash means it
does not.

The table below is generated from the headers under `dpsim-models/include/dpsim-models` by
`scripts/docs/generate_model_availability.py`. Do not edit it by hand; run the script with `--write`
instead. A model class the script does not recognise makes it fail rather than silently drop the
model, so the table cannot fall behind the code. For the equations behind a model, see
[models]({{< ref "/docs/Concepts/Models" >}}).

<!-- BEGIN GENERATED -->

## Passive elements and sources

| Model | SP::Ph1 | SP::Ph3 | DP::Ph1 | DP::Ph3 | EMT::Ph1 | EMT::Ph3 |
| --- | :---: | :---: | :---: | :---: | :---: | :---: |
| Resistor | &check; | &check; | &check; | &check; | &check; | &check; |
| Inductor | &check; | &check; | &check; | &check; | &check; | &check; |
| Capacitor | &check; | &check; | &check; | &check; | &check; | &check; |
| VoltageSource | &check; | &check; | &check; | &check; | &check; | &check; |
| CurrentSource | &ndash; | &ndash; | &check; | &check; | &check; | &check; |
| VoltageSourceNorton | &ndash; | &ndash; | &check; | &ndash; | &check; | &check; |
| VoltageSourceRamp | &ndash; | &ndash; | &check; | &ndash; | &check; | &ndash; |
| ProfileVoltageSource | &ndash; | &ndash; | &check; | &ndash; | &ndash; | &ndash; |
| ControlledVoltageSource | &check; | &ndash; | &check; | &ndash; | &ndash; | &check; |
| ControlledCurrentSource | &check; | &ndash; | &check; | &ndash; | &ndash; | &check; |
| NetworkInjection | &check; | &ndash; | &check; | &check; | &ndash; | &check; |

## Branches

| Model | SP::Ph1 | SP::Ph3 | DP::Ph1 | DP::Ph3 | EMT::Ph1 | EMT::Ph3 |
| --- | :---: | :---: | :---: | :---: | :---: | :---: |
| PiLine | &check; | &ndash; | &check; | &check; | &check; | &check; |
| RxLine | &ndash; | &check; | &check; | &ndash; | &ndash; | &check; |
| RXLine | &check; | &ndash; | &ndash; | &ndash; | &ndash; | &ndash; |
| SeriesResistor | &ndash; | &ndash; | &ndash; | &check; | &ndash; | &check; |
| ResIndSeries | &ndash; | &ndash; | &check; | &ndash; | &ndash; | &ndash; |
| Transformer | &check; | &ndash; | &check; | &ndash; | &ndash; | &check; |
| SolidStateTransformer | &check; | &ndash; | &ndash; | &ndash; | &ndash; | &ndash; |

## Switches and loads

| Model | SP::Ph1 | SP::Ph3 | DP::Ph1 | DP::Ph3 | EMT::Ph1 | EMT::Ph3 |
| --- | :---: | :---: | :---: | :---: | :---: | :---: |
| Switch | &check; | &ndash; | &check; | &check; | &check; | &check; |
| SeriesSwitch | &ndash; | &ndash; | &ndash; | &check; | &ndash; | &check; |
| varResSwitch | &check; | &ndash; | &check; | &ndash; | &ndash; | &ndash; |
| RXLoad | &ndash; | &ndash; | &check; | &ndash; | &ndash; | &check; |
| RXLoadSwitch | &ndash; | &ndash; | &check; | &ndash; | &ndash; | &ndash; |
| PQLoadCS | &ndash; | &ndash; | &check; | &ndash; | &ndash; | &ndash; |
| Load | &check; | &ndash; | &ndash; | &ndash; | &ndash; | &ndash; |
| Shunt | &check; | &ndash; | &check; | &ndash; | &ndash; | &check; |
| SVC | &ndash; | &ndash; | &check; | &ndash; | &ndash; | &ndash; |

## Synchronous generators

| Model | SP::Ph1 | SP::Ph3 | DP::Ph1 | DP::Ph3 | EMT::Ph1 | EMT::Ph3 |
| --- | :---: | :---: | :---: | :---: | :---: | :---: |
| SynchronGenerator | &check; | &ndash; | &ndash; | &ndash; | &ndash; | &ndash; |
| SynchronGeneratorDQ | &ndash; | &ndash; | &ndash; | &check; | &ndash; | &check; |
| SynchronGeneratorDQODE | &ndash; | &ndash; | &ndash; | &check; | &ndash; | &check; |
| SynchronGeneratorDQTrapez | &ndash; | &ndash; | &ndash; | &check; | &ndash; | &check; |
| SynchronGeneratorVBR | &ndash; | &ndash; | &ndash; | &ndash; | &ndash; | &check; |
| SynchronGenerator3OrderVBR | &check; | &ndash; | &check; | &ndash; | &ndash; | &check; |
| SynchronGenerator4OrderVBR | &check; | &ndash; | &check; | &ndash; | &ndash; | &check; |
| SynchronGenerator5OrderVBR | &check; | &ndash; | &check; | &ndash; | &ndash; | &check; |
| SynchronGenerator6aOrderVBR | &check; | &ndash; | &check; | &ndash; | &ndash; | &check; |
| SynchronGenerator6bOrderVBR | &check; | &ndash; | &check; | &ndash; | &ndash; | &check; |
| SynchronGenerator4OrderPCM | &ndash; | &ndash; | &check; | &ndash; | &ndash; | &check; |
| SynchronGenerator6OrderPCM | &ndash; | &ndash; | &check; | &ndash; | &ndash; | &ndash; |
| SynchronGenerator4OrderTPM | &ndash; | &ndash; | &check; | &ndash; | &ndash; | &ndash; |
| SynchronGeneratorIdeal | &ndash; | &ndash; | &check; | &ndash; | &ndash; | &check; |
| SynchronGeneratorIter | &ndash; | &ndash; | &check; | &ndash; | &ndash; | &ndash; |
| SynchronGeneratorTrStab | &check; | &ndash; | &check; | &ndash; | &ndash; | &check; |

## Power electronics

| Model | SP::Ph1 | SP::Ph3 | DP::Ph1 | DP::Ph3 | EMT::Ph1 | EMT::Ph3 |
| --- | :---: | :---: | :---: | :---: | :---: | :---: |
| AvVoltageSourceInverterDQ | &check; | &ndash; | &check; | &ndash; | &ndash; | &check; |
| AvVoltSourceInverterStateSpace | &ndash; | &ndash; | &check; | &check; | &ndash; | &check; |
| Inverter | &ndash; | &ndash; | &check; | &ndash; | &ndash; | &ndash; |
| VoltageSourceInverter | &check; | &ndash; | &ndash; | &ndash; | &ndash; | &ndash; |
| VSIVoltageControlVCO | &ndash; | &ndash; | &ndash; | &ndash; | &ndash; | &check; |
| SSN_GFM | &ndash; | &ndash; | &ndash; | &ndash; | &ndash; | &check; |

## State-space nodal components

| Model | SP::Ph1 | SP::Ph3 | DP::Ph1 | DP::Ph3 | EMT::Ph1 | EMT::Ph3 |
| --- | :---: | :---: | :---: | :---: | :---: | :---: |
| SSN_Full_Serial_RLC | &ndash; | &ndash; | &check; | &check; | &check; | &check; |
| SSN_Variable_Serial_RLC | &ndash; | &ndash; | &check; | &ndash; | &ndash; | &ndash; |
| SSN_Capacitor | &ndash; | &ndash; | &ndash; | &ndash; | &ndash; | &check; |
| SSN_Inductor | &ndash; | &ndash; | &ndash; | &ndash; | &ndash; | &check; |
| SSNTypeV2T | &check; | &ndash; | &ndash; | &ndash; | &check; | &ndash; |
| SSNTypeI2T | &check; | &ndash; | &ndash; | &ndash; | &check; | &ndash; |
| PiecewiseLinearInductor | &ndash; | &ndash; | &ndash; | &ndash; | &ndash; | &check; |
| GenericTwoTerminalVTypeSSN | &ndash; | &ndash; | &check; | &check; | &ndash; | &check; |
| GenericTwoTerminalITypeSSN | &ndash; | &ndash; | &check; | &check; | &ndash; | &check; |
| GenericFourTerminalVTypeSSN | &ndash; | &ndash; | &ndash; | &ndash; | &ndash; | &check; |

## Excitation and stabilizers

- `ExciterDC1`
- `ExciterDC1Simp`
- `ExciterST1Simp`
- `ExciterStatic`
- `PSS1A`

## Turbines and governors

- `SteamTurbine`
- `SteamTurbineGovernor`
- `HydroTurbine`
- `HydroTurbineGovernor`
- `TurbineGovernor`
- `TurbineGovernorType1`

## Converter control

- `PowerControllerVSI`
- `VoltageControllerVSI`
- `PLL`
- `VCO`

## Signal sources and filters

- `SignalGenerator`
- `SineWaveGenerator`
- `CosineFMGenerator`
- `DCGenerator`
- `FrequencyRampGenerator`
- `FIRFilter`
- `Integrator`

## Decoupling components

These are network components: they connect to nodes and own their own sources. They are declared in the `Signal` namespace for historical reasons, which is why their domain appears in the class name rather than in the namespace.

| Model | SP::Ph1 | SP::Ph3 | DP::Ph1 | DP::Ph3 | EMT::Ph1 | EMT::Ph3 |
| --- | :---: | :---: | :---: | :---: | :---: | :---: |
| DecouplingLine | &ndash; | &ndash; | &check; | &ndash; | &ndash; | &ndash; |
| DecouplingLineEMT | &ndash; | &ndash; | &ndash; | &ndash; | &check; | &ndash; |
| DecouplingLineEMT_Ph3 | &ndash; | &ndash; | &ndash; | &ndash; | &ndash; | &check; |
| DecouplingIdealTransformer_SP_Ph1 | &check; | &ndash; | &ndash; | &ndash; | &ndash; | &ndash; |
| DecouplingIdealTransformer_DP_Ph1 | &ndash; | &ndash; | &check; | &ndash; | &ndash; | &ndash; |
| DecouplingIdealTransformer_EMT_Ph1 | &ndash; | &ndash; | &ndash; | &ndash; | &check; | &ndash; |
| DecouplingIdealTransformer_EMT_Ph3 | &ndash; | &ndash; | &ndash; | &ndash; | &ndash; | &check; |

<!-- END GENERATED -->
