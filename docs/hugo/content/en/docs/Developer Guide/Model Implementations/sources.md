---
title: "Source Implementation"
linkTitle: "Sources"
date: 2026-07-31
description: >
  How the source components stamp, and the quirks in the non-ideal ones.
weight: 22
---

The models are derived under [sources]({{< ref "/docs/Concepts/Models/sources.md" >}}). This page
covers only the code.

## Ideal sources

`VoltageSource` requests one virtual node, which carries the source current as the extra unknown,
and stamps the constraint rows that fix the terminal voltage difference. `CurrentSource` requests
none and contributes only to the right hand side.

`ControlledVoltageSource` and `ControlledCurrentSource` are the same components with their reference
supplied as an attribute rather than a parameter, so another component or an interface can drive
them. The reference is read during the pre-step, which is why it is the previous step's value.

## `VoltageSourceNorton`

Stamps directly rather than through a virtual node. `mnaCompApplySystemMatrixStamp` adds
`mConductance` to both diagonal entries and subtracts it from the two off-diagonal entries, guarded
by `terminalNotGrounded`, and `mnaCompApplyRightSideVectorStamp` sets the equivalent current
`mIntfVoltage / mResistance` with opposite signs at the two terminals.

`mConductance` is computed in `setParameters` as `1 / resistance`, so calling `setParameters` is
mandatory before the run and a zero resistance is a division by zero rather than an ideal source.

{{% alert title="Watch out: EMT::Ph3 must set the phase type first" color="warning" %}}
The EMT::Ph3 variant was missing `mPhaseType = PhaseType::ABC` in its constructor until 2026-07-31.
Without it `SimPowerComp::initialize` sized the interface matrices to one row and the component
aborted the process on an Eigen bounds assertion when it wrote rows 1 and 2. The general rule that
came out of it is on the [reduced order generator]({{< ref "reduced-order-generator.md" >}}) page and
applies to any EMT::Ph3 component: set the phase type in the constructor, before
`setVirtualNodeNumber`.
{{% /alert %}}

## `VoltageSourceRamp`

A composite wrapping a `VoltageSource` whose reference it rewrites each step in `updateState(time)`.
Three regimes: before `mSwitchTime` the reference is unchanged; during `mRampTime` the added voltage
is interpolated linearly while the added frequency is blended by a raised sine
`0.5 + 0.5 * sin(pi * t / T - pi/2)`; afterwards both are fully applied.

The two are blended differently on purpose. A linear frequency interpolation applied as a phase
offset would step the phase at both ends of the ramp; the raised sine has zero derivative at both
ends, so the frequency contribution enters and leaves smoothly. The consequence is that the
instantaneous frequency during the ramp is not the linear interpolation between the two values, and
reading `mAddSrcFreq` as "the frequency at the midpoint" is wrong.

Note also that the added frequency term is applied as `mAddSrcFreq * time`, using absolute
simulation time rather than time since the switch, so the phase contribution depends on when in the
run the ramp occurs.

## `ProfileVoltageSource`

Holds a `std::filesystem::path`, a sample vector and an index, and reads the file in `readFromFile`
at construction. It implements `DAEInterface` in addition to the MNA hooks.

The samples are stepped by index rather than interpolated against simulation time, so the profile's
sample rate and the simulation step must match for the waveform to have the intended duration. It is
bound in Python and constructing it with a file that is not a readable sample list raises rather
than crashing, which is covered by a test.

## Source

Under `dpsim-models/src/{SP,DP,EMT}/`. Availability per domain is in
[model availability]({{< ref "/docs/Reference/model-availability.md" >}}).
