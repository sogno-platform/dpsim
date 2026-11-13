/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <array>

#include <dpsim-models/EMT/EMT_Ph3_ReducedOrderSynchronGeneratorVBR.h>

#include <array>

namespace CPS {
namespace EMT {
namespace Ph3 {
struct HipSG4State {
  double theta = 0.0, delta = 0.0, omega = 1.0;
  double electricalTorque = 0.0, mechanicalTorque = 0.0;
  double previousMechanicalTorque = 0.0;
  double fieldVoltage = 0.0, previousFieldVoltage = 0.0;
  double timestep = 0.0, inertia = 0.0, damping = 0.0;
  double baseOmegaMech = 0.0, baseVoltage = 0.0, baseCurrent = 0.0;
  double lqTransient = 0.0, ldTransient = 0.0;
  double adTransient = 0.0, bdTransient = 0.0;
  double aqTransient = 0.0, bqTransient = 0.0, dqTransient = 0.0;
  double idq0[3]{}, vdq0[3]{}, edq0Transient[3]{};
  double conductanceDq0[9]{}, conductanceAbc[9]{};
  double historyDq0[3]{}, emfAbc[3]{}, currentSourceAbc[3]{};
  double terminalVoltageAbc[3]{}, terminalCurrentAbc[3]{};
  double voltageDq0[3]{}, currentDq0[3]{};
  int busRows[3]{};
  int grounded = 0;
};

/// @brief Voltage-Behind-Reactance (VBR) implementation
/// of 4th order synchronous generator model
class SynchronGenerator4OrderVBR
    : public ReducedOrderSynchronGeneratorVBR,
      public SharedFactory<SynchronGenerator4OrderVBR> {
public:
  /// Plain data needed by a specialized device executor for the benchmark's
  /// controller-free, Norton-form SG4 VBR model. Values are captured only
  /// after normal DPsim initialization has established per-unit state and MNA
  /// node indices.
  struct MetalDeviceData {
    std::array<Real, 3> voltageDq0;
    std::array<Real, 3> currentDq0;
    std::array<Real, 3> transientVoltageDq0;
    std::array<Real, 9> conductanceDq0;
    std::array<Real, 9> conductanceAbc;
    std::array<Real, 3> sourceAbc;
    std::array<UInt, 3> matrixNodeIndices;
    Real timeStep;
    Real baseVoltage;
    Real baseCurrent;
    Real baseImpedance;
    Real baseOmegaElec;
    Real baseOmegaMech;
    Real inertia;
    Real damping;
    Real ldTransient;
    Real lqTransient;
    Real adTransient;
    Real bdTransient;
    Real aqTransient;
    Real bqTransient;
    Real dqTransient;
    Real fieldVoltage;
    Real previousFieldVoltage;
    Real mechanicalTorque;
    Real previousMechanicalTorque;
    Real electricalTorque;
    Real rotorSpeed;
    Real rotorAngle;
    Real loadAngle;
  };

  // ### Model specific elements ###
  /// transient voltage
  const Attribute<Matrix>::Ptr mEdq0_t;

protected:
  /// history term of VBR
  Matrix mEhs_vbr;

public:
  ///
  SynchronGenerator4OrderVBR(const String &uid, const String &name,
                             Logger::Level logLevel = Logger::Level::off);
  ///
  SynchronGenerator4OrderVBR(const String &name,
                             Logger::Level logLevel = Logger::Level::off);

  // #### General Functions ####
  ///
  void specificInitialization() final;
  ///
  void stepInPerUnit() final;

  MetalDeviceData metalDeviceData();
  /// Export/import the SG4 VBR subset supported by the HIP batch kernel.
  bool supportsHipBatch() noexcept;
  HipSG4State exportHipState();
  void applyHipPreState(const HipSG4State &state, Real time);
  void applyHipPostState(const HipSG4State &state);
};
} // namespace Ph3
} // namespace EMT
} // namespace CPS
