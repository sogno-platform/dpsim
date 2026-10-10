// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <limits>

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/PFSolverInterfaceBus.h>

namespace CPS {
namespace SP {
namespace Ph1 {

/// Static VAR compensator for power flow: PQ bus, no active power, no admittance stamp
class SVC : public SimPowerComp<Complex>,
            public SharedFactory<SVC>,
            public PFSolverInterfaceBus {
private:
  /// Base voltage [V], set via setBaseVoltage() before power-flow init
  Real mBaseVoltage = 0;
  /// Base apparent power [VA]
  Real mBaseApparentPower = 0;

public:
  /// Voltage set point [V]
  const Attribute<Real>::Ptr mSetPointVoltage;
  /// Voltage set point [pu]
  const Attribute<Real>::Ptr mSetPointVoltagePerUnit;
  /// Reactive power injection [VAr] (maintained by the SVC outer loop)
  const Attribute<Real>::Ptr mSetPointReactivePower;
  /// Reactive power injection [pu]
  const Attribute<Real>::Ptr mSetPointReactivePowerPerUnit;
  /// Maximum reactive power limit [VAr] (default +inf = unlimited)
  const Attribute<Real>::Ptr mReactivePowerMax;
  /// Minimum reactive power limit [VAr] (default -inf = unlimited)
  const Attribute<Real>::Ptr mReactivePowerMin;
  /// Maximum reactive power limit [pu]
  const Attribute<Real>::Ptr mReactivePowerMaxPerUnit;
  /// Minimum reactive power limit [pu]
  const Attribute<Real>::Ptr mReactivePowerMinPerUnit;

  /// Defines UID, name and logging level
  SVC(String uid, String name, Logger::Level logLevel = Logger::Level::off);
  /// Defines name and logging level
  SVC(String name, Logger::Level logLevel = Logger::Level::off)
      : SVC(name, name, logLevel) {}
  /// Set SVC specific parameters; the rated values are logged only
  void setParameters(Real ratedApparentPower, Real ratedVoltage,
                     Real setPointVoltage,
                     Real qLimMax = std::numeric_limits<Real>::infinity(),
                     Real qLimMin = -std::numeric_limits<Real>::infinity());
  // #### Powerflow section ####
  /// Get base voltage
  Real getBaseVoltage() const;
  /// Set base voltage
  void setBaseVoltage(Real baseVoltage);
  /// Initializes component from power flow data
  void calculatePerUnitParameters(Real baseApparentPower, Real baseOmega);
  /// Modify powerflow bus type; only PQ is valid for an SVC
  void modifyPowerFlowBusType(PowerflowBusType powerflowBusType) override;
  /// Update reactive power injection (driven by the SVC outer control loop)
  void updateReactivePowerInjection(Complex powerInj);
  /// Get apparent power of the power-flow solution (P is always 0)
  Complex getApparentPower() const;
};
} // namespace Ph1
} // namespace SP
} // namespace CPS
