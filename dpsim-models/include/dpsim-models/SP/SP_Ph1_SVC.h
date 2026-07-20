// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <limits>

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/PFSolverInterfaceBus.h>

namespace CPS {
namespace SP {
namespace Ph1 {

/// \brief Static VAR compensator (SVC) power-flow model.
///
/// Voltage-controlling shunt reactive device. Modelled as a PQ bus whose
/// reactive injection Q_set is driven by the solver's SVC outer control loop
/// until the bus voltage reaches V_set (clamped to [Q_min, Q_max]). This
/// sidesteps the ill-conditioned derived-Q of the ordinary PV-bus formulation
/// at electrically stiff, near-zero-P buses. No active power (always 0) and no
/// admittance stamp: it participates purely through bus classification + the
/// reactive setpoint the outer loop maintains.
class SVC : public SimPowerComp<Complex>,
            public SharedFactory<SVC>,
            public PFSolverInterfaceBus {
private:
  /// Base voltage [V]
  Real mBaseVoltage;
  /// Base apparent power [VA]
  Real mBaseApparentPower;

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
  /// Setter for SVC parameters. setPointVoltage is the controlled voltage [V];
  /// qLimMax/qLimMin are the reactive-power band [VAr] the outer control loop
  /// respects. The device holds no active power (always 0).
  void setParameters(Real ratedApparentPower, Real ratedVoltage,
                     Real setPointVoltage,
                     Real qLimMax = std::numeric_limits<Real>::infinity(),
                     Real qLimMin = -std::numeric_limits<Real>::infinity());
  // #### Powerflow section ####
  Real getBaseVoltage() const;
  /// Set base voltage
  void setBaseVoltage(Real baseVoltage);
  /// Initializes component from power flow data
  void calculatePerUnitParameters(Real baseApparentPower, Real baseOmega);
  /// Modify powerflow bus type
  void modifyPowerFlowBusType(PowerflowBusType powerflowBusType) override;
  /// Update reactive power injection (driven by the SVC outer control loop)
  void updateReactivePowerInjection(Complex powerInj);
  /// Get apparent power of the power-flow solution (P is always 0)
  Complex getApparentPower() const;
};
} // namespace Ph1
} // namespace SP
} // namespace CPS
