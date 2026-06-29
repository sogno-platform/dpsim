/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <limits>

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/PFSolverInterfaceBus.h>

namespace CPS {

namespace SP {
namespace Ph1 {

class SynchronGenerator : public SimPowerComp<Complex>,
                          public SharedFactory<SynchronGenerator>,
                          public PFSolverInterfaceBus {
private:
  /// Base voltage [V]
  Real mBaseVoltage;
  /// Base apparent power[VA]
  Real mBaseApparentPower;

public:
  /// Active power set point of the machine [W]
  const Attribute<Real>::Ptr mSetPointActivePower;
  /// Reactive power set point of the machine [VAr]
  const Attribute<Real>::Ptr mSetPointReactivePower;
  /// Voltage set point of the machine [V]
  const Attribute<Real>::Ptr mSetPointVoltage;
  /// Active power set point of the machine [pu]
  const Attribute<Real>::Ptr mSetPointActivePowerPerUnit;
  /// Reactive power set point of the machine [pu]
  const Attribute<Real>::Ptr mSetPointReactivePowerPerUnit;
  /// Voltage set point of the machine [pu]
  const Attribute<Real>::Ptr mSetPointVoltagePerUnit;
  /// Maximum reactive power limit [VAr] (default +inf = unlimited)
  const Attribute<Real>::Ptr mReactivePowerMax;
  /// Minimum reactive power limit [VAr] (default -inf = unlimited)
  const Attribute<Real>::Ptr mReactivePowerMin;
  /// Maximum reactive power limit [pu]
  const Attribute<Real>::Ptr mReactivePowerMaxPerUnit;
  /// Minimum reactive power limit [pu]
  const Attribute<Real>::Ptr mReactivePowerMinPerUnit;

  /// Defines UID, name and logging level
  SynchronGenerator(String uid, String name,
                    Logger::Level logLevel = Logger::Level::off);
  /// Defines name and logging level
  SynchronGenerator(String name, Logger::Level logLevel = Logger::Level::off)
      : SynchronGenerator(name, name, logLevel) {}
  /// Setter for synchronous generator parameters. qLimMax/qLimMin are the
  /// reactive-power limits [VAr] enforced by the PF Q-limit outer loop; the
  /// default +/-inf means unlimited (no PV->PQ switching for this machine).
  void setParameters(Real ratedApparentPower, Real ratedVoltage,
                     Real setPointActivePower, Real setPointVoltage,
                     PowerflowBusType powerflowBusType,
                     Real setPointReactivepower = 0,
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
  /// Update reactive power injection (PV Bus)
  void updateReactivePowerInjection(Complex powerInj);
  /// Update active & reactive power injection (VD bus)
  void updatePowerInjection(Complex powerInj);
  /// Get Apparent power of Powerflow solution
  Complex getApparentPower() const;
};
} // namespace Ph1
} // namespace SP
} // namespace CPS
