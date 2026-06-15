// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/Base/Base_Turbine.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/SimSignalComp.h>

namespace CPS {
namespace Signal {

class HydroTurbineParameters : public Base::TurbineParameters,
                               public SharedFactory<HydroTurbineParameters> {
public:
  /// Water starting time (s)
  Real Tw = 0;
};

/// Hydro turbine — water starting time model.
/// Transfer function: (1 - s*Tw) / (1 + 0.5*s*Tw)
/// Ref.: A. Roehder, B. Fuchs, J. Massman, M. Quester, A. Schnettler,
///       "Transmission system stability assessment within an integrated grid
///       development process"
class HydroTurbine : public SimSignalComp,
                     public Base::Turbine,
                     public SharedFactory<HydroTurbine> {
private:
  std::shared_ptr<HydroTurbineParameters> mParameters;

  // ### State variables (current step) — loggable via .attr() ###
  /// Intermediate PT1 state at time k
  const Attribute<Real>::Ptr mX1;
  /// Mechanical output power (pu) at time k
  const Attribute<Real>::Ptr mPm;

  // ### Staging variable for next step ###
  Real mX1_next = 0;

public:
  HydroTurbine(const String &name,
               CPS::Logger::Level logLevel = CPS::Logger::Level::off);

  void setParameters(std::shared_ptr<Base::TurbineParameters> parameters) final;
  void initializeStates(Real Pminit) final;
  Real step(Real Pgv, Real dt) final;
};

} // namespace Signal
} // namespace CPS
