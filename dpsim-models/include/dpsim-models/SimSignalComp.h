/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Task.h>
#include <dpsim-models/TopologicalSignalComp.h>

namespace CPS {
/// Base class for all signal type components
/// that have only unidirectional connections
class SimSignalComp : public TopologicalSignalComp {
public:
  enum Behaviour { Initialization, Simulation };

protected:
  /// Determine state of the simulation, e.g. to implement
  /// special behavior for components during initialization
  Bool mBehaviour = Behaviour::Simulation;

public:
  typedef std::shared_ptr<SimSignalComp> Ptr;
  typedef std::vector<Ptr> List;

  ///
  SimSignalComp(String uid, String name,
                Logger::Level logLevel = Logger::Level::off)
      : TopologicalSignalComp(uid, name, logLevel) {}
  ///
  SimSignalComp(String name, Logger::Level logLevel = Logger::Level::off)
      : SimSignalComp(name, name, logLevel) {}
  ///
  virtual ~SimSignalComp() {}

  ///
  virtual void initialize(Real timeStep) {}
  ///
  virtual void initialize(Real omega, Real timeStep) { initialize(timeStep); }
  ///
  virtual Task::List getTasks() { return Task::List(); }
  /// Set behavior of component, e.g. initialization
  void setBehaviour(Behaviour behaviour) { mBehaviour = behaviour; }
};
} // namespace CPS
