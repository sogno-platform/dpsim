/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/IdentifiedObject.h>
#include <dpsim-models/MathUtils.h>
#include <dpsim-models/PtrFactory.h>

namespace CPS {
/// Base class for all signal type components
/// that have only unidirectional connections
class TopologicalSignalComp : public IdentifiedObject {
protected:
  /// Component logger
  Logger::Log mSLog;
  /// Component logger control for internal variables
  Logger::Level mLogLevel;

public:
  typedef std::shared_ptr<TopologicalSignalComp> Ptr;
  typedef std::vector<Ptr> List;

  /// Basic constructor that takes UID, name and log level
  TopologicalSignalComp(String uid, String name,
                        Logger::Level logLevel = Logger::Level::off)
      : IdentifiedObject(uid, name),
        /* We also want to set the CLI loglevel according to the logLevel
         * std::max(Logger::Level::info, logLevel). But because of excessive
         * logging to Level::info that is currently infeasible. */
        mSLog(Logger::get(name, logLevel, Logger::Level::warn)),
        mLogLevel(logLevel) {}

  /// Basic constructor that takes name and log level and sets the UID to name as well
  TopologicalSignalComp(String name,
                        Logger::Level logLevel = Logger::Level::off)
      : TopologicalSignalComp(name, name, logLevel) {}
  ///
  virtual ~TopologicalSignalComp() {}
};
} // namespace CPS
