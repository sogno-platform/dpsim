/* A data logger for real-time simulation data logging.
 *
 * Author: Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-FileCopyrightText: 2024 Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <system_error>

#include <dpsim-models/Attribute.h>
#include <dpsim-models/PtrFactory.h>
#include <dpsim-models/SimNode.h>
#include <dpsim-models/Task.h>
#include <dpsim/DataLoggerInterface.h>
#include <dpsim/Definitions.h>
#include <dpsim/Scheduler.h>

namespace DPsim {

class RealTimeDataLogger : public DataLoggerInterface,
                           public SharedFactory<RealTimeDataLogger> {

protected:
  std::filesystem::path mFilename;
  size_t mRowNumber;
  size_t mCurrentRow;
  size_t mCurrentAttribute;

  std::vector<std::vector<Real>> mAttributeData;

public:
  typedef std::shared_ptr<RealTimeDataLogger> Ptr;

  RealTimeDataLogger(std::filesystem::path &filename, Real finalTime,
                     Real timeStep);
  RealTimeDataLogger(std::filesystem::path &filename, size_t rowNumber);

  virtual void start() override;
  virtual void stop() override;

  virtual void log(Real time, Int timeStepCount) override;

  virtual CPS::Task::Ptr getTask() override;

  class Step : public CPS::Task {
  public:
    Step(RealTimeDataLogger &logger)
        : Task("RealTimeDataLogger.Write"), mLogger(logger) {
      for (auto attr : logger.mAttributes) {
        mAttributeDependencies.push_back(attr.second);
      }
      mModifiedAttributes.push_back(Scheduler::external);
    }

    void execute(Real time, Int timeStepCount);

  private:
    RealTimeDataLogger &mLogger;
  };
};
} // namespace DPsim
