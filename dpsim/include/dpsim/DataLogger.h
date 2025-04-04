/* Copyright 2017-2024 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <fstream>
#include <iostream>
#include <map>

#include <dpsim-models/Attribute.h>
#include <dpsim-models/Filesystem.h>
#include <dpsim-models/PtrFactory.h>
#include <dpsim-models/SimNode.h>
#include <dpsim-models/Task.h>
#include <dpsim/DataLoggerInterface.h>
#include <dpsim/Definitions.h>
#include <dpsim/Scheduler.h>

namespace DPsim {

class DataLogger : public DataLoggerInterface,
                   public SharedFactory<DataLogger> {

protected:
  std::ofstream mLogFile;
  String mName;
  Bool mEnabled;
  UInt mDownsampling;
  fs::path mFilename;

  virtual void logDataLine(Real time, Real data);
  virtual void logDataLine(Real time, const Matrix &data);
  virtual void logDataLine(Real time, const MatrixComp &data);

public:
  typedef std::shared_ptr<DataLogger> Ptr;

  DataLogger(Bool enabled = true);
  DataLogger(String name, Bool enabled = true, UInt downsampling = 1);
  virtual ~DataLogger(){};

  virtual void start() override;
  virtual void stop() override;

  virtual void setColumnNames(std::vector<String> names);
  void logPhasorNodeValues(Real time, const Matrix &data, Int freqNum = 1);
  void logEMTNodeValues(Real time, const Matrix &data);

  virtual void log(Real time, Int timeStepCount) override;

  virtual CPS::Task::Ptr getTask() override;

  class Step : public CPS::Task {
  public:
    Step(DataLogger &logger) : Task(logger.mName + ".Write"), mLogger(logger) {
      for (auto attr : logger.mAttributes) {
        mAttributeDependencies.push_back(attr.second);
      }
      mModifiedAttributes.push_back(Scheduler::external);
    }

    void execute(Real time, Int timeStepCount);

  private:
    DataLogger &mLogger;
  };
};
} // namespace DPsim
