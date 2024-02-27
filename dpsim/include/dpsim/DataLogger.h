/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
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
#include <dpsim/Definitions.h>
#include <dpsim/Scheduler.h>

namespace DPsim {

class DataLogger : public SharedFactory<DataLogger> {

protected:
  std::ofstream mLogFile;
  String mName;
  Bool mEnabled;
  UInt mDownsampling;
  fs::path mFilename;

  std::map<String, CPS::AttributeBase::Ptr> mAttributes;

  void logDataLine(Real time, Real data);
  void logDataLine(Real time, const Matrix &data);
  void logDataLine(Real time, const MatrixComp &data);

public:
  typedef std::shared_ptr<DataLogger> Ptr;
  typedef std::vector<DataLogger::Ptr> List;

  DataLogger(Bool enabled = true);
  DataLogger(String name, Bool enabled = true, UInt downsampling = 1);

  void open();
  void close();
  void reopen() {
    close();
    open();
  }

  void logPhasorNodeValues(Real time, const Matrix &data, Int freqNum = 1);
  void logEMTNodeValues(Real time, const Matrix &data);

  void setColumnNames(std::vector<String> names);

  void logAttribute(const String &name, CPS::AttributeBase::Ptr attr,
                    UInt rowsMax = 0, UInt colsMax = 0);

  ///DEPRECATED: Only use for compatiblity, otherwise this just adds extra overhead to the logger. Instead just call logAttribute multiple times for every coefficient using `attr->deriveCoeff<>(a,b)`.
  void logAttribute(const std::vector<String> &name,
                    CPS::AttributeBase::Ptr attr);

  void log(Real time, Int timeStepCount);

  CPS::Task::Ptr getTask();

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
