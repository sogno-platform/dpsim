/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <iomanip>

#include <dpsim-models/Logger.h>
#include <dpsim/DataLogger.h>

using namespace DPsim;

DataLogger::DataLogger(Bool enabled) : DataLoggerInterface(), mLogFile(), mEnabled(enabled), mDownsampling(1) { mLogFile.setstate(std::ios_base::badbit); }

DataLogger::DataLogger(String name, Bool enabled, UInt downsampling) : DataLoggerInterface(), mName(name), mEnabled(enabled), mDownsampling(downsampling) {
  if (!mEnabled)
    return;

  mFilename = CPS::Logger::logDir() + "/" + name + ".csv";

  if (mFilename.has_parent_path() && !fs::exists(mFilename.parent_path()))
    fs::create_directory(mFilename.parent_path());
}

void DataLogger::start() {
  if (!mEnabled)
    return;

  mLogFile =
      std::ofstream(mFilename, std::ios_base::out | std::ios_base::trunc);
  if (!mLogFile.is_open()) {
    // TODO: replace by exception
    std::cerr << "Cannot open log file " << mFilename << std::endl;
    mEnabled = false;
  }
}

void DataLogger::stop() { mLogFile.close(); }

void DataLogger::setColumnNames(std::vector<String> names) {
  if (mLogFile.tellp() == std::ofstream::pos_type(0)) {
    mLogFile << std::right << std::setw(14) << "time";
    for (auto name : names) {
      mLogFile << ", " << std::right << std::setw(13) << name;
    }
    mLogFile << '\n';
  }
}

void DataLogger::logDataLine(Real time, Real data) {
  if (!mEnabled)
    return;

  mLogFile << std::scientific << std::right << std::setw(14) << time;
  mLogFile << ", " << std::right << std::setw(13) << data;
  mLogFile << '\n';
}

void DataLogger::logDataLine(Real time, const Matrix &data) {
  if (!mEnabled)
    return;

  mLogFile << std::scientific << std::right << std::setw(14) << time;
  for (Int i = 0; i < data.rows(); ++i) {
    mLogFile << ", " << std::right << std::setw(13) << data(i, 0);
  }
  mLogFile << '\n';
}

void DataLogger::logDataLine(Real time, const MatrixComp &data) {
  if (!mEnabled)
    return;
  mLogFile << std::scientific << std::right << std::setw(14) << time;
  for (Int i = 0; i < data.rows(); ++i) {
    mLogFile << ", " << std::right << std::setw(13) << data(i, 0);
  }
  mLogFile << '\n';
}

void DataLogger::logPhasorNodeValues(Real time, const Matrix &data,
                                     Int freqNum) {
  if (mLogFile.tellp() == std::ofstream::pos_type(0)) {
    std::vector<String> names;

    Int harmonicOffset = data.rows() / freqNum;
    Int complexOffset = harmonicOffset / 2;

    for (Int freq = 0; freq < freqNum; ++freq) {
      for (Int node = 0; node < complexOffset; ++node) {
        std::stringstream name;
        name << "n" << std::setfill('0') << std::setw(5) << node << "f"
             << std::setfill('0') << std::setw(2) << freq << ".re";
        names.push_back(name.str());
      }
      for (Int node = 0; node < complexOffset; ++node) {
        std::stringstream name;
        name << "n" << std::setfill('0') << std::setw(5) << node << "f"
             << std::setfill('0') << std::setw(2) << freq << ".im";
        names.push_back(name.str());
      }
    }
    setColumnNames(names);
  }
  logDataLine(time, data);
}

void DataLogger::logEMTNodeValues(Real time, const Matrix &data) {
  if (mLogFile.tellp() == std::ofstream::pos_type(0)) {
    std::vector<String> names;
    for (Int i = 0; i < data.rows(); ++i) {
      std::stringstream name;
      name << "node" << std::setfill('0') << std::setw(5) << i;
      names.push_back(name.str());
    }
    setColumnNames(names);
  }
  logDataLine(time, data);
}

void DataLogger::log(Real time, Int timeStepCount) {
  if (!mEnabled || !(timeStepCount % mDownsampling == 0))
    return;

  if (mLogFile.tellp() == std::ofstream::pos_type(0)) {
    mLogFile << std::right << std::setw(14) << "time";
    for (auto it : mAttributes)
      mLogFile << ", " << std::right << std::setw(13) << it.first;
    mLogFile << '\n';
  }

  mLogFile << std::scientific << std::right << std::setw(14) << time;
  for (auto it : mAttributes)
    mLogFile << ", " << std::right << std::setw(13) << it.second->toString();
  mLogFile << '\n';
}

void DataLogger::Step::execute(Real time, Int timeStepCount) { mLogger.log(time, timeStepCount); }

CPS::Task::Ptr DataLogger::getTask() {
  return std::make_shared<DataLogger::Step>(*this);
}
