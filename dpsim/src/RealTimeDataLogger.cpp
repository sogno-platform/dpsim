/* A data logger for real-time simulation data logging.
 *
 * Author: Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-FileCopyrightText: 2024 Niklas Eiling <niklas.eiling@eonerc.rwth-aachen.de>
 * SPDX-License-Identifier: Apache-2.0
 */
#include "dpsim-models/Attribute.h"
#include <iomanip>

#include <dpsim-models/Logger.h>
#include <dpsim/RealTimeDataLogger.h>
#include <memory>

using namespace DPsim;

RealTimeDataLogger::RealTimeDataLogger(std::filesystem::path &filename,
                                       size_t rowNumber)
    : DataLoggerInterface(), mFilename(filename), mRowNumber(rowNumber),
      mCurrentRow(0), mCurrentAttribute(0), mAttributeData() {}

RealTimeDataLogger::RealTimeDataLogger(std::filesystem::path &filename,
                                       Real finalTime, Real timeStep)
    : DataLoggerInterface(), mFilename(filename),
      mRowNumber((finalTime / timeStep + 0.5)), mCurrentRow(0),
      mCurrentAttribute(0), mAttributeData() {}

void RealTimeDataLogger::start() {
  double mb_size =
      static_cast<double>(mRowNumber) * (mAttributes.size() + 1) * sizeof(Real);
  auto log = CPS::Logger::get("RealTimeDataLogger", CPS::Logger::Level::off,
                              CPS::Logger::Level::info);
  log->info("Preallocating memory for real-time data logger: {} rows for {} "
            "attributes ({} MB)",
            mRowNumber, mAttributes.size(), mb_size / (1024 * 1024));
  // We are doing real time so preallocate everything
  mAttributeData.resize(mRowNumber);
  for (auto &it : mAttributeData) {
    // We have to add one to the size because we also log the time
    it.resize(mAttributes.size() + 1);
  }
}

void RealTimeDataLogger::stop() {
  auto mLogFile =
      std::ofstream(mFilename, std::ios_base::out | std::ios_base::trunc);
  if (!mLogFile.is_open()) {
    throw std::runtime_error("Cannot open log file " + mFilename.string());
  }

  mLogFile << std::right << std::setw(14) << "time";
  for (auto it : mAttributes)
    mLogFile << ", " << std::right << std::setw(13) << it.first;
  mLogFile << '\n';

  for (auto row : mAttributeData) {
    mLogFile << std::scientific << std::right << std::setw(14) << row[0];
    for (size_t i = 1; i < row.size(); ++i)
      mLogFile << ", " << std::right << std::setw(13) << row[i];
    mLogFile << '\n';
  }
  mLogFile.close();
}

void RealTimeDataLogger::log(Real time, Int timeStepCount) {
  mCurrentRow = timeStepCount;
  if (timeStepCount < 0 || static_cast<size_t>(timeStepCount) >= mRowNumber) {
    throw std::runtime_error(
        "RealTimeDataLogger: timeStepCount out of bounds. Please verify the "
        "logger was initialized correctly.");
  }
  if (mAttributeData.size() != mRowNumber ||
      mAttributeData[mCurrentRow].size() != mAttributes.size() + 1) {
    throw std::runtime_error(
        "RealTimeDataLogger: Attribute data size mismatch");
  }
  mAttributeData[mCurrentRow][0] = time;
  mCurrentAttribute = 1;

  for (auto it : mAttributes) {
    if (it.second->getType() == typeid(Real)) {
      mAttributeData[mCurrentRow][mCurrentAttribute++] =
          **std::dynamic_pointer_cast<std::shared_ptr<CPS::Attribute<Real>>>(
              it.second.getPtr());
    } else if (it.second->getType() == typeid(Int)) {
      mAttributeData[mCurrentRow][mCurrentAttribute++] =
          **std::dynamic_pointer_cast<std::shared_ptr<CPS::Attribute<Int>>>(
              it.second.getPtr());
    }
  }
}

void RealTimeDataLogger::Step::execute(Real time, Int timeStepCount) {
  mLogger.log(time, timeStepCount);
}

CPS::Task::Ptr RealTimeDataLogger::getTask() {
  return std::make_shared<RealTimeDataLogger::Step>(*this);
}
