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
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include <dpsim-models/DP/DP_Ph1_AvVoltageSourceInverterDQ.h>
#include <dpsim-models/DP/DP_Ph1_PQLoadCS.h>
#include <dpsim-models/Filesystem.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/SP/SP_Ph1_AvVoltageSourceInverterDQ.h>
#include <dpsim-models/SP/SP_Ph1_Load.h>
#include <dpsim-models/SystemTopology.h>

namespace CPS {
/// reads load profiles (csv files only) and assign them to the corresponding load object
class CSVReader {
private:
  /// Logger
  Logger::Log mSLog;
  /// path of load profile files (csv file)
  String mPath;
  /// list of load profile files with path
  std::list<fs::path> mFileList;
  /// assign pattern, used when the MANUAL mode is selected
  std::map<String, String> mAssignPattern;
  /// Skip first row if it has no digits at beginning
  Bool mSkipFirstRow = true;

public:
  /// set load profile assigning pattern. AUTO for assigning load profile name (csv file name) to load object with the same name (mName)
  /// MANUAL for providing an assign pattern manually. see power flow example: CIM/CIGRE_MV_PowerFlowTest_LoadProfiles.cpp
  enum class Mode { AUTO, MANUAL };

  /* Time Stamp Format.
   *
   *  HHMMSS:  Hours : Minutes : Seconds, it be casted to the corresponding SECONDS.
   *  SECONDS: profiles recorded with total seconds.
   *  PVGEN: format comply with https://www.fein-aachen.org/projects/PVgenerator/
   */
  enum class DataFormat { HHMMSS, SECONDS, HOURS, MINUTES };

  ///
  CSVReader(String name, std::list<fs::path> path, Logger::Level logLevel);
  ///
  CSVReader(String name, String path, Logger::Level logLevel);
  ///
  CSVReader(String name, std::list<fs::path> path,
            std::map<String, String> &assignList, Logger::Level logLevel);
  ///
  CSVReader(String name, String path, std::map<String, String> &assignList,
            Logger::Level logLevel);

  /// Convert HH:MM:SS format timestamp into total seconds.
  ///  e.g.: 00 : 01 : 00 -- > 60.
  Real time_format_convert(const String &time);
  /// Skip first row if it has no digits at beginning
  void doSkipFirstRow(Bool value = true) { mSkipFirstRow = value; }
  ///
  MatrixRow csv2Eigen(const String &path);

  std::vector<PQData> readLoadProfileDP(
      fs::path file, Real start_time = -1, Real time_step = 1,
      Real end_time = -1, Real scale_factor = 1,
      CSVReader::DataFormat format = CSVReader::DataFormat::SECONDS);

  void assignLoadProfileDP(
      std::vector<std::shared_ptr<CPS::DP::Ph1::AvVoltageSourceInverterDQ>>
          &loads,
      Real start_time = -1, Real time_step = 1, Real end_time = -1,
      Real scale_factor = 1, CSVReader::Mode mode = CSVReader::Mode::AUTO,
      CSVReader::DataFormat format = CSVReader::DataFormat::SECONDS);

  /// TODO : deprecate in the future
  /// read in load profile with time stamp format specified
  PowerProfile readLoadProfile(
      fs::path file, Real start_time = -1, Real time_step = 1,
      Real end_time = -1,
      CSVReader::DataFormat format = CSVReader::DataFormat::SECONDS);
  ///
  std::vector<Real>
  readPQData(fs::path file, Real start_time = -1, Real time_step = 1,
             Real end_time = -1,
             CSVReader::DataFormat format = CSVReader::DataFormat::SECONDS);
  /// assign load profile to corresponding load object
  void assignLoadProfile(
      SystemTopology &sys, Real start_time = -1, Real time_step = 1,
      Real end_time = -1, CSVReader::Mode mode = CSVReader::Mode::AUTO,
      CSVReader::DataFormat format = CSVReader::DataFormat::SECONDS);
  ///
  void assignPVGeneration(SystemTopology &sys, Real start_time = -1,
                          Real time_step = 1, Real end_time = -1,
                          CSVReader::Mode mode = CSVReader::Mode::AUTO);

  /// interpolation for PQ data points
  PQData interpol_linear(std::map<Real, PQData> &data_PQ, Real x);

  /// interpolation for weighting factor data points
  Real interpol_linear(std::map<Real, Real> &data_wf, Real x);
};

// #### csv reader section
class CSVRow {
public:
  ///
  String const &get(std::size_t index) const { return m_data[index]; }
  ///
  Int size() const;
  ///
  void readNextRow(std::istream &str);

private:
  ///
  std::vector<String> m_data;
};

class CSVReaderIterator {
public:
  CSVReaderIterator(std::istream &str);
  CSVReaderIterator();

  CSVReaderIterator &next();
  CSVReaderIterator next(Int);
  CSVReaderIterator &step(Int time_step);
  CSVRow const &operator*() const { return m_row; };
  Bool operator==(CSVReaderIterator const &rhs) {
    return ((this == &rhs) || ((this->m_str == NULL) && (rhs.m_str == NULL)));
  }
  Bool operator!=(CSVReaderIterator const &rhs) { return !((*this) == rhs); }

private:
  std::istream *m_str;
  CSVRow m_row;
};
} // namespace CPS
