/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO
#include <spdlog/spdlog.h>

#if defined(SPDLOG_VER_MAJOR) && SPDLOG_VER_MAJOR >= 1
#include <spdlog/sinks/basic_file_sink.h>
#else
#include <spdlog/sinks/file_sinks.h>
#endif

#include <spdlog/fmt/ostr.h>

#include <dpsim-models/Definitions.h>
#include <dpsim-models/MathUtils.h>

namespace CPS {

class Logger {

public:
  using Level = spdlog::level::level_enum;
  using Log = std::shared_ptr<spdlog::logger>;

private:
  static Log create(const std::string &name, Level filelevel = Level::info,
                    Level clilevel = Level::off);

public:
  Logger();
  ~Logger();

  static String prefix();
  static String logDir();
  static void setLogDir(String path);

  // #### SPD log wrapper ####
  ///
  static Log get(const std::string &name, Level filelevel = Level::info,
                 Level clilevel = Level::off);
  ///
  static void setLogLevel(std::shared_ptr<spdlog::logger> logger,
                          Logger::Level level);
  ///
  static void setLogPattern(std::shared_ptr<spdlog::logger> logger,
                            std::string pattern);

  // #### to string methods ####
  static String matrixToString(const Matrix &mat);
  static String matrixCompToString(const MatrixComp &mat);
  static String sparseMatrixToString(const SparseMatrix &mat);
  static String sparseMatrixCompToString(const SparseMatrixComp &mat);
  static String phasorMatrixToString(const MatrixComp &mat);
  static String phasorToString(const Complex &num);
  static String complexToString(const Complex &num);
  static String realToString(const Real &num);

  static String getCSVColumnNames(std::vector<String> names);
  static String getCSVLineFromData(Real time, Real data);
  static String getCSVLineFromData(Real time, const Matrix &data);
  static String getCSVLineFromData(Real time, const MatrixComp &data);
};
} // namespace CPS
