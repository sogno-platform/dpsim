/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <iomanip>
#include <memory>

#include <spdlog/sinks/null_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <dpsim-models/Filesystem.h>
#include <dpsim-models/Logger.h>

using namespace CPS;

void Logger::setLogLevel(std::shared_ptr<spdlog::logger> logger,
                         Logger::Level level) {
  logger->set_level(level);
}

void Logger::setLogPattern(std::shared_ptr<spdlog::logger> logger,
                           std::string pattern) {
  logger->set_pattern(pattern);
}

// #### to string methods ####
template <typename VarType>
String Logger::matrixVarToString(const MatrixVar<VarType> &mat) {
  std::stringstream ss;
  ss << std::scientific << "\n" << mat;
  return ss.str();
}
template String Logger::matrixVarToString<Real>(const MatrixVar<Real> &mat);
template String
Logger::matrixVarToString<Complex>(const MatrixVar<Complex> &mat);

String Logger::matrixToString(const Matrix &mat) {
  std::stringstream ss;
  ss << std::scientific << "\n" << mat;
  return ss.str();
}

String Logger::matrixCompToString(const MatrixComp &mat) {
  std::stringstream ss;
  ss << std::scientific << "\n" << mat;
  return ss.str();
}

String Logger::sparseMatrixToString(const SparseMatrix &mat) {
  return matrixToString(Matrix(mat));
}

String Logger::sparseMatrixCompToString(const SparseMatrixComp &mat) {
  return matrixCompToString(MatrixComp(mat));
}

String Logger::phasorMatrixToString(const MatrixComp &mat) {
  std::stringstream ss;
  ss << std::scientific << Math::abs(mat) << "\n\n" << Math::phase(mat);
  return ss.str();
}

String Logger::phasorToString(const Complex &num) {
  std::stringstream ss;
  ss << std::defaultfloat << Math::abs(num) << "<" << Math::phaseDeg(num);
  return ss.str();
}

String Logger::complexToString(const Complex &num) {
  std::stringstream ss;
  ss << std::defaultfloat << num.real() << "+j" << num.imag();
  return ss.str();
}

String Logger::realToString(const Real &num) {
  std::stringstream ss;
  ss << std::defaultfloat << num;
  return ss.str();
}

String Logger::prefix() {
  char *p = getenv("CPS_LOG_PREFIX");

  return p ? p : "";
}

String Logger::logDir() {
  char *p = getenv("CPS_LOG_DIR");

  return p ? p : "logs";
}

/// Set env variable CPS_LOG_DIR and overwrite
void Logger::setLogDir(String path) {
#ifdef __linux__
  setenv("CPS_LOG_DIR", path.c_str(), 1);
#elif defined(_WIN32)
  String var = "CPS_LOG_DIR=" + path;
  _putenv(var.c_str());
#endif
}

String Logger::getCSVColumnNames(std::vector<String> names) {
  std::stringstream ss;
  ss << std::right << std::setw(14) << "time";
  for (auto name : names) {
    ss << ", " << std::right << std::setw(13) << name;
  }
  ss << '\n';

  return ss.str();
}

String Logger::getCSVLineFromData(Real time, Real data) {
  std::stringstream ss;
  ss << std::scientific << std::right << std::setw(14) << time;
  ss << ", " << std::right << std::setw(13) << data;
  ss << '\n';

  return ss.str();
}

String Logger::getCSVLineFromData(Real time, const Matrix &data) {
  std::stringstream ss;
  ss << std::scientific << std::right << std::setw(14) << time;
  for (Int i = 0; i < data.rows(); i++) {
    ss << ", " << std::right << std::setw(13) << data(i, 0);
  }
  ss << '\n';

  return ss.str();
}

String Logger::getCSVLineFromData(Real time, const MatrixComp &data) {
  std::stringstream ss;
  ss << std::scientific << std::right << std::setw(14) << time;
  for (Int i = 0; i < data.rows(); i++) {
    ss << ", " << std::right << std::setw(13) << data(i, 0);
  }
  ss << '\n';

  return ss.str();
}

Logger::Log Logger::get(const std::string &name, Level filelevel,
                        Level clilevel) {
  Logger::Log logger = spdlog::get(name);

  if (!logger) {
    logger = create(name, filelevel, clilevel);
  }

  return logger;
}

Logger::Log Logger::create(const std::string &name, Level filelevel,
                           Level clilevel) {
  String logDir = Logger::logDir();
  String filename = logDir + "/" + name + ".log";
  std::vector<spdlog::sink_ptr> sinks;
  Logger::Log ret;

  // Create log folder if it does not exist
  fs::path p = filename;
  if (p.has_parent_path() && !fs::exists(p.parent_path()))
    fs::create_directories(p.parent_path());

  if (clilevel != Logger::Level::off) {
    auto console_sink = std::make_shared<spdlog::sinks::stderr_color_sink_mt>();
    console_sink->set_level(clilevel);
    console_sink->set_pattern(
        fmt::format("{}[%T.%f %n %^%l%$] %v", CPS::Logger::prefix()));
    sinks.push_back(console_sink);
  }

  if (filelevel != Logger::Level::off) {
    auto file_sink =
        std::make_shared<spdlog::sinks::basic_file_sink_mt>(filename, true);
    file_sink->set_level(filelevel);
    file_sink->set_pattern(prefix() + "[%L] %v");
    sinks.push_back(file_sink);
  }

  if (filelevel == Logger::Level::off && clilevel == Logger::Level::off) {
    ret = spdlog::create<spdlog::sinks::null_sink_st>(name);
  } else {
    ret = std::make_shared<spdlog::logger>(name, begin(sinks), end(sinks));
  }

  return ret;
}
