/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <memory>

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO
#include <spdlog/spdlog.h>

#if defined(SPDLOG_VER_MAJOR) && SPDLOG_VER_MAJOR >= 1
#include <spdlog/sinks/basic_file_sink.h>
#else
#include <spdlog/sinks/file_sinks.h>
#endif

#include <spdlog/fmt/ostr.h>

#include <dpsim-models/Attribute.h>
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

#if FMT_VERSION >= 90000
template <>
class fmt::formatter<CPS::String> : public fmt::ostream_formatter {};
template <>
class fmt::formatter<CPS::Complex> : public fmt::ostream_formatter {};
template <>
class fmt::formatter<CPS::Vector> : public fmt::ostream_formatter {};
template <>
class fmt::formatter<CPS::VectorComp> : public fmt::ostream_formatter {};
template <>
class fmt::formatter<CPS::SparseMatrix> : public fmt::ostream_formatter {};
template <>
class fmt::formatter<CPS::SparseMatrixRow> : public fmt::ostream_formatter {};
template <>
class fmt::formatter<CPS::SparseMatrixComp> : public fmt::ostream_formatter {};
template <>
class fmt::formatter<CPS::SparseMatrixCompRow> : public fmt::ostream_formatter {
};
template <>
class fmt::formatter<CPS::Matrix> : public fmt::ostream_formatter {};
template <>
class fmt::formatter<CPS::MatrixComp> : public fmt::ostream_formatter {};
template <>
class fmt::formatter<CPS::MatrixInt> : public fmt::ostream_formatter {};
template <>
class fmt::formatter<CPS::MatrixRow> : public fmt::ostream_formatter {};
template <>
class fmt::formatter<CPS::LUFactorized> : public fmt::ostream_formatter {};
template <>
class fmt::formatter<CPS::LUFactorizedSparse> : public fmt::ostream_formatter {
};
template <typename VarType>
class fmt::formatter<CPS::MatrixVar<VarType>> : public fmt::ostream_formatter {
};
template <int rows, int cols>
class fmt::formatter<CPS::MatrixFixedSize<rows, cols>>
    : public fmt::ostream_formatter {};
template <int rows, int cols>
class fmt::formatter<CPS::MatrixFixedSizeComp<rows, cols>>
    : public fmt::ostream_formatter {};

template <>
class fmt::formatter<Eigen::Block<CPS::Matrix>>
    : public fmt::ostream_formatter {};
template <>
class fmt::formatter<Eigen::Block<CPS::MatrixComp>>
    : public fmt::ostream_formatter {};

namespace fmt {
template <typename T> struct formatter<CPS::Attribute<T>> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const CPS::Attribute<T> &attr, FormatContext &ctx) const {
    auto &nc = const_cast<CPS::Attribute<T> &>(attr);
    return fmt::format_to(ctx.out(), "{}", nc.get());
  }
};

template <typename T> struct formatter<std::shared_ptr<CPS::Attribute<T>>> {
  template <typename ParseContext> constexpr auto parse(ParseContext &ctx) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const std::shared_ptr<CPS::Attribute<T>> &p,
              FormatContext &ctx) const {
    if (!p)
      return fmt::format_to(ctx.out(), "<null>");
    auto &nc = const_cast<CPS::Attribute<T> &>(*p);
    return fmt::format_to(ctx.out(), "{}", nc.get());
  }
};

} // namespace fmt

#endif
