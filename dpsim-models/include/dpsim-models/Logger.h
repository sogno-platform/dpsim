// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dpsim-models/Config.h>

#if defined(DEBUG_BUILD)
	#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#else
	#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO
#endif


#include <spdlog/spdlog.h>

#if defined(SPDLOG_VER_MAJOR) && SPDLOG_VER_MAJOR >= 1
  #include <spdlog/sinks/basic_file_sink.h>
#else
  #include <spdlog/sinks/file_sinks.h>
#endif

#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/null_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <dpsim-models/Definitions.h>
#include <dpsim-models/MathUtils.h>

namespace CPS {

	class Logger {

	public:
		using Level = spdlog::level::level_enum;
		using Log = std::shared_ptr<spdlog::logger>;

		enum class LoggerType {
			SIMULATION,
			COMPONENT,
			DEBUG
		};

		/// Holds the CLI sink shared by all loggers
		static spdlog::sink_ptr mCliSink;
		/// Holds the file sink shared by all component loggers
		static spdlog::sink_ptr mComponentFileSink;

	private:
		static Log create(Logger::LoggerType type, const std::string &name, const std::string &fileName, Level filelevel, Level clilevel);

	public:
		Logger() = default;
		~Logger() = default;

		static String prefix();
		static String logDir();
		static void setLogDir(String path);

		// #### SPD log wrapper ####
		///
		static Log get(LoggerType type, const std::string &name, Level filelevel = Level::info, Level clilevel = Level::off);
		///
		static void setLogLevel(std::shared_ptr<spdlog::logger> logger, Logger::Level level);
		///
		static void setLogPattern(std::shared_ptr<spdlog::logger> logger, std::string pattern);

		// #### to string methods ####
		static String matrixToString(const Matrix& mat);
		static String matrixCompToString(const MatrixComp& mat);
		static String sparseMatrixToString(const SparseMatrix& mat);
		static String sparseMatrixCompToString(const SparseMatrixComp& mat);
		static String phasorMatrixToString(const MatrixComp& mat);
		static String phasorToString(const Complex& num);
		static String complexToString(const Complex& num);
		static String realToString(const Real& num);

		static String getCSVColumnNames(std::vector<String> names);
		static String getCSVLineFromData(Real time, Real data);
		static String getCSVLineFromData(Real time, const Matrix& data);
		static String getCSVLineFromData(Real time, const MatrixComp& data);
	};
}
