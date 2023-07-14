// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <dpsim-models/Config.h>

#if defined(DEBUG_BUILD)
	#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#else
	#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_INFO
#endif
#define SPDLOG_DISABLE_DEFAULT_LOGGER

#include <spdlog/spdlog.h>

#if defined(SPDLOG_VER_MAJOR) && SPDLOG_VER_MAJOR >= 1
  #include <spdlog/sinks/basic_file_sink.h>
#else
  #include <spdlog/sinks/file_sinks.h>
#endif

#include <spdlog/fmt/ostr.h>
#include <spdlog/sinks/null_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include "spdlog/sinks/base_sink.h"

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

		/// Holds the file sink shared by all simulation loggers
		static spdlog::sink_ptr mSimulationFileSink;
		/// Holds the file sink shared by all component loggers
		static spdlog::sink_ptr mComponentFileSink;
		/// Holds the stdout cli sink shared by all loggers
		static spdlog::sink_ptr mStdoutSink;
		/// Holds the stderr cli sink shared by all loggers
		static spdlog::sink_ptr mStderrSink;

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

		class dpsim_sink : public spdlog::sinks::base_sink <std::mutex>
		{
		private:
			spdlog::sink_ptr mFileSink;
			spdlog::sink_ptr mStdoutSink;
			spdlog::sink_ptr mStderrSink;
			Level mFileLevel;
			Level mCliLevel;
		public:
			dpsim_sink(spdlog::sink_ptr fileSink, spdlog::sink_ptr stdoutSink, spdlog::sink_ptr stderrSink, Level fileLevel, Level cliLevel) :
				spdlog::sinks::base_sink<std::mutex>(),
				mFileSink(fileSink),
				mStdoutSink(stdoutSink),
				mStderrSink(stderrSink),
				mFileLevel(fileLevel),
				mCliLevel(cliLevel) { };
		protected:
			void sink_it_(const spdlog::details::log_msg& msg) override
			{
				if (mFileSink && msg.level >= mFileLevel) {
					mFileSink->log(msg);
				}
				if (mStdoutSink && msg.level >= mCliLevel && msg.level < Level::warn) {
					mStdoutSink->log(msg);
				}
				if (mStderrSink && msg.level >= mCliLevel && msg.level >= Level::warn) {
					mStderrSink->log(msg);
				}
			}

			void flush_() override
			{
				if (mFileSink) {
					mFileSink->flush();
				}
				if (mStdoutSink) {
					mStdoutSink->flush();
				}
				if (mStderrSink) {
					mStderrSink->flush();
				}
			}
		};
	};
}
