/**
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017-2018, Institute for Automation of Complex Power Systems, EONERC
 *
 * CPowerSystems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
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

#include <cps/Definitions.h>
#include <cps/MathUtils.h>

namespace CPS {

	class Logger {

	public:
		using Level = spdlog::level::level_enum;
		using Log = std::shared_ptr<spdlog::logger>;

	private:
		static Log create(const std::string &name, Level logLevel = Level::info);

	public:
		Logger();
		~Logger();

		static String prefix();
		static String logDir();
		static void setLogDir(String path);

		// #### SPD log wrapper ####
		///
		static Log get(const std::string &name, Level logLevel = Level::info);
		///
		static void setLogLevel(std::shared_ptr<spdlog::logger> logger, Logger::Level level) {
			logger->set_level(level);
		}
		///
		static void setLogPattern(std::shared_ptr<spdlog::logger> logger, std::string pattern) {
			logger->set_pattern(pattern);
		}

		// #### to string methods ####
		static String matrixToString(const Matrix& mat) {
			std::stringstream ss;
			ss << std::scientific << mat;
			return ss.str();
		}

		static String matrixCompToString(const MatrixComp& mat) {
			std::stringstream ss;
			ss << std::scientific << mat;
			return ss.str();
		}

		static String phasorMatrixToString(const MatrixComp& mat) {
			std::stringstream ss;
			ss << std::scientific << Math::abs(mat) << "\n\n" << Math::phase(mat);
			return ss.str();
		}

		static String phasorToString(const Complex& num) {
			return std::to_string(Math::abs(num)) + "<" + std::to_string(Math::phaseDeg(num));
		}

		static String complexToString(const Complex& num) {
			return std::to_string(num.real()) + "+j" + std::to_string(num.imag());
		}

		static String getCSVColumnNames(std::vector<String> names);
		static String getCSVLineFromData(Real time, Real data);
		static String getCSVLineFromData(Real time, const Matrix& data);
		static String getCSVLineFromData(Real time, const MatrixComp& data);
	};
}
