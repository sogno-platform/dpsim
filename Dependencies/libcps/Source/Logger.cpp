/**
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

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

#include <iomanip>

#include <cps/Logger.h>
#include "spdlog/sinks/null_sink.h"

using namespace CPS;

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

String Logger::getCSVLineFromData(Real time, const Matrix& data) {
	std::stringstream ss;
    ss << std::scientific << std::right << std::setw(14) << time;
	for (Int i = 0; i < data.rows(); i++) {
		ss << ", " << std::right << std::setw(13) << data(i, 0);
	}
	ss << '\n';

	return ss.str();
}

String Logger::getCSVLineFromData(Real time, const MatrixComp& data) {
	std::stringstream ss;
    ss << std::scientific << std::right << std::setw(14) << time;
	for (Int i = 0; i < data.rows(); i++) {
		ss << ", " << std::right << std::setw(13) << data(i, 0);
	}
	ss << '\n';

	return ss.str();
}

Logger::Log Logger::get(const std::string &name, Level level) {
	Logger::Log logger = spdlog::get(name);

	if (!logger) {
		if (level != Logger::Level::off)
			logger = create(name, level);
		else
			logger = spdlog::create<spdlog::sinks::null_sink_st>(name);
	}

	return logger;
}

Logger::Log Logger::create(const std::string &name, Level level) {
	String logDir = Logger::logDir();
	String filename = logDir + "/" + name + ".log";

	// Create log folder if it does not exist
	fs::path p = filename;
	if (p.has_parent_path() && !fs::exists(p.parent_path()))
		fs::create_directories(p.parent_path());

	auto logger = spdlog::basic_logger_mt(name, filename, true);

	logger->set_level(level);
	logger->set_pattern(prefix() + "[%L] %v");

	return logger;
}
