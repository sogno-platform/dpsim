/** Logger
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
 *
 * DPsim
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

#include "Logger.h"

std::ostringstream Logger::nullStream;

Logger::Logger() : mLogFile() {
	mLogLevel = LogLevel::NONE;
	mLogFile.setstate(std::ios_base::badbit);
}

Logger::Logger(std::string filename, LogLevel level) : mLogFile(filename), mLogLevel(level) {
	if (!mLogFile.is_open()) {
		std::cerr << "Cannot open log file " << filename << std::endl;
		mLogLevel = LogLevel::NONE;
	}
}

Logger::~Logger() {
	if (mLogFile.is_open())
		mLogFile.close();
}

std::ostream& Logger::Log(LogLevel level) {
	if (level > mLogLevel) {
		return getNullStream();
	}

	switch (level) {
		case LogLevel::INFO:
			mLogFile << "INFO: ";
			break;
		case LogLevel::WARN:
			mLogFile << "WARN: ";
			break;
		case LogLevel::ERROR:
			mLogFile << "ERROR: ";
			break;
		case LogLevel::NONE:
			return getNullStream();
			break;
	}
	return mLogFile;
}

void Logger::LogDataLine(double time, DPSMatrix& data) {
	mLogFile << std::scientific << time;
	for (int i = 0; i < data.rows(); i++) {
		mLogFile << ", " << data(i, 0);
	}
	mLogFile << std::endl;
}

void Logger::LogDataLine(double time, double data) {
	mLogFile << std::scientific << time;
	mLogFile << ", " << data;
	mLogFile << std::endl;
}
std::ostream& Logger::getNullStream() {
	if (nullStream.good())
		nullStream.setstate(std::ios_base::badbit);
	return nullStream;
}
