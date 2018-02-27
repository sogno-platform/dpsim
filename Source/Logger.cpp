/** Logger
 *
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
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

#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;

#include "Logger.h"
#include <iomanip>

using namespace DPsim;

std::ostringstream Logger::nullStream;

Logger::Logger() : mLogFile() {
	mLogLevel = Level::NONE;
	mLogFile.setstate(std::ios_base::badbit);
}

Logger::Logger(String filename, Level level) : mLogLevel(level) {
	if (mLogLevel == Level::NONE) {
		return;
	}

	fs::path p = filename;

	if (p.has_parent_path() && !fs::exists(p.parent_path()))
		fs::create_directory(p.parent_path());

	mLogFile = std::ofstream(filename);
	if (!mLogFile.is_open()) {
		std::cerr << "Cannot open log file " << filename << std::endl;
		mLogLevel = Level::NONE;
	}
}

Logger::~Logger() {
	if (mLogFile.is_open())
		mLogFile.close();
}

std::ostream& Logger::Log(Level level) {
	if (level > mLogLevel) {
		return getNullStream();
	}

	switch (level) {
		case Level::DEBUG:
			mLogFile << "DEBUG: ";
			break;
		case Level::INFO:
			mLogFile << "INFO: ";
			break;
		case Level::WARN:
			mLogFile << "WARN: ";
			break;
		case Level::ERROR:
			mLogFile << "ERROR: ";
			break;
		case Level::NONE:
			return getNullStream();
			break;
	}
	return mLogFile;
}

void Logger::Log(Level level, String str) {
	if (level > mLogLevel) {
		return;
	}

	switch (level) {
		case Level::DEBUG:
			mLogFile << "DEBUG: " << str << std::endl;
			break;
		case Level::INFO:
			mLogFile << "INFO: " << str << std::endl;
			break;
		case Level::WARN:
			mLogFile << "WARN: " << str << std::endl;
			break;
		case Level::ERROR:
			mLogFile << "ERROR: " << str << std::endl;
			break;
		case Level::NONE:
			return;
			break;
	}
}

void Logger::LogMatrix(Level level, Matrix& data) {
	if (level > mLogLevel) {
		return;
	}

	mLogFile << data << std::endl;
}

void Logger::LogMatrix(Level level, const Matrix& data) {
	if (level > mLogLevel) {
		return;
	}

	mLogFile << data << std::endl;
}

void Logger::LogDataLine(Real time, Matrix& data) {
	mLogFile << std::scientific << std::right << std::setw(14) << time;
	for (Int i = 0; i < data.rows(); i++) {
		mLogFile << ", " << std::right << std::setw(13) << data(i, 0);
	}
	mLogFile << std::endl;
}

void Logger::LogDataLine(Real time, Real data) {
	mLogFile << std::scientific << std::right << std::setw(14) << time;
	mLogFile << ", " << std::right << std::setw(13) << data;
	mLogFile << std::endl;
}

void Logger::LogNodeValues(Real time, Matrix& data) {
	if (mLogFile.tellp() == std::ofstream::pos_type(0)) {
		mLogFile << std::right << std::setw(14) << "time";
		for (Int i = 0; i < data.rows(); i++) {
			if (i < data.rows() / 2) {
				mLogFile << ", " << std::right << std::setfill(' ') << std::setw(13 - 4) << "NodeRe" << std::setfill('0') << std::setw(4) << i;
			}
			else {
				mLogFile << ", " << std::right << std::setfill(' ') << std::setw(13 - 4) << "NodeIm" << std::setfill('0') << std::setw(4) << (i - data.rows() / 2);
			}
		}
		mLogFile << std::endl;
	}
	LogDataLine(time, data);
}

void Logger::LogGen(Real time, Matrix& data) {
	if (mLogFile.tellp() == std::ofstream::pos_type(0)) {
			mLogFile << std::right << std::setw(14) << "time";
			mLogFile << ", " << std::right << std::setfill(' ') << std::setw(15 - 4) << "Ia" << std::setw(4);
			mLogFile << ", " << std::right << std::setfill(' ') << std::setw(15 - 4) << "Ib" << std::setw(4);
			mLogFile << ", " << std::right << std::setfill(' ') << std::setw(15 - 4) << "Ic" << std::setw(4);
			mLogFile << ", " << std::right << std::setfill(' ') << std::setw(15 - 4) << "Te" << std::setw(4);
			mLogFile << ", " << std::right << std::setfill(' ') << std::setw(15 - 4) << "omega" << std::setw(4);
			mLogFile << ", " << std::right << std::setfill(' ') << std::setw(16 - 4) << "Step Duration" << std::setw(4);
			mLogFile << std::endl;
	}
		LogDataLine(time, data);
}

void Logger::LogGenDP(Real time, Matrix& data) {
		if (mLogFile.tellp() == std::ofstream::pos_type(0)) {
				mLogFile << std::right << std::setw(14) << "time";
				mLogFile << ", " << std::right << std::setfill(' ') << std::setw(15 - 4) << "Ia_Re" << std::setw(4);
				mLogFile << ", " << std::right << std::setfill(' ') << std::setw(15 - 4) << "Ib_Re" << std::setw(4);
				mLogFile << ", " << std::right << std::setfill(' ') << std::setw(15 - 4) << "Ic_Re" << std::setw(4);
				mLogFile << ", " << std::right << std::setfill(' ') << std::setw(15 - 4) << "Ia_Im" << std::setw(4);
				mLogFile << ", " << std::right << std::setfill(' ') << std::setw(15 - 4) << "Ib_Im" << std::setw(4);
				mLogFile << ", " << std::right << std::setfill(' ') << std::setw(15 - 4) << "Ic_Im" << std::setw(4);
				mLogFile << ", " << std::right << std::setfill(' ') << std::setw(15 - 4) << "Te" << std::setw(4);
				mLogFile << ", " << std::right << std::setfill(' ') << std::setw(15 - 4) << "omega" << std::setw(4);
				mLogFile << ", " << std::right << std::setfill(' ') << std::setw(16 - 4) << "Step Duration" << std::setw(4);
				mLogFile << std::endl;
		}
		LogDataLine(time, data);
}

std::ostream& Logger::getNullStream()
{
	if (nullStream.good()) {
		nullStream.setstate(std::ios_base::badbit);
	}

	return nullStream;
}
