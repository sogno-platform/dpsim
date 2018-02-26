/** Logger
 *
 * @file
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

#pragma once

#include <iostream>
#include <fstream>

#include "Definitions.h"

namespace DPsim {

	class Logger {

	public:
		enum class Level { NONE, ERROR, WARN, INFO, DEBUG };

	private:
		std::ofstream mLogFile;
		Level mLogLevel;

		static std::ostringstream nullStream;
		static std::ostream& getNullStream();

	public:
		Logger();
		Logger(String filename, Level level = Level::INFO);
		~Logger();

		std::ostream& Log(Level level = Level::INFO);
		void Log(Level level, String str);
		void LogMatrix(Level level, Matrix& data);
		void LogMatrix(Level level, const Matrix& data);
		void LogDataLine(Real time, Matrix& data);
		void LogDataLine(Real time, Real data);
		void LogNodeValues(Real time, Matrix& data);
		void LogVBR(Real time, Matrix& data);
	};
}

