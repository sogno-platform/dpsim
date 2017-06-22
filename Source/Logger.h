#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <fstream>

#include "MathLibrary.h"

enum class LogLevel { NONE, ERROR, WARN, INFO };

class Logger {
	private:
		std::ofstream mLogFile;
		LogLevel mLogLevel;

		static std::ostringstream nullStream;
		static std::ostream& getNullStream();

	public:
		Logger();
		Logger(std::string filename, LogLevel level = LogLevel::INFO);
		~Logger();
	
		std::ostream& Log(LogLevel level = LogLevel::INFO);
		void LogDataLine(double time, DPSMatrix& data);
};
#endif

