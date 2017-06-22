#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <fstream>

#include "MathLibrary.h"

enum class LogLevel { NONE, ERROR, WARN, INFO };

class Logger {
	private:
		std::ofstream mLogFile;
		std::ostringstream mNullStream;
		LogLevel mLogLevel;

	public:
		Logger(std::string filename, LogLevel level = LogLevel::INFO);
		~Logger();
	
		std::ostream& Log(LogLevel level = LogLevel::INFO);
		void LogDataLine(double time, DPSMatrix& data);
};
#endif

