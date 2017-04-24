#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <fstream>

#include "MathLibrary.h"

enum class LogLevel { NONE, ERROR, WARN, INFO };

class Logger {
	private:
		std::string mLogFileName;
		std::ofstream mLogFile;
		std::ostringstream mLogStream;
		std::ostringstream mNullStream;
		LogLevel mLogLevel;

	public:
		Logger();
		Logger(LogLevel level);
		~Logger();
	
		int AddDataLine(double time, DPSMatrix data);
		std::ostringstream& Log();
		std::ostringstream& Log(LogLevel level);
		void WriteLogToFile(std::string fileName);
		static std::ostringstream VectorToDataLine(double time, DPSMatrix vector);
};
#endif

