#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <fstream>

#include "MathLibrary.h"

enum class Logtype { INFO, WARN, ERROR };

class Logger {
	private:
		std::string mLogFileName;
		std::ofstream mLogFile;
		std::ostringstream mLogStream;

	public:
		Logger();
		~Logger();
	
		int AddDataLine(double time, DPSMatrix data);
		std::ostringstream& Log();
		std::ostringstream& Log(Logtype type);
		void WriteLogToFile(std::string fileName);
		static std::ostringstream VectorToDataLine(double time, DPSMatrix vector);
};
#endif

