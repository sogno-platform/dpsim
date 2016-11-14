#include <iostream>
#include <fstream>

#include "MathLibrary.h"

class Log {
	private:
		std::string dataFile;
		std::string logFile;
		std::ofstream dataStream;
		std::ofstream logStream;

	public:
		Log(std::string dataFile, std::string logFile);
		~Log();
	
		int AddDataLine(double time, DPSMatrix data);
};
