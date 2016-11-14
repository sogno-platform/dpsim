#include "Log.h"

Log::Log(std::string dataFile, std::string logFile) {
	this->dataFile = dataFile;
	this->logFile = logFile;
	dataStream.open(dataFile);
	logStream.open(logFile);
}

Log::~Log() {
	dataStream.close();
	logStream.close();
}

int Log::AddDataLine(double time, DPSMatrix data) {
	dataStream << std::scientific << time;
	for (int i = 0; i < data.rows(); i++) {
		dataStream << ", " << data(i, 0);
	}
	dataStream << std::endl;

	return 0;
}

