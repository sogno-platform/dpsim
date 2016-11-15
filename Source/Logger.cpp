#include "Logger.h"

Logger::Logger() {

}

Logger::~Logger() {

}

int Logger::AddDataLine(double time, DPSMatrix data) {
	mLogStream << std::scientific << time;
	for (int i = 0; i < data.rows(); i++) {
		mLogStream << ", " << data(i, 0);
	}
	mLogStream << std::endl;

	return 0;
}

std::ostringstream& Logger::Log() {
	return mLogStream;
}

std::ostringstream& Logger::Log(Logtype type) {
	switch (type) {
		case Logtype::INFO:
			mLogStream << "INFO: ";
			break;
		case Logtype::WARN:
			mLogStream << "WARN: ";
			break;
		case Logtype::ERROR:
			mLogStream << "ERROR: ";
			break;
	}
	return mLogStream;
}

void Logger::WriteLogToFile(std::string fileName) {
	mLogFileName = fileName;
	mLogFile.open(mLogFileName);

	if (!mLogFile.is_open()) {
		std::cout << "Cannot open log file" << mLogFileName << std::endl;
	}

	mLogFile << mLogStream.str();

	mLogFile.close();
}
