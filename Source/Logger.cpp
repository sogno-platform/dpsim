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

std::ostream& Logger::getNullStream() {
	if (nullStream.good())
		nullStream.setstate(std::ios_base::badbit);
	return nullStream;
}
