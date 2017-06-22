#include "Logger.h"

Logger::Logger(std::string filename, LogLevel level) : mLogFile(filename), mLogLevel(level), mNullStream() {
	mNullStream.setstate(std::ios_base::badbit);
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
		mNullStream.str("");
		return mNullStream;
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
			return mNullStream;
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
