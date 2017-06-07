#include "Logger.h"

Logger::Logger() {
	mLogLevel = LogLevel::INFO;
}

Logger::Logger(LogLevel level) {
	mLogLevel = level;
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

std::ostringstream& Logger::Log(LogLevel level) {
	if (level > mLogLevel) {
		mNullStream.str("");
		return mNullStream;
	}

	switch (level) {
		case LogLevel::INFO:		
			mLogStream << "INFO: ";
			break;
		case LogLevel::WARN:
			mLogStream << "WARN: ";
			break;
		case LogLevel::ERROR:
			mLogStream << "ERROR: ";
			break;
		case LogLevel::NONE:
			mNullStream.str("");
			return mNullStream;
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

std::ostringstream Logger::VectorToDataLine(double time, DPSMatrix vector) {
	std::ostringstream output;
	output << std::scientific << time;
	for (int i = 0; i < vector.rows(); i++) {
		output << ", " << vector(i, 0);
	}
	output << std::endl;
	return output;
}