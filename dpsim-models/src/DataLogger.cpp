/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <iomanip>

#include <dpsim-models/DataLogger.h>
#include <dpsim-models/Logger.h>

using namespace CPS;

DataLogger::DataLogger(Bool enabled) :
	mLogFile(),
	mEnabled(enabled),
	mDownsampling(1) {
	mLogFile.setstate(std::ios_base::badbit);
}

DataLogger::DataLogger(String name, Bool enabled, UInt downsampling) :
	mName(name),
	mEnabled(enabled),
	mDownsampling(downsampling) {
	if (!mEnabled)
		return;

	mFilename = CPS::Logger::logDir() + "/" + name + ".csv";

	if (mFilename.has_parent_path() && !fs::exists(mFilename.parent_path()))
		fs::create_directory(mFilename.parent_path());
}

void DataLogger::open() {
	if (!mEnabled) {
		return;
	}
	mLogFile = std::ofstream(mFilename, std::ios_base::out|std::ios_base::trunc);
	if (!mLogFile.is_open()) {
		// TODO: replace by exception
		std::cerr << "Cannot open log file " << mFilename << std::endl;
		mEnabled = false;
	}
}

void DataLogger::close() {
	if (mLogFile) {
		mLogFile.close();
	}
}

void DataLogger::setColumnNames(std::vector<String> names) {
	if (mLogFile.tellp() == std::ofstream::pos_type(0)) {
		mLogFile << std::right << std::setw(14) << "time";
		for (auto name : names) {
			mLogFile << ", " << std::right << std::setw(13) << name;
		}
		mLogFile << '\n';
	}
}

void DataLogger::logDataLine(Real time, Real data) {
	if (!mEnabled)
		return;

	mLogFile << std::scientific << std::right << std::setw(14) << time;
	mLogFile << ", " << std::right << std::setw(13) << data;
	mLogFile << '\n';
}

void DataLogger::logDataLine(Real time, std::vector<Real> data) {
	if (!mEnabled)
		return;

	mLogFile << std::scientific << std::right << std::setw(14) << time;
	for (const auto& it : data)
		mLogFile << ", " << std::right << std::setw(13) << it;
	mLogFile << '\n';
}

void DataLogger::logDataLine(Real time, const Matrix& data) {
	if (!mEnabled)
		return;

	mLogFile << std::scientific << std::right << std::setw(14) << time;
	for (Int i = 0; i < data.rows(); ++i) {
		mLogFile << ", " << std::right << std::setw(13) << data(i, 0);
	}
	mLogFile << '\n';
}

void DataLogger::logDataLine(Real time, const MatrixComp& data) {
	if (!mEnabled)
		return;
	mLogFile << std::scientific << std::right << std::setw(14) << time;
	for (Int i = 0; i < data.rows(); ++i) {
		mLogFile << ", " << std::right << std::setw(13) << data(i, 0);
	}
	mLogFile << '\n';
}

void DataLogger::logPhasorNodeValues(Real time, const Matrix& data, Int freqNum) {
	if (mLogFile.tellp() == std::ofstream::pos_type(0)) {
		std::vector<String> names;

		Int harmonicOffset = data.rows() / freqNum;
		Int complexOffset = harmonicOffset / 2;

		for (Int freq = 0; freq < freqNum; ++freq) {
			for (Int node = 0; node < complexOffset; ++node) {
				std::stringstream name;
				name << "n" << std::setfill('0') << std::setw(5) << node
						<< "f" << std::setfill('0') << std::setw(2) << freq << ".re";
				names.push_back(name.str());
			}
			for (Int node = 0; node < complexOffset; ++node) {
				std::stringstream name;
				name << "n" << std::setfill('0') << std::setw(5) << node
						<< "f" << std::setfill('0') << std::setw(2) << freq << ".im";
				names.push_back(name.str());
			}
		}
		setColumnNames(names);
	}
	logDataLine(time, data);
}

void DataLogger::logEMTNodeValues(Real time, const Matrix& data) {
	if (mLogFile.tellp() == std::ofstream::pos_type(0)) {
		std::vector<String> names;
		for (Int i = 0; i < data.rows(); ++i) {
			std::stringstream name;
			name << "node" << std::setfill('0') << std::setw(5) << i;
			names.push_back(name.str());
		}
		setColumnNames(names);
	}
	logDataLine(time, data);
}

void DataLogger::log(Real time, Int timeStepCount) {
	if (!mEnabled || !(timeStepCount % mDownsampling == 0))
		return;

	if (mLogFile.tellp() == std::ofstream::pos_type(0)) {
		mLogFile << std::right << std::setw(14) << "time";
		for (auto it : mAttributes)
			mLogFile << ", " << std::right << std::setw(13) << it.first;
		mLogFile << '\n';
	}

	mLogFile << std::scientific << std::right << std::setw(14) << time;
	for (auto it : mAttributes)
		mLogFile << ", " << std::right << std::setw(13) << it.second->toString();
	mLogFile << '\n';
}

void DataLogger::Step::execute(Real time, Int timeStepCount) {
	mLogger.log(time, timeStepCount);
}

CPS::Task::Ptr DataLogger::getTask() {
	return std::make_shared<DataLogger::Step>(*this);
}

void DataLogger::logAttribute(const std::vector<String> &name, CPS::AttributeBase::Ptr attr) {
	if (auto attrMatrix = std::dynamic_pointer_cast<CPS::Attribute<Matrix>>(attr.getPtr())) {
		if ((**attrMatrix).rows() == 1 && (**attrMatrix).cols() == 1) {
			logAttribute(name[0], attrMatrix->deriveCoeff<CPS::Real>(0, 0));
		}
		else if ((**attrMatrix).cols() == 1) {
			for (UInt k = 0; k < (**attrMatrix).rows(); ++k) {
				logAttribute(name[k],
					attrMatrix->deriveCoeff<CPS::Real>(k, 0));
			}
		}
		else {
			for (UInt k = 0; k < (**attrMatrix).rows(); ++k) {
				for (UInt l = 0; l < (**attrMatrix).cols(); ++l) {
					logAttribute(name[k*(**attrMatrix).cols()+l],
						attrMatrix->deriveCoeff<CPS::Real>(k, l));
				}
			}
		}
	} else if (auto attrMatrix = std::dynamic_pointer_cast<CPS::Attribute<MatrixComp>>(attr.getPtr())) {
		if ((**attrMatrix).rows() == 1 && (**attrMatrix).cols() == 1) {
			logAttribute(name[0], attrMatrix->deriveCoeff<CPS::Complex>(0, 0));
		}
		else if ((**attrMatrix).cols() == 1) {
			for (UInt k = 0; k < (**attrMatrix).rows(); ++k) {
				logAttribute(name[k],
					attrMatrix->deriveCoeff<CPS::Complex>(k, 0));
			}
		}
		else {
			for (UInt k = 0; k < (**attrMatrix).rows(); ++k) {
				for (UInt l = 0; l < (**attrMatrix).cols(); ++l) {
					logAttribute(name[k*(**attrMatrix).cols()+l],
						attrMatrix->deriveCoeff<CPS::Complex>(k, l));
				}
			}
		}
	}
}

void DataLogger::logAttribute(const String &name, CPS::AttributeBase::Ptr attr, UInt rowsMax, UInt colsMax) {
	if (auto attrReal = std::dynamic_pointer_cast<CPS::Attribute<Real>>(attr.getPtr())) {
		mAttributes[name] = attrReal;
	} else if (auto attrComp = std::dynamic_pointer_cast<CPS::Attribute<Complex>>(attr.getPtr())) {
		mAttributes[name + ".re"] = attrComp->deriveReal();
		mAttributes[name + ".im"] = attrComp->deriveImag();
	} else if (auto attrMatrix = std::dynamic_pointer_cast<CPS::Attribute<Matrix>>(attr.getPtr())) {
		UInt rows = static_cast<UInt>((**attrMatrix).rows());
		UInt cols = static_cast<UInt>((**attrMatrix).cols());
		if (rowsMax == 0 || rowsMax > rows) rowsMax = rows;
		if (colsMax == 0 || colsMax > cols) colsMax = cols;
		if (rows == 1 && cols == 1) {
			mAttributes[name] = attrMatrix->deriveCoeff<Real>(0,0);
		} else if (cols == 1) {
			for (UInt k = 0; k < rowsMax; ++k) {
				mAttributes[name + "_" + std::to_string(k)] = attrMatrix->deriveCoeff<Real>(k,0);
			}
		} else {
			for (UInt k = 0; k < rowsMax; ++k) {
				for (UInt l = 0; l < colsMax; ++l) {
					mAttributes[name + "_" + std::to_string(k) + "_" + std::to_string(l)] = attrMatrix->deriveCoeff<Real>(k,l);
				}
			}
		}
	} else if (auto attrMatrix = std::dynamic_pointer_cast<CPS::Attribute<MatrixComp>>(attr.getPtr())) {
		UInt rows = static_cast<UInt>((**attrMatrix).rows());
		UInt cols = static_cast<UInt>((**attrMatrix).cols());
		if (rowsMax == 0 || rowsMax > rows) rowsMax = rows;
		if (colsMax == 0 || colsMax > cols) colsMax = cols;
		if (rows == 1 && cols == 1) {
			mAttributes[name + ".re"] = attrMatrix->deriveCoeff<Complex>(0,0)->deriveReal();
			mAttributes[name + ".im"] = attrMatrix->deriveCoeff<Complex>(0,0)->deriveImag();
		} else if (cols == 1) {
			for (UInt k = 0; k < rowsMax; ++k) {
				mAttributes[name + "_" + std::to_string(k) + ".re"] = attrMatrix->deriveCoeff<Complex>(k,0)->deriveReal();
				mAttributes[name + "_" + std::to_string(k) + ".im"] = attrMatrix->deriveCoeff<Complex>(k,0)->deriveImag();
			}
		} else {
			for (UInt k = 0; k < rowsMax; ++k) {
				for (UInt l = 0; l < colsMax; ++l) {
					mAttributes[name + "_" + std::to_string(k) + "_" + std::to_string(l) + ".re"] = attrMatrix->deriveCoeff<Complex>(k,l)->deriveReal();
					mAttributes[name + "_" + std::to_string(k) + "_" + std::to_string(l) + ".im"] = attrMatrix->deriveCoeff<Complex>(k,l)->deriveImag();
				}
			}
		}
	} else {
		mAttributes[name] = attr;
	}
}
