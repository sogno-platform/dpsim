/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/MNASolverPlugin.h>
#include <dpsim/SequentialScheduler.h>
#include <Eigen/Eigen>
#include <dlfcn.h>

using namespace DPsim;
using namespace CPS;

namespace DPsim {

template <typename VarType>
MnaSolverPlugin<VarType>::MnaSolverPlugin(String pluginName,
	String name, CPS::Domain domain, 
	std::shared_ptr<SolverParametersMNA> solverParams, CPS::Logger::Level logLevel) :
    MnaSolverDirect<VarType>(name, domain, solverParams, logLevel),
	mPluginName(pluginName),
	mPlugin(nullptr),
	mDlHandle(nullptr)
{
}

template <typename VarType>
MnaSolverPlugin<VarType>::~MnaSolverPlugin() {
	if (mPlugin != nullptr) {
		mPlugin->cleanup();
	}
	if (mDlHandle != nullptr) {
		dlclose(mDlHandle);
	}
}

extern "C" void pluginLogger(const char * str)
{
	CPS::Logger::Log log = CPS::Logger::get("Plugin", CPS::Logger::Level::debug, CPS::Logger::Level::debug);
	log->info(str);
}

template <typename VarType>
void MnaSolverPlugin<VarType>::recomputeSystemMatrix(Real time) {
	// Start from base matrix
	this->mVariableSystemMatrix = this->mBaseSystemMatrix;

	// Now stamp switches into matrix
	for (auto sw : this->mMNAIntfSwitches)
		sw->mnaApplySystemMatrixStamp(this->mVariableSystemMatrix);

	// Now stamp variable elements into matrix
	for (auto comp : this->mMNAIntfVariableComps)
		comp->mnaApplySystemMatrixStamp(this->mVariableSystemMatrix);

    int size = this->mRightSideVector.rows();
	int nnz = this->mVariableSystemMatrix.nonZeros();
	struct dpsim_csr_matrix matrix = {
		.values = this->mVariableSystemMatrix.valuePtr(),
		.rowIndex = this->mVariableSystemMatrix.outerIndexPtr(),
		.colIndex = this->mVariableSystemMatrix.innerIndexPtr(),
		.row_number = size,
		.nnz = nnz,
	};
	// Refactorization of matrix assuming that structure remained
	// constant by omitting analyzePattern
	if (mPlugin->lu_decomp(&matrix) != 0) {
		SPDLOG_LOGGER_ERROR(this->mSLog, "error recomputing decomposition");
		return;
	}
	++this->mNumRecomputations;
}

template <typename VarType>
void MnaSolverPlugin<VarType>::initialize() {
    MnaSolver<VarType>::initialize();
    int size = this->mRightSideVector.rows();
	auto hMat = this->mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)];
	int nnz = hMat[0].nonZeros();

	struct dpsim_mna_plugin* (*get_mna_plugin)(const char *);

	String pluginFileName = mPluginName + ".so";

	if ((mDlHandle = dlopen(pluginFileName.c_str(), RTLD_NOW)) == nullptr) {
		SPDLOG_LOGGER_ERROR(this->mSLog, "error opening dynamic library {}: {}", mPluginName, dlerror());
		throw CPS::SystemError("error opening dynamic library.");
	}

	get_mna_plugin = (struct dpsim_mna_plugin* (*)(const char *)) dlsym(mDlHandle, "get_mna_plugin");
	if (get_mna_plugin == NULL) {
		SPDLOG_LOGGER_ERROR(this->mSLog, "error reading symbol from library {}: {}", mPluginName, dlerror());
		throw CPS::SystemError("error reading symbol from library.");
	}

	if ((mPlugin = get_mna_plugin(mPluginName.c_str())) == nullptr) {
		SPDLOG_LOGGER_ERROR(this->mSLog, "error getting plugin class");
		throw CPS::SystemError("error getting plugin class.");
	}

	mPlugin->log = pluginLogger;

	struct dpsim_csr_matrix matrix = {
		.values = hMat[0].valuePtr(),
		.rowIndex = hMat[0].outerIndexPtr(),
		.colIndex = hMat[0].innerIndexPtr(),
		.row_number = size,
		.nnz = nnz,
	};

	if (mPlugin->init(&matrix) != 0) {
		SPDLOG_LOGGER_ERROR(this->mSLog, "error initializing plugin");
		return;
	}
}

template <typename VarType>
Task::List MnaSolverPlugin<VarType>::getTasks() {
    Task::List l;

    for (auto comp : this->mMNAComponents) {
		for (auto task : comp->mnaTasks()) {
			l.push_back(task);
		}
	}
	for (auto node : this->mNodes) {
		for (auto task : node->mnaTasks())
			l.push_back(task);
	}
	// TODO signal components should be moved out of MNA solver
	for (auto comp : this->mSimSignalComps) {
		for (auto task : comp->getTasks()) {
			l.push_back(task);
		}
	}
	l.push_back(std::make_shared<MnaSolverPlugin<VarType>::SolveTask>(*this));
    l.push_back(std::make_shared<MnaSolverPlugin<VarType>::LogTask>(*this));
	return l;
}

template <typename VarType>
void MnaSolverPlugin<VarType>::solve(Real time, Int timeStepCount) {
    // Reset source vector
	this->mRightSideVector.setZero();

    // Add together the right side vector (computed by the components'
	// pre-step tasks)
	for (const auto &stamp : this->mRightVectorStamps)
		this->mRightSideVector += *stamp;

	if (!this->mIsInInitialization)
		this->updateSwitchStatus();

	mPlugin->solve((double*)this->mRightSideVector.data(), (double*)this->leftSideVector().data());

	// TODO split into separate task? (dependent on x, updating all v attributes)
	for (UInt nodeIdx = 0; nodeIdx < this->mNumNetNodes; ++nodeIdx)
		this->mNodes[nodeIdx]->mnaUpdateVoltage(**(this->mLeftSideVector));


	// Components' states will be updated by the post-step tasks
}


}
template class DPsim::MnaSolverPlugin<Real>;
template class DPsim::MnaSolverPlugin<Complex>;
