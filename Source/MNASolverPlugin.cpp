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
	String name,
	CPS::Domain domain, CPS::Logger::Level logLevel) :
    MnaSolverEigenSparse<VarType>(name, domain, logLevel),
	mPluginName("plugin.so"),
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

void pluginLogger(const char * str)
{
	CPS::Logger::Log log = CPS::Logger::get("Plugin", CPS::Logger::Level::debug, CPS::Logger::Level::debug);
	log->info(str);
}

template <typename VarType>
void MnaSolverPlugin<VarType>::initialize() {
    MnaSolver<VarType>::initialize();
    int size = this->mRightSideVector.rows();
	auto hMat = this->mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)];
	size_t nnz = hMat[0].nonZeros();

	DPsimPlugin::MnaSolverDynInterface* (*getMNASolverPlugin)(const char *);


	if ((mDlHandle = dlopen(mPluginName.c_str(), RTLD_NOW)) == nullptr) {
		mSLog->error("error opening dynamic library {}: {}", mPluginName, dlerror());
		return;
	}

	getMNASolverPlugin = (DPsimPlugin::MnaSolverDynInterface* (*)(const char *)) dlsym(mDlHandle, "getMNASolverPlugin");
	if (getMNASolverPlugin == nullptr) {
		mSLog->error("error reading symbol from library {}: {}", mPluginName, dlerror());
		return;
	}

	if ((mPlugin = getMNASolverPlugin(mPluginName.c_str())) == nullptr) {
		mSLog->error("error getting plugin class");
		return;
	}

	mPlugin->set_logger(pluginLogger);

	if (mPlugin->initialize(size, nnz,
		hMat[0].valuePtr(),
		hMat[0].outerIndexPtr(),
		hMat[0].innerIndexPtr()) != 0) {
		mSLog->error("error initializing plugin");
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
		this->mNodes[nodeIdx]->mnaUpdateVoltage(this->mLeftSideVector);


	// Components' states will be updated by the post-step tasks
}


}
template class DPsim::MnaSolverPlugin<Real>;
template class DPsim::MnaSolverPlugin<Complex>;
