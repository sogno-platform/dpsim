#include "magma_types.h"
#include <dpsim/MNASolverGpuMagma.h>
#include <dpsim/SequentialScheduler.h>
#include <Eigen/Eigen>

using namespace DPsim;
using namespace CPS;

namespace DPsim {

template <typename VarType>
MnaSolverGpuMagma<VarType>::MnaSolverGpuMagma(String name,
	CPS::Domain domain, CPS::Logger::Level logLevel) :
    MnaSolverEigenSparse<VarType>(name, domain, logLevel)
{
	magma_init();
	magma_queue_create(0, &mMagmaQueue);
	mHostSysMat = {Magma_CSR};
	mDevSysMat = {Magma_CSR};
	mHostRhsVec = {Magma_CSR};
	mDevRhsVec = {Magma_CSR};
	mHostLhsVec = {Magma_CSR};
	mDevLhsVec = {Magma_CSR};
}

template <typename VarType>
MnaSolverGpuMagma<VarType>::~MnaSolverGpuMagma() {
	magma_dmfree(&mDevSysMat, mMagmaQueue);
	magma_dmfree(&mDevRhsVec, mMagmaQueue);
	magma_dmfree(&mDevLhsVec, mMagmaQueue);

	magma_queue_destroy(mMagmaQueue);
	magma_finalize();
}


template <typename VarType>
void MnaSolverGpuMagma<VarType>::initialize() {
    MnaSolver<VarType>::initialize();

	auto hMat = this->mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)];
    int size = this->mRightSideVector.rows();
	magma_dcsrset(size, size,
				  hMat[0].outerIndexPtr(),
				  hMat[0].innerIndexPtr(),
				  hMat[0].valuePtr(),
				  &mHostSysMat,
				  mMagmaQueue);

    mMagmaOpts.solver_par.solver     = Magma_PIDRMERGE;
    mMagmaOpts.solver_par.restart    = 8;
    mMagmaOpts.solver_par.maxiter    = 1000;
    mMagmaOpts.solver_par.rtol       = 1e-10;
    mMagmaOpts.solver_par.maxiter    = 1000;
    mMagmaOpts.precond_par.solver    = Magma_ILU;
    mMagmaOpts.precond_par.levels    = 0;
    mMagmaOpts.precond_par.trisolver = Magma_CUSOLVE;

	magma_dsolverinfo_init(&mMagmaOpts.solver_par,
						   &mMagmaOpts.precond_par,
						   mMagmaQueue);

	magma_dmtransfer(mHostSysMat, &mDevSysMat, Magma_CPU, Magma_DEV, mMagmaQueue);
    magma_d_precondsetup(mDevSysMat, mDevSysMat, &mMagmaOpts.solver_par, &mMagmaOpts.precond_par, mMagmaQueue);
}

template <typename VarType>
Task::List MnaSolverGpuMagma<VarType>::getTasks() {
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
	l.push_back(std::make_shared<MnaSolverGpuMagma<VarType>::SolveTask>(*this));
    l.push_back(std::make_shared<MnaSolverGpuMagma<VarType>::LogTask>(*this));
	return l;
}

template <typename VarType>
void MnaSolverGpuMagma<VarType>::solve(Real time, Int timeStepCount) {
	int size = this->mRightSideVector.rows();
	int one = 0;
    // Reset source vector
	this->mRightSideVector.setZero();

    // Add together the right side vector (computed by the components'
	// pre-step tasks)
	for (const auto &stamp : this->mRightVectorStamps)
		this->mRightSideVector += *stamp;

	if (!this->mIsInInitialization)
		this->updateSwitchStatus();

    //Copy right vector to device
	magma_dvset(size, 1, this->mRightSideVector.data(), &mHostRhsVec, mMagmaQueue);
	magma_dmtransfer(mHostRhsVec, &mDevRhsVec, Magma_CPU, Magma_DEV, mMagmaQueue);
	magma_dvinit(&mDevLhsVec, Magma_DEV, mHostRhsVec.num_rows, mHostRhsVec.num_cols, 0.0, mMagmaQueue);

    // Solve
	magma_d_solver(mDevSysMat, mDevRhsVec, &mDevLhsVec, &mMagmaOpts, mMagmaQueue);

    //Copy Solution back
	magma_dmtransfer(mDevLhsVec, &mHostLhsVec, Magma_DEV, Magma_CPU, mMagmaQueue);
	magma_dvcopy(mDevLhsVec, &size, &one, this->leftSideVector().data(), mMagmaQueue);

    mSLog->debug("result has size ({},{})", size, one);

	// TODO split into separate task? (dependent on x, updating all v attributes)
	for (UInt nodeIdx = 0; nodeIdx < this->mNumNetNodes; ++nodeIdx)
		this->mNodes[nodeIdx]->mnaUpdateVoltage(this->mLeftSideVector);


	// Components' states will be updated by the post-step tasks
}

}
template class DPsim::MnaSolverGpuMagma<Real>;
template class DPsim::MnaSolverGpuMagma<Complex>;
