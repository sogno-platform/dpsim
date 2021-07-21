/* Copyright 2017-2020 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/PFSolver.h>
#include <dpsim/SequentialScheduler.h>
#include <iostream>

using namespace DPsim;
using namespace CPS;

PFSolver::PFSolver(CPS::String name, CPS::SystemTopology system, CPS::Real timeStep, CPS::Logger::Level logLevel) :
	Solver(name + "_PF", logLevel) {
	mSystem = system;
	mTimeStep = timeStep;
}

void PFSolver::initialize(){
	mSLog->info("#### INITIALIZATION OF POWERFLOW SOLVER ");
    for (auto comp : mSystem.mComponents) {
        if (std::shared_ptr<CPS::SP::Ph1::SynchronGenerator> gen = std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(comp))
            mSynchronGenerators.push_back(gen);
        else if (std::shared_ptr<CPS::SP::Ph1::Load> load = std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp))
            mLoads.push_back(load);
        else if (std::shared_ptr<CPS::SP::Ph1::Transformer> trafo = std::dynamic_pointer_cast<CPS::SP::Ph1::Transformer>(comp))
            mTransformers.push_back(trafo);
		else if (std::shared_ptr<CPS::SP::Ph1::Transformer3W> trafo3w = std::dynamic_pointer_cast<CPS::SP::Ph1::Transformer3W>(comp))
            mTransformers3W.push_back(trafo3w);
        else if (std::shared_ptr<CPS::SP::Ph1::PiLine> line = std::dynamic_pointer_cast<CPS::SP::Ph1::PiLine>(comp))
            mLines.push_back(line);
        else if (std::shared_ptr<CPS::SP::Ph1::NetworkInjection> extnet = std::dynamic_pointer_cast<CPS::SP::Ph1::NetworkInjection>(comp))
            mExternalGrids.push_back(extnet);
		else if (std::shared_ptr<CPS::SP::Ph1::Shunt> shunt = std::dynamic_pointer_cast<CPS::SP::Ph1::Shunt>(comp))
            mShunts.push_back(shunt);
		else if(std::shared_ptr<CPS::SP::Ph1::SolidStateTransformer> sst = std::dynamic_pointer_cast<CPS::SP::Ph1::SolidStateTransformer>(comp))
			mSolidStateTransformers.push_back(sst);
		else if (std::shared_ptr<CPS::SP::Ph1::AvVoltageSourceInverterDQ> vsi = std::dynamic_pointer_cast<CPS::SP::Ph1::AvVoltageSourceInverterDQ>(comp)){
			mAverageVoltageSourceInverters.push_back(vsi);
		}
    }

	setBaseApparentPower();
	assignMatrixNodeIndices();
    initializeComponents();
    determinePFBusType();
    composeAdmittanceMatrix();

	mJ.setZero(mNumUnknowns,mNumUnknowns);
	mX.setZero(mNumUnknowns);
	mF.setZero(mNumUnknowns);
}

void PFSolver::assignMatrixNodeIndices() {
	mSLog->info("Assigning simulation nodes to topology nodes:");
	UInt matrixNodeIndexIdx = 0;
	for (UInt idx = 0; idx < mSystem.mNodes.size(); ++idx) {
		mSystem.mNodes[idx]->setMatrixNodeIndex(0, matrixNodeIndexIdx);
		mSLog->info("Node {}: MatrixNodeIndex {}", mSystem.mNodes[idx]->uid(), mSystem.mNodes[idx]->matrixNodeIndex());
		++matrixNodeIndexIdx;
	}
	mSLog->info("Number of simulation nodes: {:d}", matrixNodeIndexIdx);
}

void PFSolver::initializeComponents(){
    for (auto comp : mSystem.mComponents) {
        std::dynamic_pointer_cast<SimPowerComp<Complex>>(comp)->updateMatrixNodeIndices();
    }

	mSLog->info("-- Initialize components from terminals or nodes of topology");
	for (auto comp : mSystem.mComponents) {
		auto pComp = std::dynamic_pointer_cast<SimPowerComp<Complex>>(comp);
		if (!pComp)	continue;
		if (mInitFromNodesAndTerminals)
			pComp->initializeFromNodesAndTerminals(mSystem.mSystemFrequency);
	}

	mSLog->info("-- Calculate per unit parameters for all components");
	for (auto extnet : mExternalGrids) {
			extnet->calculatePerUnitParameters(mBaseApparentPower, mSystem.mSystemOmega);
	}
	for (auto line : mLines) {
			line->calculatePerUnitParameters(mBaseApparentPower, mSystem.mSystemOmega);
	}
	for(auto trans : mTransformers) {
		trans->calculatePerUnitParameters(mBaseApparentPower, mSystem.mSystemOmega);
	}
	for(auto trans3w : mTransformers3W) {
		trans3w->calculatePerUnitParameters(mBaseApparentPower, mSystem.mSystemOmega);
	}
	for(auto shunt : mShunts) {
		shunt->calculatePerUnitParameters(mBaseApparentPower, mSystem.mSystemOmega);
	}
	for(auto load : mLoads) {
		load->calculatePerUnitParameters(mBaseApparentPower, mSystem.mSystemOmega);
	}
	for(auto gen : mSynchronGenerators) {
		gen->calculatePerUnitParameters(mBaseApparentPower, mSystem.mSystemOmega);
	}
	for(auto sst : mSolidStateTransformers){
		sst->calculatePerUnitParameters(mBaseApparentPower, mSystem.mSystemOmega);
	}

}

void PFSolver::setBaseApparentPower() {
	Real maxPower = 0.;
	if (!mSynchronGenerators.empty()) {
		for (auto gen : mSynchronGenerators)
			if (std::abs(gen->attribute<Real>("P_set")->get()) > maxPower)
				maxPower = std::abs(gen->attribute<Real>("P_set")->get());
	}
	else if (!mTransformers3W.empty()) {
		for (auto trafo3w : mTransformers3W){
			if (trafo3w->attribute<Real>("S1")->get() > maxPower)
				maxPower = trafo3w->attribute<Real>("S1")->get();
			if (trafo3w->attribute<Real>("S2")->get() > maxPower)
				maxPower = trafo3w->attribute<Real>("S2")->get();
			if (trafo3w->attribute<Real>("S3")->get() > maxPower)
				maxPower = trafo3w->attribute<Real>("S3")->get();			
		}
	}
	else if (!mTransformers.empty()) {
		for (auto trafo : mTransformers)
			if (trafo->attribute<Real>("S")->get() > maxPower)
				maxPower = trafo->attribute<Real>("S")->get();
	}
    if (maxPower != 0.)
        mBaseApparentPower = pow(10, 1 + floor(log10(maxPower)));
	else
	{
		mBaseApparentPower = 100000000;
		mSLog->warn("No suitable quantity found for setting mBaseApparentPower. Using {} VA.", mBaseApparentPower);
	}
	mSLog->info("Base power = {} VA", mBaseApparentPower);
}

void PFSolver::determinePFBusType() {
	mPQBusIndices.clear();
	mPVBusIndices.clear();
	mVDBusIndices.clear();

	mSLog->info("-- Determine powerflow bus type for each node");

    // Determine powerflow bus type of each node through analysis of system topology
	for (auto node : mSystem.mNodes) {
		bool connectedPV = false;
		bool connectedPQ = false;
		bool connectedVD = false;

		for (auto comp : mSystem.mComponentsAtNode[node]) {
			if (std::shared_ptr<CPS::SP::Ph1::Load> load = std::dynamic_pointer_cast<CPS::SP::Ph1::Load>(comp)) {
				if (load->mPowerflowBusType == CPS::PowerflowBusType::PQ) {
					connectedPQ = true;
				}
			}
			else if (std::shared_ptr<CPS::SP::Ph1::SynchronGenerator> gen = std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(comp)) {
				if (gen->mPowerflowBusType == CPS::PowerflowBusType::PV) {
					connectedPV = true;
				}
				else if (gen->mPowerflowBusType == CPS::PowerflowBusType::VD) {
					connectedVD = true;
				}
			}
			else if (std::shared_ptr<CPS::SP::Ph1::NetworkInjection> extnet = std::dynamic_pointer_cast<CPS::SP::Ph1::NetworkInjection>(comp)) {
				if (extnet->mPowerflowBusType == CPS::PowerflowBusType::VD) {
					connectedVD = true;
				}
				else if (extnet->mPowerflowBusType == CPS::PowerflowBusType::PV) {
					connectedPV = true;
				}
			}
		}

		// determine powerflow bus types according connected type of connected components
		// only PQ type component connected -> set as PQ bus
		if (!connectedPV && connectedPQ && !connectedVD) {
			mSLog->debug("{}: only PQ type component connected -> set as PQ bus", node->name());
			mPQBusIndices.push_back(node->matrixNodeIndex());
			mPQBuses.push_back(node);
		} // no component connected -> set as PQ bus (P & Q will be zero)
		else if (!connectedPV && !connectedPQ && !connectedVD) {
			mSLog->debug("{}: no component connected -> set as PQ bus", node->name());
			mPQBusIndices.push_back(node->matrixNodeIndex());
			mPQBuses.push_back(node);
		} // only PV type component connected -> set as PV bus
		else if (connectedPV && !connectedPQ && !connectedVD) {
			mSLog->debug("{}: only PV type component connected -> set as PV bus", node->name());
			mPVBusIndices.push_back(node->matrixNodeIndex());
			mPVBuses.push_back(node);
		} // PV and PQ type component connected -> set as PV bus (TODO: bus type should be modifiable by user afterwards)
		else if (connectedPV && connectedPQ && !connectedVD) {
			mSLog->debug("{}: PV and PQ type component connected -> set as PV bus", node->name());
			mPVBusIndices.push_back(node->matrixNodeIndex());
			mPVBuses.push_back(node);
		} // only VD type component connected -> set as VD bus
		else if (!connectedPV && !connectedPQ && connectedVD) {
			mSLog->debug("{}: only VD type component connected -> set as VD bus", node->name());
			mVDBusIndices.push_back(node->matrixNodeIndex());
			mVDBuses.push_back(node);
		} // VD and PV type component connect -> set as VD bus
		else if (connectedPV && !connectedPQ && connectedVD) {
			mSLog->debug("{}: VD and PV type component connect -> set as VD bus", node->name());
			mVDBusIndices.push_back(node->matrixNodeIndex());
			mVDBuses.push_back(node);
		} // VD, PV and PQ type component connect -> set as VD bus
		else if (connectedPV && connectedPQ && connectedVD) {
			mSLog->debug("{}: VD, PV and PQ type component connect -> set as VD bus", node->name());
			mVDBusIndices.push_back(node->matrixNodeIndex());
			mVDBuses.push_back(node);
		}
		else {
			std::stringstream ss;
			ss << "Node>>" << node->name() << ": combination of connected components is invalid";
			throw std::invalid_argument(ss.str());
		}
	}

    mNumPQBuses = mPQBusIndices.size();
	mNumPVBuses = mPVBusIndices.size();
    mNumVDBuses = mVDBusIndices.size();
    mNumUnknowns = 2*mNumPQBuses + mNumPVBuses;

	// Aggregate PQ bus and PV bus index vectors for easy handling in solver
	mPQPVBusIndices.reserve(mNumPQBuses + mNumPVBuses);
    mPQPVBusIndices.insert(mPQPVBusIndices.end(), mPQBusIndices.begin(), mPQBusIndices.end());
    mPQPVBusIndices.insert(mPQPVBusIndices.end(), mPVBusIndices.begin(), mPVBusIndices.end());

	mSLog->info("#### Create index vectors for power flow solver:");
    mSLog->info("PQ Buses: {}", logVector(mPQBusIndices));
    mSLog->info("PV Buses: {}", logVector(mPVBusIndices));
    mSLog->info("VD Buses: {}", logVector(mVDBusIndices));
}

void PFSolver::setVDNode(CPS::String name) {
	if (!mExternalGrids.empty()) {
		if (mExternalGrids[0]->node(0)->name() == name) {
			mExternalGrids[0]->modifyPowerFlowBusType(CPS::PowerflowBusType::VD);
		}
	} else {
		for (auto gen : mSynchronGenerators) {
			if (gen->node(0)->name() == name)
			{
				gen->modifyPowerFlowBusType(CPS::PowerflowBusType::VD);
				return;
			}
		}
		throw std::invalid_argument("Invalid slack bus, no external grid or synchronous generator attached");
	}
}

void PFSolver::modifyPowerFlowBusComponent(CPS::String name, CPS::PowerflowBusType powerFlowBusType) {
	for (auto comp : mSystem.mComponents) {
		if (comp->name() == name) {
			if (std::shared_ptr<CPS::SP::Ph1::NetworkInjection> extnet = std::dynamic_pointer_cast<CPS::SP::Ph1::NetworkInjection>(comp))
				extnet->modifyPowerFlowBusType(powerFlowBusType);
			else if(std::shared_ptr<CPS::SP::Ph1::SynchronGenerator> gen = std::dynamic_pointer_cast<CPS::SP::Ph1::SynchronGenerator>(comp))
				gen->modifyPowerFlowBusType(powerFlowBusType);
		}
	}
}

void PFSolver::composeAdmittanceMatrix() {
	int n = mSystem.mNodes.size();
	if (n > 0) {
		mY = CPS::SparseMatrixComp(n, n);
		for (auto line : mLines) {
			line->pfApplyAdmittanceMatrixStamp(mY);
		}
		for(auto trans : mTransformers) {
			//to check if this transformer could be ignored
			if (trans->attribute("R") == 0 && trans->attribute("L") == 0) {
				mSLog->info("{} {} ignored for R = 0 and L = 0",trans->type(), trans->name());
				continue;
			}
			trans->pfApplyAdmittanceMatrixStamp(mY);
		}
		for(auto trans3w : mTransformers3W) {
			//to check if this transformer could be ignored
			if (trans3w->attribute("R1") == 0 && trans3w->attribute("L1") == 0 &&
				trans3w->attribute("R2") == 0 && trans3w->attribute("L2") == 0 &&
				trans3w->attribute("R3") == 0 && trans3w->attribute("L3") == 0) {
				mSLog->info("{} {} ignored for R = 0 and L = 0 in all windings", trans3w->type(), trans3w->name());
				continue;
			}
			trans3w->pfApplyAdmittanceMatrixStamp(mY);
		}
		for(auto shunt : mShunts) {
			shunt->pfApplyAdmittanceMatrixStamp(mY);
		}
	}
	if(mLines.empty() && mTransformers.empty() && mTransformers3W.empty()) {
		throw std::invalid_argument("There are no bus");
	}
}

CPS::Real PFSolver::G(int i, int j) {
	return mY.coeff(i, j).real();
}

CPS::Real PFSolver::B(int i, int j) {
	return mY.coeff(i, j).imag();
}

CPS::Bool PFSolver::checkConvergence() {
	// Converged if all mismatches are below the tolerance
    for (CPS::UInt i = 0; i < mNumUnknowns; i++) {
        if (abs(mF(i)) > mTolerance)
            return false;
	}
    return true;
}

Bool PFSolver::solvePowerflow() {
	// Calculate the mismatch according to the initial solution
    calculateMismatch();

    // Check whether model already converged
    isConverged = checkConvergence();

    mIterations = 0;
    for (unsigned i = 1; i < mMaxIterations && !isConverged; ++i) {

        calculateJacobian();
		auto sparseJ = mJ.sparseView();

		// Solve system mJ*mX = mF
        Eigen::SparseLU<SparseMatrix>lu(sparseJ);

		mX = lu.solve(mF);	/* code */

		// Calculate new solution based on mX increments obtained from equation system
		updateSolution();

        // Calculate the mismatch according to the current solution
        calculateMismatch();

		mSLog->debug("Mismatch vector at iteration {}: \n {}", i, mF);
		mSLog->flush();

		// Check convergence
        isConverged = checkConvergence();
        mIterations = i;
    }
	return isConverged;
}

void PFSolver::SolveTask::execute(Real time, Int timeStepCount) {
	// apply keepLastSolution to save computation time
    mSolver.generateInitialSolution(time);
	mSolver.solvePowerflow();
	mSolver.setSolution();
}

Task::List PFSolver::getTasks() {
	return Task::List{std::make_shared<SolveTask>(*this)};
}
