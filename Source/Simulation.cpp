#include "Simulation.h"

using namespace DPsim;

Simulation::Simulation() {
	mTime = 0;
	mCurrentSwitchTimeIndex = 0;
	mSystemModel.setSimType(SimulationType::DynPhasor);
}

Simulation::Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf) 
	: Simulation() {

	mSystemModel.setTimeStep(dt);
	mSystemModel.setOmega(om);
	mFinalTime = tf;
	
	initialize(elements);
}

Simulation::Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf, Logger& logger) 
	: Simulation(elements, om, dt, tf) {

	for (std::vector<BaseComponent*>::iterator it = elements.begin(); it != elements.end(); ++it) {
		logger.Log(Logtype::INFO) << "Added " << (*it)->getName() << " of type " << typeid(*(*it)).name() << " to simulation." << std::endl;
	}
	logger.Log(Logtype::INFO) << "System matrix A:" << std::endl;
	logger.Log() << mSystemModel.getCurrentSystemMatrix() << std::endl;
	logger.Log(Logtype::INFO) << "LU decomposition:" << std::endl;
	logger.Log() << mSystemModel.getLUdecomp() << std::endl;
	logger.Log(Logtype::INFO) << "Known variables matrix j:" << std::endl;
	logger.Log() << mSystemModel.getRightSideVector() << std::endl;
}

Simulation::Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf, Logger& logger, SimulationType simType) {
	mSystemModel.setSimType(simType);

	Simulation(elements, om, dt, tf, logger);
}


Simulation::~Simulation() {

}


void Simulation::initialize(std::vector<BaseComponent*> newElements) {
	mSystemModel.initialize(newElements);

	mElementsVector.push_back(newElements);

	int maxNode = 0;
	int numIdealVS = 0;
	int numRxLines = 0;
	for (std::vector<BaseComponent*>::iterator it = newElements.begin(); it != newElements.end(); ++it) {
		if ((*it)->getNode1() > maxNode)
			maxNode = (*it)->getNode1();
		if ((*it)->getNode2() > maxNode)
			maxNode = (*it)->getNode2();
		std::string type = typeid(*(*it)).name();

		if (type == "class DPsim::IdealVoltageSource")
		{
			numIdealVS = numIdealVS + 1;
		}

		if (type == "class RxLine")
		{
			numRxLines = numRxLines + 1;
		}

	}

	mNumNodes = maxNode + 1 + numIdealVS + numRxLines;
	mCompOffset = mNumNodes;

	mElements = mElementsVector[0];

	addSystemMatrix(newElements);
	
	// Initialize right side vector and components
	for (std::vector<BaseComponent*>::iterator it = mSystemModel.getElements().begin(); it != mSystemModel.getElements().end(); ++it) {
		(*it)->init(mSystemModel.getOmega(), mSystemModel.getTimeStep());
		(*it)->applyRightSideVectorStamp(mRightSideVector, mCompOffset, mSystemOmega, mTimeStep);
	}
}

void Simulation::addSystemMatrix(std::vector<BaseComponent*> newElements) {
	Matrix systemMatrix;

	if (mSimType == SimulationType::EMT) {
		systemMatrix = Matrix::Zero(mNumNodes, mNumNodes);
	}
	else {
		systemMatrix = Matrix::Zero(2 * mNumNodes, 2 * mNumNodes);
	}
	for (std::vector<BaseComponent*>::iterator it = newElements.begin(); it != newElements.end(); ++it) {
		(*it)->applySystemMatrixStamp(systemMatrix, mCompOffset, mSystemOmega, mTimeStep);
	}
	mSystemMatrixVector.push_back(systemMatrix);

	Eigen::PartialPivLU<DPSMatrix> luFactored = Eigen::PartialPivLU<DPSMatrix>(systemMatrix);
	mLuFactoredVector.push_back(luFactored);
}


int Simulation::step(Logger& logger)
{
	mSystemModel.getRightSideVector().setZero();
	
	for (std::vector<BaseComponent*>::iterator it = mElements.begin(); it != mElements.end(); ++it) {
		(*it)->step(mSystemMatrix, mRightSideVector, mCompOffset, mSystemOmega, mTimeStep, mTime);
	}
	
	mLeftSideVector = mLuFactored.solve(mRightSideVector);
 
	for (std::vector<BaseComponent*>::iterator it = mElements.begin(); it != mElements.end(); ++it) {
		(*it)->postStep(mSystemMatrix, mRightSideVector, mLeftSideVector, mCompOffset, mSystemOmega, mTimeStep, mTime);
	}

	if (mCurrentSwitchTimeIndex < mSwitchEventVector.size()) {
		if (mTime >= mSwitchEventVector[mCurrentSwitchTimeIndex].switchTime) {
			switchSystemMatrix(mSwitchEventVector[mCurrentSwitchTimeIndex].systemIndex);			
			mCurrentSwitchTimeIndex++;	
			logger.Log(Logtype::INFO) << "Switched to system " << mCurrentSwitchTimeIndex << " at " << mTime << std::endl;
		}
	}

	if (mTime >= mFinalTime) {
		return 0;
	}
	else {
		return 1;
	}

}

int Simulation::step(Logger& logger, Logger& leftSideVectorLog, Logger& rightSideVectorLog)  {
	int retValue = step(logger);

	leftSideVectorLog.Log() << Logger::VectorToDataLine(getTime(), getLeftSideVector()).str();
	rightSideVectorLog.Log() << Logger::VectorToDataLine(getTime(), getRightSideVector()).str();

	return retValue;
}

void Simulation::switchSystemMatrix(int systemMatrixIndex) {
	mSystemModel.switchSystemMatrix(systemMatrixIndex);
}

void Simulation::setSwitchTime(Real switchTime, Int systemIndex) {
	switchConfiguration newSwitchConf;
	newSwitchConf.switchTime = switchTime;
	newSwitchConf.systemIndex = systemIndex;
	mSwitchEventVector.push_back(newSwitchConf);
}

void Simulation::increaseByTimeStep() {
	mTime = mTime + mSystemModel.getTimeStep();
}
