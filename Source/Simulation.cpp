#include "Simulation.h"

using namespace DPsim;

Simulation::Simulation() {
	this->mCurrentSwitchTimeIndex = 0;
	mSimType = SimulationType::DynPhasor;
}

Simulation::Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf) : Simulation() {
	mTimeStep = dt;
	mFinalTime = tf;
	mSystemOmega = om;
	
	CreateSystemMatrix(elements);
	Initialize();
}

Simulation::Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf, Logger& logger) : Simulation() {
	mTimeStep = dt;
	mFinalTime = tf;
	mSystemOmega = om;

	CreateSystemMatrix(elements);
	Initialize();

	for (std::vector<BaseComponent*>::iterator it = elements.begin(); it != elements.end(); ++it) {
		logger.Log(Logtype::INFO) << "Added " << (*it)->getName() << " of type " << typeid(*(*it)).name() << " to simulation." << std::endl;
	}
	logger.Log(Logtype::INFO) << "System matrix A:" << std::endl;
	logger.Log() << mSystemMatrix << std::endl;
	logger.Log(Logtype::INFO) << "LU decomposition:" << std::endl;
	logger.Log() << mLuFactored.matrixLU() << std::endl;
	logger.Log(Logtype::INFO) << "Known variables matrix j:" << std::endl;
	logger.Log() << mRightSideVector << std::endl;
}

Simulation::Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf, Logger& logger, SimulationType simType) {
	mSimType = simType;

	mTimeStep = dt;
	mFinalTime = tf;
	mSystemOmega = om;

	CreateSystemMatrix(elements);
	Initialize();

	for (std::vector<BaseComponent*>::iterator it = elements.begin(); it != elements.end(); ++it) {
		logger.Log(Logtype::INFO) << "Added " << (*it)->getName() << " of type " << typeid(*(*it)).name() << " to simulation." << std::endl;
	}
	logger.Log(Logtype::INFO) << "System matrix A:" << std::endl;
	logger.Log() << mSystemMatrix << std::endl;
	logger.Log(Logtype::INFO) << "LU decomposition:" << std::endl;
	logger.Log() << mLuFactored.matrixLU() << std::endl;
	logger.Log(Logtype::INFO) << "Known variables matrix j:" << std::endl;
	logger.Log() << mRightSideVector << std::endl;
}


Simulation::~Simulation() {

}


void Simulation::Initialize() {
	mSystemMatrix = mSystemMatrixVector[0];
	mLuFactored = mLuFactoredVector[0];
	if (mSimType == SimulationType::EMT) {
		mRightSideVector = DPSMatrix::Zero(mNumNodes, 1);
		mLeftSideVector = DPSMatrix::Zero(mNumNodes, 1);
	}
	else {
		mRightSideVector = DPSMatrix::Zero(2 * mNumNodes, 1);
		mLeftSideVector = DPSMatrix::Zero(2 * mNumNodes, 1);
	}	
	mElements = mElementsVector[0];

	// Initialize time variable
	mTime = 0;

	// Initialize right side vector and components
	for (std::vector<BaseComponent*>::iterator it = mElements.begin(); it != mElements.end(); ++it) {
		(*it)->init(mSystemOmega, mTimeStep);
		(*it)->applyRightSideVectorStamp(mRightSideVector, mCompOffset, mSystemOmega, mTimeStep);
	}
}


void Simulation::CreateSystemMatrix(std::vector<BaseComponent*> newElements) {
	std::vector<BaseComponent*> elements;
	for (std::vector<BaseComponent*>::iterator it = newElements.begin(); it != newElements.end(); ++it) {
		elements.push_back((*it));
	}
	mElementsVector.push_back(elements);

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
	DPSMatrix systemMatrix;

	if (mSimType == SimulationType::EMT) {
		systemMatrix = DPSMatrix::Zero(mNumNodes, mNumNodes);
	}
	else {
		systemMatrix = DPSMatrix::Zero(2 * mNumNodes, 2 * mNumNodes);
	}
	for (std::vector<BaseComponent*>::iterator it = elements.begin(); it != elements.end(); ++it) {
			(*it)->applySystemMatrixStamp(systemMatrix, mCompOffset, mSystemOmega, mTimeStep);
	}
	

	mSystemMatrixVector.push_back(systemMatrix);

	Eigen::PartialPivLU<DPSMatrix> luFactored = Eigen::PartialPivLU<DPSMatrix>(systemMatrix);
	mLuFactoredVector.push_back(luFactored);	
}


int Simulation::step(Logger& logger)
{
	mRightSideVector.setZero();
	
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
	if (systemMatrixIndex < mSystemMatrixVector.size()) {
		mSystemMatrix = mSystemMatrixVector[systemMatrixIndex];
		mLuFactored = mLuFactoredVector[systemMatrixIndex];
	}	
}

void Simulation::setSwitchTime(Real switchTime, Int systemIndex) {
	switchConfiguration newSwitchConf;
	newSwitchConf.switchTime = switchTime;
	newSwitchConf.systemIndex = systemIndex;
	mSwitchEventVector.push_back(newSwitchConf);
}

void Simulation::increaseByTimeStep() {
	mTime = mTime + mTimeStep;
}
