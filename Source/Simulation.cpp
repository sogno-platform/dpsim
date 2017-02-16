#include "Simulation.h"

using namespace DPsim;

Simulation::Simulation() {
	this->mCurrentSwitchTimeIndex = 0;
}

Simulation::Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf) : Simulation() {
	this->mTimeStep = dt;
	this->mFinalTime = tf;
	this->mSystemOmega = om;
	
	CreateSystemMatrix(elements);
	Initialize();
}

Simulation::Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf, Logger& logger) : Simulation() {
	this->mTimeStep = dt;
	this->mFinalTime = tf;
	this->mSystemOmega = om;

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
	mRightSideVector = DPSMatrix::Zero(2 * mNumNodes, 1);
	mLeftSideVector = DPSMatrix::Zero(2 * mNumNodes, 1);
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
	for (std::vector<BaseComponent*>::iterator it = newElements.begin(); it != newElements.end(); ++it) {
		if ((*it)->getNode1() > maxNode)
			maxNode = (*it)->getNode1();
		if ((*it)->getNode2() > maxNode)
			maxNode = (*it)->getNode2();
	}		

	mNumNodes = maxNode + 1;
	mCompOffset = mNumNodes;
	DPSMatrix systemMatrix = DPSMatrix::Zero(2 * mNumNodes, 2 * mNumNodes);	

	for (std::vector<BaseComponent*>::iterator it = elements.begin(); it != elements.end(); ++it) {
		(*it)->applySystemMatrixStamp(systemMatrix, mCompOffset, mSystemOmega, mTimeStep);
	}
	mSystemMatrixVector.push_back(systemMatrix);

	Eigen::PartialPivLU<DPSMatrix> luFactored = Eigen::PartialPivLU<DPSMatrix>(systemMatrix);
	mLuFactoredVector.push_back(luFactored);	
}


int Simulation::Step(Logger& logger)
{
	mRightSideVector.setZero();
	
	for (std::vector<BaseComponent*>::iterator it = mElements.begin(); it != mElements.end(); ++it) {
		(*it)->step(mSystemMatrix, mRightSideVector, mCompOffset, mSystemOmega, mTimeStep, mTime);
	}
	
	mLeftSideVector = mLuFactored.solve(mRightSideVector);
 
	for (std::vector<BaseComponent*>::iterator it = mElements.begin(); it != mElements.end(); ++it) {
		(*it)->postStep(mSystemMatrix, mRightSideVector, mLeftSideVector, mCompOffset, mSystemOmega, mTimeStep, mTime);
	}

	mTime += mTimeStep;

	if (mCurrentSwitchTimeIndex < mSwitchEventVector.size()) {
		if (mTime >= mSwitchEventVector[mCurrentSwitchTimeIndex].switchTime) {
			switchSystemMatrix(mSwitchEventVector[mCurrentSwitchTimeIndex].systemIndex);			
			mCurrentSwitchTimeIndex++;	
			logger.Log(Logtype::INFO) << "Switched to system" << mCurrentSwitchTimeIndex << std::endl;
		}
	}

	if(mTime >= mFinalTime)
		return 0;
	else
		return 1;
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