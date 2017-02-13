#include "Simulation.h"

Simulation::Simulation() {

}

Simulation::Simulation(std::vector<BaseComponent*> elements, double om, double dt, double tf) {
	this->mTimeStep = dt;
	this->mFinalTime = tf;
	this->mSystemOmega = om;
	
	CreateSystemMatrix(elements);
	Initialize();

	mSystemMatrix = mSystemMatrixVector[0];
	mRightSideVector = DPSMatrix::Zero(2 * mNumNodes, 1);
	mLeftSideVector = DPSMatrix::Zero(2 * mNumNodes, 1);
}

Simulation::Simulation(std::vector<BaseComponent*> elements, double om, double dt, double tf, Logger& logger) : Simulation(elements, om, dt, tf) {
	for (std::vector<BaseComponent*>::iterator it = elements.begin(); it != elements.end(); ++it) {
		logger.Log(Logtype::INFO) << "Added " << (*it)->getName() << " of type " << typeid(*(*it)).name() << " to simulation." << std::endl;
	}
	logger.Log(Logtype::INFO) << "System matrix A:" << std::endl;
	logger.Log() << mSystemMatrix << std::endl;
	logger.Log(Logtype::INFO) << "Known variables matrix j:" << std::endl;
	logger.Log() << mRightSideVector << std::endl;
}


Simulation::~Simulation() {
}


void Simulation::Initialize() {
	// Initialize time variable
	mTime = 0;
	
	// 
	for (std::vector<BaseComponent*>::iterator it = mElements.begin(); it != mElements.end(); ++it) {
		(*it)->init(mSystemOmega, mTimeStep);
		(*it)->applyRightSideVectorStamp(mRightSideVector, mCompOffset, mSystemOmega, mTimeStep);
	}


	mLuFactored = Eigen::PartialPivLU<DPSMatrix>(mSystemMatrix);
}
	
void Simulation::CreateSystemMatrix(std::vector<BaseComponent*> newElements) {
	std::vector<BaseComponent*> elements;
	for (std::vector<BaseComponent*>::iterator it = newElements.begin(); it != newElements.end(); ++it) {
		elements.push_back((*it));
	}
	mElementsVector.push_back(elements);

	int maxNode = 0;
	for (std::vector<BaseComponent*>::iterator it = mElements.begin(); it != mElements.end(); ++it) {
		if ((*it)->getNode1() > maxNode)
			maxNode = (*it)->getNode1();
		if ((*it)->getNode2() > maxNode)
			maxNode = (*it)->getNode2();
	}
		
	mNumNodes = maxNode + 1;
	mCompOffset = mNumNodes;
	DPSMatrix systemMatrix = DPSMatrix::Zero(2 * mNumNodes, 2 * mNumNodes);
	mSystemMatrixVector.push_back(systemMatrix);

	for (std::vector<BaseComponent*>::iterator it = elements.begin(); it != elements.end(); ++it) {
		(*it)->applySystemMatrixStamp(mSystemMatrix, mCompOffset, mSystemOmega, mTimeStep);
	}

	Eigen::PartialPivLU<DPSMatrix> luFactored = Eigen::PartialPivLU<DPSMatrix>(systemMatrix);
	mLuFactoredVector.push_back(luFactored);
	
}


int Simulation::Step()
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

	if(mTime >= mFinalTime)
		return 0;
	else
		return 1;
}
