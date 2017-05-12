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
		logger.Log(LogLevel::INFO) << "Added " << (*it)->getName() << " of type " << typeid(*(*it)).name() << " to simulation." << std::endl;
	}
	logger.Log(LogLevel::INFO) << "System matrix A:" << std::endl;
	logger.Log() << mSystemModel.getCurrentSystemMatrix() << std::endl;
	logger.Log(LogLevel::INFO) << "LU decomposition:" << std::endl;
	logger.Log() << mSystemModel.getLUdecomp() << std::endl;
	logger.Log(LogLevel::INFO) << "Known variables matrix j:" << std::endl;
	logger.Log() << mSystemModel.getRightSideVector() << std::endl;
}

Simulation::Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf, Logger& logger, SimulationType simType) {
	mSystemModel.setSimType(simType);

	Simulation(elements, om, dt, tf, logger);
}


Simulation::~Simulation() {

}


void Simulation::initialize(std::vector<BaseComponent*> newElements) {	
	int maxNode = 0;
	Int numIdealVS = 0;
	int numLines = 0;

	mElementsVector.push_back(newElements);
	mElements = mElementsVector[0];

	// Calculate the number of nodes by going through the list of elements
	for (std::vector<BaseComponent*>::iterator it = mElements.begin(); it != mElements.end(); ++it) {
		if ((*it)->getNode1() > maxNode) {
			maxNode = (*it)->getNode1();
		}		
		if ((*it)->getNode2() > maxNode) {
			maxNode = (*it)->getNode2();
		}
		std::string type = typeid(*(*it)).name();

		if (type == "class DPsim::IdealVoltageSource") {
			numIdealVS = numIdealVS + 1;
		}
		if (type == "class DPsim::RxLine" || type == "class DPsim::PiLine") {
			if ((*it)->getNode3() != -1) {
				numLines = numLines + 1;
			}
			
		}
	}

	Int numNodes = maxNode + 1 + numIdealVS + numLines;
	mSystemModel.initialize(numNodes,numIdealVS);
	addSystemTopology(mElements);
	
	// Initialize right side vector and components
	for (std::vector<BaseComponent*>::iterator it = mElements.begin(); it != mElements.end(); ++it) {
		(*it)->init(mSystemModel.getOmega(), mSystemModel.getTimeStep());
		(*it)->applyRightSideVectorStamp(mSystemModel);
	}
}

void Simulation::addSystemTopology(std::vector<BaseComponent*> newElements) {
	Matrix systemMatrix;

	if (mSystemModel.getSimType() == SimulationType::EMT) {
		systemMatrix = Matrix::Zero(mSystemModel.getNumNodes(), mSystemModel.getNumNodes());
	}
	else {
		systemMatrix = Matrix::Zero(2 * mSystemModel.getNumNodes(), 2 * mSystemModel.getNumNodes());
	}

	for (std::vector<BaseComponent*>::iterator it = newElements.begin(); it != newElements.end(); ++it) {
		(*it)->applySystemMatrixStamp(mSystemModel);
	}

	systemMatrix = getSystemMatrix();

	mSystemModel.addSystemMatrix(systemMatrix);
}


int Simulation::step(Logger& logger)
{
	mSystemModel.setRightSideVectorToZero(getRightSideVector());
	
	for (std::vector<BaseComponent*>::iterator it = mElements.begin(); it != mElements.end(); ++it) {
		(*it)->step(mSystemModel, mTime);
	}
	
	mSystemModel.solve();
 
	for (std::vector<BaseComponent*>::iterator it = mElements.begin(); it != mElements.end(); ++it) {
		(*it)->postStep(mSystemModel);
	}

	if (mCurrentSwitchTimeIndex < mSwitchEventVector.size()) {
		if (mTime >= mSwitchEventVector[mCurrentSwitchTimeIndex].switchTime) {
			switchSystemMatrix(mSwitchEventVector[mCurrentSwitchTimeIndex].systemIndex);			
			mCurrentSwitchTimeIndex++;	
			logger.Log(LogLevel::INFO) << "Switched to system " << mCurrentSwitchTimeIndex << " at " << mTime << std::endl;
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

int Simulation::stepGeneratorTest(Logger& logger, Logger& leftSideVectorLog, Logger& rightSideVectorLog, BaseComponent* generator,
	Logger& synGenLogFlux, Logger& synGenLogVolt, Logger& synGenLogCurr, Real fieldVoltage, Real mechPower, Real logTimeStep, Real& lastLogTime)
{
	// Set to zero because all components will add their contribution for the current time step to the current value
	mSystemModel.getRightSideVector().setZero();

	// Execute step for all circuit components
	for (std::vector<BaseComponent*>::iterator it = mElements.begin(); it != mElements.end(); ++it) {
		(*it)->step(mSystemModel, mTime);
	}

	// Individual step function for generator
	if (mSystemModel.getSimType() == SimulationType::DynPhasor) {
		((SynchronGenerator*)generator)->step(mSystemModel, fieldVoltage, mechPower);
	} 
	else {
		((SynchronGeneratorEMT*)generator)->step(mSystemModel, fieldVoltage, mechPower);
	}
	
	// Solve circuit for vector j with generator output current
	mSystemModel.solve();

	// Execute PostStep for all components, generator states are recalculated based on new terminal voltage	
	for (std::vector<BaseComponent*>::iterator it = mElements.begin(); it != mElements.end(); ++it) {
		(*it)->postStep(mSystemModel);
	}

	if (mCurrentSwitchTimeIndex < mSwitchEventVector.size()) {
		if (mTime >= mSwitchEventVector[mCurrentSwitchTimeIndex].switchTime) {
			switchSystemMatrix(mSwitchEventVector[mCurrentSwitchTimeIndex].systemIndex);

			mCurrentSwitchTimeIndex++;
			logger.Log(LogLevel::INFO) << "Switched to system " << mCurrentSwitchTimeIndex << " at " << mTime << std::endl;
		}
	}

	// Save simulation step data
	if (mTime >= lastLogTime + logTimeStep) {
		lastLogTime = mTime;
		std::cout << mTime << std::endl;

		leftSideVectorLog.Log() << Logger::VectorToDataLine(getTime(), getLeftSideVector()).str();
		rightSideVectorLog.Log() << Logger::VectorToDataLine(getTime(), getRightSideVector()).str();

		if (mSystemModel.getSimType() == SimulationType::DynPhasor) {
			synGenLogFlux.Log() << Logger::VectorToDataLine(mTime, ((SynchronGenerator*)generator)->getFluxes()).str();
			synGenLogVolt.Log() << Logger::VectorToDataLine(mTime, ((SynchronGenerator*)generator)->getVoltages()).str();
			synGenLogCurr.Log() << Logger::VectorToDataLine(mTime, ((SynchronGenerator*)generator)->getCurrents()).str();
		}
		else {
			synGenLogFlux.Log() << Logger::VectorToDataLine(mTime, ((SynchronGeneratorEMT*)generator)->getFluxes()).str();
			synGenLogVolt.Log() << Logger::VectorToDataLine(mTime, ((SynchronGeneratorEMT*)generator)->getVoltages()).str();
			synGenLogCurr.Log() << Logger::VectorToDataLine(mTime, ((SynchronGeneratorEMT*)generator)->getCurrents()).str();
		}
	}

	if (mTime >= mFinalTime) {
		return 0;
	}
	else {
		return 1;
	}
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
