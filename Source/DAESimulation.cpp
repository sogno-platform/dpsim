#pragma once 
#include "DAESimulation.h"


Simulation::DAESimulation(String name, Component::List comps, Real om, Real dt,SimulationType simType)
{
	/*mTime = 0;
	mLastLogTimeStep = 0;
	mCurrentSwitchTimeIndex = 0;
	*/
	mName = name;
	//mLogLevel = logLevel;
	mSystemModel.setSimType(simType);
	mSystemModel.setTimeStep(dt);
	mSystemModel.setOmega(om);
	//mFinalTime = tf;
	//mDownSampleRate = downSampleRate;

	initialize(comps);

	/*for (auto comp : comps) {
		mLog.Log(Logger::Level::INFO) << "Added " << comp->getType() << " '" << comp->getName() << "' to simulation." << std::endl;
	}

	mLog.Log(Logger::Level::INFO) << "System matrix:" << std::endl;
	mLog.LogMatrix(Logger::Level::INFO, mSystemModel.getCurrentSystemMatrix());
	mLog.Log(Logger::Level::INFO) << "LU decomposition:" << std::endl;
	mLog.LogMatrix(Logger::Level::INFO, mSystemModel.getLUdecomp());
	mLog.Log(Logger::Level::INFO) << "Right side vector:" << std::endl;
	mLog.LogMatrix(Logger::Level::INFO, mSystemModel.getRightSideVector());
	*/
}

void DAESimulation::initialize(Component::List newComponents)
{
	Int maxNode = 0;
	Int currentVirtualNode = 0;

	//mLog.Log(Logger::Level::INFO) << "#### Start Initialization ####" << std::endl;

	// Calculate the mNumber of nodes by going through the list of components
	// TODO we use the values from the first component vector right now and assume that
	// these values don't change on switches
	for (auto comp : newComponents) {
		// determine maximum node in component list
		if (comp->getNode1() > maxNode) {
			maxNode = comp->getNode1();
		}
		if (comp->getNode2() > maxNode) {
			maxNode = comp->getNode2();
		}
	}

	//mLog.Log(Logger::Level::INFO) << "Maximum node number: " << maxNode << std::endl;
	//currentVirtualNode = maxNode;

	// Check if component requires virtual node and if so set one
	for (auto comp : newComponents) {
		if (comp->hasVirtualNodes()) {
			for (Int node = 0; node < comp->getVirtualNodesNum(); node++) {
				currentVirtualNode++;
				comp->setVirtualNode(node, currentVirtualNode);
				//mLog.Log(Logger::Level::INFO) << "Created virtual node"<< node << "=" << currentVirtualNode
				//	<< " for " << comp->getName() << std::endl;
			}
		}
	}

	// Calculate size of system matrix
	//Int numNodes = maxNode + currentVirtualNode + 1;
	Int numNodes = currentVirtualNode + 1;

	// Create right and left vector
	mSystemModel.initialize(numNodes);

	// Initialize right side vector and components
	for (auto comp : newComponents) {
		comp->initialize(mSystemModel);
		comp->applyRightSideVectorStamp(mSystemModel);
	}

	// Create new system matrix and apply matrix stamps
	addSystemTopology(newComponents);

	switchSystemMatrix(0);
	mComponents = mComponentsVector[0];
}

void DAESimulation::addSystemTopology(Component::List newComponents)
{
	mComponentsVector.push_back(newComponents);

	// It is assumed that the system size does not change
	mSystemModel.createEmptySystemMatrix();

	for (auto comp : newComponents) {
		comp->applySystemMatrixStamp(mSystemModel);
	}

	mSystemModel.addSystemMatrix();
}

void DAESimulation::switchSystemMatrix(Int systemMatrixIndex)
{
	mSystemModel.switchSystemMatrix(systemMatrixIndex);
}

//TO-DO: Add DAE Residual Vector calculation
void DAESimulation::run()
{
	std::cout<<"Future Res Vector"<<endl;
}
