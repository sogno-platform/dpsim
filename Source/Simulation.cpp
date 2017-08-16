#include "Simulation.h"

#include <signal.h>
//#include <sys/timerfd.h>
#include <time.h>
//#include <unistd.h>

using namespace DPsim;

Simulation::Simulation() {
	mTime = 0;
	mCurrentSwitchTimeIndex = 0;
}

Simulation::Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf, SimulationType simType)
	: Simulation() {

	mSystemModel.setSimType(simType);
	mSystemModel.setTimeStep(dt);
	mSystemModel.setOmega(om);
	mFinalTime = tf;
	
	initialize(elements);
}

Simulation::Simulation(std::vector<BaseComponent*> elements, Real om, Real dt, Real tf, Logger& logger, SimulationType simType)
	: Simulation(elements, om, dt, tf, simType) {

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


Simulation::~Simulation() {

}


void Simulation::initialize(std::vector<BaseComponent*> newElements) {	
	int maxNode = 0;
	Int numIdealVS = 0;
	int numLines = 0;

	// Calculate the number of nodes by going through the list of elements
	// TODO we use the values from the first element vector right now and assume that
	// these values don't change on switches
	for (std::vector<BaseComponent*>::iterator it = newElements.begin(); it != newElements.end(); ++it) {
		if ((*it)->getNode1() > maxNode) {
			maxNode = (*it)->getNode1();
		}		
		if ((*it)->getNode2() > maxNode) {
			maxNode = (*it)->getNode2();
		}
		std::string type = typeid(*(*it)).name();

		if (dynamic_cast<IdealVoltageSource*>(*it)) {
			numIdealVS = numIdealVS + 1;
		}
		if (dynamic_cast<RxLine*>(*it) || dynamic_cast<PiLine*>(*it)) {
			if ((*it)->getNode3() != -1) {
				numLines = numLines + 1;
			}
			
		}
	}

	Int numNodes = maxNode + 1 + numIdealVS + numLines;
	mSystemModel.initialize(numNodes,numIdealVS);
	addSystemTopology(newElements);
	switchSystemMatrix(0);
	mElements = mElementsVector[0];
	
	// Initialize right side vector and components
	for (std::vector<BaseComponent*>::iterator it = mElements.begin(); it != mElements.end(); ++it) {
		(*it)->init(mSystemModel.getOmega(), mSystemModel.getTimeStep());
		(*it)->applyRightSideVectorStamp(mSystemModel);
	}
}

void Simulation::addSystemTopology(std::vector<BaseComponent*> newElements) {
	mElementsVector.push_back(newElements);
	// TODO: it would be cleaner to pass the matrix reference to all the stamp methods
	// and not have an implicit "current" matrix for those in SystemModel
	Matrix& systemMatrix = mSystemModel.getCurrentSystemMatrix();
	// save old matrix in case we already defined one
	Matrix systemMatrixCopy = systemMatrix;

	if (mSystemModel.getSimType() == SimulationType::EMT) {
		systemMatrix = Matrix::Zero(mSystemModel.getNumNodes(), mSystemModel.getNumNodes());
	}
	else {
		systemMatrix = Matrix::Zero(2 * mSystemModel.getNumNodes(), 2 * mSystemModel.getNumNodes());
	}

	for (std::vector<BaseComponent*>::iterator it = newElements.begin(); it != newElements.end(); ++it) {
		(*it)->applySystemMatrixStamp(mSystemModel);
	}

	mSystemModel.addSystemMatrix(systemMatrix);
	// restore saved copy
	systemMatrix = systemMatrixCopy;
}


int Simulation::step(Logger& logger, bool blocking)
{
	mSystemModel.setRightSideVectorToZero();
	
	for (auto it = mExternalInterfaces.begin(); it != mExternalInterfaces.end(); ++it) {
		(*it)->readValues(blocking);
	}

	for (std::vector<BaseComponent*>::iterator it = mElements.begin(); it != mElements.end(); ++it) {
		(*it)->step(mSystemModel, mTime);
	}
	
	mSystemModel.solve();
 
	for (std::vector<BaseComponent*>::iterator it = mElements.begin(); it != mElements.end(); ++it) {
		(*it)->postStep(mSystemModel);
	}

	for (auto it = mExternalInterfaces.begin(); it != mExternalInterfaces.end(); ++it) {
		(*it)->writeValues(mSystemModel);
	}

	if (mCurrentSwitchTimeIndex < mSwitchEventVector.size()) {
		if (mTime >= mSwitchEventVector[mCurrentSwitchTimeIndex].switchTime) {
			switchSystemMatrix(mSwitchEventVector[mCurrentSwitchTimeIndex].systemIndex);			
			mElements = mElementsVector[++mCurrentSwitchTimeIndex];	
			logger.Log(LogLevel::INFO) << "Switched to system " << mCurrentSwitchTimeIndex << " at " << mTime << std::endl;
			logger.Log(LogLevel::INFO) << "New matrix:" << std::endl << mSystemModel.getCurrentSystemMatrix() << std::endl;
			logger.Log(LogLevel::INFO) << "New decomp:" << std::endl << mSystemModel.getLUdecomp() << std::endl;
		}
	}

	if (mTime >= mFinalTime) {
		return 0;
	}
	else {
		return 1;
	}

}

//void Simulation::alarmHandler(int sig, siginfo_t* si, void* ctx) {
//	Simulation *sim = static_cast<Simulation*>(si->si_value.sival_ptr);
//	/* only throw an exception if we're actually behind */
//	if (++sim->mRtTimerCount * sim->mSystemModel.getTimeStep() > sim->mTime)
//		throw TimerExpiredException();
//}
//
//void Simulation::runRT(RTMethod rtMethod, bool startSynch, Logger& logger, Logger& llogger, Logger& rlogger ) {
//	char timebuf[8];
//	int ret, sig, timerfd;
//	sigset_t alrmset;
//	struct sigaction sa;
//	struct sigevent evp;
//	struct itimerspec ts;
//	timer_t timer;
//	uint64_t overrun;
//
//	// initialize timer / timerfd
//	if (rtMethod == RTTimerFD) {
//		timerfd = timerfd_create(CLOCK_MONOTONIC, 0);
//		if (timerfd < 0) {
//			std::perror("Failed to create timerfd");
//			std::exit(1);
//		}
//	} else if (rtMethod == RTExceptions) {
//		sa.sa_sigaction = Simulation::alarmHandler;
//		sa.sa_flags = SA_SIGINFO;
//		sigemptyset(&sa.sa_mask);
//		if (sigaction(SIGALRM, &sa, NULL)) {
//			std::perror("Failed to establish SIGALRM handler");
//			std::exit(1);
//		}
//
//		evp.sigev_notify = SIGEV_SIGNAL;
//		evp.sigev_signo = SIGALRM;
//		evp.sigev_value.sival_ptr = this;
//		if (timer_create(CLOCK_MONOTONIC, &evp, &timer)) {
//			std::perror("Failed to create timer");
//			std::exit(1);
//		}
//
//		sigemptyset(&alrmset);
//		sigaddset(&alrmset, SIGALRM);
//	} else {
//		std::cerr << "invalid rt method, exiting" << std::endl;
//		std::exit(1);
//	}
//
//	ts.it_value.tv_sec = (time_t) mSystemModel.getTimeStep();
//	ts.it_value.tv_nsec = (long) (mSystemModel.getTimeStep() * 1e9);
//	ts.it_interval = ts.it_value;
//
//	// optional start synchronization
//	if (startSynch) {
//		step(logger, llogger, rlogger, false); // first step, sending the initial values
//		step(logger, llogger, rlogger, true); // blocking step for synchronization + receiving the initial state of the other network
//		increaseByTimeStep();
//	}
//
//	// arm timer
//	if (rtMethod == RTTimerFD) {
//		if (timerfd_settime(timerfd, 0, &ts, 0) < 0) {
//			std::perror("Failed to arm timerfd");
//			std::exit(1);
//		}
//	} else if (rtMethod == RTExceptions) {
//		if (timer_settime(timer, 0, &ts, NULL)) {
//			std::perror("Failed to arm timer");
//			std::exit(1);
//		}
//	}
//	
//	// main loop
//	do {
//		if (rtMethod == RTExceptions) {
//			try {
//				ret = step(logger, llogger, rlogger, false);
//				sigwait(&alrmset, &sig);
//			} catch (TimerExpiredException& e) {
//				std::cerr << "timestep expired at " << mTime << std::endl;
//			}
//		} else if (rtMethod == RTTimerFD) {
//			ret = step(logger, llogger, rlogger, false);
//			if (read(timerfd, timebuf, 8) < 0) {
//				std::perror("Read from timerfd failed");
//				std::exit(1);
//			}
//			overrun = *((uint64_t*) timebuf);
//			if (overrun > 1) {
//				std::cerr << "timerfd overrun of " << overrun-1 << " at " << mTime << std::endl;
//			}
//		}
//		increaseByTimeStep();
//		if (!ret)
//			break;
//	} while (ret);
//
//	// cleanup
//	if (rtMethod == RTTimerFD) {
//		close(timerfd);
//	} else if (rtMethod == RTExceptions) {
//		timer_delete(timer);
//	}
//}

int Simulation::step(Logger& logger, Logger& leftSideVectorLog, Logger& rightSideVectorLog, bool blocking) {
	int retValue = step(logger, blocking);

	leftSideVectorLog.LogDataLine(getTime(), getLeftSideVector());
	rightSideVectorLog.LogDataLine(getTime(), getRightSideVector());

	return retValue;
}

int Simulation::stepGeneratorTest(Logger& logger, Logger& leftSideVectorLog, Logger& rightSideVectorLog, BaseComponent* generator,
	Logger& synGenLogFlux, Logger& synGenLogVolt, Logger& synGenLogCurr, Real fieldVoltage, Real mechPower, Real logTimeStep, Real& lastLogTime, Real time)
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
		((SynchronGeneratorEMT*)generator)->step(mSystemModel, fieldVoltage, mechPower, time);
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

		leftSideVectorLog.LogDataLine(getTime(), getLeftSideVector());
		rightSideVectorLog.LogDataLine(getTime(), getRightSideVector());

		if (mSystemModel.getSimType() == SimulationType::DynPhasor) {
			synGenLogFlux.LogDataLine(mTime, ((SynchronGenerator*)generator)->getFluxes());
			synGenLogVolt.LogDataLine(mTime, ((SynchronGenerator*)generator)->getVoltages());
			synGenLogCurr.LogDataLine(mTime, ((SynchronGenerator*)generator)->getCurrents());
		}
		else {
			synGenLogFlux.LogDataLine(mTime, ((SynchronGeneratorEMT*)generator)->getFluxes());
			synGenLogVolt.LogDataLine(mTime, ((SynchronGeneratorEMT*)generator)->getVoltages());
			synGenLogCurr.LogDataLine(mTime, ((SynchronGeneratorEMT*)generator)->getCurrents());
		}
	}

	if (mTime >= mFinalTime) {
		return 0;
	}
	else {
		return 1;
	}
}

int Simulation::stepGeneratordq(Logger& logger, Logger& leftSideVectorLog, Logger& rightSideVectorLog, BaseComponent* generator,
	Logger& synGenLogFlux, Logger& synGenLogVolt, Logger& synGenLogCurr, Real fieldVoltage, Real mechPower, Real logTimeStep, Real& lastLogTime, Real time)
{
	
	// Individual step function for generator
	((SynchronGeneratorEMTdq*)generator)->step(mSystemModel, fieldVoltage, mechPower, time);

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

	//	leftSideVectorLog.LogDataLine(getTime(), getLeftSideVector());
	//	rightSideVectorLog.LogDataLine(getTime(), getRightSideVector());

		synGenLogFlux.LogDataLine(mTime, ((SynchronGeneratorEMTdq*)generator)->getFluxes());
		synGenLogVolt.LogDataLine(mTime, ((SynchronGeneratorEMTdq*)generator)->getVoltages());
		synGenLogCurr.LogDataLine(mTime, ((SynchronGeneratorEMTdq*)generator)->getCurrents());

	}

	if (mTime >= mFinalTime) {
		return 0;
	}
	else {
		return 1;
	}
}


int Simulation::stepGeneratorVBR(Logger& logger, BaseComponent* generator,
 Logger& synGenLogVolt, Logger& synGenLogCurr, Real fieldVoltage, Real mechPower, Real logTimeStep, Real& lastLogTime, Real time)
{

	// Individual step function for generator
	((VoltageBehindReactanceEMT*)generator)->step(mSystemModel, fieldVoltage, mechPower, time);


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
			synGenLogVolt.LogDataLine(mTime, ((VoltageBehindReactanceEMT*)generator)->getVoltages());
			synGenLogCurr.LogDataLine(mTime, ((VoltageBehindReactanceEMT*)generator)->getCurrents());
	
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

void Simulation::addExternalInterface(ExternalInterface *eint) {
	this->mExternalInterfaces.push_back(eint);
}

void Simulation::setNumericalMethod(NumericalMethod numMethod) {
	mSystemModel.setNumMethod(numMethod);
}
