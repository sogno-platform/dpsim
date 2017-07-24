#include "Simulation.h"

#include <signal.h>
#include <sys/timerfd.h>
#include <time.h>
#include <unistd.h>

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
	mSystemModel.setRightSideVectorToZero();
	
	for (auto it = mExternalInterfaces.begin(); it != mExternalInterfaces.end(); ++it) {
		(*it)->readValues();
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

void Simulation::alarmHandler(int sig, siginfo_t* si, void* ctx) {
	Simulation *sim = static_cast<Simulation*>(si->si_value.sival_ptr);
	/* only throw an exception if we're actually behind */
	if (++sim->mRtTimerCount * sim->mSystemModel.getTimeStep() > sim->mTime)
		throw TimerExpiredException();
}

void Simulation::runRTSignal(Logger& logger, bool do_sigwait) {
	int ret, sig;
	sigset_t alrmset;
	struct sigaction sa;
	struct sigevent evp;
	struct itimerspec ts;
	timer_t timer;

	sa.sa_sigaction = Simulation::alarmHandler;
	sa.sa_flags = SA_SIGINFO;
	sigemptyset(&sa.sa_mask);
	if (sigaction(SIGALRM, &sa, NULL)) {
		std::perror("Failed to establish SIGALRM handler");
		std::exit(1);
	}

	evp.sigev_notify = SIGEV_SIGNAL;
	evp.sigev_signo = SIGALRM;
	evp.sigev_value.sival_ptr = this;
	if (timer_create(CLOCK_MONOTONIC, &evp, &timer)) {
		std::perror("Failed to create timer");
		std::exit(1);
	}

	if (do_sigwait) {
		sigemptyset(&alrmset);
		sigaddset(&alrmset, SIGALRM);
	}

	ts.it_value.tv_sec = (time_t) mSystemModel.getTimeStep();
	ts.it_value.tv_nsec = (long) (mSystemModel.getTimeStep() * 1e9);
	ts.it_interval = ts.it_value;
	if (timer_settime(timer, 0, &ts, NULL)) {
		std::perror("Failed to arm timer");
		std::exit(1);
	}
	while (true) {
		try {
			ret = step(logger);
		} catch (TimerExpiredException& e) {
			// TODO recover here
			std::cerr << "timestep expired at " << mTime << std::endl;
			std::abort();
		}
		/* explicit synchronization?
		 * best way for real cases is probably implicit synchronization by
		 * blocking reads from the shmem interface */
		if (do_sigwait)
			sigwait(&alrmset, &sig);
		increaseByTimeStep();
		if (!ret)
			break;
	}
	timer_delete(timer);
}

void Simulation::runRTTimerfd(Logger& logger) {
	int timerfd;
	struct itimerspec ts;
	char timebuf[8];
	uint64_t overrun;

	timerfd = timerfd_create(CLOCK_MONOTONIC, 0);
	if (timerfd < 0) {
		std::perror("Failed to create timerfd");
		std::exit(1);
	}

	ts.it_value.tv_sec = (time_t) mSystemModel.getTimeStep();
	ts.it_value.tv_nsec = (long) (mSystemModel.getTimeStep() * 1e9);
	ts.it_interval = ts.it_value;

	if (timerfd_settime(timerfd, 0, &ts, 0) < 0) {
		std::perror("Failed to arm timerfd");
		std::exit(1);
	}
	while (step(logger)) {
		if (read(timerfd, timebuf, 8) < 0) {
			std::perror("Read from timerfd failed");
			std::exit(1);
		}
		overrun = *((uint64_t*) timebuf);
		if (overrun > 1) {
			std::cerr << "timerfd overrun of " << overrun-1 << " at " << mTime << std::endl;
			std::abort();
		}
	}
	close(timerfd);
}

int Simulation::step(Logger& logger, Logger& leftSideVectorLog, Logger& rightSideVectorLog)  {
	int retValue = step(logger);

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
