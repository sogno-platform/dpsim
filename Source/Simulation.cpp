/** The simulation
 *
 * @file
 * @author Markus Mirz <mmirz@eonerc.rwth-aachen.de>
 * @copyright 2017, Institute for Automation of Complex Power Systems, EONERC
 * @license GNU General Public License (version 3)
 *
 * DPsim
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *********************************************************************************/

#include "Simulation.h"

#ifdef __linux__
#include <signal.h>
#include <sys/timerfd.h>
#include <time.h>
#include <unistd.h>
#endif

using namespace DPsim;

Simulation::Simulation() {
	mTime = 0;
	mLastLogTimeStep = 0;
	mCurrentSwitchTimeIndex = 0;
}

Simulation::Simulation(ElementList elements, Real om, Real dt, Real tf, Logger& logger, SimulationType simType)
	: Simulation() {

	mLogger = &logger;
	mSystemModel.setSimType(simType);
	mSystemModel.setTimeStep(dt);
	mSystemModel.setOmega(om);
	mFinalTime = tf;
	initialize(elements);

	for (ElementPtr c : elements)
		mLogger->Log(LogLevel::INFO) << "Added " << c->getType() << " '" << c->getName() << "' to simulation." << std::endl;

	mLogger->Log(LogLevel::INFO) << "System matrix A:" << std::endl;
	mLogger->LogMatrix(LogLevel::INFO, mSystemModel.getCurrentSystemMatrix());
	mLogger->Log(LogLevel::INFO) << "LU decomposition:" << std::endl;
	mLogger->LogMatrix(LogLevel::INFO, mSystemModel.getLUdecomp());
	mLogger->Log(LogLevel::INFO) << "Known variables matrix j:" << std::endl;
	mLogger->LogMatrix(LogLevel::INFO, mSystemModel.getRightSideVector());
}

Simulation::Simulation(ElementList elements, Real om, Real dt, Real tf, Logger& logger, Int downSampleRate, SimulationType simType)
	: Simulation(elements, om, dt, tf, logger, simType) {

	mDownSampleRate = downSampleRate;
}


Simulation::~Simulation() {

}


void Simulation::initialize(ElementList newElements) {
	Int maxNode = 0;
	Int currentVirtualNode = 0;

	mLogger->Log(LogLevel::INFO) << "#### Start Initialization ####" << std::endl;
	// Calculate the mNumber of nodes by going through the list of elements
	// TODO we use the values from the first element vector right now and assume that
	// these values don't change on switches
	for (ElementPtr element : newElements) {

		// determine maximum node in component list
		if (element->getNode1() > maxNode) {
			maxNode = element->getNode1();
		}
		if (element->getNode2() > maxNode) {
			maxNode = element->getNode2();
		}		
	}
	mLogger->Log(LogLevel::INFO) << "Maximum node number: " << maxNode << std::endl;

	// Check if element requires virtual node and if so set one
	for (ElementPtr element : newElements) {
		if (element->hasVirtualNodes()) {
			for (Int node = 0; node < element->getVirtualNodesNum(); node++) {
				currentVirtualNode++;
				element->setVirtualNode(node, currentVirtualNode);
				mLogger->Log(LogLevel::INFO) << "Created virtual node "<< node << "=" << currentVirtualNode 
					<< " for " << element->getName() << std::endl;
			}
		}
	}

	// Calculate size of system matrix
	Int numNodes = maxNode + currentVirtualNode + 1;

	// Create right and left vector
	mSystemModel.initialize(numNodes);
	
	// Initialize right side vector and components
	for (ElementPtr element : newElements) {
		element->init(mSystemModel.getOmega(), mSystemModel.getTimeStep());
		element->applyRightSideVectorStamp(mSystemModel);
	}

	// Create new system matrix and apply matrix stamps
	addSystemTopology(newElements);

	switchSystemMatrix(0);
	mElements = mElementsVector[0];
}

void Simulation::addSystemTopology(ElementList newElements) {
	mElementsVector.push_back(newElements);
	
	// It is assumed that the system size does not change
	mSystemModel.createEmptySystemMatrix();

	for (ElementPtr element : newElements) {
		element->applySystemMatrixStamp(mSystemModel);
	}
	mSystemModel.addSystemMatrix();
}


Int Simulation::step(Logger& logger, bool blocking)
{
	mSystemModel.setRightSideVectorToZero();

	for (auto it = mExternalInterfaces.begin(); it != mExternalInterfaces.end(); ++it) {
		(*it)->readValues(blocking);
	}

	for (ElementList::iterator it = mElements.begin(); it != mElements.end(); ++it) {
		(*it)->step(mSystemModel, mTime);
	}

	mSystemModel.solve();

	for (ElementList::iterator it = mElements.begin(); it != mElements.end(); ++it) {
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

Int Simulation::step(Logger& logger, Logger& leftSideVectorLog, Logger& rightSideVectorLog, bool blocking) {
	Int retValue = step(logger, blocking);

	leftSideVectorLog.LogDataLine(getTime(), getLeftSideVector());
	rightSideVectorLog.LogDataLine(getTime(), getRightSideVector());

	return retValue;
}

Int Simulation::stepGeneratorTest(Logger& logger, Logger& leftSideVectorLog, Logger& rightSideVectorLog,
	ElementPtr generator, Real time) {
	// Set to zero because all components will add their contribution for the current time step to the current value
	mSystemModel.getRightSideVector().setZero();

	// Execute step for all circuit components
	for (ElementList::iterator it = mElements.begin(); it != mElements.end(); ++it) {
		(*it)->step(mSystemModel, mTime);
	}

	// Solve circuit for vector j with generator output current
	mSystemModel.solve();

	// Execute PostStep for all components, generator states are recalculated based on new terminal voltage
	for (ElementList::iterator it = mElements.begin(); it != mElements.end(); ++it) {
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
	if (mLastLogTimeStep == 0) {
		std::cout << mTime << std::endl;
		leftSideVectorLog.LogDataLine(getTime(), getLeftSideVector());
		rightSideVectorLog.LogDataLine(getTime(), getRightSideVector());
	}

	mLastLogTimeStep++;
	if (mLastLogTimeStep == mDownSampleRate) {
		mLastLogTimeStep = 0;
	}

	if (mTime >= mFinalTime) {
		return 0;
	}
	else {
		return 1;
	}
}

int Simulation::stepGeneratorVBR(Logger& logger, Logger& leftSideVectorLog, Logger& rightSideVectorLog,
	ElementPtr generator, Real time) {

	// Set to zero because all components will add their contribution for the current time step to the current value
	mSystemModel.getRightSideVector().setZero();

	// Execute step for all circuit components
	for (ElementList::iterator it = mElements.begin(); it != mElements.end(); ++it) {
		(*it)->step(mSystemModel, mTime);
	}

	// Solve circuit for vector j with generator output current
	mSystemModel.solve();

	// Execute PostStep for all components, generator states are recalculated based on new terminal voltage
	for (ElementList::iterator it = mElements.begin(); it != mElements.end(); ++it) {
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
	if (mLastLogTimeStep == 0) {
		std::cout << mTime << std::endl;
		leftSideVectorLog.LogDataLine(getTime(), getLeftSideVector());
		rightSideVectorLog.LogDataLine(getTime(), getRightSideVector());
	}

	mLastLogTimeStep++;
	if (mLastLogTimeStep == mDownSampleRate) {
		mLastLogTimeStep = 0;
	}

	if (mTime >= mFinalTime) {
		return 0;
	}
	else {
		return 1;
	}
}

void Simulation::switchSystemMatrix(Int systemMatrixIndex) {
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

#ifdef __linux__
void Simulation::alarmHandler(int sig, siginfo_t* si, void* ctx) {
	Simulation *sim = static_cast<Simulation*>(si->si_value.sival_ptr);
	/* only throw an exception if we're actually behind */
	if (++sim->mRtTimerCount * sim->mSystemModel.getTimeStep() > sim->mTime)
		throw TimerExpiredException();
}

void Simulation::runRT(RTMethod rtMethod, bool startSynch, Logger& logger, Logger& llogger, Logger& rlogger ) {
	char timebuf[8];
	int ret, sig, timerfd;
	sigset_t alrmset;
	struct sigaction sa;
	struct sigevent evp;
	struct itimerspec ts;
	timer_t timer;
	uint64_t overrun;

	// initialize timer / timerfd
	if (rtMethod == RTTimerFD) {
		timerfd = timerfd_create(CLOCK_MONOTONIC, 0);
		if (timerfd < 0) {
			std::perror("Failed to create timerfd");
			std::exit(1);
		}
	} else if (rtMethod == RTExceptions) {
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

		sigemptyset(&alrmset);
		sigaddset(&alrmset, SIGALRM);
	} else {
		std::cerr << "invalid rt method, exiting" << std::endl;
		std::exit(1);
	}

	ts.it_value.tv_sec = (time_t) mSystemModel.getTimeStep();
	ts.it_value.tv_nsec = (long) (mSystemModel.getTimeStep() * 1e9);
	ts.it_interval = ts.it_value;

	// optional start synchronization
	if (startSynch) {
		step(logger, llogger, rlogger, false); // first step, sending the initial values
		step(logger, llogger, rlogger, true); // blocking step for synchronization + receiving the initial state of the other network
		increaseByTimeStep();
	}

	// arm timer
	if (rtMethod == RTTimerFD) {
		if (timerfd_settime(timerfd, 0, &ts, 0) < 0) {
			std::perror("Failed to arm timerfd");
			std::exit(1);
		}
	} else if (rtMethod == RTExceptions) {
		if (timer_settime(timer, 0, &ts, NULL)) {
			std::perror("Failed to arm timer");
			std::exit(1);
		}
	}

	// main loop
	do {
		if (rtMethod == RTExceptions) {
			try {
				ret = step(logger, llogger, rlogger, false);
				sigwait(&alrmset, &sig);
			} catch (TimerExpiredException& e) {
				std::cerr << "timestep expired at " << mTime << std::endl;
			}
		} else if (rtMethod == RTTimerFD) {
			ret = step(logger, llogger, rlogger, false);
			if (read(timerfd, timebuf, 8) < 0) {
				std::perror("Read from timerfd failed");
				std::exit(1);
			}
			overrun = *((uint64_t*) timebuf);
			if (overrun > 1) {
				std::cerr << "timerfd overrun of " << overrun-1 << " at " << mTime << std::endl;
			}
		}
		increaseByTimeStep();
		if (!ret)
			break;
	} while (ret);

	// cleanup
	if (rtMethod == RTTimerFD) {
		close(timerfd);
	} else if (rtMethod == RTExceptions) {
	timer_delete(timer);
	}
}
#endif
