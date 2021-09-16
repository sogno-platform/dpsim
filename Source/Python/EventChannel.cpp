/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <dpsim/Python/EventChannel.h>
#include <cps/Definitions.h>

#ifdef _WIN32
  #include <winsock2.h>
#else
  #include <sys/socket.h>
#endif

using namespace DPsim::Python;

void EventChannel::addFd(int fd) {
	mFds.push_back(fd);
}

void EventChannel::removeFd(int fd) {
	for (auto it = mFds.begin(); it != mFds.end(); ) {
		if (*it == fd)
			it = mFds.erase(it);
		else
			++it;
	}
}

void EventChannel::sendEvent(uint32_t evt) {
	int ret;

	for (int fd : mFds) {
		ret = send(fd, (char *) &evt, 4, 0);
		if (ret < 0)
			throw CPS::SystemError("Failed notify");
	}
}
