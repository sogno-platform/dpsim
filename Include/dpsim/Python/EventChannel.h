/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <cstdint>
#include <vector>

#include <dpsim/Config.h>

namespace DPsim {
namespace Python {

class EventChannel {

protected:

	std::vector<int> mFds;

public:
	void addFd(int fd);
	void removeFd(int fd);

	void sendEvent(uint32_t evt);
};

}
}
