/**
 * @file
 * @author Steffen Vogel <stvogel@eonerc.rwth-aachen.de>
 * @copyright 2018, Institute for Automation of Complex Power Systems, EONERC
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

#pragma once

#include <cstdint>

#include <dpsim/Config.h>

#ifdef _WIN32
  #include <winsock2.h>
#else
  #include <sys/socket.h>
#endif

#ifdef _WIN32
using Socket = SOCKET;
#else
using Socket = int;
#endif

namespace DPsim {
namespace Python {

class EventChannel {

protected:

	/// Socketpair for asynchronous inter-process communication (IPC) to the Python world
	Socket mFds[2];

public:
	EventChannel();
	~EventChannel();

	int fd();

	void sendEvent(uint32_t evt);
};

}
}
