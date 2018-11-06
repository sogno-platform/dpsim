/**
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
