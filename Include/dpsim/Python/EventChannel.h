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

#include <cps/Definitions.h>

#ifdef WITH_PIPE
  #include <unistd.h>
#endif

namespace DPsim {
namespace Python {

class EventChannel {

protected:
#ifdef HAVE_PIPE
	/// Pipe for asynchronous inter-process communication (IPC) to the Python world
	int mPipe[2];
#endif /* HAVE_PIPE */

public:
	EventChannel() {
#ifdef HAVE_PIPE
		int ret;

		ret = pipe(mPipe);
		if (ret < 0)
			throw CPS::SystemError("Failed to create pipe");
#endif
	}

	~EventChannel() {
#ifdef HAVE_PIPE
		if (mPipe[0] >= 0) {
			close(mPipe[0]);
			close(mPipe[1]);
		}
#endif /* HAVE_PIPE */
	}

	int fd() {
		return mPipe[0];
	}

	void sendEvent(uint32_t evt) {
#ifdef HAVE_PIPE
		int ret;

		ret = write(mPipe[1], &evt, 4);
		if (ret < 0)
			throw CPS::SystemError("Failed notify");
#endif /* HAVE_PIPE */
	}

};

}
}
