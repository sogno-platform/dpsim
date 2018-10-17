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

using namespace DPsim::Python;

#ifdef _WIN32
// Source: https://github.com/libevent/libevent/blob/master/evutil.c
static int createSocketPair(int family, int type, int protocol, Socket fds[2]) {
	int ret;
	Socket listener = -1;
	Socket connector = -1;
	Socket acceptor = -1;

	struct sockaddr_in listen_addr;
	struct sockaddr_in connect_addr;
	int size;

	int test = family != AF_INET || type != SOCK_STREAM;
#ifdef AF_UNIX
	test = test && (family != AF_UNIX);
#endif
	if (test)
		return -1;

	if (!fds)
		return -1;

	listener = socket(AF_INET, type, 0);
	if (listener < 0)
		return -1;

	memset(&connect_addr, 0, sizeof(connect_addr));
	memset(&listen_addr, 0, sizeof(listen_addr));

	listen_addr.sin_family = AF_INET;
	listen_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	listen_addr.sin_port = 0; // kernel chooses port

	ret = bind(listener, (struct sockaddr *) &listen_addr, sizeof (listen_addr));
	if (ret == -1)
		goto fail;

	ret = listen(listener, 1);
	if (ret == -1)
		goto fail;

	connector = socket(AF_INET, type, 0);
	if (connector < 0)
		goto fail;

	/* We want to find out the port number to connect to.  */
	size = sizeof(connect_addr);
	ret = getsockname(listener, (struct sockaddr *) &connect_addr, &size);
	if (ret == -1)
		goto fail;

	if (size != sizeof(connect_addr))
		goto fail;

	ret = connect(connector, (struct sockaddr *) &connect_addr, sizeof(connect_addr));
	if (ret == -1)
		goto fail;

	size = sizeof(listen_addr);
	acceptor = accept(listener, (struct sockaddr *) &listen_addr, &size);
	if (acceptor < 0)
		goto fail;

	if (size != sizeof(listen_addr))
		goto fail;

	// Now check we are talking to ourself by matching port and host on the two sockets
	ret = getsockname(connector, (struct sockaddr *) &connect_addr, &size);
	if (ret == -1)
		goto fail;

	if (size != sizeof (connect_addr)
		|| listen_addr.sin_family != connect_addr.sin_family
		|| listen_addr.sin_addr.s_addr != connect_addr.sin_addr.s_addr
		|| listen_addr.sin_port != connect_addr.sin_port)
		goto fail;

	closesocket(listener);

	fds[0] = (int) connector;
	fds[1] = (int) acceptor;

	return 0;

fail:
	if (listener != -1)
		closesocket(listener);

	if (connector != -1)
		closesocket(connector);

	if (acceptor != -1)
		closesocket(acceptor);

	return -1;
}

void closeSocketPair(Socket fds[2]) {
	closesocket(fds[0]);
	closesocket(fds[1]);
}
#else
static int createSocketPair(int family, int type, int protocol, Socket fds[2]) {
	return socketpair(family, type, protocol, fds);
}

void closeSocketPair(Socket fds[2]) {
	close(fds[0]);
	close(fds[1]);
}
#endif /* _WIN32 */

EventChannel::EventChannel() {
	int ret;

#ifdef _WIN32
	int af = AF_INET;
#else
	int af = AF_LOCAL;
#endif

	ret = createSocketPair(af, SOCK_STREAM, 0, mFds);
	if (ret < 0)
		throw CPS::SystemError("Failed to create pipe");
	}

EventChannel::~EventChannel() {
	if (mFds[0] >= 0)
		closeSocketPair(mFds);
}

int EventChannel::fd() {
	return (int) mFds[0];
}

void EventChannel::sendEvent(uint32_t evt) {
	int ret;

	ret = send(mFds[1], (char *) &evt, 4, 0);
	if (ret < 0)
		throw CPS::SystemError("Failed notify");
}
