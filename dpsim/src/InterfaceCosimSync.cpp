// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <thread>
#include <vector>

#include <arpa/inet.h>
#include <endian.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <dpsim/InterfaceCosimSync.h>

using namespace DPsim;
using namespace CPS;

namespace {
void packU32(uint8_t *p, uint32_t v) {
  v = htobe32(v);
  std::memcpy(p, &v, 4);
}
void packU64(uint8_t *p, uint64_t v) {
  v = htobe64(v);
  std::memcpy(p, &v, 8);
}
uint32_t unpackU32(const uint8_t *p) {
  uint32_t v;
  std::memcpy(&v, p, 4);
  return be32toh(v);
}
uint64_t unpackU64(const uint8_t *p) {
  uint64_t v;
  std::memcpy(&v, p, 8);
  return be64toh(v);
}

uint64_t nowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

void setNonBlocking(int fd, bool nb) {
  int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0)
    return;
  if (nb)
    flags |= O_NONBLOCK;
  else
    flags &= ~O_NONBLOCK;
  ::fcntl(fd, F_SETFL, flags);
}

bool remainingTv(uint64_t deadlineMs, timeval &tv) {
  uint64_t now = nowMs();
  if (now >= deadlineMs)
    return false;
  uint64_t rem = deadlineMs - now;
  tv.tv_sec = static_cast<time_t>(rem / 1000);
  tv.tv_usec = static_cast<suseconds_t>((rem % 1000) * 1000);
  return true;
}
} // namespace

void InterfaceCosimSync::open() {
  if (mOpened)
    return;

  if (mRole == Role::Leader)
    openListener();

  mOpened = true;
}

void InterfaceCosimSync::close() {
  if (!mOpened)
    return;

  if (mListenFd >= 0) {
    ::close(mListenFd);
    mListenFd = -1;
  }
  mOpened = false;
}

void InterfaceCosimSync::openListener() {
  int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    SPDLOG_LOGGER_ERROR(mLog, "CosimSync: socket failed: {}",
                        std::strerror(errno));
    throw SystemError("CosimSync: socket failed");
  }

  int one = 1;
  ::setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
  setNonBlocking(fd, true);

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(mPort);

  if (::bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    SPDLOG_LOGGER_ERROR(mLog, "CosimSync: bind to port {} failed: {}", mPort,
                        std::strerror(errno));
    ::close(fd);
    throw SystemError("CosimSync: bind failed");
  }
  if (::listen(fd, SOMAXCONN) < 0) {
    SPDLOG_LOGGER_ERROR(mLog, "CosimSync: listen failed: {}",
                        std::strerror(errno));
    ::close(fd);
    throw SystemError("CosimSync: listen failed");
  }

  mListenFd = fd;
}

int InterfaceCosimSync::connectToLeader(uint64_t timeoutMs) {
  bool forever = (timeoutMs == 0);
  uint64_t deadline = forever ? 0 : nowMs() + timeoutMs;

  addrinfo hints{};
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_STREAM;
  std::string portStr = std::to_string(mPort);

  for (;;) {
    addrinfo *res = nullptr;
    int gai = ::getaddrinfo(mHost.c_str(), portStr.c_str(), &hints, &res);
    if (gai == 0) {
      for (addrinfo *ai = res; ai != nullptr; ai = ai->ai_next) {
        int fd = ::socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0)
          continue;
        setNonBlocking(fd, true);

        int rc = ::connect(fd, ai->ai_addr, ai->ai_addrlen);
        if (rc == 0) {
          setNonBlocking(fd, false);
          ::freeaddrinfo(res);
          return fd;
        }
        if (errno == EINPROGRESS) {
          timeval tv{};
          timeval *ptv = nullptr;
          if (!forever) {
            if (!remainingTv(deadline, tv)) {
              ::close(fd);
              ::freeaddrinfo(res);
              return -1;
            }
            ptv = &tv;
          }
          fd_set wfds;
          FD_ZERO(&wfds);
          FD_SET(fd, &wfds);
          int r = ::select(fd + 1, nullptr, &wfds, nullptr, ptv);
          if (r > 0) {
            int soErr = 0;
            socklen_t len = sizeof(soErr);
            if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &soErr, &len) == 0 &&
                soErr == 0) {
              setNonBlocking(fd, false);
              ::freeaddrinfo(res);
              return fd;
            }
          }
        }
        ::close(fd);
      }
      ::freeaddrinfo(res);
    } else {
      SPDLOG_LOGGER_WARN(mLog, "CosimSync: getaddrinfo({}) failed: {}", mHost,
                         ::gai_strerror(gai));
    }

    if (!forever) {
      uint64_t now = nowMs();
      if (now >= deadline)
        return -1;
      uint64_t rem = deadline - now;
      std::this_thread::sleep_for(
          std::chrono::milliseconds(rem < 50 ? rem : 50));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
}

int InterfaceCosimSync::acceptFollower(uint64_t deadlineMs, bool forever) {
  for (;;) {
    timeval tv{};
    timeval *ptv = nullptr;
    if (!forever) {
      if (!remainingTv(deadlineMs, tv))
        return -1;
      ptv = &tv;
    }
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(mListenFd, &rfds);
    int r = ::select(mListenFd + 1, &rfds, nullptr, nullptr, ptv);
    if (r < 0) {
      if (errno == EINTR)
        continue;
      SPDLOG_LOGGER_WARN(mLog, "CosimSync: select failed: {}",
                         std::strerror(errno));
      return -1;
    }
    if (r == 0)
      return -1;

    int cfd = ::accept(mListenFd, nullptr, nullptr);
    if (cfd >= 0)
      return cfd;
    if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK ||
        errno == ECONNABORTED)
      continue;
    SPDLOG_LOGGER_WARN(mLog, "CosimSync: accept failed: {}",
                       std::strerror(errno));
    return -1;
  }
}

bool InterfaceCosimSync::armRecvTimeout(int fd, uint64_t deadlineMs,
                                        bool forever) {
  if (forever)
    return true;
  timeval tv{};
  if (!remainingTv(deadlineMs, tv))
    return false;
  ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  return true;
}

bool InterfaceCosimSync::sendAll(int fd, const void *buf, size_t len) {
  const uint8_t *p = static_cast<const uint8_t *>(buf);
  size_t sent = 0;
  while (sent < len) {
    ssize_t n = ::send(fd, p + sent, len - sent, MSG_NOSIGNAL);
    if (n < 0) {
      if (errno == EINTR)
        continue;
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  return true;
}

bool InterfaceCosimSync::recvAll(int fd, void *buf, size_t len) {
  uint8_t *p = static_cast<uint8_t *>(buf);
  size_t got = 0;
  while (got < len) {
    ssize_t n = ::recv(fd, p + got, len - got, 0);
    if (n == 0)
      return false;
    if (n < 0) {
      if (errno == EINTR)
        continue;
      return false;
    }
    got += static_cast<size_t>(n);
  }
  return true;
}

bool InterfaceCosimSync::publishConfig(
    const std::chrono::system_clock::time_point &startAt, uint64_t timeStepNs,
    uint64_t durationNs, uint32_t expectedFollowers, uint64_t timeoutMs) {
  if (!mOpened || mRole != Role::Leader)
    throw SystemError("CosimSync: publishConfig called in wrong state");
  if (timeStepNs == 0)
    throw SystemError("CosimSync: time step must be greater than zero");
  if (expectedFollowers == 0)
    throw SystemError("CosimSync: at least one follower is required");

  auto delta = startAt - std::chrono::system_clock::now();
  if (delta > std::chrono::hours(1) || delta < -std::chrono::hours(1)) {
    double secs =
        std::chrono::duration_cast<std::chrono::duration<double>>(delta)
            .count();
    SPDLOG_LOGGER_WARN(
        mLog, "CosimSync: start time is more than 1 hour away ({} s).", secs);
  }

  static_assert(WIRE_SIZE == 32, "wire layout is 4+4+8+8+8 bytes");
  uint8_t wire[WIRE_SIZE];
  packU32(wire + 0, MAGIC);
  packU32(wire + 4, VERSION);
  packU64(wire + 8, toEpochNs(startAt));
  packU64(wire + 16, timeStepNs);
  packU64(wire + 24, durationNs);

  bool forever = (timeoutMs == 0);
  uint64_t deadline = forever ? 0 : nowMs() + timeoutMs;

  std::vector<int> followerFds;
  followerFds.reserve(expectedFollowers);
  bool ok = true;

  for (uint32_t i = 0; i < expectedFollowers && ok; i++) {
    int cfd = acceptFollower(deadline, forever);
    if (cfd < 0) {
      SPDLOG_LOGGER_WARN(mLog, "CosimSync: follower {} did not connect.", i);
      ok = false;
      break;
    }
    followerFds.push_back(cfd);

    int one = 1;
    ::setsockopt(cfd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    uint8_t ack[4];
    ok = armRecvTimeout(cfd, deadline, forever) &&
         sendAll(cfd, wire, WIRE_SIZE) && recvAll(cfd, ack, sizeof(ack)) &&
         unpackU32(ack) == ACK;
    if (!ok)
      SPDLOG_LOGGER_WARN(
          mLog, "CosimSync: follower {} did not acknowledge the config.", i);
  }

  if (ok) {
    uint8_t go[4];
    packU32(go, GO);
    for (size_t i = 0; i < followerFds.size(); i++) {
      if (!sendAll(followerFds[i], go, sizeof(go))) {
        SPDLOG_LOGGER_WARN(mLog, "CosimSync: failed to release follower {}.",
                           i);
        ok = false;
      }
    }
  }

  for (int fd : followerFds)
    ::close(fd);
  return ok;
}

bool InterfaceCosimSync::waitForConfig(ConfigNs &outCfg, uint64_t timeoutMs) {
  if (!mOpened || mRole != Role::Follower)
    throw SystemError("CosimSync: waitForConfig called in wrong state");

  bool forever = (timeoutMs == 0);
  uint64_t deadline = forever ? 0 : nowMs() + timeoutMs;

  uint64_t connectBudget = timeoutMs;
  if (!forever) {
    timeval tmp{};
    if (!remainingTv(deadline, tmp))
      return false;
    connectBudget =
        static_cast<uint64_t>(tmp.tv_sec) * 1000 + tmp.tv_usec / 1000;
    if (connectBudget == 0)
      connectBudget = 1;
  }

  int fd = connectToLeader(connectBudget);
  if (fd < 0)
    return false;

  if (!armRecvTimeout(fd, deadline, forever)) {
    ::close(fd);
    return false;
  }

  uint8_t wire[WIRE_SIZE];
  if (!recvAll(fd, wire, WIRE_SIZE)) {
    ::close(fd);
    SPDLOG_LOGGER_ERROR(mLog, "CosimSync: incomplete config from leader.");
    return false;
  }

  uint32_t magic = unpackU32(wire + 0);
  uint32_t version = unpackU32(wire + 4);
  if (magic != MAGIC || version != VERSION) {
    ::close(fd);
    SPDLOG_LOGGER_ERROR(mLog,
                        "CosimSync: invalid header (magic=0x{:x}, version={}).",
                        magic, version);
    return false;
  }
  outCfg.start_time_ns = unpackU64(wire + 8);
  outCfg.time_step_ns = unpackU64(wire + 16);
  outCfg.duration_ns = unpackU64(wire + 24);

  if (outCfg.time_step_ns == 0 ||
      outCfg.start_time_ns > static_cast<uint64_t>(INT64_MAX)) {
    ::close(fd);
    SPDLOG_LOGGER_ERROR(mLog,
                        "CosimSync: implausible config (start_time_ns={}, "
                        "time_step_ns={}).",
                        outCfg.start_time_ns, outCfg.time_step_ns);
    return false;
  }

  uint8_t ack[4];
  packU32(ack, ACK);
  if (!sendAll(fd, ack, sizeof(ack))) {
    ::close(fd);
    SPDLOG_LOGGER_ERROR(mLog, "CosimSync: failed to acknowledge config.");
    return false;
  }

  uint8_t go[4];
  bool released = armRecvTimeout(fd, deadline, forever) &&
                  recvAll(fd, go, sizeof(go)) && unpackU32(go) == GO;
  ::close(fd);
  if (!released) {
    SPDLOG_LOGGER_ERROR(mLog, "CosimSync: leader did not release the group.");
    return false;
  }

  auto delta =
      toTimePoint(outCfg.start_time_ns) - std::chrono::system_clock::now();
  if (delta > std::chrono::hours(1) || delta < -std::chrono::hours(1)) {
    double secs =
        std::chrono::duration_cast<std::chrono::duration<double>>(delta)
            .count();
    SPDLOG_LOGGER_WARN(
        mLog, "CosimSync: received start time is more than 1 hour away ({} s).",
        secs);
  }
  return true;
}

std::chrono::system_clock::time_point
InterfaceCosimSync::toTimePoint(uint64_t nsSinceEpoch) {
  using TP = std::chrono::time_point<std::chrono::system_clock,
                                     std::chrono::nanoseconds>;
  return TP(std::chrono::nanoseconds(nsSinceEpoch));
}

uint64_t
InterfaceCosimSync::toEpochNs(const std::chrono::system_clock::time_point &tp) {
  auto ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(tp)
                .time_since_epoch()
                .count();
  return static_cast<uint64_t>(ns);
}
