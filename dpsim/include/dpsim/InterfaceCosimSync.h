// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include <dpsim/Interface.h>

namespace DPsim {

class InterfaceCosimSync : public Interface,
                           public SharedFactory<InterfaceCosimSync> {
public:
  enum class Role { Leader, Follower };

  struct ConfigNs {
    uint64_t start_time_ns;
    uint64_t time_step_ns;
    uint64_t duration_ns;
  };

  InterfaceCosimSync(const String &name, const String &host, uint16_t port,
                     Role role)
      : Interface(name), mHost(host), mPort(port), mRole(role) {}

  ~InterfaceCosimSync() override {
    if (mOpened)
      close();
  }

  void open() override;
  void close() override;
  void syncExports() override {}
  void syncImports() override {}
  CPS::Task::List getTasks() override { return CPS::Task::List(); }

  static constexpr uint64_t DEFAULT_TIMEOUT_MS = 60000;

  bool publishConfig(const std::chrono::system_clock::time_point &startAt,
                     uint64_t timeStepNs, uint64_t durationNs,
                     uint32_t expectedFollowers = 1,
                     uint64_t timeoutMs = DEFAULT_TIMEOUT_MS);

  bool waitForConfig(ConfigNs &outCfg, uint64_t timeoutMs = DEFAULT_TIMEOUT_MS);

  static std::chrono::system_clock::time_point
  toTimePoint(uint64_t nsSinceEpoch);
  static uint64_t toEpochNs(const std::chrono::system_clock::time_point &tp);

private:
  String mHost;
  uint16_t mPort;
  Role mRole;

  int mListenFd = -1;

  static constexpr size_t WIRE_SIZE = 32;
  static constexpr uint32_t MAGIC = 0x44505353;
  static constexpr uint32_t VERSION = 1;
  static constexpr uint32_t ACK = 0x4441434B;
  static constexpr uint32_t GO = 0x44474F21;

  void openListener();
  int connectToLeader(uint64_t timeoutMs);
  int acceptFollower(uint64_t deadlineMs, bool forever);
  static bool armRecvTimeout(int fd, uint64_t deadlineMs, bool forever);
  static bool sendAll(int fd, const void *buf, size_t len);
  static bool recvAll(int fd, void *buf, size_t len);
};

} // namespace DPsim
