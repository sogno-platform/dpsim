// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include <dpsim/Interface.h>

namespace DPsim {

// One-shot start-time rendezvous for RT co-simulation over TCP: the leader
// listens, all followers connect, then the leader broadcasts an absolute start
// instant, timestep and duration so both sides run(startAt) at the same time.
class InterfaceCosimSync : public Interface,
                           public SharedFactory<InterfaceCosimSync> {
public:
  enum class Role { Leader, Follower };

  struct ConfigNs {
    uint64_t start_time_ns; // epoch: system_clock
    uint64_t time_step_ns;
    uint64_t duration_ns;
  };

  // host is used by followers to reach the leader; a leader ignores it and
  // binds on all interfaces.
  InterfaceCosimSync(const String &name, const String &host, uint16_t port,
                     Role role)
      : Interface(name), mHost(host), mPort(port), mRole(role) {}

  ~InterfaceCosimSync() override {
    if (mOpened)
      close();
  }

  // Interface overrides (no per-step tasks; this is control-path only)
  void open() override;
  void close() override;
  void syncExports() override {}
  void syncImports() override {}
  CPS::Task::List getTasks() override { return CPS::Task::List(); }

  // Leader: send the config to expectedFollowers as each connects, and wait
  // for every one to acknowledge receipt. Blocks until all have acknowledged
  // or timeoutMs elapses (0 = forever); returns false if any did not.
  bool publishConfig(const std::chrono::system_clock::time_point &startAt,
                     uint64_t timeStepNs, uint64_t durationNs,
                     uint32_t expectedFollowers = 1, uint64_t timeoutMs = 0);

  // Follower: connect to the leader and receive the config. Retries until the
  // leader is listening or timeoutMs elapses (0 = forever).
  bool waitForConfig(ConfigNs &outCfg, uint64_t timeoutMs = 0);

  static std::chrono::system_clock::time_point
  toTimePoint(uint64_t nsSinceEpoch);
  static uint64_t toEpochNs(const std::chrono::system_clock::time_point &tp);

private:
  String mHost;
  uint16_t mPort;
  Role mRole;

  int mListenFd = -1; // leader only

  // On-wire config: fixed 32 bytes, all fields big-endian. Never sent as a
  // raw struct, so layout/padding/endianness stay host-independent.
  static constexpr size_t WIRE_SIZE = 32;
  static constexpr uint32_t MAGIC = 0x44505353; // 'DPSS'
  static constexpr uint32_t VERSION = 1;
  static constexpr uint32_t ACK = 0x4441434B; // 'DACK', follower -> leader

  void openListener();
  int connectToLeader(uint64_t timeoutMs);
  static bool sendAll(int fd, const void *buf, size_t len);
  static bool recvAll(int fd, void *buf, size_t len);
};

} // namespace DPsim
