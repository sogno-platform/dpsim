---
title: "Starting Together"
linkTitle: "Starting Together"
date: 2026-08-05
description: >
  Getting separately launched real-time processes to begin the same run at the same instant.
weight: 7
---

A co-simulation is usually more than one process, and they are started by hand, by a script or by a
scheduler that offers no guarantee about ordering. Whichever starts first sits there stepping
against a partner that does not exist yet. In a
[real-time]({{< ref "real-time.md" >}}) run that is not a startup nuisance to be waited out: the
early process has already consumed wall-clock time its partner never simulated, and the two are
offset by that amount for the rest of the run.

`InterfaceCosimSync` removes the question of who was launched first. One process is the leader; the
others are followers. The leader decides an absolute start instant, in the future, and hands it to
everyone. Each process then passes that instant to its real-time simulation, and they all begin
together regardless of the order they were started in.

## The handshake

The leader listens on a TCP port. Every follower connects to it, receives the start instant together
with the time step and duration, and acknowledges what it received. Only once **all** the expected
followers have acknowledged does the leader release them, and only then does a follower return
successfully.

That second phase is what makes the group atomic. A follower that has connected and acknowledged is
not yet free to run; it is waiting to be released. If any other follower fails to appear, the leader
never sends the release, and every follower that did appear fails too.

{{% alert title="An incomplete group starts nobody" color="warning" %}}
This is the property worth relying on. A partial co-simulation is worse than a failed one: the
processes that did start produce output that looks entirely normal while being coupled to nothing.
Because the release is withheld from everyone unless the whole group is ready, that outcome cannot
arise from a missing peer.
{{% /alert %}}

Neither side blocks forever by default. Both calls take a timeout in milliseconds, defaulting to
60 s, and report failure rather than hanging when a peer never arrives, which is the appropriate
behaviour for an unattended run. Passing `0` restores indefinite waiting if you want it.

## Using it

The leader publishes; the followers wait. `expected_followers` is how many the leader will hold out
for.

```python
import time
import dpsimpy

leader = dpsimpy.InterfaceCosimSync("leader", "", 47129, "leader")
leader.open()

start_time_ns = time.time_ns() + 1_000_000_000
ok = leader.publish_config(
    start_time_ns,
    time_step_ns=1_000_000,
    duration_ns=1_000_000_000,
    expected_followers=3,
    timeout_ms=10000,
)
leader.close()
```

```python
follower = dpsimpy.InterfaceCosimSync("follower", "10.0.0.1", 47129, "follower")
follower.open()
ok, start_time_ns, time_step_ns, duration_ns = follower.wait_for_config(timeout_ms=10000)
follower.close()
```

Both report a boolean. Nothing should start unless it is true, on either side. The start instant is
nanoseconds since the `system_clock` epoch, which is what `time.time_ns()` returns, and it crosses
the wire as an exact integer so that no rounding creeps in between the processes.

The role is `"leader"` or `"follower"`; anything else raises. A misspelled role would otherwise
leave the group with no leader, and the run would fail later as a timeout with nothing pointing at
the cause.

The host argument is where the follower reaches the leader; the leader ignores it and binds on all
interfaces. Addressing is IPv4 only. The transport is TCP, so the processes may sit on separate
hosts, and an external real-time target can join the same rendezvous.

Both calls release the Python GIL while they block, so a leader and its followers can be driven from
threads of one interpreter, and unrelated Python threads keep running through the wait.

## Starting the simulation on it

`RealTimeSimulation.run_at` takes the instant the rendezvous produced and blocks until it arrives
before stepping:

```python
sim = dpsimpy.RealTimeSimulation("follower")
sim.set_system(system)
sim.set_time_step(time_step_ns / 1e9)
sim.set_final_time(duration_ns / 1e9)
sim.run_at(start_time_ns)
```

It takes nanoseconds rather than a `datetime`, which is only microsecond-resolution and would
discard the precision the exchange was built to preserve. `run` remains the relative form: it starts
a given number of seconds from now, which is per-process and therefore not a rendezvous.

## On the wire

The exchange is three fixed-size messages, so a peer that is not DPsim can implement it. All fields
are big-endian, and the config is packed field by field, so layout, padding and endianness stay
host-independent.

| Step | Direction | Bytes | Contents |
| --- | --- | --- | --- |
| 1 | leader → follower | 32 | `magic` (u32, `DPSS`), `version` (u32, currently 1), `start_time_ns` (u64), `time_step_ns` (u64), `duration_ns` (u64) |
| 2 | follower → leader | 4 | `DACK`, sent only after the header and the config validate |
| 3 | leader → follower | 4 | `DGO!`, sent only after every follower has acknowledged |

A follower rejects a mismatched magic or version, a zero `time_step_ns`, and a `start_time_ns` above
`INT64_MAX`, which would otherwise wrap to a time in the past. The leader rejects a zero time step
or zero expected followers before it opens the exchange. Step 3 is the release: a connection that
closes before it arrives means the group was incomplete.

{{% alert title="Clocks are only as good as the hosts they run on" color="info" %}}
Agreeing on an instant is not the same as agreeing on the time. The processes now hold an identical
target, but each waits for it on its own `system_clock`. Across hosts those clocks agree only to
whatever your time synchronization provides, and any offset between them appears directly as an
offset between the runs. On one machine this does not arise; across machines it becomes the
accuracy limit, and PTP rather than NTP is what closes it.
{{% /alert %}}

This is a Linux-only feature, built when `WITH_RT` is enabled, and it coordinates the start only.
Exchanging data during the run is a separate concern, covered under
[co-simulation]({{< ref "co-simulation.md" >}}).

A worked example with one leader and three followers, including the case where a follower is
missing, is in `examples/Notebooks/Features/CosimStartSync.ipynb`.
