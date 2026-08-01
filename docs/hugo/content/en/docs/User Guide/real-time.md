---
title: "Real-Time"
aliases: ["/docs/getting-started/real-time/"]
linkTitle: "Real-Time"
date: 2025-02-13
description: >
  Running a simulation against a wall clock, and what that requires.
weight: 5
---

Normally a simulation runs as fast as it can: a one-second study finishes in whatever time the
solver needs. In a real-time simulation, one second of simulated time takes one second of wall clock
time, no faster and no slower.

## When you need it

Only when something outside the simulation has its own clock. A controller running on real hardware,
another simulator you are coupled to, or a person turning a dial all move at their own pace, and the
simulation has to move with them. If nothing outside is waiting, running in real time only makes the
study slower.

This is why [co-simulation]({{< ref "co-simulation.md" >}}) and real-time usually appear together.

## What it changes

Two things. The simulation waits at the end of each step until the wall clock catches up, so a run
takes as long as the time it simulates. And a step that takes longer than its own duration is an
**overrun**: the simulation has missed its deadline and can no longer claim to be in step with the
outside world.

Overruns are the whole difficulty. Everything else about real-time execution is arranging for them
not to happen: keeping the step's work bounded, and keeping the operating system from interrupting
it.

{{% alert title="Watch out: an overrun does not stop the simulation" color="warning" %}}
A missed deadline is reported, not fatal. The run continues and its results remain numerically
correct; what is no longer true is that it kept pace with anything external. A run that overran
repeatedly is not a real-time run, however normal its output looks.
{{% /alert %}}

## What it takes

A time step that comfortably exceeds the work done in it, models that do nothing slow inside the
step, and a host that will not interrupt at the wrong moment. Millisecond steps are undemanding;
microsecond steps need a tuned kernel and careful models, and are where most of the effort goes.

The requirements on the host and on the models are in
[real-time execution]({{< ref "/docs/Developer Guide/Writing a Model/real-time.md" >}}). With that
tuning, steps as low as 5 us synchronised to an FPGA through VILLASnode have been achieved.

## Running a Real-Time Simulation

Before running a simulation, you can run the following commands as root:

```bash
echo "evacuating cores"
tuna isolate -c 9,11,13,15

echo "disabling RT throttling"
echo -1 > /proc/sys/kernel/sched_rt_runtime_us

echo "stopping systemd services"
systemctl stop polkit
systemctl stop containerd
systemctl stop crond
systemctl stop chronyd
```

As a reference, real-time simulation examples are provided in the `dpsim/examples/cxx` and `dpsim-villas/examples/cxx` folder of the DPsim repository.

To benefit from the `PREEMPT_RT` feature and the isolated cores, the simulation has to be started using the `chrt` command to set the scheduling policy and priority, and the `taskset` command to pin the process to the isolated cores.

- [chrt man-page](http://man7.org/linux/man-pages/man1/chrt.1.html)
- [taskset man-page](http://man7.org/linux/man-pages/man1/taskset.1.html)

In the following example, we set the FIFO scheduling policy with the highest priority (99) and pin the execution of the simulation to CPU cores 9,11,13,15 which have been reserved previously (see above).

```bash
# the simple RT_DP_CS_R_1 simulation
taskset -c 9,11,13,15 chrt -f 99 build/dpsim/examples/cxx/RT_DP_CS_R_1

# Cosimulation using VILLASnode, FPGA synchronized time step, and exchanging data via Aurora interface.
# Here we need sudo, to interact with the FPGA. We disable logging (log=false) and set the time step to 50 us (-t 0.00005).
sudo taskset -c 9,11,13,15 chrt -f 99 build/dpsim-villas/examples/cxx/FpgaCosim3PhInfiniteBus -o log=false -t 0.00005 -d 10
```
