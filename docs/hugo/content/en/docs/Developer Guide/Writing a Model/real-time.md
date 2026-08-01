---
title: "Real-Time Execution"
linkTitle: "Real-Time"
weight: 8
date: 2026-07-31
description: >
  Tuning the host and writing a model that can hold a deadline.
---

Why you would run in real time, and how to start such a run, is under
[real-time simulation]({{< ref "/docs/User Guide/real-time.md" >}}). This page is what has to be
true of the host and of the models for a deadline to be met.

DPsim runs in real time on any system, but without tuning the smallest reliable step is nowhere near
microseconds, because operating system noise and other processes interfere. With the tuning below,
steps as low as 5 us synchronised to an FPGA through VILLASnode have been achieved.

## Operating System and Kernel

A kernel built with `PREEMPT_RT` improves latency when issuing system calls and enables the FIFO
scheduler that avoids preemption during the run.

This used to mean tracking down an out-of-tree patch set. It no longer does: `PREEMPT_RT` was merged
into the mainline Linux kernel in 6.12, so a recent kernel can be built with it directly and a
growing number of distributions ship or package one. Check what you already have before installing
anything:

```bash
uname -v | grep -q PREEMPT_RT && echo "already real-time" || echo "not a PREEMPT_RT kernel"
```

If you need one, most distributions still offer a binary package. On Rocky Linux:

```bash
sudo dnf --enablerepo=rt install kernel-rt kernel-rt-devel
```

More aggressive tuning can involve isolating a set of cores for exclusive use by the real-time simulation.
This way, the kernel will not schedule any processes on these cores.
Add the kernel parameters `isolcpus` and `nohz_full` using, for example, `grubby`:

```bash
sudo grubby --update-kernel=ALL --args="isolcpus=9,11,13,15 nohz_full=9,11,13,15"
```

Something similar, but less invasive and non-permanent can be achieved using `tuna`:

```bash
sudo tuna isolate -c 9,11,13,15
```

To avoid real-time throttling to cause overruns disable this feature:

```bash
sudo bash -c "echo -1 > /proc/sys/kernel/sched_rt_runtime_us"
```

Note that this is not persistent when rebooting.

## Simulation Model Tuning

Real time capable models cannot issue any system calls during simulation as the context switch to the kernel introduces unacceptable latencies.
This means models cannot allocate memory, use mutexes or other interrupt-driven synchronization primitives, read or write data from files.
You should turn off logging, when time steps in the low milliseconds are desired.
There is a `RealTimeDataLogger` that can be used to output simulation results in these cases.
Note however, that this logger pre-allocated the memory required for all of the logging required during simulations.
Your machine may run out of memory, when the simulation is long or you log too many signals.

You can increase the performance of your simulation by adding the `-flto` and  `-march=native` compiler flags:

```diff
diff --git a/CMakeLists.txt b/CMakeLists.txt
index 8801cbe8d..4a2843269 100644
--- a/CMakeLists.txt
+++ b/CMakeLists.txt
@@ -79,7 +79,7 @@ include(CheckSymbolExists)
 check_symbol_exists(timerfd_create sys/timerfd.h HAVE_TIMERFD)
 check_symbol_exists(getopt_long getopt.h HAVE_GETOPT)
 if(CMAKE_BUILD_TYPE STREQUAL "Release" OR CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo")
-       add_compile_options(-Ofast)
+       add_compile_options(-Ofast -flto -march=native)
 endif()

 # Get version info and buildid from Git
```

## Where the time step comes from

By default the simulation paces itself against the host clock, which is enough for almost everything.
Synchronising the step to an external source instead is only necessary when the accuracy of the step
itself matters at the nanosecond level, which in practice means hardware in the loop against
equipment with its own clock.

That distinction is worth making before reaching for it: locking to an external source constrains the
whole run and is not a general improvement, only the answer to a specific requirement.

## Writing a model that can hold a deadline

A real-time capable model must issue no system calls during simulation: the context switch into the
kernel costs more than the deadline allows.

{{% alert title="Watch out: a single allocation can miss a deadline" color="warning" %}}
No allocating memory, no mutexes or other interrupt-driven synchronisation, and no reading or
writing files inside the step. Any one of these can block for longer than the step, and the failure
appears as an occasional overrun rather than as an error, so it is easy to miss in a short test run.
{{% /alert %}}

Turn logging off when the step is in the low milliseconds. `RealTimeDataLogger` exists for the cases
that still need results: it buffers in memory and writes at the end rather than touching the disk
inside the step.

{{% alert title="Watch out: the real-time logger preallocates everything" color="warning" %}}
`RealTimeDataLogger` allocates the memory for the entire run up front, which is what keeps it off the
critical path. A long run, or too many logged attributes, can therefore exhaust memory before the
simulation starts. See [loggers]({{< ref "loggers.md" >}}).
{{% /alert %}}
