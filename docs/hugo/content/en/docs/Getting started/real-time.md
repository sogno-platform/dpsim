---
title: "Real-Time"
linkTitle: "Real-Time"
date: 2025-02-13
---

This page describes recommended techniques to optimize the host operating system for real-time execution of DPsim.

DPsim supports real-time execution, where the simulation time equals the real or wall clock time, on any system.
However, without modifying system parameters the minimum time step reliably achievable will not be in the range of microseconds.
This is due to operating system noise and other processes interfering with the simulation.
With proper tuning, we have achieved real-time time steps as low as 5 us synchronized to a FPGA using VILLASnode.
Synchronizing the time step to an external source is only necessary, when very high time step accuracy, with maximum deviations in the nanoseconds, is required.


## Operating System and Kernel

Using a Linux kernel with the `PREEMPT_RT` feature improves latency when issuing system calls and enables the FIFO scheduler that lets us avoid preemption during the real-time simulation.

Most distributions offer a binary package for a `PREEMPT_RT` enabled kernel. For example on Rocky Linux:
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
```
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
