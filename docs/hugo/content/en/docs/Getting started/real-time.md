---
title: "Real-Time"
linkTitle: "Real-Time"
date: 2017-01-05
---

This page describes recommended techniques to optimize the host operating system for real-time execution of DPsim.

In principle, real-time execution is supported on all platforms.
However, we recommend to use an optimized Linux installation.

## Operating System and Kernel

For minimum latency several kernel and driver settings can be optimized.

To get started, we recommend the [Redhat Real-time Tuning Guide](https://access.redhat.com/documentation/en-US/Red_Hat_Enterprise_MRG/2/html/Realtime_Tuning_Guide/index.html).

A [PREEMPT_RT patched Linux](https://rt.wiki.kernel.org/index.php/Main_Page) kernel is recommended.
Precompiled kernels for Fedora can be found here: http://ccrma.stanford.edu/planetccrma/software/

Use the *tuned* tool for improving general real-time performance.
Please adjust the setting `isolated_cpucores` according to your hardware and enable the `realtime` profile as follows:

      $ dnf install tuned-profiles-realtime
      $ echo "realtime" > /etc/tuned/active_profile
      $ echo "isolated_cpucores=6-7" >> /etc/tuned/realtime-variables.conf
      $ systemctl enable tuned && systemctl start tuned
      $ systemctl reboot

## Running a real-time simulation

As a reference, real-time simulation examples are provided in the Examples/Cxx folder of the DPsim repository.

In order to run a real-time simulation, the simulation process must be started in a special way in order to change the execution priority, scheduler and CPU affinity.
For this purpose the `chrt` and `taskset` commands are used.
In the following example, we pin the execution of the simulation to CPU cores 6-7 which have been reserved previously by the tuned real-time profile (see above).

      $ taskset --all-tasks --cpu-list 6-7 \
      $ chrt --fifo 99 \
      $ Examples/Cxx/RT_DP_CS_R_1

More details:

- [chrt man-page](http://man7.org/linux/man-pages/man1/chrt.1.html)
- [taskset man-page](http://man7.org/linux/man-pages/man1/taskset.1.html)

## Recommended Hardware

Some proposals for the selection of appropriate server hardware:

-  Server-grade CPU, e.g. Intel Xeon. A multi-core system enables true parallel execution of several decoupled systems
-  Server-grade network cards, e.g. Intel PRO/1000. These allow offloading of UDP checksumming to the hardware
