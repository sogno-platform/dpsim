Real-time Execution
===================

This page describes recommended techniques to optimize the host operating system for real-time execution of DPsim.

In principle, real-time execution is supported on all platforms.
However, we recommend to use an optimized Linux installation.

Operating System and Kernel
---------------------------

For minimum latency several kernel and driver settings can be optimized.

To get started, we recommend the `Redhat Real-time Tuning Guide <https://access.redhat.com/documentation/en-US/Red_Hat_Enterprise_MRG/2/html/Realtime_Tuning_Guide/index.html>`__.

A `PREEMPT_RT patched Linux <https://rt.wiki.kernel.org/index.php/Main_Page>`__ kernel is recommended.
Precompiled kernels for Fedora can be found here: http://ccrma.stanford.edu/planetccrma/software/


1. Tune overall system performance for real-time:
2. Install a ``PREEMPT_RT`` patched Linux kernel.
3. Use the ``tuned`` tool for improving general real-time performance.
    * Please adjust the setting ``isolated_cpucores`` according to your hardware.
    * Enable the ``realtime`` tuned profile by running:

.. code:: bash

           dnf install tuned-profiles-realtime
           echo "realtime" > /etc/tuned/active_profile
           echo "isolated_cpucores=6-7" >> /etc/tuned/realtime-variables.conf
           systemctl enable tuned && systemctl start tuned
           systemctl reboot

Running a real-time simulation
------------------------------

As a reference, Real-time simulation examples are provided here: https://git.rwth-aachen.de/acs/public/simulation/dpsim/tree/development/Examples/Cxx/RealTime

In order to start a real-time simulation, the simulation process must be started in a special way in order to change the execution priority, scheduler and CPU affinity.
For this purpose the ``chrt`` and ``taskset`` commands are used.
In the following example, we pin the execution of the simulation to CPU cores 6-7 which have been reserved previously by tuned's real-time profile (see above).

.. code:: bash

           taskset --all-tasks --cpu-list 6-7 \
           chrt --fifo 99 \
           Examples/Cxx/RT_DP_CS_R_1

More details:

1. `chrt man-page <http://man7.org/linux/man-pages/man1/chrt.1.html>`__
2. `taskset man-page <http://man7.org/linux/man-pages/man1/taskset.1.html>`__

Recommended Hardware
--------------------

This are some proposals for the selection of appropriate server hardware:

-  Server-grade CPU: Intel Xeon

   -  A multi-core system enables true parallel execution of several decoupled systems.

-  Server-grade network cards: Intel PRO/1000

   -  These allow offloading of UDP checksumming to the hardware