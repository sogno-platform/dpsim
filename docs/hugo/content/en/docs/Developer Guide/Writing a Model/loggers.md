---
title: "Logger Implementation"
linkTitle: "Loggers"
date: 2026-07-31
description: >
  The classes called a logger, and the seam for adding another.
weight: 13
---

Using the loggers is covered under [logging results]({{< ref "/docs/User Guide/logging.md" >}}).
This page covers the classes.

## Distinct things share the name

| Class | Purpose |
| --- | --- |
| `DPsim::DataLogger` | Numerical results to CSV, one row per step |
| `DPsim::RealTimeDataLogger` | The same results, buffered in memory for real-time runs |
| `DPsim::DataLoggerInterface` | The seam both implement, and the one to implement for a new sink |
| `CPS::Logger` | The diagnostic text log, controlled by `LogLevel` |

Only the data loggers have anything to do with results. `CPS::Logger` is a different subsystem that
happens to share the word, and conflating the two is the most common confusion here. A component
constructed with `Logger::Level::debug` writes prose about its own initialization and contributes
nothing to any CSV.

## `DataLogger`

Holds a map from column name to attribute and appends a row per step. `log(Real time, Int
timeStepCount)` returns early when the logger is disabled or when
`timeStepCount % mDownsampling != 0`, so down-sampling is a modulo on the step counter rather than a
time comparison, and it is exact regardless of step size.

The header is written lazily on the first row, by testing `mLogFile.tellp() == 0`. That means the
column set is fixed by whatever was registered before the first log call; registering an attribute
afterwards would produce rows that no longer match the header.

Values are written with `std::scientific` in fixed-width columns, which is what makes the output
readable as a table and also what makes it larger than a minimal CSV would be.

The constructor takes `(name, enabled, downsampling)`. The Python binding exposes only the name, so
`enabled` and `downsampling` are unreachable from Python. A binding that took all three would make
down-sampling available to notebook users, who currently have only the time step.

## `RealTimeDataLogger`

Exists because writing to disk inside a real-time step is not acceptable: the file system offers no
bound on how long a write takes, and one slow write overruns the step. It preallocates
`mAttributeData` from either the final time and step size or an explicit row count, fills it during
the run, and writes at the end.

The preallocation is the point, and it is also the constraint: the row count must be known before
the run, so a real-time simulation of indefinite length needs a different arrangement.

## `DataLoggerInterface`

The abstract seam. Implement it to send results somewhere other than a file, which is what the
co-simulation interfaces do rather than logging and re-reading. `Simulation::addLogger` accepts
anything implementing it.

## Scheduling

A logger contributes a task like any other component, so the scheduler places it by its declared
attribute dependencies. A logged attribute therefore keeps alive the task that produces it, which
has a consequence worth knowing: logging an attribute can change which tasks the scheduler considers
reachable. A model whose results change when a logger is added is exhibiting a missing dependency
declaration elsewhere, not a logging bug. See
[adding tasks to a component]({{< ref "adding-tasks.md" >}}).

## Source

`dpsim/src/DataLogger.cpp`, `dpsim/src/RealTimeDataLogger.cpp`,
`dpsim/include/dpsim/DataLoggerInterface.h`, and `dpsim-models/include/dpsim-models/Logger.h` for
the unrelated diagnostic logger.
