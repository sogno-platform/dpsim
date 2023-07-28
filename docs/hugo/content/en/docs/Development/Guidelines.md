---
title: "Guidelines"
linkTitle: "Guidelines"
---

This is a summary of general guidelines for the development of DPsim.

## Logging

There are two ways to log information in DPsim: The `CPS::Logger` (called the **Text Logger**), which can write to `.log` files and to the CLI, as well as the `CPS::DataLogger` (called the **Data Logger**), which can write to `.csv` files.

### Log Levels
The log levels that can be used with the Text Logger are as follows:
- `off` - no Text Logger output at all, recommended to use e.g. when running case studies where exclusively the simulation results should be logged
- `error` - all errors
- `warn` - all warnings
- `info` - logging of information that is basic and that is logged only once before/after the simulation
- `debug` - logging of information for debugging (extended static information, e.g. initialization values, matrix stamps, subcomponents)
- `trace` - logging of information in each simulation step

While the Data Logger does not have any log levels (just a boolean flag to enable or disable the logger),
there are some Data Loggers that are only activated when the object owning the Data Logger is also configured to use a certain log level.
E.g., the `<SimName>_LeftVector.csv` and `<SimName>_RightVector.csv` will only be generated when the Simulation Text Logger is set to `trace` level.

Logging with the Text Logger should always happen using the `SPDLOG_LOGGER_<LEVEL>` macros.

### Examples
The Text Loggers in DPsim are grouped into three categories:
- `Simulation`: All the Text Loggers logging into `Simulation.log`
- `Component`: All the Text Loggers logging into `Components.log`
- `Debug`: Text Loggers used for debugging purposes, each creating their own log files

The following table provides an overview on what output is produced in each category on different log levels:

| Log Level    | `Simulation` Text Logger  | `Component` Text Logger | `Debug` Text Loggers | Data Loggers |
|--------------|---------------------------|-------------------------|----------------------|--------------|
| `off`        | -                         | -                       | -                    |Only log that is explicitly requested by the user, e.g. through `DataLogger::log_attribute` or `Simulation::logStepTimes`|
| `error`      | Errors that will lead to a program crash or invalidate the simulation results | Errors that will lead to a program crash or invalidate the simulation results | Errors that will lead to a program crash or invalidate the simulation results | Only log that is explicitly requested by the user, e.g. through `DataLogger::log_attribute` or `Simulation::logStepTimes`|
| `warn`       | Problems that might lead to invalid results, but will likely not lead to a crash | Problems that might lead to invalid results, but will likely not lead to a crash | Problems that might lead to invalid results, but will likely not lead to a crash |  Only log that is explicitly requested by the user, e.g. through `DataLogger::log_attribute` or `Simulation::logStepTimes`|
| `info`       | e.g. Summary of number of network and virtual nodes, Simulation configuration / domain| e.g. Parameters set through `setParameters`, connections to network nodes | e.g. `CSVReader.log`, whenever a `CSVReader` is used | Only log that is explicitly requested by the user, e.g. through `DataLogger::log_attribute` or `Simulation::logStepTimes`|
| `debug`      | e.g. System matrices, Scheduler log, More details about the used solver | e.g. Component initialization, subcomponents, snubber components | e.g. `CIMReader.log`, whenever a `CIMReader` is used| e.g. iteration count for iterative solvers in `<SimName>.csv`|
| `trace`      | Information that changes on each simulation step | e.g. voltages / currents calculated in `mnaUpdateCurrent`/`mnaUpdateVoltage`|-|e.g. `<SimName>_LeftVector.csv`, `<SimName>_RightVector.csv`, `<SimName>_InitLeftVector.csv`, `<SimName>_InitRightVector.csv`|

## Creating New Releases

Although DPsim currently does not have any conventions on versioning, the periodic creation of
new versions can help to mark significant changes and to analyze new portions of code using tools like SonarCloud.

A new version of DPsim has to be indicated as follows:
- Create a new tag with an increased version number
- Update setup.cfg
- Update CMakeLists.txt
- Update sonar-project.properties

Due to the creation of a new tag, a new PyPi package will be deployed automatically.
To release an updated Docker image, the container workflow needs to be triggered manually.
