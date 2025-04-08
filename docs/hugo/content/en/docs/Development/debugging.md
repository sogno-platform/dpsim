---
title: "Debugging"
linkTitle: "Debugging"
---

# Mixed Python C++ Debugging

## Prerequisites

Your vscode launch.json should have two configurations, one to launch the python process and one to attach gdb:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Python: Current File",
            "type": "python",
            "request": "launch",
            "program": "${file}",
            "console": "integratedTerminal",
            "stopOnEntry": true,
            "env": {"PYTHONPATH": "${workspaceFolder}/build${pathSeparator}${env:PYTHONPATH}"}
        },
        {
            "name": "(gdb) Attach",
            "type": "cppdbg",
            "request": "attach",
            "program": "/usr/bin/python",
            "processId": "${command:pickProcess}",
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ]
        }
    ]
}
```

The python debugger will stop on entry ("stopOnEntry": true).
Make sure to adapt your PYTHONPATH variable if necessary.

The C++ code has to be build in debug mode

```shell
cmake .. -DCMAKE_BUILD_TYPE=Debug
```

## Attaching C++ Debugger

- open the python example to be debugged
- go to the debug menu and select / run the "Python: Current File" configuration
- the python debugger should stop at entry
- set C++ breakpoints
- go to the debug menu and run the "(gdb) Attach" configuration
- select a process… choose the python process with the “—adapter-access-token” part
- you can view the whole description when you hover over the process with the mouse
- press play to continue Python debugging… the c++ debugger will stop at the next breakpoint

You can automate this by using the vscode extension “Python C++ Debugger” and by adding this configuration to the launch.json above:

```json
{
    "name": "Python C++ Debugger",
    "type": "pythoncpp",
    "request": "launch",
    "pythonConfig": "custom",
    "pythonLaunchName": "Python: Current File",
    "cppConfig": "default (gdb) Attach"
}
```

This will automatically run both debuggers and select the current process.

It can take a while before the debugger hits the C++ breakpoints.

# C++ Debugging

Use the following launch.json for vscode and set the program path:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "(gdb) Launch",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/dpsim/build/Examples/Cxx/example",
            "args": [],
            "stopAtEntry": true,
            "cwd": "${workspaceFolder}",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                }
            ]
        }
    ]
}
```
