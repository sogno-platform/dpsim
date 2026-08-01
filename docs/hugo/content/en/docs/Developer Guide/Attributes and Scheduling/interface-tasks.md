---
title: "Interfaces"
linkTitle: "Interfaces"
weight: 5
date: 2026-07-31
description: >
  Configuring a VILLASnode interface, and the tasks and threads behind it.
---

Why you would exchange data at all, and what it costs, is under
[co-simulation]({{< ref "/docs/User Guide/co-simulation.md" >}}). This page is how it is configured
and what it does to the task graph.

## Choosing and configuring an interface

{{% alert title="Requires a build with VILLASnode" color="info" %}}
This feature requires DPsim compiled with the `WITH_VILLAS` flag. Using the interface from Python
additionally needs the `dpsimpyvillas` target built alongside the normal `dpsimpy` package.
{{% /alert %}}

The VILLASnode interface is designed to make use of the various node types and protocols supported by the [VILLASframework](https://github.com/VILLASframework/node).
By utilizing the nodes provided by VILLASnode, it can be configured to import and export attributes using a wide range of protocols.
There are two interface implementations for VILLASnode: `InterfaceVillas`, which is queued, and
`InterfaceVillasQueueless`.

{{% alert title="Watch out: only the queued interface is available from Python" color="warning" %}}
`dpsimpyvillas` exposes `InterfaceVillas` and nothing else, so `InterfaceVillasQueueless` can only be
used from C++. A Python script needing the unbuffered path has no way to reach it today.
{{% /alert %}}
`InterfaceVillas` uses a ring buffer to store signal data between DPsim and VILLASnode to allow the protocol used in VILLASnode to operate at a different rate and non-synchronized to the DPsim time step.
`InterfaceVillasQueueless` uses direct communication with a VILLASnode node type implementing a specific protocol without using a buffer, thus enabling significantly lower latency communication.
With `InterfaceVillasQueueless`, the protocol operates at the time step of DPsim, i.e., an attribute update directly triggers a `write()` call to the connected VILLASnode node type.
`InterfaceVillas` should be used when using non- or soft real-time protocols or communication mediums, such as MQTT or connections via the internet.
`InterfaceVillasQueueless` should be used when communicating using reliable, low latency, real-time protocols, e.g., with FPGAs, via dedicated fibre networks, or with local real-time applications.

To create and configure one of the VILLASnode interface instance, create a new shared pointer of type `InterfaceVillas` or `InterfaceVillasQueueless` and supply it with a configuration string in the first constructor argument.
This configuration must be a valid JSON object containing the settings for the VILLASnode node type that should be used for data import and export.
This means that the JSON contains a `type` key describing what node type to use, as well as any additional configuration options required for this node type.
The valid configuration keys can be found in the [VILLASnode documentation](https://villas.fein-aachen.org/doc/node-node-types.html).

{{% alert title="Watch out: the queueless interface reserves the first signal" color="warning" %}}
The queueless interface expects the first input signal in the VILLASnode configuration to be a
sequence number incremented every time step. If it does not increase by one between consecutive
steps, an overrun is detected. Because logging can cause large delays and overruns should not be
reported spuriously, the interface only warns once a large number of them occur.
{{% /alert %}}

After the object is created, the `exportAttribute` and `importAttribute` methods can be used to set up the data exchange between the DPsim simulation and the configured node.
The attributes given as the first parameter to these methods are attributes belonging to components in the simulation which should be read or updated by the interface.
As an example, for exporting and importing attributes via the MQTT protocol, the VILLASnode interfaces can be configured as follows:

Using C++:

```cpp
// JSON configuration adhering to the VILLASnode documentation
std::string mqttConfig = R"STRING({
    "type": "mqtt",
    "format": "json",
    "host": "mqtt",
    "in": {
        "subscribe": "/mqtt-dpsim"
    },
    "out": {
        "publish": "/dpsim-mqtt"
    }
})STRING";

// Creating a new InterfaceVillas object
std::shared_ptr<InterfaceVillas> intf = std::make_shared<InterfaceVillas>(mqttConfig);

// Configuring the InterfaceVillas to import and export attributes
intf->importAttribute(evs->mVoltageRef, 0, true, true);
intf->exportAttribute(r12->mIntfCurrent->deriveCoeff<Complex>(0, 0), 1, true, "v_load");
```

Using Python:

```python
# JSON configuration adhering to the VILLASnode documentation
mqtt_config = '''{
        "type": "mqtt",
        "format": "json",
        "host": "mqtt",
        "in": {
            "subscribe": "/mqtt-dpsim"
        },
        "out": {
            "publish": "/dpsim-mqtt"
        }
}'''

# Creating a new InterfaceVillas object
intf = dpsimpyvillas.InterfaceVillas(name='dpsim-mqtt', config=mqtt_config)

# Configuring the InterfaceVillas to import and export attributes
intf.import_attribute(evs.attr('V_ref'), 0, True)
intf.export_attribute(r12.attr('i_intf').derive_coeff(0, 0), 0)
```

## Adding an Interface to the Simulation

After a new interface has been created and configured, it can be added to a simulation using the `Simulation::addInterface` method:

```cpp
// Create and configure simulation
RealTimeSimulation sim(simName);
sim.setSystem(sys);
sim.setTimeStep(timeStep);
sim.setFinalTime(10.0);

// Create and configure interface
auto intf = //...

// Add interface to simulation
sim.addInterface(intf);
```

Adding an interface also adds two tasks to the simulation, one before the step and one after, so an
imported value is in place before anything reads it and an exported one is sent after everything
that could change it. The transfer itself happens on separate threads, so a slow far side does not
hold up the solver. How that is arranged, and why it matters, is under
[how an interface is scheduled]({{< ref "/docs/Developer Guide/Attributes and Scheduling/interface-tasks.md" >}}).

## Synchronizing the Simulation with the Environment

To allow for synchronizing the DPsim simulation with external services, the `Interface` class provides some additional configuration options in the `importAttribute` and `exportAttribute` methods. For imports, setting the `blockOnRead` parameter will completely halt the simulation at the start of
every time step until a new value for this attribute was read from the environment. Additionally, the `syncOnSimulationStart` parameter can be set for every
import to indicate that this attribute is used to synchronize the start of the simulation. When a simulation contains any interfaces importing attributes
which have `syncOnSimulationStart` set, the `Simulation::sync` will be called before the first time step. This method will:

- write out all attributes configured for export to the environment
- block until all attributes with `syncOnSimulationStart` set have been read from the environment at least once
- write out all exported attributes again

Note that this setting operates independently of the `blockOnRead` flag. This means that with both flags set, the simulation will block again after the synchronization at the start of the first time step until another value is received for the attribute in question.

## The two tasks

Adding an interface adds a `PreStep` and a `PostStep` task.

`PreStep` is declared to modify every attribute imported from the environment, so the scheduler
places it before any task that depends on those attributes. An imported value is therefore in place
before anything reads it.

`PostStep` is declared to depend on every attribute exported to the environment, so it runs after
anything that might modify them.

## Why PostStep declares a modified attribute

`PostStep` modifies nothing in the simulation: it only sends values outward. The scheduler prunes
tasks whose outputs nothing needs, so a task that modifies nothing is dropped, and the export would
silently never happen.

To prevent that, `PostStep` is declared to modify `Scheduler::external`. That attribute exists to
make a task reachable when its real effect is outside the simulation.

{{% alert title="Watch out: a task that modifies nothing is pruned" color="warning" %}}
This is the general rule, not a quirk of interfaces. The scheduler keeps a task only if something
needs what it produces, so any task whose effect leaves the simulation must declare a modified
attribute or it will be dropped without warning. The same mechanism explains why logging an
attribute can change which tasks run; see
[adding tasks to a component]({{< ref "adding-tasks.md" >}}).
{{% /alert %}}

## Task execution is not the moment of transfer

When these tasks execute is not when the data actually crosses the boundary. The interface spawns a
reader thread and a writer thread and communicates with them over a lock-free queue.

The consequence is the useful part: a slow import or export does not block the solver. The simulation
hands a value to the queue and continues. That is what makes an interface to a slow or unreliable
far side usable at all, and it is also why a value read this step may have been produced some time
ago.

Blocking is opt-in through `blockOnRead` and `syncOnSimulationStart` on the import, described on the
co-simulation page. Those are the only ways the exchange paces the simulation.
