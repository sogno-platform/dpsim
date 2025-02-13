---
title: "Interfaces"
linkTitle: "Interfaces"
date: 2025-02-13
---

Interfaces can be used to exchange simulation signals between a DPsim simulation and other soft- or hardware, for example an MQTT-broker or an FPGA.
Simulation signals in the form of [Attributes]({{< ref "./Attributes/index.md" >}}) can be **imported** or **exported** once per simulation time step.
Interfaces are subclasses of `Interface` and implement the methods `addExport` and `addImport`, which add dependencies to the passed attribute that forward the attribute value from or to the interface.
This way, attributes that are imported are read from the interface before they are used in any DPsim component.
Attributes that are exported are written to the interface after they are set by a DPsim component.

## Interfacing with VILLASnode

> This feature requires the compilation of DPsim with the `WITH_VILLAS` feature flag. For use of the VILLASnode interface in python, the `dpsimpyvillas` target has to built in addition to the normal `dpsimpy` package.

The VILLASnode interface is designed to make use of the various node types and protocols supported by the [VILLASframework](https://github.com/VILLASframework/node).
By utilizing the nodes provided by VILLASnode, it can be configured to import and export attributes using a wide range of protocols.
There are two interface implementations for VILLASnode: `InterfaceVillasQueued` and `InterfaceVillasQueueless`.
`InterfaceVillasQueued` uses a ring buffer to store signal data between DPsim and VILLASnode to allow the protocol used in VILLASnode to operate at a different rate and non-synchronized to the DPsim time step.
`InterfaceVillasQueueless` uses direct communication with a VILLASnode node type implementing a specific protocol without using a buffer, thus enabling significantly lower latency communication.
With `InterfaceVillasQueueless`, the protocol operates at the time step of DPsim, i.e., an attribute update directly triggers a `write()` call to the connected VILLASnode node type.
`InterfaceVillasQueued` should be used when using non- or soft real-time protocols or communication mediums, such as MQTT or connections via the internet.
`InterfaceVillasQueueless` should be used when communicating using reliable, low latency, real-time protocols, e.g., with FPGAs, via dedicated fibre networks, or with local real-time applications.

To create and configure one of the VILLASnode interface instance, create a new shared pointer of type `InterfaceVillasQueued` or `InterfaceVillasQueueless` and supply it with a configuration string in the first constructor argument.
This configuration must be a valid JSON object containing the settings for the VILLASnode node type that should be used for data import and export.
This means that the JSON contains a `type` key describing what node type to use, as well as any additional configuration options required for this node type.
The valid configuration keys can be found in the [VILLASnode documentation](https://villas.fein-aachen.org/doc/node-node-types.html).

> Note for `InterfaceVillasQueueless`:
> The queueless interface expects the first input signal in the VILLASnode configuration to be a sequence number that is incremented every time step.
> If the value of the sequence number is not incremented by one between two consecutive time steps, an overrun is detected.
> Because logging outputs can cause large delays and overruns should not occur spuriously, the interface only reports a warning when a large number of overruns occur.

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
std::shared_ptr<InterfaceVillasQueued> intf = std::make_shared<InterfaceVillasQueued>(mqttConfig);

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

This method will add two new [Tasks]({{< ref "./Scheduling/index.md" >}}) to the simulation. The interface's `PreStep` task is set to modify all attributes that are imported from the environment and is therefore scheduled to execute before any other simulation tasks that depend on these attributes.
The interface's `PostStep` task is set to depend on all attributes that are exported to the environment and is therefore scheduled to execute after any other simulation tasks that might modify these attributes. To prevent the scheduler from just dropping the `PostStep` task since it does not modify any attributes and is therefore not seen as relevant to the simulation, the task is set to modify the `Scheduler::external` attribute.
Note that the execution of these tasks might not necessarily coincide with the point in time at which the values are actually written out to or read from the environment.
This is because the interface internally spawns two new threads for exchanging data with the environment and then uses a **lock-free queue** for communication between these reader and writer threads, and the simulation. Because of this, time-intensive import or export operations will not block
the main simulation thread unless this is explicitly configured in the interface's `importAttribute` and `exportAttribute` methods.

## Synchronizing the Simulation with the Environment
To allow for synchronizing the DPsim simulation with external services, the `Interface` class provides some additional configuration options in the `importAttribute` and `exportAttribute` methods. For imports, setting the `blockOnRead` parameter will completely halt the simulation at the start of
every time step until a new value for this attribute was read from the environment. Additionally, the `syncOnSimulationStart` parameter can be set for every
import to indicate that this attribute is used to synchronize the start of the simulation. When a simulation contains any interfaces importing attributes
which have `syncOnSimulationStart` set, the `Simulation::sync` will be called before the first time step. This method will:
- write out all attributes configured for export to the environment
- block until all attributes with `syncOnSimulationStart` set have been read from the environment at least once
- write out all exported attributes again

Note that this setting operates independently of the `blockOnRead` flag. This means that with both flags set, the simulation will block again after the synchronization at the start of the first time step until another value is received for the attribute in question.
