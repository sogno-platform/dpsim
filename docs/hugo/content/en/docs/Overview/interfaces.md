---
title: "Interfaces"
linkTitle: "Interfaces"
date: 2022-12-13
---

Interfaces can be used to share data between a DPsim simulation and other, external services, for example an MQTT-broker. For the purpose of this guide, all services receiving and transmitting data besides the running DPsim instance are grouped in the term **environment**. Therefore, interfaces
provide a way for a DPsim simulation to exchange data with the environment.
This data is stored in the form of [Attributes]({{< ref "./Attributes/index.md" >}}) and can be **imported** or **exported** in every simulation time step. Exporting an attribute means that on every time step, the current value of that attribute is read and written out to the environment.
Importing an attribute means that on every time step, a new value is read from the environment and the attribute in the simulation is updated to match this value.

## Configuring an Interface
On the configuration level, an interface is an instance of the `Interface` class.
Because the base `Interface` class requires an instance of an `InterfaceWorker` to construct, it is recommended to not use this base class directly,
but instead construct a subclass derived from `Interface` which internally handles the construction of the `InterfaceWorker`. Currently, there exists
only one such subclass in DPsim which is the `InterfaceVillas`.

### Configuring the InterfaceVillas

> This feature requires the compilation of DPsim with the `WITH_VILLAS` feature flag. For use of the `InterfaceVillas` in python, the `dpsimpyvillas` target has to built in addition to the normal `dpsimpy` package.

The `InterfaceVillas` is an interface designed to make use of the various node types and protocols supported by the [VILLASframework](https://github.com/VILLASframework/node). By utilizing the nodes provided by VILLASnode, the `InterfaceVillas` can be configured to import and export attributes from and to a wide range of external services. To create and configure an `InterfaceVillas` instance, create a new shared pointer of type `InterfaceVillas` and supply it with a configuration string in the first constructor argument. This configuration must be a valid JSON object containing the settings for the `VILLASnode` node that should be used for data import and export.
This means that the JSON contains a `type` key describing what node type to use, as well as any additional configuration options required for this node type. The valid configuration keys can be found in the [VILLASnode documentation](https://villas.fein-aachen.org/doc/node-node-types.html).

After the `InterfaceVillas` object is created, the `exportAttribute` and `importAttribute` methods can be used to set up the data exchange between
the DPsim simulation and the configured node. For an explanation of the various parameters, see the code documentation in `InterfaceVillas.h`. The attributes given as the first parameter to these methods are attributes belonging to components in the simulation which should be read or updated by the interface.
As an example, for exporting and importing attributes via the MQTT protocol, the `InterfaceVillas` can be configured as follows:

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
