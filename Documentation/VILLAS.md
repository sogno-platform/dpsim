## Interface to VILLASnode

### Building

Build libvillas-ext from [VILLASnode](https://git.rwth-aachen.de/VILLASframework/VILLASnode):

```
$ git clone git@git.rwth-aachen.de:VILLASframework/VILLASnode.git
$ cd VILLASnode
$ make
# make install
```

Then, build DPsim as normal.

### Using the interface in simulations

To use the interface in simulations, create `ExternalVoltageSource` or
`ExternalCurrentSource` objects and register them with a `VillasInterface`.
Samples are passed between VILLASnode and DPsim in an array of real numbers; you
need to specify which elements from this array to use when registering a source.
Similarly, you can register currents through specific components and voltages
between nodes the be passed back to VILLASnode. See the Doxygen documentation for
`VillasInterface` and `ExternalInterface` or the examples in `Examples/VillasTest.cpp` for more information.

### Running

To actually run simulations, start VILLASnode first and then start DPsim. If
VILLASnode is running as root, DPsim needs to be started as root as well to
connect to it. You may need to adjust `LD_LIBRARY_PATH` to include the directory
where libvillas-ext was installed:

```
$ sudo env LD_LIBRARY_PATH=/usr/local/lib ./DPsolver
```

You can also use the `exec` option of the `shmem` node from VILLASnode; see its
documentation for more details.
