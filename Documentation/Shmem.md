## Shared memory interface

### Building

Build libvillas-ext from [VILLASnode](https://git.rwth-aachen.de/VILLASframework/VILLASnode):

```
$ git clone git@git.rwth-aachen.de:VILLASframework/VILLASnode.git
$ cd VILLASnode
$ make
# make install-libvillas-ext
```

There are also RPMs available for Fedora:

```
$ wget https://villas.fein-aachen.org/packages/villas.repo
# mv villas.repo /etc/yum.repos.d
# dnf -y install villas-node-devel
```

Then, build DPsim as normal.

### Using the interface in simulations

To use the interface in simulations, create `ExternalVoltageSource` or
`ExternalCurrentSource` objects and register them with a `ShmemInterface`.
Samples are passed between DPsim and the other process in an array of real numbers; you
need to specify which elements from this array to use when registering a source.
Similarly, you can register currents through specific components and voltages
between nodes the be passed back to the other process. See the Doxygen documentation for
`ShmemInterface` and `ExternalInterface` or the examples in `Examples/ShmemTest.cpp` for more information.

### Running with VILLASnode

To actually run simulations, start VILLASnode and DPsim, configuring them to use
the other process' output queue as the input queue. If VILLASnode is running as root,
DPsim needs to be started as root as well to connect to it. You may need to
adjust `LD_LIBRARY_PATH` to include the directory where libvillas-ext was
installed if you didn't install it to /usr/lib:

```
$ sudo env LD_LIBRARY_PATH=/usr/local/lib ./DPsolver
```

You can also use the `exec` option of the `shmem` node from VILLASnode; see its
documentation for more details.

### Direct communication between DPsim instances

The shmem interfaces can also be used similarily for direct communication between
2 instances of DPsim. For this, just start two instances of DPsim, each using
a ShmemInterface with them directly using the other's output queue as the input
queue. See `shmemDistributedExample` from `Examples/ShmemTest.cpp` for an example.
