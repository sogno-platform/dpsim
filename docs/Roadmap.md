# Roadmap

## Do you want to know what we are doing currently? 

The best way is to check the [DPSim Project Board](https://github.com/orgs/sogno-platform/projects/1/views/4) , the [Issues List](https://github.com/sogno-platform/dpsim/issues) or the [Pull Requests](https://github.com/sogno-platform/dpsim/pulls) on github.

## Things to be done 

Regarding the Python interface (not exhaustive):

* "constructors" for more component types
* more error checking etc. (module should only throw Python exceptions, never crash)
* port all examples from C++ to Python
* additional logfiles for component-specific data
* use a queue for attribute changes so the simulation doesn't have to be paused
* improve attribute mechanism: mark attributes as readable/writeable, whether
  matrix has to be calculated again; possibly some way to schedule changes
  ahead of time
