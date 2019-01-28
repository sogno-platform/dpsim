# Roadmap

Things to be done regarding the Python interface (not exhaustive):

* "constructors" for more component types
* more error checking etc. (module should only throw Python exceptions, never crash)
* port all examples from C++ to Python
* additional logfiles for component-specific data
* use a queue for attribute changes so the simulation doesn't have to be paused
* improve attribute mechanism: mark attributes as readable/writeable, whether
  matrix has to be calculated again; possibly some way to schedule changes
  ahead of time
