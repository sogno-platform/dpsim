---
title: "Tutorials"
linkTitle: "Tutorials"
weight: 2
menu:
  main:
    weight: 22
description: >
  A ladder of worked simulations, each adding one idea to the one before.
---

Worked simulations in order of difficulty, starting from something trivial. Each one adds exactly
one new idea to the one before it, and each is a complete script you can run rather than a fragment
to assemble.

Every script on these pages was run before it was written up, and the numbers quoted are the numbers
it produced. Where a result is surprising, the page says so rather than leaving you to wonder
whether you typed something wrong.

## Two tracks

[Python]({{< ref "Python" >}}) is the track to start with, and the difference is not a matter of
taste. With the Python package installed you edit a script and run it; there is no build step
between a change and a result, and the loop is a few seconds long. That is the right way to learn
what the simulator does.

A C++ track covers the same ground for anyone embedding the solver in an application or writing a
new model, where the Python API is not the interface being used. It costs a compile and link on
every change, so it is the wrong place to learn the concepts and the right place to work once you
know them. It also requires a working build of DPsim itself rather than only the installed package;
see [build]({{< ref "/docs/Developer Guide/Architecture and Conventions/build.md" >}}). The
[C++ examples](https://github.com/sogno-platform/dpsim/tree/master/dpsim/examples/cxx) are the
material it is being built from.

The two tracks describe the same simulator and share the concept pages behind them. Only the calling
code differs, so nothing learned in one track has to be relearned in the other.

## If a tutorial is not what you want

The [User Guide]({{< ref "/docs/User Guide" >}}) covers installation and individual features,
[Concepts]({{< ref "/docs/Concepts" >}}) has the mathematics behind the models, the
[Developer Guide]({{< ref "/docs/Developer Guide" >}}) covers changing DPsim rather than using it,
and [Reference]({{< ref "/docs/Reference" >}}) has the generated API and the model availability
tables.
