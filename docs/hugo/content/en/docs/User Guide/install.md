---
title: "Install"
aliases: ["/docs/getting-started/install/"]
linkTitle: "Install"
date: 2026-07-31
description: >
  Installing DPsim and its Python package.
weight: 1
---

DPsim is a Python module and C++ library for dynamic power system simulation.

The quickest route to a result is the Python module: install it, then work through the
[tutorials]({{< ref "/docs/Tutorials" >}}), which build up one idea at a time from a source and a
resistor. If you would rather read a finished study than build one, the
[example notebooks]({{< ref "examples.md" >}}) run complete scenarios and plot them.

Building from source is only needed for a platform without a published wheel, or to work on DPsim
itself; see [build]({{< ref "/docs/Developer Guide/Architecture and Conventions/build.md" >}}).

## Try it without installing

The example notebooks run in the browser with no local installation:

[![Binder](https://2i2c.mybinder.org/badge_logo.svg)](https://2i2c.mybinder.org/v2/gh/sogno-platform/dpsim/HEAD?urlpath=%2Fdoc%2Ftree%2Fexamples%2FIndex.ipynb)

## Python package

DPsim is published on PyPI and installs like any other Python package:

```shell
python3 -m venv venv
source venv/bin/activate
pip install dpsim
```

{{% alert title="Requires Linux for the published wheels" color="info" %}}
Two limitations are worth knowing before you start.
Only Linux wheels are currently published, for CPython 3.9 through 3.13, so on Windows and macOS
you have to build from source for now.
The package also contains only the simulation core; the example notebooks additionally need
plotting and data handling packages, which are listed in the import section of each notebook.
{{% /alert %}}

If you prefer conda, the equivalent is:

```shell
conda create -n dpsim python=3.13
conda activate dpsim
pip install dpsim
```

## Docker

You need [Docker](https://docs.docker.com/install/) installed first. The prepared image on
Docker Hub bundles the module together with a JupyterLab session:

```shell
docker run -p 8888:8888 sogno/dpsim
```

Then open <http://localhost:8888/lab?token=3adaa57df44cea75e60c0169e1b2a98ae8f7de130481b5bc>.

Note that the image pins that access token in its startup command, so it is the same for
everyone who runs the image. Publish the port on localhost only, as above, and do not expose
it to an untrusted network.

To build the image yourself rather than pulling it, see
[build]({{< ref "/docs/Developer Guide/Architecture and Conventions/build.md" >}}).
