---
title: "How to Cite"
linkTitle: "How to Cite"
weight: 7
menu:
  main:
    weight: 27
description: >
  Cite DPsim if you use it in published work.
---

If you use DPsim in your research, please cite the software paper below. If your work depends on
a specific capability, cite the corresponding paper from
[further publications](#further-publications) as well.

## Software paper

M. Mirz, S. Vogel, G. Reinke and A. Monti, "DPsim: A dynamic phasor real-time simulator for
power systems", *SoftwareX*, vol. 10, 100253, 2019.
<https://doi.org/10.1016/j.softx.2019.100253>

```bibtex
@article{mirz2019dpsim,
  title   = {DPsim: A dynamic phasor real-time simulator for power systems},
  author  = {Mirz, Markus and Vogel, Steffen and Reinke, Georg and Monti, Antonello},
  journal = {SoftwareX},
  volume  = {10},
  pages   = {100253},
  year    = {2019},
  issn    = {2352-7110},
  doi     = {10.1016/j.softx.2019.100253},
  url     = {https://www.sciencedirect.com/science/article/pii/S2352711018302760}
}
```

## Citing a specific version

To make a result reproducible, cite the version you ran alongside the paper:

<!-- x-release-please-start-version -->

```bibtex
@software{dpsim,
  title  = {DPsim},
  author = {{The DPsim Authors}},
  url    = {https://github.com/sogno-platform/dpsim},
  note   = {Version 1.5.0}
}
```

<!-- x-release-please-end -->

The repository also carries a `CITATION.cff` file, so GitHub offers a ready-made citation
through the "Cite this repository" link on the project page.

## Further publications

Cite these when your work builds on the specific method they describe.

Shifted frequency analysis and reduced-order machine models:

- J. Dinkelbach, M. Moraga and A. Monti, "Reduced-Order Synchronous Generator Modelling for
  Real-Time Simulation using Shifted Frequency Analysis", *OSMSES*, 2023.
  <https://ieeexplore.ieee.org/document/10089718>
- G. Nakti, J. Dinkelbach, M. Mirz and A. Monti, "Comparative Assessment of Shifted Frequency
  Modeling in Transient Stability Analysis using the Open Source Simulator DPsim", *OSMSES*, 2022.
  <https://ieeexplore.ieee.org/document/9769135>
- J. Dinkelbach, G. Nakti, M. Mirz and A. Monti, "Simulation of Low Inertia Power Systems Based
  on Shifted Frequency Analysis", *Energies*, vol. 14, no. 7, 1860, 2021.
  <https://www.mdpi.com/1996-1073/14/7/1860>

Power electronics modelling and parallelisation:

- M. Mirz, J. Dinkelbach and A. Monti, "DPsim: Advancements in Power Electronics Modelling Using
  Shifted Frequency Analysis and in Real-Time Simulation Capability by Parallelization",
  *Energies*, vol. 13, no. 15, 3879, 2020. <https://www.mdpi.com/1996-1073/13/15/3879>

Solver performance:

- J. Dinkelbach, L. Schumacher, L. Razik, A. Benigni and A. Monti, "Factorisation Path Based
  Refactorisation for High-Performance LU Decomposition in Real-Time Power System Simulation",
  *Energies*, vol. 14, no. 23, 7989, 2021. <https://www.mdpi.com/1996-1073/14/23/7989>

Grid data and CIM:

- J. Dinkelbach, L. Razik, M. Mirz, A. Benigni and A. Monti, "Template-based generation of
  programming language specific code for smart grid modelling compliant with CIM and CGMES",
  *The Journal of Engineering*, 2022.
  <https://onlinelibrary.wiley.com/doi/abs/10.1049/tje2.12208>

Real-time and co-simulation:

- S. Vogel, M. Mirz, L. Razik and A. Monti, "An Open Solution for Next-generation Real-time Power
  System Simulation", *IEEE EI2*, 2017. <https://ieeexplore.ieee.org/document/8245739>
- M. Mirz, A. Estebsari, F. Arrigo, E. Bompard and A. Monti, "Dynamic phasors to enable
  distributed real-time simulation", *ICCEP*, 2017.
  <https://ieeexplore.ieee.org/document/8004805>
