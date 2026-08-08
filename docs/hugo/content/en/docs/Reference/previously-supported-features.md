---
title: "Previously Supported Features"
linkTitle: "Previously Supported Features"
weight: 6
description: >
  Features that DPsim used to carry, why they were removed, and where to find the last revision that still had them.
---

Removing something is easier to understand with a pointer to what it looked like before. Each
entry below names the last release that still contained the feature and the change that took it
out, so `git log` and `git show` can recover the code.

None of these were removed while working. They are recorded here so that anyone who finds a
reference to them in an old paper, notebook or branch can see what happened, not so that they can
be revived as they were.

## GPU linear solvers, CUDA and MAGMA

Removed in [#660](https://github.com/sogno-platform/dpsim/pull/660). Last contained in release
[v1.2.1](https://github.com/sogno-platform/dpsim/releases/tag/v1.2.1); to read the code, check out
the parent of the removal commit.

The nodal solver offered three GPU adapters, selectable through `DirectLinearSolverImpl`:

| Removed value | Adapter | Backend |
| --- | --- | --- |
| `CUDADense` | `GpuDenseAdapter` | cuSOLVER dense |
| `CUDASparse` | `GpuSparseAdapter` | cuSOLVER sparse |
| `CUDAMagma` | `GpuMagmaAdapter` | MAGMA |

They did not work. `GpuSparseAdapter` treats an incomplete LU factorisation as if it were
complete, and had never been compiled at all; the dense and MAGMA paths produced wrong results
where they ran. No continuous integration job built any of them, so the breakage went unnoticed
for a long time, which is the substance of
[#164](https://github.com/sogno-platform/dpsim/issues/164) and
[#646](https://github.com/sogno-platform/dpsim/issues/646).

The removal also dropped CUDA, MAGMA and the cricket GPU-virtualisation client from the Rocky
Linux development image, which is why that image no longer carries a CUDA toolchain.

A GPU backend remains a reasonable thing to want. It should start from the current
`DirectLinearSolver` interface and arrive with a job that builds and tests it, rather than from
this code.
