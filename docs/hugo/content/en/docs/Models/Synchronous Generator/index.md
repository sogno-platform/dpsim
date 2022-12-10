---
title: "Synchronous Generator"
linkTitle: "Synchronous Generator"
date: 2020-03-18
---

<script src="https://polyfill.io/v3/polyfill.min.js?features=es6"></script>
<script id="MathJax-script" async src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js"></script>

Two different synchronous machine models are currently available:

- the full order dq0 reference frame model (EMT, DP) [Kundur, Power system stability and control, 1994]
- and the much simpler transient stability model (DP) [Eremia, Handbook of Electrical Power System Dynamics, 2003]

The machine model is interfaced to the nodal analysis network solver through a current source, which only affects the source vector and not the system matrix [Wang2010](https://ieeexplore.ieee.org/document/5411963).

## Basic Equations

The equations of the stator and rotor voltages are

\begin{align}
	\mathbf{v}_{abcs} &= \mathbf{R}_s \mathbf{i}_{abcs} + \frac{d}{dt} \boldsymbol{\lambda}_{abcs} \\
	\mathbf{v}_{dqr} &= \mathbf{R}_r \mathbf{i}_{dqr} + \frac{d}{dt} \boldsymbol{\lambda}_{dqr}
\end{align}

where

\begin{align}
  \mathbf{v}_{abcs} &=
  \begin{pmatrix}
    v_{as} & v_{bs} & v_{cs}
  \end{pmatrix}^T \\
  %
  \mathbf{v}_{dqr} &=
  \begin{pmatrix}
    v_{fd} & v_{kd} & v_{kq1} & v_{kq2}
  \end{pmatrix}^T \\
  %
  \mathbf{i}_{abcs} &=
  \begin{pmatrix}
    i_{as} & i_{bs} & i_{cs}
  \end{pmatrix}^T \\
  %
  \mathbf{i}_{dqr} &=
  \begin{pmatrix}
    i_{fd} & i_{kd} & i_{kq1} & i_{kq2}
  \end{pmatrix}^T \\
  %
  \boldsymbol{\lambda}_{abcs} &=
  \begin{pmatrix}
    \lambda_{as} & \lambda_{bs} & \lambda_{cs}
  \end{pmatrix}^T \\
  %
  \boldsymbol{\lambda}_{dqr} &=
  \begin{pmatrix}
    \lambda_{fd} & \lambda_{kd} & \lambda_{kq1} & \lambda_{kq2}
  \end{pmatrix}^T \\
  %
  \mathbf{R}_s &= diag
  \begin{bmatrix}
    R_s & R_s & R_s
  \end{bmatrix} \\
  %
  \mathbf{R}_r &= diag
  \begin{bmatrix}
    R_{fd} & R_{kd} & R_{kq1} & R_{kq2}
  \end{bmatrix}
\end{align}

The flux linkage equations are defined as

\begin{equation}
	\begin{bmatrix}
		\boldsymbol{\lambda}_{abcs} \\
		\boldsymbol{\lambda}_{dqr}
	\end{bmatrix}
	=
	\begin{bmatrix}
		\mathbf{L}_s & \mathbf{L}_{rs} \\
		{(\mathbf{L}_{rs})}^{T} & \mathbf{L}_r
	\end{bmatrix}
	\begin{bmatrix}
		\mathbf{i}_{abcs} \\
		\mathbf{i}_{dqr}
  \end{bmatrix}
\end{equation}

The inductance matrices are varying with the rotor position $\theta_r$ which varies with time.

The mechanical equations are:

\begin{align}
  \frac{d\theta_r}{dt} &= \omega_r \\
  \frac{d\omega_r}{dt} &= \frac{P}{2J} (T_e-T_m)
\end{align}

$\theta_r$ is the rotor position, $\omega_r$ is the angular electrical speed, $P$ is the number of poles, $J$ is the moment of inertia, $T_m$ and $T_e$ are the mechanical and electrical torque, respectively.
Motor convention is used for all models.

## dq0 Reference Frame 9th Order Model

For stator referred variables, the base quantities for per unit are chosen as follows:

- $v_{s base}$ peak value of rated line-to-neutral voltage in V
- $i_{s base}$ peak value of rated line current in A
- $f_{base}$ rated frequency in Hz

The synchronous generator equations in terms of per unit values in the rotor reference frame become:

\begin{equation}
  \begin{bmatrix}
    \mathbf{v}_{dq0s} \\
    \mathbf{v}_{dqr}
  \end{bmatrix}
  =
  \mathbf{R}_{sr}
  \begin{bmatrix}
    \mathbf{i}_{dq0s} \\
    \mathbf{i}_{dqr}
  \end{bmatrix}
  +
  \frac{d}{dt}
  \begin{bmatrix}
    \boldsymbol{\lambda}_{dq0s} \\
    \boldsymbol{\lambda}_{dqr}
  \end{bmatrix}
  + \omega_r
  \begin{bmatrix}
    \boldsymbol{\lambda}_{qds} \\
    0
  \end{bmatrix}
  \label{eq:rotor_reference}
\end{equation}

where

\begin{align}
  \mathbf{v}_{dq0s} &=
  \begin{pmatrix}
    v_{ds} & v_{qs} & v_{0s}
  \end{pmatrix}^T \nonumber \\
  %
  \mathbf{i}_{dq0s} &=
  \begin{pmatrix}
    i_{ds} & i_{qs} & i_{0s}
  \end{pmatrix}^T \nonumber \\
  %
  \boldsymbol{\lambda}_{dq0s} &=
  \begin{pmatrix}
    \lambda_{ds} & \lambda_{qs} & \lambda_{0s}
  \end{pmatrix}^T \nonumber \\
  %
  \mathbf{R}_{sr} &= diag
  \begin{bmatrix}
    R_s & R_s & R_s & R_{fd} & R_{kd} & R_{kq1} & R_{kq2}
  \end{bmatrix} \nonumber \\
  %
  \boldsymbol{\lambda}_{dqs} &=
  \begin{pmatrix}
    -\lambda_{qs} & \lambda_{ds} & 0
  \end{pmatrix}^T.
\end{align}

The flux linkages are:

\begin{equation}
  \begin{pmatrix}
    \boldsymbol{\lambda}_{dq0s} \\
    \boldsymbol{\lambda}_{dqr}
  \end{pmatrix}
  =
  \begin{bmatrix}
    \mathbf{L}_{dqss} & \mathbf{L}_{dqsr} \\
    \mathbf{L}_{dqrs} & \mathbf{L}_{dqrr}
  \end{bmatrix}
  \begin{pmatrix}
    \mathbf{i}_{dq0s} \\
    \mathbf{i}_{dqr}
  \end{pmatrix}
  \label{eq:flux_linkages}
\end{equation}

where

\begin{align}
  \mathbf{L}_{dqss} &=
  \begin{bmatrix}
    L_{d} & 0 & 0 \\
    0 & L_{q} & 0 \\
    0 & 0 & L_{ls}
  \end{bmatrix} \nonumber \\

  \mathbf{L}_{dqsr} &=
  \begin{bmatrix}
    L_{md} & L_{md} & 0 & 0 \\
    0 & 0 & L_{mq} & L_{mq} \\
    0 & 0 & 0 & 0
  \end{bmatrix} \nonumber \\

  \mathbf{L}_{dqrs} &=
  \begin{bmatrix}
    L_{md} & 0 & 0 \\
    L_{md} & 0 & 0 \\
    0 & L_{mq} & 0 \\
    0 & L_{mq} & 0
  \end{bmatrix} \nonumber \\

  \mathbf{L}_{rr} &=
  \begin{bmatrix}
    L_{fd} & L_{md} & 0 & 0  \\
    L_{md} & L_{kd} & 0 & 0  \\
    0 & 0 & L_{kq1} & L_{mq} \\
    0 & 0 & L_{mq} & L_{kq2}
  \end{bmatrix} \nonumber \\

\end{align}

with

\begin{align}
  L_{d} &= L_{ls} + L_{md} \nonumber \\
  L_{q} &= L_{ls} + L_{mq} \nonumber \\
  L_{fd} &= L_{lfd} + L_{md} \nonumber \\
  L_{kd} &= L_{lkd} + L_{md} \nonumber \\
  L_{kq1} &= L_{lkq1} + L_{mq} \nonumber \\
  L_{kq2} &= L_{lkq2} + L_{mq}.
\end{align}


The mechanical equations in per unit become:

\begin{align}
  T_e &= \lambda_{qs} i_{ds} - \lambda_{ds} i_{qs} \\
  \frac{d \omega_r}{dt} &= \omega_r \\
  \frac{1}{\omega_b} \frac{d \omega_r}{dt} &= \frac{1}{2H} (T_m - T_e).
\end{align}

For the simulation, fluxes are chosen as state variables.
To avoid the calculation of currents from fluxes using the inverse of the inductance matrix, the equation set needs to be solved for the fluxes analytically.
To simplify the calculations, dq axis magnetizing flux linkages are defined [Krause, Analysis of electric machinery and drive systems, 2002]:

\begin{align}
  \lambda_{md} &= L_{md} \left( i_{ds} + i_{fd} + i_{kd} \right) \nonumber \\
  \lambda_{mq} &= L_{mq} \left( i_{qs} + i_{kq1} + i_{kq2} \right).
  \label{eq:magnetizing_flux_linkage}
\end{align}

Using the flux linkages results in a simpler equation set for the fluxes:

\begin{align}
  \lambda_{ds} &= L_{ls} i_{ds} + L_{md} \left( i_{ds} + i_{fd} + i_{kd} \right) \nonumber \\
  \lambda_{qs} &= L_{ls} i_{qs} + L_{mq} \left( i_{qs} + i_{kq1} + i_{kq2} \right) \nonumber \\
  \lambda_{0s} &= L_{ls} i_{0s} \nonumber \\
  \lambda_{fd} &= L_{ls} i_{fd} + L_{md} \left( i_{ds} + i_{fd} + i_{kd} \right) \nonumber \\
  \lambda_{kd} &= L_{ls} i_{kd} + L_{md} \left( i_{ds} + i_{fd} + i_{kd} \right) \nonumber \\
  \lambda_{kq1} &= L_{ls} i_{kq1} + L_{mq} \left( i_{qs} + i_{kq1} + i_{kq2} \right) \nonumber \\
  \lambda_{kq2} &= L_{ls} i_{kq2} + L_{mq} \left( i_{qs} + i_{kq1} + i_{kq2} \right)
  \label{eq:syngen_lambda}
\end{align}

\begin{align}
  \lambda_{ds} &= L_{ls} i_{ds} + \lambda_{md} \nonumber \\
  \lambda_{qs} &= L_{ls} i_{qs} + \lambda_{mq} \nonumber \\
  \lambda_{0s} &= L_{ls} i_{0s} \nonumber \\
  \lambda_{fd} &= L_{lfd} i_{fd} + \lambda_{md} \nonumber \\
  \lambda_{kd} &= L_{lkd} i_{kd} + \lambda_{md} \nonumber \\
  \lambda_{kq1} &= L_{lkq1} i_{kq1} + \lambda_{mq} \nonumber \\
  \lambda_{kq2} &= L_{lkq2} i_{kq2} + \lambda_{mq}
  \label{eq:flux_currents_flux_linkage}
\end{align}


### Dynamic Phasor Model

The fundamental dynamic phasors are similar to the dq0 quantities for symmetrical conditions since both yield DC quantities in a rotating reference frame.
The network abc dynamic phasor quantities can be converted to dq0 dynamic phasors by applying the symmetrical components transformation and a rotation.

The angle $\delta$ is the orientation of the dq0 reference frame relative to the abc frame.

\begin{align}
  \langle i_{ds} \rangle_{0} &= \mathbf{Re} \left\{ \langle i_{p} \rangle_1 \ \mathrm{e}^{-j \delta} \right\} \nonumber \\
  \langle i_{qs} \rangle_{0} &= \mathbf{Im} \left\{ \langle i_{p} \rangle_1 \ \mathrm{e}^{-j \delta} \right\} \nonumber \\
  \langle i_{ds} \rangle_{2} &= \mathbf{Re} \left\{ \langle i_{n} \rangle_{1}^* \ \mathrm{e}^{-j \delta} \right\} \nonumber \\
  \langle i_{qs} \rangle_{2} &= \mathbf{Im} \left\{ \langle i_{n} \rangle_{1}^* \ \mathrm{e}^{-j \delta} \right\} \nonumber \\
  \langle i_{0s} \rangle_{1} &= \mathbf{Re} \left\{ \langle i_{z} \rangle_1 \right\}
  \label{eq:syngen_abc_dq0_conversion}
\end{align}

The winding currents for positive and zero sequence components can be expressed as:

\begin{align}
  \langle i_{ds} \rangle_0  &= \frac{\langle \lambda_{ds} \rangle_0 - \langle \lambda_{md} \rangle_0 }{L_{ls}} \nonumber \\
  \langle i_{qs} \rangle_0 &= \frac{\langle \lambda_{qs} \rangle_0 - \langle \lambda_{mq} \rangle_0}{L_{ls}} \nonumber \\
  \langle i_{0s} \rangle_1 &= \frac{\langle \lambda_{0s} \rangle_1}{L_{ls}} \nonumber \\
  \langle i_{fd} \rangle_0 &= \frac{\langle \lambda_{fd} \rangle_0 - \langle \lambda_{md} \rangle_0}{L_{lfd}} \nonumber \\
  \langle i_{kd} \rangle_0 &= \frac{\langle \lambda_{kd} \rangle_0 - \langle \lambda_{md} \rangle_0}{L_{lkd}} \nonumber \\
  \langle i_{kq1} \rangle_0 &= \frac{\langle \lambda_{kq1} \rangle_0 - \langle \lambda_{mq} \rangle_0}{L_{lkq1}} \nonumber \\
  \langle i_{kq2} \rangle_0 &= \frac{\langle \lambda_{kq2} \rangle_0 - \langle \lambda_{mq} \rangle_0}{L_{lkq2}}.
\end{align}

\begin{align}
  \frac{d}{dt} \langle \lambda_{ds} \rangle_0 &= \langle v_{ds} \rangle_0 + \langle \omega_r \rangle_0 \langle \lambda_{qs} \rangle_0 + \frac{R_s}{L_{ls}} \left( \langle \lambda_{md} \rangle_0 - \langle \lambda_{ds} \rangle_0 \right) \nonumber \\
  \frac{d}{dt} \langle \lambda_{qs} \rangle_0 &= \langle v_{qs} \rangle_0 - \langle \omega_r \rangle_0 \langle \lambda_{ds} \rangle_0 + \frac{R_s}{L_{ls}} \left( \langle \lambda_{mq} \rangle_0 - \langle \lambda_{qs} \rangle_0 \right) \nonumber \\
  \frac{d}{dt} \langle \lambda_{0s} \rangle_1 &= \langle v_{0s} \rangle_1 - \frac{R_s}{L_{ls}} \langle \lambda_{0s} \rangle_1 -j \omega_s \langle \lambda_{0s} \rangle_1 \nonumber \\
  \frac{d}{dt} \langle \lambda_{fd} \rangle_0 &= \langle v_{fd} \rangle_0 + \frac{R_{fd}}{L_{lfd}} \left( \langle \lambda_{md} \rangle_0 - \langle \lambda_{fd} \rangle_0 \right)  \nonumber \\
  \frac{d}{dt} \langle \lambda_{kd} \rangle_0 &= \frac{R_{kd}}{L_{lkd}} \left( \langle \lambda_{md} \rangle_0 - \langle \lambda_{kd} \rangle_0 \right)  \nonumber \\
  \frac{d}{dt} \langle \lambda_{kq1} \rangle_0 &= \frac{R_{kq1}}{L_{lkq1}} \left( \langle \lambda_{mq} \rangle_0 - \langle \lambda_{kq1} \rangle_0 \right)  \nonumber \\
  \frac{d}{dt} \langle \lambda_{kq2} \rangle_0 &= \frac{R_{kq2}}{L_{lkq2}} \left( \langle \lambda_{mq} \rangle_0 - \langle \lambda_{kq2} \rangle_0 \right).
\end{align}

In the dynamic phasor case, the equation for $\frac{d}{dt} \langle \lambda_{0s} \rangle_1$ has a frequency shift.
To complete the state model, the magnetizing flux linkages are expressed as:

\begin{align}
  \langle \lambda_{md} \rangle_0 &=  L_{ad} \left( \frac{\langle \lambda_{ds} \rangle_0}{L_{ls}} + \frac{\langle \lambda_{fd} \rangle_0}{L_{lfd}} + \frac{\langle \lambda_{kd} \rangle_0}{L_{lkd}} \right) \nonumber \\
  \langle \lambda_{mq} \rangle_0 &=  L_{aq} \left( \frac{\langle \lambda_{qs} \rangle_0}{L_{ls}} + \frac{\langle \lambda_{kq1} \rangle_0}{L_{lkq1}} + \frac{\langle \lambda_{kq2} \rangle_0}{L_{lkq2}} \right)
  \label{eq:winding_flux_linkages}
\end{align}

where

\begin{align}
  L_{ad} &=  \left( \frac{1}{L_{md}} + \frac{1}{L_{ls}} + \frac{1}{L_{lfd}} + \frac{1}{L_{lkd}} \right) \nonumber \\
  L_{aq} &=  \left( \frac{1}{L_{mq}} + \frac{1}{L_{ls}} + \frac{1}{L_{lkq1}} + \frac{1}{L_{lkq2}} \right).
\end{align}

The mechanical equations in dynamic phasors are:

\begin{align}
  T_e &= \langle \lambda_{qs} \rangle_0 \langle i_{ds} \rangle_0 - \langle \lambda_{ds} \rangle_0 \langle i_{qs} \rangle_0 \\
  \frac{1}{\omega_s} \frac{d \delta_r}{dt} &= \omega_r - 1 \\
  \frac{d \omega_r}{dt} &= \frac{1}{2H} (T_m - T_e).
\end{align}

## Transient Stability Model
