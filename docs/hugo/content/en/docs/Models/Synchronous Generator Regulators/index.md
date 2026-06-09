---
title: "Synchronous Generator Regulators"
linkTitle: "Synchronous Generator Regulators"
date: 2023-01-23
author: Martin Moraga <martin.moraga@eonerc.rwth-aachen.de>
---

In DPSim, synchronous generator control systems are solved separately from the electric network. The outputs of the electric network (active and reactive power, node voltages, branch currents and rotor speed of synchronous generators) at time $k- \Delta t$ are used as the input of the controllers to calculate their states at time $k$. Because of the relatively slow response of the controllers, the error in the network solution due to the time delay $\Delta t$ introduced by this approach is negligible.

# Exciter

DC1 type model is the standard IEEE type DC1 exciter, whereas the other model is a simplified version of the IEEE DC1 type model. The inputs of the exciters are the magnitude of the terminal voltage of the generator connected to the exciter $v_h$ and the voltage reference $v_{ref}$, which is defined as a variable since other devices such as over-excitation limiters or power system stabilizers (PSS) modify such reference with additional signals. At the moment, no over-excitation limiters have been implemented in DPSim so that the reference voltage is given by:
$$
    v_{ref}(t) = v_{ref,0} + v_{pss}(t)
$$
where $v_{ref,0}$ is initialized after the power flow computations and $v_{pss}(t)$ is the output of the (optional) PSS connected to the exciter. The output of the exciter systems is the induced emf by the field current at $t=k + \Delta t$: $v_{ef}(k + \Delta t)$ (sometimes the alternative notation $e_{fd}(k + \Delta t)$ is used).

## IEEE Type DC1 exciter model

<center>
<figure margin=30%>
    <img src="./images/DC1C_exciter.png" width=65% alt="DC1_exciter">
    <figcaption>Fig. 1: Control diagram of the IEEE Type DC1 exciter </br>
                Adapted from: Milano, Frequency Variations in Power Systems
    </figcaption>
</figure>
</center>
This model is used to represent field controlled dc commutator exciters with continuously acting voltage regulators (especially the direct-acting rheostatic, rotating
amplifier, and magnetic amplifier types). The control diagram of this exciter is depicted in Fig. 1 and it is described by the following set of differential equations:
</br></br>

$$
    T_{R} \frac{d}{dt} v_{R}(t) = v_{h}(t) - v_{R}(t)
$$

$$
    T_{b} \frac{d}{dt} v_{b}(t) = v_{ref} - v_{R}(t) - v_{f}(t) - v_{b}(t),
$$
$$
    T_{a} \frac{d}{dt} v_{a}(t) = K_{a} v_{in}(t) - v_{a}(t),
$$
$$
    T_{f} \frac{d}{dt} v_{f}(t) - K_{f} \frac{d}{dt} v_{ef}(t) = -v_{f}(t),
$$
$$
    T_{ef} \frac{d}{dt} v_{ef}(t) = v_{a}(t) - (K_{ef} + sat(t)) v_{ef}(t),
$$
where $v_h$ is the module of the machine's terminal voltage, and $v_{in}$ is the amplifier input signal, which for the IEEE Type DC1 is given by:
$$
    v_{in}(t) = T_{c} \frac{d}{dt} v_b(t) + v_b(t).
$$

The ceiling function approximates the saturation of the excitation winding:
$$
    sat(t) = A_{ef} e^{(B_{ef} | v_{ef}(t) | )}
$$

The set of differential equations are discretized using forward euler in order to solve it numerically, which leads to the following set of algebraic equations:
$$
    v_R(k + \Delta t) = v_R(k) + \frac{\Delta t}{T_R}  ( v_h(k) - v_R(k) ),
$$
$$
    v_b(k + \Delta t) = v_b(k)(1 - \frac{\Delta t}{T_b}) + \frac{\Delta t}{T_b}  ( v_{ref}(k) - v_R(k) - v_f(k)),
$$
$$
    v_{in}(k + \Delta t) = \Delta t \cdot \frac{T_c}{T_b} (v_{ref}(k) - v_R(k) - v_{f}(k) - v_b(k)) + v_b(k+1),
$$
$$
    v_a(k + \Delta t) = v_a(k) + \frac{\Delta t}{T_a} ( v_{in}(k) K_a - v_a(k) ),
$$
$$
    v_f(k + \Delta t) = (1 - \frac{\Delta t}{T_f}) v_f(k) + \frac{\Delta t K_f}{T_f T_{ef}} ( v_{a}(k) - (K_{ef} + sat(k)) v_{ef}(k) ),
$$
$$
    v_{ef}(k + \Delta t) = v_{ef}(k) + \frac{\Delta t}{T_{ef}} ( v_{a}(k) - (sat(k) + K_{ef}) v_{ef}(k)),
$$
$$
    sat(k) = A_{ef} e^{(B_{ef} | v_{ef}(k) | )}
$$

Since the values of all variables for $t=k$ are known, $v_{ef}(k+1)$ can be easily calculated using the discretised equations, which is carried out in the `preStep` function of the generator connected to each exciter.

The initial values of all variables, which are used in the first simulation step, are calculated assuming that the simulation starts in the steady. This is equivalent to assume that all derivative are equal to zero, which leads to:
$$
    v_R(k=0) = v_h(k=0),
$$
$$
    v_f(k=0) = 0
$$
$$
    v_a(k=0) = K_{ef} v_{ef}(k=0) + A_{ef} e^{B_{ef} |v_{ef} (k=0)|} v_{ef}(k=0),
$$
$$
    v_{in}(k=0) = \frac{v_a(k=0)}{K_a},
$$
$$
    v_b(k=0) = v_{in}(k=0),
$$
$$
    v_{ref}(t=0) = v_{in}(t=0) + v_b(t=0),
$$
where $v_h(k=0)$, $v_{ef}(k=0)$ are calculated after the power flow analysis and after the initialization of synchronous machines (see section initialization of SG).

## Simplified IEEE Type DC1 exciter model (DC1Simp)

<center>
<figure>
    <img src="./images/DC1CSimp_exciter.png" width=65% alt="DC1A_exciter">
    <figcaption></br>Fig. 2: Control diagram of the IEEE Type DC1 exciter </br>
            Adapted from: Milano, Power System Modelling and Scripting
    </figcaption>
</figure>
</center>
</br>

Because the time constants $T_b$ and $T_c$ of the IEEE Type DC1 exciter model are frequently small enough to be neglected, in DPSim a simplified model of this exciter which neglect these time constants is also implemented. The control diagram of this exciter is depicted in Fig. 2 and it is described by the following set of differential equations:
$$
    T_R \frac{d}{dt} v_R(t) = v_h(t) - v_R(t)
$$
$$
    T_a \frac{d}{dt} v_a(t) = - v_a(t) + K_a v_{in}(t)
$$
$$
    T_f \frac{d}{dt} v_f(t) - K_f \frac{d}{dt} v_{ef}(t) = -v_f(t),
$$
$$
    T_e \frac{d}{dt} v_{ef}(t) = v_a(t) - v_{ef}(t) (sat(t) + K_{ef})
$$
where $v_h$​ is the module of the machine's terminal voltage, and $v_{in}$​ is the amplifier input signal, which is given by:
$$
    v_{in}(t) = v_{ref} (t) - v_R(t) - v_f(t)
$$
The set of differential equations are discretized using forward euler in order to solve it numerically, which leads to the following set of algebraic equations:
$$
    v_R(k + \Delta t) = v_R(k) + \frac{\Delta t}{T_R}  ( v_h(k) - v_R(k) ),
$$
$$
    v_{in}(k) = v_{ref}(k) - v_R(k) - v_f(k),
$$
$$
    v_a(k + \Delta t) = v_a(k) + \frac{\Delta t}{T_a} ( v_{in}(k) K_a - v_a(k) ),
$$
$$
    v_f(k + \Delta t) = (1 - \frac{\Delta t}{T_f}) v_f(k) + \frac{\Delta t K_f}{T_f T_{ef}} ( v_{a}(k) - (K_{ef} + sat(k)) v_{ef}(k) ),
$$
$$
    v_{ef}(k + \Delta t) = v_{ef}(k) + \frac{\Delta t}{T_{ef}} ( v_{a}(k) - (sat(k) + K_{ef}) v_{ef}(k)),
$$
$$
    sat(k) = A_{ef} e^{(B_{ef} | v_{ef}(k) | )}
$$

Since the values of all variables for $t=k$ are known, $v_{ef}(k+1)$ can be easily calculated using the discretised equations, which is carried out in the `preStep` function of the generator connected to each exciter.

The initial values of all variables, which are used in the first simulation step, are calculated assuming that the simulation starts in the steady. This is equivalent to assume that all derivative are equal to zero, which leads to:
$$
    v_R(k=0) = v_h(k=0),
$$
$$
    v_f(k=0) = 0,
$$
$$
    v_a(k=0) = K_{ef} v_{ef}(k=0) + A_{ef} e^{B_{ef} |v_{ef} (k=0)|} v_{ef}(k=0),
$$
$$
    v_{in}(k=0) = \frac{v_a(k=0)}{K_a},
$$
$$
    v_{ref}(t=0) = v_R(t=0) + v_{in}(t=0),
$$
where $v_h(k=0)$, $v_{ef}(k=0)$ are calculated using the power flow analysis and after the initialization of synchronous machines (see section initialization of SG).

## Static Exciter

<center>
<figure margin=30%>
    <img src="./images/ExciterStatic.drawio.svg" width=70% alt="Exciter static">
    <figcaption>Fig. 3: Control diagram of the Static Exciter </br>
                Adapted from [6]
    </figcaption>
</figure>
</center>
The control diagram of this is depicted in Fig. 3. It can be observed as a simplified version of the DC1 type exciter which is composed only by the regulator, the amplifier and an optional transducer. To discretize the lead-lag compensator using forward euler it is better to split this block into two parallel blocks as depicted in Fig. 4.
<center>
<figure margin=20%>
    <img src="./images/ExciterStatic_split.drawio.svg" width=70% alt="Exciter static split">
    <figcaption>Fig. 4: Control diagram of the Static Exciter </br>
    </figcaption>
</figure>
</center>
where:

$$
    C_{a} = \frac{T_{a}}{T_{b}}, \quad C_{b} = \frac{T_{b}-T_{a}}{T_{b}}.
$$
and it is described by the following set of differential equations:
$$
    T_{R} \frac{d}{dt} v_{r}(t) = v_{h}(t) - v_{r}(t)
$$
$$
    T_{b} \frac{d}{dt} x_{b}(t) = v_{in}(t) - x_{b}(t)
$$
$$
    T_{e} \frac{d}{dt} e_{fd}(t) = K_{a} v_{e}(t) - e_{fd}(t),
$$

Then, the set of differential equations are discretized using forward euler in order to solve it numerically, which leads to the following set of algebraic equations:

$$
    v_r(k + \Delta t) = v_r(k) + \frac{\Delta t}{T_R}  ( v_h(k) - v_r(k) ),
$$
$$
    v_{in}(k) = v_{ref}(k) - v_{r}(k),
$$
$$
    X_b(k + \Delta t) = \frac{\Delta t}{T_{b}} (v_{in}(k) - x_{b}(k)) + x_{b}(k),
$$
$$
    v_e(k) = K_{a} (C_{b} x_{b}(k) + C_{a} v_{in} (k)) ,
$$
$$
    e_{fd}(k + \Delta t) = \frac{\Delta t}{T_{e}}(v_{e}(k) - e_{fd}(k)) + e_{fd}(k).
$$

To consider the saturation of $e_{fd}$ there are two different implementations, which is automatically selected depending of value of the parameter $K_{bc}$:

**Standard ($K_{bc}=0$):**

$$
e^{*}_{fd} = e_{fd, max} \quad \quad if \quad \quad e^{*}_{fd} > e_{fd, max} \\
e^{*}_{fd} = e_{fd, min} \quad \quad if \quad \quad e^{*}_{fd} < e_{fd, min},
$$

where $e^{*}_{fd}$ represents the output of the exciter.

**Anti-windup ($K_{bc}>0$):** for controllers with an integral component, i.e. also for PID controllers, the so-called "windup effect" can occur when using the *standard* saturation function. A strategy for limiting the anti-windup effect is shown in Fig. 5.

<center>
<figure margin=20%>
    <img src="./images/ExciterStatic_windup.drawio.svg" width=70% alt="Exciter static split">
    <figcaption>Fig. 5: Control diagram of the Static Exciter with anti windup strategy </br>
    </figcaption>
</figure>
</center>

which means that the input of the differential equation describing $e_{fd}$, $v_{e}$, takes now the following form:

$$
v_{e} = C_{a} v_{in} + C_{b} x_{b} - K_{bc} (e_{fd} - e_{fd}^{*})
$$

The initial values of all variables, which are used in the first simulation step, are calculated assuming that the simulation starts in the steady. This is equivalent to assume that all derivative are equal to zero, which leads to:

$$
v_{r}(t=0) = v_{h}(t=0),
$$

$$
v_{e}(t=0) = \frac{e_{fd}(t=0)}{K_{a}},
$$

$$
v_{in}(t=0) = \frac{v_{e}(t=0)}{C_{a}+C_{b}},
$$

$$
x_{b}(t=0) = v_{in}(t=0),
$$

$$
v_{ref}(t=0) = v_{in}(t=0) + v_{r}(t=0)
$$

## Power System Stabilizer (PSS)

PSS is a controller of synchronous generators used to enhance damping of electromechanical oscillations. The PSS1A implemented in DPSim accepts three optional input signals: rotor speed $\omega$, active power $P$, and terminal voltage magnitude $V_h$. The combined input signal is:
$$
    s(t) = K_w \omega(t) + K_p P(t) + K_v V_h(t)
$$
Setting $K_p = K_v = 0$ recovers the speed-only special case. The PSS output $v_{pss}$ at time $t=k$ is a signal used as the input of the AVR to calculate the field voltage at $t=k+\Delta t$, $v_{fd}(k+\Delta t)$. At present, only one PSS is implemented in DPSim which is a simplified version of the IEEE PSS1A type model.

### IEEE PSS1A type PSS

<center>
<figure margin=30%>
    <img src="./images/PSS_Type1.png" width=65% alt="DC1_exciter">
    <figcaption>Fig. 6: Control diagram of the PSS Type 1 (speed input only; </br>
                the implementation also accepts active power $K_p P$ and terminal voltage $K_v V_h$). </br>
                Adapted from: Milano, Power System Modelling and Scripting
    </figcaption>
</figure>
</center>

The control diagram of this PSS is depicted in Fig. 6. It includes a washout filter and two lead-lag blocks and is described by the following set of differential equations:
$$
    T_w \frac{d}{dt} v_1(t) = -(s(t) + v_1(t)),
$$
$$
    T_2 \frac{d}{dt} v_2(t) = (1 - \frac{T_1}{T_2})(s(t) + v_1(t)) - v_2(t),
$$
$$
    T_4 \frac{d}{dt} v_3(t) = (1 - \frac{T_3}{T_4})\left(v_2(t) + \frac{T_1}{T_2}(s(t) + v_1(t))\right) - v_3(t),
$$
$$
    v_{pss}(t) = v_3(t) + \frac{T_3}{T_4}\left(v_2(t) + \frac{T_1}{T_2}(s(t) + v_1(t))\right),
$$

where $s(t) = K_w \omega(t) + K_p P(t) + K_v V_h(t)$ is the combined input signal and $v_{pss}(t)$ is the output signal used to modify the reference voltage of the AVR.

The set of differential equations are discretized using forward euler in order to solve it numerically, which leads to the following set of algebraic equations:
$$
    v_1(k + \Delta t) = v_1(k) - \frac{\Delta t}{T_w} (s(k) + v_1(k)),
$$
$$
    v_2(k + \Delta t) = v_2(k) + \frac{\Delta t}{T_2} \left((1-\frac{T_1}{T_2})(s(k) + v_1(k)) - v_2(k)\right),
$$
$$
    v_3(k + \Delta t) = v_3(k) + \frac{\Delta t}{T_4} \left((1-\frac{T_3}{T_4})\left(v_2(k) + \frac{T_1}{T_2}(s(k) + v_1(k))\right) - v_3(k)\right),
$$
$$
    v_{pss}(k) = v_3(k) + \frac{T_3}{T_4} \left(v_2(k) + \frac{T_1}{T_2} (s(k) + v_1(k))\right)
$$

Since the values of all variables for $t=k$ are known, $v_{pss}(k)$ can be easily calculated using the discretised equations, which is carried out in the `preStep` function of the generator connected to each exciter. Then, $v_{pss}(k)$ is used as input of the AVR to calculate the field voltage at time $k+1$. The values $v_1(k+1)$, $v_2(k+1)$, $v_3(k+1)$ are stored and used to calculate the PSS output of the next time step.

The initial values of all variables, which are used in the first simulation step, are calculated assuming that the simulation starts in steady state. This is equivalent to assuming that all derivatives are equal to zero, which leads to:
$$
    v_1(k=0) = -s(k=0),
$$
$$
    v_2(k=0) = (1 - \frac{T_1}{T_2})(s(k=0) + v_1(k=0)),
$$
$$
    v_3(k=0) = (1 - \frac{T_3}{T_4})\left(v_2(k=0) + \frac{T_1}{T_2}(s(k=0) + v_1(k=0))\right),
$$
$$
    v_{pss}(k=0) = v_3(k=0) + \frac{T_3}{T_4}\left(v_2(k=0) + \frac{T_1}{T_2}(s(k=0) + v_1(k=0))\right),
$$

where $s(k=0) = K_w \omega(k=0) + K_p P(k=0) + K_v V_h(k=0)$ is evaluated after the power flow analysis and initialization of synchronous machines (see section initialization of SG). In steady state $\omega(k=0) = 1.0$ (pu), and if $K_p = K_v = 0$ then $v_2 = v_3 = v_{pss} = 0$.

## Turbine Governor Models

In DPsim there are two types of Turbine Governor implementations.  The *Turbine Governor Type 1* implements both, the turbine and the governor, in one component. In difference to that, Steam Turbine and Steam Turbine Governor are implemented as two separate classes, therefore the objects are created separately. Steam/Hydro Turbine and Steam/Hydro Turbine Governor are two blocks that has to be connected in series.

The input of the turbine governos models is the mechanical omega at time $t=k-\Delta t$ and the output is the mechanical power at time $t=k$. This variable is the used by the SG to predict the mechanical omega at time $t=k+\Delta t$.

### Turbine Governor Type 1

<center>
<figure margin=30%>
    <img src="./images/TG_Type1.png" width=65% alt="TG_Type1_governor">
    <figcaption></br>Fig. 7: Control diagram of the turbine governor type 1 </br>
                Source: Milano, Power System Modelling and Scripting
    </figcaption>
</figure>
</center>
</br>

This model includes a governor, a servo and a reheat block. The control diagram of this governor is depicted in Fig. 7 and it is described by the following set of differential-algebraic equations:
$$
    p_{in}(t) = p_{ref} + \frac{1}{R} (\omega_{ref} - \omega(t)),
$$
$$
    T_s \frac{d}{dt} x_{g1}(t) = (p_{in}(t) - x_{g1}(t)),
$$
$$
    T_c \frac{d}{dt} x_{g2}(t) = ((1- \frac{T_3}{T_c})x_{g1}(t) - x_{g2}(t)),
$$
$$
    T_5 \frac{d}{dt} x_{g3}(t) = ((1-\frac{T_4}{T_5})(x_{g2} + \frac{T_3}{T_c} x_{g1}(t)) - x_{g3}(t))
$$
$$
    \tau_m (t) = x_{g3} (t) + \frac{T_4}{T_5} (x_{g2}(t) + \frac{T_3}{T_c} x_{g1}(t))
$$
where $\omega (t)$ is the input signal and $\tau_{m}(t)$ is the output signal of the governor.

The set of differential equations are discretized using forward euler in order to solve it numerically, which leads to the following set of algebraic equations:
$$
    p_{in}(k-\Delta t) = p_{ref} + \frac{1}{R} (\omega_{ref} - \omega(k-\Delta t)),
$$
$$
    x_{g1}(k) = x_{g1}(k-\Delta t) + \frac{\Delta t}{T_s} (p_{in}(k-\Delta t) - x_{g1}(k-\Delta t)),
$$
$$
    x_{g2}(k) = x_{g2}(k-\Delta t) + \frac{\Delta t}{T_c} ((1 - \frac{T_3}{T_c}) x_{g1}(k-\Delta t) - x_{g2}(k-\Delta t)),
$$
$$
    x_{g3}(k) = x_{g3}(k-\Delta t) + \frac{\Delta t}{T_5} ((1 - \frac{T_4}{T_5}) (x_{g2}(k-\Delta t) + \frac{T_3}{T_c} x_{g1}(k-\Delta t)) - x_{g3}(k-\Delta t)),
$$
$$
    \tau_m(k) = x_{g3}(k) + \frac{T_4}{T_5} (x_{g2}(k) + \frac{T_3}{T_c} x_{g1}(k)),
$$
Since the values of all variables for $t=k-\Delta t$ are known, $\tau_m(k)$ can be easily calculated using the discretised equations, which is carried out in the `preStep` function of the generator connected to each governor. Then, $\tau_m(k)$ is used to approximate the mechanical differential equations of the generator at time $k+\Delta t$.

# References

- [1] “IEEE Recommended Practice for Excitation System Models for Power System Stability Studies,” in IEEE Std 421.5-2016 (Revision of IEEE Std 421.5-2005) , vol., no., pp.1-207, 26 Aug. 2016, doi: 10.1109/IEEESTD.2016.7553421.
- [2] F. Milano, “Power system modelling and scripting,” in Power System Modelling and Scripting. London: Springer-Verlag, 2010, ISBN: 978-3-642-13669-6. doi: 10.1007/978-3-642-13669-6.
- [3] F. Milano, A. Manjavacas, “Frequency Variations in Power Systems: Modeling, State Estimation, and Control”. ISBN: 978-1-119-55184-3.
- [4] F. Milano, “Power System Analysis Toolbox: Documentation for PSAT”, ISBN: 979-8573500560.
- [5] M. Eremia; M. Shahidehpour, “Handbook of Electrical Power System Dynamics: Modeling, Stability, and Control”, <https://ieeexplore.ieee.org/book/6480471>
- [6] A. Roehder, B. Fuchs, J. Massman, M. Quester, A. Schnettler, “Transmission system stability assessment within an integrated grid development process”.
