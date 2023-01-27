---
title: "Synchronous Generator Regulators"
linkTitle: "Synchronous Generator Regulators"
date: 2023-01-23
---

In DPSim, synchronous generator control systems are solved separately from the electric network. The outputs of the electric network (active and reactive power, node voltages, branch currents and rotor speed of synchronous generators) at time $k- \Delta t$ are used as the input of the controllers to calculate their states at time $k$. Because of the relatively slow response of the controllers, the error in the network solution due to the time delay $\Delta t$ introduced by this approach is negligible.  

## Exciter

There are currently two different exciter models in DPSim. DC1 type model is the standard IEEE type DC1 exciter, whereas the other model is a simplified version of the IEEE DC1 type model. The inputs of the exciters are the magnitude of the terminal voltage of the generator connected to the exciter $v_h$ and the voltage reference $v_{ref}$, which is defined as a variable since other devices such as over-excitation limiters or power system stabilizers (PSS) modify such reference with additional signals. At the moment, no over-excitation limiters have been implemented in DPSim so that the reference voltage is given by:
$$
    v_{ref}(t) = v_{ref,0} + v_{pss}(t)
$$
where $v_{ref,0}$ is initialized after the power flow computations and $v_{pss}(t)$ is the output of the (optional) PSS connected to the exciter. The output of the exciter systems is the induced emf by the field current $v_{ef}$. 

### IEEE Type DC1 exciter model
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
    T_R \frac{d}{dt} v_R(t) = v_h(t) - v_R(t),
$$
$$
    T_b \frac{d}{dt} v_b(t) = v_{ref} - v_R(t) - v_f(t) - v_b(t),
$$
$$
    T_a \frac{d}{dt} v_a(t) = K_a v_{in}(t) - v_a(t),
$$
$$
    T_f \frac{d}{dt} v_f(t) - K_f \frac{d}{dt} v_{ef}(t) = -v_f(t),
$$
$$
    T_{ef} \frac{d}{dt} v_{ef}(t) = v_a(t) - (K_{ef} + sat(t)) v_{ef}(t),
$$
where $v_h$ is the module of the machine's terminal voltage, and $v_{in}$ is the amplifier input signal, which for the IEEE Type DC1 is given by:
$$
    v_{in}(t) = T_{c} \frac{d}{dt} v_b(t) + v_b(t).
$$

The ceiling function approximates the saturaiton of the excitation winding:
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


### Simplified IEEE Type DC1 exciter model (DC1Simp)
<center>
<figure>
    <img src="./images/DC1CSimp_exciter.png" width=65% alt="DC1A_exciter">
    <figcaption></br>Fig. 2: Control diagram of the IEEE Type DC1 exciter </br>
            Adapted from: Milano, Power System Modelling and Scripting
    </figcaption>
</figure>
</center>
</br>

Because the time constants $T_b$ and $T_c$ of the IEEE Type DC1 exciter model are frequenty small enough to be neglected, in DPSim a simplified model of this exciter which neglect these time constants is also implemented. The control diagram of this exciter is depicted in Fig. 2 and it is described by the following set of differential equations:
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

## Power Systen Stabilizer (PSS)
PSS is a controller of synchonous generators used to enhance damping electromechanical oscillations. The input of the PSS implemented in DPSim is the rotor speed of the machine. The PSS output is a signal used to modify the reference voltage of the AVR. At present, only one PSS is implemented in DiPSim which is a simplified version of the IEEE PSS1A type model.

### IEEE PSS1A type PSS
<center>
<figure margin=30%>
    <img src="./images/PSS_Type1.png" width=65% alt="DC1_exciter">
    <figcaption>Fig. 3: Control diagram of the PSS Type 1 </br>
                Adapted from: Milano, Power System Modelling and Scripting
    </figcaption>
</figure>
</center>

The control diagram of this PSS is depicted in Fig. 3. It includes a washout filter and two lead-lag blocks and is described by the following set of differential equations:
$$
    T_w \frac{d}{dt} v_1(t) = -(K_w \omega (t) + v_1(t)),
$$
$$
    T_2 \frac{d}{dt} v_2(t) = ((1 - \frac{T_1}{T_2})(K_w \omega (t) + v_1(t)) - v_2(t)),
$$
$$
    T_4 \frac{d}{dt} v_3(t) = ((1 - \frac{T_3}{T_4})(v_2(t) + (\frac{T_1}{T_2}(K_w \omega (t) + v_1(t)))) - v_3(t)),
$$
$$
    v_{pss}(t) = v_3(t) + \frac{T_3}{T_4}(v_2(t) + \frac{T_1}{T_2}(K_w \omega (t) + v_1(t))),
$$

where $\omega (t)$ is the input signal of the PSS (rotor speed) and $v_{pss}(t)$ is the output signal of the PSS which is used as an optional signal of the AVR connected to the machine and it is used to modify the reference voltage of the AVR.

The set of differential equations are discretized using forward euler in order to solve it numerically, which leads to the following set of algebraic equations: 
$$
    v_1(k+1) = v_1(k) - \frac{\Delta t}{T_w} (K_w \omega (k) + v_1(k)),
$$
$$
    v_2(k+1) = v_2(k) + \frac{\Delta t}{T_2} ((1-\frac{T_1}{T_2})(K_w \omega (k) + v_1(k)) - v_2(k)),
$$
$$
    v_3(k+1) = v_3(k) + \frac{\Delta t}{T_4} ((1-\frac{T_3}{T_4})(v_2(k) + \frac{T_1}{T_2}(K_w \omega (k) + v_1(k))) - v_{pss}(k)),
$$
$$
    v_{pss}(k) = v_3(k) + \frac{T_3}{T_4} (v_2(k) + \frac{T_1}{T_2} (K_w \omega (k) + v_1(k)))
$$

Since the values of all variables for $t=k$ are known, $v_{pss}(k)$ can be easily calculated using the discretised equations, which is carried out in the `preStep` function of the generator connected to each exciter. Then, $v_{pss}(k)$ is used as input of the AVR to calculate the field voltage at time $k+1$. The values $v_1(k+1)$, $v_2(k+1)$, $v_3(k+1)$ are stored and used to calculate the PSS output of the next time step.  

The initial values of all variables, which are used in the first simulation step, are calculated assuming that the simulation starts in the steady. This is equivalent to assume that all derivative are equal to zero, which leads to:
$$
    v_1(k=0) = -K_w \omega (k=0),
$$
$$
    v_2(k=0) = 0,
$$
$$
    v_3(k=0) = 0,
$$
$$
    v_{PSS}(k=0) = 0,
$$
where $\omega(k=0)$ is calculated after the power flow analysis and after the initialization of synchronous machines (see section initialization of SG) and normally is equal to $1.0$ (pu).


## Turbine Governor

### Turbine Governor Type 1
<center>
<figure margin=30%>
    <img src="./images/TG_Type1.png" width=65% alt="DC1_exciter">
    <figcaption></br>Fig. 4: Control diagram of the turbine governor type 1 </br>
                Source: Milano, Power System Modelling and Scripting
    </figcaption>
</figure>
</center>
</br>

This model includes a governor, a servo and a reheat block. The control diagram of this governor is depicted in Fig. 4 and it is described by the following set of differential-algebraic equations:
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
    p_{in}(k) = p_{ref} + \frac{1}{R} (\omega_{ref} - \omega(k)),
$$
$$
    x_{g1}(k+1) = x_{g1}(k) + \frac{\Delta t}{T_s} (p_{in}(k) - x_{g1}(k)),
$$
$$
    x_{g2}(k+1) = x_{g2}(k) + \frac{\Delta t}{T_c} ((1 - \frac{T_3}{T_c}) x_{g1}(k) - x_{g2}(k)),
$$
$$
    x_{g3}(k+1) = x_{g3}(k) + \frac{\Delta t}{T_5} ((1 - \frac{T_4}{T_5}) (x_{g2}(k) + \frac{T_3}{T_c} x_{g1}(k)) - x_{g3}(k)),
$$
$$
    \tau_m(k+1) = x_{g3}(k+1) + \frac{T_4}{T_3} (x_{g2}(k+1) + \frac{T_3}{T_c} x_{g1}(k+1)),
$$
Since the values of all variables for $t=k$ are known, $\tau_m(k+1)$ can be easily calculated using the discretised equations, which is carried out in the `preStep` function of the generator connected to each exciter. Then, $\tau_m(k)$ is used to approximate the mechanical differential equations of the generator at time $k+1$. The valueof $\tau_m(k+1)$ is stored and used to approximate the machanical variables of the generator at time $k+2$, and so on.  


## References
 - "IEEE Recommended Practice for Excitation System Models for Power System Stability Studies," in IEEE Std 421.5-2016 (Revision of IEEE Std 421.5-2005) , vol., no., pp.1-207, 26 Aug. 2016, doi: 10.1109/IEEESTD.2016.7553421.
 - F. Milano, “Power system modelling and scripting,” in Power System Modelling and Scripting. London: Springer-Verlag, 2010, ISBN: 978-3-642-13669-6. doi: 10.1007/978-3-642-13669-6.
 - F. Milano, A. Manjavacas, “Frequency Variations in Power Systems: Modeling, State Estimation, and Control”. ISBN: 978-1-119-55184-3.
 - F. Milano, "Power System Analysis Toolbox: Documentation for PSAT", ISBN: 979-8573500560.