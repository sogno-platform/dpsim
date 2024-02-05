---
title: "Synchronous Generator Regulators"
linkTitle: "Synchronous Generator Regulators"
date: 2023-01-23
---

In DPSim, synchronous generator control systems are solved separately from the electric network. The outputs of the electric network (active and reactive power, node voltages, branch currents and rotor speed of synchronous generators) at time $k- \Delta t$ are used as the input of the controllers to calculate their states at time $k$. Because of the relatively slow response of the controllers, the error in the network solution due to the time delay $\Delta t$ introduced by this approach is negligible.  

## Exciter

 DC1 type model is the standard IEEE type DC1 exciter, whereas the other model is a simplified version of the IEEE DC1 type model. The inputs of the exciters are the magnitude of the terminal voltage of the generator connected to the exciter $v_h$ and the voltage reference $v_{ref}$, which is defined as a variable since other devices such as over-excitation limiters or power system stabilizers (PSS) modify such reference with additional signals. At the moment, no over-excitation limiters have been implemented in DPSim so that the reference voltage is given by:
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

### Static Exciter

The exciter has two inputs $ \Delta u = u _{ref} – u_{h}$, where $u_{h}$ ist the measured terminal voltage. The second input $u_{s}$, or also as $v_{pss}$ is the signal of PSS.

For the initialization of exciter $u_{ref}$ is unknown. Generator terminal voltage $v_{h}$ from power flow and generators EMF $e_{fd}$ computed at the generator initialization. At the initialization PSS output $u_{s}$ ist set to $0$, as steady state is assumed. Setting Laplace variable $s={0}$ steady state is assumed and unknown $u_{ref}$ can be calculated.

For the step function the lead lag block $\frac{1+sT_A}{1+sT_B}$ is represented as two parallel transfer functions $ \frac{T_A}{T_B}+ \frac{T_B-T_A}{T_B} \frac{1}{1+sT_B} $. This lead lag block provides system stabilization by transient gain reduction. Exciter machine is represented by $\frac{1}{1+sT_E}$ and a proportional controller by $K_A$.

In the .cpp file of this exciter a wind-up can activated.

<center>
<figure margin=30%>
    <img src="./images/ExciterStatic.jpg" width=65% alt="DC1_exciter">
    <figcaption>Fig. 3: Control diagram of the Static Exciter </br>
                Source: A. Roehder, B. Fuchs, J. Massman, M. Quester, A. Schnettler, "Transmission system stability assessment within an integrated grid development process"
    </figcaption>
</figure>
</center>

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


## Turbine Governor Models
In DPsim there are two type of Turbine Governor implementations.  The *Turbine Governor Type 1* implements both, the turbine and the governor, in one component. In difference to that, Steam Turbine and Steam Turbine Governor are implemented as two separate classes, therefore the objects are created separately. Steam/Hydro Turbine and Steam/Hydro Turbine Governor are two blocks that has to be connected in series. 

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
Since the values of all variables for $t=k$ are known, $\tau_m(k+1)$ can be easily calculated using the discretised equations, which is carried out in the `preStep` function of the generator connected to each governor. Then, $\tau_m(k)$ is used to approximate the mechanical differential equations of the generator at time $k+1$. The valueof $\tau_m(k+1)$ is stored and used to approximate the machanical variables of the generator at time $k+2$, and so on.

### Steam Turbine
**Note 1**: In the step function of each PT1 lag element the integrator $\frac{1}{s}$ is simulated with Forward Euler Method, it the output value for $k$ calculated in the previous step and following the Forward Euler formula calculate the value for $k+1$. To avoid unnecessary dead beat behavior, complex transfer functions, with more then one pole and zero, are represented as a sum of multiple PT1 elements connected in parallel, via partial fracture decomposition. Parallel blocks are discretized separately with the same time step. 

More information on below governors and turbines can be found in the book "Handbook of Electrical Power System Dynamics: Modeling, Stability, and Control", linked below in references.

**Steam Turbine** receives as input signal from Steam Turbine Governor $p_{gv}$ and as output gives the mechanical power $p_{m}$ to a synchronous generator. Steam Turbine is divided in high-pressure, intermediate-pressure and low-pressure stages. Each of them is modeled as a first order lag element with time constants $T_{CH}, T_{RH}, T_{CO}$. If a time constant is chosen to be equal to zero, the according lag element is deactivated. The total turbine mechanical power is a sum of powers provided by each stage, each fraction can be modeled with corresponding gain-factors $F_{HP}, F_{IP}, F_{LP}$. Note that the sum of gain-factors should be equal to one $F_{HP}+F_{IP}+F_{LP} = 1$.

<center>
<figure margin=30%>
    <img src="./images/SteamTurbine.jpg" width=65% alt="DC1_exciter">
    <figcaption></br>Fig. 4: Diagram of the steam turbine </br>
                Source: A. Roehder, B. Fuchs, J. Massman, M. Quester, A. Schnettler, "Transmission system stability assessment within an integrated grid development process"
    </figcaption>
</figure>
</center>
</br>

### Steam Turbine and Steam Turbine Governor
**Steam turbine governor** get as input the frequency (rotational speed of the generator)  deviation $\Delta\omega$ from set-frequency ($50 Hz$ or $60 Hz$) and gives target value $p_{gv}$ to the turbine , $p_0$ is mechanical power that is produced at set-frequency. The governor is implemented as a controller $\frac{K(1+sT_2)}{(1+sT_1)}$, where $K=1/R$ with $R$-droop coefficient, and a PT1 lag element with embedded limiters before and after the integrator. The Controller is Simulated as a gain parallel to a PT1 element $\frac{K(1+sT_2)}{(1+sT_1)}=K(\frac{T_2}{T_1}+\frac{T_1-T_2}{T_1} \frac{1}{1+sT_1})$. For safety reasons in the governor output signal is limited to the range [0 pu, 1 pu].

<center>
<figure margin=30%>
    <img src="./images/SteamGovernor.jpg" width=65% alt="DC1_exciter">
    <figcaption></br>Fig. 4: Control diagram of the steam turbine governor </br>
                Source: A. Roehder, B. Fuchs, J. Massman, M. Quester, A. Schnettler, "Transmission system stability assessment within an integrated grid development process"
    </figcaption>
</figure>
</center>
</br>

### Hydro Turbine
**Hydro Turbine** receives as input signal from Hydro Turbine Governor $p_{gv}$ and as output gives the mechanical power $p_{m}$ to a synchronous generator. The transfer function is specified by water starting time parameter $T_{w}$. The transfer function is a sum of two parallel blocks $\frac{1-sT_w}{1+0.5sT_w}=-2+\frac{3}{1+0.5sT_w}$ which is discretized separately.

<center>
<figure margin=30%>
    <img src="./images/HydroTurbine.jpg" width=65% alt="DC1_exciter">
    <figcaption></br>Fig. 4: Diagram of a hydro turbine </br>
                Source: A. Roehder, B. Fuchs, J. Massman, M. Quester, A. Schnettler, "Transmission system stability assessment within an integrated grid development process"
    </figcaption>
</figure>
</center>
</br>

### Hydro Turbine Governor
**Hydro Turbine Governor** get as input the frequency (rotational speed of the generator)  deviation $\Delta\omega$ from set-frequency ($50 Hz$ or $60 Hz$) and gives target value $p_{gv}$ to the turbine , $p_0$ is mechanical power that is produced at set-frequency. The controller transfer Function is defined as $K\frac{1+sT_2}{(1+sT_1)(1+sT_3)}=K(\frac{T_1-T_2}{T_1-T_3} \frac{1}{1+sT_1} + \frac{T_2-T_3}{T_1-T_3} \frac{1}{1+sT_3})$, where $K=\frac{1}{R}$ and $R$ is the droop coefficient. Two parallel PT1 blocks are discretized separately, their outputs are weighted by according factors and added. The disadvantage of the parallel block representation is the error the time constants are equal, that is why is should it must apply $T_1 \neq T_2$. The sum of both blocks, the output $p_{gv}$ is limited to the range [0pu, 1pu].  

<center>
<figure margin=30%>
    <img src="./images/HydroGovernor.jpg" width=65% alt="DC1_exciter">
    <figcaption></br>Fig. 4: Control diagram of a hydro turbine governor </br>
                Source: A. Roehder, B. Fuchs, J. Massman, M. Quester, A. Schnettler, "Transmission system stability assessment within an integrated grid development process"
    </figcaption>
</figure>
</center>
</br>

## References
 - "IEEE Recommended Practice for Excitation System Models for Power System Stability Studies," in IEEE Std 421.5-2016 (Revision of IEEE Std 421.5-2005) , vol., no., pp.1-207, 26 Aug. 2016, doi: 10.1109/IEEESTD.2016.7553421.
 - F. Milano, “Power system modelling and scripting,” in Power System Modelling and Scripting. London: Springer-Verlag, 2010, ISBN: 978-3-642-13669-6. doi: 10.1007/978-3-642-13669-6.
 - F. Milano, A. Manjavacas, “Frequency Variations in Power Systems: Modeling, State Estimation, and Control”. ISBN: 978-1-119-55184-3.
 - F. Milano, "Power System Analysis Toolbox: Documentation for PSAT", ISBN: 979-8573500560.
 - A. Roehder, B. Fuchs, J. Massman, M. Quester, A. Schnettler, "Transmission system stability assessment within an integrated grid development process".
 - M. Eremia; M. Shahidehpour, "Handbook of Electrical Power System Dynamics: Modeling, Stability, and Control", https://ieeexplore.ieee.org/book/6480471
