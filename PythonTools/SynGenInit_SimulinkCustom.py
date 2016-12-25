import numpy as np
import matplotlib.pyplot as plt


psiInit = np.array([0.5359, -0.8475, 0.8404, 0.6501, -0.7752, -0.7752])

# rated values
SRated = 555e6
VRated = 24e3
FRated = 60
nPolePairs = 1
baseIfd = 1300

# per unit base
mBaseVoltage = VRated * np.sqrt(2) / np.sqrt(3);
mBaseCurrent = SRated * np.sqrt(2) / (VRated * np.sqrt(3));
mBaseImpedance = mBaseVoltage / mBaseCurrent;
mBaseAngFreq = 2 * np.pi * FRated;
mBaseInductance = mBaseImpedance / mBaseAngFreq;
mBaseFlux = mBaseInductance * mBaseCurrent;

# Impedances
Lad = 1.66 # Stator d-axis mutual inductance, Lad
Laq = 1.61 # Stator q-axis mutual inductance, Laq
Ll = 0.15 # Stator leakage inductance, Ll
Ra = 0.003 # Stator resistance, Ra
Lfd = 0.165 # Rotor field circuit inductance, Lfd
Rfd = 0.0006 # Rotor field circuit resistance, Rfd
L1d = 0.1713 # Rotor d-axis damper winding 1 inductance, L1d
R1d = 0.0284 # Rotor d-axis damper winding 1 resistance, R1d
L1q = 0.7252 # Rotor q-axis damper winding 1 inductance, L1q
R1q = 0.00619 # Rotor q-axis damper winding 1 resistance, R1q
L2q = 0.125 # Rotor q-axis damper winding 2 inductance, L2q
R2q = 0.02368 # Rotor q-axis damper winding 2 resistance, R2q

# shift_3ph = {[0 -2*pi/3 2*pi/3],'rad'};

# Conversion factor necessary to convert pu derivatives to time derivatives
oneOverOmega = 0 # One over rated electrical angular velocity

# Derived inductances for equations
Ld = Lad+Ll # Stator d-axis self inductance, Ld
Lq = Laq+Ll # Stator q-axis self inductance, Lq
Lffd = Lad+Lfd # Rotor field circuit self inductance, Lffd
Lf1d = Lffd-Lfd # Rotor field circuit and d-axis damper winding 1 mutual inductance, Lf1d
L11d = L1d+Lf1d # Rotor d-axis damper winding 1 self inductance, L11d
L11q = L1q+Laq # Rotor q-axis damper winding 1 self inductance, L11q
L22q = L2q+Laq # Rotor q-axis damper winding 2 self inductance, L22q

# Mechanical
angular_position_diff = 0 # Rotor angle wrt synchronous reference frame

# Stator magnetic flux linkages
pu_psid = 0 # Stator d-axis magnetic flux linkage
pu_psiq = 0 # Stator q-axis magnetic flux linkage

# Rotor magnetic flux linkages
pu_psifd = 0 # Field circuit magnetic flux linkage
pu_psi1d = 0 # d-axis damper winding 1 magnetic flux linkage
pu_psi1q = 0 # q-axis damper winding 1 magnetic flux linkage
pu_psi2q = 0 # q-axis damper winding 2 magnetic flux linkage

# Mechanical
torque = 0 # Mechanical torque

# Stator currents
I = np.array([0, 0, 0]) # Phase currents

# Rotor currents
ifd = 0 # Field circuit current
pu_i1d = 0 # d-axis damper winding 1 current
pu_i1q = 0 # q-axis damper winding 1 current
pu_i2q = 0 # q-axis damper winding 2 current


# Temporary variables
base_SPerPhase = SRated/3
base_V = VRated/np.sqrt(3) # phase RMS
base_I = base_SPerPhase/base_V # phase RMS
base_wElectrical = 2*np.pi*FRated

# Base values used in equations
base_v = np.sqrt(2)*base_V # instantaneous
base_i = np.sqrt(2)*base_I # instantaneous
base_wMechanical = base_wElectrical/nPolePairs
oneOverOmega = 1/base_wElectrical
base_torque = SRated/base_wMechanical
base_rotor_circuit_ifd = Lad*baseIfd
base_rotor_circuit_efd = SRated/base_rotor_circuit_ifd
base_impedance = base_v / base_i
base_inductance = base_impedance * oneOverOmega
base_flux = base_inductance * base_i

print(base_flux)
