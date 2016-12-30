import numpy as np

### Synchronous Machine Data ###################################################
# Kundur - Exampel 3.1 p. 91
# nominal values
nom_P = 555e6 				# nominal power
nom_V = 24e3 				# nominal phase to phase RMS voltage
nom_f = 60					# nominal electrical frequency
wElec = 2 * np.pi * nom_f	# nominal electrical angular frequency

# stator and rotor parameters in their respective systems
Rs = 0.0031 		# armature winding resistance
Ll = 0.4129e-3 		# leakage inductance
Lmd = 4.5696e-3 	# d-axis mutual inductance
Lmq = 4.432e-3 		# q-axis mutual inductance
Ld = Lmd + Ll 		# d-axis self inductance
Lq = Lmq + Ll 		# q-axis self inductance

Rfd = 0.0715 		# field winding resistance
Lmfd = 40e-3 		# field winding mutual inductance
Lfd = 576.92e-3 	# field winding self inductance
Llfd = Lfd - Lmfd 	# field winding leakage inductance
Rkd = 0.0295 		# d-axis damper winding resistance
Llkd = 0.0005		# d-axis damper winding leakage inductance
Rkq1 = 0.0064 		# q-axis damper winding 1 resistance
Llkq1 = 0.002 		# q-axis damper winding 1 leakage inductance
Rkq2 = 0.0246 		# q-axis damper winding 2 resistance
Llkq2 = 0.0003 		# q-axis damper winding 2 leakage inductance

J = 27547.8 		# inertia
numPoles = 2 		# number of poles

# reactances
Xd = Ld * wElec
Xq = Lq * wElec
Xmd = Lmd * wElec
Xmq = Lmq * wElec

# per unit stator and rotor base values
base_v = nom_V * np.sqrt(2) / np.sqrt(3) 			# peak voltage
base_V = nom_V / np.sqrt(3) 						# RMS voltage
base_i = nom_P * np.sqrt(2) / (nom_V * np.sqrt(3)) 	# peak current
base_I = nom_P / (nom_V * np.sqrt(3)) 				# RMS current
base_Z = base_v / base_i 							# impedance
base_wElec = 2 * np.pi * nom_f 						# elctrical angular frequency
base_wMech = base_wElec / (numPoles / 2) 			# mechanical angular frequency
base_L = base_Z / base_wElec 						# inductance
base_psi = base_L * base_i 							# flux linkage

base_ifd = Lmd / Lmfd * base_i 						# field current
base_vfd = nom_P / base_ifd 						# field voltage
base_Zfd = base_vfd / base_ifd 						# field impedance
base_Lfd = base_Zfd / base_wElec 					# field inductance

base_T = nom_P / base_wElec 						# torque

### Scenario ###################################################################
# This is not done in the beginning but after the per unit base calculation
# because the mutual inductances which are used for the per unit base
# calculation may vary between scenarios due to saturation.

# steady state load flow conditions
vtPhPh_RMS_scen = 24e3 	# phase to phase terminal RMS voltage
# low power
P_scen = 555e3 			# active power
Q_scen = 0 				# reactive power
# high power
#P_scen = 555e6 * 0.9 	# acive power
#Q_scen = 555e6 * 0.436 # reactive power
#Lmd = Lmd * 0.835 		# saturated value of the d-axis mutual inductance
#Lmq = Lmq * 0.835 		# saturated value of the q-axis mutual inductance
#Ld = Lmd + Ll 			# d-axis self inductance recalculation
#Lq = Lmq + Ll 			# q-axis self inductance recalculation

### Calculation of initial conditions in steady state ##########################

# per unit parameters
pu_Rs = Rs / base_Z
pu_Ll = Ll / base_L
pu_Lmd = Lmd / base_L
pu_Lmq = Lmq / base_L
pu_Ld = Ld / base_L
pu_Lq = Lq / base_L

pu_Rfd = Rfd / base_Zfd
pu_Lmfd = Lmfd / base_L * base_ifd / base_i
pu_Lfd = Lfd / base_Lfd
pu_Llfd = pu_Lfd - pu_Lmfd

# steady state per unit initial values
pu_init_P = P_scen / nom_P
pu_init_Q = Q_scen / nom_P
pu_init_S = np.sqrt( pow(pu_init_P,2) + pow(pu_init_Q,2) )
pu_init_vt = vtPhPh_RMS_scen / np.sqrt(3) * np.sqrt(2) / base_v
pu_init_it = pu_init_S / pu_init_vt
pu_init_pf = np.arccos(pu_init_P / pu_init_S)
pu_init_delta = np.arctan(
	(pu_Lq * pu_init_it * np.cos(pu_init_pf)
	- pu_Rs * pu_init_it * np.sin(pu_init_pf)) /
	(pu_init_vt + pu_Rs * pu_init_it * np.cos(pu_init_pf)
	+ pu_Lq * pu_init_it * np.sin(pu_init_pf)) ) 	# load angle
pu_init_delta_deg = pu_init_delta / np.pi * 180
pu_init_vd = pu_init_vt * np.sin(pu_init_delta)
pu_init_vq = pu_init_vt * np.cos(pu_init_delta)
pu_init_id = pu_init_it * np.sin(pu_init_delta + pu_init_pf)
pu_init_iq = pu_init_it * np.cos(pu_init_delta + pu_init_pf)
pu_init_ifd = (pu_init_vq + pu_Rs * pu_init_iq + pu_Ld * pu_init_id) / pu_Lmd
pu_init_vfd = pu_Rfd * pu_init_ifd
pu_init_psid = pu_init_vq + pu_Rs * pu_init_iq
pu_init_psiq = - pu_init_vd - pu_Rs * pu_init_id
pu_init_psifd = (pu_Lmd + pu_Llfd) * pu_init_ifd - pu_Lmd * pu_init_id
pu_init_psid1 = pu_Lmd * (pu_init_ifd - pu_init_id)
pu_init_psiq1 = - pu_Lmq * pu_init_iq
pu_init_psiq2 = - pu_Lmq * pu_init_iq
pu_init_Te = pu_init_P + pu_Rs * pow(pu_init_it, 2)
pu_H = 0.5 * (2/numPoles)**2 * J * base_wElec**2 / nom_P

# steady state initial values in real units
init_P = P_scen
init_Q = Q_scen
init_S = np.sqrt( pow(init_P,2) + pow(init_Q,2) )
init_vt = vtPhPh_RMS_scen / np.sqrt(3) * np.sqrt(2)
init_it = init_S / (nom_V * np.sqrt(3)) * np.sqrt(2)
init_pf = np.arccos(init_P / init_S)
init_delta = np.arctan(
	(Xq * init_it * np.cos(init_pf)
	- Rs * init_it * np.sin(init_pf)) /
	(init_vt + Rs * init_it * np.cos(init_pf)
	+ Xq * init_it * np.sin(init_pf)) ) # load angle
init_delta_deg = init_delta / np.pi * 180
init_vd = init_vt * np.sin(init_delta)
init_vq = init_vt * np.cos(init_delta)
init_id = init_it * np.sin(init_delta + init_pf)
init_iq = init_it * np.cos(init_delta + init_pf)
# rotor variables referred to stator
init_ifd = (init_vq + pu_Rs * init_iq + Xd * init_id) / Xmd
# The remaining variables require the rotor parameters to be referred to the
# stator. The stator parameters remain unchanged.

### Calculate initial abc stator values and transform to dq ####################
# Initialize electrical system - positive currents are out of the stator.
# This has to be considered for the determination of the power factor angle
# sign.
peak_i = init_it
peak_v = init_vt
init_thetaElec = - np.pi / 2
init_ia = peak_i * np.cos(init_thetaElec + init_pf)
init_ib = peak_i * np.cos(init_thetaElec + init_pf - 2 * np.pi / 3)
init_ic = peak_i * np.cos(init_thetaElec + init_pf + 2 * np.pi / 3)
init_va = peak_v * np.cos(init_thetaElec)
init_vb = peak_v * np.cos(init_thetaElec - 2 * np.pi / 3)
init_vc = peak_v * np.cos(init_thetaElec + 2 * np.pi / 3)

sysVabcs = np.array([init_va, init_vb, init_vc]).reshape((3,1))
sysIabcs = np.array([init_ia, init_ib, init_ic]).reshape((3,1))

# Park-Transforms
thetaTr = init_thetaElec + init_delta
ParkTransformKrause = np.array([[ 2. / 3. * np.cos(thetaTr), 2. / 3. * np.cos(thetaTr - 2. * np.pi / 3.), 2. / 3. * np.cos(thetaTr + 2. * np.pi / 3.)],
	[2 / 3 * np.sin(thetaTr), 2. / 3. * np.sin(thetaTr - 2. * np.pi / 3.), 2. / 3. * np.sin(thetaTr + 2. * np.pi / 3.)],
	[1. / 3., 1. / 3., 1. / 3.]])

InvParkTransformKrause = np.array([[ np.cos(thetaTr), np.sin(thetaTr), 1],
	[np.cos(thetaTr - 2. * np.pi / 3.), np.sin(thetaTr - 2. * np.pi / 3.), 1],
	[np.cos(thetaTr + 2. * np.pi / 3.), np.sin(thetaTr + 2. * np.pi / 3.), 1]])

vdq0 = np.dot(ParkTransformKrause, sysVabcs)
idq0 = np.dot(ParkTransformKrause, sysIabcs)
