import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.interpolate import interp1d

def plotNodeVoltageInterpDpRef(filenameRef, filenameDP, node):
	node = node - 1
	dfRef = pd.read_csv(filenameRef, header=None)
	dfDP = pd.read_csv(filenameDP, header=None)
	
	if (dfRef.shape[1] - 1) < node or node < 0:
		print('Node not available')
		exit()
		
	if (dfDP.shape[1] - 1) / 2 < node or node < 0:
		print('Node not available')
		exit()
	
	# Ref
	timeRef = np.array(dfRef.ix[:,0])
	voltageRef = np.array(dfRef.ix[:,node + 1])
	
	#DP interpolated
	timeDP = np.array(dfDP.ix[:,0])
	voltageReDP = np.array(dfDP.ix[:,node + 1])
	voltageImDP = np.array(dfDP.ix[:, int((dfDP.shape[1] - 1) / 2 + node + 1)])
	
	interpTime = np.arange(dfDP.ix[0, 0], dfDP.ix[ dfDP.shape[0] - 1, 0], 0.00005)
	fVoltageRe = interp1d(timeDP, voltageReDP)
	fVoltageIm = interp1d(timeDP, voltageImDP)
	interpVoltageRe = fVoltageRe(interpTime)
	interpVoltageIm = fVoltageIm(interpTime)	
	
	voltageShiftDPInterp = interpVoltageRe*np.cos(2*np.pi*50*interpTime) - interpVoltageIm*np.sin(2*np.pi*50*interpTime)
	voltageAbsDP = np.sqrt(voltageReDP**2+voltageImDP**2)	
		
	fig, ax1 = plt.subplots()
	ax1.plot(timeRef, voltageRef, 'm:', label='Ref')
	ax1.plot(interpTime, voltageShiftDPInterp, 'b--', label='DP interp')
	ax1.plot(timeDP, voltageAbsDP, 'r-', label='DP abs')
	
	# Now add the legend with some customizations.
	legend = ax1.legend(loc='lower right', shadow=True)

	# The frame is matplotlib.patches.Rectangle instance surrounding the legend.
	frame = legend.get_frame()
	frame.set_facecolor('0.90')

	ax1.set_xlabel('time [s]')
	ax1.set_ylabel('mag [V]')
	ax1.grid(True)
	plt.show()

def plotNodeVoltageDpEmtRef(filenameRef, filenameDP, filenameEMT, node):
	node = node - 1
	dfRef = pd.read_csv(filenameRef, header=None)
	dfEMT = pd.read_csv(filenameEMT, header=None)
	dfDP = pd.read_csv(filenameDP, header=None)
	
	if (dfRef.shape[1] - 1) < node or node < 0:
		print('Node not available')
		exit()

	if (dfEMT.shape[1] - 1) < node or node < 0:
		print('Node not available')
		exit()
		
	if (dfDP.shape[1] - 1) / 2 < node or node < 0:
		print('Node not available')
		exit()
	
	# Ref
	timeRef = np.array(dfRef.ix[:,0])
	voltageRef = np.array(dfRef.ix[:,node + 1])
	
	# EMT
	timeEMT = np.array(dfEMT.ix[:,0])
	voltageEMT = np.array(dfEMT.ix[:,node + 1])	
	
	#DP
	timeDP = np.array(dfDP.ix[:,0])
	voltageReDP = np.array(dfDP.ix[:,node + 1])
	voltageImDP = np.array(dfDP.ix[:, int((dfDP.shape[1] - 1) / 2 + node + 1)])
	voltageAbsDP = np.sqrt(voltageReDP**2+voltageImDP**2)
	voltageShiftDP = voltageReDP*np.cos(2*np.pi*50*timeDP) - voltageImDP*np.sin(2*np.pi*50*timeDP)
		
	fig, ax1 = plt.subplots()
	ax1.plot(timeRef, voltageRef, 'm:', label='Ref')
	ax1.plot(timeEMT, voltageEMT, 'g--', label='EMT')
	ax1.plot(timeDP, voltageShiftDP, 'b--', label='DP shift')
	ax1.plot(timeDP, voltageAbsDP, 'r-', label='DP abs')
	
	# Now add the legend with some customizations.
	legend = ax1.legend(loc='lower right', shadow=True)

	# The frame is matplotlib.patches.Rectangle instance surrounding the legend.
	frame = legend.get_frame()
	frame.set_facecolor('0.90')

	ax1.set_xlabel('time [s]')
	ax1.set_ylabel('mag [V]')
	ax1.grid(True)
	plt.show()


def plotNodeVoltageDpEmt(filenameDP, filenameEMT, node):
	node = node - 1
	dfEMT = pd.read_csv(filenameEMT, header=None)
	dfDP = pd.read_csv(filenameDP, header=None)

	if (dfEMT.shape[1] - 1) < node or node < 0:
		print('Node not available')
		exit()
		
	if (dfDP.shape[1] - 1) / 2 < node or node < 0:
		print('Node not available')
		exit()
	
	# EMT
	timeEMT = np.array(dfEMT.ix[:,0])
	voltageEMT = np.array(dfEMT.ix[:,node + 1])	
	
	#DP
	timeDP = np.array(dfDP.ix[:,0])
	voltageReDP = np.array(dfDP.ix[:,node + 1])
	voltageImDP = np.array(dfDP.ix[:, int((dfDP.shape[1] - 1) / 2 + node + 1)])
	voltageAbsDP = np.sqrt(voltageReDP**2+voltageImDP**2)
	voltageShiftDP = voltageReDP*np.cos(2*np.pi*50*timeDP) - voltageImDP*np.sin(2*np.pi*50*timeDP)
		
	fig, ax1 = plt.subplots()
	ax1.plot(timeEMT, voltageEMT, 'g--', label='EMT')
	ax1.plot(timeDP, voltageShiftDP, 'b--', label='DP shift')
	ax1.plot(timeDP, voltageAbsDP, 'r-', label='DP abs')
	
	# Now add the legend with some customizations.
	legend = ax1.legend(loc='lower right', shadow=True)

	# The frame is matplotlib.patches.Rectangle instance surrounding the legend.
	frame = legend.get_frame()
	frame.set_facecolor('0.90')

	ax1.set_xlabel('time [s]')
	ax1.set_ylabel('mag [V]')
	ax1.grid(True)
	plt.show()

def plotEmtNodeResults(filename, node):
	node = node - 1
	df = pd.read_csv(filename, header=None)
	print(df.shape)

	if (df.shape[1] - 1) < node or node < 0:
		print('Node not available')
		exit()

	time = np.array(df.ix[:,0])
	voltage = np.array(df.ix[:,node + 1])	
	
	fig, ax1 = plt.subplots()
	ax1.plot(time, voltage, 'b-')
	#plt.yticks(np.arange(-10, 10, 1.0))
	ax1.set_xlabel('time [s]')
	ax1.set_ylabel('mag [V] or [A]')
	ax1.grid(True)
	plt.show()


def plotDpDiff(filename1, node1, filename2, node2):
    node1 = node1 - 1
    node2 = node2 - 1
    df1 = pd.read_csv(filename1, header=None)
    df2 = pd.read_csv(filename2, header=None)

    if (df1.shape[1] - 1) / 2 < node1 or node1 < 0:
        print('Node 1 not available')
        exit()

    if (df2.shape[1] - 1) / 2 < node2 or node2 < 0:
        print('Node 2 not available')
        exit()

    # this assumes same timestep for both runs
    time = np.array(df1.ix[:,0])
    re1 = np.array(df1.ix[:,node1 + 1])
    re2 = np.array(df2.ix[:,node2 + 1])
    im1 = np.array(df1.ix[:,int((df1.shape[1] - 1) / 2 + node1 + 1)])
    im2 = np.array(df2.ix[:,int((df2.shape[1] - 1) / 2 + node2 + 1)])
    abs1 = np.sqrt(re1**2+im1**2)
    abs2 = np.sqrt(re2**2+im2**2)
    diff = np.sqrt((re1-re2)**2+(im1-im2)**2)

    fig, ax = plt.subplots()
    ax.plot(time, abs1, 'b-', time, abs2, 'r-', time, diff, 'g-')
    ax.set_xlabel('time [s]')
    ax.set_ylabel('mag [V]')
    ax.grid(True)
    plt.show()

def plotNodeResults(filename, node):
	node = node - 1
	df = pd.read_csv(filename, header=None)
	print(df.shape)

	if (df.shape[1] - 1) / 2 < node or node < 0:
		print('Node not available')
		exit()

	time = np.array(df.ix[:,0])
	voltageRe = np.array(df.ix[:,node + 1])
	voltageIm = np.array(df.ix[:, int((df.shape[1] - 1) / 2 + node + 1)])

	voltage = np.sqrt(voltageRe**2+voltageIm**2)
	voltageEmt = voltageRe*np.cos(2*np.pi*50*time) - voltageIm*np.sin(2*np.pi*50*time)
	fig, ax1 = plt.subplots()
	ax1.plot(time, voltageEmt, 'b-', time, voltage, 'r-')
	#plt.yticks(np.arange(-10, 10, 1.0))
	ax1.set_xlabel('time [s]')
	ax1.set_ylabel('mag [V] or [A]')
	ax1.grid(True)
	plt.show()
	
def plotInterpolatedNodeResults(filename, node):
	node = node - 1
	df = pd.read_csv(filename, header=None)
	print(df.shape)

	if (df.shape[1] - 1) / 2 < node or node < 0:
		print('Node not available')
		exit()
	
	time = np.array(df.ix[:,0])
	voltageRe = np.array(df.ix[:,node + 1])
	voltageIm = np.array(df.ix[:, int((df.shape[1] - 1) / 2 + node + 1)])
	
	interpTime = np.arange(df.ix[0, 0], df.ix[ df.shape[0] - 1, 0], 0.00005)
	fVoltageRe = interp1d(time, voltageRe)
	fVoltageIm = interp1d(time, voltageIm)
	

	interpVoltageRe = fVoltageRe(interpTime)
	interpVoltageIm = fVoltageIm(interpTime)
	
	voltageMeas = np.sqrt(voltageRe**2+voltageIm**2)
	voltage = np.sqrt(interpVoltageRe**2+interpVoltageIm**2)
	voltageEmt = interpVoltageRe*np.cos(2*np.pi*50*interpTime) - interpVoltageIm*np.sin(2*np.pi*50*interpTime)
	fig, ax1 = plt.subplots()
	ax1.plot(interpTime, voltageEmt, 'b-')
	ax1.plot(time, voltageMeas, 'r-')
	ax1.set_xlabel('time [s]')
	ax1.set_ylabel('mag [V] or [A]')
	ax1.grid(True)
	plt.show()

def plotResultsInterfacedInductor(filename, node):
	node = node - 1
	df = pd.read_csv(filename, header=None)
	print(df.shape)

	if (df.shape[1] - 1) / 2 < node or node < 0:
		print('Voltage not available')
		exit()

	time = np.array(df.ix[:,0])
	voltageRe = np.array(df.ix[:,node + 1])
	voltageIm = np.array(df.ix[:, int((df.shape[1] - 1) / 2 + node + 1)])

	voltage = np.sqrt(voltageRe**2+voltageIm**2)
	voltageEmt = voltageRe*np.cos(2*np.pi*50*time) - voltageIm*np.sin(2*np.pi*50*time)
	fig, ax1 = plt.subplots()
	ax1.plot(time, voltageEmt, 'b-', time, voltage, 'r-')
	plt.yticks(np.arange(-10, 10, 1.0))
	ax1.set_xlabel('time [s]')
	ax1.set_ylabel('voltage [V]')
	ax1.grid(True)
	plt.show()

def plotResultsSynGenUnitTest(filename, node1, node2, node3):
	node1 = node1 - 1
	node2 = node2 - 1
	node3 = node3 - 1
	df = pd.read_csv(filename, header=None)
	print(df.shape)

	if (df.shape[1] - 1) / 2 < node1 or node1 < 0 or \
	(df.shape[1] - 1) / 2 < node2 or node2 < 0 or \
	(df.shape[1] - 1) / 2 < node3 or node3 < 0:
		print('Voltage not available')
		exit()

	time = np.array(df.ix[:,0])
	mag1 = np.array(df.ix[:,node1 + 1])
	mag2 = np.array(df.ix[:,node2 + 1])
	mag3 = np.array(df.ix[:,node3 + 1])

	fig, ax1 = plt.subplots()
	ax1.plot(time, mag1, 'b-', time, mag2, 'r-', time, mag3, 'g-')
	#ax1.plot(time, voltageEmt, 'b-', time, voltage, 'r-')
	#plt.yticks(np.arange(-10, 10, 1.0))
	ax1.set_xlabel('time [s]')
	ax1.set_ylabel('Magnitude')
	ax1.grid(True)
	plt.show()

def plotResultsSynGenUnitTestVar(filename, varNum):

	df = pd.read_csv(filename, header=None)
	print(df.shape)

	if (df.shape[1]) < varNum or varNum < 0:
		print('Variable not available')
		exit()

	time = np.array(df.ix[:,0])
	mag = np.array(df.ix[:,varNum])

	fig, ax1 = plt.subplots()
	ax1.plot(time, mag, 'b-')
	#plt.yticks(np.arange(-10, 10, 1.0))
	ax1.set_xlabel('time [s]')
	ax1.set_ylabel('Magnitude')
	ax1.grid(True)
	plt.show()
