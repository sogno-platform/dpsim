import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from scipy.interpolate import interp1d


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
	
	print(time)
	print(voltageRe)
	interpTime = np.arange(df.ix[0, 0], df.ix[ df.shape[0] - 1, 0], 0.00005)
	fVoltageRe = interp1d(time, voltageRe)
	fVoltageIm = interp1d(time, voltageIm)
	

	interpVoltageRe = fVoltageRe(interpTime)
	interpVoltageIm = fVoltageIm(interpTime)
	
	voltageMeas = np.sqrt(voltageRe**2+voltageIm**2)
	voltage = np.sqrt(interpVoltageRe**2+interpVoltageIm**2)
	voltageEmt = interpVoltageRe*np.cos(2*np.pi*50*interpTime) - interpVoltageIm*np.sin(2*np.pi*50*interpTime)
	fig, ax1 = plt.subplots()
	ax1.plot(interpTime, voltageEmt, 'b-', interpTime, voltage, 'r-')
	ax1.plot(time, voltageMeas, 'g-')
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