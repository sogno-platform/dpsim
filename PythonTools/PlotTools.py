import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

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
