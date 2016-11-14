import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

voltageNum = 0
df = pd.read_csv('..\DPSolver\VisualStudio\DPSolverWin10\DPSolverWin10\data.csv', header=None)
print(df.shape)

if (df.shape[1] - 1) / 2 - 1 < voltageNum:
    print('Voltage not available')
    exit()
    
time = np.array(df.ix[:,0])
voltageRe = np.array(df.ix[:,voltageNum + 1])
voltageIm = np.array(df.ix[:, int((df.shape[1] - 1) / 2 + voltageNum + 1)])

voltage = np.sqrt(voltageRe**2+voltageIm**2)
voltageEmt = voltageRe*np.cos(2*3.14*50*time) + voltageIm*np.sin(2*3.14*50*time)
fig, ax1 = plt.subplots()
ax1.plot(time, voltageEmt, 'b-', time, voltage, 'r-')
ax1.set_xlabel('time [s]')
ax1.set_ylabel('voltage [V]')
plt.show()
