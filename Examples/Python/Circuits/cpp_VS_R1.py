# Example to show how to run cpp simulations from Python
# and analyse the results using the dataprocessing package.

import os
import villas.dataprocessing.readtools as rt
import villas.dataprocessing.timeseries as ts
import subprocess

# Set path to example
name = 'VS_R1'
PATH = os.path.dirname(__file__)
path = PATH + "/../../build/Examples/Cxx/DP_" + name
# Windows compatibility
if os.name == 'nt':
    path = PATH + "/../../build/Examples/Cxx/Debug/DP_" + name + '.exe'

# Run cpp example
subprocess.run(path, shell=True, check=True)

# Read results and do post-processing
results = rt.read_timeseries_dpsim('logs/DP_' + name + '_LeftVector.csv')
#expected = rt.read_timeseries_dpsim_real(PATH + '/../../Results/Simulink/Circuits/SL_' + name + '.csv')
err = 0
frequency = 50
#err += ts.TimeSeries.rmse(expected[0], results[0].dynphasor_shift_to_emt('n1_emt', frequency))

print("Total RMSE: %g" % (err))
assert err < 1e-3
