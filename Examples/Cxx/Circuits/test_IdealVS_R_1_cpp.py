import os
import villas.dataprocessing.readtools as rt
import villas.dataprocessing.timeseries as ts
import subprocess

PATH = os.path.dirname(__file__)

def test_IdealVS_R_1_cpp():
    name = 'VS_R1'
    frequency = 50
    path = PATH + "/../../../build/Examples/Cxx/DP_" + name
    if os.name == 'nt':
        path = PATH + "/../../../build/Examples/Cxx/Debug/DP_" + name + '.exe'

    subprocess.run(path, shell=True, check=True)
    results = rt.read_timeseries_dpsim('logs/DP_' + name + '_LeftVector.csv')
    #expected = rt.read_timeseries_dpsim_real(PATH + '/../../Results/Simulink/Circuits/SL_' + name + '.csv')

    err = 0
    #err += ts.TimeSeries.rmse(expected[0], results[0].dynphasor_shift_to_emt('n1_emt', frequency))

    print("Total RMSE: %g" % (err))

    assert err < 1e-3
