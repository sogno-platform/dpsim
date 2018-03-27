import os
import dataprocessing.readtools as rt
import dataprocessing.timeseries as ts
import subprocess

PATH = os.path.dirname(__file__)

def test_IdealVS_R_1_cpp():
    name = 'DP_IdealVS_R_1'
    frequency = 50
    subprocess.run(PATH + "/../../../build/Examples/Cxx/" + name, shell=True, check=True)
    results = rt.read_timeseries_dpsim_cmpl('Logs/' + name + '_LeftVector.csv')
    expected = rt.read_timeseries_simulink(PATH + '/../../Results/Simulink/Circuits/SL_' + name + '.csv')

    err = 0
    err += ts.TimeSeries.rmse(expected[0], results[0].dynphasor_shift_to_emt('n1_emt', frequency))
    err += ts.TimeSeries.rmse(expected[1], results[1].dynphasor_shift_to_emt('n2_emt', frequency))

    print("Total RMSE: %g" % (err))

    assert err < 1e-3