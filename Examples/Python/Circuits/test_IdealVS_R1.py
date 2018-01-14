import os
import dpsim
import dpsim.components.dp as dp
import dataprocessing.readtools as rt
import dataprocessing.timeseries as ts

PATH = os.path.dirname(__file__)

def test_IdealVS_R1():
    sim = dpsim.Simulation('IdealVS_R1',
            [
                dp.VoltageSourceIdeal("v_in", 1, 0, 10),
                dp.Resistor("r_1", 0, -1, 1),
                dp.Resistor("r_2", 1, -1, 1),
                dp.Resistor("r_3", 1, -1, 1)
            ],
            duration=0.3
    )

    sim.run()

    results = rt.read_timeseries_dpsim_cmpl('Logs/' + sim.name() + '_LeftVector.csv')
    expected = rt.read_timeseries_dpsim_real(PATH + '/../../Results/Simulink/Circuits/SL_' + sim.name() + '.csv')

    err = 0
    err += ts.TimeSeries.rmse(expected[0], results[0].dynphasor_shift_to_emt('n1_emt', 50))
    err += ts.TimeSeries.rmse(expected[1], results[1].dynphasor_shift_to_emt('n2_emt', 50))

    print("Total RMSE: %g" % (err))

    assert err < 1e-6
