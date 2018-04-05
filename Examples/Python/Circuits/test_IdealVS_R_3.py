import os
import dpsim
import dpsim.components.dp as dp
import dataprocessing.readtools as rt
import dataprocessing.timeseries as ts

PATH = os.path.dirname(__file__)

def test_IdealVS_R_3():
    sim = dpsim.Simulation('IdealVS_R_3',
            [
                dp.VoltageSource("v_1", -1, 0, 10),
                dp.Resistor("r_1", 0, 1, 1),
                dp.Resistor("r_2", 1, -1, 1),
                dp.Resistor("r_3", 1, 2, 1),
                dp.Resistor("r_4", 2, -1, 1),
                dp.Resistor("r_5", 2, 3, 1),
                dp.VoltageSource("v_2", 3, -1, 20)
            ],
            duration=0.2,
            timestep=0.00005
    )

    sim.run()

    results = rt.read_timeseries_dpsim_cmpl('Logs/' + sim.name() + '_LeftVector.csv')
    #expected = rt.read_timeseries_dpsim_real('Examples/Results/Simulink/Circuits/SL_' + sim.name() + '.csv')

    err = 0
    #err += ts.TimeSeries.rmse(expected[0], results[0].dynphasor_shift_to_emt('n1_emt', 50))
    #err += ts.TimeSeries.rmse(expected[1], results[1].dynphasor_shift_to_emt('n2_emt', 50))
    #err += ts.TimeSeries.rmse(expected[2], results[2].dynphasor_shift_to_emt('n3_emt', 50))
    #err += ts.TimeSeries.rmse(expected[3], results[3].dynphasor_shift_to_emt('n4_emt', 50))
    #err = err / 4

    print("Total RMSE: %g" % (err))

    assert err < 1e-4
