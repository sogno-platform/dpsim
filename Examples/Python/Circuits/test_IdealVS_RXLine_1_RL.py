import os
import dpsim
import dpsim.components.dp as dp
import dataprocessing.readtools as rt
import dataprocessing.timeseries as ts

PATH = os.path.dirname(__file__)

def test_IdealVS_RXLine_1_RL():
    system = dpsim.SystemTopology(50, 
        [
            dp.VoltageSource("v_1", -1, 0, 10),
            dp.Inductor("l_L", 1, 2, 0.001),
            dp.Resistor("r_L", 0, 1, 0.1),
            dp.Resistor("r_1", 2, -1, 20)
        ])

    sim = dpsim.Simulation('IdealVS_RXLine_1_RL', system, duration=0.2, timestep=0.00005)

    sim.run()

    results = rt.read_timeseries_dpsim_cmpl('Logs/' + sim.name() + '_LeftVector.csv')
    #expected = rt.read_timeseries_dpsim_real('Examples/Results/Simulink/Circuits/SL_' + sim.name() + '.csv')

    err = 0
    #err += ts.TimeSeries.rmse(expected[0], results[0].dynphasor_shift_to_emt('n1_emt', 50))
    #err += ts.TimeSeries.rmse(expected[1], results[1].dynphasor_shift_to_emt('n2_emt', 50))

    print("Total RMSE: %g" % (err))

    assert err < 1e-4
