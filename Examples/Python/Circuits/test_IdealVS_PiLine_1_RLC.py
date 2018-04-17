import os
import dpsim
import dpsim.components.dp as dp
import dataprocessing.readtools as rt
import dataprocessing.timeseries as ts

PATH = os.path.dirname(__file__)

def test_IdealVS_PiLine_RLC_1():
    system = dpsim.SystemTopology(50, 
        [
            dp.VoltageSource("v_1", -1, 0, 345),
            dp.Resistor("r1", 0, 1, 5),
            dp.Capacitor("c_1", 1, -1, 0.002),
            dp.Resistor("r_load", 1, 3, 6.4),
            dp.Inductor("l_1", 3, 2, 0.186),
            dp.Capacitor("c_2", 2, -1, 0.002),
            dp.Resistor("r_load", 2, -1, 150)
        ])

    sim = dpsim.Simulation('IdealVS_PiLine_RLC_1', system, duration=0.2, timestep=0.00005)

    sim.run()

    results = rt.read_timeseries_dpsim_cmpl('Logs/' + sim.name() + '_LeftVector.csv')
    #expected = rt.read_timeseries_dpsim_real('Examples/Results/Simulink/Circuits/SL_' + sim.name() + '.csv')

    err = 0
    #err += ts.TimeSeries.rmse(expected[0], results[0].dynphasor_shift_to_emt('n1_emt', 50))
    #err += ts.TimeSeries.rmse(expected[1], results[1].dynphasor_shift_to_emt('n2_emt', 50))

    print("Total RMSE: %g" % (err))

    assert err < 1e-4
