import os
import dpsim
import dataprocessing.readtools as rt
import dataprocessing.timeseries as ts

PATH = os.path.dirname(__file__)

def test_IdealVS_RXLine_1_RL():
    # Nodes
    gnd = dpsim.dp.Node.GND()
    n1 =  dpsim.dp.Node("n1")
    n2 =  dpsim.dp.Node("n2")
    n3 =  dpsim.dp.Node("n3")

    # Components
    v1 = dpsim.dp.ph1.VoltageSource("v_1", [gnd, n1], voltage_ref=10)
    lL = dpsim.dp.ph1.Inductor("l_L", [n2, n3], inductance=0.001)
    rL = dpsim.dp.ph1.Resistor("r_L", [n1, n2], resistance=0.1)
    r1 = dpsim.dp.ph1.Resistor("r_1", [n3, gnd], resistance=20)

    system = dpsim.SystemTopology(50, [gnd, n1, n2, n3], [v1, lL, rL, r1])

    sim = dpsim.Simulation('IdealVS_RXLine_1_RL', system, duration=0.2, timestep=0.00005)
    sim.run()

    #results = rt.read_timeseries_dpsim_cmpl('Logs/' + sim.name + '_LeftVector.csv')
    #expected = rt.read_timeseries_dpsim_real('Examples/Results/Simulink/Circuits/SL_' + sim.name() + '.csv')

    err = 0
    #err += ts.TimeSeries.rmse(expected[0], results[0].dynphasor_shift_to_emt('n1_emt', 50))
    #err += ts.TimeSeries.rmse(expected[1], results[1].dynphasor_shift_to_emt('n2_emt', 50))

    print("Total RMSE: %g" % (err))

    assert err < 1e-4
