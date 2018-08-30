import os
import dpsim
import dataprocessing.readtools as rt
import dataprocessing.timeseries as ts

PATH = os.path.dirname(__file__)

def test_IdealVS_R_2():
    # Nodes
    gnd = dpsim.dp.Node.GND()
    n1 =  dpsim.dp.Node("n1")
    n2 =  dpsim.dp.Node("n2")

    # Components
    vin = dpsim.dp.ph1.VoltageSource("v_in", [n1, n2], voltage_ref=10)
    r1 =  dpsim.dp.ph1.Resistor("r_1", [n1, gnd], resistance=1)
    r2 =  dpsim.dp.ph1.Resistor("r_2", [n2, gnd], resistance=1)
    r3 =  dpsim.dp.ph1.Resistor("r_3", [n2, gnd], resistance=1)

    system = dpsim.SystemTopology(50, [gnd, n1, n2], [vin, r1, r2, r3])

    sim = dpsim.Simulation('IdealVS_R_2', system, duration=0.2, timestep=0.00005)
    sim.run()

    #results = rt.read_timeseries_dpsim_cmpl('Logs/' + sim.name + '_LeftVector.csv')
    #expected = rt.read_timeseries_dpsim_real('Examples/Results/Simulink/Circuits/SL_' + sim.name + '.csv')

    err = 0
    #err += ts.TimeSeries.rmse(expected[0], results[0].dynphasor_shift_to_emt('n1_emt', 50))
    #err += ts.TimeSeries.rmse(expected[1], results[1].dynphasor_shift_to_emt('n2_emt', 50))

    print("Total RMSE: %g" % (err))

    assert err < 1e-4

if __name__ == "__main__":
    test_IdealVS_R_2()
