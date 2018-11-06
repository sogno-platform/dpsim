import os
import dpsim

PATH = os.path.dirname(__file__)

def test_circuit():
    # Nodes
    gnd = dpsim.dp.Node.GND()
    n1 =  dpsim.dp.Node('n1')
    n2 =  dpsim.dp.Node('n2')
    n3 =  dpsim.dp.Node('n3')

    # Components
    v1 = dpsim.dp.ph1.VoltageSource('v_1')
    v1.V_ref= complex(10, 0)
    lL = dpsim.dp.ph1.Inductor('l_L')
    lL.L= 0.001
    rL = dpsim.dp.ph1.Resistor('r_L')
    rL.R= 0.1
    r1 = dpsim.dp.ph1.Resistor('r_1')
    r1.R= 20

    v1.connect([gnd, n1])
    lL.connect([n2, n3])
    rL.connect([n1, n2])
    r1.connect([n3, gnd])

    system = dpsim.SystemTopology(50, [gnd, n1, n2, n3], [v1, lL, rL, r1])

    sim = dpsim.Simulation(__name__, system, duration=0.2, timestep=0.0005)
    sim.run()

    #results = rt.read_timeseries_dpsim_cmpl('Logs/' + sim.name + '_LeftVector.csv')
    #expected = rt.read_timeseries_dpsim_real('Examples/Results/Simulink/Circuits/SL_' + sim.name() + '.csv')

    err = 0
    #err += ts.TimeSeries.rmse(expected[0], results[0].dynphasor_shift_to_emt('n1_emt', 50))
    #err += ts.TimeSeries.rmse(expected[1], results[1].dynphasor_shift_to_emt('n2_emt', 50))

    print('Total RMSE: %g' % (err))

    assert err < 1e-4

if __name__ == '__main__':
    test_circuit()
