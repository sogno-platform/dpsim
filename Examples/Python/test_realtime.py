import dpsim
import datetime as dt

from dpsim.Event import Event

def test_realtime():
    # Nodes
    gnd = dpsim.dp.Node.GND()
    n1  = dpsim.dp.Node('n1')

    # Components
    v1 = dpsim.dp.ph1.VoltageSource('v_1', [gnd, n1], V_ref=10)
    r1 = dpsim.dp.ph1.Resistor('r_1', [n1, gnd], R=1)

    system = dpsim.SystemTopology(50, [gnd, n1], [v1, r1])

    start = dt.datetime.now() + dt.timedelta(seconds=4)

    print(repr(start))

    sim = dpsim.RealTimeSimulation(__name__, system, duration=10, timestep=0.001, start_time=start)
    sim.show_progressbar()

    sim.run(pbar=True)

if __name__ == '__main__':
    test_realtime()
