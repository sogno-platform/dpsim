import dpsim
import pytest
import asyncio
import datetime as dt
import logging

from dpsim.Event import Event

def my_callback(event, sim, myvar):
    assert myvar == 1337

    print("Received Event: %s" % event)

    if event in [ Event.done, Event.stopped, Event.stopped, Event.failed, Event.overrun ]:
        el = asyncio.get_event_loop()
        el.stop()

def test_async():
    el = asyncio.get_event_loop()

    # Nodes
    gnd = dpsim.dp.Node.GND()
    n1  = dpsim.dp.Node("n1")

    # Components
    v1 = dpsim.dp.ph1.VoltageSource("v_1", [gnd, n1], v_ref=10)
    r1 = dpsim.dp.ph1.Resistor("r_1", [n1, gnd], resistance=1)

    system = dpsim.SystemTopology(50, [gnd, n1], [v1, r1])

    start = dt.datetime.now() + dt.timedelta(seconds=4)

    sim = dpsim.RealTimeSimulation(__name__, system, duration=10, timestep=0.0005)#, start_time=start)

    # Start in two seconds!
    sim.start()
    sim.show_progressbar()

    sim.add_callback(my_callback, sim, 1337)

    # Pause the simulation after 5 sec
    el.call_at(el.time() + 5, sim.pause)

    # Resume after 7 sec
    el.call_at(el.time() + 10, sim.start)

    el.run_forever()

if __name__ == "__main__":
    test_async()
