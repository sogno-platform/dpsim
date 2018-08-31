import dpsim
import pytest
import asyncio
import time
import logging

def my_callback(event, sim, myvar):
    assert myvar == 1337

    if event == 1:
        print("Simulation started")
    if event == 2:
        print("Simulation stopped")
    if event == 3:
        print("Simulation finished")
    if event == 4:
        print("Simulation overrun")
    if event == 5:
        print("Simulation paused")
    if event == 6:
        print("Simulation resumed")

    if event == 3:
        el = asyncio.get_event_loop()
        el.stop()


def test_async():
    el = asyncio.get_event_loop()

    # Nodes
    gnd = dpsim.dp.Node.GND()
    n1  = dpsim.dp.Node("n1")

    # Components
    v1 = dpsim.dp.ph1.VoltageSource("v_1", [gnd, n1], voltage_ref=10)
    r1 = dpsim.dp.ph1.Resistor("r_1", [n1, gnd], resistance=1)

    system = dpsim.SystemTopology(20, [gnd, n1], [v1, r1])

    sim = dpsim.Simulation('async', system, duration=20, timestep=0.0005)

    # Start in two seconds!
    sim.start(when=time.time() + 2)
    sim.show_progressbar()

    sim.add_callback(my_callback, sim, 1337)

    # Pause the simulation after 5 sec
    el.call_at(el.time() + 5, sim.pause)

    # Resume after 7 sec
    el.call_at(el.time() + 10, sim.start)

    el.run_forever()
