import dpsim
import pytest
import asyncio
import time
import logging

def my_callback(event, sim, myvar):
    assert myvar == 1337

    print("Received Event: %d" % event)

    if event == EventChannel.Event.stopped:
        print("Simulation stopped")
    if event == EventChannel.Event.starting:
        print("Simulation starting")
    if event == EventChannel.Event.running:
        print("Simulation running")
    if event == EventChannel.Event.pausing:
        print("Simulation pausing")
    if event == EventChannel.Event.paused:
        print("Simulation paused")
    if event == EventChannel.Event.resuming:
        print("Simulation resuming")
    if event == EventChannel.Event.stopping:
        print("Simulation stopping")
    if event == EventChannel.Event.failed:
        print("Simulation failed")
    if event == EventChannel.Event.overrun:
        print("Simulation overrun")
    if event == EventChannel.Event.done:
        print("Simulation done")

    if event in [ EventChannel.Event.stopped, EventChannel.Event.stopped, EventChannel.Event.failed, EventChannel.Event.overrun ]:
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

    sim = dpsim.RealTimeSimulation('async', system, duration=20, timestep=0.0005)

    # Start in two seconds!
    sim.start(when=time.time() + 2)
    sim.show_progressbar()

    sim.add_callback(my_callback, sim, 1337)

    # Pause the simulation after 5 sec
    el.call_at(el.time() + 5, sim.pause)

    # Resume after 7 sec
    el.call_at(el.time() + 10, sim.start)

    el.run_forever()
