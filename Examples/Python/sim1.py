#!/usr/bin/python3

import dpsim
import os

def doExample(basename, comps, timestep, duration):
    sim = dpsim.Simulation(comps, timestep=timestep, duration=duration,
        log="Logs/Log_"+basename+".log", llog="Logs/LeftVectorLog_"+basename+".csv",
        rlog="Logs/RightVectorLog_"+basename+".csv")
    sim.start()
    sim.wait()

def simExample1(timestep=1e-3):
    comps = [
        dpsim.VoltSourceRes("v_in", 1, 0, 10, 1),
        dpsim.Inductor("l_1", 1, 2, 0.02),
        dpsim.Inductor("l_2", 2, 0, 0.1),
        dpsim.Inductor("l_3", 2, 3, 0.05),
        dpsim.Inductor("r_2", 3, 0, 2)
    ]

    basename = "SimulationExample1_" + str(timestep)
    doExample(basename, comps, timestep, 0.3)

if __name__ == '__main__':
    os.makedirs("Logs", exist_ok=True)
    simExample1()
