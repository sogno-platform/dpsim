import os
import urllib.request
import glob

import dpsimpy
import dpsimpyvillas

from multiprocessing import Process, Queue

import villas.dataprocessing.readtools as rt
import villas.dataprocessing.plottools as pt

if __name__ == '__main__':
    sim_name = "ShmemDistributed0"
    time_step = 0.001

    n1 = dpsimpy.dp.SimNode('n1')
    n2 = dpsimpy.dp.SimNode('n2')
    n3 = dpsimpy.dp.SimNode('n3')

    evs = dpsimpy.dp.ph1.VoltageSource('v_t')
    vs = dpsimpy.dp.ph1.VoltageSourceNorton('v_s')
    l1 = dpsimpy.dp.ph1.Inductor('l_1')
    r1 = dpsimpy.dp.ph1.Resistor('r_1')

    n4 = dpsimpy.dp.SimNode('n4')
    n5 = dpsimpy.dp.SimNode('n5')

    ecs = dpsimpy.dp.ph1.CurrentSource('v_s')
    r2a = dpsimpy.dp.ph1.Resistor('r_2')
    r2b = dpsimpy.dp.ph1.Resistor('r_2')
    sw = dpsimpy.dp.ph1.Switch('sw')

    evs.connect([dpsimpy.dp.SimNode.gnd, n3])
    vs.connect([dpsimpy.dp.SimNode.gnd, n1])
    l1.connect([n1, n2])
    r1.connect([n2, n3])

    evs.set_parameters(complex(0, 0))
    vs.set_parameters(complex(10000, 0), 1)
    l1.set_parameters(0.1)
    r1.set_parameters(1)

    ecs.connect([dpsimpy.dp.SimNode.gnd, n4])
    r2a.connect([dpsimpy.dp.SimNode.gnd, n4])
    sw.connect([n4, n5])
    r2b.connect([dpsimpy.dp.SimNode.gnd, n5])

    ecs.set_parameters(complex(0, 0))
    r2a.set_parameters(10)
    r2b.set_parameters(8)
    sw.set_parameters(open_resistance=1e9, closed_resistance=0.1, closed=False)

    sys = dpsimpy.SystemTopology(50, [n1, n2, n3, n4, n5], [evs, vs, l1, r1, ecs, r2a, sw, r2b])

    dpsimpy.Logger.set_log_dir('logs/' + sim_name)

    logger = dpsimpy.Logger(sim_name)
    logger.log_attribute('v1', 'v', n1)
    logger.log_attribute('v2', 'v', n2)
    logger.log_attribute('v3', 'v', n3)
    logger.log_attribute('v4', 'v', n4)
    logger.log_attribute('v5', 'v', n5)

    sim = dpsimpy.RealTimeSimulation(sim_name)
    sim.set_system(sys)
    sim.set_time_step(time_step)
    sim.set_final_time(20)

    sim.import_attr('v_t', 'V_ref', 0)
    sim.export_attr('v_t', 'i_intf', 0, 0, 0)
    sim.import_attr('v_s', 'I_ref', 0)
    sim.export_attr('v_s', 'v_intf', 0, 0, 0)

    evt = dpsimpy.event.SwitchEvent(10, sw, True)
    sim.add_event(evt)

    sim.add_logger(logger)
    sim.run(1)

    work_dir = 'logs/ShmemDistributed/'
    log_name = 'ShmemDistributed'

    ts_dpsim_0 = rt.read_timeseries_dpsim(work_dir + log_name + '.csv')

    ts_dpsim_0['v1'].label = 'v1'
    ts_dpsim_0['v2'].label = 'v2'
    ts_dpsim_0['v3'].label = 'v3'
    ts_dpsim_0['v4'].label = 'v4'
    ts_dpsim_0['v5'].label = 'v5'
    pt.plot_timeseries(1, ts_dpsim_0['v1'])
    pt.plot_timeseries(1, ts_dpsim_0['v2'])
    pt.plot_timeseries(1, ts_dpsim_0['v3'])
    pt.plot_timeseries(2, ts_dpsim_0['v4'])
    pt.plot_timeseries(2, ts_dpsim_0['v5'])
