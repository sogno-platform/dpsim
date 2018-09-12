#!/bin/env python3

import dpsim
import villas.node
import re
from dpsim.Event import Event
import glob
import datetime
import logging

l = logging.getLogger()

l.setLevel(logging.DEBUG)
l.debug('test')

name = 'WSCC-9bus_dyn_switch'
config = './villas-node.conf'
files = glob.glob('./Examples/CIM/WSCC-09_RX_Dyn/*.xml')

# Load system toplogy from CIM files
system = dpsim.load_cim(name, files, frequency=60, log_level=0)

## Switch
sw = dpsim.dp.ph1.Switch("Switch")

sw.resistance_open = 1e9
sw.resistance_closed = 0.1
sw.closed = False

## Load
load = dpsim.dp.ph1.PQLoadCS("Switched Load")

load.v_nom = 230950
load.power_active = 30000000
load.power_reactive = 0

## Short
res = dpsim.dp.ph1.Resistor("Short")
res.resistance = 0.1

## Topology
bus9 = system.nodes["BUS9"]
#bus10 = dpsim.dp.Node("BUS10")
gnd = dpsim.dp.Node.GND()

#system.add_node(bus10)

#system.add_component(res)
#res.connect([bus10, gnd])

#system.add_component(load)
#load.connect([ bus10 ])

system.add_component(sw)
sw.connect([ bus9, gnd ])

# Create simulation
st = datetime.datetime.now() + datetime.timedelta(seconds=4)
sim = dpsim.RealTimeSimulation(name, system, start_time=st, timestep=0.001, duration=1000, init_steady_state=False, log_level=0, pbar=True)

intf = dpsim.Interface('/dpsim-villas', '/villas-dpsim', queuelen=1024, samplelen=64, polling=False)

j = 0
for i in range(1,10):
	intf.export_attribute(system.nodes['BUS%d' % i], 'voltage', j, mode=9)
	j += 2

for i in range(1,4):
	intf.export_attribute(system.components['GEN%d' % i], 'w_r', j)
	j += 1

# Register controllable load
#intf.import_attribute(load, 'power_active', 0);

for i in range(1,4):
	sim.log_attribute(system.components['GEN%d' % i], 'w_r')

for node in system.nodes:
	sim.log_attribute(system.nodes[node], 'voltage')

sim.add_interface(intf, sync=False)

#for i in range(5,50,5):
#	sim.add_switch_event(i, sw, i % 10 == 0)

sim.add_switch_event(10, sw, True)

villas_config = {
	'http' : {
		'htdocs' : '/usr/share/villas/node/web/'
	},
	'nodes' : {
		'dpsim' : intf.get_villas_config(),
		'web' : {
			'type' : 'websocket',
			'in' : {
				'signals' : [
					{ 'name' : 'slider', 'type' : 'float' },
					{ 'name' : 'buttons', 'type' : 'float' }
				]
			},
			'out' : {
				'signals' : intf.exports
			}
		},
		'file' : {
			'type' : 'file',
			'uri' : 'output.csv',
			'format' : 'csv'
		}
	},
	'paths' : [
		{
			'in' : 'dpsim',
			'out' : [
				'web'
#				'file'
			]
		},
		{
			'in' : 'web',
			'out' : 'dpsim',
			'hooks' : [
				{ 'type' : 'print' }
			]
		}
	]
}

vn = villas.node.Node(villas_config)
vn.start()

sim.start()
sim.wait_until(Event.done)

vn.stop()
