#!/bin/bash

function join_by { local IFS="$1"; shift; echo "$*"; }

V2G_BUSSES="68 81 27 35 85 19 63 76 78"

IN=""
OUT=""

for BUS in ${V2G_BUSSES}; do
	IN="${IN} \"v2g_bus_${BUS}.data[0]\""
	OUT="${OUT} \"v2g_bus_${BUS}\""
done

INPUTS=$(join_by , ${IN})
OUTPUTS=$(join_by , ${OUT})

cat << EOF
logging = {
	level = 3
}

stats = 5

nodes = {
EOF

for BUS in ${V2G_BUSSES}; do
cat << EOF
	v2g_bus_${BUS} = {
		type = "mqtt",
		format = "json.reserve",

		builtin = false,

		host = "acs-os-villas",

		out = {
			publish = "reserve_esb_lv",

			signals = (
				{ name = "V", unit = "Volts", type = "float" }
			)
		},
		in = {
			subscribe = "reserve_esb_lv/Q_Set",

			signals = (
				{ name = "Q", unit = "Var", type = "float" }
			)
		}
	},
EOF
done

cat << EOF
	signal = {
		type = "signal",

		signal = "sine",
		offset = 230,
		amplitude = 0.5,
		frequency = 0.1,
		rate = 20,
		values = 1
	},
	opal = {
		type = "socket",
		layer = "udp",

		format = "villas.binary",

		in = {
			address = "134.130.169.82:12000",
			signals = {
				count = 18
				type = "float"
			}
		},
		out = {
			address = "134.130.169.80:12000"
		}
	},
	broker1 = {
		type = "mqtt"
		format = "villas.binary",

		username = "guest",
	        password = "guest",
		host = "acs-os-villas",
		port = 1883,

		out = {
			publish = "opal-villas"
		},
		in = {
			subscribe = "villas-opal"

			signals = {
				count = 4
				type = "float"
			}

			hooks = (
				{ type = "stats" }
			)
		}
	}
}

paths = (
	# Simulation -> VILLASweb
	{
		enabled = true,
		in = "opal",
		out = "broker1"

		hooks = (
			{ type = "limit_rate", rate = 50 }
		)
	},
	# VILLASweb -> Simulation
	{
		in = "broker1.data[0-3]",
		out = "opal",
	        hooks = (
                        { type = "print" }
                )
	},
	# Simulation -> AVM
	{
		enabled = false,

		in = "signal.data[0]",
		out = [ ${OUTPUTS} ],

		hooks = (
			{ type = "print" }
		)
	},

	# AVM -> Simulation
	{
		enabled = false,

		mode = "any",
		in = [ ${INPUTS} ],

		hooks = (
			{ type = "print" }
		)
	}
)

EOF
