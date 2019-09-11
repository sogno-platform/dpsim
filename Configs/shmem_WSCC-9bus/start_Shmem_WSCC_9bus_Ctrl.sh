#!/bin/bash

set -x

_stop() {
	echo "Caught SIGTSTP signal!"
	kill -TSTP ${CHILDS} 2>/dev/null
}

_cont() {
	echo "Caught SIGCONT signal!"
	kill -CONT ${CHILDS} 2>/dev/null
}

_term() {
	echo "Caught SIGTERM signal!"
	kill -TERM ${VN} ${CHILDS} 2>/dev/null
}

_kill() {
	echo "Caught SIGKILL signal!"
	kill -KILL ${VN} ${CHILDS} 2>/dev/null
}

trap _stop SIGTSTP
trap _cont SIGCONT
trap _term SIGTERM
trap _kill SIGKILL

CHILDS=""

# Start time
TIME=$(date -d "+20 seconds" +%Y%m%dT%H%M%S) #-Iseconds
echo "Start simulation at: $TIME"

# Simulation params
OPTS="--timestep 0.001 --duration 3600 --start-in 5 Examples/CIM/WSCC-09_RX/*.xml"
echo "Simulation params: $OPTS"

CPS_LOG_PREFIX="[Sys ] " \
build/Examples/Cxx/Shmem_WSCC-9bus_Ctrl $OPTS & P1=$!

CHILDS=$P1

sleep 2

if false; then
	VILLAS_LOG_PREFIX="[Pipe] " \
	villas-pipe Configs/shmem_WSCC-9bus/Shmem_WSCC-9bus_Ctrl.conf dpsim1
else
	VILLAS_LOG_PREFIX="[Node] " \
	villas-node Configs/shmem_WSCC-9bus/Shmem_WSCC-9bus_Ctrl.conf & VN=$!
fi

# Wait until all child processed finished
while (( $(ps --no-headers -o pid --ppid=$$ | wc -w) > 1 )); do
	wait
done
