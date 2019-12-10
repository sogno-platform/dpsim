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
TIME=$(date -d "+10 seconds" +%Y%m%dT%H%M%S) #-Iseconds
echo "Start simulation at: $TIME"

# Simulation params
OPTS="--timestep 1 --duration $((300)) --system-freq 50 --start-at $TIME --solver-domain SP --solver-type NRP"
echo "Simulation params: $OPTS"

CPS_LOG_PREFIX="[Sys ] " \
build/Examples/Cxx/Shmem_CIGRE_MV_PowerFlowTest_LoadProfiles $OPTS & P1=$!

CHILDS=$P1

sleep 2

if true; then
	VILLAS_LOG_PREFIX="[Pipe] " \
	#villas-pipe Configs/Shmem_CIGRE_MV.conf dpsim1
	villas-node Configs/Shmem_CIGRE_MV.conf
else
	VILLAS_LOG_PREFIX="[Node] " \
	villas-node /projects/reserve/Shmem_CIGRE_MV.conf & VN=$!
fi

# Wait until all child processed finished
while (( $(ps --no-headers -o pid --ppid=$$ | wc -w) > 1 )); do
	wait
done
