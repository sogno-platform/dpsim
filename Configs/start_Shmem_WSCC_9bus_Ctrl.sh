set -x

# Start time
TIME=$(date -d "+10 seconds" +%Y%m%dT%H%M%S) #-Iseconds
echo "Start simulation at: $TIME"

# Simulation params
OPTS="--timestep 0.001 --duration $((24*60*60)) --system-freq 60 --start-at $TIME"
echo "Simualtion params: $OPTS"

CPS_LOG_PREFIX="[Sys ] " \
build/Examples/Cxx/Shmem_WSCC-9bus_Ctrl $OPTS & P1=$!

sleep 2

if false; then
	VILLAS_LOG_PREFIX="[Pipe] " \
	villas-pipe Configs/villas-shmem.conf shmem
else
	VILLAS_LOG_PREFIX="[Node] " \
	villas-node /projects/reserve/Shmem_WSCC-9bus_Ctrl.conf
fi

for job in $P1; do
    wait $job || exit 1
done

exit 0
