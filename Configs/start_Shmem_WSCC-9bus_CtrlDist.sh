TIME=$(date -d "+10 seconds" +%Y%m%dT%H%M%S) #-Iseconds
echo "Start simulation at: $TIME"

build/Examples/Cxx/Shmem_WSCC-9bus_CtrlDist --start-at $TIME --scenario 0 & P1=$!
sleep 2
build/Examples/Cxx/Shmem_WSCC-9bus_CtrlDist --start-at $TIME --scenario 1 & P2=$!

sleep 1
villas-pipe Configs/villas-shmem.conf shmem

for job in $P1 $P2; do
    wait $job || exit 1
done

exit 0