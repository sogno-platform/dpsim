TIME=$(date -d "+10 minutes" -Iseconds)
echo "Start simulation at: $TIME"

build/Examples/Cxx/Shmem_WSCC-9bus_CtrlDist --time $TIME --scenario 0 & P1=$!
build/Examples/Cxx/Shmem_WSCC-9bus_CtrlDist --time $TIME --scenario 1 & P2=$!

villas-pipe Configs/villas-shmem.conf shmem

for job in $P1 $P2; do
    wait $job || exit 1
done

exit 0