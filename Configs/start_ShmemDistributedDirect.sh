build/Examples/Cxx/ShmemDistributedDirect 0 & P1=$!
    build/Examples/Cxx/ShmemDistributedDirect 1 & P2=$!
    
    for job in $P1 $P2; do
        wait $job || exit 1
    done
    
    exit 0