#!/bin/bash
for (( j = 0; j <= 12; j = j+2 ))
do
    for (( k = 1; k <= 10; k++ ))
    do
        sudo taskset --all-tasks --cpu-list 12-23 chrt --fifo 99 build/Examples/Cxx/DP_Inverter_Grid_Parallel_FreqSplit -othreads=$j -oseq=$k
    done
done

for (( j = 0; j <= 12; j = j+2 ))
do
    for (( k = 1; k <= 10; k++ ))
    do
        sudo taskset --all-tasks --cpu-list 12-23 chrt --fifo 99 build/Examples/Cxx/DP_Inverter_Grid_Parallel -othreads=$j -oseq=$k
    done
done