#!/bin/bash
for (( i = 1; i <= 20; i++ ))
do
    for (( j = 0; j <= 12; j = j+2 ))
    do
        for (( k = 1; k <= 50; k++ ))
	    do
            sudo taskset --all-tasks --cpu-list 12-23 chrt --fifo 99 build/Examples/Cxx/DP_Multimachine_DQ_Parallel -ogen=$i -othreads=$j -oseq=$k
        done
    done
done
