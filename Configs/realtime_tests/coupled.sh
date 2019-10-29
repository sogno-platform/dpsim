#!/bin/bash
for (( i = 0; i < 20; i++ ))
do
    for (( j = 0; j <= 12; j = j+2 ))
    do
        for (( k = 1; k <= 10; k++ ))
	    do
            sudo taskset --all-tasks --cpu-list 12-23 chrt --fifo 99 build/Examples/Cxx/WSCC_9bus_mult_coupled -ocopies=$i -othreads=$j -oseq=$k
        done
    done
done
