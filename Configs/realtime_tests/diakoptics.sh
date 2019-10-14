#!/bin/bash
for (( m = 1 ; m <= 10; m++ ))
do
    for (( i = 0; i < 20 ; i++ ))
    do
        for (( j = 0 ; j <= 12; j = j+2 ))
        do
            for (( k = 0 ; k <= $i+1; k++ ))
            do
                sudo taskset --all-tasks --cpu-list 12-23 chrt --fifo 99 build/Examples/Cxx/WSCC_9bus_mult_diakoptics -ocopies=$i -othreads=$j -osplits=$k -oseq=$m
            done
        done
    done
done