#!/bin/bash
for (( i = 0; i < 20 ; i++ ))
do
    for (( j = 0 ; j <= 12; j = j+2 ))
    do
        for (( k = 0 ; k <= $i; k++ ))
        do
            build/Examples/Cxx/WSCC_9bus_mult_diakoptics -ocopies=$i -othreads=$j -osplits=$k -oseq=0
        done
    done
done