#!/bin/bash
for (( i = 0; i < 20 ; i++ ))
do
    for (( j = 0 ; j <= 12; j = j+2 ))
    do
         build/Examples/Cxx/WSCC_9bus_mult_decoupled -ocopies=$i -othreads=$j -oseq=0
    done
done