#!/bin/bash

CPS_LOG_PREFIX="[Left ] " build/dpsim-villas/examples/cxx/SocketCosimExample 0 & P1=$!
CPS_LOG_PREFIX="[Right] " build/dpsim-villas/examples/cxx/SocketCosimExample 1 & P2=$!
    
for job in $P1 $P2; do
    wait $job || exit 1
done
