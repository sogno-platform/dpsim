#!/bin/bash

INPUT=$(mktemp)
OUTPUT="logs/Shmem_Example/villas-pipe-out.log"
CONFIG=$(mktemp)

cat > ${CONFIG} <<EOF
nodes = {
    dpsim = {
        type = "shmem"

        in = {
			name = "/villas1-in" # Name of shared memory segment for receiving side

            signals = (
                { name = "i_intf", type = "complex" }
            )
		}
		out = {
			name = "/villas1-out" # Name of shared memory segment for sending side

            signals = (
                { name = "V_ref", type = "complex" }
            )
		}

        exec = [ "build/dpsim-villas/examples/cxx/ShmemExample" ]
    }

    input = {
        type = "file"

        uri = "${INPUT}"

        in = {
            signals = (
                { name = "V_ref", type = "complex" }
            )
        }
    }

    output = {
        type = "file"

        uri = "${OUTPUT}"
    }
}

paths = (
    {
        in = "input"
        out = "dpsim"
    },
    {
        in = "dpsim"
        out = "output"
    }
)
EOF

for i in $(seq 0 499); do
    echo "0.$(( 1000000 * $i  ))($i) 5+0i" >> ${INPUT}
done

for i in $(seq 500 1000); do
    echo "0.$(( 1000000 * $i  ))($i) 10+0i" >> ${INPUT}
done

villas-node ${CONFIG}

rm ${INPUT} ${CONFIG}
