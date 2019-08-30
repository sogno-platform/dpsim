#!/bin/bash

INPUT=$(mktemp)
OUTPUT=$(mktemp)
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

        exec = [ "build/Examples/Cxx/ShmemExample" ]
    }

    input = {
        type = "file"

        uri = "${INPUT}"
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
    echo "0.$(( 1000000 * $i  ))($i) 5" >> ${INPUT}
done

for i in $(seq 500 1000); do
    echo "0.$(( 1000000 * $i  ))($i) 10" >> ${INPUT}
done

villas-node ${CONFIG}

cat ${OUTPUT}

rm ${INPUT} ${OUTPUT} ${CONFIG}
