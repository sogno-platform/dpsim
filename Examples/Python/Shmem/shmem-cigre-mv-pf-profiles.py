#build/Examples/Cxx/Shmem_CIGRE_MV_PowerFlowTest_LoadProfiles
#villas-node Configs/shmem_CIGRE_MV_PF/Shmem_CIGRE_MV.conf

import asyncio
import time

async def run(cmd):
    proc = await asyncio.create_subprocess_shell(
        cmd,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE)

    time.sleep(30)
    proc.kill()
    stdout, stderr = await proc.communicate()

    print(f'[{cmd!r} exited with {proc.returncode}]')
    if stdout:
        print(f'[stdout]\n{stdout.decode()}')
    if stderr:
        print(f'[stderr]\n{stderr.decode()}')

async def main():
    
    await asyncio.gather(
        run('villas-node villas-node.conf'),
        run('sleep 1; echo "hello"'))

villas_conf = """
    nodes = {
        broker1 = {
            type = "mqtt"

            format = "json"
            host = "172.17.0.1"

            in = {
                subscribe = "/powerflow-dpsim"
            }
            out = {
                publish = "/dpsim-powerflow"
            }
        }

        dpsim1 = {
            type = "shmem"

            in = {
                name = "/dpsim1-villas",	# Name of shared memory segment for sending side

                hooks = (
                    {
                        type = "stats"
                    }
                ),
                signals = {
                    count = 30
                    type = "float"
                }
            },
            out = {
                name = "/villas-dpsim1"	# Name of shared memory segment for receiving side
            }
        }
    }

    paths = (
        {
            in = "dpsim1"
            out = "broker1"

            hooks = (
                {
                    type = "limit_rate"
                    rate = 50
                }
            )
        }
    )"""

with open("villas-node.conf", "w") as text_file:
    text_file.write("%s" % villas_conf)

#asyncio.run(main())

