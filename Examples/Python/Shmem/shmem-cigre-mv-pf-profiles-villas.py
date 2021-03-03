import asyncio
import time

async def run(cmd):
    proc = await asyncio.create_subprocess_shell(
        cmd,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE)

    #time.sleep(30)
    #proc.kill()
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

def write_villas_conf():

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
                    name = "/dpsim-villas",	# Name of shared memory segment for sending side

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
                    name = "/villas-dpsim"	# Name of shared memory segment for receiving side
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


write_villas_conf()

#asyncio.run(main())
#asyncio.run(run('villas-node villas-node.conf'))

import os
os.system('villas-node villas-node.conf')