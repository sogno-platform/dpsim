import time
from villas.node.node import Node as VILLASnode

# This could be moved to the DPsim Python code later
def get_dpsim_shmem_interface_config():
    return {
            'type': 'shmem',
            'in': {
                'name': '/dpsim1-villas',
                'hooks': [
                    {'type': 'stats'}
                ],
                'signals': {
                    'count': 30,
                    'type': 'float'
                }
            },
            'out': {
                'name': '/villas-dpsim1'
            }
        }

def get_villas_config():
    return {
        'nodes': {
            'broker1': {
                'type': 'mqtt',
                'format': 'json',
                'host': '172.17.0.1',
                'in': {
                    'subscribe': '/powerflow-dpsim'},
                'out': {
                    'publish': '/dpsim-powerflow'
                }
            },
            'dpsim1': get_dpsim_shmem_interface_config(),
        },
        'paths': [
            {
                'in': 'dpsim1',
                'out': 'broker1',

                'hooks': [
                    {
                        'type': 'limit_rate',
                        'rate': 50
                    }
                ]
            }
        ]
    }

def main():

    node = VILLASnode(
        config=get_villas_config()
    )

    node.start() # VILLASnode starts running in the background from here..

    # Some infos from the running VILLASnode instance queried via its REST API
    print('VILLASnode running?: ', node.is_running())
    print('VILLASnode status: ', node.status)
    print('VILLASnode nodes: ', node.nodes)
    print('VILLASnode paths: ', node.paths)
    print('VILLASnode config: ', node.active_config)
    print('VILLASnode version: ', node.get_version())

    # Load a new config into the running VILLASnode instance (old config will be replaced)
    new_config = node.active_config
    new_config['paths'].append({
        'out': 'dpsim1',
        'in': 'broker1'
    })

    node.load_config(new_config)

    time.sleep(100)

    node.stop()


if __name__ == '__main__':
    main()