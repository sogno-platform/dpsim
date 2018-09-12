import json
import tempfile
import subprocess
import logging

LOGGER = logging.getLogger('villas.node')

class Node(object):

    def __init__(self, cfg):
        self.config = cfg

    def start(self):
        self.config_file = tempfile.NamedTemporaryFile(mode='w+', suffix='.json')

        json.dump(self.config, self.config_file)
        self.config_file.flush()

        LOGGER.info("Starting VILLASnode with config: %s", self.config_file.name)

        self.child = subprocess.Popen(['villas-node', self.config_file.name])

    def stop(self):
        self.child.kill()
        self.child.wait()

