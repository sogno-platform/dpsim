import _dpsim
import asyncio
import time
import struct
import os

class Simulation(_dpsim.Simulation):

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        if 'loop' in kwargs:
            self.loop = kwargs.get('loop')
        else:
            self.loop = asyncio.get_event_loop()

        self.started = 0
        self.callbacks = []

        self.fd = self.get_eventfd()
        self.loop.add_reader(self.fd, self.__event_handler)

    def __event_handler(self):

        unpacked = struct.unpack('i', os.read(self.fd, 4))
        evt = unpacked[0]

        if evt == 1: # started
            self.started = time.time()

        # Call user defined callbacks
        for cb in self.callbacks:
            cb(self, evt)

        print("Received event: %d" % evt)

    def run(self):
        """Start a simulation and wait for its completion."""

        self.start()
        self.wait()

    def start(self, **kwargs):
        """Start the simulation at a specific point in time."""

        if 'when' in kwargs:
            when = kwargs.get('when')

            delta = when - time.time()
            self.loop.call_at(self.loop.time() + delta, self.start)
        else:
            # Start immediately
            super().start()

    def register_callback(self, cb):
        self.callbacks.append(cb)
