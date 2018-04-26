import _dpsim
import asyncio
import time
import progressbar
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

            if self.pbar:
                self.pbar_task = self.loop.create_task(self.__update_progressbar())

        if evt == 3 or evt == 2: # stopped
            if self.pbar_task:
                self.pbar_task.cancel()

        # Call user defined callbacks
        for e in self.callbacks:
            cb = e[0]
            args = e[1]

            cb(self, evt, *args)

        print("Received event: %d" % evt)

    async def __update_progressbar(self):
        self.pbar.start()
        while True:
            elapsed = time.time() - self.started
            rtfactor = self.time / elapsed

            self.pbar.update(self.time,
                step = self.steps,
                elapsed = elapsed,
                rtmul = rtfactor
            )
            await asyncio.sleep(0.05)
        self.pbar.finish()

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

    def show_progressbar(self):
        self.pbar = progressbar.ProgressBar(
            widgets = [
                ' ', progressbar.Percentage(),
                ' ', progressbar.SimpleProgress(format='%(value).2f of %(max_value).2f secs'),
                ' ', progressbar.Bar('=', fill='.'),
                ' ', progressbar.DynamicMessage('elapsed'),
                ' ', progressbar.ETA(format='eta: %(eta_seconds).2f', ),
                ' ', progressbar.DynamicMessage('step'),
                ' ', progressbar.DynamicMessage('rtmul')
            ],
            max_value = self.final_time,
            redirect_stdout = True
        )

    def register_callback(self, cb, *args):
        self.callbacks.append((cb, args))
