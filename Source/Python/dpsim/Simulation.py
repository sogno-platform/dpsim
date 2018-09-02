import _dpsim
import asyncio
import time
import sys
import logging

LOGGER = logging.getLogger('dpsim.simulation')

from .EventQueue import EventQueue, Event

def is_interactive():
    import __main__ as main
    return not hasattr(main, '__file__')

def is_ipython():
    if 'ipykernel' in sys.modules:
        return 'notebook'
    elif 'IPython' in sys.modules:
        return 'terminal'
    else:
        return False

class Simulation(_dpsim.Simulation):

    def __init__(self, *args, loop = None, **kwargs):
        super().__init__(*args, **kwargs)

        self._loop = loop
        if loop is None:
            self._loop = asyncio.get_event_loop()

        if 'timestep' in kwargs:
            self.ts = kwargs.get('timestep')

        self._start_time = None
        self._pbar_tui = None
        self._pbar_widget = None

        self._fd = self.get_eventfd()

        self.events = EventQueue(self._fd, self._loop)

        self.events.add_callback(self.started, self, event=Event.running)
        self.events.add_callback(self.stopped, self, event=Event.stopped)

    def add_callback(self, cb, *args, evt=None):
        self.events.add_callback(cb, *args, event=evt)

    def started(self, *args):
        LOGGER.info("Started simulation!")

        self._start_time = time.time()

        if self._pbar_tui:
            self._pbar_tui.start()

        self._pbar_task = self._loop.create_task(self.__update_progressbar_task())

    def stopped(self, *args):
        self.__update_progressbar()

        if self._pbar_tui:
            self._pbar_tui.finish()

        LOGGER.info('Finished simulation!')

    async def __update_progressbar_task(self):
        while self.time < self.final_time:
            self.__update_progressbar()
            await asyncio.sleep(0.05)

        self.__update_progressbar()

    def __update_progressbar(self):
        if self._pbar_widget:
            elapsed = time.time() - self._start_time
            rtfactor = self.time / elapsed
            progress = self.time / self.final_time
            total_steps = int(self.final_time / self.ts)

            if self.time >= self.final_time:
                state = 'finished'
            elif self.time == 0:
                state = 'starting'
            else:
                state = 'running'

            self._pbar_widget.value = self.time # signal to increment the progress bar
            self._pbar_widget_text.value = "Simulation is {:s}: <pre>{:8d}/{:d} steps, progress={:.0%} time={:.2f}, elapsed={:.2f}, rtfactor={:.2f}</pre>".format(state, self.steps, total_steps, progress, self.time, elapsed, rtfactor)

            self._pbar_widget.value = self.time

        elif self._pbar_tui:
            elapsed = time.time() - self._start_time
            rtfactor = self.time / elapsed

            self._pbar_tui.update(self.time,
                step = self.steps,
                elapsed = elapsed,
                rtmul = rtfactor
            )

    def start(self, **kwargs):
        """Start the simulation at a specific point in time."""

        if 'when' in kwargs:
            when = kwargs.get('when')

            delta = when - time.time()
            self._loop.call_at(self._loop.time() + delta, super().start)
        else:
            # Start immediately
            super().start()

    async def simulate(self, **kwargs):
        self.start(**kwargs)

        await self.events.wait(Event.running)
        await self.events.wait(Event.stopped)

    def run(self, **kwargs):
        if 'pbar' in kwargs and kwargs['pbar']:
            self.show_progressbar()

        self._loop.run_until_complete(self.simulate(**kwargs))

    def show_progressbar(self):
        if is_ipython():
            from ipywidgets import FloatProgress, HTML
            from IPython.display import display

            self._pbar_widget = FloatProgress(min = 0, max = self.final_time)
            self._pbar_widget_text = HTML(value = 'Simulation start is pending...')

            display(self._pbar_widget_text, self._pbar_widget)
        else:
            import progressbar

            self._pbar_tui = progressbar.ProgressBar(
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

class RealTimeSimulation(Simulation):
     def __init__(self, *args, **kwargs):
        super().__init__(*args, rt=True, **kwargs)
