import _dpsim
import asyncio
import time
import sys
import datetime
import logging

LOGGER = logging.getLogger('dpsim.simulation')

from .EventChannel import EventChannel, Event

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

    def __init__(self, *args, loop=None, start_time=None, timestep=1e-3, pbar=False, **kwargs):
        if start_time is not None:
            if type(start_time) is datetime.datetime:
                kwargs['start_time'] = int(start_time.timestamp())
                kwargs['start_time_us'] = start_time.microsecond
            elif type(start_time) is float:
                kwargs['start_time'] = int(start_time)
                kwargs['start_time_us'] = fmod(start_time, 1) * 1e6
            elif type(start_time) is int:
                kwargs['start_time'] = start_time
            else:
                raise TypeError('Keyword start_time must be of type datetime.datetime')

        super().__init__(*args, timestep=timestep, **kwargs)

        self._ts = timestep
        self._pbar_tui = None
        self._pbar_widget = None

        self._loop = loop
        if loop is None:
            self._loop = asyncio.get_event_loop()

        self._fd = self.get_eventfd()

        self._events = EventChannel(self._fd, self._loop)

        self._events.add_callback(self.running, self, event=Event.running)
        self._events.add_callback(self.stopped, self, event=Event.stopped)
        self._events.add_callback(self.stopped, self, event=Event.done)
        self._events.add_callback(self.overrun, self, event=Event.overrun)

        if pbar:
            self.show_progressbar()

    def add_callback(self, cb, *args, event=None):
        self._events.add_callback(cb, *args, event=event)

    def running(self, *args):
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

    def overrun(self, *args):
        raise RuntimeError("Simulation overrun!")

    async def __update_progressbar_task(self):
        while self.time < self.final_time:
            self.__update_progressbar()
            await asyncio.sleep(0.2)

        self.__update_progressbar()

    def __update_progressbar(self):
        if self._pbar_widget:
            elapsed = time.time() - self._start_time
            rtfactor = self.time / elapsed
            progress = self.time / self.final_time
            total_steps = int(self.final_time / self._ts)

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
            t = self.time
            if t > self.final_time:
                t = self.final_time

            elapsed = time.time() - self._start_time
            rtfactor = t / elapsed

            self._pbar_tui.update(t,
                elapsed = elapsed,
                rtfactor = rtfactor
            )

    async def simulate(self):
        self.start()

        await self._events.wait(Event.running)
        await self._events.wait(Event.done)

    async def wait(self, evt=None):
        await self._events.wait(evt)

    def wait_until(self, evt=None):
        return self._loop.run_until_complete(self._events.wait(evt))

    def run(self, **kwargs):
        # https://github.com/jupyter/notebook/issues/3397#issuecomment-419474214
        if self._loop.is_running():
            raise RuntimeError("Event loop is already running! Please use: await sim.simulate()")

        self._loop.run_until_complete(self.simulate())

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
                    ' ', progressbar.DynamicMessage('rtfactor')
                ],
                max_value = self.final_time,
                redirect_stdout = True
            )

    def add_switch_event(self, time, switch, state):
        self.add_event(time, switch, 'closed', state)

class RealTimeSimulation(Simulation):
     def __init__(self, *args, **kwargs):
        super().__init__(*args, rt=True, **kwargs)
