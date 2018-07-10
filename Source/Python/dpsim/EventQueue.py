import asyncio
import struct
import os
import logging
import enum

LOGGER = logging.getLogger('dpsim.eventqueue')

class Event(enum.IntEnum):
    UNKNOWN = 0
    STARTED = 1
    STOPPED = 3

class EventQueue(object):
    def __init__(self, fd, loop = None):
        self._loop = loop
        if self._loop is None:
            self._loop = asyncio.get_event_loop()

        self._fd = fd
        self.sr = asyncio.StreamReader(loop = loop)

        self._callbacks = []

        self._last = Event.UNKNOWN
        self._lock = asyncio.Lock(loop = loop)
        self._cond = asyncio.Condition(lock = self._lock, loop = loop)

        LOGGER.debug("Created new event queue with fd=%d", self._fd)

        self._loop.create_task(self.__task())
        self._loop.add_reader(self._fd, self.__reader)

    def add_callback(self, evt, cb, *args):
        self._callbacks.append((evt, cb, args))

    async def wait(self, evt):
        await self._lock.acquire()

        while evt != self._last:
            LOGGER.debug("Waiting for event: %s. Current event %s" % (evt, self._last))
            await self._cond.wait()

        self._lock.release()

    async def notify(self, evt):
        await self._lock.acquire()

        self._last = evt
        self._cond.notify_all()

        self._lock.release()

    def __reader(self):
        data = os.read(self._fd, 4)
        self.sr.feed_data(data)

        LOGGER.debug("Read from: fd=%d, data=%s", self._fd, repr(data))

    async def __task(self):
        LOGGER.debug("Entering events task")

        while True:
            data = await self.sr.readexactly(4)

            unpacked = struct.unpack('i', data)
            evt = Event(unpacked[0])

            LOGGER.debug("Received event: evt=%s", evt)

            # Call user defined callbacks
            for e in self._callbacks:
                tevt = e[0]
                cb = e[1]
                args = e[2]

                if tevt == evt:
                    cb(self, *args)

            await self.notify(evt)
