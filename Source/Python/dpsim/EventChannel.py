import asyncio
import struct
import os
import logging
import queue
import socket

from .Event import Event

LOGGER = logging.getLogger('dpsim.EventChannel')

class EventChannel(object):
    def __init__(self, sock, loop = None):
        self._loop = loop
        if self._loop is None:
            self._loop = asyncio.get_event_loop()

        self._sock = sock
        self._callbacks = []
        self._queue = asyncio.Queue()

        self._last = Event.unknown

        self._read_task = self._loop.create_task(self.__task())

    def add_callback(self, cb, *args, event = None):
        self._callbacks.append((event, cb, args))

    def close(self):
        self._read_task.cancel()

    async def wait(self, evt):
        if evt == None:
            LOGGER.debug("Waiting for any event")
            revt = await self._queue.get()
        else:
            while True:
                LOGGER.debug("Waiting for event: %s. Current event %s" % (evt, self._last))
                revt = await self._queue.get()

                if evt == revt:
                    break

        LOGGER.debug("Received event in wait: event=%s", revt)
        return revt

    async def __task(self):
        LOGGER.debug("Entering events task")

        (reader, writer) = await asyncio.open_connection(loop = self._loop, sock = self._sock)

        while True:
            data = await reader.readexactly(4)

            unpacked = struct.unpack('i', data)
            evt = Event(unpacked[0])

            LOGGER.debug("Received event: event=%s", evt)

            # Call user defined callbacks
            for e in self._callbacks:
                tevt = e[0]
                cb = e[1]
                args = e[2]

                if tevt == evt or tevt is None:
                    cb(evt, *args)

            await self._queue.put(evt)
