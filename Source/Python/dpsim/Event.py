import enum

class Event(enum.IntEnum):
    stopped = 0
    starting = 1
    running = 2
    pausing = 3
    paused = 4
    resuming = 5
    stopping = 6
    failed = 7
    overrun = 8
    done = 9
    unknown = -1
