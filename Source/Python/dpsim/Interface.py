import _dpsim
import datetime
import logging

LOGGER = logging.getLogger('dpsim.interface')

try:
    from _dpsim import Interface
    class Interface(_dpsim.Interface):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)

        def get_villas_config(self):
            return {
                'type' : 'shmem',
                'out' : {
                    'name' : self.rname
                },
                'in' : {
                    'name' : self.wname,
                    'signals' : self.exports
                },
                'polling' : bool(self.polling),
                'queuelen' : self.queuelen,
                'samplelen' : self.samplelen
            }
except:
    class Interface():
        def __init__(self, *args, **kwargs):
            raise NotImplementedError("dpsim compiled without shmem interface support")
