import _dpsim

class Simulation(_dpsim.Simulation):

    def run(self):
        """Start a simulation and wait for its completion."""

        self.start()
        self.wait()
