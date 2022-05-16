
class State():
    def __init__(self, scf, pc, multiranger):
        self.scf = scf
        self.pc = pc
        self.multiranger = multiranger
        pass

    def run_once(self):
        raise NotImplementedError

    def step(self):
        raise NotImplementedError
    
