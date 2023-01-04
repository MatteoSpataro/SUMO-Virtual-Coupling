from abc import ABC, abstractmethod

__author__ = "Matteo Spataro"
__license__ = "Eclipse Public License"
__version__ = "2.0"
__maintainer__ = "Matteo Spataro"
__email__ = "matteo.spataro@stud.unifi.it"

class Rbc(ABC):

    def __init__(self):
        super().__init__()
        self.DEFAULT_SPEED = 20.8
        self.MIN_SPEED = 14.0 #equals to 140 km/h
        self.MAX_SPEED = 30.0 #equals to 300 km/h
        self.DEFAULT_ACCEL = 0.7 #m/s^2
        self.DEFAULT_DECEL = 0.7 #m/s^2
        self.MAX_DECEL = 1.0 #m/s^2
        
    @abstractmethod
    def getTrainList(self):
        pass

    @abstractmethod
    def printAllSpeed(self):
        pass

    @abstractmethod
    def printDistances(self):
        pass

    @abstractmethod
    def _updateTrainsActive(self):
        pass

    @abstractmethod
    def _addTrain(self):
        pass

    @abstractmethod
    def run(self):
        pass