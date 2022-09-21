from abc import ABC, abstractmethod

__author__ = "Matteo Spataro"
__license__ = "Eclipse Public License"
__version__ = "2.0"
__maintainer__ = "Matteo Spataro"
__email__ = "matteo.spataro@stud.unifi.it"

class Rbc(ABC):

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