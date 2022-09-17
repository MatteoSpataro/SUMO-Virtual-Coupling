from abc import ABC, abstractmethod

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
    def run(self):
        pass