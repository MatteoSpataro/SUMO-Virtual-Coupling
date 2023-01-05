from ast import If
import math
from train import *
import matplotlib.pyplot as plt
import numpy as np

from ctypes.wintypes import INT
import os
import sys

__author__ = "Matteo Spataro"
__license__ = "Eclipse Public License"
__version__ = "2.0"
__maintainer__ = "Matteo Spataro"
__email__ = "matteo.spataro@stud.unifi.it"

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary
import traci

from rbc import Rbc


class RbcNoVC(Rbc):
       
    def __init__(self, nTrain, isVariant):
        super().__init__()
        self.__trainList = [] #List of active trains
        self.__distances = [] #List with the distances between trains
        self.__oldSpeed = []   #List with the speeds of the trains in the previous step
        self.__incomingTrains = 0 #Number of trains that are coming
        self.__distToPlot = [0] #To plot the distance graph between the first 2 trains
        self.__variant = isVariant #If is True, than we are in the second version of the circuit
        self.__step = 1 #step of the simulation
        if isVariant:
            self.__roadToCheck = ["E23","E22","E21"]
        else:
            self.__roadToCheck = ["E36","E35","E34"]

        for idTrain in range(0, 3):
            defaultSpeed = self.DEFAULT_SPEED - 0.8*idTrain
            train = Train(str(idTrain), defaultSpeed, self.DEFAULT_ACCEL, self.DEFAULT_DECEL)
            self.__trainList.append(train)
        self.__incomingTrains = nTrain - 3

    def getTrainList(self):
        return self.__trainList

    def printAllSpeed(self):
        print("\nSpeeds:")
        for train in self.__trainList:
            print("--Train", train.getId(), ":", round(traci.vehicle.getSpeed(train.getId()),6),
                  " # Decel:", round(traci.vehicle.getDecel(train.getId()),4))

    def printDistances(self):
        print("\n-Distance between trains: ")
        for i in range(0, len(self.__trainList)-1):
            if self.__distances[i] != -1:
                print("---T", self.__trainList[i].getId(), " and T", int(self.__trainList[i].getId())+1, ": ", self.__distances[i], "m.")
            else:
                print("---T", self.__trainList[i].getId(), " and T", int(self.__trainList[i].getId())+1, ": undefined.")
    
    def _updateOldSpeed(self):
        self.__oldSpeed.clear()
        for train in self.__trainList:
            self.__oldSpeed.append(traci.vehicle.getSpeed(train.getId()))

    #Updates the list of active trains
    def _updateTrainsActive(self):
        idTrains = traci.vehicle.getIDList()
        for train in self.__trainList:
            thereis = False
            for id in idTrains:
                if train.getId().__eq__(id):
                    thereis = True
            if thereis == False:
                self.__trainList.remove(train)

    def _freeRoad(self, roadToCheck):
        for train in self.__trainList:
            for roadId in roadToCheck:
                if traci.vehicle.getRoadID(train.getId()).__eq__(roadId):
                    return False
        return True

    # Check if the next Train is already loaded from SUMO.
    def _isTrainLoaded(self):
        idTrains = traci.vehicle.getIDList()
        idNextTrain = str(int(self.__trainList[-1].getId())+1)
        for id in idTrains:
            if id.__eq__(idNextTrain):
                return True
        return False

    def _addTrain(self):
        lastTrain = self.__trainList[-1]
        newTrain = Train(str(int(lastTrain.getId())+1), lastTrain.getDefaultSpeed(),
                         self.DEFAULT_ACCEL, self.DEFAULT_DECEL)
        self.__trainList.append(newTrain)
        
    # Method that returns True if the train in input is in any roadID of the list "roadIDs" given in input.
    def _trainIsInRoadID(self, train, roadIDs):
        for roadID in roadIDs:
            if traci.vehicle.getRoadID(train.getId()).__eq__(roadID):
                return True
        return False

    # This method does the operations to control the speed of the train in input if this is trying to enter the circuit.
    def _controlTrainIncoming(self, pos):
        train = self.__trainList[pos]
        trainAhead = self.__trainList[pos-1]
        idTrain = int(train.getId())
        if traci.vehicle.getRoadID(train.getId()).__eq__("E3"):
            if self._freeRoad(["E5", "E55"]):
                traci.vehicle.setSpeed(str(idTrain), train.getDefaultSpeed()*0.35)
                train.setSpeed(train.getDefaultSpeed()*0.35)
            else:
                traci.vehicle.setSpeed(str(idTrain), self.__oldSpeed[pos-1])
                train.setSpeed(self.__oldSpeed[pos-1])
        roadFree = self._freeRoad(self.__roadToCheck)
        if traci.vehicle.getRoadID(train.getId()).__eq__("E5"):
            if roadFree:
                if train.getSpeed() < train.getDefaultSpeed():
                    traci.vehicle.setSpeed(str(idTrain), train.getDefaultSpeed()+1)
                    train.setSpeed(train.getDefaultSpeed()+1)
            elif traci.vehicle.getRoadID(train.getId()).__eq__("E55"):
                traci.vehicle.setSpeed(str(idTrain), 0.5)
                train.setSpeed(0.5)
            else:
                traci.vehicle.setSpeed(str(idTrain), train.getDefaultSpeed()*0.02)
                train.setSpeed(train.getDefaultSpeed()*0.02)
        if traci.vehicle.getRoadID(train.getId()).__eq__("E55"):
            if ((traci.vehicle.getRoadID(trainAhead.getId()).__eq__("E0") 
                or traci.vehicle.getRoadID(trainAhead.getId()).__eq__("E6"))
                                and roadFree):
                if train.getSpeed() < train.getDefaultSpeed():
                    traci.vehicle.setSpeed(str(idTrain), train.getDefaultSpeed()+1)
                    train.setSpeed(train.getDefaultSpeed()+1)
            else: 
                traci.vehicle.setSpeed(str(idTrain), 0.0)
                train.setSpeed(0.0)

    #Method to plot the distance graph between the first 2 trains
    def _plotDist(self):
        steps = np.arange(0, self.__step, 1)
        for i in range(0,len(self.__distToPlot)):
            self.__distToPlot[i] = self.__distToPlot[i]*10
        plt.plot(steps, self.__distToPlot, label="T0 - T1")
        plt.xlabel('Time [s]')
        plt.ylabel('Distance [m]')
        plt.title('Without Virtual Coupling')
        plt.legend()
        plt.show()

    def run(self):
        traci.simulationStep()
        for train in self.__trainList:
            traci.vehicle.setSpeed(train.getId(), train.getDefaultSpeed())
        #Start the run of the trains
        while traci.simulation.getMinExpectedNumber() > 0:
            print("\n\n-------Step ", self.__step)
            if len(traci.vehicle.getIDList()) > 1:
                self._updateTrainsActive()
                #Check if there is an incoming train
                if self.__incomingTrains > 0 :
                    if self._isTrainLoaded():
                        self._addTrain()
                        self.__incomingTrains -= 1
                self.__distances.clear()
                #update follower list
                for train in self.__trainList:
                    distance = traci.vehicle.getFollower(train.getId(), 0) #[idFollower, distance]
                    self.__distances.append(distance[1])
                self.printDistances()
                
                stepsToWait = 250 #steps to wait before the first change of a train's direction
                if self.__trainList[0].getDefaultSpeed() <= 15: 
                    stepsToWait = 250+25
                
                if self.__step > stepsToWait:
                    road1 = "E30" #roadID where send the comand to the first train
                    road2 = "E31" #roadID where send the comand to the second train
                    if self.__variant:
                        road1 = "E17"
                        road2 = "E18"
                    if traci.vehicle.getRoadID(self.__trainList[0].getId()).__eq__(road1): 
                        print("\n### Set change of direction for Train", self.__trainList[0].getId())
                        traci.vehicle.changeTarget(self.__trainList[0].getId(), "E48")
                    if len(self.__trainList)>2:
                        if traci.vehicle.getRoadID(self.__trainList[1].getId()).__eq__(road2): 
                            print("\n### Set change of direction for Train", self.__trainList[1].getId())
                            traci.vehicle.changeTarget(self.__trainList[1].getId(), "E51")      
                
                self.printAllSpeed()

                self._updateOldSpeed()
                #If there are more than a cert amount of trains, the new train has to wait the last to enter the circuit
                for i in range(1, len(self.__trainList)):
                    self._controlTrainIncoming(i)
                if self.__incomingTrains > 0: print("#\nIncoming trains:",self.__incomingTrains)

                #Run these instructions to plot the distance graph
                if self.__distances[0] != -1:
                    self.__distToPlot.append(self.__distances[0])
                else:
                    self.__distToPlot.append(self.__distToPlot[-1])
            else:
                self.__distToPlot.append(0)
                self._updateTrainsActive()
                traci.vehicle.changeTarget(self.__trainList[0].getId(), "E48")
                traci.vehicle.setSpeed(self.__trainList[0].getId(), self.__trainList[0].getDefaultSpeed())
            traci.simulationStep()
            self.__step += 1
        print("\n\nSimulation completed.")
        #To plot the distance graph you have to run this instruction
        self._plotDist()
        traci.close()
        sys.stdout.flush()
                