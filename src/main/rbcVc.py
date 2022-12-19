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

MIN_DIST_COUP = 5     #min distance of virtual coupling
MAX_DIST_COUP = 50    #max distance of virtual coupling
MIN_DIST_DECOUP = 45  #min distance of virtual decoupling
MAX_DIST_DECOUP = 150 #max distance of virtual decoupling
PARAM_COUPLING = 5.5  #VC policy parameter
PARAM_DECOUPLING = 5.5
DEFAULT_SPEED = 20.0
MIN_SPEED = 14.0 #equals to 140 km/h
MAX_SPEED = 30.0 #equals to 300 km/h
DEFAULT_ACCEL = 0.7 #m/s^2
DEFAULT_DECEL = 0.7 #m/s^2
MAX_DECEL = 1.0 #m/s^2
MAX_DISCONNECTIONS = 6 #max number of tolerated disconnections between trains
MARGIN_VC = 0.1 #error margin of VC, equals to 10%

class RbcVC(Rbc):
       
    def __init__(self, nTrain, DEPARTURE_INTERVAL, isVariant):
        self.__distanceCoupling = 10    #equals to x10 real meters
        self.__distanceDecoupling = 100 #equals to x10 real meters
        self.__trainList = []  #List of active trains
        self.__distances = []  #List with the distances between trains
        self.__oldSpeed = []   #List with the speeds of the trains in the previous step
        self.__couplingTrain = [True]    #Train is trying to reach the coupling if is "True"
        self.__decouplingTrain = [False] #Train is trying to reach the decoupling if is "True"
        self.__state = ["decoupled"] #List with the state of each train
        self.__isBraking = [False]   #Is "True" if the train ahead is braking
        self.__incomingTrains = 0    #Number of trains that are coming
        self.__countDisconnection = [0]  #Number of sequential disconnections for each train
        self.DEPARTURE_INTERVAL = DEPARTURE_INTERVAL
        self.TRAINS_TO_WAIT = 0
        self.__factorSpeed = 6
        self.__variant = isVariant #If is True, than we are in the second version of the circuit
        if isVariant:
            self.__roadToCheck = ["E23","E22","E21","E20"]
        else:
            self.__roadToCheck = ["E36","E35","E34","E33"]
        self.__distToPlot = [0] #To plot the distance graph between the first 2 trains
        self.__step = 1 #step of the simulation
        #initialize trainList:
        for idTrain in range(0, 3):
            defaultSpeed = DEFAULT_SPEED - 0.8*idTrain
            train = Train(str(idTrain), defaultSpeed, DEFAULT_ACCEL, DEFAULT_DECEL)
            self.__trainList.append(train)
        self.__incomingTrains = nTrain - 3

    def getTrainList(self):
        return self.__trainList

    def getDistanceCoupling(self):
        return self.__distanceCoupling

    def getDistanceDecoupling(self):
        return self.__distanceDecoupling

    def getDistances(self):
        return self.__distances

    def getOldSpeeds(self):
        return self.__oldSpeed

    def getCouplingList(self):
        return self.__couplingTrain

    def getDecouplingList(self):
        return self.__decouplingTrain

    def getStateList(self):
        return self.__state
    
    def getIsBrakingList(self):
        return self.__isBraking

    def getIncomingTrains(self):
        return self.__incomingTrains

    def getCountDisconnectionList(self):
        return self.__countDisconnection

    def setDistanceCoupling(self, distance):
        if distance < MIN_DIST_COUP or distance > MAX_DIST_COUP:
            raise Exception("Sorry, coupling distance must be between",MIN_DIST_COUP,"and",MAX_DIST_COUP)
        self.__distanceCoupling = distance

    def setDistanceDecoupling(self, distance):
        if distance < MIN_DIST_DECOUP or distance > MAX_DIST_DECOUP:
            raise Exception("Sorry, decoupling distance must be between",MIN_DIST_DECOUP,"and",MAX_DIST_DECOUP)
        self.__distanceDecoupling = distance
    
    #Method to initialize the parameters before the start of the simulation.
    def _setInitialParameters(self):
        for train in range(2, len(self.__trainList)):
            self.__couplingTrain.append(True)
            self.__decouplingTrain.append(False)
            self.__isBraking.append(False)
            self.__state.append("decoupled")
            self.__countDisconnection.append(0)
        self.__factorSpeed = self.__trainList[0].getDefaultSpeed() - 15

    def printAllSpeed(self):
        print("\nSpeeds:")
        for (pos, train) in enumerate(self.__trainList):
            print("--Train", train.getId(), ":", round(self.__oldSpeed[pos],4),
                  " # Decel:", round(traci.vehicle.getDecel(train.getId()),4))

    #Method to set the velocity profile of the trainAhead into the trainFollower.
    def _setSameSpeedFactores(self, trainFollower, trainAhead):
        idFollower = trainFollower.getId()
        idAhead = trainAhead.getId()
        traci.vehicle.setSpeedFactor(idFollower, traci.vehicle.getSpeedFactor(idAhead))
        traci.vehicle.setAccel(idFollower, traci.vehicle.getAccel(idAhead))
        traci.vehicle.setDecel(idFollower, traci.vehicle.getDecel(idAhead))
        trainFollower.setAccel(trainAhead.getAccel())
        trainFollower.setDecel(trainAhead.getDecel())

    def _changeDecel(self, trainFollower, decelAhead, increment):
        newDecel = decelAhead+increment
        if newDecel <= MAX_DECEL: trainFollower.setDecel(newDecel)
        else: trainFollower.setDecel(MAX_DECEL)

    def _stepDecoupling(self, pos):
        trainAhead = self.__trainList[pos]
        trainFollower = self.__trainList[pos+1]
        if self.__distances[pos] == -1: return True
        if self.__distances[pos] < self.__distanceDecoupling:    
            trainFollowerSpeed = self.__oldSpeed[pos+1]
            traci.vehicle.setSpeed(trainFollower.getId(), trainFollowerSpeed - 1)
            trainFollower.setSpeed(trainFollowerSpeed - 1)
            print("In decoupling, Train", trainFollower.getId(), "is decreasing speed.")
            self.__state[pos] = "almost_decoupled"
            posAhead = pos+1
            while posAhead < len(self.__trainList)-1:
                idFollower = self.__trainList[posAhead+1].getId()
                trainFollower = self.__trainList[posAhead+1]
                trainAhead = self.__trainList[posAhead]
                #speedAhead = traci.vehicle.getSpeed(idAhead)
                speedAhead = trainAhead.getSpeed()
                decelAhead = trainAhead.getDecel()
                speedFollower = self.__oldSpeed[posAhead+1]
                if self.__state[posAhead].__eq__("coupled"):
                    newSpeed = speedAhead
                    self._changeDecel(trainFollower, decelAhead, 0.010)
                    if self.__oldSpeed[posAhead] != self.__oldSpeed[posAhead+1]:
                        self.__state[posAhead] = "almost_coupled"
                    traci.vehicle.setSpeed(idFollower, newSpeed)
                    trainFollower.setSpeed(newSpeed)
                if self.__state[posAhead].__eq__("almost_coupled"):
                    newSpeed = 0.1
                    # when distance is '-1' then there was an error reading the distance
                    if self.__distances[posAhead] == -1: 
                        newSpeed = trainFollower.getSpeed()-0.3
                    elif speedFollower-speedAhead >= 1.5:
                        if decelAhead < MAX_DECEL:
                            newSpeed = speedFollower*0.40 
                            self._changeDecel(trainFollower, decelAhead, 0.040)
                        else:
                            newSpeed = 0.1 
                            trainFollower.setDecel(MAX_DECEL)
                    elif self.__distances[posAhead] <= self.__distanceCoupling+self.__distanceCoupling*0.50:
                        newSpeed = speedAhead*0.40
                        self._changeDecel(trainFollower, decelAhead, 0.025)
                    elif self.__distances[posAhead] <= self.__distanceCoupling*2:
                        newSpeed = 0.1
                        self._changeDecel(trainFollower, decelAhead, 0.020)
                        #newSpeed = speedAhead*0.50
                    elif self.__distances[posAhead] <= self.__distanceCoupling*3:
                        newSpeed = speedAhead*0.60
                        self._changeDecel(trainFollower, decelAhead, 0.015)
                    elif self.__distances[posAhead] <= self.__distanceCoupling*5:
                        newSpeed = speedAhead*0.80
                        self._changeDecel(trainFollower, decelAhead, 0.010)
                    else:
                        newSpeed = speedAhead
                        self._changeDecel(trainFollower, decelAhead, 0.0)
                    traci.vehicle.setSpeed(idFollower, newSpeed)
                    trainFollower.setSpeed(newSpeed)
                posAhead += 1
        else:
            #now trains are decoupled and they have to remain decoupled
            if trainAhead.getDefaultSpeed() < trainFollower.getDefaultSpeed():
                traci.vehicle.setSpeed(trainFollower.getId(), trainAhead.getDefaultSpeed())
                trainFollower.setSpeed(trainAhead.getDefaultSpeed())
            else:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollower.getDefaultSpeed())
                trainFollower.setSpeed(trainFollower.getDefaultSpeed())
            print("\n\nDECOUPLING COMPLETED BETWEEN T", trainAhead.getId(), " AND T", trainFollower.getId(), "\n")
            self.__state[pos] = "decoupled"
            #Reset the speed of all the trains coupled or almost coupled with the follower train
            for posAhead in range(pos+1, len(self.__trainList)-1):
                if self.__state[posAhead].__eq__("coupled") or self.__state[posAhead].__eq__("almost_coupled"):
                    trainBehind = self.__trainList[posAhead+1]
                    traci.vehicle.setSpeed(trainBehind.getId(), trainFollower.getSpeed())
                    trainBehind.setSpeed(trainFollower.getSpeed())
                    if trainBehind.getDecel() > DEFAULT_DECEL:
                        traci.vehicle.setDecel(trainBehind.getId(), DEFAULT_DECEL)
                    print("\nIn decoupling, Train", trainBehind.getId(), "reset the speed.")
            return False
        return True

    #Method to check if the train ahead is decoupling: in this case we can't modify the speed of the following trains 
    #in order not to overwrite the changes made during the decoupling phase of the train ahead.
    def _trainAheadDecoupling(self, idTrainAhead):
        while idTrainAhead > 0:
            if self.__decouplingTrain[idTrainAhead-1] == True:
                return True
            elif self.__state[idTrainAhead-1].__eq__("coupled") or self.__state[idTrainAhead-1].__eq__("almost_coupled"):
                return self._trainAheadDecoupling(idTrainAhead-1)
            else:
                return False
        return False

    def _increase(self, train, speed, increment):
        traci.vehicle.setSpeed(train.getId(), speed+increment)
        train.setSpeed(speed+increment)
        print("-Train", train.getId(), "is increasing speed.")

    def _decrease(self, train, speed, decrement):
        traci.vehicle.setSpeed(train.getId(), speed+decrement)
        train.setSpeed(speed+decrement)
        print("-Train", train.getId(), "is decreasing speed.")

    def _stepCoupling(self, pos):
        trainAhead = self.__trainList[pos]
        trainFollower = self.__trainList[pos+1]
        trainAheadSpeed = self.__oldSpeed[pos]
        trainFollowerSpeed = self.__oldSpeed[pos+1]
        speedDiff = trainFollowerSpeed - trainAheadSpeed
        #check if the follower train is entering the circuit:
        if traci.vehicle.getRoadID(trainFollower.getId()).__eq__("E3") or traci.vehicle.getRoadID(trainFollower.getId()).__eq__("E5"):
            self._setSameSpeedFactores(trainFollower, trainAhead)
            print("Train", trainFollower.getId(), "is waiting.")
            return True
        if self.__state[pos].__eq__("almost_coupled"):    
            #check if there is a train ahead that is decoupling:
            if self._trainAheadDecoupling(pos+1):
                print("Train", trainFollower.getId(), "skip step coupling.")
                return True
        #check if there is a connection problem:
        if self.__distances[pos] == -1 and (not traci.vehicle.getRoadID(trainFollower.getId()).__eq__("E2")): 
            print("Train", trainFollower.getId(), "has connectivity problems caused by the road.")
            if self.__countDisconnection[pos] < MAX_DISCONNECTIONS:
                #There are connectivity problems in some roads.
                traci.vehicle.setSpeed(trainFollower.getId(), trainAheadSpeed)
                trainFollower.setSpeed(trainAheadSpeed)
                traci.vehicle.setDecel(trainFollower.getId(), trainFollower.getDecel()+0.02)
                self.__countDisconnection[pos] += 1
            else:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollower.getDefaultSpeed()*1.50)
                trainFollower.setSpeed(trainFollower.getDefaultSpeed()*1.50)
                traci.vehicle.setDecel(trainFollower.getId(), trainFollower.getDecel())
            return True
        self.__countDisconnection[pos] = 0
        traci.vehicle.setDecel(trainFollower.getId(), DEFAULT_DECEL)
        # Check if the trains are too near
        if self.__distances[pos] > 0 and self.__distances[pos] <= self.__distanceCoupling*0.50:
            print("Train", trainFollower.getId(), "coupling 0.")
            self._decrease(trainFollower, trainFollowerSpeed, -1)
            self._changeDecel(trainFollower, trainAhead.getDecel(), 0.25)
            return True
        # Check if trains are coupled
        if self.__distances[pos] <= self.__distanceCoupling + self.__distanceCoupling*MARGIN_VC:
            print("\nCOUPLING COMPLETED BETWEEN T", trainAhead.getId(), " AND T", trainFollower.getId(), "\n")
            traci.vehicle.setSpeed(trainFollower.getId(), traci.vehicle.getSpeed(trainAhead.getId()))
            trainFollower.setSpeed(trainAhead.getSpeed())
            self._setSameSpeedFactores(trainFollower, trainAhead)
            self.__state[pos] = "coupled"
            return False
        if self.__distances[pos] >= self.__distanceDecoupling*PARAM_DECOUPLING:
            if speedDiff < 3.5 and trainFollowerSpeed < MAX_SPEED-1:
                print("Train", trainFollower.getId(), "coupling 1.")
                self._increase(trainFollower, trainFollowerSpeed, 1)
                trainFollower.setAccel(trainAhead.getAccel())
            else:
                print("Train", trainFollower.getId(), "coupling 2.")
                self._decrease(trainFollower, trainFollowerSpeed, -1)
        elif self.__distances[pos] >= self.__distanceDecoupling:
            if self.__distances[pos] < self.__distanceDecoupling*2.0:
                self.__state[pos] = "almost_coupled"
            if speedDiff <= 1.5 and trainFollowerSpeed < MAX_SPEED-1:
                print("Train", trainFollower.getId(), "coupling 3.")
                self._increase(trainFollower, trainFollowerSpeed, 1)
                trainFollower.setAccel(trainAhead.getAccel())
            else:
                print("Train", trainFollower.getId(), "coupling 4.")
                self._decrease(trainFollower, trainFollowerSpeed, -1)
        elif self.__distances[pos] >= self.__distanceCoupling*PARAM_COUPLING + 5*self.__factorSpeed:
            self.__state[pos] = "almost_coupled"
            if speedDiff < 1.5 and trainFollowerSpeed < MAX_SPEED-1:
                print("Train", trainFollower.getId(), "coupling 5.")
                self._increase(trainFollower, trainFollowerSpeed, 1)
                trainFollower.setAccel(trainAhead.getAccel())
            else:
                print("Train", trainFollower.getId(), "coupling 6.")
                self._decrease(trainFollower, trainFollowerSpeed, -1)
                self._changeDecel(trainFollower, trainAhead.getDecel(), 0.02)
        elif self.__distances[pos] > self.__distanceCoupling + self.__distanceCoupling*MARGIN_VC:
            self.__state[pos] = "almost_coupled"
            if speedDiff > 3:
                print("Train", trainFollower.getId(), "coupling 7.")
                self._decrease(trainFollower, trainFollowerSpeed, -4)
                self._changeDecel(trainFollower, 1, MAX_DECEL)
            elif speedDiff > 2.5:
                print("Train", trainFollower.getId(), "coupling 8.")
                self._decrease(trainFollower, trainFollowerSpeed, -2.5)
                self._changeDecel(trainFollower, trainAhead.getDecel(), 0.01)
            elif speedDiff > 1.5:
                print("Train", trainFollower.getId(), "coupling 9.")
                self._decrease(trainFollower, trainFollowerSpeed, -1.3)
                self._changeDecel(trainFollower, trainAhead.getDecel(), 0.01)
            elif speedDiff > 1:
                print("Train", trainFollower.getId(), "coupling 10.")
                self._decrease(trainFollower, trainFollowerSpeed, -0.7)
            elif speedDiff>=0:
                print("Train", trainFollower.getId(), "coupling 11.")
                self._increase(trainFollower, self.__trainList[pos].getSpeed(), 0.65)
                trainFollower.setAccel(trainAhead.getAccel())
            else:
                # The train ahead is going faster than the follower
                print("Train", trainFollower.getId(), "coupling 12.")
                self._increase(trainFollower, self.__trainList[pos].getSpeed(), 0.05)
                self._setSameSpeedFactores(trainFollower, trainAhead)
        return True

    def _stepHoldState(self, pos):
        trainAhead = self.__trainList[pos]
        trainFollower = self.__trainList[pos+1]
        speedAhead = self.__oldSpeed[pos]
        speedFollower = self.__oldSpeed[pos+1]
        if self._trainAheadDecoupling(pos+1):
            print("Train ", trainFollower.getId(), "skip step hold state.")
            return
        if self.__oldSpeed[pos] > traci.vehicle.getSpeed(trainAhead.getId()):
            # the train ahead is decreasing his speed
            traci.vehicle.setSpeed(trainFollower.getId(), trainAhead.getSpeed())
            trainFollower.setSpeed(trainAhead.getSpeed())
            self._changeDecel(trainFollower, trainAhead.getDecel(), 0.10)
            self._setSameSpeedFactores(trainFollower, trainAhead)
            self.__isBraking[pos] = True
            self.__state[pos] = "almost_coupled"
            return
        if self.__isBraking[pos] == True:
            # The train ahead is no more braking, because self.__oldSpeed[pos] >= traci.vehicle.getSpeed(trainAhead.getId())
            self.__isBraking[pos] = False 
            self.__couplingTrain[pos] = True #The trains must retrieve their coupling
            traci.vehicle.setSpeed(trainFollower.getId(), speedAhead)
            trainFollower.setSpeed(speedAhead)
            self._setSameSpeedFactores(trainFollower, trainAhead)
            print("Trains T", trainAhead.getId(), "and T", trainFollower.getId(), "retrieve their coupling.")
            self.__state[pos] = "almost_coupled"
            return
        if self.__distances[pos] == -1:
            traci.vehicle.setSpeed(trainFollower.getId(), speedAhead)
            trainFollower.setSpeed(speedAhead)
            trainFollower.setDecel(trainAhead.getDecel())
            return
        elif self.__distances[pos] < (self.__distanceCoupling/2.0 + 1):
            traci.vehicle.setSpeed(trainFollower.getId(), speedFollower-0.8)
            trainFollower.setSpeed(speedFollower-0.8)
            self._changeDecel(trainFollower, trainAhead.getDecel(), 0.010)
            self.__state[pos] = "almost_coupled"
            self.__couplingTrain[pos] = True #The trains must retrieve their coupling
            return
        elif self.__distances[pos] > self.__distanceCoupling + MARGIN_VC*10:
            traci.vehicle.setSpeed(trainFollower.getId(), speedFollower+0.1)
            trainFollower.setSpeed(speedFollower+0.1)
            self.__state[pos] = "almost_coupled"
            self.__couplingTrain[pos] = True #The trains must retrieve their coupling
            return
        traci.vehicle.setSpeed(trainFollower.getId(), speedAhead)
        trainFollower.setSpeed(speedAhead)
        trainFollower.setDecel(trainAhead.getDecel())
        self._setSameSpeedFactores(trainFollower, trainAhead)
        if self.__state[pos].__eq__("almost_coupled"): self.__state[pos] = "coupled"

    def printDistances(self):
        print("\nDistance between trains: ")
        for i in range(0, len(self.__trainList)-1):
            if self.__distances[i] != -1:
                print("---T", self.__trainList[i].getId(), " and T", int(self.__trainList[i].getId())+1, ": ", self.__distances[i], "m.")
            else:
                print("---T", self.__trainList[i].getId(), " and T", int(self.__trainList[i].getId())+1, ": undefined.")

    # Read the acceleration and deceleration of a trains in the simulation
    def _readAccelDecel(self, train):
        train.setAccel(traci.vehicle.getAccel(train.getId()))
        train.setDecel(traci.vehicle.getDecel(train.getId()))

    # Set the accelerations and decelerations of all trains in the simulation
    def _writeAccelDecel(self):
        for train in self.__trainList:
            traci.vehicle.setAccel(train.getId(), train.getAccel())
            traci.vehicle.setDecel(train.getId(), train.getDecel())

    # Method to update the list of active trains.
    def _updateTrainsActive(self):
        idTrains = traci.vehicle.getIDList()
        for train in self.__trainList:
            thereis = False
            for id in idTrains:
                if train.getId().__eq__(id):
                    thereis = True
                    self._readAccelDecel(train)
            if thereis == False:
                self.__trainList.remove(train)
                self.__decouplingTrain.pop(0)
                self.__couplingTrain.pop(0)
                self.__state.pop(0)
                self.__isBraking.pop(0)
                self.__countDisconnection.pop(0)
    
    def _updateOldSpeed(self):
        self.__oldSpeed.clear()
        for train in self.__trainList:
            self.__oldSpeed.append(traci.vehicle.getSpeed(train.getId()))

    #Method to check if there are no trains in transit in a given road.
    def _freeRoad(self, roadToCheck):
        for train in self.__trainList:
            for roadId in roadToCheck:
                if traci.vehicle.getRoadID(train.getId()).__eq__(roadId):
                    return False
        return True

    def _addTrain(self):
        lastTrain = self.__trainList[-1]
        newTrain = Train(str(int(lastTrain.getId())+1), lastTrain.getDefaultSpeed(),
                         DEFAULT_ACCEL, DEFAULT_DECEL)
        self.__trainList.append(newTrain)
        traci.vehicle.setSpeed(newTrain.getId(), newTrain.getSpeed())
        self.__oldSpeed.append(0.0)
        self.__couplingTrain.append(True)
        self.__decouplingTrain.append(False)
        self.__isBraking.append(False)
        self.__state.append("decoupled")
        self.__countDisconnection.append(0)
        self._setSameSpeedFactores(newTrain, lastTrain)

    def _trainIsInRoadID(self, train, roadIDs):
        for roadID in roadIDs:
            if traci.vehicle.getRoadID(train.getId()).__eq__(roadID):
                return True
        return False

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
        roadIDs = ["E0","E6","E7","E8","E9","E10","E11","E12","E13"] #RoadIDs where has to be the trainAhead
        if traci.vehicle.getRoadID(train.getId()).__eq__("E5"):
            if (roadFree and (self._trainIsInRoadID(trainAhead, roadIDs))):
                if train.getSpeed() < train.getDefaultSpeed():
                    traci.vehicle.setSpeed(str(idTrain), train.getDefaultSpeed()+1)
                    train.setSpeed(train.getDefaultSpeed()+1)
                    traci.vehicle.setSpeedMode(train.getId(), 30)
            elif traci.vehicle.getRoadID(train.getId()).__eq__("E55"):
                traci.vehicle.setSpeed(str(idTrain), 0.0)
                train.setSpeed(0.0)
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
                    traci.vehicle.setSpeedMode(train.getId(), 30)
            else: 
                traci.vehicle.setSpeed(str(idTrain), 0.0)
                train.setSpeed(0.0)

    # Check if the next Train is already loaded from SUMO.
    def _isTrainLoaded(self):
        idTrains = traci.vehicle.getIDList()
        idNextTrain = str(int(self.__trainList[-1].getId())+1)
        for id in idTrains:
            if id.__eq__(idNextTrain):
                return True
        return False

    def _toStringState(self, pos):
        return f"State T{self.__trainList[pos].getId()}-T{self.__trainList[pos+1].getId()}: {self.__state[pos]}; InCoupling: {self.__couplingTrain[pos]}; InDecoupling: {self.__decouplingTrain[pos]}."

    #Method to plot the distance graph between the first 2 trains.
    def _plotDist(self):
        steps = np.arange(0, self.__step, 1)
        for i in range(0,len(self.__distToPlot)):
            self.__distToPlot[i] = self.__distToPlot[i]*10
        plt.plot(steps, self.__distToPlot, label="T0 - T1")
        plt.xlabel('Time [s]')
        plt.ylabel('Distance [m]')
        plt.title('Virtual Coupling')
        plt.legend()
        plt.show()

    def run(self):
        traci.simulationStep()
        self._setInitialParameters()
        self.TRAINS_TO_WAIT = 10+math.floor(20-self.__trainList[0].getDefaultSpeed())
        for train in self.__trainList:
            self.__oldSpeed.append(0)
            traci.vehicle.setSpeed(train.getId(), train.getDefaultSpeed())
        
        firstTrain = self.__trainList[0]
        #Delete the limit on the distance between vehicles imposed by SUMO
        traci.vehicle.setSpeedMode(firstTrain.getId(), 29)
        #Start the run of the trains
        while traci.simulation.getMinExpectedNumber() > 0:
            print("\n-----------Step ", self.__step)
            if len(traci.vehicle.getIDList()) > 1:
                self._updateTrainsActive()
                #Check if there is an incoming train
                if self.__incomingTrains > 0 :
                    if self._isTrainLoaded():
                        self._addTrain()
                        self.__incomingTrains -= 1
                self.__distances.clear()
                #Update follower list
                for train in self.__trainList:
                    distance = traci.vehicle.getFollower(train.getId(), 0) #getFollower() -> [idFollower, distance]
                    self.__distances.append(distance[1])
                self.printDistances()
                print("\n")
                #Change the limit on the distance for a train that is in the right roadID
                for train in self.__trainList:
                    if train.getId().__eq__("0"):
                        if traci.vehicle.getRoadID(train.getId()).__eq__("E13"):
                            traci.vehicle.setSpeed(train.getId(), train.getDefaultSpeed())
                            train.setSpeed(train.getDefaultSpeed())
                            traci.vehicle.setSpeedMode(train.getId(), 30)
                    elif traci.vehicle.getRoadID(train.getId()).__eq__("E6"):
                        traci.vehicle.setSpeedMode(train.getId(), 30)
                
                stepsToWait = 250 #steps to wait before the first change of a train's direction
                if self.__trainList[0].getDefaultSpeed() <= 15: 
                    stepsToWait = 250+25*self.__factorSpeed
                if self.__step > stepsToWait:
                    road1 = "E30" #roadID where send the comand to the first train
                    road2 = "E31" #roadID where send the comand to the second train
                    if self.__variant:
                        road1 = "E17"
                        road2 = "E18"
                    if traci.vehicle.getRoadID(self.__trainList[0].getId()).__eq__(road1): 
                        print("\n### Set change of direction for Train", self.__trainList[0].getId())
                        traci.vehicle.changeTarget(self.__trainList[0].getId(), "E48")
                        self.__decouplingTrain[0] = True
                        self.__couplingTrain[0] = False
                    if len(self.__trainList)>2:
                        if traci.vehicle.getRoadID(self.__trainList[1].getId()).__eq__(road2): 
                            print("\n### Set change of direction for Train", self.__trainList[1].getId())
                            traci.vehicle.changeTarget(self.__trainList[1].getId(), "E51")
                            self.__decouplingTrain[1] = True
                            self.__couplingTrain[1] = False
                self.printAllSpeed()
                for i in range(0, len(self.__trainList)-1):
                    if self.__decouplingTrain[i] == True:
                        self.__decouplingTrain[i] = self._stepDecoupling(i)
                    elif self.__couplingTrain[i] == True:
                        self.__couplingTrain[i] = self._stepCoupling(i)
                    elif not self.__state[i].__eq__("decoupled"):
                        self._stepHoldState(i)
                    print(self._toStringState(i))
                self._updateOldSpeed()
                #If there are more than a cert amount of trains, the new train has to wait the last to enter the circuit
                for i in range(1, len(self.__trainList)):
                    self._controlTrainIncoming(i)
                if self.__incomingTrains > 0: print("#\nIncoming trains:",self.__incomingTrains)
                self._writeAccelDecel()
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