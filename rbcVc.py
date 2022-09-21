from ast import If
import math
from train import *

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

MIN_DIST_COUP = 5
MAX_DIST_COUP = 40
MIN_DIST_DECOUP = 45
MAX_DIST_DECOUP = 150
PARAM_COUPLING = 5.5
DEFAULT_SPEED = 20.8
MIN_SPEED = 10.0 #equals to 100 km/h
MAX_SPEED = 30.0 #equals to 300 km/h
DEFAULT_DECEL = 0.7 #m/s^2
MAX_DECEL = 0.9 #m/s^2
MAX_DISCONNECTIONS = 6

class RbcVC(Rbc):
       
    def __init__(self, nTrain, DEPARTURE_INTERVAL):
        self.__distanceCoupling = 10 #equals to 100 m
        self.__distanceDecoupling = 100 #equals to 1 km
        self.__trainList = [] #List of active trains
        self.__distances = [] #List with the distances between trains
        self.__oldSpeed = [] #List with the speeds of the trains in the previous step
        self.__couplingTrain = [True] #Train is trying to reach the coupling if is "True"
        self.__decouplingTrain = [False] #Train is trying to reach the decoupling if is "True"
        self.__state = ["decoupled"] #List with the state of each train
        self.__isBraking = [False] #Is "True" if the train ahead is braking
        self.__incomingTrains = 0 #Number of trains that are coming
        self.__countDisconnection = [0] #Number of sequential disconnections for each train
        self.DEPARTURE_INTERVAL = DEPARTURE_INTERVAL
        self.__factorSpeed = 6
        self.__step = 1 #step of the simulation

        for idTrain in range(0, 3):
            defaultSpeed = DEFAULT_SPEED - 0.8*idTrain
            train = Train(str(idTrain), defaultSpeed)
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
        for train in self.__trainList:
            print("--Train", train.getId(), ":", round(traci.vehicle.getSpeed(train.getId()),6),
                  " # Decel:", round(traci.vehicle.getDecel(train.getId()),4))

    def _updateOldSpeed(self):
        self.__oldSpeed.clear()
        for train in self.__trainList:
            self.__oldSpeed.append(traci.vehicle.getSpeed(train.getId()))

    def _setSameSpeedFactores(self, trainFollower, trainAhead):
        traci.vehicle.setSpeedFactor(trainFollower, traci.vehicle.getSpeedFactor(trainAhead))
        traci.vehicle.setAccel(trainFollower, traci.vehicle.getAccel(trainAhead))
        traci.vehicle.setDecel(trainFollower, traci.vehicle.getDecel(trainAhead))

    def _stepDecoupling(self, pos):
        trainAhead = self.__trainList[pos]
        trainFollower = self.__trainList[pos+1]
        if self.__distances[pos] == -1: return True
        if self.__distances[pos] < self.__distanceDecoupling:    
            trainFollowerSpeed = traci.vehicle.getSpeed(trainFollower.getId())
            traci.vehicle.setSpeed(trainFollower.getId(), trainFollowerSpeed - 1)
            trainFollower.setSpeed(trainFollowerSpeed - 1)
            print("In decoupling, Train", trainFollower.getId(), "is decreasing speed.")
            self.__state[pos] = "almost_decoupled"
            posAhead = pos+1
            while posAhead < len(self.__trainList)-1:
                idFollower = self.__trainList[posAhead+1].getId()
                speedAhead = traci.vehicle.getSpeed(self.__trainList[posAhead].getId())
                if self.__state[posAhead].__eq__("almost_coupled") or self.__state[posAhead].__eq__("coupled"):
                    newSpeed = 0.1
                    if self.__distances[posAhead] == -1: return True
                    if self.__distances[posAhead] < self.__distanceCoupling:
                        newSpeed = speedAhead*0.40
                        if traci.vehicle.getDecel(idFollower) < MAX_DECEL:
                            traci.vehicle.setDecel(idFollower, traci.vehicle.getDecel(idFollower)+0.05)
                        print(" Train", idFollower, "is decreasing speed.")
                    if self.__distances[posAhead] < self.__distanceCoupling+self.__distanceCoupling*0.25:
                        newSpeed = speedAhead*0.50
                        if traci.vehicle.getDecel(idFollower) < MAX_DECEL:
                            traci.vehicle.setDecel(idFollower, traci.vehicle.getDecel(idFollower)+0.05)
                        print(" Train", idFollower, "is decreasing speed.")
                    elif self.__distances[posAhead] < self.__distanceCoupling+self.__distanceCoupling*0.70:
                        newSpeed = speedAhead*0.55
                        if traci.vehicle.getDecel(idFollower) < MAX_DECEL:
                                traci.vehicle.setDecel(idFollower, traci.vehicle.getDecel(idFollower)+0.05)
                        print(" Train", idFollower, "is decreasing speed.")
                    elif traci.vehicle.getSpeed(idFollower)-speedAhead >= 2:
                        newSpeed = speedAhead*0.60
                        if traci.vehicle.getDecel(idFollower) < MAX_DECEL:
                            traci.vehicle.setDecel(idFollower, traci.vehicle.getDecel(idFollower)+0.05)
                        print(" Train", idFollower, "is decreasing speed.")
                    elif traci.vehicle.getSpeed(idFollower)-speedAhead >= 1:
                        newSpeed = speedAhead*0.80
                        if traci.vehicle.getDecel(idFollower) < MAX_DECEL:
                            traci.vehicle.setDecel(idFollower, traci.vehicle.getDecel(idFollower)+0.05)
                        print(" Train", idFollower, "is decreasing speed.")
                    elif traci.vehicle.getSpeed(idFollower)-speedAhead >= 0.5:
                        newSpeed = speedAhead*0.90
                        if traci.vehicle.getDecel(idFollower) < MAX_DECEL:
                            traci.vehicle.setDecel(idFollower, traci.vehicle.getDecel(idFollower)+0.05)
                        print(" Train", idFollower, "is decreasing speed.")
                    else:
                        newSpeed = speedAhead
                        print(" Train", idFollower, "is decreasing speed.")
                    traci.vehicle.setSpeed(idFollower, newSpeed)
                    self.__trainList[posAhead+1].setSpeed(newSpeed)
                    if self.__state[posAhead].__eq__("coupled"):
                        self.__state[posAhead] = "almost_coupled"
                posAhead += 1
        else:
            #Now trains are decoupled and they have to remain decoupled
            if trainAhead.getDefaultSpeed() < trainFollower.getDefaultSpeed():
                traci.vehicle.setSpeed(trainFollower.getId(), trainAhead.getDefaultSpeed())
                trainFollower.setSpeed(trainAhead.getDefaultSpeed())
            else:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollower.getDefaultSpeed())
                trainFollower.setSpeed(trainFollower.getDefaultSpeed())
            #Reset the speed of all the trains coupled or almost coupled with the follower train
            for idFollower in range(int(trainFollower.getId()), len(self.__trainList)):
                if self.__state[idFollower-1].__eq__("coupled") or self.__state[idFollower-1].__eq__("almost_coupled"):
                    traci.vehicle.setSpeed(str(idFollower+1), trainFollower.getSpeed())
                    self.__trainList[idFollower].setSpeed(trainFollower.getSpeed())
                    if traci.vehicle.getDecel(str(idFollower+1)) > DEFAULT_DECEL:
                        traci.vehicle.setDecel(str(idFollower+1), DEFAULT_DECEL)
                    print("\nIn decoupling, Train ", idFollower+1, "reset the speed.")
            print("\n\nDECOUPLING COMPLETED BETWEEN T", trainAhead.getId(), " AND T", trainFollower.getId(), "\n")
            self.__state[pos] = "decoupled"
            return False
        return True

    #Check if the train ahead is decoupling: in this case we can't modify the speed of the following trains 
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

    def _stepCoupling(self, pos):
        trainAhead = self.__trainList[pos]
        trainFollower = self.__trainList[pos+1]
        trainAheadSpeed = traci.vehicle.getSpeed(trainAhead.getId())
        trainFollowerSpeed = traci.vehicle.getSpeed(trainFollower.getId())
        speedDiff = trainFollowerSpeed - trainAheadSpeed
        #The follower train is coming 
        if traci.vehicle.getRoadID(trainFollower.getId()).__eq__("E3") or traci.vehicle.getRoadID(trainFollower.getId()).__eq__("E5"):
            self._setSameSpeedFactores(trainFollower.getId(), trainAhead.getId())
            return True
        if self.__state[pos].__eq__("almost_coupled"):    
            #Check if there is a train ahead that is decoupling.
            if self._trainAheadDecoupling(pos+1):
                print("Train ", trainFollower.getId(), "skip step coupling.")
                return True
        #Check if there is a connection problem
        if self.__distances[pos] == -1 and (not traci.vehicle.getRoadID(trainFollower.getId()).__eq__("E2")): 
            if self.__countDisconnection[pos] < MAX_DISCONNECTIONS:
                #There are connectivity problems in some roads.
                traci.vehicle.setSpeed(trainFollower.getId(), trainAheadSpeed)
                trainFollower.setSpeed(trainAheadSpeed)
                traci.vehicle.setDecel(trainFollower.getId(), 
                                           traci.vehicle.getDecel(trainFollower.getId())+0.02)
                self.__countDisconnection[pos] += 1
            else:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollower.getDefaultSpeed()*1.35)
                trainFollower.setSpeed(trainFollower.getDefaultSpeed()*1.35)
                traci.vehicle.setDecel(trainFollower.getId(), traci.vehicle.getDecel(trainFollower.getId()))
            return True
        self.__countDisconnection[pos] = 0
        traci.vehicle.setDecel(trainFollower.getId(), DEFAULT_DECEL)

        if self.__distances[pos] >= self.__distanceDecoupling and trainFollowerSpeed < MAX_SPEED-1:
            if speedDiff < 3.2:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollowerSpeed+1)
                trainFollower.setSpeed(trainFollowerSpeed+1)
                print(" In coupling, Train", trainFollower.getId(), ": increasing speed.")
            else:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollowerSpeed-1)
                trainFollower.setSpeed(trainFollowerSpeed-1)
                print(" In coupling, Train", trainFollower.getId(), "is decreasing his speed.")
        elif self.__distances[pos] >= self.__distanceCoupling*PARAM_COUPLING + 5*self.__factorSpeed:
            if speedDiff < 2:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollowerSpeed+1)
                trainFollower.setSpeed(trainFollowerSpeed+1)
                print(" In coupling, Train", trainFollower.getId(), ": increasing speed.")
            else:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollowerSpeed-1)
                trainFollower.setSpeed(trainFollowerSpeed-1)
                print(" In coupling, Train", trainFollower.getId(), "is decreasing his speed.")
        elif self.__distances[pos] > self.__distanceCoupling+self.__distanceCoupling*0.1:
            self.__state[pos] = "almost_coupled"
            if speedDiff > 4:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollowerSpeed-4)
                trainFollower.setSpeed(trainFollowerSpeed-4)
                if traci.vehicle.getDecel(trainFollower.getId()) < MAX_DECEL:
                    traci.vehicle.setDecel(trainFollower.getId(), traci.vehicle.getDecel(trainFollower.getId())+0.1)
                print(" In coupling, Train", trainFollower.getId(), "is decreasing his speed.")
            elif speedDiff > 2.5:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollowerSpeed-2.5)
                trainFollower.setSpeed(trainFollowerSpeed-2.5)
                print(" In coupling, Train", trainFollower.getId(), "is decreasing his speed.")
            elif speedDiff > 1.5:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollowerSpeed-1.3)
                trainFollower.setSpeed(trainFollowerSpeed-1.3)
                print(" In coupling, Train", trainFollower.getId(), "is decreasing his speed.")
            elif speedDiff > 1:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollowerSpeed-0.7)
                trainFollower.setSpeed(trainFollowerSpeed-0.7)
                print(" In coupling, Train", trainFollower.getId(), "is decreasing his speed.")
            if speedDiff>=0 and speedDiff<0.7 and self.__state[pos-1].__eq__("almost_coupled"):
                traci.vehicle.setSpeed(trainFollower.getId(), self.__trainList[pos].getSpeed()+0.3)
                trainFollower.setSpeed(self.__trainList[pos].getSpeed()+0.3)
                print(" In coupling, Train", trainFollower.getId(), "is increasing his speed.")
            
        elif self.__distances[pos] > 0:
            print("\n\nCOUPLING COMPLETED BETWEEN T", trainAhead.getId(), " AND T", trainFollower.getId(), "\n")
            traci.vehicle.setSpeed(trainFollower.getId(), trainAheadSpeed)
            trainFollower.setSpeed(trainAheadSpeed)
            traci.vehicle.setDecel(trainFollower.getId(), DEFAULT_DECEL)
            self._setSameSpeedFactores(trainFollower.getId(), trainAhead.getId())
            self.__state[pos] = "coupled"
            return False
        return True

    def _stepHoldState(self, pos):
        trainAhead = self.__trainList[pos]
        trainFollower = self.__trainList[pos+1]
        speedAhead = traci.vehicle.getSpeed(trainAhead.getId())
        speedFollower = traci.vehicle.getSpeed(trainFollower.getId())
        if self._trainAheadDecoupling(pos+1):
            print("Train ", trainFollower.getId(), "skip step hold state.")
            return
        if self.__oldSpeed[pos] > speedAhead:
            #the train ahead is decreasing his speed
            traci.vehicle.setSpeed(trainFollower.getId(), self.__oldSpeed[pos]-speedAhead - 2)
            trainFollower.setSpeed(self.__oldSpeed[pos]-speedAhead - 2)
            self._setSameSpeedFactores(trainFollower.getId(), trainAhead.getId())
            print("\nTrain",trainAhead.getId(),"is decreasing his speed.")
            self.__isBraking[pos] = True
            self.__state[pos] = "almost_coupled"
            return
        if self.__isBraking[pos] == True:
            #The train ahead is no more braking
            self.__isBraking[pos] = False 
            self.__couplingTrain[pos] = True #The trains must retrieve their coupling
            traci.vehicle.setSpeed(trainFollower.getId(), speedAhead)
            trainFollower.setSpeed(speedAhead)
            print("\nTrains T", trainAhead.getId(), "and T", trainFollower.getId(), "retrieve their coupling.")
            return
        if self.__distances[pos] == -1:
            traci.vehicle.setSpeed(trainFollower.getId(), speedAhead)
            trainFollower.setSpeed(speedAhead)
            return
        elif self.__distances[pos] < (self.__distanceCoupling/2.0 + 1):
            traci.vehicle.setSpeed(trainFollower.getId(), speedFollower-0.8)
            trainFollower.setSpeed(speedFollower-0.8)
            self.__state[pos] = "almost_coupled"
            self.__couplingTrain[pos] = True #The trains must retrieve their coupling
            return
        elif self.__distances[pos] > self.__distanceCoupling+1 and not(self.__state[pos].__eq__("decoupled")):
            traci.vehicle.setSpeed(trainFollower.getId(), speedFollower+0.1)
            trainFollower.setSpeed(speedFollower+0.1)
            self.__state[pos] = "almost_coupled"
            self.__couplingTrain[pos] = True #The trains must retrieve their coupling
            return
        traci.vehicle.setSpeed(trainFollower.getId(), speedAhead)
        trainFollower.setSpeed(speedAhead)
        self._setSameSpeedFactores(trainFollower.getId(), trainAhead.getId())
        if self.__state[pos].__eq__("almost_coupled"): self.__state[pos] = "coupled"

    def printDistances(self):
        print("\n-Distance between trains: ")
        for i in range(0, len(self.__trainList)-1):
            if self.__distances[i] != -1:
                print("---T", self.__trainList[i].getId(), " and T", int(self.__trainList[i].getId())+1, ": ", self.__distances[i], "m.")
            else:
                print("---T", self.__trainList[i].getId(), " and T", int(self.__trainList[i].getId())+1, ": undefined.")

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
                self.__decouplingTrain.pop(0)
                self.__couplingTrain.pop(0)
                self.__state.pop(0)
                self.__isBraking.pop(0)
                self.__countDisconnection.pop(0)
    
    def _freeRoad(self, roadToCheck):
        for train in self.__trainList:
            for roadId in roadToCheck:
                if traci.vehicle.getRoadID(train.getId()).__eq__(roadId):
                    return False
        return True

    def _addTrain(self):
        thirdTrain = self.__trainList[2]
        newTrain = Train(str(int(self.__trainList[-1].getId())+1), thirdTrain.getDefaultSpeed())
        self.__trainList.append(newTrain)
        traci.vehicle.setSpeed(newTrain.getId(), newTrain.getSpeed())
        self.__oldSpeed.append(0.0)
        self.__couplingTrain.append(True)
        self.__decouplingTrain.append(False)
        self.__isBraking.append(False)
        self.__state.append("decoupled")
        self.__countDisconnection.append(0)
        self._setSameSpeedFactores(str(len(self.__trainList)-1), str(len(self.__trainList)-2))

    def _toStringState(self, pos):
        return f"State T{self.__trainList[pos].getId()}-T{self.__trainList[pos+1].getId()}: {self.__state[pos]}; InCoupling: {self.__couplingTrain[pos]}; InDecoupling: {self.__decouplingTrain[pos]}."

    def run(self):
        traci.simulationStep()
        self._setInitialParameters()
        trainsToWait = 10+math.floor(20-self.__trainList[0].getDefaultSpeed())

        for train in self.__trainList:
            self.__oldSpeed.append(0)
        for train in self.__trainList:
            traci.vehicle.setSpeed(train.getId(), train.getDefaultSpeed())
        #Delete the limit on the distance between vehicles imposed by SUMO
        firstTrain = self.__trainList[0]
        traci.vehicle.setSpeedMode(firstTrain.getId(), 29)
        #Start the run of the trains
        while traci.simulation.getMinExpectedNumber() > 0:
            print("\n\n-------Step ", self.__step)
            if len(traci.vehicle.getIDList()) > 1:
                self._updateTrainsActive()
                #Check if there is an incoming train
                if self.__incomingTrains > 0 and self._freeRoad(["E3"]):
                    if self.__step > self.DEPARTURE_INTERVAL-1 and (self.__step%self.DEPARTURE_INTERVAL == 0):
                        self._addTrain()
                        self.__incomingTrains -= 1
                self.__distances.clear()
                #Update follower list
                for train in self.__trainList:
                    distance = traci.vehicle.getFollower(train.getId(), 0) #getFollower() -> [idFollower, distance]
                    self.__distances.append(distance[1])
                self.printDistances()
                print("#")
                for train in self.__trainList:
                    #Do not change the limit on the distance until they are not in the right position
                    if train.getId().__eq__("0"):
                        if (traci.vehicle.getRoadID(train.getId()).__eq__("E6") 
                        or traci.vehicle.getRoadID(train.getId()).__eq__("E7") 
                        or traci.vehicle.getRoadID(train.getId()).__eq__("E8")
                        or traci.vehicle.getRoadID(train.getId()).__eq__("E9")
                        or traci.vehicle.getRoadID(train.getId()).__eq__("E10")
                        or traci.vehicle.getRoadID(train.getId()).__eq__("E11")
                        or traci.vehicle.getRoadID(train.getId()).__eq__("E12")):
                            traci.vehicle.setSpeed(train.getId(), 15)
                            train.setSpeed(15)
                        else:
                            traci.vehicle.setSpeed(train.getId(), train.getDefaultSpeed())
                            train.setSpeed(train.getDefaultSpeed())
                            traci.vehicle.setSpeedMode(train.getId(), 30)
                    elif traci.vehicle.getRoadID(train.getId()).__eq__("E6"):
                        traci.vehicle.setSpeedMode(train.getId(), 30)
                
                stepsToWait = 250
                if self.__trainList[0].getDefaultSpeed() <= 15: 
                    stepsToWait = 250+25*self.__factorSpeed
                if self.__step > stepsToWait:
                    if traci.vehicle.getRoadID(self.__trainList[0].getId()).__eq__("E30"): 
                        print("\n##### Set change of direction for Train", self.__trainList[0].getId())
                        traci.vehicle.changeTarget(self.__trainList[0].getId(), "E48")
                        self.__decouplingTrain[0] = True
                        self.__couplingTrain[0] = False
                
                    if len(self.__trainList)>2:
                        if traci.vehicle.getRoadID(self.__trainList[1].getId()).__eq__("E31"): 
                            print("\n##### Set change of direction for Train", self.__trainList[1].getId())
                            traci.vehicle.changeTarget(self.__trainList[1].getId(), "E51")
                            self.__decouplingTrain[1] = True
                            self.__couplingTrain[1] = False
                
                for i in range(0, len(self.__trainList)-1):
                    #Look if there are trains in decoupling mode
                    if self.__decouplingTrain[i] == True:
                        self.__decouplingTrain[i] = self._stepDecoupling(i)
                    #Look if there are trains in coupling mode
                    elif self.__couplingTrain[i] == True:
                        self.__couplingTrain[i] = self._stepCoupling(i)
                    elif not self.__state[i].__eq__("decoupled"):
                        self._stepHoldState(i)
                    print(self._toStringState(i))
                print("#")
                #If there are more than a cert amount of trains, the next train has to wait the last to enter the circuit
                for train in self.__trainList:
                    idTrain = int(train.getId())
                    roadToCheck = ["E36","E35","E34","E33"]
                    if idTrain > trainsToWait:
                        if traci.vehicle.getRoadID(str(idTrain)).__eq__("E5"):
                            if ((traci.vehicle.getRoadID(str(idTrain-1)).__eq__("E0") or traci.vehicle.getRoadID(str(idTrain-1)).__eq__("E6"))
                                and self._freeRoad(roadToCheck)):
                                if train.getSpeed() < train.getDefaultSpeed():
                                    traci.vehicle.setSpeed(str(idTrain), train.getDefaultSpeed()+1)
                                    train.setSpeed(train.getDefaultSpeed()+1)
                                    traci.vehicle.setSpeedMode(train.getId(), 30)
                            else:
                                traci.vehicle.setSpeed(str(idTrain), train.getDefaultSpeed()*0.005)
                                train.setSpeed(train.getDefaultSpeed()*0.005)
                        if traci.vehicle.getRoadID(str(idTrain)).__eq__("E3"):
                            if self._freeRoad(["E5"]):
                                traci.vehicle.setSpeed(str(idTrain), train.getDefaultSpeed()*0.2)
                                train.setSpeed(train.getDefaultSpeed()*0.2)
                            else:
                                traci.vehicle.setSpeed(str(idTrain), traci.vehicle.getSpeed(str(idTrain-1)))
                                train.setSpeed(traci.vehicle.getSpeed(str(idTrain-1)))
                
                self._updateOldSpeed()
                self.printAllSpeed()
                if self.__incomingTrains > 0: print("Incoming trains:",self.__incomingTrains)
            else:
                self._updateTrainsActive()
                traci.vehicle.changeTarget(self.__trainList[0].getId(), "E48")
                traci.vehicle.setSpeed(self.__trainList[0].getId(), self.__trainList[0].getDefaultSpeed())
            traci.simulationStep()
            self.__step += 1
        print("\n\nSimulation completed.")
        traci.close()
        sys.stdout.flush()