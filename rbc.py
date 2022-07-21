import math
from train import *

from ctypes.wintypes import INT
import os
import sys

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary
import traci

DEFAULT_SPEED = 20.8
MIN_DIST_COUP = 5
MAX_DIST_COUP = 40
MIN_DIST_DECOUP = 45
MAX_DIST_DECOUP = 150
MIN_SPEED = 10.0
MAX_SPEED = 30.0
PARAM_COUPLING = 5.5

class Rbc:
       
    def __init__(self, nTrain, DEPARTURE_INTERVAL):
        self.__distanceCoupling = 10
        self.__distanceDecoupling = 100
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

        for idTrain in range(0, 3):
            defaultSpeed = DEFAULT_SPEED - 0.8*idTrain
            train = Train(str(idTrain+1), defaultSpeed)
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

    def printAllSpeed(self):
        print("\nSpeeds:")
        pos = 0
        for train in self.__trainList:
            print("\nTrain", train.getId(), ":", traci.vehicle.getSpeed(train.getId()))
            pos += 1

    def _updateOldSpeed(self):
        self.__oldSpeed.clear()
        for train in self.__trainList:
            self.__oldSpeed.append(traci.vehicle.getSpeed(train.getId()))

    def _setSameSpeedFactores(self, trainFollower, trainAhead):
        traci.vehicle.setSpeedFactor(trainFollower, traci.vehicle.getSpeedFactor(trainAhead))
        traci.vehicle.setAccel(trainFollower, traci.vehicle.getAccel(trainAhead))
        traci.vehicle.setDecel(trainFollower, traci.vehicle.getAccel(trainAhead))

    def _stepDecoupling(self, pos):
        trainAhead = self.__trainList[pos]
        trainFollower = self.__trainList[pos+1]
        if self.__distances[pos] < self.__distanceDecoupling:    
            trainFollowerSpeed = traci.vehicle.getSpeed(trainFollower.getId())
            traci.vehicle.setSpeed(trainFollower.getId(), trainFollowerSpeed - 1)
            trainFollower.setSpeed(trainFollowerSpeed - 1)
            print("\nIn decoupling, Train", trainFollower.getId(), "is decreasing speed.")
            self.__state[pos] = "almost_decoupled"
            posFollower = pos+1
            while posFollower < len(self.__trainList)-1:
                idNext = self.__trainList[posFollower+1].getId()
                #speedAhead = traci.vehicle.getSpeed(self.__trainList[posFollower].getId())
                speedAhead = self.__trainList[posFollower].getSpeed()
                if self.__state[posFollower].__eq__("coupled"):
                    traci.vehicle.setSpeed(idNext, speedAhead-0.1)
                    self.__trainList[posFollower+1].setSpeed(speedAhead-0.1)
                    print("\nIn decoupling, Train", idNext, "is decreasing speed.")
                elif self.__state[posFollower].__eq__("almost_coupled"):
                    if traci.vehicle.getSpeed(idNext) > 2.99:
                        traci.vehicle.setSpeed(idNext, speedAhead - 2)
                        self.__trainList[posFollower+1].setSpeed(speedAhead - 2)
                    elif traci.vehicle.getSpeed(idNext) > 0.65:
                        traci.vehicle.setSpeed(idNext, speedAhead - 0.55)
                        self.__trainList[posFollower+1].setSpeed(speedAhead - 0.55)
                    print("\nIn decoupling, Train", idNext, "is decreasing speed.")
                posFollower += 1
        else:
            #Now trains are decoupled and they have to remain decoupled
            if trainAhead.getDefaultSpeed() < trainFollower.getDefaultSpeed():
                traci.vehicle.setSpeed(trainFollower.getId(), trainAhead.getDefaultSpeed())
                trainFollower.setSpeed(trainAhead.getDefaultSpeed())
            else:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollower.getDefaultSpeed())
                trainFollower.setSpeed(trainFollower.getDefaultSpeed())
            #Reset the speed of all the trains coupled with the follower train
            for idFollower in range(int(trainFollower.getId()), len(self.__trainList)):
                if self.__state[idFollower-1].__eq__("coupled") or self.__state[idFollower-1].__eq__("almost_coupled"):
                    traci.vehicle.setSpeed(str(idFollower+1), trainFollower.getSpeed())
                    self.__trainList[idFollower].setSpeed(trainFollower.getSpeed())
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
        trainSpeed = traci.vehicle.getSpeed(trainAhead.getId())
        trainFollowerSpeed = traci.vehicle.getSpeed(trainFollower.getId())
        speedDiff = trainFollowerSpeed - trainSpeed
        #The follower train is coming 
        if traci.vehicle.getRoadID(trainFollower.getId()).__eq__("E3") or traci.vehicle.getRoadID(trainFollower.getId()).__eq__("E5"):
            self._setSameSpeedFactores(trainFollower.getId(), trainAhead.getId())
            return True
        if self.__state[pos].__eq__("almost_coupled"):    
            #Check if there is a train ahead that is decoupling.
            if self._trainAheadDecoupling(pos+1):
                return True
        #Check if there is a connection problem
        if self.__distances[pos] == -1 and (not traci.vehicle.getRoadID(trainFollower.getId()).__eq__("E2")): 
            if self.__countDisconnection[pos] < 6:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollowerSpeed-1)
                trainFollower.setSpeed(trainFollowerSpeed-1)
                traci.vehicle.setDecel(trainFollower.getId(), 
                                       traci.vehicle.getDecel(trainFollower.getId())+0.02)
                self.__countDisconnection[pos] += 1
            else:
                traci.vehicle.setSpeed(trainFollower.getId(), self.__trainList[pos].getDefaultSpeed())
                trainFollower.setSpeed(self.__trainList[pos].getDefaultSpeed())
                traci.vehicle.setDecel(trainFollower.getId(), 0.7)
            return True
        self.__countDisconnection[pos] = 0
        
        if self.__distances[pos] >= self.__distanceCoupling*PARAM_COUPLING and trainFollowerSpeed <= MAX_SPEED-1:
            if speedDiff < 4:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollowerSpeed+1)
                trainFollower.setSpeed(trainFollowerSpeed+1)
                print("\nTrain", trainFollower.getId(), ": increasing speed.")
            else:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollowerSpeed-1)
                trainFollower.setSpeed(trainFollowerSpeed-1)
                print("\nTrain", trainFollower.getId(), "isn't increasing is speed.")
        elif self.__distances[pos] > self.__distanceCoupling+1:
            if self.__state[pos].__eq__("decoupled") and self.__distances[pos] < self.__distanceCoupling*PARAM_COUPLING:
                self.__state[pos] = "almost_coupled"
            if speedDiff > 4.5:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollowerSpeed-5)
                trainFollower.setSpeed(trainFollowerSpeed-5)
                traci.vehicle.setDecel(trainFollower.getId(), traci.vehicle.getDecel(trainFollower.getId())+0.1)
            elif speedDiff > 2.5:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollowerSpeed-2.5)
                trainFollower.setSpeed(trainFollowerSpeed-2.5)
            elif speedDiff > 1.5:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollowerSpeed-1.3)
                trainFollower.setSpeed(trainFollowerSpeed-1.3)
            elif speedDiff > 1:
                traci.vehicle.setSpeed(trainFollower.getId(), trainFollowerSpeed-0.7)
                trainFollower.setSpeed(trainFollowerSpeed-0.7)
            print("\nIn coupling, Train", trainFollower.getId(), "is decreasing his speed.")
        elif self.__distances[pos] > 0:
            print("\n\nCOUPLING COMPLETED BETWEEN T", trainAhead.getId(), " AND T", trainFollower.getId(), "\n")
            traci.vehicle.setSpeed(trainFollower.getId(), trainSpeed)
            trainFollower.setSpeed(trainSpeed)
            traci.vehicle.setDecel(trainFollower.getId(), 0.7)
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

    def printDistances(self):
        print("\n-Distance between trains: ")
        i = 0
        for train in self.__trainList:
            if i != len(self.__trainList)-1:
                print("\n---T", train.getId(), " and T", int(train.getId())+1, ": ", self.__distances[i], "m")
            i += 1

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
    
    def _addTrain(self):
        thirdTrain = self.__trainList[2]
        newTrain = Train(str(len(self.__trainList)+1), thirdTrain.getDefaultSpeed())
        self.__trainList.append(newTrain)
        traci.vehicle.setSpeed(str(len(self.__trainList)), newTrain.getSpeed())
        self.__oldSpeed.append(0.0)
        self.__couplingTrain.append(True)
        self.__decouplingTrain.append(False)
        self.__isBraking.append(False)
        self.__state.append("decoupled")
        self.__countDisconnection.append(0)
        self._setSameSpeedFactores(str(len(self.__trainList)), str(len(self.__trainList)-1))

    def run(self):
        traci.simulationStep()
        self._setInitialParameters()
        step = 1 #step of the simulation
        for train in self.__trainList:
            self.__oldSpeed.append(0)
        for train in self.__trainList:
            traci.vehicle.setSpeed(train.getId(), train.getDefaultSpeed())
        #Delete the limit on the distance between vehicles imposed by SUMO
        firstTrain = self.__trainList[0]
        traci.vehicle.setSpeedMode(firstTrain.getId(), 29)
        #Start the run of the trains
        while traci.simulation.getMinExpectedNumber() > 0:
            print("\n-----Step ", step)
            if len(traci.vehicle.getIDList()) > 1:
                self._updateTrainsActive()
                #Check if there is an incoming train
                if self.__incomingTrains > 0:
                    if step > self.DEPARTURE_INTERVAL-1 and (step%self.DEPARTURE_INTERVAL == 0):
                        self._addTrain()
                        self.__incomingTrains -= 1
                self.__distances.clear()
                #update follower list
                for train in self.__trainList:
                    distance = traci.vehicle.getFollower(train.getId(), 0) #[idFollower, distance]
                    self.__distances.append(distance[1])
                self.printDistances()
                
                for train in self.__trainList:
                    #Don't delete the limit on the distance if they are not in the right position
                    if train.getId().__eq__("1"):
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
                """
                if (traci.vehicle.getRoadID(self.__trainList[0].getId()).__eq__("E36")): 
                    print("\n######### Set change of direction for the FIRST TRAIN.")
                    traci.vehicle.changeTarget(self.__trainList[0].getId(), "E43")
                    self.__decouplingTrain[0] = True
                    self.__couplingTrain[0] = False
                
                if step == 2000: 
                    print("\n######### Set change of direction for Train 1.")
                    traci.vehicle.changeTarget("1", "E43")
                    self.__decouplingTrain[0] = True
                    self.__couplingTrain[0] = False
                
                if step == 2020:
                    for train in self.__trainList:
                        traci.vehicle.changeTarget(train.getId(), "E43")
                        self.__decouplingTrain[0] = True
                        self.__couplingTrain[0] = False
                
                if step == 225: 
                    print("\n######### Set change of direction for Train", self.__trainList[1].getId())
                    traci.vehicle.changeTarget(self.__trainList[1].getId(), "E48")
                    self.__decouplingTrain[1] = True
                    self.__couplingTrain[1] = False
                """
                if traci.vehicle.getRoadID(self.__trainList[0].getId()).__eq__("E39"): 
                    print("\n######### Set change of direction for Train", self.__trainList[0].getId())
                    traci.vehicle.changeTarget(self.__trainList[0].getId(), "E48")
                    self.__decouplingTrain[0] = True
                    self.__couplingTrain[0] = False
                
                if len(self.__trainList)>2:
                    if traci.vehicle.getRoadID(self.__trainList[1].getId()).__eq__("E40"): 
                        print("\n######### Set change of direction for Train", self.__trainList[1].getId())
                        traci.vehicle.changeTarget(self.__trainList[1].getId(), "E51")
                        self.__decouplingTrain[1] = True
                        self.__couplingTrain[1] = False
                elif len(self.__trainList) == 1:
                    print("\n######### Set change of direction for Train", self.__trainList[0].getId())
                    traci.vehicle.changeTarget(self.__trainList[1].getId(), "E48")
                    self.__decouplingTrain[0] = True
                    self.__couplingTrain[0] = False
                
                for i in range(0, len(self.__trainList)-1):
                    #Look if there are trains in decoupling mode
                    if self.__decouplingTrain[i] == True:
                        self.__decouplingTrain[i] = self._stepDecoupling(i)
                    #Look if there are trains in coupling mode
                    elif self.__couplingTrain[i] == True:
                        self.__couplingTrain[i] = self._stepCoupling(i)
                    elif not self.__state[i].__eq__("decoupled"):
                        self._stepHoldState(i)
                
                    print("\n## State",i+1,":",self.__state[i],"## InCoupling",i+1,":",self.__couplingTrain[i],
                          "## InDecoupling",i+1,":", self.__decouplingTrain[i])
                self._updateOldSpeed()
                self.printAllSpeed()
            traci.simulationStep()
            step += 1
        print("\n\nSimulation completed.")
        traci.close()
        sys.stdout.flush()