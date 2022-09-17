from ctypes.wintypes import INT
import os
import sys
import random

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary
import traci


class Channel:

    def __init__(self):
        self.__noise = 0.0001 #failure rate
        self.errors = 0

    def setSpeedMode(self, vehID, speedMode):
        if random.random() > self.__noise:
            traci.vehicle.setSpeedMode(vehID, speedMode)
        self.errors += 1

    def setSpeedFactor(self, trainFollower, trainAhead):
        if random.random() > self.__noise:
            traci.vehicle.setSpeedFactor(trainFollower, trainAhead)
        self.errors += 1

    def getSpeedFactor(self, train):
        if random.random() > self.__noise:
            return traci.vehicle.getSpeedFactor(train)
        self.errors += 1
        return 0

    def setAccel(self, trainFollower, trainAhead):
        if random.random() > self.__noise:
            traci.vehicle.setAccel(trainFollower, trainAhead)
        self.errors += 1

    def setDecel(self, trainFollower, trainAhead):
        if random.random() > self.__noise:
            traci.vehicle.setDecel(trainFollower, trainAhead)
        self.errors += 1

    def setSpeed(self, trainFollower, trainAhead):
        if random.random() > self.__noise:
            traci.vehicle.setSpeed(trainFollower, trainAhead)
        self.errors += 1

    def getAccel(self, train):
        if random.random() > self.__noise:
            return traci.vehicle.getAccel(train)
        self.errors += 1
        return 00.0

    def getDecel(self, train):
        if random.random() > self.__noise:
            return traci.vehicle.getDecel(train)
        self.errors += 1
        return 0.0

    def getSpeed(self, train):
        if random.random() > self.__noise:
            return traci.vehicle.getSpeed(train)
        self.errors += 1
        return 0.0

    def changeTarget(self, train, edgeID):
        if random.random() > self.__noise:
            traci.vehicle.changeTarget(train, edgeID)
        self.errors += 1

    def setLine(self, vehID, line):
        if random.random() > self.__noise:
            traci.vehicle.setLine(vehID, line)
        self.errors += 1

    def getFollower(self, train, dist):
        return traci.vehicle.getFollower(train, dist)
    def getRoute(self, vehID):
        return traci.vehicle.getRoute(vehID)
    def getIDList(self):
        return traci.vehicle.getIDList()
    def getRoadID(self, train):
        return traci.vehicle.getRoadID(train)