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

__author__ = "Matteo Spataro"
__license__ = "Eclipse Public License"
__version__ = "2.0"
__maintainer__ = "Matteo Spataro"
__email__ = "matteo.spataro@stud.unifi.it"

class Channel:

    def __init__(self):
        self.__noise = 0.0001 #failure rate
        self.__errors = 0

    def getNoise(self):
        return self.__noise

    def getErrors(self):
        return self.__errors

    def setSpeedMode(self, vehID, speedMode):
        if random.random() > self.__noise:
            traci.vehicle.setSpeedMode(vehID, speedMode)
        else: self.__errors += 1

    def setSpeedFactor(self, trainFollower, trainAhead):
        if random.random() > self.__noise:
            traci.vehicle.setSpeedFactor(trainFollower, trainAhead)
        else: self.__errors += 1

    def getSpeedFactor(self, train):
        if random.random() > self.__noise:
            return traci.vehicle.getSpeedFactor(train)
        self.__errors += 1
        return 0

    def setAccel(self, trainFollower, trainAhead):
        if random.random() > self.__noise:
            traci.vehicle.setAccel(trainFollower, trainAhead)
        else: self.__errors += 1

    def setDecel(self, trainFollower, trainAhead):
        if random.random() > self.__noise:
            traci.vehicle.setDecel(trainFollower, trainAhead)
        else: self.__errors += 1

    def setSpeed(self, trainFollower, trainAhead):
        if random.random() > self.__noise:
            traci.vehicle.setSpeed(trainFollower, trainAhead)
        else: self.__errors += 1

    def getAccel(self, train):
        if random.random() > self.__noise:
            return traci.vehicle.getAccel(train)
        self.__errors += 1
        return 0.0

    def getDecel(self, train):
        if random.random() > self.__noise:
            return traci.vehicle.getDecel(train)
        self.__errors += 1
        return 0.0

    def getSpeed(self, train):
        if random.random() > self.__noise:
            return traci.vehicle.getSpeed(train)
        self.__errors += 1
        return 0.0

    def changeTarget(self, train, edgeID):
        if random.random() > self.__noise:
            traci.vehicle.changeTarget(train, edgeID)
        else: self.__errors += 1

    def setLine(self, vehID, line):
        if random.random() > self.__noise:
            traci.vehicle.setLine(vehID, line)
        else: self.__errors += 1

    def getFollower(self, train, dist):
        return traci.vehicle.getFollower(train, dist)
    def getRoute(self, vehID):
        return traci.vehicle.getRoute(vehID)
    def getIDList(self):
        return traci.vehicle.getIDList()
    def getRoadID(self, train):
        return traci.vehicle.getRoadID(train)