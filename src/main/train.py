__author__ = "Matteo Spataro"
__license__ = "Eclipse Public License"
__version__ = "2.0"
__maintainer__ = "Matteo Spataro"
__email__ = "matteo.spataro@stud.unifi.it"

class Train:
    def __init__(self, id, speed, accel, decel):
        self.__id = id
        self.__speed = speed
        self.__defaultSpeed = speed
        self.__accel = accel
        self.__decel = decel

    def getId(self):
        return self.__id

    def getSpeed(self):
        return self.__speed

    def getDefaultSpeed(self):
        return self.__defaultSpeed

    def getAccel(self):
        return self.__accel

    def getDecel(self):
        return self.__decel

    def setSpeed(self, speed):
        self.__speed = speed

    def setAccel(self, accel):
        self.__accel = accel

    def setDecel(self, decel):
        self.__decel = decel

    def setDefaultSpeed(self, speed):
        self.__defaultSpeed = speed