class Train:
    def __init__(self, id, speed):
        self.__id = id
        self.__speed = speed
        self.__defaultSpeed = speed

    def getId(self):
        return self.__id

    def getSpeed(self):
        return self.__speed

    def getDefaultSpeed(self):
        return self.__defaultSpeed

    def setSpeed(self, speed):
        self.__speed = speed

    def setDefaultSpeed(self, speed):
        self.__defaultSpeed = speed