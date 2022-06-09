import os
import sys
import optparse

DISTANCE_COUPLING = 10
DISTANCE_DECOUPLING = 100
DEFAULT_SPEED = 20
MIN_DIST_COUP = 5
MAX_DIST_COUP = 40
MIN_DIST_DECOUP = 45
MAX_DIST_DECOUP = 150
MIN_SPEED = 10.0
MAX_SPEED = 30.0
trainList = [["1",20.8],["2",20]] #List of active trains
distances = [] #List with the distances between trains
oldSpeed = [] #List with the speeds of the trains in the previous step
couplingTrain = [False] #Train is trying to reach the coupling if is "True"
decouplingTrain = [False] #Train is trying to reach the decoupling if is "True"
state = ["decoupled"]
isBraking = [False]

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary
import traci

def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true", default=False, 
                          help="Run the commandline version of SUMO.")
    opt_parser.add_option("--multiTrain", action="store_true", default=False, 
                          help="Run the simulation with a chosen the number of trains.")
    opt_parser.add_option("--setParam", action="store_true", default=False, 
                          help="Set the parameters of the simulation.")
    options, args = opt_parser.parse_args()
    return options

def stepDecoupling(idTrain):
    if distances[idTrain-1] < DISTANCE_DECOUPLING:    
        speedT2 = traci.vehicle.getSpeed(str(idTrain+1))
        traci.vehicle.setSpeed(str(idTrain+1), speedT2-1)
        print("\nIn decoupling, Train ", idTrain+1, "is decreasing speed\n")
        
        #Decrease the speed of all the trains coupled with the follower train
        for idFollower in range(idTrain+1,len(trainList)):
            if state[idFollower-1].__eq__("coupled"):
                traci.vehicle.setSpeed(str(idFollower+1), speedT2-1)
                print("\nIn decoupling, Train ", idFollower+1, "is decreasing speed\n")
    else:
        traci.vehicle.setSpeed(str(idTrain+1), DEFAULT_SPEED)
        print("\n\nDECOUPLING COMPLETED BETWEEN T", idTrain, " AND T", idTrain+1, "\n")
        state[idTrain-1] = "decoupled"
        #Set the speed of all the trains coupled with the follower train
        for idFollower in range(idTrain+1,len(trainList)):
            if state[idFollower-1].__eq__("coupled"):
                traci.vehicle.setSpeed(str(idFollower+1), DEFAULT_SPEED)
                print("\nIn decoupling, Train ", idFollower+1, "is increasing speed\n")
        return False
    return True

#Check if the train ahead is decoupling: in this case we can't modify the speed of the following trains 
#in order not to overwrite the changes made during the decoupling phase of the train ahead.
def trainAheadDecoupling(idTrain):
    firstTrain = trainList[0]
    if firstTrain[0].__eq__(str(idTrain)):
        return False
    if decouplingTrain[idTrain-2] == True:
        return True
    return False

def stepCoupling(idTrain):
    speedT1 = traci.vehicle.getSpeed(str(idTrain))
    speedT2 = traci.vehicle.getSpeed(str(idTrain+1))
    speedDiff = speedT2 - speedT1
    
    if trainAheadDecoupling(idTrain):
        if state[idTrain-1].__eq__("coupled"):
            return False
        else:
            return True
    if distances[idTrain-1] >= DISTANCE_COUPLING*5.50:
        print("\nTrain ", idTrain+1, ": increasing speed\n")
        if speedT2 <= 29 and speedDiff < 6: #Speed must be under 300 km/h
            traci.vehicle.setSpeed(str(idTrain+1), speedT2+1)
    elif distances[idTrain-1] > DISTANCE_COUPLING: 
        if speedDiff > 5:
            traci.vehicle.setSpeed(str(idTrain+1), speedT2-5)
        elif speedDiff > 2.5:
            traci.vehicle.setSpeed(str(idTrain+1), speedT2-2.5)
        elif speedDiff > 1.5:
            traci.vehicle.setSpeed(str(idTrain+1), speedT2-1)
        print("\nIn coupling, Train ", idTrain+1, ": decreasing speed\n")
    elif distances[idTrain-1] > 0:
        print("\n\nCOUPLING COMPLETED BETWEEN T", idTrain, " AND T", idTrain+1, "\n")
        traci.vehicle.setSpeed(str(idTrain+1), speedT1)
        traci.vehicle.setSpeedFactor(str(idTrain+1), traci.vehicle.getSpeedFactor(str(idTrain)))
        traci.vehicle.setAccel(str(idTrain+1), traci.vehicle.getAccel(str(idTrain)))
        state[idTrain-1] = "coupled"
        return False
    return True    

#Hold the current state of the trains
def stepHoldState(idTrain):
    speedT1 = traci.vehicle.getSpeed(str(idTrain))
    speedT2 = traci.vehicle.getSpeed(str(idTrain+1))
    if trainAheadDecoupling(idTrain):
        return
    if (oldSpeed[idTrain-1] - speedT1) > 0:
        #the train ahead is decreasing his speed.
        traci.vehicle.setSpeed(str(idTrain+1), speedT2-8)
        print("\nThe train ahead is decreasing his speed.")
        isBraking[idTrain-1] = True
        return
    elif distances[idTrain-1] < (DISTANCE_COUPLING/2 + 1):
        traci.vehicle.setSpeed(str(idTrain+1), speedT2-0.8)
        return
    elif isBraking[idTrain-1] == True:
        isBraking[idTrain-1] = False #The train ahead is no more braking
        if state[idTrain-1].__eq__("coupled"):
            couplingTrain[idTrain-1] = True #The trains must retrieve their coupling
            state[idTrain-1].__eq__("almost_coupled")
    traci.vehicle.setSpeed(str(idTrain+1), speedT1)
    traci.vehicle.setSpeedFactor(str(idTrain+1), traci.vehicle.getSpeedFactor(str(idTrain)))
    traci.vehicle.setAccel(str(idTrain+1), traci.vehicle.getAccel(str(idTrain)))

def printDistances():
    print("\n-Distance between trains: ")
    for idTrain in range(1,len(trainList)):
        print("\n---T", idTrain, " and T", idTrain+1, ": ", distances[idTrain-1], "m")

def setCouplingDecoupling():
    couplingTrain.clear()
    decouplingTrain.clear()
    isBraking.clear()
    state.clear()
    for train in range(1, len(trainList)):
        couplingTrain.append(True)
        decouplingTrain.append(False)
        isBraking.append(False)
        state.append("decoupled")

def printAllSpeed():
    print("\nSpeeds:")
    for train in trainList:
        print("\nTrain", train[0], ": ", traci.vehicle.getSpeed(str(train[0])))

def updateOldSpeed():
    for idTrain in range(1, len(trainList)):
        oldSpeed[idTrain-1] = traci.vehicle.getSpeed(str(idTrain))

#Updates the list of active trains
def updateTrainsActive():
    idTrains = traci.vehicle.getIDList()
    for train in trainList:
        thereis = False
        for id in idTrains:
            if train[0].__eq__(id):
                thereis = True
        if thereis == False:
            trainList.remove(train)
            

def run():
    traci.simulationStep()
    setCouplingDecoupling()
    step = 1 #step of the simulation
    
    #Set the previous speeds to zero
    for train in trainList:
        oldSpeed.append(0)

    #Set the speeds
    for train in trainList:
        traci.vehicle.setSpeed(train[0], train[1])

    #Delete the limit on the distance between vehicles imposed by SUMO
    for idTrain in range(2,len(trainList)+1):
        traci.vehicle.setSpeedMode(str(idTrain), 28)

    #Start the run of the trains
    while traci.simulation.getMinExpectedNumber() > 0:
        print("\n---Step ", step)
        distances.clear()
        if len(traci.vehicle.getIDList()) > 1:
            updateTrainsActive()
            
            #update follower list
            for train in trainList:
                distance = traci.vehicle.getFollower(train[0], 0) #[idFollower, distance]
                distances.append(distance[1])
            printDistances()

            if step == 370: 
                print("\nSet change of direction for Train 1")
                traci.vehicle.changeTarget("1", "E3")
                decouplingTrain[0] = True
            
            for idTrain in range(1,len(trainList)):
                #Look if there are trains in decoupling mode
                if decouplingTrain[idTrain-1] == True:
                    decouplingTrain[idTrain-1] = stepDecoupling(idTrain)
                #Look if there are trains in coupling mode
                elif couplingTrain[idTrain-1] == True:
                    couplingTrain[idTrain-1] = stepCoupling(idTrain)
                else:
                    stepHoldState(idTrain)
            printAllSpeed()
            updateOldSpeed()
        traci.simulationStep()
        step += 1
    print("\n\nSimulation completed.")
    traci.close()
    sys.stdout.flush()


def changeSpeeds():
    print("\nThe speed of the train must be between ",
          MIN_SPEED," and ",MAX_SPEED,"(",MIN_SPEED*10," Km/h - ",MAX_SPEED*10," Km/h).")
    for train in trainList:
        loop = True
        while loop:
            print("\nSet the speed of the Train ", train[0])
            speed = float(input(": "))
            if (speed < MIN_SPEED) or (speed > MAX_SPEED):
                print("\nThe speed of the train must be between ",MIN_SPEED," and ",MAX_SPEED,".")
            else:
                train[1] = speed;
                loop = False
    

if __name__ == "__main__":
    options = get_options()
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
    if options.multiTrain:
        while True:
            nTrains = int(input("\nSet the number of trains: "))
            if (nTrains < 2 or nTrains > 10):
                print("\nThe number of trains must be between 2 and 10")
            else:
                trainList.clear()
                for i in range(0,nTrains):
                    trainList.append([str(i+1), DEFAULT_SPEED])
                break
        answer = input("\n\nDo you want change the default speed of the trains? (Y, N) ")
        if answer == 'Y' or answer == 'y':
            changeSpeeds()        
    if options.setParam:
        print("\nSet the parameters of the simulation.")
        print("\nRemember that the distance expressed in SUMO is 10 times greater than the real one: to set a distance of 100 meters real you need to enter '10'.")
        while True:
            distCoupling = float(input("\nSet the distance of virtual coupling (default = ", DISTANCE_COUPLING, "): "))
            if (distCoupling < MIN_DIST_COUP) or (distCoupling > MAX_DIST_COUP):
                print("\nThe distance of virtual coupling must be between ", MIN_DIST_COUP
                      ," and ", MAX_DIST_COUP, ".")
            else:
                DISTANCE_COUPLING = distCoupling
                break
        while True:
            distDecoupling = float(input("\nSet the distance of virtual decoupling (default = ", DISTANCE_DECOUPLING, "): "))
            if (distDecoupling < MIN_DIST_DECOUP) or (distDecoupling > MAX_DIST_DECOUP):
                print("\nThe distance of virtual decoupling must be between ", MIN_DIST_DECOUP
                      ," and ", MAX_DIST_DECOUP, ".")
            else:
                DISTANCE_DECOUPLING = distDecoupling
                break
        changeSpeeds()
        

    traci.start([sumoBinary, "-c", "railvc.sumocfg", "--tripinfo-output", "tripinfo.xml"])
    run()
    if options.nogui:
        fine = input("\nPress any button to end the simulation.")
