from ctypes.wintypes import INT
import os
import sys
import optparse

DISTANCE_COUPLING = 10
DISTANCE_DECOUPLING = 100
DEFAULT_SPEED = 20.8
MIN_DIST_COUP = 5
MAX_DIST_COUP = 40
MIN_DIST_DECOUP = 45
MAX_DIST_DECOUP = 150
MIN_SPEED = 10.0
MAX_SPEED = 30.0
trainList = [["1",20.8],["2",20]] #List of active trains
distances = [] #List with the distances between trains
oldSpeed = [] #List with the speeds of the trains in the previous step
couplingTrain = [True] #Train is trying to reach the coupling if is "True"
decouplingTrain = [False] #Train is trying to reach the decoupling if is "True"
state = ["decoupled"]
isBraking = [False]
incomingTrains = 0
defaultSpeeds = [20.8, 20]
countDisconnection = [0] #the number of sequential disconnection for each train

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

def setInitialParameters():
    for train in range(2, len(trainList)):
        couplingTrain.append(True)
        decouplingTrain.append(False)
        isBraking.append(False)
        state.append("decoupled")
        defaultSpeeds.append(trainList[train][1])
        countDisconnection.append(0)

def printAllSpeed():
    print("\nSpeeds:")
    for train in trainList:
        train[1] = traci.vehicle.getSpeed(train[0])
        print("\nTrain", train[0], ":", train[1])

def updateOldSpeed():
    oldSpeed.clear()
    for train in trainList:
        oldSpeed.append(train[1])

def setSameSpeedFactores(trainFollower, trainAhead):
    traci.vehicle.setSpeedFactor(trainFollower, traci.vehicle.getSpeedFactor(trainAhead))
    traci.vehicle.setAccel(trainFollower, traci.vehicle.getAccel(trainAhead))
    traci.vehicle.setDecel(trainFollower, traci.vehicle.getAccel(trainAhead))

def stepDecoupling(pos):
    trainAhead = trainList[pos]
    trainFollower = trainList[pos+1]
    if distances[pos] < DISTANCE_DECOUPLING:    
        trainFollowerSpeed = traci.vehicle.getSpeed(trainFollower[0])
        traci.vehicle.setSpeed(trainFollower[0], trainFollowerSpeed - 1)
        print("\nIn decoupling, Train", trainFollower[0], "is decreasing speed")
        state[pos] = "almost_decoupled"

        #Decrease the speed of all the trains coupled with the follower train
        i = 1
        for train in trainList:
            if (int(train[0]) >= int(trainFollower[0])) and (i < len(trainList)): 
                if (state[i-1].__eq__("coupled") or 
                        state[i-1].__eq__("almost_coupled")):
                    idTrainFollower = str(int(train[0])+1)
                    traci.vehicle.setSpeed(idTrainFollower, trainFollowerSpeed - 2)
                    print("\nIn decoupling, Train", idTrainFollower, "is decreasing speed")
            i += 1
    else:
        #The trains are decoupled
        traci.vehicle.setSpeed(trainFollower[0], defaultSpeeds[int(trainFollower[0])-1])
        #Reset the speed of all the trains coupled with the follower train
        for idFollower in range(int(trainFollower[0]),len(trainList)):
            if state[idFollower-1].__eq__("coupled") or state[idFollower-1].__eq__("almost_coupled"):
                traci.vehicle.setSpeed(str(idFollower+1), defaultSpeeds[int(trainFollower[0])-1])
                print("\nIn decoupling, Train ", idFollower+1, "reset the speed")
        print("\n\nDECOUPLING COMPLETED BETWEEN T", trainAhead[0], " AND T", trainFollower[0], "\n")
        state[pos] = "decoupled"
        return False
    return True

#Check if the train ahead is decoupling: in this case we can't modify the speed of the following trains 
#in order not to overwrite the changes made during the decoupling phase of the train ahead.
def trainAheadDecoupling(idTrainAhead):
    while idTrainAhead > 0:
        if decouplingTrain[idTrainAhead-1] == True:
            return True
        elif state[idTrainAhead-1].__eq__("coupled") or state[idTrainAhead-1].__eq__("almost_coupled"):
            return trainAheadDecoupling(idTrainAhead-1)
        else:
            return False
    return False

def stepCoupling(pos):
    trainAhead = trainList[pos]
    trainFollower = trainList[pos+1]
    trainSpeed = traci.vehicle.getSpeed(trainAhead[0])
    trainFollowerSpeed = traci.vehicle.getSpeed(trainFollower[0])
    speedDiff = trainFollowerSpeed - trainSpeed
    #The follower train is coming 
    if traci.vehicle.getRoadID(trainFollower[0]).__eq__("E3") or traci.vehicle.getRoadID(trainFollower[0]).__eq__("E5"):
        setSameSpeedFactores(trainFollower[0], trainAhead[0])
        return True
    #Check if there is a connection problem
    if distances[pos] == -1 and (not traci.vehicle.getRoadID(trainFollower[0]).__eq__("E2")): 
        if countDisconnection[pos] < 6:
            traci.vehicle.setSpeed(trainFollower[0], trainFollowerSpeed-1)
            traci.vehicle.setDecel(trainFollower[0], 
                                   traci.vehicle.getDecel(trainFollower[0])+0.02)
            countDisconnection[pos] += 1
        else:
            traci.vehicle.setSpeed(trainFollower[0], defaultSpeeds[pos])
            traci.vehicle.setDecel(trainFollower[0], 0.7)
        return True
    countDisconnection[pos] = 0
    if state[pos].__eq__("almost_coupled"):
        print("\nTrains T", trainAhead[0], " and T", trainFollower[0], "are almost coupled\n")
    if distances[pos] >= DISTANCE_COUPLING*5.50 and trainFollowerSpeed <= MAX_SPEED-1:
        if speedDiff < 6:
            traci.vehicle.setSpeed(trainFollower[0], trainFollowerSpeed+1)
            print("\nTrain", trainFollower[0], ": increasing speed\n")
        else:
            traci.vehicle.setSpeed(trainFollower[0], trainFollowerSpeed-1)
            print("\nTrain", trainFollower[0], "isn't increasing is speed.\n")
    elif distances[pos] > DISTANCE_COUPLING+1:
        if speedDiff > 5:
            traci.vehicle.setSpeed(trainFollower[0], trainFollowerSpeed-5)
            traci.vehicle.setDecel(trainFollower[0], 
                                   traci.vehicle.getDecel(trainFollower[0])+0.2)
        elif speedDiff > 2.5:
            traci.vehicle.setSpeed(trainFollower[0], trainFollowerSpeed-2.5)
        elif speedDiff > 1.5:
            traci.vehicle.setSpeed(trainFollower[0], trainFollowerSpeed-1.2)
        print("\nIn coupling, Train", trainFollower[0], "is decreasing his speed")
    elif distances[pos] > 0:
        print("\n\nCOUPLING COMPLETED BETWEEN T", trainAhead[0], " AND T", trainFollower[0], "\n")
        traci.vehicle.setSpeed(trainFollower[0], trainSpeed)
        setSameSpeedFactores(trainFollower[0], trainAhead[0])
        state[pos] = "coupled"
        return False
    return True    

#Hold the current state of the trains
def stepHoldState(pos):
    trainAhead = trainList[pos]
    trainFollower = trainList[pos+1]
    trainSpeed = traci.vehicle.getSpeed(trainAhead[0])
    speedT2 = traci.vehicle.getSpeed(trainFollower[0])
    if trainAheadDecoupling(pos+1):
        return
    if oldSpeed[pos] > trainSpeed:
        #the train ahead is decreasing his speed
        if state[pos].__eq__("coupled") or state[pos].__eq__("almost_coupled"):
            traci.vehicle.setSpeed(trainFollower[0], speedT2-2)
            setSameSpeedFactores(trainFollower[0], trainAhead[0])
            print("\nTrain",trainAhead[0],"is decreasing his speed.")
            trainFollower[1] = speedT2-1
            isBraking[pos] = True
            state[pos] = "almost_coupled"
            return
    if isBraking[pos] == True:
        #The train ahead is no more braking
        isBraking[pos] = False 
        couplingTrain[pos] = True #The trains must retrieve their coupling
        traci.vehicle.setSpeed(trainFollower[0], trainSpeed+1)
        print("\nTrains T", trainAhead[0], "and T", trainFollower[0], "retrieve their coupling\n")
        return
    if distances[pos] == -1:
        return
    elif distances[pos] < (DISTANCE_COUPLING/2.0 + 1):
        traci.vehicle.setSpeed(trainFollower[0], speedT2-0.8)
        state[pos] = "almost_coupled"
        couplingTrain[pos] = True #The trains must retrieve their coupling
        return
    elif distances[pos] > DISTANCE_COUPLING+1:
        traci.vehicle.setSpeed(trainFollower[0], speedT2-0.8)
        state[pos] = "almost_coupled"
        couplingTrain[pos] = True #The trains must retrieve their coupling
        return
    traci.vehicle.setSpeed(trainFollower[0], trainSpeed)
    setSameSpeedFactores(trainFollower[0], trainAhead[0])

def printDistances():
    print("\n-Distance between trains: ")
    i = 0
    for train in trainList:
        if i != len(trainList)-1:
            print("\n---T", train[0], " and T", int(train[0])+1, ": ", distances[i], "m")
        i += 1

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
            decouplingTrain.pop(0)
            couplingTrain.pop(0)
            state.pop(0)
            isBraking.pop(0)
            countDisconnection.pop(0)

def addTrain():
    speed = DEFAULT_SPEED - 0.8*3
    trainList.append([str(len(trainList)+1), speed])
    traci.vehicle.setSpeed(str(len(trainList)), speed)
    oldSpeed.append(0.0)
    couplingTrain.append(True)
    decouplingTrain.append(False)
    isBraking.append(False)
    state.append("decoupled")
    defaultSpeeds.append(speed)
    countDisconnection.append(0)
    setSameSpeedFactores(str(len(trainList)), str(len(trainList)-1))

def run():
    traci.simulationStep()
    setInitialParameters()
    step = 1 #step of the simulation
    
    #Set the previous speeds to zero
    for train in trainList:
        oldSpeed.append(0)

    #Set the speeds
    for train in trainList:
        traci.vehicle.setSpeed(train[0], train[1])
    
    #Delete the limit on the distance between vehicles imposed by SUMO
    firstTrain = trainList[0]
    traci.vehicle.setSpeedMode(str(firstTrain[0]), 29)

    #Start the run of the trains
    while traci.simulation.getMinExpectedNumber() > 0:
        print("\n-----Step ", step)
        if len(traci.vehicle.getIDList()) > 1:
            updateTrainsActive()
            #Check if there is an incoming train
            global incomingTrains
            if incomingTrains > 0:
                if step>24 and (step%25 == 0):
                    addTrain()
                    incomingTrains -= 1
                               
            if step > 1:
                updateOldSpeed()
            distances.clear()
            #update follower list
            for train in trainList:
                distance = traci.vehicle.getFollower(train[0], 0) #[idFollower, distance]
                distances.append(distance[1])
            printDistances()
            
            for train in trainList:
                #Don't delete the limit on the distance if they are not in the right position
                if train[0].__eq__("1"):
                    if (traci.vehicle.getRoadID(train[0]).__eq__("E6") 
                    or traci.vehicle.getRoadID(train[0]).__eq__("E7") 
                    or traci.vehicle.getRoadID(train[0]).__eq__("E8")
                    or traci.vehicle.getRoadID(train[0]).__eq__("E9")
                    or traci.vehicle.getRoadID(train[0]).__eq__("E10")
                    or traci.vehicle.getRoadID(train[0]).__eq__("E11")
                    or traci.vehicle.getRoadID(train[0]).__eq__("E12")):
                        traci.vehicle.setSpeed(train[0], 15)
                    else:
                        traci.vehicle.setSpeed(train[0], defaultSpeeds[0])
                        traci.vehicle.setSpeedMode(train[0], 30)
                elif traci.vehicle.getRoadID(train[0]).__eq__("E6"):
                    traci.vehicle.setSpeedMode(train[0], 30)
            
            if step == 175: 
                print("\nSet change of direction for Train 1")
                traci.vehicle.changeTarget("1", "E35")
                decouplingTrain[0] = True
                couplingTrain[0] = False
            
            if step == 200: 
                print("\nSet change of direction for Train 2")
                traci.vehicle.changeTarget("2", "E35")
                decouplingTrain[1] = True
                couplingTrain[1] = False
            
            if step == 400: 
                print("\nSet change of direction for Train 3")
                traci.vehicle.changeTarget("3", "E35")
                decouplingTrain[0] = True
                couplingTrain[0] = False
            
            i = 0
            for train in trainList:
                if i == len(trainList)-1:
                    break
                #Look if there are trains in decoupling mode
                if decouplingTrain[i] == True:
                    decouplingTrain[i] = stepDecoupling(i)
                #Look if there are trains in coupling mode
                elif couplingTrain[i] == True:
                    couplingTrain[i] = stepCoupling(i)
                elif not state[i].__eq__("decoupled"):
                    stepHoldState(i)
                
                print("\n## State",i+1,":",state[i],"## InCoupling",i+1,":",couplingTrain[i],
                      "## InDecoupling",i+1,":", decouplingTrain[i])
                i += 1
            printAllSpeed()
        traci.simulationStep()
        step += 1
    print("\n\nSimulation completed.")
    traci.close()
    sys.stdout.flush()


def changeSpeeds():
    print("\nThe speed of the train must be between",
          MIN_SPEED,"and",MAX_SPEED,"(",MIN_SPEED*10,"Km/h -",MAX_SPEED*10,"Km/h).")
    defaultSpeeds.clear()
    for i in range(0,len(trainList)):
        loop = True
        while loop:
            print("\nSet the speed of the Train", i+1)
            speed = float(input(": "))
            if (speed < MIN_SPEED) or (speed > MAX_SPEED):
                print("\nThe speed of the train must be between",MIN_SPEED,"and",MAX_SPEED,".")
            else:
                defaultSpeeds.append(speed);
                loop = False

def setFileRou():
    with open('default.rou.xml', 'r') as rf:
        with open('railvc2.rou.xml', 'w') as wf:
            for line in rf:
                wf.write(line)


def addNewTrain(idTrain):
    colors = ["0,0,1","1,0,0","0,1,0","0,1,1","1,1,0","1,0,1","0.5,0.2,0","0.4,0.3,1","1,0.3,0.25","0,0.55,0"]
    with open('railvc2.rou.xml', 'a') as f:
        line = "<vType id=\"rail"+str(idTrain)+"\" priority=\"1\" vClass=\"rail\" length=\"100\" accel=\"0.7\" decel=\"0.7\" sigma=\"1.0\" maxSpeed=\"30\" guiShape=\"rail\" color=\""+colors[idTrain-1]+"\"/>\n"
        f.write(line)
        depart = 24 + 25*(idTrain-4)
        line = "<vehicle id=\""+str(idTrain)+"\" type=\"rail"+str(idTrain)+"\" route=\"route3\" depart=\""+str(depart)+"\" />\n"
        f.write(line)

if __name__ == "__main__":
    options = get_options()
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
    if options.multiTrain:
        while True:
            nTrains = int(input("\nSet the number of trains: "))
            if (nTrains < 3 or nTrains > 10):
                print("\nThe number of trains must be between 3 and 10")
            else:
                trainList.clear()
                if nTrains > 3:
                    for i in range(0,3):
                        trainList.append([str(i+1), DEFAULT_SPEED - 0.8*(i+1)])
                    incomingTrains = nTrains - 3
                else:
                    for i in range(0,nTrains):
                        trainList.append([str(i+1), DEFAULT_SPEED - 0.8*(i+1)])
                setFileRou()
                for i in range(4,nTrains+1):
                    addNewTrain(i)
                with open('railvc2.rou.xml', 'a') as f:
                    f.write('</routes>')
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
