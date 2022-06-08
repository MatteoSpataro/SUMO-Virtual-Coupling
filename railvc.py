import os
import sys
import optparse

DISTANCE_COUPLING = 10
DISTANCE_DECOUPLING = 100
DEFAULT_SPEED = 20
trainList = [["1",20.8],["2",20]]
distances = [] #List with the distances between trains
oldSpeed = [] #List with the speeds of the trains in the previous step
couplingTrain = [False] #Train is trying to reach the coupling if is "False"
decouplingTrain = [False] #Train is trying to reach the decoupling if is "True"

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
        print("\nTrain ", idTrain+1, ": decreasing speed\n")
    else:
        traci.vehicle.setSpeed(str(idTrain+1), DEFAULT_SPEED)
        print("\n\nDECOUPLING COMPLETED BETWEEN T", idTrain, " AND T", idTrain+1, "\n")
        return False
    return True

def stepCoupling(idTrain):
    speedT1 = traci.vehicle.getSpeed(str(idTrain))
    speedT2 = traci.vehicle.getSpeed(str(idTrain+1))
    speedDiff = speedT2 - speedT1
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
        print("\nTrain ", idTrain+1, ": decreasing speed\n")
    elif distances[idTrain-1] > 0:
        print("\n\nCOUPLING COMPLETED BETWEEN T", idTrain, " AND T", idTrain+1, "\n")
        return True
    return False    
    
def stepHoldCoupling(idTrain):
    speedT1 = traci.vehicle.getSpeed(str(idTrain))
    speedT2 = traci.vehicle.getSpeed(str(idTrain+1))
    if (oldSpeed[idTrain-1] - speedT1) > 0:
        #the train ahead is decreasing his speed.
        traci.vehicle.setSpeed(str(idTrain+1), speedT2-8)
        traci.vehicle.setDecel(str(idTrain+1), traci.vehicle.getDecel(str(idTrain))+0.4)
        print("\nThe train ahead is decreasing his speed.")
        couplingTrain[idTrain-1] = False
    elif distances[idTrain-1] < (DISTANCE_COUPLING/2 + 1):
        traci.vehicle.setSpeed(str(idTrain+1), speedT2-0.8)
    else:
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
    for train in range(1, len(trainList)):
        couplingTrain.append(False)
        decouplingTrain.append(False)

def printAllSpeed():
    print("\nSpeeds:")
    for train in trainList:
        print("\nTrain", train[0], ": ", traci.vehicle.getSpeed(str(train[0])))

def updateOldSpeed():
    for idTrain in range(1, len(trainList)):
        oldSpeed[idTrain-1] = traci.vehicle.getSpeed(str(idTrain))

def updateTrainsActive():
    idTrains = traci.vehicle.getIDList()
    for train in trainList:
        thereis = False
        for id in idTrains:
            if str(train[0]).__eq__(id):
                thereis = True
        if thereis == False:
            trainList.remove(train)
            

def run():
    traci.simulationStep()
    setCouplingDecoupling()
    step = 1 #step of the simulation
    
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
                distance = traci.vehicle.getFollower(str(train[0]), 0) #[idFollower, distance]
                distances.append(distance[1])
            printDistances()
            if step == 370: 
                print("\nSet change of direction for Train 1")
                traci.vehicle.changeTarget("1", "E17")
                decouplingTrain[0] = True
            
            for idTrain in range(1,len(trainList)):
                #Look if there are trains in decoupling mode
                if decouplingTrain[idTrain-1] == True:
                    decouplingTrain[idTrain-1] = stepDecoupling(idTrain)
                #Look if there are trains in coupling mode
                elif couplingTrain[idTrain-1] == False:
                    couplingTrain[idTrain-1] = stepCoupling(idTrain)
                    stepHoldCoupling(idTrain)
            printAllSpeed()
            updateOldSpeed()
        traci.simulationStep()
        step += 1
    print("\n\nSimulation completed.")
    traci.close()
    sys.stdout.flush()

def changeSpeeds():
    print("\nThe speed of the train must be between 10.0 and 30.0 (100 Km/h - 300 Km/h).")
    for train in trainList:
        loop = True
        while loop:
            print("\nSet the speed of the Train ", train[0])
            speed = float(input(": "))
            if (speed < 10.0 or speed > 30.0):
                print("\nThe speed of the train must be between 10.0 and 30.0.")
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
                    trainList.append([str(i+1), 20])
                break
        answer = input("\n\nDo you want change the default speed of the trains? (Y, N) ")
        if answer == 'Y' or answer == 'y':
            changeSpeeds()        
    if options.setParam:
        print("\nSet the parameters of the simulation.")
        print("\nRemember that the distance expressed in SUMO is 10 times greater than the real one: to set a distance of 100 meters real you need to enter '10'.")
        while True:
            distCoupling = float(input("\nSet the distance of virtual coupling (default = 10): "))
            if (distCoupling < 5 or distCoupling > 40):
                print("\nThe distance of virtual coupling must be between 5 and 40.")
            else:
                DISTANCE_COUPLING = distCoupling
                break
        while True:
            distDecoupling = float(input("\nSet the distance of virtual decoupling (default = 100): "))
            if (distDecoupling < 45 or distDecoupling > 150):
                print("\nThe distance of virtual decoupling must be between 45 and 150.")
            else:
                DISTANCE_DECOUPLING = distDecoupling
                break
        changeSpeeds()
        

    traci.start([sumoBinary, "-c", "railvc.sumocfg", "--tripinfo-output", "tripinfo.xml"])
    run()
    if options.nogui:
        fine = input("\nPress any button to end the simulation.")
