import os
import sys
import optparse

DISTANCE_COUPLING = 10
DISTANCE_DECOUPLING = 100
DEFAULT_SPEED = 20
TRAINS = 2
trains_speed = [20.8, 20]
distances = []
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
        print("\nTrain ", idTrain, ": ", traci.vehicle.getSpeed(str(idTrain)))
        print("\nTrain ", idTrain+1, ": ", traci.vehicle.getSpeed(str(idTrain+1)))
        print("\n\nTrain ", idTrain+1, ": decreasing speed\n")
    else:
        traci.vehicle.setSpeed(str(idTrain+1), DEFAULT_SPEED)
        print("\nSpeed Train ", idTrain, ": ", traci.vehicle.getSpeed(str(idTrain)))
        print("\nSpeed Train ", idTrain+1, ": ", traci.vehicle.getSpeed(str(idTrain+1)))
        print("\n\nDECOUPLING COMPLETED\n")
        return False
    return True

def stepCoupling(idTrain):
    oldSpeedT1 = traci.vehicle.getSpeed(str(idTrain))
    oldSpeedT2 = traci.vehicle.getSpeed(str(idTrain+1))
    if distances[idTrain-1] >= DISTANCE_COUPLING*5:
        print("\nTrain ", idTrain, ": ", traci.vehicle.getSpeed(str(idTrain)))
        print("\nTrain ", idTrain+1, ": ", traci.vehicle.getSpeed(str(idTrain+1)))
        print("\nTrain ", idTrain+1, ": increasing speed\n")
        speedDiff = oldSpeedT2 - oldSpeedT1
        if oldSpeedT2 <= 29 and speedDiff < 6: #Speed must be under 300 km/h
            traci.vehicle.setSpeed(str(idTrain+1), oldSpeedT2+1)
    elif distances[idTrain-1] > DISTANCE_COUPLING: 
        if oldSpeedT2 > oldSpeedT1 + 5:
            traci.vehicle.setSpeed(str(idTrain+1), oldSpeedT2-5)
        elif oldSpeedT2 > oldSpeedT1 + 2.5:
            traci.vehicle.setSpeed(str(idTrain+1), oldSpeedT2-2.5)
        elif oldSpeedT2 > oldSpeedT1 + 1.5:
            traci.vehicle.setSpeed(str(idTrain+1), oldSpeedT2-0.5)
        print("\nTrain ", idTrain, ": ", traci.vehicle.getSpeed(str(idTrain)))
        print("\nTrain ", idTrain+1, ": ", traci.vehicle.getSpeed(str(idTrain+1)))
        print("\n\nTrain ", idTrain+1, ": decreasing speed\n")
    elif distances[idTrain-1] > 0:
        traci.vehicle.setSpeed(str(idTrain+1), oldSpeedT1)
        traci.vehicle.setSpeedFactor(str(idTrain+1), traci.vehicle.getSpeedFactor(str(idTrain)))
        print("\n\nCOUPLING COMPLETED\n")
        return True
    return False    
    
def stepHoldCoupling(idTrain):
    oldSpeedT1 = traci.vehicle.getSpeed(str(idTrain))
    oldSpeedT2 = traci.vehicle.getSpeed(str(idTrain+1))
    if oldSpeedT1 < oldSpeedT2:
        #Train ahead is slowing down
        traci.vehicle.setSpeed(str(idTrain+1), oldSpeedT1-(oldSpeedT2-oldSpeedT1)-0.3)
    else:
        traci.vehicle.setSpeed(str(idTrain+1), oldSpeedT1)
        traci.vehicle.setSpeedFactor(str(idTrain+1), traci.vehicle.getSpeedFactor(str(idTrain)))
        traci.vehicle.setAccel(str(idTrain+1), traci.vehicle.getAccel(str(idTrain)))

def printDistances():
    print("\n-Distance between trains: ")
    for idTrain in range(1,TRAINS):
        print("\n---T", idTrain, " and T", idTrain+1, ": ", distances[idTrain-1], "m")

def setCouplingDecoupling():
    couplingTrain.clear()
    decouplingTrain.clear()
    for train in range(1, TRAINS):
        couplingTrain.append(False)
        decouplingTrain.append(False)

def run():
    traci.simulationStep()
    setCouplingDecoupling()
    step = 1 #step of the simulation
    #Set the speeds
    pos = 1
    for speed in trains_speed:
        traci.vehicle.setSpeed(str(pos), speed)
        pos += 1
    #Delete the limit on the distance between vehicles imposed by SUMO
    for idTrain in range(2,TRAINS+1):
        traci.vehicle.setSpeedMode(str(idTrain), 28)
    #Start the run of the trains
    while traci.simulation.getMinExpectedNumber() > 0:
        print("\n---Step ", step)
        distances.clear()
        if len(traci.vehicle.getIDList()) > 1:
            #update follower list
            for idTrain in range(1,TRAINS):
                distance = traci.vehicle.getFollower(str(idTrain), 0) #[idFollower, distance]
                distances.append(distance[1])
            printDistances()
            if step == 370: 
                print("\nSet change of direction for Train 1")
                traci.vehicle.changeTarget("1", "E17")
                decouplingTrain[0] = True
            
            for idTrain in range(1,TRAINS):
                #Look if there are trains in decoupling mode
                if decouplingTrain[idTrain-1] == True:
                    stepDecoupling(idTrain)
                #Look if there are trains in coupling mode
                elif couplingTrain[idTrain-1] == False:
                    stepCoupling(idTrain)
                else:
                    stepHoldCoupling(idTrain)
                
        traci.simulationStep()
        step += 1
    print("\n\nSimulation completed.")
    traci.close()
    sys.stdout.flush()

def changeSpeeds():
    print("\nThe speed of the train must be between 10.0 and 30.0 (100 Km/h - 300 Km/h).")
    trains_speed.clear()
    for i in range(1,TRAINS+1):
        loop = True
        while loop:
            print("\nSet the speed of the Train ", i)
            speed = float(input(": "))
            if (speed < 10.0 or speed > 30.0):
                print("\nThe speed of the train must be between 10.0 and 30.0.")
            else:
                trains_speed.append(speed);
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
                TRAINS = nTrains
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