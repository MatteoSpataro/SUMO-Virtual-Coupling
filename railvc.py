import os
import sys
import optparse

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

def stepDecoupling(distance):
    if distance < 100:    
        speedT2 = traci.vehicle.getSpeed("2")
        traci.vehicle.setSpeed("2", speedT2-1)
        print("\nTrain 1: ", traci.vehicle.getSpeed("1"))
        print("\nTrain 2: ", traci.vehicle.getSpeed("2"))
        print("\n\nTrain 2: decreasing speed\n")
    else:
        traci.vehicle.setSpeed("2", 20)
        print("\nSpeed Train 1: ", traci.vehicle.getSpeed("1"))
        print("\nSpeed Train 2: ", traci.vehicle.getSpeed("2"))
        print("\n\nDECOUPLING COMPLETED\n")
        return False
    return True

def stepCoupling(distance):
    oldSpeedT1 = traci.vehicle.getSpeed("1")
    oldSpeedT2 = traci.vehicle.getSpeed("2")
    if distance >= 50: #Distance bigger than 500m
        print("\nTrain 1: ", traci.vehicle.getSpeed("1"))
        print("\nTrain 2: ", traci.vehicle.getSpeed("2"))
        print("\nTrain 2: increasing speed\n")
        if oldSpeedT2 <= 29 and (oldSpeedT2-oldSpeedT1)<6: #Speed must be under 300 km/h
            traci.vehicle.setSpeed("2", oldSpeedT2+1)
    elif distance > 10: #Distance between 100m and 500m
        if oldSpeedT2 > oldSpeedT1+5:
            traci.vehicle.setSpeed("2", oldSpeedT2-5)
        elif oldSpeedT2 > oldSpeedT1+2.5:
            traci.vehicle.setSpeed("2", oldSpeedT2-2.5)
        elif oldSpeedT2 > oldSpeedT1+1.5:
            traci.vehicle.setSpeed("2", oldSpeedT2-0.5)
        print("\nTrain 1: ", traci.vehicle.getSpeed("1"))
        print("\nTrain 2: ", traci.vehicle.getSpeed("2"))
        print("\n\nTrain 2: decreasing speed\n")
    elif distance > 0:
        traci.vehicle.setSpeed("2", oldSpeedT1)
        traci.vehicle.setSpeedFactor("2", traci.vehicle.getSpeedFactor("1"))
        print("\n\nCOUPLING COMPLETED\n")
        return True
    return False    
    
def run():
    coupling = False #They are trying to reach the coupling if is "False"
    decoupling = False #They are trying to reach the decoupling if is "True"
    traci.simulationStep()
    step = 1
    #Set the speeds
    traci.vehicle.setSpeed("1", 20.8)
    traci.vehicle.setSpeed("2", 20)
    #Delete the limit on the distance between vehicles imposed by SUMO
    traci.vehicle.setSpeedMode("2", 28)
    while traci.simulation.getMinExpectedNumber() > 0:
        print("\n---Step ", step)
        if len(traci.vehicle.getIDList()) > 1:
            follower = traci.vehicle.getFollower("1", 0)
            distance = follower[1]
            print("\n-Distance between trains: ", distance, "m")
            if step == 370: 
                print("\nSet change of direction for Train 1")
                traci.vehicle.changeTarget("1", "E3")
                decoupling = True
            if decoupling == True:
                decoupling = stepDecoupling(distance)
            elif coupling == False:
                coupling = stepCoupling(distance)
            if coupling == True and decoupling == False:
                oldSpeedT1 = traci.vehicle.getSpeed("1")
                oldSpeedT2 = traci.vehicle.getSpeed("2")
                if oldSpeedT1 < oldSpeedT2:
                    #Train 1 is slowing down
                    traci.vehicle.setSpeed("2", oldSpeedT1-(oldSpeedT2-oldSpeedT1)-0.3)
                else:
                    traci.vehicle.setSpeed("2", oldSpeedT1)
                traci.vehicle.setSpeedFactor("2", traci.vehicle.getSpeedFactor("1"))
                traci.vehicle.setAccel("2", traci.vehicle.getAccel("1"))
        traci.simulationStep()
        step += 1
    print("\n\nSimulation completed.")
    traci.close()
    sys.stdout.flush()

if __name__ == "__main__":
    options = get_options()
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')
    if options.multiTrain:
        print("\nHai scelto l'esecuzione con piu' treni")
        while True:
            nTrains = int(input("\nSet the number of the trains: "))
            if (nTrains < 2 or nTrains > 10):
                print("\nThe number of the trains must be between 2 and 10")
            else:
                break
        print(nTrains)
        
    if options.setParam:
        print("\nSet the parameters of the simulation.")
        print("Remember that the distance expressed in SUMO is 10 times greater than the real one. So, to enter a distance of 100 meters real you need to enter '10'.")
        while True:
            distCoupling = float(input("\nSet the distance of virtual coupling: "))
            if (distCoupling < 5 or distCoupling > 40):
                print("\nThe distance of virtual coupling must be between 5 and 40.")
            else:
                break
        while True:
            distDecoupling = float(input("\nSet the distance of virtual decoupling: "))
            if (distDecoupling < 45 or distDecoupling > 150):
                print("\nThe distance of virtual decoupling must be between 45 and 150.")
            else:
                break
        if options.multiTrain:
            for i in range(1, nTrains):
                loop = True
                while True:
                    trainSpeed = float(input("\nSet the speed of the Train ", i, ": "))
                    if (trainSpeed < 10 or trainSpeed > 30):
                        print("\nThe speed of the train must be between 10 and 30.")
                    else:
                        loop = False
        print(distCoupling, distDecoupling)
        

    traci.start([sumoBinary, "-c", "railvc.sumocfg", "--tripinfo-output", "tripinfo.xml"])
    run()
    if options.nogui:
        fine = input("\nPress any button to end the simulation.")