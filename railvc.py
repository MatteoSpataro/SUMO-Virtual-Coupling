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
    opt_parser.add_option("--nogui", action="store_true", default=False, help="run the commandline version of sumo")
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
    traci.start([sumoBinary, "-c", "railvc.sumocfg", "--tripinfo-output", "tripinfo.xml"])
    run()
    if options.nogui:
        fine = input("\nPress any button to end the simulation.")