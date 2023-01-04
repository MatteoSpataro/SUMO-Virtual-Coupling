from ctypes.wintypes import INT
import os
import sys
import optparse

#Import "RbcVC" from "rbcNoise" instead of "rbcVc" to emulate a communication channel with noise, 
# where the communication between SUMO and RBC can fail.

#from rbcNoise import MIN_DIST_COUP, MAX_DIST_COUP, MIN_DIST_DECOUP, MAX_DIST_DECOUP, RbcVC
from rbcVc import MIN_DIST_COUP, MAX_DIST_COUP, MIN_DIST_DECOUP, MAX_DIST_DECOUP, RbcVC
from rbcNoVc import RbcNoVC

from sumolib import checkBinary
import traci

__author__ = "Matteo Spataro"
__license__ = "Eclipse Public License"
__version__ = "2.0"
__maintainer__ = "Matteo Spataro"
__email__ = "matteo.spataro@stud.unifi.it"

MIN_NUMBER_OF_TRAINS = 3
MAX_NUMBER_OF_TRAINS = 22
DEPARTURE_INTERVAL = 25
DEFAULT_FILE = "default.rou.xml"
NET_FILE = "railvc.rou.xml"

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("Please declare environment variable 'SUMO_HOME'")

def get_options():
    opt_parser = optparse.OptionParser()
    opt_parser.add_option("--nogui", action="store_true", default=False, 
                          help="Run the commandline version of SUMO.")
    opt_parser.add_option("--setParam", action="store_true", default=False, 
                          help="Set the parameters of the simulation.")
    opt_parser.add_option("--novc", action="store_true", default=False, 
                          help="Run the simulation without Virtual Coupling.")
    opt_parser.add_option("--maxTrains", action="store_true", default=False, 
                          help="Run the simulation with the maximum capacity: 30 trains.")
    opt_parser.add_option("--variant", action="store_true", default=False, 
                          help="Run the simulation in the circuit variant.")
    options, args = opt_parser.parse_args()
    return options

def changeSpeeds(trainList, MIN_SPEED):
    MAX_DEFAULT_SPEED = 21
    print("\nThe speed of the train must be between",
          MIN_SPEED,"and",MAX_DEFAULT_SPEED,"(",MIN_SPEED*10,"Km/h -",MAX_DEFAULT_SPEED*10,"Km/h).")
    for train in trainList:
        tries = 0
        while tries < 15:
            print("\nSet the speed of the Train", train.getId())
            newSpeed = float(input(": "))
            if (newSpeed < MIN_SPEED) or (newSpeed > MAX_DEFAULT_SPEED):
                print("\nThe speed of the train must be between",MIN_SPEED,"and",MAX_DEFAULT_SPEED,".")
                tries = tries + 1
            else:
                train.setDefaultSpeed(newSpeed)
                break
        if tries == 15: 
            print("\nThe maximum number of attempts has been reached.")
            quit()

#Method to reset the .rou.xml file for a new simulation
def setFileRou():
    with open(DEFAULT_FILE, 'r') as rf:
        with open(NET_FILE, 'w') as wf:
            for line in rf:
                wf.write(line)

#Method to add a new train in the .rou.xml file
def addTrainInFile(idTrain, rbc):
    idTrain = idTrain-1
    colors = ["1,1,0","1,0,0","0,1,0","0,1,1","0.1,0.3,1","1,0,1","0.3,0.9,0.9","1,0.5,0","1,0.2,0.2","0.9,0.9,0.9"]
    with open(NET_FILE, 'a') as f:
        line = "<vType id=\"rail"+str(idTrain)+"\" priority=\"1\" vClass=\"rail\" length=\"100\" accel=\""+str(rbc.DEFAULT_ACCEL)+"\" decel=\""+str(rbc.DEFAULT_DECEL)+"\" sigma=\"1.0\" maxSpeed=\"30\" guiShape=\"rail\" color=\""+colors[(idTrain)%10]+"\"/>\n"
        f.write(line)
        depart = DEPARTURE_INTERVAL + DEPARTURE_INTERVAL*(idTrain-3)
        line = "<vehicle id=\""+str(idTrain)+"\" type=\"rail"+str(idTrain)+"\" route=\"route2\" depart=\""+str(depart)+"\" />\n"
        f.write(line)

if __name__ == "__main__":
    options = get_options()
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    if options.maxTrains:
        nTrain = 30
    else:
        tries = 0
        while tries < 15:
            nTrain = int(input("\nSet the number of trains: "))
            if (nTrain < MIN_NUMBER_OF_TRAINS or nTrain > MAX_NUMBER_OF_TRAINS):
                print("\nThe number of trains must be between",MIN_NUMBER_OF_TRAINS,"and",MAX_NUMBER_OF_TRAINS)
                tries = tries + 1
            else:
                break
        if tries == 15: 
            print("\nThe maximum number of attempts has been reached.")
            quit()

    setFileRou()

    rbc = RbcNoVC(nTrain, options.variant)  

    for i in range(4, nTrain+1):
        addTrainInFile(i, rbc)
    with open(NET_FILE, 'a') as f:
        f.write('</routes>')

    if not options.novc:
        rbc = RbcVC(nTrain, DEPARTURE_INTERVAL, options.variant)
        if options.setParam:
            print("\nSet the parameters of the simulation.")
            print("\nRemember that the distance expressed in SUMO is 10 times greater than the real one: "+
                  "to set a distance of 100 meters real you need to enter '10'.")
            tries = 0
            while tries < 15:
                distCoupling = float(input("\nSet the distance of virtual coupling (default = "+str(rbc.getDistanceCoupling())+"): "))
                if (distCoupling < MIN_DIST_COUP) or (distCoupling > MAX_DIST_COUP):
                    print("\nThe distance of virtual coupling must be between ", MIN_DIST_COUP
                          ," and ", MAX_DIST_COUP, ".")
                    tries = tries + 1
                else:
                    rbc.setDistanceCoupling(distCoupling)
                    break
            while tries < 15:
                distDecoupling = float(input("\nSet the distance of virtual decoupling (default = "+str(rbc.getDistanceDecoupling())+"): "))
                if (distDecoupling < MIN_DIST_DECOUP) or (distDecoupling > MAX_DIST_DECOUP):
                    print("\nThe distance of virtual decoupling must be between ", MIN_DIST_DECOUP
                          ," and ", MAX_DIST_DECOUP, ".")
                    tries = tries + 1
                else:
                    rbc.setDistanceDecoupling(distDecoupling)
                    break
            if tries == 15: 
                print("\nThe maximum number of attempts has been reached.")
                quit()
    if options.maxTrains:
        print("You are running the simulation with the maximum  capacity: 30 trains.")
        print("The default speed of the trains is 150 Km/h.")
        trainList = rbc.getTrainList()
        for train in trainList:
            train.setDefaultSpeed(15)
    else:
        answer = input("\n\nDo you want change the default speed of the trains? (Y, N) ")
        if answer == 'Y' or answer == 'y':
            changeSpeeds(rbc.getTrainList(), rbc.MIN_SPEED)

    if options.variant:
        traci.start([sumoBinary, "-c", "railvc2.sumocfg", "--tripinfo-output", "tripinfo.xml"])
    else:
        traci.start([sumoBinary, "-c", "railvc.sumocfg", "--tripinfo-output", "tripinfo.xml"])
    rbc.run()
        
    if options.nogui:
        fine = input("\nPress Enter button to end the simulation.")