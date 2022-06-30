from ctypes.wintypes import INT
import os
import sys
import optparse

from rbc import MIN_DIST_COUP, MAX_DIST_COUP, MIN_DIST_DECOUP, MAX_DIST_DECOUP, MAX_SPEED, MIN_DIST_DECOUP, MIN_SPEED, Rbc

from sumolib import checkBinary
import traci

MIN_NUMBER_OF_TRAINS = 3
MAX_NUMBER_OF_TRAINS = 10

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
    options, args = opt_parser.parse_args()
    return options

def changeSpeeds(trainList):
    print("\nThe speed of the train must be between",
          MIN_SPEED,"and",MAX_SPEED,"(",MIN_SPEED*10,"Km/h -",MAX_SPEED*10,"Km/h).")
    
    for i in range(0, len(trainList)):
        loop = True
        while loop:
            print("\nSet the speed of the Train", i+1)
            newSpeed = float(input(": "))
            if (newSpeed < MIN_SPEED) or (newSpeed > MAX_SPEED):
                print("\nThe speed of the train must be between",MIN_SPEED,"and",MAX_SPEED,".")
            else:
                trainList[i].setDefaultSpeed(newSpeed)
                loop = False

#Method to reset the .rou.xml file for a new simulation
def setFileRou():
    with open('default.rou.xml', 'r') as rf:
        with open('railvc2.rou.xml', 'w') as wf:
            for line in rf:
                wf.write(line)

#Method to add a new train in the .rou.xml file
def addNewTrain(idTrain):
    colors = ["1,1,0","1,0,0","0,1,0","0,1,1","0.1,0.3,1","1,0,1","0.3,0.9,0.9","1,0.5,0","1,0.2,0.2","0.9,0.9,0.9"]
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

    while True:
        nTrain = int(input("\nSet the number of trains: "))
        if (nTrain < MIN_NUMBER_OF_TRAINS or nTrain > MAX_NUMBER_OF_TRAINS):
            print("\nThe number of trains must be between",MIN_NUMBER_OF_TRAINS,"and",MAX_NUMBER_OF_TRAINS)
        else:
            setFileRou()
            break
    
    rbc = Rbc(nTrain)
        
    for i in range(4, nTrain+1):
        addNewTrain(i)
            
    with open('railvc2.rou.xml', 'a') as f:
        f.write('</routes>')
                   
    if options.setParam:
        print("\nSet the parameters of the simulation.")
        print("\nRemember that the distance expressed in SUMO is 10 times greater than the real one: "+
              "to set a distance of 100 meters real you need to enter '10'.")
        while True:
            distCoupling = float(input("\nSet the distance of virtual coupling (default = "+str(rbc.getDistanceCoupling())+"): "))
            if (distCoupling < MIN_DIST_COUP) or (distCoupling > MAX_DIST_COUP):
                print("\nThe distance of virtual coupling must be between ", MIN_DIST_COUP
                      ," and ", MAX_DIST_COUP, ".")
            else:
                rbc.setDistanceCoupling(distCoupling)
                break
        while True:
            distDecoupling = float(input("\nSet the distance of virtual decoupling (default = "+str(rbc.getDistanceDecoupling())+"): "))
            if (distDecoupling < MIN_DIST_DECOUP) or (distDecoupling > MAX_DIST_DECOUP):
                print("\nThe distance of virtual decoupling must be between ", MIN_DIST_DECOUP
                      ," and ", MAX_DIST_DECOUP, ".")
            else:
                rbc.setDistanceDecoupling(distDecoupling)
                break
    
    answer = input("\n\nDo you want change the default speed of the trains? (Y, N) ")
    if answer == 'Y' or answer == 'y':
        changeSpeeds() 

    traci.start([sumoBinary, "-c", "railvc.sumocfg", "--tripinfo-output", "tripinfo.xml"])
    
    rbc.run()
        
    if options.nogui:
        fine = input("\nPress any button to end the simulation.")