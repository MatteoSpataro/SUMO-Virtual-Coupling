# SUMO-Virtual-Coupling

This project is based on today's need to optimize railway capacity through the Virtual Coupling technique applied to a platoon of trains. The aim is to carry out a simulation of various Virtual Coupling scenarios using the Eclipse SUMO (Simulation of Urban MObility) software, through which it is possible to draw considerations regarding the variations in the capacity of a railway network.

What is SUMO
------------

["Simulation of Urban MObility" (SUMO)](https://sumo.dlr.de/) is an open source, highly portable, microscopic traffic simulation package designed to handle large road networks and different modes of transport.

It is mainly developed by employees of the [Institute of Transportation Systems at the German Aerospace Center](https://www.dlr.de/ts).


Where to get SUMO
---------------

You can download SUMO via our [downloads site](https://sumo.dlr.de/docs/Downloads.html).

As the program is still under development and is extended continuously, we advice you to use the latest sources from our GitHub repository. Using a command line client the following command should work:

        git clone --recursive https://github.com/eclipse/sumo


Execute this project 
---------------

The program can be run in two ways: by clicking on the executable "menu.py" or by entering the name of the program in a terminal within the directory containing the latter. Note that starting the program from the terminal is the only way to access the advanced settings that allow you to change some parameters. To do this, you need to enter the following instruction in the terminal: 

        python menu.py --setParam
        
By default, the program run the SUMO simulation with graphical interface (sumo-gui). If you want to run the simulation without it, you can add the option "--nogui”. Furthermore, if you don't want a simulation with Virtual Coupling, you can add the option "--novc" before you execute the instruction in the terminal.

In the end, if you want to run the simulation with the maximum capacity (30 trains), you have to enter the following instruction in the terminal: 
        
        python menu.py --maxTrains

License
-------

SUMO is licensed under the [Eclipse Public License Version 2](https://eclipse.org/legal/epl-v20.html).
