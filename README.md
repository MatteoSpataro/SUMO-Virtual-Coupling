# SUMO-Virtual-Coupling
The goal is to create a program that allows you to simulate multiple Virtual Coupling scenarios in the SUMO environment.

What is SUMO
------------

["Simulation of Urban MObility" (SUMO)](https://sumo.dlr.de/) is an open source, highly portable, microscopic traffic simulation package designed to handle large road networks and different modes of transport.

It is mainly developed by employees of the [Institute of Transportation Systems at the German Aerospace Center](https://www.dlr.de/ts).


Where to get it
---------------

You can download SUMO via our [downloads site](https://sumo.dlr.de/docs/Downloads.html).

As the program is still under development and is extended continuously, we advice you to use the latest sources from our GitHub repository. Using a command line client the following command should work:

        git clone --recursive https://github.com/eclipse/sumo


Execute the program
---------------

The program can be run in two ways: by clicking on the executable "menu.py" or by entering the name of the program in an open terminal within the directory containing the latter. Note that starting the program from the terminal is the only way to access the advanced settings that allow you to change some parameters. To do this, you need to enter the following instruction in the terminal: “python menu.py --setParam”.
By default, the program run the SUMO simulation with graphical interface (sumo-gui). If you want to run the simulation without it, you can enter the following instruction in the terminal: “python menu.py --nogui” or enter “python menu.py --setParam --nogui”.


License
-------

SUMO is licensed under the [Eclipse Public License Version 2](https://eclipse.org/legal/epl-v20.html).
