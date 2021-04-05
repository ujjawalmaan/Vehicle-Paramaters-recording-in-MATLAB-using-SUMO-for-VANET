# Vehicle-Paramaters-recording-in-MATLAB-using-SUMO-for-VANET

This repository is the part of my PhD work for FOG nodes offloading using Deep Reinforcement learning. During that, we have used the SUMO Traci API for MATLAB to record the vehicle parameters. The code for this part is shared here. The steps to load the SUMO map and configuration files for MATLAB are shared here.

### Requirement: SUMO, Traci API configured for MATLAB (instructions can be followed from the API)

### Steps: 
1. download the .osm file from openstreetmap 
2. convert the .osm file in network file for sumo as
"C:\Program Files (x86)\Eclipse\Sumo\bin\netedit.exe" --osm-files "map.osm" -o "network.net.xml"
3. create vehicle mobility randomly by randomTrips.py in sumo
python "C:\Program Files (x86)\Eclipse\Sumo\tools\randomTrips.py" -n "network.net.xml" -r "vehicle.rou.xml" -e 10
4.create the configuration file. copy the template from https://sumo.dlr.de/docs/Tutorials/Hello_Sumo.html and change the network xml, route file name
5. To let n vehicles depart between times t0 and t1 set the options

-b t0 -e t1 -p ((t1 - t0) / n)

-->for 100 vehicles 
python "C:\Program Files (x86)\Eclipse\Sumo\tools\randomTrips.py" -n "network.net.xml" -r "vehicle.rou.xml" -b 0 -e 10 -p 0.1
