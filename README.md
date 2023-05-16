## THIS REPO IS MADE FOR BACHELOR PROJECT ANOMALY DETECTION AND MODEL SWAPPING
##  [Aarhus F1Tenth Autonomous Racing Team](https://ece.au.dk/en/collaboration/collaboration-with-engineering-students/autonomous-racing-team-f1tenth) 

# F1TENTH Racecar Simulator

This is a lightweight 2D simulator of the UPenn F1TENTH Racecar.
It can be built with ROS, or it can be used as a standalone C++ library.

This repo contains the algorithm for anomaly detection and model swapping. In this case anomaly detection is slip detection.
The official f1tenth website
https://f1tenth.readthedocs.io/en/stable/

## ROS

### Dependencies

If you have ```ros-melodic-desktop``` installed, the additional dependencies you must install are:

- tf2_geometry_msgs
- ackermann_msgs
- joy
- map_server

You can install them by running:

    sudo apt-get install ros-melodic-tf2-geometry-msgs ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-map-server

The full list of dependencies can be found in the ```package.xml``` file.

### Installation

To clone the repo with the simulator and slip detection into your catkin workspace:

    cd ~/catkin_ws/src
    git clone https://github.com/kevinschoenberg/f1tenth_sim.git
Then navigate to ```/f1tenth_sim```:

    cd ~/catkin_ws/src/f1tenth_sim

Change parameters in ```run_test.sh```.
Then run ```run_test.sh``` to build it and start simulation experiment(s):

    ./run_test.sh

## Based on f1tenth simulator and read to further follow the link
    https://github.com/f1tenth/f1tenth_simulator

