# Drone-Swarm
The end goal of this project is to develop a drone swarm which scans an area for certain pre-defined objects and sends back their location to the base station for further action.
## Phase 1 - Leader Follower RC Swarm
The leader drone is manually controlled while the follower drones autonomously follow behind the leader at a safe distance (here 2m).<br> 
The following requirements need to be satisfied before using this - <br>
1. Install ROS Noetic or run the docker files of ROS Noetic<br>
2. Install MAVROS using the following commands:<br>
`sudo apt update`<br>
`sudo apt install ros-noetic-mavros ros-noetic-mavros-extras`<br>
`rosrun mavros install_geographiclib_datasets.sh`<br>
3. Use ESP32 or NODEMCU for telemetry on both the drones and flash the dronebridge software to both the ESPs. Configure both the ESPs as WiFi Clients to the same network that your base station is on. Configure Drone A (leader) to listen on UDP port 14550, and drone B (follower) to UDP port 14551.<br>
To flash the ESPs, use this online tool: https://dronebridge.github.io/ESP32/install.html <br>
To configure the network on the ESPs, connect your base station to the same WiFi and open up this: http://dronebridge.local/ <br>
4. Clone the repo into your workspace and build usint `catkin_make`
5. Open up 3 terminals, source your workspace and run the following commands:<br>
`roslaunch drone_following drone_a.launch`<br>
`roslaunch drone_following drone_b.launch`<br>
`rosrun drone_following follower.py`<br>
6. Additionally, to see if your leader is sending it's pose, run `rostopic echo /drone_a/mavros/local_position/pose`
