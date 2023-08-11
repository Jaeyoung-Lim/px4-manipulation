# px4-aerial-manipulation


## Setup
Add the repository to the ros2 workspace
```
git clone https://github.com/Jaeyoung-Lim/px4-manipulation.git
```

## Running
You will make use of 3 different terminals to run the offboard demo.

On the first terminal, run a SITL instance from the PX4 Autopilot firmware.
```
make px4_sitl gz_omnicopter
```

On the second terminal terminal, run the micro-ros-agent which will perform the mapping between Micro XRCE-DDS and uORB. So that ROS2 Nodes are able to communicate with the PX4 micrortps_client.
```
micro-ros-agent udp4 --port 8888
```

In order to run the offboard position control example, open a third terminal and run the the node.
This runs two ros nodes, which publishes offboard position control setpoints and the visualizer.
```
ros2 launch px4_manipulation run.launch.py
```
