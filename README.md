# Float Simulation

This repo contains the files necessary for simulating the bottom following floats in the [UUV Simulator](https://uuvsimulator.github.io/).

## Requirements
- ROS (tested on Melodic)
- Gazebo (tested on Gazebo 9)
- UUV Simulator


## Building

Move to the source directory of your catkin workspace:
```bash
cd ~/WORKSPACE/src
```

Clone this repo:
```bash
git clone https://github.com/jacksonhshields/float_simulation.git
```

Also clone the float repos:
```bash
git clone https://github.com/acfrmarine/float_ros.git
git clone https://github.com/acfrmarine/float_ros_msgs.git
git clone https://github.com/acfrmarine/bff_ros.git  # Add CATKIN_IGNORE TO ALL PACKAGES EXCEPT MSG PACKAGES 
```


Build the catkin workspace:
```bash
cd ~/WORKSPACE
catkin_make
```

## Usage

### Basic Demonstration

First, launch an instance of the 'ocean waves' world in uuv simulator:
```bash
roslaunch uuv_gazebo_worlds ocean_waves.launch
```

Next, spawn the float into this world (remember to source the workspace), this also launches controllers. All topics are mapped to be the same as in the float.
```bash
roslaunch bffv2_description bffv2_description.launch
```
#### Basic Control Calls
Similar to the real float, run a depth controller by calling the following service:
```bash
rosservice call /set_depth_target "depth_target: 10.0
timeout: 30.0"
```
To run the altitude controller:
```bash
rosservice call /set_altitude_target "altitude_target: 2.0
timeout: 60.0"
```

#### Float Missions

##### Adding A Mission
Adding the mission is done through a service call to /float2/add_mission, with the type bff_ros_msgs/add_float_mission.msg.
```bash
rosservice call /float2/add_mission "mission_stamped:
  header:
    seq: 0
    stamp:
      secs: 0
      nsecs: 0
    frame_id: ''
  mission:
    mission_id: {uuid: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}
    float_name: 'float2'
    altitude: 10.0
    bottom_duration: 0.0
    mission_duration: 5.0
    dive_abort_duration: 0.0
    abort_depth: 10.0
    abort_time: {secs: 0, nsecs: 0}"
```

##### Notify Float Deploying
This service is called immediately before the float is to be deployed. The cameras and localisation will start immediately. The thrusters will start once the wait time is elapsed.
```bash
rosservice call /float2/notify_deploying "wait: 30.0"
```


##### Notify Float Recovered
When the float is recovered, call the notify float recovered service. The float will respond with data from its mission.
```bash
rosservice call /float2/notify_recovered "surface_vessel_name: 'workboat1'"
```

### Currents
UUV simulator has various ways of adding currents to the simulator [(link)](https://uuvsimulator.github.io/packages/uuv_simulator/docs/tutorials/disturbances/). Here is one way:
```bash
roslaunch uuv_control_utils set_timed_current_perturbation.launch starting_time:=0.0 end_time:=1000.0 current_vel:=0.1 horizontal_angle:=150.0
```