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

Also clone the float repo:
```bash
git clone https://github.com/acfrmarine/floats.git --recursive
```
If you don't have the necessary packages for building cameras, VINS-fusion etc. add CATKIN_IGNORE files to the packages that don't work. You should only need the 'float_control' package.


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
roslaunch bffv1_description bffv1_description.launch
```

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

UUV simulator has various ways of adding currents to the simulator [(link)](https://uuvsimulator.github.io/packages/uuv_simulator/docs/tutorials/disturbances/). Here is one way:
```bash
roslaunch uuv_control_utils set_timed_current_perturbation.launch starting_time:=0.0 end_time:=1000.0 current_vel:=0.1 horizontal_angle:=150.0
```