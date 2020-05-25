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

Next, spawn the float into this world (remember to source the workspace):
```bash
roslaunch bffv1_description upload.launch
```

To test controlling the float, run the following node:
```bash
rosrun bff_sim_utils publish_thrusters.py _cmd:=100
```
Where cmd is the thruster command in the range -1000,1000. positive currently thrusts down.
