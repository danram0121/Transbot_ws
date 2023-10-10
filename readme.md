## Packages for Transbot robot

These packages are the foundation for the substation inspection robot.

#### Packages:

- **transbot**
  This package has two launch files **listener\.launch\.py** and **talker\.launch\.py**. It is intended to use as a test between raspberry pi and developer station.
  <br>

- **transbot_bringup**
  This package launches the simulator **gazebo** with the robot state visualizer **rviz**.
  <br>

- **transbot_description**
  This package contains the robot description file including 3D models for the visualizers.
  <br>

#### Complete:

- Robot description model: Base model (frame, wheels)

#### To Do:

- Robot description model: Lidar (RPLidar A1)
- Robot description model: Camera (intel realsense d435)
- - later we will need to integrate the thermal sensor into the description model as well
- Robot control: teleop_twist_keyboard (keyboard directed movement)
- Robot control: teleop_twist_joy (controller directed movement)
- Nav2 package for autonomous navigation
- a function that launches the robot navigation on raspberry pi startup
- a way to control what package is running (auto navigation, keyboard nav, controller nav) as in toggling between packages
