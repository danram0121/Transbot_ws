## Packages for Transbot robot

These packages are the foundation for the substation inspection robot.

#### Recommendations for install

Make sure you create a directory for the packages in this repo. I am using a directory named **Transbot_ws**. In the **Transbot_ws** there is a **src** folder and all the packages in this repo are intended to be in that **src** directory.
<br>

Remember to build in the Transbot_ws directory whenever a new file is created

```
    colcon build
```

Colcon build with symlink will let you skip building only if no new files have been created.

```
    colcon build --symlink-install
```

#### Packages:

- **transbot**
  This package has two launch files **listener\.launch\.py** and **talker\.launch\.py**. It is intended to use as a test between raspberry pi and developer station.
  <br>

  to run talker

  ```
    cd
    cd Transbot_ws
    source install/setup.bash
    ros2 launch transbot_test talker.launch.py
  ```

  <br>

  to run listener

  ```
    cd
    cd Transbot_ws
    source install/setup.bash
    ros2 launch transbot_test listener.launch.py
  ```

  <br>

- **transbot_bringup**
  This package launches the simulator **gazebo** with the robot state visualizer **rviz**.
  <br>

  to run

  ```
    cd
    cd Transbot_ws
    source install/setup.bash
    ros2 launch transbot_bringup transbot_gazebo.launch.xml
  ```

  <br>

- **transbot_description**
  This package contains the robot description file including 3D models for the visualizers.
  <br>

  to run

  ```
    cd
    cd Transbot_ws
    source install/setup.bash
    ros2 launch transbot_description display.launch.xml
  ```

  <br>

#### Complete:

- Robot description model: Base model (frame, wheels)
- Robot description model: Lidar (RPLidar A1)

#### To Do:

- Robot description model: Camera (intel realsense d435)
- - later we will need to integrate the thermal sensor into the description model as well
- Robot control: teleop_twist_keyboard (keyboard directed movement)
- Robot control: teleop_twist_joy (controller directed movement)
- Nav2 package for autonomous navigation
- a function that launches the robot navigation on raspberry pi startup
- a way to control what package is running (auto navigation, keyboard nav, controller nav) as in toggling between packages
