# Robot Navigation Simulator

## 16520373 / 13520110 - Farrel Ahmad

## Temporary Readme File

### Robot Navigation Simulator using Gazebo Simulator with ROS (Robot Operating System). Tugas Challenge SPARTA HMIF 2020.

<br>

## Launching World
**29 July 2021 Update-1**
1. This program is being developed using ROS Noetic, Gazebo 11, and Ubuntu 20.04 LTS. Try to use these version.

2. Clone the repository
```
$ git clone https://github.com/farrel-a/robot-nav-simulator.git
```

3. cd to `/robot-nav-simulator/nav_ws`
```
$ cd robot-nav-simulator/nav_ws
```

4. run `catkin_make`
```
$ catkin_make
```

5. source `setup.bash`
```
$ source devel/setup.bash
```

6. run launch command
```
$ roslaunch navrobot_gazebo navrobot.launch
```

**30 July 2021 Update-1**

7. The result will look like this. It will spawn turtlebot3_burger at x = 0.0, y = 0.0, z = 0.0 and then go to (2.0 ; 2.0).

![](photos/launch.png)

![](photos/move.gif)