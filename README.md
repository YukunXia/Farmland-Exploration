## Environments

- farmland_with_walls.world
![farmland_with_walls](images/farmland_with_walls.jpg)

## Prerequisites

- Install Husky packages
    - https://www.clearpathrobotics.com/assets/guides/kinetic/ros/Drive%20a%20Husky.html

## Instructions

- Run simulation via the following commands

```
catkin_make
source devel/setup.bash
roslaunch farmland_simulator husky_farmland.launch
```

- Move the robot by, for example,

```
rostopic pub /husky_velocity_controller/cmd_vel geometry_msgs/Twist "linear:
        x: 0.5
        y: 0.0
        z: 0.0
angular:
        x: 0.0
        y: 0.0
        z: 0.0" -r 10
```