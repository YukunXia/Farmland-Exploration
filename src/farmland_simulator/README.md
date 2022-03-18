# Farmland Simulator

## Instructions to Measure Robot Dimensions

### Overall Dimensions

- Edit `/opt/ros/noetic/share/husky_description/urdf/husky.urdf.xacro` line 50
    - change 0 to 1
    - Line 50 becomes `  <xacro:arg name="laser_3d_enabled" default="$(optenv HUSKY_LASER_3D_ENABLED 1)" />`
- Run `xacro /opt/ros/noetic/share/husky_description/urdf/husky.urdf.xacro -o model.urdf`
- Install collada_urdf by `sudo apt-get install ros-noetic-collada-urdf`
- Run `rosrun collada_urdf urdf_to_collada model.urdf model.dae`
- Open Blender
    - Import `model.dae`
    - Select all robot components, and join them together
    - Press `N` to show the properties panel, and read the dimensions
        - Reference: https://blender.stackexchange.com/questions/31367/how-to-set-object-dimension

With Velodyne VLP-16, the dimensions of Husky are
    - x: 0.9854 m
    - y: 0.6851 m
    - z: 1.08983 m

### Component Dimensions

Read `model.urdf`, look for lines like `<cylinder length="0.1143" radius="0.1651"/>`, from which we know that the radii of wheels are all 0.1651.


## Transformations

### world__T__base_link

Gazebo magic returns the absolute pose of the robot `base_link`, i.e. $T_{base_link}^{world}$. Point cloud center is at `velodyne` link. The transformation between these two links is defined in the tf tree.

The default setting in `husky_velodyne_spawn.launch` assigns the robot pose to be:
```xml
  <arg name="robot_init_x" default="20.0"/>
  <arg name="robot_init_y" default="20.0"/>
  <arg name="robot_init_z" default="0.0"/>
  <arg name="robot_init_yaw" default="0.0"/>
```
, and from Gazebo magic, initially robot pose is
```yaml
position:
  x: 20.000076757763342
  y: 19.99997709945979
  z: 0.13227261495344791
orientation:
  x: 3.2100818533505595e-06
  y: 4.226063762422636e-08
  z: 2.1034668619371185e-06
  w: 0.9999999999926346
```

Initial z is not 0, but ~0.13227. That may come from the height of base_link from the robot bottom, verified by the following lines from `model.urdf`.

```xml
  <!-- Base footprint is on the ground under the robot -->
  <joint name="base_footprint_joint" type="fixed">
    <origin rpy="0 0 0" xyz="0 0 -0.13228"/>
    <parent link="base_link"/>
    <child link="base_footprint"/>
  </joint>
```

### base_link__T__velodyne

```
 1      0      0 0.0812
 0      1      0      0
 0      0      1 0.9127
 0      0      0      1
```