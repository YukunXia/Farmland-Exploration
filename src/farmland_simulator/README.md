# Farmland Simulator

## Instructions to Measure Robot Dimensions

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