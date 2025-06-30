## ABB IRB4600 + "MiR1350" + Other Components 
Required Packages for this to work: 
- `moveit2` 
- `abb_arm_moveit_config` (in my repos) 
- `gz_ros2_control` (changes made, look at `my_ur` README for more info)
- `slam_toolbox`
- `nav2_bringup` 
- `stability_checker` (package in my Github repos)

If all dependencies above are met, go ahead and build the package.
```
colcon build --packages-select abb_arm_description
source install/local_setup.bash
```
---
### Capabilities
This is a simulation of a ABB IRB4600 robot on a "MiR1350". It has a LIDAR and mapping/navigation/localization capabilities using `slam_toolbox` and `Nav2`.
The ABB arm is also capable of moving in simulation, using `Moveit`.

---
### Before using
Two changes to the code need to be made in order to work for your local machine.
1. In file *launch/abb_sim_control.launch.py*, look for `map_yaml_file`. Change the file path name to the absolute path to *maps/my_map_save.yaml*
2. In file *config/mapper_params_online_async.yaml*, look for `map_file_name`. Change the file path name to the absolute path to *maps/my_map_serial.pgm*

---
#### How to map with Mobile Robot (with slam_toolbox)
0. Change directory into the maps folder `cd abb_arm_description/maps`
1. Run `ros2 launch abb_arm_description abb_sim_control.launch.py mapping_mode:='true'`
2. Wait for Rviz, Gazebo and `slam_toolbox` to launch. (~6 seconds)
3. In another terminal, run `ros2 run teleop_twist_keyboard teleop_twist_keyboard` to control the robot
4. Move the robot around the simulation. Take note that the map will start building in Rviz
5. Once map is complete, on Rviz:
  - Click on "Panels" Tab -> "Add New Panel"
  - Look for SlamToolboxPlugin, under `slam_toolbox`
  - Add the panel, it should appear on the left side of Rviz
6. Save map and serialize map as "my_map_save" and "my_map_serial" respectively. 
  - Naming it any other map will require you to look for all instances and replace accordingly
7. Map saved! Now, `amcl` in `nav2` can use it.

---
#### How to navigate with Mobile Robot (with Nav2 amcl)
0. Change directory into your workstation. Ensure that there is a map in the maps folder that `Nav2` can use.
1. Run `ros2 launch abb_arm_description abb_sim_control.launch.py mapping_mode:='false'`
2. Wait for Rviz, Gazebo and `Nav2` to launch. (~6 seconds)
3. Using *2D Pose Estimate*, set the initial pose of the robot for `amcl`. 
  - Tip: uncheck RobotModel, enable TF axes and find robot origin point
  - RobotModel should now be visible in Rviz, once initial pose is set
4. Using *2D Goal Pose*, set the goal pose for the robot
5. Let robot navigate to desired goal. 

---
#### How to control ABB arm (with Moveit)
