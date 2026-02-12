# Lunabotics Nav Stack
must have ROS2 Humble installed. If it's not then follow this guide https://docs.ros.org/en/humble/Installation.html
Add to .bashrc to avoid having to source humble in every terminal: echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

## Clone Repo
git clone https://github.com/Symonowicz/Lunabot_3D_lidar_mapping.git
cd ros2_lidar_ws

## Stack
- MID360
- Point-LIO
- Odom Bridge
- slam_toolbox
- Nav2

## Build Dependencies 
- install required dependencies
./scripts/install_deps.sh
- install ws specific dependencies
rosdep install --from-paths src --ignore-src -r -y

## Run Order for Livox

1. Mid360 Driver  
>>ros2 launch livox_ros_driver2 msg_MID360_launch.py 
2. Point_lio
>>colcon build --packages-select point_lio --symlink-install --event-handlers console_direct+
>>ros2 launch point_lio mapping_mid360.launch.py
3. TF
>>colcon build --packages-select robot_bringup --symlink-install --event-handlers console_direct+
>>ros2 launch robot_bringup frames.launch.py
4. odom_bridge
>>ros2 run robot_bringup odom_bridge
5. pointcloud_to_laserscan
>>ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node --ros-args -r __node:=pcl2scan -r cloud_in:=/cloud_registered_body   -r scan:=/scan   -p target_frame:=base_link   -p transform_tolerance:=0.5   -p min_height:=-0.2   -p max_height:=0.2   -p queue_size:=400 -p range_min:=0.0 -p range_max:=7.0
6. slam_toolbox
>>colcon build --packages-select robot_bringup --symlink-install --event-handlers console_direct+ 
>>ros2 launch robot_bringup slam.launch.py
7. Nav2 Rviz
>>ros2 launch nav2_bringup rviz_launch.py
8. Nav2 Navigation 
>>ros2 launch nav2_brinup navigation_launch.py
The values in the transforms are different depending whether you're using the Unitree or Livox
