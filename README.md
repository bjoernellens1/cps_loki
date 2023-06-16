# cps_bot_mini_ws
temporary ws for developing ros2 control on robot mini with odrive

needed repos are:
- https://github.com/bjoernellens1/odrive_ros2_control on branch humble-fw-v0.5.1
- https://github.com/bjoernellens1/rmp220_teleop on branch bot_mini
- https://github.com/bjoernellens1/ros2_cam_openCV
- https://github.com/bjoernellens1/bot_mini_bringup.git

For initialization, just call "python3 initialize.py"

TODO: extend bot_mini_bringup, python scripts for simplyfiyng startup process.

## Useful commands:
rosdep install --from-paths src --ignore-src -r -y


### Localization using predefined map and navigation
don't forget to set transient_local in rviz to see the map
ros2 launch bot_mini_bringup nav2.launch.py map_subscribe_transient_local:=true
ros2 launch nav2_bringup localization_launch.py map:=/home/bjorn/Documents/ros_projects/cps_bot_mini_ws/src/bot_mini_bringup/maps/cps_save_map.yaml 

