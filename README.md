# cps_bot_mini_ws

Now working via Docker!
Follow the instructions: To use this you need the latest Docker installed and for the multiplatform build the binfmt dependencies for your system. Otherwise just use the pre-built Docker containers. I will try to make the images available public.

Run:
For instance on your PC:
    docker compose run guis
This will launch the "guis" container where you will find a full ros2 humble installation and all the dependencies this robot needs.

On the robot:
    docker compose up -d controller teleop


### Attention: do not launch docker compose without arguments else you will start all services at once and you won't need them.


Old-fashioned way:

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


### Real-Time priority
Process real-time settings

These are the options that allows to configure the process real-time settings:

    priority: changes the process priority and set a real-time FIFO priority. This will set the priority of the thread where the ROS 2 executor is running.
    cpu-affinity: binds the application to a specific CPU core by setting a CPU mask. For example, to bind the process or thread to CPU 3 (or 2 starting from 0) we set the CPU mask to 4 (100 in binary).
    lock-memory: pre-faults memory until no more page faults are seen. This usually allocated a high amount of memory so make sure there is enough memory in the system.
    lock-memory-size: specifies the amount of memory we want to pre-allocate. For example lock-memory-size 100 pre-allocates 100 MB.
    config-child-threads: specifies if the RMW middleware child threads will inherit the main process settings. This applies for priority and cpu-affinity options.. For example, if config-child-threads False is set, only the main thread where the ROS executor is set with the priority and cpu-affinity options. If, config-child-threads True is set, the DDS threads will also inherit the same priority and CPU affinity configuration than the main thread.

Example using ros2 launch:

ros2 launch pendulum_bringup pendulum_bringup.launch.py priority:=80 cpu-affinity:=4 lock-memory-size:=100 config-child-threads:=True 

Example using the executable command line arguments:

ros2 run pendulum_demo pendulum_demo --priority 80 --cpu-affinity:=4 --lock-memory-size 100 --config-child-threads True 
Source: https://github.com/ros2-realtime-demo/pendulum/blob/rolling/docs/real_time_tutorial.md
