import subprocess
import signal
import os
import time

processes = []
current_directory = os.getcwd()

def source_setup_files():
    subprocess.Popen("source /opt/ros/humble/setup.bash", shell=True, executable="/bin/bash")
    subprocess.Popen("source ./install/setup.bash", shell=True, executable="/bin/bash")
    print("Setup files sourced")

def start_process(command):
    process = subprocess.Popen(command, shell=True, executable="/bin/bash", preexec_fn=os.setsid)
    processes.append(process)
    print(f"Started process with command: {command}")

def stop_processes():
    for process in processes:
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    print("Stopped all processes")

def handle_interrupt(signal, frame):
    print("Keyboard interrupt detected")
    stop_processes()
    exit(0)

# Define the base command
base_command = "ros2 launch bot_mini_bringup"

# Define the commands to launch
launch_commands = [
    "rsp.launch.py",
    "robot_controller.launch.py",
    "robot_joy_teleop.launch.py",
    "robot_twist_mux.launch.py",
    "robot_lidar.launch.py",
    #"robot_cam.launch.py"
]

# Create the commands by joining the base command with each launch command
commands = [f"{base_command} {command}" for command in launch_commands]

# Add the path to the map file
map_file = os.path.join(current_directory, "src", "bot_mini_bringup", "maps", "default_map_save.yaml")
map_command = f"ros2 launch nav2_bringup localization_launch.py map:={map_file}"
commands.append(map_command)

# Register the signal handler
signal.signal(signal.SIGINT, handle_interrupt)

# Source the setup files
source_setup_files()

# Start the processes
for command in commands:
    start_process(command)

# Wait for user input to stop the processes
input("Press Enter to stop all processes...")

# Stop all processes
stop_processes()
