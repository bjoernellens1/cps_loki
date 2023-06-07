import subprocess
import signal
import os
import time

processes = []

def source_setup_files():
    subprocess.Popen("source /opt/ros/humble/setup.bash", shell=True, executable="/bin/bash")
    #subprocess.Popen("source ./install/setup.bash", shell=True, executable="/bin/bash")
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

# Add your commands here
commands = [
    "colcon build"
]

# Register the signal handler
signal.signal(signal.SIGINT, handle_interrupt)

# Source the setup files
source_setup_files()

# Start the remaining processes
for command in commands:
    start_process(command)

# Wait for user input to stop the processes
input("Press Enter to stop all processes...")

# Stop all processes
stop_processes()
