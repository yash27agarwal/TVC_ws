#!/bin/bash

# Navigate to the workspace directory
cd ~/px4_ws

# Build the workspace
echo "Building workspace..."
echo ">>>colcon build"
colcon build

# Check if build was successful
if [ $? -eq 0 ]; then
    echo "Build successful! Sourcing setup file..."
    echo ">>>source install/setup.bash"

    # Source the setup file
    source install/setup.bash
    

    # Launch the LQR controller
    echo "Launching TVC controller..."
    ros2 launch tvc_controller lqr.launch.py
else
    echo "Build failed! Aborting launch."
    exit 1
fi