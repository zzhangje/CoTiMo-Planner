#!/bin/bash

cd /ws

# Build the workspace
catkin_make

# Source the setup file
source devel/setup.bash

# Check if the cyber-planner package exists
if [ -d "src/cyber-planner" ]; then
  # Check if the demo.launch file exists
  if [ -f "src/cyber-planner/launch/cyber-planner.launch" ]; then
    # Launch the demo
    roslaunch cyber-planner cyber-planner.launch
  else
    echo "Error: demo.launch file not found in cyber-planner package."
  fi
else
  echo "Error: cyber-planner package not found in the workspace."
fi