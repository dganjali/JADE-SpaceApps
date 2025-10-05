#!/bin/bash

# Set environment variables for Gazebo
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$(pwd)/build

echo "================================================"
echo "LEO Orbital Docking Simulation"
echo "================================================"
echo "Models path: $(pwd)/models"
echo "Plugin path: $(pwd)/build"
echo "================================================"
echo ""
echo "Starting Gazebo simulation..."
echo ""

# Run Gazebo with the LEO world
gz sim worlds/leo.sdf
