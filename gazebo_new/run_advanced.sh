#!/bin/bash

# run_advanced.sh - Launch advanced orbital controller with sensor models and thruster allocation

echo "🚀 Starting Advanced Orbital Docking Simulation"
echo "================================================"
echo "Features:"
echo "  ✓ Sensor Models (rangefinder, IMU, camera noise)"
echo "  ✓ Thruster Allocation Matrix (8 thrusters per spacecraft)"
echo "  ✓ Two-Body Orbital Gravity"
echo "  ✓ Hill's Equations"
echo "  ✓ PD Control with Optimal Gains"
echo "  ✓ Random Scenarios (~30m separation)"
echo "================================================"
echo ""

# Randomize spacecraft positions
echo "🎲 Generating random scenario..."
./randomize_poses.py
echo ""

# Set plugin paths
export GZ_SIM_SYSTEM_PLUGIN_PATH=$(pwd)/build
export GZ_GUI_PLUGIN_PATH=$(pwd)/build

# Run simulation with custom GUI
gz sim worlds/leo_advanced.sdf --gui-config gui_dashboard.config -r
