#!/bin/bash

# Test script to run simulation and capture debug output

cd /Users/dganjali/GitHub/JADE-SpaceApps/gazebo_new

echo "Starting Gazebo simulation with debug output..."
echo "Initial conditions:"
echo "  Chaser: (0, 0, 0)"
echo "  Dock: (15, 5, 3)"
echo "  Expected distance: ~16m"
echo ""
echo "Watch for:"
echo "  - [ThrusterController] configured"
echo "  - [ThrusterController] found target/dock"
echo "  - [ThrusterController] DEBUG output"
echo ""
echo "Press Ctrl+C to stop"
echo "================================"
echo ""

# Run gz sim and capture output
gz sim worlds/leo.sdf --verbose 1
