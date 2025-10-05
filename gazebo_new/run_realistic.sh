#!/bin/bash

echo "=================================================="
echo "  REALISTIC LEO ORBITAL DOCKING SIMULATION"
echo "=================================================="
echo ""
echo "This simulation includes:"
echo "  ✓ Two-body orbital gravity"
echo "  ✓ Hill's equations (Clohessy-Wiltshire)"
echo "  ✓ Realistic thruster physics"
echo "  ✓ Propellant consumption (Tsiolkovsky)"
echo "  ✓ PD control with orbital compensation"
echo ""
echo "Optional (configurable):"
echo "  • J2 perturbation (Earth oblateness)"
echo "  • Atmospheric drag"
echo ""
echo "=================================================="
echo ""

# Get the directory of this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Check if build exists
if [ ! -d "$SCRIPT_DIR/build" ]; then
    echo "❌ Build directory not found. Building now..."
    mkdir -p "$SCRIPT_DIR/build"
    cd "$SCRIPT_DIR/build"
    cmake ..
    make
    cd "$SCRIPT_DIR"
    echo ""
fi

# Check if plugin exists
if [ ! -f "$SCRIPT_DIR/build/librealistic_orbital_controller.dylib" ]; then
    echo "❌ Plugin not found. Building now..."
    cd "$SCRIPT_DIR/build"
    make
    cd "$SCRIPT_DIR"
    echo ""
fi

# Set plugin path
export GZ_SIM_SYSTEM_PLUGIN_PATH="$SCRIPT_DIR/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$SCRIPT_DIR/build:$IGN_GAZEBO_SYSTEM_PLUGIN_PATH"

echo "🚀 Launching realistic orbital simulation..."
echo "📂 Plugin path: $SCRIPT_DIR/build"
echo "🌍 World: leo_realistic.sdf"
echo ""
echo "Watch the console for real-time telemetry:"
echo "  [T+X.Xs] dist=XX.XXm rel_v=X.XXm/s thrust=X.XXN fuel=X.XXXkg"
echo ""
echo "=================================================="
echo ""

# Run Gazebo
gz sim "$SCRIPT_DIR/worlds/leo_realistic.sdf" -v 3

# Alternative if gz sim doesn't work:
# ign gazebo "$SCRIPT_DIR/worlds/leo_realistic.sdf" -v 3
