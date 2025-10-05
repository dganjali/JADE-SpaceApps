#!/bin/bash

echo "================================================"
echo "Gazebo LEO Docking - System Check"
echo "================================================"
echo ""

# Check Gazebo installation
if command -v gz &> /dev/null; then
    echo "✅ Gazebo installed: $(gz sim --version)"
else
    echo "❌ Gazebo not found! Install with: brew install gz-sim"
    exit 1
fi

# Check if built
if [ -f "build/libthruster_controller.dylib" ]; then
    echo "✅ Plugin built: $(ls -lh build/libthruster_controller.dylib | awk '{print $5}')"
else
    echo "❌ Plugin not built! Run: mkdir -p build && cd build && cmake .. && make"
    exit 1
fi

# Check models
if [ -f "models/dock/model.sdf" ] && [ -f "models/chaser/model.sdf" ]; then
    echo "✅ Models present: dock, chaser"
else
    echo "❌ Models missing!"
    exit 1
fi

# Check world file
if [ -f "worlds/leo.sdf" ]; then
    echo "✅ World file: worlds/leo.sdf"
else
    echo "❌ World file missing!"
    exit 1
fi

echo ""
echo "================================================"
echo "🚀 System ready! Run: ./run_simulation.sh"
echo "================================================"
