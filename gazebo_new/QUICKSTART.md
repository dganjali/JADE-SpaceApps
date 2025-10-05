# Quick Start Guide - LEO Orbital Docking Simulation

## ✅ What We Fixed

1. **CMake Configuration**: Updated from `ignition-gazebo6` to `gz-sim` (Gazebo Harmonic)
2. **C++ API Updates**: Migrated from `ignition::` namespace to `gz::` namespace
3. **Plugin Registration**: Updated macro from `IGNITION_ADD_PLUGIN` to `GZ_ADD_PLUGIN`
4. **Entity API**: Fixed entity iteration and validation (`.Valid()` → `kNullEntity` checks)
5. **Mass Accessor**: Changed from `.Mass()` to `.MassMatrix().Mass()`
6. **Plugin Naming**: Updated SDF to use correct plugin name `ThrusterController` and `.dylib` extension
7. **Lighting**: Enhanced ambient lighting and added fill lights for better visibility
8. **Camera**: Added default GUI camera position for better initial view

## 🚀 Build & Run

From the `gazebo_new` directory:

```bash
# 1. Build (only needed once or after code changes)
mkdir -p build
cd build
cmake ..
make
cd ..

# 2. Run the simulation
./run_simulation.sh
```

## 🎮 What to Expect

1. **Gazebo launches** with a dark space environment
2. **Orange dock** (20kg cube) at origin with random initial velocity (~0.2 m/s)
3. **Blue chaser** satellite (10kg) starts 5 meters away on the X-axis
4. **PD controller** automatically activates and guides chaser toward dock
5. **Console output** shows:
   - Plugin configuration messages
   - Periodic distance and force updates
   - Success message when distance < 0.3m

## 🎯 Features

- ✅ Zero-gravity physics simulation
- ✅ PD (Proportional-Derivative) autonomous controller
- ✅ Random dock motion for realistic challenge
- ✅ Real-time distance and force monitoring
- ✅ Automatic success detection
- ✅ Visual thruster markers on chaser
- ✅ Configurable controller parameters via SDF

## ⚙️ Tuning Parameters

Edit `worlds/leo.sdf` to adjust:

```xml
<kp>1.5</kp>              <!-- Proportional gain (approach speed) -->
<kd>0.8</kd>              <!-- Derivative gain (damping) -->
<max_force>15.0</max_force> <!-- Maximum thruster force (N) -->
<dock_init_speed>0.2</dock_init_speed> <!-- Initial dock velocity (m/s) -->
<reach_distance>0.3</reach_distance>   <!-- Success threshold (m) -->
```

## 🔧 Troubleshooting

### Models not visible
- Check that `GZ_SIM_RESOURCE_PATH` includes the models directory
- Try adjusting camera view with mouse: Left-drag to rotate, Right-drag to pan, Scroll to zoom

### Plugin not loading
- Verify `GZ_SIM_SYSTEM_PLUGIN_PATH` includes the build directory
- Check console for error messages
- Ensure library was built successfully: `ls build/libthruster_controller.dylib`

### No movement
- Check console output for controller messages
- Verify both models are loaded (check entity manager)
- Try increasing `kp` gain for faster approach

## 📊 System Information

- **Gazebo Version**: 10.0.0 (Harmonic)
- **Platform**: macOS with Homebrew
- **Physics Engine**: ODE with 1kHz update rate
- **Chaser Mass**: 10 kg
- **Dock Mass**: 20 kg

## 🎓 Educational Notes

This simulation demonstrates:
- **Orbital mechanics** in zero-gravity
- **PD control** for autonomous navigation
- **Relative motion** between two spacecraft
- **Force-limited propulsion** (realistic thruster constraints)
- **Moving target** interception and rendezvous

Perfect for testing autonomous docking algorithms in a simplified LEO environment!
