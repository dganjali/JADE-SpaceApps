# LEO Orbital Docking Simulation

This project simulates autonomous orbital docking between a chaser satellite and a passive dock module in Low Earth Orbit (LEO) using Ignition Gazebo.

## Project Structure

```
gazebo_new/
├── worlds/
│   └── leo.sdf                      # Main simulation world with zero gravity
├── models/
│   ├── dock/                        # Passive dock module
│   │   ├── model.config
│   │   └── model.sdf
│   └── chaser/                      # Active chaser satellite with thrusters
│       ├── model.config
│       └── model.sdf
├── src/
│   └── thruster_controller.cpp     # PD controller plugin for autonomous docking
├── CMakeLists.txt
└── README.md
```

## Features

- **Zero Gravity Environment**: Simulates space conditions with no gravitational forces
- **Autonomous Docking**: PD (Proportional-Derivative) controller guides the chaser to the dock
- **Random Dock Motion**: The dock moves with random initial velocity to simulate realistic scenarios
- **Visual Feedback**: Both models have distinct colors and thruster markers
- **Success Detection**: Automatically detects when docking is successful (within 0.3m threshold)

## Prerequisites

- **Ignition Gazebo** (version 3, 4, 5, or 6)
  - Check your version: `gz sim --version` or `ign gazebo --version`
- **CMake** (>= 3.10)
- **C++ compiler** with C++17 support
- **ignition-cmake2** package

### Install Dependencies (Ubuntu/Debian)

```bash
# For Ignition Fortress (gazebo6)
sudo apt-get install libignition-gazebo6-dev ignition-gazebo6

# Or for other versions, replace '6' with your version number
```

### Install Dependencies (macOS with Homebrew)

```bash
brew install ignition-fortress
# or
brew install ignition-edifice
```

## Build Instructions

1. **Create a build directory:**

```bash
cd gazebo_new
mkdir build
cd build
```

2. **Configure with CMake:**

```bash
cmake ..
```

**Note**: If CMake cannot find `ignition-gazebo6`, adjust the version in `CMakeLists.txt`:
- Change `find_package(ignition-gazebo6 REQUIRED)` to match your installed version
- For example: `ignition-gazebo5`, `ignition-gazebo4`, etc.

3. **Build the plugin:**

```bash
make
```

4. **Install the plugin (optional):**

```bash
sudo make install
```

Or copy the built library manually:

```bash
# Copy to a location in your GZ_SIM_SYSTEM_PLUGIN_PATH
cp libthruster_controller.so ~/.gz/sim/plugins/
# or
cp libthruster_controller.dylib ~/.gz/sim/plugins/  # macOS
```

## Run Instructions

### Set Environment Variables

Before running, ensure Gazebo can find your models and plugins:

```bash
# Make the run script executable
chmod +x run_simulation.sh

# Run the simulation
./run_simulation.sh
```

**Or manually:**

```bash
# Set model path (from gazebo_new directory)
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$(pwd)/models

# Set plugin path to build directory
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$(pwd)/build

# Launch Gazebo
gz sim worlds/leo.sdf
```

### What to Expect

1. The simulation will start with:
   - A large Earth sphere in the background (for visual reference)
   - The **chaser** (blue box) at the origin (0, 0, 0)
   - The **dock** (orange cube) positioned ~16 meters away at (15, 5, 3)

2. The dock will begin moving with a random velocity (~0.5 m/s) in a random direction

3. The chaser will automatically activate its thruster controller and pursue the moving dock

4. Watch the dramatic chase as the chaser intercepts the dock!

5. Console output will show:
   - Initial configuration messages
   - Periodic distance and force updates every 0.2 seconds
   - Success message when docking is achieved (distance < 0.5m)

## Controller Parameters

You can tune the controller by editing `worlds/leo.sdf`. Key parameters in the `<plugin>` section:

```xml
<reach_distance>0.5</reach_distance>      <!-- Success threshold in meters -->
<kp>2.0</kp>                               <!-- Proportional gain -->
<kd>1.2</kd>                               <!-- Derivative gain -->
<max_force>20.0</max_force>                <!-- Maximum thruster force (N) -->
<dock_init_speed>0.5</dock_init_speed>     <!-- Dock's initial random speed (m/s) -->
```

### Tuning Tips

- Increase `kp` for faster approach (but risk overshooting)
- Increase `kd` for better damping (smoother approach)
- Adjust `max_force` based on the chaser's mass (10 kg by default)
- Higher `dock_init_speed` makes the scenario more challenging

## Troubleshooting

### Plugin not loading

1. Check that the plugin library exists in the build directory
2. Verify `GZ_SIM_SYSTEM_PLUGIN_PATH` includes the correct path
3. Check console output for plugin loading errors

### Models not found

1. Ensure `GZ_SIM_RESOURCE_PATH` includes the `models` directory
2. Verify model.config and model.sdf files are in the correct structure

### CMake version mismatch

If you see errors about `ignition-gazebo6` not found:
1. Check your installed version: `pkg-config --modversion ignition-gazebo6`
2. Update `CMakeLists.txt` to use the correct version number

### Controller not working

1. Check console output for initialization messages
2. Verify both models are loaded correctly
3. Ensure physics is running (check simulation time is advancing)

## Advanced Usage

### Modify Model Properties

Edit the model SDF files to change:
- **Mass and inertia**: In `<inertial>` tags
- **Visual appearance**: In `<visual>` tags (colors, geometry)
- **Collision geometry**: In `<collision>` tags

### Add More Features

Consider extending the simulation with:
- Attitude control (rotation matching)
- Sensor-based guidance (cameras, lidar)
- Multiple chaser satellites
- Orbital mechanics (gravity gradient, atmospheric drag)
- Collision detection and soft-docking dynamics

## License

This project is provided as-is for educational and research purposes.

## Credits

Created for the JADE SpaceApps Challenge project.
