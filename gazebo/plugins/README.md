# Thruster Controller Plugin

This Gazebo Sim system plugin listens to per-thruster topics and applies motion in the direction of each thruster link’s +Z axis. It’s a lightweight training scaffold for later replacement with a proper wrench/force application.

Topics:
- /model/<model_name>/thrusters/<thruster_link_name> (double) — thrust magnitude in Newtons
- /model/<model_name>/thrusters/zero (bool) — clear all thrust commands

Note: For simplicity this uses a world linear velocity command scaled to approximate acceleration. Replace with a wrench-based approach for physical accuracy.

## Build

```bash
# From repo root
mkdir -p gazebo/plugins/build && cd gazebo/plugins/build
cmake ..
make -j
```

Export library path for Gazebo to discover the plugin:

```bash
# Bash/zsh
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD"
# Or, for Ignition Gazebo
export IGN_GAZEBO_SYSTEM_PLUGIN_PATH="$PWD"
```

## Run

Ensure the model path and plugin path are exported, then launch a world:

```bash
# If using model URI
export GZ_SIM_RESOURCE_PATH="$(pwd)/../../models"  # adjust to absolute path if needed

# Launch
cd ../../..
# Option A: model URI world
gz sim gazebo/worlds/leo_world.sdf
# Option B: absolute-path world (no model path needed)
gz sim gazebo/worlds/leo_world_local.sdf
```

## Send thrust commands (random walk test)

Use the provided Python script in `scripts/random_walk.py` to publish random thrusts to each thruster topic.
