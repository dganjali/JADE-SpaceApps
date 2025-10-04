# Gazebo Sim (new Gazebo) LEO environment and satellite model

This folder contains a minimal Low Earth Orbit (LEO) microgravity world and a simple satellite model designed for the modern Gazebo Sim (a.k.a. Ignition Gazebo / `gz sim`). The world uses near-zero gravity with a dark space-like scene. The satellite includes an IMU and multiple thruster frames you can wire up later with a plugin or controller.

## Contents

- `worlds/leo_world.sdf` — World file with microgravity, dark space scene, and the satellite placed at the origin.
- `models/satellite/` — SDF satellite model with IMU and 8 thruster frames around the body.

## Requirements

- Gazebo Sim (Garden/Harmonic or later). Command may be `gz sim`, or older `ign gazebo`.

## Run

Using `gz` (preferred in recent Gazebo releases):

```bash
# From repo root. Ensure Gazebo can find local models:
export GZ_SIM_RESOURCE_PATH="$PWD/gazebo/models"
gz sim gazebo/worlds/leo_world.sdf
```

If you have an older Ignition Gazebo:

```bash
export IGN_GAZEBO_RESOURCE_PATH="$PWD/gazebo/models"
ign gazebo gazebo/worlds/leo_world.sdf
```

## Notes

- Gravity is set to effectively zero. For pure free-fall orbital dynamics, you’d typically include central gravity + orbital initial conditions; here we approximate microgravity for thruster training.
- Wind is disabled and scene background is dark to mimic space.
- IMU sensor is added to `base_link`. Thruster links are present as frames (no force application by default). You can later add a Gazebo system plugin to apply forces when you publish thrust commands.
- If you want a simple placeholder thruster plugin scaffold, let me know and I’ll add a `gazebo/plugins` C++ system plugin you can build locally.

If Gazebo fails to find `model://satellite`, verify the resource path env var above or install the model into your Gazebo user model path (e.g., `~/.gazebo/models` for classic or `~/.ignition/fuel` for Ignition/Gz Fuel).
