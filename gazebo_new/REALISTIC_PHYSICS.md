# Realistic Orbital Mechanics Implementation Guide

## Overview

This implementation incorporates realistic orbital physics for LEO (Low Earth Orbit) satellite docking simulation. The system models proper gravitational dynamics, atmospheric effects, thruster physics, and relative motion using established astrodynamics principles.

---

## 1. Physical Constants (SI Units)

All calculations use SI units throughout:

| Constant | Symbol | Value | Unit |
|----------|--------|-------|------|
| Gravitational constant | G | 6.67430×10⁻¹¹ | m³ kg⁻¹ s⁻² |
| Earth mass | M_E | 5.97219×10²⁴ | kg |
| Earth radius (mean) | R_E | 6.371×10⁶ | m |
| Standard gravity | g₀ | 9.80665 | m/s² |
| J2 coefficient | J₂ | 1.08263×10⁻³ | dimensionless |
| Std gravitational param | GM | 3.986×10¹⁴ | m³/s² |

---

## 2. Two-Body Orbital Gravity

### Implementation: `ComputeOrbitalGravity()`

The gravitational acceleration from Earth's central mass:

```
a_grav(r) = -GM/|r|³ × r
```

Where:
- `r` = position vector from Earth's center (m)
- Returns acceleration vector in m/s²

**Code location:** `realistic_orbital_controller.cpp:247-254`

**Usage:** Applied every physics timestep to both chaser and dock
- Force = `mass × a_grav`
- Provides realistic orbital motion even in "zero-g" simulation

**Note:** Singularity protection at r < R_EARTH

---

## 3. J2 Perturbation (Earth Oblateness)

### Implementation: `ComputeJ2Perturbation()`

Earth's equatorial bulge causes non-spherical gravity:

```
a_J2 = -1.5 × J₂ × GM × R_E² / r⁵ × [
  x(1 - 5z²/r²)
  y(1 - 5z²/r²)  
  z(3 - 5z²/r²)
]
```

**Effect:** Causes orbital precession, node regression
**Magnitude:** ~10⁻⁶ m/s² at 400 km
**Enable for:** Long-duration simulations (hours/days)

**Parameter:** `<use_j2>true</use_j2>` in world file

---

## 4. Atmospheric Drag

### Implementation: `ComputeDrag()`

Exponential atmosphere model with drag force:

```
F_drag = -0.5 × C_d × A × ρ(h) × v × |v|
ρ(h) = ρ₀ × exp(-(h - h₀)/H)
```

Where:
- `C_d` = drag coefficient (~2.2 for box-like satellites)
- `A` = cross-sectional area (m²)
- `ρ(h)` = atmospheric density at altitude h
- `H` = scale height (~8500 m)
- `v` = velocity relative to atmosphere

**Typical densities:**
- 400 km: ~10⁻¹² kg/m³
- 300 km: ~10⁻¹¹ kg/m³
- 200 km: ~10⁻¹⁰ kg/m³

**Effect:** Orbital decay, energy dissipation
**Enable for:** Modeling long-term orbital maintenance

**Parameter:** `<use_drag>true</use_drag>` in world file

---

## 5. Hill's Equations (Clohessy-Wiltshire)

### Relative motion in rotating orbital frame

For close-range rendezvous (<few km), linearized dynamics:

```
ẍ - 2nẏ - 3n²x = F_x/m
ÿ + 2nẋ      = F_y/m
z̈ + n²z      = F_z/m
```

Where:
- `n = √(GM/r₀³)` = mean motion (rad/s)
- `(x, y, z)` = relative position in Hill frame
  - x: radial (toward/away from Earth)
  - y: along-track (direction of orbit)
  - z: cross-track (normal to orbit)

**Implementation:** `ComputeControlAcceleration()`

The controller compensates for orbital mechanics terms:
```cpp
accel.X() += 3.0 * n² * relPos.X() + 2.0 * n * relVel.Y();
accel.Y() += -2.0 * n * relVel.X();
accel.Z() += -n² * relPos.Z();
```

**Physical meaning:**
- `3n²x` term: tidal force (radial restoring)
- `2nẏ` term: Coriolis effect (along-track coupling)
- `n²z` term: out-of-plane restoring

**Result:** Natural orbital motion accounted for in control law

---

## 6. Rigid Body Dynamics (6-DOF)

### Translational (Newton's 2nd Law)

```
m dv/dt = F_grav + F_thrust + F_drag + F_other
```

Integrated using semi-implicit Euler at 1 kHz:
```
v_{k+1} = v_k + Δt × (F_k/m)
r_{k+1} = r_k + Δt × v_{k+1}
```

### Rotational (Euler's Equation)

```
I ω̇ + ω × (Iω) = τ_control + τ_env
```

Where:
- `I` = inertia tensor (3×3 matrix)
- `ω` = angular velocity (body frame)
- `τ` = applied torque

**Attitude representation:** Quaternions (avoids gimbal lock)
```
q̇ = 0.5 × Ω(ω) × q
```

**Current implementation:** Translational only (rotational available for expansion)

---

## 7. Thruster Physics

### Single Thruster Model

```
F_i = T_i × d̂_i
τ_i = r_i × F_i
```

Where:
- `T_i` = thrust magnitude (N)
- `d̂_i` = unit direction vector (body frame)
- `r_i` = position from center of mass
- `τ_i` = moment/torque contribution

### Total Forces
```
F_total = Σ F_i
τ_total = Σ τ_i
```

### Thrust Saturation
```
0 ≤ T_i ≤ T_max
```

**Parameters:**
- Chaser: `<chaser_thrust_max>5.0</chaser_thrust_max>` (Newtons)
- Dock: `<dock_thrust_max>3.0</dock_thrust_max>` (Newtons)

### Propellant Consumption (Tsiolkovsky)

Mass flow rate:
```
ṁ = -T / (I_sp × g₀)
```

Updated mass:
```
m(t + Δt) = m(t) + ṁ × Δt
```

Delta-v capability:
```
Δv = I_sp × g₀ × ln(m₀/m_f)
```

**Parameters:**
- `<chaser_isp>220.0</chaser_isp>` seconds (hydrazine-class)
- Typical values:
  - Cold gas: 50-70 s
  - Hydrazine: 220-300 s
  - Ion/Hall: 1000-3000 s

**Tracking:** Fuel consumption logged in real-time:
```
Chaser Fuel Used: X.XXX kg
Dock Fuel Used: Y.YYY kg
```

---

## 8. Control System

### PD Controller with Hill's Compensation

```
a_cmd = K_p × e_pos + K_d × e_vel + Hill_compensation
```

Where:
- `e_pos = r_target - r_current` (position error)
- `e_vel = v_target - v_current` (velocity error)
- Hill compensation accounts for orbital mechanics

**Gains:**
- `<kp_pos>0.5</kp_pos>` - proportional gain
- `<kd_pos>1.0</kd_pos>` - derivative gain

**Tuning guidance:**
- Higher K_p: faster convergence, risk of overshoot
- Higher K_d: more damping, slower but smoother
- Start low, increase until marginally critically damped

### Thruster Allocation

Convert desired acceleration to force:
```
F_cmd = m × a_cmd
```

Apply magnitude limit:
```
if |F_cmd| > T_max:
    F_cmd = F_cmd/|F_cmd| × T_max
```

**Minimum Impulse Bit:** 0.01 N threshold to model discrete thruster pulses

---

## 9. Docking Success Criteria

Soft capture conditions (both must be satisfied):

```
distance ≤ reach_distance  AND  relative_velocity ≤ reach_velocity
```

**Parameters:**
- `<reach_distance>0.8</reach_distance>` meters
- `<reach_velocity>0.15</reach_velocity>` m/s

**Realistic values:**
- ISS docking: ~0.1 m/s approach velocity
- Commercial: 0.05-0.2 m/s typical
- Margin of safety built in

---

## 10. Orbital Parameters (400 km LEO)

Computed automatically from altitude:

```
r₀ = R_E + altitude = 6,771,000 m
n = √(GM/r₀³) = 0.001117 rad/s
v_orbital = √(GM/r₀) = 7,669 m/s
Period = 2π/n = 92.6 minutes
```

**Output on startup:**
```
Orbital Altitude: 400.0 km
Orbital Velocity: 7669.0 m/s
Mean Motion (n): 0.001117 rad/s
Orbital Period: 92.6 minutes
```

---

## 11. Numerical Integration

### Timestep
- **Fixed:** Δt = 0.001 s (1 ms)
- **Update rate:** 1000 Hz
- **Method:** Semi-implicit Euler (energy stable)

**Rationale:**
- Control loop stability requires fast sampling
- Orbital mechanics at ~400 km: n ≈ 0.001 rad/s
- Nyquist criterion satisfied with margin
- Balanced between accuracy and performance

### Solver Settings (Gazebo ODE)
```xml
<max_step_size>0.001</max_step_size>
<real_time_update_rate>1000</real_time_update_rate>
```

---

## 12. Model Specifications

### Chaser Satellite
- **Mass:** 100 kg (typical small satellite)
- **Size:** 0.5 × 0.5 × 0.5 m
- **Inertia:** 16.67 kg·m² (uniform cube)
- **Thrust:** 5 N max
- **I_sp:** 220 s (hydrazine)
- **Appearance:** Blue with orange thruster markers

### Dock/Station
- **Mass:** 500 kg (station module or large satellite)
- **Size:** 1.0 × 1.0 × 1.0 m
- **Inertia:** 83.33 kg·m² (uniform cube)
- **Thrust:** 3 N (station-keeping)
- **I_sp:** 220 s
- **Appearance:** Orange with gray thruster markers
- **Behavior:** Active station-keeping with random thrust direction

---

## 13. Sensor & Actuator Models (Future Expansion)

### Camera (Pinhole Model)
```
[u, v, 1]ᵀ ∝ K × [R | t] × [X, Y, Z, 1]ᵀ
```

### Stereo Depth
```
Z = f × B / d
```
- f: focal length
- B: baseline
- d: disparity

### IMU Model
- Gyro bias/drift: ~0.1 deg/hr
- Accel bias: ~10⁻⁴ m/s²
- Sample rate: 100-1000 Hz

### Thruster Dynamics (1st order lag)
```
Ṫ = (T_cmd - T) / τ_act
```
τ_act ~ 10-50 ms

---

## 14. Usage Examples

### Basic Realistic Simulation
```bash
cd /Users/dganjali/GitHub/JADE-SpaceApps/gazebo_new
./run_realistic.sh
```

### Enable All Physics Features
Edit `worlds/leo_realistic.sdf`:
```xml
<use_orbital_gravity>true</use_orbital_gravity>
<use_j2>true</use_j2>
<use_drag>true</use_drag>
```

### Tune Control Gains
```xml
<kp_pos>0.8</kp_pos>  <!-- More aggressive -->
<kd_pos>1.5</kd_pos>  <!-- More damping -->
```

### Change Orbit Altitude
```xml
<orbital_altitude>600000.0</orbital_altitude>  <!-- 600 km -->
```

---

## 15. Telemetry Output

Console displays real-time status every 0.2 seconds:

```
[T+5.0s] dist=48.23m rel_v=0.85m/s thrust=4.12N fuel=0.023kg
[T+10.0s] dist=35.67m rel_v=1.02m/s thrust=4.95N fuel=0.051kg
...
[T+120.0s] dist=0.65m rel_v=0.08m/s thrust=0.15N fuel=0.345kg

====================================================
🎉 DOCKING SUCCESS! 🎉
====================================================
Final Distance: 0.65 m
Relative Velocity: 0.08 m/s
Mission Time: 125.3 s
Chaser Fuel Used: 0.351 kg
Dock Fuel Used: 0.089 kg
====================================================
```

---

## 16. Parameter Quick Reference

| Parameter | Default | Realistic Range | Notes |
|-----------|---------|-----------------|-------|
| orbital_altitude | 400000 m | 200000-800000 m | ISS at ~420 km |
| chaser_thrust_max | 5.0 N | 0.1-20 N | Depends on sat size |
| dock_thrust_max | 3.0 N | 0.5-50 N | Station-keeping |
| chaser_isp | 220 s | 50-3000 s | Propellant type |
| kp_pos | 0.5 | 0.1-2.0 | Start low |
| kd_pos | 1.0 | 0.5-3.0 | Tune for damping |
| reach_distance | 0.8 m | 0.2-1.0 m | Capture tolerance |
| reach_velocity | 0.15 m/s | 0.05-0.3 m/s | Safety margin |

---

## 17. Validation Tests

### Energy Conservation
With no thrusters active, total energy should remain constant:
```
E = 0.5 × m × v² - GM×m/r
```

### Orbital Period Check
Enable orbital gravity and verify period matches theoretical:
```
T = 2π × √(r³/GM)
```

### Thrust Integration
Apply constant 1 N thrust to 100 kg mass for 10 s:
```
Expected Δv = F×t/m = 0.1 m/s
```

### Fuel Consumption
Verify Tsiolkovsky equation:
```
Δv = I_sp × g₀ × ln(m₀/m_f)
```

---

## 18. Known Limitations & Future Work

### Current Limitations
- Rotational dynamics not implemented (translational only)
- Single thruster per vehicle (no multi-thruster allocation)
- Perfect sensors (no noise/delay)
- Instantaneous thrust response (no actuator lag)
- Simplified collision detection

### Recommended Enhancements
1. **Full 6-DOF**: Add attitude control with quaternion integration
2. **Multi-thruster allocation**: Solve constrained optimization for force/torque
3. **Sensor models**: Add camera, rangefinder, IMU with realistic noise
4. **Actuator dynamics**: First-order lag, minimum impulse bit (MIB)
5. **Contact mechanics**: Soft-body docking port with spring-damper
6. **Thermal/power**: Battery discharge, solar panel modeling
7. **MPC controller**: Fuel-optimal trajectory planning
8. **Extended missions**: Long-duration orbit with perturbations

---

## 19. References & Theory

### Astrodynamics
- Vallado, D. "Fundamentals of Astrodynamics and Applications"
- Curtis, H. "Orbital Mechanics for Engineering Students"
- Battin, R. "An Introduction to the Mathematics and Methods of Astrodynamics"

### Guidance & Control
- Wie, B. "Space Vehicle Dynamics and Control"
- Clohessy-Wiltshire equations (1960) - Hill's equations application
- PD control tuning via Ziegler-Nichols method

### Spacecraft Systems
- Wertz, J. "Space Mission Analysis and Design" (SMAD)
- Sutton & Biblarz "Rocket Propulsion Elements"

### Numerical Methods
- RK4 integration for high accuracy
- Semi-implicit Euler for energy stability
- Quaternion kinematics for attitude

---

## 20. Quick Start Commands

```bash
# Build the realistic controller
cd /Users/dganjali/GitHub/JADE-SpaceApps/gazebo_new/build
cmake ..
make

# Run realistic simulation
cd ..
gz sim worlds/leo_realistic.sdf

# Or use the launch script
./run_realistic.sh
```

**Expected behavior:**
1. Console shows orbital parameters on startup
2. Chaser applies thrust toward dock
3. Dock performs random station-keeping
4. Real-time telemetry displays distance, velocity, thrust, fuel
5. Success message when docking criteria met

---

## 21. Troubleshooting

### Models don't move
- Check `use_orbital_gravity` is true
- Verify plugin loads: look for "RealisticOrbital" in console
- Check masses are reasonable (not 0)

### Unstable oscillations
- Reduce K_p and K_d gains
- Increase damping term (K_d)
- Check timestep is 0.001 s

### Too fast/slow convergence
- Increase K_p for faster (risk overshoot)
- Increase K_d for more damping
- Adjust thrust limits

### Wrong orbital behavior
- Verify altitude setting (meters!)
- Check GM = 3.986e14 m³/s²
- Enable J2 for long sims

---

**Implementation by:** JADE Team  
**Date:** October 5, 2025  
**Gazebo Version:** Harmonic (gz-sim 10)  
**License:** MIT

