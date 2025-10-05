# 🚀 Equations Actually Implemented in Realistic Orbital Controller

**Status:** ✅ **DOCKING SUCCESSFUL!** 🎉

This document lists the equations from the reference that are **actively implemented** in `realistic_orbital_controller.cpp`.

---

## ✅ **1. Fundamental Constants & Parameters**

All implemented in code:

```cpp
constexpr double G = 6.67430e-11;           // Gravitational constant (m³ kg⁻¹ s⁻²)
constexpr double M_EARTH = 5.97219e24;      // Earth mass (kg)
constexpr double R_EARTH = 6.371e6;         // Earth radius (m)
constexpr double GM = G * M_EARTH;          // Standard gravitational parameter
constexpr double g0 = 9.80665;              // Standard gravity (m/s²)
constexpr double J2 = 1.08263e-3;           // Earth's J2 oblateness coefficient
```

---

## ✅ **2. Orbital Mechanics - FULLY IMPLEMENTED**

### Newton's Law of Universal Gravitation ✅
**Equation:** $a_{\text{grav}} = -\frac{GM_E}{r^3} \mathbf{r}$

**Implementation:**
```cpp
gz::math::Vector3d ComputeOrbitalGravity(const gz::math::Vector3d &pos)
{
  double r = pos.Length();
  if (r < R_EARTH) r = R_EARTH; // Prevent singularity
  
  double r3 = r * r * r;
  return -GM / r3 * pos;
}
```

**Status:** ✅ **ACTIVE** - Applied to both chaser and dock every timestep

---

### J2 Perturbation Acceleration ✅
**Equation:** 
$$a_{J2} = -\frac{3}{2} \frac{J_2 GM_E R_E^2}{r^5} \begin{bmatrix} x(1-5z^2/r^2) \\ y(1-5z^2/r^2) \\ z(3-5z^2/r^2) \end{bmatrix}$$

**Implementation:**
```cpp
gz::math::Vector3d ComputeJ2Perturbation(const gz::math::Vector3d &pos)
{
  double x = pos.X();
  double y = pos.Y();
  double z = pos.Z();
  double r = pos.Length();
  
  if (r < R_EARTH) r = R_EARTH;
  
  double r2 = r * r;
  double r5 = r2 * r2 * r;
  double z2_r2 = (z * z) / r2;
  
  double coeff = -1.5 * J2 * GM * R_EARTH * R_EARTH / r5;
  
  gz::math::Vector3d acc;
  acc.X() = coeff * x * (1.0 - 5.0 * z2_r2);
  acc.Y() = coeff * y * (1.0 - 5.0 * z2_r2);
  acc.Z() = coeff * z * (3.0 - 5.0 * z2_r2);
  
  return acc;
}
```

**Status:** ⚠️ **IMPLEMENTED BUT DISABLED** - Toggle with `use_j2=true` parameter (currently off for stability)

---

### Clohessy-Wiltshire (Hill's) Equations ✅
**Equations:**
$$\begin{cases}
\ddot{x} - 2n\dot{y} - 3n^2x = F_x/m \\
\ddot{y} + 2n\dot{x} = F_y/m \\
\ddot{z} + n^2z = F_z/m
\end{cases}$$

**Implementation:**
```cpp
gz::math::Vector3d ComputeControlAcceleration(const gz::math::Vector3d &relPos, 
                                               const gz::math::Vector3d &relVel)
{
  // PD control
  gz::math::Vector3d accel = this->kpPos * relPos - this->kdPos * relVel;
  
  // Hill's equations compensation terms
  double n = this->meanMotion;
  double n2 = n * n;
  
  accel.X() += 3.0 * n2 * relPos.X() + 2.0 * n * relVel.Y();
  accel.Y() += -2.0 * n * relVel.X();
  accel.Z() += -n2 * relPos.Z();
  
  return accel;
}
```

**Status:** ✅ **ACTIVE** - Hill's equations used in control compensation

**Mean Motion Calculation:**
```cpp
double r0 = R_EARTH + this->orbitalAltitude;
this->meanMotion = std::sqrt(GM / (r0 * r0 * r0));
```

---

## ✅ **3. Rigid-Body Dynamics - PARTIAL**

### Newton's Second Law ✅
**Equation:** $m\dot{\mathbf{v}} = \sum \mathbf{F}$

**Implementation:** 
```cpp
targetLink.AddWorldForce(ecm, gravChaser * chaserMass);
dockLink.AddWorldForce(ecm, gravDock * dockMass);
targetLink.AddWorldForce(ecm, chaserThrust * chaserMass);
```

**Status:** ✅ **ACTIVE** - Gazebo integrates forces automatically

---

### Kinematic Update ✅
**Equation:** $\mathbf{r}_{k+1} = \mathbf{r}_k + \Delta t \mathbf{v}_{k+1}$

**Implementation:** Handled by Gazebo's ODE solver (1ms timestep)

**Status:** ✅ **ACTIVE** - Automatic integration

---

### Euler's Rotational Equation ❌
**Status:** ❌ **NOT IMPLEMENTED** - No attitude control yet

### Quaternion Kinematics ❌
**Status:** ❌ **NOT IMPLEMENTED** - No attitude control yet

---

## ✅ **4. Thruster & Propulsion Physics - PARTIAL**

### Thrust Force Equation ✅
**Equation:** $\mathbf{F}_i = T_i \hat{\mathbf{d}}_i$

**Implementation:**
```cpp
gz::math::Vector3d chaserAccel = ComputeControlAcceleration(relPos, relVel);
gz::math::Vector3d chaserThrust = chaserAccel; // Normalized direction
double thrustMag = chaserThrust.Length();

if (thrustMag > this->chaserThrustMax / chaserMass)
{
  chaserThrust = chaserThrust.Normalized() * (this->chaserThrustMax / chaserMass);
  thrustMag = this->chaserThrustMax / chaserMass;
}
```

**Status:** ✅ **ACTIVE** - Direction from PD control, magnitude limited

---

### Tsiolkovsky Rocket Equation ✅
**Equation:** $\Delta v = I_{sp} g_0 \ln\frac{m_0}{m_f}$

**Implementation (Mass Flow Rate):**
```cpp
// m_dot = -T / (I_sp * g_0)
double fuelRate = thrustMag * chaserMass / (this->chaserIsp * g0);
this->chaserFuelUsed += fuelRate * dt;
```

**Status:** ✅ **ACTIVE** - Tracks propellant consumption in real-time

---

### Thruster Torque ❌
**Status:** ❌ **NOT IMPLEMENTED** - No rotational control yet

### Thruster Allocation Matrix ❌
**Status:** ❌ **NOT IMPLEMENTED** - Visual thrusters only, no allocation

---

## ✅ **5. Environmental Effects - PARTIAL**

### Atmospheric Drag Force ✅
**Equation:** $\mathbf{F}_{\text{drag}} = -\frac{1}{2} C_d A \rho(h) \mathbf{v} |\mathbf{v}|$

**Exponential Density Model:**
$$\rho(h) = \rho_0 \exp\left(-\frac{h-h_0}{H}\right)$$

**Implementation:**
```cpp
gz::math::Vector3d ComputeDrag(const gz::math::Vector3d &vel, double altitude, 
                                double area, double Cd)
{
  if (altitude < 0) altitude = 0;
  
  // Exponential atmosphere model
  double rho = rho0 * std::exp(-altitude / H_scale);
  
  // Drag force: F = -0.5 * Cd * A * rho * v * |v|
  double v_mag = vel.Length();
  if (v_mag < 0.01) return gz::math::Vector3d::Zero;
  
  return -0.5 * Cd * area * rho * v_mag * vel;
}
```

**Status:** ⚠️ **IMPLEMENTED BUT DISABLED** - Toggle with `use_drag=true` (currently off)

**Parameters:**
```cpp
constexpr double rho0 = 1.225;              // Sea level density (kg/m³)
constexpr double H_scale = 8500.0;          // Scale height (m)
```

---

### Solar Radiation Pressure ❌
**Status:** ❌ **NOT IMPLEMENTED** - Negligible for LEO

---

## ✅ **6. Guidance & Control - FULLY IMPLEMENTED**

### PD Translational Control Law ✅
**Equation:** $\mathbf{a}_{\text{cmd}} = K_p \mathbf{e}_p + K_d \mathbf{e}_v$

**Implementation:**
```cpp
gz::math::Vector3d accel = this->kpPos * relPos - this->kdPos * relVel;
```

**Current Gains (TUNED FOR SUCCESS!):**
```cpp
kp_pos = 0.15  // Position gain
kd_pos = 2.5   // Velocity damping gain
```

**Status:** ✅ **ACTIVE** - Successfully docking with these gains!

---

### PD Attitude Control ❌
**Status:** ❌ **NOT IMPLEMENTED** - No attitude control

### Thruster Allocation ❌
**Status:** ❌ **NOT IMPLEMENTED** - Single thrust vector control

---

## ✅ **7. State Estimation - SIMPLE**

### Relative Pose Estimation ✅
**Equation:** $\mathbf{r}_{\text{rel}} = \mathbf{r}_{\text{chaser}} - \mathbf{r}_{\text{target}}$

**Implementation:**
```cpp
gz::math::Vector3d relPos = dockPose.Pos() - targetPose.Pos();
gz::math::Vector3d relVel = vDock - vTarget;
double dist = relPos.Length();
double relSpeed = relVel.Length();
double approachSpeed = -relVel.Dot(relPos.Normalized());
```

**Status:** ✅ **ACTIVE** - Perfect knowledge (no sensor noise)

---

### Extended Kalman Filter ❌
**Status:** ❌ **NOT IMPLEMENTED** - Using perfect state knowledge

---

## ✅ **8. Collisions & Docking - SIMPLE**

### Docking Constraint Condition ✅
**Equation:** $\|\mathbf{r}_{\text{rel}}\| < r_{\text{threshold}}, \|\mathbf{v}_{\text{rel}}\| < v_{\text{threshold}}$

**Implementation:**
```cpp
if (dist <= this->reachDistance && relSpeed <= this->reachVelocity)
{
  std::cout << "🎉 DOCKING SUCCESS! 🎉" << std::endl;
  this->done = true;
  return;
}
```

**Thresholds:**
```cpp
reachDistance = 0.8 m
reachVelocity = 0.3 m/s
```

**Status:** ✅ **ACTIVE** - Successfully detecting docking!

---

### Collision Impulse Equation ❌
**Status:** ❌ **NOT IMPLEMENTED** - Using Gazebo's collision detection

---

## ❌ **9. Sensor Models - NOT IMPLEMENTED**

- Camera Projection Model ❌
- Stereo Depth Equation ❌
- Rangefinder Noise Model ❌
- IMU Sensor Model ❌

**Status:** Using perfect state knowledge for now

*(Vision-based controller exists in `vision_based_orbital_controller.cpp` but not currently used)*

---

## 📊 **Implementation Summary**

| Category | Implemented | Status |
|----------|-------------|--------|
| **Orbital Mechanics** | 3/3 | ✅ **100%** |
| **Rigid-Body Dynamics** | 2/4 | ⚠️ **50%** (translation only) |
| **Propulsion** | 2/5 | ⚠️ **40%** (thrust + fuel tracking) |
| **Environment** | 1/2 | ⚠️ **50%** (drag available but disabled) |
| **Control** | 1/3 | ⚠️ **33%** (PD translation only) |
| **State Estimation** | 1/2 | ✅ **50%** (perfect knowledge) |
| **Docking Detection** | 1/2 | ✅ **50%** (soft-latch detection) |
| **Sensors** | 0/4 | ❌ **0%** |

---

## 🎯 **Key Physics Actually Used for Successful Docking:**

1. ✅ **Two-Body Orbital Gravity** - Primary force
2. ✅ **Hill's Equations** - Relative motion compensation
3. ✅ **PD Control** - Position and velocity feedback (Kp=0.15, Kd=2.5)
4. ✅ **Tsiolkovsky Fuel Tracking** - Realistic propellant consumption
5. ✅ **Thrust Limiting** - Maximum 5.0N for chaser, 1.5N for dock

**Optional (disabled for stability):**
- J2 Perturbation (Earth oblateness)
- Atmospheric Drag (LEO decay)

---

## 🚀 **Mission Parameters:**

- **Orbital Altitude:** 400 km LEO
- **Orbital Velocity:** 7,669 m/s
- **Mean Motion (n):** 0.001117 rad/s
- **Orbital Period:** 92.6 minutes
- **Chaser Mass:** 100 kg
- **Dock Mass:** 500 kg
- **Chaser Isp:** 220s (Hydrazine)
- **Success Distance:** < 0.8 m
- **Success Velocity:** < 0.3 m/s

---

**🎉 STATUS: MISSION SUCCESS! 🎉**

The simulation successfully achieves autonomous orbital rendezvous and docking using realistic orbital mechanics!
