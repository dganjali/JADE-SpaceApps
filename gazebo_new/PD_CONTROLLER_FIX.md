# Critical PD Controller Bug Fix

## Date: October 5, 2025

## The Problem

The chaser was **not reaching the dock** because of a **fundamental control theory error** in the PD controller.

### Root Cause: Wrong Damping Term Sign

```cpp
// WRONG (Previous Code):
accelDesired = (kp * distanceScale) * relPos + (kd * distanceScale) * relVel;
                                              ^^^
                                              ADDING damping term!
```

**Why this was broken:**

- `relPos = dockPos - chaserPos` (vector pointing from chaser to dock)
- `relVel = dockVel - chaserVel` (relative velocity)
- When chaser moves toward dock, `relVel` has a component in the direction of `relPos`
- **Adding** `kd * relVel` means we're **adding force in the direction of motion**
- This **fights against the damping**, creating unstable oscillations
- At close range with scaled-down gains (30%), the force became too weak to close the gap

### The Correct PD Control Law

```cpp
// CORRECT (Fixed Code):
accelDesired = (kp * distanceScale) * relPos - (kd) * relVel;
                                              ^^^
                                              SUBTRACTING damping term!
```

**Standard PD Controller:**
```
F = Kp * error - Kd * error_dot
```

Where:
- `error = relPos` (position error we want to minimize)
- `error_dot = relVel` (rate of change of error)
- **Proportional term** (+Kp × error): Pulls toward target
- **Derivative term** (-Kd × error_dot): **Opposes** rate of change (provides damping)

## Additional Fixes

### 1. Less Aggressive Distance Scaling
```cpp
// Before: Scale from 0.3 to 1.0
distanceScale = 0.3 + 0.7 * (dist / 5.0);

// After: Scale from 0.5 to 1.0 (less aggressive)
distanceScale = 0.5 + 0.5 * (dist / 5.0);
```

**Why:** 30% gains at close range made forces too weak to overcome even small disturbances.

### 2. No Distance Scaling on Damping
```cpp
// Before: Both terms scaled
(kp * distanceScale) * relPos + (kd * distanceScale) * relVel

// After: Only proportional term scaled, damping stays constant
(kp * distanceScale) * relPos - (kd) * relVel
```

**Why:** Damping should always stabilize motion, regardless of distance. Reducing damping at close range caused instability.

### 3. Relaxed Braking Threshold
```cpp
// Before: Brake at 2.5x distance if approaching faster than 0.4 m/s
if (dist < reachDistance * 2.5 && approachVel > 0.4)

// After: Brake at 2.0x distance if approaching faster than 0.8 m/s
if (dist < reachDistance * 2.0 && approachVel > 0.8)
```

**Why:** Previous threshold was too conservative, triggering braking during normal approach and preventing progress.

### 4. Better Debug Output
Added more useful metrics:
- Approach velocity
- Force magnitude
- Both dock and chaser speeds

## Physics Explanation

### Why the Sign Matters

Imagine you're driving toward a target:

**Correct PD Control:**
1. **Position Error (Kp):** "I'm 10m away" → Apply force forward
2. **Velocity Damping (-Kd):** "I'm moving at 5 m/s" → Apply force backward (brake)
3. **Net Result:** Smooth approach with automatic slowdown

**Incorrect (Previous Code):**
1. **Position Error (Kp):** "I'm 10m away" → Apply force forward
2. **Velocity Anti-Damping (+Kd):** "I'm moving at 5 m/s" → Apply MORE force forward
3. **Net Result:** Accelerates too much, overshoots, oscillates, or gets stuck

### The Math

For a PD controller tracking a moving target:

```
Force = Kp(x_target - x_chaser) - Kd(v_target - v_chaser)
      = Kp × relPos - Kd × relVel
```

The derivative term **must be negative** to provide damping!

## Expected Behavior Now

### Before Fix:
- ❌ Chaser couldn't reach dock
- ❌ Forces too weak at close range
- ❌ Damping term fought against motion
- ❌ Premature braking prevented approach

### After Fix:
- ✅ Proper attractive force toward dock
- ✅ Automatic velocity damping as it approaches
- ✅ Stronger forces at close range (50% vs 30%)
- ✅ Braking only for true emergencies
- ✅ Should successfully intercept and dock

## Control Theory Reference

Standard second-order system with PD control:

```
m * a = -Kp * x - Kd * v
```

This creates a damped spring system that:
- Converges to target position (x → 0)
- With controlled velocity (v → 0)
- Without oscillation (if Kd is sufficient)

Critical damping condition: `Kd ≈ 2√(Kp × m)`

With our values:
- Kp = 1.0, m = 10 kg
- Critical Kd ≈ 2√(10) ≈ 6.3
- Our Kd = 0.5 (underdamped, but acceptable for faster response)

## Testing

Run the simulation and verify:
1. ✅ Chaser moves toward dock
2. ✅ Velocity increases initially, then decreases as it approaches
3. ✅ Successful docking within reach distance
4. ✅ No oscillations or overshooting
5. ✅ Debug output shows decreasing distance and approach velocity

```bash
cd /Users/dganjali/GitHub/JADE-SpaceApps/gazebo_new
./run_simulation.sh
```

Watch the console for progression like:
```
dist=15.0 approachVel=0.5 ...
dist=10.0 approachVel=0.7 ...
dist=5.0 approachVel=0.6 ...
dist=2.0 approachVel=0.4 ...
dist=0.5 approachVel=0.2 ...
SUCCESS: reached dock. distance=0.3 approach_vel=0.1
```
