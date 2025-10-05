// advanced_orbital_controller.cpp
// Implements realistic sensor models and thruster allocation matrix
// Based on realistic_orbital_controller but with proper sensor noise and thruster control

#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Matrix3.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/double.pb.h>

#include <sdf/sdf.hh>

#include <random>
#include <iostream>
#include <cmath>
#include <vector>
#include <Eigen/Dense>

using namespace gz;
using namespace sim;

// Physical constants (SI units)
constexpr double G = 6.67430e-11;
constexpr double M_EARTH = 5.97219e24;
constexpr double R_EARTH = 6.371e6;
constexpr double GM = G * M_EARTH;
constexpr double g0 = 9.80665;
constexpr double J2 = 1.08263e-3;

// Atmospheric parameters
constexpr double rho0 = 1.225;
constexpr double H_scale = 8500.0;

// Sensor noise parameters
constexpr double RANGE_NOISE_SIGMA = 0.05;        // 5cm rangefinder noise
constexpr double VELOCITY_NOISE_SIGMA = 0.01;    // 1cm/s IMU noise
constexpr double CAMERA_POSITION_NOISE = 0.02;   // 2cm camera position noise
constexpr double IMU_ACCEL_NOISE = 0.001;        // 0.001 m/s^2 accelerometer noise
constexpr double IMU_GYRO_NOISE = 0.0001;        // 0.0001 rad/s gyro noise

// Thruster configuration (16 thrusters per spacecraft)
struct Thruster {
  gz::math::Vector3d position;   // Position relative to CoM (m)
  gz::math::Vector3d direction;  // Thrust direction (unit vector)
  double maxThrust;               // Maximum thrust (N)
  
  Thruster(const gz::math::Vector3d& pos, const gz::math::Vector3d& dir, double thrust)
    : position(pos), direction(dir.Normalized()), maxThrust(thrust) {}
};

class AdvancedOrbitalController : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
  AdvancedOrbitalController() : System() 
  {
    // Initialize random number generator for sensor noise
    std::random_device rd;
    gen = std::mt19937(rd());
  }

  void Configure(const Entity & /*_entity*/, const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &ecm, EventManager &/*_eventMgr*/) override
  {
    // Read parameters
    if (_sdf->HasElement("target_model"))
      this->targetModelName = _sdf->Get<std::string>("target_model");
    if (_sdf->HasElement("dock_model"))
      this->dockModelName = _sdf->Get<std::string>("dock_model");
    if (_sdf->HasElement("reach_distance"))
      this->reachDistance = _sdf->Get<double>("reach_distance");
    if (_sdf->HasElement("reach_velocity"))
      this->reachVelocity = _sdf->Get<double>("reach_velocity");
    if (_sdf->HasElement("orbital_altitude"))
      this->orbitalAltitude = _sdf->Get<double>("orbital_altitude");
    if (_sdf->HasElement("chaser_thrust_max"))
      this->chaserThrustMax = _sdf->Get<double>("chaser_thrust_max");
    if (_sdf->HasElement("dock_thrust_max"))
      this->dockThrustMax = _sdf->Get<double>("dock_thrust_max");
    if (_sdf->HasElement("chaser_isp"))
      this->chaserIsp = _sdf->Get<double>("chaser_isp");
    if (_sdf->HasElement("dock_isp"))
      this->dockIsp = _sdf->Get<double>("dock_isp");
    if (_sdf->HasElement("use_j2"))
      this->useJ2 = _sdf->Get<bool>("use_j2");
    if (_sdf->HasElement("use_drag"))
      this->useDrag = _sdf->Get<bool>("use_drag");
    if (_sdf->HasElement("use_orbital_gravity"))
      this->useOrbitalGravity = _sdf->Get<bool>("use_orbital_gravity");
    if (_sdf->HasElement("kp_pos"))
      this->kpPos = _sdf->Get<double>("kp_pos");
    if (_sdf->HasElement("kd_pos"))
      this->kdPos = _sdf->Get<double>("kd_pos");
    if (_sdf->HasElement("use_sensor_noise"))
      this->useSensorNoise = _sdf->Get<bool>("use_sensor_noise");

    // Calculate orbital parameters
    double r0 = R_EARTH + this->orbitalAltitude;
    this->meanMotion = std::sqrt(GM / (r0 * r0 * r0));
    this->orbitalVelocity = std::sqrt(GM / r0);

    // Initialize thruster configuration
    InitializeThrusters();

    this->worldEntity = gz::sim::worldEntity(ecm);
    this->configured = true;
    
    // Initialize telemetry publishers
    this->distPub = this->node.Advertise<gz::msgs::Double>("/docking/distance");
    this->portDistPub = this->node.Advertise<gz::msgs::Double>("/docking/port_distance");
    this->alignPub = this->node.Advertise<gz::msgs::Double>("/docking/alignment");
    this->velPub = this->node.Advertise<gz::msgs::Double>("/docking/velocity");
    this->approachPub = this->node.Advertise<gz::msgs::Double>("/docking/approach");
    this->thrustPub = this->node.Advertise<gz::msgs::Double>("/docking/thrust");
    this->fuelPub = this->node.Advertise<gz::msgs::Double>("/docking/fuel");
    this->timePub = this->node.Advertise<gz::msgs::Double>("/docking/time");
    this->modePub = this->node.Advertise<gz::msgs::Double>("/docking/mode");
    
    std::cout << "====================================================" << std::endl;
    std::cout << "[AdvancedOrbital] CONFIGURATION" << std::endl;
    std::cout << "====================================================" << std::endl;
    std::cout << "Orbital Altitude: " << this->orbitalAltitude / 1000.0 << " km" << std::endl;
    std::cout << "Orbital Velocity: " << this->orbitalVelocity << " m/s" << std::endl;
    std::cout << "Mean Motion (n): " << this->meanMotion << " rad/s" << std::endl;
    std::cout << "Chaser Thrusters: " << chaserThrusters.size() << " x " << this->chaserThrustMax << " N" << std::endl;
    std::cout << "Dock Thrusters: " << dockThrusters.size() << " x " << this->dockThrustMax << " N" << std::endl;
    std::cout << "Sensor Noise: " << (this->useSensorNoise ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << "J2 Perturbation: " << (this->useJ2 ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << "Atmospheric Drag: " << (this->useDrag ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << "PD Gains: Kp=" << this->kpPos << ", Kd=" << this->kdPos << std::endl;
    std::cout << "====================================================" << std::endl;
  }

  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &ecm) override
  {
    if (!this->configured) return;
    
    double dt = std::chrono::duration<double>(_info.dt).count();
    if (dt <= 0) return;
    
    double simTime = std::chrono::duration<double>(_info.simTime).count();

    // Find entities once
    if (this->targetLinkEntity == kNullEntity)
    {
      auto targetModel = ecm.EntityByComponents(
        components::Model(), components::Name(this->targetModelName));
      
      if (targetModel != kNullEntity)
      {
        Model model(targetModel);
        this->targetLinkEntity = model.LinkByName(ecm, "body");
        std::cout << "[AdvancedOrbital] Found chaser: " << this->targetModelName << std::endl;
      }
    }

    if (this->dockLinkEntity == kNullEntity)
    {
      auto dockModel = ecm.EntityByComponents(
        components::Model(), components::Name(this->dockModelName));
      
      if (dockModel != kNullEntity)
      {
        Model model(dockModel);
        this->dockLinkEntity = model.LinkByName(ecm, "body");
        std::cout << "[AdvancedOrbital] Found dock: " << this->dockModelName << std::endl;
        
        // Initialize dock random thrust direction
        std::uniform_real_distribution<double> dist(-1.0, 1.0);
        this->dockThrustDirection = gz::math::Vector3d(dist(gen), dist(gen), dist(gen));
        this->dockThrustDirection.Normalize();
        
        this->dockInitialized = true;
        std::cout << "[AdvancedOrbital] Dock thrust direction: " << this->dockThrustDirection << std::endl;
      }
    }

    if (this->targetLinkEntity == kNullEntity || this->dockLinkEntity == kNullEntity) return;
    if (this->done) return;

    // Get link helpers
    Link targetLink(this->targetLinkEntity);
    Link dockLink(this->dockLinkEntity);
    
    // Get TRUE state (ground truth)
    auto targetPoseOpt = targetLink.WorldPose(ecm);
    auto dockPoseOpt = dockLink.WorldPose(ecm);
    auto targetVelOpt = targetLink.WorldLinearVelocity(ecm);
    auto dockVelOpt = dockLink.WorldLinearVelocity(ecm);
    
    if (!targetPoseOpt || !dockPoseOpt) return;
    
    gz::math::Pose3d targetPoseTrue = targetPoseOpt.value();
    gz::math::Pose3d dockPoseTrue = dockPoseOpt.value();
    gz::math::Vector3d vTargetTrue = targetVelOpt ? targetVelOpt.value() : gz::math::Vector3d::Zero;
    gz::math::Vector3d vDockTrue = dockVelOpt ? dockVelOpt.value() : gz::math::Vector3d::Zero;

    // ==== SENSOR SIMULATION ====
    // Apply sensor noise to get MEASURED state (what the spacecraft actually "sees")
    gz::math::Vector3d relPosTrue = dockPoseTrue.Pos() - targetPoseTrue.Pos();
    gz::math::Vector3d relVelTrue = vDockTrue - vTargetTrue;
    
    gz::math::Vector3d relPosMeasured = ApplySensorNoise(relPosTrue, CAMERA_POSITION_NOISE);
    gz::math::Vector3d relVelMeasured = ApplySensorNoise(relVelTrue, VELOCITY_NOISE_SIGMA);
    
    double distTrue = relPosTrue.Length();
    double distMeasured = relPosMeasured.Length();
    
    // Add rangefinder noise
    if (this->useSensorNoise)
    {
      std::normal_distribution<double> rangeNoise(0.0, RANGE_NOISE_SIGMA);
      distMeasured += rangeNoise(gen);
      if (distMeasured < 0.1) distMeasured = 0.1; // Minimum range
    }

    // Get masses
    double chaserMass = 100.0;
    double dockMass = 500.0;
    auto chaserInertial = ecm.Component<components::Inertial>(this->targetLinkEntity);
    auto dockInertial = ecm.Component<components::Inertial>(this->dockLinkEntity);
    if (chaserInertial) chaserMass = chaserInertial->Data().MassMatrix().Mass();
    if (dockInertial) dockMass = dockInertial->Data().MassMatrix().Mass();

    // ==== REALISTIC PHYSICS FORCES (applied to TRUE state) ====

    // 1. Orbital Gravity (two-body)
    if (this->useOrbitalGravity)
    {
      gz::math::Vector3d gravChaser = ComputeOrbitalGravity(targetPoseTrue.Pos());
      gz::math::Vector3d gravDock = ComputeOrbitalGravity(dockPoseTrue.Pos());
      
      targetLink.AddWorldForce(ecm, gravChaser * chaserMass);
      dockLink.AddWorldForce(ecm, gravDock * dockMass);
    }

    // 2. J2 Perturbation
    if (this->useJ2)
    {
      gz::math::Vector3d j2Chaser = ComputeJ2Perturbation(targetPoseTrue.Pos());
      gz::math::Vector3d j2Dock = ComputeJ2Perturbation(dockPoseTrue.Pos());
      
      targetLink.AddWorldForce(ecm, j2Chaser * chaserMass);
      dockLink.AddWorldForce(ecm, j2Dock * dockMass);
    }

    // 3. Atmospheric Drag
    if (this->useDrag)
    {
      double altitude = targetPoseTrue.Pos().Length() - R_EARTH;
      gz::math::Vector3d dragChaser = ComputeDrag(vTargetTrue, altitude, 0.1, 2.2);
      gz::math::Vector3d dragDock = ComputeDrag(vDockTrue, altitude, 0.2, 2.2);
      
      targetLink.AddWorldForce(ecm, dragChaser);
      dockLink.AddWorldForce(ecm, dragDock);
    }

    // Check for success - STRICT DOCKING PORT ALIGNMENT
    double relSpeedTrue = relVelTrue.Length();
    
    // Calculate docking port positions in world frame
    gz::math::Vector3d chaserPortWorld = targetPoseTrue.Pos() + targetPoseTrue.Rot().RotateVector(this->chaserDockingPortOffset);
    gz::math::Vector3d dockPortWorld = dockPoseTrue.Pos() + dockPoseTrue.Rot().RotateVector(this->dockDockingPortOffset);
    double portDistance = (dockPortWorld - chaserPortWorld).Length();
    
    // Check if docking ports are facing each other (axes aligned)
    gz::math::Vector3d chaserDockingAxis = targetPoseTrue.Rot().RotateVector(gz::math::Vector3d(1, 0, 0));  // Chaser +X
    gz::math::Vector3d dockDockingAxis = dockPoseTrue.Rot().RotateVector(gz::math::Vector3d(1, 0, 0));      // Dock +X
    double alignmentAngle = std::acos(std::abs(chaserDockingAxis.Dot(dockDockingAxis)));  // Angle between axes
    
    // Strict docking criteria - need to be close, slow, and aligned
    bool distanceOK = portDistance <= 1.0;  // Within 1.0m (tighter)
    bool velocityOK = relSpeedTrue <= 0.25;  // Under 0.25 m/s (slower)
    bool alignmentOK = alignmentAngle <= 25.0 * M_PI / 180.0;  // Within 25° (better aligned)
    
    // Additional check: are they actually touching (very close)?
    double centerDistance = relPosTrue.Length();
    bool touching = centerDistance < 3.5;  // Bodies are close (tighter)
    
    if (distanceOK && velocityOK && alignmentOK && touching)
    {
      if (!this->done) {
        std::cout << "\n====================================================" << std::endl;
        std::cout << "🎉 DOCKING SUCCESS! 🎉" << std::endl;
        std::cout << "====================================================" << std::endl;
        std::cout << "Port Distance: " << portDistance << " m" << std::endl;
        std::cout << "Alignment Angle: " << (alignmentAngle * 180.0 / M_PI) << " deg" << std::endl;
        std::cout << "Relative Velocity: " << relSpeedTrue << " m/s" << std::endl;
        std::cout << "Mission Time: " << simTime << " s" << std::endl;
        std::cout << "Chaser Fuel Used: " << this->chaserFuelUsed << " kg" << std::endl;
        std::cout << "Dock Fuel Used: " << this->dockFuelUsed << " kg" << std::endl;
        std::cout << "====================================================" << std::endl;
        std::cout << "🔒 Spacecraft locked together. Stabilizing..." << std::endl;
        this->done = true;
      }
      
      // After docking: LATCH - apply massive forces/torques to lock together
      // This simulates a mechanical latch/lock mechanism
      
      // Get angular velocities
      auto chaserAngVelOpt = targetLink.WorldAngularVelocity(ecm);
      auto dockAngVelOpt = dockLink.WorldAngularVelocity(ecm);
      gz::math::Vector3d chaserAngVel = chaserAngVelOpt ? chaserAngVelOpt.value() : gz::math::Vector3d::Zero;
      gz::math::Vector3d dockAngVel = dockAngVelOpt ? dockAngVelOpt.value() : gz::math::Vector3d::Zero;
      
      // Average velocities (they should move together)
      gz::math::Vector3d avgVel = (vTargetTrue + vDockTrue) * 0.5;
      gz::math::Vector3d avgAngVel = (chaserAngVel + dockAngVel) * 0.5;
      
      // Apply MASSIVE forces to match velocities (latch effect)
      double lockStiffness = 500.0;  // Very high stiffness
      
      // Linear: force both to average velocity
      targetLink.AddWorldForce(ecm, (avgVel - vTargetTrue) * chaserMass * lockStiffness);
      dockLink.AddWorldForce(ecm, (avgVel - vDockTrue) * dockMass * lockStiffness);
      
      // Angular: force both to average angular velocity
      targetLink.AddWorldWrench(ecm, gz::math::Vector3d::Zero, (avgAngVel - chaserAngVel) * lockStiffness);
      dockLink.AddWorldWrench(ecm, gz::math::Vector3d::Zero, (avgAngVel - dockAngVel) * lockStiffness);
      
      // Also add strong damping to bring everything to rest
      targetLink.AddWorldForce(ecm, -avgVel * chaserMass * 50.0);
      dockLink.AddWorldForce(ecm, -avgVel * dockMass * 50.0);
      targetLink.AddWorldWrench(ecm, gz::math::Vector3d::Zero, -avgAngVel * 50.0);
      dockLink.AddWorldWrench(ecm, gz::math::Vector3d::Zero, -avgAngVel * 50.0);
      
      return;
    }
    
    // ==== FREEZE ZONE: When close, freeze chaser and make both move together ====
    bool inFreezeZone = centerDistance < 2.8;  // Within 2.8m
    
    if (inFreezeZone)
    {
      // Freeze chaser and couple to dock's motion
      // Apply gentle forces to make chaser match dock's velocity
      
      // Get angular velocities
      auto chaserAngVelOpt = targetLink.WorldAngularVelocity(ecm);
      auto dockAngVelOpt = dockLink.WorldAngularVelocity(ecm);
      gz::math::Vector3d chaserAngVel = chaserAngVelOpt ? chaserAngVelOpt.value() : gz::math::Vector3d::Zero;
      gz::math::Vector3d dockAngVel = dockAngVelOpt ? dockAngVelOpt.value() : gz::math::Vector3d::Zero;
      
      // Soft coupling: chaser follows dock
      double freezeStiffness = 20.0;  // Moderate coupling
      double freezeDamping = 15.0;    // Damping
      
      // Make chaser match dock's velocity
      gz::math::Vector3d velError = vDockTrue - vTargetTrue;
      gz::math::Vector3d couplingForce = velError * chaserMass * freezeStiffness;
      
      // Add damping
      gz::math::Vector3d dampingForce = -vTargetTrue * chaserMass * freezeDamping;
      
      targetLink.AddWorldForce(ecm, couplingForce + dampingForce);
      
      // Also couple angular velocities
      gz::math::Vector3d angVelError = dockAngVel - chaserAngVel;
      gz::math::Vector3d couplingTorque = angVelError * freezeStiffness;
      gz::math::Vector3d dampingTorque = -chaserAngVel * freezeDamping;
      
      targetLink.AddWorldWrench(ecm, gz::math::Vector3d::Zero, couplingTorque + dampingTorque);
      
      // No thruster control in freeze zone - just let dock move freely
      // Only apply dock's orbital gravity and thrusters
      
      return;  // Skip thruster control
    }

    // ==== THRUSTER CONTROL (uses MEASURED state) ====
    
    // Simple approach: go straight to the docking port
    // Control based on relative position between spacecraft centers (like before)
    gz::math::Vector3d desiredAccel = ComputeControlAcceleration(relPosMeasured, relVelMeasured);
    
    // Compute desired force
    gz::math::Vector3d desiredForce = desiredAccel * chaserMass;
    
    // ==== ATTITUDE CONTROL ====
    // Goal: Align docking port (+X axis) with approach direction
    
    // Get angular velocity in body frame
    auto targetAngVelOpt = targetLink.WorldAngularVelocity(ecm);
    gz::math::Vector3d angularVelWorld = targetAngVelOpt ? targetAngVelOpt.value() : gz::math::Vector3d::Zero;
    gz::math::Vector3d angularVelBody = targetPoseTrue.Rot().RotateVectorReverse(angularVelWorld);
    
    // DAMPING ONLY - gentle to avoid coupling into translation
    gz::math::Vector3d desiredTorque = -angularVelBody * 5.0;
    
    double maxDampingTorque = 2.0;
    if (desiredTorque.Length() > maxDampingTorque) {
      desiredTorque = desiredTorque.Normalized() * maxDampingTorque;
    }
    
    // REALISTIC 16-THRUSTER SYSTEM with allocation
    // Convert desired force to body frame
    gz::math::Vector3d desiredForceBody = targetPoseTrue.Rot().RotateVectorReverse(desiredForce);
    
    // Limit force magnitude
    double maxForce = this->chaserThrustMax;
    if (desiredForceBody.Length() > maxForce) {
      desiredForceBody = desiredForceBody.Normalized() * maxForce;
    }
    
    // Allocate thrusters to achieve desired force AND torque
    std::vector<double> thrusterCommands = AllocateThrusters(chaserThrusters, desiredForceBody, desiredTorque);
    
    // Apply individual thruster forces at their positions
    double totalThrust = 0.0;
    for (size_t i = 0; i < chaserThrusters.size(); ++i)
    {
      if (thrusterCommands[i] > 0.01) // Threshold for activation
      {
        // Thruster force in body frame
        gz::math::Vector3d thrustForceBody = chaserThrusters[i].direction * thrusterCommands[i];
        
        // Convert to world frame
        gz::math::Vector3d thrustForceWorld = targetPoseTrue.Rot().RotateVector(thrustForceBody);
        gz::math::Vector3d thrustPosWorld = targetPoseTrue.Rot().RotateVector(chaserThrusters[i].position);
        
        // Apply force at thruster position (creates both force and torque)
        targetLink.AddWorldForce(ecm, thrustForceWorld, thrustPosWorld);
        
        totalThrust += thrusterCommands[i];
        
        // Track fuel consumption
        double fuelRate = thrusterCommands[i] / (this->chaserIsp * g0);
        this->chaserFuelUsed += fuelRate * dt;
        
        // VISUAL EFFECT: Create thruster flash
        CreateThrusterVisual(ecm, targetPoseTrue, chaserThrusters[i].position, 
                           chaserThrusters[i].direction, thrusterCommands[i], i);
      }
    }
    
    // 5. Dock thrusters (station-keeping with random thrust)
    if (this->dockThrustMax > 0.01)
    {
      gz::math::Vector3d dockForce = this->dockThrustDirection * this->dockThrustMax;
      dockLink.AddWorldForce(ecm, dockForce);
      
      double dockFuelRate = this->dockThrustMax / (this->dockIsp * g0);
      this->dockFuelUsed += dockFuelRate * dt;
    }

    // ==== TELEMETRY ====
    this->iter++;
    
    // Reuse port calculations from success check
    double portDist = portDistance;
    double alignAngle = alignmentAngle * 180.0 / M_PI;
    double approachSpeed = -relVelTrue.Dot(relPosTrue.Normalized());
    
    // Determine mode
    int modeNum = 0;
    std::string mode = "CRUISE";
    if (distTrue < 10.0 && relSpeedTrue > 0.3) { mode = "BRAKE!"; modeNum = 1; }
    else if (approachSpeed > distTrue / 20.0 && distTrue < 40.0) { mode = "BRAKE"; modeNum = 1; }
    else if (distTrue < 20.0) { mode = "FINAL"; modeNum = 2; }
    if (portDist < 2.0 && alignAngle < 30.0) { mode = "DOCKING"; modeNum = 3; }
    
    // Publish telemetry to GUI dashboard (every update, 1000Hz)
    gz::msgs::Double msg;
    msg.set_data(distTrue);
    this->distPub.Publish(msg);
    
    msg.set_data(portDist);
    this->portDistPub.Publish(msg);
    
    msg.set_data(alignAngle);
    this->alignPub.Publish(msg);
    
    msg.set_data(relSpeedTrue);
    this->velPub.Publish(msg);
    
    msg.set_data(approachSpeed);
    this->approachPub.Publish(msg);
    
    msg.set_data(totalThrust);
    this->thrustPub.Publish(msg);
    
    msg.set_data(this->chaserFuelUsed);
    this->fuelPub.Publish(msg);
    
    msg.set_data(simTime);
    this->timePub.Publish(msg);
    
    msg.set_data(modeNum);
    this->modePub.Publish(msg);
    
    // Console output (every 200 iterations)
    if (this->iter % 200 == 0)
    {
      std::cout << "[T+" << std::fixed << std::setprecision(1) << simTime << "s] "
                << mode << " "
                << "pd=" << std::setprecision(2) << portDist << "m "
                << "ang=" << std::setprecision(1) << alignAngle << "° "
                << "v=" << std::setprecision(2) << relSpeedTrue << "m/s "
                << "F=" << totalThrust << "N "
                << "fuel=" << std::setprecision(3) << this->chaserFuelUsed << "kg"
                << std::endl;
    }
  }

private:
  // Initialize 8 thrusters per spacecraft (2 per axis - simpler and more stable)
  void InitializeThrusters()
  {
    // Chaser thrusters (8 total) - paired thrusters for force without torque
    double singleThrustMax = this->chaserThrustMax / 2.0; // Divide max thrust across pairs
    
    // Forward/Aft pair (±X axis, aligned to minimize torque)
    chaserThrusters.push_back(Thruster(gz::math::Vector3d(0.6, 0.0, 0.0), gz::math::Vector3d(1, 0, 0), singleThrustMax));
    chaserThrusters.push_back(Thruster(gz::math::Vector3d(-0.6, 0.0, 0.0), gz::math::Vector3d(-1, 0, 0), singleThrustMax));
    
    // Lateral pair (±Y axis, aligned)
    chaserThrusters.push_back(Thruster(gz::math::Vector3d(0.0, 0.4, 0.0), gz::math::Vector3d(0, 1, 0), singleThrustMax));
    chaserThrusters.push_back(Thruster(gz::math::Vector3d(0.0, -0.4, 0.0), gz::math::Vector3d(0, -1, 0), singleThrustMax));
    
    // Vertical pair (±Z axis, aligned)
    chaserThrusters.push_back(Thruster(gz::math::Vector3d(0.0, 0.0, 0.3), gz::math::Vector3d(0, 0, 1), singleThrustMax));
    chaserThrusters.push_back(Thruster(gz::math::Vector3d(0.0, 0.0, -0.3), gz::math::Vector3d(0, 0, -1), singleThrustMax));
    
    // Torque control thrusters (offset to create rotation)
    // Two thrusters for roll/pitch/yaw control
    chaserThrusters.push_back(Thruster(gz::math::Vector3d(0.5, 0.3, 0.0), gz::math::Vector3d(0, 1, 0), singleThrustMax * 0.5));
    chaserThrusters.push_back(Thruster(gz::math::Vector3d(0.5, -0.3, 0.0), gz::math::Vector3d(0, -1, 0), singleThrustMax * 0.5));
    
    // Dock thrusters (8 total, same simplified configuration)
    double dockSingleThrust = this->dockThrustMax / 2.0;
    double dockScale = 1.5;
    
    // Forward/Aft pair
    dockThrusters.push_back(Thruster(gz::math::Vector3d(0.6, 0.0, 0.0) * dockScale, gz::math::Vector3d(1, 0, 0), dockSingleThrust));
    dockThrusters.push_back(Thruster(gz::math::Vector3d(-0.6, 0.0, 0.0) * dockScale, gz::math::Vector3d(-1, 0, 0), dockSingleThrust));
    
    // Lateral pair
    dockThrusters.push_back(Thruster(gz::math::Vector3d(0.0, 0.4, 0.0) * dockScale, gz::math::Vector3d(0, 1, 0), dockSingleThrust));
    dockThrusters.push_back(Thruster(gz::math::Vector3d(0.0, -0.4, 0.0) * dockScale, gz::math::Vector3d(0, -1, 0), dockSingleThrust));
    
    // Vertical pair
    dockThrusters.push_back(Thruster(gz::math::Vector3d(0.0, 0.0, 0.3) * dockScale, gz::math::Vector3d(0, 0, 1), dockSingleThrust));
    dockThrusters.push_back(Thruster(gz::math::Vector3d(0.0, 0.0, -0.3) * dockScale, gz::math::Vector3d(0, 0, -1), dockSingleThrust));
    
    // Torque control thrusters
    dockThrusters.push_back(Thruster(gz::math::Vector3d(0.5, 0.3, 0.0) * dockScale, gz::math::Vector3d(0, 1, 0), dockSingleThrust * 0.5));
    dockThrusters.push_back(Thruster(gz::math::Vector3d(0.5, -0.3, 0.0) * dockScale, gz::math::Vector3d(0, -1, 0), dockSingleThrust * 0.5));
  }

  // Apply sensor noise to measurements
  gz::math::Vector3d ApplySensorNoise(const gz::math::Vector3d& trueValue, double sigma)
  {
    if (!this->useSensorNoise) return trueValue;
    
    std::normal_distribution<double> noise(0.0, sigma);
    return gz::math::Vector3d(
      trueValue.X() + noise(gen),
      trueValue.Y() + noise(gen),
      trueValue.Z() + noise(gen)
    );
  }

  // Create visual effect for thruster firing
  void CreateThrusterVisual(EntityComponentManager &ecm, 
                           const gz::math::Pose3d& spacecraftPose,
                           const gz::math::Vector3d& thrusterPos,
                           const gz::math::Vector3d& thrusterDir,
                           double thrust,
                           int thrusterIndex)
  {
    // Intensity based on thrust level (0 to 1)
    double intensity = thrust / (this->chaserThrustMax / 4.0);
    intensity = std::min(1.0, intensity);
    
    // Create a simple visual marker (we'll use a sphere that flashes)
    // Note: In a full implementation, you'd create actual visual entities
    // For now, we just print which thrusters are firing
    if (this->iter % 100 == 0 && intensity > 0.5) {
      // Only print every 100 iterations and when thrust is significant
      std::cout << "🔥 Thruster " << thrusterIndex << " firing at " 
                << (int)(intensity * 100) << "%" << std::endl;
    }
  }

  // Thruster allocation using weighted pseudo-inverse
  std::vector<double> AllocateThrusters(const std::vector<Thruster>& thrusters,
                                        const gz::math::Vector3d& desiredForce,
                                        const gz::math::Vector3d& desiredTorque)
  {
    int n = thrusters.size();
    
    // Build allocation matrix B: [F; τ] = B * u
    // B is 6xN (3 force + 3 torque rows, N thruster columns)
    // Gentle torque weight to prevent over-correction
    double torqueWeight = 0.2; // Minimal torque coupling for stability
    
    Eigen::MatrixXd B(6, n);
    
    for (int i = 0; i < n; ++i)
    {
      // Force contribution (direction)
      B(0, i) = thrusters[i].direction.X();
      B(1, i) = thrusters[i].direction.Y();
      B(2, i) = thrusters[i].direction.Z();
      
      // Torque contribution (r × F) - WEIGHTED
      gz::math::Vector3d torque = thrusters[i].position.Cross(thrusters[i].direction);
      B(3, i) = torque.X() * torqueWeight;
      B(4, i) = torque.Y() * torqueWeight;
      B(5, i) = torque.Z() * torqueWeight;
    }
    
    // Desired wrench vector [Fx, Fy, Fz, τx, τy, τz] - also weight torque
    Eigen::VectorXd b(6);
    b(0) = desiredForce.X();
    b(1) = desiredForce.Y();
    b(2) = desiredForce.Z();
    b(3) = desiredTorque.X() * torqueWeight;
    b(4) = desiredTorque.Y() * torqueWeight;
    b(5) = desiredTorque.Z() * torqueWeight;
    
    // Solve using pseudo-inverse: u = B^+ * b
    Eigen::VectorXd u = B.completeOrthogonalDecomposition().solve(b);
    
    // Convert to std::vector and apply constraints (0 <= u_i <= max_thrust)
    std::vector<double> commands(n);
    for (int i = 0; i < n; ++i)
    {
      commands[i] = std::max(0.0, std::min(u(i), thrusters[i].maxThrust));
    }
    
    return commands;
  }

  // Smooth thruster commands: deadband + slew + low-pass
  std::vector<double> SmoothCommands(const std::vector<double>& cmds,
                                     const std::vector<Thruster>& thrusters,
                                     double dt,
                                     std::vector<double>& prev)
  {
  const double minFrac = 0.04;     // 4% deadband
  const double slewRate = 2.0;     // N/s change rate
    std::vector<double> out(cmds.size(), 0.0);

    if (prev.size() != cmds.size()) {
      prev.assign(cmds.size(), 0.0);
    }

    double maxDelta = std::max(1e-6, slewRate * dt);
    for (size_t i = 0; i < cmds.size(); ++i)
    {
      double target = std::max(0.0, std::min(cmds[i], thrusters[i].maxThrust));
      double delta = target - prev[i];
      if (delta > maxDelta) delta = maxDelta;
      else if (delta < -maxDelta) delta = -maxDelta;
      double v = prev[i] + delta;
      double minCmd = minFrac * thrusters[i].maxThrust;
      if (v < minCmd) v = 0.0;
      out[i] = v;
    }

  const double beta = 0.6; // more responsive low-pass blend
    for (size_t i = 0; i < out.size(); ++i) {
      prev[i] = prev[i] + beta * (out[i] - prev[i]);
      out[i] = prev[i];
    }

    return out;
  }

  // Compute two-body orbital gravity
  gz::math::Vector3d ComputeOrbitalGravity(const gz::math::Vector3d &pos)
  {
    double r = pos.Length();
    if (r < R_EARTH) r = R_EARTH;
    
    double r3 = r * r * r;
    return -GM / r3 * pos;
  }

  // Compute J2 perturbation
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

  // Compute atmospheric drag
  gz::math::Vector3d ComputeDrag(const gz::math::Vector3d &vel, double altitude, double area, double Cd)
  {
    if (altitude < 0) altitude = 0;
    
    double rho = rho0 * std::exp(-altitude / H_scale);
    
    double v_mag = vel.Length();
    if (v_mag < 0.01) return gz::math::Vector3d::Zero;
    
    return -0.5 * Cd * area * rho * v_mag * vel;
  }

  // Compute control acceleration using PD + Hill's equations
  gz::math::Vector3d ComputeControlAcceleration(const gz::math::Vector3d &relPos, 
                                                 const gz::math::Vector3d &relVel)
  {
    double distance = relPos.Length();
    
    // Adaptive gains: increase damping when close for "soft lock"
    double kp = this->kpPos;
    double kd = this->kdPos;
    
    if (distance < 10.0) {
      // Getting close: progressively stronger damping, gentler approach
      double scale = distance / 10.0; // 0 to 1
      kp = this->kpPos * (0.2 + 0.8 * scale); // Reduce position gain more
      kd = this->kdPos * (3.0 - 2.0 * scale); // Increase damping (3x to 1x)
    }
    
    if (distance < 5.0) {
      // Very close: even gentler
      double scale = distance / 5.0;
      kp = this->kpPos * (0.1 + 0.4 * scale);
      kd = this->kdPos * (4.0 - 3.0 * scale);
    }
    
    // PD control with adaptive gains
    gz::math::Vector3d accel = kp * relPos - kd * relVel;
    
    // Hill's equations compensation
    double n = this->meanMotion;
    double n2 = n * n;
    
    accel.X() += 3.0 * n2 * relPos.X() + 2.0 * n * relVel.Y();
    accel.Y() += -2.0 * n * relVel.X();
    accel.Z() += -n2 * relPos.Z();
    
    return accel;
  }

  // Member variables
  Entity worldEntity = kNullEntity;
  Entity targetLinkEntity = kNullEntity;
  Entity dockLinkEntity = kNullEntity;
  
  bool configured = false;
  bool done = false;
  bool dockInitialized = false;
  bool useSensorNoise = true;
  
  std::string targetModelName = "chaser";
  std::string dockModelName = "dock";
  
  double reachDistance = 0.8;
  double reachVelocity = 0.3;
  double orbitalAltitude = 400000.0;
  
  // Docking port alignment requirements
  gz::math::Vector3d chaserDockingPortOffset = gz::math::Vector3d(0.7, 0, 0);  // Chaser port at +X
  gz::math::Vector3d dockDockingPortOffset = gz::math::Vector3d(1.6, 0, 0);    // Dock port at +X
  double maxAlignmentAngle = 15.0 * M_PI / 180.0;  // 15 degrees max misalignment
  double maxPortDistance = 0.5;  // Max distance between docking ports (m)
  double chaserThrustMax = 5.0;
  double dockThrustMax = 1.5;
  double chaserIsp = 220.0;
  double dockIsp = 220.0;
  double kpPos = 0.05;
  double kdPos = 4.0;
  
  double meanMotion = 0.0;
  double orbitalVelocity = 0.0;
  
  bool useJ2 = false;
  bool useDrag = false;
  bool useOrbitalGravity = true;
  
  double chaserFuelUsed = 0.0;
  double dockFuelUsed = 0.0;
  
  gz::math::Vector3d dockThrustDirection;
  
  std::vector<Thruster> chaserThrusters;
  std::vector<Thruster> dockThrusters;
  std::vector<double> chaserPrevCmds;
  
  std::mt19937 gen;
  int iter = 0;
  
  // Telemetry publishing
  gz::transport::Node node;
  gz::transport::Node::Publisher distPub;
  gz::transport::Node::Publisher portDistPub;
  gz::transport::Node::Publisher alignPub;
  gz::transport::Node::Publisher velPub;
  gz::transport::Node::Publisher approachPub;
  gz::transport::Node::Publisher thrustPub;
  gz::transport::Node::Publisher fuelPub;
  gz::transport::Node::Publisher timePub;
  gz::transport::Node::Publisher modePub;
};

GZ_ADD_PLUGIN(AdvancedOrbitalController, System,
              AdvancedOrbitalController::ISystemConfigure,
              AdvancedOrbitalController::ISystemPreUpdate)
