// realistic_orbital_controller.cpp
// Implements realistic orbital mechanics for LEO docking simulation
// Based on proper two-body dynamics, Hill's equations, and realistic constraints

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
#include <gz/plugin/Register.hh>

#include <sdf/sdf.hh>

#include <random>
#include <iostream>
#include <cmath>

using namespace gz;
using namespace sim;

// Physical constants (SI units)
constexpr double G = 6.67430e-11;           // Gravitational constant (m^3 kg^-1 s^-2)
constexpr double M_EARTH = 5.97219e24;      // Earth mass (kg)
constexpr double R_EARTH = 6.371e6;         // Earth radius (m)
constexpr double GM = G * M_EARTH;          // Standard gravitational parameter
constexpr double g0 = 9.80665;              // Standard gravity (m/s^2)
constexpr double J2 = 1.08263e-3;           // Earth's J2 oblateness coefficient

// Atmospheric density model parameters (exponential)
constexpr double rho0 = 1.225;              // Sea level density (kg/m^3)
constexpr double H_scale = 8500.0;          // Scale height (m)

class RealisticOrbitalController : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
  RealisticOrbitalController() : System() {}

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

    // Calculate orbital parameters
    double r0 = R_EARTH + this->orbitalAltitude;
    this->meanMotion = std::sqrt(GM / (r0 * r0 * r0));
    
    // Calculate orbital velocity for reference
    this->orbitalVelocity = std::sqrt(GM / r0);

    this->worldEntity = gz::sim::worldEntity(ecm);
    this->configured = true;
    
    std::cout << "====================================================" << std::endl;
    std::cout << "[RealisticOrbital] CONFIGURATION" << std::endl;
    std::cout << "====================================================" << std::endl;
    std::cout << "Orbital Altitude: " << this->orbitalAltitude / 1000.0 << " km" << std::endl;
    std::cout << "Orbital Velocity: " << this->orbitalVelocity << " m/s" << std::endl;
    std::cout << "Mean Motion (n): " << this->meanMotion << " rad/s" << std::endl;
    std::cout << "Orbital Period: " << (2.0 * M_PI / this->meanMotion) / 60.0 << " minutes" << std::endl;
    std::cout << "Chaser Thrust: " << this->chaserThrustMax << " N (Isp: " << this->chaserIsp << " s)" << std::endl;
    std::cout << "Dock Thrust: " << this->dockThrustMax << " N (Isp: " << this->dockIsp << " s)" << std::endl;
    std::cout << "J2 Perturbation: " << (this->useJ2 ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << "Atmospheric Drag: " << (this->useDrag ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << "Orbital Gravity: " << (this->useOrbitalGravity ? "ENABLED" : "DISABLED") << std::endl;
    std::cout << "====================================================" << std::endl;
  }

  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &ecm) override
  {
    if (!this->configured) return;

    // Get simulation time and timestep
    auto simTime = std::chrono::duration<double>(_info.simTime).count();
    double dt = std::chrono::duration<double>(_info.dt).count();
    
    if (dt <= 0) return; // Skip if paused

    // Lazy find model and link entities
    if (this->targetLinkEntity == kNullEntity || this->dockLinkEntity == kNullEntity)
    {
      ecm.Each<components::Name, components::Model>(
        [&](const Entity &_entity, const components::Name *_name, const components::Model *) -> bool
        {
          std::string name = _name->Data();
          if (name == this->targetModelName)
          {
            this->targetModelEntity = _entity;
            Model model(_entity);
            auto links = model.Links(ecm);
            if (!links.empty())
            {
              this->targetLinkEntity = links[0];
              std::cout << "[RealisticOrbital] Found chaser: " << name << std::endl;
            }
          }
          if (name == this->dockModelName)
          {
            this->dockModelEntity = _entity;
            Model model(_entity);
            auto links = model.Links(ecm);
            if (!links.empty())
            {
              this->dockLinkEntity = links[0];
              std::cout << "[RealisticOrbital] Found dock: " << name << std::endl;
            }
          }
          return true;
        });
      
      // Initialize dock with random thrust direction
      if (this->dockLinkEntity != kNullEntity && !this->dockInitialized)
      {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dist(-1.0, 1.0);
        this->dockThrustDirection = gz::math::Vector3d(dist(gen), dist(gen), dist(gen));
        this->dockThrustDirection.Normalize();
        
        this->dockInitialized = true;
        std::cout << "[RealisticOrbital] Dock thrust direction: " << this->dockThrustDirection << std::endl;
      }
    }

    if (this->targetLinkEntity == kNullEntity || this->dockLinkEntity == kNullEntity) return;
    if (this->done) return;

    // Get link helpers
    Link targetLink(this->targetLinkEntity);
    Link dockLink(this->dockLinkEntity);
    
    // Get world poses and velocities
    auto targetPoseOpt = targetLink.WorldPose(ecm);
    auto dockPoseOpt = dockLink.WorldPose(ecm);
    auto targetVelOpt = targetLink.WorldLinearVelocity(ecm);
    auto dockVelOpt = dockLink.WorldLinearVelocity(ecm);
    
    if (!targetPoseOpt || !dockPoseOpt) return;
    
    gz::math::Pose3d targetPose = targetPoseOpt.value();
    gz::math::Pose3d dockPose = dockPoseOpt.value();
    gz::math::Vector3d vTarget = targetVelOpt ? targetVelOpt.value() : gz::math::Vector3d::Zero;
    gz::math::Vector3d vDock = dockVelOpt ? dockVelOpt.value() : gz::math::Vector3d::Zero;

    // Get masses
    double chaserMass = 10.0;
    double dockMass = 20.0;
    auto chaserInertial = ecm.Component<components::Inertial>(this->targetLinkEntity);
    auto dockInertial = ecm.Component<components::Inertial>(this->dockLinkEntity);
    if (chaserInertial) chaserMass = chaserInertial->Data().MassMatrix().Mass();
    if (dockInertial) dockMass = dockInertial->Data().MassMatrix().Mass();

    // ==== REALISTIC PHYSICS FORCES ====

    // 1. Orbital Gravity (two-body)
    if (this->useOrbitalGravity)
    {
      gz::math::Vector3d gravChaser = ComputeOrbitalGravity(targetPose.Pos());
      gz::math::Vector3d gravDock = ComputeOrbitalGravity(dockPose.Pos());
      
      targetLink.AddWorldForce(ecm, gravChaser * chaserMass);
      dockLink.AddWorldForce(ecm, gravDock * dockMass);
    }

    // 2. J2 Perturbation (Earth oblateness)
    if (this->useJ2)
    {
      gz::math::Vector3d j2Chaser = ComputeJ2Perturbation(targetPose.Pos());
      gz::math::Vector3d j2Dock = ComputeJ2Perturbation(dockPose.Pos());
      
      targetLink.AddWorldForce(ecm, j2Chaser * chaserMass);
      dockLink.AddWorldForce(ecm, j2Dock * dockMass);
    }

    // 3. Atmospheric Drag
    if (this->useDrag)
    {
      double altitude = targetPose.Pos().Length() - R_EARTH;
      gz::math::Vector3d dragChaser = ComputeDrag(vTarget, altitude, 0.1, 2.2); // Area ~0.1 m^2, Cd ~2.2
      gz::math::Vector3d dragDock = ComputeDrag(vDock, altitude, 0.2, 2.2);     // Area ~0.2 m^2
      
      targetLink.AddWorldForce(ecm, dragChaser);
      dockLink.AddWorldForce(ecm, dragDock);
    }

    // ==== RELATIVE DYNAMICS ====
    
    gz::math::Vector3d relPos = dockPose.Pos() - targetPose.Pos();
    gz::math::Vector3d relVel = vDock - vTarget;
    double dist = relPos.Length();
    double relSpeed = relVel.Length();

    // Check for success (soft docking criteria)
    if (dist <= this->reachDistance && relSpeed <= this->reachVelocity)
    {
      std::cout << "\n====================================================" << std::endl;
      std::cout << "🎉 DOCKING SUCCESS! 🎉" << std::endl;
      std::cout << "====================================================" << std::endl;
      std::cout << "Final Distance: " << dist << " m" << std::endl;
      std::cout << "Relative Velocity: " << relSpeed << " m/s" << std::endl;
      std::cout << "Mission Time: " << simTime << " s" << std::endl;
      std::cout << "Chaser Fuel Used: " << this->chaserFuelUsed << " kg" << std::endl;
      std::cout << "Dock Fuel Used: " << this->dockFuelUsed << " kg" << std::endl;
      std::cout << "====================================================" << std::endl;
      this->done = true;
      return;
    }

    // ==== THRUSTER CONTROL ====

    // 4. Dock thrusters (continuous station-keeping) - DISABLED FOR NOW
    // The random thrust is causing the chaser to chase a moving target
    // gz::math::Vector3d dockThrust = this->dockThrustDirection * this->dockThrustMax;
    // dockLink.AddWorldForce(ecm, dockThrust);
    
    // Update dock fuel consumption
    // double dockMassFlow = this->dockThrustMax / (this->dockIsp * g0);
    // this->dockFuelUsed += dockMassFlow * dt;

    // 5. Chaser PD controller with Hill's equations influence
    gz::math::Vector3d accelCmd = ComputeControlAcceleration(relPos, relVel);
    
    // Convert acceleration to force
    gz::math::Vector3d chaserThrustCmd = accelCmd * chaserMass;
    
    // Distance-based thrust limiting (progressive slowdown)
    double maxThrustAtDistance = this->chaserThrustMax;
    if (dist < 60.0)
    {
      // Reduce max thrust as we get closer - more aggressive
      double distFactor = dist / 60.0; // 0.0 at dock, 1.0 at 60m
      distFactor = std::max(0.05, distFactor); // Never below 5% thrust
      maxThrustAtDistance = this->chaserThrustMax * distFactor;
    }
    
    // Calculate approach velocity (how fast we're closing)
    double approachSpeed = -relVel.Dot(relPos.Normalized());
    
    // Aggressive braking if approaching too fast for the distance
    // Safety rule: approach speed should be < distance/20 m/s
    double maxSafeSpeed = dist / 20.0; // At 20m: max 1 m/s, at 5m: max 0.25 m/s
    if (approachSpeed > maxSafeSpeed && dist < 40.0)
    {
      // Apply strong reverse thrust to slow down
      gz::math::Vector3d brakeDirection = relVel.Normalized();
      chaserThrustCmd = -brakeDirection * this->chaserThrustMax;
      
      // Override distance-based limit during emergency brake
      maxThrustAtDistance = this->chaserThrustMax;
    }
    
    // Extra safety: if very close and moving fast at all, brake hard
    if (dist < 10.0 && relSpeed > 0.3)
    {
      gz::math::Vector3d brakeDirection = relVel.Normalized();
      chaserThrustCmd = -brakeDirection * this->chaserThrustMax;
      maxThrustAtDistance = this->chaserThrustMax;
    }
    
    // Limit thrust magnitude with distance-based limit
    double thrustMag = chaserThrustCmd.Length();
    if (thrustMag > maxThrustAtDistance)
    {
      chaserThrustCmd = chaserThrustCmd.Normalized() * maxThrustAtDistance;
      thrustMag = maxThrustAtDistance;
    }
    
    // Apply chaser thrust
    if (thrustMag > 0.01) // Minimum impulse bit
    {
      targetLink.AddWorldForce(ecm, chaserThrustCmd);
      
      // Update fuel consumption (Tsiolkovsky)
      double chaserMassFlow = thrustMag / (this->chaserIsp * g0);
      this->chaserFuelUsed += chaserMassFlow * dt;
    }

    // ==== TELEMETRY ====
    this->iter++;
    if (this->iter % 200 == 0)
    {
      double approachSpeed = -relVel.Dot(relPos.Normalized());
      std::string mode = "CRUISE";
      if (dist < 10.0 && relSpeed > 0.3) mode = "BRAKE!";
      else if (approachSpeed > dist / 20.0 && dist < 40.0) mode = "BRAKE";
      else if (dist < 20.0) mode = "FINAL";
      
      std::cout << "[T+" << std::fixed << std::setprecision(1) << simTime << "s] "
                << mode << " "
                << "dist=" << std::setprecision(2) << dist << "m "
                << "v=" << relSpeed << "m/s "
                << "approach=" << approachSpeed << "m/s "
                << "thrust=" << thrustMag << "N "
                << "fuel=" << std::setprecision(3) << this->chaserFuelUsed << "kg"
                << std::endl;
    }
  }

private:
  // Compute two-body orbital gravity acceleration
  gz::math::Vector3d ComputeOrbitalGravity(const gz::math::Vector3d &pos)
  {
    double r = pos.Length();
    if (r < R_EARTH) r = R_EARTH; // Prevent singularity
    
    double r3 = r * r * r;
    return -GM / r3 * pos;
  }

  // Compute J2 perturbation (Earth oblateness)
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
    
    // Exponential atmosphere model
    double rho = rho0 * std::exp(-altitude / H_scale);
    
    // Drag force: F = -0.5 * Cd * A * rho * v * |v|
    double v_mag = vel.Length();
    if (v_mag < 0.01) return gz::math::Vector3d::Zero;
    
    return -0.5 * Cd * area * rho * v_mag * vel;
  }

  // Compute control acceleration using PD + Hill's equations
  gz::math::Vector3d ComputeControlAcceleration(const gz::math::Vector3d &relPos, 
                                                 const gz::math::Vector3d &relVel)
  {
    // PD control: accel should point FROM chaser TO dock
    // relPos = dock - chaser (vector pointing to dock)
    // We want to accelerate TOWARD the dock, so positive relPos
    // But we want to oppose velocity (damping), so negative relVel
    gz::math::Vector3d accel = this->kpPos * relPos - this->kdPos * relVel;
    
    // Optional: Add Hill's equations terms for better accuracy
    // x_ddot - 2*n*y_dot - 3*n^2*x = F_x/m
    // y_ddot + 2*n*x_dot = F_y/m
    // z_ddot + n^2*z = F_z/m
    
    double n = this->meanMotion;
    double n2 = n * n;
    
    // Compensate for orbital mechanics effects
    accel.X() += 3.0 * n2 * relPos.X() + 2.0 * n * relVel.Y();
    accel.Y() += -2.0 * n * relVel.X();
    accel.Z() += -n2 * relPos.Z();
    
    return accel;
  }

  // Parameters
  std::string targetModelName = "chaser";
  std::string dockModelName = "dock";
  double reachDistance = 0.5;
  double reachVelocity = 0.1;
  double orbitalAltitude = 400000.0;  // 400 km default
  double chaserThrustMax = 5.0;
  double dockThrustMax = 3.0;
  double chaserIsp = 220.0;  // Hydrazine-like
  double dockIsp = 220.0;
  double kpPos = 0.5;
  double kdPos = 1.0;
  bool useJ2 = false;
  bool useDrag = false;
  bool useOrbitalGravity = false;

  // Computed orbital parameters
  double meanMotion = 0.0;
  double orbitalVelocity = 0.0;

  // Internal state
  bool configured{false};
  bool dockInitialized{false};
  bool done{false};
  uint64_t iter{0};
  double chaserFuelUsed{0.0};
  double dockFuelUsed{0.0};
  gz::math::Vector3d dockThrustDirection{0, 0, 0};

  // Entity refs
  Entity worldEntity;
  Entity targetModelEntity;
  Entity dockModelEntity;
  Entity targetLinkEntity{kNullEntity};
  Entity dockLinkEntity{kNullEntity};
};

// Register system plugin
GZ_ADD_PLUGIN(
  RealisticOrbitalController,
  System,
  RealisticOrbitalController::ISystemConfigure,
  RealisticOrbitalController::ISystemPreUpdate
)
