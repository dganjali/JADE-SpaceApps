// vision_based_orbital_controller.cpp
// Realistic orbital docking with VISION-BASED navigation
// Uses simulated stereo camera triangulation instead of perfect position knowledge

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
constexpr double G = 6.67430e-11;
constexpr double M_EARTH = 5.97219e24;
constexpr double R_EARTH = 6.371e6;
constexpr double GM = G * M_EARTH;
constexpr double g0 = 9.80665;

class VisionBasedOrbitalController : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
  VisionBasedOrbitalController() : System() 
  {
    // Initialize random number generator for sensor noise
    std::random_device rd;
    this->rng = std::mt19937(rd());
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
    if (_sdf->HasElement("kp_pos"))
      this->kpPos = _sdf->Get<double>("kp_pos");
    if (_sdf->HasElement("kd_pos"))
      this->kdPos = _sdf->Get<double>("kd_pos");
    if (_sdf->HasElement("camera_noise"))
      this->cameraNoise = _sdf->Get<double>("camera_noise");
    if (_sdf->HasElement("camera_rate"))
      this->cameraRate = _sdf->Get<double>("camera_rate");

    // Calculate orbital parameters
    double r0 = R_EARTH + this->orbitalAltitude;
    this->meanMotion = std::sqrt(GM / (r0 * r0 * r0));
    this->orbitalVelocity = std::sqrt(GM / r0);

    this->worldEntity = gz::sim::worldEntity(ecm);
    this->configured = true;
    
    std::cout << "====================================================" << std::endl;
    std::cout << "[VisionBased] VISION-BASED NAVIGATION ENABLED" << std::endl;
    std::cout << "====================================================" << std::endl;
    std::cout << "Camera Update Rate: " << this->cameraRate << " Hz" << std::endl;
    std::cout << "Range Noise: ±" << this->cameraNoise << " m" << std::endl;
    std::cout << "Orbital Altitude: " << this->orbitalAltitude / 1000.0 << " km" << std::endl;
    std::cout << "Mean Motion: " << this->meanMotion << " rad/s" << std::endl;
    std::cout << "====================================================" << std::endl;
  }

  void PreUpdate(const UpdateInfo &_info, EntityComponentManager &ecm) override
  {
    if (!this->configured) return;

    auto simTime = std::chrono::duration<double>(_info.simTime).count();
    double dt = std::chrono::duration<double>(_info.dt).count();
    
    if (dt <= 0) return;

    // Lazy find entities
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
              std::cout << "[VisionBased] Found chaser: " << name << std::endl;
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
              std::cout << "[VisionBased] Found dock: " << name << std::endl;
            }
          }
          return true;
        });
      
      if (this->dockLinkEntity != kNullEntity && !this->dockInitialized)
      {
        std::uniform_real_distribution<double> dist(-1.0, 1.0);
        this->dockThrustDirection = gz::math::Vector3d(dist(rng), dist(rng), dist(rng));
        this->dockThrustDirection.Normalize();
        this->dockInitialized = true;
      }
    }

    if (this->targetLinkEntity == kNullEntity || this->dockLinkEntity == kNullEntity) return;
    if (this->done) return;

    Link targetLink(this->targetLinkEntity);
    Link dockLink(this->dockLinkEntity);
    
    // Get TRUE positions (ground truth)
    auto targetPoseOpt = targetLink.WorldPose(ecm);
    auto dockPoseOpt = dockLink.WorldPose(ecm);
    auto targetVelOpt = targetLink.WorldLinearVelocity(ecm);
    auto dockVelOpt = dockLink.WorldLinearVelocity(ecm);
    
    if (!targetPoseOpt || !dockPoseOpt) return;
    
    gz::math::Pose3d targetPose = targetPoseOpt.value();
    gz::math::Pose3d dockPose = dockPoseOpt.value();
    gz::math::Vector3d vTarget = targetVelOpt ? targetVelOpt.value() : gz::math::Vector3d::Zero;
    gz::math::Vector3d vDock = dockVelOpt ? dockVelOpt.value() : gz::math::Vector3d::Zero;

    // ==== VISION-BASED MEASUREMENT ====
    
    // Camera updates at limited rate
    this->timeSinceLastMeasurement += dt;
    double measurementPeriod = 1.0 / this->cameraRate;
    
    if (this->timeSinceLastMeasurement >= measurementPeriod)
    {
      this->timeSinceLastMeasurement = 0.0;
      
      // TRUE relative position
      gz::math::Vector3d trueRelPos = dockPose.Pos() - targetPose.Pos();
      double trueDistance = trueRelPos.Length();
      
      // Simulate stereo camera range measurement with noise
      std::normal_distribution<double> rangeDist(0.0, this->cameraNoise);
      double rangeNoise = rangeDist(rng);
      double measuredDistance = trueDistance + rangeNoise;
      measuredDistance = std::max(0.1, measuredDistance); // Prevent negative
      
      // Simulate bearing measurement (direction) with angular noise
      gz::math::Vector3d trueDirection = trueRelPos.Normalized();
      std::normal_distribution<double> angleDist(0.0, 0.01); // ~0.6 degrees
      gz::math::Vector3d angleNoise(angleDist(rng), angleDist(rng), angleDist(rng));
      gz::math::Vector3d measuredDirection = (trueDirection + angleNoise).Normalized();
      
      // Reconstruct estimated position from range + bearing
      this->estimatedRelPos = measuredDirection * measuredDistance;
      
      // Estimate velocity from position change (finite difference)
      if (this->lastEstimatedRelPos.Length() > 0.01)
      {
        this->estimatedRelVel = (this->estimatedRelPos - this->lastEstimatedRelPos) / measurementPeriod;
      }
      this->lastEstimatedRelPos = this->estimatedRelPos;
      
      this->measurementCount++;
    }

    // Use ESTIMATED values for control (not ground truth!)
    gz::math::Vector3d relPos = this->estimatedRelPos;
    gz::math::Vector3d relVel = this->estimatedRelVel;
    double dist = relPos.Length();
    double relSpeed = relVel.Length();

    // Check success using TRUE values (for scoring)
    gz::math::Vector3d trueRelPos = dockPose.Pos() - targetPose.Pos();
    gz::math::Vector3d trueRelVel = vDock - vTarget;
    double trueDist = trueRelPos.Length();
    double trueRelSpeed = trueRelVel.Length();

    if (trueDist <= this->reachDistance && trueRelSpeed <= this->reachVelocity)
    {
      std::cout << "\n====================================================" << std::endl;
      std::cout << "🎉 DOCKING SUCCESS! 🎉" << std::endl;
      std::cout << "====================================================" << std::endl;
      std::cout << "Final Distance: " << trueDist << " m" << std::endl;
      std::cout << "Relative Velocity: " << trueRelSpeed << " m/s" << std::endl;
      std::cout << "Mission Time: " << simTime << " s" << std::endl;
      std::cout << "Camera Measurements: " << this->measurementCount << std::endl;
      std::cout << "Final Estimation Error: " << (relPos - trueRelPos).Length() << " m" << std::endl;
      std::cout << "Fuel Used: " << this->chaserFuelUsed << " kg" << std::endl;
      std::cout << "====================================================" << std::endl;
      this->done = true;
      return;
    }

    // Get masses
    double chaserMass = 100.0;
    double dockMass = 500.0;
    auto chaserInertial = ecm.Component<components::Inertial>(this->targetLinkEntity);
    auto dockInertial = ecm.Component<components::Inertial>(this->dockLinkEntity);
    if (chaserInertial) chaserMass = chaserInertial->Data().MassMatrix().Mass();
    if (dockInertial) dockMass = dockInertial->Data().MassMatrix().Mass();

    // Dock thrusters
    gz::math::Vector3d dockThrust = this->dockThrustDirection * this->dockThrustMax;
    dockLink.AddWorldForce(ecm, dockThrust);
    
    double dockMassFlow = this->dockThrustMax / (this->dockIsp * g0);
    this->dockFuelUsed += dockMassFlow * dt;

    // Chaser control using ESTIMATED position/velocity
    gz::math::Vector3d accelCmd = ComputeControlAcceleration(relPos, relVel);
    gz::math::Vector3d chaserThrustCmd = accelCmd * chaserMass;
    
    // Distance-based thrust limiting
    double maxThrustAtDistance = this->chaserThrustMax;
    if (dist < 50.0)
    {
      double distFactor = std::max(0.15, dist / 50.0);
      maxThrustAtDistance = this->chaserThrustMax * distFactor;
    }
    
    // Emergency braking
    double approachSpeed = -relVel.Dot(relPos.Normalized());
    if (dist < 20.0 && approachSpeed > 0.5)
    {
      chaserThrustCmd = -relVel.Normalized() * this->chaserThrustMax * 0.8;
    }
    
    double thrustMag = chaserThrustCmd.Length();
    if (thrustMag > maxThrustAtDistance)
    {
      chaserThrustCmd = chaserThrustCmd.Normalized() * maxThrustAtDistance;
      thrustMag = maxThrustAtDistance;
    }
    
    if (thrustMag > 0.01)
    {
      targetLink.AddWorldForce(ecm, chaserThrustCmd);
      double chaserMassFlow = thrustMag / (this->chaserIsp * g0);
      this->chaserFuelUsed += chaserMassFlow * dt;
    }

    // Telemetry
    this->iter++;
    if (this->iter % 200 == 0)
    {
      double estError = (relPos - trueRelPos).Length();
      std::cout << "[T+" << std::fixed << std::setprecision(1) << simTime << "s] "
                << "dist=" << std::setprecision(2) << dist << "m (true:" << trueDist << "m) "
                << "err=" << estError << "m "
                << "rel_v=" << relSpeed << "m/s "
                << "thrust=" << thrustMag << "N"
                << std::endl;
    }
  }

private:
  gz::math::Vector3d ComputeControlAcceleration(const gz::math::Vector3d &relPos, 
                                                 const gz::math::Vector3d &relVel)
  {
    // PD control with proper signs
    gz::math::Vector3d accel = this->kpPos * relPos - this->kdPos * relVel;
    
    // Hill's equations compensation
    double n = this->meanMotion;
    double n2 = n * n;
    
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
  double orbitalAltitude = 400000.0;
  double chaserThrustMax = 2.5;
  double dockThrustMax = 1.5;
  double chaserIsp = 220.0;
  double dockIsp = 220.0;
  double kpPos = 0.02;
  double kdPos = 0.8;
  double cameraNoise = 0.05; // 5cm range noise
  double cameraRate = 10.0;  // 10 Hz

  // Computed
  double meanMotion = 0.0;
  double orbitalVelocity = 0.0;

  // Vision state
  gz::math::Vector3d estimatedRelPos{0, 0, 0};
  gz::math::Vector3d estimatedRelVel{0, 0, 0};
  gz::math::Vector3d lastEstimatedRelPos{0, 0, 0};
  double timeSinceLastMeasurement = 0.0;
  uint64_t measurementCount = 0;

  // Internal state
  bool configured{false};
  bool dockInitialized{false};
  bool done{false};
  uint64_t iter{0};
  double chaserFuelUsed{0.0};
  double dockFuelUsed{0.0};
  gz::math::Vector3d dockThrustDirection{0, 0, 0};
  std::mt19937 rng;

  Entity worldEntity;
  Entity targetModelEntity;
  Entity dockModelEntity;
  Entity targetLinkEntity{kNullEntity};
  Entity dockLinkEntity{kNullEntity};
};

GZ_ADD_PLUGIN(
  VisionBasedOrbitalController,
  System,
  VisionBasedOrbitalController::ISystemConfigure,
  VisionBasedOrbitalController::ISystemPreUpdate
)
