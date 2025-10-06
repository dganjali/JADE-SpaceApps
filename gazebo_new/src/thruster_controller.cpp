// thruster_controller.cpp
// Build with CMake against Gazebo libraries.
// This file uses gz::sim system plugin interface.

#include <gz/sim/System.hh>
#include <gz/sim/World.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/msgs/wrench.pb.h>
#include <gz/plugin/Register.hh>

#include <sdf/sdf.hh>

#include <random>
#include <iostream>

using namespace gz;
using namespace sim;

class ThrusterController : public System, public ISystemConfigure, public ISystemPreUpdate
{
public:
  ThrusterController() : System() {}

  void Configure(const Entity & /*_entity*/, const std::shared_ptr<const sdf::Element> &_sdf,
                 EntityComponentManager &ecm, EventManager &/*_eventMgr*/) override
  {
    // read parameters
    if (_sdf->HasElement("target_model"))
      this->targetModelName = _sdf->Get<std::string>("target_model");
    if (_sdf->HasElement("dock_model"))
      this->dockModelName = _sdf->Get<std::string>("dock_model");
    if (_sdf->HasElement("reach_distance"))
      this->reachDistance = _sdf->Get<double>("reach_distance");
    if (_sdf->HasElement("kp")) this->kp = _sdf->Get<double>("kp");
    if (_sdf->HasElement("kd")) this->kd = _sdf->Get<double>("kd");
    if (_sdf->HasElement("max_force")) this->maxForce = _sdf->Get<double>("max_force");
    if (_sdf->HasElement("dock_init_speed")) this->dockInitSpeed = _sdf->Get<double>("dock_init_speed");
    if (_sdf->HasElement("dock_thrust_force")) this->dockThrustForce = _sdf->Get<double>("dock_thrust_force");
    if (_sdf->HasElement("max_dock_speed")) this->maxDockSpeed = _sdf->Get<double>("max_dock_speed");

    // find world (first world)
    this->worldEntity = gz::sim::worldEntity(ecm);

    // find model entities by name (deferred — we'll search in PreUpdate)
    this->configured = true;
    std::cout << "[ThrusterController] configured. target=" << this->targetModelName
              << " dock=" << this->dockModelName << std::endl;
  }

  void PreUpdate(const UpdateInfo & /*_info*/, EntityComponentManager &ecm) override
  {
    if (!this->configured) return;

    // lazy find model and link entities
    if (this->targetModelEntity == kNullEntity || this->dockModelEntity == kNullEntity)
    {
      // search all entities
      ecm.Each<components::Name, components::Model>(
        [&](const Entity &_entity, const components::Name *_name, const components::Model *) -> bool
        {
          std::string name = _name->Data();
          if (name == this->targetModelName)
          {
            this->targetModelEntity = _entity;
            // Find the link entity
            Model model(_entity);
            auto links = model.Links(ecm);
            if (!links.empty())
            {
              this->targetLinkEntity = links[0];
              std::cout << "[ThrusterController] found target model and link: " << name << std::endl;
            }
          }
          if (name == this->dockModelName)
          {
            this->dockModelEntity = _entity;
            // Find the link entity
            Model model(_entity);
            auto links = model.Links(ecm);
            if (!links.empty())
            {
              this->dockLinkEntity = links[0];
              std::cout << "[ThrusterController] found dock model and link: " << name << std::endl;
            }
          }
          return true;
        });
      
      // if found both, init dock random thrust direction once
      if (this->dockLinkEntity != kNullEntity && !this->dockInitialized)
      {
        // set random thrust direction for dock
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dist(-1.0, 1.0);
        this->dockThrustDirection = gz::math::Vector3d(dist(gen), dist(gen), dist(gen));
        this->dockThrustDirection.Normalize();
        
        this->dockInitialized = true;
        std::cout << "[ThrusterController] dock thrust direction set to " << this->dockThrustDirection << std::endl;
      }
    }

    // Apply continuous thrust to dock to keep it moving, but limit velocity
    if (this->dockLinkEntity != kNullEntity && this->dockInitialized)
    {
      Link dockLink(this->dockLinkEntity);
      auto dockVelOpt = dockLink.WorldLinearVelocity(ecm);
      gz::math::Vector3d vDock = dockVelOpt ? dockVelOpt.value() : gz::math::Vector3d::Zero;
      
      // Only apply thrust if below max velocity to prevent runaway acceleration
      double dockSpeed = vDock.Length();
      if (dockSpeed < this->maxDockSpeed)
      {
        gz::math::Vector3d dockThrust = this->dockThrustDirection * this->dockThrustForce;
        dockLink.AddWorldForce(ecm, dockThrust);
      }
      else if (dockSpeed > this->maxDockSpeed * 1.1)
      {
        // Apply small damping if exceeding max speed by too much
        gz::math::Vector3d dampingForce = -vDock.Normalized() * this->dockThrustForce * 0.2;
        dockLink.AddWorldForce(ecm, dampingForce);
      }
    }

    if (this->targetLinkEntity == kNullEntity || this->dockLinkEntity == kNullEntity) return;

    // if already reached do nothing
    if (this->done) return;

    // get target (chaser) and dock WORLD poses using Link helper
    Link targetLink(this->targetLinkEntity);
    Link dockLink(this->dockLinkEntity);
    
    auto targetPoseOpt = targetLink.WorldPose(ecm);
    auto dockPoseOpt = dockLink.WorldPose(ecm);
    
    if (!targetPoseOpt || !dockPoseOpt) return;
    
    gz::math::Pose3d targetPose = targetPoseOpt.value();
    gz::math::Pose3d dockPose = dockPoseOpt.value();

    // get linear velocity components from links (world velocities)
    auto targetVelOpt = targetLink.WorldLinearVelocity(ecm);
    auto dockVelOpt = dockLink.WorldLinearVelocity(ecm);
    
    gz::math::Vector3d vTarget = targetVelOpt ? targetVelOpt.value() : gz::math::Vector3d::Zero;
    gz::math::Vector3d vDock = dockVelOpt ? dockVelOpt.value() : gz::math::Vector3d::Zero;

    // relative position and velocity (dock - target)
    gz::math::Vector3d relPos = dockPose.Pos() - targetPose.Pos();
    gz::math::Vector3d relVel = vDock - vTarget;
    double dist = relPos.Length();
    
    // Calculate approach velocity (closing speed) - positive means approaching
    double approachVel = 0.0;
    if (dist > 1e-6) // avoid division by zero
    {
      gz::math::Vector3d dirToTarget = relPos / dist; // normalized
      approachVel = -relVel.Dot(dirToTarget); // negative relVel dot gives approach speed
    }
    double relSpeed = relVel.Length();

    // if within reach threshold and approach velocity is low, success!
    if (dist <= this->reachDistance && std::abs(approachVel) < 0.15 && relSpeed < 0.25)
    {
      std::cout << "[ThrusterController] SUCCESS: reached dock. distance=" << dist 
                << " approach_vel=" << approachVel << " rel_speed=" << relSpeed << " m/s" << std::endl;
      this->done = true;
      return;
    }
    
    // Emergency braking if approaching too fast when close
    // Trigger earlier and more gradually than before
    if (dist < this->reachDistance * 4.0 && approachVel > 0.5)
    {
      // Calculate how much we need to slow down
      // Braking strength scales with approach velocity and proximity
      double brakingStrength = std::min(approachVel / 0.5, 2.0); // 1.0 at 0.5 m/s, 2.0 at 1.0+ m/s
      double proximityScale = 1.0;
      
      if (dist < this->reachDistance * 2.0)
      {
        // Stronger braking when very close
        proximityScale = 2.0 - (dist / (this->reachDistance * 2.0)); // 2.0 at 0m, 1.0 at 2x reach
      }
      
      // Apply braking force opposite to approach direction
      if (relSpeed > 1e-6)
      {
        gz::math::Vector3d brakingForce = -relVel.Normalized() * this->maxForce * 0.7 * brakingStrength * proximityScale;
        targetLink.AddWorldForce(ecm, brakingForce);
      }
      
      if (this->iter % 80 == 0)
      {
        std::cout << "[ThrusterController] BRAKING: dist=" << dist 
                  << " approachVel=" << approachVel << " brakingStrength=" << brakingStrength * proximityScale
                  << " relSpeed=" << relSpeed << " m/s" << std::endl;
      }
      this->iter++;
      return;
    }

    // IMPROVED PD CONTROLLER with Proper Interception Logic
    // Goal: Intercept the moving dock and perform soft docking (match velocity)
    
    // Get mass for force calculation
    double mass = 10.0; // default mass from chaser model
    auto inertialComp = ecm.Component<components::Inertial>(this->targetLinkEntity);
    if (inertialComp && inertialComp->Data().MassMatrix().Mass() > 0.0)
    {
      mass = inertialComp->Data().MassMatrix().Mass();
    }
    
    // Calculate time to intercept (estimate)
    double timeToIntercept = 1.0; // default 1 second
    if (relSpeed > 0.01)
    {
      timeToIntercept = dist / std::max(relSpeed, 0.5); // don't divide by very small numbers
      timeToIntercept = std::min(timeToIntercept, 10.0); // cap at 10 seconds
    }
    
    // Predicted dock position (lead the target)
    gz::math::Vector3d predictedDockPos = dockPose.Pos() + vDock * timeToIntercept;
    
    // Interception point error
    gz::math::Vector3d interceptError = predictedDockPos - targetPose.Pos();
    double interceptDist = interceptError.Length();
    
    // Velocity matching error (we want to match dock velocity for rendezvous)
    gz::math::Vector3d velocityError = vTarget - vDock;
    
    // Adaptive gains based on distance
    double distanceScale = 1.0;
    double approachPhase = 0.0; // 0 = far, 1 = very close
    
    if (dist < 8.0)
    {
      // Smoothly reduce gains as we get closer
      approachPhase = 1.0 - (dist / 8.0); // 0 at 8m, 1 at 0m
      distanceScale = 0.3 + 0.7 * (1.0 - approachPhase); // 100% at 8m+, 30% at 0m
    }
    
    // PD control with velocity matching
    // P term: drive towards intercept point
    // D term: damp relative velocity and match dock velocity
    gz::math::Vector3d positionForce = this->kp * distanceScale * interceptError;
    gz::math::Vector3d dampingForce = -this->kd * distanceScale * velocityError;
    
    gz::math::Vector3d force = positionForce + dampingForce;
    
    // Additional soft approach logic when very close
    if (dist < this->reachDistance * 3.0)
    {
      // Reduce force based on approach velocity
      double softScale = 1.0;
      if (approachVel > 0.3) // approaching too fast
      {
        softScale = 0.3 / std::max(approachVel, 0.31); // inverse scaling
      }
      force = force * softScale;
      
      // Add explicit velocity matching component
      gz::math::Vector3d matchingForce = (vDock - vTarget) * mass * 0.5;
      force = force + matchingForce;
    }
    
    // Convert to actual force (F = ma is already implied above)
    force = force * mass;
    
    // Limit force magnitude with safety check
    double forceMag = force.Length();
    if (forceMag > this->maxForce)
    {
      force = (force / forceMag) * this->maxForce;
    }
    else if (forceMag < 1e-6 && dist > this->reachDistance * 2.0)
    {
      // If force is too small but we're not close enough, apply minimum thrust towards dock
      if (dist > 1e-6)
      {
        force = (relPos / dist) * (this->maxForce * 0.15);
      }
    }
    
    // Debug output every 50 iterations
    if (this->iter % 50 == 0)
    {
      std::cout << "[ThrusterController] dist=" << dist << " interceptDist=" << interceptDist
                << " approachVel=" << approachVel << " phase=" << approachPhase
                << " vTarget=" << vTarget.Length() << " vDock=" << vDock.Length()
                << " force=" << forceMag << "N" << std::endl;
    }

    // Apply force to target link
    targetLink.AddWorldForce(ecm, force);

    // Optionally we can also set angular velocity / torque handling (omitted here for simplicity).
    // print debug occasionally
    this->iter++;
    if (this->iter % 200 == 0)
    {
      std::cout << "[ThrusterController] dist=" << dist << " approachVel=" << approachVel 
                << " force_mag=" << forceMag << " vTarget=" << vTarget.Length() 
                << " vDock=" << vDock.Length() << std::endl;
    }
  }

private:
  // parameters
  std::string targetModelName = "chaser";
  std::string dockModelName = "dock";
  double reachDistance = 0.5; // increased for easier success
  double kp = 1.8;  // optimized for interception
  double kd = 2.2;  // higher damping for smooth approach
  double maxForce = 12.0; // increased for better acceleration
  double dockInitSpeed = 0.15; // deprecated, keeping for compatibility
  double dockThrustForce = 4.0; // moderate thrust force for dock
  double maxDockSpeed = 0.35; // maximum velocity for dock (m/s)

  // internal
  bool configured{false};
  bool dockInitialized{false};
  bool done{false};
  uint64_t iter{0};
  gz::math::Vector3d dockThrustDirection{0, 0, 0};

  // entity refs
  Entity worldEntity;
  Entity targetModelEntity;
  Entity dockModelEntity;
  Entity targetLinkEntity{kNullEntity};
  Entity dockLinkEntity{kNullEntity};
};

// Register system plugin
GZ_ADD_PLUGIN(
  ThrusterController,
  System,
  ThrusterController::ISystemConfigure,
  ThrusterController::ISystemPreUpdate
)
