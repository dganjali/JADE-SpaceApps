#include <gz/sim/Model.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>

#include <unordered_map>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <chrono>
#include <random>

namespace random_movement_controller
{
  class Plugin : public gz::sim::System,
                 public gz::sim::ISystemConfigure,
                 public gz::sim::ISystemPreUpdate
  {
    public: void Configure(const gz::sim::Entity &entity,
                           const std::shared_ptr<const sdf::Element> &sdf,
                           gz::sim::EntityComponentManager &ecm,
                           gz::sim::EventManager &/*eventMgr*/) override
    {
      this->model = gz::sim::Model(entity);
      this->modelName = this->model.Name(ecm);

      // Cache base link
      auto base = this->model.LinkByName(ecm, "dock_body");
      if (base)
        this->baseLink = base;

      // Random movement parameters
      if (sdf)
      {
        if (sdf->HasElement("movement_rate"))
          this->movementRate = sdf->Get<double>("movement_rate");
        if (sdf->HasElement("max_force"))
          this->maxForce = sdf->Get<double>("max_force");
        if (sdf->HasElement("movement_radius"))
          this->movementRadius = sdf->Get<double>("movement_radius");
        if (sdf->HasElement("center_position"))
        {
          auto centerPos = sdf->Get<gz::math::Vector3d>("center_position");
          this->centerPosition = centerPos;
        }
      }

      // Initialize random number generator
      this->rng.seed(std::chrono::steady_clock::now().time_since_epoch().count());
      
      // Initialize target position
      this->targetPosition = this->centerPosition;
      this->lastUpdateTime = 0.0;
    }

    public: void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
    {
      if (this->baseLink == gz::sim::kNullEntity)
        return;

      double now = std::chrono::duration<double>(info.simTime).count();
      double dt = std::chrono::duration<double>(info.dt).count();
      
      if (dt <= 0) return;

      // Update target position periodically
      if (now - this->lastUpdateTime >= (1.0 / this->movementRate))
      {
        this->lastUpdateTime = now;
        this->UpdateTargetPosition();
      }

      // Get current pose
      auto poseComp = ecm.Component<gz::sim::components::Pose>(this->baseLink);
      if (!poseComp) return;

      const auto &currentPose = poseComp->Data();
      gz::math::Vector3d currentPos = currentPose.Pos();

      // Calculate force to move toward target
      gz::math::Vector3d positionError = this->targetPosition - currentPos;
      double distance = positionError.Length();

      // Apply force proportional to distance (with damping)
      gz::math::Vector3d force(0, 0, 0);
      if (distance > 0.1) // Only apply force if not at target
      {
        gz::math::Vector3d direction = positionError.Normalized();
        double forceMagnitude = std::min(distance * 2.0, this->maxForce);
        force = direction * forceMagnitude;
      }

      // Add some random drift
      std::uniform_real_distribution<double> driftDist(-0.5, 0.5);
      force.X() += driftDist(this->rng);
      force.Y() += driftDist(this->rng);
      force.Z() += driftDist(this->rng);

      // Apply force
      gz::math::Vector3d torque(0, 0, 0);
      gz::msgs::Wrench wrenchMsg;
      gz::msgs::Set(wrenchMsg.mutable_force(), force);
      gz::msgs::Set(wrenchMsg.mutable_torque(), torque);
      
      auto wrenchComp = ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(this->baseLink);
      if (!wrenchComp)
        ecm.CreateComponent(this->baseLink, gz::sim::components::ExternalWorldWrenchCmd(wrenchMsg));
      else
        wrenchComp->Data() = wrenchMsg;
    }

    private: void UpdateTargetPosition()
    {
      // Generate random position within movement radius
      std::uniform_real_distribution<double> angleDist(0, 2 * M_PI);
      std::uniform_real_distribution<double> radiusDist(0, this->movementRadius);
      std::uniform_real_distribution<double> heightDist(-1, 1);

      double angle = angleDist(this->rng);
      double radius = radiusDist(this->rng);
      double height = heightDist(this->rng);

      this->targetPosition = this->centerPosition + gz::math::Vector3d(
        radius * cos(angle),
        radius * sin(angle),
        height
      );
    }

    private: gz::sim::Model model{gz::sim::kNullEntity};
    private: std::string modelName;
    private: gz::sim::Entity baseLink{gz::sim::kNullEntity};
    
    // Movement parameters
    private: double movementRate{0.5}; // Hz
    private: double maxForce{2.0}; // Newtons
    private: double movementRadius{2.0}; // meters
    private: gz::math::Vector3d centerPosition{5, 0, 0}; // meters
    
    // State
    private: gz::math::Vector3d targetPosition;
    private: double lastUpdateTime;
    
    // Random number generator
    private: std::mt19937 rng;
  };
}

// Register system plugin
GZ_ADD_PLUGIN(
  random_movement_controller::Plugin,
  gz::sim::System,
  random_movement_controller::Plugin::ISystemConfigure,
  random_movement_controller::Plugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(random_movement_controller::Plugin, "random_movement_controller::Plugin")
