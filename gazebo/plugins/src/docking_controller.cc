#include <gz/sim/Model.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
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

namespace docking_controller
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
      auto base = this->model.LinkByName(ecm, "base_link");
      if (base)
        this->baseLink = base;

      // Target satellite name (default to astro_dock)
      if (sdf && sdf->HasElement("target_model"))
        this->targetModelName = sdf->Get<std::string>("target_model");
      else
        this->targetModelName = "astro_dock";

      // Control gains
      if (sdf)
      {
        if (sdf->HasElement("kp_linear"))
          this->kpLinear = sdf->Get<double>("kp_linear");
        if (sdf->HasElement("kp_angular"))
          this->kpAngular = sdf->Get<double>("kp_angular");
        if (sdf->HasElement("max_velocity"))
          this->maxVelocity = sdf->Get<double>("max_velocity");
        if (sdf->HasElement("approach_distance"))
          this->approachDistance = sdf->Get<double>("approach_distance");
      }
    }

    public: void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
    {
      if (this->baseLink == gz::sim::kNullEntity)
        return;

      // Find target model
      auto targetModel = ecm.EntityByComponents(gz::sim::components::Name(this->targetModelName));
      if (targetModel == gz::sim::kNullEntity)
        return;

      // Get poses
      auto basePoseComp = ecm.Component<gz::sim::components::Pose>(this->baseLink);
      auto targetPoseComp = ecm.Component<gz::sim::components::Pose>(targetModel);
      
      if (!basePoseComp || !targetPoseComp)
        return;

      const auto &basePose = basePoseComp->Data();
      const auto &targetPose = targetPoseComp->Data();

      // Calculate position and orientation errors
      gz::math::Vector3d positionError = targetPose.Pos() - basePose.Pos();
      gz::math::Vector3d angularError = this->CalculateAngularError(basePose.Rot(), targetPose.Rot());

      // Calculate distance
      double distance = positionError.Length();
      
      // Only approach if within reasonable distance
      if (distance > this->approachDistance)
        return;

      // Calculate desired velocities (PD controller)
      gz::math::Vector3d desiredLinearVel = positionError * this->kpLinear;
      gz::math::Vector3d desiredAngularVel = angularError * this->kpAngular;

      // Limit velocities
      desiredLinearVel = this->LimitVelocity(desiredLinearVel, this->maxVelocity);
      desiredAngularVel = this->LimitVelocity(desiredAngularVel, this->maxVelocity);

      // Apply velocity commands
      auto vComp = ecm.Component<gz::sim::components::LinearVelocityCmd>(this->baseLink);
      if (!vComp)
        ecm.CreateComponent(this->baseLink, gz::sim::components::LinearVelocityCmd(desiredLinearVel));
      else
        vComp->Data() = desiredLinearVel;

      auto wComp = ecm.Component<gz::sim::components::AngularVelocityCmd>(this->baseLink);
      if (!wComp)
        ecm.CreateComponent(this->baseLink, gz::sim::components::AngularVelocityCmd(desiredAngularVel));
      else
        wComp->Data() = desiredAngularVel;
    }

    private: gz::math::Vector3d CalculateAngularError(const gz::math::Quaterniond &current, const gz::math::Quaterniond &target)
    {
      // Calculate the rotation needed to go from current to target
      gz::math::Quaterniond errorQuat = target * current.Inverse();
      
      // Convert to axis-angle representation
      gz::math::Vector3d axis;
      double angle;
      errorQuat.AxisAngle(axis, angle);
      
      return axis * angle;
    }

    private: gz::math::Vector3d LimitVelocity(const gz::math::Vector3d &velocity, double maxSpeed)
    {
      double speed = velocity.Length();
      if (speed > maxSpeed)
        return velocity.Normalized() * maxSpeed;
      return velocity;
    }

    private: gz::sim::Model model{gz::sim::kNullEntity};
    private: std::string modelName;
    private: std::string targetModelName;
    private: gz::sim::Entity baseLink{gz::sim::kNullEntity};
    
    // Control parameters
    private: double kpLinear{0.1};
    private: double kpAngular{0.2};
    private: double maxVelocity{0.5};
    private: double approachDistance{10.0};
  };
}

// Register system plugin
GZ_ADD_PLUGIN(
  docking_controller::Plugin,
  gz::sim::System,
  docking_controller::Plugin::ISystemConfigure,
  docking_controller::Plugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(docking_controller::Plugin, "docking_controller::Plugin")
