#include <gz/sim/Model.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/boolean.pb.h>

#include <unordered_map>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <chrono>
#include <random>

namespace realistic_thruster_controller
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

      // Read thruster configuration
      this->thrusterNames = {
        "thruster_px","thruster_nx","thruster_py",
        "thruster_ny","thruster_pz","thruster_nz"
      };

      // Resolve thruster link entities and their poses
      for (const auto &name : this->thrusterNames)
      {
        auto link = this->model.LinkByName(ecm, name);
        if (link)
        {
          this->thrusterLinks[name] = link;
          // Subscribe to per-thruster topic for thrust command in Newtons
          std::string topic = "/model/" + this->modelName + "/thrusters/" + name;
          std::function<void(const gz::msgs::Double &)> callback = [this, topic](const gz::msgs::Double &_msg)
          {
            this->OnThrustTopic(topic, _msg.data());
          };
          this->node.Subscribe(topic, callback);
        }
      }

      // Cache base link
      auto base = this->model.LinkByName(ecm, "base_link");
      if (base)
        this->baseLink = base;

      // Target satellite name
      if (sdf && sdf->HasElement("target_model"))
        this->targetModelName = sdf->Get<std::string>("target_model");
      else
        this->targetModelName = "astro_dock";

      // Realistic thruster parameters
      if (sdf)
      {
        if (sdf->HasElement("max_thrust"))
          this->maxThrust = sdf->Get<double>("max_thrust");
        if (sdf->HasElement("thrust_efficiency"))
          this->thrustEfficiency = sdf->Get<double>("thrust_efficiency");
        if (sdf->HasElement("fuel_mass"))
          this->fuelMass = sdf->Get<double>("fuel_mass");
        if (sdf->HasElement("isp"))
          this->isp = sdf->Get<double>("isp");
        if (sdf->HasElement("kp_linear"))
          this->kpLinear = sdf->Get<double>("kp_linear");
        if (sdf->HasElement("kp_angular"))
          this->kpAngular = sdf->Get<double>("kp_angular");
        if (sdf->HasElement("kd_linear"))
          this->kdLinear = sdf->Get<double>("kd_linear");
        if (sdf->HasElement("kd_angular"))
          this->kdAngular = sdf->Get<double>("kd_angular");
      }

      // Initialize random number generator
      this->rng.seed(std::chrono::steady_clock::now().time_since_epoch().count());
    }

    public: void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
    {
      if (this->baseLink == gz::sim::kNullEntity)
        return;

      double dt = std::chrono::duration<double>(info.dt).count();
      if (dt <= 0) return;

      // Find target model
      auto targetModel = ecm.EntityByComponents(gz::sim::components::Name(this->targetModelName));
      if (targetModel == gz::sim::kNullEntity)
        return;

      // Get poses and velocities
      auto basePoseComp = ecm.Component<gz::sim::components::Pose>(this->baseLink);
      auto targetPoseComp = ecm.Component<gz::sim::components::Pose>(targetModel);
      auto baseVelComp = ecm.Component<gz::sim::components::LinearVelocity>(this->baseLink);
      auto baseAngVelComp = ecm.Component<gz::sim::components::AngularVelocity>(this->baseLink);
      
      if (!basePoseComp || !targetPoseComp)
        return;

      const auto &basePose = basePoseComp->Data();
      const auto &targetPose = targetPoseComp->Data();
      
      gz::math::Vector3d baseVel = baseVelComp ? baseVelComp->Data() : gz::math::Vector3d::Zero;
      gz::math::Vector3d baseAngVel = baseAngVelComp ? baseAngVelComp->Data() : gz::math::Vector3d::Zero;

      // Calculate position and orientation errors
      gz::math::Vector3d positionError = targetPose.Pos() - basePose.Pos();
      gz::math::Vector3d angularError = this->CalculateAngularError(basePose.Rot(), targetPose.Rot());

      // Calculate distance
      double distance = positionError.Length();
      
      // Only approach if within reasonable distance
      if (distance > 20.0)
        return;

      // PD Controller for desired accelerations
      gz::math::Vector3d desiredLinearAccel = positionError * this->kpLinear - baseVel * this->kdLinear;
      gz::math::Vector3d desiredAngularAccel = angularError * this->kpAngular - baseAngVel * this->kdAngular;

      // Convert desired accelerations to thruster commands
      this->CalculateThrusterCommands(desiredLinearAccel, desiredAngularAccel, basePose, ecm);

      // Apply realistic thruster forces
      this->ApplyThrusterForces(ecm, dt);
    }

    private: void CalculateThrusterCommands(const gz::math::Vector3d &desiredLinearAccel,
                                           const gz::math::Vector3d &desiredAngularAccel,
                                           const gz::math::Pose3d &basePose,
                                           gz::sim::EntityComponentManager &ecm)
    {
      // Get satellite mass (approximate)
      double mass = 10.0; // kg
      
      // Calculate required forces and torques
      gz::math::Vector3d requiredForce = desiredLinearAccel * mass;
      gz::math::Vector3d requiredTorque = desiredAngularAccel * 0.5; // Approximate moment of inertia

      // Clear previous commands
      this->thrust.clear();

      // Calculate thruster directions and positions
      std::map<std::string, gz::math::Vector3d> thrusterDirections;
      std::map<std::string, gz::math::Vector3d> thrusterPositions;
      
      // Define thruster directions (in world frame)
      thrusterDirections["thruster_px"] = gz::math::Vector3d(1, 0, 0);
      thrusterDirections["thruster_nx"] = gz::math::Vector3d(-1, 0, 0);
      thrusterDirections["thruster_py"] = gz::math::Vector3d(0, 1, 0);
      thrusterDirections["thruster_ny"] = gz::math::Vector3d(0, -1, 0);
      thrusterDirections["thruster_pz"] = gz::math::Vector3d(0, 0, 1);
      thrusterDirections["thruster_nz"] = gz::math::Vector3d(0, 0, -1);

      // Get thruster positions from their poses
      for (const auto &name : this->thrusterNames)
      {
        auto it = this->thrusterLinks.find(name);
        if (it != this->thrusterLinks.end())
        {
          auto poseComp = ecm.Component<gz::sim::components::Pose>(it->second);
          if (poseComp)
            thrusterPositions[name] = poseComp->Data().Pos();
        }
      }

      // Simple allocation: try to match required force/torque
      gz::math::Vector3d totalForce(0, 0, 0);
      gz::math::Vector3d totalTorque(0, 0, 0);

      for (const auto &name : this->thrusterNames)
      {
        auto dirIt = thrusterDirections.find(name);
        auto posIt = thrusterPositions.find(name);
        
        if (dirIt != thrusterDirections.end() && posIt != thrusterPositions.end())
        {
          const auto &direction = dirIt->second;
          const auto &position = posIt->second;
          
          // Calculate how much this thruster contributes to required force/torque
          double forceContribution = direction.Dot(requiredForce);
          gz::math::Vector3d torqueContribution = (position - basePose.Pos()).Cross(direction * forceContribution);
          
          // Calculate desired thrust (with some heuristics)
          double desiredThrust = forceContribution * 0.5; // Scale down for stability
          
          // Add some torque contribution
          double torqueFactor = 0.1;
          if (name.find("px") != std::string::npos || name.find("nx") != std::string::npos)
            desiredThrust += torqueFactor * requiredTorque.Y();
          else if (name.find("py") != std::string::npos || name.find("ny") != std::string::npos)
            desiredThrust += torqueFactor * requiredTorque.X();
          else if (name.find("pz") != std::string::npos || name.find("nz") != std::string::npos)
            desiredThrust += torqueFactor * requiredTorque.Z();

          // Limit thrust
          desiredThrust = std::clamp(desiredThrust, -this->maxThrust, this->maxThrust);
          
          // Apply efficiency and fuel consumption
          double actualThrust = desiredThrust * this->thrustEfficiency;
          
          // Fuel consumption (simplified)
          double fuelConsumption = std::abs(actualThrust) / (this->isp * 9.81);
          this->fuelMass -= fuelConsumption * 0.001; // Scale down for simulation
          
          this->thrust[name] = actualThrust;
          
          totalForce += direction * actualThrust;
          totalTorque += (position - basePose.Pos()).Cross(direction * actualThrust);
        }
      }
    }

    private: void ApplyThrusterForces(gz::sim::EntityComponentManager &ecm, double dt)
    {
      // Apply forces to each thruster link
      for (const auto &kv : this->thrust)
      {
        const auto &name = kv.first;
        double thrustForce = kv.second;
        
        auto it = this->thrusterLinks.find(name);
        if (it != this->thrusterLinks.end())
        {
          auto linkEntity = it->second;
          
          // Get thruster pose
          auto poseComp = ecm.Component<gz::sim::components::Pose>(linkEntity);
          if (!poseComp) continue;
          
          const auto &thrusterPose = poseComp->Data();
          
          // Calculate thrust direction in world frame
          gz::math::Vector3d thrustDirection = thrusterPose.Rot().RotateVector(gz::math::Vector3d(0, 0, 1));
          
          // Apply force to the thruster link
          gz::math::Vector3d force = thrustDirection * thrustForce;
          gz::math::Vector3d torque(0, 0, 0); // No torque from thrusters themselves
          
          // Apply external wrench command
          gz::msgs::Wrench wrenchMsg;
          gz::msgs::Set(wrenchMsg.mutable_force(), force);
          gz::msgs::Set(wrenchMsg.mutable_torque(), torque);
          
          auto wrenchComp = ecm.Component<gz::sim::components::ExternalWorldWrenchCmd>(linkEntity);
          if (!wrenchComp)
            ecm.CreateComponent(linkEntity, gz::sim::components::ExternalWorldWrenchCmd(wrenchMsg));
          else
            wrenchComp->Data() = wrenchMsg;
        }
      }
    }

    private: gz::math::Vector3d CalculateAngularError(const gz::math::Quaterniond &current, const gz::math::Quaterniond &target)
    {
      gz::math::Quaterniond errorQuat = target * current.Inverse();
      gz::math::Vector3d axis;
      double angle;
      errorQuat.AxisAngle(axis, angle);
      return axis * angle;
    }

    private: void OnThrustTopic(const std::string &_topic, const double &_n)
    {
      auto pos = _topic.rfind('/');
      if (pos != std::string::npos)
      {
        auto name = _topic.substr(pos+1);
        this->thrust[name] = _n;
      }
    }

    private: gz::sim::Model model{gz::sim::kNullEntity};
    private: std::string modelName;
    private: std::string targetModelName;
    private: gz::transport::Node node;
    private: std::vector<std::string> thrusterNames;
    private: std::unordered_map<std::string, gz::sim::Entity> thrusterLinks;
    private: gz::sim::Entity baseLink{gz::sim::kNullEntity};
    private: std::unordered_map<std::string, double> thrust;
    
    // Realistic thruster parameters
    private: double maxThrust{5.0}; // Newtons
    private: double thrustEfficiency{0.95};
    private: double fuelMass{50.0}; // kg
    private: double isp{300.0}; // seconds
    private: double kpLinear{0.5};
    private: double kpAngular{1.0};
    private: double kdLinear{0.1};
    private: double kdAngular{0.2};
    
    // Random number generator
    private: std::mt19937 rng;
  };
}

// Register system plugin
GZ_ADD_PLUGIN(
  realistic_thruster_controller::Plugin,
  gz::sim::System,
  realistic_thruster_controller::Plugin::ISystemConfigure,
  realistic_thruster_controller::Plugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(realistic_thruster_controller::Plugin, "realistic_thruster_controller::Plugin")
