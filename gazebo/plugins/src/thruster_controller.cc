#include <gz/sim/Model.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/WorldLinearVelocityCmd.hh>
#include <gz/sim/components/WorldAngularVelocityCmd.hh>
#include <gz/sim/components/WorldPose.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <gz/math/Vector3.hh>
#include <gz/math/Rand.hh>
#include <gz/plugin/Register.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/boolean.pb.h>

#include <unordered_map>
#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <chrono>

// Simple message: double thrust Newtons
// Topic: /model/<model_name>/thrusters/<thruster_link_name>

namespace thruster_controller
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

      // Read thruster names from SDF list or default to known link names
      if (sdf && sdf->HasElement("thrusters"))
      {
        auto thrEl = sdf->GetElement("thrusters");
        auto el = thrEl->GetFirstElement();
        while (el)
        {
          if (el->GetName() == "thruster")
          {
            auto name = el->Get<std::string>("name");
            if (!name.empty())
              this->thrusterNames.push_back(name);
          }
          el = el->GetNextElement();
        }
      }
      if (this->thrusterNames.empty())
      {
        this->thrusterNames = {
          "thruster_px","thruster_nx","thruster_py",
          "thruster_ny","thruster_pz","thruster_nz"
        };
      }

      // Resolve thruster link entities
      for (const auto &name : this->thrusterNames)
      {
        auto link = this->model.LinkByName(ecm, name);
        if (link)
        {
          this->thrusterLinks[name] = link.Entity();
          // Subscribe to per-thruster topic for thrust command in Newtons (gz.msgs.Double)
          std::string topic = "/model/" + this->modelName + "/thrusters/" + name;
          this->node.Subscribe(topic, [this, topic](const gz::msgs::Double &_msg)
          {
            this->OnThrustTopic(topic, _msg.data());
          });
        }
      }

      // Optional: global command to zero all thrusters
      std::string zeroTopic = "/model/" + this->modelName + "/thrusters/zero";
      this->node.Subscribe(zeroTopic, [this](const gz::msgs::Boolean &_msg)
      {
        if (_msg.data()) this->thrust.clear();
      });

      // Cache base link
      auto base = this->model.LinkByName(ecm, "base_link");
      if (base)
        this->baseLink = base.Entity();

      // Random walk config
      if (sdf)
      {
        if (sdf->HasElement("random_walk"))
          this->randomWalk = sdf->Get<bool>("random_walk");
        if (sdf->HasElement("rate"))
          this->rateHz = sdf->Get<double>("rate");
        if (sdf->HasElement("maxN"))
          this->maxN = sdf->Get<double>("maxN");
        if (sdf->HasElement("step"))
          this->step = sdf->Get<double>("step");
      }
    }

    public: void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
    {
      // Optional internal random-walk generator
      if (this->randomWalk)
      {
        double now = std::chrono::duration<double>(info.simTime).count();
        if (now - this->lastUpdate >= (1.0 / std::max(1e-3, this->rateHz)))
        {
          this->lastUpdate = now;
          for (const auto &name : this->thrusterNames)
          {
            double cur = this->thrust[name];
            double delta = gz::math::Rand::DblUniform(-this->step, this->step);
            cur = std::clamp(cur + delta, -this->maxN, this->maxN);
            this->thrust[name] = cur;
          }
        }
      }

      if (this->baseLink == gz::sim::kNullEntity)
        return;

      auto basePoseComp = ecm.Component<gz::sim::components::WorldPose>(this->baseLink);
      if (!basePoseComp)
        return;
      const auto &basePose = basePoseComp->Data();

      gz::math::Vector3d F(0,0,0);
      gz::math::Vector3d Tau(0,0,0);

      // Sum forces and torques about base link origin
      for (auto &kv : this->thrust)
      {
        const auto &name = kv.first;
        double mag = kv.second; // Newtons
        auto itLink = this->thrusterLinks.find(name);
        if (itLink == this->thrusterLinks.end()) continue;
        auto linkEntity = itLink->second;

        auto poseComp = ecm.Component<gz::sim::components::WorldPose>(linkEntity);
        if (!poseComp) continue;
        const auto &pose = poseComp->Data();

        gz::math::Vector3d dir = pose.Rot().RotateVector(gz::math::Vector3d(0,0,1));
        gz::math::Vector3d force = dir * mag;
        F += force;

        // lever arm from base to thruster
        gz::math::Vector3d r = pose.Pos() - basePose.Pos();
        Tau += r.Cross(force);
      }

      // Map force/torque to simple velocity commands for visible motion (non-physical)
      // Tune gains as needed for your training visualization.
      const double kV = 0.002;  // m/s per Newton
      const double kW = 0.002;  // rad/s per N*m

      auto vComp = ecm.Component<gz::sim::components::WorldLinearVelocityCmd>(this->baseLink);
      if (!vComp)
        ecm.CreateComponent(this->baseLink, gz::sim::components::WorldLinearVelocityCmd(F * kV));
      else
        vComp->Data() = F * kV;

      auto wComp = ecm.Component<gz::sim::components::WorldAngularVelocityCmd>(this->baseLink);
      if (!wComp)
        ecm.CreateComponent(this->baseLink, gz::sim::components::WorldAngularVelocityCmd(Tau * kW));
      else
        wComp->Data() = Tau * kW;
    }

    private: void OnThrustTopic(const std::string &_topic, const double &_n)
    {
      // Extract thruster name from topic suffix
      auto pos = _topic.rfind('/');
      if (pos != std::string::npos)
      {
        auto name = _topic.substr(pos+1);
        this->thrust[name] = _n;
      }
    }

    private: gz::sim::Model model{gz::sim::kNullEntity};
    private: std::string modelName;
    private: gz::transport::Node node;
    private: std::vector<std::string> thrusterNames;
    private: std::unordered_map<std::string, gz::sim::Entity> thrusterLinks;
    private: gz::sim::Entity baseLink{gz::sim::kNullEntity};
    private: std::unordered_map<std::string, double> thrust; // N
    // Random-walk parameters
    private: bool randomWalk{false};
    private: double rateHz{10.0};
    private: double maxN{2.0};
    private: double step{0.2};
    private: double lastUpdate{0.0};
  };
}

// Register system plugin
GZ_ADD_PLUGIN(
  thruster_controller::Plugin,
  gz::sim::System,
  thruster_controller::Plugin::ISystemConfigure,
  thruster_controller::Plugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(thruster_controller::Plugin, "thruster_controller::Plugin")
