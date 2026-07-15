#ifndef NUBOT_PLUGIN_BALL_GAZEBO_HH_
#define NUBOT_PLUGIN_BALL_GAZEBO_HH_

#include <memory>
#include <mutex>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace nubot_plugins
{

class BallGazebo:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  BallGazebo() = default;
  ~BallGazebo() override = default;

  void Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) override;

private:
  void kick_velocity_cb(const gz::msgs::Twist &_msg);
  void publish_cmd_vel(const gz::math::Vector3d &_vel);

  template<typename T>
  T sdf_value(
    const std::shared_ptr<const sdf::Element> &_sdf,
    const std::string &_name,
    const T &_default_value) const;

private:
  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Entity model_entity_{gz::sim::kNullEntity};

  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher cmd_vel_pub_;

  mutable std::mutex mutex_;
  gz::math::Vector3d active_velocity_{0, 0, 0};
  bool active_{false};
  double last_update_time_sec_{-1.0};

  std::string model_name_{"football"};
  double mu_{0.5};
  double gravity_{9.8};
  double stop_speed_{0.05};
  double max_speed_{10.0};
};

}  // namespace nubot_plugins

#endif  // NUBOT_PLUGIN_BALL_GAZEBO_HH_

