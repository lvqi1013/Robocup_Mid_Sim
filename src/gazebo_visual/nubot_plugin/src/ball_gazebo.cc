#include "ball_gazebo.hh"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>

namespace nubot_plugins
{

void BallGazebo::Configure(
  const gz::sim::Entity &_entity,
  const std::shared_ptr<const sdf::Element> &_sdf,
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::EventManager &)
{
  model_entity_ = _entity;
  model_ = gz::sim::Model(_entity);
  model_name_ = model_.Name(_ecm);

  mu_ = sdf_value<double>(_sdf, "mu", 0.5);
  gravity_ = sdf_value<double>(_sdf, "gravity", 9.8);
  stop_speed_ = sdf_value<double>(_sdf, "stop_speed", 0.05);
  max_speed_ = sdf_value<double>(_sdf, "max_speed", 10.0);

  cmd_vel_pub_ = gz_node_.Advertise<gz::msgs::Twist>("/model/" + model_name_ + "/cmd_vel");
  gz_node_.Subscribe("/model/" + model_name_ + "/nubot/kick_velocity", &BallGazebo::kick_velocity_cb, this);
}

void BallGazebo::PreUpdate(
  const gz::sim::UpdateInfo &_info,
  gz::sim::EntityComponentManager &)
{
  if (_info.paused) {
    return;
  }

  const double now = std::chrono::duration<double>(_info.simTime).count();
  double dt = 0.0;
  if (last_update_time_sec_ >= 0.0) {
    dt = std::max(0.0, now - last_update_time_sec_);
  }
  last_update_time_sec_ = now;

  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_) {
    return;
  }

  const double planar_speed = std::hypot(active_velocity_.X(), active_velocity_.Y());
  const double vertical_speed = std::fabs(active_velocity_.Z());
  if (planar_speed < stop_speed_ && vertical_speed < stop_speed_) {
    active_velocity_ = gz::math::Vector3d::Zero;
    active_ = false;
    publish_cmd_vel(active_velocity_);
    return;
  }

  publish_cmd_vel(active_velocity_);

  if (dt <= 0.0) {
    return;
  }

  if (planar_speed > 1e-9) {
    const double next_speed = std::max(0.0, planar_speed - mu_ * gravity_ * dt);
    const double scale = next_speed / planar_speed;
    active_velocity_.X() *= scale;
    active_velocity_.Y() *= scale;
  }

  if (std::fabs(active_velocity_.Z()) > 1e-9) {
    active_velocity_.Z() -= gravity_ * dt;
  }
}

void BallGazebo::kick_velocity_cb(const gz::msgs::Twist &_msg)
{
  gz::math::Vector3d velocity(
    _msg.linear().x(),
    _msg.linear().y(),
    _msg.linear().z());

  const double speed = velocity.Length();
  if (speed > max_speed_ && speed > 1e-9) {
    velocity *= max_speed_ / speed;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  active_velocity_ = velocity;
  active_ = active_velocity_.Length() >= stop_speed_;
}

void BallGazebo::publish_cmd_vel(const gz::math::Vector3d &_vel)
{
  gz::msgs::Twist cmd;
  cmd.mutable_linear()->set_x(_vel.X());
  cmd.mutable_linear()->set_y(_vel.Y());
  cmd.mutable_linear()->set_z(_vel.Z());
  cmd.mutable_angular()->set_x(0.0);
  cmd.mutable_angular()->set_y(0.0);
  cmd.mutable_angular()->set_z(0.0);
  cmd_vel_pub_.Publish(cmd);
}

template<typename T>
T BallGazebo::sdf_value(
  const std::shared_ptr<const sdf::Element> &_sdf,
  const std::string &_name,
  const T &_default_value) const
{
  if (_sdf && _sdf->HasElement(_name)) {
    return _sdf->Get<T>(_name);
  }
  return _default_value;
}

}  // namespace nubot_plugins

GZ_ADD_PLUGIN(
  nubot_plugins::BallGazebo,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(nubot_plugins::BallGazebo, "nubot_plugins::BallGazebo")
