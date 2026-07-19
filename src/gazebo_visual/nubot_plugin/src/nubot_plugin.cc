#include "nubot_plugin.hh"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>

#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <builtin_interfaces/msg/time.hpp>
#include <gz/msgs/twist.pb.h>

namespace nubot_plugins
{
namespace
{
constexpr int RUN = 1;
constexpr int FLY = -1;
constexpr double PI = 3.14159265;
constexpr double CM2M_CONVERSION = 0.01;
constexpr double M2CM_CONVERSION = 100.0;
constexpr int SEEBALLBYOWN = 1;
constexpr double GOAL_X = 9.0;
constexpr double GOAL_HEIGHT = 1.0;
constexpr double MAX_ACC_LINEAR = 2.5;
constexpr double MAX_DCC_LINEAR = 5.0;
constexpr double SPEED_THRESH = 5.0;
constexpr double WHEEL_DISTANCE = 20.3 * CM2M_CONVERSION;
constexpr int WHEELS = 4;

std::mutex g_ros_init_mutex;
std::atomic<int> g_plugin_count{0};
rclcpp::Context::SharedPtr g_ros_context;
builtin_interfaces::msg::Time stamp_from_time(const rclcpp::Time &time)
{
  builtin_interfaces::msg::Time stamp;
  const int64_t nanoseconds = time.nanoseconds();
  stamp.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
  return stamp;
}
std::string scoped_topic(const std::string &ns, const std::string &topic)
{
  if (topic.empty() || topic.front() == '/') {
    return topic;
  }
  return "/" + ns + "/" + topic;
}

double clamp_unit(double value)
{
  return std::max(-1.0, std::min(1.0, value));
}
}  // namespace

NubotGazebo::NubotGazebo()
{
  std::random_device rd;
  rng_.seed(rd());
}

NubotGazebo::~NubotGazebo()
{
  shutdown_ros();
}

void NubotGazebo::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &)
{
    model_entity_ = _entity;
    model_ = gz::sim::Model(_entity);
    model_name_ = model_.Name(_ecm);
    gz_cmd_vel_pub_ = gz_node_.Advertise<gz::msgs::Twist>("/model/" + model_name_ + "/cmd_vel");

    ball_name_ = sdf_value<std::string>(_sdf, "ball_name", "football");
    cyan_pre_ = sdf_value<std::string>(_sdf, "cyan_prefix", "nubot");
    mag_pre_ = sdf_value<std::string>(_sdf, "magenta_prefix", "rival");
    dribble_distance_thres_ = sdf_value<double>(_sdf, "dribble_distance_thres", 0.50);
    dribble_angle_thres_ = sdf_value<double>(_sdf, "dribble_angle_thres", 30.0);
    field_length_ = sdf_value<double>(_sdf, "field_length", 22.0);
    field_width_ = sdf_value<double>(_sdf, "field_width", 14.0);
    noise_scale_ = sdf_value<double>(_sdf, "noise_scale", 0.10);
    noise_rate_ = sdf_value<double>(_sdf, "noise_rate", 0.01);
    flip_cord_ = sdf_value<bool>(_sdf, "flip_cord", false);
    gz_ball_kick_pub_ = gz_node_.Advertise<gz::msgs::Twist>("/model/" + ball_name_ + "/nubot/kick_velocity");
    const std::string &prefix = flip_cord_ ? mag_pre_ : cyan_pre_;
    if (model_name_.rfind(prefix, 0) == 0 && model_name_.size() > prefix.size()) {
        agent_id_ = std::atoi(model_name_.substr(prefix.size(), 1).c_str());
    }

    set_or_create_component<gz::sim::components::LinearVelocity>(
        _ecm, model_entity_, gz::math::Vector3d::Zero);
    set_or_create_component<gz::sim::components::AngularVelocity>(
        _ecm, model_entity_, gz::math::Vector3d::Zero);

    update_entities(_ecm);
    init_ros();

    RCLCPP_INFO(
        ros_node_->get_logger(),
        "%s configured: id=%d flip_cord=%d ball=%s noise=(%f,%f)",
        model_name_.c_str(), agent_id_, flip_cord_, ball_name_.c_str(), noise_scale_, noise_rate_);
}
void NubotGazebo::publish_cmd_vel_zero()
{
    gz::msgs::Twist cmd;

    cmd.mutable_linear()->set_x(0.0);
    cmd.mutable_linear()->set_y(0.0);
    cmd.mutable_linear()->set_z(0.0);

    cmd.mutable_angular()->set_x(0.0);
    cmd.mutable_angular()->set_y(0.0);
    cmd.mutable_angular()->set_z(0.0);

    gz_cmd_vel_pub_.Publish(cmd);
}

void NubotGazebo::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
    if (_info.paused) {
        return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    update_entities(_ecm);

    if (pending_robot_pose_) {
        set_or_create_component<gz::sim::components::Pose>(
        _ecm, model_entity_, pending_robot_pose_value_);
        set_or_create_component<gz::sim::components::LinearVelocity>(
        _ecm, model_entity_, gz::math::Vector3d::Zero);
        set_or_create_component<gz::sim::components::AngularVelocity>(
        _ecm, model_entity_, gz::math::Vector3d::Zero);
        pending_robot_pose_ = false;
    }


    const double now = ros_node_ ? ros_node_->now().seconds() : 0.0;
    const bool command_alive = last_command_time_sec_ > 0.0 && (now - last_command_time_sec_) < 0.3;

    if (command_alive) 
    {
        nubot_locomotion(_ecm, desired_trans_vector_, desired_rot_vector_);
    } else {
        desired_trans_vector_ = gz::math::Vector3d::Zero;
        desired_rot_vector_ = gz::math::Vector3d::Zero;
        publish_cmd_vel_zero();  // 或直接在这里发布 zero Twist
    }


    nubot_be_control(_ecm);
}

void NubotGazebo::PostUpdate(
  const gz::sim::UpdateInfo &,
  const gz::sim::EntityComponentManager &_ecm)
{
  std::lock_guard<std::mutex> lock(mutex_);
  update_entities(_ecm);
  if (update_model_info(_ecm)) {
    message_publish();
  }
}

void NubotGazebo::init_ros()
{
  {
    std::lock_guard<std::mutex> lock(g_ros_init_mutex);
    if (!g_ros_context || !g_ros_context->is_valid()) {
      int argc = 0;
      const char * const *argv = nullptr;
      g_ros_context = std::make_shared<rclcpp::Context>();
      g_ros_context->init(argc, argv);
    }
    ++g_plugin_count;
    ros_context_ = g_ros_context;
  }

  const std::string node_name = "nubot_gazebo_" + model_name_;
  rclcpp::NodeOptions node_options;
  node_options.context(ros_context_);
  node_options.use_intra_process_comms(false);
  ros_node_ = std::make_shared<rclcpp::Node>(
    node_name,
    node_options);

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = ros_context_;
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(executor_options);

  omni_vision_pub_ =
    ros_node_->create_publisher<nubot_interfaces::msg::OminiVisionInfo>(
      scoped_topic(model_name_, "omnivision/OmniVisionInfo"), 10);
  ball_is_holding_pub_ =
    ros_node_->create_publisher<nubot_interfaces::msg::BallIsHolding>(
      scoped_topic(model_name_, "ballisholding/BallIsHolding"), 10);

  vel_cmd_sub_ = ros_node_->create_subscription<nubot_interfaces::msg::VelCmd>(
    scoped_topic(model_name_, "nubotcontrol/velcmd"), 100,
    [this](const nubot_interfaces::msg::VelCmd::SharedPtr msg) { vel_cmd_cb(msg); });
  action_cmd_sub_ = ros_node_->create_subscription<nubot_interfaces::msg::ActionCmd>(
    scoped_topic(model_name_, "nubotcontrol/actioncmd"), 100,
    [this](const nubot_interfaces::msg::ActionCmd::SharedPtr msg) { action_cmd_cb(msg); });
  coach_info_sub_ = ros_node_->create_subscription<nubot_interfaces::msg::CoachInfo>(
    "/" + cyan_pre_ + "/receive_from_coach", 100,
    [this](const nubot_interfaces::msg::CoachInfo::SharedPtr msg) { coach_info_cb(msg); });
  cyan_sendingoff_sub_ = ros_node_->create_subscription<nubot_interfaces::msg::SendingOff>(
    "/" + cyan_pre_ + "/redcard/chatter", 100,
    [this](const nubot_interfaces::msg::SendingOff::SharedPtr msg) { sending_off_cb(msg); });
  magenta_sendingoff_sub_ = ros_node_->create_subscription<nubot_interfaces::msg::SendingOff>(
    "/" + mag_pre_ + "/redcard/chatter", 100,
    [this](const nubot_interfaces::msg::SendingOff::SharedPtr msg) { sending_off_cb(msg); });

  dribble_id_client_ =
    ros_node_->create_client<nubot_interfaces::srv::DribbleId>("/DribbleId");

  executor_->add_node(ros_node_);
  spinning_.store(true);
  spin_thread_ = std::thread([this]() {
    while (spinning_.load() && ros_context_ && ros_context_->is_valid()) {
      executor_->spin_some(std::chrono::milliseconds(10));
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  });
}

void NubotGazebo::shutdown_ros()
{
  spinning_.store(false);
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
  if (executor_ && ros_node_) {
    executor_->remove_node(ros_node_);
  }
  executor_.reset();
  vel_cmd_sub_.reset();
  action_cmd_sub_.reset();
  coach_info_sub_.reset();
  cyan_sendingoff_sub_.reset();
  magenta_sendingoff_sub_.reset();
  omni_vision_pub_.reset();
  ball_is_holding_pub_.reset();
  dribble_id_client_.reset();
  if (ros_node_) {
    ros_node_.reset();
  }

  std::lock_guard<std::mutex> lock(g_ros_init_mutex);
  if (ros_context_ && g_plugin_count > 0 && --g_plugin_count == 0) {
    if (g_ros_context && g_ros_context->is_valid()) {
      g_ros_context->shutdown("last nubot gazebo plugin unloaded");
    }
    g_ros_context.reset();
  }
  ros_context_.reset();
}

/**
 * @brief ROS2 速度指令回调函数：负责坐标系转换
 * 
 * 将上层决策/导航节点发送的 VelCmd（机器人本体坐标系，cm/s）
 * 转换为 Gazebo VelocityControl 插件所需的世界坐标系速度矢量（m/s），
 * 
 * @param msg 速度指令消息，包含 vx/vy (cm/s) 和 w (rad/s)
 */
void NubotGazebo::vel_cmd_cb(const nubot_interfaces::msg::VelCmd::SharedPtr msg)
{
    //加锁保护共享变量，因为此回调在 ROS2 executor 线程中执行，而 PreUpdate/nubot_locomotion 在 Gazebo 仿真线程中读取这些变量
    std::lock_guard<std::mutex> lock(mutex_);

    // ========== 1. 单位转换 + 坐标翻转 ==========
    // VelCmd 的 vx/vy 单位为 cm/s，需乘以 0.01 转为 m/s（Gazebo 使用 SI 单位）
    // flip_cord_ 为 true 时表示该机器人为对手（magenta），

    if (flip_cord_) {
        vx_cmd_ = -msg->vx * CM2M_CONVERSION;
        vy_cmd_ = -msg->vy * CM2M_CONVERSION;
    } else {
        vx_cmd_ = msg->vx * CM2M_CONVERSION;    // cm/s → m/s
        vy_cmd_ = msg->vy * CM2M_CONVERSION;    // cm/s → m/s
    }
    w_cmd_ = msg->w; // 角速度赋值


    // ========== 2. 获取机器人当前朝向（世界坐标系下的前方矢量）==========
    // kick_vector_world_ 是机器人本体 x 轴在世界坐标系中的方向，
    gz::math::Vector3d forward = kick_vector_world_;
    forward.Z() = 0.0;

    // 安全兜底：如果朝向矢量退化（如初始化时位姿尚未更新），默认使用世界坐标系 X 轴正方向作为前方
    if (forward.Length() < 1e-9) {
        forward = {1, 0, 0};
    } else {
        forward.Normalize();
    }

    // ========== 3. 构建本体坐标系的横向基矢量 ==========
    // 通过叉乘得到垂直于 forward 的横向矢量（本体 y 轴在世界系中的投影）
    // Z 轴 × 前方 = 左方（右手坐标系下，lateral 指向机器人左侧）

    const gz::math::Vector3d lateral = gz::math::Vector3d(0, 0, 1).Cross(forward);

    // ========== 4. 将本体速度分解到世界坐标系 ==========
    // VelCmd 中的 vx/vy 是相对于机器人本体的前后/左右速度，
    // 但 Gazebo 的 VelocityControl 插件期望的是世界坐标系下的绝对速度矢量
    // 因此需要用当前朝向基矢量做线性组合：
    //   世界速度 = vx * 前方单位矢量 + vy * 左方单位矢量
    desired_trans_vector_ = vx_cmd_ * forward + vy_cmd_ * lateral;

    // 角速度直接赋值到 Z 轴（平面旋转只有 Z 分量）
    desired_rot_vector_ = gz::math::Vector3d(0, 0, w_cmd_);

    // ========== 5. 记录最后有效指令时间戳 ==========
    last_command_time_sec_ = ros_node_ ? ros_node_->now().seconds() : 0.0;
}

void NubotGazebo::action_cmd_cb(const nubot_interfaces::msg::ActionCmd::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  dribble_req_ = msg->handle_enable != 0;
  force_ = static_cast<double>(msg->strength);
  mode_ = static_cast<int>(msg->shoot_pos);

  if (force_ != 0.0) {
    if (get_is_hold_ball()) {
      dribble_req_ = false;
      shot_req_ = true;
    } else {
      shot_req_ = false;
    }
  } else {
    shot_req_ = false;
  }
}

void NubotGazebo::coach_info_cb(const nubot_interfaces::msg::CoachInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  match_mode_ = msg->match_mode;
}

void NubotGazebo::sending_off_cb(const nubot_interfaces::msg::SendingOff::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if ((msg->team_info == false && !flip_cord_) || (msg->team_info == true && flip_cord_)) {
    if (msg->player_num != agent_id_) {
      return;
    }

    const double sign = flip_cord_ ? 1.0 : -1.0;
    double x = sign * (12.0 - 2.0 * msg->player_num);
    if (!flip_cord_) {
      x = -12.0 + 2.0 * msg->player_num;
    }
    double y = -8.5;
    if (msg->id_maxvel_isvalid == msg->player_num + 10) {
      y = -7.0;
    }

    pending_robot_pose_value_ =
      gz::math::Pose3d(gz::math::Vector3d(x, y, 0.0), gz::math::Quaterniond::Identity);
    pending_robot_pose_ = true;
  }
}

void NubotGazebo::update_entities(const gz::sim::EntityComponentManager &_ecm)
{
  if (ball_entity_ != gz::sim::kNullEntity) {
    return;
  }

  _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
    [this](const gz::sim::Entity &_entity,
    const gz::sim::components::Model *,
    const gz::sim::components::Name *_name) -> bool {
      if (_name->Data() == ball_name_) {
        ball_entity_ = _entity;
        return false;
      }
      return true;
    });
}

bool NubotGazebo::update_model_info(const gz::sim::EntityComponentManager &_ecm)
{
  if (ball_entity_ == gz::sim::kNullEntity || model_entity_ == gz::sim::kNullEntity) {
    return false;
  }

  model_states_.clear();
  _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
    [this, &_ecm](const gz::sim::Entity &_entity,
    const gz::sim::components::Model *,
    const gz::sim::components::Name *_name) -> bool {
      const std::string &name = _name->Data();
      const bool relevant =
        name.rfind(cyan_pre_, 0) == 0 ||
        name.rfind(mag_pre_, 0) == 0 ||
        name == ball_name_;
      if (!relevant) {
        return true;
      }

      SimModelState state;
      state.model_name = name;
      const auto pose = gz::sim::worldPose(_entity, _ecm);
      state.pose.position = pose.Pos();
      state.pose.position.X() += noise(noise_scale_, noise_rate_);
      state.pose.position.Y() += noise(noise_scale_, noise_rate_);
      state.pose.orient = pose.Rot();

      if (const auto linear = _ecm.Component<gz::sim::components::LinearVelocity>(_entity)) {
        state.twist.linear = linear->Data();
      }
      if (const auto angular = _ecm.Component<gz::sim::components::AngularVelocity>(_entity)) {
        state.twist.angular = angular->Data();
      }
      state.twist.linear.X() += noise(noise_scale_, noise_rate_);
      state.twist.linear.Y() += noise(noise_scale_, noise_rate_);

      if (flip_cord_) {
        state.pose.position.X() *= -1.0;
        state.pose.position.Y() *= -1.0;
        state.twist.linear.X() *= -1.0;
        state.twist.linear.Y() *= -1.0;
      }

      model_states_.push_back(state);
      return true;
    });

  bool has_robot = false;
  bool has_ball = false;
  for (const auto &state : model_states_) {
    if (state.model_name == ball_name_) {
      ball_state_ = state;
      has_ball = true;
    } else if (state.model_name == model_name_) {
      robot_state_ = state;
      has_robot = true;
    }
  }
  if (!has_robot || !has_ball) {
    return false;
  }

  nubot_ball_vec_ = ball_state_.pose.position - robot_state_.pose.position;
  nubot_ball_vec_len_ = nubot_ball_vec_.Length();
  kick_vector_world_ = robot_state_.pose.orient.RotateVector(gz::math::Vector3d(1, 0, 0));

  obs_.world_obs.clear();
  obs_.real_obs.clear();
  omni_info_.robotinfo.clear();

  for (const auto &state : model_states_) {
    const bool is_robot =
      state.model_name.rfind(cyan_pre_, 0) == 0 ||
      state.model_name.rfind(mag_pre_, 0) == 0;
    if (!is_robot) {
      continue;
    }

    if (state.model_name != model_name_) {
      const auto &obs_pos = state.pose.position;
      obs_.world_obs.emplace_back(obs_pos.X(), obs_pos.Y());
      const auto nubot_obs_vec = obs_pos - robot_state_.pose.position;
      obs_.real_obs.emplace_back(
        nubot::Angle(signed_angle_pi(kick_vector_world_, nubot_obs_vec)),
        nubot_obs_vec.Length());
    }

    const std::string team_pre = flip_cord_ ? mag_pre_ : cyan_pre_;
    if (state.model_name.rfind(team_pre, 0) == 0) {
      int robot_id = std::atoi(state.model_name.substr(team_pre.size(), 1).c_str());
      teammate_info_ = nubot_interfaces::msg::RobotInfo();
      teammate_info_.header.stamp = stamp_from_time(ros_node_->now());
      teammate_info_.agent_id = robot_id;
      teammate_info_.pos.x = state.pose.position.X() * M2CM_CONVERSION;
      teammate_info_.pos.y = state.pose.position.Y() * M2CM_CONVERSION;
      teammate_info_.heading.theta = state.pose.orient.Yaw();
      teammate_info_.vrot = state.twist.angular.Z();
      teammate_info_.vtrans.x = state.twist.linear.X() * M2CM_CONVERSION;
      teammate_info_.vtrans.y = state.twist.linear.Y() * M2CM_CONVERSION;
      teammate_info_.is_valid = is_robot_valid(state.pose.position.X(), state.pose.position.Y());
      teammate_info_.is_stuck = state.model_name == model_name_ ? get_nubot_stuck() : false;
      teammate_info_.is_dribble = state.model_name == model_name_ ? is_dribble_ : false;
      omni_info_.robotinfo.push_back(teammate_info_);
    }
  }

  return true;
}

void NubotGazebo::nubot_be_control(gz::sim::EntityComponentManager &_ecm)
{
    if (robot_state_.pose.position.Z() < 0.05) {
        can_move_ = true;
        if (dribble_req_ && get_is_hold_ball() && match_mode_ != STOPROBOT) {
        dribble_ball(_ecm);
        if (!is_dribble_ && dribble_id_client_ && dribble_id_client_->service_is_ready()) {
            auto request = std::make_shared<nubot_interfaces::srv::DribbleId::Request>();
            request->agent_id = flip_cord_ ? agent_id_ + 5 : agent_id_;
            dribble_id_client_->async_send_request(request);
            is_dribble_ = true;
        }
        } else if (is_dribble_) {
        if (dribble_id_client_ && dribble_id_client_->service_is_ready()) {
            auto request = std::make_shared<nubot_interfaces::srv::DribbleId::Request>();
            request->agent_id = -1;
            dribble_id_client_->async_send_request(request);
        }
        is_dribble_ = false;
        }

        if (shot_req_ && get_is_hold_ball() && match_mode_ != STOPROBOT) {
        kick_ball(_ecm, mode_, force_);
        shot_req_ = false;
        }
    } else {
        can_move_ = false;
    }
}

void NubotGazebo::nubot_locomotion(
  gz::sim::EntityComponentManager &,
  const gz::math::Vector3d &linear_vel_vector,
  const gz::math::Vector3d &angular_vel_vector)
{
    static std::vector<double> last_robot_time(10, 0.0);
    static std::vector<gz::math::Vector3d> last_robot_linear(10, gz::math::Vector3d::Zero);
    static std::vector<gz::math::Vector3d> last_robot_angular(10, gz::math::Vector3d::Zero);

    int index = agent_id_ - 1;
    if (model_name_.rfind(mag_pre_, 0) == 0) {
        index += 5;
    }
    if (index < 0 || index >= static_cast<int>(last_robot_time.size())) {
        index = 0;
    }

    const double now = ros_node_ ? ros_node_->now().seconds() : 0.0;
    const double duration = last_robot_time[index] > 0.0 ? now - last_robot_time[index] : 0.0;

    desired_trans_vector_ = linear_vel_vector;
    desired_rot_vector_ = angular_vel_vector;
    desired_trans_vector_.Z() = 0.0;
    desired_rot_vector_.X() = 0.0;
    desired_rot_vector_.Y() = 0.0;

    auto result = speed_limit(desired_trans_vector_, desired_rot_vector_);
    desired_rot_vector_ = result.Dot(gz::math::Vector3d(0, 0, 1)) * gz::math::Vector3d(0, 0, 1);
    desired_trans_vector_ = result - desired_rot_vector_;

    result = accelerate_limit(
        duration,
        last_robot_linear[index],
        desired_trans_vector_,
        last_robot_angular[index],
        desired_rot_vector_);
    desired_rot_vector_ = result.Dot(gz::math::Vector3d(0, 0, 1)) * gz::math::Vector3d(0, 0, 1);
    desired_trans_vector_ = result - desired_rot_vector_;

    result = speed_limit(desired_trans_vector_, desired_rot_vector_);
    desired_rot_vector_ = result.Dot(gz::math::Vector3d(0, 0, 1)) * gz::math::Vector3d(0, 0, 1);
    desired_trans_vector_ = result - desired_rot_vector_;

    gz::msgs::Twist cmd;
    cmd.mutable_linear()->set_x(desired_trans_vector_.X());
    cmd.mutable_linear()->set_y(desired_trans_vector_.Y());
    cmd.mutable_linear()->set_z(0.0);
    cmd.mutable_angular()->set_x(0.0);
    cmd.mutable_angular()->set_y(0.0);
    cmd.mutable_angular()->set_z(desired_rot_vector_.Z());

    gz_cmd_vel_pub_.Publish(cmd);

    judge_nubot_stuck_ = true;
    last_robot_linear[index] = desired_trans_vector_;
    last_robot_angular[index] = desired_rot_vector_;
    last_robot_time[index] = now;
}

void NubotGazebo::dribble_ball(gz::sim::EntityComponentManager &_ecm)
{
  if (ball_entity_ == gz::sim::kNullEntity) {
    return;
  }

  gz::math::Vector3d relative_pos = kick_vector_world_ * 0.35;
  gz::math::Vector3d target_pos =
    flip_cord_ ? -(robot_state_.pose.position + relative_pos) :
    robot_state_.pose.position + relative_pos;
  target_pos.Z() = 0.12;

  gz::math::Pose3d target_pose(target_pos, robot_state_.pose.orient);
  set_or_create_component<gz::sim::components::Pose>(_ecm, ball_entity_, target_pose);
  set_or_create_component<gz::sim::components::LinearVelocity>(
    _ecm, ball_entity_, gz::math::Vector3d::Zero);
  ball_state_.twist.linear = robot_state_.twist.linear;
}

void NubotGazebo::kick_ball(gz::sim::EntityComponentManager &, int mode, double vel)
{
  if (ball_entity_ == gz::sim::kNullEntity) {
    return;
  }

  gz::math::Vector3d kick_vector_planar(kick_vector_world_.X(), kick_vector_world_.Y(), 0.0);
  if (kick_vector_planar.Length() < 1e-9) {
    kick_vector_planar = gz::math::Vector3d(1, 0, 0);
  } else {
    kick_vector_planar.Normalize();
  }

  gz::math::Vector3d vel_vector{0, 0, 0};
  if (mode == RUN) {
    const double vel2 = std::min(vel, 10.0);
    vel_vector = (flip_cord_ ? -kick_vector_planar : kick_vector_planar) * vel2;
  } else if (mode == FLY) {
    const double vx = std::min(vel, 10.0);
    vel_vector = flip_cord_ ?
      gz::math::Vector3d(-0.8 * vx * kick_vector_planar.X(), -0.8 * vx * kick_vector_planar.Y(), 0.6 * vx) :
      gz::math::Vector3d(0.8 * vx * kick_vector_planar.X(), 0.8 * vx * kick_vector_planar.Y(), 0.6 * vx);
  } else {
    RCLCPP_ERROR(ros_node_->get_logger(), "%s kick_ball(): incorrect mode", model_name_.c_str());
    return;
  }

  gz::msgs::Twist cmd;
  cmd.mutable_linear()->set_x(vel_vector.X());
  cmd.mutable_linear()->set_y(vel_vector.Y());
  cmd.mutable_linear()->set_z(vel_vector.Z());
  cmd.mutable_angular()->set_x(0.0);
  cmd.mutable_angular()->set_y(0.0);
  cmd.mutable_angular()->set_z(0.0);

  gz_ball_kick_pub_.Publish(cmd);
}

void NubotGazebo::message_publish()
{
  if (!ros_node_) {
    return;
  }

  ball_info_ = nubot_interfaces::msg::BallInfo();
  ball_info_.header.stamp = stamp_from_time(ros_node_->now());
  ball_info_.ball_info_state = SEEBALLBYOWN;
  ball_info_.pos.x = ball_state_.pose.position.X() * M2CM_CONVERSION;
  ball_info_.pos.y = ball_state_.pose.position.Y() * M2CM_CONVERSION;
  ball_info_.real_pos.angle = signed_angle_pi(kick_vector_world_, nubot_ball_vec_);
  ball_info_.real_pos.radius = nubot_ball_vec_len_ * M2CM_CONVERSION;
  ball_info_.velocity.x = ball_state_.twist.linear.X() * M2CM_CONVERSION;
  ball_info_.velocity.y = ball_state_.twist.linear.Y() * M2CM_CONVERSION;
  ball_info_.pos_known = true;
  ball_info_.velocity_known = true;

  obstacles_info_ = nubot_interfaces::msg::ObstaclesInfo();
  obstacles_info_.header.stamp = stamp_from_time(ros_node_->now());
  for (std::size_t i = 0; i < obs_.real_obs.size(); ++i) {
    nubot_interfaces::msg::Point2D point;
    point.x = obs_.world_obs[i].x_ * M2CM_CONVERSION;
    point.y = obs_.world_obs[i].y_ * M2CM_CONVERSION;
    nubot_interfaces::msg::PPoint polar_point;
    polar_point.angle = obs_.real_obs[i].angle_.radian_;
    polar_point.radius = obs_.real_obs[i].radius_;
    obstacles_info_.pos.push_back(point);
    obstacles_info_.polar_pos.push_back(polar_point);
  }

  omni_info_.header.stamp = stamp_from_time(ros_node_->now());
  omni_info_.ballinfo = ball_info_;
  omni_info_.obstacleinfo = obstacles_info_;
  omni_vision_pub_->publish(omni_info_);

  ball_is_holding_info_.ball_is_holding = get_is_hold_ball();
  ball_is_holding_pub_->publish(ball_is_holding_info_);
}

bool NubotGazebo::get_is_hold_ball()
{
  gz::math::Vector3d norm = nubot_ball_vec_;
  norm.Z() = 0.0;
  if (norm.Length() < 1e-9) {
    angle_error_degree_ = 0.0;
    return nubot_ball_vec_len_ <= dribble_distance_thres_;
  }
  norm.Normalize();

  gz::math::Vector3d kick_vector = kick_vector_world_;
  kick_vector.Z() = 0.0;
  angle_error_degree_ = signed_angle_pi(kick_vector, norm) * (180.0 / PI);

  const bool aligned =
    angle_error_degree_ <= dribble_angle_thres_ / 2.0 &&
    angle_error_degree_ >= -dribble_angle_thres_ / 2.0;
  const bool near_ball = nubot_ball_vec_len_ <= dribble_distance_thres_;
  return near_ball && aligned;
}

bool NubotGazebo::get_nubot_stuck()
{
  static int time_count = 0;
  static bool last_time_stuck = false;
  static bool is_stuck = false;
  constexpr int time_limit = 40;
  constexpr double scale = 0.5;

  if (!judge_nubot_stuck_) {
    return false;
  }
  judge_nubot_stuck_ = false;

  const double desired_trans_length = desired_trans_vector_.Length();
  const double desired_rot_length = std::fabs(desired_rot_vector_.Z());
  const double actual_trans_length = robot_state_.twist.linear.Length();
  const double actual_rot_length = std::fabs(robot_state_.twist.angular.Z());

  if (actual_trans_length < desired_trans_length * scale ||
      actual_rot_length < desired_rot_length * scale) {
    time_count = last_time_stuck ? time_count + 1 : 0;
    last_time_stuck = true;
    if (time_count > time_limit) {
      time_count = 0;
      is_stuck = true;
    }
  } else {
    last_time_stuck = false;
    is_stuck = false;
  }
  return is_stuck;
}

bool NubotGazebo::is_robot_valid(double x, double y) const
{
  return !(std::fabs(x) > field_length_ / 2.0 + 1.0 ||
           std::fabs(y) > field_width_ / 2.0 + 1.0);
}

double NubotGazebo::noise(double scale, double probability)
{
  if (std::fabs(scale) < 1e-12) {
    return 0.0;
  }
  if (uniform_(rng_) <= probability) {
    return scale * gaussian_(rng_);
  }
  return 0.0;
}

double NubotGazebo::signed_angle_pi(gz::math::Vector3d reference, gz::math::Vector3d target) const
{
  if (reference.Length() < 1e-9 || target.Length() < 1e-9) {
    return 0.0;
  }
  reference.Normalize();
  target.Normalize();
  const double cos_angle = clamp_unit(reference.Dot(target));
  const double sin_angle = reference.Cross(target).Z();
  const double angle = std::acos(cos_angle);
  return sin_angle > 0.0 ? angle : -angle;
}

gz::math::Vector3d NubotGazebo::speed_limit(
  const gz::math::Vector3d &target_linear_vel,
  const gz::math::Vector3d &target_ang_vel) const
{
  double wheel_speed[WHEELS];
  const double target_vx = target_linear_vel.X();
  const double target_vy = target_linear_vel.Y();
  const double target_w = target_ang_vel.Z();

  wheel_speed[0] = 0.707 * (target_vx - target_vy) - target_w * WHEEL_DISTANCE;
  wheel_speed[1] = 0.707 * (target_vx + target_vy) - target_w * WHEEL_DISTANCE;
  wheel_speed[2] = 0.707 * (-target_vx + target_vy) - target_w * WHEEL_DISTANCE;
  wheel_speed[3] = 0.707 * (-target_vx - target_vy) - target_w * WHEEL_DISTANCE;

  double speed_thresh_ratio = 1.0;
  for (double &speed : wheel_speed) {
    speed_thresh_ratio = std::max(speed_thresh_ratio, std::fabs(speed) / SPEED_THRESH);
  }
  if (speed_thresh_ratio > 1.0) {
    for (double &speed : wheel_speed) {
      speed /= speed_thresh_ratio;
    }
  }

  const double w = -(wheel_speed[0] + wheel_speed[1] + wheel_speed[2] + wheel_speed[3]) /
    (4.0 * WHEEL_DISTANCE);
  const double vx = (wheel_speed[0] + wheel_speed[1] - wheel_speed[2] - wheel_speed[3]) /
    (2.0 * 1.414);
  const double vy = (wheel_speed[1] + wheel_speed[2] - wheel_speed[0] - wheel_speed[3]) /
    (2.0 * 1.414);
  return {vx, vy, w};
}

gz::math::Vector3d NubotGazebo::accelerate_limit(
  double duration,
  const gz::math::Vector3d &model_linear_vel,
  const gz::math::Vector3d &target_linear_vel,
  const gz::math::Vector3d &model_ang_vel,
  const gz::math::Vector3d &target_ang_vel) const
{
  if (std::fabs(duration) <= 1e-6) {
    return {model_linear_vel.X(), model_linear_vel.Y(), model_ang_vel.Z()};
  }

  double wheel_speed_old[WHEELS];
  double wheel_speed[WHEELS];
  double wheel_acc[WHEELS];

  const double target_vx = target_linear_vel.X();
  const double target_vy = target_linear_vel.Y();
  const double target_w = target_ang_vel.Z();
  const double model_vx = model_linear_vel.X();
  const double model_vy = model_linear_vel.Y();
  const double model_w = model_ang_vel.Z();

  wheel_speed[0] = 0.707 * (target_vx - target_vy) - target_w * WHEEL_DISTANCE;
  wheel_speed[1] = 0.707 * (target_vx + target_vy) - target_w * WHEEL_DISTANCE;
  wheel_speed[2] = 0.707 * (-target_vx + target_vy) - target_w * WHEEL_DISTANCE;
  wheel_speed[3] = 0.707 * (-target_vx - target_vy) - target_w * WHEEL_DISTANCE;
  wheel_speed_old[0] = 0.707 * (model_vx - model_vy) - model_w * WHEEL_DISTANCE;
  wheel_speed_old[1] = 0.707 * (model_vx + model_vy) - model_w * WHEEL_DISTANCE;
  wheel_speed_old[2] = 0.707 * (-model_vx + model_vy) - model_w * WHEEL_DISTANCE;
  wheel_speed_old[3] = 0.707 * (-model_vx - model_vy) - model_w * WHEEL_DISTANCE;

  double acc_thresh_ratio = 1.0;
  for (int i = 0; i < WHEELS; ++i) {
    wheel_acc[i] = (wheel_speed[i] - wheel_speed_old[i]) / duration;
    const double thresh = wheel_acc[i] * wheel_speed_old[i] >= 0.0 ?
      MAX_ACC_LINEAR : MAX_DCC_LINEAR;
    acc_thresh_ratio = std::max(acc_thresh_ratio, std::fabs(wheel_acc[i]) / thresh);
  }

  if (acc_thresh_ratio > 1.0) {
    for (int i = 0; i < WHEELS; ++i) {
      wheel_acc[i] /= acc_thresh_ratio;
      wheel_speed[i] = wheel_speed_old[i] + wheel_acc[i] * duration;
    }
  }

  const double w = -(wheel_speed[0] + wheel_speed[1] + wheel_speed[2] + wheel_speed[3]) /
    (4.0 * WHEEL_DISTANCE);
  const double vx = (wheel_speed[0] + wheel_speed[1] - wheel_speed[2] - wheel_speed[3]) /
    (2.0 * 1.414);
  const double vy = (wheel_speed[1] + wheel_speed[2] - wheel_speed[0] - wheel_speed[3]) /
    (2.0 * 1.414);
  return {vx, vy, w};
}

template<typename ComponentT, typename DataT>
void NubotGazebo::set_or_create_component(
  gz::sim::EntityComponentManager &_ecm,
  gz::sim::Entity _entity,
  const DataT &_data)
{
  if (_entity == gz::sim::kNullEntity) {
    return;
  }
  if (auto comp = _ecm.Component<ComponentT>(_entity)) {
    comp->SetData(_data, [](const DataT &, const DataT &) { return false; });
  } else {
    _ecm.CreateComponent(_entity, ComponentT(_data));
  }
}

template<typename T>
T NubotGazebo::sdf_value(
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
  nubot_plugins::NubotGazebo,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate,
  gz::sim::ISystemPostUpdate)

GZ_ADD_PLUGIN_ALIAS(nubot_plugins::NubotGazebo, "nubot_plugins::NubotGazebo")
