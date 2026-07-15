#ifndef NUBOT_PLUGIN_HH_
#define NUBOT_PLUGIN_HH_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

#include <rclcpp/rclcpp.hpp>

#include <nubot_interfaces/msg/action_cmd.hpp>
#include <nubot_interfaces/msg/ball_info.hpp>
#include <nubot_interfaces/msg/ball_is_holding.hpp>
#include <nubot_interfaces/msg/coach_info.hpp>
#include <nubot_interfaces/msg/obstacles_info.hpp>
#include <nubot_interfaces/msg/omini_vision_info.hpp>
#include <nubot_interfaces/msg/robot_info.hpp>
#include <nubot_interfaces/msg/sending_off.hpp>
#include <nubot_interfaces/msg/vel_cmd.hpp>
#include <nubot_interfaces/srv/dribble_id.hpp>

#include "core.hpp"
#include <gz/transport/Node.hh>
namespace nubot_plugins
{

enum NubotState
{
  CHASE_BALL,
  DRIBBLE_BALL,
  KICK_BALL,
  RESET
};

enum NubotSubState
{
  MOVE_BALL,
  ROTATE_BALL
};

struct SimPose
{
  gz::math::Vector3d position{0, 0, 0};
  gz::math::Quaterniond orient{1, 0, 0, 0};
};

struct SimTwist
{
  gz::math::Vector3d linear{0, 0, 0};
  gz::math::Vector3d angular{0, 0, 0};
};

struct SimModelState
{
  std::string model_name;
  SimPose pose;
  SimTwist twist;
};

struct Obstacles
{
  std::vector<nubot::PPoint> real_obs;
  std::vector<nubot::DPoint> world_obs;
};

class NubotGazebo: 
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
{
    public:
    NubotGazebo();
    ~NubotGazebo() override;

    void Configure(
        const gz::sim::Entity &_entity,
        const std::shared_ptr<const sdf::Element> &_sdf,
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::EventManager &_eventMgr) override;

    void PreUpdate(
        const gz::sim::UpdateInfo &_info,
        gz::sim::EntityComponentManager &_ecm) override;

    void PostUpdate(
        const gz::sim::UpdateInfo &_info,
        const gz::sim::EntityComponentManager &_ecm) override;

    private:
    void init_ros();
    void shutdown_ros();

    void vel_cmd_cb(const nubot_interfaces::msg::VelCmd::SharedPtr msg);
    void action_cmd_cb(const nubot_interfaces::msg::ActionCmd::SharedPtr msg);
    void coach_info_cb(const nubot_interfaces::msg::CoachInfo::SharedPtr msg);
    void sending_off_cb(const nubot_interfaces::msg::SendingOff::SharedPtr msg);

    void update_entities(const gz::sim::EntityComponentManager &_ecm);
    bool update_model_info(const gz::sim::EntityComponentManager &_ecm);
    void nubot_be_control(gz::sim::EntityComponentManager &_ecm);
    void nubot_locomotion(
        gz::sim::EntityComponentManager &_ecm,
        const gz::math::Vector3d &linear_vel_vector,
        const gz::math::Vector3d &angular_vel_vector);
    void dribble_ball(gz::sim::EntityComponentManager &_ecm);
    void kick_ball(gz::sim::EntityComponentManager &_ecm, int mode, double vel);
    void message_publish();

    bool get_is_hold_ball();
    bool get_nubot_stuck();
    bool is_robot_valid(double x, double y) const;
    void publish_cmd_vel_zero();

    double noise(double scale, double probability);
    double signed_angle_pi(gz::math::Vector3d reference, gz::math::Vector3d target) const;
    gz::math::Vector3d speed_limit(
        const gz::math::Vector3d &target_linear_vel,
        const gz::math::Vector3d &target_ang_vel) const;
    gz::math::Vector3d accelerate_limit(
        double duration,
        const gz::math::Vector3d &model_linear_vel,
        const gz::math::Vector3d &target_linear_vel,
        const gz::math::Vector3d &model_ang_vel,
        const gz::math::Vector3d &target_ang_vel) const;

    template<typename ComponentT, typename DataT>
    void set_or_create_component(
        gz::sim::EntityComponentManager &_ecm,
        gz::sim::Entity _entity,
        const DataT &_data);

    template<typename T>
    T sdf_value(
        const std::shared_ptr<const sdf::Element> &_sdf,
        const std::string &_name,
        const T &_default_value) const;

private:
  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Entity model_entity_{gz::sim::kNullEntity};
  gz::sim::Entity ball_entity_{gz::sim::kNullEntity};

  std::shared_ptr<rclcpp::Node> ros_node_;
  rclcpp::Context::SharedPtr ros_context_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
  std::atomic_bool spinning_{false};

  rclcpp::Subscription<nubot_interfaces::msg::VelCmd>::SharedPtr vel_cmd_sub_;
  rclcpp::Subscription<nubot_interfaces::msg::ActionCmd>::SharedPtr action_cmd_sub_;
  rclcpp::Subscription<nubot_interfaces::msg::CoachInfo>::SharedPtr coach_info_sub_;
  rclcpp::Subscription<nubot_interfaces::msg::SendingOff>::SharedPtr cyan_sendingoff_sub_;
  rclcpp::Subscription<nubot_interfaces::msg::SendingOff>::SharedPtr magenta_sendingoff_sub_;
  rclcpp::Publisher<nubot_interfaces::msg::OminiVisionInfo>::SharedPtr omni_vision_pub_;
  rclcpp::Publisher<nubot_interfaces::msg::BallIsHolding>::SharedPtr ball_is_holding_pub_;
  rclcpp::Client<nubot_interfaces::srv::DribbleId>::SharedPtr dribble_id_client_;

  mutable std::mutex mutex_;

  std::vector<SimModelState> model_states_;
  SimModelState robot_state_;
  SimModelState ball_state_;
  Obstacles obs_;

  nubot_interfaces::msg::BallInfo ball_info_;
  nubot_interfaces::msg::RobotInfo teammate_info_;
  nubot_interfaces::msg::ObstaclesInfo obstacles_info_;
  nubot_interfaces::msg::OminiVisionInfo omni_info_;
  nubot_interfaces::msg::BallIsHolding ball_is_holding_info_;

  gz::math::Vector3d desired_rot_vector_{0, 0, 0};
  gz::math::Vector3d desired_trans_vector_{0, 0, 0};
  gz::math::Vector3d nubot_ball_vec_{1, 0, 0};
  gz::math::Vector3d kick_vector_world_{1, 0, 0};

  std::string model_name_;
  std::string ball_name_{"football"};
  std::string cyan_pre_{"nubot"};
  std::string mag_pre_{"rival"};

  double nubot_ball_vec_len_{1.0};
  double dribble_distance_thres_{0.50};
  double dribble_angle_thres_{30.0};
  double vx_cmd_{0.0};
  double vy_cmd_{0.0};
  double w_cmd_{0.0};
  double force_{0.0};
  double field_length_{22.0};
  double field_width_{14.0};
  double angle_error_degree_{0.0};
  double noise_scale_{0.10};
  double noise_rate_{0.01};
  double last_command_time_sec_{0.0};

  int mode_{1};
  int agent_id_{0};
  int match_mode_{STARTROBOT};

  bool dribble_req_{false};
  bool is_dribble_{false};
  bool shot_req_{false};
  bool judge_nubot_stuck_{false};
  bool flip_cord_{false};
  bool can_move_{true};
  bool pending_robot_pose_{false};
  gz::math::Pose3d pending_robot_pose_value_{gz::math::Vector3d::Zero, gz::math::Quaterniond::Identity};

  std::mt19937 rng_;
  std::normal_distribution<double> gaussian_{0.0, 1.0};
  std::uniform_real_distribution<double> uniform_{0.0, 1.0};

  NubotState state_{CHASE_BALL};
  NubotSubState sub_state_{MOVE_BALL};

    gz::transport::Node gz_node_;
    gz::transport::Node::Publisher gz_cmd_vel_pub_;
    gz::transport::Node::Publisher gz_ball_kick_pub_;
};

}  // namespace nubot_plugins

#endif  // NUBOT_PLUGINS_NUBOT_GAZEBO_HH_
