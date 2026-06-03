import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose, PoseArray, PoseStamped, Quaternion, Twist
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


@dataclass
class PlanarPose:
    x: float
    y: float
    z: float
    yaw: float


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    return q


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def parameter_as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class PlanarMotionController(Node):
    """Minimal ROS 2 to Gazebo motion loop for migration validation.

    This node intentionally uses Gazebo's SetEntityPose service instead of a
    wheel dynamics plugin. It gives the migration a deterministic first loop:
    ROS 2 velocity command in, Gazebo model pose change out, ROS 2 pose feedback
    back from the world dynamic pose topic.
    """

    def __init__(self) -> None:
        super().__init__("planar_motion_controller")

        self.declare_parameter("world_name", "RoboCup15MSL")
        self.declare_parameter("robot_name", "nubot1")
        self.declare_parameter("pose_topic", "/world/default/dynamic_pose/info")
        self.declare_parameter("robot_pose_topic", "/nubot1/pose")
        self.declare_parameter("use_model_pose_topics", True)
        self.declare_parameter("model_pose_topic_type", "pose_array")
        self.declare_parameter("cmd_vel_topic", "/nubot1/cmd_vel")
        # 不再默认发布到 /nubot1/pose。
        # 这个名字很容易被 ros_gz_bridge 用来桥接 Gazebo 原生位姿，若本节点也发布
        # PoseStamped，会造成同名 topic 多类型/多来源冲突，ros2 topic echo 也会变得不稳定。
        self.declare_parameter("pose_feedback_topic", "/nubot1/pose_feedback")
        self.declare_parameter("motion_active_topic", "/nubot1/motion_active")
        self.declare_parameter("update_rate", 50.0)
        self.declare_parameter("command_timeout", 0.25)
        self.declare_parameter("max_linear_speed", 2.0)
        self.declare_parameter("max_angular_speed", 6.0)
        self.declare_parameter("fixed_z", 0.01)

        self.world_name = self.get_parameter("world_name").value
        self.robot_name = self.get_parameter("robot_name").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.robot_pose_topic = self.get_parameter("robot_pose_topic").value
        self.use_model_pose_topics = parameter_as_bool(self.get_parameter("use_model_pose_topics").value)
        self.model_pose_topic_type = str(self.get_parameter("model_pose_topic_type").value).lower()
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.pose_feedback_topic = self.get_parameter("pose_feedback_topic").value
        self.motion_active_topic = self.get_parameter("motion_active_topic").value
        self.update_rate = float(self.get_parameter("update_rate").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self.fixed_z = float(self.get_parameter("fixed_z").value)

        self.pose: Optional[PlanarPose] = None
        self.command = Twist()
        self.last_command_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()
        self.pending_request = None

        service_name = f"/world/{self.world_name}/set_pose"
        self.set_pose_client = self.create_client(SetEntityPose, service_name)

        if self.use_model_pose_topics:
            if self.model_pose_topic_type == "pose":
                self.create_subscription(Pose, self.robot_pose_topic, self.robot_pose_callback, 10)
            else:
                self.create_subscription(PoseArray, self.robot_pose_topic, self.robot_pose_array_callback, 10)
        else:
            self.create_subscription(TFMessage, self.pose_topic, self.pose_callback, 10)
        self.create_subscription(Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, self.pose_feedback_topic, 10)
        self.active_pub = self.create_publisher(Bool, self.motion_active_topic, 10)

        period = 1.0 / max(self.update_rate, 1.0)
        self.create_timer(period, self.update)

        self.get_logger().info(
            "planar_motion_controller ready: robot=%s cmd=%s pose_input=%s set_pose=%s"
            % (
                self.robot_name,
                self.cmd_vel_topic,
                self.robot_pose_topic if self.use_model_pose_topics else self.pose_topic,
                service_name,
            )
        )

    def pose_callback(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            if not self.entity_matches(transform, self.robot_name):
                continue

            translation = transform.transform.translation
            rotation = transform.transform.rotation
            self.pose = PlanarPose(
                x=translation.x,
                y=translation.y,
                z=translation.z,
                yaw=yaw_from_quaternion(rotation),
            )
            self.publish_pose()
            break

    def robot_pose_callback(self, msg: Pose) -> None:
        """接收 /nubot1/pose 这种单模型位姿桥接话题。"""

        self.pose = PlanarPose(
            x=msg.position.x,
            y=msg.position.y,
            z=msg.position.z,
            yaw=yaw_from_quaternion(msg.orientation),
        )
        self.publish_pose()

    def robot_pose_array_callback(self, msg: PoseArray) -> None:
        """接收 /nubot1/pose 这种 PoseArray 位姿桥接话题。

        当前你的 bridge 将 /model/nubot1/pose 桥接成 geometry_msgs/msg/PoseArray。
        单模型 topic 一般只有一个 pose，这里取 poses[0] 作为机器人位姿。
        """

        if not msg.poses:
            return
        self.robot_pose_callback(msg.poses[0])

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.command = msg
        self.last_command_time = self.get_clock().now()

    def entity_name(self, transform) -> str:
        name = transform.child_frame_id or transform.header.frame_id
        if "::" in name:
            return name.split("::", 1)[0]
        if "/" in name:
            return name.strip("/").split("/")[-1]
        return name

    def entity_matches(self, transform, target_name: str) -> bool:
        """判断 Gazebo 动态位姿中的实体名是否匹配目标模型。

        ros_gz_bridge 从 Gazebo Pose_V 转成 TFMessage 时，frame 名可能出现多种形式：
        - nubot1
        - nubot1::chassis
        - /world/default/model/nubot1/link/chassis
        - default/nubot1
        旧实现只取最后一段，遇到 link/chassis 时会误判为 chassis。
        这里保守地检查多个候选形式，提高不同 bridge 配置下的兼容性。
        """

        raw_names = [transform.child_frame_id, transform.header.frame_id]
        for raw in raw_names:
            if not raw:
                continue
            stripped = raw.strip("/")
            parts = [part for part in stripped.replace("::", "/").split("/") if part]
            if raw == target_name or stripped == target_name:
                return True
            if target_name in parts:
                return True
            if raw.startswith(f"{target_name}::") or raw.endswith(f"/{target_name}"):
                return True
        return False

    def update(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds * 1e-9
        self.last_update_time = now

        if self.pending_request is not None and self.pending_request.done():
            try:
                response = self.pending_request.result()
                if response is not None and not response.success:
                    self.get_logger().warn("Gazebo rejected SetEntityPose for robot")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"SetEntityPose call failed: {exc}")
            self.pending_request = None

        active = self.command_is_fresh(now)
        self.publish_active(active)
        if self.pose is None or not active or dt <= 0.0:
            return

        if self.pending_request is not None:
            return
        if not self.set_pose_client.service_is_ready():
            self.set_pose_client.wait_for_service(timeout_sec=0.0)
            if not self.set_pose_client.service_is_ready():
                self.get_logger().warn("Waiting for Gazebo set_pose service", throttle_duration_sec=2.0)
                return

        vx = clamp(float(self.command.linear.x), self.max_linear_speed)
        vy = clamp(float(self.command.linear.y), self.max_linear_speed)
        wz = clamp(float(self.command.angular.z), self.max_angular_speed)

        cos_yaw = math.cos(self.pose.yaw)
        sin_yaw = math.sin(self.pose.yaw)
        world_vx = cos_yaw * vx - sin_yaw * vy
        world_vy = sin_yaw * vx + cos_yaw * vy

        next_pose = PlanarPose(
            x=self.pose.x + world_vx * dt,
            y=self.pose.y + world_vy * dt,
            z=self.fixed_z,
            yaw=normalize_angle(self.pose.yaw + wz * dt),
        )
        self.send_pose(next_pose)
        self.pose = next_pose
        self.publish_pose()

    def command_is_fresh(self, now) -> bool:
        age = (now - self.last_command_time).nanoseconds * 1e-9
        if age > self.command_timeout:
            return False
        return (
            abs(self.command.linear.x) > 1e-6
            or abs(self.command.linear.y) > 1e-6
            or abs(self.command.angular.z) > 1e-6
        )

    def send_pose(self, pose: PlanarPose) -> None:
        target = Pose()
        target.position.x = pose.x
        target.position.y = pose.y
        target.position.z = pose.z
        target.orientation = quaternion_from_yaw(pose.yaw)

        request = SetEntityPose.Request()
        request.entity = Entity()
        request.entity.name = self.robot_name
        request.entity.type = Entity.MODEL
        request.pose = target

        self.pending_request = self.set_pose_client.call_async(request)

    def publish_pose(self) -> None:
        if self.pose is None:
            return
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = self.pose.x
        msg.pose.position.y = self.pose.y
        msg.pose.position.z = self.pose.z
        msg.pose.orientation = quaternion_from_yaw(self.pose.yaw)
        self.pose_pub.publish(msg)

    def publish_active(self, active: bool) -> None:
        msg = Bool()
        msg.data = active
        self.active_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PlanarMotionController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
