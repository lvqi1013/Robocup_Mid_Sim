import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose, PoseArray, Quaternion, TransformStamped
from rclpy.node import Node
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose
from std_msgs.msg import Bool, String
from tf2_msgs.msg import TFMessage


@dataclass
class PlanarPose:
    x: float
    y: float
    z: float
    yaw: float
    orientation: Quaternion


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def parameter_as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class DribbleController(Node):
    """Emulates the original NuBot Gazebo dribble behavior in ROS 2.

    This node deliberately does not rely on physical trapping. When the ball is
    close to the robot's front and dribble is enabled, it teleports the ball to
    a stable point in front of the kicking direction every timer tick.
    """

    def __init__(self) -> None:
        super().__init__("dribble_controller")

        self.declare_parameter("world_name", "RoboCup15MSL")
        self.declare_parameter("robot_name", "nubot1")
        self.declare_parameter("ball_name", "football")
        self.declare_parameter("pose_topic", "/world/default/dynamic_pose/info")
        self.declare_parameter("robot_pose_topic", "/nubot1/pose")
        self.declare_parameter("ball_pose_topic", "/football/pose")
        self.declare_parameter("use_model_pose_topics", True)
        self.declare_parameter("model_pose_topic_type", "pose_array")
        self.declare_parameter("dribble_enable_topic", "/nubot1/dribble_enable")
        self.declare_parameter("holding_topic", "/nubot1/ball_is_holding")
        self.declare_parameter("debug_topic", "/nubot1/dribble_debug")
        self.declare_parameter("update_rate", 100.0)
        self.declare_parameter("dribble_offset", 0.32)
        self.declare_parameter("capture_distance", 0.47)
        self.declare_parameter("capture_angle_deg", 20.0)
        self.declare_parameter("ball_center_z", 0.11)
        self.declare_parameter("release_grace_seconds", 0.12)

        self.world_name = self.get_parameter("world_name").value
        self.robot_name = self.get_parameter("robot_name").value
        self.ball_name = self.get_parameter("ball_name").value
        self.pose_topic = self.get_parameter("pose_topic").value
        self.robot_pose_topic = self.get_parameter("robot_pose_topic").value
        self.ball_pose_topic = self.get_parameter("ball_pose_topic").value
        self.use_model_pose_topics = parameter_as_bool(self.get_parameter("use_model_pose_topics").value)
        self.model_pose_topic_type = str(self.get_parameter("model_pose_topic_type").value).lower()
        self.dribble_enable_topic = self.get_parameter("dribble_enable_topic").value
        self.holding_topic = self.get_parameter("holding_topic").value
        self.debug_topic = self.get_parameter("debug_topic").value
        update_rate = float(self.get_parameter("update_rate").value)

        self.dribble_offset = float(self.get_parameter("dribble_offset").value)
        self.capture_distance = float(self.get_parameter("capture_distance").value)
        self.capture_angle = math.radians(float(self.get_parameter("capture_angle_deg").value))
        self.ball_center_z = float(self.get_parameter("ball_center_z").value)
        self.release_grace_seconds = float(self.get_parameter("release_grace_seconds").value)

        self.robot_pose: Optional[PlanarPose] = None
        self.ball_pose: Optional[PlanarPose] = None
        self.dribble_enabled = False
        self.is_holding = False
        self.last_hold_time = self.get_clock().now()
        self.pending_request = None
        self.last_debug_text = ""

        service_name = f"/world/{self.world_name}/set_pose"
        self.set_pose_client = self.create_client(SetEntityPose, service_name)

        # 兼容两种 Gazebo -> ROS2 位姿输入：
        # 1. /world/<world>/dynamic_pose/info -> TFMessage，适合 Pose_V 桥接；
        # 2. /nubot1/pose 和 /football/pose -> PoseArray，适合你当前每个模型单独桥接的结构。
        if self.use_model_pose_topics:
            if self.model_pose_topic_type == "pose":
                self.create_subscription(Pose, self.robot_pose_topic, self.robot_pose_callback, 10)
                self.create_subscription(Pose, self.ball_pose_topic, self.ball_pose_callback, 10)
            else:
                self.create_subscription(PoseArray, self.robot_pose_topic, self.robot_pose_array_callback, 10)
                self.create_subscription(PoseArray, self.ball_pose_topic, self.ball_pose_array_callback, 10)
        else:
            self.create_subscription(TFMessage, self.pose_topic, self.pose_callback, 10)
        self.create_subscription(Bool, self.dribble_enable_topic, self.dribble_enable_callback, 10)
        self.holding_pub = self.create_publisher(Bool, self.holding_topic, 10)
        self.debug_pub = self.create_publisher(String, self.debug_topic, 10)

        period = 1.0 / max(update_rate, 1.0)
        self.create_timer(period, self.update)

        self.get_logger().info(
            "dribble_controller ready: robot=%s ball=%s pose_input=%s/%s set_pose=%s"
            % (
                self.robot_name,
                self.ball_name,
                self.robot_pose_topic if self.use_model_pose_topics else self.pose_topic,
                self.ball_pose_topic if self.use_model_pose_topics else self.pose_topic,
                service_name,
            )
        )

    def dribble_enable_callback(self, msg: Bool) -> None:
        self.dribble_enabled = bool(msg.data)
        if not self.dribble_enabled:
            self.is_holding = False
            self.publish_holding()

    def pose_callback(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            pose = self.to_planar_pose(transform)
            if self.entity_matches(transform, self.robot_name):
                self.robot_pose = pose
            elif self.entity_matches(transform, self.ball_name):
                self.ball_pose = pose

    def robot_pose_callback(self, msg: Pose) -> None:
        """接收单独桥接出来的机器人位姿。

        你的环境中有 /model/nubot1/pose -> /nubot1/pose，这通常会桥接成
        geometry_msgs/Pose。使用这个话题比等待 /world/.../dynamic_pose/info 更直接。
        """

        self.robot_pose = self.pose_msg_to_planar_pose(msg)

    def ball_pose_callback(self, msg: Pose) -> None:
        """接收单独桥接出来的足球位姿。

        你的环境中有 /model/football/pose -> /football/pose。
        """

        self.ball_pose = self.pose_msg_to_planar_pose(msg)

    def robot_pose_array_callback(self, msg: PoseArray) -> None:
        """接收 /nubot1/pose 这种 PoseArray 位姿桥接话题。

        你当前 ros2 topic type 显示 /nubot1/pose 是 geometry_msgs/msg/PoseArray。
        单模型 pose topic 通常只有一个 pose，所以这里取 poses[0]。
        """

        pose = self.first_pose_from_array(msg, self.robot_pose_topic)
        if pose is not None:
            self.robot_pose = self.pose_msg_to_planar_pose(pose)

    def ball_pose_array_callback(self, msg: PoseArray) -> None:
        """接收 /football/pose 这种 PoseArray 位姿桥接话题。"""

        pose = self.first_pose_from_array(msg, self.ball_pose_topic)
        if pose is not None:
            self.ball_pose = self.pose_msg_to_planar_pose(pose)

    def entity_name(self, transform: TransformStamped) -> str:
        name = transform.child_frame_id or transform.header.frame_id
        if "::" in name:
            return name.split("::", 1)[0]
        if "/" in name:
            return name.strip("/").split("/")[-1]
        return name

    def entity_matches(self, transform: TransformStamped, target_name: str) -> bool:
        """判断 Gazebo 动态位姿中的实体名是否匹配目标模型。

        不同 ros_gz_bridge 配置下，TFMessage 中的 frame 名可能是模型名、
        model::link、或 /world/default/model/<name>/link/<link>。如果只取最后一段，
        很容易把 nubot1::chassis 误判为 chassis，导致 robot_pose 一直为空。
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

    def to_planar_pose(self, transform: TransformStamped) -> PlanarPose:
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return PlanarPose(
            x=translation.x,
            y=translation.y,
            z=translation.z,
            yaw=yaw_from_quaternion(rotation),
            orientation=rotation,
        )

    def pose_msg_to_planar_pose(self, msg: Pose) -> PlanarPose:
        return PlanarPose(
            x=msg.position.x,
            y=msg.position.y,
            z=msg.position.z,
            yaw=yaw_from_quaternion(msg.orientation),
            orientation=msg.orientation,
        )

    def first_pose_from_array(self, msg: PoseArray, topic_name: str) -> Optional[Pose]:
        if msg.poses:
            return msg.poses[0]
        self.publish_debug(f"empty_pose_array topic={topic_name}")
        return None

    def update(self) -> None:
        if self.pending_request is not None and self.pending_request.done():
            try:
                response = self.pending_request.result()
                if response is not None and not response.success:
                    self.get_logger().warn("Gazebo rejected SetEntityPose for ball")
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warn(f"SetEntityPose call failed: {exc}")
            self.pending_request = None

        if self.robot_pose is None or self.ball_pose is None:
            self.is_holding = False
            self.publish_holding()
            self.publish_debug("waiting_pose")
            return

        in_capture_zone, distance, angle_error = self.ball_capture_state()
        can_capture = self.dribble_enabled and in_capture_zone
        if can_capture:
            self.is_holding = True
            self.last_hold_time = self.get_clock().now()
        elif self.is_holding:
            elapsed = (self.get_clock().now() - self.last_hold_time).nanoseconds * 1e-9
            if not self.dribble_enabled or elapsed > self.release_grace_seconds:
                self.is_holding = False

        if self.is_holding:
            self.place_ball_in_front()

        self.publish_holding()
        self.publish_debug(
            "enabled=%s holding=%s distance=%.3f angle_error_deg=%.1f in_zone=%s"
            % (
                self.dribble_enabled,
                self.is_holding,
                distance,
                math.degrees(angle_error),
                in_capture_zone,
            )
        )

    def ball_capture_state(self) -> tuple[bool, float, float]:
        """返回球是否位于捕获区，以及距离和角度误差。

        捕获区条件：
        1. 球到机器人中心的距离 <= capture_distance；
        2. 球在机器人朝向前方 capture_angle 范围内。
        debug_topic 会输出这两个数值，方便判断一直 False 的原因。
        """

        assert self.robot_pose is not None
        assert self.ball_pose is not None

        dx = self.ball_pose.x - self.robot_pose.x
        dy = self.ball_pose.y - self.robot_pose.y
        distance = math.hypot(dx, dy)
        ball_angle = math.atan2(dy, dx)
        angle_error = normalize_angle(ball_angle - self.robot_pose.yaw)
        in_zone = distance <= self.capture_distance and abs(angle_error) <= self.capture_angle
        return in_zone, distance, angle_error

    def place_ball_in_front(self) -> None:
        if self.pending_request is not None:
            return
        if not self.set_pose_client.service_is_ready():
            self.set_pose_client.wait_for_service(timeout_sec=0.0)
            if not self.set_pose_client.service_is_ready():
                self.get_logger().warn("Waiting for Gazebo set_pose service", throttle_duration_sec=2.0)
                return

        assert self.robot_pose is not None
        target = Pose()
        target.position.x = self.robot_pose.x + math.cos(self.robot_pose.yaw) * self.dribble_offset
        target.position.y = self.robot_pose.y + math.sin(self.robot_pose.yaw) * self.dribble_offset
        target.position.z = self.ball_center_z
        target.orientation.w = 1.0

        request = SetEntityPose.Request()
        request.entity = Entity()
        request.entity.name = self.ball_name
        request.entity.type = Entity.MODEL
        request.pose = target

        self.pending_request = self.set_pose_client.call_async(request)

    def publish_holding(self) -> None:
        msg = Bool()
        msg.data = self.is_holding
        self.holding_pub.publish(msg)

    def publish_debug(self, text: str) -> None:
        # 避免每个 100Hz tick 都重复刷同一条诊断信息。
        if text == self.last_debug_text:
            return
        self.last_debug_text = text
        msg = String()
        msg.data = text
        self.debug_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DribbleController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
