import math
import re
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

try:
    from nubot_interfaces.msg import ActionCmd, SendingOff, VelCmd
except ImportError:  # The interface package is introduced in a later migration step.
    ActionCmd = None
    SendingOff = None
    VelCmd = None


NO_ACTION = 21
TELEOP_JOY = 20
CATCH_POSITIONED = 11
CATCH_BALL = 16
CATCH_BALL_SLOW = 17


@dataclass
class ActionState:
    """保存最近一次上层 ActionCmd。

    旧 C++ 版本直接把 ActionCmd 中的字段存进一堆成员变量；这里集中到
    一个 dataclass，后续要替换控制算法时不需要到处追变量。
    """

    target_x: float = 0.0
    target_y: float = 0.0
    target_ori: float = 0.0
    target_vel_x: float = 0.0
    target_vel_y: float = 0.0
    maxvel: float = 0.0
    maxw: float = 0.0
    robot_x: float = 0.0
    robot_y: float = 0.0
    robot_ori: float = 0.0
    move_action: int = NO_ACTION
    rotate_action: int = NO_ACTION
    rotate_mode: int = 0
    handle_enable: bool = False
    strength: float = 0.0
    shoot_pos: int = 0


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle <= -math.pi:
        angle += 2.0 * math.pi
    return angle


def clamp(value: float, limit: float) -> float:
    if limit <= 0.0:
        return 0.0
    return max(-limit, min(limit, value))


def char_to_int(value) -> int:
    """兼容 ROS2 char 字段的不同生成形式。

    ROS1 的 char 在 C++ 里表现为小整数；ROS2 Python 迁移后可能是 int、
    bytes 或单字符 str。统一转成 int，便于和 core.hpp 里的动作枚举比较。
    """

    if isinstance(value, str):
        return ord(value) if value else 0
    if isinstance(value, bytes):
        return value[0] if value else 0
    return int(value)


def get_point_xy(point) -> tuple[float, float]:
    """读取 Point2d 字段。

    这里不用强绑定消息类型，是为了让本包在 nubot_interfaces 尚未生成时
    仍然可以通过 Twist 覆盖通道编译和测试。
    """

    return float(getattr(point, "x", 0.0)), float(getattr(point, "y", 0.0))


def get_first_existing_attr(obj, names: tuple[str, ...], default):
    """按多个候选名字读取字段。

    ROS1 消息迁移到 ROS2 时，经常会把驼峰或大写字段改成下划线字段。
    例如 shootPos 可能变成 shoot_pos，VelCmd.Vx 可能变成 VelCmd.vx。
    这里集中做兼容，节点主体逻辑就不用关心字段最终采用哪种命名。
    """

    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    return default


def set_first_existing_field(msg, names: tuple[str, ...], value: float) -> None:
    """给迁移后的 VelCmd 写字段。

    旧消息字段是 Vx/Vy/w；ROS2 迁移时如果按规范改成 vx/vy/w，这里也能
    自动适配，避免字段大小写成为迁移阻塞点。
    """

    for name in names:
        if hasattr(msg, name):
            setattr(msg, name, value)
            return


class HwControllerNode(Node):
    """nubot_hwcontroller 的 ROS2/Python 迁移版。

    旧 ROS1 节点的职责是：
    1. 接收 nubot_control 发布的 ActionCmd；
    2. 根据目标位置/目标朝向计算底盘速度 VelCmd；
    3. 把带球、射门、罚下等底层执行意图交给 Gazebo Classic 插件。

    Gazebo Harmonic 不能直接加载旧的 Gazebo Classic ModelPlugin，所以这里把
    “硬控层”和“仿真执行层”拆开：
    - 本节点负责 ActionCmd -> Twist/VelCmd/dribble/shoot；
    - Gazebo adapter 节点负责把 Twist/dribble/shoot 映射到 Harmonic 的服务或话题。
    """

    def __init__(self) -> None:
        super().__init__("nubot_hwcontroller_py")

        self.declare_parameter("robot_name", "nubot1")
        self.declare_parameter("team_prefix", "nubot")
        self.declare_parameter("action_topic", "/nubot1/nubotcontrol/actioncmd")
        self.declare_parameter("velcmd_topic", "/nubot1/nubotcontrol/velcmd")
        self.declare_parameter("cmd_vel_topic", "/nubot1/cmd_vel")
        self.declare_parameter("cmd_vel_override_topic", "/nubot1/hwcontroller/cmd_vel_override")
        self.declare_parameter("dribble_enable_topic", "/nubot1/dribble_enable")
        self.declare_parameter("shoot_power_topic", "/nubot1/shoot_power")
        self.declare_parameter("ball_holding_topic", "/nubot1/ball_is_holding")
        self.declare_parameter("redcard_topic", "/nubot/redcard/chatter")
        self.declare_parameter("motion_enabled_topic", "/nubot1/hwcontroller/enabled")
        self.declare_parameter("update_rate", 100.0)
        self.declare_parameter("action_timeout", 0.25)
        self.declare_parameter("position_gain", 5.0)
        self.declare_parameter("rotation_gain", 2.0)
        self.declare_parameter("linear_scale", 0.01)
        self.declare_parameter("max_linear_speed", 2.0)
        self.declare_parameter("max_angular_speed", 6.0)

        self.robot_name = self.get_parameter("robot_name").value
        self.team_prefix = self.get_parameter("team_prefix").value
        self.action_topic = self.get_parameter("action_topic").value
        self.velcmd_topic = self.get_parameter("velcmd_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.cmd_vel_override_topic = self.get_parameter("cmd_vel_override_topic").value
        self.dribble_enable_topic = self.get_parameter("dribble_enable_topic").value
        self.shoot_power_topic = self.get_parameter("shoot_power_topic").value
        self.ball_holding_topic = self.get_parameter("ball_holding_topic").value
        self.redcard_topic = self.get_parameter("redcard_topic").value
        self.motion_enabled_topic = self.get_parameter("motion_enabled_topic").value
        self.update_rate = float(self.get_parameter("update_rate").value)
        self.action_timeout = float(self.get_parameter("action_timeout").value)
        self.position_gain = float(self.get_parameter("position_gain").value)
        self.rotation_gain = float(self.get_parameter("rotation_gain").value)
        self.linear_scale = float(self.get_parameter("linear_scale").value)
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_angular_speed = float(self.get_parameter("max_angular_speed").value)

        self.robot_id = self.robot_id_from_name(self.robot_name)
        self.enabled = True
        self.action_state = ActionState()
        self.last_action_time = self.get_clock().now()
        self.override_twist: Optional[Twist] = None
        self.last_override_time = self.get_clock().now()
        self.ball_is_holding = False
        self.last_shoot_power = 0.0

        # /cmd_vel 是迁移后的主要底盘接口；当前可接 planar_motion_controller，
        # 后续可接 ros2_control 或 Gazebo Sim System 插件。
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        # 保留旧项目的 VelCmd 语义，便于 nubot_control_py/world_model_py 逐步迁移。
        self.enabled_pub = self.create_publisher(Bool, self.motion_enabled_topic, 10)
        self.dribble_pub = self.create_publisher(Bool, self.dribble_enable_topic, 10)
        self.shoot_pub = self.create_publisher(Float32, self.shoot_power_topic, 10)
        self.velcmd_pub = self.create_velcmd_publisher()

        if ActionCmd is not None:
            self.create_subscription(ActionCmd, self.action_topic, self.action_callback, 10)
        else:
            self.get_logger().warn(
                "nubot_interfaces is not available; ActionCmd input and VelCmd output are disabled"
            )

        if SendingOff is not None:
            self.create_subscription(SendingOff, self.redcard_topic, self.redcard_callback, 10)

        self.create_subscription(Twist, self.cmd_vel_override_topic, self.override_callback, 10)
        self.create_subscription(Bool, self.ball_holding_topic, self.ball_holding_callback, 10)

        period = 1.0 / max(self.update_rate, 1.0)
        self.create_timer(period, self.update)

        self.get_logger().info(
            "hwcontroller ready: robot=%s action=%s cmd_vel=%s velcmd=%s"
            % (self.robot_name, self.action_topic, self.cmd_vel_topic, self.velcmd_topic)
        )

    def create_velcmd_publisher(self):
        if VelCmd is None:
            return None
        return self.create_publisher(VelCmd, self.velcmd_topic, 10)

    def action_callback(self, msg) -> None:
        target_x, target_y = get_point_xy(msg.target)
        target_vel_x, target_vel_y = get_point_xy(msg.target_vel)
        robot_x, robot_y = get_point_xy(msg.robot_pos)

        rotate_action = get_first_existing_attr(msg, ("rotate_acton", "rotate_action"), NO_ACTION)
        handle_enable = get_first_existing_attr(msg, ("handle_enable",), 0)
        strength = get_first_existing_attr(msg, ("strength",), 0.0)
        shoot_pos = get_first_existing_attr(msg, ("shootPos", "shoot_pos"), 0)

        self.action_state = ActionState(
            target_x=target_x,
            target_y=target_y,
            target_ori=float(msg.target_ori),
            target_vel_x=target_vel_x,
            target_vel_y=target_vel_y,
            maxvel=float(msg.maxvel),
            maxw=float(msg.maxw),
            robot_x=robot_x,
            robot_y=robot_y,
            robot_ori=float(msg.robot_ori),
            move_action=char_to_int(msg.move_action),
            rotate_action=char_to_int(rotate_action),
            rotate_mode=int(msg.rotate_mode),
            handle_enable=bool(handle_enable),
            strength=float(strength),
            shoot_pos=int(shoot_pos),
        )
        self.last_action_time = self.get_clock().now()

    def redcard_callback(self, msg) -> None:
        team_name = str(getattr(msg, "team_name", getattr(msg, "TeamName", "")))
        flag = int(getattr(msg, "id_maxvel_isvalid", 0))
        if team_name and team_name != self.robot_name:
            return
        if flag == self.robot_id:
            self.enabled = False
            self.get_logger().info(f"{self.robot_name} disabled by red-card command")
        elif flag == self.robot_id + 10:
            self.enabled = True
            self.get_logger().info(f"{self.robot_name} re-enabled by red-card command")

    def override_callback(self, msg: Twist) -> None:
        """测试入口：不经过 ActionCmd，直接给硬控层塞 Twist。

        这个通道用于接口包尚未迁移完成时验证 Gazebo 运动闭环。正式策略链路
        应使用 action_topic。
        """

        self.override_twist = msg
        self.last_override_time = self.get_clock().now()

    def ball_holding_callback(self, msg: Bool) -> None:
        """接收带球状态。

        旧 Gazebo 插件发布 BallIsHolding；ROS2 迁移后由 dribble_controller 发布
        /nubotX/ball_is_holding。本节点用它做速度限制和射门有效性判断。
        """

        self.ball_is_holding = bool(msg.data)

    def update(self) -> None:
        now = self.get_clock().now()
        self.publish_enabled()
        self.publish_dribble_and_shoot(now)

        twist = self.pick_override_twist(now)
        if twist is None:
            twist = self.compute_twist_from_action(now)

        if not self.enabled:
            twist = Twist()

        self.cmd_vel_pub.publish(twist)
        self.publish_velcmd(twist)

    def pick_override_twist(self, now) -> Optional[Twist]:
        if self.override_twist is None:
            return None
        age = (now - self.last_override_time).nanoseconds * 1e-9
        if age > self.action_timeout:
            return None
        return self.limit_twist(self.override_twist)

    def compute_twist_from_action(self, now) -> Twist:
        age = (now - self.last_action_time).nanoseconds * 1e-9
        if age > self.action_timeout:
            return Twist()

        state = self.action_state
        vx_raw, vy_raw = self.compute_body_velocity(state)
        wz_raw = self.compute_angular_velocity(state)

        if state.move_action == NO_ACTION:
            vx_raw = 0.0
            vy_raw = 0.0
        if state.rotate_action == NO_ACTION:
            wz_raw = 0.0

        twist = Twist()
        twist.linear.x = vx_raw * self.linear_scale
        twist.linear.y = vy_raw * self.linear_scale
        twist.angular.z = wz_raw
        return self.limit_twist(twist)

    def compute_body_velocity(self, state: ActionState) -> tuple[float, float]:
        """根据目标点计算机器人本体坐标系速度。

        对应旧 C++ 的 move2target()：
        - target/robot_pos 用世界坐标；
        - 先算世界坐标下目标方向；
        - 再减去 robot_ori，得到机器人本体坐标系下的 Vx/Vy。
        """

        dx = state.target_x - state.robot_x
        dy = state.target_y - state.robot_y
        distance = math.hypot(dx, dy)
        if distance < 1e-6:
            return state.target_vel_x, state.target_vel_y

        target_theta = math.atan2(dy, dx)
        maxvel = max(0.0, state.maxvel)
        speed = clamp(self.position_gain * distance, maxvel)

        vx = speed * math.cos(target_theta - state.robot_ori) + state.target_vel_x
        vy = speed * math.sin(target_theta - state.robot_ori) + state.target_vel_y
        magnitude = math.hypot(vx, vy)
        if maxvel > 0.0 and magnitude > maxvel:
            vx *= maxvel / magnitude
            vy *= maxvel / magnitude
        # 旧代码中带球时会进一步限制线速度，避免高速带球导致丢球。
        if self.ball_is_holding and maxvel > 300.0:
            hold_limit = 300.0
            hold_magnitude = math.hypot(vx, vy)
            if hold_magnitude > hold_limit:
                vx *= hold_limit / hold_magnitude
                vy *= hold_limit / hold_magnitude
        return vx, vy

    def compute_angular_velocity(self, state: ActionState) -> float:
        """根据目标朝向计算角速度。

        对应旧 C++ 的 rotate2AbsOrienation()。rotate_mode 含义：
        - 0：走最短旋转方向；
        - 1：强制顺时针/正方向；
        - -1：强制逆时针/负方向。
        """

        error = state.target_ori - state.robot_ori
        if state.rotate_mode == -1:
            if error > 0.0:
                error -= 2.0 * math.pi
        elif state.rotate_mode == 1:
            if error < 0.0:
                error += 2.0 * math.pi
        else:
            error = normalize_angle(error)

        maxw = max(0.0, state.maxw)
        if state.rotate_action not in (CATCH_POSITIONED, CATCH_BALL, CATCH_BALL_SLOW, TELEOP_JOY):
            maxw = min(maxw, 3.0) if maxw > 0.0 else 3.0
        return clamp(self.rotation_gain * error, maxw)

    def limit_twist(self, twist: Twist) -> Twist:
        """最终安全限幅。

        ActionCmd 的 maxvel/maxw 是策略给出的限制；这里的 max_linear_speed/
        max_angular_speed 是硬控层统一安全上限，防止测试通道或异常输入过大。
        """

        limited = Twist()
        limited.linear.x = clamp(float(twist.linear.x), self.max_linear_speed)
        limited.linear.y = clamp(float(twist.linear.y), self.max_linear_speed)
        limited.angular.z = clamp(float(twist.angular.z), self.max_angular_speed)
        return limited

    def publish_velcmd(self, twist: Twist) -> None:
        """发布旧语义 VelCmd。

        这一路主要用于兼容后续还没迁移完的模块；真正驱动 Harmonic 的推荐
        通道是 /cmd_vel。
        """

        if self.velcmd_pub is None or VelCmd is None:
            return
        msg = VelCmd()
        vx = float(twist.linear.x / self.linear_scale) if self.linear_scale != 0.0 else 0.0
        vy = float(twist.linear.y / self.linear_scale) if self.linear_scale != 0.0 else 0.0
        set_first_existing_field(msg, ("vx", "Vx"), vx)
        set_first_existing_field(msg, ("vy", "Vy"), vy)
        set_first_existing_field(msg, ("w",), float(twist.angular.z))
        self.velcmd_pub.publish(msg)

    def publish_dribble_and_shoot(self, now) -> None:
        """发布带球和射门执行意图。

        旧 Gazebo Classic 插件直接订阅 ActionCmd，并在插件内部处理：
        - handle_enable -> dribble_req_
        - strength/shootPos -> shot_req_
        Harmonic 迁移后不再把这些逻辑放在旧插件里，而是拆成 ROS2 话题：
        - /nubotX/dribble_enable 交给 dribble_controller
        - /nubotX/shoot_power 预留给 kick/shoot adapter
        """

        action_fresh = (now - self.last_action_time).nanoseconds * 1e-9 <= self.action_timeout
        dribble_msg = Bool()
        dribble_msg.data = bool(self.enabled and action_fresh and self.action_state.handle_enable)
        self.dribble_pub.publish(dribble_msg)

        shoot_power = 0.0
        if self.enabled and action_fresh and self.ball_is_holding and self.action_state.strength > 0.0:
            shoot_power = float(self.action_state.strength)

        # 只在射门请求变化时发布非零值；下一周期发布 0，避免下游重复射门。
        shoot_msg = Float32()
        if shoot_power > 0.0 and self.last_shoot_power <= 0.0:
            shoot_msg.data = shoot_power
            self.last_shoot_power = shoot_power
        else:
            shoot_msg.data = 0.0
            self.last_shoot_power = 0.0 if shoot_power <= 0.0 else self.last_shoot_power
        self.shoot_pub.publish(shoot_msg)

    def publish_enabled(self) -> None:
        msg = Bool()
        msg.data = self.enabled
        self.enabled_pub.publish(msg)

    def robot_id_from_name(self, name: str) -> int:
        match = re.search(r"(\d+)$", name)
        return int(match.group(1)) if match else 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HwControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
