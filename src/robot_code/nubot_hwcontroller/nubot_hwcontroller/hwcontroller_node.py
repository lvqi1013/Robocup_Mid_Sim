"""ROS 2 仿真硬件控制节点。

该节点兼容旧 ROS 1 nubot_hwcontroller 的核心职责：订阅 ActionCmd，
通过本地控制器计算 VelCmd，再发布给 Gazebo 插件执行底盘运动。
"""

import re
import time
from typing import Optional

from nubot_interfaces.msg import ActionCmd, SendingOff, VelCmd
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .controller import MotionController, MotionTarget, Velocity


def _robot_identity(robot_name: str) -> tuple[str, int]:
    """从机器人名称中解析队伍前缀和编号，例如 nubot3 -> ('nubot', 3)。"""
    match = re.fullmatch(r'(.*?)(\d+)', robot_name)
    if match is None:
        raise ValueError(
            f'robot_name must end in a numeric id, got {robot_name!r}'
        )
    return match.group(1), int(match.group(2))


class NubotHwController(Node):
    """将上层 ActionCmd 目标转换为 Gazebo 插件使用的 VelCmd。"""

    def __init__(self) -> None:
        """初始化 ROS 参数、控制器、发布/订阅和周期控制定时器。"""
        super().__init__('nubot_hwcontroller_node')

        # ROS 参数：
        # robot_name: 当前机器人名称/命名空间，例如 nubot1，必须以数字编号结尾。
        # team_prefix: 队伍前缀；为空时从 robot_name 去掉末尾编号推断。
        # team_info: 红牌消息中的队伍标识，False/True 分别对应旧工程两队。
        # control_period: 控制循环周期，单位秒，默认 0.005s 即 200Hz。
        # command_timeout: ActionCmd 超时时间；超时后发布零速度并重置控制器。
        # move_kp/move_kd: 平移距离误差 PD 增益。
        # rotation_kp/rotation_kd: 朝向误差 PD 增益。
        # linear_speed_limit: 节点级平移速度上限，单位通常为 cm/s。
        # angular_speed_limit: 节点级角速度上限，单位 rad/s。
        self.declare_parameter('robot_name', 'nubot1')
        self.declare_parameter('team_prefix', '')
        self.declare_parameter('team_info', False)
        self.declare_parameter('control_period', 0.005)
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('move_kp', 5.0)
        self.declare_parameter('move_kd', 0.0)
        self.declare_parameter('rotation_kp', 2.0)
        self.declare_parameter('rotation_kd', 0.0)
        self.declare_parameter('linear_speed_limit', 500.0)
        self.declare_parameter('angular_speed_limit', 6.0)

        self.robot_name = str(self.get_parameter('robot_name').value).strip('/')
        inferred_prefix, self.robot_id = _robot_identity(self.robot_name)
        configured_prefix = str(self.get_parameter('team_prefix').value)
        self.team_prefix = (configured_prefix or inferred_prefix).strip('/')
        self.team_info = bool(self.get_parameter('team_info').value)
        self.command_timeout = max(
            0.0, float(self.get_parameter('command_timeout').value)
        )

        self.controller = MotionController(
            move_kp=float(self.get_parameter('move_kp').value),
            move_kd=float(self.get_parameter('move_kd').value),
            rotation_kp=float(self.get_parameter('rotation_kp').value),
            rotation_kd=float(self.get_parameter('rotation_kd').value),
            linear_speed_limit=float(
                self.get_parameter('linear_speed_limit').value
            ),
            angular_speed_limit=float(
                self.get_parameter('angular_speed_limit').value
            ),
        )

        base_topic = f'/{self.robot_name}/nubotcontrol'
        # 输出：底盘速度命令，Gazebo 插件订阅后转换为仿真运动。
        self.velocity_publisher = self.create_publisher(
            VelCmd, f'{base_topic}/velcmd', 10
        )
        # 输入：上层控制/策略节点给出的目标位置、目标朝向和速度限制。
        self.action_subscription = self.create_subscription(
            ActionCmd,
            f'{base_topic}/actioncmd',
            self._on_action,
            10,
        )
        # 输入：队伍级红牌/恢复消息；匹配当前机器人时禁用或恢复速度输出。
        self.sending_off_subscription = self.create_subscription(
            SendingOff,
            f'/{self.team_prefix}/redcard/chatter',
            self._on_sending_off,
            10,
        )

        # 最近一次 ActionCmd 转换后的控制目标；None 表示尚未收到命令。
        self._latest_target: Optional[MotionTarget] = None
        # 最近一次 ActionCmd 到达时间，使用 monotonic 避免系统时间调整影响超时判断。
        self._last_command_time = 0.0
        # 红牌状态开关；False 时无论是否有命令都输出零速度。
        self._enabled = True
        # 防止命令超时后每个周期重复 reset 控制器。
        self._stopped_for_timeout = False

        period = max(
            0.001, float(self.get_parameter('control_period').value)
        )
        self.timer = self.create_timer(period, self._on_timer)
        self.get_logger().info(
            f'{self.robot_name}: ActionCmd -> VelCmd controller started; '
            f'period={period:.3f}s, timeout={self.command_timeout:.3f}s'
        )

    def _on_action(self, message: ActionCmd) -> None:
        """处理 ActionCmd，把消息字段转换为 MotionTarget 缓存。

        ActionCmd 中的位置/朝向是世界坐标语义，控制器会在定时器中转换为本体速度。
        收到新命令后刷新超时时间，并允许下一次超时时重新触发 reset。
        """
        self._latest_target = MotionTarget(
            target_x=float(message.target.x),
            target_y=float(message.target.y),
            target_vx=float(message.target_vel.x),
            target_vy=float(message.target_vel.y),
            target_orientation=float(message.target_ori),
            robot_x=float(message.robot_pos.x),
            robot_y=float(message.robot_pos.y),
            robot_orientation=float(message.robot_ori),
            max_velocity=float(message.maxvel),
            max_angular_velocity=float(message.maxw),
            move_action=int(message.move_action),
            rotate_action=int(message.rotate_action),
            rotate_mode=int(message.rotate_mode),
        )
        self._last_command_time = time.monotonic()
        self._stopped_for_timeout = False

    def _on_sending_off(self, message: SendingOff) -> None:
        """处理红牌/恢复上场消息。

        只处理 team_info 与当前节点匹配、且 player_num 为 0 或当前 robot_id 的消息。
        id_maxvel_isvalid 等于 robot_id 时禁用机器人；等于 robot_id + 10 时恢复机器人。
        """
        if bool(message.team_info) != self.team_info:
            return
        event_id = int(message.id_maxvel_isvalid)
        player_id = int(message.player_num)
        if player_id not in (0, self.robot_id):
            return

        if event_id == self.robot_id:
            self._enabled = False
            self.controller.reset()
            self._publish_velocity(Velocity())
            self.get_logger().warning(
                f'{self.robot_name} disabled by sending-off command'
            )
        elif event_id == self.robot_id + 10:
            self._enabled = True
            self.controller.reset()
            self.get_logger().info(
                f'{self.robot_name} enabled by return-to-field command'
            )

    def _on_timer(self) -> None:
        """周期控制回调。

        如果节点被红牌禁用，或 ActionCmd 尚未收到/已经超时，则发布零速度；
        否则使用 MotionController 根据最新目标计算并发布 VelCmd。
        """
        now = time.monotonic()
        timed_out = (
            self._latest_target is None
            or (
                self.command_timeout > 0.0
                and now - self._last_command_time > self.command_timeout
            )
        )
        if not self._enabled or timed_out:
            if timed_out and not self._stopped_for_timeout:
                self.controller.reset()
                self._stopped_for_timeout = True
            self._publish_velocity(Velocity())
            return

        velocity = self.controller.calculate(self._latest_target)
        self._publish_velocity(velocity)

    def _publish_velocity(self, velocity: Velocity) -> None:
        """把内部 Velocity 数据类转换为 VelCmd 消息并发布。"""
        message = VelCmd()
        message.vx = float(velocity.vx)
        message.vy = float(velocity.vy)
        message.w = float(velocity.w)
        self.velocity_publisher.publish(message)


def main(args=None) -> None:
    """ROS 2 入口函数：启动 NubotHwController 节点并处理正常退出。"""
    rclpy.init(args=args)
    node = None
    try:
        node = NubotHwController()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
