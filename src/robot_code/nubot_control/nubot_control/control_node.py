"""ROS2 nubot_control 节点：世界模型到 ActionCmd 的适配入口."""

import re
import time
from typing import Optional

from nubot_interfaces.msg import (
    ActionCmd,
    BallIsHolding,
    StrategyInfo,
    WorldModelInfo,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .constants import Action, MatchMode, TEAM_SIZE
from .geometry import Point
from .models import BallState, RobotState, WorldState
from .strategy import SimpleMatchStrategy


def robot_identity(robot_name: str) -> tuple[str, int]:
    """从 ``nubot3`` 一类名称解析队伍前缀和机器人编号."""
    match = re.fullmatch(r'(.*?)(\d+)', robot_name.strip('/'))
    if match is None:
        raise ValueError(f'robot_name must end with a number: {robot_name!r}')
    return match.group(1), int(match.group(2))


class NubotControlNode(Node):
    """为单台机器人运行现有简单比赛策略."""

    def __init__(self) -> None:
        super().__init__('nubot_control_node')
        self.declare_parameter('robot_name', 'nubot1')
        self.declare_parameter('team_prefix', '')
        self.declare_parameter('team_size', TEAM_SIZE)
        self.declare_parameter('control_period', 0.015)
        self.declare_parameter('world_model_timeout', 0.25)

        self.robot_name = str(self.get_parameter('robot_name').value).strip('/')
        inferred_prefix, self.agent_id = robot_identity(self.robot_name)
        configured_prefix = str(self.get_parameter('team_prefix').value)
        self.team_prefix = (configured_prefix or inferred_prefix).strip('/')
        self.team_size = int(self.get_parameter('team_size').value)
        self.world_timeout = float(
            self.get_parameter('world_model_timeout').value
        )

        self.world = WorldState(agent_id=self.agent_id)
        self.strategy = SimpleMatchStrategy()
        self.ball_holding = False
        self.last_world_update = 0.0
        self.latest_pass_command = None

        base = f'/{self.robot_name}'
        self.action_publisher = self.create_publisher(
            ActionCmd, f'{base}/nubotcontrol/actioncmd', 10
        )
        self.strategy_publisher = self.create_publisher(
            StrategyInfo, f'{base}/nubotcontrol/strategy', 10
        )
        self.create_subscription(
            WorldModelInfo,
            f'{base}/worldmodel/worldmodelinfo',
            self._on_world_model,
            10,
        )
        self.create_subscription(
            BallIsHolding,
            f'{base}/ballisholding/BallIsHolding',
            self._on_ball_holding,
            10,
        )

        period = max(0.005, float(self.get_parameter('control_period').value))
        self.create_timer(period, self._control_loop)
        self.get_logger().info(
            f'nubot_control started for {self.robot_name}: '
            f'agent_id={self.agent_id}, period={period:.3f}s'
        )

    def _on_world_model(self, message: WorldModelInfo) -> None:
        robots = {}
        for item in message.robotinfo:
            if 1 <= int(item.agent_id) <= self.team_size:
                robots[int(item.agent_id)] = RobotState(
                    agent_id=int(item.agent_id),
                    position=Point(float(item.pos.x), float(item.pos.y)),
                    heading=float(item.heading.theta),
                    velocity=Point(
                        float(item.vtrans.x), float(item.vtrans.y)
                    ),
                    angular_velocity=float(item.vrot),
                    valid=bool(item.is_valid),
                    stuck=bool(item.is_stuck),
                    dribbling=bool(item.is_dribble),
                    current_role=int(item.current_role),
                )
        for agent_id in range(1, self.team_size + 1):
            robots.setdefault(agent_id, RobotState(agent_id))

        ball = BallState()
        ball_index = self.agent_id - 1
        if 0 <= ball_index < len(message.ballinfo):
            item = message.ballinfo[ball_index]
            ball = BallState(
                position=Point(float(item.pos.x), float(item.pos.y)),
                velocity=Point(
                    float(item.velocity.x), float(item.velocity.y)
                ),
                known=bool(item.pos_known),
            )

        self.world = WorldState(
            agent_id=self.agent_id,
            robots=robots,
            ball=ball,
            match_mode=int(message.coachinfo.match_mode),
            previous_match_mode=int(message.coachinfo.match_type),
            world_received=True,
        )
        self.latest_pass_command = message.pass_cmd
        self.last_world_update = time.monotonic()

    def _on_ball_holding(self, message: BallIsHolding) -> None:
        self.ball_holding = bool(message.ball_is_holding)

    def _control_loop(self) -> None:
        now = time.monotonic()
        world_fresh = (
            self.world.world_received
            and now - self.last_world_update <= self.world_timeout
        )
        if world_fresh:
            decision = self.strategy.step(self.world, self.ball_holding)
        else:
            # 世界模型丢失时只发停止命令，避免沿旧目标继续运动。
            stale_world = WorldState(
                agent_id=self.agent_id,
                robots=self.world.robots,
                match_mode=int(MatchMode.STOP),
            )
            decision = self.strategy.step(stale_world, False)

        self.action_publisher.publish(self._action_message(decision))
        self.strategy_publisher.publish(self._strategy_message(decision))

    def _action_message(self, decision) -> ActionCmd:
        robot = self.world.self_robot
        message = ActionCmd()
        message.target.x = float(decision.target.x)
        message.target.y = float(decision.target.y)
        message.target_ori = float(decision.target_orientation)
        message.target_w = float(decision.target_angular_velocity)
        message.target_vel.x = float(decision.target_velocity.x)
        message.target_vel.y = float(decision.target_velocity.y)
        message.maxvel = float(decision.max_velocity)
        message.maxw = float(decision.max_angular_velocity)
        message.robot_pos.x = float(robot.position.x)
        message.robot_pos.y = float(robot.position.y)
        message.robot_vel.x = float(robot.velocity.x)
        message.robot_vel.y = float(robot.velocity.y)
        message.robot_ori = float(robot.heading)
        message.robot_w = float(robot.angular_velocity)
        message.move_action = int(decision.move_action)
        message.rotate_action = int(decision.rotate_action)
        message.rotate_mode = int(decision.rotate_mode)
        message.handle_enable = int(decision.handle_enable)
        message.strength = float(decision.strength)
        message.shoot_pos = int(decision.shoot_position)
        return message

    def _strategy_message(self, decision) -> StrategyInfo:
        message = StrategyInfo()
        message.header.stamp = self.get_clock().now().to_msg()
        message.agent_id = self.agent_id
        message.role = int(decision.role)
        message.action = int(decision.move_action)
        message.is_dribble = self.ball_holding
        message.is_kickoff = self.world.match_mode in (
            int(MatchMode.OUR_KICKOFF),
            int(MatchMode.OPP_KICKOFF),
        )
        message.role_time = float(self.strategy.role_time)
        if self.latest_pass_command is not None:
            message.pass_cmd = self.latest_pass_command
        return message

    def publish_stop(self) -> None:
        """退出节点前发布一次明确的停止动作."""
        message = ActionCmd()
        message.move_action = int(Action.NO_ACTION)
        message.rotate_action = int(Action.NO_ACTION)
        message.robot_pos.x = float(self.world.self_robot.position.x)
        message.robot_pos.y = float(self.world.self_robot.position.y)
        message.robot_ori = float(self.world.self_robot.heading)
        self.action_publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[NubotControlNode] = None
    try:
        node = NubotControlNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.publish_stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
