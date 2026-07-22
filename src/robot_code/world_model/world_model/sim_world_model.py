import math
import re
from typing import Dict, Iterable, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16

from nubot_interfaces.msg import (
    BallInfo,
    CoachInfo,
    OminiVisionInfo,
    ObstaclesInfo,
    PassCommands,
    Point2D,
    RobotInfo,
    SimulationStrategy,
    StrategyInfo,
    WorldModelInfo,
)


# WorldModelInfo 中球状态使用的旧工程常量：1 表示本机器人/本节点视角能看到球。
SEE_BALL_BY_OWN = 1
# CoachInfo 的默认比赛控制状态：0 表示停止机器人。
STOP_ROBOT = 0


def _copy_point(src: Point2D) -> Point2D:
    """复制二维点，避免发布前直接修改订阅消息对象。"""
    dst = Point2D()
    dst.x = src.x
    dst.y = src.y
    return dst


def _copy_pass_cmd(src: PassCommands) -> PassCommands:
    """复制传球命令，保留传球/接球编号、目标点和有效性标志。"""
    dst = PassCommands()
    dst.pass_id = src.pass_id
    dst.catch_id = src.catch_id
    dst.pass_pt = _copy_point(src.pass_pt)
    dst.catch_pt = _copy_point(src.catch_pt)
    dst.is_passout = src.is_passout
    dst.is_dynamic_pass = src.is_dynamic_pass
    dst.is_static_pass = src.is_static_pass
    dst.is_valid = src.is_valid
    return dst


def _copy_robot(src: RobotInfo) -> RobotInfo:
    """复制机器人状态，并包含策略目标、角色、带球/射门等扩展字段。"""
    dst = RobotInfo()
    dst.header = src.header
    dst.agent_id = src.agent_id
    dst.target_num1 = src.target_num1
    dst.target_num2 = src.target_num2
    dst.target_num3 = src.target_num3
    dst.target_num4 = src.target_num4
    dst.staticpass_num = src.staticpass_num
    dst.staticcatch_num = src.staticcatch_num
    dst.pos = _copy_point(src.pos)
    dst.heading.theta = src.heading.theta
    dst.vrot = src.vrot
    dst.vtrans = _copy_point(src.vtrans)
    dst.is_kick = src.is_kick
    dst.is_valid = src.is_valid
    dst.is_stuck = src.is_stuck
    dst.is_dribble = src.is_dribble
    dst.current_role = src.current_role
    dst.role_time = src.role_time
    dst.target = _copy_point(src.target)
    return dst


def _copy_ball(src: BallInfo) -> BallInfo:
    """复制球状态，包括世界坐标、相对极坐标、速度和可信标志。"""
    dst = BallInfo()
    dst.header = src.header
    dst.ball_info_state = src.ball_info_state
    dst.pos = _copy_point(src.pos)
    dst.real_pos.angle = src.real_pos.angle
    dst.real_pos.radius = src.real_pos.radius
    dst.velocity = _copy_point(src.velocity)
    dst.pos_known = src.pos_known
    dst.velocity_known = src.velocity_known
    return dst


def _agent_id_from_name(robot_name: str) -> int:
    """从机器人名称末尾提取编号，例如 nubot3 -> 3；无法提取时默认 1。"""
    match = re.search(r'(\d+)$', robot_name)
    if not match:
        return 1
    return max(1, int(match.group(1)))


def _prefix_from_name(robot_name: str) -> str:
    """从机器人名称中去掉末尾编号得到队伍前缀，例如 nubot3 -> nubot。"""
    return re.sub(r'\d+$', '', robot_name) or robot_name


def _distance(a: Point2D, b: Point2D) -> float:
    """计算两个二维点的欧氏距离；本项目位置单位通常为厘米。"""
    return math.hypot(a.x - b.x, a.y - b.y)


class SimWorldModel(Node):
    """仿真世界模型节点。

    节点面向单台机器人运行：订阅该机器人 Gazebo 插件发布的全向视觉信息、
    队伍级教练信息、队伍级策略信息和当前持球机器人编号，然后周期性发布
    `/<robot_name>/worldmodel/worldmodelinfo`，供控制/决策节点读取。
    """

    def __init__(self) -> None:
        """初始化 ROS 2 参数、状态缓存、订阅者、发布者和定时发布器。"""
        super().__init__('world_model_node')

        # ROS 参数：
        # robot_name: 机器人模型/命名空间名称，例如 nubot1；为空时从 namespace 推断。
        # team_prefix: 队伍级话题前缀，例如 nubot；为空时由 robot_name 去掉末尾编号得到。
        # team_size: 本队机器人数量，用于生成 robotinfo/ballinfo 固定长度列表。
        # update_period: 世界模型发布周期，单位秒；默认约 66.7Hz。
        # teammate_filter_radius_cm: 从 obstacleinfo 中剔除队友的距离阈值，单位厘米。
        # dribble_id_topic: 当前持球机器人编号话题；-1 表示没有全局持球者。
        self.declare_parameter('robot_name', '')
        self.declare_parameter('team_prefix', '')
        self.declare_parameter('team_size', 5)
        self.declare_parameter('update_period', 0.015)
        self.declare_parameter('teammate_filter_radius_cm', 35.0)
        self.declare_parameter('dribble_id_topic', '/dribble_id')

        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.robot_name = robot_name or self.get_namespace().strip('/') or 'nubot1'
        team_prefix = self.get_parameter('team_prefix').get_parameter_value().string_value
        self.team_prefix = team_prefix or _prefix_from_name(self.robot_name)
        self.agent_id = _agent_id_from_name(self.robot_name)
        self.team_size = self.get_parameter('team_size').get_parameter_value().integer_value
        self.filter_radius_cm = (
            self.get_parameter('teammate_filter_radius_cm').get_parameter_value().double_value
        )

        # coach_info 保存最近一次教练/裁判盒信息；启动时默认为停止机器人模式。
        self.coach_info = CoachInfo()
        self.coach_info.match_mode = STOP_ROBOT
        self.coach_info.match_type = STOP_ROBOT

        # robots 按 agent_id 缓存最近一次视觉中的机器人状态。
        # strategy 按 agent_id 缓存最近一次策略状态，发布时会覆盖 robotinfo 中的策略字段。
        # latest_omni/latest_ball/latest_obstacles 保存最近一次全向视觉输入。
        # current_dribble_id 保存全局持球者编号，-1 表示不强制改写 is_dribble。
        self.robots: Dict[int, RobotInfo] = {
            idx: self._new_robot(idx) for idx in range(1, self.team_size + 1)
        }
        self.strategy: Dict[int, StrategyInfo] = {}
        self.latest_omni: Optional[OminiVisionInfo] = None
        self.latest_ball = BallInfo()
        self.latest_obstacles = ObstaclesInfo()
        self.current_dribble_id = -1

        # 输入：Gazebo 插件发布的本机全向视觉数据。
        self.create_subscription(
            OminiVisionInfo,
            f'/{self.robot_name}/omnivision/OmniVisionInfo',
            self._on_omni,
            10,
        )
        # 输入：队伍级教练/比赛控制信息。
        self.create_subscription(
            CoachInfo,
            f'/{self.team_prefix}/receive_from_coach',
            self._on_coach,
            10,
        )
        # 输入：队伍级策略状态，用于补齐各机器人角色、目标和传球命令。
        self.create_subscription(
            SimulationStrategy,
            f'/{self.team_prefix}/nubotcontrol/strategy',
            self._on_strategy,
            10,
        )
        # 输入：当前持球者编号，通常由带球状态汇总节点从 /DribbleId 服务转换而来。
        self.create_subscription(
            Int16,
            self.get_parameter('dribble_id_topic').get_parameter_value().string_value,
            self._on_dribble_id,
            10,
        )
        # 输出：本机器人命名空间下的世界模型信息。
        self.world_pub = self.create_publisher(
            WorldModelInfo,
            f'/{self.robot_name}/worldmodel/worldmodelinfo',
            10,
        )

        update_period = self.get_parameter('update_period').get_parameter_value().double_value
        # 定时将最新缓存组装成 WorldModelInfo，解耦输入消息频率和世界模型发布频率。
        self.create_timer(update_period, self._publish_world_model)

        self.get_logger().info(
            f'simulation world_model started for {self.robot_name}: '
            f'agent_id={self.agent_id} team_prefix={self.team_prefix}'
        )

    def _new_robot(self, agent_id: int) -> RobotInfo:
        """创建指定编号的默认机器人状态；未收到视觉前标记为无效。"""
        robot = RobotInfo()
        robot.agent_id = agent_id
        robot.is_valid = False
        return robot

    def _on_omni(self, msg: OminiVisionInfo) -> None:
        """处理全向视觉消息，刷新球、障碍物和有效编号范围内的机器人状态。"""
        self.latest_omni = msg
        self.latest_ball = _copy_ball(msg.ballinfo)
        self.latest_obstacles = msg.obstacleinfo

        for robot in msg.robotinfo:
            if 1 <= robot.agent_id <= self.team_size:
                self.robots[robot.agent_id] = _copy_robot(robot)

    def _on_coach(self, msg: CoachInfo) -> None:
        """保存最近一次教练/裁判盒比赛控制信息。"""
        self.coach_info = msg

    def _on_dribble_id(self, msg: Int16) -> None:
        """保存当前持球机器人编号；-1 表示清空全局持球状态。"""
        self.current_dribble_id = int(msg.data)

    def _on_strategy(self, msg: SimulationStrategy) -> None:
        """处理队伍策略消息，按机器人编号缓存并同步到机器人状态字段。"""
        for strategy in msg.strategy_msgs:
            if 1 <= strategy.agent_id <= self.team_size:
                self.strategy[strategy.agent_id] = strategy
                self._apply_strategy(strategy)

    def _apply_strategy(self, strategy: StrategyInfo) -> None:
        """把单个机器人策略字段写入 robotinfo 缓存，供发布时直接使用。"""
        robot = self.robots.get(strategy.agent_id, self._new_robot(strategy.agent_id))
        robot.target_num1 = strategy.target_num1
        robot.target_num2 = strategy.target_num2
        robot.target_num3 = strategy.target_num3
        robot.target_num4 = strategy.target_num4
        robot.staticpass_num = strategy.staticpass_num
        robot.staticcatch_num = strategy.staticcatch_num
        robot.current_role = strategy.role
        robot.role_time = strategy.role_time
        robot.is_dribble = strategy.is_dribble
        robot.is_kick = strategy.is_kickoff
        self.robots[strategy.agent_id] = robot

    def _publish_world_model(self) -> None:
        """组装并发布 WorldModelInfo。

        输出内容包括队友 robotinfo、过滤后的本机障碍物、推断出的对手障碍物、
        每个队友视角下的球信息、教练信息和当前有效传球命令。
        """
        msg = WorldModelInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.robotinfo = [self._robot_for_publish(idx) for idx in range(1, self.team_size + 1)]
        msg.obstacleinfo = self._self_obstacles()
        msg.oppinfo = self._opponent_obstacles(msg.robotinfo, msg.obstacleinfo)
        msg.ballinfo = self._ball_list(msg.robotinfo)
        msg.coachinfo = self.coach_info
        msg.pass_cmd = self._selected_pass_cmd()
        self.world_pub.publish(msg)

    def _robot_for_publish(self, agent_id: int) -> RobotInfo:
        """生成单个机器人发布状态。

        先复制视觉缓存，再用策略缓存覆盖角色/目标/传球相关字段，最后用全局
        current_dribble_id 统一修正 is_dribble，避免不同来源的带球状态互相冲突。
        """
        robot = _copy_robot(self.robots.get(agent_id, self._new_robot(agent_id)))
        strategy = self.strategy.get(agent_id)
        if strategy is not None:
            robot.target_num1 = strategy.target_num1
            robot.target_num2 = strategy.target_num2
            robot.target_num3 = strategy.target_num3
            robot.target_num4 = strategy.target_num4
            robot.staticpass_num = strategy.staticpass_num
            robot.staticcatch_num = strategy.staticcatch_num
            robot.current_role = strategy.role
            robot.role_time = strategy.role_time
            robot.is_dribble = strategy.is_dribble
            robot.is_kick = strategy.is_kickoff
        if self.current_dribble_id == agent_id:
            robot.is_dribble = True
        elif self.current_dribble_id != -1:
            robot.is_dribble = False
        return robot

    def _self_obstacles(self) -> ObstaclesInfo:
        """复制最近一次全向视觉障碍物，作为本机视角 obstacleinfo 发布。"""
        obstacles = ObstaclesInfo()
        obstacles.header.stamp = self.get_clock().now().to_msg()
        obstacles.pos = list(self.latest_obstacles.pos)
        obstacles.polar_pos = list(self.latest_obstacles.polar_pos)
        return obstacles

    def _opponent_obstacles(
        self,
        robots: Iterable[RobotInfo],
        obstacleinfo: ObstaclesInfo,
    ) -> ObstaclesInfo:
        """从障碍物列表中剔除已知队友，剩余点作为对手/未知障碍物发布。

        Gazebo 插件的 obstacleinfo 包含除本机外的所有机器人。这里根据队友世界坐标
        和 `teammate_filter_radius_cm` 判断障碍物是否为队友；超过阈值的点保留到
        oppinfo。polar_pos 与 pos 使用相同索引同步保留。
        """
        opponents = ObstaclesInfo()
        opponents.header.stamp = self.get_clock().now().to_msg()
        teammates = [robot for robot in robots if robot.is_valid]

        for idx, point in enumerate(obstacleinfo.pos):
            is_teammate = any(
                _distance(point, teammate.pos) <= self.filter_radius_cm
                for teammate in teammates
            )
            if is_teammate:
                continue
            opponents.pos.append(point)
            if idx < len(obstacleinfo.polar_pos):
                opponents.polar_pos.append(obstacleinfo.polar_pos[idx])
        return opponents

    def _ball_list(self, robots: Iterable[RobotInfo]) -> list:
        """为每个队友生成一份球信息列表。

        `WorldModelInfo.ballinfo` 是数组，因此这里复制同一份世界坐标球状态给每个
        agent_id；如果该机器人状态有效，则重新计算球相对该机器人朝向的极坐标
        `real_pos`，供旧控制逻辑按机器人视角读取。
        """
        balls = []
        base_ball = _copy_ball(self.latest_ball)
        base_ball.header.stamp = self.get_clock().now().to_msg()
        base_ball.ball_info_state = SEE_BALL_BY_OWN
        base_ball.pos_known = True
        base_ball.velocity_known = True

        robots_by_id = {robot.agent_id: robot for robot in robots}
        for agent_id in range(1, self.team_size + 1):
            ball = _copy_ball(base_ball)
            robot = robots_by_id.get(agent_id)
            if robot is not None and robot.is_valid:
                dx = ball.pos.x - robot.pos.x
                dy = ball.pos.y - robot.pos.y
                ball.real_pos.angle = math.atan2(dy, dx) - robot.heading.theta
                ball.real_pos.radius = math.hypot(dx, dy)
            balls.append(ball)
        return balls

    def _selected_pass_cmd(self) -> PassCommands:
        """选择要发布的传球协作命令。

        当前节点跳过自身策略，只从其他机器人策略中选择第一条有效的静态传球、
        动态传球或传出命令；若没有有效命令，则发布无效的空 PassCommands。
        """
        empty = PassCommands()
        empty.catch_id = 0
        empty.pass_id = 0
        empty.is_valid = False

        for agent_id, strategy in sorted(self.strategy.items()):
            pass_cmd = strategy.pass_cmd
            if agent_id == self.agent_id:
                continue
            if pass_cmd.is_valid and (
                pass_cmd.is_static_pass or pass_cmd.is_dynamic_pass or pass_cmd.is_passout
            ):
                return _copy_pass_cmd(pass_cmd)
        return empty


def main(args=None) -> None:
    """ROS 2 入口函数：启动 SimWorldModel 节点并阻塞 spin。"""
    rclpy.init(args=args)
    node = SimWorldModel()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
