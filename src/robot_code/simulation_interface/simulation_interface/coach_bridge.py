from functools import partial
from typing import Dict, Optional

import rclpy
from rclpy.node import Node

from nubot_interfaces.msg import CoachInfo, CoachWorldModelInfo, WorldModelInfo


# CoachInfo 的默认比赛控制状态：0 表示停止机器人。
STOP_ROBOT = 0


class CoachBridge(Node):
    """教练信息桥接节点。

    该节点周期性向队伍级 `/<team_prefix>/receive_from_coach` 发布 CoachInfo，
    同时订阅各机器人 world_model 输出，从最近且未超时的一份 WorldModelInfo 中
    转换出 coach 侧使用的 `/<team_prefix>/coach/worldmodelinfo`。
    """

    def __init__(self) -> None:
        """初始化 ROS 参数、教练信息缓存、世界模型缓存和发布/订阅定时器。"""
        super().__init__('coach_bridge')

        # ROS 参数：
        # team_prefix: 队伍前缀，例如 nubot；用于拼接队伍级和机器人级话题。
        # team_size: 需要监听的机器人数量，默认监听 nubot1 到 nubot5。
        # publish_rate: CoachInfo 和 CoachWorldModelInfo 的发布频率，单位 Hz。
        # match_mode/match_type/test_mode/kickforce: 启动时写入 CoachInfo 的默认裁判/测试状态。
        # input_topic: 外部覆盖 CoachInfo 的输入话题，便于裁判盒或调试工具动态改比赛状态。
        # world_model_timeout_sec: world_model 缓存最大可用时间；超时数据不会转发给 coach。
        self.declare_parameter('team_prefix', 'nubot')
        self.declare_parameter('team_size', 5)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('match_mode', STOP_ROBOT)
        self.declare_parameter('match_type', STOP_ROBOT)
        self.declare_parameter('test_mode', STOP_ROBOT)
        self.declare_parameter('kickforce', 0)
        self.declare_parameter('input_topic', '/coach/set_coach_info')
        self.declare_parameter('world_model_timeout_sec', 0.5)

        self.team_prefix = self.get_parameter('team_prefix').value
        self.team_size = int(self.get_parameter('team_size').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.world_model_timeout_sec = float(
            self.get_parameter('world_model_timeout_sec').value
        )

        self.coach_info = CoachInfo()
        self._load_coach_params()

        # latest_world_model 保存每台机器人最近一次 world_model。
        # latest_world_time_ns 保存接收时刻，用于过滤掉长时间未更新的数据。
        self.latest_world_model: Dict[int, WorldModelInfo] = {}
        self.latest_world_time_ns: Dict[int, int] = {}

        # 输出：Gazebo 插件和 world_model 节点都会订阅的队伍级教练信息。
        self.coach_pub = self.create_publisher(
            CoachInfo,
            f'/{self.team_prefix}/receive_from_coach',
            10,
        )
        # 输出：coach 工具使用的队伍级世界模型。
        self.coach_world_pub = self.create_publisher(
            CoachWorldModelInfo,
            f'/{self.team_prefix}/coach/worldmodelinfo',
            10,
        )
        # 输入：外部 CoachInfo 覆盖源；收到后直接替换当前 coach_info 缓存。
        self.create_subscription(
            CoachInfo,
            self.get_parameter('input_topic').value,
            self._on_coach_override,
            10,
        )

        # 输入：每台机器人发布的单机器人视角世界模型。
        for agent_id in range(1, self.team_size + 1):
            self.create_subscription(
                WorldModelInfo,
                f'/{self.team_prefix}{agent_id}/worldmodel/worldmodelinfo',
                partial(self._on_world_model, agent_id=agent_id),
                10,
            )

        # 定时发布，解耦外部覆盖输入和各机器人 world_model 的频率。
        self.create_timer(1.0 / publish_rate, self._publish)

        self.get_logger().info(
            f'coach bridge started: command=/{self.team_prefix}/receive_from_coach '
            f'world=/{self.team_prefix}/coach/worldmodelinfo'
        )

    def _load_coach_params(self) -> None:
        """从 ROS 参数读取启动默认值并写入 CoachInfo 缓存。"""
        self.coach_info.match_mode = int(self.get_parameter('match_mode').value)
        self.coach_info.match_type = int(self.get_parameter('match_type').value)
        self.coach_info.test_mode = int(self.get_parameter('test_mode').value)
        self.coach_info.kickforce = int(self.get_parameter('kickforce').value)

    def _on_coach_override(self, msg: CoachInfo) -> None:
        """接收外部 CoachInfo 覆盖消息，用于运行时切换比赛/测试状态。"""
        self.coach_info = msg

    def _on_world_model(self, msg: WorldModelInfo, agent_id: int) -> None:
        """缓存指定机器人最新的 WorldModelInfo 及其接收时间。"""
        self.latest_world_model[agent_id] = msg
        self.latest_world_time_ns[agent_id] = self.get_clock().now().nanoseconds

    def _publish(self) -> None:
        """周期发布 CoachInfo，并在存在新鲜 world_model 时同步发布 coach 世界模型。"""
        now = self.get_clock().now()
        self.coach_info.header.stamp = now.to_msg()
        self.coach_pub.publish(self.coach_info)

        latest = self._latest_fresh_world_model(now.nanoseconds)
        if latest is not None:
            self.coach_world_pub.publish(self._to_coach_world_model(latest, now.to_msg()))

    def _latest_fresh_world_model(self, now_ns: int) -> Optional[WorldModelInfo]:
        """从缓存中选择最新且未超过 `world_model_timeout_sec` 的 WorldModelInfo。"""
        max_age_ns = int(self.world_model_timeout_sec * 1_000_000_000)
        newest_agent = None
        newest_stamp = -1

        for agent_id, stamp_ns in self.latest_world_time_ns.items():
            if now_ns - stamp_ns > max_age_ns:
                continue
            if stamp_ns > newest_stamp:
                newest_agent = agent_id
                newest_stamp = stamp_ns

        if newest_agent is None:
            return None
        return self.latest_world_model[newest_agent]

    def _to_coach_world_model(self, src: WorldModelInfo, stamp) -> CoachWorldModelInfo:
        """把单机器人 WorldModelInfo 转换为 CoachWorldModelInfo。

        CoachWorldModelInfo 的 obstacleinfo 是数组，为兼容旧接口，这里把单个
        WorldModelInfo.obstacleinfo 包成长度为 1 的列表，其余字段按消息类型直接转发。
        """
        dst = CoachWorldModelInfo()
        dst.header.stamp = stamp
        dst.obstacleinfo = [src.obstacleinfo]
        dst.oppinfo = src.oppinfo
        dst.robotinfo = list(src.robotinfo)
        dst.ballinfo = list(src.ballinfo)
        dst.pass_cmd = src.pass_cmd
        return dst


def main(args=None) -> None:
    """ROS 2 入口函数：启动 CoachBridge 节点并阻塞 spin。"""
    rclpy.init(args=args)
    node = CoachBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
