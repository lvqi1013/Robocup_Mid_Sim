from functools import partial
from typing import Dict

import rclpy
from rclpy.node import Node

from nubot_interfaces.msg import SimulationStrategy, StrategyInfo


class StrategyAggregator(Node):
    """策略聚合节点。

    每台机器人可以发布自己的 `/<team_prefix><id>/nubotcontrol/strategy`
    StrategyInfo。本节点把多个单机器人策略缓存后聚合成队伍级
    `/<team_prefix>/nubotcontrol/strategy` SimulationStrategy，供 world_model
    一次性读取全队策略状态。
    """

    def __init__(self) -> None:
        """初始化 ROS 参数、策略缓存、订阅者、发布者和定时聚合器。"""
        super().__init__('strategy_pub_node')

        # ROS 参数：
        # team_prefix: 队伍前缀，例如 nubot；用于拼接输入和输出话题。
        # team_size: 队伍机器人数量；输入监听范围到该编号为止。
        # first_agent_id: 第一个监听的机器人编号，通常为 1。
        # publish_rate: 聚合后的 SimulationStrategy 发布频率，单位 Hz。
        # timeout_sec: 单机器人策略缓存最大有效时间；超时策略不会进入聚合消息。
        self.declare_parameter('team_prefix', 'nubot')
        self.declare_parameter('team_size', 5)
        self.declare_parameter('first_agent_id', 1)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('timeout_sec', 0.10)

        self.team_prefix = self.get_parameter('team_prefix').value
        self.team_size = int(self.get_parameter('team_size').value)
        self.first_agent_id = int(self.get_parameter('first_agent_id').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        # latest_strategy 按 agent_id 保存最近一次 StrategyInfo。
        # latest_time_ns 保存接收时刻，用于 publish 时剔除超时策略。
        self.latest_strategy: Dict[int, StrategyInfo] = {}
        self.latest_time_ns: Dict[int, int] = {}

        # 输入：每台机器人各自的策略话题。
        for agent_id in range(self.first_agent_id, self.team_size + 1):
            topic = f'/{self.team_prefix}{agent_id}/nubotcontrol/strategy'
            self.create_subscription(
                StrategyInfo,
                topic,
                partial(self._on_strategy, agent_id=agent_id),
                10,
            )

        # 输出：队伍级策略集合，world_model 订阅该话题补齐 robotinfo 的策略字段。
        self.publisher = self.create_publisher(
            SimulationStrategy,
            f'/{self.team_prefix}/nubotcontrol/strategy',
            10,
        )
        # 定时聚合，避免每个机器人策略消息到达时都触发全队发布。
        self.create_timer(1.0 / publish_rate, self._publish)

        self.get_logger().info(
            f'strategy aggregator started: '
            f'input=/{self.team_prefix}{self.first_agent_id}..{self.team_size}/nubotcontrol/strategy '
            f'output=/{self.team_prefix}/nubotcontrol/strategy'
        )

    def _on_strategy(self, msg: StrategyInfo, agent_id: int) -> None:
        """缓存单机器人 StrategyInfo。

        如果上游未填写 agent_id（值为 0），则使用订阅话题对应的 agent_id 补齐，
        以保证聚合消息中的每条策略都能被 world_model 按编号匹配。
        """
        if msg.agent_id == 0:
            msg.agent_id = agent_id
        self.latest_strategy[agent_id] = msg
        self.latest_time_ns[agent_id] = self.get_clock().now().nanoseconds

    def _publish(self) -> None:
        """发布未超时的策略集合，超时或尚未收到的机器人策略会被跳过。"""
        now_ns = self.get_clock().now().nanoseconds
        max_age_ns = int(self.timeout_sec * 1_000_000_000)
        strategy = SimulationStrategy()

        for agent_id in range(self.first_agent_id, self.team_size + 1):
            stamp_ns = self.latest_time_ns.get(agent_id)
            if stamp_ns is None:
                continue
            if now_ns - stamp_ns <= max_age_ns:
                strategy.strategy_msgs.append(self.latest_strategy[agent_id])

        self.publisher.publish(strategy)


def main(args=None) -> None:
    """ROS 2 入口函数：启动 StrategyAggregator 节点并阻塞 spin。"""
    rclpy.init(args=args)
    node = StrategyAggregator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
