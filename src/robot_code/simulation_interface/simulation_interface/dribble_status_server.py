import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16

from nubot_interfaces.srv import DribbleId


class DribbleStatusServer(Node):
    """带球状态服务节点。

    Gazebo 插件通过 `/DribbleId` 服务上报进入/退出带球的机器人编号；本节点保存
    最新编号，并周期性发布到 `/dribble_id`，供 world_model 等只订阅话题的节点使用。
    """

    def __init__(self) -> None:
        """初始化服务端、状态发布者和定时发布器。"""
        super().__init__('dribble_status_server')

        # ROS 参数：
        # service_name: 接收 Gazebo 插件带球编号上报的服务名，默认 /DribbleId。
        # status_topic: 周期发布当前持球编号的话题，默认 /dribble_id。
        # publish_rate: 持球编号话题发布频率，单位 Hz。
        self.declare_parameter('service_name', '/DribbleId')
        self.declare_parameter('status_topic', '/dribble_id')
        self.declare_parameter('publish_rate', 10.0)

        # 当前持球机器人编号；-1 表示没有机器人处于全局带球状态。
        self.current_dribble_id = -1
        # 输入：服务请求中的 agent_id 会直接更新 current_dribble_id。
        self.service = self.create_service(
            DribbleId,
            self.get_parameter('service_name').value,
            self._on_dribble_id,
        )
        # 输出：将服务状态转换为话题，便于其他节点低耦合订阅。
        self.publisher = self.create_publisher(
            Int16,
            self.get_parameter('status_topic').value,
            10,
        )

        publish_rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / publish_rate, self._publish_status)
        self.get_logger().info(
            f"dribble status server started: service={self.get_parameter('service_name').value} "
            f"status_topic={self.get_parameter('status_topic').value}"
        )

    def _on_dribble_id(self, request, response):
        """处理 DribbleId 服务请求，更新当前持球编号并立即发布一次状态。"""
        self.current_dribble_id = int(request.agent_id)
        self._publish_status()
        return response

    def _publish_status(self) -> None:
        """发布当前持球机器人编号；-1 表示清空/无持球者。"""
        msg = Int16()
        msg.data = self.current_dribble_id
        self.publisher.publish(msg)


def main(args=None) -> None:
    """ROS 2 入口函数：启动 DribbleStatusServer 节点并阻塞 spin。"""
    rclpy.init(args=args)
    node = DribbleStatusServer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
