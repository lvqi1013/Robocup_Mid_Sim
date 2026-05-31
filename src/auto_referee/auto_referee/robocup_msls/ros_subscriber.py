from .qt_compat import QObject, QTimer
from .score_bridge import ScoreBridge


class RosScoreSubscriber(QObject):
    """Optional ROS2 subscriber. The UI remains usable when ROS2 is unavailable."""

    def __init__(self, bridge: ScoreBridge):
        super().__init__()
        self.bridge = bridge
        self.available = False
        self.node = None
        self.rclpy = None
        self.timer = None
        self._setup_ros()

    def _setup_ros(self):
        try:
            import rclpy
            from rclpy.node import Node
            from std_msgs.msg import Int8
        except Exception:
            return

        class ScoreNode(Node):
            def __init__(self, bridge):
                super().__init__("robocup_msls_ui")
                self.create_subscription(
                    Int8,
                    "black_team_score",
                    lambda msg: bridge.blackScoreChanged.emit(int(msg.data)),
                    10,
                )
                self.create_subscription(
                    Int8,
                    "red_team_score",
                    lambda msg: bridge.redScoreChanged.emit(int(msg.data)),
                    10,
                )

        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self.rclpy = rclpy
            self.node = ScoreNode(self.bridge)
            self.timer = QTimer(self)
            self.timer.timeout.connect(lambda: rclpy.spin_once(self.node, timeout_sec=0.0))
            self.timer.start(30)
            self.available = True
        except Exception:
            self.available = False

    def shutdown(self):
        if self.timer:
            self.timer.stop()
        if self.node:
            self.node.destroy_node()
        if self.rclpy and self.rclpy.ok():
            self.rclpy.shutdown()
