import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseArray

class SubFootballPose(Node):
    def __init__(self):
        super().__init__('sub_football_pose')
        self.pose_sub = self.create_subscription(PoseArray, '/football/pose', self.pose_callback, 10)

    def pose_callback(self, msg: PoseArray):
        # self.get_logger().info(f'Football pose: {msg.poses[0]}')
        if len(msg.poses) > 0:
            football_pose_x = msg.poses[0].position.x
            football_pose_y = msg.poses[0].position.y
            football_pose_z = msg.poses[0].position.z
            self.get_logger().info(f'football_pose_X: {football_pose_x:.2f}, football_pose_Y: {football_pose_y:.2f}, football_pose_Z: {football_pose_z:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = SubFootballPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()