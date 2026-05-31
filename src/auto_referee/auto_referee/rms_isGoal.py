import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int8

import math
from compition_state import FieldConfig

robocupmsls_field = FieldConfig()
class IsGoal(Node):
    def __init__(self,):
        super().__init__('isGoalNode')

        self.football_pose_sub = self.create_subscription(
            PoseArray,'/football/pose',self.football_pose_callback, 10
        )

        self.black_team_score_pub_ = self.create_publisher(Int8,'black_team_score', 10)
        self.red_team_score_pub_ = self.create_publisher(Int8,'red_team_score', 10)


        self.black_team_score = 0
        self.red_team_score = 0

        self.black_team_score_msg = Int8()
        self.red_team_score_msg = Int8()

        self.get_logger().info('qidong')

    def football_pose_callback(self, msg: PoseArray):
        self.football_pose_msg = msg
        if len(msg.poses) > 0:
            football_pose_x = msg.poses[0].position.x
            football_pose_y = msg.poses[0].position.y
            football_pose_z = msg.poses[0].position.z
            football_pose_x = robocupmsls_field.m2cm(football_pose_x)
            football_pose_y = robocupmsls_field.m2cm(football_pose_y)
            football_pose_z = robocupmsls_field.m2cm(football_pose_z)
            
            # check the black team is Goal
            if football_pose_x > (robocupmsls_field.FIELD_LENGTH / 2.0 - robocupmsls_field.BALL_RADIUS) \
                and (math.fabs(football_pose_y) > -109.0 and math.fabs(football_pose_y) < 109.0) \
                and math.fabs(football_pose_z) < robocupmsls_field.GOAL_HEIGHT - robocupmsls_field.BALL_RADIUS:

                self.black_team_score += 1

                self.black_team_score_msg.data = self.black_team_score
                self.black_team_score_pub_.publish(self.black_team_score_msg)
                self.get_logger().info("black jinqiu")

            # check the red team is Goal
            elif football_pose_x < -(robocupmsls_field.FIELD_LENGTH / 2.0 - robocupmsls_field.BALL_RADIUS) \
                and (math.fabs(football_pose_y) > -109.0 and math.fabs(football_pose_y) < 109.0) \
                and math.fabs(football_pose_z) < robocupmsls_field.GOAL_HEIGHT - robocupmsls_field.BALL_RADIUS:

                self.red_team_score += 1

                self.red_team_score_msg.data = self.red_team_score
                self.red_team_score_pub_.publish(self.red_team_score_msg)
                self.get_logger().info("red jinqiu")


def main():
    rclpy.init()
    node = IsGoal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()