import rclpy
from rclpy.node import Node
import math
import yaml

from geometry_msgs.msg import Twist, PoseArray
from sensor_msgs.msg import Joy
from ros_gz_interfaces.msg import Entities, EntityWrench

class ModernBallController(Node):
    def __init__(self):
        super().__init__('modern_ball_controller')
        
        # --- 参数配置 (相当于以前的 rosnode_->param) ---
        self.declare_parameter('field_length', 22.0)
        self.declare_parameter('field_width', 14.0)
        self.field_length = self.get_parameter('field_length').value
        self.field_width = self.get_parameter('field_width').value
        
        # --- 订阅与发布 ---
        # 1. 订阅 ROS 2 标准手柄数据
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # 2. 发布速度指令给足球
        self.vel_pub = self.create_publisher(Twist, '/football/cmd_vel', 10)
        
        # 3. 订阅足球的实时位置（用于越界检测）
        self.pose_sub = self.create_subscription(PoseArray, '/football/pose', self.pose_callback, 10)
        
        self.get_logger().info("⚽ 现代足球控制节点已启动！")

    def joy_callback(self, msg: Joy):
        """替代 C++ 中的 joyCallback 和 UpdateChild 的一部分"""
        # 读取手柄摇杆 (根据你原本的逻辑 idx_X=1, idx_Y=0)
        joy_x = msg.axes[1] * 5.0
        joy_y = -msg.axes[0] * 5.0
        
        # 如果摇杆推力大于阈值，发布速度指令
        if math.sqrt(joy_x**2 + joy_y**2) > 1.0:
            twist_msg = Twist()
            twist_msg.linear.x = float(joy_x)
            twist_msg.linear.y = float(joy_y)
            twist_msg.linear.z = 0.0
            # 发布速度，Gazebo 的 VelocityControl 插件会自动接管物理运动
            self.vel_pub.publish(twist_msg)

    def pose_callback(self, msg: PoseArray):
        """替代 C++ 中的 detect_ball_out"""
        # 注意：PoseArray 中可能包含多个 link，这里假设第一个就是球体
        if not msg.poses:
            return
            
        ball_pose = msg.poses[0]
        pos_x = ball_pose.position.x
        pos_y = ball_pose.position.y
        
        # 越界检测逻辑
        out_of_bounds = False
        target_x = pos_x
        target_y = pos_y
        
        if abs(pos_x) > self.field_length / 2.0:
            sign_x = 1.0 if pos_x > 0 else -1.0
            target_x = sign_x * (self.field_length / 2.0 - 0.02)
            out_of_bounds = True
            
        if abs(pos_y) > self.field_width / 2.0:
            sign_y = 1.0 if pos_y > 0 else -1.0
            target_y = sign_y * (self.field_width / 2.0 - 0.02)
            out_of_bounds = True
            
        if out_of_bounds:
            self.get_logger().warn(f"⚠️ 足球出界! 准备重置位置: x={target_x:.2f}, y={target_y:.2f}")
            # 在 ROS 2 中，你可以调用服务或者发布特定话题来重置 Gazebo 模型位置
            # 此处可以调用 ros_gz 桥接出来的 /set_pose 服务来实现 C++ 中 SetWorldPose 的效果
            self.reset_ball_pose(target_x, target_y)

    def reset_ball_pose(self, x, y):
        """调用 Gazebo 接口瞬移球体 (简写演示)"""
        # 实际开发中，这里通常通过创建一个 ROS 2 客户端，
        # 调用 '/world/football_world/set_pose' 服务来实现瞬间位置重置。
        # 同时发布一个全零的 Twist 消息来抵消掉它的惯性速度。
        stop_msg = Twist()
        self.vel_pub.publish(stop_msg)
        self.get_logger().info("已发送停止和重置指令。")

def main(args=None):
    rclpy.init(args=args)
    node = ModernBallController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()