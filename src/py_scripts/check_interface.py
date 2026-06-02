import sys
import subprocess

subprocess.run(['ros2', 'interface', 'show', 'geometry_msgs/msg/Twist'])
print('--------------------------------')
subprocess.run(['ros2', 'interface', 'show', 'geometry_msgs/msg/PoseArray'])
print('--------------------------------')