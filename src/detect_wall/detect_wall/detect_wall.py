#! /usr/bin/env python

import rclpy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSDurabilityPolicy
msg = LaserScan

class DetectWall(Node):

  def __init__(self):
    super().__init__('detect_wall')

    qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=1)       

    self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)

    self.sub_cmd_vel = self.create_subscription(LaserScan, "scan", self.callback_cmd_vel, qos_profile=qos_policy)


  def callback_cmd_vel(self, msg):

    vel_cmd = Twist()

    # self.get_logger().info('Value at 180 degrees: %.5f' % msg.ranges[720])

    # print(len(msg.ranges))

    # print('Value at 0 degrees:', msg.ranges[-5])

    # print('Value at 90 degrees:', msg.ranges[360])
  
    # print('Value at 180 degrees:', msg.ranges[720])

    # print('Value at 270 degrees:', msg.ranges[1080])


    # If the distance to an obstacle in front of the robot is bigger than 1 meter, the robot will move forward
    if msg.ranges[720] >= 0.5:
        vel_cmd.linear.x = 0.15
        vel_cmd.angular.z = 0.0

    # If the distance to an obstacle in front of the robot is smaller than 1 meter, the robot will stop
    if msg.ranges[720] <= 0.5:
        vel_cmd.linear.x = 0.0
        vel_cmd.angular.z = 0.0
    self.pub_cmd_vel.publish(vel_cmd)

def main():
    rclpy.init()
    node = DetectWall()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()  