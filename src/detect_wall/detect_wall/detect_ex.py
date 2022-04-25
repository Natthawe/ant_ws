## This code is going to publish on topic "cmd_vel" and subscribe "/scan" topic
#  Written by Muhammad Luqman
# 8/6/21
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidingBot(Node):
    def __init__(self):
        super().__init__('Go_to_position_node') ## name of the node
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,history=rclpy.qos.HistoryPolicy.KEEP_LAST,depth=1)       

        # publisher
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 40)
        #subscriber
        self.subscription=self.create_subscription(LaserScan,'/scan',self.get_scan_values,qos_profile=qos_policy)
        #periodic publisher call
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.send_cmd_vel)
        ## Initializing Global values
        self.linear_vel = 0.15 ## given a value
        self.velocity=Twist()
        self.regions={'right':[],'mid':[],'left':[]}

    def get_scan_values(self,scan_data):
        ## We have 360 data points so we divide them in 3 regions
        ## we say if there is something in the region get the smallest value
        self.regions = {
        'right':   min(min(scan_data.ranges[360:600]), 100),
        'mid':     min(min(scan_data.ranges[600:840]), 100),
        'left':    min(min(scan_data.ranges[840:1080]), 100),
        }
        print(self.regions['left']," | ",self.regions['mid']," | ",self.regions['right'])
        ## testing defined regions
        # print("///////left     = ", self.regions['left']) 
        # print("///////mid      = ", self.regions['mid'])
        # print("///////right    = ", self.regions['right'])


    

    def send_cmd_vel(self):
        if(self.regions['left'] > 1  and self.regions['mid'] > 1   and self.regions['right'] > 1 ): # 111
            self.velocity.linear.x=self.linear_vel
            self.velocity.angular.z=0.0 # condition in which area is total clear
            print("forward")
        elif(self.regions['left'] > 1 and self.regions['mid'] > 1  and self.regions['right'] < 1 ): # 110
            self.velocity.angular.z=0.1# object on right,taking  left 
            self.velocity.linear.x=self.linear_vel
            print("left")
        elif(self.regions['left'] > 1 and self.regions['mid'] < 1  and self.regions['right'] > 1 ): # 101
            self.velocity.angular.z=3.14# object ahead take full turn
            print("reverse")  
        elif(self.regions['left'] > 1 and self.regions['mid'] < 1  and self.regions['right'] < 1 ): # 100
            self.velocity.angular.z=0.1# object on right,taking  left 
            self.velocity.linear.x=self.linear_vel   
            print("left")
        elif(self.regions['left'] < 1 and self.regions['mid'] > 1  and self.regions['right'] > 1 ): # 011
            self.velocity.angular.z=-0.1 #object  on left, taking  right
            self.velocity.linear.x=self.linear_vel
            print("right")  
        elif(self.regions['left'] > 1  and self.regions['mid'] > 1   and self.regions['right'] > 1 ): # 010
            self.velocity.linear.x=self.linear_vel
            self.velocity.angular.z=0.0 # condition in which area is total clear
            print("forward")    
        elif(self.regions['left'] < 1 and self.regions['mid'] < 1  and self.regions['right'] > 1 ): # 001
            self.velocity.angular.z=-0.1 #object  on left, taking  right
            self.velocity.linear.x=self.linear_vel
            print("right")                             
        elif(self.regions['left'] < 1 and self.regions['mid'] < 1  and self.regions['right'] < 1 ): # 000
            self.velocity.angular.z=3.14# object ahead take full turn
            print("reverse")           
        else:## lThis code is not completed ->  you have  to add more conditions  ot make it robust
            print("some other conditions are required to be programmed") 
        self.publisher.publish(self.velocity)

def main(args=None):
    rclpy.init(args=args)
    oab=ObstacleAvoidingBot()
    rclpy.spin(oab)
    rclpy.shutdown()

if __name__ == '__main__':
    main()