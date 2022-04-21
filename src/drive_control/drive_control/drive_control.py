#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from serial.tools import list_ports
from math import sin, cos, pi

class control_node(Node):
    def __init__(self, SerialPort, Baudrate):
        super().__init__("control_diff")
        self.nodename = "control_diff"
        self.get_logger().info(f"-I- {self.nodename} started")
        #### init port ####
        self.serial_port = SerialPort
        self.baud_rate = Baudrate
        
        ### variable init ###
        self.x = 0.0  # position in xy plane
        self.y = 0.0
        self.th = 0.0
        self.dx = 0.0  # speeds in x/rotation
        self.dr = 0.0
        self.then = self.get_clock().now()
        
        #### init global value ####
        self.enc_wheel_left = 0
        self.enc_wheel_right = 0
        self.enc_wheel_left_pv = 0
        self.enc_wheel_right_pv = 0
        
        ### enc funtion wrap value init ###
        self.enc_wheel_left_mult = 0.0
        self.enc_wheel_right_mult = 0.0
        self.prev_enc_left = 0
        self.prev_enc_right = 0
        
        #### parameters ####
        self.ticks_meter = float(self.declare_parameter('ticks_meter', 33970.276).value) 
        self.base_width = float(self.declare_parameter('base_width', 0.51).value)
        
        self.NS_TO_SEC = 1000000000
        
        #### frame id node ####
        self.base_frame_id = self.declare_parameter('base_frame_id','base_link').value
        self.odom_frame_id = self.declare_parameter('odom_frame_id','odom').value
        
        #### encoder min max value ####
        self.encoder_min = self.declare_parameter('encoder_min', -2147483648).value
        self.encoder_max = self.declare_parameter('encoder_max', 2147483647).value
        
        #### encoder min max wrap value ####
        self.encoder_low_wrap = self.declare_parameter('wheel_low_wrap', (
                self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min).value
        self.encoder_high_wrap = self.declare_parameter('wheel_high_wrap', (
                self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min).value
        
        ### init def frist start ###
        self._serial = self.serial_connect(self.serial_port, self.baud_rate)
        
        #### init subscription topic ###
        self.sub_cmd_vel = self.create_subscription(
            Twist, 
            "cmd_vel", 
            self.callback_cmd_vel, 
            10
        )
        
        #### init publisher topic ####
        self.odom_pub = self.create_publisher(Odometry, "odometry/wheel", 10)
        
        #### init tf broadcaster
        self.odom_broadcaster = TransformBroadcaster(self)
        
        ### init timer rate hz ###
        self.rate_hz = self.declare_parameter("rate_hz", 100.0).value
        self.create_timer(1.0/self.rate_hz, self.update)
        
        
    ### def funtion node ###
    def callback_cmd_vel(self, twist):
        speed = twist.linear.x
        angRate = twist.angular.z
        # print(speed, angRate)
        wheel_left_set = ((speed - angRate) * self.ticks_meter)
        wheel_right_set = ((speed + angRate) * self.ticks_meter)
        str_send = str(wheel_left_set) + "," + str(wheel_right_set) + '\n'
        self.serial_write(str_send)
        
    def update(self):
        self.enc_wheel_left, self.enc_wheel_right = self.wheel_enc_ticks()
        
        ## time calculate
        now = self.get_clock().now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.nanoseconds / self.NS_TO_SEC
        
        # calculate odometry
        distance_wheel_left = (self.enc_wheel_left - self.enc_wheel_left_pv) / self.ticks_meter
        distance_wheel_right = (self.enc_wheel_right - self.enc_wheel_right_pv) / self.ticks_meter
        
        # distance traveled is the average of the two wheels 
        dist = (distance_wheel_left + distance_wheel_right) / 2
        # this approximation works (in radians) for small angles
        th = (distance_wheel_right - distance_wheel_left) / self.base_width
        # calculate velocities
        self.dx = dist / elapsed
        self.dr = th / elapsed
        
        if dist != 0:
            # calculate distance traveled in x and y
            x = cos(th) * dist
            y = -sin(th) * dist
            # calculate the final position of the robot
            self.x = self.x + (cos(self.th) * x - sin(self.th) * y)
            self.y = self.y + (sin(self.th) * x + cos(self.th) * y)
        if th != 0:
            self.th = self.th + th
            
        # publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2)
        quaternion.w = cos(self.th / 2)
        
        transform_stamped_msg = TransformStamped()
        transform_stamped_msg.header.stamp = now.to_msg()
        transform_stamped_msg.header.frame_id = self.odom_frame_id
        transform_stamped_msg.child_frame_id = self.base_frame_id
        transform_stamped_msg.transform.translation.x = self.x
        transform_stamped_msg.transform.translation.y = self.y
        transform_stamped_msg.transform.translation.z = 0.0
        transform_stamped_msg.transform.rotation.x = quaternion.x
        transform_stamped_msg.transform.rotation.y = quaternion.y
        transform_stamped_msg.transform.rotation.z = quaternion.z
        transform_stamped_msg.transform.rotation.w = quaternion.w
        
        # self.odom_broadcaster.sendTransform(transform_stamped_msg)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.dr
        self.odom_pub.publish(odom)
        
        self.enc_wheel_left_pv = self.enc_wheel_left
        self.enc_wheel_right_pv = self.enc_wheel_right
        
    ### def funtion ###
    def wheel_enc_ticks(self):
        recv = self._readline().decode("utf-8")
        buf_lr = recv.split(",")
        enc_wheel_left_raw = int(buf_lr[0])
        enc_wheel_right_raw = int(buf_lr[1])
        
        if enc_wheel_left_raw < self.encoder_low_wrap and self.prev_enc_left > self.encoder_high_wrap:
            self.enc_wheel_left_mult = self.enc_wheel_left_mult + 1
            
        if enc_wheel_left_raw > self.encoder_high_wrap and self.prev_enc_left < self.encoder_low_wrap:
            self.enc_wheel_left_mult = self.enc_wheel_left_mult - 1
            
        enc_wheel_left = 1.0 * (enc_wheel_left_raw + self.enc_wheel_left_mult * (self.encoder_max - self.encoder_min))
        self.prev_enc_left = enc_wheel_left_raw
        
        if enc_wheel_right_raw < self.encoder_low_wrap and self.prev_enc_right > self.encoder_high_wrap:
            self.enc_wheel_right_mult = self.enc_wheel_right_mult + 1
            
        if enc_wheel_right_raw > self.encoder_high_wrap and self.prev_enc_right < self.encoder_low_wrap:
            self.enc_wheel_right_mult = self.enc_wheel_right_mult - 1
            
        enc_wheel_right = 1.0 * (enc_wheel_right_raw + self.enc_wheel_right_mult * (self.encoder_max - self.encoder_min))
        self.prev_enc_right = enc_wheel_right_raw
        
        return enc_wheel_left, enc_wheel_right
    
    def serial_connect(self, serial_port, baudrate):
        _serial_port = serial.Serial(serial_port, baudrate)
        return _serial_port
    
    def serial_read(self, _serial, range_buf=100):
        return _serial.read(range_buf)

    def serial_write(self, speed_vel):
        res = bytes(speed_vel, 'utf-8')
        self._serial.write(res)
        
    def _readline(self):
        eol = b'\r\n'
        leneol = len(eol)
        line = bytearray()
        while True:
            reader = self.serial_read(self._serial, 1)
            if reader:
                line += reader
                if line[-leneol:] == eol:
                    break
            else:
                break
        return bytes(line)
    
# start
STR_USBPORT = "USB VID:PID=16C0:0483 SER=7442840 LOCATION=2-7:1.0"
_baudrate = 9600
    
def getControl_drivePort():
    for port in list(list_ports.comports()):
        print(port[2])
        if port[2] == STR_USBPORT:
            return port[0]
    
def main(args=None):
    rclpy.init(args=args)
    _serial_port = getControl_drivePort()
    _control_node = control_node(_serial_port, _baudrate)
    rclpy.spin(_control_node)
    _control_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()