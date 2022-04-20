#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from serial.tools import list_ports

class Drive_Communicate(Node):
    _serial = None
    wheel_l_old = 0.0
    wheel_r_old = 0.0
    v_range = 33970.276 # 100 cm
    def __init__(self, SerialPort, Baudrate):
        super().__init__('DRIVE_COMUNICATE')
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.Listener_Callback, 10)
        self.publisher_l = self.create_publisher(Int32, 'wheel_l' , 10)
        self.publisher_r = self.create_publisher(Int32, 'wheel_r' , 10)
        time_seconds = 1/60.0
        self.timer = self.create_timer(time_seconds, self.Timer_Callback)
        self._serial = self.Serial_Connect(SerialPort, Baudrate)

    def Listener_Callback(self, twist):
        linear = twist.linear.x
        angular = twist.linear.z
        wheel_l = ((linear - angular) * self.v_range)
        wheel_r = ((linear + angular) * self.v_range)
        send = str(wheel_l) + "," + str(wheel_r) + '\n'
        self.Serial_write(send)

    def Serial_write(self, Speed_val):
        res = bytes(Speed_val, 'utf-8')
        self._serial.write(res)

    def Timer_Callback(self):
        reader = self._readline().decode('utf-8')
        buffer_lr = reader.split(",")
        self.msg_l = Int32()
        self.msg_r = Int32()
        try:
            self.msg_l.data = int(buffer_lr[0])
            self.msg_r.data = int(buffer_lr[1])
            self.publisher_l.publish(self.msg_l)
            self.publisher_r.publish(self.msg_r)
            data_lr = "LEFT-> " + str(self.msg_l.data) + " , " + "RIGHT-> " + str(self.msg_r.data)
            self.get_logger().info(data_lr)
        except:
            pass

    def _readline(self):
        end_of_line = b'\r\n'
        len_end_of_line = len(end_of_line)
        line = bytearray()
        while True:
            reader = self.Serial_Read(self._serial, 1)
            if reader:
                line += reader
                if line[-len_end_of_line:] == end_of_line:
                    break
            else:
                break
        return bytes(line)

    def Serial_Connect(self, SerialPort, Baudrate):
        _serial = serial.Serial(SerialPort, Baudrate, timeout=0)
        return _serial

    def Serial_Read(self, _serial, buffer_range=100):
        return _serial.read(buffer_range)
    
    def Wheel(self, wheel_l_old, wheel_r_old):
        self.wheel_l_old = wheel_l_old
        self.wheel_r_old = wheel_r_old

USB_TEENSY_PORT = "USB VID:PID=16C0:0483 SER=7442840 LOCATION=2-7:1.0"
_Baudrate = 9600

def Teensy_Port():
    for port in list(list_ports.comports()):
        print(port[2])
        if port[2] ==  USB_TEENSY_PORT:
            return port[0]

def main(args=None):
    global Drive_Communicate
    rclpy.init(args=args)
    Port = Teensy_Port()
    Drive_Communicate = Drive_Communicate(Port, _Baudrate)
    rclpy.spin(Drive_Communicate)
    Drive_Communicate.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()