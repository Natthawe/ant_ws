#!/usr/bin/env python3
# Import the necessary libraries
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import time
  
class StreamingCameraCV(Node):
  """
  Create an StreamingCameraCV class, which is a subclass of the Node class.
  """
  def __init__(self):
    """
    Class constructor to set up the node
    """
    # Initiate the Node class's constructor and give it a name
    super().__init__('streaming_camera_cv')
       
    # Create the publisher. This publisher will publish an Image
    # to the video_frames topic. The queue size is 10 messages.
    self.publisher_ = self.create_publisher(Image, 'image_raw', 5)
       
    # We will publish a message every 0.1 seconds
    timer_period = 1/30  # seconds
       
    # Create the timer
    self.timer = self.create_timer(timer_period, self.timer_callback)
          
    # Create a VideoCapture object
    # The argument '0' gets the default webcam.
    self.cap = cv2.VideoCapture(0)
    self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')) 
    self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280) # 1280
    self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720) # 720
    # self.cap.set(cv2.CAP_PROP_FPS, 30)
    
    self.prev_frame_time = 0
    self.new_frame_time = 0
          
    # Used to convert between ROS and OpenCV images
    self.br = CvBridge()
    
  def timer_callback(self):
    """
    Callback function.
    This function gets called every 0.01 seconds.
    """
    # Capture frame-by-frame
    # This method returns True/False as well
    # as the video frame.
    ret, frame = self.cap.read()
           
    if ret == True:
      # Publish the image.
      # The 'cv2_to_imgmsg' method converts an OpenCV
      # image to a ROS 2 image message
      self.new_frame_time = time.time()
      self.publisher_.publish(self.br.cv2_to_imgmsg(frame, "bgr8"))
      fps = 1/(self.new_frame_time-self.prev_frame_time)
      self.prev_frame_time = self.new_frame_time
      self.get_logger().info("Capturing {:.2f} frames".format(fps))

  
    # Display the message on the console
    # self.get_logger().info('Publishing video frame')
   
    # def destroy_piple(self):
    #    self.cap.release()
   
def main(args=None):
   
  # Initialize the rclpy library
  rclpy.init(args=args)
   
  # Create the node
  streaming_camera_cv = StreamingCameraCV()
   
  # Spin the node so the callback function is called.
  rclpy.spin(streaming_camera_cv)
   
   
  # streaming_camera_cv.destroy_piple()
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  streaming_camera_cv.destroy_node()
   
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
   
if __name__ == '__main__':
  main()