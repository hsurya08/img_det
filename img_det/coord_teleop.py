#!/usr/bin/env python3

import rclpy # Import the ROS client library for Python
from rclpy.node import Node # Enables the use of rclpy's Node class
from std_msgs.msg import Int64 # Enable use of the std_msgs/Float64MultiArray message type
import numpy as np # NumPy Python library
import random # Python library to generate random numbers
import cv2
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class CameraPublisher(Node):
  """
  Create a CameraPublisher class, which is a subclass of the Node class.
  The class publishes the position of an object every 3 seconds.
  The position of the object are the x and y coordinates with respect to 
  the camera frame.
  """

  
  def __init__(self):
    """
    Class constructor to set up the node
    """
   
    # Initiate the Node class's constructor and give it a name
    super().__init__('teleop_publisher')
     
    # Create publisher(s)
     
    # This node publishes the position of an object every 3 seconds.
    # Maximum queue size of 10. 
    self.publisher_position_cam_frame = self.create_publisher(Twist, '/cmd_vel', 10)

    self._video_subscriber = self.create_subscription(
				Int64,
				'/ball_coordinates',
				self._image_callback,
				1)
    self._video_subscriber # Prevents unused variable warning.
    
  def _image_callback(self, msg):
    twist= Twist()
    twist.linear.x = 0.0
    twist.linear.y = 0.0
    twist.linear.z = 0.0
    twist.angular.x = 0.0
    twist.angular.y = 0.0
    twist.angular.z = 0.0  
    x = msg.data
    if (x<130 and x!=0):
        twist.angular.z = 0.5

    elif (x>160):
        twist.angular.z = -0.5
    
    else:
        twist.angular.z = 0.0    	

    self.publisher_position_cam_frame.publish(twist)


 
def main(args=None):
 
  # Initialize the rclpy library
  rclpy.init(args=args)
 
  # Create the node
  camera_publisher = CameraPublisher()
 
  # Spin the node so the callback function is called.
  # Publish any pending messages to the topics.
  rclpy.spin(camera_publisher)
 
  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  camera_publisher.destroy_node()
 
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
 
if __name__ == '__main__':
  main()
