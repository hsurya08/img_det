#!/usr/bin/env python3

import rclpy # Import the ROS client library for Python
from rclpy.node import Node # Enables the use of rclpy's Node class
from std_msgs.msg import Int64 
import numpy as np # NumPy Python library
import random # Python library to generate random numbers
import cv2
from sensor_msgs.msg import CompressedImage
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
    super().__init__('img_publisher')
     
    # Create publisher(s)
     
    # This node publishes the position of an object every 3 seconds.
    # Maximum queue size of 10. 
    self.publisher_position_cam_frame = self.create_publisher(Int64, 'ball_coordinates', 10)
    self.dist = lambda x1, y1, x2, y2: (x1-x2)**2 + (y1 - y2)**2  
    self.lastCaptured = None 
    self._video_subscriber = self.create_subscription(
				CompressedImage,
				'/camera/image/compressed',
				self._image_callback,
				1)
    self._video_subscriber # Prevents unused variable warning.
    
  def _image_callback(self, CompressedImage):

    self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")

    x = 0
    frame = self._imgBGR
    original_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred_image = cv2.GaussianBlur(original_img, (15, 15), 0)
    detected_circles = cv2.HoughCircles(blurred_image, cv2.HOUGH_GRADIENT, dp=0.8, minDist=100, param1=100, param2=30, minRadius=30, maxRadius=150)

    if detected_circles is not None:
        detected_circles = np.uint16(np.around(detected_circles))
        current = None
        for i in detected_circles[0, :]:
            if current is None: current = i
            if self.lastCaptured is not None:
                if self.dist(current[0], current[1], self.lastCaptured[0], self.lastCaptured[1]) <= self.dist(i[0], i[1], self.lastCaptured[0], self.lastCaptured[1]):
                    current = i
        frame= cv2.putText(frame, "[X,Y,r]:"+ str(current), (current[0],current[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA , False)  #Pixel co-ordinates display
        cv2.circle(frame, (current[0], current[1]), 1, (0, 100, 100), 3)
        cv2.circle(frame, (current[0], current[1]), current[2], (255, 0, 255), 3)
        self.lastCaptured = current
        x = int(current[0])

    object_position = x
    msg=Int64()
    msg.data=object_position




    cv2.imshow("Detected circles", frame)
    cv2.waitKey(10)
    
    self.publisher_position_cam_frame.publish(msg)
   # if cv2.waitKey(1) & 0xFF == ord('q'):
    #    break

		#cap.release()
		#cv2.destroyAllWindows()
     
     
  
 
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
