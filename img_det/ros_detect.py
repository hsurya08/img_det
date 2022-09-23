#!/usr/bin/env python3

import rclpy # Import the ROS client library for Python
from rclpy.node import Node # Enables the use of rclpy's Node class
from std_msgs.msg import Float64MultiArray # Enable use of the std_msgs/Float64MultiArray message type
import numpy as np # NumPy Python library
import random # Python library to generate random numbers
import cv2
from geometry_msgs.msg import Twist
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
    self.publisher_position_cam_frame = self.create_publisher(Float64MultiArray, '/ball_coord', 10)
    
    cap = cv2.VideoCapture(0)
    lastCaptured = None 
    dist = lambda x1, y1, x2, y2: (x1-x2)**2 + (y1 - y2)**2  
    
    while(True):

        ret, frame = cap.read()


        original_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    

        blurred_image = cv2.GaussianBlur(original_img, (15, 15), 0)
    

        detected_circles = cv2.HoughCircles(blurred_image, cv2.HOUGH_GRADIENT, dp=0.8, minDist=100, param1=100, param2=30, minRadius=200, maxRadius=350)
    

        if detected_circles is not None:
            detected_circles = np.uint16(np.around(detected_circles))
            current = None
        
            for i in detected_circles[0, :]:
                if current is None: current = i
                if lastCaptured is not None:
                    if dist(current[0], current[1], lastCaptured[0], lastCaptured[1]) <= dist(i[0], i[1], lastCaptured[0], lastCaptured[1]):
                        current = i
            frame= cv2.putText(frame, "[X,Y,r]:"+ str(current), (current[0],current[1]), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2, cv2.LINE_AA , False)  #Pixel co-ordinates display
            cv2.circle(frame, (current[0], current[1]), 1, (0, 100, 100), 3)
            cv2.circle(frame, (current[0], current[1]), current[2], (255, 0, 255), 3)
            lastCaptured = current
            x = float(current[0])
            y = float(current[1])          
            object_position = [x, y]
            self.publish_coordinates(object_position)
		
        cv2.imshow("Detected circles", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
     
  def publish_coordinates(self,position):
    """
    Publish the coordinates of the object to ROS2 topics
    :param: The position of the object in centimeter coordinates [x , y] 
    """
    msg = Float64MultiArray() # Create a message of this type 
    msg.data = position # Store the x and y coordinates of the object
    self.publisher_position_cam_frame.publish(msg) # Publish the position to the topic    
 
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
