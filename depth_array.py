#!/usr/bin/env python
from __future__ import print_function

import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import String, Float32MultiArray
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher('depth_array', Float32MultiArray, queue_size=10)
    print('callback')
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/depth_registered/image_raw", Image ,self.callback)

  def callback(self,data):
    
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data)
      cv_image_array = np.array(cv_image, dtype = np.dtype('f8'))
      # Normalize the depth image to fall between 0 (black) and 1 (white)
      # http://docs.ros.org/electric/api/rosbag_video/html/bag__to__video_8cpp_source.html lines 95-125
      depthimg = cv2.normalize(cv_image_array, cv_image_array, 0, 1, cv2.NORM_MINMAX)
      print(np.array(depthimg)[0:480, 0:640])
      print(depthimg.size)
      # Resize to the desired size
      cv2.imshow("Image from my node", cv_image_array)
      cv2.waitKey(1)
    except CvBridgeError as e:
      print(e)

    try:
      self.image_pub.publish(np.array(depthimg))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
