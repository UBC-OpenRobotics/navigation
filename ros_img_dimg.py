from sensor_msgs.msg import Image
import numpy as np
import rospy
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages')
import time

rgb_img_topic = '/camera/rgb/image_raw'
depth_img_topic = '/camera/depth_registered/image_raw'
res = {'w': 640, 'h': 480}

class ROS2CV():
    def __init__(self):
        self.rgb = None
        self.depth = None
        # rospy.init_node('cv_listener')

    def get_images(self, show=False):
        start = time.time()
        self.rgb = rospy.wait_for_message(rgb_img_topic, Image, timeout=2)
        self.depth = rospy.wait_for_message(depth_img_topic, Image, timeout=2)
        
        # Processing rgb image
        rgb_img = np.frombuffer(self.rgb.data, dtype = np.uint8)
        r,g,b = rgb_img[0::3], rgb_img[1::3],  rgb_img[2::3]
        r,g,b = [np.array(np.split(channel, res['h'])) for channel in (r, g, b)]
        rgb_img = cv2.merge((b,g,r))
        
        
        # Processing depth image
        depth_img = np.frombuffer(self.depth.data, dtype = np.float32)
        depth_img = np.array(np.split(depth_img, res['h']))
        depth_img_array = np.array(depth_img, dtype = np.dtype('f8'))
        depth_img = cv2.normalize(depth_img_array, depth_img_array, 0, 1, cv2.NORM_MINMAX)
        depth_img = np.array(depth_img)
        
        if show:
            cv2.imshow("rgb_img", rgb_img)
            cv2.imshow("depth_img", depth_img)
            cv2.waitKey(1)
        
        # framerate
        framerate = 1/(time.time()-start)
        print(f"framerate: {framerate}")
        return rgb_img, depth_img