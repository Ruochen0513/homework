#!/usr/bin/env python3
# 用于图像显示的ROS节点
from cv_bridge import CvBridge, CvBridgeError
import rospy
import cv2
import time
from sensor_msgs.msg import Image

def callback(data):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print(e)

if __name__ == '__main__':
    rospy.init_node('take_photo', anonymous=False)
    print("1")
    img_topic = '/ip_camera/image_raw'
    # img_topic = '/camera/rgb/image_raw'
    print("1")
    image_sub = rospy.Subscriber(img_topic, Image, callback)
    print("1")
    rospy.sleep(10)