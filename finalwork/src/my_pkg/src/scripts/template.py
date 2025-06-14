#!/usr/bin/env python3
#用于图像匹配的ROS节点
from cv_bridge import CvBridge, CvBridgeError
import rospy
import cv2
import numpy as np
import time
from sensor_msgs.msg import Image
import glob
import os

def call_back(data):
    try:
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        
        # 显示实时图像
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(1)

        # 获取最新的模板图片
        template_files = glob.glob("/home/amadeus/桌面/template.png")
        if not template_files:
            print("未找到模板图片")
            return
            
        latest_template = max(template_files, key=os.path.getctime)
        template = cv2.imread(latest_template)
        
        if template is None:
            print(f"无法读取模板图片: {latest_template}")
            return
            
        # 显示模板图片
        cv2.imshow("Template Window", template)
        cv2.waitKey(1)

        # 进行模板匹配
        h,w = template.shape[:2]
        # 使用TM_CCOEFF_NORMED算法，返回值范围在[-1,1]之间，1表示完全匹配
        res = cv2.matchTemplate(cv_image, template, cv2.TM_CCOEFF_NORMED)
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

        # 降低匹配阈值到0.3
        threshold = 0.3
        print(f"当前匹配度: {max_val:.3f}")  # 打印匹配度

        if max_val >= threshold:
            # 在图像上标记匹配位置
            top_left = max_loc  # 注意：对于TM_CCOEFF_NORMED，使用max_loc
            bottom_right = (top_left[0] + w, top_left[1] + h)
            cv2.rectangle(cv_image, top_left, bottom_right, (0, 255, 0), 2)
            
            # 显示匹配度
            match_text = f"Match: {max_val:.2f}"
            cv2.putText(cv_image, match_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            # 显示未匹配信息
            cv2.putText(cv_image, "No match", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # 显示匹配结果
        cv2.imshow("Detected Window", cv_image)
        cv2.waitKey(1)
        
    except CvBridgeError as e:
        print(e)
    except Exception as e:
        print(f"发生错误: {str(e)}")

if __name__ == '__main__':
    rospy.init_node('image_matcher', anonymous=True)
    #img_topic = '/usb_cam/image_raw'
    img_topic = '/camera/image_raw'
    image_sub = rospy.Subscriber(img_topic, Image, call_back)
    
    print("正在运行图像匹配...")
    print("按'q'键退出")
    rospy.spin()
    
    cv2.destroyAllWindows()