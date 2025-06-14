#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

class IPCameraNode:
    def __init__(self):
        rospy.init_node('ip_camera_publisher', anonymous=True)
        
        self.bridge = CvBridge()
        self.current_frame = None
        
        # 初始化发布者
        self.image_pub = rospy.Publisher("/ip_camera/image_raw", Image, queue_size=10)
        self.visual_info_pub = rospy.Publisher("visual_info", Image, queue_size=10)
        
        # 初始化订阅者 - 统一话题名称
        rospy.Subscriber("visual_command", String, self.cmd_callback)
        
        # 替换为你的IP摄像头URL
        self.cap = cv2.VideoCapture("http://192.168.189.69:11311/videofeed?mjpeg=1")
        
        if not self.cap.isOpened():
            rospy.logwarn("无法打开摄像头，尝试使用默认摄像头")
            self.cap = cv2.VideoCapture(0)
        
        rospy.loginfo("IP摄像头节点已启动")
    
    def cmd_callback(self, msg):
        """处理视觉识别指令"""
        if msg.data.lower() == "true" and self.current_frame is not None:
            # 当接收到"true"时，发布当前帧到visual_info话题
            try:
                visual_image = self.bridge.cv2_to_imgmsg(self.current_frame, "bgr8")
                self.visual_info_pub.publish(visual_image)
                rospy.loginfo("图像已捕获并发布到visual_info话题")
            except Exception as e:
                rospy.logerr(f"图像转换失败: {e}")
        else:
            rospy.logwarn("未收到有效指令或当前帧为空")
    
    def run(self):
        """运行摄像头循环"""
        rate = rospy.Rate(30)  # 30Hz
        
        while not rospy.is_shutdown():
            try:
                ret, frame = self.cap.read()
                if ret:
                    self.current_frame = frame  # 保存当前帧
                    ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                    self.image_pub.publish(ros_image)
                else:
                    rospy.logwarn_throttle(5, "无法读取摄像头帧")
            except Exception as e:
                rospy.logerr_throttle(5, f"摄像头读取错误: {e}")
            
            rate.sleep()
        
        self.cap.release()
        rospy.loginfo("摄像头资源已释放")

def main():
    try:
        node = IPCameraNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("IP摄像头节点关闭")

if __name__ == '__main__':
    main()