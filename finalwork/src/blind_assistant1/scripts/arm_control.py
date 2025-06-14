#!/usr/bin/env python
import rospy
import subprocess
import time
from std_msgs.msg import Bool, Int32, String

class ArmController:
    def __init__(self):
        # 初始化节点
        rospy.init_node('arm_controller', anonymous=True)
        
        # 创建发布者
        self.tts_pub = rospy.Publisher('tts_text', String, queue_size=10, latch=True)
        self.arm_demo_pub = rospy.Publisher('arm_action', String, queue_size=10, latch=True)
        self.nav_pub = rospy.Publisher('navigation_command', String, queue_size=10)  # 添加导航发布者
        
        # 订阅arm_command话题 - 统一话题名称
        rospy.Subscriber('arm_command', String, self.arm_command_callback)
        
        rospy.loginfo("机械臂控制器已初始化，等待指令...")
    
    def arm_command_callback(self, msg):
        """处理机械臂指令"""
        if msg.data.lower() == "true":
            rospy.loginfo("收到机械臂启动指令")
            
            # 启动机械臂演示
            self.arm_demo_pub.publish("start")
            rospy.loginfo("机械臂演示已启动")
    
            # 等待机械臂动作完成
            rospy.loginfo("等待机械臂动作完成...")
            rospy.sleep(8)
            
            # 发布完成消息
            self.tts_pub.publish("我已拾取水瓶，现在递给您")
            rospy.loginfo("机械臂动作完成，已通知TTS")
            
            # 等待TTS播放完毕后再发送导航指令
            rospy.sleep(4)
            
            # 发送导航指令回到302房间
            self.nav_pub.publish("302")
            rospy.loginfo("已发送导航指令：回到302房间")

def main():
    try:
        controller = ArmController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("机械臂控制器节点关闭")

if __name__ == '__main__':
    main()