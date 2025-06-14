#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
from std_msgs.msg import String

class SystemInfo:
    def __init__(self):
        rospy.init_node('system_info', anonymous=True)
        
        # 系统状态监控
        self.asr_status = "未知"
        self.tts_status = "未知"
        self.nav_status = "未知"
        
        # 订阅状态话题
        rospy.Subscriber('asr_status', String, self.asr_status_callback)
        rospy.Subscriber('tts_status', String, self.tts_status_callback)
        rospy.Subscriber('navigation_status', String, self.nav_status_callback)
        
        self.show_system_info()
        self.monitor_system()
    
    def asr_status_callback(self, msg):
        self.asr_status = msg.data
    
    def tts_status_callback(self, msg):
        self.tts_status = msg.data
    
    def nav_status_callback(self, msg):
        self.nav_status = msg.data
    
    def show_system_info(self):
        """显示系统启动信息"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("盲人引导机器人系统已启动")
        rospy.loginfo("=" * 60)
        rospy.loginfo("系统架构：")
        rospy.loginfo("  核心节点: center_node (统一ASR控制)")
        rospy.loginfo("  语音服务: asr.py + tts.py")
        rospy.loginfo("  功能模块: 导航 + 运动 + 机械臂 + 视觉")
        rospy.loginfo("=" * 60)
        rospy.loginfo("关键话题：")
        rospy.loginfo("  /raw_text          - ASR输出")
        rospy.loginfo("  /tts_text          - TTS输入")
        rospy.loginfo("  /tts_status        - TTS状态")
        rospy.loginfo("  /navigation_command - 导航指令")
        rospy.loginfo("  /arm_command       - 机械臂指令")
        rospy.loginfo("  /visual_command    - 视觉指令")
        rospy.loginfo("  /robot_command     - 运动指令")
        rospy.loginfo("=" * 60)
        rospy.loginfo("修复问题：")
        rospy.loginfo("  ✅ ASR控制统一管理")
        rospy.loginfo("  ✅ TTS-ASR严格互斥")
        rospy.loginfo("  ✅ 话题架构统一")
        rospy.loginfo("  ✅ 代码逻辑简化")
        rospy.loginfo("=" * 60)
        rospy.loginfo("监控命令：")
        rospy.loginfo("  rostopic list                 - 查看所有话题")
        rospy.loginfo("  rostopic echo /raw_text       - 监控ASR输出")
        rospy.loginfo("  rostopic echo /tts_status     - 监控TTS状态")
        rospy.loginfo("  rosnode list                  - 查看所有节点")
        rospy.loginfo("=" * 60)
        rospy.loginfo("系统已就绪，可以开始使用语音指令！")
        rospy.loginfo("=" * 60)
    
    def monitor_system(self):
        """监控系统状态"""
        rate = rospy.Rate(0.1)  # 10秒更新一次
        
        while not rospy.is_shutdown():
            # 检查节点状态
            try:
                nodes = rospy.get_published_topics()
                active_topics = [topic[0] for topic in nodes]
                
                # 检查关键话题是否存在
                key_topics = ['/raw_text', '/tts_text', '/tts_status', 
                             '/navigation_command', '/arm_command', '/visual_command']
                
                missing_topics = [topic for topic in key_topics if topic not in active_topics]
                
                if missing_topics:
                    rospy.logwarn(f"缺失话题: {missing_topics}")
                
                # 每60秒显示一次状态
                if int(time.time()) % 60 == 0:
                    rospy.loginfo(f"系统状态 - ASR: {self.asr_status}, TTS: {self.tts_status}, 导航: {self.nav_status}")
                
            except Exception as e:
                rospy.logwarn(f"监控系统状态时出错: {e}")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        system_info = SystemInfo()
    except rospy.ROSInterruptException:
        rospy.loginfo("系统信息节点关闭") 