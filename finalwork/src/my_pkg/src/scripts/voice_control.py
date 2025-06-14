#!/usr/bin/env python
# 用于语音控制小车运动的ROS节点
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class VoiceControl:
    def __init__(self):
        rospy.init_node('voice_control', anonymous=True)
        
        # 订阅语音识别结果
        self.voice_sub = rospy.Subscriber('/iat_text', String, self.voice_callback)
        
        # 发布运动控制命令
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        
        # 创建Twist消息对象
        self.move_cmd = Twist()
        
        # 设置运动参数
        self.linear_speed = 2  # 线速度
        self.angular_speed = 0.5  # 角速度
        
        rospy.loginfo("Voice control node initialized")
    
    def voice_callback(self, msg):
        command = msg.data.lower()
        rospy.loginfo("Received voice command: %s", command)
        
        # 重置运动命令
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0
        
        # 根据语音命令设置运动参数
        if "前进" in command or " forward." in command:
            self.move_cmd.linear.x = self.linear_speed
        elif "后退" in command or "backward" in command:
            self.move_cmd.linear.x = -self.linear_speed
        elif "左转" in command or "turn left" in command:
            self.move_cmd.angular.z = self.angular_speed
        elif "右转" in command or "turn right" in command:
            self.move_cmd.angular.z = -self.angular_speed
        elif "停止" in command or "stop" in command:
            self.move_cmd.linear.x = 0.0
            self.move_cmd.angular.z = 0.0
        
        # 发布运动命令
        self.cmd_vel_pub.publish(self.move_cmd)

if __name__ == '__main__':
    try:
        voice_control = VoiceControl()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass