#!/usr/bin/env python
import rospy
from homework3.msg import CustomMessage

def callback(data):
    rospy.loginfo("Received: number=%f, text=%s", data.number, data.text)

def listener():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("chatter", CustomMessage, callback)  # 订阅自定义消息
    rospy.spin()

if __name__ == '__main__':
    listener()