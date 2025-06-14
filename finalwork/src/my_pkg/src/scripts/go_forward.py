#!/usr/bin/env python
# 用于控制小车前进的ROS节点
import rospy
from geometry_msgs.msg import Twist

if __name__ =='__main__':
    rospy.init_node('go_forward', anonymous=False)
    cmd_vel = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    move_cmd = Twist()
    
    move_cmd.linear.x = 0.2 # Move forward with a speed of 0.2 m/s
    move_cmd.angular.z = 0.0 # No rotation

    while not rospy.is_shutdown():
        cmd_vel.publish(move_cmd)
        rate.sleep()