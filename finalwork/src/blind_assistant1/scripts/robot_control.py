#!/usr/bin/env python
import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
w/s  :increase/decrease linear speed 0.01
a/d  :increase/decrease angular speed 0.1
---------------------------
x    :speed to 0
CTRL-C to quit
"""

moveBindings = {
        'w':(0.2,0,0,0),
        's':(-0.2,0,0,0),
        'a':(0,0,0,0.1),
        'd':(0,0,0,-0.1),
           }

speedBindings={
        'x':(0,0,0,0),
          }

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed,turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

# 添加命令回调函数
def command_callback(data):
    global x, y, z, th
    command = data.data.lower()
    
    if command == "前进":
        x += moveBindings['w'][0]
        y += moveBindings['w'][1]
        z += moveBindings['w'][2]
        th += moveBindings['w'][3]
        rospy.loginfo("Command: 前进 - %s", vels(x, th))
    elif command == "后退":
        x += moveBindings['s'][0]
        y += moveBindings['s'][1]
        z += moveBindings['s'][2]
        th += moveBindings['s'][3]
        rospy.loginfo("Command: 后退 - %s", vels(x, th))
    elif command == "左转":
        x += moveBindings['a'][0]
        y += moveBindings['a'][1]
        z += moveBindings['a'][2]
        th += moveBindings['a'][3]
        rospy.loginfo("Command: 左转 - %s", vels(x, th))
    elif command == "右转":
        x += moveBindings['d'][0]
        y += moveBindings['d'][1]
        z += moveBindings['d'][2]
        th += moveBindings['d'][3]
        rospy.loginfo("Command: 右转 - %s", vels(x, th))
    elif command == "停止":
        x = 0
        y = 0
        z = 0
        th = 0
        rospy.loginfo("Command: 停止 - %s", vels(x, th))
    else:
        rospy.logwarn("未知命令: %s", command)
        
    # 发布速度命令
    twist = Twist()
    twist.linear.x = x; twist.linear.y = y; twist.linear.z = z
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
    pub.publish(twist)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    
    pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size = 1)
    rospy.init_node('teleop_twist_keyboard')
    
    # 创建订阅者订阅 robot_command 话题
    rospy.Subscriber('robot_command', String, command_callback)
    
    # 设置全局变量，以便在回调函数中使用
    global x, y, z, th
    speed = rospy.get_param("~speed", 0)
    turn = rospy.get_param("~turn", 0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed,turn))
        
        # 保持键盘控制功能
        while(1):
            key = getKey()
            if key in moveBindings.keys():
                x = x+moveBindings[key][0]
                y = y+moveBindings[key][1]
                z = z+moveBindings[key][2]
                th = th+moveBindings[key][3]
                print(vels(x,th))
            elif key in speedBindings.keys():
                x = 0
                y = 0
                z = 0
                th = 0
                print(vels(x,th))
            else:
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x; twist.linear.y = y; twist.linear.z = z;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th;
            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)