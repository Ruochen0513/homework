#!/usr/bin/env python3
import rospy
from rosgraph_msgs.msg import Log
from std_msgs.msg import String
import time

class NavigationMonitor:
    def __init__(self):
        rospy.init_node('navigation_monitor', anonymous=True)
        
        # Publisher for TTS
        self.tts_pub = rospy.Publisher('tts_text', String, queue_size=10)
        
        # Subscriber for rosout
        rospy.Subscriber('/rosout_agg', Log, self.log_callback)
        
        # Rate limiter
        self.last_speech_time = 0
        self.speech_cooldown = 20  # seconds
        
        # Keywords to detect recovery behaviors
        self.recovery_keywords = [
            "Starting recovery behavior",
            "Rotate recovery",
            "Clearing costmap"
        ]
        
        rospy.loginfo("Navigation monitor started.")

    def log_callback(self, msg):
        # We are interested in messages from move_base
        if msg.name == '/move_base':
            # Check if enough time has passed since the last announcement
            if time.time() - self.last_speech_time > self.speech_cooldown:
                # Check for recovery keywords
                for keyword in self.recovery_keywords:
                    if keyword in msg.msg:
                        rospy.loginfo(f"Detected recovery behavior: {msg.msg}")
                        
                        # Announce that we are trying to recover
                        self.tts_pub.publish("检测到障碍物，正在尝试绕行，请稍候。")
                        
                        # Update the timestamp
                        self.last_speech_time = time.time()
                        
                        # Break after the first match to avoid multiple announcements for one log message
                        break

if __name__ == '__main__':
    try:
        monitor = NavigationMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 