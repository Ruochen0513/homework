#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time

def wait_for_services():
    """等待ASR和TTS服务完全启动"""
    rospy.init_node('wait_for_services', anonymous=True)
    
    rospy.loginfo("等待ASR和TTS服务启动...")
    
    # 等待ASR服务
    try:
        rospy.wait_for_service('start_asr', timeout=30)
        rospy.wait_for_service('stop_asr', timeout=30)
        rospy.loginfo("ASR服务已启动")
    except rospy.ROSException:
        rospy.logwarn("ASR服务启动超时，但继续启动其他节点")
    
    # 等待TTS话题出现
    try:
        # 检查TTS话题是否可用
        topics = rospy.get_published_topics()
        tts_topic_found = any('/tts_text' in topic[0] for topic in topics)
        
        timeout = 30
        start_time = time.time()
        while not tts_topic_found and (time.time() - start_time) < timeout:
            time.sleep(1)
            topics = rospy.get_published_topics()
            tts_topic_found = any('/tts_text' in topic[0] for topic in topics)
        
        if tts_topic_found:
            rospy.loginfo("TTS服务已启动")
        else:
            rospy.logwarn("TTS服务启动超时，但继续启动其他节点")
            
    except Exception as e:
        rospy.logwarn(f"检查TTS服务时出错: {e}")
    
    # 额外等待2秒确保服务完全初始化
    rospy.loginfo("等待服务完全初始化...")
    time.sleep(2)
    
    rospy.loginfo("服务等待完成，可以启动其他节点")

if __name__ == '__main__':
    try:
        wait_for_services()
    except rospy.ROSInterruptException:
        pass 