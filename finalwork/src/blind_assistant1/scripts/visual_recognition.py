#!/usr/bin/env python3
import os
from typing import Any, List, cast
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import base64
from volcenginesdkarkruntime import Ark
import threading

class VisualRecognitionNode:
    def __init__(self):
        rospy.init_node('visual_recognition', anonymous=True)
        # Get image topics and model ID from ROS parameters
        raw_topics = rospy.get_param('~image_topics', ['visual_info'])
        self.topics: List[str] = [raw_topics] if isinstance(raw_topics, str) else raw_topics
        
        raw_model = rospy.get_param('~model', 'doubao-1.5-vision-pro-250328')
        self.model: str = str(raw_model)
        
        self.bridge = CvBridge()
        self.buffers = {}
        self.expected_count = len(self.topics)
        
        # Publisher for TTS
        self.pub = rospy.Publisher('tts_text', String, queue_size=10)
        # Initialize Ark client
        self.client = Ark(api_key='xxx')
        
        # Subscribe to all image topics
        for topic in self.topics:
            rospy.Subscriber(topic, Image, self.image_callback)
        
        # Subscribe to visual command topic
        rospy.Subscriber('visual_command', String, self.visual_command_callback)
        
        # 添加状态控制
        self.should_process = False
        
        rospy.loginfo(f"Subscribed to topics {self.topics}, using model {self.model}")
        rospy.loginfo("视觉识别节点等待visual_command指令")

    def visual_command_callback(self, msg):
        """处理视觉识别指令"""
        if msg.data.lower() == "true":
            rospy.loginfo("收到场景识别指令，开始处理图像")
            self.should_process = True
        else:
            rospy.logwarn(f"未知的视觉指令: {msg.data}")

    def image_callback(self, msg: Image):
        # 只有在收到视觉识别指令时才处理图像
        if not self.should_process:
            return
        
        ts = msg.header.stamp.to_nsec()
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ret, buf = cv2.imencode('.jpg', cv_img)
            if not ret:
                raise ValueError('JPEG encoding failed')
            raw_b64 = base64.b64encode(buf.tobytes()).decode('utf-8')
            data_uri = f"data:image/jpeg;base64,{raw_b64}"
        except Exception as e:
            rospy.logerr(f"Image conversion error: {e}")
            return
            
        lst = self.buffers.setdefault(ts, [])
        lst.append(data_uri)
        
        if len(lst) >= self.expected_count:
            images = self.buffers.pop(ts)
            # 重置处理状态，避免连续处理
            self.should_process = False
            # Process in a new thread to keep the callback non-blocking
            threading.Thread(target=self.process_images_stream, args=(images,)).start()

    def process_images_stream(self, images: List[str]):
        content: List[dict] = []
        for img_b64 in images:
            content.append({'type': 'image_url', 'image_url': {'url': img_b64}})
        content.append({'type': 'text', 'text': '请描述你看到了什么。控制在40字以内'})
        
        rospy.loginfo("收到场景识别请求，正在进行视觉识别...")
        try:
            # Use stream=True for streaming response
            resp_stream = cast(Any, self.client.chat.completions.create(
                model=self.model,
                messages=[{'role': 'user', 'content': content}],
                stream=True
            ))

            buffer = ""
            for chunk in resp_stream:
                chunk_content = chunk.choices[0].delta.content
                if chunk_content:
                    buffer += chunk_content
                    # Publish sentence by sentence
                    if any(p in buffer for p in ["。", "！", "？", "...", "\n"]):
                        sentence, _, remainder = buffer.partition(next(p for p in ["。", "！", "？", "...", "\n"] if p in buffer))
                        sentence_to_publish = sentence.strip()
                        if sentence_to_publish:
                            self.pub.publish(sentence_to_publish)
                            rospy.loginfo(f"Published sentence: {sentence_to_publish}")
                        buffer = remainder

            # Publish any remaining text after the loop
            final_sentence = buffer.strip()
            if final_sentence:
                self.pub.publish(final_sentence)
                rospy.loginfo(f"Published final sentence: {final_sentence}")
                
        except Exception as e:
            rospy.logerr(f"Ark API error: {e}")
            self.pub.publish("抱歉，我在识别图像时遇到了问题。")

if __name__ == '__main__':
    try:
        node = VisualRecognitionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass