#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import re
from std_msgs.msg import String
from std_srvs.srv import SetBool
from zhipuai import ZhipuAI
import threading
import time

class CenterNode:
    def __init__(self):
        rospy.init_node('center_node', anonymous=True)
        
        # 智谱AI客户端初始化
        self.client = ZhipuAI(api_key="ca93604709044884b178da7f8604b070.83NBe82V7DRdsKBF", timeout=30.0)  
        self.model = "glm-4-flash"
        
        # System state - 统一状态管理
        self.is_processing = False
        self.is_navigating = False
        self.tts_is_playing = False
        self.asr_is_active = False
        
        # 状态锁，确保状态变更的原子性
        self.state_lock = threading.Lock()
        
        # Conversation history for multi-turn dialogue
        self.conversation_history = []
        self.history_lock = threading.Lock()
        
        # System prompt defines the robot's capabilities and response format
        self.system_prompt = """你现在是一个盲人引导机器人，具有运动控制/导航/机械臂夹取/场景识别的功能，请分析用户的要求，如果你认为用户的要求属于运动控制/导航/机械臂夹取/场景识别中的一类，请按照下属json格式响应，请严格按以下JSON格式响应：
{
  "content": "回复内容",  # 当用户的要求为"场景识别"时，"content"保持空字符串即可，不要回复任何内容
  "request": "运动控制/导航/机械臂夹取/场景识别",        # 仅用户的要求为"运动控制"或者"导航"或者"机械臂夹取"或者"场景识别"时需要，选择"运动控制"或者"导航"或者"机械臂夹取"或者"场景识别"四者之一输出，其他情况设为"NONE"
  "text": "前进/后退/左转/右转/停止",  # 仅"request"为运动控制时需要，选择"前进"或者"后退"或者"左转"或者"右转"或者"停止"五者之一输出，其他情况设为"NONE"
  "place": "116"  # 仅导航时需要        # 仅"request"为导航时需要，且只有两个选项：327，302，303,且只响应数字即可，其他情况设为"NONE"  
  "arm": "true"   # 仅机械臂夹取时需要，且只要"request"为机械臂夹取，"arm"即为true， 其他情况设为"NONE"    
  "visual": "true"  # 仅场景识别时需要，且只要"request"为场景识别，"visual"即为true，其他情况设为"NONE"       
}
例如：
如果用户说"请你介绍一下你自己"，你的响应应该是：
{
  "content": "你好，我是一个盲人引导机器人，能够帮助你进行运动控制、导航、机械臂夹取和场景识别。",
  "request": "NONE",
  "text": "NONE",
  "place": "NONE",
  "arm": "NONE",
  "visual": "NONE"
}
如果用户说"请你前进"，你的响应应该是：
{
  "content": "好的，我将前进",
  "request": "运动控制",
  "text": "前进",
  "place": "NONE",
  "arm": "NONE",
  "visual": "NONE"
}
如果用户说"请你去327"，你的响应应该是：
{
  "content": "好的，我将带您前往327房间。",
  "request": "导航",
  "text": "NONE",
  "place": "327",
  "arm": "NONE",
  "visual": "NONE"
}
如果用户说"请你带我去115"，你的响应应该是：
{
  "content": "好的，我将带你去115教室",
  "request": "导航",
  "text": "NONE",
  "place": "NONE",
  "arm": "NONE",
  "visual": "NONE"
}
如果用户说"请帮我夹一瓶水"，你的响应应该是：
{
  "content": "好的，我将帮你夹一瓶水",
  "request": "机械臂夹取",
  "text": "NONE",
  "place": "NONE",
  "arm": "true",
  "visual": "NONE"
}
如果用户说"请帮我描述一下眼前的景象"或者类似，你的响应应该是：
{
  "content": "",   # 不用回答，保持空字符串即可
  "request": "场景识别",
  "text": "NONE",
  "place": "NONE",
  "arm": "NONE",
  "visual": "true"
}
"""
        
        # Subscribers
        rospy.Subscriber('raw_text', String, self.voice_command_callback)
        rospy.Subscriber('tts_status', String, self.tts_status_callback)
        rospy.Subscriber('navigation_status', String, self.nav_status_callback)
        
        # Publishers - 统一话题名称
        self.command_pub = rospy.Publisher('robot_command', String, queue_size=10, latch=True)
        self.nav_pub = rospy.Publisher('navigation_command', String, queue_size=10)
        self.arm_pub = rospy.Publisher('arm_command', String, queue_size=10)
        self.visual_pub = rospy.Publisher('visual_command', String, queue_size=10, latch=True)
        self.speech_pub = rospy.Publisher('tts_text', String, queue_size=10)
        
        # Service clients for ASR control - 只有center_node控制ASR
        rospy.loginfo("等待ASR服务...")
        rospy.wait_for_service('start_asr')
        rospy.wait_for_service('stop_asr')
        self.start_asr_client = rospy.ServiceProxy('start_asr', SetBool)
        self.stop_asr_client = rospy.ServiceProxy('stop_asr', SetBool)
        rospy.loginfo("ASR服务已连接")
        
        # 初始化对话历史
        self.initialize_history()
        
        # 启动ASR
        self.start_asr()

    def initialize_history(self):
        """Initializes or resets the conversation history."""
        with self.history_lock:
            self.conversation_history = [{"role": "system", "content": self.system_prompt}]

    def add_to_history(self, role, content):
        """Adds a message to the history and manages its size."""
        with self.history_lock:
            self.conversation_history.append({"role": role, "content": content})
            # Keep the last 10 turns (system prompt + 5 user/assistant pairs)
            if len(self.conversation_history) > 11:
                # Keep the system prompt and the last 10 messages
                self.conversation_history = [self.conversation_history[0]] + self.conversation_history[-10:]

    def nav_status_callback(self, msg):
        """处理导航状态更新"""
        with self.state_lock:
            was_navigating = self.is_navigating
            self.is_navigating = (msg.data == "active")
            
            if self.is_navigating and not was_navigating:
                rospy.loginfo("导航开始，停止ASR以避免干扰")
                self._stop_asr_internal()
            elif not self.is_navigating and was_navigating:
                rospy.loginfo("导航结束")
                # 导航结束后，如果没有其他任务在进行，重启ASR
                if not self.is_processing and not self.tts_is_playing:
                    rospy.loginfo("导航结束，重新启动ASR")
                    self._start_asr_internal()

    def start_asr(self):
        """公共方法：启动ASR"""
        with self.state_lock:
            self._start_asr_internal()

    def stop_asr(self):
        """公共方法：停止ASR"""
        with self.state_lock:
            self._stop_asr_internal()

    def _start_asr_internal(self):
        """内部方法：启动ASR（需要在state_lock内调用）"""
        if not self.asr_is_active and not self.tts_is_playing and not self.is_navigating:
            try:
                self.stop_asr_client(True)  # 确保先停止
                rospy.sleep(0.1)
                result = self.start_asr_client(True)
                if result.success:
                    self.asr_is_active = True
                    self.is_processing = False
                    rospy.loginfo("ASR已启动")
                else:
                    rospy.logerr(f"启动ASR失败: {result.message}")
            except Exception as e:
                rospy.logerr(f"启动ASR服务失败: {e}")

    def _stop_asr_internal(self):
        """内部方法：停止ASR（需要在state_lock内调用）"""
        if self.asr_is_active:
            try:
                result = self.stop_asr_client(True)
                if result.success:
                    self.asr_is_active = False
                    rospy.loginfo("ASR已停止")
                else:
                    rospy.logerr(f"停止ASR失败: {result.message}")
            except Exception as e:
                rospy.logerr(f"停止ASR服务失败: {e}")

    def voice_command_callback(self, msg):
        """处理语音指令"""
        with self.state_lock:
            if self.is_processing:
                rospy.logwarn("正在处理中，忽略新指令")
                return
            
            if self.is_navigating:
                rospy.logwarn("导航中，忽略语音指令")
                return
                
            self.is_processing = True
            self._stop_asr_internal()
        
        user_text = msg.data.strip()
        if not user_text:
            with self.state_lock:
                self.is_processing = False
            return

        rospy.loginfo(f"收到用户指令: {user_text}")
        self.add_to_history("user", user_text)
        
        # 使用线程处理LLM请求，避免阻塞
        threading.Thread(target=self.process_with_llm, daemon=True).start()

    def process_with_llm(self):
        """调用大语言模型处理"""
        is_json_response = False
        full_response_content = ""
        buffer = ""
        
        try:
            with self.history_lock:
                rospy.loginfo("正在向大模型发送请求...")
                response_stream = self.client.chat.completions.create(
                    model=self.model,
                    messages=self.conversation_history,
                    stream=True,
                )

            for chunk in response_stream:
                content = chunk.choices[0].delta.content
                if content is None:
                    continue

                rospy.loginfo(f"收到LLM流式响应: {content}")

                # 智能检测是否为JSON响应
                if not full_response_content and '{' in content:
                    is_json_response = True
                
                full_response_content += content
                
                if is_json_response:
                    # 缓冲整个JSON响应
                    if '}' in full_response_content:
                        try:
                            json_str = full_response_content[full_response_content.find('{'):full_response_content.rfind('}')+1]
                            data = json.loads(json_str)
                            self.execute_function_commands(data)
                            self.add_to_history("assistant", json_str)
                            return
                        except json.JSONDecodeError:
                            pass
                else:
                    # 处理流式文本
                    buffer += content
                    if any(p in buffer for p in ["。", "！", "？", "\n"]):
                        sentence_to_publish = buffer.split(next(p for p in ["。", "！", "？", "\n"] if p in buffer))[0]
                        if sentence_to_publish:
                            self.speech_pub.publish(sentence_to_publish)
                        buffer = buffer[len(sentence_to_publish)+1:]

            # 处理非JSON响应
            if not is_json_response:
                if buffer.strip():
                    self.speech_pub.publish(buffer.strip())
                self.add_to_history("assistant", full_response_content)

        except Exception as e:
            rospy.logerr(f"LLM处理失败: {e}")
            self.speech_pub.publish("抱歉，我遇到了一点问题，请您再说一遍。")
        finally:
            # 等待TTS播放完毕后重启ASR
            self._wait_and_restart_asr()

    def _wait_and_restart_asr(self):
        """等待TTS播放完毕后重启ASR（备用机制）"""
        # 等待TTS开始播放
        time.sleep(0.5)
        
        # 检查是否有TTS正在播放，如果没有，立即重启ASR
        with self.state_lock:
            if not self.tts_is_playing:
                rospy.loginfo("没有TTS播放，立即重启ASR")
                self.is_processing = False
                if not self.is_navigating:
                    self._start_asr_internal()
                return
        
        # 如果TTS正在播放，设置备用定时器作为最后保障
        def backup_restart():
            time.sleep(15)  # 15秒后检查（给TTS足够时间）
            with self.state_lock:
                if self.is_processing and not self.is_navigating and not self.tts_is_playing:
                    rospy.logwarn("TTS状态回调可能失效，备用机制重启ASR")
                    self.is_processing = False
                    self._start_asr_internal()
        
        backup_thread = threading.Thread(target=backup_restart, daemon=True)
        backup_thread.start()
        
        rospy.loginfo("主要依赖TTS状态回调处理ASR重启，已设置15秒备用机制")

    def execute_function_commands(self, data):
        """执行功能指令"""
        try:
            request_type = data.get("request")
            
            # 特殊处理：场景识别时不发布任何TTS内容，完全交给visual_recognition节点处理
            if request_type == "场景识别":
                visual_flag = data.get("visual")
                if visual_flag == "true":
                    self.visual_pub.publish("true")
                    rospy.loginfo("发送场景识别指令，由visual_recognition节点处理TTS输出")
                else:
                    rospy.logwarn(f"非法场景识别指令参数: {visual_flag}")
                return  # 直接返回，不执行后续逻辑
            
            # 其他请求类型的正常处理
            spoken_content = data.get("content")
            if spoken_content:
                self.speech_pub.publish(spoken_content)
                rospy.loginfo(f"发送TTS内容: {spoken_content}")

            # 检查是否在导航中，如果是，则阻止手动控制
            if self.is_navigating and request_type in ["运动控制", "机械臂夹取"]:
                rospy.logwarn("导航进行中，已阻止手动运动或机械臂指令")
                return

            if request_type == "运动控制":
                move_command = data.get("text")
                if move_command in ["前进", "后退", "左转", "右转", "停止"]:
                    self.command_pub.publish(move_command)
                    rospy.loginfo(f"发送运动控制指令: {move_command}")
                else:
                    rospy.logwarn(f"非法运动指令: {move_command}")
            
            elif request_type == "导航":
                nav_place = data.get("place")
                if nav_place in ["327", "302", "303"]:
                    self.nav_pub.publish(nav_place)
                    rospy.loginfo(f"发送导航指令: {nav_place}")
                else:
                    rospy.logwarn(f"非法导航地点: {nav_place}")
            
            elif request_type == "机械臂夹取":
                arm_flag = data.get("arm")
                if arm_flag == "true":
                    self.arm_pub.publish("true")
                    rospy.loginfo("发送机械臂指令")
                else:
                    rospy.logwarn(f"非法机械臂指令参数: {arm_flag}")
            

            elif request_type == "NONE":
                rospy.loginfo("处理普通对话，内容已发送至TTS")
            
            else:
                rospy.logwarn(f"未知请求类型: {request_type}")

        except Exception as e:
            rospy.logerr(f"执行功能指令出错: {e}")

    def tts_status_callback(self, msg):
        """处理TTS状态回调"""
        with self.state_lock:
            was_playing = self.tts_is_playing
            self.tts_is_playing = (msg.data == "playing")
            
            rospy.loginfo(f"TTS状态更新: {msg.data}")
            
            # 如果TTS开始播放，确保ASR已停止
            if self.tts_is_playing and not was_playing:
                self._stop_asr_internal()
            
            # 如果TTS播放结束且之前在播放，重启ASR
            elif not self.tts_is_playing and was_playing:
                rospy.loginfo("TTS播放结束，重置处理状态并重启ASR")
                self.is_processing = False  # 重置处理状态
                if not self.is_navigating:
                    rospy.loginfo("TTS结束，立即重启ASR")
                    self._start_asr_internal()
                else:
                    rospy.loginfo("导航进行中，ASR重启被推迟")


if __name__ == '__main__':
    try:
        CenterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass