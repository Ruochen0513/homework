#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import asyncio
import websockets
import uuid
import json
import threading
import struct
import gzip
import copy
from std_msgs.msg import String

import pyaudio
import queue
import time
import base64


class TTSNode:
    def __init__(self):
        rospy.init_node('tts_node', anonymous=True)
        
        # 订阅语音合成话题
        rospy.Subscriber('tts_text', String, self.tts_callback)
        
        # 添加TTS状态发布者
        self.tts_status_pub = rospy.Publisher('tts_status', String, queue_size=10)
        
        # 从环境变量读取鉴权信息
        self.app_id = rospy.get_param('~app_id', '7709498845')
        self.access_key = rospy.get_param('~access_key', 'xxx')
        self.voice_type = rospy.get_param('~voice_type', 'zh_male_shaonianzixin_moon_bigtts')
        
        # 使用标准的大模型语音合成WebSocket API
        self.api_url = 'wss://openspeech.bytedance.com/api/v1/tts/ws_binary'
        
        # 音频播放参数 - 使用PCM格式
        self.format = pyaudio.paInt16  # 16-bit PCM
        self.channels = 1              # 单声道
        self.rate = 16000              # 采样率 - 使用更通用的16kHz
        self.chunk = 2048              # 优化的缓冲区大小
        
        # 协议头定义
        self.default_header = bytearray(b'\x11\x10\x11\x00')
        
        # 消息类型定义
        self.MESSAGE_TYPES = {11: "audio-only server response", 12: "frontend server response", 15: "error message from server"}
        self.MESSAGE_TYPE_SPECIFIC_FLAGS = {0: "no sequence number", 1: "sequence number > 0",
                                           2: "last message from server (seq < 0)", 3: "sequence number < 0"}
        
        # 初始化音频播放器
        self.audio = pyaudio.PyAudio()
        self.stream = None
        self.audio_lock = threading.Lock()
        
        # 音频队列
        self.audio_queue = queue.Queue(maxsize=100)
        
        # 播放状态管理
        self.is_playing = False
        self.tts_finished = True  # 标志TTS数据是否接收完毕
        
        # 启动音频播放线程
        self.play_thread = threading.Thread(target=self.audio_playback_thread)
        self.play_thread.daemon = True
        self.play_thread.start()
        
        rospy.loginfo("语音合成节点已启动")
        rospy.loginfo(f"当前使用的音色: {self.voice_type}")

    def init_audio_stream(self):
        """初始化音频流"""
        with self.audio_lock:
            try:
                if self.stream and not self.stream.is_stopped():
                    self.stream.stop_stream()
                if self.stream:
                    self.stream.close()
                
                # 选择音频设备
                output_device = None
                for i in range(self.audio.get_device_count()):
                    device_info = self.audio.get_device_info_by_index(i)
                    if device_info['maxOutputChannels'] > 0:
                        if 'pulse' in device_info['name'].lower():
                            output_device = i
                            break
                        elif 'default' in device_info['name'].lower() and output_device is None:
                            output_device = i
                
                if output_device is None:
                    output_device = self.audio.get_default_output_device_info()['index']
                
                # 尝试不同的采样率
                supported_rates = [16000, 22050, 44100, 48000]
                for rate in supported_rates:
                    try:
                        self.stream = self.audio.open(
                            format=self.format,
                            channels=self.channels,
                            rate=rate,
                            output=True,
                            output_device_index=output_device,
                            frames_per_buffer=self.chunk
                        )
                        if rate != self.rate:
                            self.rate = rate
                        break
                    except Exception:
                        continue
                
                if not self.stream:
                    raise Exception("无法创建音频流")
                    
                rospy.loginfo("音频流已成功初始化")
                return True
            except Exception as e:
                rospy.logerr(f"音频流初始化失败: {e}")
                self.stream = None
                return False

    def audio_playback_thread(self):
        """音频播放线程"""
        self.init_audio_stream()
        
        while not rospy.is_shutdown():
            try:
                # 确保音频流有效
                if self.stream is None:
                    rospy.loginfo("音频流为空，尝试重新初始化")
                    self.init_audio_stream()
                    if self.stream is None:
                        time.sleep(0.1)
                        continue

                data = self.audio_queue.get(timeout=0.1) 

                if data == b'':  # 中断标记
                    rospy.loginfo("收到音频中断标记")
                    if self.is_playing:
                        self.tts_status_pub.publish("finished")
                        self.is_playing = False
                    self.audio_queue.task_done() 
                    continue

                # 处理音频数据
                if self.stream is None:
                    rospy.logerr("音频流为空，跳过音频块")
                    self.audio_queue.task_done()
                    continue

                if not self.is_playing:
                    self.is_playing = True
                    rospy.loginfo("TTS开始播放")
                    self.tts_status_pub.publish("playing")
                
                self.stream.write(data)
                self.audio_queue.task_done()

            except queue.Empty:
                # 队列为空，检查是否需要结束播放
                if self.is_playing and self.tts_finished:
                    rospy.loginfo("音频队列为空且TTS数据接收完毕，等待播放完成")
                    try:
                        # 等待内部缓冲区播放完毕
                        timeout_sec = 2.0
                        start = time.time()
                        while self.stream and self.stream.is_active():
                            if rospy.is_shutdown():
                                break
                            if time.time() - start > timeout_sec:
                                rospy.logwarn(f"音频流超时 {timeout_sec}s，强制结束")
                                break
                            time.sleep(0.05)
                    except Exception as e:
                        rospy.logwarn(f"等待音频流完成时出错: {e}")
                    finally:
                        if self.is_playing:
                            rospy.loginfo("TTS播放完成，发布finished状态")
                            self.tts_status_pub.publish("finished")
                            self.is_playing = False
                elif self.is_playing and not self.tts_finished:
                    # 还在播放，但TTS数据还没接收完，继续等待
                    pass
                elif not self.is_playing and self.tts_finished:
                    # 没有在播放，但数据已接收完（可能是很短的音频），确保发布finished状态
                    rospy.loginfo("检测到短音频播放完成，发布finished状态")
                    self.tts_status_pub.publish("finished")
                    self.tts_finished = False  # 重置状态避免重复发布
                time.sleep(0.01)

            except Exception as e:
                rospy.logerr(f"音频播放线程错误: {e}")
                if self.is_playing: 
                    self.tts_status_pub.publish("finished")
                    self.is_playing = False
                
                # 清理并重新初始化音频流
                if self.stream:
                    try:
                        if self.stream.is_active(): 
                            self.stream.stop_stream()
                        self.stream.close()
                    except Exception as e_close:
                        rospy.logerr(f"关闭音频流时出错: {e_close}")
                    finally:
                        self.stream = None 
                
                self.init_audio_stream()
                time.sleep(0.1)

    def create_request_frame(self, text):
        """创建请求帧"""
        request_json = {
            "app": {
                "appid": self.app_id,
                "token": self.access_key,
                "cluster": "volcano_tts"
            },
            "user": {
                "uid": str(uuid.uuid4())
            },
            "audio": {
                "voice_type": self.voice_type,
                "encoding": "pcm",
                "speed_ratio": 1.0,
                "volume_ratio": 1.0,
                "pitch_ratio": 1.0,
                "rate": self.rate
            },
            "request": {
                "reqid": str(uuid.uuid4()),
                "text": text,
                "text_type": "plain",
                "operation": "submit"
            }
        }
        
        payload_bytes = str.encode(json.dumps(request_json))
        payload_bytes = gzip.compress(payload_bytes)
        
        full_client_request = bytearray(self.default_header)
        full_client_request.extend((len(payload_bytes)).to_bytes(4, 'big'))
        full_client_request.extend(payload_bytes)
        
        return bytes(full_client_request)

    def parse_response(self, res):
        """解析响应"""
        if len(res) < 4:
            return None, False
        
        protocol_version = res[0] >> 4
        header_size = res[0] & 0x0f
        message_type = res[1] >> 4
        message_type_specific_flags = res[1] & 0x0f
        serialization_method = res[2] >> 4
        message_compression = res[2] & 0x0f
        reserved = res[3]
        
        header_bytes = header_size * 4
        if len(res) < header_bytes:
            return None, False
            
        payload = res[header_bytes:]
        
        if message_type == 0xb:  # audio-only server response
            if message_type_specific_flags == 0:
                return None, False
            else:
                if len(payload) < 8:
                    return None, False
                    
                sequence_number = int.from_bytes(payload[:4], "big", signed=True)
                payload_size = int.from_bytes(payload[4:8], "big", signed=False)
                audio_data = payload[8:]
                
                is_last = sequence_number < 0
                return audio_data, is_last
                
        elif message_type == 0xf:  # error message
            if len(payload) >= 8:
                code = int.from_bytes(payload[:4], "big", signed=False)
                msg_size = int.from_bytes(payload[4:8], "big", signed=False)
                error_msg = payload[8:]
                if message_compression == 1:
                    error_msg = gzip.decompress(error_msg)
                error_msg = str(error_msg, "utf-8")
                rospy.logerr(f"服务器错误 {code}: {error_msg}")
            return None, True
        
        return None, False

    def process_pcm_audio(self, pcm_data):
        """处理PCM音频数据"""
        try:
            if not pcm_data:
                return
                
            chunk_size = self.chunk * 2
            
            for i in range(0, len(pcm_data), chunk_size):
                chunk = pcm_data[i:i+chunk_size]
                if len(chunk) > 0:
                    try:
                        self.audio_queue.put(chunk, timeout=1.0)
                    except queue.Full:
                        rospy.logwarn("音频队列已满")
                        break
                        
        except Exception as e:
            rospy.logerr(f"PCM音频处理失败: {e}")

    async def tts_request(self, text):
        """语音合成请求"""
        try:
            headers = {
                'Authorization': f'Bearer; {self.access_key}'
            }
            
            async with websockets.connect(self.api_url, extra_headers=headers, ping_interval=None) as ws:
                request_frame = self.create_request_frame(text)
                await ws.send(request_frame)
                
                audio_finished = False
                while not audio_finished and not rospy.is_shutdown():
                    try:
                        response = await asyncio.wait_for(ws.recv(), timeout=10.0)
                        if not response:
                            break
                            
                        audio_data, is_last = self.parse_response(response)
                        
                        if audio_data:
                            self.process_pcm_audio(audio_data)
                        
                        if is_last:
                            audio_finished = True
                            
                    except asyncio.TimeoutError:
                        rospy.logwarn("接收响应超时")
                        break
                    except Exception as e:
                        rospy.logerr(f"接收响应时发生错误: {e}")
                        break
                
            rospy.loginfo("TTS数据接收完成")

        except Exception as e:
            rospy.logerr(f"语音合成请求失败: {e}")

    def tts_callback(self, msg):
        """处理TTS请求"""
        text = msg.data
        rospy.loginfo(f"收到语音合成请求: {text}")
        
        # 如果正在播放，中断当前播放
        if self.is_playing:
            rospy.loginfo("中断当前TTS播放")
            # 清空音频队列
            cleared_count = 0
            while not self.audio_queue.empty():
                try:
                    self.audio_queue.get_nowait()
                    self.audio_queue.task_done()
                    cleared_count += 1
                except queue.Empty:
                    break
            
            if cleared_count > 0:
                rospy.loginfo(f"清空了{cleared_count}个音频包")
            
            # 插入中断标记
            try:
                self.audio_queue.put(b'', timeout=0.1)
            except queue.Full:
                rospy.logwarn("无法插入中断标记")
        
        # 重置状态
        self.tts_finished = False

        # 异步处理TTS
        def run_async_tts():
            new_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(new_loop)
            try:
                new_loop.run_until_complete(self.tts_request(text))
                rospy.loginfo(f"TTS请求完成: {text[:20]}...")
            except Exception as e:
                rospy.logerr(f"TTS异步处理发生错误: {e}")
                # 如果TTS请求失败，确保发布finished状态
                if self.is_playing:
                    self.tts_status_pub.publish("finished")
                    self.is_playing = False
            finally:
                self.tts_finished = True
                new_loop.close()
                # 给播放线程一些时间来检查状态
                time.sleep(0.1)
        
        tts_thread = threading.Thread(target=run_async_tts)
        tts_thread.daemon = True
        tts_thread.start()

    def shutdown(self):
        """关闭节点"""
        rospy.loginfo("TTS节点正在关闭...")
        
        self.tts_status_pub.publish("stopped")
        
        with self.audio_lock:
            if self.stream:
                try:
                    self.stream.stop_stream()
                    self.stream.close()
                except Exception as e:
                    rospy.logerr(f"关闭音频流时出错: {e}")
        if self.audio:
            try:
                self.audio.terminate()
            except Exception as e:
                rospy.logerr(f"终止PyAudio时出错: {e}")
        
        rospy.loginfo("TTS节点关闭完成")


if __name__ == '__main__':
    try:
        tts_node = TTSNode()
        rospy.on_shutdown(tts_node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS中断，节点关闭")
    except Exception as e:
        rospy.logerr(f"TTS节点启动时发生异常: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())