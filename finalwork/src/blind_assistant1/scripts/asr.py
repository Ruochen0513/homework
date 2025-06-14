#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import os
import sys
import time
import uuid
import json
import pyaudio
import wave
import threading
import gzip
import websocket
import struct
import queue
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse

# 录音参数
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
CHUNK = 3200  # 0.2秒一包，即5帧每秒

class VolcEngineASR:
    def __init__(self):
        rospy.init_node('speech_recognition', anonymous=True)
        
        # 发布识别结果和状态
        self.voice_pub = rospy.Publisher('raw_text', String, queue_size=10)
        self.status_pub = rospy.Publisher('asr_status', String, queue_size=10)
        
        # 提供启停服务
        self.start_service = rospy.Service('start_asr', SetBool, self.start_asr_callback)
        self.stop_service = rospy.Service('stop_asr', SetBool, self.stop_asr_callback)
        
        # 从环境变量读取鉴权信息
        self.app_id = os.environ.get('VOLC_APP_ID', '4286471456')
        self.access_key = os.environ.get('VOLC_ACCESS_KEY', 'xxx')
        self.resource_id = os.environ.get('VOLC_RESOURCE_ID', 'volc.bigasr.sauc.duration')
        if not self.app_id or not self.access_key:
            rospy.logerr('请设置VOLC_APP_ID和VOLC_ACCESS_KEY环境变量！')
            sys.exit(1)

        self.ws_url = 'wss://openspeech.bytedance.com/api/v3/sauc/bigmodel'
        self.connect_id = None
        
        # 状态控制
        self.is_active = False  # 默认不激活
        self.is_recording = False
        self.is_connected = False
        self.audio_queue = queue.Queue()
        self.ws = None

        # 无声检测参数
        self.silence_threshold = 30
        self.max_silence_frames = 30
        self.silence_frames = 0 
        
        # 识别状态控制
        self.current_recognition = ""
        self.is_processing = False
        
        # 语音活动检测参数
        self.text_lock = threading.Lock()
        self.speech_activity_timer = None
        self.speech_timeout_duration = 2
        self.pending_text_to_publish = ""
        
        # 线程控制
        self.ws_thread = None
        self.record_thread = None
        
        rospy.loginfo('语音识别节点已启动 - 等待启动指令')

    def start_asr_callback(self, req):
        """启动ASR服务"""
        try:
            if not self.is_active:
                self.is_active = True
                self.reset_recognition_state()
                # 为每个新会话生成唯一的ID
                self.connect_id = str(uuid.uuid4())
                
                # 启动线程
                if not self.ws_thread or not self.ws_thread.is_alive():
                    self.ws_thread = threading.Thread(target=self.websocket_thread)
                    self.ws_thread.daemon = True
                    self.ws_thread.start()
                
                if not self.record_thread or not self.record_thread.is_alive():
                    self.record_thread = threading.Thread(target=self.recording_thread)
                    self.record_thread.daemon = True
                    self.record_thread.start()
                
                self.status_pub.publish("started")
                rospy.loginfo("ASR已启动，开始语音识别")
                return SetBoolResponse(True, "ASR启动成功")
            else:
                return SetBoolResponse(False, "ASR已经在运行")
        except Exception as e:
            rospy.logerr(f"启动ASR失败: {e}")
            return SetBoolResponse(False, f"启动失败: {e}")

    def stop_asr_callback(self, req):
        """停止ASR服务"""
        try:
            if self.is_active:
                self.is_active = False
                self.is_recording = False
                
                # 清空队列
                while not self.audio_queue.empty():
                    try:
                        self.audio_queue.get_nowait()
                    except queue.Empty:
                        break
                
                # 让websocket_thread自行关闭连接，以使服务调用非阻塞
                # if self.ws:
                #     try:
                #         self.ws.close()
                #     except:
                #         pass
                
                self.status_pub.publish("stopped")
                rospy.loginfo("ASR已停止")
                return SetBoolResponse(True, "ASR停止成功")
            else:
                return SetBoolResponse(False, "ASR已经停止")
        except Exception as e:
            rospy.logerr(f"停止ASR失败: {e}")
            return SetBoolResponse(False, f"停止失败: {e}")

    def reset_recognition_state(self):
        """重置识别状态"""
        with self.text_lock:
            self.current_recognition = ""
            self.pending_text_to_publish = ""
            self.is_processing = False
            self.silence_frames = 0
            if self.speech_activity_timer:
                self.speech_activity_timer.cancel()
                self.speech_activity_timer = None

    def is_silence(self, audio_data):
        """静音检测"""
        if not audio_data:
            return True
            
        try:
            values = []
            for i in range(0, len(audio_data), 2):
                if i+1 < len(audio_data):
                    value = int.from_bytes(audio_data[i:i+2], byteorder='little', signed=True)
                    values.append(abs(value))
            
            if values:
                avg_amplitude = sum(values) / len(values)
                return avg_amplitude < self.silence_threshold
            return True
        except Exception as e:
            rospy.logwarn(f"检测静音时出错: {e}")
            return False

    def create_header(self, msg_type, msg_flags, serialization, compression):
        """创建二进制消息头"""
        byte1 = (1 << 4) | 1
        byte2 = (msg_type << 4) | msg_flags
        byte3 = (serialization << 4) | compression
        byte4 = 0
        return bytes([byte1, byte2, byte3, byte4])

    def create_full_request(self):
        """创建初始请求"""
        req = {
            "app": {
                "service_type": "asr",
                "version": "v1"
            },
            "audio": {
                "format": "pcm",
                "sample_rate": 16000,
                "channel": 1,
                "encoding": ""
            }
        }
        json_data = json.dumps(req).encode('utf-8')
        compressed_data = gzip.compress(json_data)
        
        header = self.create_header(1, 0, 1, 1)
        size_bytes = struct.pack('>I', len(compressed_data))
        
        return header + size_bytes + compressed_data

    def create_audio_request(self, audio_data, is_last=False):
        """创建音频数据请求"""
        if not audio_data:
            compressed_data = gzip.compress(b'')
        else:
            compressed_data = gzip.compress(audio_data)
        
        flags = 2 if is_last else 0
        header = self.create_header(2, flags, 0, 1)
        size_bytes = struct.pack('>I', len(compressed_data))
        
        return header + size_bytes + compressed_data

    def extract_latest_text(self, full_text):
        """从完整识别文本中提取最新部分"""
        if not full_text:
            return ""
            
        sentences = full_text.split('。')
        sentences = [s.strip() for s in sentences if s.strip()]
        
        if not sentences:
            return full_text
            
        return sentences[-1] + '。'

    def parse_response(self, response):
        """解析服务器响应"""
        if not response or len(response) < 12:
            return None
        
        b1, b2, b3, b4 = response[0:4]
        msg_type = (b2 >> 4) & 0xF
        msg_flags = b2 & 0xF
        serialization = (b3 >> 4) & 0xF
        compression = b3 & 0xF
        
        sequence = int.from_bytes(response[4:8], byteorder='big')
        payload_size = int.from_bytes(response[8:12], byteorder='big')
        
        if len(response) < 12 + payload_size:
            return None
            
        payload = response[12:12+payload_size]
        
        if compression == 1:
            try:
                payload = gzip.decompress(payload)
            except Exception as e:
                rospy.logerr(f"解压响应失败: {e}")
                return None
        
        if serialization == 1:
            try:
                result = json.loads(payload.decode('utf-8'))
                
                with self.text_lock:
                    if result and 'result' in result and 'text' in result['result']:
                        text = result['result']['text'].strip()
                        if text:
                            latest_text = self.extract_latest_text(text)
                            if latest_text and latest_text != self.current_recognition:
                                self.current_recognition = latest_text
                                self.pending_text_to_publish = latest_text
                                
                                rospy.loginfo(f"实时识别更新: {latest_text}")
                                
                                self.is_processing = True
                                
                                if self.speech_activity_timer:
                                    self.speech_activity_timer.cancel()
                                
                                self.speech_activity_timer = threading.Timer(self.speech_timeout_duration, self.publish_final_sentence)
                                self.speech_activity_timer.start()
                
                return result
            except Exception as e:
                rospy.logerr(f"解析JSON响应失败: {e}")
                return None
        
        return None

    def publish_final_sentence(self):
        """发布最终识别的句子，之后由center_node决定是否停止ASR"""
        with self.text_lock:
            final_text = self.pending_text_to_publish
            if final_text:
                rospy.loginfo(f"最终识别结果: {final_text}")
                
                # 发布识别结果
                msg = String()
                msg.data = final_text
                self.voice_pub.publish(msg)
                rospy.loginfo(f"发布语音指令: {final_text}")
            
            self.pending_text_to_publish = ""
            self.is_processing = False

    def recording_thread(self):
        """录音线程"""
        audio = pyaudio.PyAudio()
        stream = None
        
        try:
            stream = audio.open(
                format=FORMAT, 
                channels=CHANNELS, 
                rate=RATE, 
                input=True, 
                frames_per_buffer=CHUNK,
                input_device_index=None
            )
            
            rospy.loginfo("录音线程已启动")
            
            while not rospy.is_shutdown():
                if not self.is_active:
                    time.sleep(0.1)
                    continue
                    
                if not self.is_recording:
                    self.is_recording = True
                    rospy.loginfo("开始录音...")
                
                try:
                    audio_data = stream.read(CHUNK, exception_on_overflow=False)
                    
                    is_silent_chunk = self.is_silence(audio_data)
                    
                    with self.text_lock:
                        if is_silent_chunk:
                            self.silence_frames += 1
                            if self.silence_frames >= self.max_silence_frames:
                                if not self.is_processing and self.is_connected:
                                    self.audio_queue.put((b'', True))
                                    rospy.loginfo("检测到长时间静音，发送结束帧")
                                    self.current_recognition = ""
                                self.silence_frames = 0
                        else:
                            self.silence_frames = 0
                        
                        if self.is_connected and not is_silent_chunk:
                            self.audio_queue.put((audio_data, False))
                    
                except Exception as e:
                    if self.is_active:
                        rospy.logerr(f"录音过程出错: {e}")
                    time.sleep(0.1)
        
        except Exception as e:
            rospy.logerr(f"初始化录音设备时出错: {e}")
        
        finally:
            if stream:
                stream.stop_stream()
                stream.close()
            audio.terminate()
            rospy.loginfo("录音线程已停止")

    def websocket_thread(self):
        """WebSocket通信线程"""
        while not rospy.is_shutdown():
            if not self.is_active:
                time.sleep(0.1)
                continue
                
            try:
                headers = {
                    'X-Api-App-Key': self.app_id,
                    'X-Api-Access-Key': self.access_key,
                    'X-Api-Resource-Id': self.resource_id,
                    'X-Api-Connect-Id': self.connect_id
                }
                
                self.ws = websocket.create_connection(
                    self.ws_url, 
                    header=[f'{k}: {v}' for k, v in headers.items()]
                )
                self.is_connected = True
                rospy.loginfo("WebSocket连接已建立")
                
                initial_request = self.create_full_request()
                self.ws.send_binary(initial_request)
                
                response = self.ws.recv()
                result = self.parse_response(response)
                if result:
                    rospy.loginfo(f"初始响应: {result}")
                
                while self.is_active and not rospy.is_shutdown():
                    try:
                        audio_data, is_last = self.audio_queue.get(timeout=0.5)
                    except queue.Empty:
                        continue
                    
                    if not self.is_active:
                        break
                    
                    audio_request = self.create_audio_request(audio_data, is_last)
                    self.ws.send_binary(audio_request)
                    
                    self.ws.settimeout(2.0)
                    try:
                        response = self.ws.recv()
                        self.parse_response(response)
                        
                        if is_last:
                            rospy.loginfo("结束帧已发送，准备进行下一次识别。")
                            break
                        
                    except websocket.WebSocketTimeoutException:
                        rospy.logwarn("WebSocket响应超时，继续处理下一个音频包")
                        continue
                    except Exception as e:
                        if self.is_active:
                            if is_last and isinstance(e, (websocket.WebSocketConnectionClosedException, ConnectionResetError)):
                                rospy.loginfo("服务器在最终响应后关闭连接，这是预期的。")
                            else:
                                rospy.logerr(f"接收响应出错: {e}")
                        break
            
            except Exception as e:
                if self.is_active:
                    rospy.logerr(f"WebSocket连接出错: {e}")
            
            finally:
                if self.ws:
                    try:
                        self.ws.close()
                    except:
                        pass
                self.is_connected = False
                self.ws = None
                if self.is_active:
                    rospy.loginfo("WebSocket连接已关闭，将在2秒后重新连接...")
                    time.sleep(2)
    
    def shutdown(self):
        """关闭节点时的清理工作"""
        self.is_active = False
        self.is_recording = False
        
        if hasattr(self, 'text_lock'):
            with self.text_lock:
                if self.speech_activity_timer:
                    self.speech_activity_timer.cancel()
                    self.speech_activity_timer = None

        if self.ws:
            try:
                self.ws.close()
            except:
                pass
        rospy.loginfo("语音识别节点已关闭")

if __name__ == '__main__':
    try:
        asr = VolcEngineASR()
        rospy.on_shutdown(asr.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass