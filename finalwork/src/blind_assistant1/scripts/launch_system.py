#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import subprocess
import time
import signal
import sys
import os

class SystemLauncher:
    def __init__(self):
        self.processes = []
        signal.signal(signal.SIGINT, self.signal_handler)
        rospy.init_node('system_launcher', anonymous=True)
        
    def launch_node(self, script_name, node_name):
        """启动单个节点"""
        script_path = os.path.join(os.path.dirname(__file__), script_name)
        try:
            rospy.loginfo(f"启动 {node_name}...")
            process = subprocess.Popen(['python3', script_path])
            self.processes.append((process, node_name))
            time.sleep(1)  # 给节点启动时间
            return True
        except Exception as e:
            rospy.logerr(f"启动 {node_name} 失败: {e}")
            return False
    
    def launch_all(self):
        """启动所有系统节点"""
        rospy.loginfo("开始启动盲人引导机器人系统...")
        
        # 启动顺序很重要
        nodes = [
            ('asr.py', 'ASR语音识别'),
            ('tts.py', 'TTS语音合成'),
            ('center_node.py', '核心控制节点'),
            ('nav_controller.py', '导航控制器'),
            ('robot_control.py', '运动控制器'),
            ('arm_control.py', '机械臂控制器'),
            ('video_cap.py', '视频捕获'),
            ('visual_recognition.py', '视觉识别'),
            ('navigation_monitor.py', '导航监控')
        ]
        
        success_count = 0
        for script, name in nodes:
            if self.launch_node(script, name):
                success_count += 1
            else:
                rospy.logwarn(f"节点 {name} 启动失败，但系统将继续启动其他节点")
        
        rospy.loginfo(f"系统启动完成！成功启动 {success_count}/{len(nodes)} 个节点")
        rospy.loginfo("系统架构说明：")
        rospy.loginfo("- center_node: 统一控制ASR启停，避免冲突")
        rospy.loginfo("- 严格的TTS-ASR互斥机制，防止声音回环")
        rospy.loginfo("- 统一的话题命名，简化系统架构")
        rospy.loginfo("- 错误处理和状态管理改进")
        
        # 保持运行直到手动终止
        try:
            while not rospy.is_shutdown():
                # 检查进程状态
                active_processes = []
                for process, name in self.processes:
                    if process.poll() is None:
                        active_processes.append((process, name))
                    else:
                        rospy.logwarn(f"节点 {name} 已退出")
                
                self.processes = active_processes
                time.sleep(5)
                
        except KeyboardInterrupt:
            pass
        
        self.cleanup()
    
    def signal_handler(self, signum, frame):
        """信号处理"""
        rospy.loginfo("收到终止信号，正在关闭系统...")
        self.cleanup()
        sys.exit(0)
    
    def cleanup(self):
        """清理进程"""
        rospy.loginfo("正在关闭所有节点...")
        for process, name in self.processes:
            try:
                process.terminate()
                rospy.loginfo(f"已终止 {name}")
            except Exception as e:
                rospy.logwarn(f"终止 {name} 时出错: {e}")
        
        # 等待进程清理完成
        time.sleep(2)
        for process, name in self.processes:
            try:
                if process.poll() is None:
                    process.kill()
                    rospy.logwarn(f"强制杀死 {name}")
            except Exception:
                pass

if __name__ == '__main__':
    launcher = SystemLauncher()
    launcher.launch_all() 