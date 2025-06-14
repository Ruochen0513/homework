#!/bin/bash
# 安装盲人引导/陪伴机器人所需的Python依赖项

echo "正在安装盲人引导/陪伴机器人所需的依赖项..."

# 安装系统依赖
sudo apt-get update
sudo apt-get install -y python3-pip python3-numpy python3-pyaudio portaudio19-dev

# 安装Python依赖
pip3 install requests wave numpy pyaudio

echo "安装完成！"

# 设置Python脚本的执行权限
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
chmod +x ${SCRIPT_DIR}/speech_recognition.py
chmod +x ${SCRIPT_DIR}/speech_synthesis.py
chmod +x ${SCRIPT_DIR}/semantic_understanding.py

echo "已设置脚本执行权限"
echo "请确保您已获取VolcEngine API密钥和密钥对，并在启动时提供它们。"
echo "例如: roslaunch blind_assistant voice_interaction.launch api_key:=YOUR_API_KEY api_secret:=YOUR_API_SECRET" 