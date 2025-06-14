# 盲人引导机器人系统修复说明

## 修复的主要问题

### 1. ASR控制混乱问题 ✅ 已修复

**原问题：**
- `center_node.py` 和 `nav_controller.py` 都在调用ASR服务
- 多个节点同时控制ASR导致状态冲突

**解决方案：**
- **统一ASR控制权**：只有 `center_node.py` 负责ASR的启停
- `nav_controller.py` 移除所有ASR控制代码
- 通过 `navigation_status` 话题进行状态协调

### 2. TTS-ASR声音回环问题 ✅ 已修复

**原问题：**
- TTS播放时ASR没有正确关闭
- 造成声音回环和误识别

**解决方案：**
- **严格的互斥机制**：TTS播放时强制停止ASR
- 改进TTS状态管理：`playing` → `finished` 状态流程
- `center_node` 等待TTS完成后再重启ASR
- 线程安全的状态管理（使用`state_lock`）

### 3. 话题架构混乱问题 ✅ 已修复

**原问题：**
- 存在重复话题（`voice_cmd` 和 `voice_cmd2`）
- 话题命名不统一

**解决方案：**
- **统一话题命名**：
  - `voice_cmd` → `navigation_command`
  - `arm_cmd` → `arm_command`  
  - `visual_cmd` → `visual_command`
- 移除冗余话题 `voice_cmd2`

### 4. 代码逻辑混乱问题 ✅ 已修复

**原问题：**
- 状态管理分散在多个节点
- 错误处理不完善
- 代码可读性差

**解决方案：**
- **集中状态管理**：`center_node` 作为核心协调者
- 改进错误处理和日志
- 简化代码逻辑，增加注释
- 统一编码规范（中文注释和日志）

## 新的系统架构

```
用户语音 → ASR → center_node → [TTS, 导航, 运动, 机械臂, 视觉]
                     ↑
           统一状态管理和ASR控制
```

### 核心设计原则

1. **单一责任**：每个节点只负责特定功能
2. **集中控制**：center_node 统一管理系统状态
3. **严格互斥**：TTS和ASR不会同时运行
4. **清晰通信**：标准化话题命名和消息格式

## 修复后的文件清单

### 核心修改文件
- ✅ `center_node.py` - 重构ASR控制逻辑，改进状态管理
- ✅ `nav_controller.py` - 移除ASR控制，简化逻辑  
- ✅ `tts.py` - 优化状态发布，改进音频处理
- ✅ `arm_control.py` - 更新话题名称
- ✅ `video_cap.py` - 更新话题名称，改进错误处理

### 新增文件
- ✅ `launch_system.py` - 系统启动脚本
- ✅ `SYSTEM_FIXES.md` - 本文档

### 保持不变的文件
- `asr.py` - ASR服务提供者（无需修改）
- `robot_control.py` - 运动控制（无需修改）
- `visual_recognition.py` - 视觉识别（无需修改）
- `navigation_monitor.py` - 导航监控（无需修改）

## 使用说明

### 环境配置
```bash
# 设置环境变量
export VOLC_APP_ID="7709498845"
export VOLC_ACCESS_KEY="tiKoWmAWbbi-tIkY81klLAk-evHvOxlL"
```

### 启动系统

**方法1：使用启动脚本（推荐）**
```bash
cd src/blind_assistant1/scripts
python3 launch_system.py
```

**方法2：手动启动节点**
```bash
# 按顺序启动各个节点
python3 asr.py &
python3 tts.py &
python3 center_node.py &
python3 nav_controller.py &
python3 robot_control.py &
python3 arm_control.py &
python3 video_cap.py &
python3 visual_recognition.py &
python3 navigation_monitor.py &
```

### 系统监控

使用以下命令监控系统状态：
```bash
# 查看话题列表
rostopic list

# 监控核心话题
rostopic echo /raw_text          # ASR输出
rostopic echo /tts_text          # TTS输入  
rostopic echo /tts_status        # TTS状态
rostopic echo /navigation_status # 导航状态

# 查看节点列表
rosnode list
```

## 测试验证

### 功能测试
1. **语音识别**：说话测试ASR是否正常工作
2. **语音合成**：TTS播放时ASR应自动停止
3. **导航功能**：说"去327房间"测试导航
4. **运动控制**：说"前进"测试运动控制
5. **机械臂控制**：说"帮我夹东西"测试机械臂
6. **视觉识别**：说"描述眼前景象"测试视觉

### 回环测试
- TTS播放时，ASR应完全停止
- TTS播放结束后，ASR应自动重启
- 不应出现声音回环现象

## 关键改进点

### 1. 线程安全
```python
# center_node.py 中使用锁保护状态
with self.state_lock:
    # 原子性状态操作
```

### 2. 严格状态管理
```python
# 三种主要状态的互斥管理
self.is_processing    # 正在处理LLM请求
self.is_navigating    # 正在导航
self.tts_is_playing   # TTS正在播放
```

### 3. 统一话题架构
```python
# 统一命名规范
navigation_command  # 导航指令
arm_command        # 机械臂指令  
visual_command     # 视觉指令
```

## 故障排除

### 常见问题

1. **ASR服务启动失败**
   - 检查环境变量是否设置
   - 确认网络连接正常

2. **TTS播放异常**
   - 检查音频设备
   - 查看TTS状态话题

3. **导航失败**
   - 确认move_base服务运行正常
   - 检查地图和定位

4. **话题不通信**
   - 使用 `rostopic list` 检查话题存在
   - 使用 `rostopic echo` 检查消息流

### 调试命令
```bash
# 检查节点状态
rosnode ping /center_node
rosnode ping /nav_controller

# 检查话题通信
rostopic hz /raw_text
rostopic hz /tts_status

# 查看日志
rosrun rqt_console rqt_console
```

## 维护建议

1. **定期监控**：关注系统日志中的警告和错误
2. **性能优化**：根据实际使用情况调整参数
3. **功能扩展**：按照现有架构原则添加新功能
4. **备份配置**：定期备份重要配置文件

---

**修复完成时间：** 2024年1月（当前）
**修复状态：** 已完成主要问题修复
**系统稳定性：** 大幅提升
**维护性：** 显著改善 