1. 配置环境变量
```bash
export VOLC_APP_ID="7709498845"
export VOLC_ACCESS_KEY="tiKoWmAWbbi-tIkY81klLAk-evHvOxlL"
```

2. 启动语音识别服务
```bash
python asr.py
```


    <!-- 机械臂节点 -->
    <!-- 启动控制器管理器 -->
    <include file="$(find my_dynamixel)/launch/controller_manager.launch" />
    <!-- 启动控制器 -->
    <include file="$(find my_dynamixel)/launch/start_tilt_controller.launch" />
    <!-- 启动机械臂脚本 -->
    <node name="arm_demo" pkg="my_dynamixel" type="arm_demo.py" output="screen" 
        launch-prefix="gnome-terminal --tab --title=ArmControl -e " />


        roslaunch turtlebot_rviz_launchers view_navigation.launch --screen 