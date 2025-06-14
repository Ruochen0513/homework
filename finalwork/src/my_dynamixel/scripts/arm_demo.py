#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, String
from dynamixel_msgs.msg import JointState
import threading

class ArmController:
    def __init__(self, error_threshold=0.01, settle_time=1.0, short_wait=3.5):
        rospy.on_shutdown(self.cleanup)

        # 发布器
        self.joint_tilt_pub = rospy.Publisher('tilt_controller/command', Float64, queue_size=10)
        self.joint_shoulder_pub = rospy.Publisher('shoulder_controller/command', Float64, queue_size=10)
        self.joint_elbow_pub = rospy.Publisher('elbow_controller/command', Float64, queue_size=10)
        self.joint_wrist_pub = rospy.Publisher('wrist_controller/command', Float64, queue_size=10)
        self.joint_hand_pub = rospy.Publisher('hand_controller/command', Float64, queue_size=10)

        # 关节误差跟踪
        self.joint_errors = {
            'tilt': float('inf'),
            'shoulder': float('inf'),
            'elbow': float('inf'),
            'wrist': float('inf'),
            'hand': float('inf')
        }
        self.error_threshold = error_threshold
        self.settle_time = settle_time
        self.short_wait = short_wait
        self.lock = threading.Lock()

        # 订阅关节状态
        rospy.Subscriber('tilt_controller/state', JointState, self._make_state_cb('tilt'))
        rospy.Subscriber('shoulder_controller/state', JointState, self._make_state_cb('shoulder'))
        rospy.Subscriber('elbow_controller/state', JointState, self._make_state_cb('elbow'))
        rospy.Subscriber('wrist_controller/state', JointState, self._make_state_cb('wrist'))
        rospy.Subscriber('hand_controller/state', JointState, self._make_state_cb('hand'))

        # 目标位置
        self.target_positions_initial = {
            'tilt': 2.55,
            'shoulder': 2.55,
            'elbow': 2.58,
            'wrist': 2.63,
            'hand': 2.0
        }
        self.target_positions_prepare = {
            'tilt': 2.71,
            'shoulder': 2.16,
            'elbow': 1.05,
            'wrist': 2.96,
            'hand': 2.0
        }
        self.target_positions_grip = {
            'tilt': 2.71,
            'shoulder': 2.16,
            'elbow': 1.05,
            'wrist': 2.96,
            'hand': 2.8
        }
        self.target_positions_deliver = {
            'tilt': 2.22,
            'shoulder': 2.65,
            'elbow': 1.9,
            'wrist': 2.82,
            'hand': 2.8
        }
        # 订阅动作触发话题
        self.busy = False
        rospy.Subscriber('arm_action', String, self._action_cb)

        rospy.loginfo("Arm controller initialized. Will start reciprocating motion based on joint error.")

    def _make_state_cb(self, joint_name):
        """创建关节状态回调函数"""
        def cb(msg):
            with self.lock:
                self.joint_errors[joint_name] = abs(msg.error)
        return cb

    def joints_are_settled(self):
        """检查所有关节是否已到达目标位置"""
        with self.lock:
            settled_joints = []
            unsettled_joints = []
            
            for joint_name, error in self.joint_errors.items():
                if error < self.error_threshold:
                    settled_joints.append(f"{joint_name}: {error:.4f}")
                else:
                    unsettled_joints.append(f"{joint_name}: {error:.4f}")
            
            all_settled = len(unsettled_joints) == 0
            
            # 显示调试信息
            if unsettled_joints:
                rospy.loginfo_throttle(2, f"未到位关节: {unsettled_joints} (阈值: {self.error_threshold})")
            else:
                rospy.loginfo_throttle(2, f"所有关节已到位: {settled_joints}")
            
            return all_settled

    def send_joint_positions(self, positions):
        self.joint_tilt_pub.publish(Float64(positions['tilt']))
        self.joint_shoulder_pub.publish(Float64(positions['shoulder']))
        self.joint_elbow_pub.publish(Float64(positions['elbow']))
        self.joint_wrist_pub.publish(Float64(positions['wrist']))
        self.joint_hand_pub.publish(Float64(positions['hand']))

    def run_bottle_operation_mode(self):
        """瓶子操作模式 - 等待话题触发"""
        rospy.loginfo("机械臂进入瓶子操作模式")
        rospy.loginfo("发送 'start' 到话题 'arm_action' 来开始瓶子抓取序列")
        rospy.loginfo("例如: rostopic pub /arm_action std_msgs/String \"data: 'start'\"")
        
        # 保持节点运行，等待话题触发
        rospy.spin()

    def _action_cb(self, msg):
        """收到启动指令后按序执行状态序列"""
        if msg.data.strip().lower() != 'start':
            return
        if self.busy:
            rospy.logwarn('动作序列已在运行中，忽略新指令')
            return
        threading.Thread(target=self.execute_sequence).start()

    def execute_sequence(self):
        """依次执行初始、预备、夹瓶、递给操作，每个动作间等待2秒"""
        self.busy = True
        sequence = [
            ('初始状态', self.target_positions_initial),
            ('预备夹瓶子', self.target_positions_prepare),
            ('夹瓶子', self.target_positions_grip),
            ('递给用户', self.target_positions_deliver)
        ]
        
        for i, (name, positions) in enumerate(sequence):
            rospy.loginfo(f"执行步骤 {i+1}/4: {name}")
            rospy.loginfo(f"目标位置: {positions}")
            self.send_joint_positions(positions)
            
            # 简单等待2秒，不跟踪误差
            rospy.loginfo(f"{name} 已发送命令，等待 2 秒...")
            try:
                rospy.sleep(2.0)  # 固定等待2秒
            except rospy.ROSInterruptException:
                rospy.loginfo("操作被中断")
                break
                
            if rospy.is_shutdown():
                break
                
        rospy.loginfo("瓶子抓取序列完成！")
        self.busy = False

    def cleanup(self):
        rospy.loginfo('Shutting down arm controller demo...')
        rospy.loginfo('Arm controller shutdown complete.')

if __name__== '__main__':
    rospy.init_node('arm_bottle_operation_demo', anonymous=True)
    try:
        controller = ArmController(error_threshold=0.01, settle_time=1.0, short_wait=2.0)  # 等待时间2秒
        controller.run_bottle_operation_mode()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS Interrupt Exception caught in main. Node shutting down.")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {e}")
    finally:
        rospy.loginfo("Exiting arm_bottle_operation_demo node.")