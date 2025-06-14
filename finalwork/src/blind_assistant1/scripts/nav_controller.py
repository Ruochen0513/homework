#!/usr/bin/env python3
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
import actionlib
from actionlib_msgs.msg import GoalStatus

class NavController:
    def __init__(self):
        rospy.init_node('nav_controller')
        
        self.is_navigating = False

        # 房间位置预设
        self.room_locations = {
            "302": [3.2856, -0.095932, 0, 0, 0.63057, 0.77614],
            "327": [0.79482, 2.2434, 0, 0, 0.94996, 0.21236]
        }
        
        # Publisher for TTS and navigation status
        self.tts_pub = rospy.Publisher('tts_text', String, queue_size=10)
        self.nav_status_pub = rospy.Publisher('navigation_status', String, queue_size=1, latch=True)
        
        # 动作客户端
        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待move_base服务器...")
        
        # 设置超时等待
        if not self.move_base.wait_for_server(rospy.Duration(30.0)):
            rospy.logerr("无法连接到move_base服务器！请检查导航系统是否正常启动。")
            self.tts_pub.publish("导航系统连接失败，请检查系统状态。")
        else:
            rospy.loginfo("move_base服务器已连接")

        # 订阅导航指令 - 统一话题名称
        rospy.Subscriber('navigation_command', String, self.navigation_callback)
        
        # 发布初始状态
        self.nav_status_pub.publish("idle")
        rospy.loginfo("导航控制器已启动")

    def navigation_callback(self, msg):
        """处理导航指令"""
        if self.is_navigating:
            rospy.logwarn("正在导航中，请等待导航结束后再试")
            self.tts_pub.publish("我正在导航，请稍等。")
            return
            
        room = msg.data
        if room in self.room_locations:
            rospy.loginfo(f"接收到导航命令，前往 {room} 房间")
            self.navigate_to_room(room)
        else:
            rospy.logwarn(f"未知的导航目标: {room}")
            self.tts_pub.publish(f"抱歉，我不知道{room}房间的位置。")

    def navigate_to_room(self, room):
        """导航到指定房间"""
        self.is_navigating = True
        
        # 更新导航状态
        self.nav_status_pub.publish("active")
        
        # 创建导航目标
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.room_locations[room][0]
        goal.target_pose.pose.position.y = self.room_locations[room][1]
        goal.target_pose.pose.orientation.x = self.room_locations[room][2]
        goal.target_pose.pose.orientation.y = self.room_locations[room][3]
        goal.target_pose.pose.orientation.z = self.room_locations[room][4]
        goal.target_pose.pose.orientation.w = self.room_locations[room][5]
        
        # 发送导航目标
        self.move_base.send_goal(goal, done_cb=self.navigation_done_callback)
        rospy.loginfo(f"正在导航到 {room} 房间")
        self.tts_pub.publish(f"出发")

    def navigation_done_callback(self, status, result):
        """导航完成回调"""
        self.is_navigating = False
        self.nav_status_pub.publish("idle")
        
        rospy.loginfo(f"导航完成，状态码: {status} ({GoalStatus.to_string(status)})")

        if status == GoalStatus.SUCCEEDED:
            final_message = "已成功到达目的地。现在您可以下达新的指令。"
        elif status == GoalStatus.PREEMPTED:
            final_message = "导航任务已被新的指令取消。现在您可以下达新的指令。"
        elif status == GoalStatus.ABORTED:
            final_message = "导航失败，可能是因为路径被完全阻塞，或者我暂时找不到路。请您寻求他人帮助。"
        elif status == GoalStatus.REJECTED:
            final_message = "导航目标无法到达，它可能在障碍物内部或地图范围之外。"
        else:   
            final_message = f"导航因未知原因失败，状态为 {GoalStatus.to_string(status)}。"

        rospy.loginfo(final_message)
        self.tts_pub.publish(final_message)

    def cancel_navigation(self):
        """取消当前导航"""
        if self.is_navigating:
            self.move_base.cancel_goal()
            rospy.loginfo("导航已被用户取消")
            self.tts_pub.publish("导航已取消")
        else:
            self.tts_pub.publish("当前没有进行中的导航任务")

    def report_navigation_status(self):
        """报告导航状态"""
        if self.is_navigating:
            state = self.move_base.get_state()
            if state == GoalStatus.ACTIVE:
                status_msg = "正在执行导航任务"
            elif state == GoalStatus.PENDING:
                status_msg = "导航任务已接收，等待执行"
            else:
                status_msg = f"导航状态：{GoalStatus.to_string(state)}"
        else:
            status_msg = "当前空闲，可以接收新的导航指令"
        
        available_rooms = ", ".join(self.room_locations.keys())
        full_status = f"{status_msg}。可用房间：{available_rooms}"
        rospy.loginfo(full_status)
        self.tts_pub.publish(full_status)

if __name__ == '__main__':
    try:
        nav_controller = NavController()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("导航控制器节点关闭")
