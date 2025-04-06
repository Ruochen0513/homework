#include <ros/ros.h>
#include <homework3/CustomMessage.h> 
#include <sstream>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    
    // 发布自定义消息到 "chatter" 话题
    ros::Publisher chatter_pub = n.advertise<homework3::CustomMessage>("chatter", 1000);
    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok()) {
        homework3::CustomMessage msg;  // 自定义消息类型
        msg.number = count * 1.0f;             // 设置 float 字段
        msg.text = "BanGDream!Its MyGO!!!!!";            // 设置 string 字段

        ROS_INFO("Publishing: number=%f, text=%s", msg.number, msg.text.c_str());
        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }
    return 0;
}