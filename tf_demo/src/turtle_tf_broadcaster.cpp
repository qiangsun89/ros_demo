/*
 * @file: 
 * @Author: Qiang Sun
 * @Date: 2023-03-31 09:20:30
 * @LastEditTime: 2023-03-31 14:12:03
 * @version: 
 * @copyright: ROSIWIT Copyright (c) 2023
 * @brief: 
 */
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>

std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg)
{
// tf广播器
static tf2_ros::TransformBroadcaster br;

// 根据海龟当前的位姿，设置相对于世界坐标系的坐标变换
geometry_msgs::TransformStamped transformStamped;
transformStamped.header.stamp = ros::Time::now();
transformStamped.header.frame_id = "world";
transformStamped.child_frame_id = turtle_name;
transformStamped.transform.translation.x = msg->x;
transformStamped.transform.translation.y = msg->y;
transformStamped.transform.translation.z = 0.0;
tf2::Quaternion q;
q.setRPY(0, 0, msg->theta);
transformStamped.transform.rotation.x = q.x();
transformStamped.transform.rotation.y = q.y();
transformStamped.transform.rotation.z = q.z();
transformStamped.transform.rotation.w = q.w();

// 发布坐标变换
br.sendTransform(transformStamped);
}

int main(int argc, char** argv)
{
// 初始化节点
ros::init(argc, argv, "my_tf_broadcaster");
if (argc != 2)
{
ROS_ERROR("need turtle name as argument"); 
return -1;
};
turtle_name = argv[1];

// 订阅海龟的pose信息
ros::NodeHandle node;
ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

ros::spin();

return 0;
}
