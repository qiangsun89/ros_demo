/*
 * @file: 
 * @Author: Qiang Sun
 * @Date: 2023-03-31 09:24:06
 * @LastEditTime: 2023-03-31 14:35:17
 * @version: 
 * @copyright: ROSIWIT Copyright (c) 2023
 * @brief: 
 */
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv)
{
// 初始化节点
ros::init(argc, argv, "my_tf_listener");

ros::NodeHandle node;

// 通过服务调用，产生第二只乌龟turtle2
ros::service::waitForService("spawn");
ros::ServiceClient add_turtle = node.serviceClient<turtlesim::Spawn>("spawn");
turtlesim::Spawn srv;
add_turtle.call(srv);

// 定义turtle2的速度控制发布器
ros::Publisher turtle_vel = node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);

// tf监听器
tf2_ros::Buffer buffer; 
tf2_ros::TransformListener listener(buffer);

ros::Rate rate(10.0);
while (node.ok())
{
try
{
// 查找turtle2与turtle1的坐标变换
geometry_msgs::TransformStamped tfs = buffer.lookupTransform("turtle2","turtle1",ros::Time(0));
// 根据turtle1和turtle2之间的坐标变换，计算turtle2需要运动的线速度和角速度
// 并发布速度控制指令，使turtle2向turtle1移动
geometry_msgs::Twist vel_msg;
vel_msg.angular.z = 4.0 * atan2(tfs.transform.translation.y,tfs.transform.translation.x);
vel_msg.linear.x = 0.5 * sqrt(pow(tfs.transform.translation.x, 2) + pow(tfs.transform.translation.y, 2));
turtle_vel.publish(vel_msg);
}
catch (const std::exception& e) 
{
ROS_ERROR("%s",e.what());
ros::Duration(1.0).sleep();
continue;
}



rate.sleep();
ros::spinOnce();
}
return 0;
}