#include<iostream>
#include <ros/ros.h>
#include <wittenstein_msgs/wittenstein.h>
#include <visualization_msgs/Marker.h>
wittenstein_msgs::wittenstein global_msg;

void callback(const wittenstein_msgs::wittenstein::ConstPtr& msg){
    global_msg.fx = msg->fx;
    global_msg.fy = msg->fy;
    global_msg.fz = msg->fz;
    ROS_INFO_STREAM(global_msg);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "wittenstein_listener");
	ros::NodeHandle nh;
    ros::Subscriber subscriber = nh.subscribe("wittenstein_topic", 1, &callback);
    ros::spin();
    return 0;
}