#include<iostream>
using namespace std;
#include "HexFT.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <wittenstein_msgs/wittenstein.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "main_wittenstein");
	ros::NodeHandle nh;
	//ros::Rate r(1);
	

	//schunk : joint_states -> position[0], velocity[0]

	int success = 0;
	ros::Publisher publisher = nh.advertise<wittenstein_msgs::wittenstein>("wittenstein_topic", 1);
	//ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	// Create HexFT sensor object.
	// Please change serial port name according to your setup!
	HexFT *hexFT = new HexFT("/dev/ttyACM1");

	SensorData sensorData;
	ROS_INFO_STREAM("*****Connecting*****");
	//uint32_t shape = visualization_msgs::Marker::ARROW;
	while(ros::ok()){
 		sensorData = hexFT->ReadSensorData();
		if(success == 0){
			success = 1;
			ROS_INFO_STREAM("Connected succesfully!");
		}
		wittenstein_msgs::wittenstein wittenstein_msg;
		wittenstein_msg.fx = sensorData.fX;
		wittenstein_msg.fy = sensorData.fY;
		wittenstein_msg.fz = sensorData.fZ;

		publisher.publish(wittenstein_msg);
		//ROS_INFO_STREAM("Fz: " << sensorData.fZ);
		

		// Print results
		//cout << "Fx: " << sensorData.fZ << endl;
		ros::spinOnce();
		//r.sleep();
	}
	return 0;
}
