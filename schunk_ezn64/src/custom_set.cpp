#include <ros/ros.h>
#include <ezn64_usb_control.h>
#include <schunk_ezn64/set_position.h>
#include <std_msgs/Float32MultiArray.h>

int  main(int argc, char *argv[])
{
	ros::init(argc, argv, "listener1");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("setData", 1);
	ros::Rate loop_rate(500);

	std_msgs::Float32MultiArray grip;
	grip.data.resize(2);

	while(ros::ok()){
		
		grip.data[0] = 10;
		grip.data[1] = 10;	

		pub.publish(grip);
	
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
