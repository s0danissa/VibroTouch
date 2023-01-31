// -------------------------------------------------
// INCLUDES

#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int16MultiArray.h>
#include <wittenstein_msgs/wittenstein.h>
#include <rosbag/bag.h>
#include <sensor_msgs/JointState.h>

using namespace std;

// -------------------------------------------------
// GLOBAL VARIABLES

float wittForceZ;
float forceGoal = 0.0;
float oldPosition = 20.0;
float newPosition = 0.0;
float forceError = 0.0;
float positionStep = 0.0;

int forceGoalIndexCounter = 0;
int expStatus = 0;
int gripperCount = 0;

bool forceControlStart = 0;
bool firstLoop = 0;
bool goalAchieved = 0;
bool gripperRelease = 0;

// -------------------------------------------------
// CALLBACK FUNCTIONS

// Callback for recieveing data from Wittenstein F/T
void forceCallback(const wittenstein_msgs::wittenstein::ConstPtr& msgForce){
	wittForceZ = msgForce->fz;
}

// Callback for recieving experimental procedure flags
void expCallback(const std_msgs::Float32::ConstPtr& msgExp){
	forceGoal = msgExp->data;
	forceControlStart = 1;
}

// -------------------------------------------------
// MAIN
// -------------------------------------------------

int main(int argc, char **argv)
{	
	// -------------------------------------------------
	// INITIALIZING ROS HANDLES
	// -------------------------------------------------

	ros::init(argc, argv, "force_control");
	ros::NodeHandle n;
	ros::Rate rate(500); 
	sensor_msgs::JointState schunk_st;

	// ------------------------------------------------- 
	// ROS MESSAGES
	// -------------------------------------------------

	// Gripper position and speed message that is being sent to SCHUNK EGN100
	std_msgs::Float32MultiArray gripMessage;
	gripMessage.data.resize(2);

	// Experiment status
	std_msgs::Int16MultiArray expStatusData;
	expStatusData.data.resize(1);

	ros::Subscriber sub_wittenstein = n.subscribe("wittenstein_topic", 1, &forceCallback);
	ros::Subscriber sub_force = n.subscribe("send_force", 1, &expCallback); 

	ros::Publisher pub = n.advertise<std_msgs::Float32MultiArray>("setData", 1);
	ros::Publisher exp_stat = n.advertise<std_msgs::Int16MultiArray>("exp_status",1);
		
	ROS_INFO("Force control launched successfully");
	while(ros::ok()){

		// -------------------------------------------------
		// PROCEDURE 1: ONE-TIME CUSTOM DESIRED FORCE GRIP CONTROL
		// -------------------------------------------------

		if(forceControlStart == 1 && forceGoal > 0.0){ 

			if(firstLoop == 0){
				positionStep = forceGoal-forceGoal/3;
				firstLoop = 1;
			}

			if(wittForceZ < -30 || forceGoal > 30){
				ROS_INFO("URGENT STOP| pos: %.5f", newPosition);
				return 0;
			} else if (wittForceZ > -0.5){
				newPosition = oldPosition - positionStep;
			
				gripMessage.data[0] = 5;
				gripMessage.data[1] = newPosition;

				pub.publish(gripMessage);

				oldPosition = newPosition;
				if(positionStep > 0.001){
					positionStep = positionStep/10;
				} else {
					positionStep = 0.001;
				}
			}else{
				forceError = (abs(wittForceZ)-forceGoal)/forceGoal;			
				newPosition = oldPosition + forceError*0.001;

				gripMessage.data[0] = 0.5;
				gripMessage.data[1] = newPosition;

				pub.publish(gripMessage);
				oldPosition = newPosition;
			
				if(abs(abs(wittForceZ)-forceGoal)<=0.01){
					forceGoalIndexCounter++;
				}

				if(forceGoalIndexCounter >= 200){
					ROS_INFO("GOAL ACHIEVED - END FORCE = %.5f N", wittForceZ);
					forceGoalIndexCounter = 0;
					forceControlStart = 0;
					firstLoop = 0;
				}
			}

		// -------------------------------------------------
		// PROCEDURE 2: RELEASE THE GRIP
		// -------------------------------------------------

		} else if (forceControlStart == 1 && forceGoal == -10.0){ // RELEASE 
			gripMessage.data[0] = 11.75;
			gripMessage.data[1] = 20;

			pub.publish(gripMessage);
			forceControlStart = 0;
			firstLoop = 0;
			ROS_INFO("GRIPPER RELEASED");

		// -------------------------------------------------
		// PROCEDURE 3: EXPERIMENTAL PROTOCOLS
		// -------------------------------------------------

		} else if (forceControlStart == 1 && forceGoal == -5.0){ 
			if(firstLoop == 0){ // First loop: Desired force is set to 2.0 N
				positionStep = 2.0-2.0/3;
				firstLoop = 1;
			}

			if(wittForceZ < -30){ // Caution handle
				ROS_INFO("URGENT STOP| pos: %.5f", newPosition);
				return 0;
			} else if (wittForceZ > -0.5){ // Move untill the contact of 0.5 N is established
				newPosition = oldPosition - positionStep;
			
				gripMessage.data[0] = 2;
				gripMessage.data[1] = newPosition;

				pub.publish(gripMessage);

				oldPosition = newPosition;
				if(positionStep > 0.001){
					positionStep = positionStep/10;
				} else {
					positionStep = 0.001;
				}
			}else{ // Once the contact of 0.5 is established
				if(goalAchieved == 0){ // Start achieving the goal force of 2.0
					forceError = (abs(wittForceZ)-2.0)/2.0;			
					newPosition = oldPosition + forceError*0.001;

					gripMessage.data[0] = 0.5;
					gripMessage.data[1] = newPosition;

					pub.publish(gripMessage);
					oldPosition = newPosition;
			
					if(abs(abs(wittForceZ)-2.0)<=0.01){
						forceGoalIndexCounter++;
					}
					if(forceGoalIndexCounter >= 200){
						ROS_INFO("GOAL ACHIEVED - END FORCE = %.5f N | STARTING THE PROCEDURE", wittForceZ);
						goalAchieved = 1;
					}
				} else if(goalAchieved == 1){ // Once the 2.0 force is achieved begin the experiment
					if (abs(wittForceZ) < 5.0 && expStatus == 0){ // Slowly move to 5.0 N (0.25 mm/s speed)

						gripMessage.data[0] = 0.25;
						gripMessage.data[1] = newPosition-5;

						pub.publish(gripMessage);
						expStatus = 1;
						expStatusData.data[0] = 2;
						exp_stat.publish(expStatusData);
					}else if (abs(abs(wittForceZ)-5.0) < 0.1 && expStatus == 1){

						ROS_INFO("DONE, GOING BACK %.5f", abs(abs(wittForceZ)-5.0));
						gripMessage.data[0] = 0.25;
						gripMessage.data[1] = newPosition;

						pub.publish(gripMessage);
						expStatus = 2;
					}else if (abs(abs(wittForceZ)-2.0) < 0.1 && expStatus == 2) {
						expStatusData.data[0] = 1;
						ROS_INFO("EXP. FINISHED");
						exp_stat.publish(expStatusData);
						forceGoalIndexCounter = 0;
						forceControlStart = 0;
						firstLoop = 0;
						goalAchieved = 0;
						expStatus = 0;
					}
				}
			}

		// -------------------------------------------------
		// PROCEDURE 4: CONTACT DETECTION EXPERIMENT
		// -------------------------------------------------

		} else if (forceControlStart == 1 && forceGoal == -15.0){ // ESTABLISHING TOUCH
			if (gripperRelease == 0){ // establishing touch
				if (gripperCount == 0){
					ROS_INFO("STARTING THE CONFIGURATION EXPERIMENT");
					gripMessage.data[1] = 20;
					gripMessage.data[0] = 20;
					pub.publish(gripMessage);
					expStatusData.data[0] = 2;
					exp_stat.publish(expStatusData);
				}

				gripperCount = gripperCount + 1;
				gripMessage.data[0] = 5; 
				gripMessage.data[1] = 20.0 - gripperCount*0.01;
				
				pub.publish(gripMessage);
				ROS_INFO("PUBLISHED: %.5f", 20.0 - gripperCount*0.01);
				sleep(1);
				if (gripperCount == 110){
					gripperRelease = 1;
					gripperCount = 0;
				}
			} else{ // releasing from touch event (once the desired dimension is reached)
				if (gripperCount == 0){
					ROS_INFO("GOING BACK");
				}
				gripperCount = gripperCount + 1;
				gripMessage.data[0] = 5;
				gripMessage.data[1] = 18.9 + gripperCount*0.01;

				pub.publish(gripMessage);
				ROS_INFO("PUBLISHED: %.5f", 18.9 + gripperCount*0.01);
				sleep(1);
				if (gripperCount == 110){
					gripperRelease = 0;
					forceControlStart = 0;
					expStatusData.data[0] = 1;
					exp_stat.publish(expStatusData);
				
					ROS_INFO("EXP. FINISHED");
					gripperCount = 0;
				}
			}
		}
		ros::spinOnce();
		rate.sleep();	
	
	}
}
