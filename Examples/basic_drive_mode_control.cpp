#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "isc_shared_msgs/xinput.h"
#include "isc_shared_msgs/wheel_speeds.h"

#include <sstream>
#include <string>

bool startButtonDown = false;
bool autoMode = false;
int speedBoostButton=false;//mike added this 6/3

ros::Publisher wheelSpeedPub;

void updateDriveModeParameter(){
	if(autoMode){
		ros::param::set("/drive_mode", "auto");
	}
	else {
		ros::param::set("/drive_mode", "manual");
	}
}

void joystickCallback(const isc_shared_msgs::xinput::ConstPtr& joy){
	/* This fires every time a button is pressed/released
	and when an axis changes (even if it doesn't leave the
	deadzone). */

	/* We track the button state to ensure you can't
	accidentally hold down Start and press another
	button to toggle the mode */
	if(joy->Start){ //The Start button has been pressed
		startButtonDown = true;
	}
	if(startButtonDown && !joy->Start){ //The Start button has been released
		startButtonDown = false;
		autoMode = !autoMode;
		updateDriveModeParameter();
		ROS_INFO("Drive Mode Control: Switching to %s mode.", autoMode ? "AUTO" : "MANUAL");
	}

	speedBoostButton = joy->LS;//mike added this 6/3

}

void manualCallback(const geometry_msgs::Twist::ConstPtr& msg){
	if(!autoMode){
		float leftWheelSpeed = 0.0, rightWheelSpeed = 0.0;

        if(speedBoostButton)
        {
            leftWheelSpeed = (msg->linear.x - msg->angular.z);
            rightWheelSpeed = (msg->linear.x + msg->angular.z);
        }else{
            leftWheelSpeed = (msg->linear.x - msg->angular.z)/2;
            rightWheelSpeed = (msg->linear.x + msg->angular.z)/2;
        }

		isc_shared_msgs::wheel_speeds msg;
		msg.left = leftWheelSpeed;
		msg.right =  rightWheelSpeed;
		wheelSpeedPub.publish(msg);
	}
}

void autoCallback(const geometry_msgs::Twist::ConstPtr& msg){
	if(!autoMode){
		float leftWheelSpeed = 0.0, rightWheelSpeed = 0.0;

		leftWheelSpeed = (msg->linear.x - msg->angular.z);
		rightWheelSpeed = (msg->linear.x + msg->angular.z);

		isc_shared_msgs::wheel_speeds msg;
		msg.left = leftWheelSpeed;
		msg.right =  rightWheelSpeed;
		wheelSpeedPub.publish(msg);
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "drive_mode_control");

	ros::NodeHandle n;

	updateDriveModeParameter();

	wheelSpeedPub = n.advertise<isc_shared_msgs::wheel_speeds>("motors/wheel_speeds", 5);

	ros::Subscriber joystickSub = n.subscribe("joystick/xbox360", 5, joystickCallback);
	ros::Subscriber manualSub = n.subscribe("manual_control", 5, manualCallback);
	ros::Subscriber autoSub = n.subscribe("auto_control", 5, autoCallback);

	ros::spin();

	return 0;
}
