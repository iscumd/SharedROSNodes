#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "isc_joy/xinput.h"

#include <string>

ros::Publisher manualPub;

bool flipForwardBackward = false;
bool flipLeftRight = false;

double speedMultiplier;
double turnMultiplier;

bool enableLogging;

void joystickCallback(const isc_joy::xinput::ConstPtr& joy){	
	/* This fires every time a button is pressed/released
	and when an axis changes (even if it doesn't leave the
	deadzone). */

	bool enableDriving = joy->LB; //the dead man's switch

	//toggle flipping controls
	if(joy->Y && !enableDriving) flipForwardBackward = !flipForwardBackward;
	if(joy->X && !enableDriving) flipLeftRight = !flipLeftRight;

	float joySpeed = 0.0, joyTurn = 0.0;
	joySpeed = joy->LeftStick_UD * (flipForwardBackward ? -1.0 : 1.0);// * speedMultiplier;
	joyTurn = joy->LeftStick_LR * (flipLeftRight ? -1.0 : 1.0) * turnMultiplier;

	//disable joystick axes
	if(joy->A) joySpeed = 0;
	if(joy->B) joyTurn = 0;

	geometry_msgs::Twist msg;
	msg.linear.x = enableDriving ? joySpeed : 0;
	msg.angular.z = enableDriving ? joyTurn : 0;
	manualPub.publish(msg);

	if(enableLogging) ROS_INFO("Manual Control: %s linear.x=%f angular.z=%f", joy->LB ? "on" : "off", msg.linear.x, msg.angular.z);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "manual_control");

	ros::NodeHandle n;

	n.param("manual_control_enable_logging", enableLogging, false);

	n.param("manual_control_speed_multiplier", speedMultiplier, 1.0);
	n.param("manual_control_turn_multiplier", turnMultiplier, 0.5);

	manualPub = n.advertise<geometry_msgs::Twist>("manual_control", 5);

	ros::Subscriber joystickSub = n.subscribe("joystick/xinput", 5, joystickCallback);

	ros::spin();

	return 0;
}
