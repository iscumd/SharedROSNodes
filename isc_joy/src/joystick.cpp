#include "ros/ros.h"
#include "isc_joy/xinput.h"
#include "sensor_msgs/Joy.h"
#include <string>
#include <vector>

/* Xbox 360 wireless uses WIRED indexes http://wiki.ros.org/joy
#define BUTTON_A joy->buttons[0]
#define BUTTON_B joy->buttons[1]
#define BUTTON_X joy->buttons[2]
#define BUTTON_Y joy->buttons[3]
#define BUTTON_LB joy->buttons[4]
#define BUTTON_RB joy->buttons[5]
#define BUTTON_BACK joy->buttons[6]
#define BUTTON_START joy->buttons[7]
#define BUTTON_GUIDE joy->buttons[8]
#define BUTTON_LS joy->buttons[9]
#define BUTTON_RS joy->buttons[10]

#define AXIS_L_LR joy->axes[0]
#define AXIS_L_UD joy->axes[1]
#define AXIS_LT joy->axes[2]
#define AXIS_R_LR joy->axes[3]
#define AXIS_R_UD joy->axes[4]
#define AXIS_RT joy->axes[5]
#define AXIS_DPAD_LR joy->axes[6]
#define AXIS_DPAD_UD joy->axes[7]
*/

#define CONTROL_TYPE_BUTTON 0
#define CONTROL_TYPE_AXIS 1

typedef int control_type;

ros::Publisher joystickPub;

void setup_mappings() {
	ros::NodeHandle n("~");
	
	// controls are the buttons and axes on the gamepad
	std::vector<std::string> controls = {"A", "B", "X", "Y", "LB", "RB", "BACK", "START", "GUIDE", "LS", "RS", "L_LR", "L_UD", "LT", "R_LR", "R_UD", "RT", "DPAD_LR", "DPAD_UD"};
 
	for(auto control = controls.begin(); control != controls.end(); ++control) {
		int tmp;
		std::string s_tmp;
		if(n.hasParam("mappings/" + *control + "/type") && n.hasParam("mappings/" + *control + "/index")) { // check if both type and index are present for this control
			n.getParamCached("mappings/" + *control + "/type", s_tmp); // cache the params and we don't care about the actual value yet
			n.getParamCached("mappings/" + *control + "/index", tmp); 
		} else { // if either one is missing, this control is malformed and will not be reported
			n.setParam("mappings/" + *control + "/type", -1);
			n.setParam("mappings/" + *control + "/index", -1);

			ROS_WARN("Malformed or missing mapping for control %s. Control value will be set to 0", control->c_str()); 			

			n.getParamCached("mappings/" + *control + "/type", s_tmp); // even if a control is missing or malformed, we must cache it to avoid 
			n.getParamCached("mappings/" + *control + "/index", tmp); // the param server lookup
		}
	}
}

float get_mapping(std::string control, const sensor_msgs::Joy::ConstPtr &joy) {
	ros::NodeHandle n("~");
	std::string type;
	int index;
	
	n.getParamCached("mappings/" + control + "/type", type); // since it's cached the lookup is actually very quick
	n.getParamCached("mappings/" + control + "/index", index);

	ROS_INFO("%s: type=%s, index=%d", control.c_str(), type, index);

	if(type == "button") {
		if(index < joy->buttons.size() && index >= 0) { return joy->buttons[index]; } // do some quick bounds checking too, just in case index is wrong
		else {
			ROS_WARN("%s: index %d is out of bounds", control.c_str(), index);
			return 0.0;
		}
	} else if(type == "axis") {
		if(index < joy->axes.size() && index >= 0) { return joy->axes[index]; }
		else {
			ROS_WARN("%s: index %d is out of bounds", control.c_str(), index);
			return 0.0;
		}
	} else { // if this control was improperly defined or not defined at all, just report 0 for the control
		ROS_WARN("%s is ill-defined.", control.c_str());			
		return 0.0;
	}
}

void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy){	
	/* This fires every time a button is pressed/released
	and when an axis changes (even if it doesn't leave the
	deadzone). */
	
	isc_joy::xinput msg;
	msg.A = get_mapping("A", joy);
	msg.B = get_mapping("B", joy);
	msg.X = get_mapping("X", joy);
	msg.Y = get_mapping("Y", joy);
	msg.LB = get_mapping("LB", joy);
	msg.RB = get_mapping("RB", joy);
	msg.Back = get_mapping("BACK", joy);
	msg.Start = get_mapping("START", joy);
	msg.Guide = get_mapping("GUIDE", joy);
	msg.LS = get_mapping("LS", joy);
	msg.RS = get_mapping("RS", joy);

	msg.LeftStick_LR = get_mapping("L_LR", joy);
	msg.LeftStick_UD = get_mapping("L_UD", joy);
	msg.RightStick_LR = get_mapping("R_LR", joy);
	msg.RightStick_UD = get_mapping("R_UD", joy);
	msg.LT = get_mapping("LT", joy);
	msg.RT = get_mapping("RT", joy);
	msg.DPad_LR = get_mapping("DPAD_LR", joy);
	msg.DPad_UD = get_mapping("DPAD_UD", joy);

	joystickPub.publish(msg);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "joystick_xbox360");

	ros::NodeHandle n;

	setup_mappings();

	joystickPub = n.advertise<isc_joy::xinput>("joystick/xinput", 1000);
	ros::Subscriber sub = n.subscribe("joy", 1000, joystickCallback);

	ros::spin();
	
	return 0;
}
