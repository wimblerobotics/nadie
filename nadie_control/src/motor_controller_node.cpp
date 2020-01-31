#include "ros/ros.h"

#include "nadie_control/nadie_motor_controller.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "motor_controller_node");
	ros::NodeHandle nh;

	ROS_INFO("[motor_controller_node] starting spinner");
	ros::AsyncSpinner spinner(50);
	spinner.start();

  	boost::shared_ptr<NadieMotorController> hw;

	hw.reset(new NadieMotorController(nh));

	ROS_INFO("[motor_controller_node] about to call hw->init");
	hw->init();
	hw->controlLoop();

	return 0;
}