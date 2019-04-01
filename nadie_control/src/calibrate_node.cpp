#include "ros/ros.h"
#include <stdlib.h>
#include "nadie_control/calibrate.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "calibrate_node");
	ros::NodeHandle nh;

	Calibrate calibrate(nh);

	if (!calibrate.init()) {
		ROS_ERROR("[calibrate_node] Calibrate::init failed");
		exit(-1);
	}

	ROS_INFO("[calibrate_node] starting loop");
    ros::Rate r(10);
	bool continueGoal = true;
	while (continueGoal && ros::ok()) {
		continueGoal = calibrate.run();
        ros::spinOnce();
        r.sleep();
	}

	ROS_INFO("[calibrate_node] shutdown");

	return 0;
}