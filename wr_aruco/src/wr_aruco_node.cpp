#include "ros/ros.h"
#include <stdlib.h>
#include "wr_aruco/wr_aruco.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "wr_aruco_node");
	ros::NodeHandle nh;

	WrAruco wrAruco;

	if (!wrAruco.init()) {
		ROS_ERROR("[wr_aruco_node] WrAruco::init failed");
		exit(-1);
	}

	ROS_INFO("[wr_aruco_node] starting loop");
	while (ros::ok()) {
		wrAruco.processVideoFrame();
	}

	ROS_INFO("[wr_aruco_node] shutdown");

	return 0;
}