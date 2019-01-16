#ifndef __WR_AUCO
#define __WR_AUCO

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


class WrAruco {
public:
	WrAruco(const ros::NodeHandle& nh);

	bool init();

	void processVideoFrame();

private:
	// ROS Parameters.
	std::string cameraIntrinsicsPath_;

	// OpenCV variables.
	cv::VideoCapture* cap_;				// Video feed.
	cv::Mat cameraMatrix_;				// Camera intrinsics.
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;
	cv::Ptr<cv::aruco::Dictionary> dictionary_;	// Dictionary to be used for aruco markers.
	cv::Mat distortionCoefficients_;	// Camera intrinsics.

	// ROS variables;
	const ros::NodeHandle& nh_;			// ROS node handle;
	ros::Time prev_;					// Previous loop time, for computing durations.

};

#endif
