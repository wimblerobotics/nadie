#ifndef __WR_AUCO
#define __WR_AUCO

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "wr_aruco/DetectorParamsConfig.h"
#include "wr_aruco/FiducialMapEntryArray.h"

class WrAruco {
public:
	WrAruco(ros::NodeHandle& nh);

	bool init();

	void processVideoFrame();

private:
	// ROS Parameters.
	int arucoDictionaryNumber_;
	std::string cameraFrame_;
	std::string cameraIntrinsicsPath_;
	std::string mapFrame_;
	int videoDeviceNumber_;
	int videoFrameHeight_;
	int videoFrameWidth_;
	bool visualizeMarkers_;
    dynamic_reconfigure::Server<wr_aruco::DetectorParamsConfig> configServer_;
    dynamic_reconfigure::Server<wr_aruco::DetectorParamsConfig>::CallbackType callbackType_;

	// OpenCV variables.
	cv::VideoCapture* cap_;				// Video feed.
	cv::Mat cameraMatrix_;				// Camera intrinsics.
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;
	cv::Ptr<cv::aruco::Dictionary> dictionary_;	// Dictionary to be used for aruco markers.
	cv::Mat distortionCoefficients_;	// Camera intrinsics.

	// ROS variables;
	ros::Publisher fiducialMapEntryArrayPub_;	// For publishing a FiducialMapEtryArray
	ros::NodeHandle& nh_;					// ROS node handle;
	ros::Time prev_;							// Previous loop time, for computing durations.

	void configCallback(wr_aruco::DetectorParamsConfig &config, uint32_t level);

	void displayVideoFrameStats(cv::Mat& frame, std::vector<int>& ids, std::vector<cv::Vec3d>& rvecs, std::vector<cv::Vec3d>& tvecs);

	void publishMap(std::vector<int>& ids, std::vector<cv::Vec3d>& rvecs, std::vector<cv::Vec3d>& tvecs);

	void setDetectorParameters();
};

#endif
