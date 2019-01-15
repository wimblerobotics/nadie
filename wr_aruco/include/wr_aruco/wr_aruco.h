#ifndef __WR_AUCO
#define __WR_AUCO

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>


class WrAruco {
public:
	WrAruco();

	bool init();

	void processVideoFrame();

private:
	cv::VideoCapture* cap_;
	cv::Mat cameraMatrix_;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;
	cv::Mat distortionCoefficients_;

	ros::Time prev_;

	cv::Ptr<cv::aruco::Dictionary> dictionary_;
};

#endif
