#include <ros/ros.h>
#include <ros/console.h>

#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include <vector>

// #include "opencv2/imgproc/imgproc.hpp"
// #include <opencv2/opencv.hpp>
// #include <image_transport/image_transport.h>
// #include <cv_bridge/cv_bridge.h>
// #include <sensor_msgs/image_encodings.h>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/objdetect/objdetect.hpp>
// #include <opencv2/highgui/highgui.hpp>

#include "wr_aruco/wr_aruco.h"

using namespace cv;

WrAruco::WrAruco() {
	ROS_INFO("[WrAruco::WrAruco] invoked");
}

bool WrAruco::init() {
	ROS_INFO("[WrAruco::init] start");

    const std::string inputSettingsFile = "/home/wimble/.ros/camera_info/camerav2_1280x720.yml";
	auto fs = FileStorage(inputSettingsFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        ROS_ERROR("Could not open the configuration file: \"%s\"", inputSettingsFile.c_str());;
        return false;
    }

    fs["camera_matrix"] >> cameraMatrix_;
    fs["distortion_coefficients"] >> distortionCoefficients_;
    fs.release(); 

    ROS_INFO_STREAM("[WrAruco::init] camera_matrix: " << cameraMatrix_);
    ROS_INFO_STREAM("[WrAruco::init] distortion_coefficients: " << distortionCoefficients_);

	// Initialize the aruco dictionary.
	dictionary_ = aruco::getPredefinedDictionary(7/*###*/);

	cap_ = new cv::VideoCapture(0);
	if (!cap_->isOpened()) {
		ROS_ERROR("[WrAruco::init] VideoCapture FAILED");
		return false;
	}

	ROS_INFO("[WrAruco::init] VideoCapture successful");
	return true;
}

void WrAruco::processVideoFrame() {
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

	cv::Mat	frame;
	*cap_ >> frame;

	// cv::Mat undistortedFrame;
	// cv::undistort(frame, undistortedFrame, cameraMatrix_, distortionCoefficients_);

	std::vector<int> ids;
	std::vector <std::vector <cv::Point2f> > corners;
	cv::aruco::detectMarkers(frame, dictionary_, corners, ids);
	ROS_INFO("[wr_aruco::processVideoFrame] Detected %d markers", (int) ids.size());

	if(ids.size() > 0) {
		cv::aruco::drawDetectedMarkers(frame, corners, ids);
		std::vector<cv::Vec3d> rvecs;
		std::vector<cv::Vec3d> tvecs;
		cv::aruco::estimatePoseSingleMarkers(corners, 0.104, cameraMatrix_, distortionCoefficients_, rvecs, tvecs);

		std::cout << "rvecs: ";
		for (auto i = rvecs.begin(); i != rvecs.end(); ++i) std::cout << *i << ' ';
		std::cout << "  --  ";

		std::cout << "tvecs: ";
		for (auto i = tvecs.begin(); i != tvecs.end(); ++i) std::cout << *i << ' ';
		std::cout << std::endl;

        // draw axis for each marker
        for (unsigned int i = 0; i < ids.size(); i++) {
        	std::cout << "id" << ids[i] << std::endl;
            cv::aruco::drawAxis(frame, cameraMatrix_, distortionCoefficients_, rvecs[i], tvecs[i], 0.1);
    	}
    }

   	cv::imshow("pic", frame);
	cv::waitKey(1);
}

