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

    const std::string inputSettingsFile = "/home/wimble/catkin_ws/src/nadie/wr_aruco/cfg/640x480_ost.yaml";
	cv::FileStorage fs = cv::FileStorage(inputSettingsFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        ROS_ERROR("Could not open the configuration file: \"%s\"", inputSettingsFile.c_str());;
        return false;
    }

    fs["camera_matrix"] >> cameraMatrix_;
    fs["distortion_coefficients"] >> distortionCoefficients_;
    fs.release(); 

    ROS_INFO_STREAM("[WrAruco::init] camera_matrix: " << cameraMatrix_);
    ROS_INFO_STREAM("[WrAruco::init] distortion_coefficients: " << distortionCoefficients_);
    ROS_INFO("[WrAruco::init] image_width: %f, image_height: %f", fs["image_width"].real(), fs["image_height"].real());

    detectorParams_ = new aruco::DetectorParameters();
	detectorParams_->adaptiveThreshConstant = 7;
    detectorParams_->adaptiveThreshWinSizeMax = 53; /* defailt 23 */
    detectorParams_->adaptiveThreshWinSizeMin = 3;
    detectorParams_->adaptiveThreshWinSizeStep = 4; /* default 10 */
    detectorParams_->cornerRefinementMaxIterations = 30;
    detectorParams_->cornerRefinementMinAccuracy = 0.01; /* default 0.1 */
    detectorParams_->cornerRefinementWinSize = 5;
	detectorParams_->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
    detectorParams_->errorCorrectionRate = 0.6;
    detectorParams_->minCornerDistanceRate = 0.05;
    detectorParams_->markerBorderBits = 1;
    detectorParams_->maxErroneousBitsInBorderRate = 0.04;
    detectorParams_->minDistanceToBorder = 3;
    detectorParams_->minMarkerDistanceRate = 0.05;
    detectorParams_->minMarkerPerimeterRate = 0.1; /* default 0.3 */
    detectorParams_->maxMarkerPerimeterRate = 4.0;
    detectorParams_->minOtsuStdDev = 5.0;
    detectorParams_->perspectiveRemoveIgnoredMarginPerCell = 0.13;
    detectorParams_->perspectiveRemovePixelPerCell = 8;
    detectorParams_->polygonalApproxAccuracyRate = 0.01; /* default 0.05 */

	// Initialize the aruco dictionary.
	dictionary_ = aruco::getPredefinedDictionary(7/*###*/);

	cap_ = new cv::VideoCapture(0);
	if (!cap_->isOpened()) {
		ROS_ERROR("[WrAruco::init] VideoCapture FAILED");
		return false;
	}

	/*###
	  At 1280x720, I get about 3.5 fps throughput.
	  At 640x480, about 11
	  ###*/
    // cap_->set(CV_CAP_PROP_FRAME_HEIGHT, 720);
    // cap_->set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	cap_->set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    cap_->set(CV_CAP_PROP_FRAME_WIDTH, 640);
	ROS_INFO("[WrAruco::init] VideoCapture successful");

	prev_ = ros::Time::now();
	return true;
}

void WrAruco::processVideoFrame() {
	const bool visualizeMarkers = false;
	cv::Mat	frame;
	*cap_ >> frame;
	cv::flip(frame, frame, -1);

	//ROS_INFO("[WrAruco::processVideoFrame] width: %d, height: %d", frame.size().width, frame.size().height);//#####

	// cv::Mat undistortedFrame;
	// cv::undistort(frame, undistortedFrame, cameraMatrix_, distortionCoefficients_);

	std::vector<int> ids;
	std::vector <std::vector <cv::Point2f> > corners;
	cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detectorParams_);
	//ROS_INFO("[wr_aruco::processVideoFrame] Detected %d markers", (int) ids.size());

	if(ids.size() > 0) {
		// std::cout << "corners: " << std::endl; for (std::vector <std::vector <cv::Point2f> >::iterator i = corners.begin(); i != corners.end(); ++i) std::cout << *i << std::endl;
		// if (visualizeMarkers) {
		// 	cv::aruco::drawDetectedMarkers(frame, corners, ids);
		// }

		std::vector<cv::Vec3d> rvecs;
		std::vector<cv::Vec3d> tvecs;
		cv::aruco::estimatePoseSingleMarkers(corners, 0.140, cameraMatrix_, distortionCoefficients_, rvecs, tvecs);

		// std::cout << "rvecs: ";
		// for (auto i = rvecs.begin(); i != rvecs.end(); ++i) std::cout << *i << ' ';
		// std::cout << "  --  ";

		std::cout << "tvecs: " << std::endl;
		int idIndex = 0;
		for (std::vector<cv::Vec3d>::iterator i = tvecs.begin(); i != tvecs.end(); ++i) {
			std::cout << ids[idIndex++] << "  " << *i << std::endl;
		}

     //    // draw axis for each marker
     //    for (unsigned int i = 0; i < ids.size(); i++) {
     //    	std::cout << "id" << ids[i] << std::endl;
     //        cv::aruco::drawAxis(frame, cameraMatrix_, distortionCoefficients_, rvecs[i], tvecs[i], 0.1);
    	// }
    }

    ros::Time now = ros::Time::now();
    ros::Duration duration = now - prev_;
    std::cout << "duration: " << duration.toSec() << ", fps: " << (1.0 / duration.toSec()) << std::endl;
	prev_ = now;

   	if (visualizeMarkers) {
   		cv::imshow("pic", frame);
		cv::waitKey(1);
	}
}

