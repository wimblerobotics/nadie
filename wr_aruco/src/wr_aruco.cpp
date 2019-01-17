#include <ros/console.h>

#include <assert.h>
#include <boost/algorithm/string.hpp>
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

WrAruco::WrAruco(ros::NodeHandle& nh)
	: nh_(nh) {
  	assert(nh_.param<int>("wr_aruco/aruco_dictionay_number", arucoDictionaryNumber_, 7));
	assert(nh_.param<std::string>("wr_aruco/camera_intrinsics_path", cameraIntrinsicsPath_, ""));
	assert(nh_.param<int>("wr_aruco/video_device_number", videoDeviceNumber_, 0));
	assert(nh_.param<int>("wr_aruco/video_frame_height", videoFrameHeight_, 480));
	assert(nh_.param<int>("wr_aruco/video_frame_width", videoFrameWidth_, 640));
	assert(nh_.param<bool>("wr_aruco/visualize_markers", visualizeMarkers_, false));

	fiducialMapEntryArrayPub_ = nh_.advertise<wr_aruco::FiducialMapEntryArray>("/fiducial_map", 1, false);
	callbackType_ = boost::bind(&WrAruco::configCallback, this, _1, _2);
    configServer_.setCallback(callbackType_);
}


void WrAruco::configCallback(wr_aruco::DetectorParamsConfig & config, uint32_t level)
{
    /* Don't load initial config, since it will overwrite the rosparam settings */
    if (level == 0xFFFFFFFF) {
        return;
    }

    detectorParams_->adaptiveThreshConstant = config.adaptiveThreshConstant;
    detectorParams_->adaptiveThreshWinSizeMin = config.adaptiveThreshWinSizeMin;
    detectorParams_->adaptiveThreshWinSizeMax = config.adaptiveThreshWinSizeMax;
    detectorParams_->adaptiveThreshWinSizeStep = config.adaptiveThreshWinSizeStep;
    detectorParams_->cornerRefinementMaxIterations = config.cornerRefinementMaxIterations;
    detectorParams_->cornerRefinementMinAccuracy = config.cornerRefinementMinAccuracy;
    detectorParams_->cornerRefinementWinSize = config.cornerRefinementWinSize;
    if (config.doCornerRefinement) {
       if (config.cornerRefinementSubpix) {
         detectorParams_->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
       }
       else {
         detectorParams_->cornerRefinementMethod = aruco::CORNER_REFINE_CONTOUR;
       }
    }
    else {
       detectorParams_->cornerRefinementMethod = aruco::CORNER_REFINE_NONE;
    }

    detectorParams_->errorCorrectionRate = config.errorCorrectionRate;
    detectorParams_->markerBorderBits = config.markerBorderBits;
    detectorParams_->maxErroneousBitsInBorderRate = config.maxErroneousBitsInBorderRate;
    detectorParams_->maxMarkerPerimeterRate = config.maxMarkerPerimeterRate;
    detectorParams_->minCornerDistanceRate = config.minCornerDistanceRate;
    detectorParams_->minDistanceToBorder = config.minDistanceToBorder;
    detectorParams_->minMarkerDistanceRate = config.minMarkerDistanceRate;
    detectorParams_->minMarkerPerimeterRate = config.minMarkerPerimeterRate;
    detectorParams_->minOtsuStdDev = config.minOtsuStdDev;
    detectorParams_->perspectiveRemoveIgnoredMarginPerCell = config.perspectiveRemoveIgnoredMarginPerCell;
    detectorParams_->perspectiveRemovePixelPerCell = config.perspectiveRemovePixelPerCell;
    detectorParams_->polygonalApproxAccuracyRate = config.polygonalApproxAccuracyRate;
}


bool WrAruco::init() {
	ROS_INFO("[WrAruco::init] Loading camera intrinsics file: %s", cameraIntrinsicsPath_.c_str());
    const std::string inputSettingsFile = cameraIntrinsicsPath_.c_str();
	cv::FileStorage fs = cv::FileStorage(inputSettingsFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        ROS_ERROR("[WrAruco::init] Could not open the configuration file: \"%s\"", inputSettingsFile.c_str());;
        return false;
    }

    fs["camera_matrix"] >> cameraMatrix_;
    fs["distortion_coefficients"] >> distortionCoefficients_;
   
    ROS_INFO_STREAM("[WrAruco::init] camera_matrix: " << cameraMatrix_);
    ROS_INFO_STREAM("[WrAruco::init] distortion_coefficients: " << distortionCoefficients_);
    ROS_INFO("[WrAruco::init] Intrinsics image_width: %f, image_height: %f", fs["image_width"].real(), fs["image_height"].real());
	assert(fs["image_height"].real() == videoFrameHeight_);
	assert(fs["image_width"].real() == videoFrameWidth_);
	
	fs.release(); 

	setDetectorParameters();

	// Initialize the aruco dictionary.
	dictionary_ = aruco::getPredefinedDictionary(arucoDictionaryNumber_);

	cap_ = new cv::VideoCapture(videoDeviceNumber_);
	if (!cap_->isOpened()) {
		ROS_ERROR("[WrAruco::init] VideoCapture FAILED");
		return false;
	}

	/*###
	  At 1280x720, I get about 3.5 fps throughput.
	  At 640x480, about 11
	  ###*/
	cap_->set(CV_CAP_PROP_FRAME_HEIGHT, videoFrameHeight_);
    cap_->set(CV_CAP_PROP_FRAME_WIDTH, videoFrameWidth_);

	prev_ = ros::Time::now();
	return true;
}

void WrAruco::displayVideoFrameStats(cv::Mat& frame, std::vector<int>& ids, std::vector<cv::Vec3d>& rvecs, std::vector<cv::Vec3d>& tvecs) {
	// std::cout << "corners: " << std::endl; for (std::vector <std::vector <cv::Point2f> >::iterator i = corners.begin(); i != corners.end(); ++i) std::cout << *i << std::endl;
	// if (visualizeMarkers) {
	// 	cv::aruco::drawDetectedMarkers(frame, corners, ids);
	// }

	int idIndex = 0;
	std::cout << "  --  " << std::endl;
	std::cout << "rvecs: " << std::endl;
	for (std::vector<cv::Vec3d>::iterator i = rvecs.begin(); i != rvecs.end(); ++i) {
		std::cout << ids[idIndex++] << "  " << *i << std::endl;
	}
	
	idIndex = 0;
	std::cout << "tvecs: " << std::endl;
	for (std::vector<cv::Vec3d>::iterator i = tvecs.begin(); i != tvecs.end(); ++i) {
		std::cout << ids[idIndex++] << "  " << *i << std::endl;
	}

	// draw axis for each marker
	for (unsigned int i = 0; i < ids.size(); i++) {
		cv::aruco::drawAxis(frame, cameraMatrix_, distortionCoefficients_, rvecs[i], tvecs[i], 0.1);
	}

	int id1 = -1;
	int id2 = -1;
	int id3 = -1;
	int id5 = -1;
	int id6 = -1;
	int id7 = -1;
	idIndex = 0;
	for (std::vector<int>::iterator i = ids.begin(); i != ids.end(); ++i) {
		if (*i == 1) id1 = idIndex;
		if (*i == 2) id2 = idIndex;
		if (*i == 3) id3 = idIndex;
		if (*i == 5) id5 = idIndex;
		if (*i == 6) id6 = idIndex;
		if (*i == 7) id7 = idIndex;
		idIndex++;
	}

	if ((id1 != -1) && (id2 != -1)) std::cout << "1->2 " << tvecs[id2][0] - tvecs[id1][0] << std::endl;
	if ((id2 != -1) && (id3 != -1)) std::cout << "2->3 " << tvecs[id3][0] - tvecs[id2][0] << std::endl;
	if ((id1 != -1) && (id5 != -1)) std::cout << "1->5 " << tvecs[id5][1] - tvecs[id1][1] << std::endl;
	if ((id2 != -1) && (id6 != -1)) std::cout << "2->6 " << tvecs[id6][1] - tvecs[id2][1] << std::endl;
	if ((id3 != -1) && (id7 != -1)) std::cout << "3->7 " << tvecs[id7][1] - tvecs[id3][1] << std::endl;

    ros::Time now = ros::Time::now();
    ros::Duration duration = now - prev_;
    std::cout << "duration: " << duration.toSec() << ", fps: " << (1.0 / duration.toSec()) << std::endl;

	cv::imshow("pic", frame);
	cv::waitKey(1);
}

void WrAruco::processVideoFrame() {
	cv::Mat	frame;
	*cap_ >> frame;
	cv::flip(frame, frame, -1);

	std::vector<int> ids;
	std::vector <std::vector <cv::Point2f> > corners;
	cv::aruco::detectMarkers(frame, dictionary_, corners, ids, detectorParams_);
	//ROS_INFO("[wr_aruco::processVideoFrame] Detected %d markers", (int) ids.size());

	if(ids.size() > 0) {
		std::vector<cv::Vec3d> rvecs;
		std::vector<cv::Vec3d> tvecs;
		cv::aruco::estimatePoseSingleMarkers(corners, 0.140, cameraMatrix_, distortionCoefficients_, rvecs, tvecs);

		if (visualizeMarkers_) {
			displayVideoFrameStats(frame, ids, rvecs, tvecs);
		}

		publishMap(ids, rvecs, tvecs);
    }
}


void WrAruco::publishMap(std::vector<int>& ids, std::vector<cv::Vec3d>& rvecs, std::vector<cv::Vec3d>& tvecs) {
	wr_aruco::FiducialMapEntryArray fiducialMapEntryArray;
	for (unsigned int i = 0; i < ids.size(); i++) {
		wr_aruco::FiducialMapEntry fiducialMapEntry;
		fiducialMapEntry.fiducial_id = ids[i];
		fiducialMapEntry.x = tvecs[i][0];
		fiducialMapEntry.y = tvecs[i][1];
		fiducialMapEntry.z = tvecs[i][2];
		fiducialMapEntry.rx = rvecs[i][0];
		fiducialMapEntry.ry = rvecs[i][1];
		fiducialMapEntry.rz = rvecs[i][2];
		fiducialMapEntryArray.fiducials.push_back(fiducialMapEntry);
	}

	fiducialMapEntryArrayPub_.publish(fiducialMapEntryArray);
}



void WrAruco::setDetectorParameters() {

    detectorParams_ = new aruco::DetectorParameters();

	assert(nh_.param<bool>  ("wr_aruco/visualize_markers", visualizeMarkers_, false));
   	assert(nh_.param<double>("wr_aruco/adaptiveThreshConstant", detectorParams_->adaptiveThreshConstant, 7));
    assert(nh_.param<int>   ("wr_aruco/adaptiveThreshWinSizeMax", detectorParams_->adaptiveThreshWinSizeMax, 53));
    assert(nh_.param<int>   ("wr_aruco/adaptiveThreshWinSizeMin", detectorParams_->adaptiveThreshWinSizeMin, 3));
    assert(nh_.param<int>   ("wr_aruco/adaptiveThreshWinSizeStep", detectorParams_->adaptiveThreshWinSizeStep, 4));
    assert(nh_.param<int>   ("wr_aruco/cornerRefinementMaxIterations", detectorParams_->cornerRefinementMaxIterations, 30));
    assert(nh_.param<double>("wr_aruco/cornerRefinementMinAccuracy", detectorParams_->cornerRefinementMinAccuracy, 0.01)); /* default 0.1 */
    assert(nh_.param<int>   ("wr_aruco/cornerRefinementWinSize", detectorParams_->cornerRefinementWinSize, 5));
    bool doCornerRefinement = true;
    assert(nh_.param<bool>  ("wr_aruco/doCornerRefinement", doCornerRefinement, true));
    if (doCornerRefinement) {
       bool cornerRefinementSubPix = true;
       assert(nh_.param<bool>("wr_aruco/cornerRefinementSubPix", cornerRefinementSubPix, true));
       if (cornerRefinementSubPix) {
         detectorParams_->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;
       }
       else {
         detectorParams_->cornerRefinementMethod = aruco::CORNER_REFINE_CONTOUR;
       }
    }
    else {
       detectorParams_->cornerRefinementMethod = aruco::CORNER_REFINE_NONE;
    }

    assert(nh_.param<double>("wr_aruco/errorCorrectionRate", detectorParams_->errorCorrectionRate , 0.6));
    assert(nh_.param<int>   ("wr_aruco/markerBorderBits", detectorParams_->markerBorderBits, 1));
    assert(nh_.param<double>("wr_aruco/maxErroneousBitsInBorderRate", detectorParams_->maxErroneousBitsInBorderRate, 0.04));
    assert(nh_.param<double>("wr_aruco/maxMarkerPerimeterRate", detectorParams_->maxMarkerPerimeterRate, 4.0));
    assert(nh_.param<double>("wr_aruco/minCornerDistanceRate", detectorParams_->minCornerDistanceRate , 0.05));
    assert(nh_.param<int>   ("wr_aruco/minDistanceToBorder", detectorParams_->minDistanceToBorder, 3));
    assert(nh_.param<double>("wr_aruco/minMarkerDistanceRate", detectorParams_->minMarkerDistanceRate, 0.05));
    assert(nh_.param<double>("wr_aruco/minMarkerPerimeterRate", detectorParams_->minMarkerPerimeterRate, 0.1)); /* default 0.3 */
    assert(nh_.param<double>("wr_aruco/minOtsuStdDev", detectorParams_->minOtsuStdDev, 5.0));
    assert(nh_.param<double>("wr_aruco/perspectiveRemoveIgnoredMarginPerCell", detectorParams_->perspectiveRemoveIgnoredMarginPerCell, 0.13));
    assert(nh_.param<int>   ("wr_aruco/perspectiveRemovePixelPerCell", detectorParams_->perspectiveRemovePixelPerCell, 8));
    assert(nh_.param<double>("wr_aruco/polygonalApproxAccuracyRate", detectorParams_->polygonalApproxAccuracyRate, 0.01)); /* default 0.05 */
}


