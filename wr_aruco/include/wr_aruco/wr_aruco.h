#ifndef __WR_AUCO
#define __WR_AUCO

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
	cv::Mat distortionCoefficients_;

	cv::Ptr<cv::aruco::Dictionary> dictionary_;
};

#endif
