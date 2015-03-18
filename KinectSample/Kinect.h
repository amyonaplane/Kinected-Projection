#pragma once

// Class to wrap the code for a single Kinect sensor
// Since there can be only one, it's a singleton

#include <opencv/cv.h>

#include <Windows.h>
#include <NuiApi.h>

#include <exception>
#include <memory>
#include <vector>

class kinect_exception : public std::exception {};

class Kinect {

public:

	enum Resolution {
		RESOLUTION_80x60,
		RESOLUTION_320x240,
		RESOLUTION_640x480,
		RESOLUTION_1280x960
	};

	enum DepthMode {
		DEPTH_MODE_NEAR,
		DEPTH_MODE_FAR
	};

	static size_t count();

	static Kinect& get(size_t ix);

	~Kinect();

	int getAngle();
	void setAngle(int angle);
	
	void openVideoStream(Resolution resolution = RESOLUTION_640x480);
	bool grabVideoFrame();
	void fetchVideoImage(cv::Mat *img);
	long int videoWidth();
	long int videoHeight();

	void openDepthStream(Resolution resolution = RESOLUTION_640x480);
	bool grabDepthFrame();
	void fetchDepthImage(cv::Mat *img);
	long int depthWidth();
	long int depthHeight();

	void setDepthMode(DepthMode depthMode);

	cv::Point3d depth2xyz(const cv::Point2d& depth);
	cv::Point2d depth2video(const cv::Point2d& depth);

protected:
	
	Kinect();
	
	bool initialised_;
	INuiSensor* pSensor_;

	NUI_IMAGE_RESOLUTION videoResolution_;
	HANDLE hVideoStream_;
	NUI_IMAGE_FRAME videoFrame_;
	bool haveVideoFrame_;
	HANDLE videoEvent_;

	NUI_IMAGE_RESOLUTION depthResolution_;
	HANDLE hDepthStream_;
	NUI_IMAGE_FRAME depthFrame_;
	bool haveDepthFrame_;
	HANDLE depthEvent_;


	static std::vector<std::unique_ptr<Kinect>> kinects_;

private:

	// Non-copyable semantics
	Kinect(const Kinect&);
	Kinect& operator=(const Kinect&);

};

