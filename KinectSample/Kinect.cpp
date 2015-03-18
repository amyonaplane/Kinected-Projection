#include "Kinect.h"

#include <Windows.h>
#include <NuiApi.h>

#include <iostream>

std::vector<std::unique_ptr<Kinect>> Kinect::kinects_(0);

size_t Kinect::count() {
	int n;
	NuiGetSensorCount(&n);
	if (n < 0) return 0;
	return n;
}

Kinect& Kinect::get(size_t ix) {
	size_t n = count();
	if (ix >= n) {
		throw kinect_exception();
	}

	if (kinects_.size() < n) {
		kinects_.resize(n);
	}

	if (!kinects_[ix]) {
		kinects_[ix] = std::unique_ptr<Kinect>(new Kinect());
		HRESULT hr = NuiCreateSensorByIndex(int(ix), &kinects_[ix]->pSensor_);
		if (hr != S_OK) {
			throw kinect_exception();
		}
		hr = kinects_[ix]->pSensor_->NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH);
		if (hr != S_OK) {
			throw kinect_exception();
		}
	}

	return *kinects_[ix];
}

Kinect::Kinect() : 
initialised_(false),
pSensor_(NULL),
videoResolution_(NUI_IMAGE_RESOLUTION_640x480),
hVideoStream_(NULL),
videoFrame_(),
haveVideoFrame_(false),
depthResolution_(NUI_IMAGE_RESOLUTION_640x480),
hDepthStream_(NULL),
depthFrame_(),
haveDepthFrame_(false) {
	
}

Kinect::~Kinect() {
	if (pSensor_) {
		pSensor_->NuiShutdown();
	}
}

int Kinect::getAngle() {
	LONG angle;
	pSensor_->NuiCameraElevationGetAngle(&angle);
	return angle;
}

void Kinect::setAngle(int angle) {
	pSensor_->NuiCameraElevationSetAngle(angle);
}

void Kinect::openVideoStream(Resolution resolution) {
	HANDLE videoEvent_ = CreateEvent(NULL, TRUE, FALSE, NULL);
	switch (resolution) {
	case RESOLUTION_320x240:
		depthResolution_ = NUI_IMAGE_RESOLUTION_320x240;
		break;
	case RESOLUTION_640x480:
		depthResolution_ = NUI_IMAGE_RESOLUTION_640x480;
		break;
	case RESOLUTION_1280x960:
		depthResolution_ = NUI_IMAGE_RESOLUTION_1280x960;
		break;
	default:
		throw kinect_exception();
		break;
	}
	HRESULT hr = pSensor_->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, depthResolution_, 0, 2, videoEvent_, &hVideoStream_);
	if (hr != S_OK) {
		throw kinect_exception();
	}
}


bool Kinect::grabVideoFrame() {
	HRESULT hr;
	
	if (haveVideoFrame_) {
		hr = pSensor_->NuiImageStreamReleaseFrame(hVideoStream_, &videoFrame_);
		if (hr != S_OK) {
			throw kinect_exception();
		}
		haveVideoFrame_ = false;
	}
	
	hr = pSensor_->NuiImageStreamGetNextFrame(hVideoStream_, 100, &videoFrame_);
	haveVideoFrame_ = (hr == S_OK);
	if (FAILED(hr)) {
		std::cout << "FAIL: " << HRESULT_CODE(hr) << std::endl;
	} 
		
	return haveVideoFrame_;
}

void Kinect::fetchVideoImage(cv::Mat* img) {
	if (!haveVideoFrame_) {
		img->setTo(cv::Scalar(0));
		return;
	}
	
	if (hVideoStream_ == NULL) {
		throw kinect_exception();
	}

	if (img->channels() != 3 || img->size().width != videoWidth() || img->size().height != videoHeight()) {
		*img = cv::Mat(cv::Size(videoWidth(), videoHeight()), CV_8UC3);
	}

	NUI_LOCKED_RECT rect;
	RECT blah;
	cv::Vec3b col;
	videoFrame_.pFrameTexture->LockRect(0, &rect, &blah, 0);
	for (int y = 0; y < img->size().height; ++y) {
		for (int x = 0; x < img->size().width; ++x) {
			col[0] = rect.pBits[rect.Pitch*y + 4*x];
			col[1] = rect.pBits[rect.Pitch*y + 4*x+1];
			col[2] = rect.pBits[rect.Pitch*y + 4*x+2];
			img->at<cv::Vec3b>(y,x) = col;
		}
	}
	videoFrame_.pFrameTexture->UnlockRect(0);
	
}

long int Kinect::videoWidth() {
	switch (videoResolution_) {
	case NUI_IMAGE_RESOLUTION_320x240:
		return 320;
		break;
	case NUI_IMAGE_RESOLUTION_640x480:
		return 640;
		break;
	case NUI_IMAGE_RESOLUTION_1280x960:
		return 1280;
		break;
	default:
		return 0;
		break;
	}
}

long int Kinect::videoHeight() {
	switch (videoResolution_) {
	case NUI_IMAGE_RESOLUTION_320x240:
		return 240;
		break;
	case NUI_IMAGE_RESOLUTION_640x480:
		return 480;
		break;
	case NUI_IMAGE_RESOLUTION_1280x960:
		return 960;
		break;
	default:
		return 0;
		break;
	}
}



void Kinect::openDepthStream(Resolution resolution) {
	HANDLE depthEvent_ = CreateEvent(NULL, TRUE, FALSE, NULL);
	switch (resolution) {
	case RESOLUTION_80x60:
		depthResolution_ = NUI_IMAGE_RESOLUTION_80x60;
		break;
	case RESOLUTION_320x240:
		depthResolution_ = NUI_IMAGE_RESOLUTION_320x240;
		break;
	case RESOLUTION_640x480:
		depthResolution_ = NUI_IMAGE_RESOLUTION_640x480;
		break;
	default:
		throw kinect_exception();
		break;
	}
	HRESULT hr = pSensor_->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, depthResolution_, 0, 2, depthEvent_, &hDepthStream_);
		
	if (hr != S_OK) {
		throw kinect_exception();
	}
}

bool Kinect::grabDepthFrame() {
	HRESULT hr;
	
	if (haveDepthFrame_) {
		hr = pSensor_->NuiImageStreamReleaseFrame(hDepthStream_, &depthFrame_);
		if (hr != S_OK) {
			throw kinect_exception();
		}
		haveDepthFrame_ = false;
	}
	
	hr = pSensor_->NuiImageStreamGetNextFrame(hDepthStream_, 100, &depthFrame_);
	haveDepthFrame_ = (hr == S_OK);
	if (FAILED(hr)) {
		std::cout << "FAIL: " << HRESULT_CODE(hr) << std::endl;
	} 
		
	return haveDepthFrame_;
}

void Kinect::fetchDepthImage(cv::Mat *img) {
	if (!haveDepthFrame_) {
		img->setTo(cv::Scalar(0));
		return;
	}
	
	if (hDepthStream_ == NULL) {
		throw kinect_exception();
	}

	if (img->channels() != 1 || img->size().width != depthWidth() || img->size().height != depthHeight()) {
		*img = cv::Mat(cv::Size(depthWidth(), depthHeight()), CV_8UC1);
	}

	NUI_LOCKED_RECT rect;
	RECT blah;
	const int maxDepth = 1 << 15;
	depthFrame_.pFrameTexture->LockRect(0, &rect, &blah, 0);
	for (int y = 0; y < img->size().height; ++y) {
		for (int x = 0; x < img->size().width; ++x) {

			USHORT depth = rect.pBits[y*rect.Pitch + 2*x+1] << 8;
			depth += rect.pBits[y*rect.Pitch + 2*x];
		
			img->at<unsigned char>(y,x) = 255*depth/maxDepth;

		}
	}
	depthFrame_.pFrameTexture->UnlockRect(0);
}

long int Kinect::depthWidth() {
	switch (depthResolution_) {
	case NUI_IMAGE_RESOLUTION_80x60:
		return 80;
		break;
	case NUI_IMAGE_RESOLUTION_320x240:
		return 320;
		break;
	case NUI_IMAGE_RESOLUTION_640x480:
		return 640;
		break;
	default:
		return 0;
		break;
	}
}

long int Kinect::depthHeight() {
	switch (depthResolution_) {
	case NUI_IMAGE_RESOLUTION_80x60:
		return 60;
		break;
	case NUI_IMAGE_RESOLUTION_320x240:
		return 240;
		break;
	case NUI_IMAGE_RESOLUTION_640x480:
		return 480;
		break;
	default:
		return 0;
		break;
	}
}

void Kinect::setDepthMode(DepthMode depthMode) {
	DWORD flags;
	pSensor_->NuiImageStreamGetImageFrameFlags(hDepthStream_, &flags);
	if (depthMode == DEPTH_MODE_NEAR) {
		flags |= NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE;
	} else {
		flags &= !NUI_IMAGE_STREAM_FLAG_ENABLE_NEAR_MODE;
	}
	pSensor_->NuiImageStreamSetImageFrameFlags(hDepthStream_, flags);

}

cv::Point3d Kinect::depth2xyz(const cv::Point2d& depth) {
	long int dx = depth.x;
	long int dy = depth.y;
	unsigned short dd;
	NUI_LOCKED_RECT rectDepth;
	RECT blah;
	depthFrame_.pFrameTexture->LockRect(0, &rectDepth, &blah, 0);
	long int yOffset = dy*rectDepth.Pitch;
	dd = rectDepth.pBits[yOffset + 2*dx+1] << 8;
	dd += rectDepth.pBits[yOffset + 2*dx];
	depthFrame_.pFrameTexture->UnlockRect(0);

	if (dd > 0) {
		Vector4 pnt3d = NuiTransformDepthImageToSkeleton(dx, dy, dd, depthResolution_);
		return cv::Point3d(pnt3d.x, pnt3d.y, pnt3d.z);
	} else {
		return cv::Point3d(0,0,-1);
	}

	
}

cv::Point2d Kinect::depth2video(const cv::Point2d& depth) {
	long int dx = depth.x;
	long int dy = depth.y;
	unsigned short dd;
	NUI_LOCKED_RECT rectDepth;
	RECT blah;
	depthFrame_.pFrameTexture->LockRect(0, &rectDepth, &blah, 0);
	long int yOffset = dy*rectDepth.Pitch;
	dd = rectDepth.pBits[yOffset + 2*dx+1] << 8;
	dd += rectDepth.pBits[yOffset + 2*dx];
	depthFrame_.pFrameTexture->UnlockRect(0);
	long int vx, vy;
	if (dd > 0) {
		pSensor_->NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
			videoResolution_, depthResolution_, &videoFrame_.ViewArea, dx, dy, dd, &vx, &vy);
		return cv::Point2d(vx, vy);
	} else {
		return cv::Point2d(-1,-1);
	}
}
