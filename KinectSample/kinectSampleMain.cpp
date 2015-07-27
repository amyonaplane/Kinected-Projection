#include "Kinect.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include "Eigen/Eigen/Eigen"
#include "Eigen/Eigen/StdVector"
#include "Eigen/Eigen/SVD"
#include "Eigen/Eigen/Jacobi"

#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;
using namespace Eigen;

int main (int argc, char *argv[]) {

	char pattern[100];
	sprintf_s(pattern, 100, "%s%%03d.ply", argv[1]);
	int modelNum = atoi(argv[2]);

	Mat video(cv::Size(200,200), CV_8UC3);
	video.setTo(cv::Scalar(128,128,128));

	Mat depth(cv::Size(200,200), CV_8UC1);
	depth.setTo(cv::Scalar(128,128,128));

	Mat lightImage, darkImage, greyHomographyImage, projector2KinectHomography, kinect2RealHomography;

	vector<Point2f> kinect2ProjectorCorners, projectorCorners, representationCorners, realCorners;

	Size projectorBoard_sz=Size(8,6);
	Size realBoard_sz=Size(12,7);

	bool imageCornersFound, projectorCornersFound;

	Mat chessBoardImage=imread("127chessboard.jpg");
	cvtColor(chessBoardImage,chessBoardImage, CV_RGB2GRAY);
	imageCornersFound = findChessboardCorners(chessBoardImage,realBoard_sz,representationCorners,CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
	if(imageCornersFound){
		cornerSubPix(chessBoardImage, representationCorners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
	}

	Mat projectorImage=imread("checkerboard.png");
	cvtColor(projectorImage,projectorImage, CV_RGB2GRAY);
	projectorCornersFound = findChessboardCorners(projectorImage,projectorBoard_sz,projectorCorners,CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
	if(projectorCornersFound){
		cornerSubPix(projectorImage, projectorCorners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
	}

	Size projectorSize(1024,768);
	Size mainScreenSize(1920,1080);
	namedWindow("Projector");
	imshow("Projector", projectorImage);
	waitKey(10);

	// Get a Windows handle to the window we made
	HWND windowHandle = FindWindow(0, "Projector");
	if (!windowHandle) {
		cout << "Couldn't find window" << std::endl;
	} else {
   // Make the window full screen with no border or title,e tc.
   SetWindowLongPtr(windowHandle, GWL_STYLE, WS_SYSMENU | 	
     WS_POPUP | WS_CLIPCHILDREN | WS_CLIPSIBLINGS | WS_VISIBLE);

   // Shift the window so it is just to the right of the main screen
   MoveWindow(windowHandle, mainScreenSize.width, 0,
     projectorSize.width, projectorSize.height, TRUE);
	}
	waitKey(10);

	try {
		Kinect& kinect = Kinect::get(0);
		kinect.openVideoStream(Kinect::RESOLUTION_640x480);
		kinect.openDepthStream(Kinect::RESOLUTION_640x480);

		kinect.setDepthMode(Kinect::DEPTH_MODE_NEAR);
		double angle = 5;
		kinect.setAngle(5);
		
		bool done = false;
		bool light = false;
		bool k2PHomographyFound = false;

		while (!done) {

			bool haveVideo = kinect.grabVideoFrame();
			bool haveDepth = kinect.grabDepthFrame();

			if (haveVideo) {
				kinect.fetchVideoImage(&video);
				flip(video,video,1);
			}

			if (haveDepth) {
				kinect.fetchDepthImage(&depth);
				flip(depth,depth,1);
			}

			if (haveVideo && haveDepth) {
				int keyPressed = waitKey(1);
				Matrix3f p2KHomography, k2RHomography;
				cvtColor(video,video,CV_RGB2GRAY);
				imshow("greyvid",video);

				if (keyPressed == 'q' || keyPressed == 'Q') {
					done = true;
				} 
				else if(keyPressed=='f'){
					kinect.fetchVideoImage(&lightImage);
					cvtColor(lightImage, lightImage, CV_RGB2GRAY);
					flip(lightImage,lightImage,1);
					projectorImage=NULL;
					imshow("Projector",projectorImage);
					light=true;
				}
				else if(keyPressed=='y'&&light){
					kinect.fetchVideoImage(&darkImage);
					flip(darkImage,darkImage,1);
					cvtColor(darkImage, darkImage, CV_RGB2GRAY);
					absdiff(lightImage,darkImage,greyHomographyImage);
					Mat greyHomographyCopy=greyHomographyImage;
					imshow("greyImage", greyHomographyCopy);

					bool kinectFound = findChessboardCorners(greyHomographyImage,projectorBoard_sz,kinect2ProjectorCorners,CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
					if(kinectFound){
						cornerSubPix(greyHomographyCopy, kinect2ProjectorCorners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
						drawChessboardCorners(greyHomographyImage, projectorBoard_sz, Mat(kinect2ProjectorCorners), kinectFound);

						projector2KinectHomography=findHomography(kinect2ProjectorCorners, projectorCorners, RANSAC);
						projector2KinectHomography.convertTo(projector2KinectHomography,CV_32FC2);
						warpPerspective(greyHomographyImage, greyHomographyImage, projector2KinectHomography, projectorSize);//size?
						imshow("homography",greyHomographyImage);
						k2PHomographyFound=true;
						waitKey(400);
					}
					projectorImage=NULL;
					imshow("Projector", projectorImage);
				}
				//cover the lens
				else if(keyPressed=='j'&&k2PHomographyFound==true){
					bool realCornersFound=findChessboardCorners(video, realBoard_sz, realCorners, CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
					if(realCornersFound){
						cout<<" homography found ";
						Mat greyVideo;
						cornerSubPix(video, realCorners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
						kinect2RealHomography=findHomography(realCorners, representationCorners, RANSAC);//find homography
						kinect2RealHomography.convertTo(kinect2RealHomography,CV_32FC2);
						warpPerspective(video, greyVideo, kinect2RealHomography, projectorSize);//size?
						imshow("homography vid",greyVideo);
					}
				}
			}
		}

	} catch (kinect_exception) {
		std::cerr << "Caught an exception" << std::endl;
	}

	return 0;
}