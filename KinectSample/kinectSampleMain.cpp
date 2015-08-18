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

	Mat lightImage, darkImage, greyHomographyImage, projector2KinectHomography, kinect2RealHomography, grey, kinectDistortCoeff, projectorDistortCoeff, objectKinectMat, objectProjectorMat,
		stereoR, stereoT, stereoE, stereoF;
	vector<Mat> kinectRvect, kinectTvect, projectorRvect, projectorTvect;
	vector<Point2f> kinect2ProjectorCorners, projectorCorners, representationCorners, realCorners;

	Size projectorBoard_sz=Size(8,6);
	Size realBoard_sz=Size(12,7);

	Mat chessBoardImage=imread("127chessboard.jpg");
	cvtColor(chessBoardImage,chessBoardImage, CV_RGB2GRAY);
	bool imageCornersFound = findChessboardCorners(chessBoardImage,realBoard_sz,representationCorners,CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
	if(imageCornersFound){
		cornerSubPix(chessBoardImage, representationCorners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
	}

	//create object points of real-world chessboard representation to be used in calibration
	vector<vector<Point3f>>objectPoints;
	vector<Point3f>vecPoints;

	//change Point2f into Point3f
	for(int i=0;i<representationCorners.size();i++){
		Point3f point2fToPoint3f(representationCorners[i].x, representationCorners[i].y,0);
		vecPoints.push_back(point2fToPoint3f);
	}
	for(int j=0;j<2;j++){
		objectPoints.push_back(vecPoints);
	}

	//create vector of vectors for image points found from Kinect
	vector<vector<Point2f>>kinectPoints;
	//create vector of vectors for image points found from projector
	vector<vector<Point2f>>projectorPoints;

	Mat projectorImage=imread("checkerboard.png");
	cvtColor(projectorImage,grey, CV_RGB2GRAY);
	bool projectorCornersFound = findChessboardCorners(grey,projectorBoard_sz,projectorCorners,CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
	if(projectorCornersFound){
		cornerSubPix(grey, projectorCorners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
	}

	Size projectorSize(1024,768);
	Size mainScreenSize(1920,1080);
	namedWindow("Projector");
	imshow("Projector", projectorImage);

	// Get a Windows handle to the window we made
	HWND windowHandle = FindWindow(0, "Projector");
	if (!windowHandle) {
		cout << "Couldn't find window" << std::endl;
	} 
	else {
		// Make the window full screen with no border or title,e tc.
		SetWindowLongPtr(windowHandle, GWL_STYLE, WS_SYSMENU | WS_POPUP | WS_CLIPCHILDREN | WS_CLIPSIBLINGS | WS_VISIBLE);

		//Shift the window so it is just to the right of the main screen
		MoveWindow(windowHandle, mainScreenSize.width, 0, projectorSize.width, projectorSize.height, TRUE);
	}

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
		bool realCornersFound = false;

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

				//calibrate the Kinect
				else if(keyPressed=='z'&&kinectPoints.size()<2){
					bool realCornersFound=findChessboardCorners(video, realBoard_sz, realCorners, CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
					if(realCornersFound){
						cornerSubPix(video, realCorners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
						kinectPoints.push_back(realCorners);
						cout<<"K"<<kinectPoints.size()<<" ";
						if(kinectPoints.size()==2){
							calibrateCamera(objectPoints, kinectPoints, Size(640,480), objectKinectMat, kinectDistortCoeff, kinectRvect, kinectTvect);
							cout<<objectKinectMat;
						}
					}
				}

				//find projector corners
				else if(keyPressed=='f'&&light==false&&k2PHomographyFound==false){
					kinect.fetchVideoImage(&lightImage);
					cvtColor(lightImage, lightImage, CV_RGB2GRAY);
					flip(lightImage,lightImage,1);
					projectorImage=NULL;
					imshow("Projector", projectorImage);
					light=true;
				}

				else if(keyPressed=='y'&&light&&k2PHomographyFound==false){
					kinect.fetchVideoImage(&darkImage);
					flip(darkImage,darkImage,1);
					cvtColor(darkImage, darkImage, CV_RGB2GRAY);
					absdiff(lightImage,darkImage,greyHomographyImage);
					Mat greyHomographyCopy=greyHomographyImage;
					bool kinectFound = findChessboardCorners(greyHomographyImage,projectorBoard_sz,kinect2ProjectorCorners,CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
					if(kinectFound){
						cout<<"-k2P found-";
						cornerSubPix(greyHomographyCopy, kinect2ProjectorCorners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
						projector2KinectHomography=findHomography(kinect2ProjectorCorners, projectorCorners, RANSAC);
						//convert to Eigen
						for(int i=0;i<projector2KinectHomography.rows;i++){
							for(int j=0;j<projector2KinectHomography.cols;j++){
								p2KHomography(i,j)=projector2KinectHomography.at<double>(i,j);//convert to eigen
							}
						}
						k2PHomographyFound=true;
					}
					else{
						light=false;
						projectorImage=imread("checkerboard.png");
						imshow("Projector", projectorImage);
					}
				}

				//if not working originally, cover the lens
				else if(keyPressed=='j'&&light&&k2PHomographyFound&&projectorPoints.size()<2){
					realCornersFound=findChessboardCorners(video, realBoard_sz, realCorners, CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
					
					if(realCornersFound){
						vector<Point2f>u;
						u.clear();
						cout<<"-k2R found-";
						cornerSubPix(video, realCorners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
						kinect2RealHomography=findHomography(realCorners, representationCorners, RANSAC);//find homography
						//convert to Eigen
						for(int i=0;i<kinect2RealHomography.rows;i++){
							for(int j=0;j<kinect2RealHomography.cols;j++){
								k2RHomography(i,j)=kinect2RealHomography.at<double>(i,j);//save as eigen matrix
							}
						}
						Mat greyVideo;
						Matrix3f p2RHomography(p2KHomography*k2RHomography.inverse());
						Mat p2R=projector2KinectHomography*kinect2RealHomography.inv();
						
						for(Point2f point:representationCorners){//for each corner found from Kinect in k2Real-World chessboard						
							Vector3f point2Vector(point.x,point.y,1);//convert point into vector
							Vector3f vectorHomography(p2RHomography*point2Vector);//multiply vector by homograpy inverse
							vectorHomography/=vectorHomography(2);//divide by p(2) - generalize
							//cout<<vectorHomography(0)<<" ";
							//cout<<vectorHomography(1)<<endl;
							circle(projectorImage,Point2f(vectorHomography(0),vectorHomography(1)),3,Scalar(0,255,0),3);//print circle on found points
							u.push_back(Point2f(vectorHomography(0),vectorHomography(1)));
						}
						projectorPoints.push_back(u);
						if(projectorPoints.size()==2){
							calibrateCamera(objectPoints, projectorPoints, projectorSize, objectProjectorMat, projectorDistortCoeff, projectorRvect, projectorTvect);
							cout<<objectProjectorMat;
						}
						imshow("Projector", projectorImage);
					}
					else{
						k2PHomographyFound=false;
						light=false;
						projectorImage=imread("checkerboard.png");
						imshow("Projector", projectorImage);	
					}
				}
				else if(keyPressed=='k'&&realCornersFound){
					k2PHomographyFound=false;
					realCornersFound=false;
						light=false;
						projectorImage=imread("checkerboard.png");
						imshow("Projector", projectorImage);	
				}
				else if(keyPressed=='c'&&kinectPoints.size()==2&&projectorPoints.size()==2){
					double errorValue=stereoCalibrate(objectPoints, kinectPoints, projectorPoints, objectKinectMat, kinectDistortCoeff, objectProjectorMat, projectorDistortCoeff, Size(640,480), stereoR, stereoT, stereoE, stereoF);
					cout<<errorValue;
				}
			}
		}
	} 
	catch (kinect_exception) {
		std::cerr << "Caught an exception" << std::endl;
	}
	return 0;
}