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
		stereoR, stereoT, stereoE, stereoF, R1, R2, P1, P2, Q, rodriguesMat;
	vector<Mat> kinectRvect, kinectTvect, projectorRvect, projectorTvect;
	vector<Point2f> kinect2ProjectorCorners, projectorCorners, representationCorners, realCorners;

	Size projectorBoard_sz=Size(8,6);
	Size realBoard_sz=Size(12,7);
	int calibrationSize=20;

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
		for(int y=0;y<560;y+=80){
			for(int x=0;x<960;x+=80){
				Point3f point2fToPoint3f(x,y,0);
				vecPoints.push_back(point2fToPoint3f);
			}
		}
	for(int j=0;j<calibrationSize;j++){
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

	//import 20 kinectPoints from file
	ifstream outputFileKI("kinect.txt");
	float x,y;
	char comma;
	char semicolon;
	int cornerSize=0;
	while(outputFileKI>> x>>comma >>y>>semicolon){
		if(cornerSize<83){
			realCorners.push_back(Point2f(x,y));
			cornerSize++;
		}
		else{
			realCorners.push_back(Point2f(x,y));
			kinectPoints.push_back(realCorners);
			realCorners.clear();
			cornerSize=0;
		}
	}

	//import 20 projectorPoints from file
	ifstream outputFilePI("projector.txt");
	float px,py;
	char pcomma;
	char psemicolon;
	int pcornerSize=0;
	while(outputFilePI>> px>>pcomma >>py>>psemicolon){
		if(pcornerSize<83){
			realCorners.push_back(Point2f(px,py));
			pcornerSize++;
		}
		else{
			realCorners.push_back(Point2f(px,py));
			projectorPoints.push_back(realCorners);
			realCorners.clear();
			pcornerSize=0;
		}
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
		bool stereoCalibrated = false;

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
				imshow("depth",depth);

				if (keyPressed == 'q' || keyPressed == 'Q') {
					done = true;
				} 
				//calibrate the Kinect, make sure the chessboard is close to the Kinect to ensure smallest error value
				else if(keyPressed=='z'&&kinectPoints.size()<calibrationSize){
					bool realCornersFound=findChessboardCorners(video, realBoard_sz, realCorners, CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
					if(realCornersFound){
						cornerSubPix(video, realCorners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
						kinectPoints.push_back(realCorners);
						ofstream outputFileKO;
						outputFileKO.open("kinect2.txt");
						cout<<"K"<<kinectPoints.size()<<" ";
						if(kinectPoints.size()==calibrationSize){
							double kinectCalibrate=calibrateCamera(objectPoints, kinectPoints, Size(640,480), objectKinectMat, kinectDistortCoeff, kinectRvect, kinectTvect);
							cout<<objectKinectMat;
							cout<<kinectCalibrate;
							for(int i=0;i<kinectPoints.size();i++){
								outputFileKO<<kinectPoints[i]<<endl;
							}
						}
					}
				}
				//if kinect points in text document have been found and recorded
				else if(keyPressed=='z'&&kinectPoints.size()==20){
					double kinectCalibrate=calibrateCamera(objectPoints, kinectPoints, Size(640,480), objectKinectMat, kinectDistortCoeff, kinectRvect, kinectTvect);
					cout<<objectKinectMat;
					cout<<kinectCalibrate;
				}
				//find projector corners, use cloth over the chessboard
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
					bool kinectFound = findChessboardCorners(greyHomographyImage,projectorBoard_sz,kinect2ProjectorCorners,CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
					if(kinectFound){
						cout<<"-k2P found-"<<endl;
						cornerSubPix(greyHomographyImage, kinect2ProjectorCorners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
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
				//remove cloth and cover the lens
				else if(keyPressed=='j'&&light&&k2PHomographyFound&&projectorPoints.size()<20){
					realCornersFound=findChessboardCorners(video, realBoard_sz, realCorners, CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);		
					if(realCornersFound){
						vector<Point2f>u;
						u.clear();
						cout<<"-k2R found-"<<endl;
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
							circle(projectorImage,Point2f(vectorHomography(0),vectorHomography(1)),3,Scalar(0,255,0),3);//print circle on found points
							u.push_back(Point2f(vectorHomography(0),vectorHomography(1)));
							cornerSubPix(video, realCorners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
						}
						projectorPoints.push_back(u);
						if(projectorPoints.size()==calibrationSize){
							double projectCalibrate=calibrateCamera(objectPoints, projectorPoints, projectorSize, objectProjectorMat, projectorDistortCoeff, projectorRvect, projectorTvect);
							cout<<objectProjectorMat;
							cout<<projectCalibrate;
							ofstream outputFilePO;
							outputFilePO.open("projector2.txt");
							for(int i=0;i<projectorPoints.size();i++){
								outputFilePO<<projectorPoints[i]<<endl;
							}
							cout<<"P"<<projectorPoints.size();
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
				else if(keyPressed=='j'&&projectorPoints.size()==20){
					double projectCalibrate=calibrateCamera(objectPoints, projectorPoints, projectorSize, objectProjectorMat, projectorDistortCoeff, projectorRvect, projectorTvect);
					cout<<objectProjectorMat<<endl;
					cout<<projectCalibrate<<endl;
				}
				else if(keyPressed=='c'&&kinectPoints.size()==calibrationSize&&projectorPoints.size()==calibrationSize){
					double errorValue=stereoCalibrate(objectPoints, kinectPoints, projectorPoints, objectKinectMat, kinectDistortCoeff, objectProjectorMat, projectorDistortCoeff, Size(640,480), stereoR, stereoT, stereoE, stereoF);
					cout<<"stereoCalibration rms value: "<<errorValue<<endl; //kinect error 0.2 project error 0.16 why is stereo error 23?
					stereoRectify(objectKinectMat, kinectDistortCoeff, objectProjectorMat, projectorDistortCoeff,Size(640,480), stereoR, stereoT, R1, R2, P1, P2, Q);
					//cout<<"R: "<<stereoR<<endl;
					//cout<<"T: "<<stereoT<<endl;
					stereoCalibrated=true;
				}
				else if(keyPressed=='v'&&stereoCalibrated){
					//find objects that are 2 meters away and light them up with color
					//create projection matrix
					Mat circleProjection=Mat(3,4,CV_64F);
					for(int ko=0;ko<3;ko++){
						for(int jo=0;jo<3;jo++){
							circleProjection.at<double>(ko,jo)=stereoR.at<double>(ko,jo);
						}
					}
						for(int m=0;m<3;m++){
							circleProjection.at<double>(m,3)=stereoT.at<double>(m,0);
						}
					cout<<circleProjection;
					
					//if z in any part of depth map is greater than 2
					Mat imageClone=Mat(1024,768,CV_64F);
					for(int j=0;j<1024;j++){
						for(int k=0;k<786;k++){
							Point3d kinectPoint=kinect.depth2xyz(Point2d(j,k));
							kinectPoint.x*=100;
							kinectPoint.y*=100;
							kinectPoint.z*=100;
							Mat kinectPointMat=Mat(4,1,CV_64F);
							kinectPointMat.at<double>(0,0)=kinectPoint.x;
							kinectPointMat.at<double>(1,0)=kinectPoint.y;
							kinectPointMat.at<double>(2,0)=kinectPoint.z;
							kinectPointMat.at<double>(3,0)=1;
							Mat projectPoint=objectProjectorMat*circleProjection*kinectPointMat;
							Point3d solution(projectPoint);
							//solution/=solution.z;
							if (solution.z>200){
								imageClone.at<uchar>(j,k)=128;
							}
							else{
								imageClone.at<uchar>(j,k)=0;
							}
						}
					}
					imshow("Projector",imageClone);
					//Point3d kinectPoint=kinect.depth2xyz(Point2d(0,0));
					/*Mat kinectPointMat=Mat(4,1,CV_64F);
					kinectPointMat.at<double>(0,0)=kinectPoint.x;
					kinectPointMat.at<double>(1,0)=kinectPoint.y;
					kinectPointMat.at<double>(2,0)=kinectPoint.z;
					kinectPointMat.at<double>(3,0)=1;
					Mat projectPoint=objectProjectorMat*circleProjection*kinectPointMat;
					Point3d solution(projectPoint);
					solution/=solution.z;
					cout<<solution;*/
				}

				else if(keyPressed=='u'&&stereoCalibrated){
					Mat greyv, greyd;
					GaussianBlur(depth,greyd, Size(3,3),3,3);

					//imshow("Depth", greyd);
					vector<Vec3f> dcircles;
					Point2d centerd;
					int radiusd=0;
					HoughCircles(greyd, dcircles, HOUGH_GRADIENT, 1, greyd.rows/8, 20, 15, 30, 45); 
					for(int i=0;i<dcircles.size();i++){
						centerd=Point2d(cvRound(dcircles[0][0]), cvRound(dcircles[0][1]));
						radiusd=cvRound(dcircles[0][2]);
						circle(greyd, centerd, 3, Scalar(0,255,0),-1,8,0);
						circle(greyd, centerd, radiusd, Scalar(0,0,255),3,8,0);
					}
					projectorImage=NULL;
					
					Mat circleProjection=Mat(3,4,CV_64F);
					for(int ko=0;ko<3;ko++){
						for(int jo=0;jo<3;jo++){
							circleProjection.at<double>(ko,jo)=stereoR.at<double>(ko,jo);
						}
					}
					for(int m=0;m<3;m++){
						circleProjection.at<double>(m,3)=stereoT.at<double>(m,0);
					}

					Point3d kinectPoint=kinect.depth2xyz(centerd);
					kinectPoint.x*=100;
					kinectPoint.y*=100;
					kinectPoint.z*=100;
					cout<<"kinect point"<<kinectPoint<<endl;
					Mat kinectPointMat=Mat(4,1,CV_64F);
					kinectPointMat.at<double>(0,0)=kinectPoint.x;
					kinectPointMat.at<double>(1,0)=kinectPoint.y;
					kinectPointMat.at<double>(2,0)=kinectPoint.z;
					kinectPointMat.at<double>(3,0)=1;
					Mat projectPoint=objectProjectorMat*circleProjection*kinectPointMat;
					Point3d solution(projectPoint);
					cout<<"solution:"<<solution<<endl;
					solution/=solution.z;
					Point s2(solution.x, solution.y);
					cout<<"s2:"<<s2<<endl;
					circle(projectorImage,s2,radiusd*solution.z, Scalar(0,0,255),-1,8,0);
					imshow("keypoints", greyd);
					imshow("Projector",projectorImage);
				}
			}
		}
	} 
	catch (kinect_exception) {
		std::cerr << "Caught an exception" << std::endl;
	}
	return 0;
}