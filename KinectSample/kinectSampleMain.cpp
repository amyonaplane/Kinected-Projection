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
	Size projectorBoard_sz=Size(8,6); //size of chessboard
	vector<Point2f> projectorCorners; //store points of corners found
	char pattern[100];
	sprintf_s(pattern, 100, "%s%%03d.ply", argv[1]);
	int modelNum = atoi(argv[2]);

	//define real-world corners that are 8cmx8cm
	vector<Point2f> representationCorners;
	for(int y=0; y<=6; y++){
		for(int x=0; x<=11;x++){
			representationCorners.push_back(Point2f(x,y));
		}
	}

	Size kinect2ProjectorBoard_sz=Size(8,6);
	vector<Point2f> kinect2ProjectorCorners;
	Mat projectorImage, greyProjectorImage, homographyImage, greyHomographyImage, projector2KinectHomography, H2, lightImage, darkImage, chessboardImage;//, greyvid;
	Size realBoard_sz=Size(12,7);
	vector<Point2f> realCorners;
	char fname[100];

	//namedWindow("Video");
	//namedWindow("Depth");
	Size projectorSize(1024,768); //resolution of projector
	Size mainScreenSize(1920,1080); //resolution of main display
	namedWindow("Projector");
	projectorImage=imread("checkerboard.png");
	//find corners of chessboard using image found on projector
	cvtColor(projectorImage, greyProjectorImage, CV_RGB2GRAY);
	bool projectorCornersFound = findChessboardCorners(greyProjectorImage,projectorBoard_sz,projectorCorners,CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
	if(projectorCornersFound){
		cornerSubPix(greyProjectorImage, projectorCorners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
	}
	bool k2PHomographyFound = false;
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
	Mat video(Size(200,200), CV_8UC3);
	video.setTo(Scalar(128,128,128));
	Mat depth(Size(200,200), CV_8UC1);
	depth.setTo(Scalar(128,128,128));
	try {
		Mat H;
		Kinect& kinect = Kinect::get(0);
		kinect.openVideoStream(Kinect::RESOLUTION_640x480);
		kinect.openDepthStream(Kinect::RESOLUTION_640x480);
		kinect.setDepthMode(Kinect::DEPTH_MODE_NEAR);
		double angle = 5;
		kinect.setAngle(5);
		bool done = false;
		bool light = false;
		while (!done) {
			bool haveVideo = kinect.grabVideoFrame();
			bool haveDepth = kinect.grabDepthFrame();	
			if (haveVideo) {
				kinect.fetchVideoImage(&video);
				//flip(video,video,1);
			}
			if (haveDepth) {
				kinect.fetchDepthImage(&depth);
			}
			if (haveVideo && haveDepth) {
				int keyPressed = waitKey(1);
				Mat greyVideo;
				Matrix3f p2KHomography,H3;
				cvtColor(video, greyVideo, CV_RGB2GRAY);
				imshow("g",greyVideo);
				imshow("video",video);
				if (keyPressed == 'q' || keyPressed == 'Q') {
					done = true;
				}
				else if(keyPressed=='f'){
					kinect.fetchVideoImage(&lightImage);
					cvtColor(lightImage, lightImage, CV_RGB2GRAY);
					projectorImage=NULL;
					imshow("Projector",projectorImage);
				light=true;
				}
				else if(keyPressed=='y'&&light){
					kinect.fetchVideoImage(&darkImage);//get current video image to find kinect2Projector chessboard corners
					//flip(homographyImage,homographyImage,1);
					cvtColor(darkImage, darkImage, CV_RGB2GRAY);
					//cvtColor(homographyImage, greyHomographyImage, CV_RGB2GRAY);
					absdiff(lightImage,darkImage,greyHomographyImage);
					//imshow("y",chessboardImage);
					imshow("greyImage", greyHomographyImage);

					//find corners from checkerboard in video image
					bool kinectFound = findChessboardCorners(greyHomographyImage,kinect2ProjectorBoard_sz,kinect2ProjectorCorners,CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
					if(kinectFound){
						cout<<"!!";
						cornerSubPix(greyHomographyImage, kinect2ProjectorCorners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
						drawChessboardCorners(greyHomographyImage, kinect2ProjectorBoard_sz, Mat(kinect2ProjectorCorners), kinectFound);
						imshow("greyImage",greyHomographyImage);
						projector2KinectHomography=findHomography(projectorCorners, kinect2ProjectorCorners, RANSAC);//0,RANSAC,LMEDS, homography between video and image corners
						for(int i=0;i<projector2KinectHomography.rows;i++){
							for(int j=0;j<projector2KinectHomography.cols;j++){
								p2KHomography(i,j)=projector2KinectHomography.at<double>(i,j);
							}
						}
						//find corners from the physical chessboard
						k2PHomographyFound=true;
						waitKey(4000);
					}//end vfound
					projectorImage=NULL;
					imshow("Projector", projectorImage);
				}
				//find corners on real-world chessboard
				else if(keyPressed=='j'&&k2PHomographyFound==true){
					projectorImage=NULL;
					imshow("Projector", projectorImage);
					bool realCornersFound=findChessboardCorners(greyVideo, realBoard_sz, realCorners, CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
					if(realCornersFound){
						cornerSubPix(greyVideo, realCorners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
						Mat representation2RealHomography=findHomography(representationCorners, realCorners, RANSAC);
						//drawChessboardCorners(greyVideo, realBoard_sz, Mat(realCorners), realCornersFound);
						for(int i=0;i<representation2RealHomography.rows;i++){
							for(int j=0;j<representation2RealHomography.cols;j++){
								H3(i,j)=representation2RealHomography.at<double>(i,j);
							}
						}
						//imshow("i", greyVideo);
						for(Point2f point:realCorners){//for each corner found
							Vector3f point2Vector(point.x,point.y,1);//convert point into vector
							Vector3f vectorHomography(p2KHomography.inverse()*point2Vector);//multiply vector by homograpy inverse
							vectorHomography/=vectorHomography(2);//divide by p(2) - generalize
							circle(projectorImage,Point2f(vectorHomography(0),vectorHomography(1)),3,Scalar(0,255,0),-1);//print circle on found points
							//if s is pressed, move dot to left (p.x--)?
							//if f is pressed, move dot to right (p.x++)?
							//if e is pressed, move dot up (p.y++)?
							//if x is pressed, move dot down (p.y--)?
							//if v is pressed, move onto the next point
							imshow("Projector",projectorImage);
						}
					}
				}
				else if (keyPressed == ' ') {
					sprintf_s(fname, 100, "image%04d.png", modelNum+1);
					imwrite(fname, video);
					sprintf_s(fname, 100, pattern, modelNum++);
					cout << fname << endl;
					ofstream out(fname);
					out << "ply\n"
						<< "format ascii 1.0\n"
						<< "element dimensions 1\n"
						<< "property int width\n"
						<< "property int height\n"
						<< "element vertex " << depth.size().area() << "\n"
						<< "property float x\n"
						<< "property float y\n"
						<< "property float z\n"
						<< "property uchar red\n"
						<< "property uchar green\n"
						<< "property uchar blue\n"
						<< "end_header\n";
					out << 640 << endl;
					out << 480 << endl;		
					Point2d p2d;
					for (p2d.x = 0; p2d.x < 640; ++p2d.x) {
						for (p2d.y = 0; p2d.y < 480; ++p2d.y) {
							Point3d xyz = kinect.depth2xyz(p2d);
							Point2d rgb = kinect.depth2video(p2d);
							int ix = int(rgb.x);
							int iy = int(rgb.y);
							if (ix < 0 || ix >= video.size().width ||
								iy < 0 || iy >= video.size().height ||
								xyz.z < 0.1 ) {
									out << "0 0 0 0 0 0\n";
							} else {
								out << xyz.x << " " << xyz.y << " " << xyz.z << " ";
							
								Vec3b col = video.at<Vec3b>(iy, ix);
								out << int(col.val[2]) << " " << int(col.val[1]) << " " << int(col.val[1]) << "\n";
							}
						}
					}
					out.close();
				} else if (keyPressed == 'a' || keyPressed == 'A') {
					kinect.setAngle(angle+2.0);
					angle = kinect.getAngle();
					cout << "Angle increased to " << angle << endl;
				} else if (keyPressed == 'z' || keyPressed == 'Z') {
					kinect.setAngle(angle-2);
					angle = kinect.getAngle();
					cout << "Angle decreased to " << angle << endl;
				}//end else if
			}//end if video and depth
		}//end while
	}//end try
	catch (kinect_exception) {
		cerr << "Caught an exception" << endl;
	}//end catch
	return 0;
}//end main