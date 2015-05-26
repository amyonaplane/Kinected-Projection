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

using namespace cv;
using namespace std;
using namespace Eigen;

int main (int argc, char *argv[]) {
	Size board_sz=Size(8,6); //size of chessboard
	vector<Point2f> corners; //store points of corners found
	char pattern[100];
	sprintf_s(pattern, 100, "%s%%03d.ply", argv[1]);
	int modelNum = atoi(argv[2]);
	bool pfound=false;

	char fname[100];
	//namedWindow("Video");
	//namedWindow("Depth");
	Size projectorSize(1024,768); //resolution of projector
	Size mainScreenSize(1920,1080); //resolution of main display
	namedWindow("Projector");
	Mat proj=imread("checkerboard.png");

	//find corners of chessboard using image found on projector
	Mat grey;
	cvtColor(proj, grey, CV_RGB2GRAY);
	bool found = findChessboardCorners(grey,board_sz,corners,CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE+CALIB_CB_FAST_CHECK);
	if(found){
		cornerSubPix(grey, corners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
	}
	bool search = false;
	//for each point found in corners in
	imshow("Projector", proj);
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
		while (!done) {

			bool haveVideo = kinect.grabVideoFrame();
			bool haveDepth = kinect.grabDepthFrame();
			
			if (haveVideo) {
				kinect.fetchVideoImage(&video);
			}

			if (haveDepth) {
				kinect.fetchDepthImage(&depth);
			}

			if (haveVideo && haveDepth) {
				int keyPressed = waitKey(1);
				Mat greyvid;
				Matrix3f H1;
				cvtColor(video, greyvid, CV_RGB2GRAY);
				imshow("g",greyvid);
				imshow("video",video);
				if(search){
					proj=NULL;
					imshow("Projector",proj);
					vector<Point2f> pcorners;
					pfound = findChessboardCorners(greyvid, board_sz, pcorners, CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
					if(pfound){
							cornerSubPix(greyvid, pcorners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
							drawChessboardCorners(greyvid, board_sz, Mat(pcorners), pfound);
							imshow("i",greyvid);
							for(Point2f p:pcorners){//for each corner found
								Vector3f v(p.x,p.y,1);//convert point into vector
								Vector3f v2(H1*v);//multiply vector by homograpy
								//v2/=v2(2);//divide by p(2) - generalize
								circle(proj,Point2f(v2(0),v2(1)),3,Scalar(0,255,0),3);//print circle on found points
							}
							imshow("Projector",proj);
					}
				}

				if (keyPressed == 'q' || keyPressed == 'Q') {
					done = true;
				}
				else if(keyPressed=='y'){
					Size vboard_sz=Size(8,6);
					vector<Point2f> vcorners;
					Mat pic, pgrey;//, greyvid;
					kinect.fetchVideoImage(&pic);//get current video image to find chessboard corners
					proj=NULL;
					imshow("Projector",proj);
					cvtColor(pic, pgrey, CV_RGB2GRAY);
					imshow("v",pgrey);
					//find corners from checkerboard in video image
					bool vfound = findChessboardCorners(pgrey,vboard_sz,vcorners,CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
					if(vfound){
						cornerSubPix(pgrey, vcorners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
						Mat H=findHomography(vcorners, corners, RANSAC);//0,RANSAC,LMEDS, homography between video and image corners
						for(int i=0;i<H.rows;i++){
							for(int j=0;j<H.cols;j++){
								H1(i,j)=H.at<double>(i,j);
							}
						}
						//find corners from the physical checkerboard
						Size pboard_sz=Size(8,6);//(12,7);
						vector<Point2f> pcorners;
					//	waitKey(10000);
						search=true;
						waitKey(4000);
						//pfound = findChessboardCorners(greyvid, pboard_sz, pcorners, CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE);
						//if(pfound){
		//					cornerSubPix(greyvid, pcorners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
		//					drawChessboardCorners(video, pboard_sz, Mat(pcorners), pfound);
		//					imshow("g",greyvid);
							/*for(Point2f p:pcorners){//for each corner found
								Vector3f v(p.x,p.y,1);//convert point into vector
								Vector3f v2(H1*v);//multiply vector by homograpy
								v2/=v2(2);//divide by p(2) - generalize
								circle(proj,Point2f(v2(0),v2(1)),3,Scalar(0,255,0),3);//print circle on found points
							}
							imshow("Projector",proj);*/
			//			}//end pfound
					}//end vfound
					//proj=NULL;
					//imshow("Projector", proj);
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
	//}//end pfound
	catch (kinect_exception) {
		cerr << "Caught an exception" << endl;
	}//end catch

	return 0;
}//end main