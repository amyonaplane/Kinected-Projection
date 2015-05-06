#include "Kinect.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>

#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

int main (int argc, char *argv[]) {
	Size board_sz=Size(8,6); //size of chessboard
	vector<Point2f> corners; //store points of corners found
	char pattern[100];
	sprintf_s(pattern, 100, "%s%%03d.ply", argv[1]);
	int modelNum = atoi(argv[2]);

	char fname[100];
	//namedWindow("Video");
	//namedWindow("Depth");
	Size projectorSize(1024,768); //resolution of projector
	Size mainScreenSize(1920,1080); //resolution of main display
	namedWindow("Projector");
	Mat proj=imread("checkerboard.png");

	//find corners of chessboard using image
	Mat grey;
	cvtColor(proj, grey, CV_RGB2GRAY);
	bool found = findChessboardCorners(grey,board_sz,corners,CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE+CALIB_CB_FAST_CHECK);
	if(found){
		cornerSubPix(grey, corners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
		drawChessboardCorners(proj, board_sz, Mat(corners), found);
	}

	//Mat proj(projectorSize, CV_8UC3);
	//proj.setTo(Scalar(128,128,128));
	//rectangle(proj, Point(50, 100), Point(200,300), Scalar(255,0,255),5,8);

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

				bool vidCirc=false;
				bool depthCirc=false;
				Mat greyv, greyd;
				Size vboard_sz=Size(8,6); //size of chessboard
				vector<Point2f> vcorners; //store points of corners found
				Mat vgrey;
				cvtColor(video, vgrey, CV_RGB2GRAY);
				bool found = findChessboardCorners(vgrey,vboard_sz,vcorners,CALIB_CB_ADAPTIVE_THRESH+CALIB_CB_NORMALIZE_IMAGE+CALIB_CB_FAST_CHECK);
				if(found){
					cornerSubPix(vgrey, vcorners, Size(11,11), Size(-1,-1),TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_EPS,30,0.1));
					drawChessboardCorners(video, vboard_sz, Mat(vcorners), found);
				}


				cvtColor(video, greyv, CV_BGR2GRAY);
				GaussianBlur(greyv, greyv, Size(5,5), 3, 3);
				//greyd=depth;
				GaussianBlur(depth,greyd, Size(3,3),3,3);
				//depth.convertTo(greyd, -1, 1.0,0);
				//imshow("Video", video);
				//imshow("Depth", depth);
				vector<Vec3f> dcircles;
				//vector<Vec3f> vcircles;

				/*HoughCircles(greyv, vcircles, HOUGH_GRADIENT, 1, greyv.rows/4, 25, 30, 30, 45);
				//HoughCircles(greyv, vcircles, HOUGH_GRADIENT, 1, greyd.rows, 60, 60, 0, 0);
				if (vcircles.size()>0){
					//create matrix that contains center and a radius of 'radius
					//euclidean distance
					//find threshold
					//if in threshold it is white, continue
					vidCirc=true;
				}*/
				HoughCircles(greyd, dcircles, HOUGH_GRADIENT, 1, greyd.rows/8, 20, 15, 30, 45); 
				//if(dcircles.size()>0){
				//	depthCirc=true; 
				//}

					//Point3d p=kinect.depth2xyz(center);
					//cout<<p;
					//use RANSAC to find three/four points and determine center of square using triangles and distance
					//find radius, depth and center of sphere
				//if(depthCirc==true && vidCirc==true){
							//if distance/difference between centers is less than certain threshold
							//float distance = sqrt(pow((vcircles[0][0]-dcircles[0][0]),2)+pow((vcircles[0][1]-dcircles[0][1]),2));
							//if(distance<=15){
								//Point2d centerd(cvRound((vcircles[0][0]+dcircles[0][0])/2), cvRound((vcircles[0][1])+dcircles[0][1])/2);
				for(int i=0;i<dcircles.size();i++){
				Point2d centerd(cvRound(dcircles[0][0]), cvRound(dcircles[0][1]));
								//int radiusd=cvRound((vcircles[0][2]+dcircles[0][2])/2);
				int radiusd=cvRound(dcircles[0][2]);
								circle(greyd, centerd, 3, Scalar(0,255,0),-1,8,0);
								circle(greyd, centerd, radiusd, Scalar(0,0,255),3,8,0);
						//} 
				}
				//imshow("keypoints", greyd);

				//detect four points
				//compute homography from four points to projector points
				//detect corners of chessboard
				//transform detected corners to projector poitns using homography
				//for all corner points
					//project projector plan points in neighbourhood of transformed corner point
					//captre image and detect point's coordinates
					///select the one closest to detected corner
				//end for

				imshow("video", video);

				int keyPressed = waitKey(1);

				if (keyPressed == 'q' || keyPressed == 'Q') {
					done = true;
				}
				/*else if(keyPressed=='y'){
					sprintf_s(fname, 100, "imagev%04d.png", modelNum+1);
					imwrite(fname, depth);
				}*/
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
				}
			}
		}

	} catch (kinect_exception) {
		cerr << "Caught an exception" << endl;
	}

	return 0;
}