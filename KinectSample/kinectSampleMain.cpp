#include "Kinect.h"
#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>

using namespace cv;
using namespace std;

int main (int argc, char *argv[]) {

	char pattern[100];
	sprintf_s(pattern, 100, "%s%%03d.ply", argv[1]);
	int modelNum = atoi(argv[2]);

	char fname[100]; 

	//namedWindow("Video");
	//namedWindow("Depth");

	Mat video(Size(200,200), CV_8UC3);
	video.setTo(Scalar(128,128,128));

	cv::Mat depth(Size(200,200), CV_8UC1);
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

				/*grey video feed*/
				Mat grey; 
				depth.convertTo(grey,-1,1.5,1);
				//cvtColor(video, grey, CV_BGR2GRAY);
				//cvtColor(depth, grey, CV_BGR2GRAY);
				//GaussianBlur(grey, grey, Size(5,5), 3, 3);

				//GaussianBlur(depth, depth, Size(5,5), 3,3);
				//imshow("Video", video);
				//imshow("Depth", depth);

				//show single image and convert to greyscale
				//Mat im = imread("demov2.png");
				//Mat grey = imread("image0001.png");
				//cvtColor(depth, grey, CV_BGR2GRAY);

				//using HoughCircles
				//GaussianBlur(grey, grey, Size(5,5), 2, 2);
				vector<Vec3f> circles;
				HoughCircles(grey, circles, HOUGH_GRADIENT, 1, grey.rows/8, 100, 50, 0, 0);
				for(int i=0; i<circles.size();i++){
					Point2d center(cvRound(circles[i][0]), cvRound(circles[i][1]));
					int radius=cvRound(circles[i][2]);
					//transpose points of circle from video feed to depth feed
					circle(grey, center, 3, Scalar(0,255,0),-1,8,0);
					circle(grey, center, radius, Scalar(0,0,255),3,8,0);
					//use depth2xyz on points to get x,y, and depth (z)
					//Point3d p=kinect.depth2xyz(center);
					//cout<<p;
				}
				/*HoughCircles(grey, circles, HOUGH_GRADIENT, 1, grey.rows/8, 65, 69, 0, 0); //for video feed
				for(int i=0; i<circles.size();i++){
					Point2d center(cvRound(circles[i][0]), cvRound(circles[i][1]));
					int radius=cvRound(circles[i][2]);
					//transpose points of circle from video feed to depth feed
					circle(grey, center, 3, Scalar(0,255,0),-1,8,0);
					circle(grey, center, radius, Scalar(0,0,255),3,8,0);
					//use depth2xyz on points to get x,y, and depth (z)
					Point3d p=kinect.depth2xyz(center);
					cout<<p;
					//use RANSAC to find three/four points and determine center of square using triangles and distance
					//find radius, depth and center of sphere
				}*/
				imshow("keypoints", grey);

				int keyPressed = waitKey(1);

				if (keyPressed == 'q' || keyPressed == 'Q') {
					done = true;
				}
				else if(keyPressed=='y'){
					sprintf_s(fname, 100, "imagev%04d.png", modelNum+1);
					imwrite(fname, grey);
				}
				else if (keyPressed == ' ') {
					
					sprintf_s(fname, 100, "image%04d.png", modelNum+1);
					imwrite(fname, depth);

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