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

	cv::namedWindow("Video");
	cv::namedWindow("Depth");

	cv::Mat video(cv::Size(200,200), CV_8UC3);
	video.setTo(cv::Scalar(128,128,128));

	cv::Mat depth(cv::Size(200,200), CV_8UC1);
	depth.setTo(cv::Scalar(128,128,128));

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
				//cvtColor(video, grey, CV_BGR2GRAY);
				//Mat im=
				//GaussianBlur(grey, grey, Size(5,5), 2, 2);
				//threshold(grey,grey,75,100, THRESH_BINARY);
				/*smooth and define depth feed*/
				//Mat dil, dil2;
				//erode(depth,dil,Mat(), Point(-1,-1), 4);
				//dilate(depth, dil2, Mat(), Point(-1,-1),3);

				//imshow("Video", video);
				//imshow("Depth", depth);

				//show single image and use simpleblobdetector
				Mat im = imread("sphere1dbody.png");
				cvtColor(im, grey, CV_BGR2GRAY);
				SimpleBlobDetector detector;
				vector<KeyPoint> kp;
				detector.detect(grey,kp);
				drawKeypoints(grey,kp,grey, Scalar(0,0,255));
				//GaussianBlur(grey, grey, Size(5,5), 2, 2);
				/*vector<Vec3f> circles;
				HoughCircles(grey, circles, HOUGH_GRADIENT, 1, grey.rows/8, 65, 75, 0, 0);//grey.rows = 480 aka img height; 55 30 20 50 radius
				for(int i=0; i<circles.size();i++){
					Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
					int radius =cvRound(circles[i][2]);
					circle(grey, center, 3, Scalar(0,255,0),-1,8,0);
					circle(grey, center, radius, Scalar(0,0,255),3,8,0);
				}*/
				imshow("keypoints", grey);

				int keyPressed = cv::waitKey(1);

				if (keyPressed == 'q' || keyPressed == 'Q') {
					done = true;
				} else if (keyPressed == ' ') {
					
					sprintf_s(fname, 100, "image%04d.png", modelNum+1);
					cv::imwrite(fname, depth);

					sprintf_s(fname, 100, pattern, modelNum++);
					std::cout << fname << std::endl;

					std::ofstream out(fname);
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

					out << 640 << std::endl;
					out << 480 << std::endl;
					
					cv::Point2d p2d;
					for (p2d.x = 0; p2d.x < 640; ++p2d.x) {
						for (p2d.y = 0; p2d.y < 480; ++p2d.y) {
							cv::Point3d xyz = kinect.depth2xyz(p2d);
							cv::Point2d rgb = kinect.depth2video(p2d);
							int ix = int(rgb.x);
							int iy = int(rgb.y);
							if (ix < 0 || ix >= video.size().width ||
								iy < 0 || iy >= video.size().height ||
								xyz.z < 0.1 ) {
									out << "0 0 0 0 0 0\n";
							} else {
								out << xyz.x << " " << xyz.y << " " << xyz.z << " ";
							
								cv::Vec3b col = video.at<cv::Vec3b>(iy, ix);
								out << int(col.val[2]) << " " << int(col.val[1]) << " " << int(col.val[1]) << "\n";
							}
						}
					}

					out.close();

				} else if (keyPressed == 'a' || keyPressed == 'A') {
					kinect.setAngle(angle+2.0);
					angle = kinect.getAngle();
					std::cout << "Angle increased to " << angle << std::endl;
				} else if (keyPressed == 'z' || keyPressed == 'Z') {
					kinect.setAngle(angle-2);
					angle = kinect.getAngle();
					std::cout << "Angle decreased to " << angle << std::endl;
				}
			}
		}

	} catch (kinect_exception) {
		std::cerr << "Caught an exception" << std::endl;
	}

	return 0;
}