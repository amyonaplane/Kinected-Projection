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
				bool vidCirc=false;
				bool depthCirc=false;

				Mat greyv, greyd; 
				//depth.convertTo(grey,-1,2,0);

				cvtColor(video, greyv, CV_BGR2GRAY);
				GaussianBlur(greyv, greyv, Size(5,5), 3, 3);
				//imshow("Video", video);
				imshow("Depth", depth);

				//using HoughCircles
				//GaussianBlur(grey, grey, Size(5,5), 2, 2);
				/*vector<Vec3f> dcircles;
				HoughCircles(greyd, dcircles, HOUGH_GRADIENT, 1, greyd.rows/8, 20, 15, 60, 100);
				for(int i=0; i<dcircles.size();i++){
					Point2d center(cvRound(dcircles[i][0]), cvRound(dcircles[i][1]));
					int radius=cvRound(dcircles[i][2]);
					//transpose points of circle from video feed to depth feed
					//circle(greyd, center, 3, Scalar(0,255,0),-1,8,0);
					//circle(greyd, center, radius, Scalar(0,0,255),3,8,0);

					//Point3d p=kinect.depth2xyz(center);
					//literal center of sphere = circle center distance - radius
					//cout<<p;
					depthCirc=true;
				}*/
				vector<Vec3f> vcircles;
				vector<Vec3f> dcircles;
				HoughCircles(greyv, vcircles, HOUGH_GRADIENT, 1, greyv.rows/4, 25, 30, 30, 45);
				//HoughCircles(greyv, vcircles, HOUGH_GRADIENT, 1, greyv.rows, 60, 60, 0, 0);
				for(int i=0; i<vcircles.size();i++){
					Point2d center(cvRound(vcircles[i][0]), cvRound(vcircles[i][1]));
					int radius=cvRound(vcircles[i][2]);
					//euclidean distance
					//find threshold
					//if in threshold it is white, continue
					vidCirc=true;
					HoughCircles(greyd, dcircles, HOUGH_GRADIENT, 1, greyd.rows/8, 20, 15, 60, 100);
					for(int i=0; i<dcircles.size();i++){
						Point2d vcenter(cvRound(dcircles[i][0]), cvRound(dcircles[i][1]));
						int vradius=cvRound(dcircles[i][2]);
						depthCirc=true;
					}

					//Point3d p=kinect.depth2xyz(center);
					//cout<<p;
					//use RANSAC to find three/four points and determine center of square using triangles and distance
					//find radius, depth and center of sphere
				}
				if(depthCirc==true && vidCirc==true){
						for(int l=0;l<dcircles.size();l++){
							//if distance/difference between vcircles[i][0],dcircles[i][0] and vcircles[i][1],dcircles[i][1] is less than certain threshold
							long distance = sqrt((vcircles[l][0]+dcircles[l][0])+(vcircles[l][1]+dcircles[l][1]));
							if(distance<=10){
								Point2d centerd(cvRound((vcircles[l][0]+dcircles[l][0])/2), cvRound((vcircles[l][1])+dcircles[l][1])/2);
								int radiusd=cvRound((vcircles[l][2]+dcircles[l][2])/2);
								circle(greyv, centerd, 3, Scalar(0,255,0),-1,8,0);
								circle(greyv, centerd, radiusd, Scalar(0,0,255),3,8,0);
							}
						}

				}
				imshow("keypoints", greyv);

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